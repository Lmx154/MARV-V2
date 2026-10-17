#!/usr/bin/env python3
"""FC3v2 bench IMU log (quad spec 4.0 M3, packet P3a; decision 0042 item 2). Bench only; never run by CI.

  python3 tools/target/imu_log.py capture /dev/ttyACM1 60 run.bin
      reads the USB CDC port of an image built with MARV_FC3V2_IMU_LOG for the given seconds, DTR asserted, and writes
      the bytes exactly as they arrive (raw, no terminal translation).
  python3 tools/target/imu_log.py decode run.bin [--csv samples.csv] [--npz samples.npz]
      checks the records and prints the summary and a verdict line, "LOG VALID" or "LOG INVALID: <reasons>". Exit status
      0 only for LOG VALID. --npz needs numpy (arrays t_us u64, gyro_rad_s and accel_m_s2 (N, 3) f32, flags u32).
  python3 tools/target/imu_log.py selftest
      the decoder against synthetic logs: a clean one, and one each with a CRC failure, a sequence gap, a stamp gap and a
      ring overflow count.

Record format: fw/hal/target/include/marv/hal_target/imu_log_format.hpp (magic, u32 sequence, u32 count, items, CRC-32).
Standard library only, so the capture runs under the system python3 as bench_stats.py does.
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCENARIO_REGISTER = ROOT / "fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml"
TIMING_REGISTER = ROOT / "fw/compositions/quad_fc/params/fc3v2_rate_timing_register.yaml"

DATA_MAGIC = b"MILD"
STATUS_MAGIC = b"MILS"
HEADER = struct.Struct("<4sII")  # magic, sequence, count
WORD = 4
SAMPLE = struct.Struct("<Q6fI")  # t_us, gyro xyz, accel xyz, flags
# imu_log_format.hpp StatusField, in order.
STATUS_FIELDS = ("ring_overflows", "ring_high_water", "deadline_misses", "fifo_overflows", "timing_faults",
                 "stamp_clamps", "order_faults", "spi_errors", "delivered", "logged")
# A count above this is not a record: it bounds a corrupt header's claimed length (the largest real record is a status
# record of len(STATUS_FIELDS) words or a data record of a few tens of samples).
MAX_COUNT = 1 << 16
# Counters that must stay at zero for a valid log (decision 0042 item 2: no deadline miss, FIFO overflow or stamp gap;
# the ring, timing, clamp, order and bad-tag counters are the other ways the sample stream can differ from the target's).
MUST_BE_ZERO = ("ring_overflows", "deadline_misses", "fifo_overflows", "timing_faults", "stamp_clamps", "order_faults",
                "spi_errors")


def capture(dev: str, seconds: float, out: Path) -> None:
    import fcntl
    import os
    import select
    import struct as st
    import termios
    import tty

    tiocmbis, tiocm_dtr = 0x5416, 0x002  # Linux ioctl TIOCMBIS and the DTR bit (asm-generic/ioctls.h, termios.h)
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)  # no CR/LF translation, no flow control characters: the bytes are binary
    a = termios.tcgetattr(fd)
    a[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, a)
    fcntl.ioctl(fd, tiocmbis, st.pack("I", tiocm_dtr))
    t0 = time.time()
    total = 0
    with open(out, "wb") as f:
        while time.time() - t0 < seconds:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                try:
                    b = os.read(fd, 1 << 20)
                except BlockingIOError:
                    continue
                f.write(b)
                total += len(b)
    os.close(fd)
    dt = time.time() - t0
    print(f"captured {total} bytes in {dt:.1f} s = {total / dt / 1000:.1f} kB/s -> {out}")


def tick_period() -> tuple[int, int, int]:
    """(num, den, tolerance_us): the FC3v2 tick period num / den us (fc3v2_scenario_values.yaml, ST DS13476 9.49 at this
    part's FREQ_FINE) and rate_period_tolerance_us (fc3v2_rate_timing_register.yaml). Its derivation there is
    ceil(T odr_error + 2 delta + 1 us) with delta the stamp-entry deviation, odr_error the measured residual of the
    entry spacing against T and 1 us the stamp truncation: for one tick, T = 133.33 us, 133.33 x 510e-6 + 2 x 0.76 + 1 =
    2.59, which also rounds up to 3 us. The same register value therefore bounds one tick's stamp spacing."""
    scen = SCENARIO_REGISTER.read_text()
    num = re.search(r"^tick_period_num_us:\s*\n\s*value:\s*(\d+)", scen, re.M)
    den = re.search(r"^tick_period_den:\s*\n\s*value:\s*(\d+)", scen, re.M)
    tol = re.search(r"^rate_period_tolerance_us:\s*\n?\s*\{[^}]*?value:\s*(\d+)", TIMING_REGISTER.read_text(), re.M)
    if not (num and den and tol):
        raise SystemExit("imu_log: cannot read the tick period or the tolerance from the registers; pass --tick-num, "
                         "--tick-den and --tol-us")
    return int(num.group(1)), int(den.group(1)), int(tol.group(1))


@dataclass
class Result:
    size: int = 0
    data_records: int = 0
    status_records: int = 0
    crc_failures: int = 0
    unframed_bytes: int = 0  # bytes outside valid records after the first valid one
    leading_bytes: int = 0  # bytes before the first valid record
    truncated_tail: int = 0
    seq_gaps: int = 0
    seq_lost: int = 0
    seq_first: int | None = None
    seq_last: int | None = None
    samples: list[tuple] = field(default_factory=list)
    status: list[dict] = field(default_factory=list)
    status_at_sample: list[int] = field(default_factory=list)  # samples decoded when each status record arrived
    text: bytes = b""


def parse(buf: bytes) -> Result:
    r = Result(size=len(buf))
    pos = 0
    n = len(buf)
    unframed_start = 0
    text = bytearray()
    first_valid_seen = False

    def note_unframed(a: int, b: int) -> None:
        nonlocal text
        if b > a:
            if first_valid_seen:
                r.unframed_bytes += b - a
            else:
                r.leading_bytes += b - a
            if len(text) < 4096:
                text += buf[a:b][:4096]

    i_d = i_s = -2  # next occurrence of each magic at or after pos; -1: none left; refound only once passed
    while True:
        if i_d != -1 and i_d < pos:
            i_d = buf.find(DATA_MAGIC, pos)
        if i_s != -1 and i_s < pos:
            i_s = buf.find(STATUS_MAGIC, pos)
        cand = [i for i in (i_d, i_s) if i >= 0]
        if not cand:
            break
        i = min(cand)
        if n - i < HEADER.size:
            break
        magic, seq, count = HEADER.unpack_from(buf, i)
        item = SAMPLE.size if magic == DATA_MAGIC else WORD
        if count > MAX_COUNT or (magic == STATUS_MAGIC and count != len(STATUS_FIELDS)):
            r.crc_failures += 1  # a header that cannot be a record: counted with the corrupt ones
            pos = i + 1
            continue
        end = i + HEADER.size + count * item + WORD
        if end > n:
            r.truncated_tail = n - i
            note_unframed(unframed_start, i)
            unframed_start = n
            pos = n
            break
        crc = struct.unpack_from("<I", buf, end - WORD)[0]
        if zlib.crc32(buf[i:end - WORD]) != crc:
            r.crc_failures += 1
            pos = i + 1
            continue
        note_unframed(unframed_start, i)
        first_valid_seen = True
        if r.seq_first is None:
            r.seq_first = seq
        else:
            expect = (r.seq_last + 1) & 0xFFFFFFFF
            if seq != expect:
                r.seq_gaps += 1
                r.seq_lost += (seq - expect) & 0xFFFFFFFF
        r.seq_last = seq
        body = buf[i + HEADER.size:end - WORD]
        if magic == DATA_MAGIC:
            r.data_records += 1
            r.samples.extend(SAMPLE.iter_unpack(body))
        else:
            r.status_records += 1
            r.status.append(dict(zip(STATUS_FIELDS, struct.unpack(f"<{len(STATUS_FIELDS)}I", body))))
            r.status_at_sample.append(len(r.samples))
        pos = end
        unframed_start = end
    if pos < n and not r.truncated_tail:
        note_unframed(unframed_start, n)
    r.text = bytes(text)
    return r


def stamp_gaps(samples: list[tuple], num: int, den: int, tol_us: int) -> tuple[int, int, int, list[tuple[int, int]]]:
    """(count, min spacing, max spacing, first few (index, spacing)): spacings with |dt - num/den| > tol_us, exact in
    integers (|dt den - num| > tol den)."""
    bad = 0
    lo = hi = None
    first: list[tuple[int, int]] = []
    prev = None
    for k, s in enumerate(samples):
        t = s[0]
        if prev is not None:
            dt = t - prev
            lo = dt if lo is None or dt < lo else lo
            hi = dt if hi is None or dt > hi else hi
            if abs(dt * den - num) > tol_us * den:
                bad += 1
                if len(first) < 5:
                    first.append((k, dt))
        prev = t
    return bad, lo or 0, hi or 0, first


def report(r: Result, num: int, den: int, tol_us: int) -> tuple[list[str], bool]:
    out: list[str] = []
    gaps, lo, hi, first = stamp_gaps(r.samples, num, den, tol_us)
    last = r.status[-1] if r.status else {}
    duration = (r.samples[-1][0] - r.samples[0][0]) / 1e6 if len(r.samples) > 1 else 0.0
    out.append(f"bytes {r.size}  data records {r.data_records}  status records {r.status_records}")
    out.append(f"CRC failures {r.crc_failures}  sequence gaps {r.seq_gaps} (records lost {r.seq_lost})  "
               f"unframed bytes {r.unframed_bytes} (before the first record {r.leading_bytes})  "
               f"truncated tail {r.truncated_tail} bytes")
    out.append(f"samples {len(r.samples)}  duration {duration:.3f} s  mean rate "
               f"{(len(r.samples) - 1) / duration if duration > 0 else 0.0:.1f} samples/s")
    out.append(f"tick period {num}/{den} us = {num / den:.3f} us, tolerance {tol_us} us (fc3v2 registers); spacing "
               f"min {lo} max {hi} us; stamp gaps {gaps}" + (f", first at (sample index, spacing us): {first}" if first else ""))
    if r.status:
        out.append("status (last record, cumulative since boot): " + "  ".join(f"{k}={last[k]}" for k in STATUS_FIELDS))
        tail = len(r.samples) - r.status_at_sample[-1]
        out.append(f"samples after the last status record (their counters are not in any record): {tail}")
    else:
        out.append("status: no status record")
    if r.text:
        printable = r.text.decode(errors="replace").replace("\r", "").strip()
        if printable:
            out.append("text outside the records: " + printable[:300])
    reasons: list[str] = []
    if not r.samples:
        reasons.append("no samples")
    if not r.status:
        reasons.append("no status record")
    if r.crc_failures:
        reasons.append(f"{r.crc_failures} CRC failures")
    if r.seq_gaps:
        reasons.append(f"{r.seq_gaps} sequence gaps")
    if r.unframed_bytes:
        reasons.append(f"{r.unframed_bytes} unframed bytes after the first record")
    if gaps:
        reasons.append(f"{gaps} stamp gaps")
    for k in MUST_BE_ZERO:
        if last.get(k, 0):
            reasons.append(f"{k} {last[k]}")
    out.append("LOG VALID" if not reasons else "LOG INVALID: " + "; ".join(reasons))
    return out, not reasons


def write_csv(r: Result, path: Path) -> None:
    with open(path, "w") as f:
        f.write("t_us,gx,gy,gz,ax,ay,az,flags\n")
        for s in r.samples:
            f.write(f"{s[0]},{s[1]!r},{s[2]!r},{s[3]!r},{s[4]!r},{s[5]!r},{s[6]!r},{s[7]}\n")


def write_npz(r: Result, path: Path) -> None:
    import numpy as np

    a = np.array(r.samples, dtype=np.float64).reshape(-1, 8) if r.samples else np.zeros((0, 8))
    np.savez(path, t_us=np.array([s[0] for s in r.samples], dtype=np.uint64),
             gyro_rad_s=a[:, 1:4].astype(np.float32), accel_m_s2=a[:, 4:7].astype(np.float32),
             flags=np.array([s[7] for s in r.samples], dtype=np.uint32))


# --- selftest ---------------------------------------------------------------------------------------------------------


def record(magic: bytes, seq: int, items: bytes, count: int) -> bytes:
    body = HEADER.pack(magic, seq, count) + items
    return body + struct.pack("<I", zlib.crc32(body))


def synth(n_records: int = 6, per: int = 8, drop: int | None = None, flip: int | None = None, jump: int | None = None,
          overflow: int = 0, tick: tuple[int, int] = (1953125, 14649)) -> bytes:
    out = bytearray()
    seq = 0
    k = 0
    for rec in range(n_records):
        items = bytearray()
        for _ in range(per):
            t = (k * tick[0]) // tick[1] + (1000 if jump is not None and k >= jump else 0)
            items += SAMPLE.pack(t, 0.0, 0.0, 0.0, 0.0, 0.0, -9.8, 0xC0)
            k += 1
        data = record(DATA_MAGIC, seq, bytes(items), per)
        if flip == rec:
            data = data[:20] + bytes([data[20] ^ 1]) + data[21:]
        if drop != rec:
            out += data
        seq += 1
    f = [0] * len(STATUS_FIELDS)
    f[0] = overflow
    out += record(STATUS_MAGIC, seq, struct.pack(f"<{len(f)}I", *f), len(f))
    return bytes(out)


def selftest() -> int:
    assert zlib.crc32(b"123456789") == 0xCBF43926  # the CRC-32 check value (constants.hpp kCrc32PolyReflected)
    num, den, tol = tick_period()
    cases = [("clean", synth(), True, {}), ("crc failure", synth(flip=2), False, {"crc_failures": 1, "seq_gaps": 1}),
             ("sequence gap", synth(drop=3), False, {"seq_gaps": 1, "seq_lost": 1}),
             ("stamp gap", synth(jump=20), False, {"stamp_gaps": 1}),
             ("ring overflow count", synth(overflow=1), False, {})]
    for name, buf, valid, expect in cases:
        r = parse(buf)
        lines, ok = report(r, num, den, tol)
        gaps = stamp_gaps(r.samples, num, den, tol)[0]
        got = {"crc_failures": r.crc_failures, "seq_gaps": r.seq_gaps, "seq_lost": r.seq_lost, "stamp_gaps": gaps}
        for k, v in expect.items():
            assert got[k] == v, (name, k, got[k], v)
        if name == "clean":
            assert got == {"crc_failures": 0, "seq_gaps": 0, "seq_lost": 0, "stamp_gaps": 0} and len(r.samples) == 48
        if name == "ring overflow count":
            assert r.status[-1]["ring_overflows"] == 1
        assert ok == valid, (name, lines)
        assert ("LOG VALID" in lines[-1]) == valid and (valid or lines[-1].startswith("LOG INVALID")), lines[-1]
        print(f"selftest {name}: {lines[-1]}")
    junk = b"PANIC: x\r\n" + synth() + b"tail"
    r = parse(junk)
    assert r.leading_bytes == 10 and r.unframed_bytes == 4 and not report(r, num, den, tol)[1]
    print("selftest ok")
    return 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("dev")
    c.add_argument("seconds", type=float)
    c.add_argument("out", type=Path)
    d = sub.add_parser("decode")
    d.add_argument("log", type=Path)
    d.add_argument("--csv", type=Path)
    d.add_argument("--npz", type=Path)
    d.add_argument("--tick-num", type=int)
    d.add_argument("--tick-den", type=int)
    d.add_argument("--tol-us", type=int)
    sub.add_parser("selftest")
    a = p.parse_args(argv)
    if a.cmd == "capture":
        capture(a.dev, a.seconds, a.out)
        return 0
    if a.cmd == "selftest":
        return selftest()
    num, den, tol = tick_period()
    num, den, tol = a.tick_num or num, a.tick_den or den, a.tol_us if a.tol_us is not None else tol
    r = parse(a.log.read_bytes())
    lines, ok = report(r, num, den, tol)
    print("\n".join(lines))
    if a.csv:
        write_csv(r, a.csv)
    if a.npz:
        write_npz(r, a.npz)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
