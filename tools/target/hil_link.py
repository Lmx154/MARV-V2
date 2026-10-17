#!/usr/bin/env python3
"""FC3v2 bench HIL link measurement, host side (HIL stage S1). Bench only; never run by CI.

  python3 tools/target/hil_link.py run /dev/ttyACM1 60 --rate 7500 --batch 8 s1_b8.bin
      drives an image built with MARV_FC3V2_HIL_LINK for the given seconds, DTR asserted: sends framed downlink records of
      --batch samples at --rate samples/s (record j goes out when its last sample is due, t0 + ((j + 1) batch - 1) / rate),
      reads the uplink at the same time, writes the uplink bytes exactly as they arrive to OUT.bin, the host timing to
      OUT.sends.csv and OUT.chunks.csv, prints the summary (also to OUT.summary.txt) and a verdict line, "LINK OK" or
      "LINK NOT OK: <reasons>". Exit status 0 only for LINK OK.
  python3 tools/target/hil_link.py selftest
      the analysis against synthetic uplinks (a clean one and one with each defect), and the downlink encoder against a
      Python model of the board's parser.

Record formats: fw/hal/target/include/marv/hal_target/hil_link_format.hpp (the imu_log framing: magic, u32 sequence, u32
count, items, CRC-32). Standard library only.

Timing. All host times are time.perf_counter_ns() (CLOCK_MONOTONIC). The send time of tick k is the time just before the
os.write that carries its record; the receive time of its echo is the time os.read returned the chunk that completed the
report record. The record is sent when its last sample is due, so the round trip carries no batch-fill wait; it does carry the
host's kernel and USB host-controller queues and Python scheduling (tens of microseconds on an idle host, not measured).
"""

from __future__ import annotations

import argparse
import bisect
import os
import statistics
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path

DOWN_MAGIC = b"MHLD"
REPORT_MAGIC = b"MHLU"
STATUS_MAGIC = b"MHLS"
HEADER = struct.Struct("<4sII")  # magic, sequence, count
WORD = 4
SAMPLE = struct.Struct("<Q6fI")  # tick, gyro xyz, accel xyz, flags
REPORT_ITEM = struct.Struct("<Q4HQ")  # tick, 4 x DShot placeholder, board rx_us
# hil_link_format.hpp StatusField, in order.
STATUS_FIELDS = ("down_records", "down_samples", "crc_failures", "bad_headers", "seq_gaps", "seq_lost", "tick_gaps",
                 "bytes_dropped", "rx_bytes", "up_reports", "max_arrival_gap_us", "max_send_us", "board_time_us",
                 "max_batch")
MOTORS = 4
SETTLE_S = 0.5  # after opening the port, before the first record: the board resets its counters when it sees DTR
DRAIN_S = 2.5  # after the last record: more than two status periods, so a status that counts every sample arrives
STALL_S = 5.0  # a write that makes no progress for this long aborts the run


def record(magic: bytes, seq: int, items: bytes, count: int) -> bytes:
    body = HEADER.pack(magic, seq, count) + items
    return body + struct.pack("<I", zlib.crc32(body))


def down_record(seq: int, first_tick: int, batch: int) -> bytes:
    """A downlink record of `batch` samples, ticks first_tick..; the other fields are fixed functions of the tick."""
    items = bytearray()
    for k in range(first_tick, first_tick + batch):
        x = (k % 1000) * 1e-3
        items += SAMPLE.pack(k, x, -x, 0.5 * x, 0.0, 0.0, -9.80665, 0xC0)
    return record(DOWN_MAGIC, seq, bytes(items), batch)


def dshot_expected(tick: int) -> tuple[int, ...]:
    """hil_link.cpp: the low 16 bits of tick + motor index."""
    return tuple((tick + m) & 0xFFFF for m in range(MOTORS))


# --- uplink parsing ---------------------------------------------------------------------------------------------------


@dataclass
class Uplink:
    size: int = 0
    reports: list[tuple] = field(default_factory=list)  # (tick, (d1..d4), rx_us)
    report_end: list[int] = field(default_factory=list)  # byte offset just past each report record
    status: list[dict] = field(default_factory=list)
    status_reports_before: list[int] = field(default_factory=list)  # reports decoded when each status arrived
    crc_failures: int = 0
    bad_headers: int = 0
    seq_gaps: int = 0
    seq_lost: int = 0
    seq_first: int | None = None
    unframed_bytes: int = 0
    leading_bytes: int = 0
    truncated_tail: int = 0
    text: bytes = b""


def parse_uplink(buf: bytes) -> Uplink:
    r = Uplink(size=len(buf))
    n = len(buf)
    pos = 0
    start = 0
    seq_last: int | None = None
    seen = False
    text = bytearray()

    def unframed(a: int, b: int) -> None:
        if b > a:
            if seen:
                r.unframed_bytes += b - a
            else:
                r.leading_bytes += b - a
            if len(text) < 4096:
                text.extend(buf[a:b][:4096])

    while True:
        cand = [i for i in (buf.find(REPORT_MAGIC, pos), buf.find(STATUS_MAGIC, pos)) if i >= 0]
        if not cand:
            break
        i = min(cand)
        if n - i < HEADER.size:
            break
        magic, seq, count = HEADER.unpack_from(buf, i)
        legal = count == 1 if magic == REPORT_MAGIC else count == len(STATUS_FIELDS)
        if not legal:
            r.bad_headers += 1
            pos = i + 1
            continue
        item = REPORT_ITEM.size if magic == REPORT_MAGIC else WORD
        end = i + HEADER.size + count * item + WORD
        if end > n:
            r.truncated_tail = n - i
            unframed(start, i)
            start = n
            pos = n
            break
        if zlib.crc32(buf[i:end - WORD]) != struct.unpack_from("<I", buf, end - WORD)[0]:
            r.crc_failures += 1
            pos = i + 1
            continue
        unframed(start, i)
        seen = True
        if r.seq_first is None:
            r.seq_first = seq
            if seq:
                r.seq_gaps += 1
                r.seq_lost += seq
        else:
            expect = (seq_last + 1) & 0xFFFFFFFF
            if seq != expect:
                r.seq_gaps += 1
                r.seq_lost += (seq - expect) & 0xFFFFFFFF
        seq_last = seq
        body = buf[i + HEADER.size:end - WORD]
        if magic == REPORT_MAGIC:
            tick, d1, d2, d3, d4, rx = REPORT_ITEM.unpack(body)
            r.reports.append((tick, (d1, d2, d3, d4), rx))
            r.report_end.append(end)
        else:
            r.status.append(dict(zip(STATUS_FIELDS, struct.unpack(f"<{len(STATUS_FIELDS)}I", body))))
            r.status_reports_before.append(len(r.reports))
        pos = end
        start = end
    if pos < n and not r.truncated_tail:
        unframed(start, n)
    r.text = bytes(text)
    return r


# --- analysis ---------------------------------------------------------------------------------------------------------


def pct(sorted_v: list[float], p: float) -> float:
    if not sorted_v:
        return float("nan")
    return sorted_v[min(len(sorted_v) - 1, int(p / 100.0 * len(sorted_v)))]


def dist(v: list[float]) -> str:
    if not v:
        return "no data"
    s = sorted(v)
    return (f"min {s[0]:.1f}  median {statistics.median(s):.1f}  mean {statistics.fmean(s):.1f}  p99 {pct(s, 99):.1f}  "
            f"p99.9 {pct(s, 99.9):.1f}  max {s[-1]:.1f}")


@dataclass
class RunTimes:
    rate: float
    batch: int
    sent_records: int
    send_ns: list[int]  # per record: time before the write
    due_ns: list[int]  # per record: scheduled time
    chunk_end: list[int]  # cumulative uplink bytes after each read
    chunk_ns: list[int]  # time of each read
    stalls: int = 0  # writes that hit EAGAIN
    down_bytes: int = 0


def analyse(up: Uplink, t: RunTimes) -> tuple[list[str], bool]:
    out: list[str] = []
    reasons: list[str] = []
    sent_samples = t.sent_records * t.batch
    down_wire = t.down_bytes
    dur = (t.send_ns[-1] - t.send_ns[0]) / 1e9 if len(t.send_ns) > 1 else 0.0
    out.append(f"config: rate {t.rate:g} samples/s  batch {t.batch}  record {len(down_record(0, 0, t.batch))} bytes")
    out.append(f"downlink sent: {t.sent_records} records  {sent_samples} samples  {down_wire} bytes  over {dur:.3f} s  "
               f"= {sent_samples / dur if dur else 0:.1f} samples/s  {down_wire / dur / 1000 if dur else 0:.1f} kB/s on the wire")
    late = [(s - d) / 1e3 for s, d in zip(t.send_ns, t.due_ns)]
    out.append(f"send lateness vs schedule (us): {dist(late)}  writes that hit a full tty buffer {t.stalls}")

    last = up.status[-1] if up.status else None
    out.append(f"uplink received: {len(up.reports)} reports  {len(up.status)} status records  {up.size} bytes  "
               f"= {up.size / dur / 1000 if dur else 0:.1f} kB/s over the send window")
    out.append(f"uplink host-side: CRC failures {up.crc_failures}  bad headers {up.bad_headers}  sequence gaps {up.seq_gaps} "
               f"(records lost {up.seq_lost})  unframed bytes {up.unframed_bytes} (before the first record {up.leading_bytes})  "
               f"truncated tail {up.truncated_tail} bytes")
    if last:
        out.append("downlink board-side (last status): " + "  ".join(f"{k}={last[k]}" for k in STATUS_FIELDS))
    else:
        out.append("downlink board-side: no status record")
    text = "".join(c if c.isprintable() or c == "\n" else "." for c in up.text.decode(errors="replace")).strip()
    if text:
        out.append("text outside the records: " + text[:300])

    # Downlink, from the board's counters.
    if last is None:
        reasons.append("no status record: the board's downlink counters are unknown")
    else:
        for k in ("crc_failures", "bad_headers", "seq_gaps", "tick_gaps", "bytes_dropped"):
            if last[k]:
                reasons.append(f"downlink {k} {last[k]}")
        if last["down_samples"] != sent_samples or last["down_records"] != t.sent_records:
            reasons.append(f"board received {last['down_records']} records / {last['down_samples']} samples, "
                           f"host sent {t.sent_records} / {sent_samples}")
        if last["max_batch"] < t.batch:
            reasons.append(f"batch {t.batch} exceeds the image's limit {last['max_batch']}")
    # Uplink, from the host's parse and the board's report counter at each status.
    for name in ("crc_failures", "bad_headers", "seq_gaps", "unframed_bytes"):
        if getattr(up, name):
            reasons.append(f"uplink {name} {getattr(up, name)}")
    mism = [k for k, s in enumerate(up.status) if s["up_reports"] != up.status_reports_before[k]]
    out.append(f"report-counter cross-check: {len(up.status)} status records, {len(mism)} whose up_reports differs from the "
               f"reports decoded before them" + (f" (first: record {mism[0]} board {up.status[mism[0]]['up_reports']} "
                                                  f"host {up.status_reports_before[mism[0]]})" if mism else ""))
    if mism:
        reasons.append(f"{len(mism)} status records whose up_reports differs from the host's count")
    if len(up.reports) != sent_samples:
        reasons.append(f"{len(up.reports)} reports received, {sent_samples} sent")
    breaks = sum(1 for k, r in enumerate(up.reports) if r[0] != (up.reports[k - 1][0] + 1 if k else 0))
    wrong = sum(1 for r in up.reports if r[1] != dshot_expected(r[0]))
    if breaks:
        reasons.append(f"{breaks} echoed ticks that are not the previous tick + 1 (the first must be 0)")
    if wrong:
        reasons.append(f"{wrong} reports whose DShot placeholders differ from the expected values")
    out.append(f"report content: {breaks} tick discontinuities, {wrong} DShot placeholder mismatches in {len(up.reports)} reports")

    # Round trip: host send time of tick k to the host receive time of its echo.
    rtt: list[float] = []
    for (tick, _, _), end in zip(up.reports, up.report_end):
        j = tick // t.batch
        if j >= t.sent_records:
            continue
        c = bisect.bisect_left(t.chunk_end, end)
        if c < len(t.chunk_ns):
            rtt.append((t.chunk_ns[c] - t.send_ns[j]) / 1e3)
    out.append(f"round trip (us, {len(rtt)} echoes): {dist(rtt)}")
    arrivals = sorted({r[2] for r in up.reports})
    gaps = [float(b - a) for a, b in zip(arrivals, arrivals[1:])]
    out.append(f"board arrival spacing of downlink records (us, board clock, {len(gaps)} spacings): {dist(gaps)}; "
               f"expected {t.batch / t.rate * 1e6:.1f}")
    if last:
        out.append(f"board longest uplink write {last['max_send_us']} us; longest arrival gap {last['max_arrival_gap_us']} us")
    out.append("LINK OK" if not reasons else "LINK NOT OK: " + "; ".join(reasons))
    return out, not reasons


# --- run --------------------------------------------------------------------------------------------------------------


def open_port(dev: str) -> int:
    import fcntl
    import termios
    import tty

    tiocmbis, tiocm_dtr = 0x5416, 0x002  # Linux ioctl TIOCMBIS and the DTR bit (asm-generic/ioctls.h, termios.h)
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, a)
    fcntl.ioctl(fd, tiocmbis, struct.pack("I", tiocm_dtr))
    return fd


def run(dev: str, seconds: float, rate: float, batch: int, out: Path) -> int:
    import select

    sys.setswitchinterval(2e-5)  # the sender spins; the reader must get the GIL promptly to stamp arrivals
    fd = open_port(dev)
    import termios
    time.sleep(SETTLE_S)
    termios.tcflush(fd, termios.TCIOFLUSH)
    chunks_end: list[int] = []
    chunks_ns: list[int] = []
    stop = threading.Event()
    total = 0

    def reader() -> None:
        nonlocal total
        with open(out, "wb") as f:
            while not stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    b = os.read(fd, 1 << 16)
                except BlockingIOError:
                    continue
                now = time.perf_counter_ns()
                f.write(b)
                total += len(b)
                chunks_end.append(total)
                chunks_ns.append(now)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    records = int(seconds * rate) // batch
    period_ns = 1e9 / rate
    send_ns: list[int] = []
    due_ns: list[int] = []
    stalls = 0
    down_bytes = 0
    t0 = time.perf_counter_ns() + 50_000_000
    clock = time.perf_counter_ns
    aborted = ""
    for j in range(records):
        rec = down_record(j, j * batch, batch)
        due = t0 + int(((j + 1) * batch - 1) * period_ns)
        while True:
            rem = due - clock()
            if rem <= 0:
                break
            if rem > 400_000:
                time.sleep((rem - 300_000) / 1e9)
        send_ns.append(clock())
        due_ns.append(due)
        view = memoryview(rec)
        last_progress = time.monotonic()
        while view:
            try:
                n = os.write(fd, view)
                view = view[n:]
                last_progress = time.monotonic()
            except BlockingIOError:
                stalls += 1
                select.select([], [fd], [], 0.05)
                if time.monotonic() - last_progress > STALL_S:
                    aborted = f"write stalled {STALL_S} s at record {j}"
                    break
        if aborted:
            break
        down_bytes += len(rec)
    sent = len(send_ns)
    time.sleep(DRAIN_S)
    stop.set()
    th.join()
    os.close(fd)

    up = parse_uplink(out.read_bytes())
    times = RunTimes(rate, batch, sent, send_ns, due_ns, chunks_end, chunks_ns, stalls, down_bytes)
    lines, ok = analyse(up, times)
    if aborted:
        lines.insert(-1, "ABORTED: " + aborted)
        lines[-1] = "LINK NOT OK: " + aborted
        ok = False
    with open(out.with_suffix(".sends.csv"), "w") as f:
        f.write("record,first_tick,due_ns,send_ns\n")
        for j, (s, d) in enumerate(zip(send_ns, due_ns)):
            f.write(f"{j},{j * batch},{d},{s}\n")
    with open(out.with_suffix(".chunks.csv"), "w") as f:
        f.write("cumulative_bytes,read_ns\n")
        for e, n in zip(chunks_end, chunks_ns):
            f.write(f"{e},{n}\n")
    text = "\n".join(lines)
    out.with_suffix(".summary.txt").write_text(text + "\n")
    print(text)
    return 0 if ok else 1


# --- selftest ---------------------------------------------------------------------------------------------------------


class BoardModel:
    """The board's parser (hil_link.cpp parse/accept) in Python, to check the host's encoder: feed it the downlink bytes,
    read back the uplink bytes. The model takes the whole stream at once, and stamps every record with `rx_us`."""

    def __init__(self, max_batch: int = 16) -> None:
        self.max_batch = max_batch
        self.c = dict.fromkeys(STATUS_FIELDS, 0)
        self.up_seq = 0
        self.last_seq: int | None = None
        self.last_tick: int | None = None
        self.out = bytearray()

    def feed(self, data: bytes, rx_us: int = 0) -> None:
        pos = 0
        n = len(data)
        self.c["rx_bytes"] += n
        while True:
            if n - pos < 4:
                break
            if data[pos:pos + 4] != DOWN_MAGIC:
                self.c["bytes_dropped"] += 1
                pos += 1
                continue
            if n - pos < HEADER.size:
                break
            _, seq, count = HEADER.unpack_from(data, pos)
            if not 1 <= count <= self.max_batch:
                self.c["bad_headers"] += 1
                self.c["bytes_dropped"] += 1
                pos += 1
                continue
            end = pos + HEADER.size + count * SAMPLE.size + WORD
            if end > n:
                break
            if zlib.crc32(data[pos:end - WORD]) != struct.unpack_from("<I", data, end - WORD)[0]:
                self.c["crc_failures"] += 1
                self.c["bytes_dropped"] += 1
                pos += 1
                continue
            if self.last_seq is not None and seq != self.last_seq:
                self.c["seq_gaps"] += 1
                self.c["seq_lost"] += seq - self.last_seq
            self.last_seq = seq + 1
            self.c["down_records"] += 1
            self.c["down_samples"] += count
            for i in range(count):
                tick = SAMPLE.unpack_from(data, pos + HEADER.size + i * SAMPLE.size)[0]
                if self.last_tick is not None and tick != self.last_tick + 1:
                    self.c["tick_gaps"] += 1
                self.last_tick = tick
                self.out += record(REPORT_MAGIC, self.up_seq, REPORT_ITEM.pack(tick, *dshot_expected(tick), rx_us), 1)
                self.up_seq += 1
                self.c["up_reports"] += 1
            pos = end

    def status(self) -> bytes:
        s = record(STATUS_MAGIC, self.up_seq, struct.pack(f"<{len(STATUS_FIELDS)}I", *(
            self.c[k] if k != "max_batch" else self.max_batch for k in STATUS_FIELDS)), len(STATUS_FIELDS))
        self.up_seq += 1
        return s


def synthetic(batch: int, records: int, rate: float, defect: str = "") -> tuple[Uplink, RunTimes]:
    down = [down_record(j, j * batch, batch) for j in range(records)]
    if defect == "down_crc":
        down[2] = down[2][:20] + bytes([down[2][20] ^ 1]) + down[2][21:]
    if defect == "down_drop":
        down[3] = b""
    m = BoardModel()
    period_ns = 1e9 / rate
    send_ns, due_ns = [], []
    chunk_end, chunk_ns = [], []
    up = bytearray()
    for j, d in enumerate(down):
        m.feed(d, rx_us=int(((j + 1) * batch - 1) * period_ns / 1e3) + 100)  # a dropped record feeds nothing
        send_ns.append(int(((j + 1) * batch - 1) * period_ns))
        due_ns.append(send_ns[-1])
        up += m.out
        m.out = bytearray()
        chunk_end.append(len(up))
        chunk_ns.append(send_ns[-1] + 500_000)
    up += m.status()
    chunk_end.append(len(up))
    chunk_ns.append(chunk_ns[-1] + 1_000_000)
    buf = bytes(up)
    if defect == "up_crc":
        k = buf.find(REPORT_MAGIC, 400)
        buf = buf[:k + 14] + bytes([buf[k + 14] ^ 1]) + buf[k + 15:]
    if defect == "up_gap":
        k = buf.find(REPORT_MAGIC, 400)
        e = k + HEADER.size + REPORT_ITEM.size + WORD
        buf = buf[:k] + buf[e:]
    if defect == "dshot":
        k = buf.find(REPORT_MAGIC, 400)
        body = bytearray(buf[k:k + HEADER.size + REPORT_ITEM.size])
        body[HEADER.size + 8] ^= 1
        buf = buf[:k] + bytes(body) + struct.pack("<I", zlib.crc32(bytes(body))) + buf[k + HEADER.size + REPORT_ITEM.size + WORD:]
    if defect == "garbage":
        buf = buf[:300] + b"xx" + buf[300:]
    t = RunTimes(rate, batch, records, send_ns, due_ns, chunk_end, chunk_ns,
                 0, sum(len(d) for d in down))
    return parse_uplink(buf), t


def selftest() -> int:
    assert zlib.crc32(b"123456789") == 0xCBF43926  # the CRC-32 check value (constants.hpp kCrc32PolyReflected)
    assert SAMPLE.size == 36 and REPORT_ITEM.size == 24
    for batch in (1, 4, 8, 16):
        up, t = synthetic(batch, 40, 7500.0)
        lines, ok = analyse(up, t)
        assert ok and lines[-1] == "LINK OK", lines
        assert up.size == len(up.reports) * 40 + len(up.status) * (HEADER.size + len(STATUS_FIELDS) * WORD + WORD)
        # round trip: every echo is stamped 0.5 ms after its record's send
        assert any("round trip (us, %d echoes): min 500.0  median 500.0" % (40 * batch) in x for x in lines), lines
    # a board that sees a corrupt downlink record drops it: both sides' counters must show it
    for defect, expect in (("down_crc", "downlink crc_failures"), ("down_drop", "downlink"), ("up_crc", "uplink crc_failures"),
                           ("up_gap", "uplink seq_gaps"), ("dshot", "DShot placeholders"), ("garbage", "uplink unframed_bytes")):
        up, t = synthetic(4, 40, 7500.0, defect)
        lines, ok = analyse(up, t)
        assert not ok and lines[-1].startswith("LINK NOT OK") and expect in lines[-1], (defect, lines[-1])
        print(f"selftest {defect}: {lines[-1][:160]}")
    up, t = synthetic(4, 40, 7500.0)
    up.status = []
    assert not analyse(up, t)[1]
    t = synthetic(4, 40, 7500.0)[1]
    assert not analyse(parse_uplink(b""), t)[1]
    # batch above the image's limit: the board counts bad headers
    m = BoardModel(max_batch=8)
    m.feed(b"".join(down_record(j, j * 16, 16) for j in range(3)))
    assert m.c["bad_headers"] > 0 and m.c["down_records"] == 0
    # the model resyncs after a corrupt record and counts the lost sequence numbers
    m = BoardModel()
    d = [down_record(j, j * 4, 4) for j in range(6)]
    d[2] = d[2][:14] + bytes([d[2][14] ^ 4]) + d[2][15:]
    m.feed(b"".join(d))
    assert (m.c["crc_failures"], m.c["seq_gaps"], m.c["seq_lost"], m.c["down_records"], m.c["tick_gaps"]) == (1, 1, 1, 5, 1), m.c
    print("selftest ok")
    return 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("dev")
    r.add_argument("seconds", type=float)
    r.add_argument("--rate", type=float, required=True, help="downlink samples per second")
    r.add_argument("--batch", type=int, required=True, help="samples per downlink record")
    r.add_argument("out", type=Path)
    sub.add_parser("selftest")
    a = p.parse_args(argv)
    if a.cmd == "selftest":
        return selftest()
    if a.rate <= 0 or a.batch < 1 or a.seconds <= 0:
        raise SystemExit("hil_link: --rate and SECONDS must be positive and --batch at least 1")
    return run(a.dev, a.seconds, a.rate, a.batch, a.out)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
