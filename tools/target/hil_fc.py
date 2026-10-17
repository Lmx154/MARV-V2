#!/usr/bin/env python3
"""FC3v2 bench HIL image (MARV_FC3V2_HIL, HIL stage S3a), minimal host driver for the smoke test. Bench only; never run by CI.

  python3 tools/target/hil_fc.py smoke /dev/ttyACM0 smoke.bin
      opens the port (DTR asserted), reads the hello record, then streams a still, level IMU (gyro 0, accel (0, 0, -g0) FRD,
      both valid bits) at the tick period the hello names, --batch samples per record, each record sent when its last
      sample's stamp is due on the host clock, with a ManualControl record every MC_PERIOD_US of stamp time: arm switch low
      and throttle 0 until ARM_AT_S, switch high (arm) until DISARM_AT_S, switch low until END_S. Then it waits DRAIN_S,
      writes the uplink bytes exactly as received to OUT, prints the hello, every status record and a summary (also to
      OUT.summary.txt) and a verdict line, "SMOKE OK" or "SMOKE NOT OK: <reasons>". Exit status 0 only for SMOKE OK.

What the verdict needs: one ManualControl-arrival record per ManualControl record sent; the board's counters show every record and sample received and ticked with no link fault; one
DShot report per rate tick (ticks 0, d, 2d, ... for the rate divisor d of the hello); every DShot value 0 before ARM_AT_S (less
one record's span: the arm frame goes out ahead of the record whose last sample reaches ARM_AT_S) and from
DISARM_AT_S + DISARM_SLACK_S on, some value not 0 between (armed: the mixer's idle floor at zero throttle); no
IMU deadline miss while the stream runs, exactly one after it stops (no record for D_HIL); no rate-loop timing fault. The
cycle counts are printed, not judged (EMB-3 is the S3 comparison's).

Record formats: fw/hal/target/include/marv/hal_target/hil_format.hpp (the S1 framing: magic, u32 sequence, u32 count, items,
CRC-32; tools/target/hil_link.py's record and open_port are reused). Standard library only.
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path

from hil_link import HEADER, WORD, open_port, record

SAMPLE_MAGIC = b"MHSM"
MANUAL_MAGIC = b"MHMC"
HELLO_MAGIC = b"MHHS"
DSHOT_MAGIC = b"MHDS"
ARRIVAL_MAGIC = b"MHMA"
STATUS_MAGIC = b"MHST"
SAMPLE = struct.Struct("<QQ6fI")  # tick, t_us, gyro xyz, accel xyz, flags
MANUAL = struct.Struct("<I4f16ff")  # frames, roll pitch yaw throttle, channels 1..16, link_quality
DSHOT_ITEM = struct.Struct("<Q4HQ")  # tick, DShot motor 1..4, board_us
ARRIVAL_ITEM = struct.Struct("<IIQ")  # frames, downlink seq, board t_us
# hil_format.hpp HelloField and StatusField, in order.
HELLO_FIELDS = ("schema_hash_low", "schema_hash_high", "param_count", "param_values_crc", "tick_period_num_us", "tick_period_den",
                "rate_loop_divisor", "max_batch", "deadline_us", "clk_sys_hz", "version_bytes")
STATUS_FIELDS = ("sample_records", "samples", "manual_records", "manual_refused", "crc_failures", "bad_headers",
                 "seq_gaps", "seq_lost", "tick_gaps", "bytes_dropped", "rx_bytes", "ticks_run", "queue_overflows",
                 "reports", "report_overflows", "arming_state", "rate_executions", "rate_timing_faults",
                 "attitude_faults", "attitude_timing_faults", "deadline_misses", "max_tick_cycles", "max_frame_cycles",
                 "max_arrival_gap_us", "max_send_us", "board_time_us")
# fw/arming/include/marv/arming/arming.hpp ArmState, in order.
ARM_STATES = ("boot", "switch_off", "switch_not_cycled", "throttle_up", "imu_stale", "imu_invalid", "saturated", "no_link",
              "failsafe", "motor_test", "armed", "armed_hold", "armed_failsafe_centred", "estimator_invalid", "calibrating",
              "not_upright")
LINK_FAULTS = ("manual_refused", "crc_failures", "bad_headers", "seq_gaps", "tick_gaps", "bytes_dropped", "queue_overflows",
               "report_overflows")

G0 = 9.80665  # standard gravity, m/s^2 (3rd CGPM 1901; fw/prim constants.hpp kStandardGravity): the still IMU's |f|
FLAGS_VALID = (1 << 6) | (1 << 7)  # imu_flag GyroValid | AccelValid (fw/types/include/marv/types/imu_sample.hpp)
CHANNELS = 16  # CRSF RC channels (fw/prim constants.hpp kCrsfRcChannels)
ARM_INDEX = 4  # the fc3v2 table's rc_channel_arm 5, index 4 (manual_control.hpp: "the arm switch is channel 5, index 4")
THROTTLE_INDEX = 2  # the fc3v2 table's rc_channel_throttle 3 (AETR), index 2
US_PER_S = 1_000_000
# Scenario values of this smoke run (not flight parameters): the phases in seconds of sample-stamp time, chosen so the
# boot gyro calibration (gyro_cal_duration_us 1.25 s) and the link recovery (failsafe_recovery_us 0.5 s) are done before
# the arm edge; the ManualControl period, the ExpressLRS 250 Hz packet rate, far inside failsafe_loss_us (100 ms).
ARM_AT_S = 3.0
DISARM_AT_S = 6.0
END_S = 7.0
DISARM_SLACK_S = 0.05  # the disarm reaches the board within one ManualControl period plus the link's round trip (S1: < 3.4 ms)
MC_PERIOD_US = 4000
HELLO_WAIT_S = 2.0  # after opening the port: the board sends the hello as soon as it sees DTR
DRAIN_S = 2.5  # after the last record: more than two status periods, so a status after D_HIL arrives
STALL_S = 5.0  # a write that makes no progress for this long aborts the run


@dataclass
class Uplink:
    hello: dict | None = None
    version: str = ""
    reports: list[tuple[int, tuple[int, ...]]] = field(default_factory=list)
    report_board_us: list[int] = field(default_factory=list)
    arrivals: list[tuple[int, int, int]] = field(default_factory=list)  # frames, downlink seq, board t_us
    status: list[dict] = field(default_factory=list)
    status_reports_before: list[int] = field(default_factory=list)
    crc_failures: int = 0
    bad_headers: int = 0
    seq_gaps: int = 0
    unframed_bytes: int = 0
    text: bytes = b""


def parse_uplink(buf: bytes) -> Uplink:
    """Every framed uplink record in `buf`; the sequence restarts at 0 with each hello."""
    r = Uplink()
    n = len(buf)
    pos = start = 0
    seq_last: int | None = None
    text = bytearray()
    magics = (HELLO_MAGIC, DSHOT_MAGIC, ARRIVAL_MAGIC, STATUS_MAGIC)
    while True:
        cand = [i for i in (buf.find(m, pos) for m in magics) if i >= 0]
        if not cand:
            break
        i = min(cand)
        if n - i < HEADER.size:
            break
        magic, seq, count = HEADER.unpack_from(buf, i)
        legal = {HELLO_MAGIC: count >= len(HELLO_FIELDS), DSHOT_MAGIC: count >= 1, ARRIVAL_MAGIC: count == 1,
                 STATUS_MAGIC: count == len(STATUS_FIELDS)}[magic]
        if not legal:
            r.bad_headers += 1
            pos = i + 1
            continue
        item = {DSHOT_MAGIC: DSHOT_ITEM.size, ARRIVAL_MAGIC: ARRIVAL_ITEM.size}.get(magic, WORD)
        end = i + HEADER.size + count * item + WORD
        if end > n:
            break
        if zlib.crc32(buf[i:end - WORD]) != struct.unpack_from("<I", buf, end - WORD)[0]:
            r.crc_failures += 1
            pos = i + 1
            continue
        if i > start:
            r.unframed_bytes += i - start
            text.extend(buf[start:i][:256])
        if magic == HELLO_MAGIC:
            seq_last = None
        if (seq_last is None and seq != 0) or (seq_last is not None and seq != (seq_last + 1) & 0xFFFFFFFF):
            r.seq_gaps += 1
        seq_last = seq
        body = buf[i + HEADER.size:end - WORD]
        if magic == HELLO_MAGIC:
            words = struct.unpack(f"<{count}I", body)
            r.hello = dict(zip(HELLO_FIELDS, words))
            r.version = body[len(HELLO_FIELDS) * WORD:][:r.hello["version_bytes"]].decode(errors="replace")
        elif magic == DSHOT_MAGIC:
            for k in range(count):
                tick, d1, d2, d3, d4, board_us = DSHOT_ITEM.unpack_from(body, k * DSHOT_ITEM.size)
                r.reports.append((tick, (d1, d2, d3, d4)))
                r.report_board_us.append(board_us)
        elif magic == ARRIVAL_MAGIC:
            r.arrivals.append(ARRIVAL_ITEM.unpack(body))
        else:
            r.status.append(dict(zip(STATUS_FIELDS, struct.unpack(f"<{count}I", body))))
            r.status_reports_before.append(len(r.reports))
        pos = start = end
    r.text = bytes(text)
    return r


def stamp_us(tick: int, num: int, den: int) -> int:
    """The SIL's exact sample stamp, floor(tick x num / den) us (decision 0046 item 1)."""
    return tick * num // den


def sample_record(seq: int, first: int, batch: int, num: int, den: int) -> bytes:
    items = b"".join(SAMPLE.pack(k, stamp_us(k, num, den), 0.0, 0.0, 0.0, 0.0, 0.0, -G0, FLAGS_VALID)
                     for k in range(first, first + batch))
    return record(SAMPLE_MAGIC, seq, items, batch)


def manual_record(seq: int, frames: int, t_s: float) -> bytes:
    arm = 1.0 if ARM_AT_S <= t_s < DISARM_AT_S else -1.0
    channels = [0.0] * CHANNELS
    channels[ARM_INDEX] = arm
    channels[THROTTLE_INDEX] = -1.0
    return record(MANUAL_MAGIC, seq, MANUAL.pack(frames, 0.0, 0.0, 0.0, 0.0, *channels, 1.0), 1)


def status_line(s: dict, clk: int) -> str:
    state = ARM_STATES[s["arming_state"]] if s["arming_state"] < len(ARM_STATES) else f"code {s['arming_state']}"
    us = lambda c: c * US_PER_S / clk if clk else float("nan")  # noqa: E731
    return (f"status t={s['board_time_us']} records={s['sample_records']} samples={s['samples']} mc={s['manual_records']} "
            f"ticks={s['ticks_run']} reports={s['reports']} arming={state} rate_exec={s['rate_executions']} "
            f"rate_timing_faults={s['rate_timing_faults']} att_faults={s['attitude_faults']}/{s['attitude_timing_faults']} "
            f"deadline_misses={s['deadline_misses']} max_tick={s['max_tick_cycles']}cy ({us(s['max_tick_cycles']):.1f} us) "
            f"max_record={s['max_frame_cycles']}cy ({us(s['max_frame_cycles']):.1f} us) max_gap={s['max_arrival_gap_us']}us "
            f"max_send={s['max_send_us']}us faults=" + ",".join(f"{k}={s[k]}" for k in LINK_FAULTS if s[k]))


def analyse(up: Uplink, sent_records: int, sent_samples: int, sent_manual: int, batch: int) -> tuple[list[str], bool]:
    out: list[str] = []
    reasons: list[str] = []
    h = up.hello
    if h is None:
        return ["no hello record"], False
    num, den, d = h["tick_period_num_us"], h["tick_period_den"], h["rate_loop_divisor"]
    clk = h["clk_sys_hz"]
    out.append(f"hello: version {up.version!r}  param schema hash 0x{(h['schema_hash_high'] << 32) | h['schema_hash_low']:016x}  "
               f"params {h['param_count']}  values CRC 0x{h['param_values_crc']:08x}  tick {num}/{den} us  rate divisor {d}  max batch {h['max_batch']}  "
               f"D_HIL {h['deadline_us']} us  clk_sys {clk} Hz")
    out.extend(status_line(s, clk) for s in up.status)
    out.append(f"sent: {sent_records} sample records ({batch} per record), {sent_samples} samples, {sent_manual} ManualControl records")
    out.append(f"uplink host-side: CRC failures {up.crc_failures}  bad headers {up.bad_headers}  sequence gaps {up.seq_gaps}  "
               f"unframed bytes {up.unframed_bytes}")
    text = "".join(c if c.isprintable() else "." for c in up.text.decode(errors="replace")).strip()
    if text:
        out.append("text outside the records: " + text[:300])
    for name in ("crc_failures", "bad_headers", "seq_gaps", "unframed_bytes"):
        if getattr(up, name):
            reasons.append(f"uplink {name} {getattr(up, name)}")
    if not up.status:
        reasons.append("no status record")
        out.append("SMOKE NOT OK: " + "; ".join(reasons))
        return out, False
    last = up.status[-1]
    if last["sample_records"] != sent_records or last["samples"] != sent_samples or last["manual_records"] != sent_manual:
        reasons.append(f"board received {last['sample_records']} records / {last['samples']} samples / "
                       f"{last['manual_records']} ManualControl, host sent {sent_records} / {sent_samples} / {sent_manual}")
    if last["ticks_run"] != sent_samples:
        reasons.append(f"board ran {last['ticks_run']} ticks for {sent_samples} samples")
    for k in LINK_FAULTS:
        if last[k]:
            reasons.append(f"board {k} {last[k]}")
    if last["rate_timing_faults"]:
        reasons.append(f"rate timing faults {last['rate_timing_faults']}")
    during = [s for s in up.status if s["ticks_run"] < sent_samples]
    if any(s["deadline_misses"] for s in during):
        reasons.append("IMU deadline miss while the stream ran")
    if last["deadline_misses"] != 1:
        reasons.append(f"{last['deadline_misses']} IMU deadline misses after the stream stopped, expected 1 (D_HIL)")
    mism = [k for k, s in enumerate(up.status) if s["reports"] != up.status_reports_before[k]]
    if mism:
        reasons.append(f"{len(mism)} status records whose report count differs from the reports decoded before them")

    if len(up.arrivals) != sent_manual or [f for f, _, _ in up.arrivals] != list(range(1, sent_manual + 1)):
        reasons.append(f"{len(up.arrivals)} ManualControl arrival records for {sent_manual} sent, or not frames 1, 2, ...")
    if any(b <= a for a, b in zip(up.report_board_us, up.report_board_us[1:])):
        reasons.append("DShot report board times not increasing")
    ticks = [t for t, _ in up.reports]
    expect = list(range(0, sent_samples, d))
    if ticks != expect:
        reasons.append(f"report ticks are not 0, {d}, 2 x {d}, ... below {sent_samples} "
                       f"({len(ticks)} reports, {len(expect)} expected)")
    t_s = lambda tick: stamp_us(tick, num, den) / US_PER_S  # noqa: E731
    armed = [(t, v) for t, v in up.reports if any(v)]
    record_s = batch * num / den / US_PER_S  # a ManualControl goes out before the record whose last sample reaches its time
    early = [t for t, _ in armed if t_s(t) < ARM_AT_S - record_s]
    late = [t for t, _ in armed if t_s(t) >= DISARM_AT_S + DISARM_SLACK_S]
    if not armed:
        reasons.append("no DShot value above 0: the board never armed")
    else:
        out.append(f"armed reports: {len(armed)}, ticks {armed[0][0]} .. {armed[-1][0]} "
                   f"(stamps {t_s(armed[0][0]):.4f} .. {t_s(armed[-1][0]):.4f} s), first values {armed[0][1]}, "
                   f"last values {armed[-1][1]}")
    if early or late:
        reasons.append(f"DShot above 0 outside the armed phase: {len(early)} before {ARM_AT_S} s, "
                       f"{len(late)} after {DISARM_AT_S + DISARM_SLACK_S} s")
    period_us = num / den
    out.append(f"EMB-3 inputs (not judged here): longest record of {batch} ticks {last['max_frame_cycles']} cycles = "
               f"{last['max_frame_cycles'] * US_PER_S / clk:.1f} us against {batch} tick periods = {batch * period_us:.1f} us; "
               f"longest tick {last['max_tick_cycles'] * US_PER_S / clk:.1f} us")
    out.append("SMOKE OK" if not reasons else "SMOKE NOT OK: " + "; ".join(reasons))
    return out, not reasons


def smoke(dev: str, out: Path, batch: int) -> int:
    import os
    import select

    sys.setswitchinterval(2e-5)  # the sender spins; the reader must get the GIL promptly
    fd = open_port(dev)
    buf = bytearray()
    lock = threading.Lock()
    stop = threading.Event()

    def reader() -> None:
        with open(out, "wb") as f:
            while not stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    b = os.read(fd, 1 << 16)
                except BlockingIOError:
                    continue
                f.write(b)
                with lock:
                    buf.extend(b)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    deadline = time.monotonic() + HELLO_WAIT_S
    hello = None
    while hello is None and time.monotonic() < deadline:
        time.sleep(0.05)
        with lock:
            hello = parse_uplink(bytes(buf)).hello
    if hello is None:
        stop.set()
        th.join()
        os.close(fd)
        print("SMOKE NOT OK: no hello record within", HELLO_WAIT_S, "s (is the MARV_FC3V2_HIL image running?)")
        return 1
    num, den = hello["tick_period_num_us"], hello["tick_period_den"]
    if batch > hello["max_batch"]:
        raise SystemExit(f"hil_fc: --batch {batch} exceeds the image's limit {hello['max_batch']}")

    records = int(END_S * US_PER_S * den / num) // batch
    seq = frames = 0
    next_mc_us = 0
    clock = time.perf_counter_ns
    t0 = clock() + 50_000_000
    aborted = ""

    def write(rec: bytes) -> None:
        nonlocal aborted
        view = memoryview(rec)
        last_progress = time.monotonic()
        while view and not aborted:
            try:
                view = view[os.write(fd, view):]
                last_progress = time.monotonic()
            except BlockingIOError:
                select.select([], [fd], [], 0.05)
                if time.monotonic() - last_progress > STALL_S:
                    aborted = f"write stalled {STALL_S} s"

    sent = 0
    for j in range(records):
        last_tick = (j + 1) * batch - 1
        due = t0 + stamp_us(last_tick, num, den) * 1000
        while True:
            rem = due - clock()
            if rem <= 0:
                break
            if rem > 400_000:
                time.sleep((rem - 300_000) / 1e9)
        while next_mc_us <= stamp_us(last_tick, num, den):
            frames += 1
            write(manual_record(seq, frames, next_mc_us / US_PER_S))
            seq += 1
            next_mc_us += MC_PERIOD_US
        write(sample_record(seq, j * batch, batch, num, den))
        seq += 1
        sent += 1
        if aborted:
            break
    time.sleep(DRAIN_S)
    stop.set()
    th.join()
    os.close(fd)

    up = parse_uplink(out.read_bytes())
    lines, ok = analyse(up, sent, sent * batch, frames, batch)
    if aborted:
        lines[-1] = "SMOKE NOT OK: " + aborted
        ok = False
    text = "\n".join(lines)
    out.with_suffix(".summary.txt").write_text(text + "\n")
    print(text)
    return 0 if ok else 1


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("smoke")
    s.add_argument("dev")
    s.add_argument("out", type=Path)
    s.add_argument("--batch", type=int, default=4, help="samples per record (default 4, the S1 configuration)")
    a = p.parse_args(argv)
    if a.batch < 1:
        raise SystemExit("hil_fc: --batch must be at least 1")
    return smoke(a.dev, a.out, a.batch)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
