#!/usr/bin/env python3
"""Bench only (ground-segment unit 11a): IMU interrupt timing with every telemetry stream of the board's link at its cap.
Never run by CI.

  uv run python tests/regression/quad/L10/results/telemetry/bench_telemetry.py capture <out_prefix> [--load-s N]
  uv run python tests/regression/quad/L10/results/telemetry/bench_telemetry.py analyze <out_prefix>

The method of ../link_board/bench_load.py (unit 12b), with the link's load replaced by the telemetry streams: the board runs the
bench-stats variant of the flight image (MARV_FC3V2_BENCH_STATS=ON), disarmed (the script never arms or commands a motor), and
the console (if00) is recorded throughout with each line's host arrival time. Phases:
  closed    the link port is not open
  open      the link port is open and only read: the heartbeat (1 Hz)
  load      every stream asked at an interval of 1 us, far above every cap, so each is granted its cap (MESSAGE_INTERVAL is read
            back for each), then only read for the load's seconds and decoded afterwards; every stream is then stopped (-1)
  after     open and only read again
analyze is bench_load.py's (the window maxima of interrupt-entry spacing, IMU interrupt and USBCTRL chain per phase, then the
EMB-3 report of tools/target/bench_stats.py over the whole capture), with the load's statistics: frames, bytes and sequence
gaps, and per stream the granted interval, the frames received and the rate from the board's own stamps.
Writes <out_prefix>.console.log, <out_prefix>.phases.json; analyze writes <out_prefix>.analysis.txt.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "link_board"))
import bench_load as bl  # noqa: E402
import link_common as lc  # noqa: E402

LOAD_S = 60  # labelled harness value: seconds of the load phase (the packet's 60 s)
ASK_US = 1  # labelled harness value: the interval asked for every stream, below every cap's interval
STREAMS = ("SYS_STATUS", "ATTITUDE", "ATTITUDE_QUATERNION", "RC_CHANNELS", "ATTITUDE_TARGET", "HIGHRES_IMU",
           "ACTUATOR_OUTPUT_STATUS", "MARV_FC_STATUS", "MARV_ESTIMATOR", "MARV_IDENTITY")
STAMP_US = {"time_usec": 1, "time_boot_ms": 1000}  # a stream's stamp field and its resolution in us


def command(client, d, cmd, *params):
    at = client.mark()
    values = [float(p) for p in params] + [0.0] * (7 - len(params))
    client.send(d.MAVLink_command_long_message(*lc.VEHICLE, cmd, 0, *values))
    return client.wait(lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd, f"COMMAND_ACK {cmd}", at)


def interval_of(client, d, msg_id):
    at = client.mark()
    client.send(d.MAVLink_command_long_message(*lc.VEHICLE, d.MAV_CMD_REQUEST_MESSAGE, 0, float(d.MAVLINK_MSG_ID_MESSAGE_INTERVAL),
                                               float(msg_id), 0, 0, 0, 0, 0))
    return client.wait(lambda m: m.get_type() == "MESSAGE_INTERVAL" and m.message_id == msg_id, f"MESSAGE_INTERVAL {msg_id}",
                       at).interval_us


def stream_phase(client, pipe, d, seconds: float):
    """Every stream at its cap for `seconds`: (start, end) host times and the received frames' statistics."""
    granted = {}
    for name in STREAMS:
        msg_id = getattr(d, f"MAVLINK_MSG_ID_{name}")
        ack = command(client, d, d.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, ASK_US)
        assert ack.result == d.MAV_RESULT_ACCEPTED, (name, ack.result)
        granted[name] = interval_of(client, d, msg_id)
    raw = bytearray()
    t0 = time.time()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        raw += pipe.read(lc.POLL_S)
    t1 = time.time()
    for name in STREAMS:
        command(client, d, d.MAV_CMD_SET_MESSAGE_INTERVAL, getattr(d, f"MAVLINK_MSG_ID_{name}"), -1)
    parser = d.MAVLink(None)
    parser.robust_parsing = True
    msgs = [m for m in parser.parse_buffer(bytes(raw)) or [] if m.get_type() != "BAD_DATA"]
    seqs = [m.get_seq() for m in msgs]
    stats = {"seconds": t1 - t0, "frames": len(msgs), "bytes": len(raw),
             "seq_gaps": sum(((b - a) % 256) - 1 for a, b in zip(seqs, seqs[1:])),
             "heartbeats": sum(1 for m in msgs if m.get_type() == "HEARTBEAT"), "streams": {}}
    stats["frames_per_s"] = stats["frames"] / stats["seconds"]
    stats["bytes_per_s"] = stats["bytes"] / stats["seconds"]
    for name in STREAMS:
        got = [m for m in msgs if m.get_type() == name]
        row = {"granted_interval_us": granted[name], "granted_hz": 1e6 / granted[name] if granted[name] > 0 else 0.0,
               "frames": len(got), "host_hz": len(got) / stats["seconds"]}
        field = next((f for f in STAMP_US if got and f in type(got[0]).fieldnames), None)
        if field is not None and len(got) >= 2:
            times = [getattr(m, field) * STAMP_US[field] for m in got]
            row["board_hz"] = (len(got) - 1) * 1e6 / (times[-1] - times[0]) if times[-1] > times[0] else 0.0
            gaps = [b - a for a, b in zip(times, times[1:])]
            row["gap_min_us"], row["gap_max_us"] = min(gaps), max(gaps)
        stats["streams"][name] = row
    return (t0, t1), stats


def capture(out: Path, load_s: float) -> int:
    d = lc.load_dialect()
    console = lc.ConsoleReader()
    console.start()
    phases, load = {}, {}
    try:
        lc.wait_console(console)
        t = time.time()
        time.sleep(bl.IDLE_S)
        phases["closed"] = (t, time.time())
        pipe = lc.SerialPipe()
        client = lc.Client(d, pipe)
        t = time.time()
        client.listen(bl.IDLE_S)
        phases["open"] = (t, time.time())
        phases["load"], load = stream_phase(client, pipe, d, load_s)
        print("load:", json.dumps(load), flush=True)
        t = time.time()
        client.listen(bl.IDLE_S)
        phases["after"] = (t, time.time())
        pipe.close()
    finally:
        console.stop()
        bl.write_capture(out, console, phases, load)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("capture", "analyze"))
    ap.add_argument("out_prefix")
    ap.add_argument("--load-s", type=float, default=LOAD_S)
    args = ap.parse_args()
    out = Path(args.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    return capture(out, args.load_s) if args.mode == "capture" else bl.analyze(out)


if __name__ == "__main__":
    sys.exit(main())
