#!/usr/bin/env python3
"""Bench only (ground-segment unit 12b): IMU interrupt timing with the board's link service idle and at its largest output.
Never run by CI.

  uv run python tests/regression/quad/L10/results/link_board/bench_load.py capture <out_prefix> [--load-s N]
  uv run python tests/regression/quad/L10/results/link_board/bench_load.py analyze <out_prefix>

The board runs the bench-stats variant of the flight image (MARV_FC3V2_BENCH_STATS=ON), disarmed or armed by Luis (the script
never arms or commands a motor). The console (if00) is recorded throughout with each line's host arrival time. Phases:
  closed    the link port is not open: the service runs, every frame is dropped at the queue
  open      the link port is open on the serial port itself and only read: the heartbeat (1 Hz)
  load      parameter listings without pause: PARAM_REQUEST_LIST every REQUEST_S, which restarts the listing before it ends.
            A listing is the largest output the service has (two frames per parameter per loop pass, paced only by the room
            in the send queue), so this is the link at the transport's throughput. The host only reads during the load and
            decodes afterwards, so its decoding does not slow the board's output
  after     open and only read again
The link traffic does not go through the helper here (its WebSocket relay adds nothing on the board side and its ground log
would hold every byte). analyze prints, per phase, the window maxima of the bench_stages line (interrupt-entry spacing, the IMU
interrupt `irq`, the USBCTRL chain `usb_irq`) and the frames, bytes and sequence gaps of the load, then the EMB-3 report of
tools/target/bench_stats.py over the whole capture.
Writes <out_prefix>.console.log, <out_prefix>.phases.json; analyze writes <out_prefix>.analysis.txt.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import link_common as lc  # noqa: E402

IDLE_S = 20  # labelled harness value: seconds of each idle phase
LOAD_S = 60  # labelled harness value: seconds of the load phase (the runbook's 60 s)
REQUEST_S = 0.02  # labelled harness value: the load's request period, shorter than one listing (38 ms on this board), so the
                  # service never runs out of output (a request restarts the listing at index 0)
CLK_DEFAULT_HZ = 150000000  # clk_sys of the flight banner when the capture lacks it (SDK clock init, 150 MHz)


def load_phase(pipe, d, seconds: float):
    """The link at the service's largest output for `seconds`: (start, end) host times and the received frames' statistics."""
    raw = bytearray()
    request = d.MAVLink_param_request_list_message(*lc.VEHICLE).pack(d.MAVLink(None, srcSystem=lc.GCS_SYSTEM,
                                                                                srcComponent=lc.GCS_COMPONENT))
    requests = 0
    t0 = time.time()
    end = time.monotonic() + seconds
    next_request = 0.0
    while time.monotonic() < end:
        if time.monotonic() >= next_request:
            pipe.write(request)
            requests += 1
            next_request = time.monotonic() + REQUEST_S
        raw += pipe.read(lc.POLL_S)
    t1 = time.time()
    parser = d.MAVLink(None)
    parser.robust_parsing = True
    msgs = [m for m in parser.parse_buffer(bytes(raw)) or [] if m.get_type() != "BAD_DATA"]
    seqs = [m.get_seq() for m in msgs]
    stats = {"seconds": t1 - t0, "requests": requests, "frames": len(msgs), "bytes": len(raw),
             "seq_gaps": sum(((b - a) % 256) - 1 for a, b in zip(seqs, seqs[1:])),
             "param_values": sum(1 for m in msgs if m.get_type() == "PARAM_VALUE"),
             "records": sum(1 for m in msgs if m.get_type() == "MARV_PARAM_RECORD"),
             "heartbeats": sum(1 for m in msgs if m.get_type() == "HEARTBEAT")}
    stats["frames_per_s"] = stats["frames"] / stats["seconds"]
    stats["bytes_per_s"] = stats["bytes"] / stats["seconds"]
    return (t0, t1), stats


def write_capture(out: Path, console, phases, load) -> None:
    Path(str(out) + ".console.log").write_bytes(bytes(console.raw))
    Path(str(out) + ".phases.json").write_text(json.dumps({"phases": phases, "load": load,
                                                           "line_times": [lt for lt, _ in console.lines]}, indent=1) + "\n",
                                               encoding="utf-8")


def capture(out: Path, load_s: float) -> int:
    d = lc.load_dialect()
    console = lc.ConsoleReader()
    console.start()
    phases, load = {}, {}
    try:
        lc.wait_console(console)
        t = time.time()
        time.sleep(IDLE_S)
        phases["closed"] = (t, time.time())
        pipe = lc.SerialPipe()
        client = lc.Client(d, pipe)
        t = time.time()
        client.listen(IDLE_S)
        phases["open"] = (t, time.time())
        phases["load"], load = load_phase(pipe, d, load_s)
        print("load:", json.dumps(load), flush=True)
        t = time.time()
        client.listen(IDLE_S)
        phases["after"] = (t, time.time())
        pipe.close()
    finally:
        console.stop()
        write_capture(out, console, phases, load)
    return 0


def analyze(out: Path) -> int:
    meta = json.loads(Path(str(out) + ".phases.json").read_text(encoding="utf-8"))
    text = Path(str(out) + ".console.log").read_bytes().decode(errors="replace")
    lines = text.split("\n")
    times = meta["line_times"]
    m_clk = re.search(r"clk_sys (\d+) Hz", text)
    clk = int(m_clk.group(1)) if m_clk else CLK_DEFAULT_HZ
    us = lambda cyc: cyc * 1e6 / clk  # noqa: E731
    report = [f"clk_sys {clk} Hz{'' if m_clk else ' (default: no banner in the capture)'}"]
    rows = {k: [] for k in meta["phases"]}
    for i, line in enumerate(lines):
        if not line.startswith("bench_stages") or i >= len(times):
            continue
        arrive = times[i]
        sp = re.search(r"\| wspacing n=\d+ sum=\d+ min=(\d+) max=(\d+)", line)
        irq = re.search(r"\| irq n=\d+ max=\d+ wmax=(\d+)", line)
        usb = re.search(r"\| usb_irq n=\d+ max=\d+ wmax=(\d+)", line)
        if not (sp and irq and usb):
            continue
        for name, (a, b) in meta["phases"].items():
            if arrive - 1.0 >= a and arrive <= b:  # the line's one-second window lies inside the phase
                rows[name].append((int(sp.group(2)), int(sp.group(1)), int(irq.group(1)), int(usb.group(1))))
    for name, r in rows.items():
        if not r:
            report.append(f"{name}: no whole windows")
            continue
        report.append(f"{name}: {len(r)} windows; interrupt-entry spacing window max {us(max(x[0] for x in r)):.2f} us, "
                      f"min {us(min(x[1] for x in r if x[1])):.2f} us; IMU interrupt (irq) window max "
                      f"{us(max(x[2] for x in r)):.2f} us; USBCTRL chain window max {us(max(x[3] for x in r)):.2f} us")
    report.append("load: " + json.dumps(meta["load"]))
    rate = re.findall(r"rate executions/s=[\d.]+ executions=(\d+) timing_faults=(\d+)", text)
    att = re.findall(r"attitude faults=(\d+) timing_faults=(\d+)", text)
    imu = re.findall(r"total samples=(\d+) ovr=(\d+) spi=(\d+) partial=(\d+) clamp=(\d+) order=(\d+) empty=(\d+)", text)
    arming = re.findall(r"arming=(\w+)", text)
    report.append(f"rate executions, timing_faults first {rate[0]} last {rate[-1]}")
    report.append(f"attitude faults, timing_faults last {att[-1]}; est_valid last {re.findall(r'est_valid=(\d)', text)[-1]}")
    report.append(f"imu samples ovr spi partial clamp order empty first {imu[0]} last {imu[-1]}")
    report.append(f"arming states seen {sorted(set(arming))}; usb_reset_refused last {re.findall(r'usb_reset_refused=(\d+)', text)[-1]}")
    p = subprocess.run([sys.executable, str(lc.ROOT / "tools" / "target" / "bench_stats.py"), "report", str(out) + ".console.log",
                        "--profile", str(lc.ROOT / "sensors" / "profiles" / "madflight_fc3v2_revc.yaml")],
                       capture_output=True, text=True, check=False)
    report.append("== tools/target/bench_stats.py report (whole capture)")
    report.append(p.stdout.rstrip() + (("\n" + p.stderr.rstrip()) if p.returncode else ""))
    body = "\n".join(report) + "\n"
    Path(str(out) + ".analysis.txt").write_text(body, encoding="utf-8")
    print(body, end="")
    return p.returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("capture", "analyze"))
    ap.add_argument("out_prefix")
    ap.add_argument("--load-s", type=float, default=LOAD_S)
    args = ap.parse_args()
    out = Path(args.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    return capture(out, args.load_s) if args.mode == "capture" else analyze(out)


if __name__ == "__main__":
    sys.exit(main())
