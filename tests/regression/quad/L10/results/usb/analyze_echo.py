#!/usr/bin/env python3
"""Bench only (ground-segment unit 12a): IMU interrupt-entry spacing windows, idle against the echo traffic.

  analyze_echo.py <out_prefix>     (files written by echo_bench.py)

Each `bench_stages` console line carries the window spacing (min/max, clk_sys cycles) and the USBCTRL handler chain's window
maximum (`usb_irq wmax`) of the second before it. The line's host arrival time (phases.json) places that second in a phase:
idle before the echo, the echo of the check and sustained streams, idle after. A line whose second straddles a boundary is not
counted. clk_sys comes from the `motors ok:` line.
"""
import json
import re
import sys

prefix = sys.argv[1]
meta = json.load(open(prefix + ".phases.json"))
text = open(prefix + ".console.log", "rb").read().decode(errors="replace")
lines = text.split("\n")
times = meta["line_times"]
m_clk = re.search(r"clk_sys (\d+) Hz", text)
# The echo image does not reprint the banner when the console host connects to a running board (see README); then the
# 150 MHz clk_sys of the flight banner (set by the SDK clock init, 150 MHz) is used.
clk = int(m_clk.group(1)) if m_clk else 150000000
phases = meta["phases"]
rows = {k: [] for k in ("idle_before", "check", "sustained", "idle_after")}
for i, line in enumerate(lines):
    if not line.startswith("bench_stages") or i >= len(times):
        continue
    arrive = times[i]
    m = re.search(r"\| wspacing n=(\d+) sum=(\d+) min=(\d+) max=(\d+)", line)
    u = re.search(r"\| usb_irq n=\d+ max=\d+ wmax=(\d+)", line)
    if not m or not u:
        continue
    for name, (a, b) in phases.items():
        if arrive - 1.0 >= a and arrive <= b:
            rows[name].append((int(m.group(4)), int(m.group(3)), int(u.group(1)), arrive))
print(f"clk_sys {clk} Hz")
for name, r in rows.items():
    if not r:
        print(f"{name}: no whole windows")
        continue
    mx = max(x[0] for x in r)
    mn = min(x[1] for x in r if x[1])
    uw = max(x[2] for x in r)
    print(f"{name}: {len(r)} windows, spacing window max {mx} cycles = {mx * 1e6 / clk:.2f} us, window min {mn} cycles = "
          f"{mn * 1e6 / clk:.2f} us, USBCTRL chain window max {uw} cycles = {uw * 1e6 / clk:.2f} us")
rate = re.findall(r"rate executions/s=[\d.]+ executions=(\d+) timing_faults=(\d+)", text)
att = re.findall(r"attitude faults=(\d+) timing_faults=(\d+)", text)
est = re.findall(r"est_valid=(\d)", text)
imu = re.findall(r"total samples=(\d+) ovr=(\d+) spi=(\d+) partial=(\d+) clamp=(\d+) order=(\d+) empty=(\d+)", text)
print("rate executions/timing_faults first, last:", rate[0], rate[-1])
print("attitude faults/timing_faults last:", att[-1], "est_valid last:", est[-1])
print("imu counters first, last (samples ovr spi partial clamp order empty):", imu[0], imu[-1])
print("usb_reset_refused last:", re.findall(r"usb_reset_refused=(\d+)", text)[-1])
