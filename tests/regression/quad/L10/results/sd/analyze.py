#!/usr/bin/env python3
"""Bench only (ground-segment unit 13): the numbers of an SD bench log (image with MARV_FC3V2_SD_BENCH and
MARV_FC3V2_BENCH_STATS), printed as text.

  analyze.py <sd_bench.log>

Reads the `sd_bench:` lines (START and DONE carry hal_time_us), the per-second `bench_stages` lines (cumulative
interrupt-entry spacing and the window spacing since the previous line, in clk_sys cycles; `motors ok: ... clk_sys N Hz`
gives the clock) and the `imu` lines. The window maximum of each `bench_stages` line is attributed to the phase its t falls
in: idle before START, traffic from START to DONE (the first line after DONE still holds the last cycles' window), idle
after DONE.
"""
import re
import sys

text = open(sys.argv[1], errors="replace").read()
clk = int(re.search(r"clk_sys (\d+) Hz", text).group(1))
start = int(re.search(r"sd_bench: START t=(\d+)", text).group(1))
done_m = re.search(r"sd_bench: DONE t=(\d+)", text)
done = int(done_m.group(1)) if done_m else None
print(f"clk_sys {clk} Hz; START t={start} us; DONE t={done} us")

phases = {"idle_before": [], "traffic": [], "idle_after": []}
pat = re.compile(r"bench_stages t=(\d+) .*?\| wspacing n=(\d+) sum=(\d+) min=(\d+) max=(\d+)")
for m in pat.finditer(text):
    t, n, _, mn, mx = (int(g) for g in m.groups())
    if t <= start:
        key = "idle_before"
    elif done is None or t <= done:
        key = "traffic"
    else:
        key = "idle_after"
    phases[key].append((t, n, mn, mx))
for key, rows in phases.items():
    if not rows:
        print(f"{key}: no windows")
        continue
    worst = max(r[3] for r in rows)
    print(f"{key}: {len(rows)} windows, window-max spacing worst {worst} cycles = {worst * 1e6 / clk:.1f} us; "
          f"min {min(r[2] for r in rows if r[2]) } cycles = {min(r[2] for r in rows if r[2]) * 1e6 / clk:.1f} us")
cum = re.findall(r"bench_stages t=(\d+) .*?\| spacing n=(\d+) sum=(\d+) min=(\d+) max=(\d+)", text)
if cum:
    t, n, s, mn, mx = (int(x) for x in cum[-1])
    print(f"cumulative at t={t}: n={n} mean {s / n * 1e6 / clk:.2f} us, min {mn * 1e6 / clk:.1f} us, max {mx} cycles = {mx * 1e6 / clk:.1f} us")
imu = re.findall(r"total samples=(\d+) ovr=(\d+) spi=(\d+) partial=(\d+) clamp=(\d+) order=(\d+) empty=(\d+)", text)
if imu:
    print("imu counters (first, last): samples/ovr/spi/partial/clamp/order/empty", imu[0], imu[-1])
tf = re.findall(r"rate executions/s=[\d.]+ executions=(\d+) timing_faults=(\d+)", text)
if tf:
    print("rate executions/timing_faults (first, last):", tf[0], tf[-1])
for line in re.findall(r"sd_bench: .*", text):
    print(line)
