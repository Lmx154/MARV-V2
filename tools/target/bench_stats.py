#!/usr/bin/env python3
"""FC3v2 bench statistics (quad spec 4.0 M1; M1 packet P12a). Bench only; never run by CI.

  python3 tools/target/bench_stats.py capture /dev/ttyACM1 660 run.log
      reads the USB console of an image built with MARV_FC3V2_BENCH_STATS for the given seconds, DTR asserted, and
      writes the bytes as they arrive (raw; a .gz name is gzip-compressed).
  uv run python tools/target/bench_stats.py report run.log [run2.log ...] --profile sensors/profiles/madflight_fc3v2_revc.yaml
      the stage times and the EMB-3 check from the logs, and the still-bench IMU statistics of the first log against
      the sensor profile.
"""

from __future__ import annotations

import argparse
import gzip
import math
import re
import sys
import time
from pathlib import Path


def open_text(path: Path, mode: str):
    if path.suffix == ".gz":
        return gzip.open(path, mode + "t", encoding="utf-8", errors="replace")
    return open(path, mode, encoding="utf-8", errors="replace")


def capture(dev: str, seconds: float, out: Path) -> None:
    import fcntl
    import os
    import select
    import struct
    import termios

    tiocmbis, tiocm_dtr = 0x5416, 0x002  # Linux ioctl TIOCMBIS and the DTR bit (asm-generic/ioctls.h, termios.h)
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = termios.B115200
    a[2] |= termios.CLOCAL | termios.CREAD
    a[3] &= ~(termios.ICANON | termios.ECHO)
    a[1] &= ~termios.OPOST
    a[0] &= ~termios.ICRNL
    termios.tcsetattr(fd, termios.TCSANOW, a)
    fcntl.ioctl(fd, tiocmbis, struct.pack("I", tiocm_dtr))
    t0 = time.time()
    with open_text(out, "w") as f:
        while time.time() - t0 < seconds:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                try:
                    f.write(os.read(fd, 65536).decode(errors="replace"))
                except BlockingIOError:
                    pass
    os.close(fd)


# --- parsing ---------------------------------------------------------------------------------------------------------

SUB_BITS = 2  # bench_stats.hpp kSubBits
SUB = 1 << SUB_BITS


def bucket_range(b: int) -> tuple[int, int]:
    """[low, high) cycles of histogram bucket b (bench_stats.hpp bucket_of)."""
    if b < 2 * SUB:
        return b, b + 1
    shift = b // SUB - 1
    top = b % SUB + SUB
    return top << shift, (top + 1) << shift


TIMING_RE = re.compile(r"(\w+) n=(\d+) max=(\d+) wmax=(\d+) h=([0-9:,]*)")
SPACING_RE = re.compile(r"(w?spacing) n=(\d+) sum=(\d+) min=(\d+) max=(\d+)")
AXES_RE = re.compile(r"(gs|gq|as|aq)=\((-?\d+),(-?\d+),(-?\d+)\)")


def parse_stages(line: str) -> dict:
    parts = line.split(" | ")
    out = {"t": int(re.search(r"t=(\d+)", parts[0]).group(1)), "timing": {}, "spacing": {}}
    for p in parts[1:]:
        m = TIMING_RE.fullmatch(p.strip())
        if m:
            hist = {}
            if m.group(5):
                for item in m.group(5).split(","):
                    b, n = item.split(":")
                    hist[int(b)] = int(n)
            out["timing"][m.group(1)] = {"n": int(m.group(2)), "max": int(m.group(3)), "wmax": int(m.group(4)),
                                         "hist": hist}
            continue
        for s in SPACING_RE.finditer(p):
            out["spacing"][s.group(1)] = dict(zip(("n", "sum", "min", "max"), map(int, s.groups()[1:])))
        m = re.search(r"max_batch=(\d+) usb_preemptions=(\d+)", p)
        if m:
            out["max_batch"], out["usb_preemptions"] = int(m.group(1)), int(m.group(2))
    return out


def parse_imu(line: str) -> dict:
    out = {"t": int(re.search(r"t=(\d+)", line).group(1)), "n": int(re.search(r" n=(\d+)", line).group(1))}
    for m in AXES_RE.finditer(line):
        out[m.group(1)] = [int(m.group(i)) for i in (2, 3, 4)]
    return out


def parse_log(path: Path) -> dict:
    stages, imus, stats, banner = [], [], [], []
    with open_text(path, "r") as f:
        for raw in f:
            if not raw.endswith("\n"):
                continue  # the line the capture's end cut
            line = raw.strip()
            try:
                if line.startswith("bench_stages "):
                    st = parse_stages(line)
                    if set(STAGES) <= set(st["timing"]) and "wspacing" in st["spacing"]:
                        stages.append(st)
                elif line.startswith("bench_imu "):
                    im = parse_imu(line)
                    if {"gs", "gq", "as", "aq"} <= set(im):
                        imus.append(im)
                elif line.startswith("imu dt="):
                    stats.append(line)
                elif line and not line.startswith(("bench_", "imu dt=")):
                    banner.append(line)
            except (AttributeError, ValueError):
                pass  # a line cut by the start or the end of the capture
    clk = None
    for b in banner:
        m = re.search(r"clk_sys (\d+) Hz", b)
        if m:
            clk = int(m.group(1))
    return {"stages": stages, "imu": imus, "stats": stats, "banner": banner, "clk": clk}


def percentile(hist: dict, q: float) -> int:
    """The upper edge of the bucket holding quantile q (an upper bound of the q-quantile)."""
    total = sum(hist.values())
    need = q * total
    acc = 0
    for b in sorted(hist):
        acc += hist[b]
        if acc >= need:
            return bucket_range(b)[1] - 1
    return 0


# --- report ----------------------------------------------------------------------------------------------------------

STAGES = ["read", "decode", "filter", "rate", "mixer", "gate", "dshot", "publish", "bench", "irq", "account", "usb_irq"]


def stage_report(name: str, log: dict, out) -> dict:
    clk = log["clk"]
    us = 1e6 / clk
    last = log["stages"][-1]
    first, final_t = log["stages"][0]["t"], last["t"]
    out.write(f"\n== {name}: {len(log['stages'])} lines, device time {first / 1e6:.1f} s to {final_t / 1e6:.1f} s "
              f"({(final_t - first) / 1e6:.1f} s of lines), clk_sys {clk} Hz\n")
    for b in log["banner"]:
        if b.startswith(("marv_fc3v2", "params ok:", "imu ok:", "bench stats:")):
            out.write(f"   {b}\n")
    out.write("   stage     count        max cyc   max us   max (after the first line) us   p50 us  p99 us  p99.99 us"
              "  mean-bucket us\n")
    result = {}
    for s in STAGES:
        t = last["timing"].get(s)
        if t is None:
            continue
        # the largest window maximum after the first line excludes the start-up entries (cold cache, enumeration)
        steady = max((ln["timing"][s]["wmax"] for ln in log["stages"][1:] if s in ln["timing"]), default=0)
        h = t["hist"]
        tot = sum(h.values())
        mean = sum(n * sum(bucket_range(b)) / 2 for b, n in h.items()) / tot if tot else 0.0
        result[s] = {"n": t["n"], "max": t["max"], "steady_max": steady, "mean": mean,
                     "p50": percentile(h, 0.5), "p99": percentile(h, 0.99), "p9999": percentile(h, 0.9999)}
        r = result[s]
        out.write(f"   {s:8s} {t['n']:10d} {t['max']:12d} {t['max'] * us:8.2f} {steady * us:30.2f} "
                  f"{r['p50'] * us:8.2f} {r['p99'] * us:7.2f} {r['p9999'] * us:9.2f} {mean * us:12.2f}\n")
    sp = last["spacing"]["spacing"]
    mean_sp = sp["sum"] / sp["n"]
    jitter_hi = max(ln["spacing"]["wspacing"]["max"] - ln["spacing"]["wspacing"]["sum"] / ln["spacing"]["wspacing"]["n"]
                    for ln in log["stages"][1:] if ln["spacing"].get("wspacing", {}).get("n"))
    jitter_lo = max(ln["spacing"]["wspacing"]["sum"] / ln["spacing"]["wspacing"]["n"] - ln["spacing"]["wspacing"]["min"]
                    for ln in log["stages"][1:] if ln["spacing"].get("wspacing", {}).get("n"))
    out.write(f"   IMU interrupt-entry spacing (one sample each): n {sp['n']}, mean {mean_sp:.1f} cyc = "
              f"{mean_sp * us:.3f} us, min {sp['min'] * us:.2f} us, max {sp['max'] * us:.2f} us; largest window "
              f"(max - mean) {jitter_hi * us:.2f} us, (mean - min) {jitter_lo * us:.2f} us\n")
    out.write(f"   max samples per interrupt {last.get('max_batch')}, IMU interrupts begun inside the USBCTRL chain "
              f"{last.get('usb_preemptions')}\n")
    out.write("   instrumentation inside the IMU interrupt: the bench stage (still sums) and the marks are inside irq; "
              "account follows irq; the default image has none of them\n")
    result["_spacing"] = {"mean": mean_sp, "jitter_hi": jitter_hi, "jitter_lo": jitter_lo}
    result["_clk"] = clk
    result["_last_stats"] = log["stats"][-1] if log["stats"] else ""
    if log["stats"]:
        out.write(f"   last counter line: {log['stats'][-1]}\n")
    return result


def emb3_report(results: dict, out) -> None:
    out.write("\n== EMB-3 (core 4.1: per core, utilization within n(2^(1/n) - 1) or exact response-time analysis, "
              "on measured worst cases)\n")
    for name, r in results.items():
        us = 1e6 / r["_clk"]
        period = r["_spacing"]["mean"]
        c_all = r["irq"]["max"] + r["account"]["max"]
        c_steady = r["irq"]["steady_max"] + r["account"]["steady_max"]
        b = r["_spacing"]["jitter_hi"]
        out.write(f"   {name}: T = {period * us:.3f} us (measured mean interrupt-entry spacing)\n")
        for label, c in (("including start-up", c_all), ("after the first line", c_steady)):
            u = c / period
            out.write(f"     C = irq + account max ({label}) = {c * us:.2f} us; U = C/T = {u:.3f} vs the n = 1 bound "
                      f"1.000: {'PASS' if u <= 1 else 'FAIL'} (margin {(1 - u) * 100:+.1f} % of T); "
                      f"RTA R = C + B = {(c + b) * us:.2f} us with B = {b * us:.2f} us vs D = T: "
                      f"{'PASS' if c + b <= period else 'FAIL'} (margin {(period - c - b) * us:+.2f} us)\n")
        mean_c = r["irq"]["mean"] + r["account"]["mean"]
        out.write(f"     mean (histogram midpoints, not a worst case): {mean_c * us:.2f} us per interrupt, "
                  f"{mean_c / period * 100:.1f} % of the core\n")


def still_report(log: dict, profile_path: Path, out) -> None:
    import yaml

    prof = yaml.safe_load(profile_path.read_text())["classes"]["imu"]["entries"]
    g_lsb = prof["gyro_sensitivity"]["value"] * 1e-3  # dps per LSB
    a_lsb = prof["accel_sensitivity"]["value"] * 1e-3  # g per LSB
    g_nd = prof["gyro_noise_density"]["value"] * 1e-3  # dps/sqrt(Hz)
    a_nd = prof["accel_noise_density"]["value"]
    if isinstance(a_nd, list):  # keyed by full-scale range, one position per ACCEL_RANGES_G; the configured range selects it
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "card"))
        from gen_imu_config import ACCEL_RANGES_G

        fsr = prof["accel_fsr"]["value"]
        if fsr not in ACCEL_RANGES_G:
            sys.exit(f"{profile_path}: accel_fsr {fsr!r} g matches no accel_noise_density position {list(ACCEL_RANGES_G)} g")
        a_nd = a_nd[ACCEL_RANGES_G.index(fsr)]
    a_nd = a_nd * 1e-6  # g/sqrt(Hz)
    g_off = prof["gyro_zero_rate_offset"]["value"]  # dps
    a_off = prof["accel_offset"]["value"] * 1e-3  # g
    odr = prof["odr_selected"]["value"]
    # Bandwidths (the driver's LPF configuration, boards/parts/lsm6dsv.hpp): gyroscope LPF1 off (CTRL7 LPF1_G_EN
    # default 0), LPF2 cut-off at 7.68 kHz 537 Hz (DS13476 Rev 5 6.7.2 Table 20); accelerometer LPF2_XL_EN = 0 (CTRL9
    # default), the LPF1 output, ODR/2 in high-performance mode (9.21 Table 68 footnote 1).
    g_bw = 537.0
    a_bw = odr / 2
    imus = log["imu"]
    first, last = imus[0], imus[-1]
    n = last["n"] - first["n"]
    out.write(f"\n== Still bench IMU ({n} samples between the first and the last bench_imu line, "
              f"{(last['t'] - first['t']) / 1e6:.1f} s; FRD, after the board mounting map)\n")
    out.write(f"   profile {profile_path}: gyro {g_nd * 1e3:.1f} mdps/rtHz x sqrt({g_bw:.0f} Hz) = "
              f"{g_nd * math.sqrt(g_bw) * 1e3:.1f} mdps; accel {a_nd * 1e6:.0f} ug/rtHz x sqrt({a_bw:.0f} Hz) = "
              f"{a_nd * math.sqrt(a_bw) * 1e3:.2f} mg; offsets +-{g_off} dps, +-{a_off * 1e3:.0f} mg; "
              f"LSB {g_lsb * 1e3:.0f} mdps, {a_lsb * 1e3:.3f} mg (quantisation LSB/sqrt(12) = "
              f"{g_lsb / math.sqrt(12) * 1e3:.1f} mdps, {a_lsb / math.sqrt(12) * 1e3:.3f} mg)\n")
    # DS13476 publishes the LPF2 cut-off, not its order: a first-order filter's noise-equivalent bandwidth is pi/2 times
    # its -3 dB cut-off, so the gyro figure above is a lower edge.
    g_enbw = math.pi / 2 * g_bw
    out.write(f"   if the gyro LPF2 is first order: noise bandwidth pi/2 x {g_bw:.0f} Hz = {g_enbw:.0f} Hz, sd "
              f"{g_nd * math.sqrt(g_enbw) * 1e3:.1f} mdps; with the quantisation "
              f"{math.hypot(g_nd * math.sqrt(g_enbw), g_lsb / math.sqrt(12)) * 1e3:.1f} mdps\n")
    stats = {}
    for key, lsb, unit, scale in (("g", g_lsb, "mdps", 1e3), ("a", a_lsb, "mg", 1e3)):
        s = [last[key + "s"][i] - first[key + "s"][i] for i in range(3)]
        q = [last[key + "q"][i] - first[key + "q"][i] for i in range(3)]
        for i, ax in enumerate("xyz"):
            mean = s[i] / n
            var = (q[i] * n - s[i] * s[i]) / (n * n)
            # within-window variance: the pooled variance inside each line-to-line window (drift removed)
            num = den = 0.0
            for a0, a1 in zip(imus, imus[1:]):
                m = a1["n"] - a0["n"]
                if m < 2:
                    continue
                ss = a1[key + "s"][i] - a0[key + "s"][i]
                qq = a1[key + "q"][i] - a0[key + "q"][i]
                num += qq - ss * ss / m
                den += m - 1
            stats[(key, ax)] = (mean * lsb, math.sqrt(max(var, 0.0)) * lsb, math.sqrt(num / den) * lsb if den else 0)
    for key, name, unit, scale, nd, bw, off in (("g", "gyro ", "mdps", 1e3, g_nd, g_bw, g_off),
                                                 ("a", "accel", "mg", 1e3, a_nd, a_bw, a_off)):
        sig = nd * math.sqrt(bw)
        for ax in "xyz":
            mean, sd, sd_w = stats[(key, ax)]
            ref = -1.0 if (key == "a" and ax == "z") else 0.0  # at rest, flat and level, FRD z reads -1 g
            off_meas = mean - ref
            out.write(f"   {name} {ax}: mean {mean * scale:+9.2f} {unit} (offset {off_meas * scale:+8.2f} vs +-"
                      f"{off * scale:.0f}: {'inside' if abs(off_meas) <= off else 'OUTSIDE'}), sd {sd * scale:7.2f} "
                      f"{unit} (within 1 s windows {sd_w * scale:7.2f}) vs profile {sig * scale:.2f}: ratio "
                      f"{sd_w / sig:.2f}\n")
    temps = []
    for line in log["stats"]:
        m = re.search(r"temp=([0-9.]+)K", line)
        if m:
            temps.append(float(m.group(1)))
    if temps:
        out.write(f"   temperature (newest sample per line): {min(temps) - 273.15:.2f} to {max(temps) - 273.15:.2f} "
                  f"degC over the run\n")
    # drift: per-window means at the start and the end of the run
    k = max(1, len(imus) // 10)
    for key, name, lsb, unit in (("g", "gyro ", g_lsb * 1e3, "mdps"), ("a", "accel", a_lsb * 1e3, "mg")):
        def window_mean(a0, a1, i):
            return (a1[key + "s"][i] - a0[key + "s"][i]) / (a1["n"] - a0["n"]) * lsb
        head = [window_mean(imus[0], imus[k], i) for i in range(3)]
        tail = [window_mean(imus[-1 - k], imus[-1], i) for i in range(3)]
        out.write(f"   {name} drift (mean of the last tenth - first tenth): "
                  f"({tail[0] - head[0]:+.2f}, {tail[1] - head[1]:+.2f}, {tail[2] - head[2]:+.2f}) {unit}\n")


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("dev")
    c.add_argument("seconds", type=float)
    c.add_argument("out", type=Path)
    r = sub.add_parser("report")
    r.add_argument("logs", type=Path, nargs="+")
    r.add_argument("--profile", type=Path, required=True)
    a = p.parse_args(argv)
    if a.cmd == "capture":
        capture(a.dev, a.seconds, a.out)
        return 0
    out = sys.stdout
    results = {}
    logs = {}
    for path in a.logs:
        logs[path.name] = parse_log(path)
        results[path.name] = stage_report(path.name, logs[path.name], out)
    emb3_report(results, out)
    still_report(logs[a.logs[0].name], a.profile, out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
