#!/usr/bin/env python3
# Stick rest offsets from a FC3v2 console capture (tools/target/readcon.py, one status line per second).
#   xz -dc console.log.xz > console.log && python3 analyze.py console.log
# A sample is "at rest" when |stick| < 0.15; prints the range of each axis at rest. Bench evidence, run by no test.
import re
import sys

rows = []
for line in open(sys.argv[1], errors="replace"):
    if not line.startswith("imu dt"):
        continue
    try:
        rows.append([float(re.search(k + r"=([-+0-9.]+) ", line).group(1)) for k in ("roll", "pitch", "throttle", "yaw")])
    except AttributeError:
        pass
print("samples", len(rows))
for i, axis in enumerate(("roll", "pitch", "throttle", "yaw")):
    rest = [r[i] for r in rows if abs(r[i]) < 0.15]
    print(f"{axis:8s} rest n={len(rest)} min={min(rest):+.3f} max={max(rest):+.3f} max|x|={max(abs(x) for x in rest):.3f}")
