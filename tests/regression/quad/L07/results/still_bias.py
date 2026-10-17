#!/usr/bin/env python3
"""Still gyro bias of a decoded bench IMU log (M3 P7). Bench evidence only; never run by CI. Standard library only.

  python3 tools/target/imu_log.py decode bench_still_30s_2026-10-16.bin --csv still.csv
  python3 still_bias.py still.csv

Prints the per-axis mean of the gyro (deg/s) over the whole CSV, its component along the mean specific force (the gravity
direction) and perpendicular to it, and the mean specific force. The log must be a still capture."""

import csv
import math
import sys


def main(path: str) -> None:
    n = 0
    g = [0.0] * 3
    a = [0.0] * 3
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for i, k in enumerate(("gx", "gy", "gz")):
                g[i] += float(row[k])
            for i, k in enumerate(("ax", "ay", "az")):
                a[i] += float(row[k])
            n += 1
    g = [v / n * 180.0 / math.pi for v in g]
    a = [v / n for v in a]
    norm = math.sqrt(sum(v * v for v in a))
    u = [v / norm for v in a]
    along = sum(x * y for x, y in zip(g, u))
    perp = [x - along * y for x, y in zip(g, u)]
    print(f"samples {n}")
    print("gyro mean (deg/s): x %.4f y %.4f z %.4f" % tuple(g))
    print("mean specific force (m/s^2): x %.4f y %.4f z %.4f, |a| %.4f" % (*a, norm))
    print(f"gyro mean along gravity {along:.4f} deg/s, perpendicular to gravity {math.sqrt(sum(v * v for v in perp)):.4f} deg/s")


if __name__ == "__main__":
    main(sys.argv[1])
