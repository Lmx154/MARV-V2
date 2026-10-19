#!/usr/bin/env python3
"""Bench only (onboard logger unit U2): the numbers of one pulled .mlog, and the gyro CSV for the bifilar-pendulum measurement.

  analyze.py FILE.mlog [--gyro-csv OUT.csv] [--gyro-rate-hz N]

Prints: sector and record counts, the IMU record rate (records / time between first and last), the largest gap between consecutive
IMU records and how many exceed 1.5 x the median spacing, the gap records and the ring's drop counters (the last Status record), the
per-axis gyro mean, standard deviation and peak-to-peak in rad/s (FRD body axes), and whether the file ends with an End record.
--gyro-csv writes t_s (seconds from the first IMU record), gyro_x, gyro_y, gyro_z in rad/s, FRD, one row per IMU record, or every
k-th row with --gyro-rate-hz (the nominal record rate / k, k = nominal // N). Standard library plus mlog.py.
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[6] / "tools" / "log"))
import mlog  # noqa: E402

GAP_FACTOR = 1.5  # labelled harness value: a spacing above 1.5 x the median is counted as a gap


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("file", type=Path)
    ap.add_argument("--gyro-csv", type=Path)
    ap.add_argument("--gyro-rate-hz", type=float)
    a = ap.parse_args()
    dec = mlog.decode(a.file.read_bytes())
    print(mlog.summary(dec))
    imu = mlog.records_of(dec, "imu")
    if len(imu) < 2:
        print("no IMU records")
        return 1
    t = [r["t_us"] for r in imu]
    dts = [b - a_ for a_, b in zip(t, t[1:])]
    med = statistics.median(dts)
    span_s = (t[-1] - t[0]) * 1e-6
    print(f"imu: {len(imu)} records over {span_s:.3f} s = {(len(imu) - 1) / span_s:.1f} Hz (nominal {dec.header['info']['odr_hz']:.1f} Hz)")
    print(f"imu spacing: median {med} us, min {min(dts)} us, max {max(dts)} us, > {GAP_FACTOR} x median: "
          f"{sum(1 for d in dts if d > GAP_FACTOR * med)}")
    print(f"gap records: {len(mlog.records_of(dec, 'gap'))}")
    st = mlog.records_of(dec, "status")
    if st:
        s = st[-1]
        print(f"last status: state {s['state']} drops imu/rate/manual {s['drops_imu']}/{s['drops_rate_tick']}/"
              f"{s['drops_manual_control']} pushed imu/rate {s['pushed_imu']}/{s['pushed_rate_tick']} "
              f"ring high water {s['ring_high_water_cells']} cells, write failures {s['write_failures']}")
    for axis in ("gyro_x", "gyro_y", "gyro_z"):
        v = [r[axis] for r in imu]
        print(f"{axis}: mean {statistics.fmean(v):+.5f} sd {statistics.pstdev(v):.5f} peak-to-peak {max(v) - min(v):.5f} rad/s")
    if a.gyro_csv:
        step = 1 if not a.gyro_rate_hz else max(1, int(dec.header["info"]["odr_hz"] // a.gyro_rate_hz))
        with a.gyro_csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s"])
            for r in imu[::step]:
                w.writerow([f"{(r['t_us'] - t[0]) * 1e-6:.6f}", f"{r['gyro_x']:.7f}", f"{r['gyro_y']:.7f}", f"{r['gyro_z']:.7f}"])
        print(f"gyro csv: {a.gyro_csv} ({len(imu[::step])} rows, every {step} record)")
    return 0 if dec.clean else 1


if __name__ == "__main__":
    sys.exit(main())
