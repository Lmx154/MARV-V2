#!/usr/bin/env python3
"""Bench only (onboard logger unit U2b): streaming analysis of a long pulled .mlog, for the 30-minute soak.

  soak_analyze.py FILE.mlog [--status-csv OUT.csv] [--resync]

Decodes the WHOLE file with tools/log/mlog.py's own decoder (strict by default: it stops at the first invalid sector and says where),
but replaces the record list by a counting sink, so a file of 20 million records is checked without holding them. Prints: sectors
valid of total and where it stopped, the record counts per kind, whether an End record closes the file, the IMU spacing (a
histogram of the integer spacings, the median, the number above 1.5 x the median and the largest), the IMU rate, the rate-tick
spacing, every gap record, the last Status record's counters (drops per kind, pushes, write failures, the state) and the largest ring
high water and backlog any Status record shows. --status-csv writes every Status record. Standard library plus mlog.py.
"""
from __future__ import annotations

import argparse
import collections
import csv
import mmap
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[6] / "tools" / "log"))
import mlog  # noqa: E402

GAP_FACTOR = 1.5  # labelled harness value: a spacing above 1.5 x the median is counted as a gap
LARGEST_SPACINGS_SHOWN = 10  # labelled harness value: how many of the largest IMU spacings are listed


class Sink:
    """Stands in for Decoded.records: counts what is appended and keeps only the rare kinds."""

    def __init__(self) -> None:
        self.counts: collections.Counter[str] = collections.Counter()
        self.imu_dt: collections.Counter[int] = collections.Counter()
        self.rate_dt: collections.Counter[int] = collections.Counter()
        self.first_imu: int | None = None
        self.last_imu: int | None = None
        self.last_rate: int | None = None
        self.kept: dict[str, list[dict]] = {"status": [], "gap": [], "end": []}
        self.imu_backwards = 0

    def append(self, row: dict) -> None:
        kind = row["kind"]
        self.counts[kind] += 1
        if kind == "imu":
            t = row["t_us"]
            if self.last_imu is None:
                self.first_imu = t
            else:
                dt = t - self.last_imu
                if dt <= 0:
                    self.imu_backwards += 1
                self.imu_dt[dt] += 1
            self.last_imu = t
        elif kind == "rate_tick":
            t = row["t_us"]
            if self.last_rate is not None:
                self.rate_dt[t - self.last_rate] += 1
            self.last_rate = t
        elif kind in self.kept:
            self.kept[kind].append(row)


@dataclass
class SinkDecoded(mlog.Decoded):
    records: Sink = field(default_factory=Sink)  # type: ignore[assignment]


def median_of(counter: collections.Counter[int]) -> float:
    half = sum(counter.values()) / 2
    seen = 0
    for v in sorted(counter):
        seen += counter[v]
        if seen >= half:
            return float(v)
    return 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("file", type=Path)
    ap.add_argument("--status-csv", type=Path)
    ap.add_argument("--resync", action="store_true")
    a = ap.parse_args()
    mlog.Decoded = SinkDecoded  # decode() builds its result by this name
    with a.file.open("rb") as fh, mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as data:
        dec = mlog.decode(data, resync=a.resync)  # type: ignore[arg-type]
    s: Sink = dec.records  # type: ignore[assignment]
    print(f"file: {a.file} ({a.file.stat().st_size} B)")
    print(f"sectors: {dec.sectors_ok} valid of {dec.sectors_total}; session: "
          + (f"{dec.session:#018x}" if dec.session is not None else "none"))
    print("records: " + ", ".join(f"{k} {v}" for k, v in sorted(s.counts.items())))
    print("end record: " + ("yes" if dec.ended else "NO (the log is cut short)"))
    if dec.ends_inside_record:
        print("the last sector ends inside a record (the partial record is not returned)")
    for p in dec.problems:
        print(f"PROBLEM {p}")
    if dec.stopped_at is not None:
        print(f"stopped at sector {dec.stopped_at}")
    n_imu = s.counts["imu"]
    if n_imu >= 2 and s.first_imu is not None and s.last_imu is not None:
        med = median_of(s.imu_dt)
        span_s = (s.last_imu - s.first_imu) * 1e-6
        big = {dt: c for dt, c in s.imu_dt.items() if dt > GAP_FACTOR * med}
        print(f"imu: {n_imu} records over {span_s:.3f} s = {(n_imu - 1) / span_s:.2f} Hz (nominal "
              f"{dec.header['info']['odr_hz']:.1f} Hz); backwards/zero spacings {s.imu_backwards}")
        print(f"imu spacing: median {med:.0f} us, min {min(s.imu_dt)} us, max {max(s.imu_dt)} us, > {GAP_FACTOR} x median: "
              f"{sum(big.values())} spacings, {sum(round(dt / med) - 1 for dt, c in big.items() for _ in range(c))} samples missing by time")
        print("imu spacing histogram (us:count) " + " ".join(f"{dt}:{c}" for dt, c in sorted(s.imu_dt.items())
                                                              if c >= 1 and dt <= GAP_FACTOR * med))
        print(f"imu largest spacings (us:count) " + " ".join(f"{dt}:{s.imu_dt[dt]}" for dt in sorted(s.imu_dt)[-LARGEST_SPACINGS_SHOWN:]))
    if s.rate_dt:
        print(f"rate-tick spacing: median {median_of(s.rate_dt):.0f} us, min {min(s.rate_dt)} us, max {max(s.rate_dt)} us")
    print(f"gap records: {len(s.kept['gap'])}")
    for g in s.kept["gap"]:
        print("  gap", g)
    st = s.kept["status"]
    print(f"status records: {len(st)}")
    if st:
        last = st[-1]
        print("last status: " + " ".join(f"{k}={v}" for k, v in last.items() if k not in ("kind", "sector")))
        first = st[0]
        keys = ("drops_imu", "drops_rate_tick", "drops_manual_control", "pushed_imu", "pushed_rate_tick", "pushed_manual_control",
                "write_failures", "close_failures", "chunks_written")
        print("counters last minus first status (this session; the first is written at the start of logging, so the drops the boot-time "
              "prepare caused before it are excluded): " + " ".join(f"{k}={last[k] - first[k]}" for k in keys))
        print(f"largest ring high water in any status: {max(r['ring_high_water_cells'] for r in st)} cells; "
              f"largest ring backlog in any status: {max(r['ring_backlog_cells'] for r in st)} cells")
    for e in s.kept["end"]:
        print("end:", {k: v for k, v in e.items() if k != "kind"})
    if a.status_csv and st:
        with a.status_csv.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(st[0]))
            w.writeheader()
            w.writerows(st)
        print(f"status csv: {a.status_csv} ({len(st)} rows)")
    return 0 if dec.clean else 1


if __name__ == "__main__":
    sys.exit(main())
