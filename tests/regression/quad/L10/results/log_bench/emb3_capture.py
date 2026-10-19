#!/usr/bin/env python3
"""Bench only (onboard logger unit U2b): the EMB-3 capture of the bench log image built with MARV_FC3V2_BENCH_STATS.

  emb3_capture.py --out FILE.console.log [--idle-s 30] [--log-s 600] [--tail-s 30]

Reads the console for idle-s seconds (logger Ready, tick hooks pushing into the ring, nothing written), sends `log start`, reads for
log-s seconds (the logger drains the ring to the card), sends `log stop`, reads tail-s more (post-roll, close) and writes the raw
console. The stage statistics are then analysed by tools/target/bench_stats.py report. The marks (host seconds of each command) are
printed.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import log_bench  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--idle-s", type=float, default=30.0)
    ap.add_argument("--log-s", type=float, default=600.0)
    ap.add_argument("--tail-s", type=float, default=30.0)
    a = ap.parse_args()
    b = log_bench.Bench(log_bench.CONSOLE, log_bench.LINK, a.out)
    t0 = time.time()

    def read_for(s: float) -> None:
        end = time.time() + s
        while time.time() < end:
            b.pump(0.2)

    read_for(a.idle_s)
    print(f"log start at +{time.time() - t0:.1f} s")
    b.send("log start")
    read_for(a.log_s)
    print(f"log stop at +{time.time() - t0:.1f} s")
    b.send("log stop")
    read_for(a.tail_s)
    print(f"end at +{time.time() - t0:.1f} s")
    b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
