#!/usr/bin/env python3
"""Invariant L-8b of the disarmed arena (onboard logger unit U4-2; serena memory quad-m4/architect-logger-u4-2026-10-20): the
image that holds the arena lease never starts core 1.

The arena's holder withdraws the ring it published to the tick, and the store then reuses the memory; that is safe only because
the tick (the IMU interrupt) and the background loop share core 0 (fw/store/include/marv/store/arena.hpp,
fw/log/include/marv/log/arena_logger.hpp). Core 1 is started only through the Pico SDK's multicore_launch_core1 family
(pico/multicore.h: multicore_launch_core1, multicore_launch_core1_with_stack, multicore_launch_core1_raw), so the check lists the
symbols of each given file with nm (defined and undefined) and looks for a symbol whose name starts with multicore_launch_core1.

  --forbid FILE   an image with the lease: any such symbol is a violation (L8B-SYMBOL)
  --require FILE  the planted control (an image that launches core 1): at least one such symbol, else the check would pass
                  vacuously (L8B-CONTROL)
  --delete-on-failure  remove each forbidden file that holds the symbol, so no such image stays behind

Exit status: 0 clean, 1 violation or failed control, 2 usage error, nm failure or nothing to check (L8B-ERROR).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

MARKER = "multicore_launch_core1"


def marked_symbols(nm: str, path: Path) -> list[str]:
    """The symbol lines of `path` whose name starts with the marker; raises RuntimeError if nm fails."""
    proc = subprocess.run([nm, str(path)], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"{nm} {path} failed ({proc.returncode}): {proc.stderr.strip()}")
    return [line.strip() for line in proc.stdout.splitlines() if line.split() and line.split()[-1].startswith(MARKER)]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nm", default="nm", help="the nm of the toolchain that built the files")
    parser.add_argument("--forbid", action="append", default=[], type=Path, help="an image with the lease (no symbol allowed)")
    parser.add_argument("--require", action="append", default=[], type=Path, help="the planted control (the symbol required)")
    parser.add_argument("--delete-on-failure", action="store_true", help="remove a forbidden file that holds the symbol")
    args = parser.parse_args(argv[1:])
    if not args.forbid and not args.require:
        print("L8B-ERROR: no file to check (give --forbid and/or --require)", file=sys.stderr)
        return 2
    failed = False
    try:
        for path in args.forbid:
            found = marked_symbols(args.nm, path)
            for line in found:
                print(f"L8B-SYMBOL: {path}: {line}: core 1 is started in an image that holds the arena lease")
            if found:
                failed = True
                if args.delete_on_failure:
                    path.unlink(missing_ok=True)
                    print(f"L8B-SYMBOL: {path} removed")
        for path in args.require:
            if not marked_symbols(args.nm, path):
                print(f"L8B-CONTROL: {path} has no symbol starting with {MARKER}: the check would pass vacuously")
                failed = True
    except (RuntimeError, OSError) as e:
        print(f"L8B-ERROR: {e}", file=sys.stderr)
        return 2
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
