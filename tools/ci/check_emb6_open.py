#!/usr/bin/env python3
"""Invariant L-5a of the onboard flight logger (unit U4-4; serena memory quad-m4/architect-logger-u4-2026-10-20): only the
measurement build MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN may link with a log ring below invariant L-5, and no other build of the image
holds its code.

That build's code (its console trigger, its banner, its trigger state; fw/hal/target/src/flight_log.cpp) is named so that every
symbol of it contains emb6_open. The check lists the symbols of each given file with nm (defined and undefined, mangled) and looks
for a symbol whose name contains the marker.

  --forbid FILE   an image that is not the measurement build (the default flight image, the flight-log image): any such symbol is
                  a violation (L5A-SYMBOL)
  --require FILE  the measurement build, or the planted control: at least one such symbol, else the check would pass vacuously
                  (L5A-CONTROL)
  --delete-on-failure  remove each forbidden file that holds the symbol, so no such image stays behind

Exit status: 0 clean, 1 violation or failed control, 2 usage error, nm failure or nothing to check (L5A-ERROR).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

MARKER = "emb6_open"


def marked_symbols(nm: str, path: Path) -> list[str]:
    """The symbol lines of `path` whose name contains the marker; raises RuntimeError if nm fails."""
    proc = subprocess.run([nm, str(path)], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"{nm} {path} failed ({proc.returncode}): {proc.stderr.strip()}")
    return [line.strip() for line in proc.stdout.splitlines() if line.split() and MARKER in line.split()[-1]]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nm", default="nm", help="the nm of the toolchain that built the files")
    parser.add_argument("--forbid", action="append", default=[], type=Path, help="not the measurement build (no symbol allowed)")
    parser.add_argument("--require", action="append", default=[], type=Path, help="the measurement build or the planted control")
    parser.add_argument("--delete-on-failure", action="store_true", help="remove a forbidden file that holds the symbol")
    args = parser.parse_args(argv[1:])
    if not args.forbid and not args.require:
        print("L5A-ERROR: no file to check (give --forbid and/or --require)", file=sys.stderr)
        return 2
    failed = False
    try:
        for path in args.forbid:
            found = marked_symbols(args.nm, path)
            for line in found:
                print(f"L5A-SYMBOL: {path}: {line}: the measurement build's code is in an image that is not the measurement build")
            if found:
                failed = True
                if args.delete_on_failure:
                    path.unlink(missing_ok=True)
                    print(f"L5A-SYMBOL: {path} removed")
        for path in args.require:
            if not marked_symbols(args.nm, path):
                print(f"L5A-CONTROL: {path} has no symbol containing {MARKER}: the check would pass vacuously")
                failed = True
    except (RuntimeError, OSError) as e:
        print(f"L5A-ERROR: {e}", file=sys.stderr)
        return 2
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
