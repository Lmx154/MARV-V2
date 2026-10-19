#!/usr/bin/env python3
"""Invariant L-6 of the bench motor-step profile (unit U3 BENCH-PROFILE): no flight image contains the bench profile.

Lists the symbols of each given file with nm -C (an executable, a shared library or an archive; defined and undefined, local
and global) and looks for the marker `bench_profile` in the demangled names. Every symbol of the profile carries it: the
composition variant's state, functions and status (fw/compositions/quad_fc/src/quad_fc.cpp under MARV_QUAD_FC_BENCH_PROFILE,
fw/compositions/quad_fc/include/marv/bench_profile.hpp), and its parameter ids are bench_profile_* (the parameter table of a
flight image is held to marv_params_fc3v2 by the post-link table check, so no bench_profile_* record can reach it either).

  --forbid FILE   a flight file: any symbol with the marker is a violation (L6-SYMBOL)
  --require FILE  a bench file, the negative control: at least one symbol with the marker, else the check would pass
                  vacuously (L6-CONTROL)
  --delete-on-failure  remove each forbidden file that holds the marker, so no flight image with the profile stays behind

Exit status: 0 clean, 1 violation or failed control, 2 usage error, nm failure or nothing to check (L6-ERROR).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

MARKER = "bench_profile"


def marked_symbols(nm: str, path: Path) -> list[str]:
    """The demangled symbol lines of `path` that carry the marker; raises RuntimeError if nm fails."""
    proc = subprocess.run([nm, "-C", str(path)], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"{nm} -C {path} failed ({proc.returncode}): {proc.stderr.strip()}")
    return [line.strip() for line in proc.stdout.splitlines() if MARKER in line]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nm", default="nm", help="the nm of the toolchain that built the files")
    parser.add_argument("--forbid", action="append", default=[], type=Path, help="a flight file (no marker allowed)")
    parser.add_argument("--require", action="append", default=[], type=Path, help="a bench file (the marker required)")
    parser.add_argument("--delete-on-failure", action="store_true", help="remove a forbidden file that holds the marker")
    args = parser.parse_args(argv[1:])
    if not args.forbid and not args.require:
        print("L6-ERROR: no file to check (give --forbid and/or --require)", file=sys.stderr)
        return 2
    failed = False
    try:
        for path in args.forbid:
            found = marked_symbols(args.nm, path)
            for line in found:
                print(f"L6-SYMBOL: {path}: {line}")
            if found:
                failed = True
                if args.delete_on_failure:
                    path.unlink(missing_ok=True)
                    print(f"L6-SYMBOL: {path} removed")
        for path in args.require:
            if not marked_symbols(args.nm, path):
                print(f"L6-CONTROL: {path} has no symbol naming {MARKER}: the check would pass vacuously")
                failed = True
    except (RuntimeError, OSError) as e:
        print(f"L6-ERROR: {e}", file=sys.stderr)
        return 2
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
