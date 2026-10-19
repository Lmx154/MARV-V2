#!/usr/bin/env python3
"""Invariant L-3 of the onboard flight logger: the logger never calls hal_panic. A failure of the card is a state and a counter.

Scans every C++ source and header under fw/log (comments and string literals blanked, so a comment that explains the rule does
not trip it) for the identifier hal_panic and for the include of marv/hal/hal.hpp, the only way fw code reaches it.

Usage: check_log_no_panic.py [DIR ...]   (default: fw/log)
Output: log-no-panic: <file>:<line>: <what>. Exit status: 0 clean, 1 findings, 2 nothing scanned.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_constants import blank_comments  # noqa: E402

SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inc"}
PANIC = re.compile(r"\bhal_panic\b")
HAL_INCLUDE = re.compile(r"#\s*include\s*[<\"]marv/hal/hal\.hpp[>\"]")


def scan(text: str) -> list[tuple[int, str]]:
    code = blank_comments(text, keep_preprocessor=True)
    found = []
    for n, line in enumerate(code.splitlines(), start=1):
        if PANIC.search(line):
            found.append((n, "names hal_panic"))
    for n, line in enumerate(text.splitlines(), start=1):
        if HAL_INCLUDE.match(line.strip()):  # an include's path is a string literal, which blank_comments blanks
            found.append((n, "includes marv/hal/hal.hpp"))
    return found


def main(argv: list[str]) -> int:
    dirs = [Path(a) for a in argv[1:]] or [ROOT / "fw" / "log"]
    files = sorted(p for d in dirs for p in (d.rglob("*") if d.is_dir() else [d]) if p.suffix in SUFFIXES)
    if not files:
        print("log-no-panic: no source files found", file=sys.stderr)
        return 2
    bad = 0
    for f in files:
        for line, what in scan(f.read_text()):
            print(f"log-no-panic: {f}:{line}: {what}")
            bad += 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
