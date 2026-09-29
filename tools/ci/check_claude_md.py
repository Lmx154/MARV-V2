#!/usr/bin/env python3
"""Gate G8 (core spec §7.4): CLAUDE.md must keep the sections that give agents the rules.

Required: the ACTIVE-spec rule, the number rule, the UNKNOWN rule and the CI gates, each as a level-2 heading with a
non-empty body; the CI-gates section must name every gate G1..G8.

Exit 0 when clean, 1 on a violation (lines tagged G8-MISSING), 2 if the file cannot be read.
"""

import argparse
import re
import sys
from pathlib import Path

REQUIRED_SECTIONS = ("ACTIVE-spec rule", "Number rule", "UNKNOWN rule", "CI gates")
GATES = tuple(f"G{i}" for i in range(1, 9))


def sections(text: str) -> dict[str, str]:
    """Map each level-2 heading to the text up to the next level-2 heading."""
    out: dict[str, str] = {}
    current = None
    for line in text.splitlines():
        m = re.match(r"^## +(.+?)\s*$", line)
        if m:
            current = m.group(1)
            out[current] = ""
        elif current is not None:
            out[current] += line + "\n"
    return out


def violations(text: str) -> list[str]:
    found = sections(text)
    problems = []
    for name in REQUIRED_SECTIONS:
        if name not in found:
            problems.append(f"section '## {name}' is missing")
        elif not found[name].strip():
            problems.append(f"section '## {name}' is empty")
    gates_text = found.get("CI gates", "")
    for gate in GATES:
        if gates_text and not re.search(rf"\b{gate}\b", gates_text):
            problems.append(f"section '## CI gates' does not name {gate}")
    return problems


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?", default="CLAUDE.md")
    args = parser.parse_args(argv)
    try:
        text = Path(args.file).read_text(encoding="utf-8")
    except OSError as e:
        print(f"G8-ERROR: cannot read {args.file}: {e}")
        return 2
    problems = violations(text)
    for p in problems:
        print(f"G8-MISSING: {args.file}: {p}")
    if problems:
        return 1
    print(f"G8 clean: {args.file} has {len(REQUIRED_SECTIONS)} required sections and names {len(GATES)} gates")
    return 0


if __name__ == "__main__":
    sys.exit(main())
