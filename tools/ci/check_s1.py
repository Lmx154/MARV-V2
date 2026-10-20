#!/usr/bin/env python3
"""Ground-segment check S-1: the vehicle's view and the simulator's channel never meet (docs/decisions/0056, GX-8).

Simulator truth reaches the GCS on a channel of its own and only feeds panels labelled SIM (gcs/src/lib/sim, gcs/src/routes/sim).
This check reads every import of every file under gcs/src and fails when

  - a SIM file (gcs/src/lib/sim/**, gcs/src/routes/sim/**) imports anything of the app except gcs/src/lib/sim/**, the plain UI
    components gcs/src/lib/components/ui/** and the helper's address reader gcs/src/lib/state/helper_api.ts (so never the vehicle client,
    the link codec, the vehicle panels or the app state that holds them); or
  - a file outside the SIM files imports gcs/src/lib/sim/** or gcs/src/routes/sim/**, except the layout (gcs/src/routes/+layout.svelte),
    which asks only whether a simulation is publishing, to list the SIM page.

Imports are found with a regular expression over the text (static `from '...'`, `import '...'`, dynamic `import('...')`), and resolved
with the app's aliases ($lib, $link) and relative paths. Exit status: 0 clean, 1 violations (lines tagged S1-VIOLATION), 2 tool failure
or nothing to check (S1-ERROR). `--root DIR` checks another tree laid out like gcs/src (the planted control of
tests/tools/test_check_s1.py).
"""

from __future__ import annotations

import argparse
import posixpath
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "gcs" / "src"
SUFFIXES = (".ts", ".svelte", ".js")
SIM_PREFIXES = ("lib/sim/", "routes/sim/")
SIM_MAY_IMPORT = ("lib/sim/", "lib/components/ui/", "lib/state/helper_api")
MAY_IMPORT_SIM = ("routes/+layout.svelte",)
IMPORT = re.compile(r"""(?:\bfrom\s*|\bimport\s*\(?\s*)(['"])([^'"\n]+)\1""")


def resolve(spec: str, importer: str) -> str | None:
    """The path under src of an import specifier, or None for a package or a framework module."""
    if spec.startswith("$lib/"):
        return "lib/" + spec[len("$lib/"):]
    if spec == "$lib":
        return "lib"
    if spec.startswith("$link/"):
        return "lib/link/" + spec[len("$link/"):]
    if spec.startswith("."):
        return posixpath.normpath(posixpath.join(posixpath.dirname(importer), spec))
    return None


def under(path: str, prefixes: tuple[str, ...]) -> bool:
    return any(path.startswith(p) or path + "/" == p for p in prefixes)


def check(src: Path) -> tuple[list[str], int]:
    violations, files = [], 0
    for path in sorted(src.rglob("*")):
        if path.suffix not in SUFFIXES or not path.is_file():
            continue
        files += 1
        rel = path.relative_to(src).as_posix()
        is_sim = under(rel, SIM_PREFIXES)
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for match in IMPORT.finditer(line):
                spec = match.group(2)
                target = resolve(spec, rel)
                if target is None:
                    continue
                if is_sim and not under(target, SIM_MAY_IMPORT):
                    violations.append(f"S1-VIOLATION: gcs/src/{rel}:{number}: a SIM file imports {spec!r} ({target}): SIM panels read "
                                      "only the sim channel (never the vehicle client, the link, the vehicle panels or the app)")
                if not is_sim and under(target, SIM_PREFIXES) and rel not in MAY_IMPORT_SIM:
                    violations.append(f"S1-VIOLATION: gcs/src/{rel}:{number}: imports {spec!r} ({target}): the vehicle's side never "
                                      "reads the sim channel")
    return violations, files


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=SRC, help="the tree to check, laid out like gcs/src (default gcs/src)")
    a = ap.parse_args(argv)
    if not a.root.is_dir():
        print(f"S1-ERROR: {a.root} is not a directory", file=sys.stderr)
        return 2
    violations, files = check(a.root)
    if files == 0:
        print(f"S1-ERROR: no source files under {a.root}", file=sys.stderr)
        return 2
    for v in violations:
        print(v)
    if violations:
        return 1
    print(f"S-1 clean: {files} files checked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
