#!/usr/bin/env python3
"""Ground-segment check GS-3: codec isolation (docs/spec/20-ground-segment.md, section 4).

CI fails if any tracked file outside the codec includes a MAVLink header, names MAVLink, or names an in-scope message.
Searched, case-insensitively: the word "mavlink" (this covers `#include <mavlink.h>`, `MAVLINK_` macros and `mavlink_`
functions); and, as whole words, case-sensitively: every message in link/scope.txt and every MARV_* name (message,
enum, enum entry) that link/marv.xml defines.

The allow-list below is the whole set of places that may do so; each entry carries its reason. With no arguments the
files are `git ls-files` and the allow-list applies. With file arguments (the planted control) exactly those files are
checked and the allow-list does not apply.

Exit status: 0 clean, 1 violations (lines tagged GS3-VIOLATION), 2 tool failure or nothing to check (GS3-ERROR).
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# (glob, reason). `*` and `?` do not cross `/`; `**/` matches any number of directories; a trailing `**` the rest.
ALLOW = (
    ("fw/link/src/**", "the C++ codec: the only code that sees the MAVLink headers"),
    ("fw/link/include/marv/link/codec.hpp", "the codec interface: a comment says no MAVLink header is above it; it includes none"),
    ("fw/link/include/marv/link/codec_mavlink.hpp", "the codec's own header (its name and class are the MAVLink codec)"),
    ("fw/link/CMakeLists.txt", "builds the codec: runs mavgen on the dialect and gives the generated headers to the codec alone"),
    ("link/**", "the MARV dialect XML, the pinned definitions and the scope list (inputs of the codecs)"),
    ("tools/link/**", "the generators and the definitions fetcher that read the dialect"),
    ("gcs/src/lib/link/**", "the TypeScript codec"),
    ("tests/regression/quad/L00/link/**", "the codec conformance suite and its vectors"),
    ("**/controls/**", "planted negative controls"),
    ("docs/**", "specs and decision records describe the codec"),
    ("tools/ci/check_gs3.py", "this checker names what it forbids"),
    ("tests/regression/quad/L00/tools/test_check_gs3.py", "the test of this checker"),
    ("tests/regression/quad/L00/tools/test_link_gen.py", "the drift guard of the generators against pymavlink's mavgen"),
    ("tests/regression/quad/L10/gz/test_link_sil.py", "the end-to-end test of the SIL link with pymavlink (ground segment unit 6)"),
    ("tests/regression/quad/L10/results/link_board/*.py", "the bench client of the board's link with pymavlink (ground segment unit 12b)"),
    ("tests/regression/quad/L10/results/link_board/README.md", "the bench record of the board's link names the messages it checked (ground segment unit 12b)"),
    ("tests/regression/quad/L10/results/slots/*.py", "the bench client of the board's config slots with pymavlink (ground segment unit 14)"),
    ("tests/regression/quad/L10/results/slots/README.md", "the bench record of the board's config slots names the messages it checked (ground segment unit 14)"),
    ("tools/gen/params_gen.py", "a docstring and a comment name MAVLink as the origin of the 16-character link id rule (decision 0053 item 1); no include, no message"),
    ("ci/run_ci.sh", "the step names of the GS-3 check"),
    ("ci/blast_radius.yaml", "the rules and reasons of the codec paths"),
    ("pyproject.toml", "pins the dev dependency pymavlink (build and test tool only)"),
    ("uv.lock", "locks pymavlink and its hashes"),
    ("THIRD_PARTY_NOTICES.md", "the MAVLink definitions and pymavlink notices"),
)

WORD_MAVLINK = re.compile(r"mavlink", re.IGNORECASE)


def glob_to_regex(glob: str) -> re.Pattern[str]:
    out = []
    i = 0
    while i < len(glob):
        if glob.startswith("**/", i):
            out.append("(?:.*/)?")
            i += 3
        elif glob.startswith("**", i):
            out.append(".*")
            i += 2
        elif glob[i] == "*":
            out.append("[^/]*")
            i += 1
        elif glob[i] == "?":
            out.append("[^/]")
            i += 1
        else:
            out.append(re.escape(glob[i]))
            i += 1
    return re.compile("".join(out) + r"\Z")


ALLOW_RE = tuple((glob_to_regex(g), g) for g, _ in ALLOW)


def allowed(path: str) -> bool:
    return any(rx.match(path) for rx, _ in ALLOW_RE)


def message_names(root: Path) -> list[str]:
    names = []
    for line in (root / "link" / "scope.txt").read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            names.append(line)
    tree = ET.parse(root / "link" / "marv.xml")
    for el in tree.iter():
        name = el.get("name", "")
        if name.startswith("MARV_"):
            names.append(name)
    return sorted(set(names))


def identifier_regex(names: list[str]) -> re.Pattern[str]:
    return re.compile(r"(?<![A-Za-z0-9_])(?:" + "|".join(re.escape(n) for n in sorted(names, key=len, reverse=True)) + r")(?![A-Za-z0-9_])")


def scan(text: str, identifiers: re.Pattern[str]) -> list[tuple[int, str]]:
    found = []
    for number, line in enumerate(text.splitlines(), 1):
        m = WORD_MAVLINK.search(line)
        if m:
            found.append((number, f"names MAVLink ('{m.group(0)}')"))
            continue
        m = identifiers.search(line)
        if m:
            found.append((number, f"names the in-scope identifier '{m.group(0)}'"))
    return found


def tracked_files(root: Path) -> list[str]:
    out = subprocess.run(["git", "-C", str(root), "ls-files", "-z"], check=True, capture_output=True).stdout
    return [p for p in out.decode("utf-8").split("\0") if p]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="*", help="check exactly these files, allow-list not applied (the planted control)")
    parser.add_argument("--root", default=str(ROOT))
    args = parser.parse_args(argv)
    root = Path(args.root)
    try:
        identifiers = identifier_regex(message_names(root))
        if args.files:
            targets = [(f, Path(f)) for f in args.files]
        else:
            targets = [(p, root / p) for p in tracked_files(root) if not allowed(p)]
    except (OSError, ET.ParseError, subprocess.CalledProcessError) as e:
        print(f"GS3-ERROR: {e}")
        return 2
    if not targets:
        print("GS3-ERROR: no file to check")
        return 2
    violations = 0
    checked = 0
    for label, path in targets:
        try:
            data = path.read_bytes()
        except OSError as e:
            print(f"GS3-ERROR: cannot read {label}: {e}")
            return 2
        if b"\0" in data:
            continue
        checked += 1
        for number, why in scan(data.decode("utf-8", errors="replace"), identifiers):
            print(f"GS3-VIOLATION: {label}:{number}: {why}")
            violations += 1
    if violations:
        return 1
    print(f"GS-3 clean: {checked} files checked outside the codec, {len(ALLOW)} allow-list entries")
    return 0


if __name__ == "__main__":
    sys.exit(main())
