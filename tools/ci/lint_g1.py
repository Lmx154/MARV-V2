#!/usr/bin/env python3
"""Gate G1: no unexplained numeric literals under fw/ (core contracts 7.4).

Allowed literals are 0, 1, 2 and 0.5 in any spelling. The one exempt file is
fw/prim/include/marv/prim/constants.hpp. Three checks run:

  tidy    clang-tidy readability-magic-numbers / cppcoreguidelines-avoid-magic-numbers (config in .clang-tidy)
  scan    token scan of every source under fw/; closes the clang-tidy hole where literals initialising
          const/constexpr variables are not flagged
  nolint  fails if a NOLINT naming either magic-number check (or a blanket NOLINT) appears outside the exempt file

With no file arguments it lints every fw/ translation unit in the host-debug compile database and every C++
source under fw/. With file arguments it lints exactly those files (used by the negative controls).
--scope sim-plant applies the same three checks to sim/plant (the truth physics) instead of fw/: its translation units
in the compile database, every C/C++ source and header under sim/plant, and clang-tidy diagnostics in sim/plant headers.
The default scope, fw, is unchanged. Constants still come only from the exempt fw/prim constants.hpp, which is not
scanned in either scope.

Exit status: 0 clean, 1 violations, 2 vacuous run or tool failure.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXEMPT = ROOT / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp"
ALLOWED = (0, 1, 2, 0.5)
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inc", ".ipp", ".tpp"}
TU_SUFFIXES = {".c", ".cc", ".cpp", ".cxx"}
MAGIC_CHECKS = ("readability-magic-numbers", "cppcoreguidelines-avoid-magic-numbers")
TIDY_DIAGNOSTIC = re.compile(r"\[(?:readability-magic-numbers|cppcoreguidelines-avoid-magic-numbers)[],]")

_IDENT = re.compile(r"[A-Za-z_]\w*")
_PP_NUMBER = re.compile(r"\.?[0-9](?:[eEpP][+-]|'[0-9A-Za-z_]|[0-9A-Za-z_.])*")
_STRING_PREFIXES = {"u8", "u", "U", "L"}
_FLOAT_SUFFIX = re.compile(r"(?:f16|f32|f64|f128|bf16|[fFlL])$")
_UDL_SUFFIX = re.compile(r"_\w*$")
_INT_SUFFIX = re.compile(r"[uUlLzZ]*$")
_NOLINT = re.compile(r"NOLINT(?:NEXTLINE|BEGIN|END)?\b(\([^)]*\))?")


def _skip_quoted(text: str, i: int, quote: str) -> int:
    n = len(text)
    i += 1
    while i < n:
        c = text[i]
        if c == "\\":
            i += 2
        elif c == quote or c == "\n":
            return i + 1
        else:
            i += 1
    return n


def _skip_raw_string(text: str, i: int) -> int:
    open_paren = text.find("(", i + 1)
    if open_paren < 0:
        return len(text)
    delim = text[i + 1 : open_paren]
    close = text.find(")" + delim + '"', open_paren)
    return len(text) if close < 0 else close + len(delim) + 2


def numeric_literals(text: str) -> list[tuple[int, str]]:
    """Return (line, spelling) for every numeric literal outside comments, strings and char literals."""
    found: list[tuple[int, str]] = []
    n = len(text)
    i = 0
    line = 1
    at_line_start = True
    while i < n:
        c = text[i]
        if c == "\n":
            line += 1
            at_line_start = True
            i += 1
            continue
        if c in " \t\r\f\v":
            i += 1
            continue
        if text.startswith("//", i):
            while i < n and text[i] != "\n":
                if text[i] == "\\" and i + 1 < n and text[i + 1] == "\n":
                    line += 1
                    i += 1
                elif text[i] == "\\" and text.startswith("\r\n", i + 1):
                    line += 1
                    i += 2
                i += 1
            continue
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            line += text.count("\n", i, end)
            i = end
            at_line_start = False
            continue
        if c == "#" and at_line_start:
            m = re.compile(r"#[ \t]*include\b[^\n]*").match(text, i)
            if m:
                i = m.end()
                continue
        at_line_start = False
        if c == '"' or c == "'":
            end = _skip_quoted(text, i, c)
            line += text.count("\n", i, end)
            i = end
            continue
        m = _IDENT.match(text, i)
        if m:
            word = m.group()
            i = m.end()
            if i < n and text[i] == '"' and word.endswith("R") and word[:-1] in _STRING_PREFIXES | {""}:
                end = _skip_raw_string(text, i)
                line += text.count("\n", i, end)
                i = end
            continue
        if c.isdigit() or (c == "." and i + 1 < n and text[i + 1].isdigit()):
            m = _PP_NUMBER.match(text, i)
            found.append((line, m.group()))
            i = m.end()
            continue
        i += 1
    return found


def literal_value(spelling: str) -> float | int | None:
    """Numeric value of a C++ literal spelling, or None if it cannot be parsed (treated as disallowed)."""
    s = spelling.replace("'", "").lower()
    s = _UDL_SUFFIX.sub("", s)
    try:
        if s.startswith("0x"):
            if "p" in s:
                return float.fromhex(_FLOAT_SUFFIX.sub("", s))
            return int(_INT_SUFFIX.sub("", s), 16)
        if s.startswith("0b"):
            return int(_INT_SUFFIX.sub("", s), 2)
        if "." in s or "e" in s:
            return float(_FLOAT_SUFFIX.sub("", s))
        digits = _INT_SUFFIX.sub("", s)
        if len(digits) > 1 and digits.startswith("0"):
            return int(digits, 8)
        return int(digits, 10)
    except ValueError:
        return None


def is_allowed(spelling: str) -> bool:
    value = literal_value(spelling)
    return value is not None and value in ALLOWED


def scan_text(text: str) -> list[tuple[int, str]]:
    return [(line, lit) for line, lit in numeric_literals(text) if not is_allowed(lit)]


def nolint_violations(text: str) -> list[tuple[int, str]]:
    """NOLINT markers that would suppress a magic-number check: naming either check, a wildcard, or blanket."""
    found = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        for m in _NOLINT.finditer(raw):
            checks = m.group(1)
            if checks is None or "*" in checks or any(name in checks for name in MAGIC_CHECKS):
                found.append((lineno, m.group()))
    return found


def is_exempt(path: Path) -> bool:
    return path.resolve() == EXEMPT


def check_file(path: Path) -> tuple[list[str], list[str]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    shown = _display(path)
    scan = [f"{shown}:{ln}: numeric literal '{lit}' is not one of 0, 1, 2, 0.5" for ln, lit in scan_text(text)]
    nolint = [f"{shown}:{ln}: '{tok}' suppresses a magic-number check outside the exempt file"
              for ln, tok in nolint_violations(text)]
    return scan, nolint


def _display(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def fw_sources() -> list[Path]:
    return sorted(p for p in (ROOT / "fw").rglob("*") if p.is_file() and p.suffix in SOURCE_SUFFIXES)


def plant_sources() -> list[Path]:
    return sorted(p for p in (ROOT / "sim" / "plant").rglob("*") if p.is_file() and p.suffix in SOURCE_SUFFIXES)


SCOPES = {"fw": (ROOT / "fw", fw_sources, None), "sim-plant": (ROOT / "sim" / "plant", plant_sources, r"(^|/)sim/plant/")}


def compile_db_units(build_dir: Path, scope_dir: Path = ROOT / "fw") -> list[dict]:
    db = build_dir / "compile_commands.json"
    if not db.is_file():
        return []
    fw_dir = scope_dir
    units = {}
    for entry in json.loads(db.read_text()):
        path = (Path(entry["directory"]) / entry["file"]).resolve()
        if path.is_relative_to(fw_dir):
            units[path] = entry
    return [units[p] for p in sorted(units)]


def run_tidy(tidy: str, files: list[Path], build_dir: Path | None, header_filter: str | None = None) -> tuple[int, str]:
    """Run clang-tidy on each file; return (failed file count, combined output)."""
    failed = 0
    output = []
    for path in files:
        cmd = [tidy, "--quiet", f"--config-file={ROOT / '.clang-tidy'}", "--extra-arg=-Wno-unknown-warning-option"]
        if header_filter is not None:
            cmd.append(f"--header-filter={header_filter}")
        if build_dir is not None:
            cmd += ["-p", str(build_dir), str(path)]
        else:
            cmd += [str(path), "--", "-std=c++20", "-x", "c++", f"-I{ROOT / 'fw' / 'prim' / 'include'}"]
        proc = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
        output.append(proc.stdout + proc.stderr)
        if proc.returncode != 0:
            failed += 1
    return failed, "".join(output)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("files", nargs="*", type=Path, help="lint exactly these files (negative controls)")
    ap.add_argument("--build-dir", type=Path, default=ROOT / "build" / "host-debug")
    ap.add_argument("--clang-tidy", default="clang-tidy-18")
    ap.add_argument("--scope", choices=sorted(SCOPES), default="fw", help="tree to lint when no files are given")
    args = ap.parse_args(argv)
    scope_dir, scope_sources, header_filter = SCOPES[args.scope]

    if args.files:
        files = [p.resolve() for p in args.files]
        tidy_files = [p for p in files if p.suffix in TU_SUFFIXES]
        tidy_db = None
        tu_count = len(tidy_files)
    else:
        files = scope_sources()
        units = compile_db_units(args.build_dir, scope_dir)
        tu_count = len(units)
        if tu_count == 0:
            print(f"G1-VACUOUS: no {_display(scope_dir)}/ translation units in {args.build_dir}/compile_commands.json")
            return 2
        tidy_files = [(Path(u["directory"]) / u["file"]).resolve() for u in units]
        tidy_db = args.build_dir

    if not files:
        print("G1-VACUOUS: no files to check")
        return 2
    missing = [p for p in files if not p.is_file()]
    if missing:
        print(f"G1-ERROR: not a file: {missing[0]}")
        return 2

    print(f"G1: {tu_count} translation unit(s) for clang-tidy, {len(files)} file(s) for token scan and NOLINT check")

    failed = False

    tidy_failed, tidy_out = run_tidy(args.clang_tidy, tidy_files, tidy_db, header_filter)
    sys.stdout.write(tidy_out)
    if tidy_failed:
        failed = True
        diagnostics = len(TIDY_DIAGNOSTIC.findall(tidy_out))
        if diagnostics == 0:
            print("G1-ERROR: clang-tidy failed without a magic-number diagnostic (compile or tool error)")
            return 2
        print(f"G1-TIDY: {diagnostics} magic-number diagnostic(s) in {tidy_failed} translation unit(s)")

    for path in files:
        if is_exempt(path):
            continue
        scan, nolint = check_file(path)
        for msg in scan:
            print(f"G1-SCAN: {msg}")
        for msg in nolint:
            print(f"G1-NOLINT: {msg}")
        failed = failed or bool(scan) or bool(nolint)

    print("G1 FAILED" if failed else "G1 clean")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
