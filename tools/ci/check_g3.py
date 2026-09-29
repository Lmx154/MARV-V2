#!/usr/bin/env python3
"""Gate G3: no truth or harness symbol in any flight composition (core contracts 7.4, row G3).

Three checks, each reading the target manifest that CMake writes into every build tree (cmake/flight_targets.cmake;
build/<preset>/g3_manifest.json). A flight target is every STATIC library under fw/ except fw/hal/sim and fw/sil; the
manifest is derived from the directory tree, never from a list.

  symbols   nm -C over every flight archive: no defined or undefined symbol (demangled) may contain
            marv::sil::, marv_sil_, marv::hal_sim::, marv::truth:: or marv_truth_
  includes  every translation unit of a flight target in compile_commands.json: no include directory (-I, -isystem,
            -iquote, -idirafter) may lie inside fw/sil, fw/hal/sim, sim/ or tests/
  exports   nm -D --defined-only over every SIL shared library: only marv_sil_* symbols

Exit status: 0 clean, 1 violations, 2 vacuous run (nothing to check) or tool failure. Diagnostics start with
G3-SYMBOL, G3-INCLUDE or G3-EXPORT; a vacuous or failed run prints G3-VACUOUS or G3-ERROR.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

FORBIDDEN_SYMBOLS = ("marv::sil::", "marv_sil_", "marv::hal_sim::", "marv::truth::", "marv_truth_")
FORBIDDEN_INCLUDE_DIRS = ("fw/sil", "fw/hal/sim", "sim", "tests")
INCLUDE_OPTIONS = ("-isystem", "-iquote", "-idirafter", "-I")
SIL_EXPORT = re.compile(r"^marv_sil_[A-Za-z0-9_]+$")

_NM_SYMBOL = re.compile(r"^(?:[0-9A-Fa-f]+|\s*)\s([A-Za-z?-])\s(.+)$")
_NM_MEMBER = re.compile(r"^(.+):$")


class G3Error(Exception):
    """Tool failure (exit status 2)."""


class G3Vacuous(G3Error):
    """A check over nothing (exit status 2)."""


def parse_nm(text: str) -> list[tuple[str, str, str]]:
    """Parse `nm` output into (member, type, name). Archive member headers set the member; blank lines are skipped."""
    symbols = []
    member = ""
    for line in text.splitlines():
        if not line.strip():
            continue
        m = _NM_SYMBOL.match(line)
        if m:
            symbols.append((member, m.group(1), m.group(2).strip()))
            continue
        h = _NM_MEMBER.match(line)
        if h:
            member = h.group(1)
            continue
        raise G3Error(f"unparsable nm line: {line!r}")
    return symbols


def forbidden_symbol_pattern(name: str) -> str | None:
    for pattern in FORBIDDEN_SYMBOLS:
        if pattern in name:
            return pattern
    return None


def find_forbidden_symbols(symbols: list[tuple[str, str, str]]) -> list[tuple[str, str, str, str]]:
    """Return (member, type, name, matched pattern) for every symbol matching a forbidden pattern."""
    found = []
    for member, kind, name in symbols:
        pattern = forbidden_symbol_pattern(name)
        if pattern is not None:
            found.append((member, kind, name, pattern))
    return found


def non_sil_exports(symbols: list[tuple[str, str, str]]) -> list[str]:
    return [name for _, _, name in symbols if not SIL_EXPORT.match(name)]


def include_dirs(args: list[str], directory: str) -> list[str]:
    """Include directories named by -I, -isystem, -iquote and -idirafter (attached or separate), resolved."""
    found = []
    i = 0
    while i < len(args):
        arg = args[i]
        for opt in INCLUDE_OPTIONS:
            if arg == opt:
                if i + 1 < len(args):
                    found.append(args[i + 1])
                i += 1
                break
            if arg.startswith(opt):
                found.append(arg[len(opt):])
                break
        i += 1
    return [os.path.realpath(os.path.join(directory, d)) for d in found]


def forbidden_include_dir(path: str, root: str) -> str | None:
    """Return the forbidden directory (relative to root) that path is, or is inside; None if it is allowed."""
    real_root = os.path.realpath(root)
    for rel in FORBIDDEN_INCLUDE_DIRS:
        base = os.path.join(real_root, rel)
        if path == base or path.startswith(base + os.sep):
            return rel
    return None


def command_args(entry: dict) -> list[str]:
    if "arguments" in entry:
        return list(entry["arguments"])
    if "command" in entry:
        return shlex.split(entry["command"])
    raise G3Error(f"compile_commands entry for {entry.get('file')} has neither command nor arguments")


def entry_output(entry: dict) -> str:
    if "output" not in entry:
        raise G3Error(f"compile_commands entry for {entry.get('file')} has no output key (Ninja generator required)")
    return os.path.normpath(os.path.join(entry["directory"], entry["output"]))


def load_manifest(path: Path) -> dict:
    try:
        manifest = json.loads(Path(path).read_text())
    except (OSError, ValueError) as exc:
        raise G3Error(f"cannot read manifest {path}: {exc}") from exc
    for key in ("flight_targets", "sil_libraries"):
        if not isinstance(manifest.get(key), list):
            raise G3Error(f"manifest {path} has no '{key}' list")
    return manifest


def run_nm(nm: str, flags: list[str], artifact: str) -> list[tuple[str, str, str]]:
    if not os.path.isfile(artifact):
        raise G3Error(f"artifact does not exist (not built?): {artifact}")
    try:
        proc = subprocess.run([nm, *flags, artifact], capture_output=True, text=True, check=False)
    except OSError as exc:
        raise G3Error(f"cannot run {nm}: {exc}") from exc
    if proc.returncode != 0:
        raise G3Error(f"{nm} {' '.join(flags)} {artifact} failed ({proc.returncode}): {proc.stderr.strip()}")
    return parse_nm(proc.stdout)


def check_symbols(manifest: dict, nm: str) -> int:
    targets = manifest["flight_targets"]
    if not targets:
        raise G3Vacuous("the manifest lists zero flight targets: the symbol check would be vacuous")
    violations = 0
    scanned = 0
    for target in targets:
        symbols = run_nm(nm, ["-C"], target["archive"])
        scanned += len(symbols)
        for member, kind, name, pattern in find_forbidden_symbols(symbols):
            violations += 1
            print(f"G3-SYMBOL {target['name']} ({target['archive']}) member {member}: '{name}' "
                  f"[{kind}] contains '{pattern}'")
    if scanned == 0:
        raise G3Vacuous("the flight archives contain no symbols at all: the symbol check would be vacuous")
    print(f"G3 symbols: {len(targets)} flight targets, {scanned} symbols scanned, "
          f"{violations} violations ({', '.join(t['name'] for t in targets)})")
    return 1 if violations else 0


def check_includes(manifest: dict, compile_commands: Path, root: Path) -> int:
    targets = manifest["flight_targets"]
    if not targets:
        raise G3Vacuous("the manifest lists zero flight targets: the include check would be vacuous")
    try:
        database = json.loads(Path(compile_commands).read_text())
    except (OSError, ValueError) as exc:
        raise G3Error(f"cannot read {compile_commands}: {exc}") from exc
    object_owner = {}
    for target in targets:
        for obj in target["objects"]:
            object_owner[os.path.normpath(obj)] = target["name"]
    tu_count = {t["name"]: 0 for t in targets}
    violations = 0
    for entry in database:
        owner = object_owner.get(entry_output(entry))
        if owner is None:
            continue
        tu_count[owner] += 1
        for directory in include_dirs(command_args(entry), entry["directory"]):
            forbidden = forbidden_include_dir(directory, str(root))
            if forbidden is not None:
                violations += 1
                print(f"G3-INCLUDE {owner}: {entry['file']} has include directory {directory} "
                      f"inside {forbidden}/")
    empty = [name for name, count in tu_count.items() if count == 0]
    if empty:
        raise G3Vacuous(f"no compile_commands entry found for flight targets {empty}: the include check would be vacuous")
    total = sum(tu_count.values())
    print(f"G3 includes: {len(targets)} flight targets, {total} translation units, {violations} violations "
          f"({', '.join(f'{n}={c}' for n, c in tu_count.items())})")
    return 1 if violations else 0


def check_exports(manifest: dict, nm: str) -> int:
    libraries = manifest["sil_libraries"]
    if not libraries:
        raise G3Vacuous("the manifest lists zero SIL libraries: the export check would be vacuous")
    violations = 0
    for lib in libraries:
        symbols = run_nm(nm, ["-D", "--defined-only"], lib["file"])
        if not symbols:
            raise G3Vacuous(f"{lib['file']} exports no symbols: the export check would be vacuous")
        for name in non_sil_exports(symbols):
            violations += 1
            print(f"G3-EXPORT {lib['name']} ({lib['file']}): exports '{name}', which is not marv_sil_*")
        print(f"G3 exports: {lib['name']} exports {len(symbols)} symbols")
    print(f"G3 exports: {len(libraries)} SIL libraries, {violations} violations")
    return 1 if violations else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="check", required=True)
    for name in ("symbols", "includes", "exports"):
        p = sub.add_parser(name)
        p.add_argument("--manifest", required=True, type=Path, help="g3_manifest.json of the build tree")
        if name == "includes":
            p.add_argument("--compile-commands", type=Path,
                           help="default: compile_commands.json next to the manifest")
            p.add_argument("--repo-root", type=Path, default=ROOT)
        else:
            p.add_argument("--nm", default="nm", help="nm of the toolchain that built the artifacts")
    args = parser.parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        if args.check == "symbols":
            return check_symbols(manifest, args.nm)
        if args.check == "includes":
            cc = args.compile_commands or args.manifest.parent / "compile_commands.json"
            return check_includes(manifest, cc, args.repo_root)
        return check_exports(manifest, args.nm)
    except G3Error as exc:
        tag = "G3-VACUOUS" if isinstance(exc, G3Vacuous) else "G3-ERROR"
        print(f"{tag} {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
