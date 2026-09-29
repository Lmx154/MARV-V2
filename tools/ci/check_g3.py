#!/usr/bin/env python3
"""Gate G3: no truth or harness symbol in any flight composition (core contracts 7.4, row G3).

Three checks, each reading the target manifest that CMake writes into every build tree (cmake/flight_targets.cmake;
build/<preset>/g3_manifest.json). A flight target is every target of any type defined under fw/ except under
fw/hal/sim and fw/sil; the manifest is derived from the directory tree, never from a list. Each entry carries its CMake
TYPE. Known types: STATIC_LIBRARY, SHARED_LIBRARY, MODULE_LIBRARY and EXECUTABLE (linked file), OBJECT_LIBRARY (object
files) and INTERFACE_LIBRARY (usage requirements only). Any other type is a G3-ERROR, so a new kind of target can
never be skipped silently.

  symbols   nm -C over the linked file of every flight target (over the object files of an OBJECT library): no defined
            or undefined symbol (demangled) may contain marv::sil::, marv_sil_, marv::hal_sim::, marv::truth::,
            marv_truth_, marv::plant:: or marv_plant_
  includes  every translation unit of a flight target in compile_commands.json: no include directory (-I, -isystem,
            -iquote, -idirafter; a leading '=' of the sysroot-relative spelling is stripped) and no forced include
            (-include, -imacros) may lie inside fw/sil, fw/hal/sim, sim/ or tests/; the INTERFACE_INCLUDE_DIRECTORIES
            and INTERFACE_SYSTEM_INCLUDE_DIRECTORIES of every INTERFACE library are held to the same rule, consumed
            or not
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

FORBIDDEN_SYMBOLS = ("marv::sil::", "marv_sil_", "marv::hal_sim::", "marv::truth::", "marv_truth_", "marv::plant::",
                     "marv_plant_")
FORBIDDEN_INCLUDE_DIRS = ("fw/sil", "fw/hal/sim", "sim", "tests")
INCLUDE_OPTIONS = ("-isystem", "-iquote", "-idirafter", "-I")
FORCED_INCLUDE_OPTIONS = ("-include", "-imacros")
LINKED_TYPES = ("STATIC_LIBRARY", "SHARED_LIBRARY", "MODULE_LIBRARY", "EXECUTABLE")
OBJECT_TYPE = "OBJECT_LIBRARY"
INTERFACE_TYPE = "INTERFACE_LIBRARY"
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


def _option_values(args: list[str], options: tuple[str, ...]) -> list[str]:
    """Values of the given options in either spelling: '-Xvalue' (attached) and '-X value' (separate)."""
    found = []
    i = 0
    while i < len(args):
        arg = args[i]
        for opt in options:
            if arg == opt:
                if i + 1 < len(args):
                    found.append(args[i + 1])
                i += 1
                break
            if arg.startswith(opt):
                found.append(arg[len(opt):])
                break
        i += 1
    return found


def include_dirs(args: list[str], directory: str) -> list[str]:
    """Include directories named by -I, -isystem, -iquote and -idirafter (attached or separate), resolved.

    The sysroot-relative spelling (-I=dir, -isystem=dir) has its '=' stripped; the remainder is checked as a path.
    """
    found = [d[1:] if d.startswith("=") else d for d in _option_values(args, INCLUDE_OPTIONS)]
    return [os.path.realpath(os.path.join(directory, d)) for d in found]


def forced_includes(args: list[str], directory: str) -> list[str]:
    """Files named by -include and -imacros (attached or separate), resolved."""
    return [os.path.realpath(os.path.join(directory, f)) for f in _option_values(args, FORCED_INCLUDE_OPTIONS)]


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


def _field(target: dict, key: str) -> list[str] | str:
    if key not in target:
        raise G3Error(f"flight target {target.get('name')} ({target.get('type')}) has no '{key}' in the manifest")
    return target[key]


def _paths(target: dict, key: str) -> list[str]:
    """A list field of a manifest entry without the empty strings that an empty generator-expression list yields."""
    value = _field(target, key)
    if not isinstance(value, list):
        raise G3Error(f"flight target {target.get('name')}: '{key}' is not a list")
    return [p for p in value if p]


def target_kind(target: dict) -> str:
    """The type of a flight target if check_g3.py knows how to check it; G3Error otherwise."""
    kind = target.get("type")
    if kind in LINKED_TYPES or kind in (OBJECT_TYPE, INTERFACE_TYPE):
        return kind
    raise G3Error(f"flight target {target.get('name')} has type {kind!r}, which check_g3.py does not know how to "
                  f"check: teach it (and add a negative control) or move the target out of fw/")


def symbol_artifacts(target: dict) -> list[str]:
    """The files whose symbols are scanned for a flight target; empty for an INTERFACE library."""
    kind = target_kind(target)
    if kind == INTERFACE_TYPE:
        return []
    if kind == OBJECT_TYPE:
        objects = _paths(target, "objects")
        if not objects:
            raise G3Vacuous(f"OBJECT library {target['name']} lists no object files: the symbol check would be vacuous")
        return objects
    artifact = _field(target, "file")
    if not artifact:
        raise G3Error(f"flight target {target['name']} ({kind}) has an empty 'file' in the manifest")
    return [artifact]


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
    checked = []
    for target in targets:
        artifacts = symbol_artifacts(target)
        if not artifacts:
            continue
        checked.append(target["name"])
        for artifact in artifacts:
            symbols = run_nm(nm, ["-C"], artifact)
            scanned += len(symbols)
            for member, kind, name, pattern in find_forbidden_symbols(symbols):
                violations += 1
                print(f"G3-SYMBOL {target['name']} ({artifact}) member {member or os.path.basename(artifact)}: "
                      f"'{name}' [{kind}] contains '{pattern}'")
    if not checked:
        raise G3Vacuous("the manifest lists no flight target with an artifact: the symbol check would be vacuous")
    if scanned == 0:
        raise G3Vacuous("the flight artifacts contain no symbols at all: the symbol check would be vacuous")
    print(f"G3 symbols: {len(checked)} flight targets, {scanned} symbols scanned, "
          f"{violations} violations ({', '.join(checked)})")
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
    tu_count = {}
    interfaces = []
    for target in targets:
        if target_kind(target) == INTERFACE_TYPE:
            interfaces.append(target)
            continue
        tu_count[target["name"]] = 0
        for obj in _paths(target, "objects"):
            object_owner[os.path.normpath(obj)] = target["name"]
    violations = 0
    for target in interfaces:
        declared = _paths(target, "include_dirs") + _paths(target, "system_include_dirs")
        for directory in declared:
            resolved = os.path.realpath(os.path.join(str(root), directory))
            forbidden = forbidden_include_dir(resolved, str(root))
            if forbidden is not None:
                violations += 1
                print(f"G3-INCLUDE {target['name']}: INTERFACE include directory {resolved} inside {forbidden}/")
    for entry in database:
        owner = object_owner.get(entry_output(entry))
        if owner is None:
            continue
        tu_count[owner] += 1
        args = command_args(entry)
        for directory in include_dirs(args, entry["directory"]):
            forbidden = forbidden_include_dir(directory, str(root))
            if forbidden is not None:
                violations += 1
                print(f"G3-INCLUDE {owner}: {entry['file']} has include directory {directory} "
                      f"inside {forbidden}/")
        for forced in forced_includes(args, entry["directory"]):
            forbidden = forbidden_include_dir(forced, str(root))
            if forbidden is not None:
                violations += 1
                print(f"G3-INCLUDE {owner}: {entry['file']} has forced include {forced} inside {forbidden}/")
    empty = [name for name, count in tu_count.items() if count == 0]
    if empty:
        raise G3Vacuous(f"no compile_commands entry found for flight targets {empty}: the include check would be vacuous")
    total = sum(tu_count.values())
    if total == 0:
        raise G3Vacuous("no flight target has a translation unit: the include check would be vacuous")
    print(f"G3 includes: {len(targets)} flight targets ({len(interfaces)} INTERFACE), {total} translation units, "
          f"{violations} violations ({', '.join(f'{n}={c}' for n, c in tu_count.items())}; "
          f"INTERFACE: {', '.join(t['name'] for t in interfaces)})")
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
