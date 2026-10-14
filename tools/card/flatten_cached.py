#!/usr/bin/env python3
"""flatten.py behind a content-keyed cache of its outputs (python stdlib plus tools/refdata; the arguments are flatten.py's).

  flatten_cached.py <flatten.py arguments>

Without $MARV_CACHE_DIR this replaces itself with flatten.py (same interpreter, same arguments): nothing else happens.

With $MARV_CACHE_DIR the outputs of flatten.py are kept in $MARV_CACHE_DIR/flatten/<key>/, where <key> is the sha256 of
  - every file in flatten.py's repo-local import closure and the files that closure reads (tools/refdata/refdata_closure.py,
    the same static closure the reference-data stamp uses; sensors/profiles/*.yaml are the declared reads of the card tools),
    uv.lock, the card, budget and scenario files, this file and refdata_closure.py, by content;
  - the python, numpy, scipy, machine, libc and OS lines of the reference-data stamp (tools/refdata/refdata.py), and the BLAS
    thread-count variables when set;
  - the arguments: --card, --budget, --scenario as repository-relative paths (their paths appear in the output headers),
    each --out-* flag by its name and extension only, so the build directory and the parameter-set name are not part of the
    key (the outputs do not contain them); --procs and --root are not (the outputs do not depend on --procs, flatten.py's
    docstring; --root must be this repository).
A hit writes the stored bytes to the requested paths. A miss runs flatten.py into a scratch directory under the cache,
stores it (written whole, renamed into place) and then writes the requested paths from it. flatten.py failing (a lint
finding, a refusal) stores nothing and exits with its status. Concurrent misses for one key take a lock, so one runs
flatten.py and the others hit. An entry is used only if its key text equals the computed one and every file's sha256 equals
the manifest's; otherwise it is replaced. A run the cache cannot key (an unknown argument, an input outside the
repository, another --root, an unresolvable read in the closure) or an unusable cache directory runs flatten.py uncached,
with a note on stderr, so its result is always flatten.py's own.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FLATTEN = HERE / "flatten.py"
sys.path.insert(0, str(REPO / "tools" / "refdata"))
import refdata  # noqa: E402
import refdata_closure  # noqa: E402

OUT_FLAGS = ("card", "register", "mixer", "scenario", "rate", "attitude", "gyro_chain", "rate_lead", "attitude_lead", "vertical")
IN_FLAGS = ("card", "budget", "scenario")
# sensors/profiles/<id>.yaml is read through `<root> / lint.PROFILE_DIR / <id>` (flatten.py, lint.py, gen_plant_config.py);
# cpu_quota.py reads the container's cgroup files (the worker count only).
DEPENDS = ("sensors/profiles/*.yaml",)
UNRESOLVED_OK = ("tools/card/cpu_quota.py", "tools/card/flatten.py", "tools/card/gen_plant_config.py", "tools/card/lint.py")
THREAD_VARS = ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")
MANIFEST = "MANIFEST"
KEY_TEXT = "KEY"


def say(text):
    print(f"flatten_cached: {text}", file=sys.stderr, flush=True)


def run_flatten(argv):
    return subprocess.run([sys.executable, str(FLATTEN), *argv], check=False).returncode


class Uncacheable(Exception):
    """A run the cache cannot key; flatten.py then runs uncached, exactly as without MARV_CACHE_DIR."""


def parse(argv):
    ap = argparse.ArgumentParser(prog="flatten_cached.py", add_help=False, allow_abbrev=False, exit_on_error=False)
    for name in IN_FLAGS:
        ap.add_argument(f"--{name}")
    for role in OUT_FLAGS:
        ap.add_argument(f"--out-{role.replace('_', '-')}")
    ap.add_argument("--procs")
    ap.add_argument("--root")
    try:
        args, unknown = ap.parse_known_args(argv)
    except argparse.ArgumentError as e:
        raise Uncacheable(f"arguments not understood ({e})") from None
    if unknown:
        raise Uncacheable(f"arguments not understood ({' '.join(unknown)})")
    joined = [a for a in argv if a.startswith("--out-") and "=" in a]
    if joined:
        raise Uncacheable(f"output flags given as --flag=value ({' '.join(joined)})")
    return args


def repo_rel(path):
    try:
        return Path(path).resolve().relative_to(REPO).as_posix()
    except ValueError:
        raise Uncacheable(f"{path} is not inside {REPO}") from None


def cpu_model():
    """The CPU model: libm may pick a different code path per CPU and nothing checks these outputs against committed hashes."""
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def key_text(args):
    files, problems = refdata_closure.closure(FLATTEN, DEPENDS, UNRESOLVED_OK)
    if problems:
        raise Uncacheable("the dependency closure of flatten.py has reads it cannot resolve (declare them in "
                          "DEPENDS / UNRESOLVED_OK): " + "; ".join(problems))
    lines = []
    for flag in IN_FLAGS:
        value = getattr(args, flag)
        if value:
            lines.append(f"arg --{flag} {repo_rel(value)}")
            files.append(repo_rel(value))
    for role in OUT_FLAGS:
        value = getattr(args, f"out_{role}")
        if value:
            lines.append(f"arg --out-{role.replace('_', '-')} *{Path(value).suffix}")
    for rel in sorted({*files, "uv.lock", repo_rel(__file__), repo_rel(refdata_closure.__file__), repo_rel(refdata.__file__)}):
        lines.append(f"file {rel} {refdata.sha256_of(REPO / rel)}")
    lines += refdata.environment_lines()
    lines.append(f"cpu {cpu_model()}")
    lines.append(f"scipy {refdata.module_version('scipy')}")
    lines += [f"env {name}={os.environ.get(name, '')}" for name in THREAD_VARS if name in os.environ]
    return "\n".join(lines) + "\n"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_entry(entry, text):
    """{relative name: bytes} of a valid entry, else None."""
    try:
        if (entry / KEY_TEXT).read_text(encoding="utf-8") != text:
            return None
        sums = {}
        for line in (entry / MANIFEST).read_text(encoding="utf-8").splitlines():
            digest, name = line.split(None, 1)
            sums[name] = digest
        data = {}
        for name, digest in sums.items():
            blob = (entry / name).read_bytes()
            if sha(blob) != digest:
                return None
            data[name] = blob
        present = {p.relative_to(entry).as_posix() for p in entry.rglob("*") if p.is_file()} - {MANIFEST, KEY_TEXT}
        return data if present == set(sums) and data else None
    except (OSError, ValueError):
        return None


def targets(args, data):
    """[(destination, bytes)]: each role directory's `out<suffix>` goes to the requested path, `out<more>` beside it with
    the requested stem; None if a requested output is not in the entry or the entry holds a name no request explains."""
    placed, used = [], set()
    for role in OUT_FLAGS:
        value = getattr(args, f"out_{role}")
        if not value:
            continue
        dest = Path(value)
        names = [n for n in data if n.startswith(role + "/out")]
        if f"{role}/out{dest.suffix}" not in data:
            return None
        for name in names:
            tail = name[len(role + "/out"):]
            placed.append((dest if tail == dest.suffix else dest.with_name(dest.stem + tail), data[name]))
            used.add(name)
    return placed if used == set(data) else None


def write_all(placed):
    for dest, blob in placed:
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_name(dest.name + f".tmp{os.getpid()}")
        tmp.write_bytes(blob)
        os.replace(tmp, dest)


def generate(argv, args, stage):
    """flatten.py's arguments with every --out-* path moved into `stage`/<role>/out<suffix>; its exit status."""
    staged = list(argv)
    for role in OUT_FLAGS:
        value = getattr(args, f"out_{role}")
        if value:
            flag = f"--out-{role.replace('_', '-')}"
            index = staged.index(flag) if flag in staged else None
            if index is None:
                raise SystemExit(f"flatten_cached: {flag} given as {flag}=... is not supported")
            staged[index + 1] = str(stage / role / f"out{Path(value).suffix}")
    return run_flatten(staged)


def store(stage, entry, text):
    names = sorted(p.relative_to(stage).as_posix() for p in stage.rglob("*") if p.is_file())
    (stage / KEY_TEXT).write_text(text, encoding="utf-8")
    (stage / MANIFEST).write_text("".join(f"{sha((stage / n).read_bytes())} {n}\n" for n in names), encoding="utf-8")
    if entry.exists():
        gone = entry.with_name(entry.name + f".rejected{os.getpid()}")
        os.replace(entry, gone)
        shutil.rmtree(gone, ignore_errors=True)
    os.replace(stage, entry)


def main(argv):
    cache = os.environ.get("MARV_CACHE_DIR")
    if not cache:
        os.execv(sys.executable, [sys.executable, str(FLATTEN), *argv])
    try:
        args = parse(argv)
        if not args.root or Path(args.root).resolve() != REPO:
            raise Uncacheable(f"--root is not {REPO}")
        text = key_text(args)
    except Uncacheable as e:
        say(f"{e}: running flatten.py uncached")
        return run_flatten(argv)
    key = sha(text.encode())
    root = Path(cache) / "flatten"
    try:
        root.mkdir(parents=True, exist_ok=True)
        lock = open(root / f"{key}.lock", "w")
    except OSError as e:
        say(f"cache directory unusable ({e}): running flatten.py uncached")
        return run_flatten(argv)
    entry = root / key
    with lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = read_entry(entry, text) if entry.exists() else None
        placed = targets(args, data) if data else None
        if placed is not None:
            write_all(placed)
            say(f"hit {key[:12]}")
            return 0
        say(f"{'rejected a bad entry, ' if entry.exists() else ''}miss {key[:12]}: running flatten.py")
        stage = root / f"{key}.stage{os.getpid()}"
        shutil.rmtree(stage, ignore_errors=True)
        stage.mkdir()
        try:
            status = generate(argv, args, stage)
            if status != 0:
                return status
            store(stage, entry, text)
        finally:
            shutil.rmtree(stage, ignore_errors=True)
        data = read_entry(entry, text)
        placed = targets(args, data) if data else None
        if placed is None:
            raise SystemExit(f"flatten_cached: the stored entry {entry} does not hold the requested outputs")
        write_all(placed)
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
