#!/usr/bin/env python3
"""Reference data that is regenerated, not stored (decision 0011).

A reference set is a generator, its committed input files and a committed SHA256SUMS (sha256sum format, one line per
generated file) in the set's source directory. `ensure <id>` makes <out>/<id>/ (out: --out, else the environment variable
MARV_REFERENCE_DIR, else <repo>/build/reference), copies the inputs in, regenerates the files when one is missing, differs
from SHA256SUMS or was generated from other inputs or another generator, and verifies every file against SHA256SUMS. A
mismatch exits non-zero naming the file and both hashes. SHA256SUMS is never rewritten by this tool.

    uv run python tools/refdata/refdata.py ensure quad/L05/t3 [--out DIR] [--procs N] [--force]
    uv run python tools/refdata/refdata.py verify quad/L05/t3 --dir DIR

The output root is, in order: --out, $MARV_REFERENCE_DIR, $MARV_CACHE_DIR/reference (the persistent CI cache), then
<repo>/build/reference. A directory is reused only when its stamp is current and every file matches SHA256SUMS (hash check
only); otherwise it is regenerated, so a cache entry exists only because the generator produced the committed hashes from
the stamped files.

The stamp is a manifest, one sha256 per line, of: the set's inputs, its generator, the generator's repository-local Python
import closure, every repository file that closure names in a path expression, the set's `depends` globs (all found by
refdata_closure.py in this directory), uv.lock, and the interpreter, numpy, libc and OS versions. `stamp <id>` prints it.

    uv run python tools/refdata/refdata.py stamp quad/L05/t3

A negative control (ci/run_ci.sh) regenerates the set from a perturbed input and requires the SHA256SUMS check to fail. Its
verdict is a function of the set's stamp, its SHA256SUMS, the perturbed input, this tool and the control's own text, so with
MARV_CACHE_DIR set the verified failure is recorded under $MARV_CACHE_DIR/reference/controls/ and not re-run while that key is
unchanged:
`control-key <id> --input FILE --text FILE...`, `control-lookup <id> <name> <key>`, `control-store <id> <name> <key> --log FILE`.

`ensure` prints the directory on stdout (progress goes to stderr). Importable: reference_dir(id) -> Path does the same.
"""

import argparse
import contextlib
import fcntl
import hashlib
import importlib.metadata
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import refdata_closure  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SUMS = "SHA256SUMS"
STAMP = ".generated-from"


class RefDataError(RuntimeError):
    pass


class RefSet:
    """depends: repo-relative globs of files the generator's closure reads that refdata_closure.py cannot resolve statically.
    unresolved_ok: repo-relative Python files in the closure whose unresolvable path expressions `depends` covers (or that
    name locations outside the repository, said in the comment beside the declaration); refdata_closure.py fails on any other."""

    def __init__(self, source, inputs, generator, procs, depends=(), unresolved_ok=()):
        self.source = REPO / source
        self.inputs = inputs
        self.generator = generator
        self.procs = procs
        self.depends = tuple(depends)
        self.unresolved_ok = tuple(unresolved_ok)


# The card tools the L5 and L11 oracles import (for their --refresh-* paths; refdata never passes those options) build
# `<root> / lint.PROFILE_DIR / <profile id>.yaml` paths, write to caller-given output directories, read the build tree's
# param_defaults.cpp and the container's cgroup files. The profile files are the repository reads, declared in `depends`;
# the rest is outside the repository.
CARD_TOOLS_UNRESOLVED = (
    "tools/card/cpu_quota.py",  # /sys/fs/cgroup: not a repository file
    "tools/card/gen_plant_config.py",  # sensors/profiles/<id>.yaml: depends
    "tools/card/gen_world.py",  # sensors/profiles/<id>.yaml: depends
    "tools/card/lint.py",  # sensors/profiles/<id>.yaml: depends
    "tools/card/report.py",  # sensors/profiles/<id>.yaml: depends
    "tools/sim/run_l4.py",  # sensors/profiles/<id>.yaml: depends; the build tree and output directories: not repository files
    "tools/sim/run_scenario.py",  # the plugin build directory and output directories: not repository files
)
CARD_PROFILES = ("sensors/profiles/*.yaml",)


REGISTRY = {
    "quad/L04/t3": RefSet(
        "tests/regression/quad/L04/t3/reference",
        ["rate_t3_inputs.txt"],
        "rate_t3_oracle.py",
        procs=False,
    ),
    "quad/L05/t3": RefSet(
        "tests/regression/quad/L05/t3/reference",
        ["attitude_t3_inputs.txt", "attitude_t3_q_inputs.txt"],
        "attitude_t3_oracle.py",
        procs=True,
        depends=CARD_PROFILES,
        unresolved_ok=CARD_TOOLS_UNRESOLVED,
    ),
    "quad/L11/t3": RefSet(
        "tests/regression/quad/L11/t3/reference",
        ["vertical_t3_inputs.txt"],
        "vertical_t3_oracle.py",
        procs=True,
        depends=CARD_PROFILES,
        unresolved_ok=CARD_TOOLS_UNRESOLVED,
    ),
}


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def read_sums(ref):
    sums = {}
    for line in (ref.source / SUMS).read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        digest, name = line.split(None, 1)
        sums[name.strip().lstrip("*")] = digest
    return sums


def mismatches(directory, sums):
    """One text per generated file of `sums` that is missing from `directory` or whose sha256 differs."""
    found = []
    for name, want in sums.items():
        path = directory / name
        if not path.is_file():
            found.append(f"{name}: missing (expected sha256 {want})")
            continue
        got = sha256_of(path)
        if got != want:
            found.append(f"{name}: sha256 {got} differs from {SUMS} {want}")
    return found


def default_out():
    env = os.environ.get("MARV_REFERENCE_DIR")
    if env:
        return Path(env)
    cache = os.environ.get("MARV_CACHE_DIR")
    return Path(cache) / "reference" if cache else REPO / "build" / "reference"


def module_version(name):
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "absent"


def environment_lines():
    os_release = {}
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            key, _, value = line.partition("=")
            os_release[key] = value.strip('"')
    except OSError:
        pass
    return [
        f"python {platform.python_implementation()} {'.'.join(map(str, sys.version_info[:3]))}",
        f"numpy {module_version('numpy')}",
        f"machine {platform.machine()}",
        f"libc {'-'.join(platform.libc_ver())}",
        f"os {os_release.get('ID', 'unknown')} {os_release.get('VERSION_ID', 'unknown')}",
    ]


def stamp_files(ref):
    """Repo-relative files whose content the set's output depends on; raises RefDataError on an unresolved read."""
    files, problems = refdata_closure.closure(ref.source / ref.generator, ref.depends, ref.unresolved_ok)
    if problems:
        raise RefDataError(
            f"{ref.generator}: the dependency closure has reads it cannot resolve (declare them in the set's depends / "
            "unresolved_ok):\n  " + "\n  ".join(problems)
        )
    names = {(ref.source / name).relative_to(REPO).as_posix() for name in [*ref.inputs, ref.generator]}
    return sorted({*files, *names, "uv.lock"})


def stamp_of(ref):
    lines = [f"file {rel} {sha256_of(REPO / rel)}" for rel in stamp_files(ref)]
    return "\n".join(lines + environment_lines()) + "\n"


def stamp_diff(old, new):
    old_lines, new_lines = set(old.splitlines()), set(new.splitlines())
    return sorted(f"-{line}" for line in old_lines - new_lines) + sorted(f"+{line}" for line in new_lines - old_lines)


def control_key(ref, perturbed_input, texts):
    """sha256 over the set's stamp manifest, its SHA256SUMS, the perturbed input, this tool and the control's text files."""
    h = hashlib.sha256()
    h.update(stamp_of(ref).encode())
    for label, path in [
        ("sums", ref.source / SUMS),
        ("perturbed-input", perturbed_input),
        ("refdata.py", Path(__file__).resolve()),
        ("refdata_closure.py", Path(refdata_closure.__file__).resolve()),
        *[("control-text", t) for t in texts],
    ]:
        h.update(f"{label} {sha256_of(path)}\n".encode())
    return h.hexdigest()


def control_record(set_id, name, key):
    """The record path of a verified control failure, or None without MARV_CACHE_DIR."""
    cache = os.environ.get("MARV_CACHE_DIR")
    if not cache:
        return None
    return Path(cache) / "reference" / "controls" / f"{set_id.replace('/', '_')}-{name}.{key}"


def copy_atomic(src, dst):
    if dst.is_file() and sha256_of(dst) == sha256_of(src):
        return
    tmp = dst.with_name(dst.name + f".tmp{os.getpid()}")
    shutil.copyfile(src, tmp)
    os.replace(tmp, dst)


@contextlib.contextmanager
def locked(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def reference_dir(set_id, out=None, procs=None, force=False):
    if set_id not in REGISTRY:
        raise RefDataError(f"unknown reference set '{set_id}' (known: {', '.join(sorted(REGISTRY))})")
    ref = REGISTRY[set_id]
    out = Path(out) if out else default_out()
    target = out / set_id
    sums = read_sums(ref)
    with locked(out / f"{set_id}.lock"):
        target.mkdir(parents=True, exist_ok=True)
        for name in ref.inputs:
            copy_atomic(ref.source / name, target / name)
        stamp = stamp_of(ref)
        stamp_path = target / STAMP
        recorded = stamp_path.read_text() if stamp_path.is_file() else None
        if force:
            why = "--force"
        elif recorded is None:
            why = "no stamp"
        elif recorded != stamp:
            why = "stamp mismatch:\n    " + "\n    ".join(stamp_diff(recorded, stamp))
        else:
            bad = mismatches(target, sums)
            if not bad:
                print(f"refdata: {set_id}: stamp current, {len(sums)} files match {SUMS}: using {target}", file=sys.stderr, flush=True)
                return target
            why = f"entry rejected, not used:\n    " + "\n    ".join(bad)
        print(f"refdata: {set_id}: regenerating ({why})", file=sys.stderr, flush=True)
        work = out / f"{set_id}.work{os.getpid()}"
        shutil.rmtree(work, ignore_errors=True)
        work.mkdir(parents=True)
        try:
            for name in ref.inputs:
                shutil.copyfile(ref.source / name, work / name)
            command = [sys.executable, str(ref.source / ref.generator), "--dir", str(work)]
            if ref.procs:
                command += ["--procs", str(procs or os.cpu_count() or 1)]
            print(f"refdata: generating {set_id}: {' '.join(command)}", file=sys.stderr, flush=True)
            proc = subprocess.run(command, cwd=REPO, stdout=sys.stderr, check=False)
            if proc.returncode != 0:
                raise RefDataError(f"{set_id}: the generator exited {proc.returncode}")
            bad = mismatches(work, sums)
            if bad:
                failed = out / f"{set_id}.failed"
                shutil.rmtree(failed, ignore_errors=True)
                os.replace(work, failed)
                raise RefDataError(
                    f"{set_id}: the generated files do not match {ref.source / SUMS} (output kept in {failed}):\n  "
                    + "\n  ".join(bad)
                )
            stamp_path.unlink(missing_ok=True)
            for name in sums:
                os.replace(work / name, target / name)
            tmp = stamp_path.with_name(STAMP + f".tmp{os.getpid()}")
            tmp.write_text(stamp)
            os.replace(tmp, stamp_path)
        finally:
            shutil.rmtree(work, ignore_errors=True)
    return target


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    ens = sub.add_parser("ensure", help="make <out>/<id>/ hold the verified reference files and print it")
    ens.add_argument("id")
    ens.add_argument("--out", help="output root (default: $MARV_REFERENCE_DIR, else $MARV_CACHE_DIR/reference, else <repo>/build/reference)")
    ens.add_argument("--procs", type=int, help="generator worker processes (default: os.cpu_count())")
    ens.add_argument("--force", action="store_true", help="regenerate even when the files are present and verified")
    stm = sub.add_parser("stamp", help="print the stamp manifest of a set (the files and versions its output depends on)")
    stm.add_argument("id")
    ckey = sub.add_parser("control-key", help="print the cache key of a negative control's verdict")
    ckey.add_argument("id")
    ckey.add_argument("--input", required=True, help="the perturbed input file")
    ckey.add_argument("--text", nargs="+", required=True, help="files holding the control's own text")
    clook = sub.add_parser("control-lookup", help="exit 0 and print the recorded failure's files when the verdict is cached")
    clook.add_argument("id")
    clook.add_argument("name")
    clook.add_argument("key")
    cstore = sub.add_parser("control-store", help="record a verified control failure (the verify stderr)")
    cstore.add_argument("id")
    cstore.add_argument("name")
    cstore.add_argument("key")
    cstore.add_argument("--log", required=True)
    ver = sub.add_parser("verify", help="check the files of a directory against the set's SHA256SUMS")
    ver.add_argument("id")
    ver.add_argument("--dir", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "ensure":
            print(reference_dir(args.id, args.out, args.procs, args.force))
            return 0
        if args.id not in REGISTRY:
            raise RefDataError(f"unknown reference set '{args.id}'")
        if args.command == "stamp":
            print(stamp_of(REGISTRY[args.id]), end="")
            return 0
        if args.command == "control-key":
            print(control_key(REGISTRY[args.id], args.input, args.text))
            return 0
        if args.command == "control-lookup":
            record = control_record(args.id, args.name, args.key)
            if record is None or not record.is_file():
                return 1
            names = re.findall(r"^\s*(\S+): sha256 ", record.read_text(), re.M)
            print(", ".join(names))
            return 0
        if args.command == "control-store":
            record = control_record(args.id, args.name, args.key)
            if record is not None:
                record.parent.mkdir(parents=True, exist_ok=True)
                tmp = record.with_name(record.name + f".tmp{os.getpid()}")
                shutil.copyfile(args.log, tmp)
                os.replace(tmp, record)
            return 0
        sums = read_sums(REGISTRY[args.id])
        bad = mismatches(Path(args.dir), sums)
        if bad:
            print(f"{args.id}: {args.dir} does not match {SUMS}:\n  " + "\n  ".join(bad), file=sys.stderr)
            return 1
        print(f"{args.id}: {args.dir} matches {SUMS}")
        return 0
    except RefDataError as err:
        print(f"refdata: FAIL: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
