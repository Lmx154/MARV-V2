#!/usr/bin/env python3
"""Reference data that is regenerated, not stored (decision 0011).

A reference set is a generator, its committed input files and a committed SHA256SUMS (sha256sum format, one line per
generated file) in the set's source directory. `ensure <id>` makes <out>/<id>/ (out: --out, else the environment variable
MARV_REFERENCE_DIR, else <repo>/build/reference), copies the inputs in, regenerates the files when one is missing, differs
from SHA256SUMS or was generated from other inputs or another generator, and verifies every file against SHA256SUMS. A
mismatch exits non-zero naming the file and both hashes. SHA256SUMS is never rewritten by this tool.

    uv run python tools/refdata/refdata.py ensure quad/L05/t3 [--out DIR] [--procs N] [--force]
    uv run python tools/refdata/refdata.py verify quad/L05/t3 --dir DIR

`ensure` prints the directory on stdout (progress goes to stderr). Importable: reference_dir(id) -> Path does the same.
"""

import argparse
import contextlib
import fcntl
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SUMS = "SHA256SUMS"
STAMP = ".generated-from"


class RefDataError(RuntimeError):
    pass


class RefSet:
    def __init__(self, source, inputs, generator, procs):
        self.source = REPO / source
        self.inputs = inputs
        self.generator = generator
        self.procs = procs


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
    return Path(env) if env else REPO / "build" / "reference"


def stamp_of(ref):
    h = hashlib.sha256()
    for name in [*ref.inputs, ref.generator]:
        h.update(f"{name} {sha256_of(ref.source / name)}\n".encode())
    return h.hexdigest() + "\n"


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
        current = stamp_path.is_file() and stamp_path.read_text() == stamp
        if not force and current and not mismatches(target, sums):
            return target
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
    ens.add_argument("--out", help="output root (default: $MARV_REFERENCE_DIR, else <repo>/build/reference)")
    ens.add_argument("--procs", type=int, help="generator worker processes (default: os.cpu_count())")
    ens.add_argument("--force", action="store_true", help="regenerate even when the files are present and verified")
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
        bad = mismatches(Path(args.dir), read_sums(REGISTRY[args.id]))
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
