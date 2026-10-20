#!/usr/bin/env python3
"""GX-13 evidence: one real derivation of the flight card by the helper's DeriveManager (tools/gs/derive.py), timed, compared byte for byte with the
files of an existing CMake build tree, with `git status` before and after (W-1).

  ci/heavy.sh --wait 10800 -- .venv/bin/python tests/regression/quad/L10/results/gcs/gx13/derive_real.py --work-dir DIR --cmake-tree build/host-debug

DIR must lie outside the repository. Exit 0 when the .mcfg, every flatten file and the four generated files are identical to the tree's and
the working tree is unchanged.
"""

import argparse
import asyncio
import hashlib
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[7]
sys.path.insert(0, str(ROOT / "tools" / "gs"))
import derive  # noqa: E402

CARD = "vehicles/marv_7in_fc3v2.yaml"


class Library:
    def scan(self):
        return {"images": [], "unkeyed": []}


def tree_state() -> str:
    status = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain=v1", "-uall"], capture_output=True, text=True, check=True).stdout
    diff = subprocess.run(["git", "-C", str(ROOT), "diff", "HEAD"], capture_output=True, check=True).stdout
    return f"{status}diff sha256 {hashlib.sha256(diff).hexdigest()}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--work-dir", type=Path, required=True)
    ap.add_argument("--cmake-tree", type=Path, required=True)
    args = ap.parse_args()
    work_dir = args.work_dir.resolve()
    if ROOT in work_dir.parents or work_dir == ROOT:
        raise SystemExit("the work directory must be outside the repository")
    before = tree_state()
    print(f"git status before:\n{before}")
    events = []

    async def run() -> Path:
        m = derive.DeriveManager(root=ROOT, run_dir=work_dir, build_root=work_dir, library=Library(), log=print, push=events.append,
                                 lock_path=lambda: work_dir / "heavy.lock")
        started = time.monotonic()
        await m.start({"cmd": "derive", "card": CARD, "edits": []})
        await m._task
        print(f"derivation took {time.monotonic() - started:.1f} s; CPUs {derive.os.cpu_count()}; state {m.state['state']}; error {m.state['error']}")
        assert m.state["ok"] is True
        print(f"config hash {m.state['config_hash']}; schema hash {m.state['schema_hash']}; mcfg {m.state['mcfg']}")
        print("events: " + ", ".join(e["event"] + (":" + e["step"] if "step" in e else "") for e in events))
        return m.derive_dir / m.state["id"]

    work = asyncio.run(run())
    generated = (ROOT / args.cmake_tree / "generated" / derive.PARAMS_SET).resolve()
    ok = True

    def same(mine: Path, theirs: Path, label: str) -> None:
        nonlocal ok
        equal = mine.read_bytes() == theirs.read_bytes()
        ok &= equal
        print(f"{'identical' if equal else 'DIFFERENT'}  {label}  sha256 {hashlib.sha256(mine.read_bytes()).hexdigest()}")

    same(work / f"{derive.PARAMS_SET}.mcfg", generated / f"{derive.PARAMS_SET}.mcfg", f"{derive.PARAMS_SET}.mcfg")
    names = sorted(p.name for p in (generated / "flatten").iterdir())
    ok &= names == sorted(p.name for p in (work / "flatten").iterdir())
    for name in names:
        same(work / "flatten" / name, generated / "flatten" / name, f"flatten/{name}")
    for name in ("params_manifest.json", "params_provenance.json", "param_ids.hpp", "param_defaults.cpp"):
        same(work / "gen" / "marv" / "params" / name, generated / "marv" / "params" / name, f"gen/{name}")
    after = tree_state()
    print(f"git status after:\n{after}")
    ok &= before == after
    print("W-1: the working tree is " + ("unchanged" if before == after else "CHANGED"))
    print("RESULT " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
