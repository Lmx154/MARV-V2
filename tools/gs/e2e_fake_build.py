#!/usr/bin/env python3
"""The fake cmake and ninja of the GX-14 tests (tests/tools/test_gs_build.py, tools/gs/e2e.sh light mode). A test tool, never flown.

`e2e_fake_build.py --install DIR` writes two executables, DIR/cmake and DIR/ninja, that run this script; a test puts DIR first on the
helper's PATH. They do what the helper's build asks of the real tools, on the tree it names, and nothing else:
  cmake --preset P -B TREE [-DK=V ...]   creates TREE/build.ninja; exit 0
  cmake --build TREE [--target T ...]    prints STEPS lines "[k/STEPS] Building CXX object ..." (STEP_S apart), then writes the SIL library
                                         TREE/sim/quad_fc/libmarv_sil_quad_fc.so (its bytes differ for each build) and TREE/.fake_built
  ninja -n -C TREE [T ...]               "ninja: no work to do." when TREE/.fake_built exists, else STEPS lines "[k/STEPS] ..." (a dry run)
Every call appends its argv as one JSON line to $MARV_FAKE_BUILD_DIR/argv.jsonl (so a test sees what ran, and that nothing did).
Environment (all optional): MARV_FAKE_BUILD_STEPS (default 3), MARV_FAKE_BUILD_STEP_S (default 0), MARV_FAKE_BUILD_STATUS (the exit status of
`--build`, default 0), MARV_FAKE_BUILD_HOLD=1 (`--build` leaves a child in its own session, as ninja's jobs are, and one in its group, writes their
pids to $MARV_FAKE_BUILD_DIR/pids and then waits), MARV_FAKE_BUILD_STUBBORN=1 (it ignores SIGTERM, and so do the children).
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HOLD_SLEEP_S = "600"  # harness value: a leftover child outlives any test; a test kills it, or the helper's cancel does


def state_dir() -> Path:
    if not os.environ.get("MARV_FAKE_BUILD_DIR"):
        raise SystemExit("MARV_FAKE_BUILD_DIR is not set")
    d = Path(os.environ["MARV_FAKE_BUILD_DIR"])
    d.mkdir(parents=True, exist_ok=True)
    return d


def steps() -> int:
    return int(os.environ.get("MARV_FAKE_BUILD_STEPS", "3"))


def install(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for tool in ("cmake", "ninja"):
        path = directory / tool
        path.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{Path(__file__).resolve()}" {tool} "$@"\n')
        path.chmod(0o755)


def build(tree: Path) -> int:
    n = steps()
    stubborn = os.environ.get("MARV_FAKE_BUILD_STUBBORN") == "1"
    if stubborn:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    for k in range(1, n + 1):
        print(f"[{k}/{n}] Building CXX object fake_{k}.o", flush=True)
        time.sleep(float(os.environ.get("MARV_FAKE_BUILD_STEP_S", "0")))
        if k == 1 and os.environ.get("MARV_FAKE_BUILD_HOLD") == "1":
            ignore = ["trap '' TERM;"] if stubborn else []
            own = subprocess.Popen(["sh", "-c", " ".join([*ignore, "exec sleep", HOLD_SLEEP_S])], start_new_session=True)
            same = subprocess.Popen(["sh", "-c", " ".join([*ignore, "exec sleep", HOLD_SLEEP_S])])
            (state_dir() / "pids").write_text(f"{os.getpid()} {own.pid} {same.pid}\n")
            print("[hold] children started", flush=True)
            while True:
                time.sleep(float(HOLD_SLEEP_S))
    status = int(os.environ.get("MARV_FAKE_BUILD_STATUS", "0"))
    if status == 0:
        count = len(list(state_dir().glob("built.*")))
        (state_dir() / f"built.{count}").write_text("x")
        lib = tree / "sim" / "quad_fc" / "libmarv_sil_quad_fc.so"
        lib.parent.mkdir(parents=True, exist_ok=True)
        lib.write_text(f"fake SIL library, build {count}\n")
        (tree / ".fake_built").write_text("x")
    return status


def main(argv: list[str]) -> int:
    if argv[:1] == ["--install"]:
        install(Path(argv[1]))
        return 0
    tool, args = argv[0], argv[1:]
    with open(state_dir() / "argv.jsonl", "a") as f:
        f.write(json.dumps([tool, *args]) + "\n")
    if tool == "cmake" and "--preset" in args:
        tree = Path(args[args.index("-B") + 1])
        tree.mkdir(parents=True, exist_ok=True)
        (tree / "build.ninja").write_text("# fake\n")
        print(f"-- Configuring done ({args[args.index('--preset') + 1]})", flush=True)
        return 0
    if tool == "cmake" and "--build" in args:
        return build(Path(args[args.index("--build") + 1]))
    if tool == "ninja" and "-n" in args:
        tree = Path(args[args.index("-C") + 1])
        if (tree / ".fake_built").exists():
            print("ninja: no work to do.")
        else:
            n = steps()
            for k in range(1, n + 1):
                print(f"[{k}/{n}] Building CXX object fake_{k}.o")
        return 0
    print(f"fake {tool}: unsupported arguments {args}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
