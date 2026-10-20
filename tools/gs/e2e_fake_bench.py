#!/usr/bin/env python3
"""The fake bench of the GCS end-to-end test (ground-segment unit GX-10; tools/gs/e2e.sh light mode). A test tool, never flown.

It stands where the debug probe, the board running the HIL image and tools/target/flash.sh stand for the Bench page, under --root:
  <root>/sys                 a sysfs with a Debug Probe (USB 2e8a:000c, serial PROBE_SERIAL) and a board (2e8a:0009) whose interface 0
                             reads "Board CDC" (the HIL image's single CDC; tools/gs/hil.py)
  <byid-dir>/<board port>    a symlink to a pty standing for the board's serial port; the process holds the pty open and never reads it
  <build-root>/m33-hil       a build tree stand-in (CMake cache with the HIL options, an ELF of a few fixed bytes) for the HIL kind
  <root>/flash.sh            a stand-in for tools/target/flash.sh: it records its arguments in <root>/flash.record, prints the lines the
                             real one prints on success and exits 0
Nothing here opens a real port or touches a board. It prints "fake bench ready" once everything is in place and then waits to be killed.

  uv run python tools/gs/e2e_fake_bench.py --root DIR --byid-dir DIR --build-root DIR
"""

from __future__ import annotations

import argparse
import os
import stat
import time
from pathlib import Path

PROBE_SERIAL = "E2EPROBE0001"  # the helper is started with MARV_PROBE_SERIAL set to it (tools/gs/e2e.sh)
BOARD_SERIAL = "E2EBOARD0001"
PORT_NAME = f"usb-Raspberry_Pi_Pico_{BOARD_SERIAL}-if00"


def device(root: Path, name: str, vendor: str, product: str, serial: str) -> Path:
    d = root / "sys" / "bus" / "usb" / "devices" / name
    d.mkdir(parents=True, exist_ok=True)
    for file, text in (("idVendor", vendor), ("idProduct", product), ("serial", serial)):
        (d / file).write_text(text + "\n")
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--byid-dir", type=Path, required=True)
    ap.add_argument("--build-root", type=Path, required=True)
    a = ap.parse_args()
    root = a.root
    device(root, "1-3.3", "2e8a", "000c", PROBE_SERIAL)
    board = device(root, "1-1", "2e8a", "0009", BOARD_SERIAL)
    master, slave = os.openpty()
    tty = os.ttyname(slave)
    interface = board / "1-1:1.0"
    interface.mkdir()
    (interface / "interface").write_text("Board CDC\n")
    tty_dir = root / "sys" / "class" / "tty" / os.path.basename(tty)
    tty_dir.mkdir(parents=True)
    (tty_dir / "device").symlink_to(interface)
    a.byid_dir.mkdir(parents=True, exist_ok=True)
    (a.byid_dir / PORT_NAME).symlink_to(tty)

    tree = a.build_root / "m33-hil"
    (tree / "fw" / "hal" / "target").mkdir(parents=True, exist_ok=True)
    (tree / "CMakeCache.txt").write_text("MARV_FC3V2_HIL:BOOL=ON\nMARV_FC3V2_HIL_RECEIVER:BOOL=ON\n")
    (tree / "fw" / "hal" / "target" / "marv_fc3v2.elf").write_bytes(b"\x7fELF-e2e-fake-hil-image")

    script = root / "flash.sh"
    script.write_text("#!/usr/bin/env bash\n"
                      f'echo "$@" > {root}/flash.record\n'
                      'echo "flash: image $1"\n'
                      'echo "** Verified OK **"\n')
    script.chmod(script.stat().st_mode | stat.S_IXUSR)
    print("fake bench ready", flush=True)
    try:
        while True:
            time.sleep(3600)  # a labelled harness value: only keeps the pty open until the script kills this process
    finally:
        os.close(master)
        os.close(slave)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
