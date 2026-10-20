#!/usr/bin/env python3
"""The fake joystick of the GCS end-to-end test (ground-segment unit GX-9; tools/gs/e2e.sh). A test tool, never flown.

It stands where the kernel's joydev stands for the helper's pilot bridge (tools/gs/pilot.py): a FIFO named js<N> in --dir that carries
struct js_event records. At start it writes the INIT burst of 8 axes (axis 4, channel 5 under the default layout, at +32767: the arm
switch high; the others centred) and then sweeps axis 0 (channel 1) as a triangle wave at --rate-hz, so that the channels the GCS shows
move.

  uv run python tools/gs/e2e_fake_joystick.py --dir DIR [--name js0] [--rate-hz R]
"""

from __future__ import annotations

import argparse
import errno
import os
import struct
import sys
import time
from pathlib import Path

JS_EVENT = struct.Struct("<IhBB")
AXIS, INIT = 0x02, 0x80
AXES = 8
ARM_AXIS = 4
FULL = 32767
SWEEP_PERIOD_S = 4.0  # harness value: the triangle wave's period


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", type=Path, required=True)
    ap.add_argument("--name", default="js0")
    ap.add_argument("--rate-hz", type=float, default=50.0)
    a = ap.parse_args(argv)
    a.dir.mkdir(parents=True, exist_ok=True)
    path = a.dir / a.name
    os.mkfifo(path)
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)  # a writer that is also a reader: the FIFO never reports end of file
    start = time.monotonic()

    def send(number: int, value: int, init: bool = False) -> None:
        stamp = int((time.monotonic() - start) * 1000) & 0xFFFFFFFF
        try:
            os.write(fd, JS_EVENT.pack(stamp, value, AXIS | (INIT if init else 0), number))
        except OSError as e:
            if e.errno != errno.EAGAIN:  # nobody reads: the buffer is full, the event is dropped
                raise

    for n in range(AXES):
        send(n, FULL if n == ARM_AXIS else 0, init=True)
    print(f"fake joystick {path}", flush=True)
    while True:
        t = (time.monotonic() - start) % SWEEP_PERIOD_S / SWEEP_PERIOD_S
        send(0, round(FULL * (4 * abs(t - 0.5) - 1)))
        time.sleep(1.0 / a.rate_hz)


if __name__ == "__main__":
    sys.exit(main())
