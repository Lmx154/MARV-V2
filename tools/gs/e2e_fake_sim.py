#!/usr/bin/env python3
"""The fake simulator of the GCS end-to-end test (ground-segment unit GX-8; tools/gs/e2e.sh). A test tool, never flown.

It stands where the lockstep plugin stands for the sim channel: it sends version 1 records (sim/gz/plugin/src/sim_telemetry.hpp, packed
with the layout of tools/sim/sim_channel.py) as UDP datagrams to 127.0.0.1:<port>, where the helper listens, at --rate-hz. The values
are a slow harness motion, not a model: the sim time advances with the wall clock (real-time factor 1), the vehicle hovers and swings
in roll, the rotors and DShot swing about a base. --hil sets the HIL flag and fills the HIL block.

  uv run python tools/gs/e2e_fake_sim.py --port N [--rate-hz R] [--hil]
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import sim_channel as sc  # noqa: E402

NS_PER_S = 1_000_000_000
SWING_RAD, SWING_PERIOD_S = 0.3, 6.0  # harness values of the fake motion: the roll swing and its period
HOVER_HEIGHT_M = 1.0
ROTOR_BASE_RAD_S, ROTOR_SWING_RAD_S = 1000.0, 100.0
DSHOT_BASE, DSHOT_SWING = 1000, 200
FLAG_IMU_MODEL, FLAG_LAUNCH_PAD, FLAG_GROUND, FLAG_HIL, FLAG_PAD_RELEASED = 1, 2, 4, 8, 32
ARMED_STATE = 10  # fw/arming ArmState::Armed
MODE_ANGLE = 2
G = 9.81


def record(seq: int, t_s: float, hil: bool) -> bytes:
    roll = SWING_RAD * math.sin(2 * math.pi * t_s / SWING_PERIOD_S)
    q = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    swing = math.sin(2 * math.pi * t_s / SWING_PERIOD_S)
    flags = FLAG_IMU_MODEL | FLAG_LAUNCH_PAD | FLAG_GROUND | FLAG_PAD_RELEASED | (FLAG_HIL if hil else 0)
    header = sc.HEADER.pack(sc.MAGIC, sc.VERSION, sc.RECORD_BYTES, seq, flags, int(t_s * NS_PER_S), seq, 1.0)
    truth = sc.TRUTH.pack(0.0, 0.0, -HOVER_HEIGHT_M, 0.0, 0.0, 0.0, *q, SWING_RAD * 2 * math.pi / SWING_PERIOD_S * math.cos(2 * math.pi * t_s / SWING_PERIOD_S), 0.0, 0.0,
                          0.0, 0.0, -G, *([ROTOR_BASE_RAD_S + ROTOR_SWING_RAD_S * swing] * 4))
    dshot = [DSHOT_BASE + round(DSHOT_SWING * swing)] * 4
    injected = sc.INJECTED.pack(0.0, 0.0, 0.0, 0.0, 0.0, -G, 0x1C0, *dshot, *dshot)
    ground = sc.GROUND.pack(0, 0.0)
    if hil:
        block = sc.HIL.pack(ARMED_STATE, 1, seq, seq, MODE_ANGLE, 0, 0, seq, 0, 0.0, 0.0, 0.0, 0.5, *([0.0] * 16), 1.0, 900.0, 2400.0, 0.0)
    else:
        block = sc.HIL.pack(*([0] * 9), *([0.0] * 4), *([0.0] * 16), *([0.0] * 4))
    return header + truth + injected + ground + block


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, required=True, help="the helper's sim channel port on 127.0.0.1")
    ap.add_argument("--rate-hz", type=float, default=sc.DEFAULT_RATE_HZ)
    ap.add_argument("--hil", action="store_true")
    a = ap.parse_args(argv)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / a.rate_hz
    start = time.monotonic()
    due = start
    seq = 0
    print(f"fake sim on udp 127.0.0.1:{a.port} at {a.rate_hz} Hz", flush=True)
    while True:
        wait = due - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        # A process that was stopped (SIGSTOP) resumes without a burst of catch-up records: the next one is due a period from now.
        sock.sendto(record(seq, time.monotonic() - start, a.hil), ("127.0.0.1", a.port))
        seq += 1
        due = max(due + period, time.monotonic())


if __name__ == "__main__":
    sys.exit(main())
