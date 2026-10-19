#!/usr/bin/env python3
"""Cross-check of the C++ logger against the Python decoder (ctest log_roundtrip_decode).

tests/regression/quad/L09/unit/log_logger_test.cpp (LogRoundTrip) runs a synthetic tick stream through the ring, the logger and
a fake card, and with MARV_LOG_ROUNDTRIP_DIR set writes roundtrip.mlog (the file as closed) and roundtrip_expected.json (every
record the ring accepted, as integers, floats as their IEEE 754 bit patterns, and the ring's drop counts). This script decodes
the file with tools/log/mlog.py and requires the decoded records to equal the accepted ones exactly, in order, with every loss
announced by Gap records that do not exceed the drops. It shares no code with the C++ writer, so a layout disagreement fails.

Usage: roundtrip_check.py <dir>   Exit status: 0 equal, 1 differs, 2 usage or missing input.
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mlog  # noqa: E402


def bits(x: float) -> int:
    return struct.unpack("<I", struct.pack("<f", x))[0]


def check(directory: Path) -> list[str]:
    data = (directory / "roundtrip.mlog").read_bytes()
    exp = json.loads((directory / "roundtrip_expected.json").read_text())
    dec = mlog.decode(data)
    errors: list[str] = []
    if dec.problems:
        errors += [f"decoder problem: {p}" for p in dec.problems]
    if not dec.ended:
        errors.append("no End record")
    if dec.session != exp["session"]:
        errors.append(f"session {dec.session} != {exp['session']}")
    if not dec.header.get("params"):
        errors.append("header has no parameter table")

    imu = [[r["t_us"], r["flags"]] + [bits(r[k]) for k in ("gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")]
           for r in mlog.records_of(dec, "imu")]
    if imu != exp["imu"]:
        errors.append(f"imu records differ: decoded {len(imu)}, expected {len(exp['imu'])}; first difference at "
                      f"{next((i for i, (a, b) in enumerate(zip(imu, exp['imu'])) if a != b), min(len(imu), len(exp['imu'])))}")
    rate = [[r["now_lo_us"], r["arm_state"], r["gate_state"], r["dshot_1"], r["dshot_2"], r["dshot_3"], r["dshot_4"]]
            for r in mlog.records_of(dec, "rate_tick")]
    want_rate = [[t & 0xFFFFFFFF] + rest for t, *rest in exp["rate_tick"]]
    if rate != want_rate:
        errors.append(f"rate_tick records differ: decoded {len(rate)}, expected {len(want_rate)}")
    man = [[r["frames"], r["t_us"], bits(r["roll"]), bits(r["throttle"]), bits(r["ch_01"]), bits(r["ch_16"]), bits(r["link_quality"])]
           for r in mlog.records_of(dec, "manual_control")]
    if man != exp["manual_control"]:
        errors.append(f"manual_control records differ: decoded {len(man)}, expected {len(exp['manual_control'])}")

    announced = [0, 0, 0]
    for g in mlog.records_of(dec, "gap"):
        announced[0] += g["dropped_imu"]
        announced[1] += g["dropped_rate_tick"]
        announced[2] += g["dropped_manual_control"]
    for i, name in enumerate(("imu", "rate_tick", "manual_control")):
        if announced[i] > exp["dropped"][i]:
            errors.append(f"gap records announce {announced[i]} {name} drops, the ring counted {exp['dropped'][i]}")
    if exp["dropped"][0] and not announced[0]:
        errors.append("the ring dropped imu records but no Gap record announces it")
    # The stamps jump across every gap (monotone unwrapped time).
    t_prev = None
    for r in mlog.records_of(dec, "imu"):
        if t_prev is not None and r["t_us"] <= t_prev:
            errors.append(f"imu stamps not strictly increasing at {r['t_us']}")
            break
        t_prev = r["t_us"]
    return errors


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    d = Path(argv[1])
    if not (d / "roundtrip.mlog").is_file() or not (d / "roundtrip_expected.json").is_file():
        print(f"roundtrip_check: no roundtrip.mlog / roundtrip_expected.json in {d}", file=sys.stderr)
        return 2
    errors = check(d)
    for e in errors:
        print("FAIL", e)
    if not errors:
        print("roundtrip_check: the decoded stream equals the accepted input")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
