#!/usr/bin/env python3
"""Cross-check of the C++ logger against the Python decoder (ctest log_roundtrip_decode).

tests/regression/quad/L09/unit/log_logger_test.cpp (LogRoundTrip) runs a synthetic tick stream through the ring, the logger and
a fake card, and with MARV_LOG_ROUNDTRIP_DIR set writes roundtrip.mlog (the file as closed) and roundtrip_expected.json (every
record the ring accepted, as integers, floats as their IEEE 754 bit patterns, and the ring's drop counts). This script decodes
the file with tools/log/mlog.py and requires the decoded records to equal the accepted ones exactly, in order, with every loss
announced by Gap records that do not exceed the drops. It shares no code with the C++ writer, so a layout disagreement fails.

Usage: roundtrip_check.py <dir>   Exit status: 0 equal, 1 differs, 2 usage or missing input.
       roundtrip_check.py --imu-raw <dir>   the format-version-2 check (tests/regression/quad/L09/unit/log_imu_raw_test.cpp,
       LogImuRawRoundTrip, writes imu_raw.mlog): each sample is in the file twice, as the 32-byte imu record and, 1 us later, as the
       16-byte raw-count record; the decoded pairs must have the same flags and the same binary32 bit patterns.
       roundtrip_check.py --flight <dir>   the flight image's wiring (tests/regression/quad/L09/unit/log_flight_test.cpp,
       FlightLogSession, writes flight.mlog and flight_expected.json: every record the tick hooks pushed and the session's times):
       the file decodes without a problem and ends with an End record of reason disarm; its imu, rate_tick and manual_control
       records are each one unbroken slice of the pushed stream that starts before the arm edge (the pre-roll) and runs to the
       end of the post-roll; every sample that is a whole number of counts is a raw-count record and every other one a 32-byte
       record; the header carries the scales, the build flags and the Telemetry fields, and every Telemetry record decodes to the
       snapshot the test made for its sequence number.
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


def check_imu_raw(directory: Path) -> list[str]:
    dec = mlog.decode((directory / "imu_raw.mlog").read_bytes())
    errors = [f"decoder problem: {p}" for p in dec.problems]
    if not dec.ended:
        errors.append("no End record")
    if "imu_raw" not in dec.header:
        errors.append("the header has no header_imu_raw record")
    rows = mlog.records_of(dec, "imu")
    if len(rows) < 2 or len(rows) % 2:
        return errors + [f"{len(rows)} imu rows, expected the v1 and v2 record of every sample"]
    keys = ("gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")
    differing = 0
    for v1, v2 in zip(rows[0::2], rows[1::2]):
        if v1["flags"] != v2["flags"] or [bits(v1[k]) for k in keys] != [bits(v2[k]) for k in keys] or v2["t_us"] != v1["t_us"] + 1:
            differing += 1
            if differing == 1:
                errors.append(f"first difference at t_us {v1['t_us']}: {v1} / {v2}")
    if differing:
        errors.append(f"{differing} of {len(rows) // 2} samples differ between the 32-byte and the 16-byte record")
    if [r["t_us"] for r in rows] != sorted(r["t_us"] for r in rows):
        errors.append("imu stamps not monotone")
    return errors


def f32_bits_of(x: float) -> int:
    return struct.unpack("<I", struct.pack("<f", x))[0]


def unbroken_slice(name: str, got: list, want: list, errors: list[str]) -> int | None:
    """The index in `want` where `got` starts as one unbroken slice of it, or None (with the reason in errors)."""
    if not got:
        errors.append(f"no {name} record")
        return None
    try:
        i = want.index(got[0])
    except ValueError:
        errors.append(f"the first {name} record {got[0]} was never pushed")
        return None
    if want[i: i + len(got)] != got:
        k = next(j for j, (a, b) in enumerate(zip(got, want[i:])) if a != b) if len(want) - i >= len(got) else len(want) - i
        errors.append(f"the {name} records are not an unbroken slice of the pushed stream (first difference after {k} records)")
        return None
    return i


def check_flight(directory: Path) -> list[str]:
    data = (directory / "flight.mlog").read_bytes()
    exp = json.loads((directory / "flight_expected.json").read_text())
    kinds: dict[int, int] = {}
    emit = mlog._Stream.emit

    def counting_emit(self, rec: bytes) -> None:  # the decoded rows do not tell the two IMU kinds apart; the raw records do
        kinds[rec[0]] = kinds.get(rec[0], 0) + 1
        emit(self, rec)

    mlog._Stream.emit = counting_emit
    try:
        dec = mlog.decode(data)
    finally:
        mlog._Stream.emit = emit
    errors = [f"decoder problem: {p}" for p in dec.problems]
    ends = mlog.records_of(dec, "end")
    if not dec.ended or not ends or ends[-1]["reason"] != "disarm":
        errors.append(f"no End record of reason disarm ({ends[-1]['reason'] if ends else 'none'})")
    if dec.session != exp["session"]:
        errors.append(f"session {dec.session} != {exp['session']}")
    info = dec.header.get("info", {})
    if info.get("build_flags") != exp["build_flags"]:
        errors.append(f"build_flags {info.get('build_flags')} != {exp['build_flags']}")
    raw = dec.header.get("imu_raw", {})
    sc = exp["imu_raw"]
    if (f32_bits_of(raw.get("gyro_rad_s_per_lsb", 0.0)), f32_bits_of(raw.get("accel_m_s2_per_lsb", 0.0)), raw.get("sign")) != (
            sc["gyro_rad_s_per_lsb"], sc["accel_m_s2_per_lsb"], sc["sign"]):
        errors.append(f"header_imu_raw {raw} is not the scales the hooks used")
    tel = dec.header.get("telemetry", {})
    if tel != {"version": 1, "field_count": exp["telemetry_fields"]}:
        errors.append(f"header_telemetry {tel}, expected version 1 with {exp['telemetry_fields']} fields")
    if not dec.header.get("params"):
        errors.append("header has no parameter table")

    keys = ("gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")
    imu = [[r["t_us"], r["flags"]] + [bits(r[k]) for k in keys] for r in mlog.records_of(dec, "imu")]
    want_imu = [row[:-1] for row in exp["imu"]]
    start = unbroken_slice("imu", imu, want_imu, errors)
    if start is not None:
        logged = exp["imu"][start: start + len(imu)]
        whole = sum(row[-1] for row in logged)
        if (kinds.get(mlog.K_IMU_RAW, 0), kinds.get(mlog.K_IMU, 0)) != (whole, len(logged) - whole):
            errors.append(f"{kinds.get(mlog.K_IMU_RAW, 0)} raw-count and {kinds.get(mlog.K_IMU, 0)} 32-byte imu records, expected "
                          f"{whole} and {len(logged) - whole}")
        if not imu[0][0] < exp["arm_us"]:
            errors.append(f"the first imu record ({imu[0][0]} us) is not before the arm edge ({exp['arm_us']} us): no pre-roll")
        if imu[-1][0] < exp["disarm_us"] + exp["post_roll_us"]:
            errors.append(f"the last imu record ({imu[-1][0]} us) is before the end of the post-roll")
    rate = [[r["now_lo_us"], r["arm_state"]] + [r[f"dshot_{i}"] for i in range(1, 5)] for r in mlog.records_of(dec, "rate_tick")]
    want_rate = [[t & 0xFFFFFFFF, *rest] for t, *rest in exp["rate_tick"]]
    if unbroken_slice("rate_tick", rate, want_rate, errors) is not None:
        armed = [r for r in mlog.records_of(dec, "rate_tick") if r["arm_state"] == exp["armed_code"]]
        if not armed:
            errors.append("no rate_tick record carries the armed state")
    man = [[r["frames"], r["t_us"], bits(r["roll"]), bits(r["ch_16"])] for r in mlog.records_of(dec, "manual_control")]
    unbroken_slice("manual_control", man, exp["manual_control"], errors)

    rows = mlog.records_of(dec, "telemetry")
    if not rows:
        errors.append("no telemetry record")
    seqs = [r.get("sequence") for r in rows]
    if seqs != sorted(set(seqs)):
        errors.append("telemetry sequence numbers do not increase")
    for r in rows:
        k = r.get("sequence", 0)
        want = {"version": 1, "time_us": r.get("t_us"), "rc_frames": 3 * k, "gyro_rad_s_x": f32(0.5 * k),
                "q_setpoint_z": f32(-0.125 * k), "rc_channels_16": f32(-0.25 * k), "motor_dshot_4": k % 2048,
                "arm_state": exp["armed_code"], "motor_gate": 1, "reserved": 0, "q_mahony_w": 0.0}
        bad = {name: (r.get(name), v) for name, v in want.items() if r.get(name) != v}
        if bad:
            errors.append(f"telemetry record of sequence {k} decodes to {bad} (decoded, expected)")
            break
    if len(rows) > 1:
        span = rows[-1]["t_us"] - rows[0]["t_us"]
        if not span / exp["slow_period_us"] * 0.8 <= len(rows) - 1 <= span / exp["slow_period_us"]:
            errors.append(f"{len(rows)} telemetry records over {span} us at a {exp['slow_period_us']} us period")
    return errors


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def main(argv: list[str]) -> int:
    if len(argv) == 3 and argv[1] == "--flight":
        d = Path(argv[2])
        if not (d / "flight.mlog").is_file() or not (d / "flight_expected.json").is_file():
            print(f"roundtrip_check: no flight.mlog / flight_expected.json in {d}", file=sys.stderr)
            return 2
        errors = check_flight(d)
        for e in errors:
            print("FAIL", e)
        if not errors:
            print("roundtrip_check: the flight session decodes clean and equals what the hooks pushed")
        return 1 if errors else 0
    if len(argv) == 3 and argv[1] == "--imu-raw":
        d = Path(argv[2])
        if not (d / "imu_raw.mlog").is_file():
            print(f"roundtrip_check: no imu_raw.mlog in {d}", file=sys.stderr)
            return 2
        errors = check_imu_raw(d)
        for e in errors:
            print("FAIL", e)
        if not errors:
            print("roundtrip_check: the 16-byte records decode to the 32-byte records' values, bit for bit")
        return 1 if errors else 0
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
