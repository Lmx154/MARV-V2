#!/usr/bin/env python3
"""Reader of the binary log of the Gazebo lockstep plugin (sim/gz/plugin/src/lockstep_log.hpp is the format's home).

  lockstep_log.py <log>          prints the header, the record counts and the trailer

Little-endian, no padding. A header, then records of one type byte and a fixed-size payload. Per host step: one STEP
record, m TICK records, one APPLIED record. A TRAILER record is the last record of a run that ended cleanly.

  header   magic b"MARVLOCK"; u32 version (1); u32 header_size; u32 m; u32 tick_period_num_us; u32 tick_period_den;
           u64 seed; u32 n, then n bytes of the gz-sim version string
  STEP 1   u64 iteration; u64 sim_time_ns; 13 doubles gz (pos_enu 3, q_flu_to_enu wxyz 4, lin_vel_enu 3, ang_vel_enu 3);
           13 doubles plant body (pos_ned 3, vel_ned 3, q wxyz 4, omega_frd 3)
  TICK 2   u64 tick; u64 sil_t_us; 32 bytes marv_imu_meas; 4 x u16 dshot (motor 1..4); u32 erpm_valid; u32 0;
           20 doubles (force_ned 3, torque_ned 3, rotor_speed 4, erpm 4, wrench force_enu 3, wrench torque_enu 3)
  APPLIED 3 6 doubles: applied wrench W_bar in ENU (force 3, torque 3)
  TRAILER 4 u64 step records, u64 tick records, u64 applied records
  TRUTH 5   only when the plugin has <attitude_source>truth</attitude_source>, directly after that tick's TICK record:
            u64 tick; the 48 bytes of marv_truth_state (u32 struct_size, u32 flags, u64 tick, 4 x f32 q wxyz, 3 x f32
            omega_frd, 4 bytes of padding); the trailer's counts do not include it

read() returns {"header": dict, "steps": [dict], "ticks": [dict], "applied": [dict], "truths": [dict],
"trailer": dict or None, "raw_records": bytes (everything after the header, for byte comparison)}. A file that is cut inside a record, has an
unknown record type, a record after the trailer or a TRUTH record that does not directly follow the TICK record of
its tick raises LogError. Doubles are Python floats (exact binary64); no
numpy.
"""

from __future__ import annotations

import struct
import sys

MAGIC = b"MARVLOCK"
VERSION = 1
STEP, TICK, APPLIED, TRAILER, TRUTH = 1, 2, 3, 4, 5
_STEP = struct.Struct("<QQ26d")
_TICK = struct.Struct("<QQ32s4HII20d")
_APPLIED = struct.Struct("<6d")
_TRAILER = struct.Struct("<QQQ")
_TRUTH = struct.Struct("<QIIQ4f3f4x")
_SIZES = {STEP: _STEP.size, TICK: _TICK.size, APPLIED: _APPLIED.size, TRAILER: _TRAILER.size, TRUTH: _TRUTH.size}


class LogError(Exception):
    pass


def _header(data: bytes):
    fixed = struct.Struct("<8sIIIIIQI")
    if len(data) < fixed.size:
        raise LogError("file shorter than the header")
    magic, version, header_size, m, num, den, seed, n = fixed.unpack_from(data)
    if magic != MAGIC:
        raise LogError(f"bad magic {magic!r}")
    if version != VERSION:
        raise LogError(f"unsupported version {version}")
    if header_size != fixed.size + n or len(data) < header_size:
        raise LogError("inconsistent header size")
    return {
        "version": version, "header_size": header_size, "m": m, "tick_period_num_us": num, "tick_period_den": den,
        "seed": seed, "gz_sim_version": data[fixed.size:header_size].decode("ascii"),
    }


def read(path):
    with open(path, "rb") as f:
        data = f.read()
    header = _header(data)
    pos = header["header_size"]
    steps, ticks, applied, truths, trailer = [], [], [], [], None
    last_kind = None
    while pos < len(data):
        if trailer is not None:
            raise LogError("record after the trailer")
        kind = data[pos]
        if kind not in _SIZES:
            raise LogError(f"unknown record type {kind} at byte {pos}")
        size = _SIZES[kind]
        if pos + 1 + size > len(data):
            raise LogError(f"record type {kind} cut at byte {pos}")
        if kind == STEP:
            v = _STEP.unpack_from(data, pos + 1)
            steps.append({"iteration": v[0], "sim_time_ns": v[1], "gz_pos_enu": v[2:5], "gz_q_wxyz": v[5:9],
                          "gz_lin_vel_enu": v[9:12], "gz_ang_vel_enu": v[12:15], "body_pos_ned": v[15:18],
                          "body_vel_ned": v[18:21], "body_q_wxyz": v[21:25], "body_omega_frd": v[25:28]})
        elif kind == TICK:
            v = _TICK.unpack_from(data, pos + 1)
            ticks.append({"tick": v[0], "sil_t_us": v[1], "imu": v[2], "dshot": v[3:7], "erpm_valid": v[7],
                          "force_ned": v[9:12], "torque_ned": v[12:15], "rotor_speed": v[15:19], "erpm": v[19:23],
                          "wrench_force_enu": v[23:26], "wrench_torque_enu": v[26:29]})
        elif kind == TRUTH:
            v = _TRUTH.unpack_from(data, pos + 1)
            if last_kind != TICK or v[0] != ticks[-1]["tick"]:
                raise LogError(f"TRUTH record of tick {v[0]} at byte {pos} does not follow that tick's TICK record")
            truths.append({"tick": v[0], "struct_size": v[1], "flags": v[2], "state_tick": v[3], "q_wxyz": v[4:8],
                           "omega_frd": v[8:11], "raw": data[pos + 9:pos + 1 + size]})
        elif kind == APPLIED:
            v = _APPLIED.unpack_from(data, pos + 1)
            applied.append({"force_enu": v[0:3], "torque_enu": v[3:6]})
        else:
            v = _TRAILER.unpack_from(data, pos + 1)
            trailer = {"steps": v[0], "ticks": v[1], "applied": v[2]}
        last_kind = kind
        pos += 1 + size
    return {"header": header, "steps": steps, "ticks": ticks, "applied": applied, "truths": truths, "trailer": trailer,
            "raw_records": data[header["header_size"]:]}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__.split("\n")[0], file=sys.stderr)
        return 2
    try:
        log = read(argv[0])
    except LogError as e:
        print(f"lockstep_log: {e}", file=sys.stderr)
        return 1
    print("header", log["header"])
    print("steps", len(log["steps"]), "ticks", len(log["ticks"]), "applied", len(log["applied"]))
    print("trailer", log["trailer"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
