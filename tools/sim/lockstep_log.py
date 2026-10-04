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
  SENSORS 6 only with a sensor element of the plugin (decision 0019), once, the first record after the header: u8
            gyro_source (0 none, 1 truth, 2 model), u8 rotor_speed, u8 clock, i8 clock_corner, f64 odr_error, u64
            host_step_ns, f64 tick_s, u64 seed, u32 imu_stream_id, u64 imu_counter_base; the IMU config: u32
            latency_samples, gyro and accel each 4 f64 (noise_density, bias_instability, lsb, full_scale) and 3 f64
            turn_on_bias, 2 f64 turn-on bias bounds (gyro, accel), 6 i8 signs, 32 bytes profile SHA-256; the rotor-speed
            config: 4 u32 (pole_count, latency_ticks, exponent_bits, mantissa_bits), f64 period_unit_s
  ROTOR 7   only with <rotor_speed_model>, after that tick's TICK record (and its TRUTH record, if any): u64 tick; the 20
            bytes of marv_rotor_speed_meas (4 x f32 omega, u32 flags); the trailer's counts do not include it

read() returns {"header": dict, "steps": [dict], "ticks": [dict], "applied": [dict], "truths": [dict],
"trailer": dict or None, "raw_records": bytes (everything after the header, for byte comparison), "sensors": dict or None,
"rotors": [dict]}. A file that is cut inside a record, has an
unknown record type, a record after the trailer or a TRUTH record that does not directly follow the TICK record of
its tick raises LogError, and so does a SENSORS record that is not the first record or a ROTOR record that does not follow
its tick's TICK (or TRUTH) record. Doubles are Python floats (exact binary64); no
numpy.
"""

from __future__ import annotations

import struct
import sys

MAGIC = b"MARVLOCK"
VERSION = 1
STEP, TICK, APPLIED, TRAILER, TRUTH, SENSORS, ROTOR = 1, 2, 3, 4, 5, 6, 7
_STEP = struct.Struct("<QQ26d")
_TICK = struct.Struct("<QQ32s4HII20d")
_APPLIED = struct.Struct("<6d")
_TRAILER = struct.Struct("<QQQ")
_TRUTH = struct.Struct("<QIIQ4f3f4x")
_SENSORS = struct.Struct("<BBBbdQdQIQI7d7d2d6b32s4Id")
_ROTOR = struct.Struct("<Q4fI")
_SIZES = {STEP: _STEP.size, TICK: _TICK.size, APPLIED: _APPLIED.size, TRAILER: _TRAILER.size, TRUTH: _TRUTH.size,
          SENSORS: _SENSORS.size, ROTOR: _ROTOR.size}
GYRO_SOURCES = ("none", "truth", "model")


def _imu_axis(v):
    return {"noise_density": v[0], "bias_instability": v[1], "lsb": v[2], "full_scale": v[3], "turn_on_bias": v[4:7]}


def _sensors(v, raw):
    if v[0] >= len(GYRO_SOURCES):
        raise LogError(f"SENSORS record has an unknown gyro_source {v[0]}")
    return {"gyro_source": GYRO_SOURCES[v[0]], "rotor_speed": v[1], "clock": v[2], "clock_corner": v[3],
            "odr_error": v[4], "host_step_ns": v[5], "tick_s": v[6], "seed": v[7], "imu_stream_id": v[8],
            "imu_counter_base": v[9],
            "imu": {"latency_samples": v[10], "gyro": _imu_axis(v[11:18]), "accel": _imu_axis(v[18:25]),
                    "gyro_turn_on_bias_bound": v[25], "accel_turn_on_bias_bound": v[26], "turn_on_bias_signs": v[27:33],
                    "profile_sha256": v[33].hex()},
            "rotor": {"pole_count": v[34], "latency_ticks": v[35], "exponent_bits": v[36], "mantissa_bits": v[37],
                      "period_unit_s": v[38]},
            "raw": raw}


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
    sensors, rotors = None, []
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
        elif kind == SENSORS:
            if last_kind is not None:
                raise LogError(f"SENSORS record at byte {pos} is not the first record")
            sensors = _sensors(_SENSORS.unpack_from(data, pos + 1), data[pos + 1:pos + 1 + size])
        elif kind == ROTOR:
            v = _ROTOR.unpack_from(data, pos + 1)
            if last_kind not in (TICK, TRUTH) or v[0] != ticks[-1]["tick"]:
                raise LogError(f"ROTOR record of tick {v[0]} at byte {pos} does not follow that tick's TICK record")
            rotors.append({"tick": v[0], "omega": v[1:5], "flags": v[5], "raw": data[pos + 9:pos + 1 + size]})
        elif kind == APPLIED:
            v = _APPLIED.unpack_from(data, pos + 1)
            applied.append({"force_enu": v[0:3], "torque_enu": v[3:6]})
        else:
            v = _TRAILER.unpack_from(data, pos + 1)
            trailer = {"steps": v[0], "ticks": v[1], "applied": v[2]}
        last_kind = kind
        pos += 1 + size
    return {"header": header, "steps": steps, "ticks": ticks, "applied": applied, "truths": truths, "trailer": trailer,
            "raw_records": data[header["header_size"]:], "sensors": sensors, "rotors": rotors}


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
