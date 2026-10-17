#!/usr/bin/env python3
"""FC3v2 bench HIL open-loop comparison (HIL stage S3b, decision 0046 stage S3). Bench only; never run by CI.

A recorded SIL sample stream replayed to the board gives the SIL's DShot outputs within float tolerance. Three steps:

  python3 tools/target/hil_compare.py reference REF.npz [--scenario sticks|motion] [--lib LIB.so] [--params param_defaults.cpp]
      runs the host SIL library (build/host-debug/sim/quad_fc/libmarv_sil_quad_fc.so) at the fc3v2 tick on a deterministic
      scenario and writes everything the board needs to repeat it and the SIL's DShot per tick to REF.npz; prints the
      scenario and, per phase, the DShot ranges of the armed rate ticks. Exit 0 only if the reference armed and disarmed
      where the scenario says.
  python3 tools/target/hil_compare.py replay /dev/ttyACM0 REF.npz OUT.bin [--batch 4]
      opens the HIL board (MARV_FC3V2_HIL image, DTR asserted), reads the hello, refuses if the schema hash, parameter count,
      values CRC or tick period differ from REF's, then streams REF's samples (--batch per record, each record sent when its
      last sample's stamp is due on the host clock, as hil_fc.py) with each ManualControl frame just before the record that
      starts at the frame's tick, waits for the drain, writes the uplink bytes as received to OUT.bin and the run's counts to
      OUT.bin.sent.json. The board must be freshly reset for each replay (see below).
  python3 tools/target/hil_compare.py compare REF.npz OUT.bin [--tolerance 1]
      per rate tick the board's DShot against REF's: exact matches, the largest |difference| in DShot counts, the first
      mismatch with context; the tick of every arming transition on each side; the board's counters (timing faults, link
      faults, deadline misses) and the EMB-3 inputs. Verdict line "HIL MATCH" or "HIL NOT MATCH: <reasons>"; exit 0 only for
      HIL MATCH.
  python3 tools/target/hil_compare.py refdiff A.npz B.npz [--tolerance 1]
      the same comparison between two references (host builds): the tolerance experiment of the README, and a determinism
      check (the same library twice must give 0 counts).
  python3 tools/target/hil_compare.py selftest
      compare() on synthetic data (match, a 1-count difference, an arming-tick mismatch), the npz round trip, the values CRC.

A fresh board per replay. The image keeps the composition (gyro calibration, arming latch, an IMU deadline miss) and its
counters from boot; the stream ends with exactly one deadline miss (no record for D_HIL), which latches a disarm. A second
replay without a reset starts from that state and the compare refuses it (counters not those of one run).

The scenarios (stamps in seconds of sample time; the IMU stream is the same bytes on both sides, the sticks too):
  sticks: a still level IMU (gyro exactly 0, accel (0, 0, -g0) FRD, both valid bits). ManualControl frames every 60 ticks
          (about 125 Hz; a multiple of the record sizes 1..6) from tick 0. Open loop: the gyro never answers the setpoint, so
          the rate error and the integrator grow while a stick is held.
  motion: the same sticks; from 2.0 s the IMU is a smooth tilt motion: roll phi = A_ROLL e(t) sin(2 pi F_ROLL t), pitch
          theta = A_PITCH e(t) sin(2 pi F_PITCH t + PHASE), yaw 0, e(t) a smoothstep from 0 at 2.0 s to 1 at 2.5 s (after the
          boot gyro calibration, 1.25 s, which wants a still gyro). Gyro = the Euler-rate body rates of that motion, accel =
          the specific force of that attitude, so the estimator, the filters and the rate loop see a consistent stream.
Phases of both (arm switch, stick values): wait 0 s (switch low), idle 3.0 s (switch high, throttle 0), throttle 3.5 s
(throttle 0.4), roll 4.5 s (roll 0.5), pitch 5.5 s (pitch 0.5), yaw 6.5 s (yaw 0.5), centred 7.5 s (sticks 0, throttle 0.4),
disarm 8.0 s (switch low), end 8.5 s.

Tolerance (default 1 DShot count). Exact equality is not expected: the host library is built with -ffp-contract=off
(cmake/flags.cmake, host targets only) and the m33 build has no -ffp-contract flag, so GCC's C++ default (fast) fuses a*b+c
into VFMA.F32 on the M33 (RateGroupStep::filter, Mahony::step, composition::tick, the mixer), and the M33 links the Pico SDK's
sinf/cosf/tanf/hypotf where the host links glibc's. A fused operation rounds once instead of twice and a libm function may
differ by an ulp, so a float result differs by a few 2^-24 relative; the DShot value is a float scaled to 2047 counts and
rounded, so such an error moves the rounded count by one only when the value lies within that error of a rounding
boundary. The error does not grow (the filters are stable and open-loop terms are sums of bounded terms). Measured on the host
by building the SIL library with -ffp-contract=fast -mfma at -O3 (a proxy for the M33, not the M33): at most 1 count apart
over both scenarios, 1.4 % and 1.7 % of the values, arming transitions on the same ticks
(tests/regression/quad/L09/results/hil_s3b/README.md). Arming transitions are compared exactly, whatever the tolerance.

The reference's frame stamp is the stamp of the tick it is staged before (age 0 at that tick, growing to one frame period);
the board stamps a frame with its clock when read from USB (a few hundred us to a record's time of age). Both are far inside
failsafe_loss_us.

Standard library only (the npz is written and read by hand: npy 1.0 arrays in a zip, readable by numpy.load).
"""

from __future__ import annotations

import argparse
import ast
import ctypes
import json
import math
import os
import select
import struct
import sys
import threading
import time
import zipfile
import zlib
from dataclasses import dataclass, field
from pathlib import Path

from hil_fc import (ARM_STATES, CHANNELS, DRAIN_S, FLAGS_VALID, G0, HELLO_WAIT_S, LINK_FAULTS, MANUAL, MANUAL_MAGIC, SAMPLE,
                    SAMPLE_MAGIC, STALL_S, US_PER_S, Uplink, parse_uplink, stamp_us, status_line)
from hil_link import open_port, record

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LIB = ROOT / "build" / "host-debug" / "sim" / "quad_fc" / "libmarv_sil_quad_fc.so"
PARAM_DEFAULTS_REL = Path("generated") / "marv_params_fc3v2" / "marv" / "params" / "param_defaults.cpp"
KNOWN_VALUES_CRC = 0x97ED61EC  # the CRC the smoke run's board reported (tests/regression/quad/L09/results/hil_s3/README.md)

# Scenario values (labelled scenario values, not flight parameters; core 2). Seconds of sample-stamp time.
FRAME_TICKS = 60  # ManualControl every 60 ticks = 8000 us at the fc3v2 tick: the 125 Hz stick link; a multiple of 1..6
END_S = 8.5
ARM_AT_S = 3.0
DISARM_AT_S = 8.0
SETTLE_S = 0.05  # the DShot reports around a switch edge: one frame period plus the rate-tick spacing
# (name, start s, roll, pitch, yaw, throttle, arm switch high)
PHASES = (
    ("wait", 0.0, 0.0, 0.0, 0.0, 0.0, False),
    ("idle", 3.0, 0.0, 0.0, 0.0, 0.0, True),
    ("throttle", 3.5, 0.0, 0.0, 0.0, 0.4, True),
    ("roll", 4.5, 0.5, 0.0, 0.0, 0.4, True),
    ("pitch", 5.5, 0.0, 0.5, 0.0, 0.4, True),
    ("yaw", 6.5, 0.0, 0.0, 0.5, 0.4, True),
    ("centred", 7.5, 0.0, 0.0, 0.0, 0.4, True),
    ("disarm", 8.0, 0.0, 0.0, 0.0, 0.4, False),
)
MOTION_START_S = 2.0
MOTION_RAMP_S = 0.5
A_ROLL, F_ROLL = 0.15, 1.7  # rad, Hz: well inside arm_small_angle (0.436 rad)
A_PITCH, F_PITCH, PHASE_PITCH = 0.10, 1.1, 0.6
RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW, RC_ARM = 0, 1, 2, 3, 4  # channel indices of the fc3v2 table (AETR, ch5 arm)
SCENARIOS = ("sticks", "motion")
FRAME_VALS = 4 + CHANNELS + 1  # roll pitch yaw throttle, channels, link_quality
EXPECT_DEADLINE_MISSES_AFTER_STREAM = 1  # D_HIL: no record after the last one


# --- npz (npy 1.0 arrays in a zip; the project's bench tools use the standard library only) -----------------------------


def npy_bytes(fmt: str, dtype: str, shape: tuple[int, ...], values) -> bytes:
    flat = list(values)
    header = f"{{'descr': '{dtype}', 'fortran_order': False, 'shape': {shape!r}, }}"
    pad = (64 - (10 + len(header) + 1) % 64) % 64
    header = (header + " " * pad + "\n").encode("latin1")
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", len(header)) + header + struct.pack(f"<{len(flat)}{fmt}", *flat)


def write_npz(path: Path, arrays: dict[str, tuple[str, str, tuple[int, ...], list]]) -> None:
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name, (fmt, dtype, shape, values) in arrays.items():
            z.writestr(name + ".npy", npy_bytes(fmt, dtype, shape, values))


_FMT = {"<f4": "f", "<u4": "I", "<u8": "Q", "<u2": "H", "|u1": "B"}


def read_npz(path: Path) -> dict[str, tuple[tuple[int, ...], list]]:
    out = {}
    with zipfile.ZipFile(path) as z:
        for n in z.namelist():
            b = z.read(n)
            if b[:6] != b"\x93NUMPY" or b[6] != 1:
                raise SystemExit(f"hil_compare: {path}:{n} is not an npy 1.0 array")
            hl = struct.unpack_from("<H", b, 8)[0]
            h = ast.literal_eval(b[10:10 + hl].decode("latin1"))
            if h["fortran_order"] or h["descr"] not in _FMT:
                raise SystemExit(f"hil_compare: {path}:{n} has an unsupported layout {h}")
            count = 1
            for s in h["shape"]:
                count *= s
            data = list(struct.unpack_from(f"<{count}{_FMT[h['descr']]}", b, 10 + hl))
            out[n[:-4]] = (tuple(h["shape"]), data)
    return out


# --- the fc3v2 table (values CRC the way the board computes it, fw/hal/target/src/hil_main.cpp param_values_crc) -------------


def load_table(path: Path) -> list[tuple[str, str, float | int]]:
    """(name, type, value) in ParamId order, from the generated param_defaults.cpp of marv_params_fc3v2."""
    import re
    text = path.read_text()
    out = []
    for name, ty, f, i in re.findall(r"// (\w+)\n\s*\{\{ParamType::(F32|I32), ([^,]+), ([^}]+)\}", text):
        out.append((name, ty, struct.unpack("<f", struct.pack("<f", float(f.strip().rstrip("fF"))))[0] if ty == "F32"
                    else int(i)))
    return out


def values_crc(table: list[tuple[str, str, float | int]]) -> int:
    buf = b"".join(struct.pack("<f", v) if ty == "F32" else struct.pack("<i", v) for _, ty, v in table)
    return zlib.crc32(buf)


# --- the host SIL library -----------------------------------------------------------------------------------------------


class Vec3(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class ImuMeas(ctypes.Structure):
    _fields_ = [("gyro", Vec3), ("accel", Vec3), ("temp_k", ctypes.c_float), ("flags", ctypes.c_uint32)]


class ManualControlC(ctypes.Structure):
    _fields_ = [("struct_size", ctypes.c_uint32), ("frames", ctypes.c_uint32), ("t_us", ctypes.c_uint64),
                ("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float), ("throttle", ctypes.c_float),
                ("channels", ctypes.c_float * CHANNELS), ("link_quality", ctypes.c_float), ("link_t_us", ctypes.c_uint64)]


class SilConfig(ctypes.Structure):
    _fields_ = [("struct_size", ctypes.c_uint32), ("imu_meas_size", ctypes.c_uint32), ("override_size", ctypes.c_uint32),
                ("tick_period_num_us", ctypes.c_uint32), ("tick_period_den", ctypes.c_uint32),
                ("n_overrides", ctypes.c_uint32), ("overrides", ctypes.c_void_p), ("param_schema_hash", ctypes.c_uint64)]


class SilInfo(ctypes.Structure):
    _fields_ = [("struct_size", ctypes.c_uint32), ("n_motors", ctypes.c_uint32), ("n_servos", ctypes.c_uint32),
                ("n_params", ctypes.c_uint32), ("param_schema_hash", ctypes.c_uint64), ("composition", ctypes.c_char_p)]


class SilOut(ctypes.Structure):
    _fields_ = [("struct_size", ctypes.c_uint32), ("capacity_ticks", ctypes.c_uint32), ("t_us", ctypes.c_void_p),
                ("dshot", ctypes.c_void_p), ("servo_us", ctypes.c_void_p)]


class Sil:
    def __init__(self, lib_path: Path):
        self.lib = ctypes.CDLL(str(lib_path))
        L = self.lib
        L.marv_sil_info_get.argtypes = [ctypes.POINTER(SilInfo)]
        L.marv_sil_init.argtypes = [ctypes.POINTER(SilConfig)]
        L.marv_sil_tick.argtypes = [ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(ImuMeas), ctypes.POINTER(SilOut)]
        L.marv_sil_stage_manual_control.argtypes = [ctypes.POINTER(ManualControlC)]
        L.marv_sil_status_str.restype = ctypes.c_char_p
        L.marv_sil_status_str.argtypes = [ctypes.c_int32]
        L.marv_sil_error_index.restype = ctypes.c_uint32
        self.info = SilInfo(struct_size=ctypes.sizeof(SilInfo))
        self.check(L.marv_sil_info_get(ctypes.byref(self.info)), "info_get")

    def check(self, status: int, what: str) -> None:
        if status != 0:
            raise SystemExit(f"hil_compare: marv_sil {what}: {self.lib.marv_sil_status_str(status).decode()} "
                             f"(error index {self.lib.marv_sil_error_index()})")

    def init(self, num: int, den: int) -> None:
        cfg = SilConfig(struct_size=ctypes.sizeof(SilConfig), imu_meas_size=ctypes.sizeof(ImuMeas),
                        override_size=ctypes.sizeof(ctypes.c_uint32) * 5, tick_period_num_us=num, tick_period_den=den,
                        n_overrides=0, overrides=None, param_schema_hash=self.info.param_schema_hash)
        self.check(self.lib.marv_sil_init(ctypes.byref(cfg)), "init")

    def stage(self, frames: int, t_us: int, vals: list[float]) -> None:
        m = ManualControlC(struct_size=ctypes.sizeof(ManualControlC), frames=frames, t_us=t_us, roll=vals[0], pitch=vals[1],
                           yaw=vals[2], throttle=vals[3], link_quality=vals[4 + CHANNELS], link_t_us=t_us)
        for i in range(CHANNELS):
            m.channels[i] = vals[4 + i]
        self.check(self.lib.marv_sil_stage_manual_control(ctypes.byref(m)), "stage_manual_control")

    def tick(self, first: int, imu: list[float], flags: list[int]) -> list[tuple[int, int, int, int]]:
        k = len(flags)
        arr = (ImuMeas * k)()
        for i in range(k):
            a = arr[i]
            a.gyro.x, a.gyro.y, a.gyro.z, a.accel.x, a.accel.y, a.accel.z = imu[6 * i:6 * i + 6]
            a.temp_k = 0.0
            a.flags = flags[i]
        t = (ctypes.c_uint64 * k)()
        d = (ctypes.c_uint16 * (4 * k))()
        s = (ctypes.c_uint16 * (max(self.info.n_servos, 1) * k))()
        out = SilOut(struct_size=ctypes.sizeof(SilOut), capacity_ticks=k, t_us=ctypes.addressof(t), dshot=ctypes.addressof(d),
                     servo_us=ctypes.addressof(s))
        self.check(self.lib.marv_sil_tick(first, k, arr, ctypes.byref(out)), "tick")
        return [(d[4 * i], d[4 * i + 1], d[4 * i + 2], d[4 * i + 3]) for i in range(k)]


# --- the scenario -------------------------------------------------------------------------------------------------------


def phase_at(t_s: float):
    cur = PHASES[0]
    for p in PHASES:
        if p[1] <= t_s:
            cur = p
    return cur


def frame_values(t_s: float) -> list[float]:
    _, _, roll, pitch, yaw, thr, arm = phase_at(t_s)
    ch = [0.0] * CHANNELS
    ch[RC_ROLL], ch[RC_PITCH], ch[RC_YAW], ch[RC_THROTTLE] = roll, pitch, yaw, 2.0 * thr - 1.0
    ch[RC_ARM] = 1.0 if arm else -1.0
    return [roll, pitch, yaw, thr, *ch, 1.0]


def imu_motion(t: float) -> tuple[float, ...]:
    """Gyro (rad/s, FRD) and accel (m/s^2, FRD) of the roll/pitch motion at time t; still before MOTION_START_S."""
    x = (t - MOTION_START_S) / MOTION_RAMP_S
    if x <= 0.0:
        e = de = 0.0
    elif x >= 1.0:
        e, de = 1.0, 0.0
    else:
        e, de = 3 * x * x - 2 * x ** 3, (6 * x - 6 * x * x) / MOTION_RAMP_S
    wr, wp = 2 * math.pi * F_ROLL, 2 * math.pi * F_PITCH
    phi = A_ROLL * e * math.sin(wr * t)
    dphi = A_ROLL * (de * math.sin(wr * t) + e * wr * math.cos(wr * t))
    th = A_PITCH * e * math.sin(wp * t + PHASE_PITCH)
    dth = A_PITCH * (de * math.sin(wp * t + PHASE_PITCH) + e * wp * math.cos(wp * t + PHASE_PITCH))
    p, q, r = dphi, dth * math.cos(phi), -dth * math.sin(phi)  # Euler rates with psi_dot = 0
    return (p, q, r, G0 * math.sin(th), -G0 * math.sin(phi) * math.cos(th), -G0 * math.cos(phi) * math.cos(th))


def build_scenario(name: str, num: int, den: int) -> dict:
    n = int(END_S * US_PER_S * den / num) // FRAME_TICKS * FRAME_TICKS
    imu: list[float] = []
    for k in range(n):
        t = stamp_us(k, num, den) / US_PER_S
        imu.extend(imu_motion(t) if name == "motion" else (0.0, 0.0, 0.0, 0.0, 0.0, -G0))
    frame_tick = list(range(0, n, FRAME_TICKS))
    frame_vals: list[float] = []
    for t in frame_tick:
        frame_vals.extend(frame_values(stamp_us(t, num, den) / US_PER_S))
    return {"n": n, "imu": imu, "flags": [FLAGS_VALID] * n, "frame_tick": frame_tick, "frame_vals": frame_vals}


# --- the reference ------------------------------------------------------------------------------------------------------


def rate_reports(dshot: list[tuple[int, ...]], divisor: int) -> list[tuple[int, tuple[int, ...]]]:
    return [(k, tuple(v)) for k, v in enumerate(dshot) if k % divisor == 0]


def phase_summary(reports: list[tuple[int, tuple[int, ...]]], num: int, den: int) -> list[str]:
    lines = []
    for i, p in enumerate(PHASES):
        end = PHASES[i + 1][1] if i + 1 < len(PHASES) else END_S
        rs = [(t, v) for t, v in reports if p[1] <= stamp_us(t, num, den) / US_PER_S < end]
        armed = [(t, v) for t, v in rs if any(v)]
        if not rs:
            continue
        if not armed:
            lines.append(f"  {p[0]:9s} {p[1]:4.1f}-{end:4.1f} s: {len(rs)} rate ticks, all stop (0)")
            continue
        per = [f"m{m + 1} {min(v[m] for _, v in armed)}..{max(v[m] for _, v in armed)}" for m in range(4)]
        lines.append(f"  {p[0]:9s} {p[1]:4.1f}-{end:4.1f} s: {len(rs)} rate ticks, {len(armed)} armed, DShot " + ", ".join(per))
    return lines


def arm_transitions(reports: list[tuple[int, tuple[int, ...]]]) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    state = False
    for t, v in reports:
        a = any(v)
        if a != state:
            out.append((t, "armed" if a else "stopped"))
            state = a
    return out


def reference(args) -> int:
    lib = Path(args.lib)
    sil = Sil(lib)
    params = Path(args.params) if args.params else lib.resolve().parents[2] / PARAM_DEFAULTS_REL
    if not params.exists():
        raise SystemExit(f"hil_compare: {params} not found (the generated fc3v2 table of the library's build tree; --params)")
    table = load_table(params)
    by_name = {n: v for n, _, v in table}
    if len(table) != sil.info.n_params:
        raise SystemExit(f"hil_compare: {params} has {len(table)} parameters, the library {sil.info.n_params}")
    crc = values_crc(table)
    num, den = int(by_name["tick_period_num_us"]), int(by_name["tick_period_den"])
    div = int(by_name["rate_loop_divisor"])
    sc = build_scenario(args.scenario, num, den)
    n = sc["n"]
    sil.init(num, den)
    dshot: list[tuple[int, ...]] = []
    for f, t0 in enumerate(sc["frame_tick"]):
        v = sc["frame_vals"][FRAME_VALS * f:FRAME_VALS * (f + 1)]
        sil.stage(f + 1, stamp_us(t0, num, den), v)
        t1 = t0 + FRAME_TICKS
        dshot.extend(sil.tick(t0, sc["imu"][6 * t0:6 * t1], sc["flags"][t0:t1]))
    meta = {"scenario": args.scenario, "ticks": n, "tick_period_num_us": num, "tick_period_den": den, "rate_loop_divisor": div,
            "schema_hash": sil.info.param_schema_hash, "param_count": sil.info.n_params, "values_crc": crc,
            "frame_ticks": FRAME_TICKS, "frames": len(sc["frame_tick"]), "end_s": END_S,
            "phases": [list(p) for p in PHASES], "composition": (sil.info.composition or b"").decode(),
            "lib": str(lib.resolve())}
    write_npz(Path(args.out), {
        "meta": ("B", "|u1", (len(json.dumps(meta)),), json.dumps(meta).encode()),
        "imu": ("f", "<f4", (n, 6), sc["imu"]),
        "flags": ("I", "<u4", (n,), sc["flags"]),
        "frame_tick": ("Q", "<u8", (len(sc["frame_tick"]),), sc["frame_tick"]),
        "frame_vals": ("f", "<f4", (len(sc["frame_tick"]), FRAME_VALS), sc["frame_vals"]),
        "dshot": ("H", "<u2", (n, 4), [x for v in dshot for x in v]),
    })
    reports = rate_reports(dshot, div)
    tr = arm_transitions(reports)
    print(f"reference scenario {args.scenario}: {n} ticks ({stamp_us(n, num, den) / US_PER_S:.3f} s), tick {num}/{den} us, "
          f"rate divisor {div}, {len(reports)} rate ticks, {len(sc['frame_tick'])} ManualControl frames every {FRAME_TICKS} ticks")
    print(f"  schema hash 0x{sil.info.param_schema_hash:016x}, {sil.info.n_params} parameters, values CRC "
          + f"0x{crc:08x}" + f", composition {meta['composition']!r}")
    print("DShot of the armed rate ticks per phase:")
    print("\n".join(phase_summary(reports, num, den)))
    print("arming transitions (rate tick, stamp s): " + ", ".join(f"{t} {s} ({stamp_us(t, num, den) / US_PER_S:.4f} s)" for t, s in tr))
    ok = [s for _, s in tr] == ["armed", "stopped"]
    if ok:
        arm_t, stop_t = tr[0][0], tr[1][0]
        ok = (ARM_AT_S <= stamp_us(arm_t, num, den) / US_PER_S <= ARM_AT_S + SETTLE_S
              and DISARM_AT_S <= stamp_us(stop_t, num, den) / US_PER_S <= DISARM_AT_S + SETTLE_S)
    print("REFERENCE OK" if ok else "REFERENCE NOT OK: the reference did not arm and disarm where the scenario says")
    print(f"wrote {args.out}")
    return 0 if ok else 1


def load_ref(path: Path) -> dict:
    a = read_npz(path)
    meta = json.loads(bytes(a["meta"][1]).decode())
    n = meta["ticks"]
    d = a["dshot"][1]
    meta["dshot"] = [tuple(d[4 * k:4 * k + 4]) for k in range(n)]
    meta["imu"] = a["imu"][1]
    meta["flags"] = a["flags"][1]
    meta["frame_tick"] = a["frame_tick"][1]
    meta["frame_vals"] = a["frame_vals"][1]
    meta["reports"] = rate_reports(meta["dshot"], meta["rate_loop_divisor"])
    return meta


# --- the replay ---------------------------------------------------------------------------------------------------------


def hello_mismatches(ref: dict, h: dict) -> list[str]:
    bad = []
    board_hash = (h["schema_hash_high"] << 32) | h["schema_hash_low"]
    if board_hash != ref["schema_hash"]:
        bad.append(f"schema hash board 0x{board_hash:016x} host 0x{ref['schema_hash']:016x}")
    if h["param_count"] != ref["param_count"]:
        bad.append(f"parameter count board {h['param_count']} host {ref['param_count']}")
    if ref["values_crc"] is not None and h["param_values_crc"] != ref["values_crc"]:
        bad.append(f"values CRC board 0x{h['param_values_crc']:08x} host 0x{ref['values_crc']:08x}")
    if (h["tick_period_num_us"], h["tick_period_den"]) != (ref["tick_period_num_us"], ref["tick_period_den"]):
        bad.append(f"tick period board {h['tick_period_num_us']}/{h['tick_period_den']} host "
                   f"{ref['tick_period_num_us']}/{ref['tick_period_den']}")
    if h["rate_loop_divisor"] != ref["rate_loop_divisor"]:
        bad.append(f"rate divisor board {h['rate_loop_divisor']} host {ref['rate_loop_divisor']}")
    return bad


def build_records(ref: dict, batch: int) -> tuple[list[tuple[int, bytes]], int, int]:
    """(last tick of the record, bytes) in send order: a ManualControl record just before the sample record that starts at
    its tick; the downlink sequence numbers 0, 1, 2, ... in that order. Also the sample-record and frame counts."""
    num, den, n = ref["tick_period_num_us"], ref["tick_period_den"], ref["ticks"]
    frames = {t: f for f, t in enumerate(ref["frame_tick"])}
    out: list[tuple[int, bytes]] = []
    seq = nrec = 0
    imu, flags = ref["imu"], ref["flags"]
    for first in range(0, n, batch):
        if first in frames:
            f = frames[first]
            vals = ref["frame_vals"][FRAME_VALS * f:FRAME_VALS * (f + 1)]
            out.append((first + batch - 1, record(MANUAL_MAGIC, seq, MANUAL.pack(f + 1, *vals[:4], *vals[4:4 + CHANNELS],
                                                                              vals[4 + CHANNELS]), 1)))
            seq += 1
        last = min(first + batch, n)
        items = b"".join(SAMPLE.pack(k, stamp_us(k, num, den), *imu[6 * k:6 * k + 6], flags[k]) for k in range(first, last))
        out.append((last - 1, record(SAMPLE_MAGIC, seq, items, last - first)))
        seq += 1
        nrec += 1
    return out, nrec, len(frames)


def replay(args) -> int:
    ref = load_ref(Path(args.ref))
    batch = args.batch
    if batch < 1 or FRAME_TICKS % batch or ref["ticks"] % batch:
        raise SystemExit(f"hil_compare: --batch {batch} must divide {FRAME_TICKS} (the frame spacing) and the {ref['ticks']} ticks")
    sys.setswitchinterval(2e-5)  # the sender spins; the reader must get the GIL promptly
    out = Path(args.out)
    fd = open_port(args.dev)
    buf = bytearray()
    lock = threading.Lock()
    stop = threading.Event()

    def reader() -> None:
        with open(out, "wb") as f:
            while not stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    b = os.read(fd, 1 << 16)
                except BlockingIOError:
                    continue
                f.write(b)
                with lock:
                    buf.extend(b)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    deadline = time.monotonic() + HELLO_WAIT_S
    hello = None
    while hello is None and time.monotonic() < deadline:
        time.sleep(0.05)
        with lock:
            hello = parse_uplink(bytes(buf)).hello
    def finish() -> None:
        stop.set()
        th.join()
        os.close(fd)

    if hello is None:
        finish()
        print(f"REPLAY REFUSED: no hello record within {HELLO_WAIT_S} s (is the MARV_FC3V2_HIL image running?)")
        return 2
    bad = hello_mismatches(ref, hello)
    if bad:
        finish()
        print("REPLAY REFUSED: the board's table is not the host library's: " + "; ".join(bad))
        return 2
    if batch > hello["max_batch"]:
        finish()
        raise SystemExit(f"hil_compare: --batch {batch} exceeds the image's limit {hello['max_batch']}")
    records, nrec, nframes = build_records(ref, batch)
    num, den = ref["tick_period_num_us"], ref["tick_period_den"]
    clock = time.perf_counter_ns
    t0 = clock() + 50_000_000
    aborted = ""

    def write(rec: bytes) -> None:
        nonlocal aborted
        view = memoryview(rec)
        last_progress = time.monotonic()
        while view and not aborted:
            try:
                view = view[os.write(fd, view):]
                last_progress = time.monotonic()
            except BlockingIOError:
                select.select([], [fd], [], 0.05)
                if time.monotonic() - last_progress > STALL_S:
                    aborted = f"write stalled {STALL_S} s"

    for last_tick, rec in records:
        due = t0 + stamp_us(last_tick, num, den) * 1000
        while True:
            rem = due - clock()
            if rem <= 0:
                break
            if rem > 400_000:
                time.sleep((rem - 300_000) / 1e9)
        write(rec)
        if aborted:
            break
    time.sleep(DRAIN_S)
    finish()
    sent = {"batch": batch, "sample_records": nrec, "samples": ref["ticks"], "manual_records": nframes, "aborted": aborted,
            "hello": hello}
    Path(str(out) + ".sent.json").write_text(json.dumps(sent, indent=1) + "\n")
    print(f"replayed {ref['ticks']} samples in {nrec} records ({batch} per record) and {nframes} ManualControl records"
          + (f"; ABORTED: {aborted}" if aborted else "") + f"; uplink in {out}; run: compare {args.ref} {out}")
    return 1 if aborted else 0


# --- the comparison -----------------------------------------------------------------------------------------------------


@dataclass
class Result:
    ref_ticks: int = 0
    common: int = 0
    missing: list[int] = field(default_factory=list)
    extra: list[int] = field(default_factory=list)
    values: int = 0
    exact_values: int = 0
    exact_ticks: int = 0
    max_diff: int = 0
    hist: dict[int, int] = field(default_factory=dict)  # max |diff| of a tick -> number of ticks
    first_mismatch: int | None = None
    first_over_tol: int | None = None
    ref_transitions: list[tuple[int, str]] = field(default_factory=list)
    board_transitions: list[tuple[int, str]] = field(default_factory=list)
    lines: list[str] = field(default_factory=list)


def compare(ref_reports: list[tuple[int, tuple[int, ...]]], board_reports: list[tuple[int, tuple[int, ...]]], tol: int) -> Result:
    """Per rate tick, the board's DShot against the reference's; the arming transitions of both. Pure."""
    r = Result(ref_ticks=len(ref_reports))
    board: dict[int, tuple[int, ...]] = {}
    dup = 0
    for t, v in board_reports:
        dup += t in board
        board[t] = v
    ref = dict(ref_reports)
    r.missing = sorted(set(ref) - set(board))
    r.extra = sorted(set(board) - set(ref))
    common = sorted(set(ref) & set(board))
    r.common = len(common)
    for t in common:
        d = max(abs(a - b) for a, b in zip(ref[t], board[t]))
        r.values += len(ref[t])
        r.exact_values += sum(a == b for a, b in zip(ref[t], board[t]))
        r.exact_ticks += d == 0
        r.hist[d] = r.hist.get(d, 0) + 1
        r.max_diff = max(r.max_diff, d)
        if d and r.first_mismatch is None:
            r.first_mismatch = t
        if d > tol and r.first_over_tol is None:
            r.first_over_tol = t
    r.ref_transitions = arm_transitions(sorted(ref.items()))
    r.board_transitions = arm_transitions(sorted(board.items()))
    L = r.lines
    L.append(f"rate ticks: reference {r.ref_ticks}, board reports compared {r.common}, missing on the board {len(r.missing)}, "
             f"extra {len(r.extra)}, duplicate ticks {dup}")
    if r.common:
        L.append(f"DShot values equal: {r.exact_values} of {r.values} ({100.0 * r.exact_values / r.values:.4f} %); rate ticks "
                 f"with all four motors equal: {r.exact_ticks} of {r.common}; max |difference| {r.max_diff} counts")
        L.append("rate ticks by their largest |difference|: " + ", ".join(f"{d}: {n}" for d, n in sorted(r.hist.items())))
    if r.first_mismatch is not None:
        L.append(f"first mismatch at tick {r.first_mismatch}" + ("" if r.first_over_tol is None else
                 f"; first beyond the tolerance of {tol} at tick {r.first_over_tol}"))
        at = r.first_over_tol if r.first_over_tol is not None else r.first_mismatch
        keys = sorted(ref)
        i = keys.index(at)
        for t in keys[max(0, i - 2):i + 3]:
            L.append(f"  tick {t:6d}{' <' if t == at else '  '} reference {ref[t]}  board {board.get(t, 'missing')}")
    L.append("reference transitions: " + (", ".join(f"{t} {s}" for t, s in r.ref_transitions) or "none"))
    L.append("board transitions:     " + (", ".join(f"{t} {s}" for t, s in r.board_transitions) or "none"))
    return r


def verdict(r: Result, up: Uplink | None, sent: dict | None, tol: int, ticks: int) -> tuple[list[str], bool]:
    reasons: list[str] = []
    if r.missing or r.extra:
        reasons.append(f"report ticks differ: {len(r.missing)} missing, {len(r.extra)} extra")
    if r.max_diff > tol:
        reasons.append(f"max |difference| {r.max_diff} counts exceeds the tolerance {tol}")
    if r.ref_transitions != r.board_transitions:
        reasons.append("arming transitions differ")
    if up is not None and sent is not None:
        if not up.status:
            reasons.append("no status record")
        else:
            last = up.status[-1]
            if (last["sample_records"], last["samples"], last["manual_records"], last["ticks_run"]) != (
                    sent["sample_records"], sent["samples"], sent["manual_records"], sent["samples"]):
                reasons.append(f"board counters {last['sample_records']} records / {last['samples']} samples / "
                               f"{last['manual_records']} ManualControl / {last['ticks_run']} ticks are not those of this run "
                               f"({sent['sample_records']} / {sent['samples']} / {sent['manual_records']}): was the board reset?")
            for k in LINK_FAULTS:
                if last[k]:
                    reasons.append(f"board {k} {last[k]}")
            for k in ("rate_timing_faults", "attitude_faults", "attitude_timing_faults", "queue_overflows"):
                if last[k]:
                    reasons.append(f"board {k} {last[k]}")
            if last["deadline_misses"] != EXPECT_DEADLINE_MISSES_AFTER_STREAM:
                reasons.append(f"{last['deadline_misses']} IMU deadline misses, expected {EXPECT_DEADLINE_MISSES_AFTER_STREAM} "
                               f"(D_HIL after the last record)")
            if any(s["deadline_misses"] for s in up.status if s["ticks_run"] < sent["samples"]):
                reasons.append("IMU deadline miss while the stream ran")
        for name in ("crc_failures", "bad_headers", "seq_gaps", "unframed_bytes"):
            if getattr(up, name):
                reasons.append(f"uplink {name} {getattr(up, name)}")
        if sent.get("aborted"):
            reasons.append("replay aborted: " + sent["aborted"])
    return reasons, not reasons


def compare_cmd(args) -> int:
    ref = load_ref(Path(args.ref))
    up = parse_uplink(Path(args.out).read_bytes())
    sidecar = Path(str(args.out) + ".sent.json")
    sent = json.loads(sidecar.read_text()) if sidecar.exists() else None
    lines: list[str] = []
    h = up.hello
    num, den = ref["tick_period_num_us"], ref["tick_period_den"]
    if h is None:
        print("HIL NOT MATCH: no hello record in the uplink")
        return 1
    clk = h["clk_sys_hz"]
    lines.append(f"reference {args.ref}: scenario {ref['scenario']}, {ref['ticks']} ticks; uplink {args.out}")
    hello_bad = hello_mismatches(ref, h)
    lines.extend("HELLO MISMATCH: " + b for b in hello_bad)
    lines.append(f"board: version {up.version!r}  schema hash 0x{(h['schema_hash_high'] << 32) | h['schema_hash_low']:016x}  "
                 f"values CRC 0x{h['param_values_crc']:08x}  clk_sys {clk} Hz")
    r = compare(ref["reports"], up.reports, args.tolerance)
    lines.extend(r.lines)
    lines.append("board arming state per status (1 s): " + ", ".join(
        f"{s['ticks_run']}:{ARM_STATES[s['arming_state']] if s['arming_state'] < len(ARM_STATES) else s['arming_state']}"
        for s in up.status))
    if up.status:
        last = up.status[-1]
        lines.append(status_line(last, clk))
        batch = sent["batch"] if sent else 4
        period = num / den
        us = lambda c: c * US_PER_S / clk  # noqa: E731
        lines.append(f"EMB-3 inputs: longest tick {last['max_tick_cycles']} cycles = {us(last['max_tick_cycles']):.1f} us "
                     f"against the tick period {period:.1f} us; longest record of {batch} ticks {last['max_frame_cycles']} "
                     f"cycles = {us(last['max_frame_cycles']):.1f} us against {batch} tick periods {batch * period:.1f} us "
                     f"({100.0 * us(last['max_frame_cycles']) / (batch * period):.1f} %); longest arrival gap "
                     f"{last['max_arrival_gap_us']} us, longest USB send {last['max_send_us']} us (reported, not judged here)")
    reasons, ok = verdict(r, up, sent, args.tolerance, ref["ticks"])
    if hello_bad:
        reasons.append("the board's hello does not match the reference")
    ok = not reasons
    lines.append("tolerance " + str(args.tolerance) + " counts (see the tolerance note in the tool's help)")
    lines.append("HIL MATCH" if ok else "HIL NOT MATCH: " + "; ".join(reasons))
    text = "\n".join(lines)
    Path(str(args.out) + ".compare.txt").write_text(text + "\n")
    print(text)
    return 0 if ok else 1


def refdiff_cmd(args) -> int:
    a, b = load_ref(Path(args.a)), load_ref(Path(args.b))
    if (a["ticks"], a["rate_loop_divisor"]) != (b["ticks"], b["rate_loop_divisor"]) or a["imu"] != b["imu"]:
        raise SystemExit("hil_compare: the two references are not the same stream")
    r = compare(a["reports"], b["reports"], args.tolerance)
    print(f"{args.a} ({a['lib']}) against {args.b} ({b['lib']}), scenario {a['scenario']}")
    print("\n".join(r.lines))
    return 0 if r.max_diff <= args.tolerance and r.ref_transitions == r.board_transitions else 1


# --- selftest -----------------------------------------------------------------------------------------------------------


def selftest() -> int:
    fails: list[str] = []

    def expect(cond: bool, what: str) -> None:
        if not cond:
            fails.append(what)

    ref = [(t, (0, 0, 0, 0)) for t in range(0, 20, 2)] + [(t, (100 + t, 101 + t, 102 + t, 103 + t)) for t in range(20, 60, 2)]
    ref += [(t, (0, 0, 0, 0)) for t in range(60, 80, 2)]
    # match
    r = compare(ref, list(ref), 0)
    reasons, ok = verdict(r, None, None, 0, 80)
    expect(ok and r.max_diff == 0 and r.exact_values == r.values and r.first_mismatch is None, "identical streams are a match")
    expect(r.ref_transitions == [(20, "armed"), (60, "stopped")] == r.board_transitions, "transitions found at ticks 20 and 60")
    # 1-count difference
    one = [(t, (v[0] + (t == 30), *v[1:])) for t, v in ref]
    r1 = compare(ref, one, 1)
    expect(r1.max_diff == 1 and r1.first_mismatch == 30 and verdict(r1, None, None, 1, 80)[1], "a 1-count difference passes at 1")
    r0 = compare(ref, one, 0)
    reasons0, ok0 = verdict(r0, None, None, 0, 80)
    expect(not ok0 and r0.first_over_tol == 30 and any("exceeds" in x for x in reasons0), "a 1-count difference fails at 0")
    expect(r1.exact_values == r1.values - 1 and r1.hist == {0: len(ref) - 1, 1: 1}, "one value differs, one tick in the histogram")
    two = [(t, (v[0] + 2 * (t == 30), *v[1:])) for t, v in ref]
    expect(not verdict(compare(ref, two, 1), None, None, 1, 80)[1], "a 2-count difference fails at 1")
    # arming tick mismatch: the board arms 2 ticks late (values otherwise equal where both armed)
    late = [(t, (0, 0, 0, 0) if t == 20 else v) for t, v in ref]
    ra = compare(ref, late, 1)
    reasonsa, oka = verdict(ra, None, None, 1, 80)
    expect(not oka and ra.board_transitions[0] == (22, "armed") and any("transitions" in x for x in reasonsa),
           "an arming tick mismatch fails even though no armed value differs by more than the tolerance would allow")
    # missing and extra reports
    rm = compare(ref, ref[:-1] + [(999, (0, 0, 0, 0))], 1)
    expect(rm.missing == [78] and rm.extra == [999] and not verdict(rm, None, None, 1, 80)[1], "missing and extra report ticks fail")
    # the npz round trip, and numpy compatibility when present
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "x.npz"
        write_npz(p, {"a": ("f", "<f4", (2, 3), [1, 2, 3, 4, 5, 6.5]), "b": ("H", "<u2", (3,), [1, 2, 65535]),
                      "m": ("B", "|u1", (2,), b"hi")})
        back = read_npz(p)
        expect(back["a"] == ((2, 3), [1, 2, 3, 4, 5, 6.5]) and back["b"] == ((3,), [1, 2, 65535]) and bytes(back["m"][1]) == b"hi",
               "npz round trip")
        try:
            import numpy
            z = numpy.load(p)
            expect(z["a"].shape == (2, 3) and z["b"][2] == 65535, "numpy reads the npz")
        except ImportError:
            pass
    # the values CRC against the smoke run's board, when the host build tree is present
    params = DEFAULT_LIB.parents[2] / PARAM_DEFAULTS_REL
    if params.exists():
        t = load_table(params)
        expect(values_crc(t) == KNOWN_VALUES_CRC, f"values CRC 0x{values_crc(t):08x} is the board's 0x{KNOWN_VALUES_CRC:08x}")
    # the scenario is a pure function of its inputs; the motion stream is continuous at the end of its ramp
    a = build_scenario("motion", 1953125, 14649)
    b = build_scenario("motion", 1953125, 14649)
    expect(a == b and a["n"] % FRAME_TICKS == 0, "scenario deterministic, frame aligned")
    g = imu_motion(MOTION_START_S + MOTION_RAMP_S)
    expect(abs(math.hypot(g[3], g[4], g[5]) - G0) < 1e-9, "motion accel has magnitude g0")
    recs, nrec, nfr = build_records({**{k: v for k, v in a.items()}, "ticks": a["n"], "tick_period_num_us": 1953125,
                                     "tick_period_den": 14649}, 4)
    expect(nrec == a["n"] // 4 and nfr == len(a["frame_tick"]), "record counts")
    print("selftest: " + ("OK" if not fails else "FAILED: " + "; ".join(fails)))
    return 0 if not fails else 1


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("reference")
    s.add_argument("out", type=Path)
    s.add_argument("--scenario", choices=SCENARIOS, default="sticks")
    s.add_argument("--lib", type=Path, default=DEFAULT_LIB)
    s.add_argument("--params", type=Path, default=None, help="param_defaults.cpp of marv_params_fc3v2 (default: beside the library's build tree)")
    s = sub.add_parser("replay")
    s.add_argument("dev")
    s.add_argument("ref", type=Path)
    s.add_argument("out", type=Path)
    s.add_argument("--batch", type=int, default=4, help="samples per record (default 4, the S1 configuration)")
    s = sub.add_parser("compare")
    s.add_argument("ref", type=Path)
    s.add_argument("out", type=Path)
    s.add_argument("--tolerance", type=int, default=1, help="largest accepted |DShot difference| in counts (default 1)")
    s = sub.add_parser("refdiff")
    s.add_argument("a", type=Path)
    s.add_argument("b", type=Path)
    s.add_argument("--tolerance", type=int, default=1)
    sub.add_parser("selftest")
    a = p.parse_args(argv)
    if a.cmd == "selftest":
        return selftest()
    if a.cmd == "reference":
        return reference(a)
    if a.cmd == "replay":
        return replay(a)
    if a.cmd == "refdiff":
        return refdiff_cmd(a)
    return compare_cmd(a)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
