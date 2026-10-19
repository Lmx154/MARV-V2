#!/usr/bin/env python3
"""HIL S2e (quad spec 4.0 M4, decision 0046 stage S2): the quad_fc flight composition flies the real FC3v2 card in Gazebo, in
angle mode, from a stick script, off the launch pad, with the N-tick command delay. Evidence, not a test: nothing here is
tuned, nothing is a flight result, and the card values it flies are INFERRED / UNVERIFIED / PLACEHOLDER.

  run_quad_fc.py --out-dir <dir> [--plugin-dir <host-gz-quadfc plugin dir>] [--runs flight|control|both] [--max-s <s>]
                 [--link-udp-port <port>] [--config-dir <dir>] [--interactive]

Needs `cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc` and gz-sim 8. Per run it writes to <dir>:
the world (<stem>.sdf), the full lockstep log (<stem>.bin, ~80 MB, not committed; its sha256 is in the analysis), the L2 run
report (<stem>.report.txt), the analysis (<stem>.analysis.txt) and the decimated traces (<stem>.traces.csv). Two runs of the
same world give the same log sha256 (the run is deterministic).

The world. The real card (vehicles/marv_7in_fc3v2.yaml) at the FC3v2 tick (fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml,
tick_period_num_us / tick_period_den, whole ns rounded by the plugin, decision 0046 item 1), the L2 hover scenario's site,
m = 1, the model gyro (nominal corner, no turn-on bias), <launch_pad/> (decision 0047), <command_delay_ticks>N</command_delay_ticks>
with N = COMMAND_DELAY_TICKS, no open-loop DShot override, the register's rc_channel_angle (8) and the script below.

The stick script (SIL time; every duration is a labelled scenario value unless it says otherwise). Frames at the link rate
FRAME_PERIOD_US; the arm switch is channel 5 at -1 / +1 (-100 / 100), the angle switch channel 8 at +1 (angle) throughout.
  1 ready    link on, sticks centred, throttle 0, arm off, until READY_US = gyro_cal_duration_us + failsafe_recovery_us + MARGIN_US
  2 arm      arm on, throttle 0, for ARM_IDLE_US
  3 hover    throttle stick = the DERIVED hover stick (below), step; the pad releases when the vertical thrust reaches the weight
  4 hold     HOLD_S
  5 roll     roll stick STEP_STICK for STEP_S, then centred for CENTRE_S
  6 pitch    pitch stick STEP_STICK for STEP_S, then centred for CENTRE_S
  7 dip      throttle stick 0 for DIP_S (below arm_throttle_min: the decision 0045 integrator hold applies), then hover again
  8 hold     HOLD2_S
  9 disarm   arm off for TAIL_US, then the run ends
The hover stick (derived, no tuning). quad_fc: the throttle stick s is linear in rotor speed (decision 0052): thrust request
T = (sqrt(T_min) + s (sqrt(T_max) - sqrt(T_min)))^2 with T_min = 4 k omega_idle^2, T_max = 4 k omega_max^2 (fw/compositions/quad_fc/src/quad_fc.cpp:455-456;
g_thrust_root_min and g_thrust_root_max, :368-369, are the square roots of 4 f_min and 4 f_max of the mixer,
fw/mixer/include/marv/mixer/mixer.hpp:76-84), the mixer at zero torque gives each motor f = c M_thrust = T / 4 (mixer.hpp:216 and
:226; M_thrust = 0.25 in the fc3v2 table), omega = sqrt(f / k), DShot = 48 + (omega - omega_min) / (omega_max - omega_min) 1999
(mixer.hpp:246-251 and :263-265, the linear-in-omega ESC map, 48 and 2047 = prim kDshotThrottleMin and Max, omega range = the card's
rotors.speed_range [0, 2297], idle_speed = omega_min = 0), rounded with the DshotDiffuser carry (fw/rate_group, decision 0017).
Hover: T = m g(phi, h0) (tools/sim/hover.py, the L2 site) gives s = (sqrt(m g) - sqrt(T_min)) / (sqrt(T_max) - sqrt(T_min)).

The integrator-hold script (--script ihold, M4 S2f, decision 0045; evidence for tests/regression/quad/L10/results/ihold). The same world,
card, firmware and N, with a constant disturbance torque and one dip:
  ready, arm and hover as above; hold IHOLD_HOLD_S from the throttle raise (the rate integrator settles against the disturbance);
  dip     throttle stick DIP_BELOW_STICK = 0 (below arm_throttle_min: the decision 0045 hold zeroes the integrator) or DIP_ABOVE_STICK =
          arm_throttle_min + 2 / 1024 (2 us of the receive contract throttle = (us - 988) / 1024 above min_check: the hold never engages),
          for DIP_S; then hover again for IHOLD_HOLD2_S, then disarm. No roll, pitch or yaw stick at any time.
The disturbance is --com-offset-m: every rotor position of the plugin element is shifted by that distance along x and y (the rotors move
with respect to the centre of mass, which is the plant's origin), so the plant torque is r x F with F = -T on body z: tau_x = -T d,
tau_y = +T d for rotor positions + d, proportional to the thrust. The firmware's mixer comes from the unshifted card, so the shift is
an unmodelled constant trim torque. No plant, plugin or card change: the rotor positions are inputs of the plugin element.

The control run (decision 0042's amended control, tests/regression/quad/L07/gz/test_t4_hover_estimate.py): the same script with
att_kp = 0, rate_ki_roll = 0 and rate_ki_pitch = 0 through <sil_override>, so no gain returns the tilt to level.

The ground link (decision 0053 Q4, ground-segment unit 6). --link-udp-port <port> writes <link_udp_port> (the SIL's link socket
on 127.0.0.1:<port>) and --config-dir <dir> writes <config_dir> (the config storage, made absolute and created if missing) into
the plugin element (sim/gz/plugin/src/lockstep.cpp); without them the world is byte-identical to one without the options. A gz
exit status of EX_TEMPFAIL is the plugin's "reboot requested" (a config command was accepted): the run stops, nothing is
analysed, and this script exits with os.EX_TEMPFAIL so that the launcher (tools/gs/sil.py) relaunches it on the same config dir.

The interactive run (--interactive, for the ground station's SIL launcher; --runs, --script, --step-stick, --com-offset-m and
--max-s do not apply). The same card, tick, site, sensors, N and launch pad, in a world of gen_world mode "pilot", whose
<real_time_factor> 1 makes gz-sim pace the host steps on the wall clock (the plugin itself reads no wall clock); one script
segment of INTERACTIVE_S keeps the link on, the sticks centred, the throttle 0 and the arm switch low (the vehicle stays
disarmed on the pad). The world also has <ground/> (decision 0047 amendment, Luis 2026-10-18): a touchdown at or below the start
height while moving down puts the vehicle at rest there and re-engages the pad's hold. No log and no analysis: the run lasts
INTERACTIVE_S unless it is stopped (SIGTERM) first. Files: <dir>/interactive/<stem>.sdf.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import math
import os
import re
import struct
import subprocess
import sys
from pathlib import Path
from xml.sax.saxutils import escape

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import gen_plant_config as gpc  # noqa: E402
import gen_world  # noqa: E402
import hover as hover_mod  # noqa: E402
import lockstep_log  # noqa: E402
import mixer as card_mixer  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402

CARD = ROOT / "vehicles" / "marv_7in_fc3v2.yaml"
BASE_SCENARIO = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
REGISTER = ROOT / "fw" / "compositions" / "quad_fc" / "params" / "quad_fc_register.yaml"
FC3_SCENARIO_VALUES = ROOT / "fw" / "compositions" / "quad_fc" / "params" / "fc3v2_scenario_values.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz-quadfc" / "sim" / "gz" / "plugin"
TIMEOUT_S = 3600  # labelled harness value: bounds a hung gz process only
SEED = 1
M = 1

COMMAND_DELAY_TICKS = 10  # decision 0046 item 2: N >= 10 needed on the nominal rows (tests/regression/quad/L09/results/hil_s2)
FRAME_PERIOD_US = 8000  # 125 frames/s, measured on Luis's ExpressLRS link (docs/m2-bench-card.md)
MARGIN_US = 1_000_000  # labelled harness value: margin after the calibration and the link recovery (as the L10 plumbing test)
ARM_IDLE_US = 250_000  # labelled harness value: armed at minimum throttle before the raise (the plumbing test's SETTLE_US)
HOLD_S = 5.0  # packet: hold ~5 s
STEP_STICK = 0.5  # labelled scenario value: half deflection (packet example); 0.5 angle_tilt_max = 30 degrees at the fc3v2 value
STEP_S = 2.0  # packet: ~2 s
CENTRE_S = 3.0  # packet: ~3 s
DIP_S = 0.3  # packet example
HOLD2_S = 3.0  # packet: ~3 s
TAIL_US = 250_000  # labelled harness value: sim time after the disarm edge (the stop command is read on it)
SWITCH_LOW, SWITCH_HIGH = -1.0, 1.0  # normalised channel values, -100 / 100
ARM_CHANNEL, ANGLE_CHANNEL = 5, 8  # the register's rc_channel_arm and rc_channel_angle (read and asserted below)
SETTLE_BAND = 0.10  # labelled analysis value: a step has settled when the error stays within 10 % of the setpoint
STEADY_WINDOW_S = 0.5  # labelled analysis value: "steady state" is the last 0.5 s of a phase
DECIMATE = 32  # labelled analysis value: one trace row per 32 ticks (about 234 Hz)
DSHOT_MIN, DSHOT_MAX = 48, 2047  # DShot protocol, core section 4
CONTROL_OVERRIDES = (("att_kp", "0.0"), ("rate_ki_roll", "0.0"), ("rate_ki_pitch", "0.0"))
STAMP_FIELD = "sil_t_us"
IHOLD_HOLD_S = 20.0  # labelled scenario value: time at hover from the throttle raise to the dip (the integrator settles in a fraction of it; the lift-off transient and the horizontal drift decay)
IHOLD_HOLD2_S = 6.0  # labelled scenario value: time at hover after the dip
IHOLD_COM_OFFSET_M = 0.005  # labelled scenario value, PLACEHOLDER: a 5 mm rotor-to-centre-of-mass offset on x and y (battery placement tolerance)
DIP_BELOW_STICK = 0.0  # the dip stick that engages the decision 0045 integrator hold (below arm_throttle_min)
IHOLD_PRE_S = 1.0  # labelled analysis value: the pre-dip window is the last second before the dip
IHOLD_RETURN_FRACTION = 0.1  # labelled analysis value: the return band is the pre-dip envelope or 10 % of the post-dip excursion, whichever is wider
INTERACTIVE_S = 24 * 3600.0  # labelled harness value: the longest interactive run (a day; the launcher stops it before that); its
# host steps must fit gz's --iterations, a C int (gz-sim 8 cmdsim8.rb refuses 4536174183)


def register_value(name):
    return yaml.safe_load(REGISTER.read_text(encoding="utf-8"))[name]["value"]


def scenario_value(name):
    return yaml.safe_load(FC3_SCENARIO_VALUES.read_text(encoding="utf-8"))[name]["value"]


# ---- the derived hover stick ------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Hover:
    mass_kg: float
    k_t: float
    omega_min: float
    omega_max: float
    omega_idle: float
    g: float
    omega_h: float
    d_star: float
    d_lo: int
    d_hi: int
    weight_n: float
    t_min: float
    t_max: float
    stick: float
    stick_f32: float
    m_thrust: float


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def derive_hover():
    cfg = gpc.plant_config(CARD)
    sc = scn.values(scn.load(BASE_SCENARIO, CARD))
    lat, h0 = sc["site_latitude_rad"], sc["site_height_m"]
    omega_h, d_star, d_lo, d_hi = hover_mod.bracket_for_card(cfg, lat, h0)
    g = hover_mod.normal_gravity(lat, h0)
    card = yaml.safe_load(CARD.read_text(encoding="utf-8"))
    m_mat, _, idle = card_mixer.mixer_matrix(_linted(), CARD)
    m_thrust = m_mat[0][0]
    k, w0, w1 = cfg["thrust_coeff"], cfg["omega_min_rad_s"], cfg["omega_max_rad_s"]
    t_min, t_max = 4.0 * k * idle * idle, 4.0 * k * w1 * w1
    weight = cfg["mass_kg"] * g
    stick = (math.sqrt(weight) - math.sqrt(t_min)) / (math.sqrt(t_max) - math.sqrt(t_min))
    assert card["mass"]["value"] == cfg["mass_kg"]
    return Hover(cfg["mass_kg"], k, w0, w1, idle, g, omega_h, d_star, d_lo, d_hi, weight, t_min, t_max, stick, f32(stick),
                 m_thrust)


def _linted():
    card, _ = gpc.load_linted(CARD, ROOT)
    return card


# ---- the stick script -------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Phase:
    name: str
    start_us: int
    duration_us: int
    arm: float
    roll: float = 0.0
    pitch: float = 0.0
    throttle: float = 0.0

    @property
    def end_us(self):
        return self.start_us + self.duration_us


def us(seconds):
    return round(seconds * 1_000_000)


def build_phases(hover, step_stick):
    cal, recovery = register_value("gyro_cal_duration_us"), register_value("failsafe_recovery_us")
    ready = cal + recovery + MARGIN_US
    hv = hover.stick_f32
    spec = [("ready", ready, SWITCH_LOW, 0.0, 0.0, 0.0),
            ("arm_idle", ARM_IDLE_US, SWITCH_HIGH, 0.0, 0.0, 0.0),
            ("hold", us(HOLD_S), SWITCH_HIGH, 0.0, 0.0, hv),
            ("roll_step", us(STEP_S), SWITCH_HIGH, step_stick, 0.0, hv),
            ("roll_centre", us(CENTRE_S), SWITCH_HIGH, 0.0, 0.0, hv),
            ("pitch_step", us(STEP_S), SWITCH_HIGH, 0.0, step_stick, hv),
            ("pitch_centre", us(CENTRE_S), SWITCH_HIGH, 0.0, 0.0, hv),
            ("dip", us(DIP_S), SWITCH_HIGH, 0.0, 0.0, 0.0),
            ("hold2", us(HOLD2_S), SWITCH_HIGH, 0.0, 0.0, hv),
            ("disarmed", TAIL_US, SWITCH_LOW, 0.0, 0.0, hv)]
    phases, t = [], 0
    for name, dur, arm, roll, pitch, thr in spec:
        phases.append(Phase(name, t, dur, arm, roll, pitch, thr))
        t += dur
    return phases


def dip_above_stick():
    """arm_throttle_min + 2 / 1024: 2 us above min_check of the receive contract throttle = (us - 988) / 1024 (the register's source)."""
    return register_value("arm_throttle_min") + 2.0 / 1024.0


def build_phases_ihold(hover, dip_stick):
    cal, recovery = register_value("gyro_cal_duration_us"), register_value("failsafe_recovery_us")
    ready = cal + recovery + MARGIN_US
    hv = hover.stick_f32
    spec = [("ready", ready, SWITCH_LOW, 0.0),
            ("arm_idle", ARM_IDLE_US, SWITCH_HIGH, 0.0),
            ("hold", us(IHOLD_HOLD_S), SWITCH_HIGH, hv),
            ("dip", us(DIP_S), SWITCH_HIGH, dip_stick),
            ("hold2", us(IHOLD_HOLD2_S), SWITCH_HIGH, hv),
            ("disarmed", TAIL_US, SWITCH_LOW, hv)]
    phases, t = [], 0
    for name, dur, arm, thr in spec:
        phases.append(Phase(name, t, dur, arm, 0.0, 0.0, thr))
        t += dur
    return phases


def script_xml(phases):
    segs = "".join(
        f"<segment><start_us>{p.start_us}</start_us><duration_us>{p.duration_us}</duration_us><link>on</link>"
        f"<roll>{p.roll!r}</roll><pitch>{p.pitch!r}</pitch><yaw>0.0</yaw><throttle>{p.throttle!r}</throttle>"
        f"<arm>{p.arm!r}</arm><angle>{SWITCH_HIGH!r}</angle></segment>" for p in phases)
    return (f"<manual_control_script><frame_period_us>{FRAME_PERIOD_US}</frame_period_us><arm_channel>{ARM_CHANNEL}</arm_channel>"
            f"<angle_channel>{ANGLE_CHANNEL}</angle_channel>{segs}</manual_control_script>")


def com_offset_edit(text, d):
    pat = re.compile(r'(<position_frd_m unit="m">)(\S+) (\S+) (\S+)(</position_frd_m>)')
    assert len(pat.findall(text)) == 4
    return pat.sub(lambda m: f"{m[1]}{float(m[2]) + d!r} {float(m[3]) + d!r} {m[4]}{m[5]}", text)


def link_xml(link_udp_port=None, config_dir=None):
    """The plugin's ground-link elements (module docstring); "" without either."""
    text = ""
    if config_dir is not None:
        text += f"<config_dir>{escape(str(config_dir))}</config_dir>\n"
    if link_udp_port is not None:
        text += f"<link_udp_port>{int(link_udp_port)}</link_udp_port>\n"
    return text


def sdf_edit_for(phases, overrides, com_offset_m=0.0, link="", ground=False):
    def edit(text):
        if com_offset_m:
            text = com_offset_edit(text, com_offset_m)
        text = re.sub(r'\s*<sil_override param="ol_dshot_m\d" type="i32">\d+</sil_override>', "", text)
        add = "<launch_pad/>\n" + ("<ground/>\n" if ground else "") + script_xml(phases) + "\n" + link
        for name, value in overrides:
            add += f'<sil_override param="{name}" type="f32">{value}</sil_override>\n'
        assert text.count("<gyro_source>") == 1
        return text.replace("<gyro_source>", add + "<gyro_source>", 1)
    return edit


def fc3_scenario(out_dir):
    num, den = scenario_value("tick_period_num_us"), scenario_value("tick_period_den")
    text = BASE_SCENARIO.read_text(encoding="utf-8")
    text, k1 = re.subn(r"(tick_period_num_us:\n  value: )\d+", rf"\g<1>{num}", text)
    text, k2 = re.subn(r"(tick_period_den:\n  value: )\d+\n", rf"\g<1>{den}\n", text)
    assert (k1, k2) == (1, 1)
    (Path(out_dir) / "scenario").mkdir(parents=True, exist_ok=True)
    path = Path(out_dir) / "scenario" / "hover.yaml"  # the scenario's name must equal the file stem
    path.write_text(text, encoding="utf-8")
    return path, num, den


# ---- analysis ---------------------------------------------------------------------------------------------------------

def tilt_split(q):
    """(tilt angle rad, world-frame tilt vector (x, y) rad, heading psi rad) of the body->NED quaternion q = q_xy (x) q_z,
    the split of fw/attitude/include/marv/attitude/tilt_yaw.hpp."""
    w, x, y, z = q
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    rho = math.hypot(w, z)
    if rho == 0.0:
        return math.pi, (math.pi, 0.0), 0.0
    a = (w * x - y * z) / rho
    b = (w * y + x * z) / rho
    n = math.hypot(a, b)
    angle = 2.0 * math.atan2(n, rho)
    psi = 2.0 * math.atan2(z / rho, w / rho)
    if n == 0.0:
        return angle, (0.0, 0.0), psi
    return angle, (angle * a / n, angle * b / n), psi


def heading_frame(v, psi):
    c, s = math.cos(psi), math.sin(psi)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1])


def effectiveness():
    card = _linted()
    _, b, _ = card_mixer.mixer_matrix(card, CARD)
    return b


def dshot_to_omega(d, w0, w1):
    return 0.0 if d == 0 else w0 + (d - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN) * (w1 - w0)


@dataclasses.dataclass
class Series:
    t_us: list
    tilt: list
    tvec: list  # heading-frame tilt vector (x, y), rad
    sp: list  # heading-frame setpoint tilt vector, rad
    psi: list
    pos: list
    vel: list
    omega: list
    dshot: list
    dshot_applied: list
    rotor: list
    wrench: list  # (T, tau_x, tau_y, tau_z) from the plant's rotor speeds, B (k w^2)
    cmd: list  # the same from the delayed DShot command through the ESC map (what the mixer asked, quantised)
    stick: list
    frames: list
    accel: list  # the model IMU's specific force, FRD, m/s^2 (what the firmware's estimator is given)
    est_tvec: list  # heading-frame tilt vector of the replica estimate, rad (None before it initialises)
    est_err: list  # angle between the estimated and the true body z, rad


G_STANDARD = 9.80665  # m/s^2, fw/prim/include/marv/prim/constants.hpp kStandardGravity (the estimator's health window is in g)


def quat_mul(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def mahony_replica(ticks):
    """A binary64 re-computation of fw/estimator/include/marv/estimator/mahony.hpp (Ki = 0, no bias estimate) on the logged model-IMU
    samples with the register's gain and health window, the gyro less the mean of the samples of the first gyro_cal_duration_us (the
    firmware's calibration, fw/estimator/include/marv/estimator/gyro_cal.hpp, up to its movement test). Analysis only: not the
    firmware's binary32 output. Returns the estimated body->NED quaternion per tick, None before the estimator initialises."""
    kp, ki = register_value("estimator_kp"), register_value("estimator_ki")
    assert ki == 0.0, "the replica has no bias estimate"
    lo, hi = register_value("estimator_accel_healthy_min") * G_STANDARD, register_value("estimator_accel_healthy_max") * G_STANDARD
    cal_us = register_value("gyro_cal_duration_us")
    imu = [struct.unpack("<3f3f", t["imu"][:24]) for t in ticks]
    cal = [i[:3] for i, t in zip(imu, ticks) if t["sil_t_us"] < cal_us]
    bias = [sum(c[a] for c in cal) / len(cal) for a in range(3)]
    q, prev, out = None, None, []
    for i, t in zip(imu, ticks):
        gyro = [i[a] - (bias[a] if t["sil_t_us"] >= cal_us else 0.0) for a in range(3)]
        f = i[3:6]
        n2 = sum(x * x for x in f)
        if q is None:
            if lo * lo < n2 < hi * hi:
                roll, pitch = math.atan2(-f[1], -f[2]), math.atan2(f[0], math.hypot(f[1], f[2]))
                qp = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
                qr = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
                q = quat_mul(qp, qr)
                prev = t["sil_t_us"]
            out.append(q)
            continue
        dt = (t["sil_t_us"] - prev) * 1e-6
        prev = t["sil_t_us"]
        e = (0.0, 0.0, 0.0)
        if n2 > 0.0:
            n = math.sqrt(n2)
            v = (f[0] / n, f[1] / n, f[2] / n)
            w, x, y, z = q
            vh = (-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(1.0 - 2.0 * (x * x + y * y)))
            e = (v[1] * vh[2] - v[2] * vh[1], v[2] * vh[0] - v[0] * vh[2], v[0] * vh[1] - v[1] * vh[0])
        om = [gyro[a] + kp * e[a] for a in range(3)]
        ang = math.sqrt(sum(x * x for x in om)) * dt
        if ang > 0.0:
            k = math.sin(ang / 2) / (ang / dt)
            d = (math.cos(ang / 2), om[0] * k, om[1] * k, om[2] * k)
            q = quat_mul(q, d)
            nrm = math.sqrt(sum(x * x for x in q))
            q = tuple(x / nrm for x in q)
        out.append(q)
    return out


def z_axis(q):
    w, x, y, z = q
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))


def z_angle(qa, qb):
    a, b = z_axis(qa), z_axis(qb)
    return math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)))))


def series_of(log, hover, tilt_max):
    steps, ticks = log["steps"], log["ticks"]
    assert len(steps) == len(ticks)  # m = 1
    manuals = log["manuals"]
    b = effectiveness()
    k, w0, w1 = hover.k_t, hover.omega_min, hover.omega_max
    s = Series(*[[] for _ in range(18)])
    qe = mahony_replica(ticks)
    mi = -1
    for j, (st, t) in enumerate(zip(steps, ticks)):
        while mi + 1 < len(manuals) and manuals[mi + 1]["tick"] <= j:
            mi += 1
        mc = manuals[mi] if mi >= 0 else None
        tilt, v, psi = tilt_split(st["body_q_wxyz"])
        sp = (tilt_max * mc["roll"], tilt_max * mc["pitch"]) if mc else (0.0, 0.0)
        s.t_us.append(t["sil_t_us"])
        s.tilt.append(tilt)
        s.tvec.append(heading_frame(v, psi))
        s.sp.append(sp)
        s.psi.append(psi)
        s.pos.append(st["body_pos_ned"])
        s.vel.append(st["body_vel_ned"])
        s.omega.append(st["body_omega_frd"])
        s.dshot.append(t["dshot"])
        da = t["dshot_applied"]
        s.dshot_applied.append(da)
        s.rotor.append(t["rotor_speed"])
        f = [k * w * w for w in t["rotor_speed"]]
        s.wrench.append(tuple(sum(b[r][i] * f[i] for i in range(4)) for r in range(4)))
        fc = [k * dshot_to_omega(d, w0, w1) ** 2 for d in da]
        s.cmd.append(tuple(sum(b[r][i] * fc[i] for i in range(4)) for r in range(4)))
        s.stick.append((mc["roll"], mc["pitch"], mc["throttle"], mc["channels"][ARM_CHANNEL - 1]) if mc else (0.0,) * 4)
        s.frames.append(mc["frames"] if mc else 0)
        s.accel.append(struct.unpack("<3f3f", t["imu"][:24])[3:6])
        if qe[j] is None:
            s.est_tvec.append((0.0, 0.0))
            s.est_err.append(0.0)
        else:
            _, ve, pe = tilt_split(qe[j])
            s.est_tvec.append(heading_frame(ve, pe))
            s.est_err.append(z_angle(qe[j], st["body_q_wxyz"]))
    return s


def deg(x):
    return math.degrees(x)


def window(s, a_us, b_us):
    lo = next((i for i, t in enumerate(s.t_us) if t >= a_us), len(s.t_us))
    hi = next((i for i, t in enumerate(s.t_us) if t >= b_us), len(s.t_us))
    return lo, hi


def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else float("nan")


def accel_tilt(a):
    """Angle between the measured specific force and -z of the body: the tilt a level-hover accelerometer would imply."""
    return math.atan2(math.hypot(a[0], a[1]), -a[2])


def phase_stats(s, p, release_us):
    a = max(p.start_us, release_us) if release_us is not None else p.start_us
    lo, hi = window(s, a, p.end_us)
    if hi <= lo:
        return None
    tl = s.tilt[lo:hi]
    z0, z1 = s.pos[lo][2], s.pos[hi - 1][2]
    return {"from_us": s.t_us[lo], "to_us": s.t_us[hi - 1], "tilt_mean": mean(tl), "tilt_max": max(tl),
            "z_start": z0, "z_end": z1, "vz_start": s.vel[lo][2], "vz_end": s.vel[hi - 1][2],
            "vz_mean": mean(v[2] for v in s.vel[lo:hi]),
            "h_speed_end": math.hypot(s.vel[hi - 1][0], s.vel[hi - 1][1]),
            "dshot_mean": [mean(d[i] for d in s.dshot[lo:hi]) for i in range(4)],
            "dshot_min": min(min(d) for d in s.dshot[lo:hi]), "dshot_max": max(max(d) for d in s.dshot[lo:hi]),
            "cmd_tau_max": [max(abs(c[r]) for c in s.cmd[lo:hi]) for r in (1, 2, 3)],
            "wr_tau_max": [max(abs(c[r]) for c in s.wrench[lo:hi]) for r in (1, 2, 3)],
            "rate_max": max(math.sqrt(sum(x * x for x in o)) for o in s.omega[lo:hi]),
            "est_err_mean": mean(s.est_err[lo:hi]), "est_err_max": max(s.est_err[lo:hi]),
            "acc_tilt_mean": mean(accel_tilt(a) for a in s.accel[lo:hi]),
            "acc_norm_mean": mean(math.sqrt(sum(x * x for x in a)) for a in s.accel[lo:hi])}


def step_stats(s, p, release_us):
    """Setpoint error of a roll or pitch step, over the stick-frame window of the phase."""
    lo, hi = window(s, p.start_us, p.end_us)
    # the firmware sees the stick at its frame, the setpoint series follows the logged frames
    err = [math.hypot(s.tvec[i][0] - s.sp[i][0], s.tvec[i][1] - s.sp[i][1]) for i in range(lo, hi)]
    ax = 0 if p.roll else 1
    target = s.sp[hi - 1][ax]
    comp = [s.tvec[i][ax] for i in range(lo, hi)]
    steady_lo, _ = window(s, p.end_us - us(STEADY_WINDOW_S), p.end_us)
    steady = [math.hypot(s.tvec[i][0] - s.sp[i][0], s.tvec[i][1] - s.sp[i][1]) for i in range(steady_lo, hi)]
    steady_signed = [s.tvec[i][ax] - s.sp[i][ax] for i in range(steady_lo, hi)]
    cross = [s.tvec[i][1 - ax] for i in range(steady_lo, hi)]
    settle = None
    band = SETTLE_BAND * abs(target)
    for i in range(hi - 1, lo - 1, -1):
        if math.hypot(s.tvec[i][0] - s.sp[i][0], s.tvec[i][1] - s.sp[i][1]) > band:
            settle = s.t_us[min(i + 1, hi - 1)] - p.start_us if i < hi - 1 else None
            break
    else:
        settle = 0
    first = next((i for i in range(lo, hi) if abs(s.sp[i][ax]) > 0), lo)
    est_err_ss = [math.hypot(s.est_tvec[i][0] - s.sp[i][0], s.est_tvec[i][1] - s.sp[i][1]) for i in range(steady_lo, hi)]
    return {"est_setpoint_err_steady": mean(est_err_ss), "setpoint": target, "err_peak": max(err), "err_steady_mean": mean(steady), "err_steady_signed": mean(steady_signed),
            "err_steady_max": max(steady), "overshoot": max(abs(c) for c in comp) - abs(target),
            "comp_peak": max(comp, key=abs), "cross_steady": mean(cross), "settle_us": settle,
            "tilt_max": max(s.tilt[lo:hi]), "first_frame_us": s.t_us[first]}


def after_step_stats(s, p):
    lo, hi = window(s, p.start_us, p.end_us)
    steady_lo, _ = window(s, p.end_us - us(STEADY_WINDOW_S), p.end_us)
    ax_peak = max(math.hypot(*s.tvec[i]) for i in range(lo, hi))
    return {"peak_tilt_vec": ax_peak, "steady_tilt_mean": mean(s.tilt[i] for i in range(steady_lo, hi)),
            "steady_tilt_max": max(s.tilt[i] for i in range(steady_lo, hi))}


def dip_stats(s, dip, hold2, release_us):
    lo, hi = window(s, dip.start_us, dip.end_us)
    z0 = s.pos[lo][2]
    end2 = hold2.end_us
    _, hi2 = window(s, dip.start_us, end2)
    dt_end = (s.t_us[hi2 - 1] - s.t_us[lo]) / 1e6
    dz_end = s.pos[hi2 - 1][2] - z0  # NED down: positive = lower
    dz_extrap = s.pos[hi2 - 1][2] - (z0 + s.vel[lo][2] * dt_end)  # against continuing at the dip-start velocity
    tilt_dip = max(s.tilt[lo:hi])
    tilt_after = max(s.tilt[hi:hi2])
    tilt_all = max(s.tilt[lo:hi2])
    i_tilt = max(range(lo, hi2), key=lambda i: s.tilt[i])
    # first second after the dip
    h1 = next((i for i, t in enumerate(s.t_us) if t >= dip.end_us + us(1.0)), hi2)
    return {"z_start": z0, "vz_start": s.vel[lo][2], "dz_dip": s.pos[hi - 1][2] - z0, "dv_dip": s.vel[hi - 1][2] - s.vel[lo][2], "dz_end": dz_end, "dz_extrap": dz_extrap, "dt_end": dt_end, "z_at_dip_end": s.pos[hi - 1][2],
            "vz_at_dip_end": s.vel[hi - 1][2], "vz_max_down": max(v[2] for v in s.vel[lo:hi2]), "tilt_dip": tilt_dip,
            "tilt_after": tilt_after, "tilt_max": tilt_all, "t_tilt_max_us": s.t_us[i_tilt], "tilt_first_s_after": max(s.tilt[hi:h1]),
            "dshot_dip_min": min(min(d) for d in s.dshot[lo:hi]), "dshot_dip_max": max(max(d) for d in s.dshot[lo:hi]),
            "rotor_dip_min": min(min(r) for r in s.rotor[lo:hi]), "rotor_dip_max": max(max(r) for r in s.rotor[lo:hi]),
            "cmd_tau_dip_max": [max(abs(c[r]) for c in s.cmd[lo:hi]) for r in (1, 2, 3)],
            "cmd_tau_after_max": [max(abs(c[r]) for c in s.cmd[hi:h1]) for r in (1, 2, 3)],
            "wr_tau_after_max": [max(abs(c[r]) for c in s.wrench[hi:h1]) for r in (1, 2, 3)],
            "rate_max_dip": max(math.sqrt(sum(x * x for x in o)) for o in s.omega[lo:hi]),
            "rate_max_after": max(math.sqrt(sum(x * x for x in o)) for o in s.omega[hi:h1])}


def fmt_s(us_):
    return f"{us_ / 1e6:.4f} s"


def write_traces(path, s, release_us):
    cols = ["t_s", "tilt_deg", "tilt_x_deg", "tilt_y_deg", "sp_x_deg", "sp_y_deg", "heading_deg", "n_m", "e_m", "z_down_m",
            "vn_m_s", "ve_m_s", "vz_down_m_s", "wx_rad_s", "wy_rad_s", "wz_rad_s", "dshot1", "dshot2", "dshot3", "dshot4",
            "applied1", "applied2", "applied3", "applied4", "rotor1_rad_s", "rotor2_rad_s", "rotor3_rad_s", "rotor4_rad_s",
            "thrust_N", "tau_roll_Nm", "tau_pitch_Nm", "tau_yaw_Nm", "cmd_thrust_N", "cmd_tau_roll_Nm", "cmd_tau_pitch_Nm",
            "cmd_tau_yaw_Nm", "stick_roll", "stick_pitch", "stick_throttle", "stick_arm", "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2", "est_tilt_x_deg", "est_tilt_y_deg", "est_err_deg"]
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        for i in range(0, len(s.t_us), DECIMATE):
            row = [s.t_us[i] / 1e6, deg(s.tilt[i]), deg(s.tvec[i][0]), deg(s.tvec[i][1]), deg(s.sp[i][0]), deg(s.sp[i][1]),
                   deg(s.psi[i]), *s.pos[i], *s.vel[i], *s.omega[i], *s.dshot[i], *s.dshot_applied[i], *s.rotor[i],
                   *s.wrench[i], *s.cmd[i], *s.stick[i], *s.accel[i], deg(s.est_tvec[i][0]), deg(s.est_tvec[i][1]), deg(s.est_err[i])]
            f.write(",".join(f"{x:.7g}" if isinstance(x, float) else str(x) for x in row) + "\n")


def analyse(label, log, hover, phases, tilt_max, log_sha, out_stem):
    s = series_of(log, hover, tilt_max)
    pad = log["pad"]
    rel_tick = pad["release_tick"] if pad else None
    release_us = log["ticks"][rel_tick]["sil_t_us"] if rel_tick is not None else None
    L = []
    P = {p.name: p for p in phases}
    L.append(f"HIL S2e analysis: {label}")
    L.append("Simulation on INFERRED / UNVERIFIED card values (vehicles/marv_7in_fc3v2.yaml), not a flight result.")
    L.append(f"log sha256 {log_sha}")
    hdr = log["header"]
    L.append(f"tick {hdr['tick_period_num_us']}/{hdr['tick_period_den']} us; host step {log['sensors']['host_step_ns']} ns "
             f"({log['rounding']['error_ppm']:+.3f} ppm); command delay N = {log['delay']['ticks']}; ticks {len(log['ticks'])}")
    L.append(f"angle_tilt_max (fc3v2) {deg(tilt_max):.6f} deg; step stick {phases[3].roll} -> setpoint {deg(tilt_max * phases[3].roll):.4f} deg")
    L.append("")
    L.append("Phases (SIL time):")
    for p in phases:
        L.append(f"  {p.name:13s} {fmt_s(p.start_us)} .. {fmt_s(p.end_us)}  arm {p.arm:+.0f}  roll {p.roll}  pitch {p.pitch}  "
                 f"throttle {p.throttle!r}")
    L.append("")
    if rel_tick is None:
        L.append("PAD: never released within the run (the thrust never reached the weight).")
    else:
        L.append(f"PAD release (lift-off) at tick {rel_tick}, t = {fmt_s(release_us)}; "
                 f"{(release_us - P['hold'].start_us) / 1e3:.3f} ms after the throttle raise ({fmt_s(P['hold'].start_us)}).")
        L.append(f"  vertical thrust of the logged rotor speeds (k w^2) at the release tick {s.wrench[rel_tick][0]!r} N, at the tick before "
                 f"{s.wrench[rel_tick - 1][0]!r} N, the weight {hover.weight_n!r} N (the log's rotor_speed is the plant output of the tick after its step, "
                 "sim/gz/plugin/src/lockstep.cpp:1224; the release rule tests the state the tick's IMU sample sees, i.e. the previous tick's logged one)")
    arm_tick = next((t["tick"] for t in log["ticks"] if all(v >= DSHOT_MIN for v in t["dshot"])), None)
    if arm_tick is not None:
        L.append(f"first throttle command (armed) at tick {arm_tick}, t = {fmt_s(log['ticks'][arm_tick]['sil_t_us'])}")
    disarm = next((t["tick"] for t in log["ticks"] if t["sil_t_us"] >= P["disarmed"].start_us
                   and all(v == 0 for v in t["dshot"])), None)
    if disarm is not None:
        L.append(f"stop command after the disarm edge at tick {disarm}, t = {fmt_s(log['ticks'][disarm]['sil_t_us'])}")
    L.append("")
    L.append("Hover stick derivation:")
    L.append(f"  mass {hover.mass_kg} kg, g({gen_site()}) = {hover.g!r} m/s^2, weight {hover.weight_n!r} N, k_T {hover.k_t!r}, "
             f"omega range [{hover.omega_min}, {hover.omega_max}] rad/s, idle {hover.omega_idle}")
    L.append(f"  T_min {hover.t_min!r} N, T_max {hover.t_max!r} N, M_thrust {hover.m_thrust!r}; hover omega {hover.omega_h!r} rad/s, "
             f"D* {hover.d_star!r} (bracket {hover.d_lo}..{hover.d_hi})")
    L.append(f"  stick s = (sqrt(m g) - sqrt(T_min)) / (sqrt(T_max) - sqrt(T_min)) = {hover.stick!r}; as the script writes it (binary32) {hover.stick_f32!r}")
    L.append("")
    L.append("Per phase (tilt = angle between body z and NED z; from lift-off for the phases that contain it; z NED down, "
             "positive = lower):")
    L.append("  phase          from      to        tilt mean/max deg   z start..end m          vz start..end (mean) m/s       hspeed end  "
             "dshot mean (min..max)   |cmd tau| max r/p/y Nm")
    stats = {}
    for p in phases[2:]:
        st = phase_stats(s, p, release_us)
        stats[p.name] = st
        if st is None:
            L.append(f"  {p.name:13s} (empty)")
            continue
        L.append(f"  {p.name:13s} {st['from_us'] / 1e6:8.4f} {st['to_us'] / 1e6:8.4f}  {deg(st['tilt_mean']):7.3f} {deg(st['tilt_max']):8.3f}"
                 f"    {st['z_start']:9.3f} .. {st['z_end']:9.3f}   {st['vz_start']:8.3f} .. {st['vz_end']:8.3f} ({st['vz_mean']:8.3f})"
                 f"  {st['h_speed_end']:8.3f}   {'/'.join(f'{x:.0f}' for x in st['dshot_mean'])} ({st['dshot_min']}..{st['dshot_max']})"
                 f"   {st['cmd_tau_max'][0]:.4f}/{st['cmd_tau_max'][1]:.4f}/{st['cmd_tau_max'][2]:.4f}")
    L.append("")
    L.append("Accelerometer (the estimator's input) and a binary64 replica of the Mahony estimate (mahony_replica, analysis only): tilt implied by the specific force (angle between it and -z of the body), mean, and |f|/g mean "
             "(estimator_accel_healthy window "
             f"{register_value('estimator_accel_healthy_min')}..{register_value('estimator_accel_healthy_max')}), against the truth tilt mean:")
    for p in phases[2:]:
        st = stats[p.name]
        if st:
            L.append(f"  {p.name:13s} accel-implied tilt {deg(st['acc_tilt_mean']):7.3f} deg, |f|/g {st['acc_norm_mean'] / hover.g:6.3f}, truth tilt {deg(st['tilt_mean']):7.3f} deg; "
                     f"replica estimate vs truth (angle between the body z axes) mean {deg(st['est_err_mean']):7.3f} deg, max {deg(st['est_err_max']):7.3f} deg")
    L.append("")
    hold = stats["hold"]
    if hold:
        dur = (hold["to_us"] - hold["from_us"]) / 1e6
        L.append(f"Hold 1 (from lift-off to the end of the phase, {dur:.3f} s): tilt mean {deg(hold['tilt_mean']):.4f} deg, max {deg(hold['tilt_max']):.4f} deg; "
                 f"vertical position {hold['z_start']:.4f} -> {hold['z_end']:.4f} m (down), velocity {hold['vz_start']:.4f} -> {hold['vz_end']:.4f} m/s; "
                 f"mean vertical acceleration {(hold['vz_end'] - hold['vz_start']) / dur:.4f} m/s^2 (open-loop throttle: drift expected)")
    L.append("")
    for name in ("roll_step", "pitch_step"):
        p = P[name]
        st = step_stats(s, p, release_us)
        ax = "roll (x)" if p.roll else "pitch (y)"
        a2 = after_step_stats(s, P[name.replace("step", "centre")])
        c = P[name.replace("step", "centre")]
        cs = stats[c.name]
        L.append(f"{name}: {ax} stick {p.roll or p.pitch} for {STEP_S} s; setpoint {deg(st['setpoint']):.4f} deg (heading frame, truth heading); "
                 f"error = |truth tilt vector - setpoint|: peak {deg(st['err_peak']):.4f} deg, steady (last {STEADY_WINDOW_S} s) mean {deg(st['err_steady_mean']):.4f} "
                 f"(signed on the step axis {deg(st['err_steady_signed']):+.4f}, max {deg(st['err_steady_max']):.4f}); cross-axis steady {deg(st['cross_steady']):+.4f}; "
                 f"overshoot of the axis component {deg(st['overshoot']):+.4f} deg (peak {deg(st['comp_peak']):+.4f}); "
                 f"within {SETTLE_BAND:.0%} of the setpoint from {'never (not within the band at the end of the step)' if st['settle_us'] is None else f'{st['settle_us'] / 1e3:.1f} ms after the phase start'}; "
                 f"tilt max {deg(st['tilt_max']):.4f} deg; replica-estimate setpoint error, last {STEADY_WINDOW_S} s mean {deg(st['est_setpoint_err_steady']):.4f} deg")
        L.append(f"  {c.name} ({STEP_S}..{STEP_S + CENTRE_S} s after): tilt mean {deg(cs['tilt_mean']):.4f} deg, max {deg(cs['tilt_max']):.4f} deg; steady (last "
                 f"{STEADY_WINDOW_S} s) tilt mean {deg(a2['steady_tilt_mean']):.4f}, max {deg(a2['steady_tilt_max']):.4f} deg; "
                 f"z {cs['z_start']:.3f} -> {cs['z_end']:.3f} m, vz {cs['vz_start']:.3f} -> {cs['vz_end']:.3f} m/s, horizontal speed at the end {cs['h_speed_end']:.3f} m/s")
        ps = stats[p.name]
        L.append(f"  {p.name} z {ps['z_start']:.3f} -> {ps['z_end']:.3f} m (down), vz {ps['vz_start']:.3f} -> {ps['vz_end']:.3f} m/s, "
                 f"horizontal speed at the end {ps['h_speed_end']:.3f} m/s")
    L.append("")
    d = dip_stats(s, P["dip"], P["hold2"], release_us)
    L.append(f"dip ({DIP_S} s, throttle stick 0, below arm_throttle_min {register_value('arm_throttle_min')!r}):")
    L.append(f"  z at dip start {d['z_start']:.4f} m, vz {d['vz_start']:.4f} m/s; at the dip end z {d['z_at_dip_end']:.4f} m (down {d['z_at_dip_end'] - d['z_start']:+.4f}), "
             f"vz {d['vz_at_dip_end']:.4f} m/s; altitude lost during the dip window {d['dz_dip']:.4f} m, descent speed gained {d['dv_dip']:+.4f} m/s "
             f"(the throttle returns to hover, the open-loop throttle does not take the speed back); by the end of hold2 ({d['dt_end']:.2f} s after the dip start) "
             f"{d['dz_end']:.3f} m lower than at the dip start, {d['dz_extrap']:.3f} m lower than continuing at the dip-start velocity; max downward speed {d['vz_max_down']:.4f} m/s")
    L.append(f"  tilt max in the dip {deg(d['tilt_dip']):.4f} deg, after it (to the end of hold2) {deg(d['tilt_after']):.4f} deg, overall {deg(d['tilt_max']):.4f} deg at "
             f"{fmt_s(d['t_tilt_max_us'])}; max in the first second after the dip {deg(d['tilt_first_s_after']):.4f} deg")
    L.append(f"  DShot during the dip {d['dshot_dip_min']}..{d['dshot_dip_max']} (the SIL's command); rotor speed during the dip (plant, after the "
             f"N-tick delay and the motor lag) {d['rotor_dip_min']:.2f}..{d['rotor_dip_max']:.2f} rad/s; |body rate| max in the dip {d['rate_max_dip']:.4f} rad/s, "
             f"in the first second after {d['rate_max_after']:.4f} rad/s")
    L.append(f"  commanded torque (mixer DShot through the ESC map and B) max |roll/pitch/yaw|: in the dip {d['cmd_tau_dip_max'][0]:.5f}/"
             f"{d['cmd_tau_dip_max'][1]:.5f}/{d['cmd_tau_dip_max'][2]:.5f} Nm, in the first second after {d['cmd_tau_after_max'][0]:.5f}/"
             f"{d['cmd_tau_after_max'][1]:.5f}/{d['cmd_tau_after_max'][2]:.5f} Nm; plant torque in the first second after "
             f"{d['wr_tau_after_max'][0]:.5f}/{d['wr_tau_after_max'][1]:.5f}/{d['wr_tau_after_max'][2]:.5f} Nm")
    L.append("  The rate integrator is a firmware internal and is not in the log; only the outputs above are observable.")
    L.append("")
    tilt_all = max(s.tilt)
    L.append(f"overall: tilt max {deg(tilt_all):.4f} deg at {fmt_s(s.t_us[s.tilt.index(tilt_all)])}; final z {s.pos[-1][2]:.3f} m (down), "
             f"final speed {math.sqrt(sum(v * v for v in s.vel[-1])):.3f} m/s")
    # tilt at 1 s intervals
    L.append("tilt at 1 s intervals (deg): " + " ".join(
        f"{t:.0f}s:{deg(s.tilt[next(i for i, x in enumerate(s.t_us) if x >= us(t))]):.2f}"
        for t in range(0, int(s.t_us[-1] / 1e6) + 1) if any(x >= us(t) for x in s.t_us[-1:])))
    text = "\n".join(L) + "\n"
    Path(f"{out_stem}.analysis.txt").write_text(text, encoding="utf-8")
    write_traces(f"{out_stem}.traces.csv", s, release_us)
    return text


def band_return(s, i0, i1, vals, pre_lo, pre_hi, peak_excursion, pre_mean):
    """(half-width, seconds after index i0's time of the last sample outside the band, None if the band is not reached by i1)."""
    half = max(pre_hi - pre_mean, pre_mean - pre_lo, IHOLD_RETURN_FRACTION * peak_excursion)
    out = [i for i in range(i0, i1) if abs(vals[i] - pre_mean) > half]
    if not out:
        return half, 0.0
    if out[-1] == i1 - 1:
        return half, None
    return half, (s.t_us[out[-1] + 1] - s.t_us[i0]) / 1e6


def analyse_ihold(label, log, hover, phases, tilt_max, log_sha, out_stem, dip_stick, com_offset_m):
    s = series_of(log, hover, tilt_max)
    pad = log["pad"]
    rel_tick = pad["release_tick"] if pad else None
    release_us = log["ticks"][rel_tick]["sil_t_us"] if rel_tick is not None else None
    P = {p.name: p for p in phases}
    dip, hold2 = P["dip"], P["hold2"]
    rate = [math.sqrt(sum(x * x for x in o)) for o in s.omega]
    L = [f"HIL S2f integrator-hold analysis: {label}",
         "Simulation on INFERRED / UNVERIFIED / PLACEHOLDER card values (vehicles/marv_7in_fc3v2.yaml), not a flight result.",
         f"log sha256 {log_sha}",
         f"tick {log['header']['tick_period_num_us']}/{log['header']['tick_period_den']} us; command delay N = {log['delay']['ticks']}; ticks {len(log['ticks'])}",
         f"disturbance: every rotor position shifted by {com_offset_m!r} m on x and y (plant torque tau_x = -T d, tau_y = +T d); at hover T = {hover.weight_n!r} N: "
         f"|tau| = {hover.weight_n * com_offset_m:.5f} Nm per axis",
         f"dip: throttle stick {dip_stick!r} for {DIP_S} s; arm_throttle_min {register_value('arm_throttle_min')!r} (quad_fc.cpp:439 throttle_low = throttle < g_throttle_min): "
         f"the decision 0045 hold {'ENGAGES' if dip_stick < register_value('arm_throttle_min') else 'does NOT engage'}", "", "Phases (SIL time):"]
    for p in phases:
        L.append(f"  {p.name:9s} {fmt_s(p.start_us)} .. {fmt_s(p.end_us)}  arm {p.arm:+.0f}  throttle {p.throttle!r}")
    L.append(f"PAD release (lift-off) at {fmt_s(release_us) if release_us is not None else 'never'}")
    L.append("")
    i0, i1 = window(s, dip.start_us, dip.end_us)  # the dip window
    ip0, _ = window(s, dip.start_us - us(IHOLD_PRE_S), dip.start_us)  # the pre-dip window [ip0, i0)
    _, ie = window(s, dip.start_us, hold2.end_us)  # the dip start to the end of hold2
    tilt = [deg(x) for x in s.tilt]
    pre_t, pre_r = tilt[ip0:i0], rate[ip0:i0]
    pt_mean, pr_mean = mean(pre_t), mean(pre_r)
    tx = [deg(v[0]) for v in s.tvec]
    ty = [deg(v[1]) for v in s.tvec]
    L.append(f"Pre-dip (the last {IHOLD_PRE_S} s before the dip, {fmt_s(s.t_us[ip0])} .. {fmt_s(s.t_us[i0 - 1])}):")
    L.append(f"  tilt mean {pt_mean:.4f} deg (min {min(pre_t):.4f}, max {max(pre_t):.4f}); heading-frame tilt vector x (roll) {mean(tx[ip0:i0]):+.4f}, y (pitch) {mean(ty[ip0:i0]):+.4f} deg")
    L.append(f"  |body rate| mean {pr_mean:.5f} rad/s (min {min(pre_r):.5f}, max {max(pre_r):.5f})")
    L.append(f"  commanded torque (mixer DShot through the ESC map and B), mean roll/pitch/yaw {mean(c[1] for c in s.cmd[ip0:i0]):+.5f}/{mean(c[2] for c in s.cmd[ip0:i0]):+.5f}/"
             f"{mean(c[3] for c in s.cmd[ip0:i0]):+.5f} Nm; plant torque mean {mean(c[1] for c in s.wrench[ip0:i0]):+.5f}/{mean(c[2] for c in s.wrench[ip0:i0]):+.5f}/"
             f"{mean(c[3] for c in s.wrench[ip0:i0]):+.5f} Nm (the integrator is not in the log; the commanded torque against the disturbance torque above is its trace)")
    L.append(f"  z {s.pos[i0][2]:.4f} m (down), vz {s.vel[i0][2]:.5f} m/s at the dip start")
    L.append("")
    it = max(range(i0, ie), key=lambda i: tilt[i])
    ir = max(range(i0, ie), key=lambda i: rate[i])
    ihi = max(range(i1, ie), key=lambda i: tilt[i])  # peak after the dip window
    irhi = max(range(i1, ie), key=lambda i: rate[i])
    th, tr = band_return(s, i1, ie, tilt, min(pre_t), max(pre_t), tilt[it] - pt_mean, pt_mean)
    rh, rr = band_return(s, i1, ie, rate, min(pre_r), max(pre_r), rate[ir] - pr_mean, pr_mean)
    L.append(f"After the dip start (to the end of hold2, {(hold2.end_us - dip.start_us) / 1e6:.2f} s):")
    L.append(f"  peak tilt {tilt[it]:.4f} deg at {fmt_s(s.t_us[it])} (in the dip window: max {max(tilt[i0:i1]):.4f}); peak after the dip window {tilt[ihi]:.4f} deg at {fmt_s(s.t_us[ihi])}; "
             f"heading-frame x/y at the peak {tx[it]:+.4f}/{ty[it]:+.4f} deg")
    L.append(f"  peak |body rate| {rate[ir]:.5f} rad/s at {fmt_s(s.t_us[ir])} (in the dip window: max {max(rate[i0:i1]):.5f}); peak after the dip window {rate[irhi]:.5f} rad/s at {fmt_s(s.t_us[irhi])}")
    L.append(f"  return rule: the last sample outside the band pre-dip mean +- max(pre-dip envelope, {IHOLD_RETURN_FRACTION:.0%} of the peak excursion), measured from the dip end ({fmt_s(dip.end_us)})")
    L.append(f"  tilt returns within +-{th:.4f} deg of {pt_mean:.4f}: " + ("never within the run" if tr is None else f"{tr:.3f} s after the dip end"))
    L.append(f"  |body rate| returns within +-{rh:.5f} rad/s of {pr_mean:.5f}: " + ("never within the run" if rr is None else f"{rr:.3f} s after the dip end"))
    z0 = s.pos[i0][2]
    zs = {}
    for name, t_us in (("dip end", dip.end_us), ("dip end + 1 s", dip.end_us + us(1.0)), ("end of hold2", hold2.end_us)):
        j = min(next((i for i, x in enumerate(s.t_us) if x >= t_us), len(s.t_us) - 1), len(s.t_us) - 1)
        dt = (s.t_us[j] - s.t_us[i0]) / 1e6
        zs[name] = (s.pos[j][2] - z0, s.pos[j][2] - z0 - s.vel[i0][2] * dt, s.vel[j][2])
    L.append("  altitude lost (down positive, relative to the dip start / to continuing at the dip-start velocity; the throttle is open-loop and the pre-dip descent speed is "
             f"{s.vel[i0][2]:.5f} m/s): " + "; ".join(f"{k}: {v[0]:.3f} / {v[1]:.3f} m (vz {v[2]:.3f} m/s)" for k, v in zs.items()))
    L.append(f"  DShot during the dip {min(min(d) for d in s.dshot[i0:i1])}..{max(max(d) for d in s.dshot[i0:i1])}; rotor speed in the dip "
             f"{min(min(r) for r in s.rotor[i0:i1]):.2f}..{max(max(r) for r in s.rotor[i0:i1]):.2f} rad/s; stick throttle in the dip (logged frame) {min(x[2] for x in s.stick[i0:i1])!r}..{max(x[2] for x in s.stick[i0:i1])!r}")
    for tag, a, b in (("in the first second after the dip", i1, next((i for i, x in enumerate(s.t_us) if x >= dip.end_us + us(1.0)), ie)),):
        L.append(f"  commanded torque max |roll/pitch/yaw| {tag}: {max(abs(c[1]) for c in s.cmd[a:b]):.5f}/{max(abs(c[2]) for c in s.cmd[a:b]):.5f}/{max(abs(c[3]) for c in s.cmd[a:b]):.5f} Nm; "
                 f"plant torque {max(abs(c[1]) for c in s.wrench[a:b]):.5f}/{max(abs(c[2]) for c in s.wrench[a:b]):.5f}/{max(abs(c[3]) for c in s.wrench[a:b]):.5f} Nm")
    L.append("")
    L.append("tilt (deg) at 0.5 s steps from 1 s before the dip: " + " ".join(
        f"{(t_us - dip.start_us) / 1e6:+.1f}s:{tilt[next(i for i, x in enumerate(s.t_us) if x >= t_us)]:.3f}"
        for t_us in range(dip.start_us - us(1.0), hold2.end_us + 1, us(0.5)) if t_us <= s.t_us[-1]))
    text = "\n".join(L) + "\n"
    Path(f"{out_stem}.analysis.txt").write_text(text, encoding="utf-8")
    write_traces(f"{out_stem}.traces.csv", s, release_us)
    return text


def gen_site():
    sc = scn.values(scn.load(BASE_SCENARIO, CARD))
    return f"phi {sc['site_latitude_rad']} rad, h {sc['site_height_m']} m"


# ---- running ----------------------------------------------------------------------------------------------------------

def tilt_max_param():
    f = next(ROOT.glob("build/host-gz-quadfc/generated/marv_params_fc3v2/marv/params/param_defaults.cpp"))
    text = f.read_text(encoding="utf-8")
    m = re.search(r"// angle_tilt_max\n\s*\{\{ParamType::F32, ([0-9.eE+-]+)f", text)
    assert m, "angle_tilt_max not found in the generated fc3v2 defaults"
    return float(m.group(1))


class RebootRequested(Exception):
    """gz exited with EX_TEMPFAIL: the plugin's "reboot requested" (module docstring)."""


def gz_exit_status(error):
    """The gz exit status a run_scenario.RunError reports (run_scenario.check_exit: "gz exited <status> (..."), else None."""
    m = re.match(r"gz exited (-?\d+) ", str(error))
    return int(m.group(1)) if m else None


def run_one(label, out_dir, plugin_dir, overrides, max_s, step_stick, ihold_dip_stick=None, com_offset_m=0.0, link=""):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    assert register_value("rc_channel_arm") == ARM_CHANNEL and register_value("rc_channel_angle") == ANGLE_CHANNEL
    hover = derive_hover()
    phases = build_phases(hover, step_stick) if ihold_dip_stick is None else build_phases_ihold(hover, ihold_dip_stick)
    scenario, num, den = fc3_scenario(out_dir)
    total_us = phases[-1].end_us if max_s is None else min(phases[-1].end_us, us(max_s))
    iterations = -(-total_us * den // num)  # ticks (= host steps at m = 1) that reach total_us
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    try:
        r = run_scenario.run(CARD, scenario, SEED, M, "test", "lo", out_dir, plugin_dir,
                             sdf_edit_for(phases, overrides, com_offset_m, link), iterations=iterations, timeout_s=TIMEOUT_S,
                             sensors=sensors, command_delay_ticks=COMMAND_DELAY_TICKS)
    except run_scenario.RunError as e:
        if link and gz_exit_status(e) == os.EX_TEMPFAIL:
            raise RebootRequested(str(e)) from e
        raise
    stem = str(Path(r.log_path).with_suffix(""))
    if ihold_dip_stick is None:
        text = analyse(label, r.log, hover, phases, tilt_max_param(), r.log_sha256, stem)
    else:
        text = analyse_ihold(label, r.log, hover, phases, tilt_max_param(), r.log_sha256, stem, ihold_dip_stick, com_offset_m)
    print(text)
    print(f"files: {stem}.{{sdf,bin,report.txt,analysis.txt,traces.csv}}; wall {r.wall_s:.1f} s, RTF {r.rtf:.3f}")
    return r


def run_interactive(out_dir, plugin_dir, link):
    """The interactive run (module docstring): writes the world and runs gz until it ends or is stopped; returns gz's exit
    status."""
    out_dir = Path(out_dir) / "interactive"
    out_dir.mkdir(parents=True, exist_ok=True)
    assert register_value("rc_channel_arm") == ARM_CHANNEL and register_value("rc_channel_angle") == ANGLE_CHANNEL
    scenario, num, den = fc3_scenario(out_dir)
    total_us = us(INTERACTIVE_S)
    iterations = -(-total_us * den // num)  # ticks (= host steps at m = 1) that reach total_us
    phases = [Phase("idle", 0, total_us, SWITCH_LOW)]
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    name, text = gen_world.generate(CARD, scenario, "pilot", M, "lo", None, ROOT, sensors=sensors,
                                    command_delay_ticks=COMMAND_DELAY_TICKS)
    world = out_dir / name
    world.write_text(sdf_edit_for(phases, (), 0.0, link, ground=True)(text), encoding="utf-8")
    cmd = [*run_scenario.GZ_ARGS, "-s", "-r", "--iterations", str(iterations), "--seed", str(SEED), str(world)]
    print(f"interactive run: {world} (real-time factor 1; until stopped, at most {INTERACTIVE_S:.0f} s)", flush=True)
    return subprocess.run(cmd, env=run_scenario.gz_env(plugin_dir), check=False).returncode


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--plugin-dir", default=str(PLUGIN_DIR))
    ap.add_argument("--runs", choices=("flight", "control", "both"), default="both")
    ap.add_argument("--step-stick", type=float, default=STEP_STICK, help="roll and pitch step stick (a scenario value, default %(default)s)")
    ap.add_argument("--script", choices=("s2e", "ihold"), default="s2e",
                    help="s2e: the roll / pitch step script (default). ihold: hover with a constant disturbance torque and one throttle dip; "
                         "--runs flight = the dip to stick 0 (the integrator hold engages), control = the dip to arm_throttle_min + 2/1024 (it does not)")
    ap.add_argument("--com-offset-m", type=float, default=IHOLD_COM_OFFSET_M,
                    help="ihold only: the rotor-position shift on x and y, m, the disturbance torque is thrust x this (a PLACEHOLDER scenario value, default %(default)s)")
    ap.add_argument("--max-s", type=float, default=None, help="end the run early at this SIM time (debugging only)")
    ap.add_argument("--link-udp-port", type=int, default=None,
                    help="the SIL's ground-link UDP port on 127.0.0.1 (<link_udp_port>, a scenario value); default: no element")
    ap.add_argument("--config-dir", default=None,
                    help="the SIL's config storage directory (<config_dir>; created if missing); default: no element")
    ap.add_argument("--interactive", action="store_true",
                    help="one real-time run, arm switch low, no log and no analysis, until stopped (module docstring)")
    a = ap.parse_args(argv)
    config_dir = None
    if a.config_dir is not None:
        config_dir = Path(a.config_dir).resolve()
        config_dir.mkdir(parents=True, exist_ok=True)
    link = link_xml(a.link_udp_port, config_dir)
    if a.interactive:
        status = run_interactive(a.out_dir, a.plugin_dir, link)
        if status == os.EX_TEMPFAIL:
            print("run_quad_fc: reboot requested (gz exit EX_TEMPFAIL); relaunch on the same config dir", file=sys.stderr)
            return os.EX_TEMPFAIL
        return 0 if status == 0 else 1
    try:
        if a.script == "ihold":
            if a.runs in ("flight", "both"):
                run_one("integrator hold engaged (dip to stick 0)", Path(a.out_dir) / "dip_below_mincheck", a.plugin_dir, (), a.max_s, a.step_stick,
                        DIP_BELOW_STICK, a.com_offset_m, link)
            if a.runs in ("control", "both"):
                run_one("integrator hold not engaged (dip to arm_throttle_min + 2/1024)", Path(a.out_dir) / "dip_above_mincheck", a.plugin_dir, (), a.max_s,
                        a.step_stick, dip_above_stick(), a.com_offset_m, link)
            return 0
        if a.runs in ("flight", "both"):
            run_one("flight run (angle mode, the loop closed)", Path(a.out_dir) / "flight", a.plugin_dir, (), a.max_s, a.step_stick,
                    link=link)
        if a.runs in ("control", "both"):
            run_one("control run (att_kp = 0, rate_ki_roll = rate_ki_pitch = 0: the attitude loop disabled)",
                    Path(a.out_dir) / "control", a.plugin_dir, CONTROL_OVERRIDES, a.max_s, a.step_stick, link=link)
    except RebootRequested as e:
        print(f"run_quad_fc: reboot requested (gz exit EX_TEMPFAIL); no analysis; relaunch on the same config dir\n{e}",
              file=sys.stderr)
        return os.EX_TEMPFAIL
    return 0


if __name__ == "__main__":
    sys.exit(main())
