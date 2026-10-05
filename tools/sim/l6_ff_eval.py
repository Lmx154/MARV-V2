#!/usr/bin/env python3
"""L6 stage (c) omega x J omega feed-forward evaluation: a T3 design-model run of the L4 acro combined full-stick segment
(quad spec L6 stage (c); decision 0014 owner decisions 1-4 and "the design", omega x J omega evaluation; decision 0005 owner
decision 12, the acro recovery known failing item). Reported, not asserted (owner decision 2).

    uv run python tools/sim/l6_ff_eval.py --out <file> [--corners all | none | <i>,<j>,...] [--sensitivity] [--jobs <n>]

--corners picks the physical J corners (1-based, in rate_lead.physical_corners order) besides the card plant; default all.
--sensitivity adds the card plant under the T4 (c) and L4 sensors (below), each judged against its own Z and F.

Question. Does the stage (c) rate loop, with omega x J omega feed-forward, pass the L4 acro recovery predicate
(tests/regression/quad/L04/gz/test_t4_acro.py, recovery) on the card plant (owner decision 4: acro closes there), and by how
much does it exceed it at the physical J corners (owner decision 3; the excess is carried to the pre-L8 gate)?

Variants (decision 0014 "the design"; the law is fw/rate/include/marv/rate/rate_loop.hpp step()):
  PID          rate_lead's N* design: per-axis f32 kp, ki, kd, the D low-pass T_f; FF off (inertia 0).
  PID+FF       plain feed-forward u += g, g = w x (J w) on the chain output (motor_tau 0).
  PID+FF+lag   lag-compensated u += g + tau_m gdot, gdot through the first-order T_ff (owner decision 1, the chosen form).
The firmware side knows only the card: J = f32 of the card's inertia_diag, tau_m = f32 of rotors.motor_lag.tau, T_ff and
the gains from tools/card/rate_lead.py (design()). tau_ref, which rate_lead does not set, is the existing rule applied to
the new loop (lead decision, 2026-10-01): rate.py rule step 9 with rate_lead's tau_cl (its rule step 11),
tau_ref_a = r32_up(max(tau_cl, rate_max_a / alpha_max_a)), alpha_max_a rate.py's.

Plants: the card (J0) and every physical J corner of rate_lead.physical_corners(J0, b_J) (owner decision 3); the motor
lag tau is the card's at every plant (the corners are J only).

Nonlinear model (T3). Per tick T_s (scenario tick, 625/4 us), m = 1 (one tick per host step, as the m = 1 gz run):
  1. sensor: the truth body rate of this tick, delayed by `latency` ticks, through the chain's stages, each the
     firmware's direct form I (gyro_chain.hpp step()), seeded at rest (zero state, the steady state of the zero rate at
     t = 0). Three configurations (setup()):
       T3   the worst case, the evaluation's: latency_samples of the sensor profile (1), the chain at rate_lead's
            operating point, every notch fixed at omega_th on all four motors (gyro_chain_design.chain_stages; the
            largest phase lag, decision 0014 "Feasibility"), and the low-pass;
       T4c  the stage (c) T4 configuration (decision 0014 "Wiring"): truth gyro (latency 0), every notch bypassed, the
            low-pass only;
       L4   the L4 law's sensor as flown in acro_cause: truth gyro, no chain, latency 0 (the cross-check; the PI law);
       T4e  the stage (e) flight configuration (decision 0014 "Wiring"): latency_samples, the low-pass, and the 12 notches
            tracking the rotors: at each rate execution, before that tick's filter, update_notches (gyro_chain.hpp, in
            double) from the plant's rotor-speed telemetry (rotor_speed_model.hpp, in double: the speed after the last
            plant step, its electrical period on the profile's eee mmmmmmmmm grid of 1 us units with pole_count poles,
            truncated, decoded and rounded to float, delayed by latency_rate_periods x rate_loop_divisor ticks; stopped
            reads 0, so its notches bypass below omega_th); every DF1 history runs through bypassed stages. Not modelled,
            as in the plant's model: the ESC clock error and Betaflight's 100-eRPM rounding. Its linear design model
            (Z, F) holds every notch at omega_hover (the trim the linear model is taken about);
  2. firmware, on the ticks the rate group is due (every rate_loop_divisor ticks, tick 0 included), the composition's
     tick (fw/compositions/l4_rate_scripted): setpoint = the scenario's script at the stamp (run_l4.acro_setpoints), the
     rate law (a double mirror of rate_loop.hpp step(): seeding, deferred forward-Euler I, D on the measurement through
     T_f, the FF term), the mixer (a double mirror of mixer.hpp allocate(): air-mode priority roll/pitch > yaw >
     collective, at the hover collective request run_l4.plan_acro thrust_n), record_allocation's freeze, and
     thrust_to_dshot (rounded, clamped to [ceil(D(omega_idle)), kDshotThrottleMax]); the DShot is held to the next write;
     dt_n = the stamp difference (312 / 313 us) in seconds, as the oracle (1e-6 per us);
  3. plant (sim/plant/src/plant_model.hpp): each rotor's speed follows the ESC map's command through the first-order lag
     tau, exact ZOH over the tick; thrust k W^2 and torque B k W^2 (card geometry, spin signs, torque ratio) from the
     speeds at the end of the tick (marv_plant_step advances the motors, then computes the wrench gz applies over the
     step); the rigid body J w' = tau - w x (J w) (Euler's equations in principal axes, J the plant's diagonal) by RK4,
     RK_SUBSTEPS steps per tick, from rest; the rotors start at rest (acro.yaml gives no rotor speeds).
  The judged rate w_a(n) is the truth body rate at execution n's tick (the T4 test's gyro sample, truth gyro).
  Hooks of stage (e) (decision 0019, the noise suite memo section 2, commit C5), each off by default, and off this
  evaluation's runs are unchanged (the committed records reproduce byte for byte):
    bias         a gyro bias per axis (rad/s), added to the sample after the latency and before the chain (the plant IMU
                 model's order, sim/plant/src/imu_model.hpp); the chain is seeded with the first sample at tick 0
                 (rate_group.cpp seed_first_sample; at rest without a bias that is the zero state above);
    clock_error  e, the IMU's relative clock error: the plant's tick is t_nom / (1 + e), the firmware's stamps and the
                 chain's coefficients stay nominal (decision 0019 item 2: fw/ never sees e);
    torque       per axis and execution, a torque (N m) added to the request before allocate (rate_group.cpp finish:
                 request = the law's torque + added), as the L4 chirp's.
  step_plan adapts an L4 step scenario (run_l4.plan) to the plan and setpoints simulate reads.

Predicate (test_t4_acro.py, recovery): per axis, |w_a(n)| <= Z_a(n) + F_a + E_a(n) at every execution n of the recovery
window (run_l4.acro_phases), run_l4.recovery_evaluation. E_a = 0: E is the difference of two gz runs (m = 1, m = 2), which
this model does not have; E >= 0, so E = 0 makes the check stricter. Z_a = run_l4.rest_bound of the envelope (lo, hi) of
the linear design model driven by the script from rest over the 17 x 17 tau x J grid of the band box, F_a = TOL_a + H_a.
  PI (the cross-check): the linear design model below (linear_response) of the PI law at the L4 sensor, on the same
  grid, F_a = H_a (TOL_a = 0, as below); its P_a (the largest |w| of the envelope) is the one test_t4_acro.design_bound
  gave on the acro_cause build (cause.txt), where the T3 oracle and run_l4.script_response were the PI law's.
  Stage (c) law, per sensor configuration: the linear design model of the same loop without the coupling
  (linear_response): per axis J w' = u_m, tau u_m' = u - u_m, exact ZOH per tick, the same sensor (latency and chain)
  and the same law with FF off (the coupling it cancels is not in the linear model); grid and bands as the test (fixture
  f32 J, tau and bands); H_a the largest change of lo, hi between the 9 x 9 subset and the 17 x 17 grid (the 9 x 9 points
  are exactly the even-index 17 x 17 points); TOL_a = 0: no float32 rounding bound is derived here for these sensors
  (the T3 oracle's covers its own configuration); TOL >= 0, so 0 is stricter.

Cross-check (PI, the L4 law as flown in acro_cause: truth gyro, no chain, no latency; the f32 gains and tau_ref of
tools/card/rate.py, rate_lead.design()["pi_l4"], the L4 PI reference that gave the L4 product parameters then): the
model's recovery counts and worst excess against decision 0005's measured table (the m = 1 and m = 2 gz runs, E
included), and its rates against the m = 1 gz trace of acro_cause/cause.txt ("roll trace every 100 executions"). The
linear model is checked against the test's run_l4.script_response bit for bit (the stage (c) law of the L4 T3 fixture,
the fixture's chain low-pass as the T3 oracle computes it (Setup.lowpass) as the sensor, nominal J and tau, each axis).

Inputs (none retyped; the output lists each with its SHA-256): the card, design/budget.yaml, design/scenario_values.yaml
and the sensor profile (rate_lead.design), scenarios/quad/L04/acro.yaml, the L4 T3 fixture, the l4_rate_scripted
register (segment capacity), cause.txt and decision 0005 (cross-check). Method constants below set resolution only.
Run times go to stderr; the output is deterministic and does not depend on --jobs.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import math
import multiprocessing
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (HERE, HERE.parent / "card", ROOT / "tests" / "regression" / "quad" / "L04" / "gz",
           ROOT / "tests" / "regression" / "quad" / "L04" / "t3" / "reference"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import cpu_quota  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import mixer  # noqa: E402
import prim_constants  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402
import rate_t3_oracle as oracle  # noqa: E402
import run_l4  # noqa: E402
import schema  # noqa: E402
import test_t4_acro as t4  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO_VALUES = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
ACRO = t4.SCENARIO
FIXTURE = ROOT / "tests" / "regression" / "quad" / "L04" / "t3" / "reference" / "rate_t3_inputs.txt"
REGISTER = ROOT / "fw" / "compositions" / "l4_rate_scripted" / "params" / "l4_rate_scripted_register.yaml"
CAUSE = ROOT / "tests" / "regression" / "quad" / "L04" / "results" / "acro_cause" / "cause.txt"
DECISION_0005 = ROOT / "docs" / "decisions" / "0005-l4-rate-loop-choices.md"
AXES = ("roll", "pitch", "yaw")
INERTIA_KEYS = tuple(oracle.INERTIA[a] for a in AXES)
GRID = t4.HALVED  # the test's 17 x 17 grid, 2 (oracle.GRID - 1) + 1: its 9 x 9 grid is the even-index subset
MICROSECOND = oracle.MICROSECOND  # s per us, as the oracle and run_l4.script_response convert the stamps

# Method constants (resolution only).
RK_SUBSTEPS = 1  # RK4 steps of the rigid body per tick; the output gives the change at twice this on the card plant
TRACE_HEAD = "-- roll trace every"  # the cause.txt section the cross-check reads

VARIANTS = ("PID", "PID+FF", "PID+FF+lag")
SENSITIVITY = ("T4c", "L4", "T4e")  # the sensor keys of --sensitivity (setup())
# Simulation keys are (law, plant index, RK4 steps per tick, coupling, sensor key).
PI_RUN = ("PI", 0, RK_SUBSTEPS, True, "L4")  # the cross-check run
PI_GRID = ("PI", "L4")  # its Z grid: (law, sensor)
PI_UNCOUPLED = ("PI", 0, RK_SUBSTEPS, False, "L4")  # its control: the plant without w x J w


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path):
    return Path(path).resolve().relative_to(ROOT)


# ---- the model's parts ---------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Law:
    """The rate law's configuration (rate_loop.hpp RateConfig), per axis tuples, as doubles."""
    name: str
    kp: tuple
    ki: tuple
    kd: tuple
    tau_ref: tuple
    d_tau: tuple
    inertia: tuple = (0.0, 0.0, 0.0)
    motor_tau: float = 0.0
    ff_tau: float = 0.0

    @property
    def ff(self):
        return any(j > 0 for j in self.inertia)


@dataclasses.dataclass(frozen=True)
class Sensor:
    """The gyro path to the rate loop: latency in ticks, then the chain's non-identity stages (b0, b1, b2, a1, a2).
    With `tracking` (a Tracking) the 12 notches instead follow the telemetered rotor speeds and `stages` holds the
    low-pass alone; `linear` is then the fixed-stage Sensor of its linear design model."""
    name: str
    latency: int
    stages: tuple
    tracking: object = None
    linear: object = None


@dataclasses.dataclass(frozen=True)
class Tracking:
    """Notch tracking (gyro_chain.hpp update_notches) on the plant's rotor-speed telemetry (sim/plant/src/
    rotor_speed_model.hpp), in double: pole pairs, the word's exponent and mantissa bits, the period unit (s), the
    telemetry delay (ticks), the notch Q per harmonic, omega_th and the tick (s)."""
    pole_pairs: int
    exponent_bits: int
    mantissa_bits: int
    unit_s: float
    latency: int
    q: tuple
    omega_th: float
    period: float


def telemetry(omega, tr):
    """(valid, omega_hat) of one rotor (rotor_speed_model.hpp steps 1-4): the period on the eee mmmmmmmmm grid,
    truncated; at or beyond the all-ones word the rotor reads stopped (valid, 0); a period under one unit is invalid."""
    if not (math.isfinite(omega) and omega >= 0):
        return False, 0.0
    top = ((1 << tr.mantissa_bits) - 1) << ((1 << tr.exponent_bits) - 1)
    periods = 2 * math.pi / (omega * tr.pole_pairs * tr.unit_s) if omega > 0 else float(top)
    if not periods < top:
        return True, 0.0
    whole = math.floor(periods)
    if whole == 0:
        return False, 0.0
    shift = 0
    while (whole >> shift) >= (1 << tr.mantissa_bits):
        shift += 1
    period = (whole >> shift) << shift
    return True, rate.r32(2 * math.pi / (period * tr.unit_s * tr.pole_pairs))


def notch_set(rotors, tr, cache):
    """The 12 notch coefficient tuples of update_notches (motor-major, harmonic-minor) for the telemetered rotors
    [(valid, omega_hat)]: active iff valid, omega_hat >= omega_th and f0 < f_s/2, else the identity."""
    out = []
    for valid, om in rotors:
        for h, qh in zip(gcd.HARMONICS, tr.q):
            f0 = h * om / (2 * math.pi)
            if valid and om >= tr.omega_th and f0 < 1 / (2 * tr.period):
                key = (f0, qh)
                if key not in cache:
                    cache[key] = gcd.notch_coeffs(f0, qh, tr.period)
                out.append(cache[key])
            else:
                out.append(gcd.IDENTITY)
    return out


@dataclasses.dataclass(frozen=True)
class Plant:
    name: str
    inertia: tuple       # the plant's principal moments, kg m^2
    tau: float           # rotor lag, s
    k: float             # thrust coefficient, N/(rad/s)^2
    c_q: float           # torque ratio, m
    pos_x: tuple         # rotor x, FRD, m (index = logical motor - 1)
    pos_y: tuple
    spin: tuple          # yaw sign
    omega_min: float     # ESC map endpoints, rad/s
    omega_max: float
    idle: float          # mixer idle speed, rad/s
    m: tuple             # mixer M rows [thrust, roll, pitch, yaw] per motor
    abs_bm: tuple        # |B||M| rows roll, pitch, yaw (rate_loop.hpp init), columns thrust, roll, pitch, yaw
    d_min: int
    d_max: int
    coupling: bool = True


def round_half_away(x):
    """C round() for x >= 0: nearest integer, halves away from zero (x - floor(x) is exact in binary floating point)."""
    f = math.floor(x)
    return f + 1 if x - f >= 0.5 else f


def exec_dts(plan):
    """dt in seconds of every execution after the first (stamp differences)."""
    return [(plan.stamp_us(k) - plan.stamp_us(k - 1)) * MICROSECOND for k in range(1, plan.end_execution + 1)]


def simulate(law, sensor, plant, plan, sps, thrust, substeps=RK_SUBSTEPS, bias=None, clock_error=0.0, torque=None):
    """The nonlinear model (module docstring; bias, clock_error and torque are its stage (e) hooks, off by default).
    Returns (w per axis per execution, flagged executions per axis)."""
    d_div = plan.divisor
    n_exec = plan.end_execution + 1
    h = float(plan.tick_s) / (1 + clock_error) if clock_error else float(plan.tick_s)
    jx, jy, jz = plant.inertia
    cx, cy, cz = ((jz - jy), (jx - jz), (jy - jx)) if plant.coupling else (0.0, 0.0, 0.0)
    decay = math.exp(-h / plant.tau)
    k_t = plant.k
    yaw_t = [s * plant.c_q for s in plant.spin]
    px, py = plant.pos_x, plant.pos_y
    span = plant.omega_max - plant.omega_min
    dspan = float(plant.d_max - plant.d_min)
    d_lo = math.ceil(plant.d_min + (plant.idle - plant.omega_min) / span * dspan)
    f_lo, f_hi = k_t * (plant.idle * plant.idle), k_t * (plant.omega_max * plant.omega_max)
    mm = plant.m
    m_thr = [row[0] for row in mm]
    pair_r = [[f_hi * m_thr[i] - f_lo * m_thr[j] for j in range(4)] for i in range(4)]
    eps32 = 2.0 ** -23  # binary32 epsilon (rate_loop.hpp record_allocation: numeric_limits<float>::epsilon)
    n_eps = 4 * eps32
    gamma = n_eps / (1 - n_eps)
    dts = exec_dts(plan)
    alpha_cache = {}

    def alpha(dt, tc):
        key = (dt, tc)
        if key not in alpha_cache:
            alpha_cache[key] = -math.expm1(-dt / tc)
        return alpha_cache[key]

    kp, ki, kd, tref, dtau = law.kp, law.ki, law.kd, law.tau_ref, law.d_tau
    fj = law.inertia
    ff_on = law.ff
    tau_m, ff_tau = law.motor_tau, law.ff_tau

    # state
    w = [0.0, 0.0, 0.0]
    rot = [0.0, 0.0, 0.0, 0.0]
    wcmd = [0.0, 0.0, 0.0, 0.0]
    delay = [[0.0, 0.0, 0.0] for _ in range(sensor.latency)]
    tr = sensor.tracking
    notch_cache = {}
    stages = ([gcd.IDENTITY] * (gcd.MOTORS * len(gcd.HARMONICS)) if tr else []) + list(sensor.stages)
    chain = [[[0.0, 0.0, 0.0, 0.0] for _ in stages] for _ in range(3)]
    tel_delay = None
    r = [0.0] * 3
    e_prev = [0.0] * 3
    integ = [0.0] * 3
    y_prev = [0.0] * 3
    d_f = [0.0] * 3
    g_prev = [0.0] * 3
    gdot = [0.0] * 3
    freeze = [False] * 3
    out = [[0.0] * n_exec for _ in range(3)]
    flagged = [0, 0, 0]
    hs = h / substeps

    def deriv(a, b, c, tx, ty, tz):
        return (tx - cx * b * c) / jx, (ty - cy * c * a) / jy, (tz - cz * a * b) / jz

    for j in range(d_div * (n_exec - 1) + 1):
        # 1. sensor; with tracking, at an execution tick update_notches first, from the telemetry of the rotor speeds
        # after the last plant step, delayed by tr.latency ticks (before any, the first sample, as the model's line)
        if tr:
            now = [telemetry(om, tr) for om in rot]
            if tel_delay is None:
                tel_delay = [now] * tr.latency
            tel_delay.append(now)
            reported = tel_delay.pop(0)
            if j % d_div == 0:
                stages[:len(stages) - len(sensor.stages)] = notch_set(reported, tr, notch_cache)
        if delay:
            delay.append(list(w))
            sample = delay.pop(0)
        else:
            sample = w
        if bias is not None:
            sample = [x + b for x, b in zip(sample, bias)]
        y = [0.0, 0.0, 0.0]
        for a in range(3):
            v = sample[a]
            if j == 0:
                for s in chain[a]:
                    s[0] = s[1] = s[2] = s[3] = v
            for (b0, b1, b2, a1, a2), s in zip(stages, chain[a]):
                o = b0 * v + b1 * s[0] + b2 * s[1] - a1 * s[2] - a2 * s[3]
                s[1] = s[0]
                s[0] = v
                s[3] = s[2]
                s[2] = o
                v = o
            y[a] = v
        # 2. firmware
        if j % d_div == 0:
            n = j // d_div
            for a in range(3):
                out[a][n] = w[a]
            u = [0.0, 0.0, 0.0]
            if n == 0:
                r = list(y)
                e_prev = [0.0] * 3
                integ = [0.0] * 3
                if ff_on:
                    jy_ = (fj[0] * y[0], fj[1] * y[1], fj[2] * y[2])
                    g_prev = [y[1] * jy_[2] - y[2] * jy_[1], y[2] * jy_[0] - y[0] * jy_[2], y[0] * jy_[1] - y[1] * jy_[0]]
            else:
                dt = dts[n - 1]
                for a in range(3):
                    sp = sps[a][n]
                    r[a] = r[a] + alpha(dt, tref[a]) * (sp - r[a])
                    e = r[a] - y[a]
                    if not freeze[a]:
                        integ[a] = integ[a] + ki[a] * e_prev[a] * dt
                    d = -kd[a] * (y[a] - y_prev[a]) / dt
                    if dtau[a] > 0:
                        d_f[a] = d_f[a] + alpha(dt, dtau[a]) * (d - d_f[a])
                        d = d_f[a]
                    u[a] = kp[a] * e + integ[a] + d
                    e_prev[a] = e
                if ff_on:
                    jy_ = (fj[0] * y[0], fj[1] * y[1], fj[2] * y[2])
                    g = (y[1] * jy_[2] - y[2] * jy_[1], y[2] * jy_[0] - y[0] * jy_[2], y[0] * jy_[1] - y[1] * jy_[0])
                    for a in range(3):
                        raw = (g[a] - g_prev[a]) / dt
                        if ff_tau > 0:
                            gdot[a] = gdot[a] + alpha(dt, ff_tau) * (raw - gdot[a])
                        else:
                            gdot[a] = raw
                        u[a] = u[a] + (g[a] + tau_m * gdot[a])
                    g_prev = list(g)
            y_prev = list(y)
            if torque is not None:
                u = [u[a] + torque[a][n] for a in range(3)]
            # mixer allocate (mixer.hpp)
            tx, ty, tz = u
            av = [row[1] * tx + row[2] * ty for row in mm]
            yv = [row[3] * tz for row in mm]
            s_ = 1.0
            for i in range(4):
                for q in range(4):
                    if i != q:
                        pa = av[q] * m_thr[i] - av[i] * m_thr[q]
                        if pa > 0:
                            rr = pair_r[i][q] / pa
                            if rr < s_:
                                s_ = rr
            t_ = 0.0
            if s_ >= 1:
                t_ = 1.0
                for i in range(4):
                    for q in range(4):
                        if i != q:
                            py_ = yv[q] * m_thr[i] - yv[i] * m_thr[q]
                            if py_ > 0:
                                room = pair_r[i][q] - (av[q] * m_thr[i] - av[i] * m_thr[q])
                                if room < 0:
                                    room = 0.0
                                rr = room / py_
                                if rr < t_:
                                    t_ = rr
            p = [s_ * av[i] + t_ * yv[i] for i in range(4)]
            c_lo = max((f_lo - p[i]) / m_thr[i] for i in range(4))
            c_hi = min((f_hi - p[i]) / m_thr[i] for i in range(4))
            c = thrust
            if c > c_hi:
                c = c_hi
            if c < c_lo:
                c = c_lo
            ach = (s_ * tx, s_ * ty, t_ * tz)
            flags = (s_ < 1 and tx != 0, s_ < 1 and ty != 0, t_ < 1 and tz != 0)
            # record_allocation (rate_loop.hpp): the freeze of the next execution's integrator increment
            vv = (abs(c), abs(ach[0]), abs(ach[1]), abs(ach[2]))
            for a in range(3):
                bound = (gamma + eps32) * sum(plant.abs_bm[a][q] * vv[q] for q in range(4))
                diff = u[a] - ach[a]
                same = (e_prev[a] > 0 and diff > 0) or (e_prev[a] < 0 and diff < 0)
                freeze[a] = flags[a] and abs(diff) > bound and same
                if flags[a]:
                    flagged[a] += 1
            # thrust_to_dshot, then the plant's ESC map
            for i in range(4):
                f = p[i] + c * m_thr[i]
                if not f >= f_lo:
                    f = f_lo
                elif f > f_hi:
                    f = f_hi
                om = math.sqrt(f / k_t)
                dq = round_half_away(plant.d_min + (om - plant.omega_min) / span * dspan)
                if not dq >= d_lo:
                    dq = d_lo
                elif dq > plant.d_max:
                    dq = plant.d_max
                wcmd[i] = plant.omega_min + (plant.omega_max - plant.omega_min) * ((dq - plant.d_min) / dspan)
            if n == n_exec - 1:
                break
        # 3. plant: motors over the tick, then the wrench from the speeds at its end, then the rigid body
        for i in range(4):
            rot[i] = wcmd[i] + (rot[i] - wcmd[i]) * decay
        thr = [k_t * om * om for om in rot]
        tx = -sum(py[i] * thr[i] for i in range(4))
        ty = sum(px[i] * thr[i] for i in range(4))
        tz = sum(yaw_t[i] * thr[i] for i in range(4))
        a0, b0_, c0 = w
        for _ in range(substeps):
            k1 = deriv(a0, b0_, c0, tx, ty, tz)
            k2 = deriv(a0 + hs / 2 * k1[0], b0_ + hs / 2 * k1[1], c0 + hs / 2 * k1[2], tx, ty, tz)
            k3 = deriv(a0 + hs / 2 * k2[0], b0_ + hs / 2 * k2[1], c0 + hs / 2 * k2[2], tx, ty, tz)
            k4 = deriv(a0 + hs * k3[0], b0_ + hs * k3[1], c0 + hs * k3[2], tx, ty, tz)
            a0 = a0 + hs / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            b0_ = b0_ + hs / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            c0 = c0 + hs / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        w = [a0, b0_, c0]
    return out, flagged


def tick_map(su1, inertia, tau):
    """(a12, b1, a22, b2) of one tick of J w' = u_m, tau u_m' = u - u_m (the oracle's plant_map at divisor 1)."""
    return su1.plant_map(inertia, tau)


def linear_response(kp, ki, kd, tau_ref, d_tau, sensor, maps, sps, stamps, divisor):
    """w per execution of one axis's linear design model (module docstring): law at the executions, plant advanced by
    `maps` = (per-execution map, per-tick map). With no sensor dynamics the per-execution map is used; with the T3
    oracle's chain low-pass as the only stage and no latency this is run_l4.script_response operation for operation."""
    stages = sensor.stages
    dynamic = bool(stages) or sensor.latency > 0
    a12, b1, a22, b2 = maps[1] if dynamic else maps[0]
    steps = divisor if dynamic else 1
    chain = [[0.0, 0.0, 0.0, 0.0] for _ in stages]
    delay = [0.0] * sensor.latency
    w = um = r = integral = e_prev = y_prev = d_f = 0.0
    alpha_d = {}
    out = []
    u = 0.0
    for n, sp in enumerate(sps):
        out.append(w)
        for k in range(steps):
            if dynamic:
                if delay:
                    delay.append(w)
                    v = delay.pop(0)
                else:
                    v = w
                for (c0, c1, c2, c3, c4), s in zip(stages, chain):
                    o = c0 * v + c1 * s[0] + c2 * s[1] - c3 * s[2] - c4 * s[3]
                    s[1] = s[0]
                    s[0] = v
                    s[3] = s[2]
                    s[2] = o
                    v = o
                yn = v
            else:
                yn = w
            if k == 0:
                if n == 0:
                    r, e_prev, integral, u, y_prev, d_f = yn, 0.0, 0.0, 0.0, yn, 0.0
                else:
                    dt = (stamps[n] - stamps[n - 1]) * MICROSECOND
                    x = dt / tau_ref
                    alpha = -math.expm1(-x)
                    r = r + alpha * (sp - r)
                    e = r - yn
                    i_prev_term = ki * e_prev * dt
                    integral = integral + i_prev_term
                    d = -kd * (yn - y_prev) / dt
                    if d_tau > 0:
                        if dt not in alpha_d:
                            alpha_d[dt] = -math.expm1(-dt / d_tau)
                        d_f = d_f + alpha_d[dt] * (d - d_f)
                        d = d_f
                    u = kp * e + integral + d
                    e_prev = e
                    y_prev = yn
            w, um = w + a12 * um + b1 * u, a22 * um + b2 * u
    return out


# ---- the run's context (module globals, so a forked pool sees them) -------------------------------------------------

_CTX = {}


def _row_task(args):
    """Envelope rows of the linear design model of law `law_name` with sensor `skey`: grid row i of axis a, its 17
    members and, on an even row, the 9 x 9 subset's members."""
    law_name, skey, a, i = args
    c = _CTX
    p, law, sensor = c["fixture"], c["laws"][law_name], c["sensors"][skey]
    sensor = sensor.linear or sensor
    inertia, tau = p[INERTIA_KEYS[a]], p["motor_tau"]
    jb, tb = p["inertia_robustness_band"], p["tau_robustness_band"]
    sps, stamps = c["sps"][a], c["stamps"]
    s = -1.0 + 2.0 * i / (GRID - 1)
    lo = hi = lo9 = hi9 = None
    for q in range(GRID):
        t = -1.0 + 2.0 * q / (GRID - 1)
        jt, tt = inertia * (1 + s * jb), tau * (1 + t * tb)
        maps = (c["su"].plant_map(jt, tt), tick_map(c["su1"], jt, tt))
        y = linear_response(law.kp[a], law.ki[a], law.kd[a], law.tau_ref[a], law.d_tau[a], sensor, maps, sps, stamps,
                            c["plan"].divisor)
        lo = y if lo is None else [min(x, z) for x, z in zip(lo, y)]
        hi = y if hi is None else [max(x, z) for x, z in zip(hi, y)]
        if i % 2 == 0 and q % 2 == 0:
            lo9 = y if lo9 is None else [min(x, z) for x, z in zip(lo9, y)]
            hi9 = y if hi9 is None else [max(x, z) for x, z in zip(hi9, y)]
    return law_name, skey, a, i, lo, hi, lo9, hi9


def _sim_task(args):
    law_name, plant_index, substeps, coupling, skey = args
    c = _CTX
    plant = c["plants"][plant_index]
    if not coupling:
        plant = dataclasses.replace(plant, coupling=False)
    w, flagged = simulate(c["laws"][law_name], c["sensors"][skey], plant, c["plan"], c["sps"], c["plan"].thrust_n,
                          substeps)
    return args, w, flagged


# ---- inputs -------------------------------------------------------------------------------------------------------

def plan_params(fixture):
    """The parameter table plan_acro reads: the fixture (tick, divisor, rate_max) and the register's segment entries."""
    params = dict(fixture)
    for name, entry in schema.load_yaml(REGISTER).items():
        if run_l4.SEGMENT_T_US.match(name):
            params[name] = entry["value"]
    return params


@dataclasses.dataclass(frozen=True)
class StepPlan:
    """An L4 step plan (run_l4.plan) as simulate reads a plan: executions 0 .. the window's last, the plan's divisor,
    tick, stamps and thrust request."""
    plan: object

    @property
    def divisor(self):
        return self.plan.divisor

    @property
    def tick_s(self):
        return self.plan.tick_s

    @property
    def thrust_n(self):
        return self.plan.thrust_n

    @property
    def end_execution(self):
        return self.plan.window.stop - 1

    def stamp_us(self, k):
        return self.plan.stamp_us(k)


def step_plan(fixture, scenario):
    """(StepPlan, setpoints per axis per execution) of the L4 step scenario file `scenario` on the parameters `fixture`:
    the composition's one segment, the step rate on the stepped axis at the stamps >= l4_seg1_t_us and 0 elsewhere
    (run_l4.Plan.overrides; the composition applies a segment from its stamp on)."""
    p = run_l4.plan(run_l4.l4s.load(scenario), fixture, CARD)
    sp = StepPlan(p)
    sps = [[p.rate_rad_s if a == p.axis_index and p.stamp_us(k) >= p.seg_t_us else 0.0
            for k in range(sp.end_execution + 1)] for a in range(3)]
    return sp, sps


def card_plant(card, inertia, tau):
    cfg, _ = gpc.config_from(*gpc.load_linted(CARD), CARD)
    m, b, idle = mixer.mixer_matrix(card, CARD)
    consts = prim_constants.load()
    abs_bm = tuple(tuple(sum(abs(b[r_][q]) * abs(m[q][col]) for q in range(4)) for col in range(4)) for r_ in (1, 2, 3))
    return Plant(name="card", inertia=tuple(inertia), tau=tau, k=cfg["thrust_coeff"], c_q=cfg["torque_ratio_m"],
                 pos_x=tuple(p[0] for p in cfg["rotor_position_frd_m"]),
                 pos_y=tuple(p[1] for p in cfg["rotor_position_frd_m"]), spin=tuple(cfg["yaw_sign"]),
                 omega_min=cfg["omega_min_rad_s"], omega_max=cfg["omega_max_rad_s"], idle=idle,
                 m=tuple(tuple(row) for row in m), abs_bm=abs_bm, d_min=consts["kDshotThrottleMin"],
                 d_max=consts["kDshotThrottleMax"])


def read_0005_table():
    """Decision 0005's measured recovery table: axis -> (peak |w|, bound, executions outside)."""
    rows = re.findall(r"^\s*\| (Roll|Pitch|Yaw) \| ([0-9.]+) \| ([0-9.]+) \| (\d+) \|", DECISION_0005.read_text(), re.M)
    out = {name.lower(): (float(pk), float(bd), int(cnt)) for name, pk, bd, cnt in rows}
    if sorted(out) != sorted(AXES):
        raise SystemExit(f"{DECISION_0005}: the recovery table was not found")
    return out


def read_cause_trace():
    """cause.txt's m = 1 trace rows: execution -> (w_roll, w_pitch, w_yaw)."""
    lines = CAUSE.read_text().splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith(TRACE_HEAD)) + 1
    out = {}
    for ln in lines[start:]:
        if not ln.strip():
            break
        f = ln.split()
        out[int(f[0])] = (float(f[3]), float(f[4]), float(f[5]))
    return out


def read_cause_pitch_bound():
    """cause.txt's pitch rate bound line: (P, F) of test_t4_acro.design_bound on the acro_cause build, as printed."""
    m = re.search(r"^pitch rate bound \(iii\): P (\S+) \+ F (\S+) \+ E\(n\)$", CAUSE.read_text(), re.M)
    if not m:
        raise SystemExit(f"{CAUSE}: the pitch rate bound line was not found")
    return m.group(1), m.group(2)


def setup(corner_sel):
    """Everything but the runs: the design, the laws, sensors, plants, the plan and the script."""
    card, budget, scenario, profile = rl.load(CARD, BUDGET, SCENARIO_VALUES, PROFILE)
    t0 = time.time()
    d = rl.design(card, budget, scenario, profile, CARD, PROFILE)
    print(f"l6_ff_eval: rate_lead.design {time.time() - t0:.1f} s", file=sys.stderr)
    fixture = oracle.read_inputs(FIXTURE)
    plan = run_l4.plan_acro(run_l4.l4s.load_acro(ACRO), plan_params(fixture), CARD)
    inp = d["inputs"]
    base = card_plant(card, tuple(inp["inertia"]), inp["tau"])
    corners = d["j_corners"]
    if corner_sel == "all":
        chosen = list(range(1, len(corners) + 1))
    elif corner_sel == "none":
        chosen = []
    else:
        chosen = [int(x) for x in corner_sel.split(",")]
        if any(not 1 <= x <= len(corners) for x in chosen) or len(set(chosen)) != len(chosen):
            raise SystemExit(f"--corners: distinct indices in 1..{len(corners)}")
    plants = [base] + [dataclasses.replace(base, name=f"corner {i}", inertia=tuple(corners[i - 1]["J"])) for i in chosen]
    pi_l4 = d["pi_l4"]
    pi_axes = [pi_l4["axes"][a] for a in AXES]
    tau_ref_c = tuple(rate.r32_up(max(d["tau_cl"], pi_l4["axes"][a]["authority_term"])) for a in AXES)
    ax32 = d["axes32"]
    pid = Law("PID", kp=tuple(x[0] for x in ax32), ki=tuple(x[1] for x in ax32), kd=tuple(x[2] for x in ax32),
              tau_ref=tau_ref_c, d_tau=tuple(x[3] for x in ax32))
    j32 = tuple(fixture[k] for k in INERTIA_KEYS)
    laws = {
        "PI": Law("PI", kp=tuple(x["kp"] for x in pi_axes), ki=tuple(x["ki"] for x in pi_axes), kd=(0.0, 0.0, 0.0),
                  tau_ref=tuple(x["tau_ref"] for x in pi_axes), d_tau=(0.0, 0.0, 0.0)),
        "PID": pid,
        "PID+FF": dataclasses.replace(pid, name="PID+FF", inertia=j32, motor_tau=0.0, ff_tau=0.0),
        "PID+FF+lag": dataclasses.replace(pid, name="PID+FF+lag", inertia=j32, motor_tau=fixture["motor_tau"],
                                          ff_tau=d["t_ff"]),
    }
    ch = d["chain"]

    def stages(bypass):
        return tuple(s for s in gcd.chain_stages(ch["t_s"], ch["f_c"], ch["q"], ch["omega_th"],
                                                 [ch["omega_th"]] * gcd.MOTORS, bypass_all=bypass)
                     if s != gcd.IDENTITY)

    sensors = {
        "T3": Sensor("T3 worst case: latency_samples, the chain with every notch at omega_th (rate_lead's operating point)",
                     inp["latency"], stages(False)),
        "T4c": Sensor("T4 (c) configuration: truth gyro (no latency), the chain with every notch bypassed (the low-pass)",
                      0, stages(True)),
        "L4": Sensor("L4 as flown in acro_cause: truth gyro, no chain, no latency", 0, ()),
    }
    rs = profile["classes"]["rotor_speed"]["entries"]
    if rs["telemetry_period_unit"]["unit"] != "us":
        raise SystemExit(f"{PROFILE}: telemetry_period_unit is not in us")
    tracking = Tracking(pole_pairs=rs["pole_count"]["value"] // 2, exponent_bits=rs["telemetry_exponent_bits"]["value"],
                        mantissa_bits=rs["telemetry_mantissa_bits"]["value"],
                        unit_s=rs["telemetry_period_unit"]["value"] * MICROSECOND,
                        latency=rs["latency_rate_periods"]["value"] * plan.divisor, q=tuple(ch["q"]),
                        omega_th=ch["omega_th"], period=ch["t_s"])
    hover = tuple(s_ for s_ in gcd.chain_stages(ch["t_s"], ch["f_c"], ch["q"], ch["omega_th"],
                                                [ch["omega_hover"]] * gcd.MOTORS) if s_ != gcd.IDENTITY)
    sensors["T4e"] = Sensor(
        "stage (e) flight configuration: latency_samples, the notches tracking the telemetered rotor speeds, the low-pass",
        inp["latency"], tuple(s_ for s_ in stages(True)), tracking=tracking,
        linear=Sensor("its linear design model: latency_samples, every notch at omega_hover, the low-pass", inp["latency"],
                      hover))
    sps, stamps = [], None
    for a in range(3):
        s_, stamps = run_l4.acro_setpoints(plan, a)
        sps.append(s_)
    phases, table = run_l4.acro_phases(plan)
    return {"design": d, "fixture": fixture, "plan": plan, "plants": plants, "corner_index": [0] + chosen,
            "laws": laws, "sensors": sensors, "sps": sps, "stamps": stamps, "su": oracle.Setup(fixture),
            "su1": oracle.Setup({**fixture, "rate_loop_divisor": 1}), "phases": phases, "table": table}


# ---- the runs and the evaluation ----------------------------------------------------------------------------------

def predicate(w, z, f, phases):
    """run_l4.recovery_evaluation per axis with E = 0 and every execution fresh (the model has no stale read)."""
    window = [k for k, ph in enumerate(phases) if ph == "recovery"]
    n = len(w[0])
    return [run_l4.recovery_evaluation(window, w[a], [0.0] * n, z[a], f[a], [True] * n) for a in range(3)]


def plan_runs(n_plants, sensitivity):
    """(the sensor keys whose Z grid is needed, the simulation tasks (law, plant, RK4 steps, coupling, sensor))."""
    sims = [PI_RUN, ("PI", 0, 2 * RK_SUBSTEPS, True, "L4"), PI_UNCOUPLED]
    sims += [(v, i, RK_SUBSTEPS, True, "T3") for i in range(n_plants) for v in VARIANTS]
    sims.append(("PID+FF+lag", 0, 2 * RK_SUBSTEPS, True, "T3"))
    grids = ["T3"]
    if sensitivity:
        grids += SENSITIVITY
        sims += [(v, 0, RK_SUBSTEPS, True, k) for k in SENSITIVITY for v in VARIANTS]
    return grids, sims



def default_jobs():
    return cpu_quota.usable_cpus()


def run(corner_sel="all", jobs=None, sensitivity=False):
    c = setup(corner_sel)
    _CTX.clear()
    _CTX.update(c)
    jobs = jobs or default_jobs()
    grids, sims = plan_runs(len(c["plants"]), sensitivity)
    keys = [PI_GRID] + [("PID", k) for k in grids]
    rows = [(law, k, a, i) for law, k in keys for a in range(3) for i in range(GRID)]
    t0 = time.time()
    with multiprocessing.get_context("fork").Pool(jobs) as pool:
        env_async = pool.map_async(_row_task, rows, chunksize=1)
        sim_async = pool.map_async(_sim_task, sims, chunksize=1)
        env_rows = env_async.get()
        sim_out = sim_async.get()
    print(f"l6_ff_eval: runs {time.time() - t0:.1f} s on {jobs} processes", file=sys.stderr)

    def reduce(rows_, fn):
        acc = None
        for r_ in rows_:
            if r_ is not None:
                acc = r_ if acc is None else [fn(x, y) for x, y in zip(acc, r_)]
        return acc

    bounds = {}
    for key in keys:
        bounds[key] = {}
        for a, axis in enumerate(AXES):
            part = [r_ for r_ in env_rows if r_[:3] == (*key, a)]
            l17, h17 = reduce([r_[4] for r_ in part], min), reduce([r_[5] for r_ in part], max)
            l9, h9 = reduce([r_[6] for r_ in part], min), reduce([r_[7] for r_ in part], max)
            halving = max(max(abs(x - y) for x, y in zip(l9, l17)), max(abs(x - y) for x, y in zip(h9, h17)))
            bounds[key][axis] = {"P": max(max(abs(x) for x in l17), max(abs(x) for x in h17)),
                                 "Z": run_l4.rest_bound(l17, h17), "H": halving, "TOL": 0.0, "F": halving}
    bound_c = {k: bounds[("PID", k)] for k in grids}
    c.update(bound_pi=bounds[PI_GRID], bound_c=bound_c, sim={args: (w, fl) for args, w, fl in sim_out},
             sensitivity=sensitivity)
    return c


def evaluate(c):
    res = {}
    for args, (w, fl) in c["sim"].items():
        law, skey = args[0], args[4]
        b = c["bound_pi"] if law == "PI" else c["bound_c"][skey]
        res[args] = {"pred": predicate(w, [b[a]["Z"] for a in AXES], [b[a]["F"] for a in AXES], c["phases"]),
                     "flagged": fl, "w": w}
    return res


# ---- the report -----------------------------------------------------------------------------------------------------

def fmt_axes(vals, spec):
    return " ".join(format(v, spec) for v in vals)


def excess(pred):
    return [-p["worst margin"] for p in pred]


def outside(pred):
    return [p["violations"] for p in pred]


def max_change(w1, w2):
    return [max(abs(x - y) for x, y in zip(w1[a], w2[a])) for a in range(3)]


def cross_check(c, res, args=PI_RUN):
    """A PI run against acro_cause (module docstring): per axis (the model's recovery evaluation, decision 0005's
    (peak |w|, bound, outside)), the cause.txt trace, the largest |model - gz| over it and its peak |w| per axis."""
    ref = read_0005_table()
    trace = read_cause_trace()
    w = res[args]["w"]
    return {
        "axes": [(res[args]["pred"][a], ref[axis]) for a, axis in enumerate(AXES)],
        "trace": trace,
        "resid": [max(abs(w[a][k] - v[a]) for k, v in trace.items()) for a in range(3)],
        "peaks": [max(abs(v[a]) for v in trace.values()) for a in range(3)],
        "pitch_bound": ((repr(c["bound_pi"]["pitch"]["P"]), repr(c["bound_pi"]["pitch"]["F"])), read_cause_pitch_bound()),
    }


def worst_corners(c, res):
    """Per variant: (largest axis excess over the corners run, its corner index), per axis largest excess and outside,
    and the number of corners passing."""
    out = {}
    corners = [(i, idx) for i, idx in enumerate(c["corner_index"]) if idx]
    for v in VARIANTS:
        rr = [(res[(v, i, RK_SUBSTEPS, True, "T3")]["pred"], idx) for i, idx in corners]
        out[v] = {"worst": max((max(excess(p)), idx) for p, idx in rr),
                  "excess": [max(excess(p)[a] for p, _ in rr) for a in range(3)],
                  "outside": [max(outside(p)[a] for p, _ in rr) for a in range(3)],
                  "passing": sum(1 for p, _ in rr if all(x["passed"] for x in p))}
    return out


def render(c, res):
    d, plan, laws = c["design"], c["plan"], c["laws"]
    inp = d["inputs"]
    n_all = len(d["j_corners"])
    sel = c["corner_index"][1:]
    corners_arg = "all" if sel == list(range(1, n_all + 1)) else (",".join(str(i) for i in sel) or "none")
    lines = [
        "MARV quad L6 stage (c): omega x J omega feed-forward, T3 evaluation on the L4 acro combined full-stick segment",
        "command: uv run python tools/sim/l6_ff_eval.py --out <file> --corners " + corners_arg
        + (" --sensitivity" if c["sensitivity"] else ""),
        "label: design model (T3), not a validation run; reported, not asserted (decision 0014 owner decision 2)",
        "inputs:",
    ]
    for path in (CARD, BUDGET, SCENARIO_VALUES, PROFILE, ACRO, FIXTURE, REGISTER, CAUSE, DECISION_0005):
        lines.append(f"  {rel(path)} sha256 {sha256(path)}")
    lines += [
        "",
        "== design (tools/card/rate_lead.py design(); tau_ref: rate.py rule step 9 with rate_lead's tau_cl)",
        f"N* {d['n_star']:.9g}, w_c,nom {d['omega_c']:.9g} rad/s, tau_cl {d['tau_cl']!r} s, T_ff {d['t_ff']!r} s (f32)",
    ]
    for name in ("PI",) + VARIANTS:
        law = laws[name]
        lines.append(f"  {name:<11} kp {fmt_axes(law.kp, '.9g')}  ki {fmt_axes(law.ki, '.9g')}  kd "
                     f"{fmt_axes(law.kd, '.9g')}  tau_ref {fmt_axes(law.tau_ref, '.9g')}  T_f {fmt_axes(law.d_tau, '.9g')}"
                     + (f"  J {fmt_axes(law.inertia, '.9g')} tau_m {law.motor_tau:.9g} T_ff {law.ff_tau:.9g}"
                        if law.ff else ""))
    lines.append("sensors (chain f_c {:.6f} Hz, omega_th {:.6f} rad/s, latency_samples {}):".format(
        d["chain"]["f_c"], d["chain"]["omega_th"], inp["latency"]))
    for k, s in c["sensors"].items():
        lines.append(f"  {k:<4} {s.name}: latency {s.latency} tick(s), {len(s.stages)} fixed filter stages"
                     + (f"; 12 tracked notches (telemetry: {s.tracking.pole_pairs} pole pairs, {s.tracking.exponent_bits} "
                        f"+ {s.tracking.mantissa_bits} bits, unit {s.tracking.unit_s:.3g} s, latency {s.tracking.latency} "
                        f"ticks); Z, F from {s.linear.name} ({len(s.linear.stages)} stages)" if s.tracking else ""))
    lines += [
        f"plan: T_s {float(plan.tick_s)!r} s, D {plan.divisor}, executions 0..{plan.end_execution}, thrust request "
        f"{plan.thrust_n!r} N (f32), segments (number, kind, first, last, phase) {[list(t) for t in c['table']]}",
        f"plants: motor tau {inp['tau']!r} s at every plant; J (kg m^2):",
    ]
    j0 = c["plants"][0].inertia
    for idx, p in zip(c["corner_index"], c["plants"]):
        tag = "card" if idx == 0 else f"corner {idx} " + ("box" if d["j_corners"][idx - 1]["box"] else "cut") + (
            " common-scale" if d["j_corners"][idx - 1]["common_scale"] else "")
        lines.append(f"  {tag:<26} {fmt_axes(p.inertia, '.6g')}  (J/J0 {fmt_axes([x / y for x, y in zip(p.inertia, j0)], '.4g')})")
    lines += ["", "== predicate terms: F = TOL + H per axis (E = 0)"]
    terms = [("PI, L4 sensor (linear_response)", c["bound_pi"])]
    terms += [(f"stage (c) law, {k} sensor (linear_response)", c["bound_c"][k]) for k in c["bound_c"]]
    for label, b in terms:
        lines.append(f"  {label}: " + "; ".join(
            f"{a} F {b[a]['F']:.6e} (TOL {b[a]['TOL']:.3e}, H {b[a]['H']:.6e})" for a in AXES))
    xc = cross_check(c, res)
    lines += ["", "== cross-check: the PI law as flown in acro_cause, card plant, L4 sensor, against the gz runs"]
    (mp, mf), (gp, gf) = xc["pitch_bound"]
    lines.append(f"  PI law (rate.py), L4 sensor: pitch P {mp} + F {mf} (TOL 0; cause.txt, the acro_cause build's "
                 f"design_bound: P {gp} + F {gf}; P identical {mp == gp})")
    for axis, (p, (pk, bd, cnt)) in zip(AXES, xc["axes"]):
        lines.append(f"  {axis:<5} outside {p['violations']} (0005: {cnt}); excess {-p['worst margin']:.4f} rad/s at "
                     f"execution {p['at execution']}, |w| {abs(p['w there']):.4f} against Z + F {p['Z + F + E there']:.4f} "
                     f"(0005: |w| {pk}, bound {bd}, excess {pk - bd:.2f})")
    tr = xc["trace"]
    lines.append(f"  trace (cause.txt, {len(tr)} rows, executions {min(tr)}..{max(tr)}): max |model - gz| "
                 f"{fmt_axes(xc['resid'], '.4f')} rad/s; trace peak |w| {fmt_axes(xc['peaks'], '.4f')}")
    base = res[PI_RUN]
    fine = res[("PI", 0, 2 * RK_SUBSTEPS, True, "L4")]
    lines.append(f"  RK4 steps per tick doubled: max change of w {fmt_axes(max_change(base['w'], fine['w']), '.9f')} rad/s")
    ctl = res[PI_UNCOUPLED]
    lines.append(f"  control, the plant without w x J w: outside {fmt_axes(outside(ctl['pred']), 'd')}, excess "
                 f"{fmt_axes(excess(ctl['pred']), '.4f')} rad/s, trace max |model - gz| "
                 f"{fmt_axes(cross_check(c, res, PI_UNCOUPLED)['resid'], '.4f')} rad/s")
    lines.append("  linear_response = run_l4.script_response bit for bit (stage (c) law and chain low-pass of the "
                 "fixture, nominal J and tau): "
                 + ", ".join(f"{a} {ok}" for a, ok in zip(AXES, linear_matches_script_response(c))))
    nwin = sum(1 for ph in c["phases"] if ph == "recovery")
    head = (f"plant          variant      outside, of {nwin} (r p y)  max excess rad/s (r p y)      "
            "flagged (r p y)    verdict")

    def row(tag, args):
        r_ = res[args]
        ok = all(p["passed"] for p in r_["pred"])
        return (f"{tag:<14} {args[0]:<12} {fmt_axes(outside(r_['pred']), '5d')}         "
                f"{fmt_axes(excess(r_['pred']), '+9.4f')}   {fmt_axes(r_['flagged'], '5d')}   {'PASS' if ok else 'FAIL'}")

    lines += ["", "== results, T3 sensor: recovery predicate |w| <= Z + F + E (E = 0); excess = max(|w| - (Z + F))", head]
    for i, idx in enumerate(c["corner_index"]):
        for v in VARIANTS:
            lines.append(row("card" if idx == 0 else f"corner {idx}", (v, i, RK_SUBSTEPS, True, "T3")))
    lag1 = res[("PID+FF+lag", 0, RK_SUBSTEPS, True, "T3")]
    lag2 = res[("PID+FF+lag", 0, 2 * RK_SUBSTEPS, True, "T3")]
    lines.append(f"card PID+FF+lag, RK4 steps per tick doubled: max change of w "
                 f"{fmt_axes(max_change(lag1['w'], lag2['w']), '.9f')} rad/s, of the excess "
                 f"{fmt_axes([abs(x - y) for x, y in zip(excess(lag2['pred']), excess(lag1['pred']))], '.9f')} rad/s")
    if c["sensitivity"]:
        lines += ["", "== sensitivity, card plant: the same predicate with the sensor (and its own Z, F) changed", head]
        for k in SENSITIVITY:
            for v in VARIANTS:
                lines.append(row(f"card, {k}", (v, 0, RK_SUBSTEPS, True, k)))
    if sel:
        lines += ["", f"== summary over the {len(sel)} corner(s) run, T3 sensor"]
        for v, s in worst_corners(c, res).items():
            lines.append(f"{v:<12} worst corner {s['worst'][1]} (excess {s['worst'][0]:.4f} rad/s); per axis largest "
                         f"excess {fmt_axes(s['excess'], '+.4f')} rad/s, largest outside {fmt_axes(s['outside'], 'd')}; "
                         f"corners passing {s['passing']} of {len(sel)}")
    return "\n".join(lines) + "\n"


def fixture_law(c, a):
    """(kp, ki, kd, T_f, tau_ref) of axis a in the L4 T3 fixture (the stage (c) product law)."""
    return tuple(c["fixture"][f"rate_{q}_{AXES[a]}"] for q in ("kp", "ki", "kd", "d_filter_tau", "tau_ref"))


def linear_matches_script_response(c):
    """The fixture's stage (c) law, nominal J and tau, the fixture's chain low-pass (the T3 oracle's Setup.lowpass) as
    the sensor, no latency: linear_response against run_l4.script_response, per axis."""
    p, su = c["fixture"], c["su"]
    sensor = Sensor("the fixture's chain low-pass", 0, (su.lowpass,))
    out = []
    for a in range(3):
        kp, ki, kd, tf, tau_ref = fixture_law(c, a)
        jt, tt = p[INERTIA_KEYS[a]], p["motor_tau"]
        maps = (su.plant_map(jt, tt), tick_map(c["su1"], jt, tt))
        mine = linear_response(kp, ki, kd, tau_ref, tf, sensor, maps, c["sps"][a], c["stamps"], c["plan"].divisor)
        ref = run_l4.script_response(kp, ki, kd, tf, tau_ref, (su.divisor, su.lowpass, su.lowpass_error),
                                     su.tick_map(jt, tt), c["sps"][a], c["stamps"])
        out.append(mine == ref)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, metavar="FILE")
    ap.add_argument("--corners", default="all", metavar="all|none|I,J,...")
    ap.add_argument("--sensitivity", action="store_true",
                    help="add the card plant with the T4 (c) and L4 sensors (each with its own Z grid)")
    ap.add_argument("--jobs", type=int, default=None, help="worker processes (default: the CPUs this process may use)")
    args = ap.parse_args(argv)
    t0 = time.time()
    c = run(args.corners, args.jobs, args.sensitivity)
    res = evaluate(c)
    text = render(c, res)
    Path(args.out).write_text(text, encoding="utf-8", newline="\n")
    print(f"l6_ff_eval: wrote {args.out} ({time.time() - t0:.1f} s)", file=sys.stderr)
    return c, res, text


if __name__ == "__main__":
    main()
