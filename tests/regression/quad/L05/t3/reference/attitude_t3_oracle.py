#!/usr/bin/env python3
"""Independent double-precision oracle of the L5 T3 suite (quad spec 4 L5, decision 0006 F "T3 step" and "T3 envelopes").

Path: tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py. Plain Python math (stdlib only), no code shared with the
C++ attitude library. Reads attitude_t3_inputs.txt (the f32 parameter values, beside this script) and writes
attitude_t3_golden.txt and attitude_t3_envelope.txt beside itself (or into --dir):

    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py
    uv run python tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params/marv/params/param_defaults.cpp

--refresh-inputs rewrites attitude_t3_inputs.txt from a generated parameter table (the product set) and stops; it needs the
repository (tools/sim/run_l4.py computes tau_held,yaw for the fallback disturbance). The file is the T3 fixture; the golden
and the envelope are functions of it.

What the golden is. The firmware path of angle mode, the attitude law and the rate loop in bypass (decision 0006 A, C, D)
closed around the design plant, all in double: per axis J w' = u_m, tau u_m' = u - u_m with u held over a tick (exact
zero-order-hold maps), no w x Jw; the body quaternion integrated from the exact per-axis angle increments of each sub-step,
q <- q (x) exp(dphi/2). The loop runs at tick resolution (a tick is one IMU sample): tick j has the stamp
floor(j num / den) us; the rate loop executes at j = 0 mod D (seed execution at j = 0), the attitude group at j = 0 mod D N
and before the rate group of the same tick, its rate setpoint held until the next attitude execution; the torque computed at
tick j acts from tick j (zero computation delay). Three scripts, every one from level and at rest, with the attitude
execution a = 0 (the seed, sticks 0) and stick segments in attitude executions (scenario test values, rule below):
  step_roll, step_pitch: the stick is 1 for executions a_s <= a < a_r (a_s = 1, a_r = 1 + H), then 0 (the release), to 1 + 2 H
  yaw_release:           the same with the yaw stick: a sustained full yaw stick, its release (braking, the crossing or
                         the fallback lock, decision 0006 D), and the heading pulled back to the lock.
Golden channels, per attitude execution (before that execution's torque acts): the unwrapped angle of the stepped axis and its
body rate. The yaw script also gives the execution of the lock and its decision margin.

Segment length (scenario test value, core 7.5 by doubling): H = ceil(DUR / T_a) attitude executions, DUR = HORIZON_TAUS / k
seconds (ten attitude time constants), doubled until every envelope's end value (tilt, yaw rate, heading relative to the lock;
the heading relative to the release is then constant) is below F = the T3 tolerance + the envelope's last-halving change + the
kinematics halving change of its script. F is a term of the T4 tolerance E + F with E >= 0, so an envelope that ends below F has
settled below the T4 tolerance.

Rounding tolerance (README, first order). Every float operation returns (1 + d) x exact, |d| <= u = 2^-24, and every library
call (sin, cos, atan2, hypot, sqrt) is within one unit in the last place of its result. The float path is mirrored step for
step in this script by a number type E (value, first-order absolute error bound): the value is the double computation, the
bound propagates the operation errors and the error of the inputs (the plant's double q and w cast to float32: u |x|). The
attitude group (measurement cast, angle mode, law) therefore yields one injection bound rho_ref(a) on the rate setpoint of each
attitude execution; the bypass loop has the nodes e (gyro cast and fl(r - y)), I and u of decision 0005 (kd = 0; the
prefilter node of L4 does not exist in bypass), with the bound rho_p(k) of each rate execution k. An injection at node p of
execution k moves a channel at execution m by g_p(m, k) times its size, so the error of a channel is at most
      sum over p, k of |g_p(m, k)| rho_p(k)      (rho_p(k) in blocks of BLOCK injections, each block at its maximum)
and the tolerance of a channel is the sum over the nodes of the largest of this over the observation executions m. g is the
response of the linearised closed loop: the per-execution ZOH rate loop of the nominal plant with the stamp dt of the
integrator and the attitude feedback gain g_fb = k c frozen at each of C_POINTS values of c in [c_min, 1] (c_min from the
trajectory's largest error; the largest bound over the c is kept); the responses are summed over every injection execution
and every phase of the rate executions within an attitude period. Before the yaw lock the attitude loop is open on yaw (the
heading setpoint tracks the heading); at the lock the regime changes: the responses are composed through the componentwise
bound of the error state at the lock, weighted by rho_p(k), and the lock heading is an added state (the lock copies the
measured heading). The decision margin of the lock (the sign of the yaw rate at the two executions around the crossing) is
recorded and must exceed the rate tolerance plus the cast bound, so the float and double runs lock at the same execution.

Kinematics (core 7.5): the golden is integrated with KIN_SUBSTEPS sub-steps per tick and again with twice as many; the largest
change of a channel between the two is recorded and must be below the tolerance.

Envelope: pointwise min / max, over the tau x J band box (J (1 +- inertia_robustness_band), tau (1 +- tau_robustness_band)), of
the design-model response of each script: the exact sampled-data rate loop of tools/card/attitude.py (bypass, the f32 nominal
gains, integrator step T) lifted to T_a, the attitude law in its single-axis closed form (tilt: r = 2 k sin(e/2); yaw lock:
r = (k/w) 2 sin(w e/2), e wrapped into [-pi, pi]), driven by the exact script from the exact initial state. The 17 x 17 grid
contains the corners and the 9 x 9 grid; halving_max_change is the largest change of any envelope point from 9 x 9 to 17 x 17.
The yaw script's envelope includes the release logic of decision 0006 D (braking, crossing, the fallback with att_yaw_t_cross
and att_yaw_alpha_min, the lock), each member's headings relative to its own release and its own lock. The fallback script
adds the constant yaw torque d = sigma_r tau_held,yaw from the release execution on; the T3 property (every member keeps
sigma_r w > 0 up to its fallback execution) halves the held stick until it holds.
"""
import argparse
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *([os.pardir] * 6)))

AXES = ("roll", "pitch", "yaw")
INERTIA = {"roll": "inertia_xx", "pitch": "inertia_yy", "yaw": "inertia_zz"}
NODES = ("ref", "e", "I", "u")

# Scenario test values (rationale in README.md).
HORIZON_TAUS = 10        # first segment length: ten attitude time constants 1 / k
MAX_DOUBLINGS = 4        # the segment length is doubled at most this often before the oracle refuses
GRID = 9                 # envelope grid points per band axis; the halving doubles the intervals (2 (GRID - 1) + 1)
C_POINTS = 3             # frozen feedback gains per loop: c_min, the midpoint to 1, and 1
KIN_SUBSTEPS = 1         # kinematic sub-steps per tick; the halving check runs 2 x this
STICK_HALVINGS = 8       # the fallback property halves the held stick at most this often before the oracle refuses
UNIT_ROUNDOFF = 2.0 ** -24  # binary32, round to nearest
MICROSECOND = 1.0e-6

INPUT_KEYS = (
    "rate_loop_divisor", "tick_period_num_us", "tick_period_den", "att_loop_ratio",
    "inertia_xx", "inertia_yy", "inertia_zz", "motor_tau",
    "rate_max_roll", "rate_max_pitch", "rate_max_yaw",
    "rate_kp_roll", "rate_kp_pitch", "rate_kp_yaw",
    "rate_ki_roll", "rate_ki_pitch", "rate_ki_yaw",
    "rate_kd_roll", "rate_kd_pitch", "rate_kd_yaw",
    "rate_tau_ref_roll", "rate_tau_ref_pitch", "rate_tau_ref_yaw",
    "tau_robustness_band", "inertia_robustness_band",
    "att_kp", "att_yaw_weight", "angle_tilt_max", "yaw_deadband", "att_yaw_alpha_min", "att_yaw_t_cross",
    "tau_held_yaw_nm",
)
INT_KEYS = ("rate_loop_divisor", "tick_period_num_us", "tick_period_den", "att_loop_ratio")


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def ulp32(x):
    """One unit in the last place of the binary32 number |x| (normal range); 0 for 0."""
    return 0.0 if x == 0 else 2.0 ** (math.frexp(abs(x))[1] - 24)


def read_inputs(path):
    values = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].split()
            if not line:
                continue
            key, text = line
            values[key] = int(text) if key in INT_KEYS else float.fromhex(text)
    missing = [k for k in INPUT_KEYS if k not in values]
    if missing:
        sys.exit(f"{path}: missing {missing}")
    return values


def refresh_inputs(defaults_cpp, path):
    sys.path.insert(0, os.path.join(ROOT, "tools", "sim"))
    sys.path.insert(0, os.path.join(ROOT, "tools", "card"))
    import run_l4  # noqa: PLC0415 (only the refresh needs the repository)

    import yaml  # noqa: PLC0415

    table = run_l4.read_param_defaults(defaults_cpp)
    card = os.path.join(ROOT, "vehicles", "uzh_neurobem_5in.yaml")
    site = yaml.safe_load(open(os.path.join(ROOT, "scenarios", "quad", "L04", "chirp_yaw.yaml")))
    thrust = r32(run_l4.hover_thrust(card, {k: site[k]["value"] for k in ("site_latitude_rad", "site_height_m")}))
    k = table["rotor_thrust_coeff"]
    f_min, f_max = k * table["idle_speed"] ** 2, k * table["rotor_speed_max"] ** 2
    rows = [[table[f"mixer_m{i}_{c}"] for c in run_l4.MIXER_COLUMNS] for i in range(1, 5)]
    held = run_l4.tau_held(rows, run_l4.MIXER_COLUMNS.index("yaw"), thrust, f_min, f_max)
    table["tau_held_yaw_nm"] = r32(held)
    with open(path, "w") as out:
        out.write("# The L5 T3 fixture: the f32 values the oracle and the T3 test use (hex floats are exact). Equal to the\n")
        out.write("# product parameter values of 2026-09-30 (attitude_t3_oracle.py --refresh-inputs from\n")
        out.write(f"# {os.path.basename(defaults_cpp)}); tau_held_yaw_nm is the collective-held yaw torque envelope of decision 0005's\n")
        out.write("# chirp rule (tools/sim/run_l4.py tau_held at the hover thrust of the L4 chirp_yaw scenario's site), rounded to f32.\n")
        out.write("# The T3 test reads this file, not the live parameters, so it does not follow later parameter changes.\n")
        for key in INPUT_KEYS:
            v = table[key]
            out.write(f"{key} {v}\n" if key in INT_KEYS else f"{key} {v.hex()}  # {v:.9g}\n")


# ---- the float path's first-order error arithmetic ----------------------------------------------------------------------


def _val(x):
    return x.v if isinstance(x, E) else x


def _is_pow2(c):
    return c != 0 and math.frexp(abs(c))[0] == 0.5


class E:
    """(v, e): v the double value of a float32 quantity, e a first-order bound of |float value - v|. A plain number is exact.
    Each rounding operation adds u |result|; a library call adds one ulp of the result (the operation errors of the
    rounding model in the module docstring). Multiplication by a power of two and by an exact zero is exact."""
    __slots__ = ("v", "e")

    def __init__(self, v, e=0.0):
        self.v = v
        self.e = e

    @staticmethod
    def of(x):
        return x if isinstance(x, E) else E(float(x), 0.0)

    def __neg__(self):
        return E(-self.v, self.e)

    def __add__(self, o):
        if not isinstance(o, E):
            o = E(float(o))
        if self.v == 0 and self.e == 0:
            return o
        if o.v == 0 and o.e == 0:
            return self
        v = self.v + o.v
        return E(v, self.e + o.e + UNIT_ROUNDOFF * abs(v))

    __radd__ = __add__

    def __sub__(self, o):
        return self + (-E.of(o))

    def __rsub__(self, o):
        return E.of(o) + (-self)

    def __mul__(self, o):
        exact = False
        if not isinstance(o, E):
            exact = _is_pow2(o)
            o = E(float(o))
        if (self.v == 0 and self.e == 0) or (o.v == 0 and o.e == 0):
            return E(0.0, 0.0)
        v = self.v * o.v
        rnd = 0.0 if exact else UNIT_ROUNDOFF * abs(v)
        return E(v, abs(self.v) * o.e + abs(o.v) * self.e + rnd)

    __rmul__ = __mul__

    def __truediv__(self, o):
        o = E.of(o)
        if self.v == 0 and self.e == 0:
            return E(0.0, 0.0)
        v = self.v / o.v
        return E(v, (self.e + abs(v) * o.e) / abs(o.v) + UNIT_ROUNDOFF * abs(v))

    def __rtruediv__(self, o):
        return E.of(o) / self


def e_sqrt(a):
    a = E.of(a)
    v = math.sqrt(a.v)
    prop = a.e / (2 * v) if v > 0 else math.sqrt(a.e)
    return E(v, prop + UNIT_ROUNDOFF * v)


def e_sin(a):
    a = E.of(a)
    if a.v == 0 and a.e == 0:
        return E(0.0, 0.0)
    v = math.sin(a.v)
    return E(v, abs(math.cos(a.v)) * a.e + ulp32(v))


def e_cos(a):
    a = E.of(a)
    v = math.cos(a.v)
    return E(v, abs(math.sin(a.v)) * a.e + ulp32(v))


def e_atan2(y, x):
    y, x = E.of(y), E.of(x)
    if y.v == 0 and y.e == 0 and x.v > 0:
        return E(0.0, 0.0)
    v = math.atan2(y.v, x.v)
    den = x.v * x.v + y.v * y.v
    return E(v, (abs(x.v) * y.e + abs(y.v) * x.e) / den + ulp32(v))


def e_hypot(a, b):
    a, b = E.of(a), E.of(b)
    if b.v == 0 and b.e == 0:
        return E(abs(a.v), a.e)
    if a.v == 0 and a.e == 0:
        return E(abs(b.v), b.e)
    v = math.hypot(a.v, b.v)
    return E(v, (abs(a.v) * a.e + abs(b.v) * b.e) / v + ulp32(v))


# ---- quaternions (w, x, y, z), the operation order of fw/prim/quat.hpp --------------------------------------------------


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(a):
    return (a[0], -a[1], -a[2], -a[3])


def qnorm(a):
    return e_sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3])


def qnormalized(a):
    inv = E.of(1.0) / qnorm(a)
    return tuple(c * inv for c in a)


def qcanonical(a):
    w, x, y, z = (_val(c) for c in a)
    if w != 0:
        flip = w < 0
    elif x != 0:
        flip = x < 0
    elif y != 0:
        flip = y < 0
    else:
        flip = z < 0
    return tuple(-c for c in a) if flip else a


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def qrotate(q, v):
    u = (q[1], q[2], q[3])
    t = tuple(c * 2 for c in cross(u, v))
    ut = cross(u, t)
    return tuple(v[i] + t[i] * q[0] + ut[i] for i in range(3))


def heading_quat(psi):
    half = psi * 0.5
    return (e_cos(half), E(0.0) * e_sin(half), E(0.0) * e_sin(half), e_sin(half))


def split_tilt_yaw(qe):
    """(q_xy, rho, a, b, psi, singular) of decision 0006 C step 3 for a canonical q_e."""
    rho = e_hypot(qe[0], qe[3])
    if not rho.v > 0:
        return qe, rho, None, None, E(0.0), True
    a = (qe[0] * qe[1] - qe[2] * qe[3]) / rho
    b = (qe[0] * qe[2] + qe[1] * qe[3]) / rho
    qz = qcanonical((qe[0] / rho, E(0.0), E(0.0), qe[3] / rho))
    psi = e_atan2(qz[3], qz[0]) * 2
    return (rho, a, b, E(0.0)), rho, a, b, psi, False


class Config:
    """The attitude configuration of the fixture (float32 values, exact in double)."""

    def __init__(self, p):
        self.kp = p["att_kp"]
        self.w = p["att_yaw_weight"]
        self.tilt_max = p["angle_tilt_max"]
        self.deadband = p["yaw_deadband"]
        self.alpha_min = p["att_yaw_alpha_min"]
        self.t_cross = p["att_yaw_t_cross"]
        self.rate_max = (p["rate_max_roll"], p["rate_max_pitch"], p["rate_max_yaw"])
        self.k_yaw = E(self.kp) / E(self.w)  # computed once at init, as PX4's setProportionalGain


def attitude_law(cfg, q, q_sp, yaw_rate_cmd):
    """AttitudeLaw::execute of decision 0006 C on E numbers: the rate setpoint (r_x, r_y, r_z) as E."""
    q_hat = qnormalized(q)
    q_sp_hat = qnormalized(q_sp)
    q_e = qcanonical(qmul(qconj(q_hat), q_sp_hat))
    q_xy, rho, a, b, psi, singular = split_tilt_yaw(q_e)
    q_ew = q_e
    if not singular:
        half = E(cfg.w) * psi * 0.5
        c, s = e_cos(half), e_sin(half)
        q_ew = (rho * c, c * a + s * b, c * b - s * a, rho * s)
    q_c = qcanonical(q_ew)
    ff = qrotate(qconj(q_hat), (E(0.0), E(0.0), E.of(yaw_rate_cmd)))
    r = (E(cfg.kp) * (q_c[1] * 2) + ff[0], E(cfg.kp) * (q_c[2] * 2) + ff[1], cfg.k_yaw * (q_c[3] * 2) + ff[2])
    return clamp(cfg, r)


def clamp(cfg, r):
    """The roll-pitch pair scaled by one factor, yaw clamped; the oracle's runs stay inside the bounds (asserted)."""
    for i in range(3):
        if abs(r[i].v) > cfg.rate_max[i]:
            sys.exit(f"the rate setpoint {r[i].v!r} reaches the clamp {cfg.rate_max[i]!r}: outside the derivation")
    return r


class AngleMode:
    """AngleMode::execute of decision 0006 D on E numbers. State: phase, sigma, omega_r, t_r, psi_lock (E)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.phase = "uninit"
        self.sigma = 0.0
        self.omega_r = 0.0
        self.t_r = 0
        self.psi_lock = E(0.0)
        self.margin_before = None
        self.margin_at = None
        self.rate_bound_at = None
        self._prev_margin = None

    @staticmethod
    def heading(q):
        w, z = q[0], q[3]
        if w.v == 0 and z.v == 0:
            return E(0.0)
        flip = w.v < 0 or (w.v == 0 and z.v < 0)
        return e_atan2(-z, -w) * 2 if flip else e_atan2(z, w) * 2

    def tilt_quat(self, s_r, s_p):
        n = e_sqrt(E(s_r * s_r + s_p * s_p))
        if not n.v > 0:
            return (E(1.0), E(0.0), E(0.0), E(0.0))
        half = E(self.cfg.tilt_max) * 0.5 * (E(1.0) if n.v > 1 else n)
        sh = e_sin(half)
        return (e_cos(half), sh * (E(s_r) / n), sh * (E(s_p) / n), E(0.0))

    def execute(self, sticks, t_us, q, omega):
        """Returns (q_sp, yaw_rate_cmd); records the crossing margins when the lock happens."""
        cfg = self.cfg
        s_r, s_p, s_y = sticks
        q_hat = qnormalized(q)
        psi_m = self.heading(q)
        rate_m = qrotate(q_hat, omega)[2]
        active = abs(s_y) > cfg.deadband
        if self.phase == "uninit":
            if active:
                self.phase = "yawrate"
            else:
                self.phase = "locked"
                self.psi_lock = psi_m
        elif active:
            self.phase = "yawrate"
        elif self.phase == "yawrate":
            self.phase = "braking"
            self.sigma = 1.0 if rate_m.v > 0 else (-1.0 if rate_m.v < 0 else 0.0)
            self.omega_r = abs(rate_m.v)
            self.t_r = t_us
            self._prev_margin = None
        if self.phase == "braking":
            dt = (t_us - self.t_r) / 1.0e6
            crossing = self.sigma * rate_m.v <= 0
            fallback = dt >= cfg.t_cross and dt * cfg.alpha_min >= self.omega_r
            margin = self.sigma * rate_m.v
            if crossing or fallback:
                self.phase = "locked"
                self.psi_lock = psi_m
                self.margin_before = self._prev_margin
                self.margin_at = margin
                self.rate_bound_at = rate_m.e
                self.lock_by = "crossing" if crossing else "fallback"
            self._prev_margin = margin
        q_xy_t = self.tilt_quat(s_r, s_p)
        if self.phase == "locked":
            return qcanonical(qmul(heading_quat(self.psi_lock), q_xy_t)), 0.0
        q_des = qmul(heading_quat(psi_m), q_xy_t)
        q_xy = split_tilt_yaw(qcanonical(qmul(qconj(q_hat), q_des)))[0]
        q_sp = qcanonical(qmul(q_hat, q_xy))
        cmd = 0.0
        if self.phase == "yawrate":
            mag = (abs(s_y) - cfg.deadband) / (1.0 - cfg.deadband)
            cmd = math.copysign(mag, s_y) * cfg.rate_max[2]
        return q_sp, cmd


# ---- the plant ---------------------------------------------------------------------------------------------------------


class Setup:
    def __init__(self, p):
        self.p = p
        self.divisor = p["rate_loop_divisor"]
        self.num = p["tick_period_num_us"]
        self.den = p["tick_period_den"]
        self.ratio = p["att_loop_ratio"]
        self.tick_s = self.num / self.den * MICROSECOND
        self.period_s = self.divisor * self.tick_s
        self.t_a = self.ratio * self.period_s
        self.tau_band = p["tau_robustness_band"]
        self.j_band = p["inertia_robustness_band"]
        self.cfg = Config(p)

    def stamp_us(self, tick):
        return (tick * self.num) // self.den

    def dt_exec(self, n):
        """Stamp spacing (s) of rate execution n >= 1, n counted from the seed execution 0."""
        return (self.stamp_us(self.divisor * n) - self.stamp_us(self.divisor * (n - 1))) * MICROSECOND

    def attitude_executions(self, h):
        return 1 + 2 * h

    def segment_length(self, dur_s):
        return math.ceil(dur_s / self.t_a)


class Axis:
    """One axis of the design plant over a step h: J w' = u_m, tau u_m' = u - u_m, u held; theta' = w."""

    def __init__(self, inertia, tau, h):
        self.j, self.tau, self.h = inertia, tau, h
        x = h / tau
        self.e = math.exp(-x)
        self.em = -math.expm1(-x)
        self.g = h - tau * self.em
        self.w = self.um = self.th = 0.0

    def step(self, u):
        j, tau, h, e, em, g = self.j, self.tau, self.h, self.e, self.em, self.g
        th = self.th + h * self.w + (tau * g * self.um + (h * h / 2 - tau * g) * u) / j
        w = self.w + (tau * em * self.um + g * u) / j
        um = e * self.um + em * u
        d = th - self.th
        self.th, self.w, self.um = th, w, um
        return d


def quat_exp(phi):
    angle = math.sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2])
    if angle == 0:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle / 2) / angle
    return (math.cos(angle / 2), phi[0] * s, phi[1] * s, phi[2] * s)


def scenario_axis(name):
    return {"step_roll": 0, "step_pitch": 1, "yaw_release": 2}[name]


def script_sticks(name, a, a_s, a_r):
    s = 1.0 if a_s <= a < a_r else 0.0
    axis = scenario_axis(name)
    return tuple(s if i == axis else 0.0 for i in range(3))


def simulate(su, name, h_seg, m_sub=KIN_SUBSTEPS):
    """The golden run. Returns a dict: th, om (per attitude execution), rho (per node, the maximum), cross_axes (maximum bound
    on the other axes' rate setpoints), rate_bound, events, e_max (maximum tilt or heading error of the axis)."""
    p, cfg = su.p, su.cfg
    axis = scenario_axis(name)
    n_att = su.attitude_executions(h_seg)
    a_s, a_r = 1, 1 + h_seg
    plants = [Axis(p[INERTIA[a]], p["motor_tau"], su.tick_s / m_sub) for a in AXES]
    q = (1.0, 0.0, 0.0, 0.0)
    angle = AngleMode(cfg)
    r_hold = (0.0, 0.0, 0.0)
    kp = [p[f"rate_kp_{a}"] for a in AXES]
    ki = [p[f"rate_ki_{a}"] for a in AXES]
    integral = [0.0] * 3
    e_prev = [0.0] * 3
    u = [0.0] * 3
    rho = dict.fromkeys(NODES, 0.0)
    rho_seq = {node: [0.0] * (n_att * su.ratio if node != "ref" else n_att) for node in NODES}
    cross_axes = 0.0
    th_out, om_out = [], []
    err_max = 0.0
    lock_a = None
    lock_info = None
    ticks = n_att * su.divisor * su.ratio
    for j in range(ticks):
        if j % (su.divisor * su.ratio) == 0:
            a = j // (su.divisor * su.ratio)
            t_us = su.stamp_us(j)
            qe = tuple(E(c, UNIT_ROUNDOFF * abs(c)) for c in q)
            om = tuple(E(pl.w, UNIT_ROUNDOFF * abs(pl.w)) for pl in plants)
            sticks = script_sticks(name, a, a_s, a_r)
            q_sp, cmd = angle.execute(sticks, t_us, qe, om)
            r = attitude_law(cfg, qe, q_sp, cmd)
            r_hold = tuple(c.v for c in r)
            rho["ref"] = max(rho["ref"], r[axis].e)
            rho_seq["ref"][a] = r[axis].e
            cross_axes = max(cross_axes, *(r[i].e for i in range(3) if i != axis))
            th_out.append(plants[axis].th)
            om_out.append(plants[axis].w)
            if angle.phase == "locked" and lock_a is None and name == "yaw_release" and a >= a_r:
                lock_a = a
                lock_info = (angle.margin_before, angle.margin_at, angle.rate_bound_at, angle.lock_by)
            if name == "yaw_release" and angle.phase == "locked":
                err_max = max(err_max, abs(wrap(angle.psi_lock.v - plants[2].th)))
            elif name != "yaw_release":
                err_max = max(err_max, abs(sticks[axis] * cfg.tilt_max - plants[axis].th))
        if j % su.divisor == 0:
            n = j // su.divisor
            dt = su.dt_exec(n) if n > 0 else 0.0
            y = [pl.w for pl in plants]
            if n == 0:
                integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
            else:
                a = axis
                inc = ki[a] * e_prev[a] * dt
                integral_new = integral[a] + inc
                e_a = r_hold[a] - y[a]
                u_a = kp[a] * e_a + integral_new
                step_rho = {"e": UNIT_ROUNDOFF * (abs(y[a]) + abs(e_a)),
                            "I": 3 * UNIT_ROUNDOFF * abs(inc) + UNIT_ROUNDOFF * abs(integral_new),
                            "u": UNIT_ROUNDOFF * abs(kp[a] * e_a) + UNIT_ROUNDOFF * abs(u_a)}
                for node, v in step_rho.items():
                    rho[node] = max(rho[node], v)
                    rho_seq[node][n] = v
                for i in range(3):
                    integral[i] = integral[i] + ki[i] * e_prev[i] * dt
                    e_i = r_hold[i] - y[i]
                    u[i] = kp[i] * e_i + integral[i]
                    e_prev[i] = e_i
        for _ in range(m_sub):
            d = [pl.step(u[i]) for i, pl in enumerate(plants)]
            q = qmul(q, quat_exp(d))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return {"th": th_out, "om": om_out, "rho": rho, "rho_seq": rho_seq, "cross_axes": cross_axes, "err_max": err_max, "lock_a": lock_a,
            "lock_info": lock_info, "a_r": a_r, "n_att": n_att, "angle_phase": angle.phase}


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


# ---- the linearised closed loop and the tolerance ------------------------------------------------------------------------


class Lin:
    """The per-execution ZOH rate loop of one axis (nominal plant, the f32 gains) with the stamp dt of the integrator, as a
    linear map on the error state [m, w, theta, I, e_prev] (plus the held lock heading as a sixth component)."""

    def __init__(self, su, axis):
        p = su.p
        self.su = su
        self.kp, self.ki = p[f"rate_kp_{AXES[axis]}"], p[f"rate_ki_{AXES[axis]}"]
        self.j, self.tau = p[INERTIA[AXES[axis]]], p["motor_tau"]
        t = su.period_s
        self.t = t
        self.e = math.exp(-t / self.tau)
        self.em = -math.expm1(-t / self.tau)
        self.g = t - self.tau * self.em
        self.q = t * t / 2 - self.tau * self.g

    def run(self, n_start, n_boundaries, state, psi, fb, kind=None, k0=None, b_inj=None, record_states=False):
        """Steps rate executions n_start ... ; the attitude boundaries are n = 0 mod N. Returns the observations (theta, w) at
        the boundaries b = ceil(n_start / N) .. n_boundaries - 1 (taken before that execution's step), and the states."""
        su = self.su
        ratio = su.ratio
        m, w, th, i_int, ep = state
        kp, ki, tau, j, e_, em, g, q2, t = self.kp, self.ki, self.tau, self.j, self.e, self.em, self.g, self.q, self.t
        obs, states = [], []
        r = 0.0
        for n in range(n_start, n_boundaries * ratio):
            if n % ratio == 0:
                b = n // ratio
                obs.append((th, w))
                if record_states:
                    states.append((m, w, th, i_int, ep))
                r = fb * (psi - th) + (1.0 if (kind == "ref" and b == b_inj) else 0.0)
            dt = su.dt_exec(n) if n > 0 else 0.0
            inj = 1.0 if (n == k0 and kind in ("e", "I", "u")) else 0.0
            i_int = i_int + ki * dt * ep + (inj if kind == "I" else 0.0)
            e = r - w + (inj if kind == "e" else 0.0)
            u = kp * e + i_int + (inj if kind == "u" else 0.0)
            th = th + t * w + (tau * g * m + q2 * u) / j
            w = w + (tau * em * m + g * u) / j
            m = e_ * m + em * u
            ep = e
        return obs, states


def strided_cum(seq, s):
    out = [0.0] * len(seq)
    for i, v in enumerate(seq):
        out[i] = v + (out[i - s] if i >= s else 0.0)
    return out


def phases(su):
    L = math.lcm(su.ratio, 2)
    return L, L // su.ratio


def node_sums(su, lin, fb, n_att, b_first):
    """Per node the strided cumulative sums of the absolute responses of (theta, w) and the raw error states: {node: (rows, s)},
    rows = [(c_p, Ctheta, Comega, states)] per phase. The injections of a phase are the rate executions k = b_first N + p + L q
    (every attitude execution b_first + q for the node ref), a sequence is indexed by the attitude boundary counted from the
    first boundary of its injection."""
    ratio = su.ratio
    L, s = phases(su)
    out = {}
    obs, sts = lin.run(b_first * ratio, n_att, [0.0] * 5, 0.0, fb, kind="ref", b_inj=b_first, record_states=True)
    out["ref"] = ([(0, strided_cum([abs(o[0]) for o in obs], 1), strided_cum([abs(o[1]) for o in obs], 1), sts)], 1)
    for node in ("e", "I", "u"):
        rows = []
        for p in range(L):
            k0 = b_first * ratio + p
            obs, sts = lin.run(k0, n_att, [0.0] * 5, 0.0, fb, kind=node, k0=k0, record_states=True)
            rows.append((-(-p // ratio), strided_cum([abs(o[0]) for o in obs], s), strided_cum([abs(o[1]) for o in obs], s), sts))
        out[node] = (rows, s)
    return out


BLOCK = 512  # injections per phase that share the maximum of their rounding injection bound (a rigorous coarsening)


def cum_at(c, x):
    return 0.0 if x < 0 else c[min(x, len(c) - 1)]


def phase_blocks(rho):
    """[(q_lo, q_hi, max rho)] over blocks of BLOCK injections."""
    out = []
    for lo in range(0, len(rho), BLOCK):
        hi = min(lo + BLOCK, len(rho)) - 1
        out.append((lo, hi, max(rho[lo:hi + 1])))
    return out


def injection_rho(run, node, base_b, p, su):
    """The rounding injection bounds of the injections of phase p that start at attitude boundary base_b: rate executions
    k = base_b N + p + L q (every attitude execution base_b + q for the node ref)."""
    if node == "ref":
        return run["rho_seq"]["ref"][base_b:]
    L, _ = phases(su)
    return run["rho_seq"][node][base_b * su.ratio + p::L]


def region_bound(su, run, sums, base_b, m_lo, m_hi, extra=None):
    """Per node the largest over the observation executions m_lo <= m < m_hi of the first-order error bound
    sum over injections k of |g(m, k)| rho_k, with rho_k the bound of the injection at its own execution (blocks of BLOCK
    injections take their maximum), as (theta, omega); extra(node, m) adds the contribution through the lock state. The
    injections are those at and after the boundary base_b."""
    res = {}
    for node, (rows, s) in sums.items():
        pre = []
        for p, (c_p, ct, cw, _) in enumerate(rows):
            pre.append((c_p, ct, cw, phase_blocks(injection_rho(run, node, base_b, p, su))))
        bt = bw = 0.0
        for m in range(m_lo, m_hi):
            st = sw = 0.0
            for c_p, ct, cw, blocks in pre:
                mp = m - base_b - c_p
                if mp < 0:
                    continue
                for q_lo, q_hi, rho in blocks:
                    if s * q_lo > mp:
                        break
                    st += rho * (cum_at(ct, mp - s * q_lo) - cum_at(ct, mp - s * (q_hi + 1)))
                    sw += rho * (cum_at(cw, mp - s * q_lo) - cum_at(cw, mp - s * (q_hi + 1)))
            if extra is not None:
                et, ew = extra(node, m)
                st, sw = st + et, sw + ew
            bt, bw = max(bt, st), max(bw, sw)
        res[node] = (bt, bw)
    return res


def state_sums(su, run, node, rows, s, b, base_b=0):
    """The componentwise bound X_i of sum over the injections before the attitude boundary b of rho_k |error state_i at b|."""
    x = [0.0] * 5
    for p, (c_p, _, _, sts) in enumerate(rows):
        rho = injection_rho(run, node, base_b, p, su)
        for q, r in enumerate(rho):
            idx = b - base_b - c_p - s * q
            if idx < 0:
                break
            if idx < len(sts):
                for i in range(5):
                    x[i] += r * abs(sts[idx][i])
    return x


def max_pair(a, b):
    return {n: (max(a[n][0], b[n][0]), max(a[n][1], b[n][1])) for n in a}


def tolerance(su, name, run):
    """(bound, tol): bound {node: (theta, omega)} the largest first-order error bound contributed by each node, and the
    per-channel tolerances (theta, omega) = the sums over the nodes."""
    cfg = su.cfg
    axis = scenario_axis(name)
    n_att = run["n_att"]
    lin = Lin(su, axis)
    if name != "yaw_release":
        c_min = math.cos(run["err_max"] / 2)
        bound = None
        for i in range(C_POINTS):
            c = c_min + (1 - c_min) * i / (C_POINTS - 1)
            sums = node_sums(su, lin, cfg.kp * c, n_att, 0)
            cur = region_bound(su, run, sums, 0, 0, n_att)
            bound = cur if bound is None else max_pair(bound, cur)
    else:
        a_l = run["lock_a"]
        sums_pre = node_sums(su, lin, 0.0, n_att, 0)
        bound = region_bound(su, run, sums_pre, 0, 0, a_l)
        xs = {node: state_sums(su, run, node, rows, s, a_l) for node, (rows, s) in sums_pre.items()}
        c_min = math.cos(cfg.w * run["err_max"] / 2)
        for i in range(C_POINTS):
            c = c_min + (1 - c_min) * i / (C_POINTS - 1)
            fb = cfg.k_yaw.v * cfg.w * c
            sums_post = node_sums(su, lin, fb, n_att, a_l)
            g = []
            for comp in range(6):
                state = [1.0 if comp == k else 0.0 for k in range(5)]
                obs, _ = lin.run(a_l * su.ratio, n_att, state, 1.0 if comp == 5 else 0.0, fb)
                g.append(obs)

            def extra(node, m, xs=xs, g=g):
                x = xs[node] + [xs[node][2]]
                d = m - a_l
                return (sum(x[k] * abs(g[k][d][0]) for k in range(6)), sum(x[k] * abs(g[k][d][1]) for k in range(6)))

            cur = region_bound(su, run, sums_post, a_l, a_l, n_att, extra)
            bound = max_pair(bound, cur)
    tol = (sum(bound[n][0] for n in NODES), sum(bound[n][1] for n in NODES))
    return bound, tol


# ---- the envelope: the design model over the band box -------------------------------------------------------------------


class Member:
    """One corner or grid point of the band box: the lifted exact ZOH rate loop of the design model on the plant with
    J (1 + s b_J) and tau (1 + t b_tau), the f32 nominal gains, integrator step T (decision 0006 E)."""

    def __init__(self, su, axis, s, t):
        p = su.p
        name = AXES[axis]
        self.su = su
        self.kp, self.ki = p[f"rate_kp_{name}"], p[f"rate_ki_{name}"]
        self.j = p[INERTIA[name]] * (1 + s * su.j_band)
        self.tau = p["motor_tau"] * (1 + t * su.tau_band)
        tt = su.period_s
        self.t = tt
        self.e = math.exp(-tt / self.tau)
        self.em = -math.expm1(-tt / self.tau)
        self.g = tt - self.tau * self.em
        self.q = tt * tt / 2 - self.tau * self.g

    def attitude_step(self, st, r, d):
        """One attitude period of N rate executions with the held reference r and a torque disturbance d."""
        m, w, th, i_int, ep = st
        kp, ki, tau, j, e_, em, g, q2, t = self.kp, self.ki, self.tau, self.j, self.e, self.em, self.g, self.q, self.t
        for _ in range(self.su.ratio):
            i_int += ki * t * ep
            e = r - w
            u = kp * e + i_int + d
            th, w, m = th + t * w + (tau * g * m + q2 * u) / j, w + (tau * em * m + g * u) / j, e_ * m + em * u
            ep = e
        return (m, w, th, i_int, ep)


def tilt_member_run(su, axis, s, t, h_seg):
    """theta per attitude execution of the tilt script on one member."""
    cfg = su.cfg
    mem = Member(su, axis, s, t)
    a_r = 1 + h_seg
    st = (0.0, 0.0, 0.0, 0.0, 0.0)
    out = []
    sin = math.sin
    k2 = 2 * cfg.kp
    bound = cfg.rate_max[axis]
    tilt = cfg.tilt_max
    for a in range(su.attitude_executions(h_seg)):
        out.append(st[2])
        sp = tilt if 1 <= a < a_r else 0.0
        r = k2 * sin((sp - st[2]) / 2)
        if abs(r) > bound:
            sys.exit(f"the design-model rate setpoint {r!r} reaches the clamp {bound!r}")
        st = mem.attitude_step(st, r, 0.0)
    return out


def yaw_member_run(su, s, t, h_seg, stick, d_amp):
    """The yaw release script on one member: dict(om, th per attitude execution; a_l the lock execution, a_fb the first
    execution with the fallback condition, min_sigma the smallest sigma_r w from the release up to and including a_fb
    (or to the end when there is none), lock_by)."""
    cfg = su.cfg
    mem = Member(su, 2, s, t)
    a_r = 1 + h_seg
    n_att = su.attitude_executions(h_seg)
    st = (0.0, 0.0, 0.0, 0.0, 0.0)
    om, th = [], []
    phase = "yawrate"
    sigma = omega_r = psi_l = 0.0
    a_l = a_fb = None
    min_sig = math.inf
    lock_by = None
    kw = cfg.k_yaw.v
    for a in range(n_att):
        w_a, th_a = st[1], st[2]
        om.append(w_a)
        th.append(th_a)
        d = 0.0
        if a < 1:
            r = 0.0
        elif a < a_r:
            r = stick * cfg.rate_max[2]
        else:
            d = d_amp
            if a == a_r:
                phase = "braking"
                sigma = 1.0 if w_a > 0 else (-1.0 if w_a < 0 else 0.0)
                omega_r = abs(w_a)
            dt = (a - a_r) * su.t_a
            fallback = dt >= cfg.t_cross and dt * cfg.alpha_min >= omega_r
            if a_fb is None and fallback:
                a_fb = a
            if a_fb is None or a <= a_fb:
                min_sig = min(min_sig, sigma * w_a)
            if phase == "braking":
                if sigma * w_a <= 0 or fallback:
                    phase = "locked"
                    psi_l = th_a
                    a_l = a
                    lock_by = "crossing" if sigma * w_a <= 0 else "fallback"
            r = 0.0 if phase == "braking" else kw * (2 * math.sin(cfg.w * wrap(psi_l - th_a) / 2))
        st = mem.attitude_step(st, r, d)
    return {"om": om, "th": th, "a_l": a_l, "a_fb": a_fb, "min_sigma": min_sig, "lock_by": lock_by, "a_r": a_r}


def grid_members():
    """[(index, s, t, in_coarse)] of the fine grid 2 (GRID - 1) + 1 points per band axis; in_coarse marks the GRID x GRID
    subset (the even indices), so one pass of the fine grid yields both envelopes."""
    n = 2 * (GRID - 1) + 1
    out = []
    for i in range(n):
        for j in range(n):
            out.append((i * n + j, -1.0 + 2.0 * i / (n - 1), -1.0 + 2.0 * j / (n - 1), i % 2 == 0 and j % 2 == 0))
    return out


class Envelope:
    """Pointwise min / max of a per-execution series over the fine grid and over its coarse subset; a series may have holes
    (None) for members that lack a value there (the heading relative to a lock not yet taken)."""

    def __init__(self, n):
        inf = math.inf
        self.lo = [inf] * n
        self.hi = [-inf] * n
        self.lo_c = [inf] * n
        self.hi_c = [-inf] * n

    def add(self, series, coarse, start=0):
        lo, hi, lo_c, hi_c = self.lo, self.hi, self.lo_c, self.hi_c
        for i in range(start, len(series)):
            v = series[i]
            if v is None:
                continue
            if v < lo[i]:
                lo[i] = v
            if v > hi[i]:
                hi[i] = v
            if coarse:
                if v < lo_c[i]:
                    lo_c[i] = v
                if v > hi_c[i]:
                    hi_c[i] = v

    def halving_change(self):
        change = 0.0
        for a, b, c, d in zip(self.lo, self.hi, self.lo_c, self.hi_c):
            if c != math.inf and a != math.inf:
                change = max(change, abs(a - c), abs(b - d))
        return change


def tilt_envelope(su, axis, h_seg):
    env = Envelope(su.attitude_executions(h_seg))
    for _, s, t, coarse in grid_members():
        env.add(tilt_member_run(su, axis, s, t, h_seg), coarse)
    return env


def yaw_envelopes(su, h_seg, stick, d_amp):
    """(envelopes, info) of the yaw release script over the grid: omega (all executions), heading relative to the release
    (from a_r), heading relative to the lock (from the member's lock); info holds the lock / fallback execution ranges and the
    smallest sigma_r w before the fallback."""
    n_att = su.attitude_executions(h_seg)
    a_r = 1 + h_seg
    env = {"omega": Envelope(n_att), "heading_release": Envelope(n_att), "heading_lock": Envelope(n_att)}
    locks, fbs, locks_c, fbs_c = [], [], [], []
    min_sig = math.inf
    by = set()
    for _, s, t, coarse in grid_members():
        run = yaw_member_run(su, s, t, h_seg, stick, d_amp)
        env["omega"].add(run["om"], coarse)
        th = run["th"]
        env["heading_release"].add([None] * a_r + [th[a] - th[a_r] for a in range(a_r, n_att)], coarse)
        a_l = run["a_l"]
        if a_l is None:
            sys.exit("a yaw-release member never locks within the run")
        env["heading_lock"].add([None] * a_l + [th[a] - th[a_l] for a in range(a_l, n_att)], coarse)
        locks.append(a_l)
        fbs.append(run["a_fb"])
        if coarse:
            locks_c.append(a_l)
            fbs_c.append(run["a_fb"])
        min_sig = min(min_sig, run["min_sigma"])
        by.add(run["lock_by"])
    info = {"lock_range": (min(locks), max(locks)), "lock_range_coarse": (min(locks_c), max(locks_c)),
            "fallback_range": (min(fbs), max(fbs)), "min_sigma": min_sig, "lock_by": sorted(by), "a_r": a_r}
    return env, info


def outward(x, up, digits=7):
    """x rounded outward (up or down) to `digits` significant digits, as a float."""
    if x == 0:
        return 0.0
    scale = 10.0 ** (math.floor(math.log10(abs(x))) - (digits - 1))
    y = x / scale
    return (math.ceil(y) if up else math.floor(y)) * scale


def channel_lines(env, first=0):
    """The 'lo hi' lines of an envelope series from execution `first` on (outward rounded), the first execution and the
    count; the series may start with holes."""
    n = len(env.lo)
    start = first
    while start < n and env.lo[start] == math.inf:
        start += 1
    lines = [f"{outward(env.lo[i], False):.7g} {outward(env.hi[i], True):.7g}" for i in range(start, n)]
    return start, lines


# ---- the run ---------------------------------------------------------------------------------------------------------------

SCENARIOS = ("step_roll", "step_pitch", "yaw_release")


def max_diff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def compute(su, dur_s):
    """Everything the two files hold for the segment length `dur_s`: golden blocks, envelope blocks and the settle checks."""
    p = su.p
    h = su.segment_length(dur_s)
    out = {"h": h, "dur_s": dur_s, "golden": {}, "envelope": {}, "checks": []}
    for name in SCENARIOS:
        run = simulate(su, name, h)
        fine = simulate(su, name, h, m_sub=2 * KIN_SUBSTEPS)
        kin = (max_diff(run["th"], fine["th"]), max_diff(run["om"], fine["om"]))
        l1, tol = tolerance(su, name, run)
        if run["cross_axes"] != 0.0:
            sys.exit(f"{name}: the error bound of an off-axis rate setpoint is {run['cross_axes']!r}, not 0: the single-axis "
                     "derivation does not hold")
        if not (kin[0] < tol[0] and kin[1] < tol[1]):
            sys.exit(f"{name}: the kinematics halving change {kin!r} is not below the tolerance {tol!r}")
        if name == "yaw_release":
            before, at, bound, _ = run["lock_info"]
            margin = min(before, -at)
            if not margin > tol[1] + bound:
                sys.exit(f"yaw_release: the lock decision margin {margin!r} does not exceed the rate tolerance {tol[1]!r}")
        out["golden"][name] = {"run": run, "kin": kin, "l1": l1, "tol": tol}
    for name, axis in (("step_roll", 0), ("step_pitch", 1)):
        env = tilt_envelope(su, axis, h)
        out["envelope"][name] = {"theta": env}
    env, info = yaw_envelopes(su, h, 1.0, 0.0)
    out["envelope"]["yaw_release"] = dict(env, info=info)
    stick = 1.0
    for _ in range(STICK_HALVINGS + 1):
        env_fb, info_fb = yaw_envelopes(su, h, stick, p["tau_held_yaw_nm"])
        if info_fb["min_sigma"] > 0:
            break
        stick /= 2
    else:
        sys.exit("the fallback property fails at every halved stick")
    out["envelope"]["yaw_fallback"] = dict(env_fb, info=info_fb, stick=stick)
    # Settle check: each envelope's end value is below F = the T3 tolerance + the envelope's last-halving change + the
    # kinematics halving change of its script (E >= 0 adds to the T4 tolerance, so a value below F is below E + F).
    g = out["golden"]
    for name in ("step_roll", "step_pitch"):
        e = out["envelope"][name]["theta"]
        end = max(abs(e.lo[-1]), abs(e.hi[-1]))
        f_val = g[name]["tol"][0] + e.halving_change() + g[name]["kin"][0]
        out["checks"].append((name, "theta", end, f_val))
    for name in ("yaw_release", "yaw_fallback"):
        e = out["envelope"][name]
        end_w = max(abs(e["omega"].lo[-1]), abs(e["omega"].hi[-1]))
        end_h = max(abs(e["heading_lock"].lo[-1]), abs(e["heading_lock"].hi[-1]))
        gy = g["yaw_release"]
        out["checks"].append((name, "omega", end_w, gy["tol"][1] + e["omega"].halving_change() + gy["kin"][1]))
        out["checks"].append((name, "heading_lock", end_h, gy["tol"][0] + e["heading_lock"].halving_change() + gy["kin"][0]))
    return out


def settle(su):
    dur = HORIZON_TAUS / su.cfg.kp
    history = []
    for k in range(MAX_DOUBLINGS + 1):
        res = compute(su, dur)
        history.append((dur, res["h"], [(c[0], c[1], c[2], c[3]) for c in res["checks"]]))
        if all(end < f_val for _, _, end, f_val in res["checks"]):
            res["history"] = history
            res["doublings"] = k
            return res
        dur *= 2
    sys.exit(f"the envelopes do not settle within {MAX_DOUBLINGS} doublings of the segment length: {history[-1]!r}")


def fmt(v):
    return f"{v:.10g}"


def write_golden(su, res, path):
    p = su.p
    lines = [
        "# T3 golden of the L5 attitude loop (decision 0006 F 'T3 step'). Written by attitude_t3_oracle.py; see README.md.",
        f"# period_us {su.period_s / MICROSECOND!r}  attitude_period_us {su.t_a / MICROSECOND!r}  tick_us {su.tick_s / MICROSECOND!r}  unit_roundoff 2^-24",
        f"# segment_seconds {res['dur_s']!r}  segment_executions {res['h']}  doublings {res['doublings']}  kinematic_substeps {KIN_SUBSTEPS}",
        "# per script: the unwrapped angle (rad) and body rate (rad/s) of the stepped axis at each attitude execution (before that",
        "# execution's torque acts); per channel and rounding node (ref: the attitude group's rate setpoint; e, I, u: the bypass",
        "# rate loop) err_<channel>_<node>, the largest over the executions of sum_k |g(m, k)| rho_k; the largest rounding",
        "# injection rho_<node> (information); and the tolerance of each channel, the sum of err over the nodes. The yaw script",
        "# adds the lock execution and its decision margin.",
    ]
    for name in SCENARIOS:
        d = res["golden"][name]
        run = d["run"]
        lines.append(f"scenario {name}")
        lines.append(f"axis {AXES[scenario_axis(name)]}")
        lines.append(f"executions {run['n_att']}")
        lines.append("step_execution 1")
        lines.append(f"release_execution {run['a_r']}")
        lines.append(f"kinematics_change_theta {d['kin'][0]!r}")
        lines.append(f"kinematics_change_omega {d['kin'][1]!r}")
        for ch, idx in (("theta", 0), ("omega", 1)):
            for node in NODES:
                lines.append(f"err_{ch}_{node} {d['l1'][node][idx]!r}")
        for node in NODES:
            lines.append(f"rho_{node} {run['rho'][node]!r}")
        lines.append(f"tolerance_theta {d['tol'][0]!r}")
        lines.append(f"tolerance_omega {d['tol'][1]!r}")
        if name == "yaw_release":
            before, at, bound, by = run["lock_info"]
            lines.append(f"lock_execution {run['lock_a']}")
            lines.append(f"lock_margin_before {before!r}")
            lines.append(f"lock_margin_at {at!r}")
            lines.append(f"lock_rate_bound {bound!r}")
            if by != "crossing":
                sys.exit("the golden yaw release locks by the fallback, not by the crossing")
        lines.append("theta")
        lines.extend(fmt(v) for v in run["th"])
        lines.append("omega")
        lines.extend(fmt(v) for v in run["om"])
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_envelope(su, res, path):
    lines = [
        "# T3 band envelopes of the L5 attitude loop (decision 0006 F 'T3 envelopes'): pointwise min / max over the tau x J band",
        "# box (17 x 17 grid, which contains the corners and the 9 x 9 grid) of the design-model response of each script, f32",
        "# nominal gains, driven by the exact script from the exact initial state. Written by attitude_t3_oracle.py; see",
        "# README.md. Values are rounded outward to 7 significant digits. halving_max_change is the largest change of any point",
        "# from the 9 x 9 grid to the 17 x 17 grid. A channel lists 'lo hi' for the executions first .. first + count - 1.",
        f"# segment_seconds {res['dur_s']!r}  segment_executions {res['h']}  doublings {res['doublings']}",
        "# settle check (the segment length doubles until every end value is below F = T3 tolerance + halving + kinematics):",
    ]
    for name, ch, end, f_val in res["checks"]:
        lines.append(f"#   {name} {ch}: end {end!r} < F {f_val!r}")

    def channel(name, env, first=0):
        start, rows = channel_lines(env, first)
        lines.append(f"channel {name} first {start} count {len(rows)} halving_max_change {env.halving_change()!r}")
        lines.append("min_max")
        lines.extend(rows)

    for name in ("step_roll", "step_pitch"):
        lines.append(f"scenario {name}")
        lines.append(f"axis {AXES[scenario_axis(name)]}")
        lines.append(f"release_execution {1 + res['h']}")
        channel("theta", res["envelope"][name]["theta"])
    for name in ("yaw_release", "yaw_fallback"):
        e = res["envelope"][name]
        info = e["info"]
        lines.append(f"scenario {name}")
        lines.append("axis yaw")
        lines.append(f"release_execution {info['a_r']}")
        lines.append(f"lock_execution_min {info['lock_range'][0]}")
        lines.append(f"lock_execution_max {info['lock_range'][1]}")
        lines.append(f"lock_execution_min_coarse {info['lock_range_coarse'][0]}")
        lines.append(f"lock_execution_max_coarse {info['lock_range_coarse'][1]}")
        lines.append(f"fallback_execution_min {info['fallback_range'][0]}")
        lines.append(f"fallback_execution_max {info['fallback_range'][1]}")
        lines.append(f"min_sigma_omega_to_fallback {info['min_sigma']!r}")
        if name == "yaw_fallback":
            lines.append(f"stick_scale {e['stick']!r}")
            lines.append(f"disturbance_nm {su.p['tau_held_yaw_nm']!r}")
        channel("omega", e["omega"])
        channel("heading_release", e["heading_release"])
        channel("heading_lock", e["heading_lock"])
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=HERE, help="directory of attitude_t3_inputs.txt and of the outputs")
    ap.add_argument("--refresh-inputs", metavar="PARAM_DEFAULTS_CPP")
    args = ap.parse_args()
    inputs_path = os.path.join(args.dir, "attitude_t3_inputs.txt")
    if args.refresh_inputs:
        refresh_inputs(args.refresh_inputs, inputs_path)
        return
    p = read_inputs(inputs_path)
    for axis in AXES:
        if p[f"rate_kd_{axis}"] != 0.0:
            sys.exit("the rounding derivation assumes kd = 0 (decision 0005, owner decision 5)")
    su = Setup(p)
    res = settle(su)
    write_golden(su, res, os.path.join(args.dir, "attitude_t3_golden.txt"))
    write_envelope(su, res, os.path.join(args.dir, "attitude_t3_envelope.txt"))


if __name__ == "__main__":
    main()
