#!/usr/bin/env python3
"""Independent double-precision oracle of the L11 T3 golden (quad spec 4 L11; decision 0020 "The proposal as ruled", section 5
"T3 golden (core)"), stored per decision 0011.

Path: tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py. Plain Python (stdlib), importing tools/sim/hover.py (WGS 84
normal gravity, the plant's g), tools/sim/reference.py (its analytic height gradient) and L5's oracle
(tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py: its method constants and its error-tracking number E); no code
shared with the C++ vertical library (fw/vertical) or the gain generator (tools/card/vertical.py). Reads
vertical_t3_inputs.txt (beside this script, or in --dir) and writes vertical_t3_golden.txt there:

    uv run python tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py [--dir DIR] [--procs N]
    uv run python tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params_l11_vertical_scripted/marv/params/param_defaults.cpp

--refresh-inputs rewrites vertical_t3_inputs.txt from the built parameter table of marv_params_l11_vertical_scripted and stops;
it needs the repository (tools/sim/run_l4.py hover_thrust, the card, design/scenario_values.yaml). The file is the T3 fixture;
the golden is a function of it. --procs spreads the tolerance's jobs over worker processes; the output does not depend on it
(every job is computed whole by one process and the results are combined in a fixed order).

What the golden is. The float firmware path of the vertical loop (fw/vertical/include/marv/vertical/vertical_law.hpp execute, as
fw/compositions/l11_vertical_scripted/src/l11_vertical_scripted.cpp tick uses it) closed around the design plant, all in double.
The design plant (NED, z down, v = dz/dt), per tick of exactly tick_period_num_us / tick_period_den us:
  the collective rotor speed w lags its command first order, dw/dt = (w_c - w)/tau, w_c = sqrt(T_cmd / (4 k)) the zero-torque
  collective inverse of the ESC map without DShot quantisation (every motor's thrust T_cmd / 4 and w = sqrt(f / k)), T_cmd held
  over the tick: w(t) = w_c + (w_0 - w_c) exp(-t / tau), closed form;
  the thrust T(t) = 4 k w(t)^2 within the tick, closed form;
  dv/dt = g - T(t) / m, dz/dt = v, g the WGS 84 normal gravity (hover.normal_gravity) at h = site_height_m - z evaluated at the
  start of each kinematic sub-step and held over it, the sub-step integrals of T(t) / m in closed form (KIN_SUBSTEPS sub-steps
  per tick: with one, g is evaluated at the tick's start, marv_plant's per-step evaluation).
Why this is exact for marv_plant v0 at level: marv_plant v0's force is gravity m g(h) along z_NED with g at
h = site_height_m - pos_ned_m[2] (sim/plant/include/marv_plant.h:81-84) plus the rotor thrusts k w_i^2 along -z_FRD and nothing
else (no drag, ground effect or battery, marv_plant.h:10); at level -z_FRD is -z_NED. Each w_i lags the ESC map's command first
order, solved exactly for a held command (sim/plant/src/plant_model.hpp: c + (w - c) exp(-h / tau)). The ESC map is linear in w,
so the unquantised DShot of the mixer's w = sqrt(f_i / k) maps back to that w exactly; at zero torque the mixer gives every motor
f_i = T_cmd / 4 (the symmetric card: mixer_m<i>_thrust = 1/4), so sum w_i^2 = 4 w^2 and the four lags are one. The model is
that system's exact solution over a tick except for: (a) g held over each kinematic sub-step at the sub-step's start height,
the within-tick variation of g (gamma |v| per unit time), bounded by the halving check below; (b) no DShot quantisation (the T4
term Q, decision 0020 section 5, not here); (c) in gz marv_plant evaluates the wrench from the END-of-step rotor speed and the
host holds it over the physics step (marv_plant.h:81-84), the simulator's time discretisation of the same equations: a T4
matter (decision 0020 section 5, T4), not part of this model.

Per-tick order (the composition's). Tick j: the vertical law executes at j = 0 mod D, D = rate_loop_divisor x vert_loop_ratio
(execution n = j / D, the seed execution n = 0 at tick 0), before the plant step of that tick, on the plant's z and v at the
start of the tick; the command it gives, T_cmd = T_req + d (d the script's thrust disturbance, added to the request as the
composition adds l11_dist_thrust_n), is held for the D ticks that follow (the rate group writes it on every rate tick; zero
computation delay). The law, in vertical_law.hpp's operation order:
  e_z = z_ref - z;  e_v = k_z e_z - v;  I_n = I_(n-1) + (k_i e_v,(n-1)) T;  u = k_v e_v + I_n;  T_req = T_h - u
with T = D ticks (the exact period), never clamped: the oracle refuses unless T_cmd stays within the mixer's zero-torque
collective range [T_min, T_max] = [4 k w_min^2, 4 k w_max^2] (decision 0020 section 4: idle_speed = speed_range.min,
tools/card/mixer.py:37; c_hi = 4 k w_max^2) over the whole run, so the mixer passes the request unchanged and the anti-windup
never freezes. Initial state: the hover trim w = sqrt(T_h / (4 k)), v = 0, z = 0, I = 0, e_v,(-1) = 0.

Scripts (scenario test values below), each from the hover trim; the seed execution n = 0 has z_ref = 0 and no disturbance:
  height_step:    z_ref = -Z_STEP_M (a climb: NED z is down) from execution 1 on, d = 0;
  thrust_offset:  z_ref = 0, d = thrust_offset_n (the card's mass conflict, decision 0020 section 4) from execution 1 on.
Golden channels per vertical execution n = 0 .. H: t_req (N, the law's output T_req before the disturbance) and the plant's z
(m) and v (m/s) at the start of tick n D (before that execution's thrust acts, as L5 records before the torque).

Run length (L5's rule, attitude_t3_oracle.py settle): H = ceil(DUR / T) executions after the seed; DUR = HORIZON_TAUS time
constants -T / ln(rho) of the slowest mode of the closed loop (rho its spectral radius: the nominal linearised loop below at
kappa = 1, gamma included, the f32 gains), doubled (core 7.5) until the end value of the error channel, |z_ref - z| at the last
execution, is below the z tolerance of its script, for every script.

Rounding tolerance (first order, L5's method: attitude_t3_oracle.py docstring and tests/regression/quad/L05/t3/README.md
"Tolerance (derived)"). Every float operation returns (1 + d) x exact, |d| <= u = 2^-24. The float path is mirrored operation
by operation on L5's error-tracking number E (value, first-order bound): at every execution n each node's local rounding bound
rho_p(n) is the bound of the node's own operations on exact operands (the errors of its operands are other nodes' injections,
which the linearised loop carries):
  e_z    fl(z_ref - f32(z)): the cast of the plant's double z, u |z|, and the subtraction, u |e_z|;
  e_v    fl(fl(k_z e_z) - f32(v)): u |k_z e_z| + u |v| (the cast) + u |e_v|;
  I      fl(I + fl(fl(k_i e_prev) T)): 2 u |k_i e_prev T| + u |I_n| (the period's rounding is the coefficient node `period`);
  u      fl(fl(k_v e_v) + I_n): u |k_v e_v| + u |u|;
  t_req  fl(T_h - u): u |T_req|;
  dist   the composition's fl(T_req + d): u |T_req + d|; 0 when d = 0 (adding 0 is exact).
Coefficient nodes: a float coefficient fixed for the whole run differs from the double one by one delta, |delta| <= its
first-order bound, and moves a channel at m by delta S(m), S the signed response of the linearised loop to the signal the
coefficient multiplies, injected where it acts; the node's bound is |delta| max_m |S(m)| (L5's lpf_coef and alpha):
  kp_z, kp_v, ki_v, hover_thrust  f32 values of the fixture that the float path and this oracle use unchanged: delta = 0 and
                                  the bound is 0 (recorded);
  period  the composition's T_f = fl(fl(fl(fl(ratio) fl(divisor)) fl(num)) / fl(den)) / 1e6) (load_vertical_config): the
          integer products are exact (below 1/u, checked), the divisions by den and by 1e6 round, |delta| <= 2 u T; signal
          k_i e_prev, into I.
The linearised loop (this oracle's own lifting, double): per execution the exact zero-order hold over T of the gamma-included
plant [m, v, z] with dm/dt = (w - m) / tau, dv/dt = gamma z - kappa m / mass, dz/dt = v (m the lagged thrust deviation, w the
command deviation, gamma = -dg/dh at the site by reference.gravity_height_gradient), the exponential of the augmented generator
by its Taylor series (scaled to infinity norm <= 1, summed until a term changes no entry, squared back), closed by the law on the
states [m, v, z, I, e_prev]. The loop is time-invariant per execution (a fixed design period: one phase), so a unit injection at
node p of execution k moves channel c at execution m by g_p,c(m - k) (z and v are observed before the execution's thrust acts,
t_req after its law), and the error of c at m is at most sum_p sum_k |g_p,c(m - k)| rho_p(k), rho_p in blocks of BLOCK
injections at the block maximum; the tolerance of c is the sum over the nodes of the largest of this over m. kappa is the
actuator's small-signal gain: around the trajectory a command deviation dT moves w_c by dT / (8 k w_c), the lag is exactly
linear in w, and the thrust moves by 8 k w dw, so the control path carries kappa = w / w_c (1 at every steady state). It is
frozen as L5 freezes its attitude feedback gain: at C_POINTS values evenly spaced over [kappa_min, kappa_max], the extremes of
w / w_c over the golden's ticks, keeping the largest bound over them per node and channel. Not covered by the bound (as L5's
frozen c): kappa varies within the run; gamma is frozen at the site height. The seed execution's bounds must be exactly 0 (the
scripts start at rest at their reference); the oracle refuses otherwise.

Kinematics (core 7.5; L5's check): the golden is integrated with KIN_SUBSTEPS sub-steps per tick and again with twice as many;
the largest change of each channel between the two is recorded and must be below that channel's tolerance, else the oracle
refuses.

Refusals (exit non-zero, nothing written): a missing input; a configuration vertical::validate refuses (gains >= 0, T > 0,
T_h > 0, no k_i without k_v); a non-positive tick or ratio; an inexact float period product; T_h outside [T_min, T_max]; a
gravity gradient that is not positive and finite; a closed loop that is not strictly stable at kappa = 1 or at a frozen kappa;
T_cmd leaving [T_min, T_max]; a non-zero bound at the seed execution; a kinematics change not below the tolerance; no settled
end value within MAX_DOUBLINGS doublings.
"""
import argparse
import importlib.util
import math
import os
import sys
from fractions import Fraction

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *([os.pardir] * 6)))
sys.path.insert(0, os.path.join(ROOT, "tools", "sim"))
import hover  # noqa: E402
import reference  # noqa: E402

_L5_PATH = os.path.join(ROOT, "tests", "regression", "quad", "L05", "t3", "reference", "attitude_t3_oracle.py")
_spec = importlib.util.spec_from_file_location("l5_attitude_t3_oracle", _L5_PATH)
l5 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(l5)

E = l5.E
r32 = l5.r32
UNIT_ROUNDOFF = l5.UNIT_ROUNDOFF  # binary32, round to nearest
MICROSECOND = l5.MICROSECOND
# The method constants are L5's (attitude_t3_oracle.py:99-104 and 859), imported, not restated: the first run length in
# time constants, the doubling limit, the frozen-gain points, the kinematic sub-steps and the injection block size.
HORIZON_TAUS = l5.HORIZON_TAUS
MAX_DOUBLINGS = l5.MAX_DOUBLINGS
C_POINTS = l5.C_POINTS
KIN_SUBSTEPS = l5.KIN_SUBSTEPS
BLOCK = l5.BLOCK

# Scenario test value (decision 0020 section 5, "T4": "(i) Height step | labelled scenario value, generator-proved linear (no
# collective clamp over the box)").
Z_STEP_M = 1.0  # height step, m, applied as a climb (z_ref = -Z_STEP_M, NED). Rationale: the scale of the height the L5 hold
#                 lost to the demonstration's roll shoves (decision 0020 Evidence: 0.06 to 0.3 m each, z = -3.5 m after six),
#                 i.e. the height this loop exists to win back; a climb, because recovering lost height is a climb. It must
#                 stay linear: the oracle refuses unless T_cmd stays within [T_min, T_max] over the whole run (a descent of
#                 this size asks T_h - k_v k_z Z_STEP_M at its first execution, below T_min on the committed gains).

SCRIPTS = ("height_step", "thrust_offset")
CHANNELS = ("t_req", "z", "v")
INJECTION_NODES = ("e_z", "e_v", "I", "u", "t_req", "dist")
COEF_NODES = ("kp_z", "kp_v", "ki_v", "hover_thrust", "period")
COEF_TARGET = {"kp_z": "e_v", "kp_v": "u", "ki_v": "I", "hover_thrust": "t_req", "period": "I"}
NODES = INJECTION_NODES + COEF_NODES

INPUT_KEYS = (
    "rate_loop_divisor", "tick_period_num_us", "tick_period_den", "vert_loop_ratio",
    "vert_kp_v", "vert_ki_v", "vert_kp_z",
    "site_latitude_rad", "site_height_m",
    "mass", "rotor_thrust_coeff", "rotor_speed_min", "rotor_speed_max", "motor_tau",
    "hover_thrust_n", "thrust_offset_n",
)
INT_KEYS = ("rate_loop_divisor", "tick_period_num_us", "tick_period_den", "vert_loop_ratio")


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
    """The fixture from a built parameter table (every key the table has, its f32 value) plus the two per-run values derived
    from the card at the design site of design/scenario_values.yaml (decision 0020 section 4)."""
    sys.path.insert(0, os.path.join(ROOT, "tools", "card"))
    import gen_plant_config as gpc  # noqa: PLC0415 (only the refresh needs the repository)
    import run_l4  # noqa: PLC0415

    import yaml  # noqa: PLC0415

    table = run_l4.read_param_defaults(defaults_cpp)
    card = os.path.join(ROOT, "vehicles", "uzh_neurobem_5in.yaml")
    register = yaml.safe_load(open(os.path.join(ROOT, "design", "scenario_values.yaml")))
    site = {k: register[k]["value"] for k in ("site_latitude_rad", "site_height_m")}
    card_doc, profile = gpc.load_linted(card, ROOT)
    m_card = gpc.config_from(card_doc, profile, card, ROOT)[0]["mass_kg"]  # the mass run_l4.hover_thrust uses
    conflicts = {float(c["value"]) for c in card_doc["mass"].get("conflict", []) or [] if c.get("state") == "open"}
    if len(conflicts) != 1:
        sys.exit(f"{card}: the mass must carry exactly one open conflict value, found {sorted(conflicts)!r}")
    (m_conflict,) = conflicts
    g0 = hover.normal_gravity(site["site_latitude_rad"], site["site_height_m"])
    table["hover_thrust_n"] = r32(run_l4.hover_thrust(card, site))
    table["thrust_offset_n"] = r32((m_card - m_conflict) * g0)
    with open(path, "w") as out:
        out.write("# The L11 T3 fixture: the f32 values the oracle and the T3 test use (hex floats are exact). Every key but the\n")
        out.write("# last two equals the parameter table of marv_params_l11_vertical_scripted of 2026-10-14, quad L11 S6\n")
        out.write(f"# (vertical_t3_oracle.py --refresh-inputs from {os.path.basename(defaults_cpp)}). hover_thrust_n and thrust_offset_n\n")
        out.write("# are per-run scenario values derived at refresh, not table entries, at the design site of design/scenario_values.yaml:\n")
        out.write("# hover_thrust_n = f32(m_card g(phi, h0)) by tools/sim/run_l4.py hover_thrust (the composition's vert_hover_thrust_n),\n")
        out.write("# thrust_offset_n = f32((m_card - m_conflict) g(phi, h0)), the card's mass conflict (decision 0020 section 4; the\n")
        out.write("# composition's l11_dist_thrust_n). The T3 test reads this file, not the live parameters.\n")
        for key in INPUT_KEYS:
            v = table[key]
            out.write(f"{key} {v}\n" if key in INT_KEYS else f"{key} {v.hex()}  # {v:.9g}\n")


# ---- linear algebra (double) ----------------------------------------------------------------------------------------------


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def inf_norm(a):
    return max(sum(abs(x) for x in row) for row in a)


def expm(a):
    """exp(a): scaled by 2^-s to infinity norm <= 1, the Taylor series summed until adding a term changes no entry, squared s
    times."""
    n = len(a)
    s = 0
    norm = inf_norm(a)
    while norm > 1:
        norm /= 2
        s += 1
    scaled = [[x / 2 ** s for x in row] for row in a]
    total = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    term = [row[:] for row in total]
    k = 0
    while True:
        k += 1
        term = [[x / k for x in row] for row in matmul(term, scaled)]
        nxt = [[x + y for x, y in zip(rt, rr)] for rt, rr in zip(total, term)]
        if nxt == total:
            break
        total = nxt
    for _ in range(s):
        total = matmul(total, total)
    return total


def log_spectral_radius(a):
    """ln of the spectral radius of a by Gelfand's formula: A^(2^k) = b_k e^(L_k) with ||b_k|| = 1 (infinity norm), squared
    and renormalised until L_k / 2^k changes no more."""
    nrm = inf_norm(a)
    b = [[x / nrm for x in row] for row in a]
    big = math.log(nrm)
    k = 0
    est = big
    while True:
        b2 = matmul(b, b)
        nrm = inf_norm(b2)
        b = [[x / nrm for x in row] for row in b2]
        big = 2 * big + math.log(nrm)
        k += 1
        new = big / 2 ** k
        if new == est:
            return new
        est = new


# ---- the setup ----------------------------------------------------------------------------------------------------------------


class Setup:
    def __init__(self, p):
        self.p = p
        for key in INT_KEYS:
            if p[key] < 1:
                sys.exit(f"{key} = {p[key]} is not >= 1")
        self.D = p["rate_loop_divisor"] * p["vert_loop_ratio"]
        self.tick = Fraction(p["tick_period_num_us"], p["tick_period_den"])  # us, exact
        self.tick_s = float(self.tick) * MICROSECOND
        self.period_s = float(self.D * self.tick) * MICROSECOND
        self.kp_z, self.kp_v, self.ki_v = p["vert_kp_z"], p["vert_kp_v"], p["vert_ki_v"]
        self.t_h, self.d = p["hover_thrust_n"], p["thrust_offset_n"]
        self.mass, self.k, self.tau = p["mass"], p["rotor_thrust_coeff"], p["motor_tau"]
        self.phi, self.h0 = p["site_latitude_rad"], p["site_height_m"]
        # vertical::validate (fw/vertical/include/marv/vertical/config.hpp), negated comparisons.
        if not (self.kp_z >= 0 and self.kp_v >= 0 and self.ki_v >= 0):
            sys.exit("vert_kp_z, vert_kp_v or vert_ki_v is negative (vertical::validate)")
        if not (self.t_h > 0) or (self.kp_v == 0 and self.ki_v != 0):
            sys.exit("hover_thrust_n is not > 0, or vert_ki_v is nonzero while vert_kp_v is 0 (vertical::validate)")
        if not (self.mass > 0 and self.k > 0 and self.tau > 0 and 0 < p["rotor_speed_min"] < p["rotor_speed_max"]):
            sys.exit("mass, rotor_thrust_coeff, motor_tau must be > 0 and 0 < rotor_speed_min < rotor_speed_max")
        # The float period of load_vertical_config: fl(ratio) fl(divisor) and that times fl(num) are integer products, exact
        # below 1/u; the division by den and the division by 1e6 round: |delta| <= 2 u T to first order.
        if not self.D * p["tick_period_num_us"] < 1 / UNIT_ROUNDOFF:
            sys.exit("vert_loop_ratio x rate_loop_divisor x tick_period_num_us is not below 2^24: the float period product "
                     "is not exact")
        self.period_delta = 2 * UNIT_ROUNDOFF * self.period_s
        self.t_min = 4 * self.k * p["rotor_speed_min"] ** 2
        self.t_max = 4 * self.k * p["rotor_speed_max"] ** 2
        if not self.t_min <= self.t_h <= self.t_max:
            sys.exit(f"hover_thrust_n {self.t_h!r} N is outside the zero-torque collective range "
                     f"[{self.t_min!r}, {self.t_max!r}] N")
        self.gamma = -reference.gravity_height_gradient(self.phi, self.h0)
        if not (math.isfinite(self.gamma) and self.gamma > 0):
            sys.exit(f"the gravity height gradient gamma = {self.gamma!r} 1/s^2 is not positive and finite")
        self.w_hover = math.sqrt(self.t_h / (4 * self.k))
        self.coef_delta = {"kp_z": 0.0, "kp_v": 0.0, "ki_v": 0.0, "hover_thrust": 0.0, "period": self.period_delta}


def script(name, n, su):
    """(z_ref, d) of execution n."""
    if n == 0:
        return 0.0, 0.0
    if name == "height_step":
        return -Z_STEP_M, 0.0
    return 0.0, su.d


# ---- the design plant ---------------------------------------------------------------------------------------------------------


class Plant:
    """The collective rotor speed w, v and z (NED) of the design plant (module docstring), stepped one tick of m_sub sub-steps
    with the rotor command held."""

    def __init__(self, su, m_sub):
        self.su = su
        self.m_sub = m_sub
        s = su.tick_s / m_sub
        tau = su.tau
        self.s = s
        self.decay = math.exp(-s / tau)
        e1 = -math.expm1(-s / tau)
        e2 = -math.expm1(-2 * s / tau)
        self.i1 = tau * e1                    # int_0^s exp(-t/tau) dt
        self.i2 = tau / 2 * e2                # int_0^s exp(-2t/tau) dt
        self.j1 = tau * (s - tau * e1)        # int_0^s int_0^t exp(-r/tau) dr dt
        self.j2 = tau / 2 * (s - tau / 2 * e2)  # int_0^s int_0^t exp(-2r/tau) dr dt
        self.a = 4 * su.k / su.mass
        self.w = su.w_hover
        self.v = 0.0
        self.z = 0.0

    def step(self, w_c):
        su, s, a = self.su, self.s, self.a
        cc = w_c * w_c
        for _ in range(self.m_sub):
            g = hover.normal_gravity(su.phi, su.h0 - self.z)
            dw = self.w - w_c
            lin = 2 * w_c * dw
            sq = dw * dw
            dv = g * s - a * (cc * s + lin * self.i1 + sq * self.i2)
            dz = self.v * s + g * s * s / 2 - a * (cc * s * s / 2 + lin * self.j1 + sq * self.j2)
            self.z += dz
            self.v += dv
            self.w = w_c + dw * self.decay


def simulate(su, name, h_seg, m_sub=KIN_SUBSTEPS):
    """The golden run: the channels per execution, the node injection bounds per execution, the coefficient signals, the range
    of kappa = w / w_c over the ticks and of T_cmd over the executions."""
    n_exec = 1 + h_seg
    uu = UNIT_ROUNDOFF
    plant = Plant(su, m_sub)
    integ = 0.0
    e_prev = 0.0
    w_c = su.w_hover
    out = {ch: [] for ch in CHANNELS}
    rho_seq = {node: [0.0] * n_exec for node in INJECTION_NODES}
    signal = {node: [0.0] * n_exec for node in COEF_NODES}
    kappa_lo = kappa_hi = plant.w / w_c
    cmd_lo = cmd_hi = su.t_h
    period = E(su.period_s)
    for j in range(n_exec * su.D):
        if j % su.D == 0:
            n = j // su.D
            z_ref, d = script(name, n, su)
            z, v = plant.z, plant.v
            ez = E(z_ref) - E(z, uu * abs(z))
            ev = E(su.kp_z) * E(ez.v) - E(v, uu * abs(v))
            i_new = E(integ) + E(su.ki_v) * E(e_prev) * period
            u = E(su.kp_v) * E(ev.v) + E(i_new.v)
            t_req = E(su.t_h) - E(u.v)
            cmd = E(t_req.v) + E(d)
            for node, x in (("e_z", ez), ("e_v", ev), ("I", i_new), ("u", u), ("t_req", t_req), ("dist", cmd)):
                rho_seq[node][n] = x.e
            signal["kp_z"][n] = ez.v
            signal["kp_v"][n] = ev.v
            signal["ki_v"][n] = e_prev * su.period_s
            signal["hover_thrust"][n] = 1.0
            signal["period"][n] = su.ki_v * e_prev
            out["t_req"].append(t_req.v)
            out["z"].append(z)
            out["v"].append(v)
            integ, e_prev = i_new.v, ev.v
            t_cmd = cmd.v
            if not su.t_min <= t_cmd <= su.t_max:
                sys.exit(f"{name}: the thrust command {t_cmd!r} N at execution {n} leaves the zero-torque collective range "
                         f"[{su.t_min!r}, {su.t_max!r}] N: the script is not linear")
            cmd_lo, cmd_hi = min(cmd_lo, t_cmd), max(cmd_hi, t_cmd)
            w_c = math.sqrt(t_cmd / (4 * su.k))
        kappa = plant.w / w_c
        kappa_lo, kappa_hi = min(kappa_lo, kappa), max(kappa_hi, kappa)
        plant.step(w_c)
    return {"out": out, "rho_seq": rho_seq, "signal": signal, "kappa": (kappa_lo, kappa_hi), "cmd": (cmd_lo, cmd_hi),
            "n_exec": n_exec, "z_ref_end": script(name, n_exec - 1, su)[0]}


# ---- the linearised closed loop and the tolerance -------------------------------------------------------------------------------


class Lin:
    """The linearised closed loop per execution (module docstring) at the frozen actuator gain kappa: states [m, v, z, I,
    e_prev]; plant [m, v, z] by the exact zero-order hold over T of the gamma-included generator."""

    def __init__(self, su, kappa):
        self.su = su
        t = su.period_s
        gen = [[-1 / su.tau, 0.0, 0.0, 1 / su.tau],
               [-kappa / su.mass, 0.0, su.gamma, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0]]
        ex = expm([[x * t for x in row] for row in gen])
        self.phi = [row[:3] for row in ex[:3]]
        self.gam = [ex[i][3] for i in range(3)]

    def step(self, x, inj):
        """One execution from state x with additive injections {node: size}: (state after, (t_req, z, v) observations)."""
        ms, v, z, integ, ep = x
        ez = -z + inj.get("e_z", 0.0)
        ev = self.su.kp_z * ez - v + inj.get("e_v", 0.0)
        integ = integ + self.su.ki_v * self.su.period_s * ep + inj.get("I", 0.0)
        u = self.su.kp_v * ev + integ + inj.get("u", 0.0)
        t = -u + inj.get("t_req", 0.0)
        w = t + inj.get("dist", 0.0)
        p, g = self.phi, self.gam
        nxt = (p[0][0] * ms + p[0][1] * v + p[0][2] * z + g[0] * w,
               p[1][0] * ms + p[1][1] * v + p[1][2] * z + g[1] * w,
               p[2][0] * ms + p[2][1] * v + p[2][2] * z + g[2] * w,
               integ, ev)
        return nxt, (t, z, v)

    def closed_loop(self):
        cols = []
        for i in range(5):
            cols.append(self.step(tuple(1.0 if k == i else 0.0 for k in range(5)), {})[0])
        return [[cols[j][i] for j in range(5)] for i in range(5)]

    def run(self, n_exec, node=None, drive=None):
        """The observations (t_req, z, v) per execution of a unit injection at `node` at execution 0, or of the signed
        sequence `drive` injected at `node` at every execution."""
        x = (0.0,) * 5
        obs = {ch: [0.0] * n_exec for ch in CHANNELS}
        ot, oz, ov = obs["t_req"], obs["z"], obs["v"]
        for n in range(n_exec):
            if drive is not None:
                inj = {node: drive[n]}
            else:
                inj = {node: 1.0} if n == 0 else {}
            x, (ot[n], oz[n], ov[n]) = self.step(x, inj)
        return obs


def cumulative_abs(seq):
    out = [0.0] * len(seq)
    acc = 0.0
    for i, x in enumerate(seq):
        acc += abs(x)
        out[i] = acc
    return out


def region_bound(rho, cum):
    """max over m of sum_k |g(m - k)| rho_k with rho_k in blocks of BLOCK injections at the block maximum (L5's region_bound
    for one phase): block [lo, hi] at rho_B adds rho_B (cum[m - lo] - cum[m - hi - 1]) to every m >= lo."""
    n = len(rho)
    acc = [0.0] * n
    windows = {}
    for lo in range(0, n, BLOCK):
        hi = min(lo + BLOCK, n) - 1
        r = max(rho[lo:hi + 1])
        if r == 0.0:
            continue
        width = hi - lo + 1
        if width not in windows:
            windows[width] = [cum[i] - (cum[i - width] if i >= width else 0.0) for i in range(n)]
        win = windows[width]
        acc[lo:] = [a + r * w for a, w in zip(acc[lo:], win)]
    return max(acc)


_CONTEXT = {}


def _job(job):
    """Worker: (script, kappa index, node) -> the node's bound per channel (t_req, z, v)."""
    name, ki, node = job
    su, run, kappa = _CONTEXT["su"], _CONTEXT["runs"][name], _CONTEXT["kappas"][name][ki]
    lin = Lin(su, kappa)
    n_exec = run["n_exec"]
    if node in COEF_NODES:
        delta = su.coef_delta[node]
        if delta == 0.0:
            return job, (0.0, 0.0, 0.0)
        obs = lin.run(n_exec, COEF_TARGET[node], run["signal"][node])
        return job, tuple(delta * max(abs(x) for x in obs[ch]) for ch in CHANNELS)
    obs = lin.run(n_exec, node)
    rho = run["rho_seq"][node]
    return job, tuple(region_bound(rho, cumulative_abs(obs[ch])) for ch in CHANNELS)


def kappa_points(run):
    lo, hi = run["kappa"]
    return [lo + (hi - lo) * i / (C_POINTS - 1) for i in range(C_POINTS)]


def tolerances(su, runs, procs):
    """{script: (bound {node: (t_req, z, v)}, tol (t_req, z, v))}: per node and channel the largest bound over the frozen
    kappas, the tolerance of a channel the sum over the nodes."""
    kappas = {}
    for name, run in runs.items():
        for node in INJECTION_NODES:
            if run["rho_seq"][node][0] != 0.0:
                sys.exit(f"{name}: the rounding bound of node {node} at the seed execution is not zero")
        kappas[name] = kappa_points(run)
        for kappa in kappas[name]:
            if not log_spectral_radius(Lin(su, kappa).closed_loop()) < 0:
                sys.exit(f"{name}: the linearised closed loop at the frozen gain kappa = {kappa!r} is not strictly stable")
    jobs = [(name, ki, node) for name in runs for ki in range(C_POINTS) for node in NODES]
    _CONTEXT.update(su=su, runs=runs, kappas=kappas)
    if procs > 1:
        import multiprocessing  # noqa: PLC0415

        with multiprocessing.get_context("fork").Pool(procs) as pool:
            results = dict(pool.map(_job, jobs, chunksize=1))
    else:
        results = dict(_job(j) for j in jobs)
    out = {}
    for name in runs:
        bound = {}
        for node in NODES:
            per = [results[(name, ki, node)] for ki in range(C_POINTS)]
            bound[node] = tuple(max(b[c] for b in per) for c in range(len(CHANNELS)))
        tol = tuple(sum(bound[node][c] for node in NODES) for c in range(len(CHANNELS)))
        out[name] = (bound, tol, kappas[name])
    return out


# ---- the run ------------------------------------------------------------------------------------------------------------------


def max_diff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def horizon(su):
    """(rho, time constant s) of the slowest mode of the nominal closed loop (kappa = 1)."""
    ln_rho = log_spectral_radius(Lin(su, 1.0).closed_loop())
    if not ln_rho < 0:
        sys.exit(f"the nominal closed loop is not strictly stable (spectral radius {math.exp(ln_rho)!r})")
    return math.exp(ln_rho), -su.period_s / ln_rho


def compute(su, dur_s, procs):
    h = math.ceil(dur_s / su.period_s)
    runs, kin = {}, {}
    for name in SCRIPTS:
        runs[name] = simulate(su, name, h)
        fine = simulate(su, name, h, m_sub=2 * KIN_SUBSTEPS)
        kin[name] = tuple(max_diff(runs[name]["out"][ch], fine["out"][ch]) for ch in CHANNELS)
    tols = tolerances(su, runs, procs)
    res = {"h": h, "dur_s": dur_s, "scripts": {}}
    for name in SCRIPTS:
        bound, tol, kappas = tols[name]
        if not all(k < t for k, t in zip(kin[name], tol)):
            sys.exit(f"{name}: the kinematics halving change {kin[name]!r} is not below the tolerance {tol!r}")
        run = runs[name]
        end = abs(run["z_ref_end"] - run["out"]["z"][-1])
        res["scripts"][name] = {"run": run, "kin": kin[name], "bound": bound, "tol": tol, "kappas": kappas, "end": end}
    return res


def settle(su, procs):
    rho, tau_s = horizon(su)
    dur = HORIZON_TAUS * tau_s
    history = []
    for k in range(MAX_DOUBLINGS + 1):
        res = compute(su, dur, procs)
        checks = [(name, d["end"], d["tol"][1]) for name, d in res["scripts"].items()]
        history.append((dur, res["h"], checks))
        if all(end < tol for _, end, tol in checks):
            res.update(history=history, doublings=k, rho=rho, tau_s=tau_s)
            return res
        dur *= 2
    sys.exit(f"the end value of the error channel does not settle within {MAX_DOUBLINGS} doublings: {history[-1]!r}")


def fmt(v):
    return f"{v:.10g}"


def write_golden(su, res, path):
    lines = [
        "# T3 golden of the L11 vertical loop (decision 0020 section 5 'T3 golden (core)'). Written by vertical_t3_oracle.py;",
        "# its docstring defines every line.",
        f"# period_us {su.period_s / MICROSECOND!r}  tick_us {su.tick_s / MICROSECOND!r}  executions_every_ticks {su.D}  "
        "unit_roundoff 2^-24",
        f"# slowest_mode_radius {res['rho']!r}  slowest_mode_seconds {res['tau_s']!r}  horizon_taus {HORIZON_TAUS}",
        f"# segment_seconds {res['dur_s']!r}  segment_executions {res['h']}  doublings {res['doublings']}  "
        f"kinematic_substeps {KIN_SUBSTEPS}",
        f"# gamma {su.gamma!r}  t_min_n {su.t_min!r}  t_max_n {su.t_max!r}  block {BLOCK}  frozen_gains {C_POINTS}",
        "# settle check (the segment doubles until every |z_ref - z| at the last execution is below the z tolerance):",
    ]
    for dur, h, checks in res["history"]:
        lines.append(f"#   segment_seconds {dur!r} segment_executions {h}: " +
                     "; ".join(f"{name} end {end!r} tolerance_z {tol!r}" for name, end, tol in checks))
    lines += [
        "# per script: t_req (N, the law's output before the disturbance), z (m) and v (m/s), NED, at each vertical execution",
        "# (z and v before that execution's thrust acts); per channel and node err_<channel>_<node>, the largest over the",
        "# executions of sum_k |g(m, k)| rho_k (rounding nodes) or |delta| |S(m)| (coefficient nodes); rho_<node> the largest",
        "# rounding injection or the coefficient's delta bound (information); the tolerance of each channel, the sum of err",
        "# over the nodes.",
    ]
    for name in SCRIPTS:
        d = res["scripts"][name]
        run = d["run"]
        lines.append(f"scenario {name}")
        lines.append(f"executions {run['n_exec']}")
        lines.append("step_execution 1")
        lines.append(f"z_ref_m {script(name, 1, su)[0]!r}")
        lines.append(f"thrust_disturbance_n {script(name, 1, su)[1]!r}")
        lines.append(f"t_cmd_min_n {run['cmd'][0]!r}")
        lines.append(f"t_cmd_max_n {run['cmd'][1]!r}")
        lines.append(f"kappa_min {run['kappa'][0]!r}")
        lines.append(f"kappa_max {run['kappa'][1]!r}")
        for ch, k in zip(CHANNELS, d["kin"]):
            lines.append(f"kinematics_change_{ch} {k!r}")
        for c, ch in enumerate(CHANNELS):
            for node in NODES:
                lines.append(f"err_{ch}_{node} {d['bound'][node][c]!r}")
        for node in INJECTION_NODES:
            lines.append(f"rho_{node} {max(run['rho_seq'][node])!r}")
        for node in COEF_NODES:
            lines.append(f"rho_{node} {su.coef_delta[node]!r}")
        for ch, t in zip(CHANNELS, d["tol"]):
            lines.append(f"tolerance_{ch} {t!r}")
        lines.append(f"end_error_z {d['end']!r}")
        for ch in CHANNELS:
            lines.append(ch)
            lines.extend(fmt(v) for v in run["out"][ch])
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=HERE, help="directory of vertical_t3_inputs.txt and of the output")
    ap.add_argument("--refresh-inputs", metavar="PARAM_DEFAULTS_CPP")
    ap.add_argument("--procs", type=int, default=1, help="worker processes of the tolerance (the output does not depend on it)")
    args = ap.parse_args()
    inputs_path = os.path.join(args.dir, "vertical_t3_inputs.txt")
    if args.refresh_inputs:
        refresh_inputs(args.refresh_inputs, inputs_path)
        return
    su = Setup(read_inputs(inputs_path))
    res = settle(su, max(args.procs, 1))
    write_golden(su, res, os.path.join(args.dir, "vertical_t3_golden.txt"))


if __name__ == "__main__":
    main()
