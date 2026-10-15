#!/usr/bin/env python3
"""Independent double-precision oracle of the L11 T3 golden (quad spec 4 L11; decision 0020 "The proposal as ruled", section 5
"T3 golden (core)"), stored per decision 0011.

Path: tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py. Plain Python (stdlib), importing tools/sim/hover.py (WGS 84
normal gravity, the plant's g), tools/sim/reference.py (its analytic height gradient) and L5's oracle
(tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py: its method constants, its error-tracking number E and, for the roll
shove, its attitude law, angle mode, mixer and quaternion kinematics); no code shared with the C++ vertical library (fw/vertical) or
the gain generator (tools/card/vertical.py). Reads vertical_t3_inputs.txt (beside this script, or in --dir) and, for the roll shove,
L5's attitude_t3_inputs.txt and attitude_t3_q_inputs.txt (in L5's reference directory), and writes vertical_t3_golden.txt,
vertical_t3_envelope.txt and vertical_t3_q.txt there (--golden-only: the golden alone):

    uv run python tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py [--dir DIR] [--procs N] [--golden-only]
    uv run python tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py --refresh-inputs \
        build/<preset>/generated/marv_params_l11_vertical_scripted/marv/params/param_defaults.cpp

--refresh-inputs rewrites vertical_t3_inputs.txt from the built parameter table of marv_params_l11_vertical_scripted and stops;
it needs the repository (tools/sim/run_l4.py hover_thrust, the card, design/scenario_values.yaml). The file is the T3 fixture;
the golden is a function of it. --procs spreads the tolerance's jobs, the envelope members and the Q members over worker processes; the output does not depend on it
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

Box envelopes (vertical_t3_envelope.txt; decision 0020 section 5 "T3 envelopes (recorded; used at T4)", L5's method:
attitude_t3_oracle.py write_envelope, tests/regression/quad/L05/t3/README.md "Band envelopes"). The box is decision 0020
section 4's: tau in tau (1 +- tau_robustness_band) and the loop-gain factor r = (k_true / k_card)(m_card / m_true) in
[r_lo, r_hi], r_lo = (1 - b_k) min R, r_hi = (1 + b_k) max R, b_k = thrust_gain_robustness_band, R = {1} and m_card / m_c for
every open conflict m_c of the card's mass (r_lo = 0.5, r_hi = 1.5 / 0.974093 on the committed card), read from design/budget.yaml
and vehicles/uzh_neurobem_5in.yaml and equal to the box tools/card/vertical.py designs against. Each member is a point of L5's
17 x 17 grid (l5.grid_members, which contains the corners and the 9 x 9 grid) over (s, t) in [-1, 1]^2 -> tau = tau (1 + b_tau s),
r = r_lo + (r_hi - r_lo)(t + 1) / 2. A member is the design plant above with the motor lag tau and the thrust deviation about
the hover trim scaled by r, T = T_h + r (4 k w^2 - T_h): every member starts at the same exact initial state (the hover trim,
v = 0, z = 0, I = 0), a gain error is not a thrust error (the thrust offset is the thrust_offset script), and the linearised
member is Lin's plant with r the loop gain. The law is the golden's in double with the f32 gains (no rounding injections), driven
by the exact script, never re-seeded; the nominal member (tau, r = 1) reproduces the golden's z to the bit. The oracle refuses
unless T_cmd of every member stays within [T_min, T_max] (the mixer passes the request unchanged: the script is linear over the
box). The segment starts at the golden's length and doubles (MAX_DOUBLINGS) until, for each of the two golden scripts, the end value (the largest
|z_ref - z| over the envelope at the last execution) is below F = tolerance_z of the script's golden + the envelope's
halving_max_change + the script's kinematics_change_z (L5's stop test; E >= 0 adds to the T4 tolerance). The golden's own length
is not changed by it. The file (every line, in order):
  '#' header: what the file is; the box (tau_robustness_band, thrust_gain_robustness_band, r_lo, r_hi); segment_seconds,
              segment_executions (H, the envelope has H + 1 executions), doublings (of the envelope's segment from the golden's),
              grid (members); per script 'name z: end E < F F' (the settle check, parsed by T4 as L5's 'theta' line) and
              'name F terms: tolerance_z, halving, kinematics_change_z' (the three terms of F);
  scenario <name>; step_execution 1; z_ref_m and thrust_disturbance_n the script's from the step on (as in the golden);
  t_cmd_min_n, t_cmd_max_n the range of T_cmd over all members and executions (inside [T_min, T_max]);
  channel z first <first> count <count> halving_max_change <change>, 'min_max', then count lines 'lo hi': the plant's z (m, NED,
              at the start of the execution's tick, as the golden's z) minimum and maximum over the 17 x 17 members at executions
              first .. first + count - 1, rounded outward to 7 significant digits; halving_max_change the largest change of any
              execution's minimum or maximum from the 9 x 9 grid to the 17 x 17 grid.
The channel is z only: T4's predicate is on z (decision 0020 section 5), and L5 records the angle, not the rate, for its tilt
scripts.

Roll shove (T4 scenario (iii); an envelope without a golden). Decision 0020 section 5 "T3 envelopes": "The roll shove uses a coupled
plant: per-rotor speed lags, f_i = k w_i^2, the mixer in double, 0006's quaternion kinematics without w x J w, and the vertical
channel with T R33. Duration doubles until the envelope's end value is below the T4 tolerance." Script: from the exact hover trim,
level, at rest, z_ref = 0, no thrust disturbance, a roll torque pulse of SHOVE_TORQUE_NM for SHOVE_DURATION_S (section 5 "T4",
Owner decisions 7) added to the rate loop's torque request before the mixer (actuator-side, as the composition adds torque_pulse,
l11_dist_torque_*), starting at the stamp t0 = floor(D num / den) us of vertical execution SHOVE_EXECUTION (the steps of the other
scripts start there) and acting on the rate ticks whose stamp is in [t0, t0 + dur) (the composition's test; torque_pulse_t0_us and
torque_pulse_dur_us in the file are the values for l11_dist_torque_t_us and _dur_us). The box is the same (tau, r) box and grid;
the inertia J is the card's value (its band is not part of the box, lead's ruling). A member is the CoupledPlant: the four rotor
speeds each lag their own command first order (the Plant's closed forms with the member's tau), f_i = k w_i^2, the torque
sum_i geo_i f_i with the card's rotor geometry and yaw reaction (L5's Quantiser.geo) on J w' with no w x J w, the quaternion
q <- q (x) exp(dphi / 2) from the exact per-axis angle increments of each sub-step (L5's qmul and quat_exp, renormalised per
tick), and v' = g - (1 - r) T_h / m - (r / m) R33 sum_i f_i, R33 = 1 - 2 (q_x^2 + q_y^2) held over a sub-step: r scales the
thrust deviation of the vertical channel as for the other scripts (at level it is the Plant's), the attitude torques are the
card's. Around it, per tick in the composition's order: the attitude group (L5's attitude_law on the plant's quaternion, against
angle mode's level set point: L5's AngleMode with zero sticks, locked at the initial heading, returns a constant, so it is called
once), the vertical group (member_run's law, the plant's z and v at the start of the tick), the gyro chain's low-pass on the
plant's rates every tick, and the rate group (the rate loop in bypass with its filtered D term, the same lines as L5's
Member.attitude_step: L5 has no function for the chain and the law apart from that class, whose plant is its own; the notches are
bypassed as there, the sample carries no rotor speed), the pulse added to the roll torque, L5's Quantiser.allocate (mixer::allocate
in double) at the collective T_cmd, w_c,i = sqrt(f_i / k) (no DShot quantisation: the T4 term Q). The attitude, rate, mixer and
inertia values are L5's fixtures' (equal to the L11 parameter table's, the same product set; the oracle refuses when a key both
fixtures hold differs). The oracle refuses unless T_cmd stays within [T_min, T_max] and the mixer never scales or clamps an
allocation at any member (the script is linear over the box). The shove has no T3 golden, so the tolerance_z term of its F is 0
and its kinematics term is the largest change of the nominal member's z between KIN_SUBSTEPS and twice as many sub-steps; its
segment starts at the golden's length and doubles on its own (its header line), the other scripts' segment is not changed by
it. Its block adds, after thrust_disturbance_n, torque_pulse_axis, torque_pulse_nm, torque_pulse_t0_us, torque_pulse_dur_us and,
after t_cmd_max_n, rotor_cmd_min_n and rotor_cmd_max_n, the range of the four rotor thrust commands f_i (N) over all members and
rate executions (inside [k idle^2, k w_max^2], the DShot range of T4's clean-run predicate).

Quantisation term Q (vertical_t3_q.txt; decision 0020 section 5 "T4": "Q = the box maximum of quantised - unquantised trajectory
through the live diffuser (0006:697-699; 0017)"; L5's method: attitude_t3_oracle.py q_script and write_q, tests/regression/quad/L05/
t3/README.md "The quantisation term Q"). Per script, channel z: the maximum over the vertical executions of |z quantised - z
unquantised| of one box member, then the maximum over the members at the box corners and the nominal (s = t = 0, the box centre)
and over the 9 x 9 grid that contains them; q is the grid maximum where it exceeds the corner maximum (rule grid), else the corner
maximum. The unquantised member is the envelope's (member_run, shove_member_run), over the envelope's own segment length. The
quantised member puts L5's Quantiser (Actuator below: mixer::allocate in double at the vertical law's collective T_cmd, then
mixer::DshotDiffuser, decision 0017, with every carry 0 at the start of the run, once per rate execution, in float32 as the
firmware runs it, then marv_plant's ESC map DShot -> omega) between the law and the plant: the rotor speed command of each motor is
the ESC map's of its DShot value, held until the next rate execution, and the plant's rotor lag and thrust k w^2 follow. The two
vertical scripts have a zero torque request, so each motor's thrust command is T_cmd / 4 (the four are one) and the collective
plant takes the one speed; the roll shove runs the coupled cascade with the actuator in place of w_c,i = sqrt(f_i / k). L5's
Quantiser takes the thrust of the plant's own k omega^2 (the card's double coefficient); here the design plant's coefficient is
the fixture's, so Q is the quantisation alone. The file (every line, in order): '#' header; '# dshot_range_allowed <d_lo> 2047';
hover_dshot_real and hover_dshot_rounded (the DShot value of the stateless rounding at the hover trim T_h, facts of L5's
q_hover_facts); per script 'scenario <name>', segment_executions (H, the envelope's), saturated_executions (0: the run is
refused otherwise), dshot_min and dshot_max (over all members and rate executions) and 'channel z q <q> rule <grid|corners>
q_corners <> q_grid <> at_s <> at_t <> at_execution <>' (T4 reads the value after 'q' as L5's parse_q does; q in m).

Refusals (exit non-zero, nothing written): a missing input; a configuration vertical::validate refuses (gains >= 0, T > 0,
T_h > 0, no k_i without k_v); a non-positive tick or ratio; an inexact float period product; T_h outside [T_min, T_max]; a
gravity gradient that is not positive and finite; a closed loop that is not strictly stable at kappa = 1 or at a frozen kappa;
T_cmd leaving [T_min, T_max] (the golden, or any box member); a roll-shove allocation the mixer scales or clamps; a key that
differs between the vertical fixture and L5's; a non-zero bound at the seed execution; a kinematics change not below the tolerance; no settled
end value within MAX_DOUBLINGS doublings (the golden, or the envelopes); a quantised member that leaves [T_min, T_max], whose
allocation the mixer scales or clamps, or whose DShot value reaches a bound of the diffuser's range [d_lo, 2047] (Q).
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
# L5's fixtures, which the roll shove's cascade reads: the product parameter values of the rate loop, the gyro chain, the attitude
# law and the mixer (equal to the L11 parameter table's, the same product set) and the card's plant side (rotor geometry).
_L5_INPUTS = os.path.join(ROOT, "tests", "regression", "quad", "L05", "t3", "reference", "attitude_t3_inputs.txt")
_L5_Q_INPUTS = os.path.join(ROOT, "tests", "regression", "quad", "L05", "t3", "reference", "attitude_t3_q_inputs.txt")

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

# The roll shove (T4 scenario (iii)): decision 0020 section 5 "T4" table, "0.02 N*m for 0.2 s, the demonstration's values" and
# Owner decisions 7 (Luis, 2026-10-13: "Roll-shove disturbance for T4: the demonstration's 0.02 N·m for 0.2 s as a labelled
# scenario value?" - "Yes"); the roll axis, like the demonstration's gz wrench.
SHOVE_TORQUE_NM = 0.02  # roll torque pulse, N*m, added to the rate loop's torque request before the mixer
SHOVE_DURATION_S = 0.2  # pulse length, s
SHOVE_AXIS = 0          # roll (the composition's l11_dist_torque_axis: 0 roll, 1 pitch, 2 yaw)
SHOVE_EXECUTION = 1     # the pulse starts at the stamp of vertical execution 1, as the other scripts' steps (script(): the seed
#                         execution 0 has no disturbance); the system starts at its trim, so the start only shifts the response in time

SCRIPTS = ("height_step", "thrust_offset")
SHOVE = "roll_shove"  # an envelope script without a golden
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
    if name == SHOVE:
        return 0.0, 0.0
    return 0.0, su.d


# ---- the design plant ---------------------------------------------------------------------------------------------------------


class Plant:
    """The collective rotor speed w, v and z (NED) of the design plant (module docstring), stepped one tick of m_sub sub-steps
    with the rotor command held."""

    def __init__(self, su, m_sub, tau=None, gain=1.0):
        self.su = su
        self.m_sub = m_sub
        s = su.tick_s / m_sub
        tau = su.tau if tau is None else tau
        self.s = s
        self.decay = math.exp(-s / tau)
        e1 = -math.expm1(-s / tau)
        e2 = -math.expm1(-2 * s / tau)
        self.i1 = tau * e1                    # int_0^s exp(-t/tau) dt
        self.i2 = tau / 2 * e2                # int_0^s exp(-2t/tau) dt
        self.j1 = tau * (s - tau * e1)        # int_0^s int_0^t exp(-r/tau) dr dt
        self.j2 = tau / 2 * (s - tau / 2 * e2)  # int_0^s int_0^t exp(-2r/tau) dr dt
        self.a = 4 * su.k / su.mass * gain
        self.bias = (1 - gain) * su.t_h / su.mass  # the trim's share of the thrust the gain does not scale (0 at gain 1)
        self.w = su.w_hover
        self.v = 0.0
        self.z = 0.0

    def step(self, w_c):
        su, s, a = self.su, self.s, self.a
        cc = w_c * w_c
        for _ in range(self.m_sub):
            g = hover.normal_gravity(su.phi, su.h0 - self.z) - self.bias
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


# ---- the T3 box envelopes (recorded, for T4) -----------------------------------------------------------------------------------


class Box:
    """The robustness box of decision 0020 section 4: tau in tau (1 +- b_tau), b_tau = tau_robustness_band (design/budget.yaml),
    and the loop-gain factor r = (k_true / k_card)(m_card / m_true) in [r_lo, r_hi], r_lo = (1 - b_k) min R, r_hi = (1 + b_k)
    max R, b_k = thrust_gain_robustness_band (design/budget.yaml), R = {1} and m_card / m_c for every open conflict m_c of the
    card's mass (vehicles/uzh_neurobem_5in.yaml): the box tools/card/vertical.py designs against, read here from the same two
    files and recomputed by this rule."""

    def __init__(self):
        import yaml  # noqa: PLC0415

        budget = yaml.safe_load(open(os.path.join(ROOT, "design", "budget.yaml")))
        card = yaml.safe_load(open(os.path.join(ROOT, "vehicles", "uzh_neurobem_5in.yaml")))
        mass = float(card["mass"]["value"])
        conflicts = [float(c["value"]) for c in card["mass"].get("conflict", []) or [] if c.get("state") == "open"]
        self.b_tau = float(budget["tau_robustness_band"]["value"])
        self.b_k = float(budget["thrust_gain_robustness_band"]["value"])
        if not (mass > 0 and all(m > 0 for m in conflicts) and 0 <= self.b_tau < 1 and 0 <= self.b_k < 1):
            sys.exit("the box inputs are out of their domain (mass and conflicts > 0, bands in [0, 1))")
        ratios = [1.0] + [mass / m for m in conflicts]
        self.r_lo = (1 - self.b_k) * min(ratios)
        self.r_hi = (1 + self.b_k) * max(ratios)

    def member(self, s, t):
        """(tau factor, r) of the grid point (s, t) in [-1, 1]^2, the end points exact."""
        u = (t + 1) / 2
        return 1 + self.b_tau * s, (1 - u) * self.r_lo + u * self.r_hi


def member_run(su, name, h_seg, tau, r, quant=None, divisor=1):
    """(z per vertical execution, T_cmd range) of one box member: the design plant with the motor lag tau and the thrust
    deviation about the hover trim scaled by r (T = T_h + r (4 k w^2 - T_h), so every member starts at the same exact trim and the
    linearised plant is the one of Lin at kappa = 1 with r the loop gain), the law in double with the f32 gains and no
    rounding, driven by the exact script. It stops at the first command outside the zero-torque collective range. With `quant`
    (an Actuator, the term Q) the rotor command is the actuator's on every rate tick (every `divisor` ticks, after the
    vertical law of the same tick) at zero torque request: each motor's thrust command is T_cmd / 4, allocated, diffused and
    quantised, and the four are one by the symmetric mixer, so one speed commands the collective w."""
    plant = Plant(su, KIN_SUBSTEPS, tau, r)
    integ = e_prev = 0.0
    w_c = su.w_hover
    cmd_lo = cmd_hi = su.t_h
    out = []
    for j in range((1 + h_seg) * su.D):
        if j % su.D == 0:
            z_ref, d = script(name, j // su.D, su)
            z, v = plant.z, plant.v
            e_v = su.kp_z * (z_ref - z) - v
            integ = integ + su.ki_v * e_prev * su.period_s
            t_cmd = su.t_h - (su.kp_v * e_v + integ) + d
            out.append(z)
            e_prev = e_v
            cmd_lo, cmd_hi = min(cmd_lo, t_cmd), max(cmd_hi, t_cmd)
            if not su.t_min <= t_cmd <= su.t_max:
                break
            w_c = math.sqrt(t_cmd / (4 * su.k))
        if quant is not None and j % divisor == 0:
            quant.c = t_cmd
            f, scaled = quant.allocate(0.0, 0.0, 0.0)
            quant.saturated += scaled
            speeds = quant.speeds(f)
            if len(set(speeds)) != 1:
                sys.exit(f"{name}: the four motors' quantised speeds differ at zero torque: the mixer is not symmetric")
            w_c = speeds[0]
        plant.step(w_c)
    return out, (cmd_lo, cmd_hi)


class Cascade:
    """What the roll shove's members share (module docstring, "Roll shove"): L5's fixtures, read and checked against the vertical
    fixture; L5's setup (the gyro low-pass coefficients, the attitude configuration), the rate gains, the D filter coefficient
    and the inertia per axis; L5's mixer (Quantiser.allocate: the mixer in double) with the card's rotor geometry; angle mode's
    level set point (its output with zero sticks, locked at the initial heading, does not depend on the state); the pulse in
    stamps."""

    def __init__(self, su):
        p, p5, q5 = su.p, l5.read_inputs(_L5_INPUTS), l5.read_q_inputs(_L5_Q_INPUTS)
        for key in ("rate_loop_divisor", "tick_period_num_us", "tick_period_den", "motor_tau"):
            if p5[key] != p[key]:
                sys.exit(f"{key} differs between the vertical fixture ({p[key]!r}) and L5's ({p5[key]!r})")
        for key, ours in (("rotor_thrust_coeff", "rotor_thrust_coeff"), ("idle_speed", "rotor_speed_min"),
                          ("rotor_speed_max", "rotor_speed_max")):
            if q5[key] != p[ours]:
                sys.exit(f"{key} differs between L5's mixer fixture ({q5[key]!r}) and the vertical fixture ({p[ours]!r})")
        su5 = l5.Setup(p5)
        self.q5 = q5
        self.cfg, self.lpf, self.period_s = su5.cfg, su5.lpf, su5.period_s
        self.divisor, self.att_divisor = su5.divisor, su5.divisor * su5.ratio
        names = l5.AXES
        self.kp = [p5[f"rate_kp_{a}"] for a in names]
        self.ki = [p5[f"rate_ki_{a}"] for a in names]
        self.kd = [p5[f"rate_kd_{a}"] for a in names]
        self.alpha = [l5.d_alpha(self.period_s, p5[f"rate_d_filter_tau_{a}"])[0] for a in names]
        self.inertia = [p5[l5.INERTIA[a]] for a in names]
        self.mixer = l5.Quantiser(q5, SHOVE_AXIS)
        self.geo = self.mixer.geo
        level = tuple(E(c) for c in (1.0, 0.0, 0.0, 0.0))
        self.q_sp, self.yaw_cmd = l5.AngleMode(self.cfg).execute((0.0, 0.0, 0.0), 0, level, tuple(E(0.0) for _ in range(3)))
        self.t0_us = (SHOVE_EXECUTION * su.D * p["tick_period_num_us"]) // p["tick_period_den"]
        self.dur_us = round(SHOVE_DURATION_S / MICROSECOND)


class CoupledPlant(Plant):
    """The coupled design plant of the roll shove (decision 0020 section 5 "T3 envelopes"): the Plant's constants for the motor lag
    tau, with four rotor speeds w_i, each lagging its own command first order (closed form over a held command), f_i = k w_i^2,
    the body rates and the quaternion (J w' = sum_i geo_i f_i with the card's rotor geometry, no w x J w; q <- q (x) exp(dphi / 2)
    from the exact per-axis angle increments of each sub-step, L5's kinematics) and the vertical channel with the thrust along
    -z_body, v' = g - b - (r / m) R33 sum_i f_i, R33 = 1 - 2 (q_x^2 + q_y^2) held over a sub-step; b = (1 - r) T_h / m as in
    Plant (at level and r = 1 this is Plant)."""

    def __init__(self, su, co, m_sub, tau, gain):
        super().__init__(su, m_sub, tau, gain)
        self.co, self.gain = co, gain
        self.rotor = [su.w_hover] * 4
        self.rate = [0.0] * 3
        self.q = (1.0, 0.0, 0.0, 0.0)

    def step(self, w_c):
        su, co, s = self.su, self.co, self.s
        for _ in range(self.m_sub):
            q = self.q
            r33 = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
            g = hover.normal_gravity(su.phi, su.h0 - self.z) - self.bias
            sum1 = sum2 = 0.0
            t1, t2 = [0.0] * 3, [0.0] * 3
            for i in range(4):
                c = w_c[i]
                dw = self.rotor[i] - c
                lin = 2 * c * dw
                sq = dw * dw
                f1 = su.k * (c * c * s + lin * self.i1 + sq * self.i2)
                f2 = su.k * (c * c * s * s / 2 + lin * self.j1 + sq * self.j2)
                sum1 += f1
                sum2 += f2
                for ax in range(3):
                    t1[ax] += co.geo[i][ax] * f1
                    t2[ax] += co.geo[i][ax] * f2
                self.rotor[i] = c + dw * self.decay
            ratio = self.gain / su.mass * r33
            dv = g * s - ratio * sum1
            dz = self.v * s + g * s * s / 2 - ratio * sum2
            angle = tuple(self.rate[ax] * s + t2[ax] / co.inertia[ax] for ax in range(3))
            self.rate = [self.rate[ax] + t1[ax] / co.inertia[ax] for ax in range(3)]
            self.z += dz
            self.v += dv
            self.q = l5.qmul(self.q, l5.quat_exp(angle))
        norm = math.sqrt(sum(c * c for c in self.q))
        self.q = tuple(c / norm for c in self.q)


def shove_member_run(su, co, h_seg, tau, r, m_sub=KIN_SUBSTEPS, quant=None):
    """(z per vertical execution, T_cmd range, range of the four rotor thrust commands) of one box member of the roll shove: the
    cascade in the composition's per-tick order (fw/compositions/l11_vertical_scripted/src/l11_vertical_scripted.cpp tick) around
    the CoupledPlant, all in double with the f32 gains, from the exact trim. Attitude group (L5's attitude_law on the quaternion
    against angle mode's level set point), vertical group (member_run's law), the gyro chain on the plant's rates every tick, rate
    group (the rate loop in bypass with its filtered D term, the roll pulse added to the torque request, the mixer's allocation at
    the collective T_cmd, w_c,i = sqrt(f_i / k)). The rate group's lines are Member.attitude_step's of L5's oracle (the chain
    and the rate law: L5 has no function for them apart from that class, which holds its own single-axis plant); the notches are
    bypassed as there (no rotor speed in the sample). It stops at the first command outside the zero-torque collective range
    (the series is then short) or at the first allocation the mixer scales or clamps (the rotor range is then None). With `quant`
    (an Actuator, the term Q) the allocation is the actuator's, with its diffuser carries, and the rotor speed commands are the
    ESC map's of the diffused DShot values (no w_c,i = sqrt(f_i / k))."""
    plant = CoupledPlant(su, co, m_sub, tau, r)
    b0, b1, b2, a1, a2 = co.lpf
    x1, x2, y1, y2, y = ([0.0] * 3 for _ in range(5))
    i_int, e_prev_r, y_prev, d_f, u = ([0.0] * 3 for _ in range(5))
    r_hold = (0.0, 0.0, 0.0)
    integ = e_prev = 0.0
    t_cmd = su.t_h
    w_c = [su.w_hover] * 4
    cmd_lo = cmd_hi = su.t_h
    f_lo, f_hi = math.inf, -math.inf
    num, den = su.p["tick_period_num_us"], su.p["tick_period_den"]
    out = []
    for j in range((1 + h_seg) * su.D):
        if j % co.att_divisor == 0:
            r_hold = tuple(c.v for c in l5.attitude_law(co.cfg, tuple(E(c) for c in plant.q), co.q_sp, co.yaw_cmd))
        if j % su.D == 0:
            z_ref, d = script(SHOVE, j // su.D, su)
            z, v = plant.z, plant.v
            e_v = su.kp_z * (z_ref - z) - v
            integ = integ + su.ki_v * e_prev * su.period_s
            t_cmd = su.t_h - (su.kp_v * e_v + integ) + d
            out.append(z)
            e_prev = e_v
            cmd_lo, cmd_hi = min(cmd_lo, t_cmd), max(cmd_hi, t_cmd)
            if not su.t_min <= t_cmd <= su.t_max:
                break
        for i in range(3):
            x = plant.rate[i]
            y[i] = b0 * x + b1 * x1[i] + b2 * x2[i] - a1 * y1[i] - a2 * y2[i]
            x2[i], x1[i], y2[i], y1[i] = x1[i], x, y1[i], y[i]
        if j % co.divisor == 0:
            stamp = (j * num) // den
            pulse = SHOVE_TORQUE_NM if co.t0_us <= stamp < co.t0_us + co.dur_us else 0.0
            for i in range(3):
                i_int[i] += co.ki[i] * co.period_s * e_prev_r[i]
                e = r_hold[i] - y[i]
                d_raw = -co.kd[i] * (y[i] - y_prev[i]) / co.period_s
                d_f[i] = d_f[i] + co.alpha[i] * (d_raw - d_f[i]) if co.alpha[i] != 1.0 else d_raw
                u[i] = co.kp[i] * e + i_int[i] + d_f[i]
                e_prev_r[i], y_prev[i] = e, y[i]
            req = list(u)
            req[SHOVE_AXIS] += pulse
            mixer = co.mixer if quant is None else quant
            mixer.c = t_cmd
            f, scaled = mixer.allocate(*req)
            if scaled:
                return out, (cmd_lo, cmd_hi), None
            f_lo, f_hi = min(f_lo, *f), max(f_hi, *f)
            w_c = [math.sqrt(fi / su.k) for fi in f] if quant is None else quant.speeds(f)
        plant.step(w_c)
    return out, (cmd_lo, cmd_hi), (f_lo, f_hi)


_MEMBER = {}


def _member_job(job):
    _, s, t, coarse = job
    su, box, name = _MEMBER["su"], _MEMBER["box"], _MEMBER["name"]
    tau_factor, r = box.member(s, t)
    if name == SHOVE:
        series, cmd, rotor = shove_member_run(su, _MEMBER["co"], _MEMBER["h"], su.tau * tau_factor, r)
    else:
        series, cmd = member_run(su, name, _MEMBER["h"], su.tau * tau_factor, r)
        rotor = (math.inf, -math.inf)
    return coarse, series, cmd, rotor


def box_envelope(su, box, co, name, h_seg, procs):
    """(Envelope of z over the 17 x 17 box grid and its 9 x 9 subset, the range of T_cmd over all members, the range of the
    rotor thrust commands over all members (the roll shove only))."""
    env = l5.Envelope(1 + h_seg)
    cmd_lo, cmd_hi, f_lo, f_hi = math.inf, -math.inf, math.inf, -math.inf
    _MEMBER.update(su=su, box=box, co=co, name=name, h=h_seg)
    members = l5.grid_members()

    def collect(results):
        nonlocal cmd_lo, cmd_hi, f_lo, f_hi
        for coarse, series, cmd, rotor in results:
            if rotor is None:
                sys.exit(f"{name}: the mixer scales or clamps a box member's allocation: the script is not linear over the box")
            cmd_lo, cmd_hi = _add_member(su, name, h_seg, env, coarse, series, cmd, cmd_lo, cmd_hi)
            f_lo, f_hi = min(f_lo, rotor[0]), max(f_hi, rotor[1])

    if procs > 1:
        import multiprocessing  # noqa: PLC0415

        with multiprocessing.get_context("fork").Pool(procs) as pool:
            collect(pool.imap(_member_job, members, chunksize=1))
    else:
        collect(map(_member_job, members))
    return env, (cmd_lo, cmd_hi), (f_lo, f_hi)


def _add_member(su, name, h_seg, env, coarse, series, cmd, cmd_lo, cmd_hi):
    if len(series) != 1 + h_seg or not su.t_min <= cmd[0] or not cmd[1] <= su.t_max:
        sys.exit(f"{name}: a box member's thrust command leaves the zero-torque collective range "
                 f"[{su.t_min!r}, {su.t_max!r}] N: the script is not linear over the box")
    env.add(series, coarse)
    return min(cmd_lo, cmd[0]), max(cmd_hi, cmd[1])


def shove_kinematics(su, co, h_seg):
    """The roll shove's kinematics halving change: the largest change of z of the nominal member (tau, r = 1) between KIN_SUBSTEPS
    and twice as many sub-steps per tick (the golden's check for the scripts that have one)."""
    coarse = shove_member_run(su, co, h_seg, su.tau, 1.0)[0]
    fine = shove_member_run(su, co, h_seg, su.tau, 1.0, 2 * KIN_SUBSTEPS)[0]
    return max_diff(coarse, fine)


def settle_envelope(su, box, co, golden, procs, names=SCRIPTS):
    """The envelopes of `names` at the golden's segment length, doubled (core 7.5) until every script's end value, the largest
    |z_ref - z| over the envelope at the last execution, is below F = the script's T3 tolerance + the envelope's last-halving
    change + the script's kinematics halving change (E >= 0 adds to the T4 tolerance, so a value below F is below E + F). The
    roll shove has no golden, so no T3 tolerance (its term is 0); its kinematics change is shove_kinematics'."""
    dur = golden["dur_s"]
    for k in range(MAX_DOUBLINGS + 1):
        h = math.ceil(dur / su.period_s)
        res = {"h": h, "dur_s": dur, "doublings": k, "scripts": {}, "checks": []}
        for name in names:
            env, cmd, rotor = box_envelope(su, box, co, name, h, procs)
            z_ref = script(name, h, su)[0]
            end = max(abs(z_ref - env.lo[-1]), abs(z_ref - env.hi[-1]))
            if name == SHOVE:
                terms = (0.0, env.halving_change(), shove_kinematics(su, co, h))
            else:
                g = golden["scripts"][name]
                terms = (g["tol"][1], env.halving_change(), g["kin"][1])
            res["scripts"][name] = {"env": env, "cmd": cmd, "rotor": rotor}
            res["checks"].append((name, end, sum(terms), terms))
        if all(end < f_val for _, end, f_val, _ in res["checks"]):
            return res
        dur *= 2
    sys.exit(f"the envelopes do not settle within {MAX_DOUBLINGS} doublings of the segment length: {res['checks']!r}")


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


def write_envelope(su, box, co, res, shove, path):
    lines = [
        "# T3 box envelopes of the L11 vertical loop (decision 0020 section 5 'T3 envelopes'): pointwise min / max over the tau x",
        "# loop-gain r box (17 x 17 grid, which contains the corners and the 9 x 9 grid) of the design-model response of each",
        "# script, f32 nominal gains, driven by the exact script from the exact initial state. Written by vertical_t3_oracle.py;",
        "# its docstring defines every line. Values are rounded outward to 7 significant digits. halving_max_change is the",
        "# largest change of any point from the 9 x 9 grid to the 17 x 17 grid. A channel lists 'lo hi' for the executions first ..",
        "# first + count - 1.",
        f"# box tau_robustness_band {box.b_tau!r}  thrust_gain_robustness_band {box.b_k!r}  r_lo {box.r_lo!r}  r_hi {box.r_hi!r}",
        f"# segment_seconds {res['dur_s']!r}  segment_executions {res['h']}  doublings {res['doublings']}  "
        f"grid {len(l5.grid_members())}",
        "# settle check (the segment length doubles until every end value is below F = T3 tolerance + halving + kinematics):",
    ]
    for name, end, f_val, terms in res["checks"]:
        lines.append(f"#   {name} z: end {end!r} < F {f_val!r}")
        lines.append(f"#   {name} F terms: tolerance_z {terms[0]!r} halving {terms[1]!r} kinematics_change_z {terms[2]!r}")
    lines.append(f"# {SHOVE} (no golden, so its own segment, started at the golden's length and doubled alone; its tolerance_z is 0): "
                 f"segment_seconds {shove['dur_s']!r}  segment_executions {shove['h']}  doublings {shove['doublings']}")
    for name, end, f_val, terms in shove["checks"]:
        lines.append(f"#   {name} z: end {end!r} < F {f_val!r}")
        lines.append(f"#   {name} F terms: tolerance_z {terms[0]!r} halving {terms[1]!r} kinematics_change_z {terms[2]!r}")
    for name, source in [(n, res) for n in SCRIPTS] + [(SHOVE, shove)]:
        d = source["scripts"][name]
        env = d["env"]
        lines.append(f"scenario {name}")
        lines.append("step_execution 1")
        lines.append(f"z_ref_m {script(name, 1, su)[0]!r}")
        lines.append(f"thrust_disturbance_n {script(name, 1, su)[1]!r}")
        if name == SHOVE:
            lines.append(f"torque_pulse_axis {SHOVE_AXIS}")
            lines.append(f"torque_pulse_nm {SHOVE_TORQUE_NM!r}")
            lines.append(f"torque_pulse_t0_us {co.t0_us}")
            lines.append(f"torque_pulse_dur_us {co.dur_us}")
        lines.append(f"t_cmd_min_n {d['cmd'][0]!r}")
        lines.append(f"t_cmd_max_n {d['cmd'][1]!r}")
        if name == SHOVE:
            lines.append(f"rotor_cmd_min_n {d['rotor'][0]!r}")
            lines.append(f"rotor_cmd_max_n {d['rotor'][1]!r}")
        start, rows = l5.channel_lines(env)
        lines.append(f"channel z first {start} count {len(rows)} halving_max_change {env.halving_change()!r}")
        lines.append("min_max")
        lines.extend(rows)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# ---- the DShot quantisation term Q of the T4 tolerance (decision 0020 section 5 "T4"; decision 0017) ----------------------------


class Actuator(l5.Quantiser):
    """L5's Quantiser as the live actuator, diffusion on (decision 0017), with the commands the vertical oracle's plants take:
    `speeds(f)` is marv_plant's ESC map (sim/plant/src/plant_model.hpp Model::omega_cmd, as Quantiser.torque evaluates it:
    linear in omega over DShot 48 .. 2047 on the card's speed range; DShot 0 is never produced, every value is clamped to at
    least d_lo) of the diffused DShot values of the thrust commands f. The design plants' thrust coefficient is the
    fixture's k and the rotor speed is what the ESC map commands, so Q is the effect of the quantisation alone (L5's torque
    path evaluates the plant's k omega^2 with the card's double k; the f32 and double coefficients differ by 3e-8, a
    plant gain error that the box factor r covers). It records the DShot range seen and counts every value that reaches a
    bound of [d_lo, d_max] (the clamps of the diffuser); `saturated` (Quantiser's) counts the allocations the mixer scales
    or clamps."""

    def __init__(self, q, axis):
        super().__init__(q, axis)
        self.seen_lo, self.seen_hi, self.hits = math.inf, -math.inf, 0

    def clamp(self, d):
        if not self.d_lo <= d <= self.d_max:
            self.hits += 1
        return super().clamp(d)

    def dshot(self, f):
        out = super().dshot(f)
        self.seen_lo, self.seen_hi = min(self.seen_lo, *out), max(self.seen_hi, *out)
        return out

    def speeds(self, f):
        span = float(l5.DSHOT_THROTTLE_MAX - l5.DSHOT_THROTTLE_MIN)
        return [self.p_min + (self.p_max - self.p_min) * (float(d - l5.DSHOT_THROTTLE_MIN) / span) for d in self.dshot(f)]


def _q_member(job):
    """Worker: (index, s, t, max |quantised - unquantised| of z with its execution, series complete, saturated count, DShot
    range) of one member of the 9 x 9 grid; one actuator (fresh carries) per run."""
    index, s, t = job
    su, box, co, name, h = (_MEMBER[k] for k in ("su", "box", "co", "name", "h"))
    tau_factor, r = box.member(s, t)
    tau = su.tau * tau_factor
    act = Actuator(co.q5, SHOVE_AXIS)
    if name == SHOVE:
        unq, quantised = (shove_member_run(su, co, h, tau, r), shove_member_run(su, co, h, tau, r, quant=act))
        act.saturated += unq[2] is None or quantised[2] is None
        unq, quantised = unq[0], quantised[0]
    else:
        unq = member_run(su, name, h, tau, r)[0]
        quantised = member_run(su, name, h, tau, r, act, co.divisor)[0]
    best, at = 0.0, 0
    for i, (x, y) in enumerate(zip(unq, quantised)):
        if abs(y - x) > best:
            best, at = abs(y - x), i
    complete = len(unq) == len(quantised) == 1 + h
    return index, s, t, best, at, complete, act.saturated + act.hits, (act.seen_lo, act.seen_hi)


def q_script(su, box, co, name, h_seg, procs):
    """Q of one script (channel z): the maximum over the time of |quantised - unquantised| of the member's z, at the corners of
    the box plus its nominal (the centre) and over the 9 x 9 grid that contains them; L5's q_script selection. Returns the
    corners, the grid, the one used, the saturation count and the DShot range over the members."""
    jobs = [(i, s, t) for i, s, t, coarse in l5.grid_members() if coarse]
    _MEMBER.update(su=su, box=box, co=co, name=name, h=h_seg)
    if procs > 1:
        import multiprocessing  # noqa: PLC0415

        with multiprocessing.get_context("fork").Pool(procs) as pool:
            results = pool.map(_q_member, jobs, chunksize=1)
    else:
        results = [_q_member(j) for j in jobs]
    if not all(r[5] for r in results):
        sys.exit(f"{name}: a member's run left the zero-torque collective range or the mixer's allocation was scaled: the "
                 "script is not linear over the box")
    saturated = sum(r[6] for r in results)
    if saturated:
        sys.exit(f"{name}: the quantised runs saturate the DShot range or the mixer ({saturated} events): Q is not defined here")
    best_c = best_g = (0.0, None)
    for _, s, t, v, at, _, _, _ in results:
        if v > best_g[0] or best_g[1] is None:
            best_g = (v, (s, t, at))
        if ((abs(s) == 1.0 and abs(t) == 1.0) or (s == 0.0 and t == 0.0)) and (v > best_c[0] or best_c[1] is None):
            best_c = (v, (s, t, at))
    grid_bound = best_g[0] > best_c[0]
    return {"corners": best_c, "grid": best_g, "used": best_g if grid_bound else best_c, "rule": "grid" if grid_bound else "corners",
            "saturated": saturated, "dshot": (min(r[7][0] for r in results), max(r[7][1] for r in results))}


def write_q(su, box, co, segments, path, procs):
    """`segments` {script: segment executions H} (the envelopes' own lengths)."""
    lines = [
        "# T3 quantisation term Q of the L11 T4 tolerance (decision 0020 section 5 'T4': the box maximum of quantised -",
        "# unquantised trajectory through the live diffuser; decision 0017): per script, channel z, the maximum over the time of",
        "# |quantised - unquantised| of the design-model z (m), where the quantised model inserts the actuator (mixer allocation at",
        "# the vertical law's collective T_cmd, the DShot diffuser of decision 0017 as the firmware runs it once per rate",
        "# execution, marv_plant's ESC map DShot -> omega, the rotor lags and thrust of the design plant) between the law and the",
        "# plant. The vertical scripts run at zero torque request (each motor T_cmd / 4); the roll shove runs the coupled cascade.",
        "# Evaluated at the corners of the tau x r box plus the nominal (q_corners), and over the 9 x 9 grid that contains them",
        "# (q_grid); q is q_grid where the grid exceeds the corners (rule grid), else q_corners (rule corners). Written by",
        "# vertical_t3_oracle.py from L5's attitude_t3_q_inputs.txt; its docstring defines every line. Rule: fw/mixer/include/",
        "# marv/mixer/mixer.hpp allocate and DshotDiffuser, sim/plant/src/plant_model.hpp Model::omega_cmd. at = the member",
        "# (s, t) and vertical execution. dshot_min / dshot_max: the DShot values over all members and rate executions, inside",
        "# the diffuser's range [d_lo, 2047] (the run is refused otherwise).",
    ]
    q5 = dict(co.q5, l5_thrust_n=su.t_h)
    facts = l5.q_hover_facts(q5)
    probe = Actuator(co.q5, SHOVE_AXIS)
    lines.append(f"# dshot_range_allowed {probe.d_lo} {l5.DSHOT_THROTTLE_MAX}")
    lines.append(f"hover_dshot_real {facts['hover_dshot_real']!r}")
    lines.append(f"hover_dshot_rounded {facts['hover_dshot_rounded']}")
    for name in SCRIPTS + (SHOVE,):
        r = q_script(su, box, co, name, segments[name], procs)
        c = r["used"]
        lines.append(f"scenario {name}")
        lines.append(f"segment_executions {segments[name]}")
        lines.append(f"saturated_executions {r['saturated']}")
        lines.append(f"dshot_min {r['dshot'][0]}")
        lines.append(f"dshot_max {r['dshot'][1]}")
        lines.append(f"channel z q {c[0]!r} rule {r['rule']} q_corners {r['corners'][0]!r} q_grid {r['grid'][0]!r} "
                     f"at_s {c[1][0]!r} at_t {c[1][1]!r} at_execution {c[1][2]}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=HERE, help="directory of vertical_t3_inputs.txt and of the output")
    ap.add_argument("--refresh-inputs", metavar="PARAM_DEFAULTS_CPP")
    ap.add_argument("--procs", type=int, default=1, help="worker processes of the tolerance, the envelopes and Q (the output does not "
                    "depend on it)")
    ap.add_argument("--golden-only", action="store_true", help="write vertical_t3_golden.txt only (no envelope, no Q)")
    args = ap.parse_args()
    inputs_path = os.path.join(args.dir, "vertical_t3_inputs.txt")
    if args.refresh_inputs:
        refresh_inputs(args.refresh_inputs, inputs_path)
        return
    su = Setup(read_inputs(inputs_path))
    res = settle(su, max(args.procs, 1))
    write_golden(su, res, os.path.join(args.dir, "vertical_t3_golden.txt"))
    if args.golden_only:
        return
    box = Box()
    co = Cascade(su)
    envelopes = settle_envelope(su, box, co, res, max(args.procs, 1))
    shove = settle_envelope(su, box, co, res, max(args.procs, 1), (SHOVE,))
    write_envelope(su, box, co, envelopes, shove, os.path.join(args.dir, "vertical_t3_envelope.txt"))
    segments = {name: envelopes["h"] for name in SCRIPTS}
    segments[SHOVE] = shove["h"]
    write_q(su, box, co, segments, os.path.join(args.dir, "vertical_t3_q.txt"), max(args.procs, 1))


if __name__ == "__main__":
    main()
