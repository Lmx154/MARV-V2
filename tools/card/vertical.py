#!/usr/bin/env python3
"""L11 vertical-loop parameters from a vehicle card, the design budget and the scenario register (quad spec L11,
decision 0020 "The proposal as ruled" sections 1, 2, 4 and 5).

Used by flatten.py (--out-vertical, which needs --scenario). Plain Python double (math, cmath), no numpy, like rate.py
and attitude.py. It is a pure function of (card, budget, scenario) and yields the params_gen entries vert_kp_v,
vert_ki_v, vert_kp_z and vert_loop_ratio, and a derivation report. The vertical axis is a fourth axis of the rate and
attitude cascade (decision 0020 section 2, "Why the cascade"): thrust <-> torque, m <-> J, v_z <-> omega, z <-> theta.
Imported, not copied: rate.py (plant constants, crossover and phase margin, spacing a, design gains, corner margins,
r32, the scan, grid, bisection and derivative-step constants), attitude.py (the lifted loop class Loop, lift, charpoly,
jury, jury_radius, matmul, identity, pm_worst, slope_at, its 2047-point grid), rate_lead.py (GOLDEN_ITERATIONS),
mixer.py (M), tools/sim/hover.py (WGS 84 normal gravity) and tools/sim/reference.py (its height gradient).

Rule (every number comes from the card, the budget register, the scenario register or the mixer; the method constants
are rate.py's and rate_lead.py's, by import, and set resolution and termination only)

  1. Period: T = rate_loop_divisor * tick_period_num_us / tick_period_den microseconds, in seconds (rate.py step 1). The
     vertical loop runs at its parent (rate) loop's rate, vert_loop_ratio = 1 (core 7.5 "Flight rate groups", decision
     0020 section 1 "Rate group", as att_loop_ratio, attitude.py step 9), so its period is T and the lift is N = 1.
  2. Site and hover: g = hover.normal_gravity(phi, h0) (WGS 84 eq. (4-3), fw/prim gravity.hpp:26-35, the plant's g) at the
     register's site_latitude_rad and site_height_m; T_h = m_card g (run_l4.hover_thrust's rule, tools/sim/run_l4.py:204-208);
     gamma = -dg/dh at the site by reference.gravity_height_gradient (the analytic h-derivative of eq. (4-3),
     tools/sim/reference.py:134-145), refused unless positive and finite. T_min, T_max: the mixer's zero-torque collective
     range max_i f_min/M[i,thrust], min_i f_max/M[i,thrust], f = k omega^2 at the speed_range ends (mixer.py:94-96;
     4 k omega^2 on the symmetric card); T_h must lie in it (the linearisation point exists).
  3. Linearised plant at hover (NED, z down, v = dz/dt): dv/dt = gamma z - r dT/m_card, dT = T - T_h the card-referenced
     thrust deviation. dT follows the request through the motor lag tau_m: T = k sum omega_i^2 with each omega_i lagging
     its command first order, so dT = 2 k omega_h d_omega lags dT_req first order with the same tau_m whatever omega_h is
     (small-signal; exact for marv_plant v0 at level and zero torque; INFERRED beyond small signals). The loop-gain
     factor r = (k_true/k_card)(m_card/m_true) multiplies the control path exactly as 1/j (j = J_true/J_a) does in rate.py
     step 2, so a corner is a rate.py corner (name, j = 1/r, tau).
  4. Robustness box (decision 0020 section 4), an explicit corner list (rate.corner_list, rate.py:183-185, is symmetric in
     J and cannot express it): tau in {(1 - b_tau) tau, (1 + b_tau) tau}, b_tau = tau_robustness_band; r in [r_lo, r_hi],
     r_lo = (1 - b_k) min R, r_hi = (1 + b_k) max R, b_k = thrust_gain_robustness_band, R = {1} and m_card/m_c for every
     open conflict m_c of the card's mass (on the committed card R = {1, m_card/m_min}, so r_lo = 1 - b_k and
     r_hi = (1 + b_k) m_card/m_min). Corners: nominal (r = 1, tau) and {r_lo, r_hi} x {tau_lo, tau_hi}.
  5. Inner loop, the vertical-velocity PI: rate.py's rule (steps 2-7, rate.py:258-358) with J -> m_card. The gamma-free
     form 1/(m s (1 + tau s)) motivates the mapping and parameterises the candidates: kappa from rate.design_gains on the
     nominal gamma-free model, C(z) = kp (z - zc)/(z - 1), k_v = m kappa_p, k_i = m kappa_i. Every margin is evaluated
     with the gravity gradient (decision 0020 section 5, "with the gravity gradient in the model"): the plant from the
     normalised command u = -dT_req/m_card to v is P_g(z) = -m_card e_v (zI - Phi)^-1 Gamma, the exact ZOH of the states
     [dT, v, z] of step 7 (z internal), and L = C P_g. It is held as rho = P_g/(P_0/j), P_0 = g (z - z0)/((z - 1)(z - e))
     rate.py's closed form: |L| = |L_0| |rho| and PM = PM_0 + arg rho, PM_0 rate.py's continuous-branch phase (step 3);
     rho lies in the right half-plane on the whole grid (checked; refused otherwise), so its principal argument is the
     continuous one (P_g has the zero at z = 1 that cancels P_0's pole and the poles e^(+-sqrt(gamma) T), so rho is about
     theta^2/(theta^2 + gamma T^2) at low frequency and 1 above). Crossover: rate.py step 3's unique sign change of |L| - 1
     on its grid, then bisection. Feasible at w: PM_worst >= PM_min and Ms_worst <= Ms_max over the corners (step 6).
     w* by rate.py's sup rule (step 5, rate.py:295-324): geometric scan from pi/T 2^-17 in
     steps of 2^(1/16) to the first infeasible point, bisection until (f32(m kappa_p), f32(m kappa_i)) of the midpoint
     equals that of a bracket end. Guard (rate.py step 6, :326-347): on the f32 gains PM_worst >= PM_min + delta_num and
     Ms_worst <= Ms_max, delta_num = (max over the corners of |dPM/dw| by a central difference of step 2^-20 w) x (final
     bracket width); a failing candidate is replaced by the previous feasible point of the history; none passes: refused.
     Rule 7 (rate.py step 7, :353-358): w_c^2 T tau_lo < 1/2 on the chosen design gains at both tau_lo corners: rate.py's
     (J_lo, tau_lo) is (r_hi, tau_lo) here (the J role is 1/r) and decision 0020 section 5 names (r_lo, tau_lo).
  6. Ms = the largest |1/(1 + L)| on the loop's grid, refined by golden section (rate_lead.GOLDEN_ITERATIONS) between the
     grid neighbours of the maximum: rate_lead.py rule step 5 (Model.sensitivity, rate_lead.py:413-440), re-implemented
     because there it is a method of the gyro-chain model. Inner: rate.py's 1024-point grid and L = L_0 rho (step 5).
     Outer: attitude.py's 2047-point grid with L = k F/(z - 1) from the grid's |F| and unwrapped arg F, Loop.response off
     the grid. A grid maximum above Ms_max already decides infeasibility (the refined peak is never below it).
  7. Outer model, the closed velocity loop with gamma (attitude.py step 1's [m, omega, theta, I, e_prev], attitude.py:205-225):
     s = [dT, v, z, I, e_prev] (N, m/s, m, N, m/s), on the f32 gains of step 5; one execution with setpoint v_sp is
         I+ = I + k_i T e_prev,  e_n = v_sp - v,  dT_req = -(k_v e_n + I+)    (T_req = T_h - (k_v e_v + I), decision 0020),
         [dT, v, z]' = Phi [dT, v, z] + Gamma dT_req,  I' = I+,  e_prev' = e_n,
     Phi, Gamma the exact ZOH of d(dT)/dt = (dT_req - dT)/tau, dv/dt = gamma z - r dT/m_card, dz/dt = v: the matrix
     exponential of [[A_c, B_c], [0, 0]] T (rate.py's closed forms carry no gamma), by scaling with 2^-s until the
     infinity norm is <= 1, the Taylor series summed until adding a term changes no entry, and s squarings. The height
     loop closes v_sp = k_z (z_ref - z): negative feedback for k_v, k_i, k_z > 0.
  8. The pole at z = 1: z feeds v through gamma, so attitude.py's factorisation G_N = F(z)/(z - 1) (z feeds no other state,
     attitude.py:21-25 and :257-260) does not hold in these coordinates. The eigenvalue 1 stays exact: the equilibria
     v = e_prev = 0, dT = gamma m z/r, I = -gamma m z/r (the integrator holds the gradient) give the eigenvector w. In the
     basis [e_dT, e_v, w, e_I, e_prev] (x'_i = x_i - w_i z for i != z, x'_z = z) the lifted A^N has column z = e_z and the
     output is still e_z^T, so VerticalLoop (attitude.Loop with __init__ overridden, nothing else) uses attitude.Loop's
     F(z), crossover, unwrapped phase margin and Jury test unchanged; the basis change leaves det(zI - A^N + k B_N C_z)
     unchanged.
  9. Outer gain, the height P: attitude.py's sup rule (step 7, attitude.py:406-452) on L = k_z G_N. Feasible at k: every
     corner has a unique crossover with PM >= PM_min, Ms <= Ms_max (step 6) and a closed loop that passes the Jury test.
     Scan from pi/T 2^-17 in steps of 2^(1/16) to the first infeasible point, bisection until f32(k) stops changing;
     guard on f32(k): PM_worst >= PM_min + delta_num (delta_num = attitude.slope_at x final bracket width), Ms_worst <=
     Ms_max and Jury at every corner; step back through the history; none passes: refused.
 10. Reported (no outputs): T_h, gamma, sqrt(gamma), T_min, T_max; per corner the crossover, PM, Ms and Jury radius of
     both loops on the f32 gains (inner: the spectral radius of the inner closed loop with gamma, A' without its z = 1
     mode; outer: of the closed height loop); the admissible r-interval: on the grid r = 2^(i/16) (rate.py's
     SCAN_STEPS_PER_OCTAVE) outward from r = 1, the run of points at which both loops on the emitted f32 gains keep
     PM >= PM_min, Ms <= Ms_max (and the outer loop Jury-stable) at tau_lo and tau_hi, with the first failing point on
     each side (resolution one grid step; the scan stops at 2^(+-17), rate.py's SCAN_LOG2_START, and says so).
 11. Emitted: vert_kp_v (f32, N s/m), vert_ki_v (f32, N/m), vert_kp_z (f32, 1/s), method derived, sigma UNKNOWN;
     vert_loop_ratio (i32, 1, sigma 0).

The generator refuses (GenError, gpc.refuse with path "vertical") a missing or UNKNOWN input (card mass, motor tau,
thrust coefficient, speed range, an open mass conflict without a value; the budget's PM_min, Ms_max,
tau_robustness_band, thrust_gain_robustness_band; the register's tick period, divisor and site), an input out of its
domain, a hover thrust outside the zero-torque range, no feasible crossover or gain, a failing guard, an unstable corner
and a failing rule 7.
"""

from __future__ import annotations

import cmath
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "sim"))
import attitude  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import hover  # noqa: E402
import mixer  # noqa: E402
import rate  # noqa: E402
import rate_lead  # noqa: E402
import reference  # noqa: E402
import schema  # noqa: E402

Z = attitude.THETA
OTHER = attitude.OTHER
STATE_N = attitude.STATE_N
LOOP_RATIO = 1
CORNER_NAMES = ("r-,tau-", "r-,tau+", "r+,tau-", "r+,tau+")
LO_TAU_CORNERS = ("r-,tau-", "r+,tau-")

GAIN_METHOD = (
    "derived(vertical-velocity PI by the rule of tools/card/rate.py with J -> card mass (decision 0020): integral zero "
    "w_c/a with a = sqrt((1 + sin PM_min)/(1 - sin PM_min)), w_c the largest crossover whose worst phase margin stays "
    ">= PM_min and worst sensitivity peak <= Ms_max over the tau_robustness_band x loop-gain box (thrust_gain_robustness_band "
    "and the mass conflicts) on the exact ZOH model 1/(m s (1 + tau s)) at the rate-loop period with the firmware's "
    "forward-Euler PI, kp = m kappa_p, ki = m kappa_i, f32 under the delta_num guard; tools/card/vertical.py)")
GAIN_SOURCE = ("vehicle card mass (with its open conflicts) and rotors.motor_lag.tau; design-budget PM_min, Ms_max, "
               "tau_robustness_band, thrust_gain_robustness_band; scenario register tick_period_num_us, tick_period_den, "
               "rate_loop_divisor (decision 0020)")
KZ_METHOD = (
    "derived(height P gain by the sup rule of tools/card/attitude.py (decision 0020): k = sup{k' : for every gain in "
    "(0, k'] each corner of the tau_robustness_band x loop-gain box has PM >= PM_min, Ms <= Ms_max and a Jury-stable "
    "closed loop} on the exact lifted sampled-data model of the closed vertical-velocity loop (f32 gains) with the "
    "gravity height gradient at the register's site, f32 under the delta_num guard; tools/card/vertical.py)")
KZ_SOURCE = ("the vertical-velocity gains' inputs; scenario register site_latitude_rad, site_height_m (gravity height "
             "gradient, WGS 84 normal gravity); decision 0020")
N_METHOD = ("derived(parent-rate rule of core contracts 7.5 'Flight rate groups': the vertical loop runs at its parent "
            "(rate) loop's rate, vert_loop_ratio = 1, as att_loop_ratio (decision 0006 E); EMB-3 is re-checked at L9 with "
            "measured WCETs; decision 0020 section 1 'Rate group'; tools/card/vertical.py)")
N_SOURCE = "core contracts 7.5 'Flight rate groups'; decision 0020 section 1 'Rate group'"


def r32(x):
    return rate.r32(x)


def refuse(card_path, reason):
    gpc.refuse(card_path, "vertical", f"{reason} (decision 0020)")


def _value(doc, name, where, card_path):
    if name not in doc or not isinstance(doc[name], dict) or not schema.is_number(doc[name].get("value")):
        refuse(card_path, f"{where} {name} is missing or UNKNOWN")
    return doc[name]["value"]


def _card_number(entry, path, card_path):
    if not schema.is_number(entry.get("value")):
        refuse(card_path, f"card {path} is UNKNOWN or not a number")
    return float(entry["value"])


# ---- step 7: the exact ZOH model with the gravity gradient -----------------------------------------------------------


def expm(m):
    """exp(m): scaled by 2^-s until the infinity norm is <= 1, Taylor series summed until a term changes no entry, then
    squared s times (rule step 7)."""
    n = len(m)
    s = 0
    norm = max(sum(abs(x) for x in row) for row in m)
    while norm > 1:
        norm /= 2
        s += 1
    a = [[x / 2 ** s for x in row] for row in m]
    total = attitude.identity(n)
    term = attitude.identity(n)
    k = 0
    while True:
        k += 1
        term = [[x / k for x in row] for row in attitude.matmul(term, a)]
        nxt = [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(total, term)]
        if nxt == total:
            break
        total = nxt
    for _ in range(s):
        total = attitude.matmul(total, total)
    return total


def zoh(T, tau, r, gamma, mass):
    """exp([[A_c, B_c], [0, 0]] T) of the plant [dT, v, z] with input dT_req (rule step 7): Phi in rows and columns 0-2,
    Gamma in column 3."""
    gen = [[-1 / tau, 0.0, 0.0, 1 / tau],
           [-r / mass, 0.0, gamma, 0.0],
           [0.0, 1.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0]]
    return expm([[x * T for x in row] for row in gen])


def vertical_model(kv, ki, T, tau, r, gamma, mass):
    """(A, B) of s' = A s + B v_sp for s = [dT, v, z, I, e_prev] (rule step 7); kv, ki the physical gains (N s/m, N/m)."""
    e = zoh(T, tau, r, gamma, mass)
    c = [0.0, kv, 0.0, -1.0, -ki * T, -kv]  # dT_req = -(kv (v_sp - v) + I + ki T e_prev) over [s, v_sp]
    a = [[0.0] * (STATE_N + 1) for _ in range(STATE_N)]
    for i in range(3):
        for col in range(STATE_N + 1):
            a[i][col] = e[i][3] * c[col]
        for j in range(3):
            a[i][j] += e[i][j]
    a[3][3] = 1.0
    a[3][4] = ki * T
    a[4][1] = -1.0
    a[4][STATE_N] = 1.0
    return [row[:STATE_N] for row in a], [row[STATE_N] for row in a]


def mode_vector(gamma, mass, r):
    """The eigenvector of eigenvalue 1 (rule step 8), z component 1."""
    held = gamma * mass / r
    return [held, 0.0, 1.0, -held, 0.0]


def to_mode_basis(p, bs, w):
    """(T^-1 p T, T^-1 bs) for T = [e_dT, e_v, w, e_I, e_prev] (w_z = 1, so T^-1 = I - (w - e_z) e_z^T exactly)."""
    n = len(w)
    pt = [row[:] for row in p]
    for i in range(n):
        pt[i][Z] = sum(p[i][k] * w[k] for k in range(n))
    out = [[pt[i][j] - w[i] * pt[Z][j] if i != Z else pt[i][j] for j in range(n)] for i in range(n)]
    b = [bs[i] - w[i] * bs[Z] if i != Z else bs[i] for i in range(n)]
    return out, b


class VerticalLoop(attitude.Loop):
    """The lifted height loop G_N of one corner (rule steps 7-8): attitude.Loop on the mode basis; only __init__ differs."""

    def __init__(self, kv, ki, T, tau, r, gamma, mass, n):
        a, b = vertical_model(kv, ki, T, tau, r, gamma, mass)
        p, bs = attitude.lift(a, b, n)
        self.p, self.bs = to_mode_basis(p, bs, mode_vector(gamma, mass, r))
        self.m4 = [[self.p[i][j] for j in OTHER] for i in OTHER]
        self.b4 = [self.bs[i] for i in OTHER]
        self.ell = [self.p[Z][j] for j in OTHER]
        self.d = self.bs[Z]
        self._evaluate_grid()
        g = attitude._GRID
        self.over_z1 = [h * cmath.exp(1j * (u - math.pi / 2 - t / 2)) for h, u, t in zip(self.h, self.unwrapped, g.theta)]

    def inner_radius(self):
        """Spectral radius of the inner closed loop with gamma (A' without its z = 1 mode), None when not below 1."""
        return attitude.jury_radius(attitude.charpoly(self.m4))

    def sensitivity(self, k, bound=None):
        """(Ms, theta of the peak) of L = k F/(z - 1) (rule step 6)."""
        vals = [1 / abs(1 + k * x) for x in self.over_z1]

        def s(th):
            z = cmath.exp(1j * th)
            return 1 / abs(1 + k * self.response(z) / (z - 1))

        return ms_peak(vals, attitude._GRID.theta, s, bound)


# ---- step 6: the sensitivity peak ------------------------------------------------------------------------------------


def ms_peak(vals, theta, s, bound=None):
    """(Ms, theta): the grid maximum of s (values vals on theta) refined by golden section between its grid neighbours
    (rate_lead.py rule step 5, Model.sensitivity). With `bound`, a grid maximum above it is returned unrefined."""
    i = max(range(len(theta)), key=vals.__getitem__)
    if bound is not None and vals[i] > bound:
        return vals[i], theta[i]
    lo, hi = theta[max(i - 1, 0)], theta[min(i + 1, len(theta) - 1)]
    g = (math.sqrt(5) - 1) / 2
    x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
    f1, f2 = s(x1), s(x2)
    for _ in range(rate_lead.GOLDEN_ITERATIONS):
        if f1 >= f2:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - g * (hi - lo)
            f1 = s(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + g * (hi - lo)
            f2 = s(x2)
    return max((vals[i], theta[i]), (f1, x1), (f2, x2))


class InnerPlant:
    """The inner loop's plant at one corner (name, j, tau) with the gravity gradient (rule step 5): rho = P_g/(P_0/j) on
    rate.py's grid, and off the grid by ratio()."""

    def __init__(self, corner, T, gamma, mass):
        self.name, self.jt, tau = corner
        self.T, self.mass = T, mass
        self.pc = rate.plant_constants(tau, T)
        e = zoh(T, tau, 1 / self.jt, gamma, mass)
        self.phi = [row[:3] for row in e[:3]]
        self.gam = [e[i][3] for i in range(3)]
        g = rate._GRID
        self.rho = [self.ratio(c, sn, h) for c, sn, h in zip(g.cos, g.sin, g.half)]
        self.rhp_ok = all(x.real > 0 for x in self.rho)

    def ratio(self, c, sn, half):
        """rho at z = c + j sn on the unit circle, half = sin(theta/2) (z - 1 = -2 half^2 + j sn, without cancellation)."""
        z = complex(c, sn)
        a = [[(z if i == j else 0) - self.phi[i][j] for j in range(3)] for i in range(3)]
        p_g = -self.mass * attitude.csolve(a, self.gam)[1]
        e, _, g, z0 = self.pc
        return p_g * complex(-2 * half * half, sn) * (z - e) * self.jt / (g * (z - z0))

    def ell0(self, kp, zc, c, sn, half):
        """rate.py's gamma-free L_0 = C P_0/j at z = c + j sn."""
        e, _, g, z0 = self.pc
        z = complex(c, sn)
        zm1 = complex(-2 * half * half, sn)
        return kp / self.jt * g * (z - zc) * (z - z0) / (zm1 * zm1 * (z - e))


def inner_margin(kp, ki, plant):
    """(crossover angle, PM, reason) of L = C P_g (rule step 5): rate.loop_crossover's grid, checks and bisection with
    |L| = |L_0| |rho|, PM = PM_0 + arg rho; angle and PM are None without a unique crossover."""
    T, pc = plant.T, plant.pc
    zc = 1 - ki * T / kp
    kj = kp / plant.jt
    g = rate._GRID

    def mag(c, sn, half, rho):
        return rate._mag(c, sn, half, kj, zc, pc) * abs(rho)

    above = [mag(g.cos[i], g.sin[i], g.half[i], plant.rho[i]) > 1 for i in range(rate.GRID_POINTS)]
    changes = [i for i in range(rate.GRID_POINTS - 1) if above[i] != above[i + 1]]
    if not above[0] or above[-1]:
        return None, None, "|L| does not fall from above 1 at low frequency to below 1 at Nyquist"
    if len(changes) != 1:
        return None, None, f"|L| = 1 has {len(changes)} solutions on the grid"
    lo, hi = g.theta[changes[0]], g.theta[changes[0] + 1]
    for _ in range(rate.MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        c, sn, half = math.cos(mid), math.sin(mid), math.sin(mid / 2)
        if mag(c, sn, half, plant.ratio(c, sn, half)) > 1:
            lo = mid
        else:
            hi = mid
    theta = (lo + hi) / 2
    c, sn = math.cos(theta), math.sin(theta)
    rho = plant.ratio(c, sn, math.sin(theta / 2))
    if not rho.real > 0:
        return None, None, "rho leaves the right half-plane at the crossover"
    return theta, rate._pm(c, sn, theta, zc, pc) + cmath.phase(rho), ""


def inner_sensitivity(kp, ki, plant, bound=None):
    """(Ms, theta) of the inner loop L = L_0 rho at a corner; kp, ki normalised."""
    zc = 1 - ki * plant.T / kp
    g = rate._GRID
    vals = [1 / abs(1 + plant.ell0(kp, zc, c, sn, h) * rho)
            for c, sn, h, rho in zip(g.cos, g.sin, g.half, plant.rho)]

    def s(th):
        c, sn, half = math.cos(th), math.sin(th), math.sin(th / 2)
        return 1 / abs(1 + plant.ell0(kp, zc, c, sn, half) * plant.ratio(c, sn, half))

    return ms_peak(vals, g.theta, s, bound)


def inner_rows(kp, ki, plants, ms_max=None):
    """[(name, PM, crossover rad/s, reason, Ms, Ms peak rad/s)] over the corners' plants."""
    out = []
    for plant in plants:
        theta, pm, why = inner_margin(kp, ki, plant)
        ms, th = (None, None) if pm is None else inner_sensitivity(kp, ki, plant, ms_max)
        out.append((plant.name, pm, None if theta is None else theta / plant.T, why, ms,
                    None if th is None else th / plant.T))
    return out


def inner_ok(kp, ki, plants, pm_min, ms_max):
    for plant in plants:
        _, pm, _ = inner_margin(kp, ki, plant)
        if pm is None or pm < pm_min:
            return False
        if inner_sensitivity(kp, ki, plant, ms_max)[0] > ms_max:
            return False
    return True


def inner_plants(corners, T, gamma, mass, refuse_=None):
    """[InnerPlant] of the corners; refuse_ (when given) if rho leaves the right half-plane, otherwise None."""
    plants = [InnerPlant(c, T, gamma, mass) for c in corners]
    if not all(p.rhp_ok for p in plants):
        if refuse_ is not None:
            refuse_("the gravity-gradient factor rho = P_g/P_0 leaves the right half-plane on the grid: the inner phase "
                    "branch is not established")
        return None
    return plants


def outer_ok(loops, k, pm_min, ms_max):
    for _, _, loop in loops:
        _, pm, _ = loop.margin(k)
        if pm is None or pm < pm_min or not loop.stable(k):
            return False
        if loop.sensitivity(k, ms_max)[0] > ms_max:
            return False
    return True


def outer_rows(loops, k):
    """attitude.pm_worst's detail with the Ms peak: (worst PM, worst Ms, [(name, PM, theta, Jury radius, reason, Ms,
    Ms theta)])."""
    worst, detail = attitude.pm_worst(loops, k)
    rows, ms_worst = [], -math.inf
    for (_, name, pm, theta, radius, why), (_, _, loop) in zip(detail, loops):
        ms, mth = loop.sensitivity(k)
        ms_worst = max(ms_worst, ms)
        rows.append((name, pm, theta, radius, why, ms, mth))
    return worst, ms_worst, rows


# ---- the sup rule (rate.py step 5 and attitude.py step 7) -------------------------------------------------------------


def sup_scan(period, feasible, key, refuse_, what):
    """Scan from pi/period 2^SCAN_LOG2_START in steps of 2^(1/SCAN_STEPS_PER_OCTAVE) to the first infeasible point,
    then bisection until key(mid) equals a bracket end's key (rate.py:295-324, attitude.py:406-436)."""
    ratio = 2.0 ** (1 / rate.SCAN_STEPS_PER_OCTAVE)
    x = math.pi / period * 2.0 ** rate.SCAN_LOG2_START
    if not feasible(x):
        refuse_(f"no feasible {what}: the scan start {x:.6g} already violates PM_min, Ms_max or the Jury test")
    history, lo, hi = [], None, None
    while lo is None:
        history.append(x)
        nxt = x * ratio
        if nxt * period >= math.pi:
            refuse_(f"the {what}'s margins hold up to Nyquist: the scan finds no bracket")
        if feasible(nxt):
            x = nxt
        else:
            lo, hi = x, nxt
    scan_bracket = (lo, hi)
    bisections = 0
    for _ in range(rate.MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if key(mid) in (key(lo), key(hi)):
            break
        bisections += 1
        if feasible(mid):
            lo = mid
            history.append(mid)
        else:
            hi = mid
    return {"history": history, "bracket": (lo, hi), "bracket_width": hi - lo, "scan_bracket": scan_bracket,
            "bisections": bisections}


def _choose(scan, passes):
    """The last history point whose f32 design passes the guard: (candidate, guard data, steps back) or None."""
    history = scan["history"]
    for back, cand in enumerate(reversed(history)):
        data = passes(cand)
        if data is not None:
            return cand, data, back
    return None


# ---- the design ------------------------------------------------------------------------------------------------------


def corner_list(tau, b_tau, r_lo, r_hi):
    """Rule step 4: [(name, j = 1/r, tau)], nominal first."""
    lo_t, hi_t = (1 - b_tau) * tau, (1 + b_tau) * tau
    return [("nominal", 1.0, tau), ("r-,tau-", 1 / r_lo, lo_t), ("r-,tau+", 1 / r_lo, hi_t),
            ("r+,tau-", 1 / r_hi, lo_t), ("r+,tau+", 1 / r_hi, hi_t)]


def build_loops(gains32, T, corners, gamma, mass, n, refuse_=None):
    """[("vertical", corner name, VerticalLoop)] on the f32 gains (k_v, k_i); refuse_ (when given) on a failed low-frequency
    or branch check (attitude.py's loops_at), otherwise None for such a set."""
    out = []
    for name, jt, tau in corners:
        loop = VerticalLoop(gains32[0], gains32[1], T, tau, 1 / jt, gamma, mass, n)
        if not (loop.start_ok and loop.branch_ok):
            if refuse_ is not None:
                refuse_(f"corner {name}: the low-frequency phase of F is not near 0 or the unwrapped phase changes under "
                        "one grid doubling")
            return None
        out.append(("vertical", name, loop))
    return out


def inputs(card, budget, scenario, card_path):
    """The validated inputs of rule steps 1-4 (raises gpc.GenError)."""
    mass_entry = card.get("mass", {})
    mass = _card_number(mass_entry, "mass", card_path)
    conflicts = []
    for c in mass_entry.get("conflict", []) or []:
        if c.get("state") == "open":
            if not schema.is_number(c.get("value")):
                refuse(card_path, "card mass has an open conflict without a numeric value")
            conflicts.append(float(c["value"]))
    rotors = card.get("rotors", {})
    tau = _card_number(rotors.get("motor_lag", {}).get("tau", {}), "rotors.motor_lag.tau", card_path)
    k_thrust = _card_number(rotors.get("thrust_coeff", {}), "rotors.thrust_coeff", card_path)
    speed = rotors.get("speed_range", {}).get("value")
    if not (isinstance(speed, list) and all(schema.is_number(v) for v in speed)):
        refuse(card_path, "card rotors.speed_range is UNKNOWN or not a number range")
    pm_min = float(_value(budget, "PM_min", "budget entry", card_path))
    ms_max = float(_value(budget, "Ms_max", "budget entry", card_path))
    b_tau = float(_value(budget, "tau_robustness_band", "budget entry", card_path))
    b_k = float(_value(budget, "thrust_gain_robustness_band", "budget entry", card_path))
    num = _value(scenario, "tick_period_num_us", "scenario entry", card_path)
    den = _value(scenario, "tick_period_den", "scenario entry", card_path)
    divisor = _value(scenario, "rate_loop_divisor", "scenario entry", card_path)
    phi = float(_value(scenario, "site_latitude_rad", "scenario entry", card_path))
    h0 = float(_value(scenario, "site_height_m", "scenario entry", card_path))
    if not (mass > 0 and all(m > 0 for m in conflicts) and tau > 0 and k_thrust > 0 and 0 < pm_min < math.pi / 2
            and ms_max > 0 and 0 <= b_tau < 1 and 0 <= b_k < 1 and num > 0 and den > 0 and divisor >= 1
            and abs(phi) <= math.pi / 2 and math.isfinite(h0)):
        refuse(card_path, "an input is out of its domain (mass, conflicts, tau, thrust_coeff > 0; 0 < PM_min < pi/2; "
                          "Ms_max > 0; bands in [0, 1); tick period > 0; divisor >= 1; |site latitude| <= pi/2)")
    ratios = [1.0] + [mass / m for m in conflicts]
    return {"mass": mass, "conflicts": conflicts, "tau": tau, "k_thrust": k_thrust, "speed": [float(v) for v in speed],
            "PM_min": pm_min, "Ms_max": ms_max, "b_tau": b_tau, "b_k": b_k, "num": num, "den": den,
            "divisor": divisor, "phi": phi, "h0": h0, "ratios": ratios,
            "r_lo": (1 - b_k) * min(ratios), "r_hi": (1 + b_k) * max(ratios)}


def hover_terms(card, i, card_path):
    """Rule step 2: g, T_h, gamma, T_min, T_max."""
    g = hover.normal_gravity(i["phi"], i["h0"])
    gamma = -reference.gravity_height_gradient(i["phi"], i["h0"])
    if not (math.isfinite(gamma) and gamma > 0):
        refuse(card_path, f"the gravity height gradient gamma = {gamma!r} 1/s^2 is not positive and finite")
    m_matrix, _, idle = mixer.mixer_matrix(card, card_path)
    f_min, f_max = i["k_thrust"] * idle * idle, i["k_thrust"] * i["speed"][1] ** 2
    t_min = max(f_min / row[0] for row in m_matrix)
    t_max = min(f_max / row[0] for row in m_matrix)
    t_h = i["mass"] * g
    if not t_min <= t_h <= t_max:
        refuse(card_path, f"the hover thrust {t_h!r} N lies outside the zero-torque collective range "
                          f"[{t_min!r}, {t_max!r}] N")
    return {"g": g, "T_h": t_h, "gamma": gamma, "T_min": t_min, "T_max": t_max, "f_min": f_min, "f_max": f_max}


def inner_design(i, T, corners, gamma, card_path):
    """Rule step 5: the vertical-velocity PI."""
    pm_min, ms_max, mass = i["PM_min"], i["Ms_max"], i["mass"]
    plants = inner_plants(corners, T, gamma, mass, lambda why: refuse(card_path, why))
    a = rate.spacing(pm_min)
    pc_nom = rate.plant_constants(i["tau"], T)

    def normalised(omega):
        return rate.design_gains(omega, a, T, pc_nom)

    def f32_gains(omega):
        kp, ki = normalised(omega)
        return r32(mass * kp), r32(mass * ki)

    def feasible(omega):
        return inner_ok(*normalised(omega), plants, pm_min, ms_max)

    scan = sup_scan(T, feasible, f32_gains, lambda why: refuse(card_path, why), "velocity crossover")
    lo = scan["bracket"][0]
    step = lo * 2.0 ** rate.DERIVATIVE_LOG2_STEP
    slope = 0.0
    up, down = normalised(lo + step), normalised(lo - step)
    for plant in plants:
        pu, pd = inner_margin(*up, plant)[1], inner_margin(*down, plant)[1]
        if pu is None or pd is None:
            slope = math.inf
            break
        slope = max(slope, abs(pu - pd) / (2 * step))
    delta_num = slope * scan["bracket_width"]

    def passes(cand):
        kv32, ki32 = f32_gains(cand)
        rows = inner_rows(kv32 / mass, ki32 / mass, plants)
        if any(r[1] is None for r in rows):
            return None
        pm = min(r[1] for r in rows)
        ms = max(r[4] for r in rows)
        if pm >= pm_min + delta_num and ms <= ms_max:
            return (kv32, ki32), rows, pm, ms
        return None

    chosen = _choose(scan, passes)
    if chosen is None:
        refuse(card_path, "no feasible velocity crossover passes the guard (PM_worst >= PM_min + delta_num, "
                          "Ms_worst <= Ms_max on the f32 gains)")
    omega_c, (gains32, rows32, pm32, ms32), back = chosen
    kp_n, ki_n = normalised(omega_c)
    rows_double = inner_rows(kp_n, ki_n, plants)
    tau_lo = (1 - i["b_tau"]) * i["tau"]
    conditions = [(r[0], r[2] ** 2 * T * tau_lo) for r in rows_double if r[0] in LO_TAU_CORNERS]
    for name, cond in conditions:
        if not cond < 0.5:
            refuse(card_path, f"the monotonicity condition w_c^2 T tau_lo = {cond:.6g} is not below 1/2 at the corner "
                              f"({name})")
    return {"a": a, "omega_c": omega_c, "kappa_p": kp_n, "kappa_i": ki_n, "kv": gains32[0], "ki": gains32[1],
            "kv_double": mass * kp_n, "ki_double": mass * ki_n, "rows": rows32, "rows_double": rows_double,
            "pm_worst": pm32, "ms_worst": ms32, "pm_worst_double": min(r[1] for r in rows_double),
            "delta_num": delta_num, "slope": slope, "steps_back": back, "stepped_down": back > 0,
            "conditions": conditions, "plants": plants, **scan}


def outer_design(i, T, loops, card_path):
    """Rule step 9: the height P on the lifted loops."""
    pm_min, ms_max = i["PM_min"], i["Ms_max"]
    t_a = LOOP_RATIO * T
    scan = sup_scan(t_a, lambda k: outer_ok(loops, k, pm_min, ms_max), r32, lambda why: refuse(card_path, why),
                    "height gain")
    slope = attitude.slope_at(loops, scan["bracket"][0])
    delta_num = slope * scan["bracket_width"]

    def passes(cand):
        k32 = r32(cand)
        pm, ms, rows = outer_rows(loops, k32)
        if pm >= pm_min + delta_num and ms <= ms_max and all(r[3] is not None for r in rows):
            return k32, pm, ms, rows
        return None

    chosen = _choose(scan, passes)
    if chosen is None:
        refuse(card_path, "no feasible height gain passes the guard (PM_worst >= PM_min + delta_num, Ms_worst <= "
                          "Ms_max, Jury at every corner on f32(k))")
    k, (k32, pm, ms, rows), back = chosen
    return {"k": k, "k32": k32, "pm_worst": pm, "ms_worst": ms, "rows": rows, "slope": slope, "delta_num": delta_num,
            "steps_back": back, "stepped_down": back > 0, **scan}


def r_admissible(i, T, gains32, k32, gamma):
    """Rule step 10's admissible r-interval: {lo, hi, lo_fail, hi_fail, lo_capped, hi_capped} on the r grid."""
    mass = i["mass"]
    taus = ((1 - i["b_tau"]) * i["tau"], (1 + i["b_tau"]) * i["tau"])
    cap = -rate.SCAN_LOG2_START * rate.SCAN_STEPS_PER_OCTAVE

    def ok(r):
        corners = [(f"r={r!r},tau={t!r}", 1 / r, t) for t in taus]
        plants = inner_plants(corners, T, gamma, mass)
        if plants is None or not inner_ok(gains32[0] / mass, gains32[1] / mass, plants, i["PM_min"], i["Ms_max"]):
            return False
        loops = build_loops(gains32, T, corners, gamma, mass, LOOP_RATIO)
        return loops is not None and outer_ok(loops, k32, i["PM_min"], i["Ms_max"])

    out = {"centre_ok": ok(1.0)}
    for side, sign in (("hi", 1), ("lo", -1)):
        last, fail, capped = 1.0, None, True
        for n in range(1, cap + 1):
            r = 2.0 ** (sign * n / rate.SCAN_STEPS_PER_OCTAVE)
            if ok(r):
                last = r
            else:
                fail, capped = r, False
                break
        out[side], out[f"{side}_fail"], out[f"{side}_capped"] = last, fail, capped
    return out


def design(card, budget, scenario, card_path):
    """The whole rule: a dict of results (raises gpc.GenError when the inputs are refused)."""
    i = inputs(card, budget, scenario, card_path)
    T = i["divisor"] * i["num"] / i["den"] * 1e-6  # µs to s (SI prefix micro)
    hv = hover_terms(card, i, card_path)
    corners = corner_list(i["tau"], i["b_tau"], i["r_lo"], i["r_hi"])
    inner = inner_design(i, T, corners, hv["gamma"], card_path)
    gains32 = (inner["kv"], inner["ki"])
    loops = build_loops(gains32, T, corners, hv["gamma"], i["mass"], LOOP_RATIO,
                        lambda why: refuse(card_path, why))
    inner["jury"] = [(name, loop.inner_radius()) for _, name, loop in loops]
    if any(rad is None for _, rad in inner["jury"]):
        refuse(card_path, "the inner closed loop with the gravity gradient is unstable at a corner (Jury test)")
    outer = outer_design(i, T, loops, card_path)
    if not all(loop.radius(outer["k32"]) is not None for _, _, loop in loops):
        refuse(card_path, "the final gains are unstable at a corner (Jury test)")
    interval = r_admissible(i, T, gains32, outer["k32"], hv["gamma"])
    return {"T": T, "N": LOOP_RATIO, "inputs": i, "hover": hv, "corners": corners, "inner": inner, "outer": outer,
            "r_interval": interval, "loops": loops}


def entries_from(result):
    inner, outer = result["inner"], result["outer"]
    return [("vert_kp_v", {"type": "f32", "value": inner["kv"], "unit": "N s/m", "method": GAIN_METHOD,
                           "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}),
            ("vert_ki_v", {"type": "f32", "value": inner["ki"], "unit": "N/m", "method": GAIN_METHOD,
                           "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}),
            ("vert_kp_z", {"type": "f32", "value": outer["k32"], "unit": "1/s", "method": KZ_METHOD,
                           "source": KZ_SOURCE, "sigma": schema.UNKNOWN}),
            ("vert_loop_ratio", {"type": "i32", "value": result["N"], "unit": "1", "method": N_METHOD,
                                 "source": N_SOURCE, "sigma": 0})]


def report_text(result, where):
    r, i, hv, deg = result, result["inputs"], result["hover"], math.degrees
    inner, outer, iv = r["inner"], r["outer"], r["r_interval"]
    t = r["T"]
    jury = dict(inner["jury"])
    lines = [
        "MARV L11 vertical-loop derivation (tools/card/vertical.py, decision 0020)",
        f"card                     {where}",
        f"mass m_card (kg)         {i['mass']!r}  open conflicts {i['conflicts']!r}",
        f"motor tau (s)            {i['tau']!r}",
        f"PM_min (rad, deg)        {i['PM_min']!r}  {deg(i['PM_min']):.6f}",
        f"Ms_max                   {i['Ms_max']!r}",
        f"bands                    tau +-{i['b_tau']!r}  thrust gain +-{i['b_k']!r}",
        f"T (s)                    {t!r}  ({1 / t:.6g} Hz), vert_loop_ratio {r['N']} (parent-rate rule)",
        f"site                     latitude {i['phi']!r} rad, height {i['h0']!r} m",
        "derived values",
        f"  g(phi, h0) (m/s^2)     {hv['g']!r}",
        f"  T_h = m g (N)          {hv['T_h']!r}",
        f"  gamma = -dg/dh (1/s^2) {hv['gamma']!r}  sqrt(gamma) {math.sqrt(hv['gamma'])!r} rad/s",
        f"  T_min, T_max (N)       {hv['T_min']!r}  {hv['T_max']!r}  (mixer zero-torque collective range)",
        f"  mass ratios R          {i['ratios']!r}",
        f"  r box                  r_lo {i['r_lo']!r}  r_hi {i['r_hi']!r}",
        "model: dv/dt = gamma z - r dT/m_card, dT lagging dT_req by tau (small-signal T = k sum omega^2; INFERRED beyond",
        "  small signals); T_req = T_h - (k_v e_v + I), e_v = k_z (z_ref - z) - v_z; every margin, Ms and Jury radius below",
        "  with gamma (inner: L = C P_g, P_g the exact ZOH of [dT, v, z] from dT_req to v; outer: the lifted closed loop).",
        "",
        "inner loop: vertical-velocity PI (rate.py's rule, J -> m_card)",
        f"  spacing a              {inner['a']!r}",
        f"  w_c,nom (rad/s)        {inner['omega_c']!r}  (the gamma-free design parameter; sqrt(gamma)/w_c "
        f"{math.sqrt(hv['gamma']) / inner['omega_c']:.3e})",
        f"  kappa_p, kappa_i       {inner['kappa_p']!r}  {inner['kappa_i']!r}",
        f"  k_v (f32, N s/m)       {inner['kv']!r}  (double {inner['kv_double']!r})",
        f"  k_i (f32, N/m)         {inner['ki']!r}  (double {inner['ki_double']!r})",
        f"  scan bracket (rad/s)   {inner['scan_bracket'][0]!r} .. {inner['scan_bracket'][1]!r}",
        f"  bisections             {inner['bisections']}  final bracket width {inner['bracket_width']!r} rad/s",
        f"  guard step-backs       {inner['steps_back']}",
        f"  delta_num (rad)        {inner['delta_num']!r}  (max |dPM/dw| {inner['slope']!r})",
        f"  PM_worst f32 (deg)     {deg(inner['pm_worst']):.6f}  (double gains {deg(inner['pm_worst_double']):.6f})",
        f"  Ms_worst f32           {inner['ms_worst']!r}",
        "  rule 7 w_c^2 T tau_lo  " + "  ".join(f"{n} {c!r}" for n, c in inner["conditions"]) + "  (each < 1/2)",
        "  corner    crossover rad/s        PM deg       Ms                   Jury radius (all with gamma)",
    ]
    for name, pm, wc, _, ms, _ in inner["rows"]:
        lines.append(f"  {name:<8}  {wc!r:<22} {deg(pm):<12.6f} {ms!r:<20} {jury[name]!r}")
    lines += [
        "",
        f"outer loop: height P (attitude.py's sup rule) on the lifted closed velocity loop, N = {r['N']}",
        f"  k_z (f32, 1/s)         {outer['k32']!r}  (double {outer['k']!r})",
        f"  scan bracket (1/s)     {outer['scan_bracket'][0]!r} .. {outer['scan_bracket'][1]!r}",
        f"  bisections             {outer['bisections']}  final bracket {outer['bracket'][0]!r} .. "
        f"{outer['bracket'][1]!r}",
        f"  guard step-backs       {outer['steps_back']}",
        f"  delta_num (rad)        {outer['delta_num']!r}  (max |dPM/dk| {outer['slope']!r})",
        f"  PM_worst f32 (deg)     {deg(outer['pm_worst']):.6f}",
        f"  Ms_worst f32           {outer['ms_worst']!r}",
        "  corner    crossover rad/s        PM deg       Ms                   Jury radius (closed loop)",
    ]
    for name, pm, theta, radius, _, ms, _ in outer["rows"]:
        lines.append(f"  {name:<8}  {theta / t!r:<22} {deg(pm):<12.6f} {ms!r:<20} {radius!r}")

    def edge(side):
        if iv[f"{side}_capped"]:
            return f"{iv[side]!r} (scan limit reached)"
        return f"{iv[side]!r} (next grid point {iv[f'{side}_fail']!r} fails)"

    inside = iv["centre_ok"] and iv["lo"] <= i["r_lo"] and i["r_hi"] <= iv["hi"]
    lines += [
        "",
        "admissible loop-gain factor r (emitted f32 gains, both loops, tau_lo and tau_hi; grid 2^(i/16) from r = 1)",
        f"  r = 1 passes           {iv['centre_ok']}",
        f"  lowest r               {edge('lo')}",
        f"  highest r              {edge('hi')}",
        f"  r box inside           {inside}",
    ]
    return lines


def vertical_entries(card, budget, scenario, card_path, where=None):
    """(ordered (name, params_gen entry) list, report lines, result dict); raises gpc.GenError on refusal. `where` is
    the card path written into the report (default card_path)."""
    result = design(card, budget, scenario, card_path)
    return entries_from(result), report_text(result, where or card_path), result
