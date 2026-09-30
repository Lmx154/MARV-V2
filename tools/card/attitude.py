#!/usr/bin/env python3
"""L5 attitude-loop parameters from a vehicle card, the design budget, the scenario register and the SIM-7 uncertainty U
(quad spec L5, QF-3, SIM-7, decision 0006 section E, decision 0005 "Gain rule").

Used by flatten.py (--out-attitude, which needs --scenario and --sim7-u). Plain Python double (math, cmath), no numpy, like
rate.py. It is a pure function of (card, budget, scenario, U) and yields the params_gen entries att_kp, att_yaw_weight,
att_loop_ratio, att_yaw_alpha_min and att_yaw_t_cross, and a derivation report that holds the SIM-7 halving table (core 7.5).

Model (0006 E)

  1. Rate loop at T (rate.py's T), per axis a and corner (j, tau_c) of rate.corner_list (nominal and the four corners of the
     tau x J box, j = J_true / J_a): the axis's f32 firmware gains kappa_p = f32(J_a kappa_p) / J_a, kappa_i likewise, kd = 0,
     the law of rate_loop.hpp in bypass (no prefilter). State s = [m, omega, theta, I, e_prev]; with e = exp(-T/tau_c) and
     g = T - tau_c (1 - e), one execution with reference r is
         I+ = I + kappa_i T e_prev,  e_n = r - omega,  u = kappa_p e_n + I+,
         m' = e m + (1 - e) u,
         omega' = omega + (tau_c (1 - e) m + g u) / j,
         theta' = theta + T omega + (tau_c g m + (T^2/2 - tau_c g) u) / j,
         I' = I+,  e_prev' = e_n,
     the exact zero-order-hold solution (rate.py's closed forms plus the angle), so s' = A s + B r.
  2. Attitude loop at T_a = N T by lifting: the attitude law samples theta at n = kN and holds its output for N rate
     executions, with no computation delay. s_(k+1)N = A^N s_kN + (sum_{i<N} A^i B) r_k, so
     G_N(z) = C_theta (zI - A^N)^-1 sum A^i B is exact and LTI at T_a. Only theta holds the pole at z = 1 (theta feeds no other
     state), so G_N(z) = F(z)/(z - 1) with F(z) = d + l (zI - M)^-1 b over the other four states; the pole is taken out
     analytically, arg(z - 1) = pi/2 + wT_a/2 on the upper unit half circle, which keeps the low-frequency phase exact.
  3. The loop is L = k G_N on every axis in the linear regime: roll and pitch at f32(k); yaw at its effective linear gain
     f32(f32(k)/f32(w)) f32(w), the product the compensated float law realises (decision 0006 C).
  4. Crossover |L(e^{j theta})| = 1, theta = w T_a, required unique on rate.py's 1024-point log grid from pi 2^-20 to pi
     (then bisected). PM = pi + arg L on the continuous branch: arg F is unwrapped along the grid from its low-frequency
     value (F(1) > 0, so L starts at -pi/2) and the unwrapping is confirmed by one grid doubling (2047 points): the branch at
     every coarse point must not change.
  5. Stability: the Jury test (Schur-Cohn form of the table, plus p(1) > 0 and (-1)^n p(-1) > 0) on the characteristic
     polynomial of A^N - k (sum A^i B) C_theta at every loop; the Jury radius is the smallest r for which the test passes
     on p(r z), the spectral radius of the closed loop.
  6. PM_worst(k) = minimum over the 15 loops (3 axes x nominal + 4 corners); a loop without a unique crossover or
     unstable makes k infeasible.
  7. Gain rule (0005's sup rule on the P gain): k = sup{k' : PM_worst(k'') >= PM_min for all k'' in (0, k']} at T_a. A
     geometric scan from (pi/T_a) 2^-17 in steps of 2^(1/16) to the first infeasible point, then bisection until f32(k)
     stops changing, the feasible end taken. Guard: PM_worst on the f32 gains (yaw effective) >= PM_min + delta_num,
     delta_num = (max over the 15 loops of |dPM/dk| by a central difference of step 2^-20 k) x (final bracket width); a
     failing candidate is replaced by the previous feasible point of the scan-and-bisection history. No feasible
     candidate: the card is refused.
  8. Yaw weight: w = min(1, alpha_yaw / min(alpha_roll, alpha_pitch)), alpha_a = tau_max,a / (J_a (1 + b_J)) from rate.py;
     w <= 0 or non-finite is refused. w does not enter the linear loop.
  9. SIM-7: k_ref is the rule at N = 1; q(N) = PM_worst(f32(k_ref), N) for N = 2^i; Delta_i = |q(2^i) - q(2^(i-1))|; N* =
     2^i* with i* the largest i for which Delta_j < U for every j <= i (N* = 1 if Delta_1 >= U). The scan stops at the first
     failure, or where k_ref has no unique crossover or is unstable. The final gains are the rule at T_a = N* T.
     U is the file given by flatten.py --sim7-u (U in rad, unit rad, method measured, source, rule).
 10. att_yaw_alpha_min = the largest f32 <= alpha_max,yaw = tau_max,yaw / (J_yaw (1 + b_J)) (owner decision 17's fallback).
 11. att_yaw_t_cross (owner decision 18): the latest over the yaw axis's nominal and four corners of the time k T_a of the
     first attitude execution k >= 1 with omega_k <= 0 of the lifted closed rate loop released from a steady 1 rad/s with
     r = 0 (no attitude feedback on yaw), rounded up to f32; a corner that never crosses is refused.

The generator refuses (GenError) an input for which a step is undefined or infeasible.
"""

from __future__ import annotations

import cmath
import json
import math
import struct

import gen_plant_config as gpc
import rate
import schema

AXES = rate.AXES
STATE_N = 5
THETA = 2
OTHER = (0, 1, 3, 4)
# Method constant of the SIM-7 scan (decision 0006 E): the largest exponent i tried for N = 2^i before the scan is
# refused. It sets termination only, not the result.
MAX_I = 30
# Method constant of the phase-branch check (rad): the coarse and doubled grids must agree to this. A branch error is a
# multiple of 2*pi and double rounding of the phase is ~1e-15, so any value far between the two gives the same verdict.
PHASE_AGREE = 1e-9
# Method constant of the release-braking scan (decision 0006 owner decision 18): the most attitude executions stepped
# before a corner is declared to have no zero crossing. It sets termination only, not the result.
BRAKE_SCAN_EXECUTIONS = 2 ** 20

K_METHOD = (
    "derived(attitude P gain by the sup rule of decision 0006 E, the 0005 rule transposed to the P gain: k = sup{k' : "
    "PM_worst(k'') >= PM_min for all k'' in (0, k']} at T_a = att_loop_ratio x the rate-loop period, PM_worst the minimum "
    "over nominal and the tau_robustness_band x inertia_robustness_band corners of each axis, on the exact lifted "
    "sampled-data model of the f32 rate gains in bypass (no prefilter), guard PM_worst(f32) >= PM_min + delta_num; "
    "tools/card/attitude.py)")
K_SOURCE = ("vehicle card; design-budget PM_min, tau_robustness_band, inertia_robustness_band; scenario register "
            "tick_period_num_us, tick_period_den, rate_loop_divisor; the derived rate gains (decision 0005); decision "
            "0006 E")
W_METHOD = ("derived(w = min(1, alpha_max,yaw / min(alpha_max,roll, alpha_max,pitch)), alpha_max,a = tau_max,a / (J_a (1 "
            "+ inertia_robustness_band)) of decision 0005 QF-2; tools/card/attitude.py)")
W_SOURCE = "owner decision 12 of decision 0006 and the rate-loop derivation (vehicle card inertia_diag, mixer M, scenario)"
ALPHA_METHOD = ("derived(alpha_min = tau_max,yaw / (J_yaw (1 + inertia_robustness_band)), decision 0005 QF-2's alpha_max,yaw "
                "with J at the top of its band (tau_max does not depend on tau or J, so this is the smallest over the box), "
                "rounded down to f32; tools/card/attitude.py)")
ALPHA_SOURCE = ("vehicle card inertia_diag, rotors and the mixer M (the air-mode tau_max,yaw of tools/card/rate.py); "
                "design-budget inertia_robustness_band; owner decision 17 of decision 0006")
T_CROSS_METHOD = ("derived(t_cross = the latest, over nominal and the four tau x J corners of the yaw axis, time k T_a of the "
                  "first attitude execution k >= 1 at which the design-model yaw rate is <= 0 after a release: closed rate "
                  "loop in bypass lifted to T_a = att_loop_ratio x the rate-loop period, steady yaw rate 1 rad/s, attitude "
                  "command held at zero yaw rate, no attitude feedback on yaw; rounded up to f32; tools/card/attitude.py)")
T_CROSS_SOURCE = ("the yaw rate gains and corners of the rate derivation (decision 0005), att_loop_ratio (SIM-7); "
                  "owner decision 18 of decision 0006")
N_METHOD = ("derived(SIM-7 halving rule of decision 0006 E: N* = 2^i*, i* the largest i with |q(2^j) - q(2^(j-1))| < U "
            "for all j <= i, q(N) the worst-corner attitude PM of the gains designed at N = 1; tools/card/attitude.py)")


def r32(x):
    return rate.r32(x)


def r32_down(x):
    """The largest float32 <= x (as a double), x > 0."""
    y = r32(x)
    if y <= x:
        return y
    bits = struct.unpack("<I", struct.pack("<f", y))[0]
    return struct.unpack("<f", struct.pack("<I", bits - 1))[0]


# ---- small linear algebra (double) -----------------------------------------------------------------------------------


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(len(v))) for i in range(len(a))]


def identity(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def csolve(a, b):
    """Solve a x = b for complex a (n x n, partial pivoting)."""
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0j] * n
    for r in reversed(range(n)):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def charpoly(a):
    """Coefficients c_0..c_n (low to high, c_n = 1) of det(zI - a), by Faddeev-LeVerrier."""
    n = len(a)
    c = [0.0] * (n + 1)
    c[n] = 1.0
    m = [[0.0] * n for _ in range(n)]
    for k in range(1, n + 1):
        m = matmul(a, m)
        for i in range(n):
            m[i][i] += c[n - k + 1]
        am = matmul(a, m)
        c[n - k] = -sum(am[i][i] for i in range(n)) / k
    return c


def jury(c):
    """Whether every root of the polynomial with coefficients c (low to high, c[-1] > 0) is inside the unit circle: the
    necessary tests p(1) > 0 and (-1)^n p(-1) > 0, then the Jury table in its Schur-Cohn form (|a_0| < a_n, then the
    polynomial (a_n p - a_0 p*)/z of one degree less)."""
    n = len(c) - 1
    if not (sum(c) > 0 and sum(ci * (-1) ** i for i, ci in enumerate(c)) * (-1) ** n > 0):
        return False
    a = list(c)
    while len(a) > 1:
        n = len(a) - 1
        if not abs(a[0]) < abs(a[n]):
            return False
        a = [a[n] * a[j + 1] - a[0] * a[n - j - 1] for j in range(n)]
    return True


def jury_radius(c):
    """The smallest r for which the Jury test passes on p(r z): the closed loop's spectral radius (None when it is not
    below 1)."""
    if not jury(c):
        return None
    lo, hi = 0.0, 1.0
    for _ in range(rate.MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if jury([ci * mid ** i for i, ci in enumerate(c)]):
            hi = mid
        else:
            lo = mid
    return hi


# ---- the rate-loop model and its lifting ----------------------------------------------------------------------------


def rate_model(kp, ki, T, tau, j):
    """(A, B) of s' = A s + B r for s = [m, omega, theta, I, e_prev]; kp, ki are kappa = gain / J_a, j = J_true / J_a."""
    e, one_minus_e, g, _ = rate.plant_constants(tau, T)
    ki_t = ki * T
    u = [0.0, -kp, 0.0, 1.0, ki_t, kp]  # u = kp (r - omega) + I + ki T e_prev over [m, omega, theta, I, e_prev, r]
    a = [[0.0] * 6 for _ in range(STATE_N)]
    for c in range(6):
        a[0][c] = one_minus_e * u[c]
        a[1][c] = g * u[c] / j
        a[2][c] = (T * T / 2 - tau * g) * u[c] / j
    a[0][0] += e
    a[1][0] += tau * one_minus_e / j
    a[1][1] += 1.0
    a[2][0] += tau * g / j
    a[2][1] += T
    a[2][2] += 1.0
    a[3][3] += 1.0
    a[3][4] += ki_t
    a[4][1] += -1.0
    a[4][5] += 1.0
    return [row[:STATE_N] for row in a], [row[STATE_N] for row in a]


def lift(a, b, n):
    """(A^n, sum_{i<n} A^i B)."""
    p = identity(STATE_N)
    bs = [0.0] * STATE_N
    for _ in range(n):
        pb = matvec(p, b)
        bs = [x + y for x, y in zip(bs, pb)]
        p = matmul(a, p)
    return p, bs


class _Grid:
    def __init__(self):
        n = 2 * rate.GRID_POINTS - 1
        self.theta = [math.pi * 2.0 ** (rate.GRID_LOG2_LOW * (1 - i / (n - 1))) for i in range(n)]
        self.cos = [math.cos(t) for t in self.theta]
        self.sin = [math.sin(t) for t in self.theta]
        self.half = [2 * math.sin(t / 2) for t in self.theta]


_GRID = _Grid()


class Loop:
    """The lifted loop G_N of one axis and corner at attitude period N T: frequency response, margin and Jury test."""

    def __init__(self, kp, ki, T, tau, j, n):
        a, b = rate_model(kp, ki, T, tau, j)
        self.p, self.bs = lift(a, b, n)
        self.m4 = [[self.p[r][c] for c in OTHER] for r in OTHER]
        self.b4 = [self.bs[r] for r in OTHER]
        self.ell = [self.p[THETA][c] for c in OTHER]
        self.d = self.bs[THETA]
        g = _GRID
        self.h, arg = [], []
        for i in range(len(g.theta)):
            f = self.response(complex(g.cos[i], g.sin[i]))
            self.h.append(abs(f) / g.half[i])
            arg.append(cmath.phase(f))
        self.unwrapped = self._unwrap(arg)
        coarse = self._unwrap(arg[::2])
        self.branch_ok = all(abs(coarse[i] - self.unwrapped[2 * i]) < PHASE_AGREE for i in range(len(coarse)))
        self.start_ok = abs(self.unwrapped[0]) < math.pi / 4

    @staticmethod
    def _unwrap(arg):
        out = [arg[0]]
        for i in range(1, len(arg)):
            out.append(out[-1] + (arg[i] - arg[i - 1] + math.pi) % (2 * math.pi) - math.pi)
        return out

    def response(self, z):
        """F(z) = d + l (zI - M)^-1 b, with G_N(z) = F(z)/(z - 1)."""
        a = [[(z if r == c else 0) - self.m4[r][c] for c in range(4)] for r in range(4)]
        x = csolve(a, self.b4)
        return self.d + sum(self.ell[i] * x[i] for i in range(4))

    def closed_poly(self, k):
        m = [row[:] for row in self.p]
        for i in range(STATE_N):
            m[i][THETA] -= k * self.bs[i]
        return charpoly(m)

    def stable(self, k):
        return jury(self.closed_poly(k))

    def margin(self, k):
        """(crossover angle theta = w T_a, PM, reason); angle and PM are None without a unique crossover."""
        g, h = _GRID, self.h
        n = rate.GRID_POINTS
        above = [k * h[2 * i] > 1 for i in range(n)]
        changes = [i for i in range(n - 1) if above[i] != above[i + 1]]
        if not above[0] or above[-1]:
            return None, None, "|L| does not fall from above 1 at low frequency to below 1 at Nyquist"
        if len(changes) != 1:
            return None, None, f"|L| = 1 has {len(changes)} solutions on the grid"
        i0 = 2 * changes[0]
        lo, hi = g.theta[i0], g.theta[i0 + 2]
        f = None
        for _ in range(rate.MAX_BISECTIONS):
            mid = (lo + hi) / 2
            if mid <= lo or mid >= hi:
                break
            f = self.response(cmath.exp(1j * mid))
            if k * abs(f) / (2 * math.sin(mid / 2)) > 1:
                lo = mid
            else:
                hi = mid
        theta = (lo + hi) / 2
        f = self.response(cmath.exp(1j * theta))
        principal = cmath.phase(f)
        phase = principal + 2 * math.pi * round((self.unwrapped[i0] - principal) / (2 * math.pi))
        return theta, math.pi / 2 - theta / 2 + phase, ""


# ---- the design ---------------------------------------------------------------------------------------------------


def axis_gains(rate_result, axis):
    v = rate_result["axes"][axis]
    return v["kp"] / v["J"], v["ki"] / v["J"]


def build_loops(rate_result, n):
    """[(axis, corner name, Loop)] for the 15 loops at N = n."""
    i = rate_result["inputs"]
    corners = rate.corner_list(i["tau"], i["b_tau"], i["b_J"])
    out = []
    for axis in AXES:
        kp, ki = axis_gains(rate_result, axis)
        for name, jt, tau in corners:
            out.append((axis, name, Loop(kp, ki, rate_result["T"], tau, jt, n)))
    return out


def yaw_effective(k32, w32):
    return r32(k32 / w32) * w32


def axis_k(axis, k, w32=None):
    """The linear gain of an axis for the P gain k: k itself, or (yaw with w32) the float law's effective gain."""
    if axis == "yaw" and w32 is not None:
        return yaw_effective(k, w32)
    return k


def pm_worst(loops, k, w32=None):
    """(PM_worst, detail) with detail [(axis, corner, PM, crossover theta, Jury radius, reason)]; PM_worst is -inf when
    a loop has no unique crossover or is unstable."""
    worst, detail = math.inf, []
    for axis, name, loop in loops:
        kk = axis_k(axis, k, w32)
        theta, pm, why = loop.margin(kk)
        radius = loop.closed_poly(kk)
        radius = jury_radius(radius)
        if pm is None or radius is None:
            worst = -math.inf
        else:
            worst = min(worst, pm)
        detail.append((axis, name, pm, theta, radius, why if pm is None else ("" if radius is not None else "unstable")))
    return worst, detail


def feasible(loops, k, pm_min):
    for axis, name, loop in loops:
        theta, pm, _ = loop.margin(k)
        if pm is None or pm < pm_min or not loop.stable(k):
            return False
    return True


def slope_at(loops, k):
    step = k * 2.0 ** rate.DERIVATIVE_LOG2_STEP
    s = 0.0
    for axis, name, loop in loops:
        up, down = loop.margin(k + step)[1], loop.margin(k - step)[1]
        if up is None or down is None:
            return math.inf
        s = max(s, abs(up - down) / (2 * step))
    return s


def sup_rule(loops, t_a, pm_min, w32, refuse):
    """The gain rule at attitude period t_a on `loops`: a dict (k, k32, bracket, delta_num, ...); refuse(reason) raises."""
    ratio = 2.0 ** (1 / rate.SCAN_STEPS_PER_OCTAVE)
    k = math.pi / t_a * 2.0 ** rate.SCAN_LOG2_START
    if not feasible(loops, k, pm_min):
        refuse(f"no feasible attitude gain: the scan start {k:.6g} 1/s already violates PM_min")
    history, lo = [], None
    while lo is None:
        history.append(k)
        nxt = k * ratio
        if nxt * t_a >= math.pi:
            refuse("the worst-case margin stays above PM_min up to Nyquist: the scan finds no bracket")
        if feasible(loops, nxt, pm_min):
            k = nxt
        else:
            lo, hi = k, nxt
    scan_bracket = (lo, hi)
    bisections = 0
    for _ in range(rate.MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if r32(mid) in (r32(lo), r32(hi)):
            break
        bisections += 1
        if feasible(loops, mid, pm_min):
            lo = mid
            history.append(mid)
        else:
            hi = mid
    width = hi - lo
    slope = slope_at(loops, lo)
    delta_num = slope * width
    chosen = None
    for cand in reversed(history):
        k32 = r32(cand)
        worst, detail = pm_worst(loops, k32, w32)
        if worst >= pm_min + delta_num:
            chosen = (cand, k32, worst, detail)
            break
    if chosen is None:
        refuse("no feasible attitude gain passes the guard PM_worst(f32 gains) >= PM_min + delta_num")
    cand, k32, worst, detail = chosen
    return {"k": cand, "k32": k32, "pm_worst": worst, "detail": detail, "bracket": (lo, hi), "bracket_width": width,
            "scan_bracket": scan_bracket, "bisections": bisections, "slope": slope, "delta_num": delta_num,
            "stepped_down": history.index(cand) != len(history) - 1, "history": history,
            "steps_back": len(history) - 1 - history.index(cand)}


def yaw_release_crossing(loops, t_a, refuse):
    """(t_cross, [(corner, execution count, time s)]): the yaw axis's design-model release. The closed rate loop in bypass,
    lifted to T_a, starts at the steady yaw rate 1 rad/s (s = [m, omega, theta, I, e_prev] = [0, 1, 0, 0, 0], the rate
    command 1 rad/s having held it) and the attitude command is held at zero yaw rate, r = 0, with no attitude feedback on
    yaw: s_k = A^N s_(k-1) at attitude execution k. The crossing is the first k >= 1 with omega_k <= 0 (sigma = +1 for a
    release from +1 rad/s; the system is linear, so the amplitude is immaterial); its time is k T_a. t_cross is the latest
    over nominal and the four corners, rounded up to f32. Refuses a corner that does not cross within the scan."""
    out = []
    for axis, name, loop in loops:
        if axis != "yaw":
            continue
        x = [0.0, 1.0, 0.0, 0.0, 0.0]
        for k in range(1, BRAKE_SCAN_EXECUTIONS + 1):
            x = matvec(loop.p, x)
            if x[1] <= 0:
                out.append((name, k, k * t_a))
                break
        else:
            refuse(f"the yaw rate at corner {name} does not cross zero within {BRAKE_SCAN_EXECUTIONS} attitude executions "
                   "after a release")
    return rate.r32_up(max(t for _, _, t in out)), out


def read_u(path, allow_scenario=False):
    """The SIM-7 uncertainty file: a mapping with U (a number of rad above 0), unit (rad), method (measured), source and
    rule. Raises gpc.GenError. Returns {value, method, source, rule}, source as compact text. `allow_scenario` is for
    tests only (a fixture with method scenario); flatten.py never passes it, so the product path accepts measured only."""
    try:
        doc = schema.load_yaml(path)
    except Exception as e:  # noqa: BLE001 (a read or parse failure of any kind is a refusal)
        raise gpc.GenError([f"{path}: cannot read the SIM-7 uncertainty file: {e}"]) from e
    if not isinstance(doc, dict):
        raise gpc.GenError([f"{path}: the SIM-7 uncertainty file is not a mapping"])
    bad = [k for k in ("U", "unit", "method", "source", "rule") if k not in doc]
    if bad:
        raise gpc.GenError([f"{path}: the SIM-7 uncertainty file lacks {', '.join(bad)}"])
    if not (schema.is_number(doc["U"]) and doc["U"] > 0):
        raise gpc.GenError([f"{path}: U must be a finite number of rad above 0, got {doc['U']!r}"])
    if doc["unit"] != "rad":
        raise gpc.GenError([f"{path}: unit must be rad, got {doc['unit']!r}"])
    if doc["method"] != "measured" and not (allow_scenario and doc["method"] == "scenario"):
        raise gpc.GenError([f"{path}: method must be measured (a committed measurement, core 2), got {doc['method']!r}"])
    if not (isinstance(doc["rule"], str) and doc["rule"].strip()):
        raise gpc.GenError([f"{path}: rule must be non-empty text"])
    if not doc["source"]:
        raise gpc.GenError([f"{path}: source must not be empty"])
    source = doc["source"] if isinstance(doc["source"], str) else json.dumps(doc["source"], ensure_ascii=True)
    return {"value": float(doc["U"]), "method": doc["method"], "source": source, "rule": " ".join(doc["rule"].split())}


def design(card, budget, scenario, card_path, u, u_where="the SIM-7 uncertainty file", rate_result=None):
    """The whole rule: a dict of results (raises gpc.GenError when the card is refused). `u` is read_u's mapping."""
    def refuse(reason):
        gpc.refuse(card_path, "attitude", f"{reason} (decision 0006)")

    rr = rate_result if rate_result is not None else rate.design(card, budget, scenario, card_path)
    pm_min = rr["inputs"]["PM_min"]
    deadband = rate._value(scenario, "yaw_deadband", "scenario entry", card_path)
    rate._value(scenario, "angle_tilt_max", "scenario entry", card_path)
    if not 0 <= deadband < 1:
        refuse(f"yaw_deadband {deadband!r} is outside [0, 1)")
    if not (schema.is_number(u["value"]) and u["value"] > 0):
        refuse("U must be a finite number of rad above 0")
    alpha = {a: rr["axes"][a]["alpha_max"] for a in AXES}
    w = min(1.0, alpha["yaw"] / min(alpha["roll"], alpha["pitch"]))
    if not (math.isfinite(w) and w > 0):
        refuse(f"the yaw weight {w!r} is not in (0, 1]")
    w32 = r32(w)
    if not w32 > 0:
        refuse("the yaw weight is not a positive f32")
    t = rr["T"]
    cache = {}

    def loops_at(n):
        if n not in cache:
            cache[n] = build_loops(rr, n)
            for axis, name, loop in cache[n]:
                if not loop.start_ok:
                    refuse(f"N = {n}, {axis} {name}: the low-frequency phase of F is not near 0 (not a single pole at z = 1)")
                if not loop.branch_ok:
                    refuse(f"N = {n}, {axis} {name}: the unwrapped phase changes under one grid doubling")
        return cache[n]

    ref = sup_rule(loops_at(1), t, pm_min, w32, refuse)
    table, n_star, stop = [], 1, None
    prev = None
    for i in range(MAX_I + 1):
        n = 2 ** i
        q, detail = pm_worst(loops_at(n), ref["k32"], w32)
        row = {"N": n, "f_hz": 1 / (n * t), "q": q if q != -math.inf else None, "delta": None, "verdict": ""}
        if q == -math.inf:
            row["verdict"] = "stop: k_ref has no unique crossover or is unstable"
            table.append(row)
            stop = row["verdict"]
            break
        if prev is None:
            row["verdict"] = "reference"
        else:
            row["delta"] = abs(q - prev)
            if row["delta"] < u["value"]:
                row["verdict"] = "accepted (Delta < U)"
                n_star = n
            else:
                row["verdict"] = "stop: Delta >= U"
                table.append(row)
                stop = row["verdict"]
                break
        table.append(row)
        prev = q
    else:
        refuse(f"the SIM-7 scan reaches N = 2^{MAX_I} without a stop")
    final = ref if n_star == 1 else sup_rule(loops_at(n_star), n_star * t, pm_min, w32, refuse)
    if not all(jury(loop.closed_poly(axis_k(axis, final["k32"], w32))) for axis, _, loop in loops_at(n_star)):
        refuse("the final gains are unstable at a corner (Jury test)")
    at_one = pm_worst(loops_at(1), ref["k32"], w32)
    alpha_min = r32_down(alpha["yaw"])
    if not (math.isfinite(alpha_min) and alpha_min > 0):
        refuse(f"att_yaw_alpha_min {alpha_min!r} is not a positive finite f32")
    t_cross, crossings = yaw_release_crossing(loops_at(n_star), n_star * t, refuse)
    return {"T": t, "rate": rr, "inputs": dict(rr["inputs"]), "U": u, "U_where": u_where, "alpha": alpha,
            "w_raw": alpha["yaw"] / min(alpha["roll"], alpha["pitch"]), "w": w, "w32": w32,
            "yaw_deadband": deadband, "ref": ref, "table": table, "stop": stop, "N": n_star, "T_a": n_star * t,
            "alpha_min": alpha_min, "t_cross": t_cross, "crossings": crossings,
            "final": final, "k": final["k"], "k32": final["k32"], "k_yaw_effective": yaw_effective(final["k32"], w32),
            "at_one": {"pm_worst": at_one[0], "detail": at_one[1]}, "pm_worst": final["pm_worst"]}


def entries_from(result, u_where):
    n_source = (f"SIM-7 uncertainty U = {result['U']['value']!r} rad from {u_where} (method {result['U']['method']}; "
                f"{result['U']['source']}); the gain rule's inputs (decision 0006 E)")
    return [("att_kp", {"type": "f32", "value": result["k32"], "unit": "1/s", "method": K_METHOD, "source": K_SOURCE,
                        "sigma": schema.UNKNOWN}),
            ("att_yaw_weight", {"type": "f32", "value": result["w32"], "unit": "1", "method": W_METHOD,
                                "source": W_SOURCE, "sigma": schema.UNKNOWN}),
            ("att_loop_ratio", {"type": "i32", "value": result["N"], "unit": "1", "method": N_METHOD,
                                "source": n_source, "sigma": 0}),
            ("att_yaw_alpha_min", {"type": "f32", "value": result["alpha_min"], "unit": "rad/s^2",
                                   "method": ALPHA_METHOD, "source": ALPHA_SOURCE, "sigma": schema.UNKNOWN}),
            ("att_yaw_t_cross", {"type": "f32", "value": result["t_cross"], "unit": "s", "method": T_CROSS_METHOD,
                                 "source": T_CROSS_SOURCE, "sigma": schema.UNKNOWN})]


def _detail_lines(detail, t_a):
    lines = ["  axis  loop      PM rad                 PM deg       crossover rad/s        Jury radius"]
    for axis, name, pm, theta, radius, why in detail:
        if pm is None:
            lines.append(f"  {axis:<5} {name:<8}  no crossover: {why}")
        else:
            lines.append(f"  {axis:<5} {name:<8}  {pm!r:<22} {math.degrees(pm):<12.6f} {theta / t_a!r:<22} {radius!r}")
    return lines


def report_text(result, where):
    r, i, deg = result, result["inputs"], math.degrees
    rr, u, f, ref = r["rate"], r["U"], r["final"], r["ref"]
    lines = [
        "MARV L5 attitude-loop derivation (tools/card/attitude.py, decision 0006 E)",
        f"card                     {where}",
        f"PM_min (rad, deg)        {i['PM_min']!r}  {deg(i['PM_min']):.6f}",
        f"bands                    tau +-{i['b_tau']!r}  J +-{i['b_J']!r}",
        f"rate-loop period T (s)   {r['T']!r}  ({1 / r['T']:.6g} Hz)",
        "f32 rate gains used (kd = 0, bypass: no prefilter), kappa = gain / J_a:",
    ]
    for axis in AXES:
        v = rr["axes"][axis]
        lines.append(f"  {axis:<5} J {v['J']!r}  kp {v['kp']!r}  ki {v['ki']!r}")
    lines += [
        "model: exact ZOH rate loop s = [m, omega, theta, I, e_prev] per corner (tau x J box), lifted to T_a = N T by",
        "  s_(k+1)N = A^N s_kN + (sum_{i<N} A^i B) r_k, zero computation delay; L = k G_N, G_N = F(z)/(z - 1); the crossover is",
        "  unique on the 1024-point log grid then bisected; PM = pi + arg L on the continuous branch, confirmed by one grid",
        "  doubling; Jury test on A^N - k (sum A^i B) C_theta at every loop.",
        "",
        "yaw weight w = min(1, alpha_yaw / min(alpha_roll, alpha_pitch))",
        f"  alpha_max (rad/s^2)    roll {r['alpha']['roll']!r}  pitch {r['alpha']['pitch']!r}  yaw {r['alpha']['yaw']!r}",
        f"  w (double, f32)        {r['w']!r}  {r['w32']!r}   (unclamped ratio {r['w_raw']!r})",
        "",
        f"SIM-7: U = {u['value']!r} rad = {deg(u['value']):.6f} deg  (source file {r['U_where']})",
        f"  U method               {u['method']}",
        f"  U source               {u['source']}",
        f"  U rule                 {u['rule']}",
        f"  k_ref (N = 1)          {ref['k']!r}  f32 {ref['k32']!r}",
        "  N     rate Hz      q(N) = PM_worst(k_ref) deg   Delta deg        verdict",
    ]
    for row in r["table"]:
        q = "n/a" if row["q"] is None else f"{deg(row['q']):.6f}"
        d = "" if row["delta"] is None else f"{deg(row['delta']):.6f}"
        lines.append(f"  {row['N']:<5} {row['f_hz']:<12.6g} {q:<28} {d:<16} {row['verdict']}")
    lines += [
        f"  N* = {r['N']}  (T_a = {r['T_a']!r} s, {1 / r['T_a']:.6g} Hz); scan stopped: {r['stop']}",
        "",
        f"N = 1 (k_ref, f32 {ref['k32']!r}): PM_worst {deg(r['at_one']['pm_worst']):.6f} deg",
        *_detail_lines(r["at_one"]["detail"], r["T"]),
        "",
        "att_yaw_alpha_min (owner decision 17's fallback): tau_max,yaw / (J_yaw (1 + b_J)), rounded down to f32",
        f"  tau_max,yaw (N m)      {rr['axes']['yaw']['tau_max']!r}",
        f"  J_yaw (kg m^2), b_J    {rr['axes']['yaw']['J']!r}  {i['b_J']!r}",
        f"  alpha (double, f32)    {r['alpha']['yaw']!r}  {r['alpha_min']!r}",
        "",
        f"att_yaw_t_cross (owner decision 18) at N = {r['N']}: the yaw release, lifted loop from 1 rad/s with r = 0",
        *[f"  {name:<8}               execution {k}  time {t!r} s" for name, k, t in r["crossings"]],
        f"  t_cross (f32, rounded up) {r['t_cross']!r} s",
        "",
        f"attitude gain k at N = {r['N']}",
        f"  k (f32, double)        {f['k32']!r}  {f['k']!r}",
        f"  yaw effective gain     {r['k_yaw_effective']!r}  = f32(f32(k)/f32(w)) f32(w)",
        f"  scan bracket (1/s)     {f['scan_bracket'][0]!r} .. {f['scan_bracket'][1]!r}",
        f"  bisections             {f['bisections']}  final bracket {f['bracket'][0]!r} .. {f['bracket'][1]!r}, width "
        f"{f['bracket_width']!r}",
        f"  stepped down by guard  {f['stepped_down']}",
        f"  guard step-backs       {f['steps_back']}",
        f"  max |dPM/dk|           {f['slope']!r} rad per 1/s",
        f"  delta_num (rad)        {f['delta_num']!r}",
        f"  PM_worst on f32 gains  {f['pm_worst']!r}  {deg(f['pm_worst']):.6f} deg  (guard: >= PM_min + delta_num)",
        *_detail_lines(f["detail"], r["T_a"]),
    ]
    return lines


def attitude_entries(card, budget, scenario, card_path, u, u_where, where=None, rate_result=None):
    """(ordered (name, params_gen entry) list, report lines, result dict); raises gpc.GenError on refusal."""
    result = design(card, budget, scenario, card_path, u, u_where, rate_result)
    return entries_from(result, u_where), report_text(result, where or card_path), result
