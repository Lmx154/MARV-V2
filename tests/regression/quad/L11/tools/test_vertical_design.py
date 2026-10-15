"""L11 T3 margins of the vertical loop, by an independent evaluation (decision 0020 "The proposal as ruled" section 5,
"T3 margins (core)"; modelled on tests/regression/quad/L04/tools/test_rate_design.py and
tests/regression/quad/L05/tools/test_attitude_design.py and test_attitude_t3.py).

What is checked is the f32 gains the generator emits (tools/card/vertical.py, read through vertical.vertical_entries), by a
model written here from the firmware law (fw/vertical/include/marv/vertical/vertical_law.hpp) and the plant of decision 0020
section 2: the thrust deviation dT follows the request with the motor lag tau, dv/dt = gamma z - r dT / m_card (NED, z down),
dz/dt = v, with gamma = -dg/dh at the register's site (tools/sim/reference.gravity_height_gradient). Nothing of the
generator's model, loop class or margin functions is used: vertical.py supplies the emitted gains and delta_num (the
generator's guard, rate.py rule 6) only.

The evaluation differs from the generator's: the plant is sampled by a matrix exponential of the augmented continuous system
(Taylor series, scaling and squaring); one law execution plus the sampled plant is a step map that is probed with unit vectors
to give the lifted matrices (P, Bs); the frequency response is a complex linear solve of (zI - P) x = Bs at the point (no
factorisation of the pole at z = 1); the crossover is found on a ratio grid and bisected, and the phase is unwrapped by stepping;
the sensitivity peak Ms is the grid maximum refined by golden section; stability is the Jury (Schur-Cohn) test on the exact
rational characteristic polynomial of the float closed-loop matrix, and the radius is the spectral radius by repeated squaring.

Loops (decision 0020 section 5; vertical.py rule 5 and 9):
  inner  the vertical-velocity PI on the plant dT_req -> v with the gravity gradient (states [dT, v, z], z internal) and the
         loop gain factor r. Its margins are those of the open loop e -> v. The gamma-free design model 1/(m s (1 + tau s)),
         the rate loop of L04 with J = m_card / r, only motivates the gain family (design_pair) and is printed as a comparison.
  outer  the height P on the closed velocity loop with gamma: L = k_z G, G = z / v_sp of the lifted 5-state loop
         s = [dT, v, z, I, e_prev], at the loop period (vert_loop_ratio = 1).
Corners: nominal (r = 1, tau) and {r_lo, r_hi} x {tau (1 -+ b_tau)}, with r_lo = (1 - b_k) min R, r_hi = (1 + b_k) max R,
R = {1} and m_card / m_c for every open mass conflict m_c of the card (decision 0020 budget text: r = (k_true / k_card)
(m_card / m_true)).

Numeric tolerance TOL = 2^-30 rad (9.3e-10), the L04/L05 family: both evaluators bisect the crossover to adjacent doubles (a
phase error of |dPM/dw| w 2^-52, below 1e-14 rad) and the solve's rounding is below 1e-12 rad, so TOL leaves three decades of
slack and stays below the generators' delta_num (7e-9 rad inner, 1.2e-7 rad outer).

The method constants below (the ratio-1.02 frequency grid from 1e-2 rad/s, the iteration caps) are the L04/L05 oracle's: they set
resolution and termination only.
"""

import cmath
import functools
import math
import struct
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import attitude  # noqa: E402  (numeric constants only: STATE_N, THETA)
import reference  # noqa: E402  (gravity_height_gradient)
import schema  # noqa: E402
import vertical  # noqa: E402  (vertical_entries: the emitted gains and delta_num; nothing of its model)

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"

TOL = 2.0 ** -30
EPS64 = 2.0 ** -52
GAIN_CONTROL = 1.1  # the T3 gain control of decision 0005/0006 F
FINE, COARSE = 17, 9  # grid points per side; the last halving is 9 -> 17 (core 7.5)
MICRO = 1e-6  # SI prefix: the register's tick period is in microseconds
W_START, W_RATIO = 1e-2, 1.02  # the oracle's frequency grid (rad/s), L04/L05
BISECT_CAP = 200
GOLDEN_STEPS = 64  # golden-section shrink 0.618^64 of one grid cell: below the double resolution of the peak
SQUARINGS = 30  # repeated squarings of the spectral-radius estimate: relative resolution below STATE_N * SQUARINGS ln2 / 2^SQUARINGS
JURY_AGREE = 2.0 ** -16  # the generator's radius is a double-precision Jury bisection; the radius here is exact in rationals
STATE_N = attitude.STATE_N
Z = attitude.THETA  # the height z is state 2 of s = [dT, v, z, I, e_prev]
V = 1
CORNER_NAMES = ("nominal", "r-,tau-", "r-,tau+", "r+,tau-", "r+,tau+")
LO_TAU = ("r-,tau-", "r+,tau-")  # rate.py rule 7's (J_lo, tau_lo) corner is (r_hi, tau_lo) here; 0020 section 5 names (r_lo, tau_lo)


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def f32_up(x):
    """The next float32 above the float32 x."""
    bits = struct.unpack("<I", struct.pack("<f", x))[0]
    return struct.unpack("<f", struct.pack("<I", bits + 1))[0]


# ---- the independent model: sampled plant, law step, lifting, complex solve ------------------------------------------------


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def expm(m):
    """exp(m) by scaling and squaring around a 40-term Taylor series."""
    n = len(m)
    norm = max(sum(abs(x) for x in row) for row in m)
    s = max(0, math.ceil(math.log2(norm)) + 1) if norm > 0 else 0
    a = [[x / 2 ** s for x in row] for row in m]
    term = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    total = [row[:] for row in term]
    for k in range(1, 40):
        term = [[x / k for x in row] for row in matmul(term, a)]
        total = [[x + y for x, y in zip(r1, r2)] for r1, r2 in zip(total, term)]
    for _ in range(s):
        total = matmul(total, total)
    return total


@functools.lru_cache(maxsize=None)
def sampled_plant(h, tau, r, gamma, mass):
    """(Phi, Gam) of [dT, v, z]: dT' = (u - dT)/tau, v' = gamma z - r dT/m, z' = v under a zero-order-hold request u over h."""
    aug = [[-h / tau, 0.0, 0.0, h / tau], [-h * r / mass, 0.0, h * gamma, 0.0], [0.0, h, 0.0, 0.0], [0.0] * 4]
    e = expm(aug)
    return tuple(tuple(e[i][:3]) for i in range(3)), tuple(e[i][3] for i in range(3))


@functools.lru_cache(maxsize=None)
def lifted(kv, ki, T, tau, r, gamma, mass, ticks=None, delay=0):
    """(P, Bs, labels) of x_(n+1) = P x_n + Bs v_sp over one loop period T, probed from the step map of the firmware law:
        I+ = I + ki T e_prev,  e = v_sp - v,  dT_req = -(kv e + I+),  e_prev+ = e,  then the exact ZOH plant over the period.
    ticks None: five states [dT, v, z, I, e_prev], the plant sampled once at T. ticks = n: six states with u_held, the plant
    sampled at the tick h = T/n (n ticks per period, the law executing at the first), and `delay` ticks (0 or 1) between the
    law's execution and the request reaching the plant (the held request applies until then)."""
    dim = STATE_N + (0 if ticks is None else 1)
    if ticks is None:
        phi, gam = sampled_plant(T, tau, r, gamma, mass)
    else:
        phi, gam = sampled_plant(T / ticks, tau, r, gamma, mass)

    def execute(x, vsp):
        dT, v, z, i_acc, e_prev = x[:STATE_N]
        i_acc = i_acc + ki * T * e_prev
        e = vsp - v
        return -(kv * e + i_acc), i_acc, e

    def plant(x, u):
        return [sum(phi[q][c] * x[c] for c in range(3)) + gam[q] * u for q in range(3)]

    def period(x, vsp):
        if ticks is None:
            u, i_acc, e = execute(x, vsp)
            return [*plant(x, u), i_acc, e]
        held = x[STATE_N]
        x = list(x[:STATE_N])
        for t in range(ticks):
            if t == 0:
                new, x[3], x[4] = execute(x, vsp)
            else:
                new = held
            applied = held if delay and t == 0 else new
            x[:3] = plant(x, applied)
            held = new
        return [*x, held]

    cols = [period([1.0 if q == c else 0.0 for q in range(dim)], 0.0) for c in range(dim)]
    p = [[cols[c][q] for c in range(dim)] for q in range(dim)]
    return p, period([0.0] * dim, 1.0)


@functools.lru_cache(maxsize=None)
def lifted_inner(kv, ki, T, tau, r, gamma, mass):
    """(P, B) of the open velocity loop e -> v: x = [dT, v, z, xi], u_c = kv e + xi, dT_req = -u_c, xi+ = xi + ki T e (the integral
    deferred one execution, as the firmware's I_n = I_(n-1) + ki e_(n-1) T); the plant [dT, v, z] sampled with `gamma` (0.0:
    the gamma-free comparison, z then feeds nothing)."""
    phi, gam = sampled_plant(T, tau, r, gamma, mass)
    p = [[*phi[i], -gam[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
    return p, [-gam[0] * kv, -gam[1] * kv, -gam[2] * kv, ki * T]


def csolve(a, b):
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    x = [0j] * n
    for r in reversed(range(n)):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


class Loop:
    """L(w) of a lifted (P, Bs) at period T: kind 'open' L = gain x_out; kind 'closed' x_out is the closed-loop T(z) and
    L = T/(1 - T) (used for the tick-resolution inner loop)."""

    def __init__(self, p, bs, out, gain, T, kind="open"):
        self.p, self.bs, self.out, self.gain, self.T, self.kind = p, bs, out, gain, T, kind
        self.n = len(bs)

    def response(self, w):
        z = cmath.exp(1j * w * self.T)
        a = [[(z if i == j else 0) - self.p[i][j] for j in range(self.n)] for i in range(self.n)]
        x = csolve(a, self.bs)[self.out]
        return self.gain * x if self.kind == "open" else x / (1 - x)

    def margins(self):
        """(PM, crossover rad/s, Ms, Ms peak rad/s); (None,) * 4 without a unique crossover on the grid (|L| > 1 at the first
        point). PM = pi + arg L, the phase unwrapped by stepping from the first grid point."""
        ws = [W_START]
        while ws[-1] * W_RATIO < math.pi / self.T:
            ws.append(ws[-1] * W_RATIO)
        vals = [self.response(w) for w in ws]
        mags = [abs(v) for v in vals]
        changes = [i for i in range(len(ws) - 1) if (mags[i] > 1) != (mags[i + 1] > 1)]
        if len(changes) != 1 or not mags[0] > 1:
            return None, None, None, None
        lo, hi = ws[changes[0]], ws[changes[0] + 1]
        for _ in range(BISECT_CAP):
            mid = (lo + hi) / 2
            if mid <= lo or mid >= hi:
                break
            if abs(self.response(mid)) > 1:
                lo = mid
            else:
                hi = mid
        wx = (lo + hi) / 2
        prev = cmath.phase(vals[0])
        for w, v in [*zip(ws[1:changes[0] + 1], vals[1:changes[0] + 1]), (wx, self.response(wx))]:
            prev += (cmath.phase(v) - cmath.phase(cmath.exp(1j * prev)) + math.pi) % (2 * math.pi) - math.pi
        sens = [1 / abs(1 + v) for v in vals]
        i = max(range(len(ws)), key=sens.__getitem__)
        lo, hi = ws[max(i - 1, 0)], ws[min(i + 1, len(ws) - 1)]
        g = (math.sqrt(5) - 1) / 2

        def s(w):
            return 1 / abs(1 + self.response(w))

        x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
        f1, f2 = s(x1), s(x2)
        for _ in range(GOLDEN_STEPS):
            if f1 >= f2:
                hi, x2, f2 = x2, x1, f1
                x1 = hi - g * (hi - lo)
                f1 = s(x1)
            else:
                lo, x1, f1 = x1, x2, f2
                x2 = lo + g * (hi - lo)
                f2 = s(x2)
        ms, wms = max((sens[i], ws[i]), (f1, x1), (f2, x2))
        return math.pi + prev, wx, ms, wms


# ---- stability: exact Jury test and the spectral radius --------------------------------------------------------------------


def charpoly(m):
    """Coefficients [1, c1, ..., cn] of det(zI - m) in exact rational arithmetic (Faddeev-LeVerrier) of the float matrix."""
    n = len(m)
    a = [[Fraction(x) for x in row] for row in m]
    coeffs = [Fraction(1)]
    mk = [[Fraction(int(i == j)) for j in range(n)] for i in range(n)]
    for k in range(1, n + 1):
        am = [[sum(a[i][t] * mk[t][j] for t in range(n)) for j in range(n)] for i in range(n)]
        c = -sum(am[i][i] for i in range(n)) / k
        coeffs.append(c)
        mk = [[am[i][j] + (c if i == j else 0) for j in range(n)] for i in range(n)]
    return coeffs


def jury_stable(coeffs):
    """Jury test of a0 z^n + ... + an, a0 > 0, exact: p(1) > 0, (-1)^n p(-1) > 0 and the Schur-Cohn reduction
    a_i <- a0 a_i - an a_(n-i) keeps |an| < a0 down to degree 0."""
    n = len(coeffs) - 1
    if not (sum(coeffs) > 0 and (-1) ** n * sum(c * (-1) ** (n - i) for i, c in enumerate(coeffs)) > 0):
        return False
    a = list(coeffs)
    while len(a) > 1:
        if not abs(a[-1]) < a[0]:
            return False
        a = [a[0] * a[i] - a[-1] * a[len(a) - 1 - i] for i in range(len(a) - 1)]
    return True


def jury_radius(coeffs):
    """The smallest r at which the Jury test passes on p(r z) (all roots inside |z| < r), by bisection of a stable r = 1 and an
    unstable r = 0 until the double bracket closes: the spectral radius of the closed loop."""
    n = len(coeffs) - 1
    lo, hi = 0.0, 1.0
    for _ in range(BISECT_CAP):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        scaled = [c * Fraction(mid) ** (n - i) for i, c in enumerate(coeffs)]
        if jury_stable(scaled):
            hi = mid
        else:
            lo = mid
    assert jury_stable(coeffs), "the radius is defined for a stable polynomial"
    return hi


def closed_matrix(p, bs, k, out=Z):
    """P - k Bs e_out^T: the closed loop of v_sp = k (z_ref - z)."""
    n = len(bs)
    return [[p[i][j] - (k * bs[i] if j == out else 0.0) for j in range(n)] for i in range(n)]


def spectral_radius(m):
    """rho of m by repeated squaring of the normalised matrix (relative resolution STATE_N SQUARINGS ln2 / 2^SQUARINGS)."""
    m = [row[:] for row in m]
    log_scale = 0.0
    for _ in range(SQUARINGS):
        m = matmul(m, m)
        s = max(sum(abs(x) for x in row) for row in m)
        m = [[x / s for x in row] for row in m]
        log_scale = 2 * log_scale + math.log(s)
    return math.exp(log_scale / 2 ** SQUARINGS)


RHO_RESOLUTION = STATE_N * SQUARINGS * math.log(2) / 2 ** SQUARINGS


# ---- the inputs, re-derived from the card, the budget and the scenario register ---------------------------------------------


@pytest.fixture(scope="module")
def real():
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    entries, _, res = vertical.vertical_entries(card, budget, scenario, CARD)
    gains = dict(entries)
    mass = card["mass"]["value"]
    ratios = [1.0] + [mass / c["value"] for c in card["mass"]["conflict"] if c["state"] == "open"]
    b_k, b_tau = budget["thrust_gain_robustness_band"]["value"], budget["tau_robustness_band"]["value"]
    tau = card["rotors"]["motor_lag"]["tau"]["value"]
    divisor = scenario["rate_loop_divisor"]["value"]
    T = divisor * scenario["tick_period_num_us"]["value"] / scenario["tick_period_den"]["value"] * MICRO
    gamma = -reference.gravity_height_gradient(scenario["site_latitude_rad"]["value"], scenario["site_height_m"]["value"])
    assert gamma > 0 and gains["vert_loop_ratio"]["value"] == 1
    r_lo, r_hi = (1 - b_k) * min(ratios), (1 + b_k) * max(ratios)
    t_lo, t_hi = (1 - b_tau) * tau, (1 + b_tau) * tau
    out = {"mass": mass, "tau": tau, "T": T, "divisor": divisor, "gamma": gamma, "b_k": b_k, "b_tau": b_tau, "r_lo": r_lo,
           "r_hi": r_hi, "tau_lo": t_lo, "tau_hi": t_hi, "PM_min": budget["PM_min"]["value"],
           "Ms_max": budget["Ms_max"]["value"],
           "kv": gains["vert_kp_v"]["value"], "ki": gains["vert_ki_v"]["value"], "kz": gains["vert_kp_z"]["value"],
           "delta_inner": res["inner"]["delta_num"], "delta_outer": res["outer"]["delta_num"], "res": res,
           "corners": [("nominal", 1.0, tau), ("r-,tau-", r_lo, t_lo), ("r-,tau+", r_lo, t_hi), ("r+,tau-", r_hi, t_lo),
                       ("r+,tau+", r_hi, t_hi)]}
    for name in ("kv", "ki", "kz"):
        assert out[name] == r32(out[name]) and out[name] > 0, name
    return out


def inner_margins(real, kv, ki, r, tau, delay=0, gamma=None):
    """(PM, wc, Ms, w_Ms) of the velocity loop with the gravity gradient (`gamma` 0.0: the gamma-free comparison); delay = ticks
    added between the law's execution and the plant."""
    T = real["T"]
    gamma = real["gamma"] if gamma is None else gamma
    if delay:
        p, bs = lifted(kv, ki, T, tau, r, gamma, real["mass"], real["divisor"], delay)
        return Loop(p, bs, V, 1.0, T, "closed").margins()
    p, bs = lifted_inner(kv, ki, T, tau, r, gamma, real["mass"])
    return Loop(p, bs, V, 1.0, T).margins()


def outer_loop(real, kv, ki, kz, r, tau, delay=0):
    T = real["T"]
    if delay:
        p, bs = lifted(kv, ki, T, tau, r, real["gamma"], real["mass"], real["divisor"], delay)
    else:
        p, bs = lifted(kv, ki, T, tau, r, real["gamma"], real["mass"])
    return p, bs, Loop(p, bs, Z, kz, T)


def outer_margins(real, kv, ki, kz, r, tau, delay=0, radius=False):
    """(PM, wc, Ms, w_Ms, Jury-stable, Jury radius or None unless `radius`) of the height loop with gamma."""
    p, bs, loop = outer_loop(real, kv, ki, kz, r, tau, delay)
    pm, wc, ms, wms = loop.margins()
    poly = charpoly(closed_matrix(p, bs, kz))
    stable = jury_stable(poly)
    return pm, wc, ms, wms, stable, (jury_radius(poly) if radius and stable else None)


def corner_table(real, kv=None, ki=None, kz=None, delay=0, radius=False):
    """[(name, inner (PM, wc, Ms, w), outer (PM, wc, Ms, w, stable, rho))] over nominal and the four corners."""
    kv, ki, kz = (real[n] if v is None else v for n, v in (("kv", kv), ("ki", ki), ("kz", kz)))
    return [(name, inner_margins(real, kv, ki, r, tau, delay), outer_margins(real, kv, ki, kz, r, tau, delay, radius))
            for name, r, tau in real["corners"]]


def d1_violations(real, table):
    """The T3 rule on a corner table: [(loop, corner, what, value)] where PM < PM_min - delta_num, Ms > Ms_max, a loop has
    no unique crossover, or the height loop is unstable. Empty means D1 holds."""
    bad = []
    for name, inner, outer in table:
        for loop, m, delta in (("inner", inner, real["delta_inner"]), ("outer", outer, real["delta_outer"])):
            if m[0] is None:
                bad.append((loop, name, "no unique crossover", None))
                continue
            if m[0] < real["PM_min"] - delta:
                bad.append((loop, name, "PM", m[0]))
            if m[2] > real["Ms_max"]:
                bad.append((loop, name, "Ms", m[2]))
        if not outer[4]:
            bad.append(("outer", name, "unstable", None))
    return bad


@pytest.fixture(scope="module")
def table(real):
    return corner_table(real, radius=True)


def worst(table):
    """(worst PM rad, its (loop, corner)), (worst Ms, its (loop, corner)) over the table."""
    pms = [(m[0], (loop, name)) for name, i, o in table for loop, m in (("inner", i), ("outer", o))]
    mss = [(m[2], (loop, name)) for name, i, o in table for loop, m in (("inner", i), ("outer", o))]
    return min(pms), max(mss)


# ---- D1: margins at nominal and the four corners ---------------------------------------------------------------------------


def test_d1_pm_and_ms_hold_at_nominal_and_the_four_corners_for_both_loops_on_the_f32_gains(real, table):
    assert [t[0] for t in table] == list(CORNER_NAMES)
    assert real["r_lo"] == pytest.approx(1 - real["b_k"], rel=8 * EPS64) and real["r_hi"] > 1 + real["b_k"]
    for name, inner, outer in table:
        print(f"D1 {name:<8} inner PM {math.degrees(inner[0])!r} deg wc {inner[1]!r} Ms {inner[2]!r} | outer PM "
              f"{math.degrees(outer[0])!r} deg wc {outer[1]!r} Ms {outer[2]!r} Jury radius {outer[5]!r}")
    free = [inner_margins(real, real["kv"], real["ki"], r, tau, gamma=0.0) for _, r, tau in real["corners"]]
    print("D1 inner PM with gamma minus the gamma-free comparison (rad): "
          + ", ".join(f"{n} {i[0] - f[0]!r}" for (n, i, _), f in zip(table, free)))
    (pm, at_pm), (ms, at_ms) = worst(table)
    print(f"D1 worst PM {math.degrees(pm)!r} deg at {at_pm} (PM_min {math.degrees(real['PM_min'])!r} deg; delta_num inner "
          f"{real['delta_inner']!r} rad, outer {real['delta_outer']!r} rad); worst Ms {ms!r} at {at_ms} (Ms_max {real['Ms_max']!r})")
    assert d1_violations(real, table) == []
    assert ms <= real["Ms_max"]
    for loop, rows in (("inner", real["res"]["inner"]["rows"]), ("outer", real["res"]["outer"]["rows"])):
        for (name, i, o), row in zip(table, rows):
            m = i if loop == "inner" else o
            assert row[0] == name
            assert abs(m[0] - row[1]) <= TOL, (loop, name, m[0] - row[1])
            assert abs(m[2] - row[4 if loop == "inner" else 5]) <= TOL, (loop, name)


def test_d1_the_tick_resolution_model_without_added_delay_reproduces_the_loop_period_model(real, table):
    """The model of the added-tick control (six states, plant sampled at the tick) with no added delay is the loop-period
    model: the control compares like with like."""
    for (name, inner, outer), (_, r, tau) in zip(table, real["corners"]):
        p, bs = lifted(real["kv"], real["ki"], real["T"], tau, r, real["gamma"], real["mass"], real["divisor"], 0)
        i_tick = Loop(p, bs, V, 1.0, real["T"], "closed").margins()
        assert abs(i_tick[0] - inner[0]) <= 16 * TOL and abs(i_tick[2] - inner[2]) <= 16 * TOL, name
        p, bs = lifted(real["kv"], real["ki"], real["T"], tau, r, real["gamma"], real["mass"], real["divisor"], 0)
        o_tick = Loop(p, bs, Z, real["kz"], real["T"]).margins()
        assert abs(o_tick[0] - outer[0]) <= 16 * TOL and abs(o_tick[2] - outer[2]) <= 16 * TOL, name


# ---- D2: the 17 x 17 box grid, refined by halving ------------------------------------------------------------------------------


def box_members(real, m):
    """[(a, c, r, tau)] of the m x m grid of the (r, tau) box, corners included."""
    return [(a, c, real["r_lo"] + (real["r_hi"] - real["r_lo"]) * a / (m - 1),
             real["tau_lo"] + (real["tau_hi"] - real["tau_lo"]) * c / (m - 1)) for a in range(m) for c in range(m)]


def grid_of(real, kv, ki, kz):
    """{(a, c): (inner PM, inner Ms, outer PM, outer Ms, outer Jury-stable)} on the FINE grid at the gains (kv, ki, kz)."""
    out = {}
    for a, c, r, tau in box_members(real, FINE):
        i = inner_margins(real, kv, ki, r, tau)
        o = outer_margins(real, kv, ki, kz, r, tau)
        out[(a, c)] = (i[0], i[2], o[0], o[2], o[4])
    return out


@pytest.fixture(scope="module")
def grid(real):
    return grid_of(real, real["kv"], real["ki"], real["kz"])


def grid_ok(real, g):
    """The T3 rule on a grid: every member of both loops has a unique crossover, PM >= PM_min - delta_num, Ms <= Ms_max and a
    Jury-stable height loop."""
    for pm_i, ms_i, delta in ((0, 1, real["delta_inner"]), (2, 3, real["delta_outer"])):
        if any(v[pm_i] is None or v[pm_i] < real["PM_min"] - delta or v[ms_i] > real["Ms_max"] for v in g.values()):
            return False
    return all(v[4] for v in g.values())


def sub_grid(grid, step):
    return {k: v for k, v in grid.items() if k[0] % step == 0 and k[1] % step == 0}


def box_side(x, names):
    return names[0] if x == 0 else names[2] if x == FINE - 1 else names[1]


def test_d2_the_17x17_grid_keeps_pm_and_ms_in_the_rule_and_the_last_halving_change_is_recorded(real, table, grid):
    assert len(grid) == FINE * FINE and None not in [v[0] for v in grid.values()] + [v[2] for v in grid.values()]
    coarse = sub_grid(grid, (FINE - 1) // (COARSE - 1))
    assert len(coarse) == COARSE * COARSE
    lines = []
    for label, pm_i, ms_i, delta in (("inner", 0, 1, real["delta_inner"]), ("outer", 2, 3, real["delta_outer"])):
        fine_pm, at_pm = min((v[pm_i], k) for k, v in grid.items())
        coarse_pm, _ = min((v[pm_i], k) for k, v in coarse.items())
        fine_ms, at_ms = max((v[ms_i], k) for k, v in grid.items())
        coarse_ms, _ = max((v[ms_i], k) for k, v in coarse.items())
        where = lambda at: (f"r {box_side(at[0], ('r-', 'r', 'r+'))}, tau {box_side(at[1], ('tau-', 'tau', 'tau+'))} "  # noqa: E731
                            f"(members a={at[0]}, c={at[1]})")
        msg = (f"D2 {label}: grid minimum PM {math.degrees(fine_pm)!r} deg at {where(at_pm)}; maximum Ms {fine_ms!r} at "
               f"{where(at_ms)}; grid refined by halving 9x9 -> 17x17 (core 7.5), last-halving change recorded: PM "
               f"{fine_pm - coarse_pm!r} rad (9x9 minimum {math.degrees(coarse_pm)!r} deg), Ms {fine_ms - coarse_ms!r} "
               f"(9x9 maximum {coarse_ms!r}); PM_min - delta_num = {real['PM_min'] - delta!r} rad")
        print(msg)
        lines.append(msg)
        assert fine_pm <= coarse_pm and fine_ms >= coarse_ms, "the 9x9 members are a subset of the 17x17 ones: " + msg
        assert fine_pm >= real["PM_min"] - delta, msg
        assert fine_ms <= real["Ms_max"], msg
        assert abs(fine_pm - coarse_pm) <= delta + TOL, "the last halving changes the minimum PM by more than delta_num: " + msg
        assert abs(fine_ms - coarse_ms) <= TOL, "the last halving changes the maximum Ms: " + msg
    assert grid_ok(real, grid)
    corner_pm = min(min(i[0], o[0]) for _, i, o in table[1:])
    assert min(min(v[0], v[2]) for v in grid.values()) <= corner_pm + TOL, "the grid contains the corners"


def test_control_d2_a_planted_height_gain_and_a_planted_pm_entry_each_fail_the_grid_rule(real, grid):
    assert grid_ok(real, grid)
    planted = grid_of(real, real["kv"], real["ki"], GAIN_CONTROL * real["kz"])
    assert not grid_ok(real, planted), "k_z x 1.1 must leave the 17x17 rule"
    print(f"D2 control: k_z x {GAIN_CONTROL} on the 17x17 grid: maximum outer Ms {max(v[3] for v in planted.values())!r} "
          f"(Ms_max {real['Ms_max']!r}), minimum outer PM "
          f"{math.degrees(min(v[2] for v in planted.values() if v[2] is not None))!r} deg")
    key = (0, FINE - 1)
    for entry, value in ((0, real["PM_min"] - 2 * real["delta_inner"]), (1, real["Ms_max"] * (1 + 2 ** -20)), (4, False)):
        g = dict(grid)
        g[key] = tuple(value if q == entry else x for q, x in enumerate(g[key]))
        assert not grid_ok(real, g), entry
    g = dict(grid)
    g[key] = (None, *g[key][1:])
    assert not grid_ok(real, g), "a member without a unique crossover must be rejected"


# ---- D3: tightness (L05's form: the recorded bracket and the history of the sup rule) ------------------------------------------


def inner_set(real, kv, ki):
    """[(PM, Ms)] of the inner loop at nominal and the four corners."""
    return [inner_margins(real, kv, ki, r, tau)[::2] for _, r, tau in real["corners"]]


def inner_feasible(real, kv, ki):
    """The sup rule's feasibility of the velocity gains (no tolerance): every corner has a unique crossover, PM >= PM_min,
    Ms <= Ms_max."""
    return all(m[0] is not None and m[0] >= real["PM_min"] and m[1] <= real["Ms_max"] for m in inner_set(real, kv, ki))


def inner_guard_fails(real, kv, ki):
    """True when the gains clearly fail the generator's guard PM_worst >= PM_min + delta_num, Ms_worst <= Ms_max (a PM within TOL
    of the guard does not count as a clear failure)."""
    ms = inner_set(real, kv, ki)
    return any(m[0] is None or m[0] < real["PM_min"] + real["delta_inner"] + TOL or m[1] > real["Ms_max"] for m in ms)


def outer_set(real, kz):
    return [outer_margins(real, real["kv"], real["ki"], kz, r, tau) for _, r, tau in real["corners"]]


def outer_feasible(real, kz):
    return all(m[0] is not None and m[0] >= real["PM_min"] and m[2] <= real["Ms_max"] and m[4] for m in outer_set(real, kz))


def outer_guard_fails(real, kz):
    return any(m[0] is None or m[0] < real["PM_min"] + real["delta_outer"] + TOL or m[2] > real["Ms_max"] or not m[4]
               for m in outer_set(real, kz))


def design_pair(real, w):
    """(k_v, k_i) in double of the design family at the nominal crossover w (vertical.py rule 5): k_i / k_v = w / a with the spacing
    a = sqrt((1 + sin PM_min) / (1 - sin PM_min)), and k_v such that |C P| = 1 at e^(jwT) on the nominal gamma-free model,
    C = k_v (z - z_c) / (z - 1), z_c = 1 - w T / a, P the sampled dT_req -> v."""
    T = real["T"]
    a = math.sqrt((1 + math.sin(real["PM_min"])) / (1 - math.sin(real["PM_min"])))
    z = cmath.exp(1j * w * T)
    phi, gam = sampled_plant(T, real["tau"], 1.0, 0.0, real["mass"])
    x = csolve([[(z if i == j else 0) - phi[i][j] for j in range(3)] for i in range(3)], list(gam))
    kv = 1 / (abs((z - (1 - w * T / a)) / (z - 1)) * abs(x[V]))
    return kv, kv * w / a


def test_d3_the_height_gain_is_tight_against_the_recorded_bracket_and_every_history_point_above_it_fails_the_guard(real):
    o = real["res"]["outer"]
    kz, steps = real["kz"], o["steps_back"]
    lo, hi = o["bracket"]
    ulp = f32_up(kz) - kz
    assert r32(o["history"][len(o["history"]) - 1 - steps]) == kz, "the emitted gain is the history point steps_back from the end"
    assert outer_feasible(real, lo) and not outer_feasible(real, hi), "the recorded bracket is (feasible, infeasible)"
    assert hi - lo <= 2 * ulp, "the bracket is wider than 2 f32 steps"
    assert o["bracket_width"] == hi - lo and o["stepped_down"] == (steps > 0), "the recorded width and step-down flag"
    above = [h for h in o["history"][len(o["history"]) - steps:]]
    assert len(above) == steps
    for h in above:
        assert outer_guard_fails(real, r32(h)), h
    assert not outer_guard_fails(real, kz)
    nxt = f32_up(kz)
    print(f"D3 height gain {kz!r}: bracket ({lo!r}, {hi!r}) wide {(hi - lo) / ulp!r} f32 steps, hi infeasible; {steps} history "
          f"point(s) above fail the guard; the next f32 value {nxt!r} is feasible {outer_feasible(real, nxt)} (not asserted)")


def test_d3_the_velocity_pair_is_tight_against_the_recorded_bracket_and_every_history_point_above_it_fails_the_guard(real):
    i = real["res"]["inner"]
    kv, ki, steps = real["kv"], real["ki"], i["steps_back"]
    lo, hi = i["bracket"]
    hist = i["history"]

    def f32_pair(w):
        return tuple(r32(x) for x in design_pair(real, w))

    assert f32_pair(hist[len(hist) - 1 - steps]) == (kv, ki), "the emitted pair is the design family at the chosen crossover"
    (kv_lo, ki_lo), (kv_hi, ki_hi) = design_pair(real, lo), design_pair(real, hi)
    assert inner_feasible(real, kv_lo, ki_lo) and not inner_feasible(real, kv_hi, ki_hi)
    assert kv_hi - kv_lo <= 2 * (f32_up(kv) - kv) and ki_hi - ki_lo <= 2 * (f32_up(ki) - ki), "the bracket is wider than 2 steps"
    assert i["bracket_width"] == hi - lo and i["stepped_down"] == (steps > 0), "the recorded width and step-down flag"
    for w in hist[len(hist) - steps:]:
        assert inner_guard_fails(real, *f32_pair(w)), w
    assert not inner_guard_fails(real, kv, ki)
    above = [f32_pair(w) for w in hist[len(hist) - steps:]] + [f32_pair(hi)]
    print(f"D3 velocity pair ({kv!r}, {ki!r}): bracket w ({lo!r}, {hi!r}) rad/s, hi infeasible; {steps} history point(s) above "
          f"fail the guard; the f32 pairs above: {above} feasible {[inner_feasible(real, *g) for g in above]} (not asserted)")
    up = (kv, f32_up(ki))
    print(f"D3 k_i one f32 step up: feasible {inner_feasible(real, *up)}")


# ---- D4: Jury at every corner (and every grid member) ---------------------------------------------------------------------------


def test_d4_the_closed_height_loop_with_gamma_is_jury_stable_at_every_corner_and_grid_member(real, table, grid):
    for name, _, outer in table:
        assert outer[4], f"{name}: not Jury-stable"
        assert outer[5] is not None and outer[5] < 1, name
    assert all(v[4] for v in grid.values()), [k for k, v in grid.items() if not v[4]]
    rho, at = max((o[5], n) for n, _, o in table)
    print(f"D4 Jury-stable at the 5 corners and all {len(grid)} grid members; largest Jury radius at the corners {rho!r} at "
          f"{at}: " + ", ".join(f"{n} {o[5]!r}" for n, _, o in table))
    for (name, _, outer), row in zip(table, real["res"]["outer"]["rows"]):
        assert abs(outer[5] - row[3]) <= JURY_AGREE, (name, outer[5], row[3])


# ---- D5: rate.py rule 7's monotonicity condition ---------------------------------------------------------------------------------


def monotone(w_c, T, tau_lo):
    return w_c ** 2 * T * tau_lo < 1 / 2


def test_d5_the_monotonicity_condition_holds_at_the_two_low_tau_corners(real, table):
    out = []
    for name, inner, _ in table:
        if name in LO_TAU:
            cond = inner[1] ** 2 * real["T"] * real["tau_lo"]
            out.append(f"{name} w_c^2 T tau_lo = {cond!r}")
            assert monotone(inner[1], real["T"], real["tau_lo"]), (name, cond)
            assert cond < 1 / 2
    assert len(out) == 2
    print("D5 " + "; ".join(out) + " (each < 1/2)")


def test_control_d5_the_predicate_fails_at_a_planted_crossover_on_the_bound(real):
    w_bound = math.sqrt(1 / (2 * real["T"] * real["tau_lo"]))
    assert not monotone(w_bound * (1 + 2 ** -20), real["T"], real["tau_lo"])
    assert monotone(w_bound * (1 - 2 ** -20), real["T"], real["tau_lo"])


# ---- controls --------------------------------------------------------------------------------------------------------------------


def describe(real, table):
    (pm, at_pm), (ms, at_ms) = worst(table)
    return (f"worst PM {math.degrees(pm)!r} deg at {at_pm} ({math.degrees(pm - real['PM_min'])!r} deg from PM_min), worst Ms "
            f"{ms!r} at {at_ms}")


def test_control_c1_the_velocity_pi_pair_one_tenth_higher_along_the_design_family_breaks_d1(real, table):
    """k_v x 1.1 alone does not break D1 (printed below): at the binding corner (r-, tau+) a larger k_v at fixed k_i raises the
    inner phase margin, and the high-gain corners hold between 3 and 5 times k_v. The pair that the design rule scales with the
    crossover is (k_v, k_i) -> (1.1 k_v, 1.1^2 k_i): kp = w / (j |P|) grows with w and k_i / k_p = w / a (vertical.py rule 5)."""
    assert d1_violations(real, table) == []
    # k_v alone cannot break the margin: more proportional gain adds phase at the binding corner (r-, tau+), so it is printed only.
    alone = corner_table(real, kv=GAIN_CONTROL * real["kv"])
    print(f"C1 (k_v alone x {GAIN_CONTROL}, not asserted): D1 violations {d1_violations(real, alone)}; {describe(real, alone)}")
    planted = corner_table(real, kv=GAIN_CONTROL * real["kv"], ki=GAIN_CONTROL ** 2 * real["ki"])
    bad = d1_violations(real, planted)
    print(f"C1 (k_v x {GAIN_CONTROL}, k_i x {GAIN_CONTROL ** 2}) breaks D1 at {[(b[0], b[1], b[2]) for b in bad]}; "
          f"{describe(real, planted)}")
    assert bad, "the pair along the design family still meets D1"


@pytest.mark.parametrize("which", ["ki", "kz"])
def test_control_c2_c3_the_gain_times_1_1_breaks_d1(real, table, which):
    assert d1_violations(real, table) == []
    planted = corner_table(real, **{which: GAIN_CONTROL * real[which]})
    bad = d1_violations(real, planted)
    print(f"C ({which} x {GAIN_CONTROL}) breaks D1 at {[(b[0], b[1], b[2]) for b in bad]}; {describe(real, planted)}")
    assert bad, f"{which} x {GAIN_CONTROL} still meets D1"


def test_control_c4_one_added_tick_of_delay_breaks_d1(real, table):
    base = corner_table(real, delay=0)
    assert d1_violations(real, base) == []
    delayed = corner_table(real, delay=1)
    bad = d1_violations(real, delayed)
    tick_s = real["T"] / real["divisor"]
    change = worst(base)[0][0] - worst(delayed)[0][0]
    print(f"C4 one added tick ({tick_s!r} s) breaks D1 at {[(b[0], b[1], b[2]) for b in bad]}; {describe(real, delayed)}; "
          f"worst-PM change {change!r} rad (delta_num inner {real['delta_inner']!r}, outer {real['delta_outer']!r})")
    assert bad, "one added tick of delay still meets D1"
    assert change > min(real["delta_inner"], real["delta_outer"]) + TOL


def test_control_c5_the_loop_disabled_is_jury_unstable_by_the_gravity_gradient_pole_and_gamma_zero_is_marginal(real):
    T, tau, mass, gamma = real["T"], real["tau"], real["mass"], real["gamma"]
    p, bs = lifted(0.0, 0.0, T, tau, 1.0, gamma, mass)
    off = closed_matrix(p, bs, 0.0)
    assert not jury_stable(charpoly(off))
    pole = math.exp(math.sqrt(gamma) * T)
    rho = spectral_radius(off)
    p0, bs0 = lifted(0.0, 0.0, T, tau, 1.0, 0.0, mass)
    rho0 = spectral_radius(closed_matrix(p0, bs0, 0.0))
    poly = charpoly(off)
    h = STATE_N * EPS64 / (pole - 1)  # eigenvalue perturbation bound from the matrix rounding over the gap to 1

    def value(x):
        return sum(c * Fraction(x) ** (len(poly) - 1 - i) for i, c in enumerate(poly))

    print(f"C5 loop disabled with gamma: spectral radius {rho!r} vs e^(sqrt(gamma) T) {pole!r}; with gamma = 0: {rho0!r} "
          f"(estimator resolution {RHO_RESOLUTION!r})")
    assert pole > 1 + RHO_RESOLUTION, "the gradient pole must be resolvable by the estimator"
    assert abs(rho - pole) <= RHO_RESOLUTION
    assert value(pole * (1 - h)) * value(pole * (1 + h)) < 0, "the characteristic polynomial changes sign at e^(sqrt(gamma) T)"
    assert rho0 <= 1 + RHO_RESOLUTION, "gamma = 0 leaves the disabled loop marginal (poles on the unit circle), not unstable"
    assert not jury_stable(charpoly(closed_matrix(p0, bs0, 0.0))), "marginal is not strictly stable"
