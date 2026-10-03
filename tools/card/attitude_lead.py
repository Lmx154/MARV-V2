#!/usr/bin/env python3
"""L6 stage (c) attitude gains on the stage (c) closed rate loop (quad spec L6 stage (c); decision 0014 D7, commit 2;
decision 0006 E's rule).

A tool, and the source of the product attitude parameters: flatten.py --out-attitude-lead calls design() and entries_from()
(decision 0014, commit 3). Plain Python double (math, cmath), no numpy, like attitude.py. Imported, not copied: attitude.py's rule (yaw_terms,
sup_rule, pm_worst, feasible, slope_at, axis_k, yaw_effective, yaw_release_crossing, Loop.margin, Loop._evaluate_grid),
rate.py (corner_list, plant_constants), rate_lead.py (the stage (c) design and lowpass_pole) and gyro_chain_design.py
(chain_response).

Rule: decision 0006 E as attitude.py states it (its module docstring, steps 1-11), with five replacements:

  1'. Rate loop (lead decision 3 of 2026-10-01, decision 0014 c2): per axis the f32 gains (kp, ki, kd, T_f) of
      rate_lead.design at N* (PI x lead with the D low-pass), kappa = f32 gain / J_a, at the corners of rate.corner_list, on
      rate_lead's LTI design model at T = D T_s: the flown chain with every notch at omega_th on all four motors (the
      flight design point; step 6' takes the rule over the whole configuration set), the profile's latency, and the
      firmware law (deferred forward-Euler I, D on the measurement by backward difference through the first-order
      low-pass, alpha = 1 - e^(-T/T_f) in double). The 2-periodic dt of the SIL
      stamps (rate_lead step 8) is left out: rate_lead's own analysis gives its effect on the rate loop's worst PM (its
      "effects", firmware model minus design model; the report prints the value). The attitude law samples the true angle at the rate
      execution, as in attitude.py: the chain and the latency are in the rate feedback only.
  2'. Attitude loop at N = att_loop_ratio = 1 (attitude.py step 9; another N is refused: the closed form below is exact at
      N = 1 only). With u = C_PI r - C y (C_PI = kp + ki T/(z - 1), C = C_PI + kd alpha (z - 1)/(T (z - beta)): the firmware
      law, D on y), y = P_y u (rate_lead.LeadLoop.plant: the tick-rate hold, plant, chain and latency, decimated by D) and
      theta = P_theta u (the exact ZOH of 1/(s^2 (1 + tau s)) at T, J normalised):
          G_1 = P_theta C_PI / (1 + C P_y),
          F(z) = (z - 1) G_1 = [(z - 1)^2 P_theta] [(z - 1) C_PI] / ((z - 1)^2 + [(z - 1) C] [(z - 1) P_y]),
          (z - 1)^2 P_theta = T^2 (z + 1)/2 - tau T (z - 1) + tau^2 (1 - e)(z - 1)^2/(z - e),  e = e^(-T/tau),
      every bracket regular at z = 1, z - 1 = 2j sin(theta/2) e^(j theta/2) exactly, and in (z - 1) P_y the alias k = 0
      taken with (z - 1)/(z_s - 1) = sum_{i<D} z_s^i. G_N = F/(z - 1) as in attitude.py step 2, so attitude.py steps 3, 4 and
      6-8 apply unchanged (Loop.margin, sup_rule, pm_worst; the f32 guard PM_min + delta_num, no other margin).
  5'. Stability (lead decision 2), on the lifted closed-loop matrix A_cl = A - k B e_theta^T, (A, B) the rate loop over one
      rate execution with input r, built by stepping the tick-rate loop on unit vectors (the construction of the L05 T3
      oracle, tests/regression/quad/L05/tools/test_attitude_t3.py tick_lifted):
        - radius: rho_K = ||A_cl^(2^K)||_inf^(2^-K) at K = RADIUS_SQUARINGS, by repeated squaring with exact power-of-two
          normalisation, the L05 oracles' method (test_attitude_design.py spectral_radius, test_attitude_t3.py rho_dim). In
          exact arithmetic rho <= rho_K (rho^m = rho(A^m) <= ||A^m||) and rho_K falls to rho as K grows, the excess about
          ln(C)/2^K for C = ||A^m||/rho^m (rho_(K-1) - rho_K is reported as its indicator). The floating-point powers carry
          no useful forward bound here: the worst-case error of a squaring grows by 2 ||M||/||M^2|| per step, and on the
          34-state loop it exceeds the matrix itself before the decay starts, so rho_K < 1 is not by itself a proof.
        - proof: a certified discrete Lyapunov (Stein) certificate. P = sum_(i < m) (A_cl^i)^T A_cl^i by Smith's doubling
          (the same squarings), symmetrised; then lower bounds on lambda_min(P - A_cl^T P A_cl) and on lambda_min(T^T P T)
          (T = L^-T, L the floating Cholesky factor of P: triangular with a nonzero diagonal, so invertible, and the
          congruence keeps the sign) by Gershgorin's discs on the computed products widened by their rounding bounds
          (|fl(XY) - XY| <= gamma_n |X| |Y|, gamma_n = n u/(1 - n u), u = 2^-53: Higham, Accuracy and Stability of Numerical
          Algorithms, 2nd ed., section 3.5; Python 3.12's float sum is at least as accurate as recursive summation), every
          bound rounded up by SLACK. Both bounds > 0 prove rho(A_cl) < 1 for the double matrix A_cl: for A_cl v = lambda v,
          v^H (P - A^T P A) v = (1 - |lambda|^2) v^H P v with both forms positive.
        - stable(k) is the certificate. The candidate P of a corner's first axis is tried on the other two axes at the same
          k (their gains differ by f32 rounding only) and recomputed when it fails; a loop without a certificate is unstable
          to the rule. radius(k) is rho_K when stable(k), else None, as attitude.jury_radius gives it.
      This replaces step 5's Jury test for this loop only; attitude.py's 5-state loop keeps it. The Jury table is not
      numerically valid at this state order (n = 3 plant + latency + 2 per chain stage + 4 controller states, 34 here).
  6'. The configuration set (owner decision 1 of decision 0014's third round: "Attitude design over every chain
      configuration flown or tested"): steps 2'-5' and attitude.py steps 6 and 7 run on the 15 loops of every configuration
      of rate_lead.py rule step 13 (the notches of all four motors at one rotor speed on the notch grid of level L, and
      every notch bypassed, each at the profile's latency and at latency 0). PM_worst(k) is the minimum over the
      configurations x the 15 loops, every one of those loops must be certified stable (step 5') at every gain the rule
      tries, and delta_num's slope is the largest over them. Step 1''s point is not the worst for this loop: the chain's
      lag sits in the rate feedback, so it shows as phase lead from rate command to true rate (on the committed card the
      bypassed, latency-0 configuration binds). Notch grid (core 7.5): L rises from 0 (the two ends) until one halving
      resolves it: at level L's f32 k, every configuration that level L + 1 adds is certified stable and none has a PM below
      level L's PM_worst (no added speed is worse, the strictest form of 7.5's rule); refused past
      rate_lead.NOTCH_GRID_HALVINGS_MAX. Certificates are computed per configuration and corner (the three axes in order,
      the first axis's P tried on the next as in step 5', from a fresh store), one gain at a time over every configuration
      in --procs worker processes; each is a function of (configuration, corner, gain) alone, so the result is the serial
      one for any count.
 11'. att_yaw_t_cross: attitude.py step 11's release, stepped on the same tick-rate loop: the steady state of tracking 1 rad/s
      (omega = 1, m = 0, the latency line at 1, every chain stage at its steady state for the input 1, I = 0, e_prev = 0,
      y_prev = 1, D filter 0) with r = 0 from the release execution on. t_cross is the latest crossing over nominal and the
      four corners AND over every configuration of step 6' evaluated (level L + 1's set; lead decision, stage (c) commit 3);
      the largest, rounded up to f32. Why: the fallback lock (decision 0006 D, guard b) is a backup for a release that does
      not cross; it must not pre-empt a real crossing in any configuration, and a later fallback is the conservative side.
      The release has no attitude feedback, so t_cross does not depend on k. The loops with notches cross earliest: their
      extra lag makes the braking overshoot sooner, while the bypassed, latency-0 loop is the most damped (on the committed
      card it crosses last, at (J+, tau-)).

    uv run python tools/card/attitude_lead.py --report
"""

from __future__ import annotations

import argparse
import cmath
import math
import multiprocessing
import sys
from operator import mul
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import attitude  # noqa: E402
import cpu_quota  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import rate  # noqa: E402
import rate_lead  # noqa: E402

ROOT = HERE.parents[1]
AXES = rate.AXES
THETA = 2                         # the angle's index in the state [omega, m, theta, ...] of LeadLoop.step
# Method constants of rule step 5' (resolution and termination only, not vehicle or design numbers).
RADIUS_SQUARINGS = 24             # the radius estimate's squarings: m = 2^24 executions
STEIN_DOUBLINGS = 40              # the most doublings of the certificate's candidate P
STEIN_TAIL = 2.0 ** -10           # doubling stops once ||A^m||_inf ||A^m||_1 is below this (then P - A^T P A ~ I)
STEIN_DIVERGE = 2.0 ** 256        # doubling stops, uncertified, once ||A^m||_inf exceeds this
UNIT_ROUNDOFF = 2.0 ** -53        # IEEE 754 binary64
SLACK = 1 + 2.0 ** -40            # upward rounding of every error bound (each bound's own rounding is a few ulps)


def _norm(m):
    return max(sum(map(abs, row)) for row in m)


def _matmul(a, b):
    bt = list(zip(*b))
    return [[sum(map(mul, row, col)) for col in bt] for row in a]


def _transpose(m):
    return [list(r) for r in zip(*m)]


def _abs(m):
    return [[abs(x) for x in row] for row in m]


def _gamma(n):
    return n * UNIT_ROUNDOFF / (1 - n * UNIT_ROUNDOFF)


def squaring_radius(a, squarings=RADIUS_SQUARINGS):
    """(rho_K, rho_(K-1)) with rho_K = ||a^(2^K)||_inf^(2^-K), K = squarings: repeated squaring with exact power-of-two
    normalisation, the L05 oracles' method (rule step 5')."""
    s = _norm(a)
    if s == 0:
        return 0.0, 0.0
    c = math.frexp(s)[1]
    m = [[math.ldexp(x, -c) for x in row] for row in a]
    prev, est = math.inf, s
    for k in range(1, squarings + 1):
        sq = _matmul(m, m)
        s = _norm(sq)
        if s == 0:
            return 0.0, est
        e = math.frexp(s)[1]
        m = [[math.ldexp(x, -e) for x in row] for row in sq]
        c = 2 * c + e
        prev, est = est, 2.0 ** ((c + math.log2(_norm(m))) / 2 ** k)
    return est, prev


def _product(x, y):
    """(fl(x y), E) with |fl(x y) - x y| <= E elementwise: gamma_n |x| |y| (Higham section 3.5), |x| |y| itself computed in
    floating point and divided by 1 - gamma_n to bound it from above."""
    g = _gamma(len(y))
    return _matmul(x, y), [[v * g / (1 - g) * SLACK for v in row] for row in _matmul(_abs(x), _abs(y))]


def _gershgorin(w, err):
    """A lower bound on the smallest eigenvalue of every symmetric matrix within err (elementwise) of w: min over the rows
    of (w_ii - err_ii) - sum_(j != i) (|w_ij| + err_ij), each side rounded outward."""
    n = len(w)
    g = _gamma(n)
    rows = [(w[i][i] - err[i][i]) * (1 - 4 * UNIT_ROUNDOFF)
            - sum(abs(w[i][j]) + err[i][j] for j in range(n) if j != i) * (1 + 2 * g) * SLACK for i in range(n)]
    # A non-finite row (overflow) proves nothing: return -inf so the certificate fails instead of min() skipping it.
    if not all(math.isfinite(r) for r in rows):
        return -math.inf
    return min(rows)


def stein_candidate(a):
    """P = sum_(i < 2^K) (a^i)^T a^i by Smith's doubling (P <- P + (a^m)^T P a^m, a^m <- (a^m)^2), symmetrised, or None
    when the doubling diverges or does not reach STEIN_TAIL within STEIN_DOUBLINGS."""
    n = len(a)
    p = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    am = a
    for _ in range(STEIN_DOUBLINGS):
        x = _matmul(_transpose(am), _matmul(p, am))
        p = [[p[i][j] + x[i][j] for j in range(n)] for i in range(n)]
        p = [[(p[i][j] + p[j][i]) / 2 for j in range(n)] for i in range(n)]
        am = _matmul(am, am)
        size = _norm(am)
        if size * _norm(_transpose(am)) < STEIN_TAIL:
            return p
        if size > STEIN_DIVERGE:
            return None
    return None


def stein_margin(a, p):
    """The certified lower bound on lambda_min(P - a^T P a) (rule step 5')."""
    n = len(a)
    pa, e1 = _product(p, a)
    at = _transpose(a)
    x, e2 = _product(at, pa)
    g = _gamma(n)
    e3 = _matmul(_abs(at), e1)
    q = [[p[i][j] - x[i][j] for j in range(n)] for i in range(n)]
    err = [[(e2[i][j] + e3[i][j] / (1 - g) + UNIT_ROUNDOFF * abs(q[i][j])) * SLACK for j in range(n)] for i in range(n)]
    return _gershgorin(q, err)


def _cholesky(p):
    n = len(p)
    low = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = p[i][j] - sum(low[i][k] * low[j][k] for k in range(j))
            if i == j:
                if not s > 0:
                    return None
                low[i][i] = math.sqrt(s)
            else:
                low[i][j] = s / low[j][j]
    return low


def positive_margin(p):
    """A certified lower bound on lambda_min(T^T P T) for T = L^-T (L the floating Cholesky factor of P, any rounding: T is
    triangular with a nonzero diagonal, hence invertible, and the congruence keeps the sign of P); -inf without a factor."""
    n = len(p)
    low = _cholesky(p)
    if low is None:
        return -math.inf
    inv = [[0.0] * n for _ in range(n)]
    for c in range(n):
        for i in range(c, n):
            inv[i][c] = ((1.0 if i == c else 0.0) - sum(low[i][k] * inv[k][c] for k in range(c, i))) / low[i][i]
    t = _transpose(inv)
    if any(t[i][i] == 0 or not math.isfinite(t[i][i]) for i in range(n)):
        return -math.inf
    pt, e1 = _product(p, t)
    tt = _transpose(t)
    w, e2 = _product(tt, pt)
    g = _gamma(n)
    e3 = _matmul(_abs(tt), e1)
    return _gershgorin(w, [[(e2[i][j] + e3[i][j] / (1 - g)) * SLACK for j in range(n)] for i in range(n)])


# ---- the inner loop and the attitude loop ----------------------------------------------------------------------------


class Inner:
    """The rate loop of rule step 1': label, per-axis f32 (kp, ki, kd, T_f) (N m s/rad, N m/rad, N m s^2/rad, s), the tick
    t_s (s), the divisor D, the latency (ticks) and the chain stages (None: H = 1). configs: rule step 6''s configuration
    set, level -> [(key, label, latency, stages)] with the first the step-1' point; None is this loop alone."""

    def __init__(self, label, axes32, t_s, divisor, latency, stages, configs=None):
        self.label, self.axes32, self.t_s, self.divisor = label, [tuple(g) for g in axes32], t_s, divisor
        self.latency, self.stages = latency, stages
        self.t = divisor * t_s
        self.configs = configs


def lead_inner(lead, latency=None, over_set=True):
    """Rule step 1''s loop from a rate_lead.design result (latency: the profile's unless given). With over_set, the rule
    runs over rate_lead's configuration set (rule step 6'); without, over this loop alone (the control)."""
    m, c = lead["model"], lead["chain"]
    lat = m.latency if latency is None else latency
    label = f"stage (c) PI x lead (rate_lead.py, N* {lead['n_star']!r}), chain at omega_th, latency {lat}"
    configs = None
    if over_set:
        def configs(level):
            return [(key, name, cl, [s for s in st if s != gcd.IDENTITY])
                    for key, name, cl, st in rate_lead.configuration_set(c, m.latency, level)]
    return Inner(label, lead["axes32"], m.t_s, m.divisor, lat, m.stages, configs)


def pi_inner(rr, divisor, latency=0):
    """Today's rate loop (rate.py's f32 PI, kd = 0, no chain) at the same tick structure."""
    return Inner(f"today's PI (rate.py), H = 1, latency {latency}",
                 [(rr["axes"][a]["kp"], rr["axes"][a]["ki"], 0.0, 0.0) for a in AXES], rr["T"] / divisor, divisor,
                 latency, None)


class LeadLoop(attitude.Loop):
    """attitude.Loop for one axis and corner on the inner loop `inner` at N = 1: F by rule step 2' (response), stability by
    step 5' (stable, radius), the release by step 11' (release_crossing). gains: the axis's (kappa_p, kappa_i, kappa_d, T_f);
    jt = J_true / J_a; tau the corner's motor lag; shared: the corner's certificate store, common to its three axes."""

    def __init__(self, gains, jt, tau, inner, shared=None, grid=True):
        kp, ki, kd, tf = gains
        self.kp, self.ki, self.kd, self.tau = kp / jt, ki / jt, kd / jt, tau
        self.t_s, self.div, self.lat = inner.t_s, inner.divisor, inner.latency
        self.stages = inner.stages or []
        self.t = inner.t
        self.alpha, self.beta = rate_lead.lowpass_pole(self.t, tf) if kd else (0.0, 0.0)
        self.e_t, self.ome_t = math.exp(-self.t / tau), -math.expm1(-self.t / tau)
        self.e_s, self.ome_s, self.g_s, self.z0_s = rate.plant_constants(tau, self.t_s)
        self.n = 3 + self.lat + 2 * len(self.stages) + 2 + (2 if self.kd else 0)
        if not grid:
            return  # the release of rule step 11' only (step and steady)
        cols = [self.step([1.0 if i == c else 0.0 for i in range(self.n)], 0.0) for c in range(self.n)]
        self.a = [[cols[c][r] for c in range(self.n)] for r in range(self.n)]
        self.b = self.step([0.0] * self.n, 1.0)
        self.shared = {} if shared is None else shared
        self._cert = {}
        self._evaluate_grid()

    def step(self, x, r):
        """One rate execution (D ticks) of the tick-rate loop from state x = [omega, m, theta, latency line, chain stages
        (2 each), I, e_prev, (y_prev, D filter when kd != 0)] with rate command r; the firmware order of rate_lead.rise_time."""
        kp, ki, kd, t, t_s, tau = self.kp, self.ki, self.kd, self.t, self.t_s, self.tau
        e, ome, g = self.e_s, self.ome_s, self.g_s
        w, m, th = x[0], x[1], x[2]
        o = 3 + self.lat
        line = list(x[3:o])
        cs = list(x[o:o + 2 * len(self.stages)])
        o += 2 * len(self.stages)
        integ, e_prev = x[o], x[o + 1]
        y_prev, dfilt = (x[o + 2], x[o + 3]) if kd else (0.0, 0.0)
        u = 0.0
        for tick in range(self.div):
            if line:
                line.append(w)
                y = line.pop(0)
            else:
                y = w
            for i, (b0, b1, b2, a1, a2) in enumerate(self.stages):
                v = y
                y = b0 * v + cs[2 * i]
                cs[2 * i] = b1 * v - a1 * y + cs[2 * i + 1]
                cs[2 * i + 1] = b2 * v - a2 * y
            if tick == 0:
                integ += ki * t * e_prev
                err = r - y
                if kd:
                    dfilt += self.alpha * (-kd * (y - y_prev) / t - dfilt)
                    y_prev = y
                u = kp * err + integ + dfilt
                e_prev = err
            w, m, th = (w + t_s * u + (m - u) * tau * ome, e * m + ome * u,
                        th + t_s * w + tau * g * m + (t_s * t_s / 2 - tau * g) * u)
        return [w, m, th, *line, *cs, integ, e_prev, *([y_prev, dfilt] if kd else [])]

    def resp(self, theta):
        """F(e^{j theta}) of rule step 2'."""
        z = cmath.exp(1j * theta)
        zm1 = 2j * math.sin(theta / 2) * cmath.exp(0.5j * theta)
        zm1sq = zm1 * zm1
        t, tau = self.t, self.tau
        p_theta = t * t * (z + 1) / 2 - tau * t * zm1 + tau * tau * self.ome_t * zm1sq / (z - self.e_t)
        c_pi = self.kp * zm1 + self.ki * t
        c_all = c_pi + (self.kd * self.alpha * zm1sq / (t * (z - self.beta)) if self.kd else 0)
        p_y = 0
        for k in range(self.div):
            ths = (theta + 2 * math.pi * k) / self.div
            zs = cmath.exp(1j * ths)
            hold = sum(zs ** -i for i in range(self.div))
            h = gcd.chain_response(self.stages, ths) if self.stages else 1
            regular = hold * self.g_s * (zs - self.z0_s) / (zs - self.e_s) * h * zs ** -self.lat
            p_y += (sum(zs ** i for i in range(self.div)) if k == 0 else zm1 / (zs - 1)) * regular
        p_y /= self.div
        return p_theta * c_pi / (zm1sq + c_all * p_y)

    def response(self, z):
        return self.resp(cmath.phase(z))

    def closed(self, k):
        """A_cl = A - k B e_theta^T."""
        m = [row[:] for row in self.a]
        for i in range(self.n):
            m[i][THETA] -= k * self.b[i]
        return m

    def certificate(self, k):
        """(margin, rho_K, rho_(K-1)) at gain k: margin the smaller certified lower bound of rule step 5' (> 0 proves
        stability; -inf without a candidate P), the radius estimate only when the margin is positive."""
        if k not in self._cert:
            a = self.closed(k)
            shared, margin = self.shared, -math.inf
            if shared.get("k") == k:
                margin = min(stein_margin(a, shared["p"]), shared["positive"])
            if not margin > 0:
                p = stein_candidate(a)
                if p is not None:
                    positive = positive_margin(p)
                    shared.update(k=k, p=p, positive=positive)
                    margin = min(stein_margin(a, p), positive)
            self._cert[k] = (margin, None, None)
        margin, est, prev = self._cert[k]
        return margin, est, prev

    def estimate(self, k):
        margin, est, prev = self.certificate(k)
        if est is None:
            est, prev = squaring_radius(self.closed(k))
            self._cert[k] = (margin, est, prev)
        return est, prev

    def closed_poly(self, k):
        raise NotImplementedError("rule step 5' replaces the characteristic polynomial for this loop")

    def stable(self, k):
        return self.certificate(k)[0] > 0

    def radius(self, k):
        return self.estimate(k)[0] if self.stable(k) else None

    def steady(self):
        """The state of tracking 1 rad/s (rule step 11')."""
        chain = [v for (b0, _, b2, _, a2) in self.stages for v in (1 - b0, b2 - a2)]
        return [1.0, 0.0, 0.0, *([1.0] * self.lat), *chain, 0.0, 0.0, *([1.0, 0.0] if self.kd else [])]

    def release_crossing(self, limit):
        x = self.steady()
        for k in range(1, limit + 1):
            x = self.step(x, 0.0)
            if x[0] <= 0:
                return k
        return None


class SetLoop(LeadLoop):
    """A LeadLoop of rule step 6''s configuration set: its certificate at a gain it has not seen comes from `table`
    (CertTable), which computes it per unit (configuration and corner)."""
    table = unit = None

    def certificate(self, k):
        if k not in self._cert:
            self.table.fill(self, k)
        return self._cert[k]


def unit_certificates(loops, k):
    """Rule step 6''s certificates of one unit (a configuration's corner, the axes in order) at gain k: LeadLoop.certificate
    from a fresh shared store, so each value depends on (unit, k) alone."""
    shared, out = {}, []
    for loop in loops:
        loop.shared = shared
        loop._cert.pop(k, None)
        out.append(LeadLoop.certificate(loop, k))
    return out


_UNITS = []   # the units of the configuration set being designed, set before the workers fork


def _unit_job(job):
    u, k = job
    return unit_certificates(_UNITS[u], k)


class CertTable:
    """The certificates of rule step 6''s units [[SetLoop per axis]]. map: None evaluates the asking loop's unit only, in
    this process; else a function mapping [(unit, k)] jobs to their certificates, and every unit is evaluated at that k."""

    def __init__(self, units):
        self.units, self.map = units, None
        for u, members in enumerate(units):
            for loop in members:
                loop.table, loop.unit = self, u

    def fill(self, loop, k):
        jobs = [(u, k) for u in (range(len(self.units)) if self.map else [loop.unit])]
        results = self.map(jobs) if self.map else [unit_certificates(self.units[u], k) for u, _ in jobs]
        for (u, _), certs in zip(jobs, results):
            for member, c in zip(self.units[u], certs):
                member._cert.setdefault(k, c)


def build_loops(rr, inner, axes=AXES, grid=True, cls=LeadLoop, prefix=""):
    """[(axis, corner name, LeadLoop)] for the 15 loops (the axes `axes`; grid False: the release of step 11' only; cls the
    loop class; prefix the corner names' prefix)."""
    i = rr["inputs"]
    corners = rate.corner_list(i["tau"], i["b_tau"], i["b_J"])
    out, shared = [], {name: {} for name, _, _ in corners}
    for axis, g in zip(AXES, inner.axes32):
        if axis not in axes:
            continue
        j = rr["axes"][axis]["J"]
        gains = (g[0] / j, g[1] / j, g[2] / j, g[3])
        for name, jt, tau in corners:
            out.append((axis, prefix + name, cls(gains, jt, tau, inner, shared[name], grid)))
    return out


def config_inner(inner, cfg):
    _, label, lat, stages = cfg
    return Inner(label, inner.axes32, inner.t_s, inner.divisor, lat, stages)


def set_loops(rr, inner, cfgs, cls=LeadLoop):
    """(rows [(axis, "<configuration>: <corner>", loop)] configuration-major, units [[loop per axis, in order] per
    configuration and corner]) over the configurations cfgs (rule step 6')."""
    rows, units = [], []
    for cfg in cfgs:
        part = build_loops(rr, config_inner(inner, cfg), cls=cls, prefix=f"{cfg[1]}: ")
        per = {}
        for _, name, loop in part:
            per.setdefault(name, []).append(loop)
        rows += part
        units += list(per.values())
    return rows, units


def set_sup_rule(loops, units, t_a, pm_min, w32, refuse, procs):
    """attitude.sup_rule on the SetLoops `loops` with their certificates from a CertTable of `units` (rule step 6'), in
    `procs` worker processes; the table is left detached (later certificates are computed per unit in this process)."""
    table = CertTable(units)
    if procs <= 1:
        return attitude.sup_rule(loops, t_a, pm_min, w32, refuse)
    _UNITS[:] = units
    try:
        with multiprocessing.get_context("fork").Pool(procs) as pool:
            table.map = lambda jobs: pool.map(_unit_job, jobs, chunksize=1)
            return attitude.sup_rule(loops, t_a, pm_min, w32, refuse)
    finally:
        table.map = None
        _UNITS[:] = []


def release_crossing(rr, inner, cfgs, t_a, refuse):
    """(t_cross, crossings) of rule step 11': attitude.yaw_release_crossing over each configuration of cfgs, the largest."""
    t_cross, crossings = 0.0, []
    for cfg in cfgs:
        t, rows = attitude.yaw_release_crossing(build_loops(rr, config_inner(inner, cfg), ("yaw",), grid=False), t_a, refuse)
        t_cross = max(t_cross, t)
        crossings += [(f"{cfg[1]}: {name}", k, tt) for name, k, tt in rows]
    return t_cross, crossings


def _check_phase(loops, refuse):
    for axis, name, loop in loops:
        if not loop.start_ok:
            refuse(f"{axis} {name}: the low-frequency phase of F is not near 0 (not a single pole at z = 1)")
        if not loop.branch_ok:
            refuse(f"{axis} {name}: the unwrapped phase changes under one grid doubling")


def design(card, budget, scenario, card_path, inner, rate_result=None, procs=None):
    """The rule on the inner loop `inner` (over its configuration set when inner.configs is set, rule step 6'): a dict of
    results like attitude.design's (raises gpc.GenError on refusal). procs: worker processes of step 6''s certificates
    (also of the step-1'-only design's, one unit per corner; default cpu_quota.usable_cpus()); the result does not depend on it."""
    def refuse(reason):
        gpc.refuse(card_path, "attitude_lead", f"{reason} (decisions 0006, 0014)")

    rr = rate_result if rate_result is not None else rate.design(card, budget, scenario, card_path)
    pm_min = rr["inputs"]["PM_min"]
    yt = attitude.yaw_terms(rr, scenario, card_path, refuse)
    n = attitude.LOOP_RATIO
    if n != 1:
        refuse(f"att_loop_ratio {n}: the closed-form loop of step 2' is exact at N = 1 only")
    t_a = n * inner.t
    levels, flight = None, ""
    procs = procs or cpu_quota.usable_cpus()
    if inner.configs is None:
        loops = build_loops(rr, inner, cls=SetLoop)
        units = {}
        for _, name, loop in loops:
            units.setdefault(name, []).append(loop)
        _check_phase(loops, refuse)
        final = set_sup_rule(loops, list(units.values()), t_a, pm_min, yt["w32"], refuse, procs)
    else:
        level, levels = 0, []
        while True:
            cfgs = inner.configs(level)
            loops, units = set_loops(rr, inner, cfgs, SetLoop)
            _check_phase(loops, refuse)
            final = set_sup_rule(loops, units, t_a, pm_min, yt["w32"], refuse, procs)
            known = {c[0] for c in cfgs}
            added = [c for c in inner.configs(level + 1) if c[0] not in known]
            new, _ = set_loops(rr, inner, added)
            _check_phase(new, refuse)
            worst_new, detail_new = attitude.pm_worst(new, final["k32"], yt["w32"])
            bad = [f"{a} {name}: {why}" for a, name, _, _, _, why in detail_new if why]
            if bad:
                refuse(f"at notch-grid level {level + 1}, the f32 k {final['k32']!r} of level {level}: " + "; ".join(bad))
            levels.append({"level": level, "configs": cfgs, "k32": final["k32"], "pm_worst": final["pm_worst"],
                           "added": added, "added_worst": worst_new, "added_detail": detail_new})
            if worst_new >= final["pm_worst"]:
                break
            level += 1
            if level > rate_lead.NOTCH_GRID_HALVINGS_MAX:
                refuse(f"an added notch speed is still worse past notch-grid level {rate_lead.NOTCH_GRID_HALVINGS_MAX} "
                       "(step 6')")
        flight, evaluated = f"{cfgs[0][1]}: ", inner.configs(level + 1)
    if not all(loop.radius(attitude.axis_k(axis, final["k32"], yt["w32"])) is not None for axis, _, loop in loops):
        refuse("the final gains are not certified stable at a corner (step 5')")
    if levels is None:
        t_cross, crossings = attitude.yaw_release_crossing(loops, t_a, refuse)
    else:
        t_cross, crossings = release_crossing(rr, inner, evaluated, t_a, refuse)
    return {"T": inner.t, "inner": inner, "rate": rr, "inputs": dict(rr["inputs"]), "loops": loops, **yt, "N": n,
            "T_a": t_a, "t_cross": t_cross, "crossings": crossings, "final": final, "k": final["k"], "k32": final["k32"],
            "k_yaw_effective": attitude.yaw_effective(final["k32"], yt["w32"]), "pm_worst": final["pm_worst"],
            "levels": levels, "flight": flight}


K_METHOD = (
    "derived(attitude P gain by the sup rule of decision 0006 E on the stage (c) rate loop (decision 0014 D7): k = sup{k' : "
    "PM_worst(k'') >= PM_min for all k'' in (0, k']} at T_a = att_loop_ratio x the rate-loop period, PM_worst the minimum over "
    "nominal and the tau_robustness_band x inertia_robustness_band corners of each axis, on the exact lifted sampled-data "
    "model of the f32 PID rate gains (rate_lead.py N*) with the gyro chain and the IMU latency, over every chain "
    "configuration flown or tested (all notches at each rotor speed of a halving-resolved grid from omega_th to omega_max, "
    "and every notch bypassed, each at the profile's latency and at 0; rule step 6'), guard PM_worst(f32) >= PM_min + "
    "delta_num, every loop of every configuration certified stable by the Stein certificate; tools/card/attitude_lead.py)")
K_SOURCE = ("vehicle card; design-budget PM_min, Ms_max, d_path_noise_budget, tau_robustness_band, inertia_robustness_band; "
            "scenario register tick_period_num_us, tick_period_den, rate_loop_divisor; the sensor profile the card names; the "
            "derived rate gains (decision 0014, rate_lead.py); decisions 0006 E, 0014 D7")
T_CROSS_METHOD = ("derived(t_cross = the latest, over nominal and the four tau x J corners of the yaw axis and over the chain "
                  "configurations the firmware flies or is tested in (all notches at each rotor speed of the halving-resolved "
                  "grid from omega_th to omega_max, and every notch bypassed, each at the profile's latency and at latency 0; "
                  "rule step 6'), time k T_a of the first attitude execution k >= 1 at which the "
                  "design-model yaw rate is <= 0 after a release: the stage (c) rate loop (PID, gyro chain, latency) stepped at "
                  "the tick, steady yaw rate 1 rad/s, attitude command held at zero yaw rate, no attitude feedback on yaw; "
                  "rounded up to f32, so the fallback never pre-empts a crossing in any configuration; "
                  "tools/card/attitude_lead.py rule step 11')")
T_CROSS_SOURCE = ("the yaw rate gains and corners of the stage (c) rate derivation (decision 0014, rate_lead.py), "
                  "att_loop_ratio (parent-rate rule); owner decision 18 of decision 0006")


def entries_from(result):
    """The 5 attitude parameters (ordered (name, params_gen entry) pairs) of a design() result, in attitude.entries_from's
    order; w, alpha_min and the loop ratio keep attitude.py's provenance (the rules are unchanged)."""
    base = dict(attitude.entries_from(result))
    k, t_cross = base["att_kp"], base["att_yaw_t_cross"]
    k.update(method=K_METHOD, source=K_SOURCE)
    t_cross.update(method=T_CROSS_METHOD, source=T_CROSS_SOURCE)
    return list(base.items())


# ---- the report ------------------------------------------------------------------------------------------------------


def summary(result):
    """(k32, k, PM nominal per axis (step 1''s configuration), (worst PM, axis, corner), crossover range rad/s) of a design
    dict (this module's or attitude.design's)."""
    det, t_a = result["final"]["detail"], result["T_a"]
    nominal = {axis: pm for axis, name, pm, _, _, _ in det if name == result.get("flight", "") + "nominal"}
    worst = min(det, key=lambda x: x[2])
    xs = [x[3] / t_a for x in det]
    return result["k32"], result["k"], nominal, (worst[2], worst[0], worst[1]), (min(xs), max(xs))


def _set_lines(r):
    """Rule step 6''s report lines: the notch-grid halvings, then the worst PM at f32(k) per configuration."""
    if r["levels"] is None:
        return []
    deg, f = math.degrees, r["final"]
    lines = ["configuration set (step 6'): notch-grid level, configurations, f32 k, PM_worst deg over them; the worst PM "
             "deg of the configurations the next level adds, at that k"]
    for lv in r["levels"]:
        w = min(lv["added_detail"], key=lambda x: x[2])
        lines.append(f"  level {lv['level']} ({len(lv['configs'])} configurations): k32 {lv['k32']!r}, PM_worst "
                     f"{deg(lv['pm_worst']):.7f}; level {lv['level'] + 1} adds {len(lv['added'])}: worst "
                     f"{deg(lv['added_worst']):.7f} ({w[0]} {w[1]})"
                     + (": resolved" if lv["added_worst"] >= lv["pm_worst"] else ": halve"))
    worst = {}
    for axis, name, pm, _, _, _ in f["detail"] + r["levels"][-1]["added_detail"]:
        label, corner = name.rsplit(": ", 1)
        worst[label] = min(worst.get(label, (math.inf,)), (pm, axis, corner))
    lines.append("  per configuration at f32(k): worst PM deg (axis corner), PM - PM_min rad")
    lines += [f"    {label:<40} {deg(pm):.7f} ({axis} {corner}) {pm - r['inputs']['PM_min']:+.3e}"
              for label, (pm, axis, corner) in worst.items()]
    return lines


def report_lines(result, today, lead=None):
    deg = math.degrees
    r, inner, f = result, result["inner"], result["final"]
    lines = [
        "MARV L6 stage (c) attitude gains on the stage (c) rate loop (tools/card/attitude_lead.py; decisions 0006 E, 0014 D7)",
        f"inner loop: {inner.label}; T {inner.t!r} s, T_s {inner.t_s!r} s, D {inner.divisor}",
        "f32 rate gains (kp N m s/rad, ki N m/rad, kd N m s^2/rad, T_f s):",
        *[f"  {a:<5} {g[0]!r}  {g[1]!r}  {g[2]!r}  {g[3]!r}" for a, g in zip(AXES, inner.axes32)],
    ]
    if lead is not None:
        eff = dict(lead["effects"])
        names = list(eff)
        lines.append(f"  rate_lead N* {lead['n_star']!r}, w_c,nom {lead['omega_c']!r} rad/s; 2-periodic dt left out (step 1'): "
                     f"rate_lead's {names[1]} minus {names[0]}, roll worst PM: {eff[names[1]] - eff[names[0]]:+.3e} rad")
    lines += [
        f"att_loop_ratio {r['N']} (parent-rate rule, attitude.py step 9; EMB-3: the attitude law's cost per execution is",
        "  unchanged by the new gains; the rate group grows by the chain at every tick and the D filter (decisions 0013,",
        "  0014), re-checked at L9 with measured WCETs)",
        f"{'':<30}{'new (this rule)':<28}today (attitude.py, live)",
    ]
    new, old = summary(r), summary(today)

    def row(label, a, b):
        lines.append(f"{label:<30}{a + ' ':<28}{b}")

    row("k (f32) 1/s", repr(new[0]), repr(old[0]))
    row("k (double) 1/s", repr(new[1]), repr(old[1]))
    for axis in AXES:
        row(f"PM nominal {axis} (deg)", f"{deg(new[2][axis]):.6f}", f"{deg(old[2][axis]):.6f}")
    row("PM worst (deg), loop", f"{deg(new[3][0]):.6f} {new[3][1]} {new[3][2]}", f"{deg(old[3][0]):.6f} {old[3][1]} {old[3][2]}")
    row("crossover range (rad/s)", f"{new[4][0]:.6f} .. {new[4][1]:.6f}", f"{old[4][0]:.6f} .. {old[4][1]:.6f}")
    row("w (f32)", repr(r["w32"]), repr(today["w32"]))
    row("att_yaw_alpha_min (rad/s^2)", repr(r["alpha_min"]), repr(today["alpha_min"]))
    row("att_yaw_t_cross (s)", repr(r["t_cross"]), repr(today["t_cross"]))
    row("yaw effective gain", repr(r["k_yaw_effective"]), repr(today["k_yaw_effective"]))
    lines += [
        f"guard (attitude.sup_rule): delta_num {f['delta_num']!r} rad, PM_worst(f32) - PM_min - delta_num "
        f"{f['pm_worst'] - r['inputs']['PM_min'] - f['delta_num']:+.3e} rad, stepped down {f['stepped_down']} "
        f"({f['steps_back']} back), bracket {f['bracket'][0]!r} .. {f['bracket'][1]!r}, {f['bisections']} bisections",
        *_set_lines(r),
        "yaw release (step 11'): " + "  ".join(f"{name} {k} ({t!r} s)" for name, k, t in r["crossings"]),
        f"per loop at f32(k) (radius: rho_K of step 5' at K = {RADIUS_SQUARINGS}, shown only with a certificate):",
        *attitude._detail_lines(f["detail"], r["T_a"]),
        "step 5' per loop at f32(k): Stein certificate margin (> 0 proves rho < 1), rho_(K-1) - rho_K, state order:",
    ]
    for axis, name, loop in r["loops"]:
        kk = attitude.axis_k(axis, f["k32"], r["w32"])
        margin = loop.certificate(kk)[0]
        est, prev = loop.estimate(kk)
        lines.append(f"  {axis:<5} {name:<8} margin {margin:.6f}  rho_K {est!r}  rho_(K-1) - rho_K {prev - est:.3e}  n {loop.n}")
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--report", action="store_true", required=True, help="print the derivation report")
    ap.add_argument("--card", default=str(ROOT / "vehicles" / "uzh_neurobem_5in.yaml"))
    ap.add_argument("--budget", default=str(ROOT / "design" / "budget.yaml"))
    ap.add_argument("--scenario", default=str(ROOT / "design" / "scenario_values.yaml"))
    ap.add_argument("--profile", default=str(ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"))
    args = ap.parse_args(argv)
    try:
        card, budget, scenario, profile = rate_lead.load(args.card, args.budget, args.scenario, args.profile)
        rr = rate.design(card, budget, scenario, args.card)
        lead = rate_lead.design(card, budget, scenario, profile, args.card, args.profile)
        today = attitude.design(card, budget, scenario, args.card, rr)
        result = design(card, budget, scenario, args.card, lead_inner(lead), rr)
        for line in report_lines(result, today, lead):
            print(line)
    except (gpc.GenError, gcd.DesignError) as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
