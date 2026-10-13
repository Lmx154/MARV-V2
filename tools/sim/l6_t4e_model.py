#!/usr/bin/env python3
"""L6 stage (e) T4e design models: the band, H, TOL and Q of the L4 and L5 scenarios regenerated on the T4e linear design
model, and the L5 noise term (decision 0019 rulings 3-6, second-round rulings 1-3; the noise suite design memo of
2026-10-04, F-a, sections 1 and 3, commit C6a). A library of the refdata set quad/L06/t4e
(tests/regression/quad/L06/t4e/reference/t4e_reference.py); no command line.

The T4e linear design model (memo F-a and section 1) is each frozen T4c design model with its sensor replaced: the gyro
sample is the plant rate `latency` ticks earlier (before tick 0 the rate of tick 0, sim/plant/src/imu_model.hpp step 1),
then the chain: the 12 notches (motor-major, harmonic-minor, as fw/gyro_chain update_notches) every one at omega_hover,
active iff omega_hover >= omega_th and f0 < f_s / 2, then the low-pass, each the firmware's direct form I, every stage's
history seeded with the first sample at tick 0 (gyro_chain.hpp seed). T4c is the same code with latency 0 and the
low-pass alone; there every model below equals its frozen original operation for operation (the T1 checks of
tests/regression/quad/L06/tools/test_l6_t4e_model.py). The stage coefficients are the fixtures' f32 values in double
(gyro_chain_design.notch_coeffs and the L4 oracle's lowpass_coeffs at the exact tick), as the T3 oracles use them.

L4 (rate_t3_oracle.closed_loop and run_l4.script_response generalised: l4_loop). Band: pointwise min / max over the
17 x 17 tau x J grid, H its last-halving change from the 9 x 9 subset (l4_band). TOL (l4_tol): the oracles' first-order
float32 rule, TOL = sum over the nodes of l1_node x max rho_node, with the nodes x (the sample's cast), one node per
chain stage (its nine roundings and its float coefficients' error, the stage's output; the low-pass is the oracle's f),
r, e, I, D and u. The notches' float coefficient bound is notch_f32_error (below), the low-pass's the oracle's
lowpass_f32_error. Q: no L4 predicate has one (decision 0005: F = TOL + H).

L5 (attitude_t3_oracle and recovery_model generalised). Band members: Member4e (the oracle's Member), tilt_member_run,
yaw_member_run (the oracle's), recovery_run (recovery_model.member_run); their envelope, H and Q (Quantiser / Quant3,
the oracle's rule) as the frozen code. The T4e golden (simulate4e: the oracle's simulate, its rounding bookkeeping per
stage: the nine roundings of each stage at its output and the signals its float coefficients multiply, the notches' bounds
from notch_f32_error) gives the kinematics term (1 against 2 sub-steps; recovery_model's rule for the recoveries) and TOL
(l5_tolerance: the oracle's tolerance, its node sums, region bounds, coefficient bounds and the yaw lock's composition
through the error state, over the T4e nodes: ref, gyro, one per stage, e, I, D, u, one coefficient node per stage, alpha;
the state as l5_state_size).

The L5 noise model (memo section 1; C4 left it UNKNOWN). L5Lin is the L5 oracle's linearised closed loop (Lin) at a band
member, with the T4e sensor and the firmware's seed path (the chain seeded with the first sample, the seed execution:
u = 0, I = 0, Df = 0, y_prev = y). The attitude law is frozen at the ends of its Jacobian's range, kp cos(theta_e / 2)
from kp to 0 (0 when clamped): fb = kp (k_yaw w on yaw, the oracle's Lin) and fb = 0, the larger sigma of the two at each
point (INFERRED, as C4's frozen operating points; verified by the nightly Monte Carlo). The noise enters at the sample,
after the latency (l6_noise_term's model); sigma_c(n) per observation from l6_noise_term.unit_sums. The L5 attitude
source is truth (memo F-f), so only the rate loop's sample carries noise.
"""

from __future__ import annotations

import dataclasses
import math
import multiprocessing
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import l6_ff_eval as fe  # noqa: E402  (puts tools/card and the L4 oracle on sys.path)
import l6_noise_term as nt  # noqa: E402

ROOT = fe.ROOT
L5_GZ = ROOT / "tests" / "regression" / "quad" / "L05" / "gz"
if str(L5_GZ) not in sys.path:
    sys.path.insert(0, str(L5_GZ))
import gyro_chain_design as gcd  # noqa: E402
import recovery_model as rm  # noqa: E402  (puts the L5 T3 oracle on sys.path)

lo4 = fe.oracle  # rate_t3_oracle
ao = rm.oracle   # attitude_t3_oracle
U = lo4.UNIT_ROUNDOFF
GRID = fe.GRID   # the oracles' 17 x 17 grid; the 9 x 9 grid is its even-index subset
L4_PHASES = (1, 2)  # 0-based: the L4 oracle's two dt phases of an injection (its main(): executions 2 and 3)
LOWPASS = "f"       # the low-pass stage's node name, the L4 oracle's f


# ---- the chain ------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Stage:
    """One direct-form-I stage: (b0, b1, b2, a1, a2) in double, the first-order bound of each float coefficient's distance
    from it (the firmware computes them in binary32), and its rounding node's name."""
    coeffs: tuple
    err: tuple
    node: str


def notch_f32_error(omega, harmonic, q, period):
    """First-order bounds on |c - fl(c)|, c = b0, b1, b2, a1, a2 of the notch at harmonic x omega, when the firmware
    computes it in binary32 (gyro_chain.hpp update_notches and notch_coeffs; the rounding model of the L4 oracle's
    lowpass_f32_error): omega cast to float, f0 = fl(fl(h omega) / two_pi_), two_pi_ = 2 fl(pi), w0 =
    fl(fl(two_pi f0) period_f), period_f = fl(fl(num / den) / 1e6) (two roundings), each relative error added (no
    cancellation is taken); sinf and cosf within one ulp of their result (INFERRED, as the oracle's tanf and expf); then
    every operation of the formula as written (2 cw and 2 q exact)."""
    h_rounds = 0 if harmonic in (1, 2) else 1  # h omega is exact for a power of two
    rel_w0 = (1 + h_rounds + 1 + 1) * U + (1 + 1 + 2 + 1) * U
    f0 = harmonic * omega / (2 * math.pi)
    w0 = 2 * math.pi * f0 * period
    dw0 = rel_w0 * w0
    sw, cw = math.sin(w0), math.cos(w0)
    dsin = abs(cw) * dw0 + lo4.ulp32(sw)
    dcos = abs(sw) * dw0 + lo4.ulp32(cw)
    alpha = sw / (2 * q)
    dalpha = dsin / (2 * q) + U * abs(alpha)
    den = 1 + alpha
    dden = dalpha + U * abs(den)
    norm = 1 / den
    dnorm = norm * (dden / den + U)
    b1 = -(2 * cw) * norm
    db1 = 2 * dcos * norm + 2 * abs(cw) * dnorm + U * abs(b1)
    m = 1 - alpha
    dm = dalpha + U * abs(m)
    a2 = m * norm
    da2 = dm * norm + abs(m) * dnorm + U * abs(a2)
    return dnorm, db1, dnorm, db1, da2


def notch_node(motor, harmonic):
    return f"notch_m{motor}_h{harmonic}"


def lowpass_stage(p):
    su = lo4.Setup(p)
    return Stage(su.lowpass, su.lowpass_error, LOWPASS)


def t4c_stages(p):
    """The T4c chain: every notch bypassed (the identity, exact), the low-pass."""
    return (lowpass_stage(p),)


def t4e_stages(p, omega_hover):
    """The T4e linear chain: the 12 notches at omega_hover (motor-major, harmonic-minor; an inactive one is the identity
    and is left out, gyro_chain_design.chain_stages's rule), then the low-pass."""
    su = lo4.Setup(p)
    period = su.tick_s
    qs = (p["gyro_notch_q_h1"], p["gyro_notch_q_h2"], p["gyro_notch_q_h3"])
    out = []
    for motor in range(1, gcd.MOTORS + 1):
        for harmonic, q in zip(gcd.HARMONICS, qs):
            f0 = harmonic * omega_hover / (2 * math.pi)
            if omega_hover >= p["gyro_notch_omega_min"] and f0 < 1 / (2 * period):
                out.append(Stage(gcd.notch_coeffs(f0, q, period), notch_f32_error(omega_hover, harmonic, q, period),
                                 notch_node(motor, harmonic)))
    out.append(lowpass_stage(p))
    return tuple(out)


@dataclasses.dataclass(frozen=True)
class Chain:
    """The sensor: latency in ticks, then the stages."""
    name: str
    latency: int
    stages: tuple

    @property
    def coeffs(self):
        return tuple(s.coeffs for s in self.stages)

    def sensor(self):
        """As l6_ff_eval's Sensor (the l6_noise_term models read latency and stages)."""
        return fe.Sensor(self.name, self.latency, self.coeffs)


# ---- L4: the generalised closed loop, its band and TOL ------------------------------------------------------------

def l4_law(p, axis):
    return (p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"], p[f"rate_tau_ref_{axis}"],
            p[f"rate_d_filter_tau_{axis}"])


def l4_loop(law, chain, tmap, sps, stamps, divisor, inject=None, want_rho=False):
    """w per execution n = 0 .. len(sps) - 1 (n = 0 the seed execution) of the linear design model: the L4 oracle's
    closed_loop and run_l4.script_response operation for operation (module docstring), with `chain`'s latency and
    stages. sps: the setpoint per execution; stamps: their stamps (us). inject = ("x", n, tick) at the sample (after the
    latency, before the chain), (node, n, tick) at a stage's output, or (node, n) at r, e, I, D, u: a unit injection.
    want_rho also returns per node the largest first-order rounding injection (the oracle's rule; per stage its own)."""
    kp, ki, kd, tau_ref, t_f = law
    a12, b1, a22, b2 = tmap
    stages = chain.stages
    hist = [[0.0, 0.0, 0.0, 0.0] for _ in stages]
    delay = [0.0] * chain.latency
    w = um = 0.0
    r = integral = e_prev = y_prev = d_f = u = 0.0
    y_out = []
    rho = dict.fromkeys(("x", *(s.node for s in stages), "r", "e", "I", "D", "u"), 0.0)
    for n, sp in enumerate(sps):
        y_out.append(w)
        for tick in range(divisor):
            if delay:
                delay.append(w)
                v = delay.pop(0)
            else:
                v = w
            x = v
            if inject and inject == ("x", n, tick):
                x += 1.0
            if n == 0 and tick == 0:
                for hs in hist:
                    hs[0] = hs[1] = hs[2] = hs[3] = x
            if want_rho:
                rho["x"] = max(rho["x"], U * abs(v))
            for st, hs in zip(stages, hist):
                c0, c1, c2, c3, c4 = st.coeffs
                x1, x2, f1, f2 = hs
                p0, p1, p2, p3, p4 = c0 * x, c1 * x1, c2 * x2, c3 * f1, c4 * f2
                s1 = p0 + p1
                s2 = s1 + p2
                s3 = s2 - p3
                f = s3 - p4
                if want_rho:
                    dc0, dc1, dc2, dc3, dc4 = st.err
                    rho[st.node] = max(rho[st.node], U * (abs(p0) + abs(p1) + abs(p2) + abs(p3) + abs(p4) + abs(s1)
                                                          + abs(s2) + abs(s3) + abs(f))
                                       + dc0 * abs(x) + dc1 * abs(x1) + dc2 * abs(x2) + dc3 * abs(f1) + dc4 * abs(f2))
                if inject and inject == (st.node, n, tick):
                    f += 1.0
                hs[0], hs[1], hs[2], hs[3] = x, x1, f, f1
                x = f
            if tick == 0:
                y = x
                if n == 0:
                    r, e_prev, integral, d_f, u = y, 0.0, 0.0, 0.0, 0.0
                else:
                    dt = (stamps[n] - stamps[n - 1]) * lo4.MICROSECOND
                    xr = dt / tau_ref
                    alpha = -math.expm1(-xr)
                    r_prev = r
                    r = r + alpha * (sp - r)
                    if inject and inject == ("r", n):
                        r += 1.0
                    e = r - y
                    if inject and inject == ("e", n):
                        e += 1.0
                    i_prev_term = ki * e_prev * dt
                    integral = integral + i_prev_term
                    if inject and inject == ("I", n):
                        integral += 1.0
                    d = -kd * (y - y_prev) / dt
                    d_prev = d_f
                    if t_f > 0.0:
                        xd = dt / t_f
                        alpha_d = -math.expm1(-xd)
                        d_f = d_f + alpha_d * (d - d_f)
                    else:
                        d_f = d
                    if inject and inject == ("D", n):
                        d_f += 1.0
                    pi_sum = kp * e + integral
                    u = pi_sum + d_f
                    if inject and inject == ("u", n):
                        u += 1.0
                    if want_rho:
                        d_alpha = math.exp(-xr) * xr * 2 * U + lo4.ulp32(math.exp(-xr)) + U * alpha
                        s = abs(sp - r_prev)
                        rho["r"] = max(rho["r"], d_alpha * s + 2 * U * alpha * s + U * abs(r))
                        rho["e"] = max(rho["e"], U * abs(e))
                        rho["I"] = max(rho["I"], 3 * U * abs(i_prev_term) + U * abs(integral))
                        rho_d = 4 * U * abs(d)
                        if t_f > 0.0:
                            d_alpha_d = math.exp(-xd) * xd * 2 * U + lo4.ulp32(math.exp(-xd)) + U * alpha_d
                            s_d = abs(d - d_prev)
                            rho_d = alpha_d * rho_d + d_alpha_d * s_d + 2 * U * alpha_d * s_d + U * abs(d_f)
                        rho["D"] = max(rho["D"], rho_d)
                        rho["u"] = max(rho["u"], U * (abs(kp * e) + abs(pi_sum) + abs(u)))
                    e_prev = e
                y_prev = y
            w, um = w + a12 * um + b1 * u, a22 * um + b2 * u
    return (y_out, rho) if want_rho else y_out


def oracle_stamps(su, n_exec):
    """The L4 oracle's stamps of its executions 1 .. n_exec (0-based here)."""
    return [su.stamp_us(n) for n in range(1, n_exec + 1)]


def members(points):
    """(i, j, s, t) of the points x points tau x J grid (the oracles' formula; the corners included)."""
    return [(i, j, -1.0 + 2.0 * i / (points - 1), -1.0 + 2.0 * j / (points - 1)) for i in range(points)
            for j in range(points)]


_CTX = {}


def _pool_map(fn, tasks, procs):
    if procs > 1 and len(tasks) > 1:
        with multiprocessing.get_context("fork").Pool(procs) as pool:
            return pool.map(fn, tasks, chunksize=1)
    return [fn(x) for x in tasks]


def _l4_member(args):
    i, j, s, t = args
    x = _CTX
    p, a = x["p"], x["axis"]
    su = lo4.Setup(p)
    tmap = su.tick_map(p[lo4.INERTIA[a]] * (1 + s * su.j_band), p["motor_tau"] * (1 + t * su.tau_band))
    return i, j, l4_loop(x["law"], x["chain"], tmap, x["sps"], x["stamps"], su.divisor)


def l4_band(p, axis, chain, sps, stamps, procs=1):
    """dict(lo, hi, lo9, hi9, H) of the linear model driven by sps over the 17 x 17 grid (and its 9 x 9 subset)."""
    _CTX.clear()
    _CTX.update(p=p, axis=axis, law=l4_law(p, axis), chain=chain, sps=list(sps), stamps=list(stamps))
    rows = _pool_map(_l4_member, members(GRID), procs)
    lo = hi = lo9 = hi9 = None
    for i, j, y in rows:
        lo = y if lo is None else [min(a, b) for a, b in zip(lo, y)]
        hi = y if hi is None else [max(a, b) for a, b in zip(hi, y)]
        if i % 2 == 0 and j % 2 == 0:
            lo9 = y if lo9 is None else [min(a, b) for a, b in zip(lo9, y)]
            hi9 = y if hi9 is None else [max(a, b) for a, b in zip(hi9, y)]
    change = max(max(abs(a - b) for a, b in zip(lo9, lo)), max(abs(a - b) for a, b in zip(hi9, hi)))
    return {"lo": lo, "hi": hi, "lo9": lo9, "hi9": hi9, "H": change}


def l4_nodes(chain):
    return ("x", *(s.node for s in chain.stages), "r", "e", "I", "D", "u")


def _l1_task(args):
    node, k, tick = args
    x = _CTX
    p, a, chain, n_exec = x["p"], x["axis"], x["chain"], x["n_l1"]
    su = lo4.Setup(p)
    tmap = su.tick_map(p[lo4.INERTIA[a]], p["motor_tau"])
    at = (node, k) if tick is None else (node, k, tick)
    g = l4_loop(x["law"], chain, tmap, [0.0] * n_exec, oracle_stamps(su, n_exec), su.divisor, inject=at)
    return node, k, sum(abs(v) for v in g[k + 1:])


def l4_tol(p, axis, chain, sps, stamps, n_l1, procs=1):
    """(TOL, l1 per node, rho per node) of the oracle's rule (module docstring): rho along the nominal trajectory driven by
    sps at stamps, l1 the largest over the two dt phases of the summed absolute responses, at executions after the
    injection, to the unit injections of the node in that execution (one per tick for x and the stages), over n_l1
    executions of the oracle's stamps."""
    su = lo4.Setup(p)
    law = l4_law(p, axis)
    tmap = su.tick_map(p[lo4.INERTIA[axis]], p["motor_tau"])
    _, rho = l4_loop(law, chain, tmap, sps, stamps, su.divisor, want_rho=True)
    _CTX.clear()
    _CTX.update(p=p, axis=axis, law=law, chain=chain, n_l1=n_l1)
    tick_nodes = ("x", *(s.node for s in chain.stages))
    tasks = [(node, k, tick) for node in tick_nodes for k in L4_PHASES for tick in range(su.divisor)]
    tasks += [(node, k, None) for node in ("r", "e", "I", "D", "u") for k in L4_PHASES]
    sums = {}
    for node, k, v in _pool_map(_l1_task, tasks, procs):
        sums[(node, k)] = sums.get((node, k), 0.0) + v
    nodes = l4_nodes(chain)
    l1 = {node: max(sums[(node, k)] for k in L4_PHASES) for node in nodes}
    return sum(l1[node] * rho[node] for node in nodes), l1, rho


# ---- L5: the band members on the T4e chain ------------------------------------------------------------------------

class Member4e:
    """The L5 oracle's Member (one band member: the exact sampled-data rate loop of the design model at J (1 + s b_J),
    tau (1 + t b_tau), integrator and D step T) with `chain` in place of the low-pass; the state is held here."""

    def __init__(self, su, axis, s, t, chain):
        p = su.p
        name = ao.AXES[axis]
        self.su = su
        self.kp, self.ki, self.kd = p[f"rate_kp_{name}"], p[f"rate_ki_{name}"], p[f"rate_kd_{name}"]
        self.alpha = ao.d_alpha(su.period_s, p[f"rate_d_filter_tau_{name}"])[0]
        self.j = p[ao.INERTIA[name]] * (1 + s * su.j_band)
        self.tau = p["motor_tau"] * (1 + t * su.tau_band)
        self.t = su.period_s
        h = su.tick_s
        self.h = h
        self.e = math.exp(-h / self.tau)
        self.em = -math.expm1(-h / self.tau)
        self.g = h - self.tau * self.em
        self.q = h * h / 2 - self.tau * self.g
        self.coeffs = chain.coeffs
        self.hist = [[0.0, 0.0, 0.0, 0.0] for _ in chain.stages]
        self.delay = [0.0] * chain.latency
        self.m = self.w = self.th = self.i_int = self.ep = self.yp = self.df = 0.0

    def attitude_step(self, r, d, quant=None):
        """The oracle's Member.attitude_step: one attitude period of N rate executions with the held reference r and a
        torque disturbance d."""
        m, w, th, i_int, ep, yp, df = self.m, self.w, self.th, self.i_int, self.ep, self.yp, self.df
        kp, ki, kd, alpha, tau, j, e_, em, g, q2, t, h = (self.kp, self.ki, self.kd, self.alpha, self.tau, self.j, self.e,
                                                          self.em, self.g, self.q, self.t, self.h)
        coeffs, hist, delay = self.coeffs, self.hist, self.delay
        for _ in range(self.su.ratio):
            for k in range(self.su.divisor):
                if delay:
                    delay.append(w)
                    v = delay.pop(0)
                else:
                    v = w
                for (b0, b1, b2, a1, a2), hs in zip(coeffs, hist):
                    y = b0 * v + b1 * hs[0] + b2 * hs[1] - a1 * hs[2] - a2 * hs[3]
                    hs[1], hs[0], hs[3], hs[2] = hs[0], v, hs[2], y
                    v = y
                y = v
                if k == 0:
                    i_int += ki * t * ep
                    e = r - y
                    d_raw = -kd * (y - yp) / t
                    df = df + alpha * (d_raw - df) if alpha != 1.0 else d_raw
                    u = kp * e + i_int + df
                    if quant is not None:
                        u = quant(u)
                    u = u + d
                    ep, yp = e, y
                th, w, m = th + h * w + (tau * g * m + q2 * u) / j, w + (tau * em * m + g * u) / j, e_ * m + em * u
        self.m, self.w, self.th, self.i_int, self.ep, self.yp, self.df = m, w, th, i_int, ep, yp, df


def tilt_member_run(su, axis, s, t, h_seg, chain, quant=None, om=None):
    """The oracle's tilt_member_run on Member4e."""
    cfg = su.cfg
    mem = Member4e(su, axis, s, t, chain)
    a_r = 1 + h_seg
    out = []
    sin = math.sin
    k2 = 2 * cfg.kp
    bound = cfg.rate_max[axis]
    tilt = cfg.tilt_max
    for a in range(su.attitude_executions(h_seg)):
        out.append(mem.th)
        if om is not None:
            om.append(mem.w)
        sp = tilt if 1 <= a < a_r else 0.0
        r = k2 * sin((sp - mem.th) / 2)
        if abs(r) > bound:
            raise ValueError(f"the design-model rate setpoint {r!r} reaches the clamp {bound!r}")
        mem.attitude_step(r, 0.0, quant)
    return out


def yaw_member_run(su, s, t, h_seg, stick, d_amp, chain, quant=None):
    """The oracle's yaw_member_run on Member4e."""
    cfg = su.cfg
    mem = Member4e(su, 2, s, t, chain)
    a_r = 1 + h_seg
    n_att = su.attitude_executions(h_seg)
    om, th = [], []
    phase = "yawrate"
    sigma = omega_r = psi_l = 0.0
    a_l = a_fb = None
    min_sig = math.inf
    lock_by = None
    kw = cfg.k_yaw.v
    for a in range(n_att):
        w_a, th_a = mem.w, mem.th
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
            r = 0.0 if phase == "braking" else kw * (2 * math.sin(cfg.w * ao.wrap(psi_l - th_a) / 2))
        mem.attitude_step(r, d, quant)
    return {"om": om, "th": th, "a_l": a_l, "a_fb": a_fb, "min_sigma": min_sig, "lock_by": lock_by, "a_r": a_r}


def _tilt_job(args):
    i, j, s, t, with_q = args
    x = _CTX
    om = []
    th = tilt_member_run(x["su"], x["axis"], s, t, x["h"], x["chain"], om=om)
    q = None
    if with_q:
        quant = ao.Quantiser(x["qfix"], x["axis"])
        om_q = []
        th_q = tilt_member_run(x["su"], x["axis"], s, t, x["h"], x["chain"], quant, om_q)
        q = {"theta": _max_diff_at(th, th_q), "omega": _max_diff_at(om, om_q), "saturated": quant.saturated}
    return i, j, s, t, th, om, q


def _max_diff_at(u, v):
    best, at = 0.0, 0
    for k, (a, b) in enumerate(zip(u, v)):
        if a is None or b is None:
            continue
        if abs(b - a) > best:
            best, at = abs(b - a), k
    return best, at


def _is_corner(s, t):
    return (abs(s) == 1.0 and abs(t) == 1.0) or (s == 0.0 and t == 0.0)


class Env:
    """The oracle's Envelope over the 17 x 17 grid and its 9 x 9 subset (holes allowed)."""

    def __init__(self, n):
        self.e = ao.Envelope(n)

    def add(self, series, coarse):
        self.e.add(series, coarse)

    @property
    def lo(self):
        return self.e.lo

    @property
    def hi(self):
        return self.e.hi

    def halving(self):
        return self.e.halving_change()


def q_reduce(per_member, channels):
    """The oracle's q_script rule over [(s, t, {channel: (value, at)})] of the 9 x 9 members: q_corners over the corners and
    the nominal, q_grid over the 9 x 9, q = q_grid where it exceeds the corners, else q_corners."""
    out = {}
    for ch in channels:
        best_c = best_g = (0.0, None)
        for s, t, per in per_member:
            v = per[ch][0]
            if v > best_g[0] or best_g[1] is None:
                best_g = (v, (s, t, per[ch][1]))
            if _is_corner(s, t) and (v > best_c[0] or best_c[1] is None):
                best_c = (v, (s, t, per[ch][1]))
        grid_bound = best_g[0] > best_c[0]
        out[ch] = {"q": (best_g if grid_bound else best_c)[0], "rule": "grid" if grid_bound else "corners",
                   "q_corners": best_c[0], "q_grid": best_g[0], "at": (best_g if grid_bound else best_c)[1]}
    return out


def tilt_envelope(su, axis, h_seg, chain, qfix, procs=1):
    """theta and omega envelopes (Env), the 9 x 9 Q (the oracle's q_script rule) and the quantiser's saturated count of a
    tilt script on `chain`."""
    _CTX.clear()
    _CTX.update(su=su, axis=axis, h=h_seg, chain=chain, qfix=qfix)
    tasks = [(i, j, s, t, i % 2 == 0 and j % 2 == 0) for i, j, s, t in members(GRID)]
    n = su.attitude_executions(h_seg)
    env = {"theta": Env(n), "omega": Env(n)}
    per_q, saturated = [], 0
    for i, j, s, t, th, om, q in _pool_map(_tilt_job, tasks, procs):
        coarse = i % 2 == 0 and j % 2 == 0
        env["theta"].add(th, coarse)
        env["omega"].add(om, coarse)
        if q is not None:
            per_q.append((s, t, {"theta": q["theta"], "omega": q["omega"]}))
            saturated += q["saturated"]
    return env, q_reduce(per_q, ("theta", "omega")), saturated


def _yaw_series(run):
    a_r, n = run["a_r"], len(run["th"])
    th = run["th"]
    rel = [None] * a_r + [th[a] - th[a_r] for a in range(a_r, n)]
    a_l = run["a_l"]
    lock = [None] * a_l + [th[a] - th[a_l] for a in range(a_l, n)] if a_l is not None else [None] * n
    return {"omega": run["om"], "heading_release": rel, "heading_lock": lock}


def _yaw_job(args):
    i, j, s, t, with_q = args
    x = _CTX
    run = yaw_member_run(x["su"], s, t, x["h"], x["stick"], x["d"], x["chain"])
    q = None
    if with_q:
        quant = ao.Quantiser(x["qfix"], 2)
        rq = yaw_member_run(x["su"], s, t, x["h"], x["stick"], x["d"], x["chain"], quant)
        a, b = _yaw_series(run), _yaw_series(rq)
        q = {ch: _max_diff_at(a[ch], b[ch]) for ch in a}
        q["saturated"] = quant.saturated
    return i, j, s, t, run, q


YAW_CHANNELS = ("omega", "heading_release", "heading_lock")


def yaw_envelopes(su, h_seg, stick, d_amp, chain, qfix=None, procs=1):
    """(Env per channel, info, Q or None, saturated) of the yaw script on `chain` (the oracle's yaw_envelopes and q_script;
    Q only when qfix is given)."""
    _CTX.clear()
    _CTX.update(su=su, h=h_seg, stick=stick, d=d_amp, chain=chain, qfix=qfix)
    tasks = [(i, j, s, t, qfix is not None and i % 2 == 0 and j % 2 == 0) for i, j, s, t in members(GRID)]
    n_att = su.attitude_executions(h_seg)
    env = {ch: Env(n_att) for ch in YAW_CHANNELS}
    locks, fbs, locks_c = [], [], []
    min_sig = math.inf
    by = set()
    per_q, saturated = [], 0
    a_r = 1 + h_seg
    for i, j, s, t, run, q in _pool_map(_yaw_job, tasks, procs):
        coarse = i % 2 == 0 and j % 2 == 0
        if run["a_l"] is None:
            raise ValueError("a yaw-release member never locks within the run")
        for ch, series in _yaw_series(run).items():
            env[ch].add(series, coarse)
        locks.append(run["a_l"])
        fbs.append(run["a_fb"])
        if coarse:
            locks_c.append(run["a_l"])
        min_sig = min(min_sig, run["min_sigma"])
        by.add(run["lock_by"])
        if q is not None:
            per_q.append((s, t, {ch: q[ch] for ch in YAW_CHANNELS}))
            saturated += q["saturated"]
    info = {"lock_range": (min(locks), max(locks)), "lock_range_coarse": (min(locks_c), max(locks_c)),
            "fallback_range": (min(f for f in fbs if f is not None), max(f for f in fbs if f is not None))
            if any(f is not None for f in fbs) else None, "min_sigma": min_sig, "lock_by": sorted(by), "a_r": a_r}
    return env, info, (q_reduce(per_q, YAW_CHANNELS) if qfix is not None else None), saturated


# ---- L5: the T4e golden with its rounding bookkeeping (kinematics term and TOL) ------------------------------------

def l5_stage_nodes(chain):
    """The L5 rounding node of each stage: the low-pass is the L5 oracle's lpf, a notch its own."""
    return tuple("lpf" if s.node == LOWPASS else s.node for s in chain.stages)


def l5_coef_node(stage_node):
    return "lpf_coef" if stage_node == "lpf" else f"{stage_node}_coef"


def l5_nodes(chain):
    """(injection nodes, tick nodes, coefficient nodes) of the L5 rounding rule on `chain` (the oracle's, one stage node
    and one coefficient node per stage)."""
    stage = l5_stage_nodes(chain)
    tick = ("gyro", *stage)
    return ("ref", *tick, *ao.RATE_NODES), tick, (*(l5_coef_node(n) for n in stage), "alpha")


def simulate4e(su, name, h_seg, chain, m_sub=ao.KIN_SUBSTEPS):
    """The L5 oracle's simulate (the golden run and its rounding bookkeeping) on `chain`: per stage the nine roundings of
    its direct-form-I step at its output (rho_seq[stage node]) and the signals its float coefficients multiply (the
    low-pass: x + 2 x1 + x2, -y1, -y2 with the oracle's bounds su.lpf_err; a notch: x + x2 for b0 = b2, x1 - y1 for
    b1 = a1, -y2 for a2, with notch_f32_error's bounds); the gyro node is the cast of the sample after the latency."""
    p, cfg = su.p, su.cfg
    axis = ao.scenario_axis(name)
    n_att = su.attitude_executions(h_seg)
    a_s, a_r = 1, 1 + h_seg
    plants = [ao.Axis(p[ao.INERTIA[a]], p["motor_tau"], su.tick_s / m_sub) for a in ao.AXES]
    q = (1.0, 0.0, 0.0, 0.0)
    angle = ao.AngleMode(cfg)
    r_hold = (0.0, 0.0, 0.0)
    kp = [p[f"rate_kp_{a}"] for a in ao.AXES]
    ki = [p[f"rate_ki_{a}"] for a in ao.AXES]
    kd = [p[f"rate_kd_{a}"] for a in ao.AXES]
    coeffs = chain.coeffs
    snodes = l5_stage_nodes(chain)
    nodes, tick_nodes, coef_nodes = l5_nodes(chain)
    lowpass = [n == "lpf" for n in snodes]
    hist = [[[0.0, 0.0, 0.0, 0.0] for _ in coeffs] for _ in range(3)]
    delay = [[0.0] * chain.latency for _ in range(3)]
    y_tick = [0.0] * 3
    integral, e_prev, y_prev, d_f, u = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3
    uu = U
    rho = dict.fromkeys(nodes + coef_nodes, 0.0)
    ticks = n_att * su.divisor * su.ratio
    rho_seq = {node: [0.0] * (ticks if node in tick_nodes else (n_att if node == "ref" else n_att * su.ratio))
               for node in nodes}
    coef_seq = [([0.0] * ticks, [0.0] * ticks, [0.0] * ticks) for _ in coeffs]
    gap_seq, dt_seq = [0.0] * (n_att * su.ratio), [0.0] * (n_att * su.ratio)
    cross_axes = 0.0
    th_out, om_out = [], []
    err_max = 0.0
    lock_a = None
    lock_info = None
    for j in range(ticks):
        if j % (su.divisor * su.ratio) == 0:
            a = j // (su.divisor * su.ratio)
            t_us = su.stamp_us(j)
            qe = tuple(ao.E(c, uu * abs(c)) for c in q)
            om = tuple(ao.E(pl.w, uu * abs(pl.w)) for pl in plants)
            sticks = ao.script_sticks(name, a, a_s, a_r)
            q_sp, cmd = angle.execute(sticks, t_us, qe, om)
            r = ao.attitude_law(cfg, qe, q_sp, cmd)
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
                err_max = max(err_max, abs(ao.wrap(angle.psi_lock.v - plants[2].th)))
            elif name != "yaw_release":
                err_max = max(err_max, abs(sticks[axis] * cfg.tilt_max - plants[axis].th))
        for i, pl in enumerate(plants):
            dl = delay[i]
            if dl:
                dl.append(pl.w)
                x = dl.pop(0)
            else:
                x = pl.w
            hs_axis = hist[i]
            if j == 0:
                for hs in hs_axis:
                    hs[0] = hs[1] = hs[2] = hs[3] = x
            if i == axis:
                v_g = uu * abs(x)
                rho["gyro"] = max(rho["gyro"], v_g)
                rho_seq["gyro"][j] = v_g
            for k, ((b0, b1, b2, a1, a2), hs) in enumerate(zip(coeffs, hs_axis)):
                x1, x2, y1, y2 = hs
                p0, p1 = b0 * x, b1 * x1
                s1 = p0 + p1
                p2 = b2 * x2
                s2 = s1 + p2
                p3 = a1 * y1
                s3 = s2 - p3
                p4 = a2 * y2
                y = s3 - p4
                if i == axis:
                    v_f = uu * (abs(p0) + abs(p1) + abs(s1) + abs(p2) + abs(s2) + abs(p3) + abs(s3) + abs(p4) + abs(y))
                    node = snodes[k]
                    rho[node] = max(rho[node], v_f)
                    rho_seq[node][j] = v_f
                    cs = coef_seq[k]
                    if lowpass[k]:
                        cs[0][j], cs[1][j], cs[2][j] = x + 2 * x1 + x2, -y1, -y2
                    else:
                        cs[0][j], cs[1][j], cs[2][j] = x + x2, x1 - y1, -y2
                hs[0], hs[1], hs[2], hs[3] = x, x1, y, y1
                x = y
            y_tick[i] = x
        if j % su.divisor == 0:
            n = j // su.divisor
            if n == 0:
                integral, e_prev, u, d_f = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3
                y_prev = list(y_tick)
            else:
                dt = su.dt_exec(n)
                for i in range(3):
                    alpha, _ = su.alpha(dt, i)
                    inc = ki[i] * e_prev[i] * dt
                    integral[i] = integral[i] + inc
                    e_i = r_hold[i] - y_tick[i]
                    d_raw = -kd[i] * (y_tick[i] - y_prev[i]) / dt
                    gap = d_raw - d_f[i]
                    d_new = d_f[i] + alpha * gap if alpha != 1.0 else d_raw
                    pe = kp[i] * e_i
                    pi_sum = pe + integral[i]
                    u[i] = pi_sum + d_new
                    if i == axis:
                        filtered = alpha * (4 * uu * abs(d_raw) + 2 * uu * abs(gap)) + uu * abs(d_new)
                        if alpha != 1.0:
                            gap_seq[n], dt_seq[n] = gap, dt
                        step_rho = {"e": uu * abs(e_i),
                                    "I": 3 * uu * abs(inc) + uu * abs(integral[i]),
                                    "D": filtered if alpha != 1.0 else 4 * uu * abs(d_raw),
                                    "u": uu * (abs(pe) + abs(pi_sum) + abs(u[i]))}
                        for node, v in step_rho.items():
                            rho[node] = max(rho[node], v)
                            rho_seq[node][n] = v
                    d_f[i] = d_new
                    e_prev[i] = e_i
                    y_prev[i] = y_tick[i]
        for _ in range(m_sub):
            d = [pl.step(u[i]) for i, pl in enumerate(plants)]
            q = ao.qmul(q, ao.quat_exp(d))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    coef = []
    for k, st in enumerate(chain.stages):
        node = snodes[k]
        if lowpass[k]:
            deltas = (su.lpf_err[0], su.lpf_err[3], su.lpf_err[4])
        else:
            deltas = (st.err[0], st.err[1], st.err[4])
        coef += [(l5_coef_node(node), node, coef_seq[k][c], deltas[c]) for c in range(3)]
    dts = sorted({dt for dt in dt_seq if dt > 0.0})
    coef += [("alpha", "D", [g if dt == v else 0.0 for g, dt in zip(gap_seq, dt_seq)], su.alpha(v, axis)[1])
             for v in dts]
    for node in coef_nodes:
        rho[node] = max([c[3] for c in coef if c[0] == node], default=0.0)
    return {"th": th_out, "om": om_out, "rho": rho, "rho_seq": rho_seq, "coef": coef, "cross_axes": cross_axes,
            "err_max": err_max, "lock_a": lock_a, "lock_info": lock_info, "a_r": a_r, "n_att": n_att,
            "angle_phase": angle.phase}


def kinematics_change(su, name, h_seg, chain, run=None):
    """(theta, omega): the oracle's kinematics term on `chain`, the golden at 1 and at 2 sub-steps per tick."""
    run = run or simulate4e(su, name, h_seg, chain)
    fine = simulate4e(su, name, h_seg, chain, 2 * ao.KIN_SUBSTEPS)
    return ao.max_diff(run["th"], fine["th"]), ao.max_diff(run["om"], fine["om"])


# ---- L5: the recoveries on the T4e chain --------------------------------------------------------------------------

def recovery_run(su, s, t, q0, w0, n_exec, chain, quant=None, m_sub=1, fn=None):
    """recovery_model.member_run with `chain` (before tick 0 the latency line holds the initial rate, imu_model.hpp)."""
    p, cfg = su.p, su.cfg
    n_ticks = su.divisor * su.ratio
    heading = 0.0 if (q0[0] == 0 and q0[3] == 0) else 2.0 * math.atan2(q0[3], q0[0])
    q_sp = (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))
    plants = []
    for i, name in enumerate(ao.AXES):
        pl = ao.Axis(p[ao.INERTIA[name]] * (1 + s * su.j_band), p["motor_tau"] * (1 + t * su.tau_band), su.tick_s / m_sub)
        pl.w = w0[i]
        plants.append(pl)
    kp = [p[f"rate_kp_{a}"] for a in ao.AXES]
    ki = [p[f"rate_ki_{a}"] for a in ao.AXES]
    kd = [p[f"rate_kd_{a}"] for a in ao.AXES]
    coeffs = chain.coeffs
    hist = [[[0.0, 0.0, 0.0, 0.0] for _ in coeffs] for _ in range(3)]
    delay = [[w0[i]] * chain.latency for i in range(3)]
    y_tick = [0.0] * 3
    integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    y_prev, d_f = [0.0] * 3, [0.0] * 3
    q = tuple(q0)
    out = []
    r_hold = [0.0] * 3
    for j in range(n_exec * n_ticks):
        for i, pl in enumerate(plants):
            dl = delay[i]
            if dl:
                dl.append(pl.w)
                x = dl.pop(0)
            else:
                x = pl.w
            hs_axis = hist[i]
            if j == 0:
                for hs in hs_axis:
                    hs[0] = hs[1] = hs[2] = hs[3] = x
            for (b0, b1, b2, a1, a2), hs in zip(coeffs, hs_axis):
                y = b0 * x + b1 * hs[0] + b2 * hs[1] - a1 * hs[2] - a2 * hs[3]
                hs[1], hs[0], hs[3], hs[2] = hs[0], x, hs[2], y
                x = y
            y_tick[i] = x
        if j % n_ticks == 0:
            a = j // n_ticks
            out.append((fn or rm.channels)(q, [pl.w for pl in plants]))
            r_hold = rm.law(cfg, q, q_sp)
            if a == 0:
                u = [0.0] * 3
                y_prev = list(y_tick)
            else:
                dt = su.dt_exec(a)
                y = list(y_tick)
                req = [0.0] * 3
                for i in range(3):
                    alpha = su.alpha(dt, i)[0]
                    integral[i] += ki[i] * e_prev[i] * dt
                    e = r_hold[i] - y[i]
                    d_raw = -kd[i] * (y[i] - y_prev[i]) / dt
                    d_f[i] = d_f[i] + alpha * (d_raw - d_f[i]) if alpha != 1.0 else d_raw
                    req[i] = kp[i] * e + integral[i] + d_f[i]
                    e_prev[i] = e
                    y_prev[i] = y[i]
                u = quant(req) if quant is not None else req
            if a == 0 and quant is not None:
                u = quant([0.0, 0.0, 0.0])
        for _ in range(m_sub):
            d = [pl.step(u[i]) for i, pl in enumerate(plants)]
            q = ao.qmul(q, ao.quat_exp(d))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return out, (quant.saturated if quant is not None else 0)


def _rec_job(args):
    i, j, s, t, with_q = args
    x = _CTX
    su, q0, w0, n, chain, fn = x["su"], x["q0"], x["w0"], x["n"], x["chain"], x["fn"]
    base, _ = recovery_run(su, s, t, q0, w0, n, chain, fn=fn)
    quantised = saturated = None
    if with_q:
        qr, saturated = recovery_run(su, s, t, q0, w0, n, chain, rm.Quant3(x["qfix"]), fn=fn)
        quantised = [max(abs(a - b) for a, b in zip(u_, v_)) for u_, v_ in zip(zip(*base), zip(*qr))]
    kin = None
    if _is_corner(s, t):
        fine, _ = recovery_run(su, s, t, q0, w0, n, chain, m_sub=2, fn=fn)
        kin = [max(abs(a[c] - b[c]) for a, b in zip(base, fine)) for c in range(len(base[0]))]
    return i, j, s, t, base, quantised, saturated, kin


def recovery_envelope(su, qfix, q0, w0, n_exec, chain, procs=1, fn=None):
    """recovery_model.envelope on `chain`: lo, hi, halving, kin, q, q_corners, saturated per channel."""
    _CTX.clear()
    _CTX.update(su=su, q0=tuple(q0), w0=tuple(w0), n=n_exec, qfix=qfix, chain=chain, fn=fn)
    tasks = []
    for i, j, s, t in members(GRID):
        coarse = i % 2 == 0 and j % 2 == 0
        tasks.append((i, j, s, t, coarse or _is_corner(s, t)))
    nch = len((fn or rm.channels)(q0, w0))
    envs = [Env(n_exec) for _ in range(nch)]
    q_best, q_corner, kin = [0.0] * nch, [0.0] * nch, [0.0] * nch
    saturated = 0
    for i, j, s, t, base, quantised, sat, k in _pool_map(_rec_job, tasks, procs):
        coarse = i % 2 == 0 and j % 2 == 0
        cols = list(zip(*base))
        for c in range(nch):
            envs[c].add(cols[c], coarse)
        if quantised is not None:
            for c in range(nch):
                q_best[c] = max(q_best[c], quantised[c])
                if _is_corner(s, t):
                    q_corner[c] = max(q_corner[c], quantised[c])
            saturated += sat
        if k is not None:
            kin = [max(a, b) for a, b in zip(kin, k)]
    return {"env": envs, "halving": [e.halving() for e in envs], "kin": kin, "q": q_best, "q_corners": q_corner,
            "saturated": saturated}


# ---- L5: the T4e linear model of the noise term ---------------------------------------------------------------------

def l5_state_size(chain):
    """The error state of L5Lin: [m, w, theta, I, e_prev, y_prev, Df], then per stage [x1, x2, y1, y2], then the latency
    line (the oracle's STATE layout for one stage and no latency)."""
    return 7 + 4 * len(chain.stages) + chain.latency


class L5Lin:
    """The L5 oracle's Lin (the linearised closed loop of one axis at tick resolution, the f32 gains, the stamp dt of the
    integrator and of the D path) at band member (s, t), with `chain` in place of the low-pass; state as l5_state_size.
    With `seed` the firmware's first sample is followed (module docstring); without it, on the T4c chain, the run is the
    oracle's Lin.run (asserted bit for bit)."""

    def __init__(self, su, axis, chain, s=0.0, t=0.0):
        p = su.p
        name = ao.AXES[axis]
        self.su, self.axis, self.chain = su, axis, chain
        self.kp, self.ki, self.kd = p[f"rate_kp_{name}"], p[f"rate_ki_{name}"], p[f"rate_kd_{name}"]
        self.j = p[ao.INERTIA[name]] * (1 + s * su.j_band)
        self.tau = p["motor_tau"] * (1 + t * su.tau_band)
        h = su.tick_s
        self.h = h
        self.e = math.exp(-h / self.tau)
        self.em = -math.expm1(-h / self.tau)
        self.g = h - self.tau * self.em
        self.q = h * h / 2 - self.tau * self.g
        self.stage_nodes = l5_stage_nodes(chain)
        self.size = l5_state_size(chain)

    def run(self, t_start, n_boundaries, state, psi, fb, kind=None, at=None, drive=None, lock=None, seed=False,
            record=None):
        """The oracle's Lin.run: ticks t_start ..; a unit injection of `kind` at `at` (the boundary for ref, the tick for
        gyro and a stage node, the rate execution for e, I, D, u); drive = (stage node or "D", sequence) a signed
        injection at that stage's output every tick or into Df every rate execution; lock = (boundary, feedback gain).
        Returns the observations (theta, w) at the boundaries ceil(t_start / D N) .. n_boundaries - 1 and, with record (a
        set of observation indices), {index: the state there} (else None)."""
        su = self.su
        div, per_att = su.divisor, su.divisor * su.ratio
        coeffs = self.chain.coeffs
        ns = len(coeffs)
        last = ns - 1
        m, w, th, i_int, ep, yp, df = state[:7]
        hist = [list(state[7 + 4 * k:11 + 4 * k]) for k in range(ns)]
        delay = list(state[7 + 4 * ns:])
        kp, ki, kd, tau, j, e_, em, g, q2, h = (self.kp, self.ki, self.kd, self.tau, self.j, self.e, self.em, self.g,
                                                self.q, self.h)
        inj_gyro = at if kind == "gyro" else -1
        inj_stage = self.stage_nodes.index(kind) if kind in self.stage_nodes else -1
        inj_stage_t = at if inj_stage >= 0 else -1
        inj_rate = at if kind in ao.RATE_NODES else -1
        drive_stage = self.stage_nodes.index(drive[0]) if drive is not None and drive[0] in self.stage_nodes else -1
        drive_y = drive[1] if drive_stage >= 0 else None
        drive_d = drive[1] if drive is not None and drive[0] == "D" else None
        states = {} if record is not None else None
        obs = []
        r = 0.0
        u = 0.0
        for t in range(t_start, n_boundaries * per_att):
            if t % per_att == 0:
                b = t // per_att
                if lock is not None and b == lock[0]:
                    psi, fb = th, lock[1]
                if record is not None and len(obs) in record:
                    st = [m, w, th, i_int, ep, yp, df]
                    for hs in hist:
                        st.extend(hs)
                    st.extend(delay)
                    states[len(obs)] = st
                obs.append((th, w))
                r = fb * (psi - th) + (1.0 if (kind == "ref" and b == at) else 0.0)
            if delay:
                delay.append(w)
                v = delay.pop(0)
            else:
                v = w
            x = v + (1.0 if t == inj_gyro else 0.0)
            if seed and t == 0:
                for hs in hist:
                    hs[0] = hs[1] = hs[2] = hs[3] = x
            for k, ((b0, b1, b2, a1, a2), hs) in enumerate(zip(coeffs, hist)):
                y = b0 * x + b1 * hs[0] + b2 * hs[1] - a1 * hs[2] - a2 * hs[3]
                if k == inj_stage or k == last:
                    y = y + (1.0 if (k == inj_stage and t == inj_stage_t) else 0.0)
                if k == drive_stage:
                    y += drive_y[t]
                hs[1], hs[0], hs[3], hs[2] = hs[0], x, hs[2], y
                x = y
            y = x
            if t % div == 0:
                n = t // div
                if seed and n == 0:
                    i_int, df, u, ep, yp = 0.0, 0.0, 0.0, 0.0, y
                else:
                    inj = 1.0 if n == inj_rate else 0.0
                    if n > 0:
                        dt = su.dt_exec(n)
                        alpha = su.alpha(dt, self.axis)[0]
                        i_int = i_int + ki * dt * ep
                        d_raw = -kd * (y - yp) / dt
                        df = df + alpha * (d_raw - df) if alpha != 1.0 else d_raw
                    if kind == "I":
                        i_int += inj
                    e = r - y + (inj if kind == "e" else 0.0)
                    if kind == "D":
                        df += inj
                    if drive_d is not None:
                        df += drive_d[n]
                    u = kp * e + i_int + df + (inj if kind == "u" else 0.0)
                    ep = e
                    yp = y
            th = th + h * w + (tau * g * m + q2 * u) / j
            w = w + (tau * em * m + g * u) / j
            m = e_ * m + em * u
        return obs, states


def l5_period_ticks(su):
    """P: the L5 linear loop is periodic in P ticks after its seed (the attitude boundary every D N ticks, the stamp dt
    alternation of l6_noise_term.period_ticks)."""
    d, num, den = su.divisor, su.num, su.den
    stamp_period = d * (den // math.gcd(d * num, den))
    return math.lcm(d * su.ratio, stamp_period)


def l5_fb_ends(su, axis):
    """The attitude law's Jacobian at the ends of its range (module docstring): (kp or k_yaw w, 0)."""
    cfg = su.cfg
    return ((cfg.k_yaw.v * cfg.w) if axis == 2 else cfg.kp, 0.0)


def l5_unit_sums(su, axis, chain, s, t, fb, n_obs):
    """{"theta": (Var_white, Var_rw), "omega": (...)} per observation 0 .. n_obs - 1 per unit variance (l6_noise_term.unit_sums
    on L5Lin): the seed run (the sample at tick 0) and one run per phase class of the loop's period."""
    lin = L5Lin(su, axis, chain, s, t)
    period = l5_period_ticks(su)
    per_obs = su.divisor * su.ratio
    base = period
    points = list(range(n_obs))
    n_run = (base + period - 1 + per_obs * max(points)) // per_obs + 1
    zero = [0.0] * lin.size
    seed = lin.run(0, n_run, zero, 0.0, fb, kind="gyro", at=0, seed=True)[0]
    runs = [lin.run(0, n_run, zero, 0.0, fb, kind="gyro", at=base + c, seed=True)[0] for c in range(period)]
    out = {}
    for ch, k in (("theta", 0), ("omega", 1)):
        out[ch] = nt.unit_sums([o[k] for o in seed], [[o[k] for o in r] for r in runs], base, period, per_obs, points)
    return out


def _l5_noise_task(args):
    axis, fb_index, s, t = args
    x = _CTX
    su, chain, noise, n_obs = x["su"], x["chain"], x["noise"], x["n_obs"]
    fb = l5_fb_ends(su, axis)[fb_index]
    sums = l5_unit_sums(su, axis, chain, s, t, fb, n_obs)
    return args, {ch: [noise.white_var * a + noise.rw_var * b for a, b in zip(vw, vr)] for ch, (vw, vr) in sums.items()}


def l5_sigma_series(su, chain, noise, n_obs, grid=GRID, procs=1):
    """{axis: {"theta": dict(var, sigma, member, fb, by_end), "omega": ...}} per observation 0 .. n_obs - 1: the largest
    variance over the grid members and the two frozen Jacobian ends at each observation; by_end[k] the largest over the
    members at end k alone (l5_fb_ends order)."""
    _CTX.clear()
    _CTX.update(su=su, chain=chain, noise=noise, n_obs=n_obs)
    vals = nt.grid_values(grid)
    tasks = [(a, k, s, t) for a in range(3) for k in range(2) for s in vals for t in vals]
    out = {a: {} for a in range(3)}
    for (a, k, s, t), per in _pool_map(_l5_noise_task, tasks, procs):
        fb = l5_fb_ends(su, a)[k]
        for ch, var in per.items():
            o = out[a].get(ch)
            if o is None:
                o = out[a][ch] = {"var": list(var), "member": [(s, t)] * len(var), "fb": [fb] * len(var),
                                  "by_end": [None, None]}
            else:
                for i, v in enumerate(var):
                    if v > o["var"][i]:
                        o["var"][i], o["member"][i], o["fb"][i] = v, (s, t), fb
            e = o["by_end"]
            e[k] = list(var) if e[k] is None else [max(x, y) for x, y in zip(e[k], var)]
    for per in out.values():
        for o in per.values():
            o["sigma"] = [math.sqrt(v) for v in o["var"]]
    return out


def l5_end_view(series, k):
    """The series at Jacobian end k alone, in l5_sigma_series's shape (sigma only)."""
    return {a: {ch: {"sigma": [math.sqrt(v) for v in o["by_end"][k]]} for ch, o in per.items()}
            for a, per in series.items()}


# ---- L5: TOL on the T4e chain (the L5 oracle's first-order rule) ------------------------------------------------

def l5_phases(su, node, tick_nodes):
    """The oracle's phases with the T4e tick nodes."""
    L = math.lcm(su.ratio, 2)
    if node == "ref":
        return L // su.ratio, L // su.ratio
    return (L * su.divisor if node in tick_nodes else L), L // su.ratio


def l5_injection_start(su, node, base_b, p, tick_nodes):
    """The oracle's injection_start with the T4e tick nodes."""
    if node == "ref":
        return base_b + p, p
    if node in tick_nodes:
        per_att = su.divisor * su.ratio
        return base_b * per_att + p, -(-p // per_att)
    return base_b * su.ratio + p, -(-p // su.ratio)


def l5_injection_rho(run, node, base_b, p, su, tick_nodes):
    n_ph, _ = l5_phases(su, node, tick_nodes)
    first, _ = l5_injection_start(su, node, base_b, p, tick_nodes)
    return run["rho_seq"][node][first::n_ph]


def l5_node_rows(su, lin, run, node, fb, n_att, b_first, tick_nodes, lock_b=None):
    """The oracle's node_sums for one node: (rows [(c_p, Ctheta, Comega)], stride, xs). xs (with lock_b) is the oracle's
    state_sums at boundary lock_b, accumulated during the runs instead of from stored states."""
    per_att = su.divisor * su.ratio
    n_ph, s = l5_phases(su, node, tick_nodes)
    rows = []
    xs = [0.0] * lin.size if lock_b is not None else None
    for p in range(n_ph):
        at, c_p = l5_injection_start(su, node, b_first, p, tick_nodes)
        shift = n_ph if b_first == 0 else 0
        at += shift
        if node == "ref":
            t0 = at * per_att
        elif node in tick_nodes:
            t0 = at
        else:
            t0 = at * su.divisor
        wanted = None
        if lock_b is not None:
            rho = l5_injection_rho(run, node, b_first, p, su, tick_nodes)
            wanted = []
            for q, r in enumerate(rho):
                idx = lock_b - b_first - c_p - s * q
                if idx < 0:
                    break
                wanted.append((idx, r))
        obs, sts = lin.run(t0, n_att + (s if shift else 0), [0.0] * lin.size, 0.0, fb, kind=node, at=at,
                           record=None if wanted is None else {idx for idx, _ in wanted})
        rows.append((c_p, ao.strided_cum([abs(o[0]) for o in obs], s), ao.strided_cum([abs(o[1]) for o in obs], s)))
        if wanted is not None:
            for idx, r in wanted:
                if idx < len(obs):
                    st = sts[idx]
                    for i in range(lin.size):
                        xs[i] += r * abs(st[i])
    return rows, s, xs


def l5_region_bound(su, run, node, rows, s, base_b, m_lo, m_hi, tick_nodes, extra=None):
    """The oracle's region_bound for one node: (theta, omega)."""
    pre = []
    for p, (c_p, ct, cw) in enumerate(rows):
        pre.append((c_p, ct, cw, ao.phase_blocks(l5_injection_rho(run, node, base_b, p, su, tick_nodes))))
    bt = bw = 0.0
    cum_at = ao.cum_at
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
            et, ew = extra(m)
            st, sw = st + et, sw + ew
        bt, bw = max(bt, st), max(bw, sw)
    return bt, bw


def _tol_node_task(args):
    """One node at one feedback gain: the pre-lock or the whole-run region bound (and xs at the lock)."""
    node, fb, b_first, m_lo, m_hi, lock_b, post = args
    x = _CTX
    su, run, chain, n_att, tick_nodes = x["su"], x["run"], x["chain"], x["n_att"], x["tick_nodes"]
    lin = L5Lin(su, x["axis"], chain)
    rows, s, xs = l5_node_rows(su, lin, run, node, fb, n_att, b_first, tick_nodes, lock_b)
    extra = None
    if post:
        g, x_lock, a_l = x["g"], x["xs"][node] + [x["xs"][node][2]], x["a_l"]
        nk = len(g)

        def extra(m):
            d = m - a_l
            return (sum(x_lock[k] * abs(g[k][d][0]) for k in range(nk)),
                    sum(x_lock[k] * abs(g[k][d][1]) for k in range(nk)))
    return node, l5_region_bound(su, run, node, rows, s, b_first, m_lo, m_hi, tick_nodes, extra), xs


def _coef_task(args):
    index, fb, lock = args
    x = _CTX
    node, kind, seq, delta = x["run"]["coef"][index]
    lin = L5Lin(x["su"], x["axis"], x["chain"])
    obs, _ = lin.run(0, x["n_att"], [0.0] * lin.size, 0.0, fb, drive=(kind, seq), lock=lock)
    return node, [(delta * abs(th), delta * abs(w)) for th, w in obs]


def _g_task(comp):
    x = _CTX
    lin = L5Lin(x["su"], x["axis"], x["chain"])
    state = [1.0 if comp == k else 0.0 for k in range(lin.size)]
    obs, _ = lin.run(x["a_l"] * x["su"].divisor * x["su"].ratio, x["n_att"], state, 1.0 if comp == lin.size else 0.0,
                     x["fb"])
    return comp, obs


def _coefficient_bound(su, run, chain, axis, fb, coef_nodes, procs, lock=None):
    _CTX.update(su=su, run=run, chain=chain, axis=axis, n_att=run["n_att"])
    acc = {node: [[0.0, 0.0] for _ in range(run["n_att"])] for node in coef_nodes}
    for node, rows in _pool_map(_coef_task, [(i, fb, lock) for i in range(len(run["coef"]))], procs):
        a = acc[node]
        for m, (th, w) in enumerate(rows):
            a[m][0] += th
            a[m][1] += w
    return {node: (max(r[0] for r in rows), max(r[1] for r in rows)) for node, rows in acc.items()}


def l5_tolerance(su, name, run, chain, procs=1):
    """The L5 oracle's tolerance (bound per node as (theta, omega), and the per-channel tolerances, the sums over the
    nodes) on `chain`, from simulate4e's run: the frozen feedback gains c in [c_min, 1] (C_POINTS of them), the yaw lock's
    regime change composed through the componentwise bound of the error state at the lock."""
    cfg = su.cfg
    axis = ao.scenario_axis(name)
    n_att = run["n_att"]
    nodes, tick_nodes, coef_nodes = l5_nodes(chain)
    for node in nodes:
        k0 = 0
        if run["rho_seq"][node][k0] != 0.0:
            raise ValueError(f"{name}: a rounding bound at the seed execution is not zero ({node})")
    _CTX.clear()
    _CTX.update(su=su, run=run, chain=chain, axis=axis, n_att=n_att, tick_nodes=tick_nodes)

    def maxpair(a, b):
        return {n: (max(a[n][0], b[n][0]), max(a[n][1], b[n][1])) for n in a}

    if name != "yaw_release":
        c_min = math.cos(run["err_max"] / 2)
        bound = None
        for i in range(ao.C_POINTS):
            c = c_min + (1 - c_min) * i / (ao.C_POINTS - 1)
            fb = cfg.kp * c
            tasks = [(node, fb, 0, 0, n_att, None, False) for node in nodes]
            cur = {node: b for node, b, _ in _pool_map(_tol_node_task, tasks, procs)}
            cur.update(_coefficient_bound(su, run, chain, axis, fb, coef_nodes, procs))
            bound = cur if bound is None else maxpair(bound, cur)
    else:
        a_l = run["lock_a"]
        tasks = [(node, 0.0, 0, 0, a_l, a_l, False) for node in nodes]
        bound, xs = {}, {}
        for node, b, x_ in _pool_map(_tol_node_task, tasks, procs):
            bound[node], xs[node] = b, x_
        bound.update(dict.fromkeys(coef_nodes, (0.0, 0.0)))
        c_min = math.cos(cfg.w * run["err_max"] / 2)
        size = l5_state_size(chain)
        for i in range(ao.C_POINTS):
            c = c_min + (1 - c_min) * i / (ao.C_POINTS - 1)
            fb = cfg.k_yaw.v * cfg.w * c
            _CTX.update(fb=fb, a_l=a_l)
            g = dict(_pool_map(_g_task, list(range(size + 1)), procs))
            _CTX.update(g=[g[k] for k in range(size + 1)], xs=xs)
            tasks = [(node, fb, a_l, a_l, n_att, None, True) for node in nodes]
            cur = {node: b for node, b, _ in _pool_map(_tol_node_task, tasks, procs)}
            cur.update(_coefficient_bound(su, run, chain, axis, 0.0, coef_nodes, procs, lock=(a_l, fb)))
            bound = maxpair(bound, cur)
    all_nodes = nodes + coef_nodes
    return bound, (sum(bound[n][0] for n in all_nodes), sum(bound[n][1] for n in all_nodes))
