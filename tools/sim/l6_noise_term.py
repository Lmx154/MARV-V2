#!/usr/bin/env python3
"""L6 stage (e) noise term W: the T4e linear design model and the propagation of the sensor noise model through it
(quad spec L6 pass bar (e), the approved spec line of decision 0019; decision 0019 rulings 3 and 4, second-round rulings
1 and 2; the noise suite design memo of 2026-10-04, section 1, commit C4).

    uv run python tools/sim/l6_noise_term.py --scenario step_roll|step_pitch|step_yaw|acro --points M_s
        [--grid G] [--clock-error e] [--jobs n]

prints, for the scenario, sigma_c and W_c per truth channel. M_s, the number of (execution, channel) points the
scenario's predicate checks, is the caller's: it comes from the predicate's own definition and is never chosen here.

T4e linear model (l6_ff_eval's T4e linear design model, its code path: l6_ff_eval.setup() gives the law, the sensor and
the fixture; axis_run is l6_ff_eval.linear_response operation for operation, so a run without noise equals it bit for
bit). Per grid member of the 17 x 17 tau x J grid (l6_ff_eval.GRID, the oracles' halved grid; J (1 + s b_J),
tau (1 + t b_tau)) and per axis: the sensor (latency_samples, then the chain: every notch at omega_hover and the
low-pass, direct form I), the rate law at the executions (prefilter seeded at the first execution, deferred
forward-Euler I, D on the measurement through its low-pass T_f, u = 0 at the seed execution), the ZOH plant J w' = u_m,
tau u_m' = u - u_m advanced tick by tick by the exact one-tick map (rate_t3_oracle.Setup.tick_map) at the plant's tick
t_nom / (1 + e) (e the IMU's relative clock error, 0 nominal; the firmware's stamps stay nominal). No w x Jw in the
plant (decision 0014's linear design model; decision 0019 ruling 6).
  Feed-forward, linearised (memo section 1): with an operating point w0 the law adds G0 y + tau_m gdot, gdot the
  first-order T_ff low-pass of the stamp difference of G0 y (g_prev := G0 y at the seed execution, gdot := 0 there),
  G0 = d(w x J w)/dw at w0 (rate_lead.gyroscopic_jacobian, J, tau_m and T_ff the PID+FF+lag law's). G0 couples the
  axes, so such a member is run on all three axes together (coupled_run). w0 = 0 gives G0 = 0: the axes decouple and
  the model is the plain T4e linear model.

Noise (decision 0012, sim/plant/src/imu_model.hpp): each gyro axis's sample k is truth(k - L) + b_k + sigma_d v_k,
b_k = b_(k-1) + K sqrt(dt) w_k from sample 0 on, then quantised to the LSB. The noise enters at the measured sample,
after the latency and before the chain; the firmware's first-sample seeding (the chain's seed_first_sample, the
prefilter's r := y, D's y_prev := y, FF's g_prev := G0 y) reads it. White part: sigma_d^2 + LSB^2 / 12 per sample (the
rounding error INFERRED uniform and white, as 0012's Allan model); random walk: K^2 dt per increment; dt the plant's tick.
sigma_d and K come from gen_imu_config.derived_report at that tick, the LSB from gen_imu_config.imu_config. The axes'
streams are independent (0012), so the variances of the three input axes add.

Variance, exact for the linear model (the memo's formula sigma_c^2 = (sigma_d^2 + LSB^2/12) sum_t h_c(t)^2 +
K^2 dt sum_t H_c(t)^2, evaluated at each checked execution with each sample's own response): the loop is periodic in
P ticks after its seed (P = lcm(D, D x the stamp period in executions); the stamps alternate 312 / 313 us, P = 4), so a
sample at tick i >= 1 moves w(T) by G_(i mod P)(T - i), one impulse run per class i mod P; the sample at tick 0 takes
the seed paths, one more run (g_seed). The step responses follow without further runs: a unit step from tick i is the
impulse at i plus the step from i + 1, S_c(m) = G_c(m) + S_(c+1 mod P)(m - 1), and the step from tick 0 is
g_seed(T) + S_1(T - 1). Then, at the execution tick T,
    Var_white(T) = g_seed(T)^2 + sum_(m=0..T-1) G_((T-m) mod P)(m)^2,
    Var_rw(T)    = (g_seed(T) + S_1(T-1))^2 + sum_(m=0..T-1) S_((T-m) mod P)(m)^2,
    sigma^2(T)   = (sigma_d^2 + LSB^2/12) Var_white(T) + K^2 dt Var_rw(T),  summed over the input axes.
Each sample carries its own dt phase, so no maximum over the two phases is needed (rate_t3_oracle.l1_norm takes it
because it adds the phases' norms without that bookkeeping; the exact sum is at most that bound). sigma_series gives
sigma_c(n), the largest sigma(T) over the grid members and the scenario's frozen operating points at each checked
execution n; sigma gives sigma_c, its largest over the checked executions (--at prints sigma_c(n) too).

Frozen operating points (memo section 1, second-round ruling 2): the L4 steps are LTI (w0 = 0 only). Acro is
time-varying through the feed-forward: w0 = 0 and w0 = (+-P_roll, +-P_pitch, +-P_yaw) for every sign pattern
(rate_lead.SIGN_PATTERNS), P_a the T4e band's peak (the largest |w_a| of the T4e linear model on the acro script over
the 17 x 17 grid). That the largest of these bounds the time-varying loop is INFERRED; the nightly Monte Carlo check
(commit C6) verifies it.

Quantile (Luis, second-round ruling 1, Bonferroni inside the run): W_c = z sigma_c,
z = Phi^-1(1 - (1 - c)/(2 N M_s)), c = noise_term_confidence, N = ceil(ln(1 - c_t)/ln p_t) (Wilks 1941) with p_t, c_t the
scenario class's t4_pass_probability_* and t4_confidence_* (all design/budget.yaml). Classes (quad spec 4 L6 (e)):
recoveries and acro coupling are the safety class; steps, chirps and the other scenarios are the tracking class.

Not here: the PM channels of the chirps (memo section 1, through run_l4.float_input_term), the L5 scenarios (their T4e
linear model with the attitude law's frozen Jacobian), the bias delta (commit C5) and the refdata (commit C6).

Inputs (none retyped): l6_ff_eval's (the card, design/budget.yaml, design/scenario_values.yaml, the sensor profile, the
L4 T3 fixture, scenarios/quad/L04/acro.yaml, the l4_rate_scripted register) and scenarios/quad/L04/step_<axis>.yaml.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import multiprocessing
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import l6_ff_eval as fe  # noqa: E402  (puts tools/card and the L4 oracle on sys.path)
import gen_imu_config as gic  # noqa: E402
import rate_lead as rl  # noqa: E402
import run_l4  # noqa: E402
import schema  # noqa: E402

ROOT = fe.ROOT
L4_SCENARIOS = ROOT / "scenarios" / "quad" / "L04"
AXES = fe.AXES
GRID = fe.GRID
MICROSECOND = fe.MICROSECOND
FF_LAW = "PID+FF+lag"  # decision 0014 owner decision 1, the chosen form
# The variance of the uniform distribution on an interval of width L is L^2 / 12 (the rounding error of one LSB).
UNIFORM_VARIANCE_DIVISOR = 12
CLASS_SAFETY, CLASS_TRACKING = "safety", "tracking"
SCENARIOS = {"step_roll": CLASS_TRACKING, "step_pitch": CLASS_TRACKING, "step_yaw": CLASS_TRACKING,
             "acro": CLASS_SAFETY}  # quad spec 4 L6 (e): acro coupling is the safety class, the steps tracking


# ---- the register: N, c and z -----------------------------------------------------------------------------------------

def register(budget=fe.BUDGET):
    return schema.load_yaml(budget)


def seed_count(klass, reg):
    """N = ceil(ln(1 - c)/ln p) of the scenario class (Wilks 1941; quad spec 4 L6 (e))."""
    p = reg[f"t4_pass_probability_{klass}"]["value"]
    c = reg[f"t4_confidence_{klass}"]["value"]
    return math.ceil(math.log(1 - c) / math.log(p))


def z_value(confidence, n_seeds, m_points):
    """Phi^-1(1 - (1 - confidence)/(2 N M_s)), from the lower tail (the upper is 1 - a tiny number)."""
    if n_seeds < 1 or m_points < 1:
        raise ValueError("N and M_s must be >= 1")
    return -statistics.NormalDist().inv_cdf((1 - confidence) / (2 * n_seeds * m_points))


# ---- the noise inputs ---------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Noise:
    """The gyro noise figures per axis at the plant's tick dt (module docstring)."""
    sigma_d: float  # white noise per sample, rad/s
    lsb: float      # rad/s per count
    k_rw: float     # random-walk coefficient K, rad/s / sqrt(s)
    dt: float       # the sample period, s

    @property
    def white_var(self):
        return self.sigma_d * self.sigma_d + self.lsb * self.lsb / UNIFORM_VARIANCE_DIVISOR

    @property
    def rw_var(self):
        return self.k_rw * self.k_rw * self.dt


def noise_inputs(c, e=0.0):
    """Noise of the profile (gen_imu_config's loaders) at the plant tick t_nom / (1 + e) of l6_ff_eval's fixture."""
    p = c["fixture"]
    num, den = p["tick_period_num_us"], p["tick_period_den"]
    si, _, _ = gic.imu_config(fe.PROFILE)
    rep, _ = gic.derived_report(si, num / (1 + e), den)
    _, _, _, k, sigma_d = rep["gyro"]
    return Noise(sigma_d=sigma_d, lsb=si["gyro_lsb"], k_rw=k, dt=plant_tick(c, e))


# ---- the T4e linear model -----------------------------------------------------------------------------------------------

def plant_tick(c, e=0.0):
    return c["su1"].tick_s if e == 0 else c["su1"].tick_s / (1 + e)


def tick_map(c, inertia, tau, e=0.0):
    """(a12, b1, a22, b2) of one plant tick at t_nom / (1 + e) (rate_t3_oracle.Setup.tick_map; e = 0: its own)."""
    su = c["su1"]
    if e != 0:
        moved = object.__new__(type(su))
        moved.__dict__.update(su.__dict__, tick_s=plant_tick(c, e))
        su = moved
    return fe.tick_map(su, inertia, tau)


def law_of(c, a, name="PID"):
    law = c["laws"][name]
    return law.kp[a], law.ki[a], law.kd[a], law.tau_ref[a], law.d_tau[a]


def sensor_of(c):
    """The T4e linear design model's sensor (l6_ff_eval: latency_samples, every notch at omega_hover, the low-pass)."""
    return c["sensors"]["T4e"].linear


def stamps_of(c, n_exec):
    """Stamps (us) of executions 0 .. n_exec - 1: floor(D k num / den), the plans' stamp_us."""
    p = c["fixture"]
    d, num, den = p["rate_loop_divisor"], p["tick_period_num_us"], p["tick_period_den"]
    return [(d * k * num) // den for k in range(n_exec)]


def axis_run(law, sensor, tmap, stamps, divisor, n_exec, sps=None, inject=None):
    """w per execution of one axis (module docstring): l6_ff_eval.linear_response with the sensor's dynamics, plus the
    first-sample seeding of the chain and a unit sample at tick `inject` (after the latency, before the chain)."""
    kp, ki, kd, tau_ref, d_tau = law
    stages = sensor.stages
    a12, b1, a22, b2 = tmap
    chain = [[0.0, 0.0, 0.0, 0.0] for _ in stages]
    delay = [0.0] * sensor.latency
    w = um = r = integral = e_prev = y_prev = d_f = 0.0
    alpha_d = {}
    out = []
    u = 0.0
    t = 0
    for n in range(n_exec):
        sp = sps[n] if sps is not None else 0.0
        out.append(w)
        for k in range(divisor):
            if delay:
                delay.append(w)
                v = delay.pop(0)
            else:
                v = w
            if t == inject:
                v = v + 1.0
            if t == 0:
                for s in chain:
                    s[0] = s[1] = s[2] = s[3] = v
            for (c0, c1, c2, c3, c4), s in zip(stages, chain):
                o = c0 * v + c1 * s[0] + c2 * s[1] - c3 * s[2] - c4 * s[3]
                s[1] = s[0]
                s[0] = v
                s[3] = s[2]
                s[2] = o
                v = o
            yn = v
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
            t += 1
    return out


def coupled_run(laws, ff, sensor, tmaps, stamps, divisor, n_exec, inject_axis, inject):
    """w per execution of the three axes (lists) with the linearised feed-forward ff = (G0, tau_m, T_ff): each axis as
    axis_run, plus u += G0 y + tau_m gdot (module docstring); a unit sample on axis `inject_axis` at tick `inject`."""
    g0, tau_m, t_ff = ff
    stages = sensor.stages
    chains = [[[0.0, 0.0, 0.0, 0.0] for _ in stages] for _ in range(3)]
    delay = [[0.0, 0.0, 0.0] for _ in range(sensor.latency)]
    w, um = [0.0] * 3, [0.0] * 3
    r, integral, e_prev, y_prev, d_f = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3
    g_prev, gdot, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    alpha_c = {}
    out = [[] for _ in range(3)]
    y = [0.0] * 3
    t = 0

    def alpha(dt, tc):
        key = (dt, tc)
        if key not in alpha_c:
            alpha_c[key] = -math.expm1(-dt / tc)
        return alpha_c[key]

    for n in range(n_exec):
        for a in range(3):
            out[a].append(w[a])
        for k in range(divisor):
            if delay:
                delay.append(list(w))
                sample = delay.pop(0)
            else:
                sample = w
            for a in range(3):
                v = sample[a]
                if t == inject and a == inject_axis:
                    v = v + 1.0
                chain = chains[a]
                if t == 0:
                    for s in chain:
                        s[0] = s[1] = s[2] = s[3] = v
                for (c0, c1, c2, c3, c4), s in zip(stages, chain):
                    o = c0 * v + c1 * s[0] + c2 * s[1] - c3 * s[2] - c4 * s[3]
                    s[1] = s[0]
                    s[0] = v
                    s[3] = s[2]
                    s[2] = o
                    v = o
                y[a] = v
            if k == 0:
                g = [g0[a][0] * y[0] + g0[a][1] * y[1] + g0[a][2] * y[2] for a in range(3)]
                if n == 0:
                    for a in range(3):
                        r[a], e_prev[a], integral[a], u[a], y_prev[a], d_f[a] = y[a], 0.0, 0.0, 0.0, y[a], 0.0
                        gdot[a] = 0.0
                else:
                    dt = (stamps[n] - stamps[n - 1]) * MICROSECOND
                    for a in range(3):
                        kp, ki, kd, tau_ref, d_tau = laws[a]
                        r[a] = r[a] + alpha(dt, tau_ref) * (0.0 - r[a])
                        e = r[a] - y[a]
                        integral[a] = integral[a] + ki * e_prev[a] * dt
                        d = -kd * (y[a] - y_prev[a]) / dt
                        if d_tau > 0:
                            d_f[a] = d_f[a] + alpha(dt, d_tau) * (d - d_f[a])
                            d = d_f[a]
                        u[a] = kp * e + integral[a] + d
                        e_prev[a] = e
                        y_prev[a] = y[a]
                        raw = (g[a] - g_prev[a]) / dt
                        gdot[a] = gdot[a] + alpha(dt, t_ff) * (raw - gdot[a]) if t_ff > 0 else raw
                        u[a] = u[a] + (g[a] + tau_m * gdot[a])
                g_prev = g
            for a in range(3):
                a12, b1, a22, b2 = tmaps[a]
                w[a], um[a] = w[a] + a12 * um[a] + b1 * u[a], a22 * um[a] + b2 * u[a]
            t += 1
    return out


# ---- the variance sums --------------------------------------------------------------------------------------------------

def period_ticks(c):
    """P: the loop is periodic in P ticks after its seed (the execution divisor and the stamps' period)."""
    p = c["fixture"]
    d, num, den = p["rate_loop_divisor"], p["tick_period_num_us"], p["tick_period_den"]
    return math.lcm(d, d * (den // math.gcd(d * num, den)))


def unit_sums(seed, runs, base, period, divisor, points):
    """(Var_white, Var_rw) per checked execution of one (input axis, output channel) pair, per unit variance (module
    docstring). seed: the response per execution to the tick-0 sample; runs[c]: to a sample at tick base + c."""
    n_lags = divisor * max(points)
    if n_lags == 0:
        return [0.0] * len(points), [0.0] * len(points)
    g = []
    for cls in range(period):
        rc, i0 = runs[cls], base + cls
        row = [0.0] * n_lags
        for m in range((-i0) % divisor, n_lags, divisor):
            row[m] = rc[(i0 + m) // divisor]
        g.append(row)
    s = [[0.0] * n_lags for _ in range(period)]
    for cls in range(period):
        s[cls][0] = g[cls][0]
    for m in range(1, n_lags):
        for cls in range(period):
            s[cls][m] = g[cls][m] + s[(cls + 1) % period][m - 1]
    cum_w, cum_r = {}, {}
    for al in range(0, period, divisor):
        aw, ar, tw, tr = [0.0] * n_lags, [0.0] * n_lags, 0.0, 0.0
        for m in range(n_lags):
            cls = (al - m) % period
            tw += g[cls][m] * g[cls][m]
            tr += s[cls][m] * s[cls][m]
            aw[m], ar[m] = tw, tr
        cum_w[al], cum_r[al] = aw, ar
    vw, vr = [], []
    first = 1 % period
    for n in points:
        tick = divisor * n
        if tick == 0:
            vw.append(seed[0] * seed[0])
            vr.append(seed[0] * seed[0])
            continue
        al = tick % period
        h0 = seed[n] + s[first][tick - 1]
        vw.append(seed[n] * seed[n] + cum_w[al][tick - 1])
        vr.append(h0 * h0 + cum_r[al][tick - 1])
    return vw, vr


@dataclasses.dataclass(frozen=True)
class Op:
    """A frozen operating point: the feed-forward linearised at w0 (rad/s, per axis); w0 = 0 is the plain T4e model."""
    omega0: tuple = (0.0, 0.0, 0.0)

    @property
    def coupled(self):
        return any(x != 0 for x in self.omega0)

    def label(self):
        return "w0 = 0" if not self.coupled else "w0 = (" + ", ".join(f"{x:+.6g}" for x in self.omega0) + ") rad/s"


def member_plant(c, s, t, a):
    """(J, tau) of grid member (s, t) on axis a: J (1 + s b_J), tau (1 + t b_tau), as l6_ff_eval's grid rows."""
    p = c["fixture"]
    jb, tb = p["inertia_robustness_band"], p["tau_robustness_band"]
    return p[fe.INERTIA_KEYS[a]] * (1 + s * jb), p["motor_tau"] * (1 + t * tb)


def member_sums(c, s, t, op, points, axes=(0, 1, 2), e=0.0, sensor=None):
    """{output axis: (Var_white, Var_rw) per checked execution} of grid member (s, t) at operating point op, the input
    axes summed (module docstring)."""
    sensor = sensor or sensor_of(c)
    period = period_ticks(c)
    divisor = c["fixture"]["rate_loop_divisor"]
    base = period
    n_run = (base + period - 1 + divisor * max(points)) // divisor + 1
    stamps = stamps_of(c, n_run)
    tmaps = [tick_map(c, *member_plant(c, s, t, a), e) for a in range(3)]
    out = {}
    if not op.coupled:
        for a in axes:
            law = law_of(c, a)

            def run(at):
                return axis_run(law, sensor, tmaps[a], stamps, divisor, n_run, inject=at)

            out[a] = unit_sums(run(0), [run(base + cls) for cls in range(period)], base, period, divisor, points)
        return out
    ffl = c["laws"][FF_LAW]
    ff = (rl.gyroscopic_jacobian(list(op.omega0), list(ffl.inertia)), ffl.motor_tau, ffl.ff_tau)
    laws = [law_of(c, a, FF_LAW) for a in range(3)]
    acc = {a: ([0.0] * len(points), [0.0] * len(points)) for a in axes}
    for b in range(3):
        def run(at):
            return coupled_run(laws, ff, sensor, tmaps, stamps, divisor, n_run, b, at)

        seed = run(0)
        cls_runs = [run(base + cls) for cls in range(period)]
        for a in axes:
            vw, vr = unit_sums(seed[a], [x[a] for x in cls_runs], base, period, divisor, points)
            acc[a] = ([x + y for x, y in zip(acc[a][0], vw)], [x + y for x, y in zip(acc[a][1], vr)])
    return acc


# ---- sigma over the grid and the operating points -------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Case:
    """A scenario's noise-term inputs: the class, the checked executions (counted from the run's seed execution 0, tick
    0), the truth-rate channels (axis indices) and the frozen operating points."""
    name: str
    klass: str
    points: tuple
    axes: tuple
    ops: tuple


def grid_values(grid):
    return [-1.0 + 2.0 * i / (grid - 1) for i in range(grid)] if grid > 1 else [0.0]


_CTX = {}


def _member_task(args):
    op_index, s, t = args
    x = _CTX
    case, noise = x["case"], x["noise"]
    sums = member_sums(x["c"], s, t, case.ops[op_index], case.points, case.axes, x["e"], x["sensor"])
    return args, {a: ([noise.white_var * p + noise.rw_var * q for p, q in zip(vw, vr)], vw, vr)
                  for a, (vw, vr) in sums.items()}


def sigma_series(c, case, noise, grid=GRID, e=0.0, jobs=1, sensor=None):
    """{axis: dict of lists over case.points: sigma, var, white, rw, member, op} with sigma(n) the largest over the grid
    members and the operating points at checked execution n; white and rw are the unit sums of that member and point
    (sigma^2 = noise.white_var white + noise.rw_var rw)."""
    _CTX.clear()
    _CTX.update(c=c, case=case, noise=noise, e=e, sensor=sensor or sensor_of(c))
    vals = grid_values(grid)
    tasks = [(k, s, t) for k in range(len(case.ops)) for s in vals for t in vals]
    if jobs > 1:
        with multiprocessing.get_context("fork").Pool(jobs) as pool:
            results = pool.imap(_member_task, tasks, chunksize=1)
            out = _series_reduce(case, results)
    else:
        out = _series_reduce(case, (_member_task(x) for x in tasks))
    for v in out.values():
        v["sigma"] = [math.sqrt(x) for x in v["var"]]
    return out


def _series_reduce(case, results):
    out = {}
    for (k, s, t), per_axis in results:
        for a, (var, vw, vr) in per_axis.items():
            if a not in out:
                out[a] = {"var": list(var), "white": list(vw), "rw": list(vr), "member": [(s, t)] * len(var),
                          "op": [case.ops[k]] * len(var)}
                continue
            o = out[a]
            for i, x in enumerate(var):
                if x > o["var"][i]:
                    o["var"][i], o["white"][i], o["rw"][i] = x, vw[i], vr[i]
                    o["member"][i], o["op"][i] = (s, t), case.ops[k]
    return out


def series_max(case, series):
    """{axis: dict(var, sigma, white, rw, point, member, op)} at the checked execution where sigma_series is largest."""
    out = {}
    for a, v in series.items():
        i = max(range(len(v["var"])), key=v["var"].__getitem__)
        out[a] = {"var": v["var"][i], "sigma": v["sigma"][i], "white": v["white"][i], "rw": v["rw"][i],
                  "point": case.points[i], "member": v["member"][i], "op": v["op"][i]}
    return out


def sigma(c, case, noise, grid=GRID, e=0.0, jobs=1, sensor=None):
    """sigma_c per axis: series_max of sigma_series, the largest over the checked executions too."""
    return series_max(case, sigma_series(c, case, noise, grid, e, jobs, sensor))


def noise_term(sig, z):
    """W_c = z sigma_c per channel."""
    return {a: z * v["sigma"] for a, v in sig.items()}


# ---- the L4 scenarios ---------------------------------------------------------------------------------------------------

def _peak_task(args):
    a, s, t = args
    c = _CTX["c"]
    sps = c["sps"][a]
    y = axis_run(law_of(c, a), sensor_of(c), tick_map(c, *member_plant(c, s, t, a)), c["stamps"],
                 c["fixture"]["rate_loop_divisor"], len(sps), sps=sps)
    return a, max(abs(v) for v in y)


def acro_peak(c, jobs=1):
    """P_a: the largest |w_a| of the T4e linear model driven by the acro script over the 17 x 17 grid (the T4e band's
    peak, l6_ff_eval's P)."""
    _CTX.clear()
    _CTX.update(c=c)
    vals = grid_values(GRID)
    tasks = [(a, s, t) for a in range(3) for s in vals for t in vals]
    if jobs > 1:
        with multiprocessing.get_context("fork").Pool(jobs) as pool:
            results = pool.map(_peak_task, tasks, chunksize=1)
    else:
        results = [_peak_task(x) for x in tasks]
    peak = [0.0, 0.0, 0.0]
    for a, v in results:
        peak[a] = max(peak[a], v)
    return tuple(peak)


def l4_case(c, name, jobs=1):
    """The Case of an L4 scenario (module docstring): a step's window (run_l4.plan on the fixture: executions
    k0 - 1 .. k0 + N - 2), acro's linear and recovery executions (run_l4.acro_phases) with its feed-forward points."""
    klass = SCENARIOS[name]
    if name == "acro":
        phases, _ = run_l4.acro_phases(c["plan"])
        points = tuple(k for k, ph in enumerate(phases) if ph in ("linear", "recovery"))
        peak = acro_peak(c, jobs)
        ops = (Op(),) + tuple(Op(tuple(sg * pk for sg, pk in zip(signs, peak))) for signs in rl.SIGN_PATTERNS)
        return Case(name, klass, points, (0, 1, 2), ops), peak
    plan = run_l4.plan(run_l4.l4s.load(L4_SCENARIOS / f"{name}.yaml"), c["fixture"], fe.CARD)
    return Case(name, klass, tuple(plan.window), (0, 1, 2), (Op(),)), None


# ---- the command line ---------------------------------------------------------------------------------------------------

def render(case, noise, series, reg, m_points, grid, e, peak, at=()):
    sig = series_max(case, series)
    conf = reg["noise_term_confidence"]["value"]
    n_seeds = seed_count(case.klass, reg)
    z = z_value(conf, n_seeds, m_points)
    w = noise_term(sig, z)
    lines = [
        f"MARV quad L6 stage (e) noise term, scenario {case.name} ({case.klass} class), T4e linear design model (T3)",
        "command: uv run python tools/sim/l6_noise_term.py --scenario " + case.name + f" --points {m_points}"
        + (f" --grid {grid}" if grid != GRID else "") + (f" --clock-error {e!r}" if e else "")
        + "".join(f" --at {n}" for n in at),
        "label: design model, not a validation run; frozen operating points INFERRED (memo section 1), verified by the "
        "nightly Monte Carlo (C6)",
        f"noise (per gyro axis, plant tick {noise.dt!r} s): sigma_d {noise.sigma_d!r} rad/s, LSB {noise.lsb!r} rad/s, "
        f"K {noise.k_rw!r}; white variance {noise.white_var!r}, random-walk increment variance {noise.rw_var!r}",
        f"quantile: c = noise_term_confidence {conf!r}, N = {n_seeds}, M_s = {m_points}: z = {z!r}",
        f"grid {grid} x {grid}; checked executions {len(case.points)} ({case.points[0]}..{case.points[-1]}); "
        f"operating points {len(case.ops)}" + (f"; P = {', '.join(repr(x) for x in peak)} rad/s" if peak else ""),
        "channel  sigma_c (rad/s)        white part      rw part         at execution  member (s, t)   op"
        "                          W_c = z sigma_c (rad/s)",
    ]
    for a in case.axes:
        v = sig[a]
        lines.append(f"w_{AXES[a]:<6} {v['sigma']:.9e}  {math.sqrt(noise.white_var * v['white']):.6e}  "
                     f"{math.sqrt(noise.rw_var * v['rw']):.6e}  {v['point']:>12}  ({v['member'][0]:+.3f}, "
                     f"{v['member'][1]:+.3f})  {v['op'].label():<28} {w[a]:.9e}")
    for n in at:
        i = case.points.index(n)
        for a in case.axes:
            v = series[a]
            lines.append(f"at execution {n}: w_{AXES[a]:<6} sigma_c(n) {v['sigma'][i]:.9e} rad/s (white "
                         f"{math.sqrt(noise.white_var * v['white'][i]):.6e}, "
                         f"rw {math.sqrt(noise.rw_var * v['rw'][i]):.6e}; "
                         f"member ({v['member'][i][0]:+.3f}, {v['member'][i][1]:+.3f}), {v['op'][i].label()}), "
                         f"z sigma_c(n) {z * v['sigma'][i]:.9e}; the max over the executions {sig[a]['sigma']:.9e}")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    ap.add_argument("--points", type=int, required=True, metavar="M_s",
                    help="the (execution, channel) points the scenario's predicate checks (the caller's)")
    ap.add_argument("--grid", type=int, default=GRID, help="grid points per band axis (default the oracles' 17)")
    ap.add_argument("--clock-error", type=float, default=0.0, metavar="e", help="the IMU's relative clock error")
    ap.add_argument("--jobs", type=int, default=None, help="worker processes (default: the CPUs this process may use)")
    ap.add_argument("--at", type=int, action="append", default=[], metavar="EXECUTION",
                    help="also print sigma_c(n) at this checked execution (repeatable)")
    args = ap.parse_args(argv)
    jobs = args.jobs or fe.default_jobs()
    t0 = time.time()
    c = fe.setup("none")
    case, peak = l4_case(c, args.scenario, jobs)
    missing = [n for n in args.at if n not in case.points]
    if missing:
        ap.error(f"--at {missing}: not checked executions of {args.scenario}")
    noise = noise_inputs(c, args.clock_error)
    t1 = time.time()
    series = sigma_series(c, case, noise, args.grid, args.clock_error, jobs)
    print(f"l6_noise_term: setup {t1 - t0:.1f} s, sums {time.time() - t1:.1f} s on {jobs} processes", file=sys.stderr)
    sys.stdout.write(render(case, noise, series, register(), args.points, args.grid, args.clock_error, peak, args.at))
    return 0


if __name__ == "__main__":
    sys.exit(main())
