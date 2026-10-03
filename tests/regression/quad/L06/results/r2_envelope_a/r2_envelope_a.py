#!/usr/bin/env python3
"""R2 under envelope A: the design model with w x Jw in the plant and ideal lag-compensated feed-forward (decision 0015;
decision 0014 owner decision 5 and second round item 1; Luis, 2026-10-01, "R2 envelope: A").

    uv run python tests/regression/quad/L06/results/r2_envelope_a/r2_envelope_a.py --out <file> [--jobs <n>]

Question. R2 (scenarios/quad/L05/recover_tumble.yaml) starts at the steady tumble: body rates w0, rotor torque tau0 = the
plant's torque at the steady-tumble rotor speeds (tools/sim/run_l5.py steady_tumble, rotor_torque), which balances
c(w0) = w0 x J w0. Its design model (tests/regression/quad/L05/gz/test_t4_recovery.py; recovery_model.member_run, no w x Jw)
is linear in deviations from an operating point. Envelope A starts that model's motor state at 0, a deviation from the
steady-tumble trim; envelope B starts it at tau0 (the literal reading, withdrawn). The owner's argument: A is the envelope a
perfect controller would track, B the envelope no controller can track. This script runs that perfect controller.

Why A, in one line. Plant C below with v = m - c(w): J w' = v and tau v' = m' tau - c' tau = u + c + tau c' - m - tau c' = u - v,
exactly the design model's axis (J w' = u_m, tau u_m' = u - u_m) with u_m = v, from v(0) = tau0 - c(w0), the trim's
rounding. So C tracks the design model's nominal member and envelope A contains it; B's members start with u_m = tau0.

Envelopes (the design model only, never an observed run). recovery_model.member_run, the R2 test's own model and gains (the
recorded T3 fixture, read by test_t4_recovery.recorded_inputs), over oracle.grid_members (17 x 17 tau x J grid), reduced by
pointwise min / max per channel and execution (oracle.Envelope, as recovery_model.envelope reduces its members):
  A  every member from the scenario's attitude and rates, motor state 0 (member_run as the test calls it), executions
     0 .. end_attitude_execution, R2's whole window;
  B  every member with motor state tau0 at t = 0 (oracle.Axis's u_m seeded at construction), over the first B_EXEC
     executions.
D(m0) is the nominal member (s = t = 0) with motor state m0: D(0) is A's nominal, D(v0) the model from the trim's rounding.

Run C (the counterfactual). member_run of the nominal member, unchanged (its attitude law, rate loop, gains, quaternion
kinematics and channels), with oracle.Axis replaced by plant C for that run only:
  rigid body  J w' = m - c(w), c = run_l5.euler_coupling (w x J w, the steady-tumble rule's own), J the nominal member's
              (the fixture's), Euler's equations in principal axes as c5's l6_ff_eval plant;
  motors      tau m' = u + f - m, the design model's first-order torque lag at the fixture's tau, from m(0) = tau0;
  ideal FF    f = c(w) + tau c'(w), c' = w' x J w + w x J w' with w' from the body equation: the lag-compensated form of
              owner decision 1 (c5's u += g + tau_m g'), exact in J and tau, continuous in time, on the plant's true w
              (no sample-and-hold, filter, noise or quantisation); u is the design model's command, held as it holds it;
  integration RK4 on (w, m, theta), SUBSTEPS steps per tick, u held over the tick; theta' = w gives the per-axis angle
              increments member_run feeds to its kinematics (q <- q (x) exp(dtheta / 2)), so only the plant differs.
The plant swap is checked: member_run must build exactly three axes in axis order with the nominal (J, tau, tick) and step
all three before each kinematic update. Control: C without the FF (f = 0), from the same m(0) = tau0.

Tolerance delta_c of channel c (rounding; every term from the rule, none from C's own residual against D):
  V_c  trim rounding: max_n |D(v0) - D(0)|, v0 = tau0 - c_d(w0), c_d with the fixture's f32 J. v0 splits into
       c_card(w0) - c_d(w0) (the f32 rounding of J; c_card = steady_tumble's torque with the card's binary64 J) and
       tau0 - c_card(w0) (the steady-tumble rule's binary64 residue, checked on the plant by test_steady_tumble.py);
  K_c  integration: max_n |C_S - C_(S/2)|, S = SUBSTEPS: the change from halving the RK4 step; RK4 is fourth order, so the
       error of C_S is about K / (2^4 - 1) (Richardson), and K bounds it;
  R_c  method floor: max_n |I_S - D(0)|, I_S the same integrator on the design model's own plant (no coupling, no FF,
       m(0) = 0), whose exact answer is D(0): RK4 against the exact ZOH and their binary64 rounding.
  delta_c = V_c + K_c + R_c. Guard (INFERRED scale, not a bound): K_c + R_c <= ROUNDOFF ticks max_n |y_c|, one binary64
  unit roundoff of the channel's largest magnitude per tick, so delta is not inflated by a faulty run.

Claims (the rate channels w_x, w_y, w_z; the attitude channels are reported):
  (a) C stays inside envelope A to rounding over R2's whole window: lo_c(n) - delta_c <= y_c(n) <= hi_c(n) + delta_c at every
      execution, and C equals D(v0) within K_c + R_c;
  (b) C leaves envelope B near t = 0: the first execution at which C is outside B by more than delta_c, and by how much;
  control: C without the FF leaves envelope A by more than delta_c (the coupling).

Inputs (none retyped; the output lists each with its sha256): the card and the R2 scenario (run_l5.steady_tumble,
rotor_torque; l5_scenario), the recorded T3 fixture attitude_t3_inputs.txt (gains, J, tau, bands, tick), the design model's
code (test_t4_recovery.py, recovery_model.py, attitude_t3_oracle.py) and run_l5.py. Method constants below set resolution
only. Run times go to stderr; the output does not depend on --jobs.
"""

import argparse
import contextlib
import hashlib
import multiprocessing
import sys
import time
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
for _p in (ROOT / "tools" / "card", ROOT / "tools" / "sim", ROOT / "tests" / "regression" / "quad" / "L05" / "gz"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import cpu_quota  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l5  # noqa: E402
import test_t4_recovery as t4r  # noqa: E402

oracle = rm.oracle
SCEN = t4r.SCEN / f"{t4r.R2}.yaml"
T3_INPUTS = t4r.T3_REFERENCE / "attitude_t3_inputs.txt"
CHANNELS = rm.CHANNELS
RATE = tuple(i for i, c in enumerate(CHANNELS) if c.startswith("w_"))
AXES = oracle.AXES
REAL_AXIS = oracle.Axis
REAL_QUAT_EXP = oracle.quat_exp
ZERO3 = (0.0, 0.0, 0.0)
ROUNDOFF = 2.0 ** -53  # binary64 unit roundoff, round to nearest (IEEE 754-2019, binary64: p = 53)
RK4_ORDER = 4  # classical Runge-Kutta: global error O(h^4)

# Method constants (resolution only).
SUBSTEPS = 4  # RK4 steps per tick of run C; K_c measures the change against half as many
LADDER = (1, 2, 4, 8)  # RK4 steps per tick of the convergence table (contains SUBSTEPS // 2 and SUBSTEPS)
B_EXEC = 32  # executions of envelope B (10 ms): (b) needs only the first exit; the r2_lower_bound proof's window length


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path):
    return Path(path).resolve().relative_to(ROOT)


def g(x):
    return f"{x:.4g}"


def gl(xs):
    return "[" + ", ".join(g(x) for x in xs) + "]"


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# ---- plant C and the plant swaps --------------------------------------------------------------------------------------

class Body:
    """Plant C (module docstring) of the three axes: state (w, m), the commands u the design model wrote this tick."""

    def __init__(self, inertia, tau, h, m0, substeps, coupled=True, ff=True):
        self.j, self.tau, self.h = tuple(inertia), tau, h
        self.substeps, self.coupled, self.ff = substeps, coupled, ff
        self.w = [0.0, 0.0, 0.0]
        self.m = [float(x) for x in m0]
        self.u = [None, None, None]

    def rates(self, s):
        """(w', m', theta') at the state s = (w, m) under the held commands."""
        j, tau = self.j, self.tau
        w, m = s[:3], s[3:6]
        c = run_l5.euler_coupling(j, w) if self.coupled else ZERO3
        wd = tuple((m[a] - c[a]) / j[a] for a in range(3))
        f = ZERO3
        if self.ff:
            jw = tuple(x * y for x, y in zip(j, w))
            jwd = tuple(x * y for x, y in zip(j, wd))
            cd = tuple(p + q for p, q in zip(cross(wd, jw), cross(w, jwd)))
            f = tuple(c[a] + tau * cd[a] for a in range(3))
        md = tuple((self.u[a] + f[a] - m[a]) / tau for a in range(3))
        return wd + md + tuple(w)

    def advance(self):
        """RK4 over one tick with u held; returns the per-axis angle increments."""
        if None in self.u:
            raise RuntimeError("member_run did not step every axis before its kinematic update")
        hs = self.h / self.substeps
        s = (*self.w, *self.m, *ZERO3)
        for _ in range(self.substeps):
            k1 = self.rates(s)
            k2 = self.rates(tuple(x + hs / 2 * k for x, k in zip(s, k1)))
            k3 = self.rates(tuple(x + hs / 2 * k for x, k in zip(s, k2)))
            k4 = self.rates(tuple(x + hs * k for x, k in zip(s, k3)))
            s = tuple(x + hs / 6 * (a + 2 * b + 2 * c + d) for x, a, b, c, d in zip(s, k1, k2, k3, k4))
        self.w, self.m = list(s[:3]), list(s[3:6])
        self.u = [None, None, None]
        return s[6:9]


class View:
    """Axis i of a Body as member_run uses an oracle.Axis: w (seeded and read) and step(u) (records the command; the
    increment reaches the kinematics through the swapped quat_exp)."""

    def __init__(self, body, i):
        self.body, self.i = body, i

    @property
    def w(self):
        return self.body.w[self.i]

    @w.setter
    def w(self, value):
        self.body.w[self.i] = value

    def step(self, u):
        self.body.u[self.i] = u
        return 0.0


@contextlib.contextmanager
def plant_c(body):
    """member_run on plant C: oracle.Axis and oracle.quat_exp swapped for this run only."""
    views = iter([View(body, i) for i in range(3)])

    def axis(inertia, tau, h):
        v = next(views)
        if (inertia, tau, h) != (body.j[v.i], body.tau, body.h):
            raise RuntimeError(f"member_run built axis {v.i} with {(inertia, tau, h)}, not plant C's nominal")
        return v

    def quat_exp(_):
        return REAL_QUAT_EXP(body.advance())

    with mock.patch.object(oracle, "Axis", axis), mock.patch.object(oracle, "quat_exp", quat_exp):
        yield
    if next(views, None) is not None:
        raise RuntimeError("member_run built fewer than three axes")


@contextlib.contextmanager
def seeded(m0):
    """member_run with every axis's motor state seeded at m0 (envelope B, D(v0))."""
    order = iter(range(3))

    def axis(inertia, tau, h):
        pl = REAL_AXIS(inertia, tau, h)
        pl.um = m0[next(order)]
        return pl

    with mock.patch.object(oracle, "Axis", axis):
        yield
    if next(order, None) is not None:
        raise RuntimeError("member_run built fewer than three axes")


# ---- the runs (module globals, so a forked pool sees them) ------------------------------------------------------------

_CTX = {}


def member(s, t, n, m0=None):
    c = _CTX
    if m0 is None:
        out, _ = rm.member_run(c["su"], s, t, c["q0"], c["w0"], n)
        return out
    with seeded(m0):
        out, _ = rm.member_run(c["su"], s, t, c["q0"], c["w0"], n)
    return out


def run_c(substeps, coupled, ff, m0):
    c = _CTX
    body = Body(c["J"], c["tau"], c["su"].tick_s, m0, substeps, coupled, ff)
    with plant_c(body):
        out, _ = rm.member_run(c["su"], 0.0, 0.0, c["q0"], c["w0"], c["n"])
    return out


def _row(kind, i):
    """Envelope rows: the 17 members of grid row i, reduced to (lo, hi) per channel, and the nominal member if in the row."""
    c = _CTX
    n = c["n"] if kind == "A" else B_EXEC
    m0 = None if kind == "A" else c["tau0"]
    env = [oracle.Envelope(n) for _ in CHANNELS]
    nominal = None
    for _, s, t, coarse in c["rows"][i]:
        out = member(s, t, n, m0)
        cols = list(zip(*out))
        for ch, e in enumerate(env):
            e.add(cols[ch], coarse)
        if s == 0.0 and t == 0.0:
            nominal = out
    return [e.lo for e in env], [e.hi for e in env], nominal


def _job(job):
    kind = job[0]
    if kind == "C":
        _, substeps, coupled, ff, m0 = job
        return job, run_c(substeps, coupled, ff, _CTX[m0])
    if kind == "Dv0":
        return job, member(0.0, 0.0, _CTX["n"], _CTX["v0"])
    return job, _row(*job)


def setup():
    su = oracle.Setup(t4r.recorded_inputs())
    vals = l5s.values(l5s.load(SCEN))
    st = vals["initial_state"]
    if st["rotor_speed_rad_s"] != l5s.ROTOR_SPEED_STEADY_TUMBLE:
        raise SystemExit(f"{rel(SCEN)}: rotor_speed_rad_s is {st['rotor_speed_rad_s']!r}, not the steady tumble")
    p = su.p
    inertia = [p[oracle.INERTIA[a]] for a in AXES]
    q0 = tuple(st["attitude_q_wxyz"])
    w0 = tuple(float(x) for x in st["body_rates_frd_rad_s"])
    trim = run_l5.steady_tumble(t4r.CARD, vals)
    tau0 = run_l5.rotor_torque(t4r.CARD, trim["speeds"])
    c_d = run_l5.euler_coupling(inertia, w0)
    grid = oracle.grid_members()
    side = 2 * (oracle.GRID - 1) + 1  # grid_members' points per band axis, row-major in s
    rows = [grid[i * side:(i + 1) * side] for i in range(side)]
    return {"su": su, "vals": vals, "q0": q0, "w0": w0, "n": vals["script"]["end_attitude_execution"] + 1, "J": inertia,
            "tau": p["motor_tau"], "trim": trim, "tau0": tau0, "c_d": c_d, "v0": tuple(a - b for a, b in zip(tau0, c_d)),
            "zero": ZERO3, "rows": rows}


def jobs():
    out = [("C", s, True, True, "tau0") for s in sorted(LADDER, reverse=True)]
    out += [("C", SUBSTEPS, True, False, "tau0"), ("C", SUBSTEPS, False, False, "zero"), ("Dv0",)]
    out += [("A", i) for i in range(len(_CTX["rows"]))] + [("B", i) for i in range(len(_CTX["rows"]))]
    return out


# ---- the evaluation -----------------------------------------------------------------------------------------------------

def dev(a, b):
    """Per channel: max_n |a(n) - b(n)|."""
    return [max(abs(x[c] - y[c]) for x, y in zip(a, b)) for c in range(len(CHANNELS))]


def outside(y, lo, hi, c, n):
    """Per execution 0 .. n - 1: max(lo - y, y - hi) of channel c (positive outside)."""
    return [max(lo[c][k] - y[k][c], y[k][c] - hi[c][k]) for k in range(n)]


def evaluate(r):
    n = r["n"]
    a_lo, a_hi, b_lo, b_hi = r["A_lo"], r["A_hi"], r["B_lo"], r["B_hi"]
    d0, dv0 = r["D0"], r["Dv0"]
    cs = {s: r["C"][(s, True, True)] for s in LADDER}
    run, half = cs[SUBSTEPS], cs[SUBSTEPS // 2]
    noff, ident = r["C"][(SUBSTEPS, True, False)], r["C"][(SUBSTEPS, False, False)]
    big_v, big_k, big_r = dev(dv0, d0), dev(run, half), dev(ident, d0)
    delta = [v + k + q for v, k, q in zip(big_v, big_k, big_r)]
    ticks = n * r["su"].divisor * r["su"].ratio
    scale = [ROUNDOFF * ticks * max(abs(y[c]) for y in run) for c in range(len(CHANNELS))]
    ladder = {s: dev(cs[s], dv0) for s in LADDER}
    dev_nom = dev(run, d0)
    per = []
    for c in range(len(CHANNELS)):
        out_a = outside(run, a_lo, a_hi, c, n)
        k_max = max(range(n), key=lambda k: out_a[k])
        open_ = [k for k in range(n) if a_lo[c][k] < a_hi[c][k]]
        k_tight = min(open_, key=lambda k: -out_a[k])
        out_b = outside(run, b_lo, b_hi, c, B_EXEC)
        b_first = next((k for k in range(B_EXEC) if out_b[k] > delta[c]), None)
        b_max = max(range(B_EXEC), key=lambda k: out_b[k])
        out_x = outside(noff, a_lo, a_hi, c, n)
        x_first = next((k for k in range(n) if out_x[k] > delta[c]), None)
        x_max = max(range(n), key=lambda k: out_x[k])
        per.append({
            "V": big_v[c], "K": big_k[c], "R": big_r[c], "delta": delta[c], "scale": scale[c],
            "res_v0": ladder[SUBSTEPS][c], "dev_nom": dev_nom[c],
            "a_out": out_a[k_max], "a_out_n": k_max, "a_beyond": sum(1 for x in out_a if x > delta[c]),
            "a_zero_width": len(open_) < n, "a_closed": n - len(open_), "a_margin": -out_a[k_tight], "a_margin_n": k_tight,
            "a_width": a_hi[c][k_tight] - a_lo[c][k_tight],
            "b_first": b_first, "b_first_out": None if b_first is None else out_b[b_first],
            "b_first_lo": None if b_first is None else b_lo[c][b_first],
            "b_first_hi": None if b_first is None else b_hi[c][b_first],
            "b_first_y": None if b_first is None else run[b_first][c], "b_max": out_b[b_max], "b_max_n": b_max,
            "x_first": x_first, "x_first_out": None if x_first is None else out_x[x_first], "x_max": out_x[x_max],
            "x_max_n": x_max, "x_beyond": sum(1 for x in out_x if x > delta[c]),
        })
    claim_a = all(per[c]["a_beyond"] == 0 and per[c]["res_v0"] <= per[c]["K"] + per[c]["R"] for c in RATE)
    claim_b = all(per[c]["b_first"] is not None for c in RATE)
    control = any(per[c]["x_beyond"] > 0 for c in RATE)
    guard = all(per[c]["K"] + per[c]["R"] <= per[c]["scale"] for c in RATE)
    return {"per": per, "ladder": ladder, "claim_a": claim_a, "claim_b": claim_b, "control": control, "guard": guard,
            "ticks": ticks}


# ---- the report ---------------------------------------------------------------------------------------------------------

def render(r, ev):
    su, trim = r["su"], r["trim"]
    per = ev["per"]
    c_card = trim["torque"]
    inputs = [("card", t4r.CARD), ("scenario", SCEN), ("attitude_t3_inputs.txt", T3_INPUTS),
              ("test_t4_recovery.py", t4r.__file__), ("recovery_model.py", rm.__file__),
              ("attitude_t3_oracle.py", oracle.__file__), ("run_l5.py", run_l5.__file__)]
    L = ["MARV quad L6 stage (c): R2 under envelope A, the design model with w x Jw in the plant and ideal lag-compensated "
         "feed-forward (decision 0015; decision 0014 owner decision 5)",
         "command: uv run python tests/regression/quad/L06/results/r2_envelope_a/r2_envelope_a.py --out <file>",
         "label: design model (T3), not a validation run",
         "inputs sha256: " + "; ".join(f"{k} {sha(p)}" for k, p in inputs),
         "",
         "== 0. setup",
         f"scenario {rel(SCEN)}: q0 {list(r['q0'])}; w0 {[repr(x) for x in r['w0']]} rad/s; executions 0 .. {r['n'] - 1} "
         f"(T_a {g(su.t_a)} s, {su.divisor * su.ratio} ticks of {g(su.tick_s)} s each, {ev['ticks']} ticks)",
         f"design model (fixture): J {[repr(x) for x in r['J']]} kg m^2; tau {r['tau']!r} s; bands J +-{g(su.j_band)}, "
         f"tau +-{g(su.tau_band)}; " + "; ".join(f"rate_kp_{a} {su.p['rate_kp_' + a]!r} rate_ki_{a} {su.p['rate_ki_' + a]!r}"
                                                 for a in AXES) + f"; att_kp {su.p['att_kp']!r}",
         f"steady tumble (run_l5.steady_tumble): collective {g(trim['collective'])} N (hover {g(trim['hover_collective'])} N, "
         f"hover feasible {trim['hover_feasible']}); rotor speeds {gl(trim['speeds'])} rad/s",
         f"tau0 (run_l5.rotor_torque at those speeds) {[repr(x) for x in r['tau0']]} N m",
         f"c_d(w0) = w0 x J w0, fixture J {[repr(x) for x in r['c_d']]} N m",
         f"v0 = tau0 - c_d(w0) {gl(r['v0'])} N m = [c_card(w0) - c_d(w0), f32 rounding of J] "
         f"{gl([a - b for a, b in zip(c_card, r['c_d'])])} + [tau0 - c_card(w0), the rule's binary64 residue] "
         f"{gl([a - b for a, b in zip(r['tau0'], c_card)])}",
         "",
         "== 1. tolerance delta = V + K + R per channel (rad for err_*, rad/s for w_*); guard: K + R <= binary64 scale "
         "ROUNDOFF ticks max|y| (INFERRED)"]
    for c, ch in enumerate(CHANNELS):
        p = per[c]
        L.append(f"{ch}: V {g(p['V'])}; K {g(p['K'])}; R {g(p['R'])}; delta {g(p['delta'])}; scale {g(p['scale'])}")
    L.append(f"guard on the rate channels: {ev['guard']}")
    L += ["",
          f"== 2. convergence: max_n |C_S - D(v0)| per channel, S RK4 steps per tick (reported run S = {SUBSTEPS})"]
    for s in LADDER:
        L.append(f"S {s}: {gl(ev['ladder'][s])}")
    L += ["",
          f"== 3. claim (a): C inside envelope A to rounding, executions 0 .. {r['n'] - 1}"]
    for c, ch in enumerate(CHANNELS):
        p = per[c]
        L.append(f"{ch}: |C - D(v0)| {g(p['res_v0'])} (<= K + R {g(p['K'] + p['R'])}: {p['res_v0'] <= p['K'] + p['R']}); "
                 f"|C - D(0)| {g(p['dev_nom'])}; max outside A {g(p['a_out'])} at n {p['a_out_n']}; executions outside "
                 f"by more than delta {p['a_beyond']}; zero-width executions {p['a_closed']}; least margin where A is open "
                 f"{g(p['a_margin'])} at n {p['a_margin_n']} (width {g(p['a_width'])})")
    L.append(f"claim (a) on the rate channels: {ev['claim_a']}")
    L += ["",
          f"== 4. claim (b): C leaves envelope B (motor state tau0), executions 0 .. {B_EXEC - 1}"]
    for c, ch in enumerate(CHANNELS):
        p = per[c]
        if p["b_first"] is None:
            L.append(f"{ch}: inside B within delta over the window; max outside {g(p['b_max'])} at n {p['b_max_n']}")
            continue
        L.append(f"{ch}: first outside B by more than delta at n {p['b_first']} (t {g(p['b_first'] * su.t_a)} s): C "
                 f"{p['b_first_y']!r}, B [{p['b_first_lo']!r}, {p['b_first_hi']!r}], outside by {g(p['b_first_out'])}; "
                 f"max outside {g(p['b_max'])} at n {p['b_max_n']}")
    L.append(f"claim (b) on the rate channels: {ev['claim_b']}")
    L += ["",
          "== 5. control: C without the feed-forward (f = 0), m(0) = tau0, against envelope A"]
    for c, ch in enumerate(CHANNELS):
        p = per[c]
        first = "none" if p["x_first"] is None else f"n {p['x_first']} by {g(p['x_first_out'])}"
        L.append(f"{ch}: first outside A by more than delta: {first}; max outside {g(p['x_max'])} at n {p['x_max_n']}; "
                 f"executions outside by more than delta {p['x_beyond']}")
    L.append(f"control leaves A on a rate channel: {ev['control']}")
    return "\n".join(L) + "\n"


def compute(jobs_n=None):
    _CTX.clear()
    _CTX.update(setup())
    procs = jobs_n or cpu_quota.usable_cpus()
    t0 = time.time()
    with multiprocessing.get_context("fork").Pool(procs) as pool:
        results = pool.map(_job, jobs(), chunksize=1)
    print(f"r2_envelope_a: runs {time.time() - t0:.1f} s on {procs} processes", file=sys.stderr)
    r = dict(_CTX)
    r["C"] = {}
    acc = {"A": None, "B": None}
    for job, res in results:
        if job[0] == "C":
            r["C"][job[1:4]] = res
        elif job[0] == "Dv0":
            r["Dv0"] = res
        else:
            lo, hi, nominal = res
            if job[0] == "A" and nominal is not None:
                r["D0"] = nominal
            prev = acc[job[0]]
            acc[job[0]] = (lo, hi) if prev is None else (
                [[min(x, y) for x, y in zip(a, b)] for a, b in zip(prev[0], lo)],
                [[max(x, y) for x, y in zip(a, b)] for a, b in zip(prev[1], hi)])
    r["A_lo"], r["A_hi"] = acc["A"]
    r["B_lo"], r["B_hi"] = acc["B"]
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, metavar="FILE")
    ap.add_argument("--jobs", type=int, default=None, help="worker processes (default: the CPUs this process may use)")
    args = ap.parse_args(argv)
    t0 = time.time()
    r = compute(args.jobs)
    ev = evaluate(r)
    text = render(r, ev)
    Path(args.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    print(f"r2_envelope_a: wrote {args.out} ({time.time() - t0:.1f} s)", file=sys.stderr)
    return r, ev, text


if __name__ == "__main__":
    main()
