"""The design-model side of the R1X coupling counterfactual (decision 0014, owner decisions third round, item 3): the model
runs on gz's branch and their reductions. Shared by capture_gz.py (which also flies gz) and r1x_coupling.py (which re-runs
this side without gz). Standard library only at import: load_tree() imports the repository tree's own modules, so the same
code runs the 69c62f2 tree's design model for the old gains and this tree's for the new.

Model runs. The nominal member (s = t = 0) of the R1X test's design model, recovery_model.member_run of the tree, unchanged
(attitude law, rate loop, gains, quaternion kinematics, channels), from the scenario's exact initial state, with up to two
swaps for one run only:
  gz's branch  the first kinematic update is pre-multiplied by conj(q0) (x) q_gz(1), q_gz(1) the m = 1 run's TRUTH attitude
               at execution 1, so the model's attitude at execution 1 is gz's (the plant has not moved: w(0) = 0 and the seed
               execution's command is 0). That is the rounding of gz's first plant step, which picks the branch of the
               tilt-yaw split at exactly 180 degrees (test_t4_recovery.py, "Side check"). Nothing else of gz enters.
  the plant    oracle.Axis replaced by plant C of tests/regression/quad/L06/results/r2_envelope_a (Body below):
               J w' = m - c(w), c(w) = w x J w, Euler's equations in principal axes;
               tau m' = u + f - m, the design model's first-order torque lag, m(0) = 0;
               f = c + tau c' (CF only), the ideal lag-compensated feed-forward, c' = w' x J w + w x J w';
               RK4 on (w, m, theta), the given steps per tick, u held over the tick; J and tau the nominal member's.
               R1X starts at rest at the hover trim, so the design model's motor deviation and c(w(0)) are both 0.
Runs (RUNS):
  D     the design model (oracle.Axis, exact ZOH, no w x Jw) on gz's branch;
  C     plant C on gz's branch;
  CF    plant C with the ideal FF on gz's branch. With v = m - c: J w' = v, tau v' = u - v, the design axis exactly, so CF
        equals D up to the integration method;
  I     the same RK4 on the design plant (no coupling, no FF) on gz's branch: R = max |I - D|, the method floor;
  C/2, CF/2  C and CF with half the RK4 steps per tick: K = max |X - X/2|, the integration term (RK4 is fourth order);
  C0    plant C from the exact start, not on gz's branch: the control of the distance claim.
"""

import contextlib
import hashlib
import math
import multiprocessing
import sys
import types
from pathlib import Path
from unittest import mock

CHANNELS = ("err_x", "err_y", "err_z", "w_x", "w_y", "w_z", "alpha")
ALPHA = CHANNELS.index("alpha")
W_X, W_Y = CHANNELS.index("w_x"), CHANNELS.index("w_y")
RATE = tuple(CHANNELS.index(c) for c in ("w_x", "w_y", "w_z"))
ZERO3 = (0.0, 0.0, 0.0)
SUBSTEPS = 4  # RK4 steps per tick, r2_envelope_a's; K compares against SUBSTEPS // 2
RUNS = ("D", "C", "CF", "I", "C/2", "CF/2", "C0")
SPEC = {  # run: (plant, RK4 steps per tick, on gz's branch)
    "D": ("D", None, True),
    "C": ("C", SUBSTEPS, True),
    "CF": ("CF", SUBSTEPS, True),
    "I": ("I", SUBSTEPS, True),
    "C/2": ("C", SUBSTEPS // 2, True),
    "CF/2": ("CF", SUBSTEPS // 2, True),
    "C0": ("C", SUBSTEPS, False),
}
SUMMARISED = ("D", "C", "CF", "C0")
RATIO_EXEC = 300  # report choice: the execution (94 ms) at which the diagnosis compared the branch's w_y / w_x
GAIN_KEYS = ("att_kp", "att_yaw_weight", "rate_kp_roll", "rate_ki_roll", "rate_kd_roll", "rate_kp_pitch", "rate_ki_pitch",
             "rate_kd_pitch", "rate_kp_yaw", "rate_ki_yaw", "rate_kd_yaw", "inertia_xx", "inertia_yy", "inertia_zz",
             "motor_tau")


def load_tree(root):
    """The tree's own test module, design model, oracle and runners (imported from `root`, refused otherwise)."""
    root = Path(root).resolve()
    for p in (root / "tools" / "card", root / "tools" / "sim", root / "tests" / "regression" / "quad" / "L05" / "gz"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import l5_scenario  # noqa: PLC0415
    import recovery_model  # noqa: PLC0415
    import run_l5  # noqa: PLC0415
    import run_scenario  # noqa: PLC0415
    import test_t4_recovery  # noqa: PLC0415
    for mod in (l5_scenario, recovery_model, recovery_model.oracle, run_l5, run_scenario, test_t4_recovery):
        if not Path(mod.__file__).resolve().is_relative_to(root):
            raise SystemExit(f"r1x_model: {mod.__name__} was imported from {mod.__file__}, not from the tree {root}")
    return types.SimpleNamespace(root=root, t4r=test_t4_recovery, rm=recovery_model, oracle=recovery_model.oracle,
                                 run_l5=run_l5, run_scenario=run_scenario, l5s=l5_scenario)


def sha_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def series_sha(rows):
    """sha256 of the rows' repr, one per line: pins a series bit for bit."""
    return sha_text("\n".join(repr(tuple(r)) for r in rows) + "\n")


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def coupling(inertia, w):
    """w x (J w), J = diag(inertia) (tools/sim/run_l5.py euler_coupling's expression)."""
    jw = [j * x for j, x in zip(inertia, w)]
    return (w[1] * jw[2] - w[2] * jw[1], w[2] * jw[0] - w[0] * jw[2], w[0] * jw[1] - w[1] * jw[0])


# ---- plant C and the swaps ---------------------------------------------------------------------------------------------

class Body:
    """Plant C (module docstring) of the three axes: state (w, m), the commands u the design model wrote this tick."""

    def __init__(self, inertia, tau, h, substeps, coupled, ff):
        self.j, self.tau, self.h = tuple(inertia), tau, h
        self.substeps, self.coupled, self.ff = substeps, coupled, ff
        self.w = [0.0, 0.0, 0.0]
        self.m = [0.0, 0.0, 0.0]
        self.u = [None, None, None]

    def rates(self, s):
        j, tau = self.j, self.tau
        w, m = s[:3], s[3:6]
        c = coupling(j, w) if self.coupled else ZERO3
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
        """One tick: RK4 on (w, m, theta) under the held commands; returns the per-axis angle increments."""
        if any(x is None for x in self.u):
            raise RuntimeError("plant swap: an axis was not stepped before the kinematic update")
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
    """One axis of a Body, with oracle.Axis's interface as member_run uses it (w, step)."""

    def __init__(self, body, i):
        self.body, self.i = body, i

    @property
    def w(self):
        return self.body.w[self.i]

    @w.setter
    def w(self, v):
        self.body.w[self.i] = v

    def step(self, u):
        self.body.u[self.i] = u
        return 0.0


_CTX = {}


def _run(key):
    tree, su, q0, w0, q1, n = (_CTX[k] for k in ("tree", "su", "q0", "w0", "q1", "n"))
    oracle, rm = tree.oracle, tree.rm
    plant, substeps, seeded = SPEC[key]
    real_qexp = oracle.quat_exp
    dq = oracle.qmul(oracle.qconj(q0), q1)
    first = [seeded]

    def seed(e):
        if first[0]:
            first[0] = False
            return oracle.qmul(dq, e)
        return e

    patches = []
    if plant == "D":
        def qexp(d):
            return seed(real_qexp(d))
    else:
        p = su.p
        body = Body([p[oracle.INERTIA[a]] for a in oracle.AXES], p["motor_tau"], su.tick_s, substeps,
                    coupled=plant != "I", ff=plant == "CF")
        views = iter([View(body, i) for i in range(3)])

        def axis(inertia, tau, h):
            v = next(views)
            if (inertia, tau, h) != (body.j[v.i], body.tau, body.h):
                raise RuntimeError(f"plant swap: axis {v.i} built with {(inertia, tau, h)}, not the nominal member's")
            return v

        def qexp(_d):
            return seed(real_qexp(body.advance()))

        patches.append(mock.patch.object(oracle, "Axis", axis))
    patches.append(mock.patch.object(oracle, "quat_exp", qexp))
    with contextlib.ExitStack() as stack:
        for pt in patches:
            stack.enter_context(pt)
        rows, _ = rm.member_run(su, 0.0, 0.0, q0, w0, n, fn=lambda q, w: (*rm.channels(q, w), rm.alpha_channel(q, None)[0]))
    if seeded and first[0]:
        raise RuntimeError("gz's branch: the seed was never applied")
    return key, rows


def model_runs(tree, su, q0, w0, q1, n, procs):
    """{run: rows} for RUNS; rows[n] = CHANNELS of execution n."""
    _CTX.update(tree=tree, su=su, q0=tuple(q0), w0=tuple(w0), q1=tuple(q1), n=n)
    with multiprocessing.get_context("fork").Pool(max(1, min(procs, len(RUNS)))) as pool:
        return dict(pool.map(_run, RUNS, chunksize=1))


# ---- reductions --------------------------------------------------------------------------------------------------------

def verdict(alpha, lo, hi, f, q, e=None):
    """The test's predicate on one alpha series (test_t4_recovery.evaluate's expressions; e = None: E = 0)."""
    st = {"violations": 0, "first": None, "worst_n": None, "worst_slack": None, "worst_alpha": None, "worst_lo": None,
          "worst_hi": None}
    for n in range(len(lo)):
        y = alpha[n]
        out = max(lo[n] - y, y - hi[n])
        slack = out - ((e[n] if e is not None else 0.0) + f + q)
        if slack > 0:
            st["violations"] += 1
            if st["first"] is None:
                st["first"] = n
        if st["worst_slack"] is None or slack > st["worst_slack"]:
            st.update(worst_n=n, worst_slack=slack, worst_alpha=y, worst_lo=lo[n], worst_hi=hi[n])
    return st


def summary(rows, design, su, tree):
    """The per-run reductions of the report."""
    a = [r[ALPHA] for r in rows]
    lo, hi = design.lo[0], design.hi[0]
    n_ex = max(range(len(lo)), key=lambda n: a[n] - hi[n])
    j = [su.p[tree.oracle.INERTIA[x]] for x in tree.oracle.AXES]
    cint = [0.0, 0.0, 0.0]
    for r in rows[:design.count]:
        c = coupling(j, r[W_X:W_X + 3])
        cint = [s + abs(x) * su.t_a for s, x in zip(cint, c)]
    return {"verdict": verdict(a, lo, hi, design.F[0], design.Q[0]),
            "alpha_minus_hi": {"n": n_ex, "value": a[n_ex] - hi[n_ex]},
            "alpha_below_half_pi_n": next((n for n in range(len(a)) if a[n] < math.pi / 2), None),
            "w_y_over_w_x": rows[RATIO_EXEC][W_Y] / rows[RATIO_EXEC][W_X],
            "coupling_integral": cint,
            "peak_abs": {c: max(abs(r[CHANNELS.index(c)]) for r in rows) for c in ("err_z", "w_x", "w_y", "w_z")}}


def max_diff(a, b):
    """Per channel max_n |a - b|."""
    return [max(abs(x[c] - y[c]) for x, y in zip(a, b)) for c in range(len(CHANNELS))]


def model_facts(runs, design, su, tree):
    """Everything model-only that the report states (the re-run reproduces it without gz)."""
    d, c = runs["D"], runs["C"]
    n_eff = max(range(len(d)), key=lambda n: abs(c[n][ALPHA] - d[n][ALPHA]))
    k_c, k_cf = max_diff(c, runs["C/2"]), max_diff(runs["CF"], runs["CF/2"])
    r = max_diff(runs["I"], d)
    cf_d, c_d = max_diff(runs["CF"], d), max_diff(c, d)
    return {"series_sha256": {k: series_sha(runs[k]) for k in RUNS},
            "summary": {k: summary(runs[k], design, su, tree) for k in SUMMARISED},
            "coupling_alpha_effect": {"n": n_eff, "value": abs(c[n_eff][ALPHA] - d[n_eff][ALPHA]), "alpha_C": c[n_eff][ALPHA],
                                      "alpha_D": d[n_eff][ALPHA]},
            "ideal_ff": {"CF_minus_D": cf_d, "K_CF": k_cf, "R": r, "within_K_plus_R": [x <= k + rr for x, k, rr in zip(cf_d, k_cf, r)],
                         "control_C_minus_D": c_d, "K_C": k_c,
                         "control_beyond_K_plus_R": [x > k + rr for x, k, rr in zip(c_d, k_c, r)]}}


def design_facts(design, tree):
    p = tree.t4r.recorded_inputs()
    return {"N": design.count, "F": design.F[0], "Q": design.Q[0], "halving": design.halving[0], "kin": design.kin[0],
            "end_ok": design.end_ok[0], "lo_hi_sha256": series_sha(zip(design.lo[0], design.hi[0])),
            "gains": {k: p[k] for k in GAIN_KEYS if k in p}}


def kv(key, value):
    return f"{key} = {value!r}"


def facts_lines(prefix, facts):
    return [kv(f"{prefix}.{k}", v) for k, v in facts.items()]
