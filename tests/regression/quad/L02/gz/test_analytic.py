"""T4 analytic tests of the gz-sim 8 lockstep plugin (quad spec 4 L2, docs/decisions/0003 items 7, 9 and 11).

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz build of libmarv_gz_lockstep.so is absent
(the gz CI step fails on a skip). Each scenario runs its m_sequence [4, 2, 1], one gz process per m
(tools/sim/run_scenario.py run_sequence), evaluates each checked quantity with tools/sim/t4_rule.py evaluate, and writes
the verdicts (d_k, rho, p_obs, E, q_hat, error, bound, F, F_ref, reasons, and the inputs) into the `halving` section of the
sequence report.

Time base of a read. Step i of the log reads the link in PreUpdate, before gz's physics step i. So the read of step i is the
state after i physics steps, at t_i = i H (step 0 is the initial state: zero velocity, position bitwise the scenario's).
The log's sim_time_ns of step i is gz's clock at that PreUpdate, (i + 1) H, one host step ahead of the state. Every
reference here is evaluated at t_i = i H, the time of the state that was read. (With (i + 1) H the reference would be one H
early or late, an O(H) shift that the rule cannot see, since it vanishes with H; it is a wrong time all the same.)

Freshness (item 11). Per run the quantity is read at the last fresh host step i_k (the raw 13-double gz read differs
bitwise from step i_k - 1's), t4_rule.freshness must be ok (last half of the run; hover: after t_s). The three m have
different i_k H_k, so the reference is evaluated per run, at that run's t_k, and the rule is applied to the errors
   e_k = q_k - q_ref(t_k),   q_ref = 0                                    (evaluate(e, 0.0, F, F_ref)).
This is the rule itself when the times coincide: every term of the rule depends on q_k only through d_k = q_k - q_{k-1},
q_K and q_hat = q_K + d_K rho / (1 - rho); with q_k = e_k + c (a common reference c) d_k is unchanged, q_K - c = e_K and
q_hat - c = e_hat, so (i) and (ii) read alike (test_error_form_is_the_rule_when_times_coincide, on dyadic numbers, where
the shift is exact, and it must fail alike). F and F_ref are each the maximum over the three runs: F_k = (N_k + extra) u Q_k
with N_k = i_k m_k ticks (the plant accumulates per tick), Q_k the magnitude scale of the reference at t_k
(tools/sim/reference.py), extra the counted roundings of the quantity's own formula (reference.py); F_ref_k the reference's
own error at t_k.

  free_fall  p_D = -z_enu and v_D = -w_enu (the exact ENU -> NED map) against reference.free_fall.
  rotation   |L| and KE of the logged plant body rates (body_omega_frd, the bytes the plant saw). The reference is the value at
             the first free step (index 1: the step after the one that applied the initial rates), of the same run; the
             rule sees e_k = q_k - q_ref,k. Liveness: some component of w_b(T) - w_b(first) exceeds its floor.
  hover      two sequences, D_lo and D_hi (0003 item 7). p_D, v_D against reference.hover_state_tick_held: marv_plant
             evaluates the wrench from the end-of-step motor state and holds it over the step, so over tick j the thrust is
             4 k w(t_{j+1})^2, and a continuous-thrust reference is ~ (1/2) a_T t_tick off in v whatever the host step (item 7,
             last paragraph). Before the rule, for every run and every tick, the logged force is compared with that same
             per-tick thrust (tick_thrust_check below): the reference is then the plant's defined input, not a fitted curve.
             a_D is the backward
             difference (v_i - v_{i-1}) / H of two fresh velocities, i the last step after t_s with i and i - 1 both fresh,
             assigned to the midpoint (i - 1/2) H, where it is exact to O(H^2) for smooth v (after t_s the analytic a_D is
             constant to F_a, so the assignment matters little). Its floor by the stencil, from the errors of v_i and v_{i-1}
             (each at most F_v = (N + extra) u Q_v, treated as independent, a worst-case bound) and three roundings of the
             quotient (the subtraction, the division, and H = m tick rounded to binary64):
                F_a = (F_v,i + F_v,i-1) / H + 3 u Q_a.
             The bracket: a(D_lo) > 0 > a(D_hi) and |a| <= 4 k (w(D_hi)^2 - w(D_lo)^2) / m, with a the measured a_D of the
             finest run. a_D keeps the continuous reference: after t_s the thrust is settled, and the tick-held and
             continuous accelerations differ there by 2 a_T E h / tau (E = e^-(t/tau), below F_a) plus g's gradient times the
             position offset (~ 5e-9 m/s^2, printed by tests/regression/quad/L02/tools/test_reference.py), inside a_D's floor
             through the stencil's F_v terms.

Negative controls, injected through the harness only (a world edit of the plugin's `sil_override` element, the SIL
override of core 4; no code changes): free_fall and rotation, motor 1 at DShot 48 (core 4, the lowest throttle) against
the unchanged reference; hover, the D_lo run with motor 1 at D_lo + 1 against the D_lo reference. Each passes only if
the rule fails, and it must fail on the rule (a reason of (i) and of (ii) from the rule), with the freshness ok, the edited
world run to completion and motor 1's command in the log the injected one.
"""

import math
import re
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import hover as hover_mod  # noqa: E402
import reference  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
PLUGIN_DIR = run_scenario.DEFAULT_PLUGIN_DIR
CONTROL_DSHOT = 48  # core section 4: the lowest throttle value
FD_ROUNDINGS = 3  # roundings of the a_D quotient beyond the two velocities: the subtraction, the division, H itself
THRUST_LOG_ROUNDINGS = 20  # relative roundings of one tick's thrust in the log-versus-reference comparison, see tick_thrust_check
PLANT_ETA_ROUNDINGS = 1  # the motor step w <- c + (w - c) r: the add
PLANT_THETA_ROUNDINGS = 2  # the subtraction and the product
PLANT_DECAY_ROUNDINGS = 3  # exp(-h / tau) as the plant evaluates it: 1 ulp of exp (<= 2 u) and the argument
RULE_REASONS_I = ("(i): E > F", "(i): |q_K - q_ref| > F + F_ref")
RULE_REASONS_II = ("(ii): rho not in (0, 1)", "(ii): |q_hat - q_ref| > bound")

needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the rule in error form -------------------------------------------------------------------------------------------

def apply_rule(qs, q_refs, floors, ref_floors, meta):
    """The rule on e_k = q_k - q_ref(t_k) with q_ref = 0, F and F_ref the maxima over the runs; (Verdict, report dict)."""
    errors = [q - r for q, r in zip(qs, q_refs)]
    verdict = t4_rule.evaluate(errors, 0.0, max(floors), max(ref_floors))
    report = verdict.report()
    report.update(meta)
    report.update({"q": list(qs), "q_ref": list(q_refs), "e": errors, "F_k": list(floors), "F_ref_k": list(ref_floors)})
    return verdict, report


def verdict_line(scenario, quantity, rep):
    def f(x):
        return "n/a" if x is None else f"{x:.6e}"

    return (f"{scenario:<14} {quantity:<4} {'PASS' if rep['passed'] else 'FAIL'} branch {rep['branch']} "
            f"d_k=[{', '.join(f(x) for x in rep['d'])}] rho={f(rep['rho'])} p_obs={f(rep['p_obs'])} E={f(rep['E'])} "
            f"F={f(rep['F'])} F_ref={f(rep['F_ref'])} error={f(rep['error'])} bound={f(rep['bound'])}"
            + ("" if rep["passed"] else f" reasons={rep['reasons']}"))


# ---- the runs ---------------------------------------------------------------------------------------------------------

class View:
    """One run of a sequence: its reads, the state times t_i = i H, and freshness."""

    def __init__(self, run, t_settle=None):
        self.run = run
        self.m = run.m
        self.steps = run.log["steps"]
        self.H = run.m * run.tick_period_s  # Fraction
        self.reads = [(*s["gz_pos_enu"], *s["gz_q_wxyz"], *s["gz_lin_vel_enu"], *s["gz_ang_vel_enu"]) for s in self.steps]
        self.times = [float(i * self.H) for i in range(len(self.steps))]
        self.freshness = t4_rule.freshness(self.reads, self.times, t_settle)
        self.fresh = set(run_scenario.fresh_steps(run.log))
        assert self.freshness.index == max(self.fresh), "run_scenario.fresh_steps and t4_rule.fresh_index disagree"
        self.i = self.freshness.index
        self.t = self.times[self.i]
        self.n_ticks = self.i * self.m
        self.h_s = float(self.H)
        self.h_tick = float(run.tick_period_s)  # bitwise the plugin's tick_period_s: one correctly rounded division

    def ticks_at(self, i):
        return i * self.m

    def z_ned(self, i):
        return -self.steps[i]["gz_pos_enu"][2]

    def w_ned(self, i):
        return -self.steps[i]["gz_lin_vel_enu"][2]


def fresh_ok(views):
    for v in views:
        assert v.freshness.ok, f"m = {v.m}: {v.freshness.reasons}"


def freshness_meta(views):
    return {"m": [v.m for v in views], "index": [v.i for v in views], "t_s": [v.t for v in views],
            "ticks": [v.n_ticks for v in views], "stale_steps": [v.freshness.stale_steps for v in views]}


def run_seq(tmp_path_factory, name, scenario, hover_member=None, sdf_edit=None):
    return run_scenario.run_sequence(CARD, SCEN / f"{scenario}.yaml", None, "test", hover_member,
                                     tmp_path_factory.mktemp(name), PLUGIN_DIR, sdf_edit)


def motor_edit(motor, dshot):
    """The harness injection: motor `motor` (1..4) command `dshot` through the plugin's sil_override element."""
    pattern = rf'(<sil_override param="ol_dshot_m{motor}" type="i32">)[^<]*(</sil_override>)'

    def edit(text):
        out, n = re.subn(pattern, rf"\g<1>{dshot}\g<2>", text)
        assert n == 1, f"the world holds {n} sil_override elements of motor {motor}"
        return out

    return edit


def scenario_values(scenario):
    doc = scn.load(SCEN / f"{scenario}.yaml", CARD)
    v = scn.values(doc)
    return v["site_latitude_rad"], v["site_height_m"]


def finish(seq, verdicts, extra=None):
    """Write the verdicts into the sequence report's halving section."""
    halving = {name: rep for name, (_, rep) in verdicts.items()}
    if extra:
        halving.update(extra)
    run_scenario.write_report(seq.runs, halving, seq.report_path)
    text = Path(seq.report_path).read_text(encoding="utf-8")
    assert "\nhalving\n" in text and "not provided" not in text
    return halving


def assert_rule_fails(seq, views, verdicts, motor, dshot):
    """A negative control passes only if the rule fails, and fails on the rule itself."""
    fresh_ok(views)
    for r in seq.runs:
        assert r.sdf_edited, "the control's world was not edited"
        assert run_scenario.complete_trailer(r.log, r.iterations, r.m)
        assert all(t["dshot"][motor - 1] == dshot for t in r.log["ticks"]), f"motor {motor} was not commanded {dshot}"
    for name, (verdict, rep) in verdicts.items():
        assert not verdict.passed, f"{name}: the rule passed the control: {rep}"
        assert "non-finite input" not in verdict.reasons, f"{name}: failed on a non-finite input, not on the rule"
        assert any(x in verdict.reasons for x in RULE_REASONS_I), f"{name}: no (i) rule reason: {verdict.reasons}"
        assert any(x in verdict.reasons for x in RULE_REASONS_II), f"{name}: no (ii) rule reason: {verdict.reasons}"
        assert set(verdict.reasons) <= set(RULE_REASONS_I + RULE_REASONS_II), f"{name}: {verdict.reasons}"


# ---- free fall --------------------------------------------------------------------------------------------------------

def analyse_free_fall(seq):
    phi, h0 = scenario_values("free_fall")
    views = [View(r) for r in seq.runs]
    fresh_ok(views)
    refs = [reference.free_fall(v.t, phi, h0) for v in views]
    for v in views:
        assert v.z_ned(v.i) == v.steps[v.i]["body_pos_ned"][2], "logged plant body p_D is not the exact map of the gz read"
        assert v.w_ned(v.i) == v.steps[v.i]["body_vel_ned"][2]
    extra = reference.FREE_FALL_ROUNDINGS
    meta = freshness_meta(views)
    verdicts = {
        "p_D": apply_rule([v.z_ned(v.i) for v in views], [r.p for r in refs],
                          [t4_rule.floor(v.n_ticks, r.Q_p, extra) for v, r in zip(views, refs)], [r.F_p for r in refs],
                          meta),
        "v_D": apply_rule([v.w_ned(v.i) for v in views], [r.v for r in refs],
                          [t4_rule.floor(v.n_ticks, r.Q_v, extra) for v, r in zip(views, refs)], [r.F_v for r in refs],
                          meta),
    }
    return views, verdicts


@pytest.fixture(scope="module")
def free_fall(tmp_path_factory):
    seq = run_seq(tmp_path_factory, "free_fall", "free_fall")
    views, verdicts = analyse_free_fall(seq)
    return seq, views, verdicts, finish(seq, verdicts)


@pytest.fixture(scope="module")
def free_fall_control(tmp_path_factory):
    seq = run_seq(tmp_path_factory, "free_fall_control", "free_fall", sdf_edit=motor_edit(1, CONTROL_DSHOT))
    views, verdicts = analyse_free_fall(seq)
    return seq, views, verdicts, finish(seq, verdicts)


@needs_gz
@pytest.mark.parametrize("quantity", ["p_D", "v_D"])
def test_free_fall(free_fall, quantity, capsys):
    seq, views, verdicts, _ = free_fall
    verdict, rep = verdicts[quantity]
    _say(capsys, verdict_line("free_fall", quantity, rep), f"    report: {seq.report_path}")
    assert [r.m for r in seq.runs] == [4, 2, 1]
    assert verdict.passed, rep


@needs_gz
def test_free_fall_control_motor1_dshot48(free_fall_control, capsys):
    seq, views, verdicts, _ = free_fall_control
    _say(capsys, *[verdict_line("free_fall ctrl", q, rep) for q, (_, rep) in verdicts.items()])
    assert_rule_fails(seq, views, verdicts, 1, CONTROL_DSHOT)


# ---- rotation ---------------------------------------------------------------------------------------------------------

def analyse_rotation(seq):
    card = reference.load_card(CARD)
    views = [View(r) for r in seq.runs]
    fresh_ok(views)
    first = 1  # the first free step: step 0 reads zero rates, the initial rates are applied in its update
    refs, floors_l, floors_ke, live = [], [], [], []
    for v in views:
        ref = reference.rotation_reference(v.steps[first]["body_omega_frd"], card.inertia_diag)
        f_l, f_ke = reference.rotation_floors(v.n_ticks, ref)
        refs.append(ref)
        floors_l.append(f_l)
        floors_ke.append(f_ke)
        w0, wt = v.steps[first]["body_omega_frd"], v.steps[v.i]["body_omega_frd"]
        f_w = t4_rule.floor(v.n_ticks, max(abs(c) for c in w0))
        live.append({"m": v.m, "max_change_rad_s": max(abs(a - b) for a, b in zip(wt, w0)), "floor_rad_s": f_w})
    inv = [reference.rotation_invariants(v.steps[v.i]["body_omega_frd"], card.inertia_diag) for v in views]
    meta = freshness_meta(views)
    verdicts = {
        "L": apply_rule([x[0] for x in inv], [r.L for r in refs], floors_l, [r.F_ref_L for r in refs], meta),
        "KE": apply_rule([x[1] for x in inv], [r.KE for r in refs], floors_ke, [r.F_ref_KE for r in refs], meta),
    }
    return views, verdicts, live


@pytest.fixture(scope="module")
def rotation(tmp_path_factory):
    seq = run_seq(tmp_path_factory, "rotation", "rotation")
    views, verdicts, live = analyse_rotation(seq)
    return seq, views, verdicts, live, finish(seq, verdicts, {"liveness": {f"m{x['m']}": x for x in live}})


@pytest.fixture(scope="module")
def rotation_control(tmp_path_factory):
    seq = run_seq(tmp_path_factory, "rotation_control", "rotation", sdf_edit=motor_edit(1, CONTROL_DSHOT))
    views, verdicts, live = analyse_rotation(seq)
    return seq, views, verdicts, live, finish(seq, verdicts)


@needs_gz
@pytest.mark.parametrize("quantity", ["L", "KE"])
def test_rotation_invariant(rotation, quantity, capsys):
    seq, views, verdicts, live, _ = rotation
    verdict, rep = verdicts[quantity]
    _say(capsys, verdict_line("rotation", quantity, rep), f"    report: {seq.report_path}")
    assert [r.m for r in seq.runs] == [4, 2, 1]
    assert verdict.passed, rep


@needs_gz
def test_rotation_body_tumbles(rotation, capsys):
    _, _, _, live, _ = rotation
    _say(capsys, *[f"rotation liveness m={x['m']}: max |w_b(T) - w_b(first)| = {x['max_change_rad_s']:.6e} rad/s, "
                   f"floor {x['floor_rad_s']:.6e}" for x in live])
    for x in live:
        assert x["max_change_rad_s"] > x["floor_rad_s"], x


@needs_gz
def test_rotation_control_motor1_dshot48(rotation_control, capsys):
    seq, views, verdicts, live, _ = rotation_control
    _say(capsys, *[verdict_line("rotation ctrl", q, rep) for q, (_, rep) in verdicts.items()])
    assert_rule_fails(seq, views, verdicts, 1, CONTROL_DSHOT)


# ---- hover ------------------------------------------------------------------------------------------------------------

def last_double_fresh(view, t_settle):
    """The last step i whose read and whose predecessor's are both fresh, at a time after t_s; None if there is none."""
    for i in range(len(view.steps) - 1, 0, -1):
        if i in view.fresh and i - 1 in view.fresh and view.times[i - 1] > t_settle:
            return i
    return None


def omega_floor(card, dshot, h):
    """Bound on |w_log - w_ref| for every tick of a run: w_ref = w_D (1 - exp(-t/tau)) evaluated by reference.tick_held_omega
    (relative error <= TICK_OMEGA_ROUNDINGS u, so absolute <= 5 u w_D) against the plant's recursion
    w' = fl(c + fl(fl(w - c) r_hat)), c = w_D, r_hat = exp(-h/tau) as evaluated (r_hat = r (1 + eps), |eps| <= 3 u).
    With x_j = w_j - c (exact recursion x_{j+1} = r x_j, x_0 = -c) the rounded one obeys
      x^_{j+1} = r_hat x^_j (1 + theta) + eta,  |theta| <= 2 u (the subtraction and the product), |eta| <= u c (the add,
      |c + p| <= c),
    so d_j = x^_j - x_j satisfies |d_{j+1}| <= r |d_j| + (|eps| + |theta|) r^{j+1} c + |eta| and, with 1/(1 - r) <= tau/h + 1
    and max_j j r^j = tau / (e h), (u squared neglected)
      |d_j| <= c u [ (tau/h + 1) + (3 + 2) tau / (e h) ] ;
    plus the reference's own 5 u c: F_w = c u [ (tau/h + 1) + 5 tau / (e h) + 5 ]."""
    c = reference.esc_omega(card, dshot)
    u = t4_rule.U
    ratio = card.tau_s / h
    return c * u * (PLANT_ETA_ROUNDINGS * (ratio + 1.0)
                    + (PLANT_DECAY_ROUNDINGS + PLANT_THETA_ROUNDINGS) * ratio / math.e
                    + reference.TICK_OMEGA_ROUNDINGS)


def tick_thrust_check(run, card, dshot, phi, h0, hold=1):
    """The logged force of every TICK record of `run` against the reference's per-tick thrust (`hold` as in
    reference.tick_held_omega: 1 the plant's definition, 0 the start-of-tick negative control). The plant's force is
    F_z = R_zz (-T) + m g, T = sum_i k w_i^2 (the logged wrench, NED down component), with g = g(phi, h0 - p_D) at the
    plant body of the tick's host step (held over its m ticks) and R_zz from the logged plant attitude. The reference's
    is F_ref = m g - R_zz T_ref, T_ref = m a_j, a_j = reference.tick_held_thrust_accel, evaluated with the same g and R_zz.
    Per-tick floor, all first order in u:
      4 k F_w (2 w_ref + F_w) R_zz     the motor speed's bound (omega_floor) through T = 4 k w^2
      + THRUST_LOG_ROUNDINGS u T_ref    20 = the plant's k w w (2), the four-motor sum (3), the row product (1), the
                                        normalised rotation entry (5), and here a_j from w (3: k w, (k w) w, / m),
                                        the product with m (1), R_zz from the logged quaternion (5)
      + (2 x 80 + 2) u m g              the two gravity evaluations (80 u each, GRAVITY_ROUNDINGS: the L1 rule) and the
                                        two products m g
      + u |F_z|                         the final add.
    Also every logged rotor speed against w_ref within F_w. Returns the report; `ok` is True iff every tick is within its
    floor."""
    h, m = float(run.tick_period_s), run.m
    steps, ticks = run.log["steps"], run.log["ticks"]
    u = t4_rule.U
    f_w = omega_floor(card, dshot, h)
    worst = {"ratio": 0.0, "tick": None, "err": 0.0, "floor": 0.0}
    worst_w = {"err": 0.0, "tick": None}
    violations = 0
    for j, t in enumerate(ticks):
        assert t["tick"] == j
        st = steps[j // m]
        g = hover_mod.normal_gravity(phi, h0 - st["body_pos_ned"][2])
        qw, qx, qy, qz = st["body_q_wxyz"]
        r_zz = (qw * qw - qx * qx - qy * qy + qz * qz) / (qw * qw + qx * qx + qy * qy + qz * qz)
        w_ref = reference.tick_held_omega(card, dshot, j, h, hold)
        t_ref = reference.tick_held_thrust_accel(card, dshot, j, h, hold) * card.mass_kg
        f_ref = card.mass_kg * g - r_zz * t_ref
        f_log = t["force_ned"][2]
        err = abs(f_log - f_ref)
        floor = (r_zz * reference.N_ROTORS * card.thrust_coeff * f_w * (2.0 * w_ref + f_w) + THRUST_LOG_ROUNDINGS * u * t_ref
                 + (2 * reference.GRAVITY_ROUNDINGS + 2) * u * card.mass_kg * g + u * abs(f_log))
        if err > floor:
            violations += 1
        if err / floor > worst["ratio"]:
            worst = {"ratio": err / floor, "tick": j, "err": err, "floor": floor}
        for w in t["rotor_speed"]:
            if abs(w - w_ref) > worst_w["err"]:
                worst_w = {"err": abs(w - w_ref), "tick": j}
    return {"m": m, "n_ticks": len(ticks), "violations": violations, "worst_ratio": worst["ratio"],
            "worst_tick": worst["tick"], "worst_err_N": worst["err"], "worst_floor_N": worst["floor"],
            "omega_floor_rad_s": f_w, "omega_worst_err_rad_s": worst_w["err"], "omega_worst_tick": worst_w["tick"],
            "omega_ok": worst_w["err"] <= f_w, "ok": violations == 0 and worst_w["err"] <= f_w}


def analyse_hover(seq, dshot_ref):
    """Verdicts of p_D, v_D and a_D of a hover sequence against the hover reference of `dshot_ref` (the run's own
    command for the scenario, the D_lo command for the control of the D_lo run). p_D and v_D use the tick-held reference
    (reference.hover_state_tick_held, evaluated after n_ticks = i m ticks, t = n_ticks h): marv_plant holds the thrust
    4 k w(t_{j+1})^2 of the end-of-step motor state over tick j (marv_plant.h, 0003 items 7 and 8), which the continuous
    reference does not model (a_T t_tick / 2 in v). a_D is the settled acceleration, where the two agree to the floor."""
    phi, h0 = scenario_values("hover")
    card = reference.load_card(CARD)
    t_s, _, _ = reference.hover_settling(card, dshot_ref)
    views = [View(r, t_s) for r in seq.runs]
    fresh_ok(views)
    refs = [reference.hover_state_tick_held(v.n_ticks, v.h_tick, card, dshot_ref, phi, h0) for v in views]
    extra = reference.HOVER_ROUNDINGS
    fd = []
    for v in views:
        i = last_double_fresh(v, t_s)
        assert i is not None, f"m = {v.m}: no two consecutive fresh reads after t_s = {t_s}"
        t_mid = float((i - Fraction(1, 2)) * v.H)
        r_mid = reference.hover_state(t_mid, card, dshot_ref, phi, h0)
        r_i = reference.hover_state(v.times[i], card, dshot_ref, phi, h0)
        r_p = reference.hover_state(v.times[i - 1], card, dshot_ref, phi, h0)
        f_a = ((t4_rule.floor(v.ticks_at(i), r_i.Q_v, extra) + t4_rule.floor(v.ticks_at(i - 1), r_p.Q_v, extra)) / v.h_s
               + t4_rule.floor(0, r_mid.Q_a, FD_ROUNDINGS))
        fd.append({"i": i, "t_mid": t_mid, "a": (v.w_ned(i) - v.w_ned(i - 1)) / v.h_s, "ref": r_mid, "F": f_a})
    meta = freshness_meta(views)
    meta_a = dict(meta, index=[x["i"] for x in fd], t_s=[x["t_mid"] for x in fd],
                  ticks=[views[k].ticks_at(x["i"]) for k, x in enumerate(fd)])
    verdicts = {
        "p_D": apply_rule([v.z_ned(v.i) for v in views], [r.p for r in refs],
                          [t4_rule.floor(v.n_ticks, r.Q_p, extra) for v, r in zip(views, refs)], [r.F_p for r in refs],
                          meta),
        "v_D": apply_rule([v.w_ned(v.i) for v in views], [r.v for r in refs],
                          [t4_rule.floor(v.n_ticks, r.Q_v, extra) for v, r in zip(views, refs)], [r.F_v for r in refs],
                          meta),
        "a_D": apply_rule([x["a"] for x in fd], [x["ref"].a for x in fd], [x["F"] for x in fd],
                          [x["ref"].F_a for x in fd], meta_a),
    }
    return views, verdicts, t_s


def commanded(seq):
    return seq.runs[0].log["ticks"][0]["dshot"]


@pytest.fixture(scope="module")
def hover_runs(tmp_path_factory):
    out = {}
    for member in scn.HOVER_MEMBERS:
        seq = run_seq(tmp_path_factory, f"hover_{member}", "hover", member)
        d = commanded(seq)
        assert len(set(d)) == 1
        views, verdicts, t_s = analyse_hover(seq, d[0])
        phi, h0 = scenario_values("hover")
        card = reference.load_card(CARD)
        thrust = [tick_thrust_check(r, card, d[0], phi, h0) for r in seq.runs]
        out[member] = {"seq": seq, "views": views, "verdicts": verdicts, "t_s": t_s, "dshot": d[0], "thrust": thrust}
    return out


@pytest.fixture(scope="module")
def hover_checked(hover_runs):
    card = reference.load_card(CARD)
    lo, hi = hover_runs["lo"], hover_runs["hi"]
    phi, h0 = scenario_values("hover")
    card_doc, profile = gpc.load_linted(CARD, ROOT)
    plant_cfg, _ = gpc.config_from(card_doc, profile, CARD, ROOT)
    _, _, d_lo, d_hi = hover_mod.bracket_for_card(plant_cfg, phi, h0)
    a_lo = lo["verdicts"]["a_D"][1]["q"][-1]
    a_hi = hi["verdicts"]["a_D"][1]["q"][-1]
    ref_lo, ref_hi = (h["verdicts"]["a_D"][1]["q_ref"][-1] for h in (lo, hi))
    bracket = {
        "D_lo": d_lo, "D_hi": d_hi, "a_lo": a_lo, "a_hi": a_hi, "ref_a_lo": ref_lo, "ref_a_hi": ref_hi,
        "bound": reference.bracket_bound(card, d_lo, d_hi), "signs_ok": reference.bracket_sign_ok(a_lo, a_hi),
        "ref_signs_ok": reference.bracket_sign_ok(ref_lo, ref_hi),
        "magnitude_lo_ok": reference.bracket_magnitude_ok(a_lo, card, d_lo, d_hi),
        "magnitude_hi_ok": reference.bracket_magnitude_ok(a_hi, card, d_lo, d_hi),
    }
    for member, h in hover_runs.items():
        finish(h["seq"], h["verdicts"], {"bracket": bracket, "tick_thrust_check": {f"m{x['m']}": x for x in h["thrust"]}})
    return bracket


@pytest.fixture(scope="module")
def hover_control(tmp_path_factory, hover_runs):
    d_lo = hover_runs["lo"]["dshot"]
    seq = run_seq(tmp_path_factory, "hover_lo_control", "hover", "lo", sdf_edit=motor_edit(1, d_lo + 1))
    views, verdicts, _ = analyse_hover(seq, d_lo)
    finish(seq, verdicts)
    return seq, views, verdicts, d_lo + 1


def thrust_line(member, dshot, x):
    return (f"hover {member} D={dshot} thrust check m={x['m']}: {x['n_ticks']} ticks, violations {x['violations']}, worst "
            f"|F_log - F_ref| = {x['worst_err_N']:.3e} N at tick {x['worst_tick']} (floor there {x['worst_floor_N']:.3e} N, "
            f"ratio {x['worst_ratio']:.3e}); rotor speed worst {x['omega_worst_err_rad_s']:.3e} rad/s at tick "
            f"{x['omega_worst_tick']} (floor {x['omega_floor_rad_s']:.3e})")


@needs_gz
@pytest.mark.parametrize("member", ["lo", "hi"])
def test_hover_logged_thrust_is_the_reference_per_tick_thrust(hover_runs, member, capsys):
    """Before the rule: every tick of every run, the plant's logged force is the tick-held reference's thrust plus
    gravity within the derived per-tick floor, so the p_D and v_D reference is the plant's defined input."""
    h = hover_runs[member]
    _say(capsys, *[thrust_line(member, h["dshot"], x) for x in h["thrust"]])
    assert [r.m for r in h["seq"].runs] == [4, 2, 1]
    for r, x in zip(h["seq"].runs, h["thrust"]):
        assert x["n_ticks"] == r.iterations * r.m and x["n_ticks"] == len(r.log["ticks"])
        assert x["ok"], x


@needs_gz
@pytest.mark.parametrize("member", ["lo", "hi"])
def test_hover_thrust_check_control_start_of_tick_hold_fails(hover_runs, member, capsys):
    """Negative control of the cross-check: the start-of-tick reference w(t_j) is not the plant's input."""
    h = hover_runs[member]
    card = reference.load_card(CARD)
    phi, h0 = scenario_values("hover")
    for r in h["seq"].runs:
        x = tick_thrust_check(r, card, h["dshot"], phi, h0, hold=0)
        _say(capsys, thrust_line(f"{member} (start-of-tick hold)", h["dshot"], x))
        assert not x["ok"] and x["violations"] > 0 and x["worst_ratio"] > 1.0, x


@needs_gz
def test_hover_thrust_check_control_motor1_one_dshot_up_fails(hover_control, capsys):
    """Negative control: motor 1 one command up (the harness injection) changes the thrust, the check must see it."""
    seq, views, verdicts, dshot = hover_control
    card = reference.load_card(CARD)
    phi, h0 = scenario_values("hover")
    for r in seq.runs:
        x = tick_thrust_check(r, card, dshot - 1, phi, h0)
        _say(capsys, thrust_line(f"ctrl m1={dshot}", dshot - 1, x))
        assert not x["ok"] and x["violations"] > 0, x


@needs_gz
@pytest.mark.parametrize("member", ["lo", "hi"])
@pytest.mark.parametrize("quantity", ["p_D", "v_D", "a_D"])
def test_hover(hover_runs, hover_checked, member, quantity, capsys):
    h = hover_runs[member]
    verdict, rep = h["verdicts"][quantity]
    _say(capsys, verdict_line(f"hover {member} D={h['dshot']}", quantity, rep),
         f"    t_s = {h['t_s']:.6f} s; report: {h['seq'].report_path}")
    assert [r.m for r in h["seq"].runs] == [4, 2, 1]
    assert all(x["ok"] for x in h["thrust"]), "the logged per-tick thrust is not the reference's (see the thrust check)"
    assert verdict.passed, rep


@needs_gz
def test_hover_bracket(hover_runs, hover_checked, capsys):
    b = hover_checked
    _say(capsys, f"hover bracket D_lo={b['D_lo']} D_hi={b['D_hi']}: a(D_lo)={b['a_lo']:.9e} (reference {b['ref_a_lo']:.9e}), "
                 f"a(D_hi)={b['a_hi']:.9e} (reference {b['ref_a_hi']:.9e}), bound 4k(w_hi^2 - w_lo^2)/m={b['bound']:.9e}")
    assert hover_runs["lo"]["dshot"] == b["D_lo"] and hover_runs["hi"]["dshot"] == b["D_hi"]
    assert b["ref_signs_ok"], "the reference itself is not a bracket"
    assert b["signs_ok"], "a(D_lo) > 0 > a(D_hi) fails"
    assert b["magnitude_lo_ok"] and b["magnitude_hi_ok"], "|a| exceeds the bracket bound"


@needs_gz
def test_hover_control_motor1_one_dshot_up(hover_control, capsys):
    seq, views, verdicts, dshot = hover_control
    _say(capsys, *[verdict_line(f"hover ctrl m1={dshot}", q, rep) for q, (_, rep) in verdicts.items()])
    assert_rule_fails(seq, views, verdicts, 1, dshot)


# ---- the error form is the rule ---------------------------------------------------------------------------------------

@pytest.mark.parametrize("q, c, F, F_ref", [
    ([1.0 + 0.25, 1.0 + 0.125, 1.0 + 0.0625], 1.0, 0.0, 0.0),  # error halving: branch (ii)
    ([1.0 + 2.0 ** -40, 1.0 + 2.0 ** -40, 1.0 + 2.0 ** -40], 1.0, 2.0 ** -39, 0.0),  # E = 0 <= F: branch (i)
    ([1.0 + 0.25, 1.0 + 0.5, 1.0 + 1.0], 1.0, 0.0, 0.0),  # diverging: fails
    ([1.0 + 0.5, 1.0 + 0.5, 1.0 + 0.5], 1.0, 2.0 ** -40, 0.0),  # converged to the wrong value: fails
])
def test_error_form_is_the_rule_when_times_coincide(q, c, F, F_ref):
    direct = t4_rule.evaluate(q, c, F, F_ref)
    errors = [x - c for x in q]
    assert all(x + c == y for x, y in zip(errors, q)), "the shift must be exact for this test"
    shifted = t4_rule.evaluate(errors, 0.0, F, F_ref)
    a, b = direct.report(), shifted.report()
    for key in ("passed", "branch", "d", "rho", "p_obs", "E", "bound", "error", "F", "F_ref", "pass_i", "pass_ii", "reasons"):
        assert a[key] == b[key], key
    if a["q_hat"] is not None:
        assert a["q_hat"] - c == b["q_hat"]
