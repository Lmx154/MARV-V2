"""L6 stage (c) attitude gains on the stage (c) rate loop (tools/card/attitude_lead.py; decision 0014 D7, commit 2; decision
0006 E's rule): the cross-check that the inner loop is the only change (today's PI, no chain, latency 0, reproduces
attitude.py) with its latency control, rule step 5' against the Jury test and the Nyquist limit with its unstable-gain
control, the closed form against the stepped model, the f32 guard with its control, k over the configuration set (rule
step 6') with the step-1'-only design as its control, the sensitivity peak Ms over the configuration set (budget Ms_max,
decision 0014 fifth round item 4) with k x 1.1 as its control, and att_yaw_t_cross over the configuration set (rule step 11')
with its control on the L5 T3 envelope's design model.

Every acceptance test has a control that breaks it (core 7.2). Tolerances are derived on their lines. rate_lead.design runs
once per pytest session (conftest.py), and so does the stage (c) attitude design over the set (about 30 s on 4
processes); the step-1'-only design runs once per module (about 20 s).
"""

import cmath
import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import attitude  # noqa: E402
import attitude_lead as al  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"

# Cross-check tolerance (PM in rad, crossover angle in rad, F relative): TOL of test_attitude_design.py and
# test_rate_lead.py, 2^-30. Both sides are exact to double rounding (observed 1.4e-13 rad on PM, 2.6e-13 on F).
TOL = 2.0 ** -30
# Step 5''s radius against attitude.py's Jury radius on the 5-state loop: 2^-18 (3.8e-6). The Jury radius resolves rho to
# about 1e-6 here (on the three axes, whose loops differ by f32 gain rounding only, it spreads by 1.2e-6 at nominal), and
# rho_K's excess over rho is about ln(C)/2^24, below 1e-6 for C < e^16.
RADIUS_TOL = 2.0 ** -18
# The stability control: gains this factor below and above the Nyquist limit (the gain at the first grid point where arg L
# passes -pi on the continuous branch).
LIMIT_FACTOR = 1.1
# The Ms control: the P gain times this labelled factor (the x 1.1 of the L6 gain controls) must exceed Ms_max at a loop of the set.
MS_GAIN_CONTROL = 1.1
# Frequencies (rad/s) of the closed-form against stepped-model check: from below the attitude crossover to the chain's band.
MODEL_CHECK_RAD_S = (1.0, 4.0, 40.0, 400.0)


def documents():
    return rl.load(CARD, BUDGET, SCENARIO, PROFILE)


@pytest.fixture(scope="module")
def l4():
    card, budget, scenario, _ = documents()
    return rate.design(card, budget, scenario, CARD)


@pytest.fixture(scope="module")
def today(l4):
    card, budget, scenario, _ = documents()
    return attitude.design(card, budget, scenario, CARD, l4)


def pi_design(l4, latency):
    card, budget, scenario, _ = documents()
    return al.design(card, budget, scenario, CARD, al.pi_inner(l4, scenario["rate_loop_divisor"]["value"], latency), l4)


@pytest.fixture(scope="module")
def pi(l4):
    return pi_design(l4, 0)


@pytest.fixture(scope="module")
def lead(attitude_lead_design):
    return attitude_lead_design


@pytest.fixture(scope="module")
def alone(l4, rate_lead_design):
    """The rule on step 1''s configuration alone (notches at omega_th, the profile's latency): the rule before owner decision
    1 of decision 0014's third round, the controls' design."""
    card, budget, scenario, _ = documents()
    return al.design(card, budget, scenario, CARD, al.lead_inner(rate_lead_design, over_set=False), l4)


def pm_residual(a, b):
    """Largest |PM| and |crossover angle| differences between two design details, loop for loop."""
    assert [(x[0], x[1]) for x in a] == [(x[0], x[1]) for x in b]
    return max(abs(x[2] - y[2]) for x, y in zip(a, b)), max(abs(x[3] - y[3]) for x, y in zip(a, b))


def nyquist_limit(loop):
    """The gain at which arg L = arg F - (pi/2 + theta/2) first passes -pi on the grid, 1/|F/(z - 1)| there."""
    for theta, h, phase in zip(attitude._GRID.theta, loop.h, loop.unwrapped):
        if phase - math.pi / 2 - theta / 2 < -math.pi:
            return 1 / h
    pytest.fail("no phase crossover on the grid")


# ---- the cross-check: today's PI through this rule is attitude.py ---------------------------------------------------


def test_todays_pi_through_this_rule_reproduces_attitude_py(pi, today):
    assert pi["k32"] == today["k32"] and pi["k"] == today["k"]
    d_pm, d_theta = pm_residual(pi["final"]["detail"], today["final"]["detail"])
    assert d_pm <= TOL and d_theta <= TOL, (d_pm, d_theta)
    assert pi["t_cross"] == today["t_cross"] and pi["crossings"] == today["crossings"]
    assert (pi["w32"], pi["alpha_min"], pi["N"]) == (today["w32"], today["alpha_min"], today["N"])


def test_control_one_sample_of_inner_latency_breaks_the_cross_check(l4, today):
    late = pi_design(l4, 1)
    d_pm, _ = pm_residual(late["final"]["detail"], today["final"]["detail"])
    assert late["k32"] != today["k32"] and d_pm > TOL


def test_control_one_more_sample_of_latency_moves_the_stage_c_margins(lead, l4, rate_lead_design):
    late = al.build_loops(l4, al.lead_inner(rate_lead_design, rate_lead_design["model"].latency + 1))
    k, w32 = lead["k32"], lead["w32"]
    moved = max(abs(loop.margin(attitude.axis_k(axis, k, w32))[1] - row[2])
                for (axis, _, loop), row in zip(late, lead["final"]["detail"]))
    assert moved > TOL


# ---- rule step 5': the radius against the Jury test and the Nyquist limit -------------------------------------------


def test_the_squaring_radius_agrees_with_the_jury_radius_on_the_5_state_loop(pi, today):
    assert al.RADIUS_SQUARINGS > 0
    for mine, jury in zip(pi["final"]["detail"], today["final"]["detail"]):
        assert mine[4] is not None and jury[4] is not None, (mine, jury)
        assert abs(mine[4] - jury[4]) <= RADIUS_TOL, (mine[:2], mine[4], jury[4])
    for axis, _, loop in pi["loops"]:
        assert loop.certificate(attitude.axis_k(axis, pi["k32"], pi["w32"]))[0] > 0


def test_control_a_gain_past_the_stability_limit_is_unstable_and_one_below_is_certified(pi, lead, l4):
    jury_loops = attitude.build_loops(l4, 1)
    # The stage (c) side runs on step 1''s 15 loops (notches at omega_th, the profile's latency), the loops this check had
    # before the configuration set: it checks the certificate against the Nyquist limit, and the set's other loops are the
    # same construction (their certificates at the design's gains are asserted below).
    for design, loops in ((pi, pi["loops"]), (lead, al.build_loops(l4, lead["inner"]))):
        for i, (axis, name, loop) in enumerate(loops):
            limit = nyquist_limit(loop)
            below, above = limit / LIMIT_FACTOR, limit * LIMIT_FACTOR
            assert loop.stable(below) and loop.radius(below) < 1, (axis, name)
            assert not loop.stable(above) and loop.radius(above) is None, (axis, name)
            assert loop.estimate(above)[0] > 1, (axis, name)
            if design is pi:
                assert jury_loops[i][2].stable(below) and not jury_loops[i][2].stable(above), (axis, name)


# ---- the stage (c) design --------------------------------------------------------------------------------------------


def test_the_closed_form_and_the_stepped_model_are_the_same_loop(lead, pi):
    """The margin (closed form F) and the proof (stepped matrices) describe one loop: (z - 1) e_theta^T (zI - A)^-1 B = F."""
    for design in (pi, lead):
        for axis, name, loop in design["loops"]:
            for w in MODEL_CHECK_RAD_S:
                theta = w * loop.t
                z = cmath.exp(1j * theta)
                g = attitude.csolve([[(z if i == j else 0) - loop.a[i][j] for j in range(loop.n)] for i in range(loop.n)],
                                    loop.b)[al.THETA]
                f = loop.resp(theta)
                assert abs((z - 1) * g - f) <= TOL * abs(f), (axis, name, w)


def test_the_stage_c_design_keeps_the_rate_independent_outputs_and_every_loop_is_certified(lead, today):
    assert lead["N"] == 1 and lead["inner"].stages and lead["inner"].latency >= 1
    assert (lead["w32"], lead["alpha_min"]) == (today["w32"], today["alpha_min"])
    for (axis, name, loop), row in zip(lead["loops"], lead["final"]["detail"]):
        kk = attitude.axis_k(axis, lead["k32"], lead["w32"])
        assert loop.certificate(kk)[0] > 0 and row[4] is not None and row[4] < 1, (axis, name)
        assert row[2] is not None and row[2] >= lead["inputs"]["PM_min"], (axis, name)


def test_the_f32_guard_holds_and_every_point_above_the_chosen_one_fails_it(lead):
    f, pm_min = lead["final"], lead["inputs"]["PM_min"]
    assert f["pm_worst"] >= pm_min + f["delta_num"]
    above = f["history"][f["history"].index(f["k"]) + 1:]
    assert above or not f["stepped_down"]
    for k in above:
        worst, _ = attitude.pm_worst(lead["loops"], rate.r32(k), lead["w32"])
        assert worst < pm_min + f["delta_num"], k
    assert not attitude.feasible(lead["loops"], f["bracket"][1], pm_min)


# ---- k over the configuration set (rule step 6') ----------------------------------------------------------------------


def pm_or_floor(loop, k):
    """The loop's PM at gain k, -inf without a unique crossover (attitude.pm_worst's convention)."""
    pm = loop.margin(k)[1]
    return -math.inf if pm is None else pm


def test_k_is_the_sup_rule_over_the_configuration_set_and_the_step_1_design_violates_pm_min_in_it(lead, alone):
    levels, cfgs = lead["levels"], lead["levels"][-1]["configs"]
    inner, pm_min = lead["inner"], lead["inputs"]["PM_min"]
    # The set of the final level: rate_lead's configurations, step 1''s point first (notches at omega_th, the profile's
    # latency), notches bypassed (the low-pass alone) and latency 0 among them; 15 loops each, in the design's detail.
    assert [c[0] for c in cfgs] == [c[0] for c in inner.configs(levels[-1]["level"])]
    assert (cfgs[0][2], cfgs[0][3]) == (inner.latency, inner.stages) and cfgs[0][0][0] == 0.0
    assert any(c[0][0] is None and len(c[3]) == 1 for c in cfgs) and {c[2] for c in cfgs} == {inner.latency, 0}
    corners = rate.corner_list(lead["inputs"]["tau"], lead["inputs"]["b_tau"], lead["inputs"]["b_J"])
    assert len(lead["loops"]) == len(lead["final"]["detail"]) == len(cfgs) * len(al.AXES) * len(corners)
    # Resolution (core 7.5): the last level's halving adds no configuration with a PM below its PM_worst at its k; every
    # earlier level's did (else the rule would have stopped there).
    assert levels[-1]["added"] and levels[-1]["added_worst"] >= levels[-1]["pm_worst"] == lead["pm_worst"]
    assert all(lv["added_worst"] < lv["pm_worst"] for lv in levels[:-1])
    # The design binds outside step 1''s configuration: the attitude loop's worst is not omega_th (the lead's correction).
    worst = min(lead["final"]["detail"], key=lambda x: x[2])
    assert not worst[1].startswith(lead["flight"]), worst[:2]
    # Control (i): the rule on step 1''s configuration alone (before owner decision 1) gives a larger k, and that k puts
    # the set below PM_min at a configuration (margins at the f32 gains, yaw at its effective gain).
    k_old = alone["k32"]
    assert alone["levels"] is None and k_old > lead["k32"]
    old = [(pm_or_floor(loop, attitude.axis_k(axis, k_old, lead["w32"])), axis, name) for axis, name, loop in lead["loops"]]
    below = [x for x in old if x[0] < pm_min]
    assert below, min(old)
    print(f"k32 over the set {lead['k32']!r} (binding {worst[0]} {worst[1]}, {math.degrees(worst[2])!r} deg); "
          f"step 1' alone {k_old!r}: {len(below)} loops below PM_min, worst {math.degrees(min(old)[0])!r} deg at "
          f"{min(old)[1]} {min(old)[2]}")


# ---- att_yaw_t_cross over the configuration set (rule step 11') ------------------------------------------------------------

T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"


def t3_oracle():
    spec = importlib.util.spec_from_file_location("attitude_t3_oracle", T3_REFERENCE / "attitude_t3_oracle.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def envelope_release(oracle, p, t_cross, s, t):
    """(lock execution, fallback execution, lock_by) of the L5 T3 envelope's yaw-release design model (attitude_t3_oracle
    yaw_member_run: notches bypassed, latency 0, the stage (c) T4 configuration) for the box member (s, t) with t_cross."""
    su = oracle.Setup(dict(p, att_yaw_t_cross=t_cross))
    h = su.segment_length(oracle.HORIZON_TAUS / su.cfg.kp)
    run = oracle.yaw_member_run(su, s, t, h, 1.0, 0.0)
    return run["a_l"], run["a_fb"], run["lock_by"]


def test_t_cross_is_the_latest_crossing_over_every_chain_configuration_and_the_step_1_loop_alone_is_too_early(
        lead, alone):
    # Every configuration evaluated (the final level's halving, rule step 6'), step 1''s point first.
    evaluated = lead["inner"].configs(lead["levels"][-1]["level"] + 1)
    labels = [c[1] for c in evaluated]
    corners = len(rate.corner_list(lead["inputs"]["tau"], lead["inputs"]["b_tau"], lead["inputs"]["b_J"]))
    assert len(lead["crossings"]) == corners * len(evaluated)
    assert [name.rsplit(": ", 1)[0] for name, _, _ in lead["crossings"]] == [x for x in labels for _ in range(corners)]
    assert lead["t_cross"] == rate.r32_up(max(t for _, _, t in lead["crossings"]))
    # Control: the step-1' loop alone (the rule before the lead decision) is the first configuration's latest crossing, and
    # it is earlier than the rule's t_cross.
    assert alone["t_cross"] == rate.r32_up(max(t for _, _, t in lead["crossings"][:corners]))
    assert alone["t_cross"] < lead["t_cross"]
    print(f"att_yaw_t_cross {lead['t_cross']!r} s; the step-1' loop alone {alone['t_cross']!r} s")
    # On the L5 T3 envelope's design model, at the box member whose crossing is the latest (J+, tau-): with the rule's
    # t_cross the release locks at its crossing, before the fallback; with the control's, the fallback pre-empts it.
    oracle = t3_oracle()
    p = oracle.read_inputs(T3_REFERENCE / "attitude_t3_inputs.txt")
    for axis, (kp, ki, kd, tf) in zip(al.AXES, lead["inner"].axes32):
        assert (p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"], p[f"rate_d_filter_tau_{axis}"]) == (
            kp, ki, kd, tf), "the T3 fixture is not the live stage (c) rate design"
    assert p["att_kp"] == lead["k32"] and p["att_yaw_t_cross"] == lead["t_cross"]
    a_l, a_fb, by = envelope_release(oracle, p, lead["t_cross"], 1.0, -1.0)
    assert by == "crossing" and a_l < a_fb, (a_l, a_fb, by)
    a_l, a_fb, by = envelope_release(oracle, p, alone["t_cross"], 1.0, -1.0)
    assert by == "fallback" and a_l == a_fb, (a_l, a_fb, by)


# ---- the sensitivity peak over the configuration set -------------------------------------------------------------------


def sensitivity_peak(loop, kk):
    """(Ms, peak frequency in rad/s) of the attitude loop at the linear gain kk: the maximum over theta of |S| = |1/(1 + L)|,
    L = kk F(e^{j theta})/(z - 1), F = LeadLoop.resp and z - 1 = 2j sin(theta/2) e^{j theta/2} (the exact form of
    attitude.Loop.margin). attitude.py and attitude_lead.py have no sensitivity function: the grid is attitude's (2047 points,
    log-spaced to the Nyquist angle) and the peak is refined around the largest grid point by golden section,
    rate_lead.GOLDEN_ITERATIONS steps, as rate_lead.Model.sensitivity does."""
    def s(th):
        zm1 = 2j * math.sin(th / 2) * cmath.exp(0.5j * th)
        return 1 / abs(1 + kk * loop.resp(th) / zm1)

    grid = attitude._GRID.theta
    vals = [s(th) for th in grid]
    i = max(range(len(grid)), key=vals.__getitem__)
    lo, hi = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    g = (math.sqrt(5) - 1) / 2
    x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
    f1, f2 = s(x1), s(x2)
    for _ in range(rl.GOLDEN_ITERATIONS):
        if f1 >= f2:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - g * (hi - lo)
            f1 = s(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + g * (hi - lo)
            f2 = s(x2)
    ms, theta = max((vals[i], grid[i]), (f1, x1), (f2, x2))
    return ms, theta / loop.t


def test_ms_is_within_ms_max_over_the_configuration_set_and_k_times_1_1_exceeds_it(lead, rate_lead_design):
    ms_max = rate_lead_design["inputs"]["Ms_max"]
    # The set evaluated (as the test of rule step 11' below): the final level's halving (level L + 1), 15 loops each.
    cfgs = lead["inner"].configs(lead["levels"][-1]["level"] + 1)
    rows, _ = al.set_loops(lead["rate"], lead["inner"], cfgs)
    corners = rate.corner_list(lead["inputs"]["tau"], lead["inputs"]["b_tau"], lead["inputs"]["b_J"])
    assert len(rows) == len(cfgs) * len(al.AXES) * len(corners)
    k, w32 = lead["k32"], lead["w32"]
    worst = (0.0,)
    for axis, name, loop in rows:
        kk = attitude.axis_k(axis, k, w32)
        ms, w = sensitivity_peak(loop, kk)
        # Cross-check of the helper on the loop's own margin: at the crossover |1 + L| = 2 sin(PM/2), so Ms >= 1/(2 sin(PM/2)).
        _, pm, _ = loop.margin(kk)
        assert ms >= 1 / (2 * math.sin(pm / 2)), (axis, name, ms, pm)
        worst = max(worst, (ms, w, axis, name))
    assert worst[0] <= ms_max, worst
    # Control: k x 1.1 puts the worst loop (and so the set) above Ms_max.
    over = sensitivity_peak(next(loop for axis, name, loop in rows if (axis, name) == worst[2:]),
                            attitude.axis_k(worst[2], k * MS_GAIN_CONTROL, w32))
    assert over[0] > ms_max, over
    print(f"Ms over {len(rows)} loops of {len(cfgs)} configurations: worst {worst[0]!r} at {worst[2]} {worst[3]}, "
          f"{worst[1]!r} rad/s (Ms_max {ms_max!r}); k x {MS_GAIN_CONTROL}: {over[0]!r}")
