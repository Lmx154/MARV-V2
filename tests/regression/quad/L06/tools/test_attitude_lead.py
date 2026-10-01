"""L6 stage (c) attitude gains on the stage (c) rate loop (tools/card/attitude_lead.py; decision 0014 D7, commit 2; decision
0006 E's rule): the cross-check that the inner loop is the only change (today's PI, no chain, latency 0, reproduces
attitude.py) with its latency control, rule step 5' against the Jury test and the Nyquist limit with its unstable-gain
control, the closed form against the stepped model, and the f32 guard with its control.

Every acceptance test has a control that breaks it (core 7.2). Tolerances are derived on their lines. rate_lead.design runs
once per pytest session (conftest.py); the stage (c) attitude design once per module (about 30 s of plain Python).
"""

import cmath
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
def lead(l4, rate_lead_design):
    card, budget, scenario, _ = documents()
    return al.design(card, budget, scenario, CARD, al.lead_inner(rate_lead_design), l4)


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
    for design in (pi, lead):
        for i, (axis, name, loop) in enumerate(design["loops"]):
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
