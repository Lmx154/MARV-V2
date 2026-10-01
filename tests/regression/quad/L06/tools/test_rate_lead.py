"""L6 stage (c) rate-loop gain rule (tools/card/rate_lead.py; decision 0014 "the design", owner decisions 3, 6 and 8): the
cross-check that the rule extends rate.py (N = 1, H = 1, latency 0), N* on the floors with its control, the noise model's
linearity and the T_ff control, the physical J corners, and the f32 guard with its control.

Every acceptance test has a control that breaks it (core 7.2). Tolerances are derived on their lines. The committed-card
design runs once (module fixture, about 45 s of plain Python).
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_imu_config as gic  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402
import schema  # noqa: E402
import test_rate_lead_checks as checks  # noqa: E402  (this directory; pytest's default import mode puts it on sys.path)

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"

# Cross-check tolerance: test_rate_design.py's TOL (2^-30 rad), also test_gyro_chain_design.py's cross-check 1. Both
# evaluators are exact to double rounding (observed 3e-14 rad and 2e-14 relative on the gains); 2^-30 leaves four decades.
TOL = 2.0 ** -30
# The T_ff control's factor (the packet's labelled value): 0.9 T_ff must break the budget.
T_FF_CONTROL = 0.9
# The noise linearity control's factor: the IMU noise density x 1.1.
DENSITY_CONTROL = 1.1
# The f32 guard control: a gain perturbed by 1e-3 relative in the direction that crosses the floors (up: the design sits
# on the sup boundary of the crossover, so more gain moves the binding corner past PM_min and Ms_max).
GUARD_CONTROL = 1e-3


def documents():
    return rl.load(CARD, BUDGET, SCENARIO, PROFILE)


@pytest.fixture(scope="module")
def lead(rate_lead_design):
    return rate_lead_design


@pytest.fixture(scope="module")
def l4():
    card, budget, scenario, _ = documents()
    return rate.design(card, budget, scenario, CARD)


# ---- the cross-check: N = 1, H = 1, latency 0 is rate.py ------------------------------------------------------------


def test_n1_without_chain_or_latency_reproduces_rate_py(l4):
    inp = l4["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    divisor = schema.load_yaml(SCENARIO)["rate_loop_divisor"]["value"]
    model = rl.Model(l4["T"] / divisor, divisor, 0, None, corners, l4["a"], inp["inertia"])
    sup = rl.sup_crossover(model, 1.0, inp["PM_min"])
    # The same sup rule, decision for decision: rate.py's scan bracket, bisection count and final width, bit for bit.
    assert sup["scan_bracket"] == l4["scan_bracket"]
    assert sup["bisections"] == l4["bisections"]
    assert sup["width"] == l4["bracket_width"]
    # rate.py's chosen point is on this rule's history (its guard, PM_f32 >= PM_min + delta_num on the LTI loop, stepped
    # down one point); the guards differ (rule step 8: the firmware loop, owner decision 6), the sup rule does not.
    assert l4["stepped_down"] and sup["history"][-2] == l4["omega_c"]
    # Gains and margins at rate.py's point.
    g = model.gains(l4["omega_c"], 1.0)
    assert g[2] == 0.0
    assert abs(g[0] / l4["kappa_p"] - 1) <= TOL and abs(g[1] / l4["kappa_i"] - 1) <= TOL
    for axis, (kp, ki, kd, _) in zip(rate.AXES, rl.f32_axes(model, g)):
        assert (kp, ki, kd) == (l4["axes"][axis]["kp"], l4["axes"][axis]["ki"], 0.0)
    mine = model.margins(g)
    assert len(mine) == len(l4["margins"]) == 5
    for (name, pm, wx, xs), (rname, rpm, rwx, _) in zip(mine, l4["margins"]):
        assert name == rname and len(xs) == 1
        assert abs(pm - rpm) <= TOL, (name, pm - rpm)
        assert abs(wx / rwx - 1) <= TOL, (name, wx / rwx - 1)
    # The step response of rule step 11 reduces to rate.py's rise_time sample for sample.
    for (name, jt, tau), (rname, t63) in zip(corners, l4["t63"]):
        assert name == rname and rl.rise_time(model, g, jt, tau) == t63


def test_cross_check_control_one_sample_of_latency_breaks_it(l4):
    inp = l4["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    model = rl.Model(l4["T"] / 2, 2, 1, None, corners, l4["a"], inp["inertia"])
    g = model.gains(l4["omega_c"], 1.0)
    # A pure delay moves the phase, not |L| (K changes by 1e-12 only, through the aliasing sum): the margins are the control.
    for (_, pm, _, _), (_, rpm, _, _) in zip(model.margins(g), l4["margins"]):
        assert abs(pm - rpm) > 1000 * TOL


# ---- N* -------------------------------------------------------------------------------------------------------------


def test_n_star_sits_on_the_floors_and_one_step_up_violates(lead):
    i, model = lead["inputs"], lead["model"]
    pm_min, ms_max = i["PM_min"], i["Ms_max"]
    # Root tolerance of PM = PM_min: the sup bracket's infeasible end hi has PM below PM_min, so PM(w_c) - PM_min is at
    # most |dPM/dw| (hi - w_c) (w_c the guarded point, at or below the feasible end). The slope by a central difference of
    # step 2^-20 w_c (rate.py's DERIVATIVE_LOG2_STEP); x 2 covers curvature over a bracket of 1e-7 relative.
    n, w = lead["n_star"], lead["omega_c"]
    hi = lead["sup"]["omega"] + lead["sup"]["width"]
    step = w * 2.0 ** rate.DERIVATIVE_LOG2_STEP
    slope = abs(rl.worst(model.margins(model.gains(w + step, n)))[1]
                - rl.worst(model.margins(model.gains(w - step, n)))[1]) / (2 * step)
    gap = lead["pm_worst"][1] - pm_min
    assert 0 <= gap <= 2 * slope * (hi - w), (gap, slope, hi - w)
    assert lead["ms_worst"][1] <= ms_max
    assert lead["noise_floor_max"] <= lead["budget_n"]
    # test_rate_lead_checks.py evaluates the rule's gains at these values without the search: they must be its output.
    assert (n, w, lead["t_ff"]) == (checks.N_STAR, checks.OMEGA_C, checks.T_FF)
    # The fixture ran the N search in worker processes (default procs): its N* record is the serial evaluation's, exactly.
    assert rl.evaluate_n(model, lead["noise"], lead["n_lo"]["n"], pm_min, lead["ops"]) == lead["n_lo"]
    # The bisection bracket is within the labelled step, so N* (1 + step) is past the boundary: the control.
    assert lead["n_hi"]["n"] / lead["n_lo"]["n"] <= 1 + rl.N_REL_STEP
    up = rl.evaluate_n(model, lead["noise"], n * (1 + rl.N_REL_STEP), pm_min, lead["ops"])
    assert up["ms"] > ms_max or up["floor"] > lead["budget_n"]
    # The rule's monotonicity premise held on the grid (design() refuses otherwise), and the grid reaches an octave of N
    # past the bracket (N_MONOTONE_EXTRA points of 2^(1/4)), so past 2 N*.
    grid = lead["grid"]
    assert all(b["ms"] >= a["ms"] and b["floor"] >= a["floor"] for a, b in zip(grid, grid[1:]))
    assert grid[-1]["n"] >= 2 * n


# ---- noise ----------------------------------------------------------------------------------------------------------


def test_noise_is_linear_in_the_density_and_t_ff_is_the_smallest_within_budget(lead):
    i, noise = lead["inputs"], lead["noise"]
    kd32, tf32 = [a[2] for a in lead["axes32"]], lead["axes32"][0][3]
    sc = schema.load_yaml(SCENARIO)
    num, den = sc["tick_period_num_us"]["value"], sc["tick_period_den"]["value"]
    si, _, _ = gic.imu_config(PROFILE)
    assert i["sigma_d"] == gic.derived_report(si, num, den)[0]["gyro"][4]
    scaled = dict(si, gyro_noise_density=si["gyro_noise_density"] * DENSITY_CONTROL)
    sigma = gic.derived_report(scaled, num, den)[0]["gyro"][4]
    w0, fw = lead["noise_max_w0"], (lead["dts"], -rl.LIBM_EXP_ULPS)   # the firmware model T_ff is set on (rule step 9)
    base = noise.rms(None, w0, kd32, tf32, lead["t_ff"], fw=fw)
    up = noise.rms(None, w0, kd32, tf32, lead["t_ff"], sigma_d=sigma, fw=fw)
    for key in ("d", "ff", "combined"):
        for a, b in zip(base[key], up[key]):
            assert b == pytest.approx(DENSITY_CONTROL * a, rel=1e-12)   # a product of doubles: rounding only

    def combined(t_ff):
        return max(max(noise.rms(sp, w, kd32, tf32, t_ff, fw=fw)["combined"]) for sp, w in lead["ops"])

    assert combined(lead["t_ff"]) <= lead["budget_n"]
    assert combined(T_FF_CONTROL * lead["t_ff"]) > lead["budget_n"]


# ---- the physical J corners -----------------------------------------------------------------------------------------


def test_physical_corners_are_physical_and_keep_the_common_scale_ones():
    card, budget, _, _ = documents()
    j0 = card["inertia_diag"]["value"]
    b = budget["inertia_robustness_band"]["value"]
    tol = rl.CORNER_TOL_ULPS * sys.float_info.epsilon * sum(j0)
    vs = rl.physical_corners(j0, b)
    pts = [v["J"] for v in vs]
    for p in pts:
        assert rl.triangle_ok(p, tol), p
        assert all((1 - b) * j - tol <= x <= (1 + b) * j + tol for x, j in zip(p, j0)), p
    bad = tuple(f * j for f, j in zip((1 - b, 1 - b, 1 + b), j0))     # (0.5, 0.5, 1.5) J
    assert not rl.triangle_ok(bad, tol) and bad not in pts
    lo, hi = tuple((1 - b) * j for j in j0), tuple((1 + b) * j for j in j0)
    assert lo in pts and hi in pts
    assert {v["J"] for v in vs if v["common_scale"]} == {lo, hi}
    # Every box vertex that satisfies the inequalities is kept; the other box vertices are not.
    for signs in rl.SIGN_PATTERNS:
        corner = tuple((1 + s * b) * j for s, j in zip(signs, j0))
        assert (corner in pts) == rl.triangle_ok(corner, tol), corner


# ---- the f32 guard --------------------------------------------------------------------------------------------------


def test_f32_guard_passes_and_a_gain_beyond_it_fails(lead):
    i, model = lead["inputs"], lead["model"]
    axes32 = lead["axes32"]
    assert all(rate.r32(x) == x for a in axes32 for x in a)
    assert lead["dts"] == [312, 313]     # the SIL's stamps at 625/4 us, D = 2: the executions alternate (rule step 8)
    rows = lead["guard"]
    assert len(rows) == len(axes32)
    assert rl.guard_passes(rows, i["PM_min"], i["Ms_max"])
    for idx in range(len(axes32)):
        bad = [list(a) for a in axes32]
        bad[idx][0] *= 1 + GUARD_CONTROL
        assert not rl.guard_passes(rl.guard(model, [tuple(a) for a in bad], lead["dts"]), i["PM_min"], i["Ms_max"])
