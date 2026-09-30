"""The acro checks by segment without gz (tools/sim/run_l4.py acro_segment_kinds, acro_phases, rest_bound,
recovery_evaluation; Luis's Option 1 and recovery ruling, 2026-09-30). L04 tests, host-only.

Membership is derived from the committed scenarios/quad/L04/acro.yaml, planned against the build's parameter values as
the scenario register gives them (tick, divisor, rate_max as float32) and the composition register's segment capacity
(its l4_seg<k>_t_us entries), so no build is needed. Each refusal of acro_phases is a negative control: a mutation of
the committed script that the rule does not cover must be refused.

The recovery predicate on synthetic traces. Scenario test values, each chosen for one case: Z_TRACE (a decaying
|w| bound, rad/s, one value per execution), F_TEST (a widening, rad/s), STUCK (a rate above every Z + F), SPIKE (an
excursion above Z + F at one execution) and ENV_LO / ENV_HI (an envelope that straddles zero, then one that does not).
"""

import dataclasses
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l4_scenario as l4s  # noqa: E402
import run_l4  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L04" / "acro.yaml"
REGISTER = schema.load_yaml(ROOT / "design" / "scenario_values.yaml")
COMPOSITION_REGISTER = ROOT / "fw" / "compositions" / "l4_rate_scripted" / "params" / "l4_rate_scripted_register.yaml"
AXES = ("roll", "pitch", "yaw")
Z_TRACE = [4.0, 2.0, 1.0, 0.5, 0.25, 0.125]
F_TEST = 0.01
STUCK = 5.0
SPIKE = 0.75
ENV_LO = [-2.0, 0.5]
ENV_HI = [1.0, 3.0]


@pytest.fixture(scope="module")
def plan():
    params = {k: REGISTER[k]["value"] for k in ("tick_period_num_us", "tick_period_den", "rate_loop_divisor")}
    for a in AXES:
        params[f"rate_max_{a}"] = run_l4.r32(REGISTER[f"rate_max_{a}"]["value"])
    with open(COMPOSITION_REGISTER, encoding="utf-8") as f:
        params.update({k: 0 for k in yaml.safe_load(f) if run_l4.SEGMENT_T_US.match(k)})
    return run_l4.plan_acro(l4s.load_acro(SCENARIO), params, CARD)


# ---- segment membership ---------------------------------------------------------------------------------------------

def test_acro_segment_kinds_follow_the_sticks(plan):
    sticks = [seg[3] for seg in plan.segments]
    kinds = run_l4.acro_segment_kinds(plan)
    assert len(kinds) == len(sticks)
    prev = (0, 0, 0)
    for stick, kind in zip(sticks, kinds):
        axes = [a for a, x in enumerate(stick) if x != 0]
        if not axes:
            assert kind == "zero"
        elif len(axes) > 1:
            assert kind == "combined"
        else:
            assert kind == ("reversal" if prev[axes[0]] * stick[axes[0]] < 0 else "single")
        prev = stick
    # the committed script has every kind, the reversal after the first roll step and one combined segment
    assert kinds == ["single", "reversal", "single", "single", "zero", "combined", "zero"]


def test_acro_phases_of_the_committed_script(plan):
    phases, table = run_l4.acro_phases(plan)
    assert len(phases) == plan.end_execution + 1
    starts = [seg[0] for seg in plan.segments]
    combined = next(i for i, row in enumerate(table) if row[1] == "combined")
    first_c, last_c = starts[combined], starts[combined + 1] - 1
    assert all(ph == "linear" for ph in phases[:first_c])  # the settle and every segment before the combined one
    assert all(ph == "combined" for ph in phases[first_c:last_c + 1])
    assert all(ph == "recovery" for ph in phases[last_c + 1:])
    assert table[combined + 1][1:] == ("zero", last_c + 1, plan.end_execution, "recovery")
    for number, kind, first, last, phase in table:
        assert first == starts[number - 1] and phases[first] == phases[last] == phase


def _with_segments(plan, segments):
    return dataclasses.replace(plan, segments=tuple(segments))


def test_acro_phases_refuses_a_script_the_rule_does_not_cover(plan):
    segs = list(plan.segments)
    combined = next(i for i, s in enumerate(segs) if sum(1 for x in s[3] if x) > 1)
    single = next(s for s in segs if sum(1 for x in s[3] if x) == 1)
    run_l4.acro_phases(_with_segments(plan, segs))  # the committed script is covered
    # a combined segment followed by a nonzero segment
    k, t_us, _, _ = segs[combined + 1]
    with pytest.raises(run_l4.PlanError, match="recovery check needs a zero segment"):
        run_l4.acro_phases(_with_segments(plan, segs[:combined + 1] + [(k, t_us, single[2], single[3])]))
    # a combined segment as the last segment
    with pytest.raises(run_l4.PlanError, match="last segment is combined"):
        run_l4.acro_phases(_with_segments(plan, segs[:combined + 1]))
    # a segment after the recovery segment
    extra = (plan.end_execution, plan.stamp_us(plan.end_execution), single[2], single[3])
    with pytest.raises(run_l4.PlanError, match="follows the recovery segment"):
        run_l4.acro_phases(_with_segments(plan, segs + [extra]))


# ---- the recovery predicate -----------------------------------------------------------------------------------------

def test_rest_bound_is_the_largest_magnitude_of_the_envelope():
    assert run_l4.rest_bound(ENV_LO, ENV_HI) == [2.0, 3.0]


def _evaluate(w, fresh=None):
    n = len(Z_TRACE)
    return run_l4.recovery_evaluation(list(range(n)), w, [0.0] * n, Z_TRACE, F_TEST, fresh or [True] * n)


def test_recovery_passes_rest_and_fails_a_stuck_rate():
    rest = _evaluate([0.0] * len(Z_TRACE))
    assert rest["passed"] and rest["violations"] == 0
    assert rest["worst margin"] == min(z + F_TEST for z in Z_TRACE)
    stuck = _evaluate([STUCK] * len(Z_TRACE))
    assert not stuck["passed"]
    over = [k for k, z in enumerate(Z_TRACE) if STUCK > z + F_TEST]
    assert (stuck["violations"], stuck["first violation"], stuck["last violation"]) == (len(over), over[0], over[-1])
    assert stuck["at execution"] == len(Z_TRACE) - 1  # the worst margin is where Z is smallest


def test_recovery_checks_every_execution_and_both_signs():
    last = len(Z_TRACE) - 1
    for sign in (1, -1):
        w = [0.0] * len(Z_TRACE)
        w[last] = sign * SPIKE
        ev = _evaluate(w)
        assert not ev["passed"] and ev["first violation"] == ev["last violation"] == last
        assert ev["worst margin"] == Z_TRACE[last] + F_TEST - SPIKE


def test_recovery_skips_a_stale_execution_as_the_rate_bound_does():
    last = len(Z_TRACE) - 1
    w = [0.0] * len(Z_TRACE)
    w[last] = SPIKE
    fresh = [True] * len(Z_TRACE)
    fresh[last] = False
    ev = _evaluate(w, fresh)
    assert ev["passed"] and ev["fresh executions checked"] == last
    assert not _evaluate(w, [False] * len(Z_TRACE))["passed"]  # no fresh execution at all is no evidence
