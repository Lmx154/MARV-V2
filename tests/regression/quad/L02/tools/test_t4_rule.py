"""T4 pass rule of docs/decisions/0003 items 9 and 11 (tools/sim/t4_rule.py). L02 tests, synthetic sequences only.

Every metric test has a negative control: a sequence, a reference or a read pattern that the same rule must reject.
Test values are labelled scenario values: q* = 4.9 (the free-fall distance scale), c = 3, c2 = 5 (first- and
second-order coefficients of the synthetic error), t_tick = 156.25 us (scenarios/quad/L02, 625 us / 4), F = 1e-12
(the order of the free-fall floor at T = 1 s).
"""

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import t4_rule  # noqa: E402

Q_STAR = 4.9
C1 = 3.0
C2 = 5.0
T_TICK = 625e-6 / 4
F = 1e-12
F_REF = 1e-13
K = 2


def steps(t_tick=T_TICK, k_max=K):
    return [2.0 ** (k_max - k) * t_tick for k in range(k_max + 1)]


def seq(c1=C1, c2=0.0, q_star=Q_STAR):
    return [q_star + c1 * h + c2 * h * h for h in steps()]


def test_unit_roundoff_is_binary64():
    assert t4_rule.U == 2.0 ** -53
    assert 1.0 + t4_rule.U == 1.0 and 1.0 + 2.0 * t4_rule.U > 1.0


def test_floor_rule():
    assert t4_rule.floor(6400, 4.9, 82) == 6482 * t4_rule.U * 4.9
    assert t4_rule.floor(0, 2.0, 0) == 0.0
    for bad in ((-1, 1.0, 0), (1, -1.0, 0), (1, 1.0, -1), (1, math.inf, 0)):
        with pytest.raises(ValueError):
            t4_rule.floor(*bad)


def test_first_order_sequence_passes_branch_ii_with_observed_order_one():
    for c2 in (0.0, C2):
        v = t4_rule.evaluate(seq(c2=c2), Q_STAR, F, F_REF)
        assert v.passed and v.branch == t4_rule.BRANCH_II and not v.pass_i
        assert v.p_obs == pytest.approx(1.0, abs=5e-3)  # c2 H_K / c1 ~ 3e-4 shifts p_obs by ~1e-3
        assert 0.0 < v.rho < 1.0 and v.rho == pytest.approx(0.5, abs=5e-3)
        assert v.E == pytest.approx(C1 * T_TICK, rel=1e-3)
        assert v.q_hat == pytest.approx(Q_STAR, abs=max(abs(c2) * T_TICK ** 2 * 20, 1e-12))  # error of q_hat is O(c2 H^2)
        assert abs(v.q_hat - Q_STAR) < v.E / 100  # extrapolation removes the first-order error
        assert len(v.d) == K and v.d[-1] == pytest.approx(-C1 * T_TICK, rel=1e-3)


def test_second_order_sequence_gives_observed_order_two():
    q = [Q_STAR + C2 * h * h for h in steps()]
    v = t4_rule.evaluate(q, Q_STAR, F, F_REF)
    assert v.passed and v.branch == t4_rule.BRANCH_II
    assert v.p_obs == pytest.approx(2.0, abs=1e-6) and v.rho == pytest.approx(0.25, abs=1e-6)


def test_report_lists_every_term():
    r = t4_rule.evaluate(seq(), Q_STAR, F, F_REF).report()
    for key in ("d", "rho", "p_obs", "E", "q_hat", "bound", "F", "branch", "passed"):
        assert key in r


def test_bound_of_branch_ii_is_the_formula():
    v = t4_rule.evaluate(seq(), Q_STAR, F, F_REF)
    assert v.bound == v.E + F * (1.0 + v.rho) / (1.0 - v.rho) + F_REF
    assert v.error == abs(v.q_hat - Q_STAR)


def test_reference_at_the_bound_edge_passes_and_just_beyond_fails():
    v = t4_rule.evaluate(seq(), Q_STAR, F, F_REF)
    for sign in (1.0, -1.0):
        inside = t4_rule.evaluate(seq(), v.q_hat + sign * v.bound * 0.99, F, F_REF)
        outside = t4_rule.evaluate(seq(), v.q_hat + sign * v.bound * 1.01, F, F_REF)
        assert inside.passed and inside.branch == t4_rule.BRANCH_II
        assert not outside.passed and outside.branch is None and outside.reasons


def test_negative_control_offset_reference_fails():
    v = t4_rule.evaluate(seq(), Q_STAR + 10.0 * C1 * T_TICK, F, F_REF)
    assert not v.passed and v.branch is None and not v.pass_i and not v.pass_ii
    assert any("(ii)" in r for r in v.reasons) and any("(i)" in r for r in v.reasons)


def test_converged_sequence_passes_branch_i_whatever_rho_does():
    for q in (
        [Q_STAR + 3e-13, Q_STAR - 2e-13, Q_STAR + 1e-13],  # oscillating, rho < 0
        [Q_STAR + 3e-13, Q_STAR + 3e-13, Q_STAR + 3e-13],  # all differences zero
        [Q_STAR + 1e-13, Q_STAR + 2e-13, Q_STAR + 3e-13],  # rho = 1
    ):
        v = t4_rule.evaluate(q, Q_STAR, F, F_REF)
        assert v.passed and v.branch == t4_rule.BRANCH_I and v.E <= F
        assert v.bound == F + F_REF


def test_negative_control_converged_but_offset_reference_fails_branch_i():
    q = [Q_STAR + 3e-13, Q_STAR - 2e-13, Q_STAR + 1e-13]
    v = t4_rule.evaluate(q, Q_STAR + 10.0 * (F + F_REF), F, F_REF)
    assert v.E <= F and not v.passed and not v.pass_i


def test_negative_control_not_converged_and_no_branch_ii_fails():
    q = [Q_STAR + 3e-11, Q_STAR - 2e-11, Q_STAR + 1e-11]  # oscillating and E > F
    v = t4_rule.evaluate(q, Q_STAR, F, F_REF)
    assert not v.passed and v.E > F


def test_oscillating_sequence_fails_unless_branch_i_holds():
    h = steps()
    q = [Q_STAR + C1 * h[0], Q_STAR - C1 * h[1], Q_STAR + C1 * h[2]]  # d alternates in sign, rho < 0
    v = t4_rule.evaluate(q, Q_STAR, F, F_REF)
    assert v.rho < 0.0 and v.p_obs is None and v.q_hat is None
    assert not v.passed and any("rho not in (0, 1)" in r for r in v.reasons)
    # the same shape below the floor passes on (i)
    tiny = [Q_STAR + 4e-13, Q_STAR - 3e-13, Q_STAR + 2e-13]
    assert t4_rule.evaluate(tiny, Q_STAR, F, F_REF).branch == t4_rule.BRANCH_I


def test_rho_at_or_above_one_fails_branch_ii():
    for q in ([1.0, 1.5, 2.0], [1.0, 1.5, 3.0]):  # rho = 1, 3 (exact in binary)
        v = t4_rule.evaluate(q, q[-1], F, F_REF)
        assert not v.pass_ii and any("rho not in (0, 1)" in r for r in v.reasons)
    assert t4_rule.evaluate(q, q[-1], F, F_REF).E > F


def test_d_k_minus_1_zero_has_no_branch_ii_and_no_division():
    q = [1.0, 1.0, 1.5]  # d_{K-1} = 0, d_K = 0.5
    v = t4_rule.evaluate(q, 1.5, F, F_REF)
    assert v.rho is None and v.p_obs is None and v.q_hat is None and not v.pass_ii
    assert not v.passed and any("d_{K-1} = 0" in r for r in v.reasons)
    flat = t4_rule.evaluate([1.0, 1.0, 1.0], 1.0, F, F_REF)  # all zero: (i) holds, still no rho
    assert flat.rho is None and flat.passed and flat.branch == t4_rule.BRANCH_I
    z = t4_rule.evaluate([5.0, 3.0, 3.0], 3.0, F, F_REF)  # d_K = 0: rho = 0 is outside (0, 1); (i) holds
    assert z.rho == 0.0 and z.p_obs is None and z.branch == t4_rule.BRANCH_I


def test_single_refinement_has_only_branch_i():
    assert t4_rule.evaluate([1.0, 1.0 + 1e-13], 1.0, F, F_REF).branch == t4_rule.BRANCH_I
    v = t4_rule.evaluate([1.0, 1.5], 1.5, F, F_REF)
    assert not v.passed and v.rho is None
    with pytest.raises(ValueError):
        t4_rule.evaluate([1.0], 1.0, F, F_REF)
    with pytest.raises(ValueError):
        t4_rule.evaluate(seq(), Q_STAR, -F, F_REF)


def test_non_finite_input_fails():
    for q in ([1.0, 1.0, math.nan], [1.0, 2.0, math.inf]):
        v = t4_rule.evaluate(q, 1.0, F, F_REF)
        assert not v.passed and v.reasons == ("non-finite input",)
    assert not t4_rule.evaluate(seq(), math.nan, F, F_REF).passed


def test_fresh_index_is_the_last_bitwise_change():
    assert t4_rule.fresh_index([1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0]) == 4
    assert t4_rule.fresh_index([1.0, 2.0, 3.0, 4.0]) == 3
    assert t4_rule.fresh_index([1.0, 2.0, 2.0, 2.0]) == 1
    assert t4_rule.fresh_index([1.0, 1.0, 1.0]) is None
    assert t4_rule.fresh_index([1.0]) is None and t4_rule.fresh_index([]) is None


def test_fresh_index_compares_bits_not_values():
    assert t4_rule.fresh_index([0.0, -0.0]) == 1  # equal as numbers, different bits
    assert t4_rule.fresh_index([1.0, 1.0 + 2.0 ** -52]) == 1  # one ulp
    assert t4_rule.fresh_index([math.nan, math.nan]) is None  # same bits, not equal as numbers
    assert t4_rule.fresh_index([(1.0, 2.0, 3.0), (1.0, 2.0, 3.0), (1.0, 2.0, 3.0 + 2.0 ** -51)]) == 2
    assert t4_rule.fresh_index([(1.0, 2.0), (1.0, 2.0)]) is None
    assert t4_rule.fresh_index([1, 1.0]) is None  # an int and its float are the same read


def test_freshness_reports_stale_steps():
    reads = [1.0, 1.0, 2.0, 2.0, 2.0, 3.0]
    times = [float(i) for i in range(6)]
    f = t4_rule.freshness(reads, times)
    assert f.ok and f.index == 5 and f.stale_steps == 3


def test_negative_control_no_fresh_read_in_the_last_half_fails():
    times = [float(i) for i in range(10)]
    stuck = [1.0, 2.0, 3.0, 4.0, 5.0] + [5.0] * 5  # last change at t = 4 < 4.5
    f = t4_rule.freshness(stuck, times)
    assert not f.ok and f.index == 4 and any("last half" in r for r in f.reasons)
    ok = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] + [6.0] * 4  # last change at t = 5 >= 4.5
    assert t4_rule.freshness(ok, times).ok
    never = t4_rule.freshness([1.0] * 10, times)
    assert not never.ok and never.index is None and never.stale_steps == 9


def test_hover_needs_a_fresh_read_after_the_settling_time():
    times = [float(i) for i in range(10)]
    reads = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 7.0, 7.0, 7.0]  # last change at t = 6
    assert t4_rule.freshness(reads, times, t_settle=5.5).ok
    f = t4_rule.freshness(reads, times, t_settle=6.0)  # the read must be strictly after t_s
    assert not f.ok and any("settling" in r for r in f.reasons)
    assert not t4_rule.freshness([1.0] * 10, times, t_settle=1.0).ok


def test_freshness_input_checks():
    with pytest.raises(ValueError):
        t4_rule.freshness([1.0, 2.0], [0.0])
    with pytest.raises(ValueError):
        t4_rule.freshness([1.0], [0.0])


# T4 behaviour predicates S, B, T (docs/decisions/0029). Known answers by hand; no negative control (core 7.2 item 3).
# Labelled scenario values: s = 0.02 (t4_settle_fraction), E = 0.001 (a stand-in |m1 - m2|), a segment of 4 executions.
S_FRACTION = 0.02
E_SIM = 0.001


def seg(**quantities):
    return {name: (unit, errors) for name, (unit, errors) in quantities.items()}


def test_settles_at_the_boundary():
    # Delta = 1.0 (rad group), bound = 0.02 * 1.0 + 0.001 = 0.021
    on = seg(a=("rad", [1.0, 0.5, 0.1, 0.021]))
    over = seg(a=("rad", [1.0, 0.5, 0.1, 0.0211]))
    (c,) = t4_rule.settles(on, S_FRACTION, {"a": E_SIM})
    assert c.ok and c.bound == pytest.approx(0.021)
    (c,) = t4_rule.settles(over, S_FRACTION, {"a": E_SIM})
    assert not c.ok
    (c,) = t4_rule.settles(seg(a=("rad", [-1.0, -0.5, -0.1, -0.021])), S_FRACTION, {"a": E_SIM})
    assert c.ok  # the sign of the error does not matter


def test_delta_is_per_unit_group_not_per_quantity():
    # roll and heading share rad (Delta = 2.0), the rate does not (Delta = 10.0 rad/s)
    segment = seg(roll=("rad", [2.0, 0.0, 0.0, 0.04]),
                  heading=("rad", [0.1, 0.0, 0.0, 0.0]),
                  rate=("rad/s", [10.0, 0.0, 0.0, 0.2]))
    E = {"roll": 0.0, "heading": 0.0, "rate": 0.0}
    res = {c.name: c for c in t4_rule.settles(segment, S_FRACTION, E)}
    assert res["roll"].bound == pytest.approx(0.04) and res["roll"].ok  # 0.02 * 2.0, not 0.02 * 0.1
    assert res["heading"].bound == pytest.approx(0.04) and res["heading"].ok
    assert res["rate"].bound == pytest.approx(0.2) and res["rate"].ok  # 0.02 * 10.0
    tight = {c.name: c for c in t4_rule.settles(seg(a=("rad", [1.0, 0.0, 0.0, 0.0]),
                                                     b=("rad", [0.0, 0.0, 0.0, 0.03])), S_FRACTION,
                                                {"a": 0.0, "b": 0.0})}
    assert tight["a"].ok and not tight["b"].ok  # b's own Delta is 0.03, but the group's is 1.0: bound 0.02


def test_e_is_added_per_quantity():
    segment = seg(a=("rad", [1.0, 0.0, 0.0, 0.03]), b=("rad", [1.0, 0.0, 0.0, 0.03]))
    res = {c.name: c.ok for c in t4_rule.settles(segment, S_FRACTION, {"a": 0.0, "b": 0.011})}
    assert res == {"a": False, "b": True}


def test_bounded_passes_and_fails():
    # first error 0.5, Delta = 1.0 (peak), bound on the excess = 0.02 + 0.001
    ok = seg(a=("rad", [0.5, 0.9, 1.0, 0.5]))
    (c,) = t4_rule.bounded(ok, S_FRACTION, {"a": E_SIM})
    assert not c.ok and c.value == pytest.approx(0.5)  # grew by 0.5 > 0.021
    flat = seg(a=("rad", [0.5, 0.51, 0.4, 0.5]))
    (c,) = t4_rule.bounded(flat, S_FRACTION, {"a": E_SIM})
    assert c.ok and c.value == pytest.approx(0.01) and c.bound == pytest.approx(0.02 * 0.51 + 0.001)
    shrinking = seg(a=("rad", [1.0, 0.5, 0.2, 0.0]))
    (c,) = t4_rule.bounded(shrinking, S_FRACTION, {"a": E_SIM})
    assert c.ok and c.value == 0.0  # the excess of the first sample over itself


def test_shrinks():
    assert t4_rule.shrinks([1.0, 2.0, 0.5])
    assert t4_rule.shrinks([-1.0, 0.3, -0.9])  # magnitudes
    assert not t4_rule.shrinks([1.0, 0.1, 1.0])  # equal is not smaller
    assert not t4_rule.shrinks([0.09, 5.0, 10.29])


def test_predicates_reject_non_finite_and_empty_segments():
    nan = float("nan")
    assert not t4_rule.settles(seg(a=("rad", [1.0, nan, 0.0])), S_FRACTION, {"a": 1.0})[0].ok
    assert not t4_rule.bounded(seg(a=("rad", [1.0, nan, 0.0])), S_FRACTION, {"a": 1.0})[0].ok
    assert not t4_rule.shrinks([1.0, nan])
    with pytest.raises(ValueError):
        t4_rule.settles(seg(a=("rad", [])), S_FRACTION, {"a": 0.0})


# T4 roles (docs/decisions/0031): scenario_checks. Known answers by hand; s = 0.02, E = 0.001 as above.
def roles(**quantities):
    return {name: (unit, role, errors) for name, (unit, role, errors) in quantities.items()}


def zero_e(segments, value=E_SIM):
    return {seg: {name: value for name in q} for seg, q in segments.items()}


def run_roles(segments, disturbed=frozenset()):
    e = zero_e(segments)
    return t4_rule.scenario_checks(segments, S_FRACTION, e, e, disturbed)


def test_stepped_and_held_with_a_moving_unit_equal_settles_and_bounded():
    q = {"seg": roles(a=("rad", t4_rule.STEPPED, [1.0, 0.5, 0.1, 0.02]), b=("rad", t4_rule.HELD, [0.0, 0.01, 0.0, 0.0]),
                      w=("rad/s", t4_rule.STEPPED, [10.0, 5.0, 1.0, 0.1]))}
    got = run_roles(q)["seg"]
    plain = {n: (u, e) for n, (u, _, e) in q["seg"].items()}
    e = {n: E_SIM for n in plain}
    assert got["S"] == t4_rule.settles(plain, S_FRACTION, e)
    assert got["B"] == t4_rule.bounded(plain, S_FRACTION, e)


def test_driven_alone_gets_s_and_no_b():
    q = {"seg": roles(a=("rad", t4_rule.DRIVEN, [0.0, 0.6, 0.3, 0.01]))}
    got = run_roles(q)["seg"]
    (c,) = got["S"]
    assert c.ok and c.bound == pytest.approx(0.02 * 0.6 + E_SIM)  # 0.013
    assert got["B"] == []


def test_the_same_series_held_next_to_a_stepped_peak_fails_b():
    q = {"seg": roles(a=("rad", t4_rule.HELD, [0.0, 0.6, 0.3, 0.01]), peak=("rad", t4_rule.STEPPED, [1.0, 0.5, 0.1, 0.0]))}
    got = run_roles(q)["seg"]
    (b,) = [c for c in got["B"] if c.name == "a"]
    assert not b.ok and b.value == pytest.approx(0.6) and b.bound == pytest.approx(0.02 * 1.0 + E_SIM)


def test_held_only_unit_takes_delta_from_another_segment():
    q = {"hold": roles(tilt=("rad", t4_rule.HELD, [1.2e-14, 1.5e-14, 1.3e-14]), w=("rad/s", t4_rule.STEPPED, [1.0, 0.5, 0.0])),
         "release": roles(tilt=("rad", t4_rule.HELD, [1.0e-14, 2.0e-14, 1.0e-14]),
                          h=("rad", t4_rule.DRIVEN, [0.0, 0.6, 0.3, 0.01]), w=("rad/s", t4_rule.STEPPED, [0.5, 0.2, 0.0]))}
    got = run_roles(q)
    for c in got["hold"]["S"] + got["hold"]["B"]:
        if c.name == "tilt":
            assert c.ok and c.bound == pytest.approx(0.02 * 0.6 + E_SIM)
    alone = {"hold": q["hold"]}
    with pytest.raises(ValueError):
        run_roles(alone)


def test_disturbed_segment_gets_s_only():
    q = {"seg": roles(a=("rad", t4_rule.STEPPED, [1.0, 2.0, 0.5, 0.0]))}
    got = run_roles(q, disturbed={"seg"})["seg"]
    assert got["B"] == [] and got["S"][0].ok
    assert run_roles(q)["seg"]["B"][0].ok is False  # without the onset the growth to 2.0 fails B


def test_driven_that_does_not_settle_fails_s():
    q = {"seg": roles(a=("rad", t4_rule.DRIVEN, [0.0, 0.6, 1.25, 1.25]))}
    (c,) = run_roles(q)["seg"]["S"]
    assert not c.ok and c.value == pytest.approx(1.25) and c.bound == pytest.approx(0.02 * 1.25 + E_SIM)


def test_roles_reject_an_unknown_role_and_non_finite_input():
    with pytest.raises(ValueError):
        run_roles({"seg": roles(a=("rad", "free", [1.0, 0.0]))})
    nan = float("nan")
    got = run_roles({"seg": roles(a=("rad", t4_rule.STEPPED, [1.0, nan, 0.0]))})["seg"]
    assert not got["S"][0].ok and not got["B"][0].ok
    with pytest.raises(ValueError):
        run_roles({"seg": roles(a=("rad", t4_rule.STEPPED, []))})
