"""The chirp's stage (c) design reference reduces to rate.py's with the lead off (owner, decision 0014 fourth round item 2
and sixth round (i)). With the lead off (rate_lead at N = 1, T_f = 0, no chain, latency 0):
  - the law: rate_lead's f32 kp and ki are rate.py's bit for bit, and kd = 0;
  - the reference: run_l5.t4_lead_reference's quantities, evaluated on that law (attitude_lead.build_loops,
    attitude.pm_worst at the build's attitude gain, steady_torque_peak_lead), equal run_l5.chirp_design's (attitude.py's
    PM and crossover of every loop, the band, G_tau per axis) within the equivalence convention of
    test_rate_lead.py::test_n1_without_chain_or_latency_reproduces_rate_py, 2^-30. They are not bit-identical:
    attitude.Loop.response solves a 4x4 complex system and LeadLoop.resp evaluates rule step 2''s closed form (decision
    0014, "The chirp special case"; observed 1.4e-13 rad on PM, 1.8e-13 relative on G_tau).
Control: the lead at its design values (rate_lead's product gains, Model.gains(OMEGA_C, N_STAR) of
test_rate_lead_checks.py, which test_rate_lead.py pins to design()'s output; kd != 0 and T_f > 0; evaluated here without
the chain and at latency 0) breaks the reference agreement. T_f alone cannot be the control: with kd = 0 the law never reads T_f (LeadLoop: the D filter
exists only when kd != 0).
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import attitude  # noqa: E402
import attitude_lead as al  # noqa: E402
import gen_imu_config as gic  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import gyro_chain_params as gcp  # noqa: E402
import rate  # noqa: E402
import rate_lead as rl  # noqa: E402
import run_l5  # noqa: E402
import schema  # noqa: E402
import test_rate_lead_checks as checks  # noqa: E402  (this directory; pytest's default import mode puts it on sys.path)

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"

# The equivalence convention of test_rate_lead.py (TOL, 2^-30): both evaluators are exact to double rounding.
TOL = 2.0 ** -30


def reference(rr, res, axes32, divisor):
    """t4_lead_reference's design-model quantities on the inner loop `axes32` (H = 1, latency 0) at the build's attitude
    gain: (PM per (axis, corner), crossover per (axis, corner), band, G_tau per axis)."""
    inner = al.Inner("lead-off check", axes32, rr["T"] / divisor, divisor, 0, None)
    loops = al.build_loops(rr, inner)
    _, detail = attitude.pm_worst(loops, res["k32"], res["w32"])
    assert all(lp.start_ok and lp.branch_ok for _, _, lp in loops)
    assert not [why for *_, why in detail if why]
    pm = {(a, n): p for a, n, p, _, _, _ in detail}
    cross = {(a, n): th / res["T_a"] for a, n, _, th, _, _ in detail}
    band = (min(cross.values()) / rr["a"], rr["a"] * max(cross.values()))
    corners = rate.corner_list(rr["inputs"]["tau"], rr["inputs"]["b_tau"], rr["inputs"]["b_J"])
    jt_of = {n: jt for n, jt, _ in corners}
    by = {(a, n): lp for a, n, lp in loops}
    peaks = {}
    for axis in rate.AXES:
        k = attitude.axis_k(axis, res["k32"], res["w32"])
        j_a = rr["axes"][axis]["J"]

        def peak(name, w, axis=axis, k=k, j_a=j_a):
            return run_l5.steady_torque_peak_lead(by[axis, name], j_a, jt_of[name], k, w)
        peaks[axis] = run_l5.peak_torque_gain(rr, axis, k, res["N"], band, corners, peak)
    return pm, cross, band, peaks


def pi_reference(rr, res):
    """run_l5.chirp_design's design quantities (its body without the build check and tau_held)."""
    pm = {(a, n): p for a, n, p, _, _, _ in res["final"]["detail"]}
    cross = {(a, n): th / res["T_a"] for a, n, _, th, _, _ in res["final"]["detail"]}
    band = (min(cross.values()) / rr["a"], rr["a"] * max(cross.values()))
    corners = rate.corner_list(rr["inputs"]["tau"], rr["inputs"]["b_tau"], rr["inputs"]["b_J"])
    peaks = {a: run_l5.peak_torque_gain(rr, a, attitude.axis_k(a, res["k32"], res["w32"]), res["N"], band, corners)
             for a in rate.AXES}
    return pm, cross, band, peaks


def deviation(mine, ref):
    """The largest (absolute PM rad, relative crossover, relative band edge, relative G_tau) difference."""
    pm, cross, band, peaks = mine
    rpm, rcross, rband, rpeaks = ref
    assert pm.keys() == rpm.keys() and peaks.keys() == rpeaks.keys()
    return (max(abs(pm[k] - rpm[k]) for k in pm),
            max(abs(cross[k] / rcross[k] - 1) for k in cross),
            max(abs(b / r - 1) for b, r in zip(band, rband)),
            max(abs(peaks[a][0] / rpeaks[a][0] - 1) for a in peaks))


@pytest.fixture(scope="module")
def pi():
    res = run_l5.attitude_design(str(CARD), str(ROOT))
    if res["N"] != 1:
        pytest.fail(f"the PI attitude design's att_loop_ratio is {res['N']!r}; attitude_lead models N = 1 only")
    rr = res["rate"]
    divisor = schema.load_yaml(SCENARIO)["rate_loop_divisor"]["value"]
    return rr, res, divisor, pi_reference(rr, res)


def lead_off_axes(rr, divisor):
    inp = rr["inputs"]
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    model = rl.Model(rr["T"] / divisor, divisor, 0, None, corners, rr["a"], inp["inertia"])
    return rl.f32_axes(model, model.gains(rr["omega_c"], 1.0))


def test_lead_off_law_is_rate_py_bit_for_bit(pi):
    rr, _, divisor, _ = pi
    for axis, (kp, ki, kd, _) in zip(rate.AXES, lead_off_axes(rr, divisor)):
        assert (kp, ki, kd) == (rr["axes"][axis]["kp"], rr["axes"][axis]["ki"], 0.0), axis


def test_lead_off_chirp_reference_is_rate_py_within_convention(pi):
    rr, res, divisor, ref = pi
    # The law, not f32_axes's T_f: the N = 1 prototype carries T_f = 1/omega_p, which kd = 0 never reads; T_f = 0 here.
    axes32 = [(kp, ki, kd, 0.0) for kp, ki, kd, _ in lead_off_axes(rr, divisor)]
    dev = deviation(reference(rr, res, axes32, divisor), ref)
    assert all(d <= TOL for d in dev), dev


def design_axes():
    """rate_lead's product f32 (kp, ki, kd, T_f): test_rate_lead_checks.py's setup, the rule's gains at its output."""
    card, budget, scenario, profile = rl.load(CARD, BUDGET, SCENARIO, PROFILE)
    l4 = rate.design(card, budget, scenario, CARD)
    inp = l4["inputs"]
    chain = gcp.design(card, budget, scenario, profile, PROFILE, CARD)
    si, _, _ = gic.imu_config(PROFILE)
    stages = gcd.chain_stages(chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"], [chain["omega_th"]] * gcd.MOTORS)
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    model = rl.Model(chain["t_s"], chain["divisor"], si["latency_samples"], stages, corners, l4["a"], inp["inertia"])
    return rl.f32_axes(model, model.gains(checks.OMEGA_C, checks.N_STAR))


def test_control_lead_at_design_values_breaks_it(pi):
    rr, res, divisor, ref = pi
    axes32 = design_axes()
    assert all(kd != 0 and tf > 0 for _, _, kd, tf in axes32)
    dev = deviation(reference(rr, res, axes32, divisor), ref)
    assert max(dev) > 1000 * TOL, dev
