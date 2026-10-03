"""The acro rate bound's design model (tools/sim/run_l4.py script_response, script_envelope, acro_setpoints) and the
chirp's float32 emulation (chirp_value_f32), without gz. L04 tests, host-only.

script_response is the T3 oracle's closed_loop with a setpoint per execution: with a constant setpoint at the oracle's
stamps it must reproduce closed_loop bit for bit (trajectory and rounding injections), and script_envelope must
reproduce band_envelope. The inputs are the T3 fixture (tests/regression/quad/L04/t3/reference/rate_t3_inputs.txt), so
no build is needed.

Reversal coverage (lead decision, 2026-09-30). The old bound rate_max (1 + overshoot of the step envelope) is not a
design-model bound for a reversal (+R to -R, a step of 2 R): a reversal from a settled +R peaks near 1.27 R at nominal
and 1.46 R at (J+, tau+). The new bound is the box peak of the design model on the script itself, so for a script that
is that reversal both peaks lie inside it by construction (both are points of the 17 x 17 grid); the test shows it and
that both exceed the old bound. Scenario test values: REVERSAL_AT_S = 3 s (the +R response settled: 18.6 tau_ref of
0.161 s) and END_S = 6 s (as long again after the reversal).
"""

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
REFERENCE = ROOT / "tests" / "regression" / "quad" / "L04" / "t3" / "reference"
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(REFERENCE))
import rate_t3_oracle as oracle  # noqa: E402
import run_l4  # noqa: E402

AXIS = "roll"
HALVED = 2 * (oracle.GRID - 1) + 1
REVERSAL_AT_S = 3.0
END_S = 6.0


@pytest.fixture(scope="module")
def fixture_inputs():
    p = oracle.read_inputs(str(REFERENCE / "rate_t3_inputs.txt"))
    return p, oracle.Setup(p)


def law(p, su, axis=AXIS):
    return (p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"], p[f"rate_d_filter_tau_{axis}"],
            p[f"rate_tau_ref_{axis}"], (su.divisor, su.lowpass, su.lowpass_error))


def test_script_response_is_the_oracle_closed_loop_for_a_step(fixture_inputs):
    p, su = fixture_inputs
    n, sp = su.executions(AXIS), p[f"rate_max_{AXIS}"]
    stamps = [su.stamp_us(k) for k in range(1, n + 1)]
    want = oracle.closed_loop(su, AXIS, p["inertia_xx"], p["motor_tau"], n, sp, want_rho=True)
    got = run_l4.script_response(*law(p, su), su.tick_map(p["inertia_xx"], p["motor_tau"]), [sp] * n, stamps, True)
    assert got == want
    lo, hi = run_l4.script_envelope(*law(p, su), su.tick_map, p["inertia_xx"], p["motor_tau"],
                                    p["inertia_robustness_band"], p["tau_robustness_band"], [sp] * n, stamps, oracle.GRID)
    assert (lo, hi) == oracle.band_envelope(su, AXIS, n, sp, oracle.GRID)
    other = run_l4.script_response(*law(p, su), su.tick_map(p["inertia_xx"], p["motor_tau"]),
                                   [math.nextafter(sp, math.inf)] * n, stamps)
    assert other != want[0]  # control: a setpoint one ulp higher is not reproduced


def test_the_rule_covers_a_settled_reversal_by_construction(fixture_inputs):
    p, su = fixture_inputs
    r = p[f"rate_max_{AXIS}"]
    period = su.period_s
    n = math.ceil(END_S / period)
    stamps = [su.stamp_us(k) for k in range(1, n + 1)]
    sps = [r if k * period < REVERSAL_AT_S else -r for k in range(n)]
    lo, hi = run_l4.script_envelope(*law(p, su), su.tick_map, p["inertia_xx"], p["motor_tau"],
                                    p["inertia_robustness_band"], p["tau_robustness_band"], sps, stamps, HALVED)
    box_peak = max(max(abs(x) for x in lo), max(abs(x) for x in hi))
    peaks = {}
    for name, jf, tf in (("nominal", 1.0, 1.0), ("J+,tau+", 1 + p["inertia_robustness_band"],
                                                    1 + p["tau_robustness_band"])):
        y = run_l4.script_response(*law(p, su), su.tick_map(p["inertia_xx"] * jf, p["motor_tau"] * tf), sps, stamps)
        peaks[name] = max(abs(v) for v in y)
        assert peaks[name] <= box_peak
    _, step_hi = oracle.band_envelope(su, AXIS, su.executions(AXIS), r, HALVED)
    old_bound = max(step_hi)  # rate_max (1 + overshoot of the step envelope)
    assert peaks["nominal"] > old_bound and peaks["J+,tau+"] > old_bound


def test_acro_setpoints_are_the_composition_setpoint_at():
    class Plan:  # scenario test values: two segments at executions 3 and 5, D = 2, the 625/4 us tick
        end_execution = 7
        segments = ((3, 937, (1.0, 2.0, 3.0), (1, 1, 1)), (5, 1562, (-1.0, 0.0, 0.0), (-1, 0, 0)))

        @staticmethod
        def stamp_us(k):
            return (2 * k * 625) // 4

    sps, stamps = run_l4.acro_setpoints(Plan, 1)
    assert stamps == [Plan.stamp_us(k) for k in range(8)]
    assert sps == [0.0, 0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 0.0]


def test_chirp_value_f32_is_the_double_chirp_to_float32_resolution_and_not_equal_to_it():
    amp, w_lo, w_hi, t0, dur = 0.5, 2.0, 32.0, 1000, 8_000_000  # scenario test values: a 16:1 band over 8 s
    ts = range(t0, t0 + dur, 3125)
    diff = [run_l4.chirp_value_f32(t, amp, w_lo, w_hi, t0, dur) - run_l4.chirp_value(t, amp, w_lo, w_hi, t0, dur)
            for t in ts]
    assert run_l4.chirp_value_f32(t0 - 1, amp, w_lo, w_hi, t0, dur) == 0.0
    assert run_l4.chirp_value_f32(t0 + dur, amp, w_lo, w_hi, t0, dur) == 0.0
    total_phase = (w_hi - w_lo) * dur / 1e6 / math.log(w_hi / w_lo)
    # float32 phase error: a few units of 2^-24 relative of phi, which grows to total_phase
    assert max(abs(x) for x in diff) <= amp * 16 * 2.0 ** -24 * total_phase
    assert any(diff)  # control: the emulation is not the double chirp
