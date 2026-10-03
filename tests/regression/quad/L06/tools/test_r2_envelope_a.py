"""R2 under envelope A (decision 0015; decision 0014 owner decision 5): tests/regression/quad/L06/results/r2_envelope_a/.

Re-runs r2_envelope_a.py once (its main, the README's command) and checks:
  - the raw output reproduces envelope_a.txt byte for byte; control: a changed result renders differently;
  - envelope A is the R2 test's: the script's reduction of recovery_model.member_run over the grid equals
    recovery_model.envelope's lo and hi (the function test_t4_recovery.make_design calls) over the first B_EXEC executions;
  - claim (a), on the rate channels: run C (w x Jw in the plant, ideal lag-compensated feed-forward, motors from tau0) is
    inside envelope A within delta = V + K + R at every execution of R2's window, and equals D(v0) within K + R; the
    integration and floor terms are at binary64 rounding scale (the guard), so delta is not inflated;
  - claim (b), on the rate channels: run C leaves envelope B (motor state tau0) by more than delta at execution 1, the
    first execution after t = 0;
  - control (core 7.2): run C without the feed-forward leaves envelope A by more than delta.
"""

import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "r2_envelope_a"
sys.path.insert(0, str(DIR))
import r2_envelope_a as ea  # noqa: E402

FIRST_EXECUTION = 1  # the first execution after t = 0: at execution 0 every member and run C sit at the initial state


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    out = tmp_path_factory.mktemp("r2_envelope_a") / "envelope_a.txt"
    r, ev, text = ea.main(["--out", str(out)])
    return r, ev, text, out


def test_raw_output_reproduces_byte_for_byte(run):
    *_, out = run
    assert out.read_bytes() == (DIR / "envelope_a.txt").read_bytes()


def test_control_a_changed_result_renders_differently(run):
    r, ev, text, _ = run
    planted = copy.deepcopy(ev)
    planted["per"][ea.RATE[0]]["a_beyond"] += 1
    assert ea.render(r, planted) != text


def test_envelope_a_is_the_r2_tests_envelope(run):
    r, *_ = run
    env = ea.rm.envelope(r["su"], ea.rm.oracle.read_q_inputs(ea.t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"), r["q0"], r["w0"],
                         ea.B_EXEC)
    assert [lo[:ea.B_EXEC] for lo in r["A_lo"]] == env["lo"]
    assert [hi[:ea.B_EXEC] for hi in r["A_hi"]] == env["hi"]


def test_tolerance_terms_are_at_rounding_scale(run):
    _, ev, *_ = run
    for c in ea.RATE:
        p = ev["per"][c]
        assert p["K"] + p["R"] <= p["scale"], (ea.CHANNELS[c], p)


def test_claim_a_ideal_ff_stays_inside_envelope_a_to_rounding(run):
    _, ev, *_ = run
    assert ev["claim_a"]
    for c in ea.RATE:
        p = ev["per"][c]
        assert p["a_beyond"] == 0 and p["a_out"] <= p["delta"], (ea.CHANNELS[c], p)
        assert p["res_v0"] <= p["K"] + p["R"], (ea.CHANNELS[c], p)


def test_claim_b_ideal_ff_leaves_envelope_b_at_the_first_execution(run):
    _, ev, *_ = run
    assert ev["claim_b"]
    for c in ea.RATE:
        p = ev["per"][c]
        assert p["b_first"] == FIRST_EXECUTION and p["b_first_out"] > p["delta"], (ea.CHANNELS[c], p)


def test_control_without_the_feed_forward_leaves_envelope_a(run):
    _, ev, *_ = run
    assert ev["control"]
    assert max(ev["per"][c]["x_max"] - ev["per"][c]["delta"] for c in ea.RATE) > 0
