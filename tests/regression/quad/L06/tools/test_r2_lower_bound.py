"""The R2 lower-bound evidence (decision 0014, owner decision 5, condition 1): tests/regression/quad/L06/results/r2_lower_bound/.

Re-runs r2_lower_bound.py once (its main, the README's command) and checks:
  - the raw output reproduces bound.txt byte for byte; control: a changed result renders differently;
  - the claim: on the binding axis the forced excess X* exceeds the R2 predicate's design tolerance F + Q; controls (core 7.2):
    NC1 (steady-tumble rotors, zero net torque at t = 0) and NC2 (equal principal moments, w x Jw = 0) each collapse it;
  - consistency with the committed R2 replay (cause.txt): the observed departure and excess are at least the bound; control:
    a planted trace held at w0 fails it.
"""

import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "r2_lower_bound"
sys.path.insert(0, str(DIR))
import r2_lower_bound as lb  # noqa: E402


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    out = tmp_path_factory.mktemp("r2_lower_bound") / "bound.txt"
    return lb.main(["--out", str(out)]), out


def test_raw_output_reproduces_byte_for_byte(run):
    _, out = run
    assert out.read_bytes() == (DIR / "bound.txt").read_bytes()


def test_control_a_changed_result_renders_differently(run):
    r, out = run
    planted = copy.copy(r)
    planted["ax"] = copy.deepcopy(r["ax"])
    b = r["binding"]
    planted["ax"][b]["X_star"] = planted["ax"][b]["tol"]
    assert lb.render(planted).encode() != out.read_bytes()


def test_the_a_priori_box_holds(run):
    r, _ = run
    for res in (r["res"], r["nc1"]["res"], r["nc2"]["res"]):
        assert res["box_ok"]


def test_forced_excess_exceeds_the_design_tolerance_on_the_binding_axis(run):
    r, _ = run
    x = r["ax"][r["binding"]]
    assert x["X_star"] > x["tol"], x


def test_control_steady_tumble_rotors_collapse_the_bound(run):
    r, _ = run
    nc1 = r["nc1"]
    assert all(abs(t) <= abs(c) for t, c in zip(nc1["net0"], r["cw0"]))
    for x in nc1["ax"]:
        assert x["L_star"] <= 0.0 and x["X_star"] <= x["tol"], x


def test_control_equal_moments_collapse_the_bound(run):
    r, _ = run
    nc2 = r["nc2"]
    assert nc2["cw0"] == (0.0, 0.0, 0.0)
    for x in nc2["ax"]:
        assert x["L_star"] <= 0.0 and x["X_star"] <= x["tol"], x


def test_bound_is_consistent_with_the_committed_r2_replay(run):
    r, _ = run
    assert r["cross"], "no cause.txt trace row inside the window"
    for row in r["cross"]:
        assert row["dep"] >= row["L"] and row["exc"] >= row["X"], row


def test_control_a_trace_held_at_w0_fails_the_consistency_check(run):
    r, _ = run
    held = [-lb.half_unit(row["y"]) for row in r["cross"]]  # departure of a planted y = w0, minus the same half unit
    assert any(d < row["L"] for d, row in zip(held, r["cross"]))
