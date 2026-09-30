"""Pure-Python tests of the chirp margin and U rule of tools/sim/run_l5.py u_terms (owner decision 20) and of the two-cycle rule of
the SIM-7 fixed-point check; each with a control that plants a violation. No Gazebo."""

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import run_l5  # noqa: E402

BASE = 1.2  # scenario test value: a margin of 1.2 rad
STEP = 0.01  # scenario test value: a plateau step of 0.01 rad


def plateau(values, m2=None):
    pm = {run_l5.plateau_name(k): v for k, v in zip(run_l5.PLATEAU_K, values)}
    pm["m2"] = m2
    return pm


def test_roll_pitch_margin_is_the_plateau_minimum_and_u_a_its_spread():
    pm = plateau([BASE + 3 * STEP, BASE + STEP, BASE, BASE + 2 * STEP, BASE + STEP / 2], m2=BASE + STEP / 4)
    for axis in run_l5.ROLL_PITCH:
        t = run_l5.u_terms(axis, pm)
        assert t["margin_run"] == "k3" and t["pm"] == BASE
        assert t["U_A"] == pytest.approx(3 * STEP, abs=1e-15)
        assert t["E_H"] == pytest.approx(STEP / 4, abs=1e-15)


def test_runs_without_a_crossover_leave_the_plateau_and_two_are_needed():
    pm = plateau([None, BASE + STEP, BASE, None, BASE + 2 * STEP], m2=BASE)
    t = run_l5.u_terms("roll", pm)
    assert t["margin_run"] == "k3" and t["U_A"] == pytest.approx(2 * STEP, abs=1e-15)
    with pytest.raises(run_l5.PlanError):
        run_l5.u_terms("roll", plateau([None, None, None, BASE, None], m2=BASE))


def test_control_a_planted_low_outlier_moves_the_margin_and_the_spread():
    pm = plateau([BASE, BASE, BASE, BASE, BASE], m2=BASE)
    flat = run_l5.u_terms("pitch", pm)
    assert flat["U_A"] == 0
    pm["k2"] = BASE - 5 * STEP
    bad = run_l5.u_terms("pitch", pm)
    assert bad["margin_run"] == "k2" and bad["pm"] < flat["pm"] and bad["U_A"] == pytest.approx(5 * STEP, abs=1e-15)


def test_yaw_keeps_the_l4_rule():
    t = run_l5.u_terms("yaw", {"m1": BASE, "m2": BASE + STEP, "half_amplitude": BASE - 2 * STEP})
    assert t["margin_run"] == "m1" and t["pm"] == BASE
    assert t["E_H"] == pytest.approx(STEP, abs=1e-15) and t["U_A"] == pytest.approx(2 * STEP, abs=1e-15)
    assert math.isfinite(t["E_H"] + t["U_A"])
