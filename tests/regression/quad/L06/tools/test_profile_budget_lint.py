"""The profile against the budget (tools/card/lint.py, decision 0013 owner decision 2): the rotor_speed esc_clock_error of a
sensor profile may not exceed the design budget's esc_clock_error_max. The committed profile (2 %) and a copy at exactly the
requirement pass; a copy at 2.5 % is rejected, naming both entries (the negative control); the check also runs when the
profile is reached through a card and in flatten.py's lint.
"""

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
LINT = ROOT / "tools" / "card" / "lint.py"
FLATTEN = ROOT / "tools" / "card" / "flatten.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
ESC_VALUE = re.compile(r"(      esc_clock_error:\n        value: )2\n")
REQUIREMENT_PERCENT = 2  # esc_clock_error_max 0.02 (design/budget.yaml) in the profile's unit, %


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def profile_at(tmp_path, esc, name="marv_v2_board_default"):
    text, n = ESC_VALUE.subn(lambda m: f"{m.group(1)}{esc}\n", PROFILE.read_text(encoding="utf-8"))
    assert n == 1
    path = tmp_path / f"{name}.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_the_committed_profile_passes_with_the_budget():
    r = run(LINT, "--profile", PROFILE, "--budget", BUDGET)
    assert r.returncode == 0, r.stderr


def test_a_profile_at_exactly_the_requirement_passes(tmp_path):
    r = run(LINT, "--profile", profile_at(tmp_path, REQUIREMENT_PERCENT), "--budget", BUDGET)
    assert r.returncode == 0, r.stderr


def test_a_profile_at_2p5_percent_is_rejected_naming_both_entries(tmp_path):
    p = profile_at(tmp_path, 2.5)
    r = run(LINT, "--profile", p, "--budget", BUDGET)
    assert r.returncode == 1
    assert f"{p}: classes.rotor_speed.entries.esc_clock_error: 2.5 %" in r.stderr
    assert "esc_clock_error_max 0.02" in r.stderr and str(BUDGET) in r.stderr


def test_without_the_budget_the_profile_is_not_checked(tmp_path):
    r = run(LINT, "--profile", profile_at(tmp_path, 2.5))
    assert r.returncode == 0, r.stderr


def test_the_requirement_follows_the_budget_value(tmp_path):
    budget = tmp_path / "budget.yaml"
    budget.write_text(BUDGET.read_text(encoding="utf-8").replace("value: 0.02\n", "value: 0.019\n", 1), encoding="utf-8")
    assert "0.019" in budget.read_text(encoding="utf-8")
    r = run(LINT, "--profile", PROFILE, "--budget", budget)
    assert r.returncode == 1 and "esc_clock_error_max 0.019" in r.stderr


def test_a_card_reaches_its_profile_through_the_root(tmp_path):
    root = tmp_path / "root"
    (root / "sensors" / "profiles").mkdir(parents=True)
    shutil.copy(profile_at(tmp_path, 2.5), root / "sensors" / "profiles" / "marv_v2_board_default.yaml")
    r = run(LINT, "--card", CARD, "--budget", BUDGET, "--root", root)
    assert r.returncode == 1
    assert "esc_clock_error" in r.stderr and "esc_clock_error_max" in r.stderr
    assert run(LINT, "--card", CARD, "--budget", BUDGET).returncode == 0


def test_flatten_lint_rejects_the_profile_before_the_generator(tmp_path):
    root = tmp_path / "root"
    (root / "sensors" / "profiles").mkdir(parents=True)
    shutil.copy(profile_at(tmp_path, 2.5), root / "sensors" / "profiles" / "marv_v2_board_default.yaml")
    out = tmp_path / "out"
    out.mkdir()
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--scenario", SCENARIO, "--root", root,
            "--out-card", out / "c.yaml", "--out-register", out / "r.yaml", "--out-scenario", out / "s.yaml",
            "--out-gyro-chain", out / "g.yaml")
    assert r.returncode == 1
    assert "classes.rotor_speed.entries.esc_clock_error" in r.stderr and "esc_clock_error_max" in r.stderr
    assert not (out / "g.yaml").exists()


@pytest.mark.parametrize("unit", ["1", "ppm"])
def test_a_unit_other_than_percent_is_rejected(tmp_path, unit):
    p = profile_at(tmp_path, REQUIREMENT_PERCENT)
    text = p.read_text(encoding="utf-8")
    new, n = re.subn(r"(      esc_clock_error:\n        value: 2\n        unit: )\"%\"", lambda m: f'{m.group(1)}"{unit}"', text)
    assert n == 1
    p.write_text(new, encoding="utf-8")
    r = run(LINT, "--profile", p, "--budget", BUDGET)
    assert r.returncode == 1 and "esc_clock_error: unit" in r.stderr
