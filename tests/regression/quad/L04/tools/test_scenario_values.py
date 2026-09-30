"""The scenario-values register (design/scenario_values.yaml): lint (tools/card/lint.py --scenario) and the flow into
the product parameter set (tools/card/flatten.py --scenario / --out-scenario, then params_gen).

The tests assert rules on the register as it is read at test time (schema.py), not its values: the owner's values are
recorded in decision 0005 and a legitimate change to the register must not break a frozen test.
"""

import json
import math
import re
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import schema  # noqa: E402

LINT = ROOT / "tools" / "card" / "lint.py"
FLATTEN = ROOT / "tools" / "card" / "flatten.py"
GEN = ROOT / "tools" / "gen" / "params_gen.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"

# name -> (declared type, unit): an integer is an i32 parameter, any other number f32.
EXPECT = {
    "tick_period_num_us": ("i32", "us"),
    "tick_period_den": ("i32", "1"),
    "rate_loop_divisor": ("i32", "1"),
    "rate_max_roll": ("f32", "rad/s"),
    "rate_max_pitch": ("f32", "rad/s"),
    "rate_max_yaw": ("f32", "rad/s"),
}
INTEGERS = ("tick_period_num_us", "tick_period_den", "rate_loop_divisor")
RATE_MAX = ("rate_max_roll", "rate_max_pitch", "rate_max_yaw")
OTHER_OUTPUTS = ("card.yaml", "register.yaml", "mixer.yaml")


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def flatten(d, scenario=None):
    d.mkdir(parents=True, exist_ok=True)
    args = [FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", d / "card.yaml",
            "--out-register", d / "register.yaml", "--out-mixer", d / "mixer.yaml", "--root", ROOT]
    if scenario is not None:
        args += ["--scenario", scenario, "--out-scenario", d / "scenario.yaml"]
    return run(*args)


def plant(tmp, name, pattern, repl, entry="rate_loop_divisor"):
    """A copy of the committed register with the one match of the regex `pattern` inside entry `entry` replaced by
    `repl` (independent of the entry's value)."""
    text = SCENARIO.read_text(encoding="utf-8")
    block = re.search(rf"^{entry}:\n(?:[ \t]+.*\n|\n)*?(?=^\S|\Z)", text, flags=re.M)
    assert block, entry
    changed, n = re.subn(pattern, repl, block.group(0), flags=re.M)
    assert n == 1, (pattern, n)
    path = tmp / f"{name}.yaml"
    path.write_text(text[:block.start()] + changed + text[block.end():], encoding="utf-8")
    return path


def reject(path, fragment):
    r = run(LINT, "--scenario", path)
    assert r.returncode == 1, r.stdout + r.stderr
    assert fragment in r.stderr, r.stderr


def test_committed_register_lints_clean():
    r = run(LINT, "--scenario", SCENARIO)
    assert r.returncode == 0, r.stderr


def test_register_holds_the_six_entries_each_scenario_choice_with_declared_type_rationale_and_used_by():
    doc = schema.load_yaml(SCENARIO)
    for name, (ptype, unit) in EXPECT.items():
        assert name in doc, name
    for name, e in doc.items():
        assert e["method"] == "scenario" and e["sigma"] == "choice", name
        assert e["rationale"].strip() and e["used_by"], name
    for name, (ptype, unit) in EXPECT.items():
        e = doc[name]
        assert e["unit"] == unit, name
        assert (ptype == "i32") == (name in INTEGERS), name
        assert (type(e["value"]) is int) == (ptype == "i32"), name


# negative controls: each planted defect must be refused, with exit status 1


def test_lint_rejects_an_entry_without_rationale(tmp_path):
    text = SCENARIO.read_text(encoding="utf-8")
    lines = text.split("\n")
    start = next(i for i, ln in enumerate(lines) if ln.startswith("rate_loop_divisor:"))
    end = next(i for i in range(start + 1, len(lines)) if lines[i].startswith("rate_max_roll:"))
    block = [ln for ln in lines[start:end] if not ln.startswith("  rationale:") and not ln.startswith("    ")]
    bad = tmp_path / "no_rationale.yaml"
    bad.write_text("\n".join(lines[:start] + block + lines[end:]), encoding="utf-8")
    assert "rationale" not in "\n".join(block)
    reject(bad, "rate_loop_divisor: rationale: missing")


def test_lint_rejects_a_method_other_than_scenario(tmp_path):
    reject(plant(tmp_path, "budget_method", r"^  method: scenario$", "  method: design-budget"),
           "method: a register entry is scenario, not 'design-budget'")
    reject(plant(tmp_path, "measured", r"^  method: scenario$", "  method: measured"),
           "method: a register entry is scenario, not 'measured'")


def test_lint_rejects_a_bare_number_in_place_of_an_entry(tmp_path):
    bad = tmp_path / "bare.yaml"
    bad.write_text(SCENARIO.read_text(encoding="utf-8") + "\nextra_gain: 3.0\n", encoding="utf-8")
    reject(bad, "extra_gain: must be a mapping of entry fields")


def test_lint_rejects_a_missing_used_by_and_a_numeric_sigma(tmp_path):
    reject(plant(tmp_path, "used_by", r"^  used_by: .*\n", ""), "rate_loop_divisor: used_by: missing")
    reject(plant(tmp_path, "sigma", r"^  sigma: choice$", "  sigma: 0.1"),
           "sigma: a scenario entry is a choice and has no uncertainty")


def test_the_budget_register_still_refuses_a_scenario_method(tmp_path):
    bad = tmp_path / "budget_scenario.yaml"
    bad.write_text(BUDGET.read_text(encoding="utf-8").replace("\n  method: design-budget", "\n  method: scenario", 1),
                   encoding="utf-8")
    r = run(LINT, "--budget", bad)
    assert r.returncode == 1 and "a register entry is design-budget, not 'scenario'" in r.stderr, r.stderr


def test_flatten_refuses_a_bad_scenario_register_and_writes_nothing(tmp_path):
    bad = plant(tmp_path, "bad", r"^  method: scenario$", "  method: measured")
    d = tmp_path / "flat"
    r = flatten(d, bad)
    assert r.returncode == 1 and "a register entry is scenario" in r.stderr
    assert not d.exists() or not list(d.iterdir())


# flatten: the flag is optional and changes nothing else


def test_scenario_flag_and_out_scenario_are_given_together(tmp_path):
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", tmp_path / "c", "--out-register",
            tmp_path / "r", "--scenario", SCENARIO)
    assert r.returncode == 2 and "given together" in r.stderr
    assert not (tmp_path / "c").exists()


def test_without_the_flag_no_scenario_file_and_the_other_outputs_are_the_same_with_any_register(tmp_path):
    """Byte identity without the flag: the flag-omitted run is compared with runs that pass a register (the committed
    one and an empty one). The outputs of the card, budget and mixer files are the same in all three, so the scenario
    code path cannot perturb them, and the flag-omitted run writes no scenario file. (The one-off comparison against
    the tool at HEAD before this change is in the change report; a committed expectation would be invented.)"""
    empty = tmp_path / "empty.yaml"
    empty.write_text("{}\n", encoding="utf-8")
    assert flatten(tmp_path / "none").returncode == 0
    assert flatten(tmp_path / "full", SCENARIO).returncode == 0
    assert flatten(tmp_path / "empty", empty).returncode == 0
    assert sorted(p.name for p in (tmp_path / "none").iterdir()) == sorted(OTHER_OUTPUTS)
    for name in OTHER_OUTPUTS:
        base = (tmp_path / "none" / name).read_bytes()
        assert base
        assert (tmp_path / "full" / name).read_bytes() == base, name
        assert (tmp_path / "empty" / name).read_bytes() == base, name
    body = [ln for ln in (tmp_path / "empty" / "scenario.yaml").read_text().splitlines() if not ln.startswith("#")]
    assert body == []


def test_flatten_is_byte_stable(tmp_path):
    assert flatten(tmp_path / "a", SCENARIO).returncode == 0
    assert flatten(tmp_path / "b", SCENARIO).returncode == 0
    assert (tmp_path / "a" / "scenario.yaml").read_bytes() == (tmp_path / "b" / "scenario.yaml").read_bytes()


@pytest.fixture(scope="module")
def flat(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("flat")
    r = flatten(tmp / "f", SCENARIO)
    assert r.returncode == 0, r.stderr
    doc = schema.load_yaml(tmp / "f" / "scenario.yaml")
    out = tmp / "gen"
    g = run(GEN, "--card", tmp / "f" / "card.yaml", "--card", tmp / "f" / "mixer.yaml",
            "--card", tmp / "f" / "scenario.yaml", "--register", tmp / "f" / "register.yaml", "--out-dir", out)
    assert g.returncode == 0, g.stderr
    return {"doc": doc, "gen": out, "dir": tmp / "f"}


def test_the_six_params_appear_with_type_value_unit_method_and_sigma(flat):
    register = schema.load_yaml(SCENARIO)
    assert list(flat["doc"]) == list(register)
    for name, (ptype, unit) in EXPECT.items():
        e, value = flat["doc"][name], register[name]["value"]
        assert e["type"] == ptype, name
        assert e["value"] == value and type(e["value"]) is type(value), name
        assert e["unit"] == unit and e["method"] == "scenario" and e["sigma"] == "choice", name
        assert "design/scenario_values.yaml" in e["source"] and name in e["source"], name


def test_params_gen_takes_them_as_the_declared_types(flat):
    params = json.loads((flat["gen"] / "params_manifest.json").read_text())["params"]
    text = (flat["gen"] / "param_defaults.cpp").read_text()
    register = schema.load_yaml(SCENARIO)
    for name, (ptype, unit) in EXPECT.items():
        value = register[name]["value"]
        assert params[name]["type"] == ptype and params[name]["unit"] == unit, name
        m = re.search(rf"    // {name}\n    \{{\{{ParamType::(\w+), ([^,]+), ([^}}]+)\}}, .*?\n(.*?)\}},\n", text, flags=re.S)
        assert m, name
        assert m.group(1) == ptype.upper(), name
        assert "ParamMethod::Scenario" in m.group(0) and "SigmaKind::Choice" in m.group(0), name
        if ptype == "i32":
            assert int(m.group(3)) == value, name
        else:
            f32 = struct.unpack("<f", struct.pack("<f", value))[0]
            assert float(m.group(2).rstrip("f")) == pytest.approx(f32, rel=1e-7), name


def test_rate_max_is_finite_and_positive():
    doc = schema.load_yaml(SCENARIO)
    for name in RATE_MAX:
        v = doc[name]["value"]
        assert math.isfinite(v) and v > 0, name


def test_tick_period_is_a_positive_rational_and_the_rate_loop_divisor_a_positive_integer():
    doc = schema.load_yaml(SCENARIO)
    num, den = doc["tick_period_num_us"]["value"], doc["tick_period_den"]["value"]
    div = doc["rate_loop_divisor"]["value"]
    assert type(num) is int and type(den) is int and num > 0 and den > 0
    assert type(div) is int and div > 0


def test_l04_param_ids_lists_the_six():
    ids = (ROOT / "tests" / "regression" / "quad" / "L04" / "param_ids").read_text().split()
    ids = [i for i in ids if not i.startswith("#")]
    for name in EXPECT:
        assert ids.count(name) == 1
