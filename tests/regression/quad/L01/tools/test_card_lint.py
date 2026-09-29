"""Card, profile and design-budget linter (tools/card/lint.py, schema.py). Frozen L01 tests.

Every rejection test mutates a copy of a committed data file that lints clean, so each negative control is one
change away from a passing file.
"""

import copy
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
LINT = ROOT / "tools" / "card" / "lint.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
FIXTURES = ROOT / "tests" / "regression" / "quad" / "L01" / "fixtures" / "card"

sys.path.insert(0, str(ROOT / "tools" / "card"))
import schema  # noqa: E402


def run_lint(flag, path, *extra):
    return subprocess.run(
        [sys.executable, str(LINT), flag, str(path), *extra], capture_output=True, text=True, check=False
    )


def load(path):
    return copy.deepcopy(schema.load_yaml(path))


def write(tmp_path, doc, name):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return p


def reject(flag, path, *needles, **kw):
    r = run_lint(flag, path, *kw.get("extra", ()))
    assert r.returncode == 1, f"expected rejection, got rc={r.returncode}\n{r.stderr}"
    assert r.stdout == ""
    lines = r.stderr.splitlines()
    for line in lines:
        assert line.startswith(f"{path}: "), line
    for needle in needles:
        assert any(needle in line for line in lines), f"no line contains {needle!r}:\n{r.stderr}"
    return lines


def card(tmp_path, mutate):
    doc = load(CARD)
    mutate(doc)
    return write(tmp_path, doc, "card.yaml")


def profile(tmp_path, mutate):
    doc = load(PROFILE)
    mutate(doc)
    return write(tmp_path, doc, "marv_v2_board_default.yaml")


def budget(tmp_path, mutate):
    doc = load(BUDGET)
    mutate(doc)
    return write(tmp_path, doc, "budget.yaml")


# The three committed data files lint clean.

@pytest.mark.parametrize("flag,path", [("--card", CARD), ("--profile", PROFILE), ("--budget", BUDGET)])
def test_committed_data_files_lint_clean(flag, path):
    r = run_lint(flag, path)
    assert (r.returncode, r.stdout, r.stderr) == (0, "", "")


def test_card_lint_also_lints_its_profile(tmp_path):
    root = tmp_path / "root"
    (root / "sensors" / "profiles").mkdir(parents=True)
    bad = load(PROFILE)
    bad["classes"]["imu"]["entries"]["gyro_fsr"]["sigma"] = 0
    write(root / "sensors" / "profiles", bad, "marv_v2_board_default.yaml")
    r = run_lint("--card", CARD, "--root", str(root))
    assert r.returncode == 1
    pfile = root / "sensors" / "profiles" / "marv_v2_board_default.yaml"
    lines = r.stderr.splitlines()
    assert lines and all(line.startswith(f"{pfile}: classes.imu.entries.gyro_fsr: sigma") for line in lines)
    assert any("sigma: 0 is rejected for method datasheet" in line for line in lines)


# The core 2.1 example card, verbatim, is rejected, and the reason names sigma.

def test_spec_example_card_is_rejected_for_missing_sigma():
    lines = reject("--card", FIXTURES / "spec_2_1_example_card.yaml", "sigma: missing")
    assert any(line.split(": ")[1] == "rotors.thrust_coeff" and "sigma" in line for line in lines)


# One rejection per rule.

@pytest.mark.parametrize("field", ["method", "source", "unit", "value"])
def test_missing_provenance_field(tmp_path, field):
    p = card(tmp_path, lambda d: d["mass"].pop(field))
    reject("--card", p, f"mass: {field}: missing")


def test_missing_sigma(tmp_path):
    p = card(tmp_path, lambda d: d["mass"].pop("sigma"))
    reject("--card", p, "mass: sigma: missing")


@pytest.mark.parametrize("method", ["published", "measured", "identified", "datasheet"])
def test_sigma_zero_rejected_on_sourced_methods(tmp_path, method):
    def m(d):
        d["mass"]["method"] = method
        d["mass"]["sigma"] = 0
    reject("--card", card(tmp_path, m), "mass: sigma: 0 is rejected for method " + method)


def test_sigma_zero_allowed_on_derived_and_negative_rejected(tmp_path):
    def zero(d):
        d["mass"]["method"] = "derived(sum of parts)"
        d["mass"]["sigma"] = 0.0
        d["mass"]["sigma_rule"] = "exact by definition"
        d["mass"].pop("conflict")
        d["mass"].pop("status")
    assert run_lint("--card", card(tmp_path, zero)).returncode == 0

    def neg(d):
        d["mass"]["sigma"] = -0.01
    reject("--card", card(tmp_path, neg), "mass: sigma: -0.01 is negative")


def test_numeric_sigma_rejected_on_design_budget(tmp_path):
    def m(d):
        d["mass"]["method"] = "design-budget"
        d["mass"]["sigma"] = 0.01
    reject("--card", card(tmp_path, m), "mass: sigma: a design-budget entry is a choice")
    reject("--budget", budget(tmp_path, lambda d: d["PM_min"].update(sigma=0.1)), "PM_min: sigma: a design-budget")


def test_unknown_sigma_rejected_on_scenario(tmp_path):
    def m(d):
        d["mass"]["method"] = "scenario"
        d["mass"]["sigma"] = "UNKNOWN"
        d["mass"].pop("sigma_rule")
    reject("--card", card(tmp_path, m), "mass: sigma: a scenario entry is a choice")


def test_scenario_and_design_budget_accept_choice(tmp_path):
    def m(d):
        d["mass"]["method"] = "scenario"
        d["mass"]["sigma"] = "choice"
        d["mass"].pop("sigma_rule")
        d["mass"].pop("conflict")
        d["mass"].pop("status")
    assert run_lint("--card", card(tmp_path, m)).returncode == 0


def test_choice_rejected_on_sourced_method(tmp_path):
    def m(d):
        d["mass"]["sigma"] = "choice"
        d["mass"].pop("sigma_rule")
    reject("--card", card(tmp_path, m), "mass: sigma: choice is only for design-budget and scenario")


def test_numeric_sigma_needs_sigma_rule(tmp_path):
    p = card(tmp_path, lambda d: d["mass"].pop("sigma_rule"))
    reject("--card", p, "mass: sigma_rule: required when sigma is numeric")


def test_sigma_rule_without_numeric_sigma_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["inertia_diag"].update(sigma_rule="stated by nobody"))
    reject("--card", p, "inertia_diag: sigma_rule: only allowed when a sigma component is numeric")


def test_unknown_value_needs_unknown_sigma(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["thrust_coeff"].update(value="UNKNOWN", sigma=0.5, sigma_rule="stated by x"))
    reject("--card", p, "rotors.thrust_coeff: sigma: value is UNKNOWN")
    ok = card(tmp_path, lambda d: d["rotors"]["thrust_coeff"].update(value="UNKNOWN", sigma="UNKNOWN"))
    assert run_lint("--card", ok).returncode == 0


# sigma: exact (integer counts only).

def _pole(d, **kw):
    d["classes"]["rotor_speed"]["entries"]["pole_count"].update(kw)


def test_exact_on_integer_datasheet_count_accepted(tmp_path):
    assert run_lint("--profile", profile(tmp_path, lambda d: _pole(d, sigma="exact"))).returncode == 0
    for method in ("published", "measured", "derived(pairs x 2)"):
        assert run_lint("--profile", profile(tmp_path, lambda d: _pole(d, sigma="exact", method=method))).returncode == 0


def test_exact_on_float_entry_rejected(tmp_path):
    p = profile(tmp_path, lambda d: _pole(d, value=14.0))
    reject("--profile", p)
    reject("--profile", profile(tmp_path, lambda d: _pole(d, value=14.5)), "pole_count: sigma: exact is only for an integer count")
    reject("--profile", profile(tmp_path, lambda d: _pole(d, value=14.0)), "sigma: exact is only for an integer count")


def test_exact_on_card_float_and_unknown_and_list_rejected(tmp_path):
    reject("--card", card(tmp_path, lambda d: d["mass"].update(sigma="exact")), "mass: sigma: exact is only for an integer count")
    reject("--profile", profile(tmp_path, lambda d: _pole(d, value="UNKNOWN")), "sigma: exact is only for an integer count")
    def lst(d):
        d["mass"].update(value=[1, 2, 3], shape="frd3", sigma="exact")
    reject("--card", card(tmp_path, lst), "sigma: exact is only for an integer count")


@pytest.mark.parametrize("method", ["identified", "design-budget", "scenario"])
def test_exact_rejected_on_identified_and_choice_methods(tmp_path, method):
    p = profile(tmp_path, lambda d: _pole(d, sigma="exact", method=method))
    reject("--profile", p, "pole_count: sigma:")


def test_exact_takes_no_sigma_rule_and_is_no_list_component(tmp_path):
    reject("--profile", profile(tmp_path, lambda d: _pole(d, sigma="exact", sigma_rule="counted")),
           "pole_count: sigma_rule: only allowed when a sigma component is numeric")
    def lst(d):
        d["inertia_diag"]["sigma"] = ["exact", "UNKNOWN", "UNKNOWN"]
    reject("--card", card(tmp_path, lst), "inertia_diag: sigma: 'exact' is not a finite number")


def test_budget_rejects_exact(tmp_path):
    reject("--budget", budget(tmp_path, lambda d: d["p"].update(sigma="exact")), "p: sigma: a design-budget entry is a choice")


# rotors.esc_map: a labelled scenario model entry.

def test_esc_map_is_a_scenario_model_entry():
    esc = schema.load_yaml(CARD)["rotors"]["esc_map"]
    assert esc["model"] == "linear_in_omega" and esc["method"] == "scenario" and esc["sigma"] == "choice"
    assert esc["status"] == ["INFERRED"] and not any(k in esc for k in ("value", "unit", "shape"))


def test_esc_map_missing_or_bad_model_rejected(tmp_path):
    reject("--card", card(tmp_path, lambda d: d["rotors"].pop("esc_map")), "rotors.esc_map: missing")
    reject("--card", card(tmp_path, lambda d: d["rotors"]["esc_map"].pop("model")), "rotors.esc_map: model: missing")
    reject("--card", card(tmp_path, lambda d: d["rotors"]["esc_map"].update(model="cubic")),
           "rotors.esc_map: model: 'cubic' is not one of linear_in_omega")


def test_esc_map_follows_sigma_policy_and_takes_no_number(tmp_path):
    reject("--card", card(tmp_path, lambda d: d["rotors"]["esc_map"].update(sigma="UNKNOWN")),
           "rotors.esc_map: sigma: a scenario entry is a choice")
    reject("--card", card(tmp_path, lambda d: d["rotors"]["esc_map"].update(value=1.0, unit="rad/s")),
           "rotors.esc_map: unknown field 'value'")
    reject("--card", card(tmp_path, lambda d: d["rotors"]["esc_map"].pop("source")), "rotors.esc_map: source: missing")


def test_open_conflict_requires_unverified(tmp_path):
    p = card(tmp_path, lambda d: d["mass"].pop("status"))
    reject("--card", p, "mass: conflict: an open conflict requires status [UNVERIFIED]")


def test_refuted_conflict_needs_refuting_log(tmp_path):
    def m(d):
        d["mass"]["conflict"][0]["state"] = "refuted"
    reject("--card", card(tmp_path, m), "refuted_by: required when state is refuted")

    def ok(d):
        d["mass"]["conflict"] = [dict(d["mass"]["conflict"][0], state="refuted", refuted_by="log 2026-10-01 run 3")]
    assert run_lint("--card", card(tmp_path, ok)).returncode == 0


def test_missing_spin_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["m2"].pop("spin"))
    reject("--card", p, "rotors.m2.spin: missing: a card without spin directions is rejected")


def test_card_with_no_spin_directions_rejected(tmp_path):
    def m(d):
        for n in ("m1", "m2", "m3", "m4"):
            d["rotors"][n].pop("spin")
    lines = reject("--card", card(tmp_path, m))
    assert sum("spin: missing" in line for line in lines) == 4


def test_spin_value_must_be_a_direction(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["m1"]["spin"].update(value="UNKNOWN"))
    reject("--card", p, "rotors.m1.spin: value: 'UNKNOWN' is not one of ccw, cw")


def test_spin_takes_no_sigma(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["m1"]["spin"].update(sigma="UNKNOWN"))
    reject("--card", p, "rotors.m1.spin: unknown field 'sigma'")


def test_missing_m3_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"].pop("m3"))
    reject("--card", p, "rotors.m3: missing")


def test_wrong_quadrant_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["m1"]["position"].update(value=[0.075, -0.10, 0.0]))
    reject("--card", p, "rotors.m1.position: value: m1 is front-right (+x, +y) in FRD")


def test_swapped_left_right_is_the_flu_mistake(tmp_path):
    def m(d):
        for n in ("m1", "m2", "m3", "m4"):
            v = d["rotors"][n]["position"]["value"]
            d["rotors"][n]["position"]["value"] = [v[0], -v[1], v[2]]
    lines = reject("--card", card(tmp_path, m))
    assert sum("wrong quadrant" in line for line in lines) == 4


@pytest.mark.parametrize("shape,value", [("frd3", [0.075, 0.10]), ("diag3", [1, 2, 3, 4]), ("range", [1, 2, 3])])
def test_bad_shape_length(tmp_path, shape, value):
    def m(d):
        d["inertia_diag"].update(shape=shape, value=value)
    reject("--card", card(tmp_path, m), f"inertia_diag: shape: {shape} needs")


def test_list_value_needs_shape(tmp_path):
    p = card(tmp_path, lambda d: d["inertia_diag"].pop("shape"))
    reject("--card", p, "inertia_diag: shape: required when value is a list")


def test_range_min_greater_than_max_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["rotors"]["speed_range"].update(value=[2800, 150]))
    reject("--card", p, "rotors.speed_range: shape: range needs min <= max")


def test_unknown_field_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["mass"].update(tolerance=0.01))
    reject("--card", p, "mass: unknown field 'tolerance'")
    reject("--card", card(tmp_path, lambda d: d.update(colour="red")), "colour: unknown top-level field")


def test_bad_status_tag_rejected(tmp_path):
    p = card(tmp_path, lambda d: d["mass"].update(status=["UNVERIFIED", "MAYBE"]))
    reject("--card", p, "mass: status: ['MAYBE'] not in UNVERIFIED, INFERRED")


@pytest.mark.parametrize(
    "lock,needle",
    [
        ({"by": "luis", "on": "2026-13-40", "via": "manual"}, "lock.on: must be a valid date"),
        ({"by": "luis", "on": "2026-09-29", "via": "auto"}, "lock.via: must be manual"),
        ({"on": "2026-09-29", "via": "manual"}, "lock.by: missing"),
        ({"by": "luis", "on": "2026-09-29", "via": "manual", "why": "x"}, "lock: unknown field 'why'"),
        ("locked", "lock: must be {by, on: YYYY-MM-DD, via: manual}"),
    ],
)
def test_bad_lock_rejected(tmp_path, lock, needle):
    p = card(tmp_path, lambda d: d["mass"].update(lock=lock))
    reject("--card", p, needle)


def test_good_lock_accepted_and_not_on_choice_entries(tmp_path):
    good = {"by": "luis", "on": "2026-09-29", "via": "manual"}
    assert run_lint("--card", card(tmp_path, lambda d: d["mass"].update(lock=good))).returncode == 0

    def m(d):
        d["mass"].update(method="scenario", sigma="choice", lock=good)
        d["mass"].pop("sigma_rule")
    reject("--card", card(tmp_path, m), "mass: lock: not allowed on a scenario entry")


def test_yaml_date_object_lock_accepted(tmp_path):
    p = tmp_path / "card.yaml"
    text = CARD.read_text(encoding="utf-8").replace(
        "  check: \"B1 log replay settles it\"\n",
        "  check: \"B1 log replay settles it\"\n  lock: {by: luis, on: 2026-09-29, via: manual}\n",
        1,
    )
    p.write_text(text, encoding="utf-8")
    assert run_lint("--card", p).returncode == 0


def test_missing_profile_file_rejected(tmp_path):
    p = card(tmp_path, lambda d: d.update(sensor_profile="no_such_profile"))
    reject("--card", p, "sensor_profile: profile file sensors/profiles/no_such_profile.yaml not found")


def test_profile_id_cannot_escape_the_profile_directory(tmp_path):
    p = card(tmp_path, lambda d: d.update(sensor_profile="../../vehicles/uzh_neurobem_5in"))
    reject("--card", p, "sensor_profile: must be a profile id")


def test_duplicate_yaml_key_rejected():
    reject("--card", FIXTURES / "duplicate_key_card.yaml", "duplicate key 'mass'")


def test_duplicate_key_in_budget_rejected(tmp_path):
    p = tmp_path / "budget.yaml"
    p.write_text(BUDGET.read_text(encoding="utf-8") + "\nk:\n  value: 3\n", encoding="utf-8")
    reject("--budget", p, "duplicate key 'k'")


def test_unreadable_file_rejected(tmp_path):
    reject("--card", tmp_path / "absent.yaml", "(file): cannot read")


# Budget.

@pytest.mark.parametrize("name", ["PM_min", "p"])
def test_budget_missing_rationale_rejected(tmp_path, name):
    p = budget(tmp_path, lambda d: d[name].pop("rationale"))
    reject("--budget", p, f"{name}: rationale: missing")


def test_budget_blank_rationale_rejected_even_when_value_unknown(tmp_path):
    p = budget(tmp_path, lambda d: d["p"].update(rationale="  "))
    reject("--budget", p, "p: rationale: missing or empty")


@pytest.mark.parametrize("used_by", [[], "QF-3", [""], None])
def test_budget_used_by_must_be_a_nonempty_list(tmp_path, used_by):
    def m(d):
        if used_by is None:
            d["k"].pop("used_by")
        else:
            d["k"]["used_by"] = used_by
    reject("--budget", budget(tmp_path, m), "k: used_by")


def test_budget_method_must_be_design_budget(tmp_path):
    p = budget(tmp_path, lambda d: d["k"].update(method="scenario"))
    reject("--budget", p, "k: method: a register entry is design-budget")


def test_budget_rejects_source_and_lock_fields(tmp_path):
    p = budget(tmp_path, lambda d: d["k"].update(source="somewhere"))
    reject("--budget", p, "k: unknown field 'source'")


def test_budget_unknown_value_keeps_choice_sigma(tmp_path):
    p = budget(tmp_path, lambda d: d["p"].update(sigma="UNKNOWN"))
    reject("--budget", p, "p: sigma: a design-budget entry is a choice")


def test_budget_holds_the_two_recorded_values():
    reg = schema.load_yaml(BUDGET)
    assert reg["PM_min"]["value"] == pytest.approx(3.141592653589793 / 4, rel=0, abs=1e-15)
    assert reg["PM_min"]["unit"] == "rad"
    assert reg["chi2_gate_quantile"]["value"] == 0.999 and reg["chi2_gate_quantile"]["unit"] == "1"
    for name in ("PM_min", "chi2_gate_quantile"):
        assert ".ts:" in reg[name]["rationale"]
    assert all(e["value"] == "UNKNOWN" for n, e in reg.items() if n not in ("PM_min", "chi2_gate_quantile"))


# Profile.

@pytest.mark.parametrize("cls", ["imu", "high_g_accel", "barometer", "rotor_speed"])
def test_profile_requires_core_classes(tmp_path, cls):
    p = profile(tmp_path, lambda d: d["classes"].pop(cls))
    reject("--profile", p, f"classes.{cls}: missing")


def test_profile_class_needs_part_and_entries(tmp_path):
    p = profile(tmp_path, lambda d: d["classes"]["imu"].pop("part"))
    reject("--profile", p, "classes.imu.part: missing or empty")


def test_profile_sigma_zero_rejected(tmp_path):
    p = profile(tmp_path, lambda d: d["classes"]["barometer"]["entries"]["noise"].update(sigma=0))
    reject("--profile", p, "classes.barometer.entries.noise: sigma: 0 is rejected for method datasheet")


def test_profile_plus_minus_figure_must_not_become_sigma_without_rule(tmp_path):
    p = profile(tmp_path, lambda d: d["classes"]["imu"]["entries"]["gyro_zero_rate_offset"].update(sigma=0.4))
    reject("--profile", p, "gyro_zero_rate_offset: sigma_rule: required when sigma is numeric")


def test_profile_id_must_match_file_name(tmp_path):
    p = profile(tmp_path, lambda d: d.update(profile="other"))
    reject("--profile", p, "profile: id 'other' must equal the file name")


def test_profile_unknown_class_rejected(tmp_path):
    p = profile(tmp_path, lambda d: d["classes"].update(sonar={"part": "x", "entries": {}}))
    reject("--profile", p, "classes.sonar: unknown sensor class")


# The mass conflict is recorded, not resolved (owner decision 2026-09-29).

def test_neurobem_card_keeps_mass_conflict_unverified():
    mass = schema.load_yaml(CARD)["mass"]
    assert mass["value"] == 0.772 and mass["status"] == ["UNVERIFIED"]
    assert [c["value"] for c in mass["conflict"]] == [0.752, 0.752]
    assert all(c["state"] == "open" for c in mass["conflict"])


def test_single_number_sigma_on_vector_rejected(tmp_path):
    def mutate(d):
        d["inertia_diag"]["sigma"] = 0.0001
        d["inertia_diag"]["sigma_rule"] = "test value"
    p = card(tmp_path, mutate)
    reject("--card", p, "inertia_diag: sigma: a vector value needs one sigma per component")


def test_per_component_sigma_on_vector_accepted(tmp_path):
    def mutate(d):
        d["inertia_diag"]["sigma"] = [0.0001, 0.0001, 0.0001]
        d["inertia_diag"]["sigma_rule"] = "test value"
    p = card(tmp_path, mutate)
    assert run_lint("--card", p).returncode == 0
