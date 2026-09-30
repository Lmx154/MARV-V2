"""Scenario schema (tools/sim/scenario.py) and the committed L2 scenarios (scenarios/quad/L02/*.yaml). L02 tests.

Every refusal is a negative control: the same mutation of a valid scenario must be refused, while the unmutated
scenario (and a legal variant of the mutated field, where one exists) is accepted.
"""

import copy
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import scenario  # noqa: E402
import schema  # noqa: E402

SCEN_DIR = ROOT / "scenarios" / "quad" / "L02"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
NAMES = ("free_fall", "rotation", "hover", "determinism")
INERTIA = [float(x) for x in schema.load_yaml(CARD)["inertia_diag"]["value"]]


def raw(name):
    return schema.load_yaml(SCEN_DIR / f"{name}.yaml")


def findings(doc, name="free_fall", inertia=INERTIA):
    return scenario.validate(doc, f"{name}.yaml", inertia)


@pytest.mark.parametrize("name", NAMES)
def test_committed_scenarios_are_valid(name):
    doc = scenario.load(SCEN_DIR / f"{name}.yaml", CARD)
    assert doc["scenario"] == name


def test_committed_scenario_values():
    for name in NAMES:
        doc = raw(name)
        assert doc["tick_period_num_us"]["value"] == 625 and doc["tick_period_den"]["value"] == 4
        assert doc["seed"]["value"] == 1 and "RESERVED" in doc["seed"]["rationale"]
        assert doc["m_sequence"]["value"] == ([1] if name == "determinism" else [4, 2, 1])
        rates = doc["initial_state"]["body_rates_frd_rad_s"]["value"]
        assert any(rates) == (name == "rotation")
        for m in doc["m_sequence"]["value"]:
            assert doc["duration_ticks"]["value"] % m == 0
    assert set(raw("hover")["command"]["hover"]["value"]) == {"lo", "hi"}
    d = raw("determinism")["command"]["dshot"]["value"]
    assert len(set(d)) == 4
    assert scenario.tick_period_s(raw("free_fall")).denominator == 6400


def test_rotation_is_off_the_separatrix_with_the_cards_inertia():
    doc = raw("rotation")
    w = doc["initial_state"]["body_rates_frd_rad_s"]["value"]
    mu = scenario.separatrix_mu(INERTIA, w)
    assert mu >= doc["separatrix_margin_min"]["value"]
    # independent restatement of the same quantity for this card (I_mid = ixx, I_max = izz)
    ix, iy, iz = INERTIA
    assert iy < ix < iz
    l2 = (ix * w[0]) ** 2 + (iy * w[1]) ** 2 + (iz * w[2]) ** 2
    two_e = ix * w[0] ** 2 + iy * w[1] ** 2 + iz * w[2] ** 2
    assert l2 > two_e * ix
    assert math.isclose(mu, (l2 - two_e * ix) / (two_e * (iz - ix)), rel_tol=1e-12)


def test_determinism_command_gives_thrust_and_all_three_torques_with_the_card():
    cfg = gpc.plant_config(CARD)
    d = raw("determinism")["command"]["dshot"]["value"]
    span = 2047 - 48
    omega = [cfg["omega_min_rad_s"] + (cfg["omega_max_rad_s"] - cfg["omega_min_rad_s"]) * (x - 48) / span for x in d]
    t = [cfg["thrust_coeff"] * w * w for w in omega]
    pos = cfg["rotor_position_frd_m"]
    roll = sum(-p[1] * ti for p, ti in zip(pos, t))
    pitch = sum(p[0] * ti for p, ti in zip(pos, t))
    yaw = sum(s * cfg["torque_ratio_m"] * ti for s, ti in zip(cfg["yaw_sign"], t))
    assert sum(t) > 0 and roll != 0 and pitch != 0 and yaw != 0


def test_valid_base_passes():
    for name in NAMES:
        assert findings(raw(name), name) == []


def mutate(name, fn):
    doc = copy.deepcopy(raw(name))
    fn(doc)
    return findings(doc, name)


@pytest.mark.parametrize("field", scenario.TOP_FIELDS)
def test_missing_field_refused(field):
    out = mutate("free_fall", lambda d: d.pop(field))
    assert any(field in f for f in out)


@pytest.mark.parametrize("field", scenario.STATE_FIELDS)
def test_missing_state_field_refused(field):
    out = mutate("free_fall", lambda d: d["initial_state"].pop(field))
    assert any(field in f and "missing" in f for f in out)


def test_unlabelled_number_refused():
    out = mutate("free_fall", lambda d: d.update(site_height_m=500.0))
    assert any("unlabelled number" in f and "site_height_m" in f for f in out)
    out = mutate("free_fall", lambda d: d["initial_state"].update(velocity_ned_m_s=[0.0, 0.0, 0.0]))
    assert any("unlabelled number" in f for f in out)
    out = mutate("free_fall", lambda d: d["site_height_m"].update(extra_note=3))
    assert any("unlabelled number" in f for f in out)


@pytest.mark.parametrize("mutation, text", [
    (lambda d: d["site_height_m"].pop("label"), "missing field 'label'"),
    (lambda d: d["site_height_m"].update(label="guess"), "not one of"),
    (lambda d: d["site_height_m"].pop("rationale"), "needs a non-empty rationale"),
    (lambda d: d["site_height_m"].update(rationale="two\nlines"), "one line"),
    (lambda d: d["site_height_m"].pop("unit"), "missing field 'unit'"),
    (lambda d: d["m_sequence"].pop("rule"), "needs a non-empty rule"),
    (lambda d: d["m_sequence"].update(bogus="x"), "unknown entry field"),
])
def test_label_and_rationale_rules(mutation, text):
    out = mutate("free_fall", mutation)
    assert any(text in f for f in out), out


@pytest.mark.parametrize("bad", [1, 2, 47, 2048, 4096, -1, 100.5, 1e3, True, "1000", None])
def test_illegal_dshot_refused(bad):
    def put(d):
        d["command"]["dshot"]["value"] = [0, 0, 0, bad]
    out = mutate("free_fall", put)
    assert any("command.dshot" in f and "motor 4" in f for f in out), out


@pytest.mark.parametrize("good", [0, 48, 1000, 2047])
def test_legal_dshot_accepted(good):
    def put(d):
        d["command"]["dshot"]["value"] = [good] * 4
    assert mutate("free_fall", put) == []


def test_dshot_length_and_command_kind_refused():
    out = mutate("free_fall", lambda d: d["command"]["dshot"].update(value=[0, 0, 0]))
    assert any("list of 4 integers" in f for f in out)
    out = mutate("free_fall", lambda d: d["command"].update(hover=copy.deepcopy(raw("hover")["command"]["hover"])))
    assert any("exactly one of" in f for f in out)
    out = mutate("free_fall", lambda d: d["command"].pop("dshot"))
    assert any("exactly one of" in f for f in out)
    out = mutate("hover", lambda d: d["command"]["hover"].update(value=["mid"]))
    assert any("subset" in f for f in out)


@pytest.mark.parametrize("bad", [2.5, 2.0, True, "2", 0, -1, None])
def test_non_integer_or_nonpositive_m_refused(bad):
    out = mutate("free_fall", lambda d: d["m_sequence"].update(value=[4, bad, 1]))
    assert any("m_sequence" in f for f in out), out


def test_m_sequence_shape_refused_and_valid_variants_accepted():
    assert any("m_sequence" in f for f in mutate("free_fall", lambda d: d["m_sequence"].update(value=[])))
    assert any("m_sequence" in f for f in mutate("free_fall", lambda d: d["m_sequence"].update(value=[2, 2])))
    assert mutate("free_fall", lambda d: d["m_sequence"].update(value=[8, 4, 2, 1])) == []
    assert mutate("free_fall", lambda d: d["m_sequence"].update(value=[1])) == []


def test_duration_must_be_a_multiple_of_every_m():
    out = mutate("free_fall", lambda d: d["duration_ticks"].update(value=6402))
    assert any("not a multiple of m = 4" in f for f in out)
    assert any("duration_ticks" in f for f in mutate("free_fall", lambda d: d["duration_ticks"].update(value=0)))
    assert any("duration_ticks" in f for f in mutate("free_fall", lambda d: d["duration_ticks"].update(value=1.5)))


def test_scalar_fields_refused():
    assert any("site_latitude_rad" in f for f in mutate("free_fall", lambda d: d["site_latitude_rad"].update(value=2.0)))
    assert any("seed" in f for f in mutate("free_fall", lambda d: d["seed"].update(value=-1)))
    assert any("seed" in f for f in mutate("free_fall", lambda d: d["seed"].update(value=1.0)))
    assert any("tick_period_den" in f for f in mutate("free_fall", lambda d: d["tick_period_den"].update(value=0)))
    assert any("tick_period_num_us" in f
               for f in mutate("free_fall", lambda d: d["tick_period_num_us"].update(value=625.0)))


def test_unknown_field_and_stem_mismatch_refused():
    assert any("unknown field" in f for f in mutate("free_fall", lambda d: d.update(extra=1)))
    doc = raw("free_fall")
    assert any("file stem" in f for f in findings(doc, "rotation"))


def test_attitude_rules():
    def put(q):
        return lambda d: d["initial_state"]["attitude_q_wxyz"].update(value=q)
    assert any("not a unit" in f or "|q|^2" in f for f in mutate("free_fall", put([1.0, 0.1, 0.0, 0.0])))
    assert any("w = " in f for f in mutate("free_fall", put([-1.0, 0.0, 0.0, 0.0])))
    assert any("attitude_q_wxyz" in f for f in mutate("free_fall", put([1.0, 0.0, 0.0])))
    assert mutate("free_fall", put([0.0, 1.0, 0.0, 0.0])) == []


def rates(w):
    return lambda d: d["initial_state"]["body_rates_frd_rad_s"].update(value=w)


def test_separatrix_rules():
    # pure spin about the intermediate axis (x for this card) is on the separatrix: mu = 0
    out = mutate("rotation", rates([1.0, 0.0, 0.0]))
    assert any("distance to the separatrix" in f for f in out), out
    # nonzero rates in a scenario without the margin field
    out = mutate("free_fall", rates([2.0, 3.0, 5.0]))
    assert any("separatrix_margin_min: missing" in f for f in out)
    # a margin with zero rates
    doc = copy.deepcopy(raw("free_fall"))
    doc["separatrix_margin_min"] = copy.deepcopy(raw("rotation")["separatrix_margin_min"])
    assert any("body rates are zero" in f for f in findings(doc))
    # no card, nonzero rates
    assert any("no card given" in f for f in scenario.validate(raw("rotation"), "rotation.yaml", None))
    # a spin about the maximum axis is far from the separatrix and accepted
    assert mutate("rotation", rates([0.0, 0.0, 5.0])) == []
    # repeated principal moments have no separatrix
    assert any("distinct" in f for f in scenario.validate(raw("rotation"), "rotation.yaml", [0.0025, 0.0025, 0.0043]))
    # the margin itself
    doc = copy.deepcopy(raw("rotation"))
    doc["separatrix_margin_min"]["value"] = 0.9
    assert any("below separatrix_margin_min" in f for f in findings(doc, "rotation"))


def test_separatrix_mu_both_sides_and_extremes():
    assert scenario.separatrix_mu(INERTIA, [0.0, 0.0, 1.0]) == pytest.approx(1.0)
    assert scenario.separatrix_mu(INERTIA, [0.0, 1.0, 0.0]) == pytest.approx(1.0)
    assert scenario.separatrix_mu(INERTIA, [1.0, 0.0, 0.0]) == pytest.approx(0.0, abs=1e-12)
    assert 0.0 < scenario.separatrix_mu(INERTIA, [2.0, 3.0, -5.0]) < 1.0


def test_load_refuses_duplicate_keys_and_a_missing_file(tmp_path):
    p = tmp_path / "free_fall.yaml"
    p.write_text("scenario: free_fall\nscenario: free_fall\n", encoding="utf-8")
    with pytest.raises(scenario.ScenarioError):
        scenario.load(p, CARD)
    with pytest.raises(scenario.ScenarioError):
        scenario.load(tmp_path / "absent.yaml", CARD)


def test_iterations():
    doc = raw("free_fall")
    assert [scenario.iterations(doc, m) for m in (4, 2, 1)] == [1600, 3200, 6400]
