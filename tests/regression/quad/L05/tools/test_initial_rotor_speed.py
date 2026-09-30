"""Initial rotor speed (decision 0007): the optional initial_state.rotor_speed_rad_s in the L2 schema
(tools/sim/scenario.py) and the L5 schema (tools/sim/l5_scenario.py), its generated world element
(tools/card/gen_world.py), the L4 and L5 runners' pass-through and the hover helper (tools/sim/run_l5.py
hover_rotor_speeds). No gz.

Every acceptance test has a control that breaks it: the defect the test refuses is present in the control, or the
absence it asserts is contradicted by the same document with the field added.
"""

import copy
import math
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import gen_world  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import run_l4  # noqa: E402
import run_l5  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
L2 = ROOT / "scenarios" / "quad" / "L02"
L4 = ROOT / "scenarios" / "quad" / "L04"
L5 = ROOT / "scenarios" / "quad" / "L05"
REGISTER = schema.load_yaml(l5s.REGISTER)
BUILD = {"tick_period_num_us": 625, "tick_period_den": 4, "rate_loop_divisor": 2, "att_loop_ratio": 2}
SPEEDS = [1000.0, 1100.5, 1200.25, 1300.125]  # scenario values of this test: distinct, in the card's speed range
ELEMENT = "initial_rotor_speed_rad_s"


def entry(value, label="scenario"):
    e = {"value": value, "unit": "rad/s", "label": label}
    e["rationale" if label == "scenario" else "rule"] = "test value"
    return e


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def l2_doc(speeds=None, name="free_fall"):
    doc = load_yaml(L2 / f"{name}.yaml")
    if speeds is not None:
        doc["initial_state"]["rotor_speed_rad_s"] = entry(speeds)
    return doc


def l5_doc(value=None, name="step_roll"):
    doc = load_yaml(L5 / f"{name}.yaml")
    if value is not None:
        doc["initial_state"]["rotor_speed_rad_s"] = entry(value, "derived" if value == "hover" else "scenario")
    return doc


# ---- the L2 schema --------------------------------------------------------------------------------------------------

def test_l2_schema_accepts_the_field_and_values_carries_it():
    doc = l2_doc(SPEEDS)
    assert scn.validate(doc, "free_fall.yaml") == []
    assert scn.values(doc)["initial_state"]["rotor_speed_rad_s"] == SPEEDS


def test_l2_schema_absent_field_is_valid_and_values_has_no_key():
    doc = l2_doc()
    assert scn.validate(doc, "free_fall.yaml") == []
    assert "rotor_speed_rad_s" not in scn.values(doc)["initial_state"]
    assert "rotor_speed_rad_s" in scn.values(l2_doc(SPEEDS))["initial_state"]  # control: the field would show


@pytest.mark.parametrize("bad", ([1.0, 2.0, 3.0], [1.0] * 5, [1.0, 2.0, 3.0, -0.5], [1.0, 2.0, 3.0, math.nan],
                                 [1.0, 2.0, 3.0, math.inf], "hover", 5.0))
def test_l2_schema_refuses_a_malformed_field(bad):
    assert scn.validate(l2_doc(bad), "free_fall.yaml") != []
    assert scn.validate(l2_doc(SPEEDS), "free_fall.yaml") == []  # control: the well-formed field is accepted


def test_l2_schema_still_refuses_an_unknown_state_field_and_a_bare_number():
    doc = l2_doc(SPEEDS)
    doc["initial_state"]["rotor_rate"] = entry(SPEEDS)
    assert any("unknown field" in f for f in scn.validate(doc, "free_fall.yaml"))
    doc = l2_doc()
    doc["initial_state"]["rotor_speed_rad_s"] = SPEEDS
    assert scn.validate(doc, "free_fall.yaml") != []
    assert scn.validate(l2_doc(SPEEDS), "free_fall.yaml") == []  # control


# ---- gen_world ------------------------------------------------------------------------------------------------------

def world(tmp_path, doc, name="free_fall"):
    doc = copy.deepcopy(doc)
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    _, text = gen_world.generate(str(CARD), str(path), "test", 1)
    return text


def element_values(text):
    plugin = ET.fromstring(text.split("?>", 1)[1]).find(".//plugin[@filename='marv_gz_lockstep']")
    els = plugin.findall(ELEMENT)
    return [[float(x) for x in e.text.split()] for e in els]


def test_gen_world_writes_the_element_exactly_once_and_round_trips_bit_for_bit(tmp_path):
    got = element_values(world(tmp_path, l2_doc([0.1, 1.0 / 3.0, 1234.5678901234567, 0.0])))
    assert got == [[0.1, 1.0 / 3.0, 1234.5678901234567, 0.0]]


def test_gen_world_writes_no_element_and_the_world_is_unchanged_when_the_field_is_absent(tmp_path):
    plain = world(tmp_path, l2_doc())
    assert ELEMENT not in plain
    with_field = world(tmp_path, l2_doc(SPEEDS))
    assert ELEMENT in with_field  # control: the field does add the element
    assert with_field.replace(next(ln for ln in with_field.splitlines() if ELEMENT in ln) + "\n", "") == plain


def test_gen_world_writes_the_element_for_an_explicit_all_zero_field(tmp_path):
    assert element_values(world(tmp_path, l2_doc([0.0] * 4))) == [[0.0] * 4]  # present means written, even when 0


# ---- the runners' pass-through --------------------------------------------------------------------------------------

def test_l4_runner_passes_the_field_through_and_only_when_present():
    plan = types.SimpleNamespace(m_sequence=(2, 1), duration_ticks=1000)
    doc = load_yaml(L4 / "step_roll.yaml")
    assert "rotor_speed_rad_s" not in run_l4.l2_scenario_doc(doc, plan, "x", "src")["initial_state"]
    doc["initial_state"]["rotor_speed_rad_s"] = entry(SPEEDS)
    assert run_l4.l2_scenario_doc(doc, plan, "x", "src")["initial_state"]["rotor_speed_rad_s"]["value"] == SPEEDS


def plan_of(doc):
    return run_l5.plan(doc, BUILD, CARD)


def test_l5_schema_accepts_a_list_and_hover_and_refuses_malformed_ones():
    assert l5s.validate(l5_doc(), "step_roll.yaml", REGISTER) == []
    assert l5s.validate(l5_doc(SPEEDS), "step_roll.yaml", REGISTER) == []
    assert l5s.validate(l5_doc("hover"), "step_roll.yaml", REGISTER) == []
    for bad in ([1.0, 2.0, 3.0], [1.0, 2.0, 3.0, -1.0], "spin", [1.0, 2.0, 3.0, math.nan]):
        assert l5s.validate(l5_doc(bad), "step_roll.yaml", REGISTER) != [], bad
    doc = l5_doc("hover")
    doc["initial_state"]["rotor_speed_rad_s"]["label"] = "scenario"
    doc["initial_state"]["rotor_speed_rad_s"]["rationale"] = "x"
    assert any("label must be derived" in f for f in l5s.validate(doc, "step_roll.yaml", REGISTER))


def test_l5_plan_resolves_hover_and_passes_a_list_and_leaves_absent_absent():
    hover = plan_of(l5_doc("hover")).rotor_speed_rad_s
    assert hover == run_l5.hover_rotor_speeds(str(CARD), l5s.values(l5_doc("hover")))
    assert len(hover) == 4 and hover != tuple(SPEEDS)
    assert plan_of(l5_doc(SPEEDS)).rotor_speed_rad_s == tuple(SPEEDS)
    assert plan_of(l5_doc()).rotor_speed_rad_s is None
    assert plan_of(l5_doc(SPEEDS)).rotor_speed_rad_s != plan_of(l5_doc()).rotor_speed_rad_s  # control


def test_l5_l2_doc_carries_the_resolved_speed_only_when_the_plan_has_one():
    p = plan_of(l5_doc("hover"))
    out = run_l5.l2_scenario_doc(l5_doc("hover"), p, "x", "src")
    assert out["initial_state"]["rotor_speed_rad_s"]["value"] == list(p.rotor_speed_rad_s)
    assert scn.validate(out | {"scenario": "x"}, "x.yaml") == []  # the generated L2 document is itself valid
    p0 = plan_of(l5_doc())
    assert "rotor_speed_rad_s" not in run_l5.l2_scenario_doc(l5_doc(), p0, "x", "src")["initial_state"]


# ---- the hover helper -----------------------------------------------------------------------------------------------

def wrench_of(speeds):
    """[thrust, roll, pitch, yaw] of the plant's forward map at the given rotor speeds, from the card by literal path."""
    card = schema.load_yaml(CARD)
    k = float(card["rotors"]["thrust_coeff"]["value"])
    c_q = float(card["rotors"]["torque_ratio"]["value"])
    w = [0.0] * 4
    for i, m in enumerate(gpc.MOTORS):
        x, y = (float(v) for v in card["rotors"][m]["position"]["value"][:2])
        s = gpc.SPIN_SIGN[card["rotors"][m]["spin"]["value"]]
        t = k * speeds[i] ** 2
        w = [w[0] + t, w[1] - y * t, w[2] + x * t, w[3] + s * c_q * t]
    return w


def hover_speeds():
    vals = l5s.values(l5s.load(L5 / "step_roll.yaml"))
    return run_l5.hover_rotor_speeds(str(CARD), vals), run_l4.hover_thrust(str(CARD), vals)


def test_hover_speeds_give_the_hover_thrust_and_zero_torque():
    speeds, thrust = hover_speeds()
    w = wrench_of(speeds)
    scale = thrust
    assert w[0] == pytest.approx(thrust, rel=1e-12)
    assert all(abs(x) <= 1e-12 * scale for x in w[1:]), w


def test_hover_speeds_control_a_perturbed_speed_breaks_the_zero_torque_check():
    speeds, thrust = hover_speeds()
    bent = [speeds[0] * 1.01, *speeds[1:]]
    w = wrench_of(bent)
    assert not all(abs(x) <= 1e-12 * thrust for x in w[1:])
    assert w[0] != pytest.approx(thrust, rel=1e-12)


def test_hover_speeds_are_in_the_cards_speed_range_and_out_of_range_is_refused():
    speeds, _ = hover_speeds()
    lo, hi = schema.load_yaml(CARD)["rotors"]["speed_range"]["value"]
    assert all(lo <= w <= hi for w in speeds)
    vals = l5s.values(l5s.load(L5 / "step_roll.yaml"))
    vals["site_height_m"] = 1.0e12  # gravity at this height is ~0: hover thrust below the idle thrust
    with pytest.raises(run_l5.PlanError, match="outside the card's speed range"):
        run_l5.hover_rotor_speeds(str(CARD), vals)
