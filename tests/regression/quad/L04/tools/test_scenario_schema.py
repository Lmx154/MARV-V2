"""The refusal cases shared by the three scenario schemas (tools/sim/scenario.py, l4_scenario.py, l5_scenario.py), one
file; it consolidates the duplicates of L02/tools/test_scenario.py, test_l4_scenario.py and test_l5_scenario.py.
Each test is one case family on one schema and loops over that schema's cases; a failure names every case that did
not refuse (or was refused with another message).
"""

import copy
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l4_scenario as l4s  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import scenario  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
INERTIA = [float(x) for x in schema.load_yaml(CARD)["inertia_diag"]["value"]]
L4_REGISTER = schema.load_yaml(l4s.REGISTER)
L5_REGISTER = schema.load_yaml(l5s.REGISTER)

SCHEMAS = {
    "L02": SimpleNamespace(
        dir=ROOT / "scenarios" / "quad" / "L02", names=("free_fall", "rotation", "hover", "determinism"),
        top_fields=scenario.TOP_FIELDS,
        validate=lambda doc, stem: scenario.validate(doc, f"{stem}.yaml", INERTIA),
        load=lambda path: scenario.load(path, CARD)),
    "L04": SimpleNamespace(
        dir=ROOT / "scenarios" / "quad" / "L04", names=("step_roll",), top_fields=l4s.TOP_FIELDS,
        validate=lambda doc, stem: l4s.validate(doc, f"{stem}.yaml", L4_REGISTER), load=l4s.load),
    "L05": SimpleNamespace(
        dir=ROOT / "scenarios" / "quad" / "L05", names=("step_roll",), top_fields=l5s.TOP_FIELDS,
        validate=lambda doc, stem: l5s.validate(doc, f"{stem}.yaml", L5_REGISTER), load=l5s.load),
}
ALL = tuple(SCHEMAS)


def raw(sid, name=None):
    s = SCHEMAS[sid]
    return schema.load_yaml(s.dir / f"{name or s.names[0]}.yaml")


def mutate(sid, fn):
    """The findings for the schema's base document after `fn` has changed it."""
    doc = raw(sid)
    fn(doc)
    return SCHEMAS[sid].validate(doc, SCHEMAS[sid].names[0])


def entry(value=1, unit="1"):
    return {"value": value, "unit": unit, "label": "scenario", "rationale": "x"}


def refused(out, *texts):
    return any(all(t in f for t in texts) for f in out)


def run(sid, cases):
    """cases: (case id, mutation, expected texts); expected texts None means the mutated document is accepted.
    Returns the case ids that did not end as expected."""
    bad = []
    for cid, mutation, texts in cases:
        out = mutate(sid, mutation)
        if (out != []) if texts is None else not refused(out, *texts):
            bad.append((cid, out))
    return bad


def setter(path, value):
    def fn(d):
        node = d
        for k in path[:-1]:
            node = node[k]
        node[path[-1]] = copy.deepcopy(value)
    return fn


@pytest.mark.parametrize("sid", ALL)
def test_valid_base_is_accepted(sid):
    bad = [n for n in SCHEMAS[sid].names if SCHEMAS[sid].validate(raw(sid, n), n) != []]
    assert not bad, bad


MISSING_TOP = {"L02": SCHEMAS["L02"].top_fields, "L04": SCHEMAS["L04"].top_fields, "L05": ("script",)}


@pytest.mark.parametrize("sid", ALL)
def test_missing_top_field_refused(sid):
    bad = run(sid, [(f, lambda d, f=f: d.pop(f), (f"{f}: missing field",)) for f in MISSING_TOP[sid]])
    assert not bad, bad


def test_missing_state_field_refused_l02():
    bad = run("L02", [(f, lambda d, f=f: d["initial_state"].pop(f), (f"initial_state.{f}: missing field",))
                      for f in scenario.STATE_FIELDS])
    assert not bad, bad


UNKNOWN_TOP = {"L02": ("extra", 1), "L04": ("duration_ticks", entry(4)), "L05": ("step", {})}


@pytest.mark.parametrize("sid", ALL)
def test_unknown_top_field_refused(sid):
    key, value = UNKNOWN_TOP[sid]
    bad = run(sid, [(key, lambda d: d.update({key: copy.deepcopy(value)}), (f"{key}: unknown field",))])
    assert not bad, bad


STEM = "must equal the file stem"


@pytest.mark.parametrize("sid, other", [("L02", "rotation"), ("L04", "step_pitch")])
def test_file_stem_mismatch_by_name_refused(sid, other):
    assert refused(SCHEMAS[sid].validate(raw(sid), other), STEM)


def test_file_stem_mismatch_by_document_field_refused_l05():
    assert refused(mutate("L05", setter(["scenario"], "other")), STEM)


def test_unlabelled_number_refused_l02():
    bad = run("L02", [
        ("top", lambda d: d.update(site_height_m=500.0), ("unlabelled number", "site_height_m")),
        ("state", lambda d: d["initial_state"].update(velocity_ned_m_s=[0.0, 0.0, 0.0]), ("unlabelled number",)),
        ("entry", lambda d: d["site_height_m"].update(extra_note=3), ("unlabelled number",)),
    ])
    assert not bad, bad


ENTRY_RULES = {
    "label missing": (lambda d: d["site_height_m"].pop("label"), ("missing field 'label'",)),
    "label unknown": (lambda d: d["site_height_m"].update(label="guess"), ("is not one of",)),
    "rationale missing": (lambda d: d["site_height_m"].pop("rationale"), ("needs a non-empty rationale",)),
    "rationale two lines": (lambda d: d["site_height_m"].update(rationale="two\nlines"), ("must be one line",)),
    "unit missing": (lambda d: d["site_height_m"].pop("unit"), ("missing field 'unit'",)),
    "rule missing": (lambda d: d["m_sequence"].pop("rule"), ("needs a non-empty rule",)),
    "entry field unknown": (lambda d: d["m_sequence"].update(bogus="x"), ("unknown entry field",)),
}
ENTRY_RULES_OF = {"L02": tuple(ENTRY_RULES), "L04": ("rationale missing", "rule missing"), "L05": ("label missing",)}


@pytest.mark.parametrize("sid", ALL)
def test_label_and_rationale_rules(sid):
    bad = run(sid, [(c, *ENTRY_RULES[c]) for c in ENTRY_RULES_OF[sid]])
    assert not bad, bad


SCALAR_RULES = [
    ("site_latitude_rad", 2.0, "pi/2"),
    ("seed", -1, ">= 0"),
    ("seed", 1.0, ">= 0"),
    ("tick_period_den", 0, "must be an integer > 0"),
    ("tick_period_num_us", 625.0, "must be an integer > 0"),
]
SCALAR_RULES_OF = {"L02": SCALAR_RULES, "L04": SCALAR_RULES[:2]}


@pytest.mark.parametrize("sid", ("L02", "L04"))
def test_scalar_value_rules(sid):
    bad = run(sid, [((f, v), setter([f, "value"], v), (f"{f}: value", t)) for f, v, t in SCALAR_RULES_OF[sid]])
    assert not bad, bad


QUATERNIONS = {
    "w < 0": ([-1.0, 0.0, 0.0, 0.0], ("w = ", "canonical sign")),
    "norm 0.1": ([1.0, 0.1, 0.0, 0.0], ("|q|^2", "exceeds")),
    "norm 1e-3": ([1.0, 1e-3, 0.0, 0.0], ("|q|^2", "exceeds")),
    "length 3": ([1.0, 0.0, 0.0], ("attitude_q_wxyz", "list of 4 finite numbers")),
    "w = 0 accepted": ([0.0, 1.0, 0.0, 0.0], None),
}
QUATERNIONS_OF = {"L02": tuple(QUATERNIONS), "L04": ("w < 0", "norm 1e-3"), "L05": ("w < 0", "norm 0.1")}


@pytest.mark.parametrize("sid", ALL)
def test_attitude_quaternion_rules(sid):
    bad = run(sid, [(c, setter(["initial_state", "attitude_q_wxyz", "value"], QUATERNIONS[c][0]), QUATERNIONS[c][1])
                    for c in QUATERNIONS_OF[sid]])
    assert not bad, bad


@pytest.mark.parametrize("sid", ("L04", "L05"))
def test_tick_differing_from_the_register_refused(sid):
    bad = run(sid, [(626, setter(["tick_period_num_us", "value"], 626), ("differs from the scenario-values register",))])
    assert not bad, bad


M_SEQUENCES_OF = {"L04": ([4, 2, 1], [1, 2], [2], [2, 2]), "L05": ([4, 2],)}


@pytest.mark.parametrize("sid", ("L04", "L05"))
def test_m_sequence_must_be_two_then_one(sid):
    bad = run(sid, [(str(b), setter(["m_sequence", "value"], b), ("m_sequence", "must be [2, 1]"))
                    for b in M_SEQUENCES_OF[sid]])
    assert not bad, bad


@pytest.mark.parametrize("sid", ("L02", "L04"))
def test_load_refuses_duplicate_keys_and_a_missing_file(sid, tmp_path):
    s = SCHEMAS[sid]
    src = s.dir / f"{s.names[0]}.yaml"
    ok = tmp_path / "ok" / src.name
    ok.parent.mkdir()
    shutil.copy(src, ok)
    assert s.load(ok)["scenario"] == s.names[0]  # control: the unmodified copy loads
    dup = tmp_path / src.name
    dup.write_text(src.read_text(encoding="utf-8") * 2, encoding="utf-8")
    with pytest.raises(scenario.ScenarioError):
        s.load(dup)
    with pytest.raises(scenario.ScenarioError):
        s.load(tmp_path / "absent.yaml")


def test_load_refuses_a_missing_file_l05(tmp_path):
    with pytest.raises(scenario.ScenarioError):
        l5s.load(tmp_path / "absent.yaml")
