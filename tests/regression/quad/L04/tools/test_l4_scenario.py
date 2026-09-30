"""The L4 T4 scenario schema (tools/sim/l4_scenario.py), the committed step scenarios (scenarios/quad/L04/*.yaml) and the
L4 runner (tools/sim/run_l4.py) without gz. L04 tests, host-only.

Every refusal is a negative control: the same mutation of a valid scenario (or plan input, or world) must be refused,
while the unmutated one is accepted. The runner is exercised end to end with a fake build directory (a CMakeCache.txt
naming the parameter set and a parameter table in the generator's format) and a fake `run_gz_process` that writes a
synthetic log (the structs of tools/sim/lockstep_log.py) to the world's <log_path>. Test values are labelled scenario
test values: SHORT_SETTLE_S = 1e-3 s and SHORT_HORIZON = 0.01 keep the fake run to a few ticks; TAU_EXACT = 0.3125 s
makes horizon tau_ref / T an exact integer (10000 at 3.2 kHz) so the ceiling rule is seen at the boundary; FAKE_GYRO =
0.5 rad/s per tick index is the synthetic gyro sample; TAU_CL = 0.1609375 s is decision 0005's tau_cl, the build's
tau_ref as float32; SETTLES and PLAN_CASES put settle_s and horizon_tau_ref on both sides of the phase and ceiling
boundaries; STAMP_CHECKS executions of the stamp phase are compared. Mutation values in the refusal tests are chosen
to violate one rule each.
"""

import copy
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from fractions import Fraction
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import gen_world  # noqa: E402
import hover  # noqa: E402
import l4_scenario as l4s  # noqa: E402
import lockstep_log as ll  # noqa: E402
import run_l4  # noqa: E402
import run_scenario as rs  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN_DIR = ROOT / "scenarios" / "quad" / "L04"
REGISTER = schema.load_yaml(ROOT / "design" / "scenario_values.yaml")
AXES = ("roll", "pitch", "yaw")
SHORT_SETTLE_S = 1e-3
SHORT_HORIZON = 0.01
TAU_EXACT = 0.3125
FAKE_GYRO = 0.5
TAU_CL = 0.1609375
SETTLES = (1e-3, 0.5, 1.0, 1.0001, 2.0)
PLAN_CASES = ((1.0, 10), (1e-3, 0.01), (0.25, 3))
STAMP_CHECKS = 64
U64 = 2.0 ** -53  # binary64 unit roundoff
# 4 k omega_h^2 against m g: m g (1 rounding), / (4 k) (1; 4 k is exact), sqrt (1, plus half the 2 u of its input),
# the square (twice omega's 2 u, plus 1), the product with 4 k (1), and m g once more on the other side (1): 8 u.
THRUST_ROUNDINGS = 8
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID


def raw(axis="roll"):
    return schema.load_yaml(SCEN_DIR / f"step_{axis}.yaml")


def findings(doc, axis="roll", register=None):
    return l4s.validate(doc, f"step_{axis}.yaml", REGISTER if register is None else register)


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def build_params(**over):
    """A parameter dict as the build's table gives it (the register's tick, divisor and rates as float32)."""
    p = {k: REGISTER[k]["value"] for k in ("tick_period_num_us", "tick_period_den", "rate_loop_divisor")}
    for a in AXES:
        p[f"rate_max_{a}"] = r32(REGISTER[f"rate_max_{a}"]["value"])
        p[f"rate_tau_ref_{a}"] = r32(TAU_CL)
    p.update(over)
    return p


def stamp(p, tick):
    return (tick * p.num_us) // p.den


# ---- the committed scenarios -----------------------------------------------------------------------------------------

@pytest.mark.parametrize("axis", AXES)
def test_committed_step_scenarios_are_valid(axis):
    doc = l4s.load(SCEN_DIR / f"step_{axis}.yaml")
    v = l4s.values(doc)
    assert v["scenario"] == f"step_{axis}" and v["step"]["axis"] == axis
    assert v["step"]["rate_rad_s"] == REGISTER[f"rate_max_{axis}"]["value"]
    assert (v["tick_period_num_us"], v["tick_period_den"]) == (REGISTER["tick_period_num_us"]["value"],
                                                              REGISTER["tick_period_den"]["value"])
    assert v["m_sequence"] == [2, 1]
    assert v["initial_state"]["body_rates_frd_rad_s"] == [0.0, 0.0, 0.0]
    assert any(v["initial_state"]["velocity_ned_m_s"][:2]), "no horizontal velocity: reads could be stale"
    for key in ("tick_period_num_us", "tick_period_den", "m_sequence"):
        assert doc[key]["label"] == "derived"
    assert doc["step"]["rate_rad_s"]["label"] == "derived" and doc["step"]["horizon_tau_ref"]["label"] == "derived"


def test_valid_base_passes():
    assert findings(raw()) == []


@pytest.mark.parametrize("field", l4s.TOP_FIELDS)
def test_missing_field_refused(field):
    doc = raw()
    del doc[field]
    assert any(field in f for f in findings(doc))


@pytest.mark.parametrize("field", l4s.STEP_FIELDS)
def test_missing_step_field_refused(field):
    doc = raw()
    del doc["step"][field]
    assert any(f"step.{field}" in f for f in findings(doc))


def test_unknown_fields_and_stem_mismatch_refused():
    doc = raw()
    doc["duration_ticks"] = {"value": 4, "unit": "1", "label": "scenario", "rationale": "x"}
    assert any("duration_ticks: unknown field" in f for f in findings(doc))
    doc = raw()
    doc["step"]["amplitude"] = {"value": 1.0, "unit": "1", "label": "scenario", "rationale": "x"}
    assert any("step.amplitude: unknown field" in f for f in findings(doc))
    assert any("file stem" in f for f in l4s.validate(raw(), "step_pitch.yaml", REGISTER))


def test_unlabelled_number_refused():
    doc = raw()
    doc["step"]["settle_s"] = 1.0
    assert any("unlabelled number" in f for f in findings(doc))


@pytest.mark.parametrize("mutate, text", [
    (lambda d: d["site_height_m"].pop("rationale"), "needs a non-empty rationale"),
    (lambda d: d["m_sequence"].pop("rule"), "needs a non-empty rule"),
    (lambda d: d["seed"].update(label="guess"), "is not one of"),
    (lambda d: d["step"]["settle_s"].update(rationale="two\nlines"), "must be one line"),
])
def test_label_and_rationale_rules(mutate, text):
    doc = raw()
    mutate(doc)
    assert any(text in f for f in findings(doc))


def test_tick_must_be_the_registers_and_derived():
    doc = raw()
    doc["tick_period_num_us"]["value"] = 626
    assert any("differs from the scenario-values register" in f for f in findings(doc))
    doc = raw()
    doc["tick_period_den"].update(label="scenario", rationale="x")
    doc["tick_period_den"].pop("rule")
    assert any("label must be derived" in f for f in findings(doc))


@pytest.mark.parametrize("axis", AXES)
def test_rate_must_be_the_registers_rate_max_and_derived(axis):
    doc = raw(axis)
    doc["step"]["rate_rad_s"]["value"] = math.nextafter(doc["step"]["rate_rad_s"]["value"], math.inf)
    assert any(f"rate_max_{axis}" in f for f in findings(doc, axis))
    doc = raw(axis)
    doc["step"]["rate_rad_s"].update(label="scenario", rationale="x")
    doc["step"]["rate_rad_s"].pop("rule")
    assert any("label must be derived" in f for f in findings(doc, axis))
    reg = copy.deepcopy(REGISTER)
    del reg[f"rate_max_{axis}"]
    assert any(f"no entry 'rate_max_{axis}'" in f for f in findings(raw(axis), axis, reg))


@pytest.mark.parametrize("bad", [[4, 2, 1], [1, 2], [2], [2, 2]])
def test_m_sequence_must_be_two_then_one(bad):
    doc = raw()
    doc["m_sequence"]["value"] = bad
    assert any("m_sequence" in f for f in findings(doc))


@pytest.mark.parametrize("path, value, text", [
    (("initial_state", "body_rates_frd_rad_s"), [0.0, 0.0, 1e-9], "must be zero"),
    (("initial_state", "attitude_q_wxyz"), [-1.0, 0.0, 0.0, 0.0], "canonical sign"),
    (("initial_state", "attitude_q_wxyz"), [1.0, 1e-3, 0.0, 0.0], "exceeds"),
    (("initial_state", "velocity_ned_m_s"), [1.0, 0.0], "list of 3"),
    (("step", "axis"), "yawn", "must be one of"),
    (("step", "settle_s"), 0.0, "> 0"),
    (("step", "settle_s"), math.inf, "> 0"),
    (("step", "horizon_tau_ref"), -10, "> 0"),
    (("site_latitude_rad"), 2.0, "pi/2"),
    (("seed"), -1, ">= 0"),
])
def test_value_rules(path, value, text):
    doc = raw()
    node = doc
    for key in (path if isinstance(path, tuple) else (path,)):
        node = node[key]
    node["value"] = value
    assert any(text in f for f in findings(doc)), findings(doc)


def test_load_refuses_duplicate_keys_and_a_missing_file(tmp_path):
    p = tmp_path / "step_roll.yaml"
    p.write_text((SCEN_DIR / "step_roll.yaml").read_text() + "seed:\n  value: 2\n", encoding="utf-8")
    with pytest.raises(scn.ScenarioError):
        l4s.load(p)
    with pytest.raises(scn.ScenarioError):
        l4s.load(tmp_path / "absent.yaml")


# ---- the plan ------------------------------------------------------------------------------------------------------

def plan(axis="roll", doc=None, **over):
    return run_l4.plan(doc or l4s.load(SCEN_DIR / f"step_{axis}.yaml"), build_params(**over), CARD)


@pytest.mark.parametrize("axis", AXES)
def test_plan_of_the_committed_scenarios(axis):
    """The planning rules (module docstring of tools/sim/run_l4.py), recomputed here from the committed scenario and
    register as read now, by a search and exact rationals rather than the planner's own steps."""
    p = plan(axis)
    doc = schema.load_yaml(SCEN_DIR / f"step_{axis}.yaml")
    num, den = REGISTER["tick_period_num_us"]["value"], REGISTER["tick_period_den"]["value"]
    d = REGISTER["rate_loop_divisor"]["value"]
    settle, horizon = Fraction(doc["step"]["settle_s"]["value"]), Fraction(doc["step"]["horizon_tau_ref"]["value"])
    t = Fraction(num, den) / 10 ** 6  # s
    assert p.axis == axis and p.axis_index == AXES.index(axis)
    assert p.divisor == d and p.tick_s == t and p.period_s == d * t

    k0 = 2  # the smallest k >= 2 with D k t >= settle_s and D (k - 1) num = 0 (mod den)
    while not (d * k0 * t >= settle and (d * (k0 - 1) * num) % den == 0):
        k0 += 1
    assert p.step_execution == k0
    assert p.seg_t_us == (d * k0 * num) // den  # the stamp rule: floor(D k0 num / den) us
    n = 1 + math.ceil(horizon * Fraction(r32(TAU_CL)) / (d * t))
    assert p.n_exec == n
    assert p.duration_ticks > d * (k0 + n - 2) and p.duration_ticks % math.lcm(*p.m_sequence) == 0
    assert p.duration_ticks - math.lcm(*p.m_sequence) <= d * (k0 + n - 2)
    assert p.rate_rad_s == r32(REGISTER[f"rate_max_{axis}"]["value"])


@pytest.mark.parametrize("settle", SETTLES)
def test_step_execution_rule_and_stamp_phase(settle):
    doc = raw()
    doc["step"]["settle_s"]["value"] = settle
    p = plan(doc=doc)

    def ok(k):
        return k >= 2 and p.divisor * k * p.tick_s >= Fraction(settle) and (p.divisor * (k - 1) * p.num_us) % p.den == 0

    assert ok(p.step_execution) and not any(ok(k) for k in range(2, p.step_execution))
    k0 = p.step_execution
    assert stamp(p, p.tick_of(k0 - 1)) < p.seg_t_us == stamp(p, p.tick_of(k0))
    for n in range(1, STAMP_CHECKS):  # the oracle's stamps floor(D (n - 1) num / den) plus one integer
        assert (stamp(p, p.tick_of(p.execution_of(n))) - stamp(p, p.tick_of(p.execution_of(1)))
                == (p.divisor * (n - 1) * p.num_us) // p.den)


def test_stamp_phase_control_an_even_step_execution_shifts_every_dt():
    p = plan()
    k0 = p.step_execution + 1  # the neighbour: the phase condition fails for it
    assert (p.divisor * (k0 - 1) * p.num_us) % p.den != 0
    diffs = [stamp(p, p.tick_of(k0 + n - 2)) - stamp(p, p.tick_of(k0 + n - 3))
             - (p.divisor * (n - 1) * p.num_us) // p.den + (p.divisor * (n - 2) * p.num_us) // p.den
             for n in range(2, STAMP_CHECKS)]
    assert all(d != 0 for d in diffs)


def test_window_count_is_exact_at_an_integer_quotient():
    doc = raw()
    p = plan(doc=doc, rate_tau_ref_roll=TAU_EXACT)
    assert Fraction(10) * Fraction(TAU_EXACT) / p.period_s == 10000
    assert p.n_exec == 10001


def test_duration_covers_the_last_window_tick_and_is_minimal():
    for settle, horizon in PLAN_CASES:
        doc = raw()
        doc["step"]["settle_s"]["value"] = settle
        doc["step"]["horizon_tau_ref"]["value"] = horizon
        p = plan(doc=doc)
        last = p.tick_of(p.execution_of(p.n_exec))
        lcm = math.lcm(*p.m_sequence)
        assert p.duration_ticks % lcm == 0 and p.duration_ticks > last and p.duration_ticks - lcm <= last


def test_hover_thrust_is_m_g_as_the_l2_hover_command():
    p = plan()
    v = l4s.values(raw())
    lat, h0 = v["site_latitude_rad"], v["site_height_m"]
    card, profile = gpc.load_linted(CARD)
    cfg, _ = gpc.config_from(card, profile, CARD)
    omega_h, _, _, _ = hover.bracket_for_card(cfg, lat, h0)
    assert p.thrust_n == r32(p.thrust_n_double)
    assert p.thrust_n_double == cfg["mass_kg"] * hover.normal_gravity(lat, h0)
    assert math.isclose(4.0 * cfg["thrust_coeff"] * omega_h ** 2, p.thrust_n_double, rel_tol=THRUST_ROUNDINGS * U64)


@pytest.mark.parametrize("over, text", [
    ({"tick_period_num_us": 626}, "would panic"),
    ({"rate_loop_divisor": 3}, "does not divide"),
    ({"rate_max_roll": r32(1.0)}, "rate_max_roll"),
])
def test_plan_refusals(over, text):
    with pytest.raises(run_l4.PlanError) as e:
        plan(**over)
    assert any(text in line for line in e.value.lines)


def test_plan_refuses_a_table_without_a_needed_parameter():
    params = build_params()
    del params["rate_tau_ref_roll"]
    with pytest.raises(run_l4.PlanError, match="lacks"):
        run_l4.plan(l4s.load(SCEN_DIR / "step_roll.yaml"), params, CARD)


def test_plan_overrides_are_exact_for_their_types():
    p = plan("pitch")
    ov = p.overrides()
    assert list(ov) == ["l4_thrust_n", "l4_seg_count", "l4_seg1_t_us", "l4_seg1_roll", "l4_seg1_pitch", "l4_seg1_yaw"]
    assert ov["l4_seg_count"] == ("i32", "1") and ov["l4_seg1_t_us"] == ("i32", str(p.seg_t_us))
    assert ov["l4_seg1_roll"] == ov["l4_seg1_yaw"] == ("f32", "0.0")
    for name in ("l4_thrust_n", "l4_seg1_pitch"):
        t, text = ov[name]
        assert t == "f32" and r32(float(text)) == float(text)
    assert float(ov["l4_seg1_pitch"][1]) == p.rate_rad_s and float(ov["l4_thrust_n"][1]) == p.thrust_n


# ---- the build under test ------------------------------------------------------------------------------------------

TABLE_ENTRY = """    // {name}
    {{{{ParamType::{kind}, {f32}, {i32}}}}}, 0.0f, ParamOrigin::DefaultFromCard, ParamMethod::Scenario, false,
     SigmaKind::Choice,
     "1", "test"}},
"""


def table_text(params):
    out = []
    for name, v in params.items():
        if isinstance(v, int):
            out.append(TABLE_ENTRY.format(name=name, kind="I32", f32="0.0f", i32=v))
        else:
            out.append(TABLE_ENTRY.format(name=name, kind="F32", f32=f"{v!r}f", i32=0))
    return "".join(out)


def fake_build(tmp_path, params, set_name=run_l4.COMPOSITION_PARAMS):
    build = tmp_path / "build"
    plugin = build / "sim" / "gz" / "plugin"
    plugin.mkdir(parents=True)
    (build / "CMakeCache.txt").write_text(f"//Parameter set\nMARV_GZ_PARAMS:STRING={set_name}\n", encoding="utf-8")
    table = build / "generated" / set_name / "marv" / "params" / "param_defaults.cpp"
    table.parent.mkdir(parents=True)
    table.write_text(table_text(params), encoding="utf-8")
    return plugin, table


def test_build_parameters_and_table(tmp_path):
    params = build_params()
    plugin, table = fake_build(tmp_path, params)
    name, path = run_l4.build_parameters(plugin)
    assert name == run_l4.COMPOSITION_PARAMS and path == table
    assert run_l4.read_param_defaults(path) == params


def test_build_parameters_refuses_another_composition_and_no_build(tmp_path):
    plugin, _ = fake_build(tmp_path, build_params(), "marv_params_l2_open_loop")
    with pytest.raises(run_l4.PlanError, match="not the host-gz-l4 build"):
        run_l4.build_parameters(plugin)
    lone = tmp_path / "lone"
    lone.mkdir()
    with pytest.raises(run_l4.PlanError):
        run_l4.build_parameters(lone)
    (tmp_path / "empty.cpp").write_text("// nothing\n", encoding="utf-8")
    with pytest.raises(run_l4.PlanError, match="no parameter entries"):
        run_l4.read_param_defaults(tmp_path / "empty.cpp")


# ---- the world -----------------------------------------------------------------------------------------------------

def generated_world(tmp_path, p, doc, log_path="run.bin"):
    stem = f"{doc['scenario']}_{run_l4.NAME_LABEL}"
    l2 = run_l4.l2_scenario_doc(doc, p, stem, "step_roll.yaml")
    path = tmp_path / f"{stem}.yaml"
    path.write_text(yaml.safe_dump(l2, sort_keys=False), encoding="utf-8")
    return l2, path, gen_world.generate(CARD, path, "test", 1, None, str(tmp_path / log_path))[1]


def test_l2_input_is_a_valid_l2_scenario_with_the_l4_state(tmp_path):
    doc = l4s.load(SCEN_DIR / "step_roll.yaml")
    p = plan(doc=doc)
    l2, path, _ = generated_world(tmp_path, p, doc)
    assert scn.validate(l2, path.name, None) == []
    v = scn.values(scn.load(path, CARD))
    assert v["initial_state"] == l4s.values(doc)["initial_state"]
    assert v["duration_ticks"] == p.duration_ticks and v["m_sequence"] == [2, 1]


def test_world_edit_swaps_the_l2_overrides_for_the_plan_and_truth_gyro(tmp_path):
    doc = l4s.load(SCEN_DIR / "step_roll.yaml")
    p = plan(doc=doc)
    _, _, text = generated_world(tmp_path, p, doc)
    assert len(re.findall(r'param="ol_dshot_m\d"', text)) == 4
    harness = {"rate_kp_roll": ("f32", "0.0")}
    out = run_l4.world_edit({**p.overrides(), **harness}, lambda t: t + "<!-- extra -->")(text)
    assert "ol_dshot" not in out and out.count(run_l4.GYRO_ELEMENT) == 1 and out.endswith("<!-- extra -->")
    plugin = ET.fromstring(out.replace("<!-- extra -->", "").split("\n", 1)[1]).find(".//plugin[@filename='marv_gz_lockstep']")
    got = {e.get("param"): (e.get("type"), e.text) for e in plugin.findall("sil_override")}
    assert got == {**p.overrides(), **harness}
    assert plugin.find("gyro_source").text == "truth"


def test_world_edit_refuses_a_world_without_the_four_l2_overrides_or_the_plugin(tmp_path):
    doc = l4s.load(SCEN_DIR / "step_roll.yaml")
    p = plan(doc=doc)
    _, _, text = generated_world(tmp_path, p, doc)
    three = re.sub(r'\s*<sil_override param="ol_dshot_m4"[^<]*</sil_override>', "", text)
    with pytest.raises(rs.RunError, match="3 L2 ol_dshot_m"):
        run_l4.world_edit(p.overrides())(three)
    no_plugin = text.replace('filename="marv_gz_lockstep"', 'filename="other"')
    with pytest.raises(rs.RunError, match="0 marv_gz_lockstep"):
        run_l4.world_edit(p.overrides())(no_plugin)


# ---- run_step with a fake build and a fake gz ----------------------------------------------------------------------

def short_scenario(tmp_path):
    doc = raw()
    doc["step"]["settle_s"]["value"] = SHORT_SETTLE_S
    doc["step"]["horizon_tau_ref"]["value"] = SHORT_HORIZON
    path = tmp_path / "scen" / "step_roll.yaml"
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return path


def imu_bytes(tick):
    return struct.pack("<7fI", FAKE_GYRO * tick, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 << GYRO_VALID_BIT)


def fake_gz(monkeypatch, stale_steps=()):
    calls = []

    def fake(sdf_path, iterations, seed, plugin_dir=None, extra_env=None, timeout_s=None):
        text = Path(sdf_path).read_text(encoding="utf-8")
        log_path = re.search(r"<log_path>([^<]*)</log_path>", text).group(1)
        m = int(re.search(r"<ticks_per_step>(\d+)</ticks_per_step>", text).group(1))
        version = b"8.99.0"
        head = struct.Struct("<8sIIIIIQI")
        out = bytearray(head.pack(ll.MAGIC, ll.VERSION, head.size + len(version), m, 625, 4, seed, len(version)) + version)
        x = 0.0
        for i in range(iterations):
            if i not in stale_steps:
                x = float(i)
            read = (x, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            out += bytes([ll.STEP]) + ll._STEP.pack(i, (i + 1) * 156250 * m, *read, *([0.0] * 13))
            for j in range(i * m, (i + 1) * m):
                out += bytes([ll.TICK]) + ll._TICK.pack(j, j * 625 // 4, imu_bytes(j), 765, 765, 765, 765, 1, 0,
                                                        *([0.0] * 20))
            out += bytes([ll.APPLIED]) + ll._APPLIED.pack(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        out += bytes([ll.TRAILER]) + ll._TRAILER.pack(iterations, iterations * m, iterations)
        Path(log_path).write_bytes(bytes(out))
        calls.append({"sdf": text, "m": m, "iterations": iterations})
        return rs.GzProcess(0, "", "", 0.5, 0.25)

    monkeypatch.setattr(rs, "run_gz_process", fake)
    monkeypatch.setattr(rs, "_dpkg_query", lambda *a: None)
    return calls


def test_run_step_end_to_end_names_report_and_executions(tmp_path, monkeypatch):
    calls = fake_gz(monkeypatch)
    plugin, _ = fake_build(tmp_path, build_params())
    scen = short_scenario(tmp_path)
    s = run_l4.run_step(CARD, scen, 1, tmp_path / "out", plugin)
    p = s.plan
    assert p.step_execution == 5 and p.n_exec == 7 and p.duration_ticks == 22
    assert calls[0]["iterations"] == 22 and calls[0]["m"] == 1
    for path in (s.run.log_path, s.run.world_path, s.run.report_path):
        assert "_truth_fed_perfect_model_" in Path(path).name
    report = Path(s.run.report_path).read_text(encoding="utf-8")
    assert report.startswith(f"MARV L4 run report: {run_l4.LABEL}\n") and run_l4.LABEL_LINE in report
    assert "card: vehicles/uzh_neurobem_5in.yaml" in report and "t4 steps: (not evaluated)" in report and "MARV L2 run report" not in report
    assert "ol_dshot" not in calls[0]["sdf"] and calls[0]["sdf"].count(run_l4.GYRO_ELEMENT) == 1
    assert f'<sil_override param="l4_seg1_t_us" type="i32">{p.seg_t_us}</sil_override>' in calls[0]["sdf"]
    assert [x["k"] for x in s.executions] == list(range(11))
    for x in s.executions:
        assert x["tick"] == 2 * x["k"] and x["t_us"] == (2 * x["k"] * 625) // 4
        assert x["gyro"] == (r32(FAKE_GYRO * x["tick"]), 0.0, 0.0) and x["flags"] == 1 << GYRO_VALID_BIT
    assert list(p.window) == list(range(4, 11)) and s.window_stale == [] and s.harness_overrides == {}


def test_run_step_reports_a_stale_read_in_the_window_and_ignores_one_before(tmp_path, monkeypatch):
    plugin, _ = fake_build(tmp_path, build_params())
    scen = short_scenario(tmp_path)
    fake_gz(monkeypatch, stale_steps=(3, 12))  # m = 1: step 3 is before the window (ticks 8..20), step 12 inside
    s = run_l4.run_step(CARD, scen, 1, tmp_path / "out", plugin)
    assert s.window_stale == [12] and s.run.stale["stale"] == 2
    assert [x["k"] for x in s.executions if not x["fresh"]] == [6]


def test_run_step_refusals(tmp_path, monkeypatch):
    fake_gz(monkeypatch)
    plugin, _ = fake_build(tmp_path, build_params())
    scen = short_scenario(tmp_path)
    with pytest.raises(run_l4.PlanError, match="not in the scenario's m_sequence"):
        run_l4.run_step(CARD, scen, 4, tmp_path / "out", plugin)
    with pytest.raises(run_l4.PlanError, match="plan parameters already"):
        run_l4.run_step(CARD, scen, 1, tmp_path / "out", plugin, overrides={"l4_seg_count": ("i32", "0")})
    other, _ = fake_build(tmp_path / "b2", build_params(tick_period_num_us=626))
    with pytest.raises(run_l4.PlanError, match="would panic"):
        run_l4.run_step(CARD, scen, 1, tmp_path / "out", other)


def test_run_sequence_runs_two_then_one_and_writes_one_report(tmp_path, monkeypatch):
    calls = fake_gz(monkeypatch)
    plugin, _ = fake_build(tmp_path, build_params())
    runs, path = run_l4.run_sequence(CARD, short_scenario(tmp_path), tmp_path / "out", plugin,
                                     overrides={"rate_kp_roll": ("f32", "0.0")})
    assert [c["m"] for c in calls] == [2, 1] and [s.run.m for s in runs] == [2, 1]
    assert all(s.harness_overrides == {"rate_kp_roll": ("f32", "0.0")} for s in runs)
    text = run_l4.write_report(runs, {"passed": True}, path)
    assert Path(path).name.endswith("_truth_fed_perfect_model_test_seed1_sequence.report.txt")
    assert "m sequence run: [2, 1]" in text and "t4 steps:\n    passed: True" in text
    assert "rate_kp_roll: f32 0.0" in text
