"""Unit tests of the L5 scenario schema and the L5 runner's plan and world edit (tools/sim/l5_scenario.py,
tools/sim/run_l5.py; decision 0006 B "Runner" and F), without gz.

Every refusal test has a control: the same document without the defect is accepted (or, for the cross-check against the
recorded T3 envelope, the recorded value is accepted), so a test cannot pass by refusing everything.
"""

import copy
import re
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l5_scenario as l5s  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402

SCEN = ROOT / "scenarios" / "quad" / "L05"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L05/t3")  # the generated golden, envelope and q (decision 0011)
ENVELOPE = T3_GENERATED / "attitude_t3_envelope.txt"
STEPS = ("step_roll", "step_pitch")
BUILD = {"tick_period_num_us": 625, "tick_period_den": 4, "rate_loop_divisor": 2, "att_loop_ratio": 2}
REGISTER = schema.load_yaml(l5s.REGISTER)


def base(name="step_roll"):
    return yaml.safe_load((SCEN / f"{name}.yaml").read_text(encoding="utf-8"))


def findings(doc, name="step_roll.yaml"):
    return l5s.validate(doc, name, REGISTER)


def mutated(fn, name="step_roll"):
    doc = base(name)
    fn(doc)
    return doc


# ---- the committed scenarios ----------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", STEPS)
def test_committed_scenarios_are_valid(name):
    assert findings(base(name), f"{name}.yaml") == []
    v = l5s.values(l5s.load(SCEN / f"{name}.yaml"))
    assert v["thrust"] == "hover" and v["m_sequence"] == [2, 1]


@pytest.mark.parametrize("name,axis", (("step_roll", 0), ("step_pitch", 1)))
def test_step_script_equals_the_recorded_t3_envelope_script(name, axis):
    """The release and the end of the scenario are the recorded envelope's release_execution and last execution."""
    text = ENVELOPE.read_text(encoding="utf-8").splitlines()
    i = text.index(f"scenario {name}")
    release = int(text[i + 2].split()[1])
    count = int(re.match(r"channel theta first 0 count (\d+) ", text[i + 3]).group(1))
    s = l5s.values(l5s.load(SCEN / f"{name}.yaml"))["script"]
    first, second = s["segments"]
    assert (first["start_attitude_execution"], second["start_attitude_execution"]) == (1, release)
    assert s["end_attitude_execution"] == count - 1
    assert first["stick"] == [1.0 if k == axis else 0.0 for k in range(3)] and second["stick"] == [0.0, 0.0, 0.0]


def test_control_a_moved_release_differs_from_the_recorded_envelope():
    # Pinned to literals on purpose (owner, decision 0014 third round item 4): the recorded release 20293 (att_kp
    # 3.15397215) and the moved one stay literals here; the positive assertions read the scenario's derived value.
    doc = mutated(lambda d: d["script"]["segments"][1]["start_attitude_execution"].update(value=20294))
    assert findings(doc) == []  # schema-valid ...
    assert l5s.values(doc)["script"]["segments"][1]["start_attitude_execution"] != 20293  # ... but not the recorded one


YAW = ("yaw_release", "yaw_fallback")
INPUTS = T3_REFERENCE / "attitude_t3_inputs.txt"


def yaw_block(name):
    """The header of the recorded envelope's `name` block: key -> text of its lines before the first channel, and the
    `omega` channel's count."""
    text = ENVELOPE.read_text(encoding="utf-8").splitlines()
    i = text.index(f"scenario {name}")
    head = {}
    for line in text[i + 1:]:
        if line.startswith("channel "):
            break
        k, v = line.split(" ", 1)
        head[k] = v
    count = int(re.match(r"channel omega first 0 count (\d+) ", next(ln for ln in text[i:] if ln.startswith("channel omega "))).group(1))
    return head, count


@pytest.mark.parametrize("name", YAW)
def test_committed_yaw_scenarios_are_valid(name):
    assert findings(base(name), f"{name}.yaml") == []
    v = l5s.values(l5s.load(SCEN / f"{name}.yaml"))
    assert v["thrust"] == "hover" and v["m_sequence"] == [2, 1]


@pytest.mark.parametrize("name", YAW)
def test_yaw_script_equals_the_recorded_t3_envelope_script(name):
    """Release, end, the stick segments, the stick scale and the disturbance are the recorded envelope's values."""
    head, count = yaw_block(name)
    release = int(head["release_execution"])
    s = l5s.values(l5s.load(SCEN / f"{name}.yaml"))["script"]
    first, second = s["segments"]
    scale = float(head.get("stick_scale", "1.0"))
    assert (first["start_attitude_execution"], second["start_attitude_execution"]) == (1, release)
    assert s["end_attitude_execution"] == count - 1 and len(s["segments"]) == 2
    assert first["stick"] == [0.0, 0.0, scale] and second["stick"] == [0.0, 0.0, 0.0]
    if name == "yaw_release":
        assert "disturbance" not in s and "disturbance_nm" not in head
    else:
        assert s["disturbance"] == {"yaw_nm": float(head["disturbance_nm"]), "start_attitude_execution": release}
        tau_held = next(float.fromhex(ln.split()[1]) for ln in INPUTS.read_text().splitlines()
                        if ln.startswith("tau_held_yaw_nm "))
        assert run_l5.l4.r32(s["disturbance"]["yaw_nm"]) == tau_held == float(head["disturbance_nm"])


def test_control_a_moved_yaw_release_or_disturbance_differs_from_the_recorded_envelope():
    head, _ = yaw_block("yaw_fallback")
    doc = mutated(lambda d: (d["script"]["segments"][1]["start_attitude_execution"].update(value=20733),
                             d["script"]["disturbance"]["yaw_nm"].update(value=0.17)), "yaw_fallback")
    assert findings(doc, "yaw_fallback.yaml") == []  # schema-valid ...
    s = l5s.values(doc)["script"]  # ... but not the recorded script
    assert s["segments"][1]["start_attitude_execution"] != int(head["release_execution"])
    assert s["disturbance"]["yaw_nm"] != float(head["disturbance_nm"])


def test_yaw_fallback_plan_switches_the_disturbance_on_at_the_release_stamp():
    p = plan_of(name="yaw_fallback")
    o = p.overrides()
    release = l5s.values(l5s.load(SCEN / "yaw_fallback.yaml"))["script"]["segments"][1]["start_attitude_execution"]
    assert o["l5_dist_t0_us"] == o["l5_seg2_t_us"] == (run_l5.I32, str(p.stamp_us(p.origin + release)))
    assert o["l5_dist_yaw_nm"] == (run_l5.F32, repr(run_l5.l4.r32(0.1634589284658432)))
    assert o["l5_seg1_yaw"] == (run_l5.F32, "1.0") and o["l5_seg1_roll"] == (run_l5.F32, "0.0")
    assert "l5_dist_t0_us" not in plan_of(name="yaw_release").overrides()  # control: no disturbance without the entry


# ---- schema refusals, each against the valid document ---------------------------------------------------------------

def _set(path, value):
    def fn(doc):
        node = doc
        for k in path[:-1]:
            node = node[k]
        node[path[-1]] = value
    return fn


CASES = {
    "unlabelled number": (lambda d: d["script"].update(extra=5), "unknown field"),
    "missing label": (lambda d: d["site_height_m"].pop("label"), "missing field 'label'"),
    "unknown top field": (lambda d: d.update(step={}), "step: unknown field"),
    "missing script": (lambda d: d.pop("script"), "script: missing field"),
    "scenario name is not the file stem": (_set(["scenario"], "other"), "must equal the file stem"),
    "tick differs from the register": (lambda d: d["tick_period_num_us"].update(value=626), "differs from the scenario-values register"),
    "m sequence": (lambda d: d["m_sequence"].update(value=[4, 2]), "must be [2, 1]"),
    "thrust is not hover": (lambda d: d["thrust"].update(value="fixed"), "must be 'hover'"),
    "quaternion w < 0": (lambda d: d["initial_state"]["attitude_q_wxyz"].update(value=[-1.0, 0.0, 0.0, 0.0]), "canonical sign"),
    "quaternion not unit": (lambda d: d["initial_state"]["attitude_q_wxyz"].update(value=[1.0, 0.1, 0.0, 0.0]), "exceeds"),
    "segment starts at 0": (lambda d: d["script"]["segments"][0]["start_attitude_execution"].update(value=0), "must be an integer >= 1"),
    "segments not increasing": (lambda d: d["script"]["segments"][1]["start_attitude_execution"].update(value=1), "not after the previous"),
    "stick above 1": (lambda d: d["script"]["segments"][0]["stick"].update(value=[1.5, 0.0, 0.0]), "outside [-1, 1]"),
    "end not after the last start": (lambda d: d["script"]["end_attitude_execution"].update(
        value=d["script"]["segments"][-1]["start_attitude_execution"]["value"]), "not after the last segment"),
    "more than eight segments": (lambda d: d["script"].update(segments=d["script"]["segments"] * 5), "at most 8"),
    "settle not positive": (lambda d: d["script"]["settle_s"].update(value=0.0), "must be a finite number > 0"),
    "chirp band inverted": (lambda d: d["script"].update(chirp={
        "axis": {"value": "roll", "unit": "1", "label": "scenario", "rationale": "x"},
        "amp_rad_s": {"value": 1.0, "unit": "rad/s", "label": "scenario", "rationale": "x"},
        "w_lo_rad_s": {"value": 5.0, "unit": "rad/s", "label": "scenario", "rationale": "x"},
        "w_hi_rad_s": {"value": 4.0, "unit": "rad/s", "label": "scenario", "rationale": "x"},
        "start_attitude_execution": {"value": 1, "unit": "1", "label": "scenario", "rationale": "x"},
        "duration_s": {"value": 1.0, "unit": "s", "label": "scenario", "rationale": "x"}}), "is not above w_lo_rad_s"),
}


@pytest.mark.parametrize("case", list(CASES))
def test_schema_refuses(case):
    fn, text = CASES[case]
    if case == "unlabelled number":
        doc = base()
        doc["script"]["end_attitude_execution"] = 5  # a bare number in place of an entry
        assert any("end_attitude_execution" in f for f in findings(doc))
        return
    got = findings(mutated(fn))
    assert any(text in f for f in got), (text, got)


def test_control_the_unmutated_document_has_no_findings():
    assert findings(base()) == []


def test_non_level_attitude_with_nonzero_rates_is_accepted():
    def fn(d):
        d["initial_state"]["attitude_q_wxyz"].update(value=[0.0, 1.0, 0.0, 0.0])
        d["initial_state"]["body_rates_frd_rad_s"].update(value=[11.7, 11.7, 11.7])
    assert findings(mutated(fn)) == []


def test_optional_chirp_and_disturbance_are_accepted_when_well_formed():
    def e(v, unit="1"):
        return {"value": v, "unit": unit, "label": "scenario", "rationale": "x"}

    def fn(d):
        d["script"]["chirp"] = {"axis": e("yaw"), "amp_rad_s": e(1.0, "rad/s"), "w_lo_rad_s": e(1.0, "rad/s"),
                                "w_hi_rad_s": e(9.0, "rad/s"), "start_attitude_execution": e(3), "duration_s": e(2.0, "s")}
        d["script"]["disturbance"] = {"yaw_nm": e(0.1, "N*m"), "start_attitude_execution": e(7)}
    doc = mutated(fn)
    assert findings(doc) == []
    v = l5s.values(doc)["script"]
    assert v["chirp"]["axis"] == "yaw" and v["disturbance"]["start_attitude_execution"] == 7


# ---- the runner's plan ----------------------------------------------------------------------------------------------

def plan_of(params=BUILD, name="step_roll"):
    return run_l5.plan(l5s.load(SCEN / f"{name}.yaml"), params, CARD)


def test_plan_origin_stamps_and_duration():
    p = plan_of()
    assert p.att_divisor == 4 and p.origin == 1600 and p.tick_of(p.origin) == 6400
    assert (p.att_divisor * p.origin * p.num_us) % p.den == 0, "the stamp phase of the oracle"
    release = l5s.values(l5s.load(SCEN / "step_roll.yaml"))["script"]["segments"][1]["start_attitude_execution"]
    assert p.segments[0][1] == p.stamp_us(p.origin + 1) and p.segments[1][1] == p.stamp_us(p.origin + release)
    assert p.stamp_us(p.origin + 1) - p.stamp_us(p.origin) == 625
    assert p.duration_ticks % 2 == 0 and p.duration_ticks > p.tick_of(p.origin + p.end)
    assert p.duration_ticks - 2 <= p.tick_of(p.origin + p.end) + 2
    o = p.overrides()
    assert o["l5_seg_count"] == (run_l5.I32, "2") and o["l5_seg1_roll"] == (run_l5.F32, "1.0")
    assert o["l5_seg2_t_us"] == (run_l5.I32, str(p.segments[1][1]))


def test_plan_refuses_a_world_tick_that_differs_from_the_build():
    with pytest.raises(run_l5.PlanError, match="differs from the build's"):
        plan_of(dict(BUILD, tick_period_num_us=500))
    with pytest.raises(run_l5.PlanError, match="differs from the build's"):
        plan_of(dict(BUILD, tick_period_den=8))
    plan_of()  # control: the matching tick plans


@pytest.mark.parametrize("divisor,ratio", ((3, 1), (1, 3), (3, 5), (7, 1)))
def test_plan_refuses_an_m_that_does_not_divide_the_attitude_divisor(divisor, ratio):
    with pytest.raises(run_l5.PlanError, match="does not divide the attitude divisor"):
        plan_of(dict(BUILD, rate_loop_divisor=divisor, att_loop_ratio=ratio))


@pytest.mark.parametrize("divisor,ratio", ((2, 1), (1, 2), (2, 2), (1, 4), (4, 1)))
def test_control_plan_accepts_an_attitude_divisor_both_m_divide(divisor, ratio):
    p = plan_of(dict(BUILD, rate_loop_divisor=divisor, att_loop_ratio=ratio))
    assert p.att_divisor % 2 == 0 and p.att_divisor == divisor * ratio


def test_plan_refuses_a_build_without_the_attitude_divisor():
    with pytest.raises(run_l5.PlanError, match="lacks"):
        plan_of({k: v for k, v in BUILD.items() if k != "att_loop_ratio"})


# ---- the build and the world ----------------------------------------------------------------------------------------

def _cache(tmp_path, params):
    plugin = tmp_path / "build" / "sim" / "gz" / "plugin"
    plugin.mkdir(parents=True)
    (tmp_path / "build" / "CMakeCache.txt").write_text(f"MARV_GZ_PARAMS:STRING={params}\n", encoding="utf-8")
    return plugin


def test_build_parameters_refuses_the_l4_build_and_accepts_the_l5_build(tmp_path):
    with pytest.raises(run_l5.PlanError, match="not the host-gz-l5 build"):
        run_l5.build_parameters(_cache(tmp_path / "l4", "marv_params_l4_rate_scripted"))
    name, path = run_l5.build_parameters(_cache(tmp_path / "l5", run_l5.COMPOSITION_PARAMS))
    assert name == run_l5.COMPOSITION_PARAMS and path.name == "param_defaults.cpp"


WORLD = ('<sdf><plugin filename="marv_gz_lockstep" name="x">\n'
         + "".join(f'  <sil_override param="ol_dshot_m{k}" type="i32">0</sil_override>\n' for k in range(1, 5))
         + "  <log_path>a</log_path>\n</plugin></sdf>")


def test_world_edit_adds_both_truth_elements_and_the_overrides():
    text = run_l5.world_edit({"att_kp": ("f32", "0.0")})(WORLD)
    assert text.count("<gyro_source>truth</gyro_source>") == 1
    assert text.count("<attitude_source>truth</attitude_source>") == 1
    assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in text and "ol_dshot" not in text


def test_world_edit_control_leaves_out_the_elements_it_is_told_to():
    assert "<attitude_source>" not in run_l5.world_edit({}, attitude_source=False)(WORLD)
    assert "<gyro_source>" not in run_l5.world_edit({}, gyro_source=False)(WORLD)
    assert "<attitude_source>" in run_l5.world_edit({}, gyro_source=False)(WORLD)


def test_world_edit_refuses_a_world_without_the_l2_dshot_overrides():
    with pytest.raises(run_scenario.RunError, match="expected 4"):
        run_l5.world_edit({})(WORLD.replace('param="ol_dshot_m1"', 'param="other"'))
    with pytest.raises(scn.ScenarioError):
        l5s.load(SCEN / "missing.yaml")


# ---- the recovery scenarios (R1, R2: decision 0006 F "T4 large-angle recovery") -------------------------------------------

RECOVERY = ("recover_inverted", "recover_inverted_exact", "recover_tumble")
PARAMS = {"tick_period_num_us": 625, "tick_period_den": 4, "rate_loop_divisor": 2, "att_loop_ratio": 1}


@pytest.mark.parametrize("name", RECOVERY)
def test_committed_recovery_scenarios_are_valid_and_have_no_segments(name):
    v = l5s.values(l5s.load(SCEN / f"{name}.yaml"))
    q = v["initial_state"]["attitude_q_wxyz"]
    assert v["script"]["segments"] == [] and v["initial_state"]["rotor_speed_rad_s"] == (
        "steady_tumble" if name == "recover_tumble" else "hover")
    assert (q == [0.0, 1.0, 0.0, 0.0]) == (name != "recover_inverted") and q[0] >= 0
    assert any(x != 0 for x in v["initial_state"]["body_rates_frd_rad_s"]) == (name == "recover_tumble")


def test_l2_world_input_of_a_tumble_carries_the_initial_state_s_own_separatrix_mu():
    doc = l5s.load(SCEN / "recover_tumble.yaml")
    p = run_l5.plan(doc, PARAMS, CARD)
    out = run_l5.l2_scenario_doc(doc, p, "recover_tumble_truth_fed_perfect_model", "recover_tumble.yaml", CARD)
    inertia = [float(x) for x in schema.load_yaml(CARD)["inertia_diag"]["value"]]
    assert scn.validate(out, "recover_tumble_truth_fed_perfect_model.yaml", inertia) == []
    assert 0 < out["separatrix_margin_min"]["value"] <= 1


def test_control_a_recovery_at_rest_has_no_separatrix_entry_and_a_tumble_without_a_card_is_refused():
    rest = l5s.load(SCEN / "recover_inverted.yaml")
    assert "separatrix_margin_min" not in run_l5.l2_scenario_doc(rest, run_l5.plan(rest, PARAMS, CARD), "s", "s.yaml", CARD)
    tumble = l5s.load(SCEN / "recover_tumble.yaml")
    with pytest.raises(run_l5.PlanError):
        run_l5.l2_scenario_doc(tumble, run_l5.plan(tumble, PARAMS, CARD), "s", "s.yaml")
