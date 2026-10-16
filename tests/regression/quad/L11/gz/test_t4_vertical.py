"""T4 of the L11 vertical loop in Gazebo on truth state (quad spec 4 L11 T4: on z, a height step settles by the deadline (S) and
stays bounded (B), and a constant thrust offset and a roll-torque shove each settle (S); predicates of decision 0029,
loop-disabled control; decision 0020 section 5 "T4 (gz, truth-fed)"; decision 0032).

Skipped only as conftest.py says; the gz CI step fails on a skip. Every run is truth-fed, perfect-model (tools/sim/run_l11.py):
not a validation run.

Runs. scenarios/quad/L11/<name>.yaml (height_step, thrust_offset, roll_shove), one gz process per m of its m_sequence [2, 1]
(tools/sim/run_l11.py run_sequence); the run's vertical execution n is the T3 oracle's execution n.

Quantity. The z of each execution's VERTICAL record (the float cast of the down position the firmware read), error z - z_ref,
z_ref the script's last segment reference (float32; 0 when the script has no segment). One segment: executions n_0 .. end,
n_0 the execution where the step or the disturbance starts (execution 1 of all three scripts; the seed execution 0 is the hover
trim at the reference). The deadline is the end of the run, the T3 golden's segment_executions (the binding below).

Predicates (decision 0029 and the roles of decision 0031, tools/sim/t4_rule.py scenario_checks, on the m = 1 run). s is the
budget's t4_settle_fraction, E = |z_1 - z_2| (m = 1 against m = 2), Delta the largest |error| of the segment (one quantity).
  S  at the last execution |error| <= s Delta + E;
  B  at every execution |error(n)| <= |error(n_0)| + s Delta + E (E the largest over the segment); not applied to a segment with
     a disturbance onset (thrust_offset, roll_shove).
Roles, static from the scenario file: z is STEPPED in height_step (the script has a segment, a reference change at n_0) and
DRIVEN in thrust_offset and roll_shove (the error is 0 at n_0, the hover trim at the reference, and the disturbance sets it
moving), so those two are S only. PASS iff every run is clean (complete trailer, a TRUTH record and a valid VERTICAL record for
every tick, no stale host step in the window), every DShot of every tick is in [idle, 2047], and S (and B where it applies)
holds. Altitude drift is not allowed here (the loop holds it).

Binding. The committed scenarios' values equal the T3 golden (build/reference/quad/L11/t3/vertical_t3_golden.txt, generated,
decision 0011): the end execution of all three is its header's segment_executions, the step and the disturbance of height_step
and thrust_offset are its blocks' (z_ref, thrust disturbance, step execution), the roll shove starts at the same step
execution; the hover thrust and the thrust offset are the recorded inputs, and the initial state is the hover trim.

Negative control, harness only. The test passes only if the predicates fail, with no stale read: the loop disabled
(vert_kp_z, vert_kp_v, vert_ki_v = 0 through sil_override; the law then outputs the hover feed-forward alone) fails S in all
three scenarios (decision 0032).
"""

import copy
import re
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import l11_gz  # noqa: E402

ROOT = l11_gz.ROOT
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402
import run_l11  # noqa: E402
import run_scenario  # noqa: E402
import l11_scenario as l11s  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = l11_gz.CARD
SCEN = ROOT / "scenarios" / "quad" / "L11"
PLUGIN_DIR = run_l11.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L11" / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L11/t3")  # the generated golden (decision 0011)
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
SCENARIOS = ("height_step", "thrust_offset", "roll_shove")
GAINS = ("vert_kp_z", "vert_kp_v", "vert_ki_v")
LOOP_OFF = {k: (run_l11.F32, "0.0") for k in GAINS}
UNIT = "m"
SEGMENT = "segment"


# ---- the T3 golden --------------------------------------------------------------------------------------------------

def parse_golden():
    """The golden's header segment_executions and, per script block, its executions, step_execution, z_ref_m and
    thrust_disturbance_n."""
    text = (T3_GENERATED / "vertical_t3_golden.txt").read_text(encoding="utf-8").splitlines()
    head = next(m for m in map(re.compile(r"^# segment_seconds \S+\s+segment_executions (\d+)\s+doublings").match, text) if m)
    out = {"segment_executions": int(head.group(1)), "scripts": {}}
    for i, line in enumerate(text):
        if line.startswith("scenario "):
            block = dict(text[i + k].split(" ", 1) for k in range(1, 5))
            assert list(block) == ["executions", "step_execution", "z_ref_m", "thrust_disturbance_n"], (line, list(block))
            out["scripts"][line.split()[1]] = {"executions": int(block["executions"]), "step_execution": int(block["step_execution"]),
                                               "z_ref_m": float(block["z_ref_m"]),
                                               "thrust_disturbance_n": float(block["thrust_disturbance_n"])}
    return out


def recorded_inputs():
    out = {}
    for line in (T3_REFERENCE / "vertical_t3_inputs.txt").read_text().splitlines():
        line = line.split("#")[0].split()
        if line:
            out[line[0]] = int(line[1]) if line[1].lstrip("-").isdigit() else float.fromhex(line[1])
    return out


def build_params():
    _, defaults = run_l11.build_parameters(PLUGIN_DIR)
    return run_l11.l4.read_param_defaults(defaults)


# ---- the runs -------------------------------------------------------------------------------------------------------

def segment_of(doc):
    """(n_0, z_ref, role, disturbed) of the scenario `doc`, static from the file: n_0 the execution where the step or a
    disturbance starts, z_ref the last segment's reference (float32; 0 without a segment), STEPPED when the script has a
    segment and DRIVEN otherwise, disturbed when it has a thrust disturbance or a torque pulse."""
    script = l11s.values(doc)["script"]
    segs, td, tp = script["segments"], script.get("thrust_disturbance"), script.get("torque_pulse")
    starts = [s["start_vertical_execution"] for s in segs] + [d["start_vertical_execution"] for d in (td, tp) if d]
    assert len(set(starts)) == 1, f"one onset execution expected, got {starts}"
    z_ref = l11_gz.l4.r32(segs[-1]["z_ref_m"]) if segs else 0.0
    return starts[0], z_ref, (t4_rule.STEPPED if segs else t4_rule.DRIVEN), bool(td or tp)


def evaluate(z1, z2, seg, stale):
    """S and B (module docstring) on the z series of the m = 1 and m = 2 runs, `seg` the segment_of tuple."""
    n0, z_ref, role, disturbed = seg
    ns = range(n0, len(z1))
    diff = [abs(z1[n] - z2[n]) for n in ns]
    checks = t4_rule.scenario_checks({SEGMENT: {"z": (UNIT, role, [z1[n] - z_ref for n in ns])}}, SETTLE_FRACTION,
                                     {SEGMENT: {"z": diff[-1]}}, {SEGMENT: {"z": max(diff)}},
                                     {SEGMENT} if disturbed else frozenset())[SEGMENT]
    return {"passed": all(c.ok for c in checks["S"] + checks["B"]) and not any(stale.values()), "S": checks["S"],
            "B": checks["B"], "stale_reads_in_window": stale, "N": len(z1), "n0": n0, "role": role, "s": SETTLE_FRACTION}


def facts(s, idle, top):
    """What the checks need of one run (the logs are dropped: three scenarios and their controls would hold hundreds of MB)."""
    log, p = s.run.log, s.plan
    ticks = log["ticks"]
    assert [d["vertical"] for d in s.executions].count(None) == 0, "an execution without a VERTICAL record"
    assert run_scenario.complete_trailer(log, s.run.iterations, s.run.m)
    assert len(log["truths"]) == len(ticks), "a tick without a TRUTH record"
    assert {t["vertical"]["flags"] for t in ticks} == {l11_gz.VALID}, "a VERTICAL record that is not valid"
    lo, hi = min(min(t["dshot"]) for t in ticks), max(max(t["dshot"]) for t in ticks)
    assert idle <= lo and hi <= top, f"m = {s.run.m}: DShot [{lo}, {hi}] outside [{idle}, {top}]"
    world = Path(s.run.world_path).read_text(encoding="utf-8")
    assert world.count("<attitude_source>truth</attitude_source>") == world.count(run_l11.VERTICAL_ELEMENT) == 1
    assert "truth_fed_perfect_model" in Path(s.run.log_path).name
    return {"m": s.run.m, "z": [d["vertical"]["z_ned_m"] for d in s.executions[:p.end + 1]], "stale": len(s.window_stale),
            "dshot": (lo, hi), "wall": s.run.wall_s, "overrides": s.harness_overrides, "world": world}


_FLOWN = {}


def fly(tmp_path_factory, name, tag="step", overrides=None):
    """The [m = 2, m = 1] sequence of scenario `name` with the harness `overrides`, evaluated on S and B; cached per (name,
    tag), so the checks share a run."""
    if (name, tag) in _FLOWN:
        return _FLOWN[name, tag]
    idle, top = run_l11.l4.dshot_idle_bound(build_params())
    runs, path = run_l11.run_sequence(CARD, SCEN / f"{name}.yaml", tmp_path_factory.mktemp(f"{name}_{tag}"), PLUGIN_DIR,
                                      overrides=overrides)
    by_m = {f["m"]: f for f in map(lambda s: facts(s, idle, top), runs)}
    assert [s.run.m for s in runs] == [2, 1] and len(by_m[1]["z"]) == len(by_m[2]["z"]) == parse_golden()["segment_executions"] + 1
    ev = evaluate(by_m[1]["z"], by_m[2]["z"], segment_of(l11s.load(SCEN / f"{name}.yaml")),
                  {f"m{m}": f["stale"] for m, f in by_m.items()})
    wall = sum(s.run.wall_s for s in runs)
    run_l11.write_report(runs, dict(ev, gz_wall_s=wall), path)
    _FLOWN[name, tag] = {"ev": ev, "by_m": by_m, "wall": wall, "report": path, "plan": runs[0].plan}
    return _FLOWN[name, tag]


def verdict_line(name, label, r):
    ev = r["ev"]

    def part(checks):
        return ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}" for c in checks) or "-"
    return (f"{name:<13} {label:<14} {'PASS' if ev['passed'] else 'FAIL'}  {ev['role']}, S [{part(ev['S'])}], B [{part(ev['B'])}], "
            f"stale {ev['stale_reads_in_window']}, N {ev['N']}, gz wall {r['wall']:.2f} s, report {r['report']}")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", SCENARIOS)
def test_vertical_loop_settles_and_stays_bounded(tmp_path_factory, name, capsys):
    r = fly(tmp_path_factory, name)
    _say(capsys, verdict_line(name, "step", r))
    assert r["ev"]["passed"], r["ev"]


# ---- the binding of the scenarios to the T3 golden ------------------------------------------------------------------

def binding_findings(name, doc, params):
    """Why the scenario `doc` does not match the T3 golden, the recorded inputs, the hover thrust and the hover trim; empty
    when it does. The roll shove has no golden block: it must start at the golden's step execution."""
    golden = parse_golden()
    head = golden["scripts"].get(name)
    step = next(iter(golden["scripts"].values()))["step_execution"]
    assert all(b["step_execution"] == step for b in golden["scripts"].values())
    inputs, v = recorded_inputs(), l11s.values(doc)
    p, script, state = run_l11.plan(doc, params, CARD), v["script"], v["initial_state"]
    out = []

    def check(what, got, want):
        if got != want:
            out.append(f"{what}: scenario {got!r}, recorded {want!r}")

    seg = script["segments"]
    check("segments", [(s["start_vertical_execution"], s["z_ref_m"]) for s in seg],
          [(head["step_execution"], head["z_ref_m"])] if head and head["z_ref_m"] != 0.0 else [])
    td = script.get("thrust_disturbance")
    check("thrust disturbance", (l11_gz.l4.r32(p.thrust_disturbance["thrust_n"]), td["start_vertical_execution"]) if td else None,
          (inputs["thrust_offset_n"], step) if head and head["thrust_disturbance_n"] != 0.0 else None)
    if td:
        check("thrust disturbance, header", head["thrust_disturbance_n"], inputs["thrust_offset_n"])
    pulse = p.torque_pulse
    t0_us = step * inputs["rate_loop_divisor"] * inputs["vert_loop_ratio"] * inputs["tick_period_num_us"] \
        // inputs["tick_period_den"]
    check("torque pulse", (pulse["t0_us"], script["torque_pulse"]["start_vertical_execution"]) if pulse else None,
          (t0_us, step) if head is None else None)
    check("end execution", (script["end_vertical_execution"], p.end), (golden["segment_executions"],) * 2)
    check("hover thrust", p.thrust_n, inputs["hover_thrust_n"])
    check("initial state", (state["position_ned_m"][2], state["attitude_q_wxyz"], state["velocity_ned_m_s"][2],
                            state["body_rates_frd_rad_s"], state["rotor_speed_rad_s"], v["thrust"]),
          (0.0, [1.0, 0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], "hover", "hover"))
    return out


def test_scenarios_equal_the_golden_and_start_at_the_hover_trim():
    params = build_params()
    for name in SCENARIOS:
        assert binding_findings(name, l11s.load(SCEN / f"{name}.yaml"), params) == [], name
    moved = copy.deepcopy(l11s.load(SCEN / "roll_shove.yaml"))  # control: a moved pulse start is found
    moved["script"]["torque_pulse"]["start_vertical_execution"]["value"] += 1
    assert any(f.startswith("torque pulse") for f in binding_findings("roll_shove", moved, params))
    ended = copy.deepcopy(l11s.load(SCEN / "height_step.yaml"))  # control: a moved deadline is found
    ended["script"]["end_vertical_execution"]["value"] -= 1
    assert any(f.startswith("end execution") for f in binding_findings("height_step", ended, params))


# ---- negative control -----------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", SCENARIOS)
def test_control_loop_disabled_fails_the_predicates(tmp_path_factory, name, capsys):
    r = fly(tmp_path_factory, name, "loop_off", LOOP_OFF)
    _say(capsys, verdict_line(name, "loop disabled", r))
    for f in r["by_m"].values():
        assert f["overrides"] == LOOP_OFF
        for k in GAINS:
            assert f'<sil_override param="{k}" type="f32">0.0</sil_override>' in f["world"]
    assert r["ev"]["stale_reads_in_window"] == {"m2": 0, "m1": 0}, "the control must fail on the predicates, not on a stale read"
    assert not r["ev"]["passed"] and any(not c.ok for c in r["ev"]["S"]), r["ev"]
