"""T4 of the L11 vertical loop in Gazebo on truth state (quad spec 4 L11 T4; decision 0020 section 5 "T4 (gz, truth-fed)"; the
form of tests/regression/quad/L05/gz/test_t4_steps.py).

Skipped only as conftest.py says; the gz CI step fails on a skip. Every run is truth-fed, perfect-model (tools/sim/run_l11.py):
not a validation run.

Runs. scenarios/quad/L11/<name>.yaml (height_step, thrust_offset, roll_shove), one gz process per m of its m_sequence [2, 1]
(tools/sim/run_l11.py run_sequence); the run's vertical execution n is the T3 oracle's execution n.

Envelope. The RECORDED T3 envelope, build/reference/quad/L11/t3/vertical_t3_envelope.txt (the box envelope of the design model
over the 17 x 17 tau x r grid, driven by the exact script from the hover trim, never re-seeded; generated, decision 0011): the
channel `z` of the script, one lo / hi pair per vertical execution n = 0 .. count - 1. Quantity: the z of each execution's
VERTICAL record (the float cast of the down position the firmware read).

Predicate (decision 0020 section 5). E(n) = |z_1(n) - z_2(n)| (m = 1 against m = 2), F the recorded T3 tolerance term of the
script's header line (T3 tolerance + last-halving change + kinematics change), Q the recorded DShot quantisation term of the
script's `z` channel (vertical_t3_q.txt). PASS iff every run is clean (complete trailer, a TRUTH record and a valid VERTICAL
record for every tick, no stale host step in the window), every DShot of every tick is in [idle, 2047], at every execution
lo(n) - (E(n) + F + Q) <= z_1(n) <= hi(n) + (E(n) + F + Q), and the last execution is within E + F + Q of the script's final
z_ref. Altitude drift is not allowed here (the loop holds it).

Binding. The committed scenario's values equal the envelope's header (z_ref, thrust disturbance, torque pulse, step execution,
count - 1 = end execution) and the recorded hover thrust, and its initial state is the hover trim.

Negative controls, harness only. Each passes only if the check fails, with no stale read: (a) the loop disabled (vert_kp_z,
vert_kp_v, vert_ki_v = 0 through sil_override; the law then outputs the hover feed-forward alone) fails the thrust offset (ii)
and the roll shove (iii); (b) vert_kp_z x 1.1 (f32(k f32(1.1)), as the T3 test scales it) fails all three scripts and vert_ki_v
x 1.1 fails (ii) and (iii). Not committed, because they pass this predicate and are enforced at T3 only: vert_kp_v x 1.1 on
every script and vert_ki_v x 1.1 on the height step (decision 0020 What changed, S7b); the one added tick is a firmware
delay, not tried here, and L5's T4 predicate does not see it either (enforced at T3).
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

CARD = l11_gz.CARD
SCEN = ROOT / "scenarios" / "quad" / "L11"
PLUGIN_DIR = run_l11.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L11" / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L11/t3")  # the generated golden, envelope and q (decision 0011)
SCENARIOS = ("height_step", "thrust_offset", "roll_shove")
DISABLED = ("thrust_offset", "roll_shove")
GAINS = ("vert_kp_z", "vert_kp_v", "vert_ki_v")
GAIN_SCALE = 1.1  # scenario test value: the spec's fine gain control (the T3 tests' kGainScale)
GAIN_CONTROLS = tuple(("vert_kp_z", n) for n in SCENARIOS)  # the fine control of every script (decision 0020, S7b)
LOOP_OFF = {k: (run_l11.F32, "0.0") for k in GAINS}


# ---- the recorded envelope ------------------------------------------------------------------------------------------

def parse_envelope(scenario):
    """lo, hi, header and F of the `z` channel of `scenario` in the recorded envelope file, and Q of vertical_t3_q.txt."""
    text = (T3_GENERATED / "vertical_t3_envelope.txt").read_text(encoding="utf-8").splitlines()
    f = re.compile(rf"^#\s+{scenario} z: end \S+ < F (\S+)$")
    F = next(float(m.group(1)) for m in map(f.match, text) if m)
    start = text.index(f"scenario {scenario}")
    head, i = {}, start + 1
    while not text[i].startswith("channel "):
        k, v = text[i].split(" ", 1)
        head[k] = v
        i += 1
    m = re.match(r"channel z first (\d+) count (\d+) halving_max_change (\S+)$", text[i])
    first, count = int(m.group(1)), int(m.group(2))
    assert first == 0 and text[i + 1] == "min_max"
    rows = [tuple(map(float, ln.split())) for ln in text[i + 2:i + 2 + count]]
    assert len(rows) == count and all(len(r) == 2 for r in rows)
    return {"scenario": scenario, "head": head, "count": count, "lo": [r[0] for r in rows], "hi": [r[1] for r in rows], "F": F,
            "Q": parse_q(T3_GENERATED / "vertical_t3_q.txt", scenario, "z")}


def parse_q(path, scenario, channel):
    """q of `channel` of `scenario` in the recorded quantisation term file (its `channel <name> q <value> ...` line)."""
    text = Path(path).read_text(encoding="utf-8").splitlines()
    start = text.index(f"scenario {scenario}")
    block = next((text[start + 1:start + 1 + k] for k, ln in enumerate(text[start + 1:]) if ln.startswith("scenario ")),
                 text[start + 1:])
    found = [float(ln.split()[3]) for ln in block if ln.startswith(f"channel {channel} q ")]
    assert len(found) == 1, f"{path}: {len(found)} q lines for {scenario} {channel}"
    return found[0]


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

def evaluate(z1, z2, env, z_end, stale):
    """The predicate of the module docstring on the z series of the m = 1 and m = 2 runs."""
    Q, F = env["Q"], env["F"]
    worst, violations = None, 0
    for n in range(env["count"]):
        e = abs(z1[n] - z2[n])
        out = max(env["lo"][n] - z1[n], z1[n] - env["hi"][n])
        slack = out - (e + F + Q)
        violations += slack > 0
        if worst is None or slack > worst["slack"]:
            worst = {"n": n, "slack": slack, "z": z1[n], "lo": env["lo"][n], "hi": env["hi"][n], "E": e}
    e_end = abs(z1[-1] - z2[-1])
    end_slack = abs(z1[-1] - z_end) - (e_end + F + Q)
    return {"passed": violations == 0 and end_slack <= 0 and not any(stale.values()), "violations": violations,
            "worst": worst, "end_slack": end_slack, "stale_reads_in_window": stale, "F": F, "Q": Q, "N": env["count"]}


def facts(s, idle, top):
    """What the checks need of one run (the logs are dropped: three scenarios and a control would hold hundreds of MB)."""
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
    """The [m = 2, m = 1] sequence of scenario `name` with the harness `overrides`, evaluated against its recorded envelope;
    cached per (name, tag), so the checks and the controls share a run."""
    if (name, tag) in _FLOWN:
        return _FLOWN[name, tag]
    env = parse_envelope(name)
    idle, top = run_l11.l4.dshot_idle_bound(build_params())
    runs, path = run_l11.run_sequence(CARD, SCEN / f"{name}.yaml", tmp_path_factory.mktemp(f"{name}_{tag}"), PLUGIN_DIR,
                                      overrides=overrides)
    by_m = {f["m"]: f for f in map(lambda s: facts(s, idle, top), runs)}
    assert [s.run.m for s in runs] == [2, 1] and len(by_m[1]["z"]) == len(by_m[2]["z"]) == env["count"]
    script = l11s.values(l11s.load(SCEN / f"{name}.yaml"))["script"]
    z_end = l11_gz.l4.r32(script["segments"][-1]["z_ref_m"]) if script["segments"] else 0.0
    ev = evaluate(by_m[1]["z"], by_m[2]["z"], env, z_end, {f"m{m}": f["stale"] for m, f in by_m.items()})
    wall = sum(s.run.wall_s for s in runs)
    run_l11.write_report(runs, dict(ev, gz_wall_s=wall), path)
    _FLOWN[name, tag] = {"ev": ev, "by_m": by_m, "wall": wall, "report": path, "plan": runs[0].plan}
    return _FLOWN[name, tag]


def verdict_line(name, label, r):
    ev, w = r["ev"], r["ev"]["worst"]
    return (f"{name:<13} {label:<14} {'PASS' if ev['passed'] else 'FAIL'}  F {ev['F']:.6e}, Q {ev['Q']:.6e}, worst n {w['n']} "
            f"(z {w['z']:+.6e} in [{w['lo']:+.6e}, {w['hi']:+.6e}], E {w['E']:.3e}, slack {w['slack']:+.6e}), end slack "
            f"{ev['end_slack']:+.6e}, violations {ev['violations']}, stale {ev['stale_reads_in_window']}, N {ev['N']}, "
            f"gz wall {r['wall']:.2f} s, report {r['report']}")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", SCENARIOS)
def test_vertical_loop_meets_the_design_envelope_and_returns_to_the_reference(tmp_path_factory, name, capsys):
    r = fly(tmp_path_factory, name)
    _say(capsys, verdict_line(name, "step", r))
    assert r["ev"]["passed"], r["ev"]


# ---- the binding of the scenarios to the recorded envelope ----------------------------------------------------------

def binding_findings(name, doc, params):
    """Why the scenario `doc` does not match the recorded envelope of `name`, the hover thrust and the hover trim; empty when
    it does."""
    head = parse_envelope(name)["head"]
    inputs, v = recorded_inputs(), l11s.values(doc)
    p, script, state = run_l11.plan(doc, params, CARD), v["script"], v["initial_state"]
    out = []

    def check(what, got, want):
        if got != want:
            out.append(f"{what}: scenario {got!r}, recorded {want!r}")

    step = int(head["step_execution"])
    seg = script["segments"]
    check("segments", [(s["start_vertical_execution"], s["z_ref_m"]) for s in seg],
          [(step, float(head["z_ref_m"]))] if float(head["z_ref_m"]) != 0.0 else [])
    td = script.get("thrust_disturbance")
    check("thrust disturbance", (l11_gz.l4.r32(p.thrust_disturbance["thrust_n"]), td["start_vertical_execution"]) if td else None,
          (inputs["thrust_offset_n"], step) if float(head["thrust_disturbance_n"]) != 0.0 else None)
    if td:
        check("thrust disturbance, header", float(head["thrust_disturbance_n"]), inputs["thrust_offset_n"])
    pulse = p.torque_pulse
    check("torque pulse", (pulse["axis"], pulse["torque_nm"], pulse["t0_us"], pulse["dur_us"],
                           script["torque_pulse"]["start_vertical_execution"]) if pulse else None,
          (int(head["torque_pulse_axis"]), float(head["torque_pulse_nm"]), int(head["torque_pulse_t0_us"]),
           int(head["torque_pulse_dur_us"]), step) if "torque_pulse_nm" in head else None)
    check("end execution", (script["end_vertical_execution"], p.end), (parse_envelope(name)["count"] - 1,) * 2)
    check("hover thrust", p.thrust_n, inputs["hover_thrust_n"])
    check("initial state", (state["position_ned_m"][2], state["attitude_q_wxyz"], state["velocity_ned_m_s"][2],
                            state["body_rates_frd_rad_s"], state["rotor_speed_rad_s"], v["thrust"]),
          (0.0, [1.0, 0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0], "hover", "hover"))
    return out


def test_scenarios_equal_the_recorded_envelope_scripts_and_start_at_the_hover_trim():
    params = build_params()
    for name in SCENARIOS:
        assert binding_findings(name, l11s.load(SCEN / f"{name}.yaml"), params) == [], name
    moved = copy.deepcopy(l11s.load(SCEN / "roll_shove.yaml"))  # control: a moved pulse start is found
    moved["script"]["torque_pulse"]["start_vertical_execution"]["value"] += 1
    assert any(f.startswith("torque pulse") for f in binding_findings("roll_shove", moved, params))


# ---- negative controls ----------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", DISABLED)
def test_control_loop_disabled_fails_the_predicate(tmp_path_factory, name, capsys):
    r = fly(tmp_path_factory, name, "loop_off", LOOP_OFF)
    _say(capsys, verdict_line(name, "loop disabled", r))
    for f in r["by_m"].values():
        assert f["overrides"] == LOOP_OFF
        for k in GAINS:
            assert f'<sil_override param="{k}" type="f32">0.0</sil_override>' in f["world"]
    assert r["ev"]["stale_reads_in_window"] == {"m2": 0, "m1": 0}, "the control must fail on the envelope, not on a stale read"
    assert not r["ev"]["passed"] and (r["ev"]["violations"] > 0 or r["ev"]["end_slack"] > 0), r["ev"]


@pytest.mark.parametrize("gain,name", GAIN_CONTROLS)
def test_control_gain_times_1_1_fails_the_predicate(tmp_path_factory, gain, name, capsys):
    k = l11_gz.l4.r32(build_params()[gain] * l11_gz.l4.r32(GAIN_SCALE))
    r = fly(tmp_path_factory, name, f"{gain}_x1_1", {gain: (run_l11.F32, repr(k))})
    _say(capsys, verdict_line(name, f"{gain} x1.1", r))
    assert k != build_params()[gain]
    for f in r["by_m"].values():
        assert f["overrides"] == {gain: ("f32", repr(k))}
        assert f'<sil_override param="{gain}" type="f32">{k!r}</sil_override>' in f["world"]
    assert r["ev"]["stale_reads_in_window"] == {"m2": 0, "m1": 0}, "the control must fail on the envelope, not on a stale read"
    assert not r["ev"]["passed"] and (r["ev"]["violations"] > 0 or r["ev"]["end_slack"] > 0), r["ev"]
