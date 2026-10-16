"""T4 acro of the L4 rate loop in Gazebo on truth gyro (quad spec 4 L4 pass bar, T4: "acro with a scripted stick sequence
completes without saturation beyond the mixer's documented behaviour"; decision 0005 "T4" acro predicate, owner decision
9; decision 0004 items 3 and 5; Luis's Option 1 and recovery ruling, 2026-09-30; Luis's ruling A, 2026-10-15,
decision 0030).

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l4 build of libmarv_gz_lockstep.so is
absent (`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`); the gz CI step fails on a skip. Every run is
truth-fed, perfect-model (tools/sim/run_l4.py): not a validation run.

Runs. scenarios/quad/L04/acro.yaml at m = 2 and m = 1 (tools/sim/run_l4.py run_acro, one gz process each): seven
segments of full-stick setpoints (float32(stick x rate_max_<axis>) of the build) at hover collective. Each run is then
replayed (run_l4.replay_acro): tests/regression/quad/L04/replay/l4_acro_replay, from the plugin's build tree, re-executes
the composition's due-tick sequence on the run's logged IMU samples with the run's sil_overrides, and gives what the
SIL ABI does not output: the torque request, the achieved torque and thrust and the L3 saturation flags per execution,
with s and t derived as achieved / requested. The flags are recovered this way, through a replay that is bit-exact.

Segment membership (run_l4.acro_phases, from acro.yaml's sticks): every execution before the combined segment (the
settle, the single-axis segments, the reversal and the zero segment before the combined one) is 'linear'; the combined
segment's executions are 'combined'; the zero segment after it is the 'recovery' window.

Predicate (decision 0029 B and T, tools/sim/t4_rule.py; decision 0030). PASS iff
  (i)   each run ends cleanly: gz accepted the run (run_scenario.check_exit), no hal_panic, and the trailer counts equal
        the run's (every host step, tick and applied record logged);
  (ii)  every DShot command of every tick of both runs lies in [ceil(D(omega_idle)), kDshotThrottleMax], the idle bound
        computed from the build's parameters as thrust_to_dshot computes it (run_l4.dshot_idle_bound, decision 0004);
  (iii) behaviour on the linear segments, on the m = 1 run, on the three gyro axes, error q = w - q_ref with w the gyro
        sample the firmware received and q_ref the segment's setpoint (float32(stick x rate_max), the plan's segments),
        at the segment's executions first .. last (run_l4.acro_phases):
          B  |q(n)| <= |q(n_0)| + s Delta + E at every execution of the segment, s the budget's t4_settle_fraction,
             Delta the largest |q| in the segment over the three axes (all rad/s), E_a the largest |w_a,m=1 - w_a,m=2|
             over the segment (decision 0003 E; the predicate takes one E per quantity). Here B is checked on the
             commanded axis, the one whose stick is deflected. B on the other axes (none commanded, or released at
             the segment's start) belongs to the known failing item below;
          T  on each single-axis segment (a 'single' or 'reversal' segment, one nonzero stick), the stepped axis's |q| at
             the segment's last execution is smaller than at its first. No number: the segments are 0.5 s, about 3.1
             tau_ref, so nothing is expected to have settled;
        and no stale read in the evaluation window of either run (0003 item 11). The design-model envelope that this
        item used to compare with (decision 0005) is not a predicate any more (decision 0030);
  (iv)  the replay reproduces both runs: per execution the log's tick, stamp and DShot, bit for bit
        (run_l4.replay_fidelity; its own test too);
  (v)   over the combined segment, from the m = 1 replay: flag consistency (run_l4.flag_findings: a set flag has
        |achieved| < |requested|, and |requested| - |achieved| > b_a, the decision 0005 anti-windup bound, has the flag
        set), and s = t = 1 at every execution (nan only where the requests it divides are exactly zero): no saturation
        is a checked fact of the run, not an assumption.
Cross-axis coupling, its own test, the known failing item (strict xfail until tests/regression/quad/L06/XFAIL_GATE_CLOSED
exists; decisions 0005, 0009, 0029, 0030), PASS iff both hold:
  B on the uncommanded axes of every linear segment, Delta and E as in (iii). The run fails it on segments 3 to 5, up
  to 0.99 rad/s of roll against a bound of 0.26 rad/s with every stick centred (decision 0030): gyroscopic coupling the
  per-axis PI does not reject, and on segment 3 possibly also the released roll axis's own lag (not separated);
  B on the recovery window (the zero segment after the combined one, sticks centred, q_ref = 0 on every axis), E as above.
  The run fails it: the roll integrator releases what it absorbed of the combined segment's coupling as an uncommanded
  roll with the stick centred, 0.09 to 10.29 rad/s (tests/regression/quad/L04/results/acro_cause/cause.txt).
  The deferred L6 feed-forward owns both (decisions 0029, 0030).
Saturation during transients would be expected (air mode, owner decision 9); altitude is not checked.

Negative controls, harness only (0003 item 9). Each test passes only if the check fails:
  (a) the loop disabled, rate_kp_<axis> = rate_ki_<axis> = 0 on all three axes through sil_override: (iii) fails;
  (b) (ii): no harness-side control exists. thrust_to_dshot clamps every command to [ceil(D(omega_idle)),
      kDshotThrottleMax] with the parameters in effect, and the build's idle speed is the ESC map's minimum, so the bound
      is kDshotThrottleMin itself; no parameter override moves a command below it without moving the bound. The check is
      instead shown not to be vacuous on the run's own data: one command set to the bound minus one must fail it;
  (d) (v): planted replay rows, one per rule (a flag set with achieved equal to requested, a reduction beyond b_a with the
      flag clear, s < 1 with the flag consistent), each must fail its check.
"""

import dataclasses
import math
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tests" / "regression" / "quad" / "L04" / "t3" / "reference"))
import rate_t3_oracle as oracle  # noqa: E402
import run_l4  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L04" / "acro.yaml"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
UNIT = "rad/s"
AXES = ("roll", "pitch", "yaw")
HALVED = 2 * (oracle.GRID - 1) + 1  # the oracle's halved grid, 17 x 17, as the step test
PHASES = (2, 3)  # the oracle's two dt phases of an injection (its main()), as the step test
XFAIL_GATE = ROOT / "tests" / "regression" / "quad" / "L06" / "XFAIL_GATE_CLOSED"  # created by L6 stage (e), decision 0009
COUPLING_KNOWN_FAILING = (
    "decisions 0005, 0029 and 0030, known failing, owned by the deferred L6 feed-forward: the PI baseline's bandwidth "
    "(crossover 8.3 rad/s) cannot reject gyroscopic coupling; stepping one axis drives the uncommanded axes beyond B on "
    "the linear segments, and the roll integrator absorbs the combined segment's coupling and then releases it, an "
    "uncommanded roll with the stick centred, B fails (tests/regression/quad/L04/results/acro_cause/cause.txt). It must "
    "pass before L6 passes and before L8.")

needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l4 build of libmarv_gz_lockstep.so is absent")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the predicate --------------------------------------------------------------------------------------------------

def dshot_findings(rows, lo, hi):
    """(tick, motor, command) of every command outside [lo, hi]."""
    return [(j, i + 1, v) for j, row in enumerate(rows) for i, v in enumerate(row) if not lo <= v <= hi]


def clean_findings(s):
    out = []
    if not run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m):
        out.append(f"m = {s.run.m}: trailer counts differ from the run")
    if "hal_panic" in s.run.stderr_tail:
        out.append(f"m = {s.run.m}: hal_panic in gz's stderr")
    return out


def segment_errors(s, first, last, setpoint):
    """Per gyro axis the run's gyro error w - q_ref over the executions first .. last of one segment."""
    return [[s.executions[k]["gyro"][a] - setpoint[a] for k in range(first, last + 1)] for a in range(len(AXES))]


def segment_e(runs, first, last):
    """E_a: the largest |w_a,m=1 - w_a,m=2| over the executions first .. last."""
    return {axis: max(abs(runs[1].executions[k]["gyro"][a] - runs[2].executions[k]["gyro"][a])
                      for k in range(first, last + 1)) for a, axis in enumerate(AXES)}


def behaviour_evaluation(runs, table):
    """B on every linear segment and T on every single-axis one (module docstring), per segment the checks: "B" on the
    commanded axis (nonzero stick), "B cross" on the others (the known failing item)."""
    p = runs[1].plan
    out = {}
    for number, kind, first, last, phase in table:
        if phase != "linear":
            continue
        _, _, setpoint, stick = p.segments[number - 1]
        errors = segment_errors(runs[1], first, last, setpoint)
        segment = {axis: (UNIT, errors[a]) for a, axis in enumerate(AXES)}
        bounded = t4_rule.bounded(segment, SETTLE_FRACTION, segment_e(runs, first, last))
        commanded = {AXES[a] for a, x in enumerate(stick) if x != 0}
        entry = {"kind": kind, "B": [c for c in bounded if c.name in commanded],
                 "B cross": [c for c in bounded if c.name not in commanded], "T": None}
        if kind in ("single", "reversal"):
            a = next(i for i, x in enumerate(stick) if x != 0)
            entry["T"] = {"axis": AXES[a], "first |q|": abs(errors[a][0]), "last |q|": abs(errors[a][-1]),
                          "ok": t4_rule.shrinks(errors[a])}
        out[f"segment {number} ({kind})"] = entry
    return out


def behaviour_passed(behaviour):
    return all(c.ok for e in behaviour.values() for c in e["B"]) and all(e["T"]["ok"] for e in behaviour.values() if e["T"])


def s_t_findings(rows):
    """(execution, s, t) of every replayed row whose s or t is not 1; nan counts as 1 only where the requests it
    divides are exactly zero."""
    def one(x, *requests):
        return x == 1.0 or (math.isnan(x) and all(r == 0.0 for r in requests))
    return [(r["k"], r["s"], r["t"]) for r in rows
            if not (one(r["s"], r["req_roll"], r["req_pitch"]) and one(r["t"], r["req_yaw"]))]


def saturation_evaluation(rows, phases, abs_bm):
    """(v) on the m = 1 replay rows of the combined segment."""
    combined = [r for r in rows if r["k"] < len(phases) and phases[r["k"]] == "combined"]
    findings, counts = run_l4.flag_findings(combined, abs_bm)
    not_one = s_t_findings(combined)
    nan = sum(1 for r in combined if math.isnan(r["s"]) or math.isnan(r["t"]))
    return {"passed": bool(combined) and not findings and not not_one,
            "combined executions": len(combined), "per axis": counts, "flag findings": len(findings),
            "first flag findings": findings[:5], "executions with s or t not 1": len(not_one),
            "first of them": not_one[:5], "executions with s or t nan (zero request)": nan}


def cross_axis_passed(behaviour):
    return all(c.ok for e in behaviour.values() for c in e["B cross"])


def recovery_evaluation(runs, table):
    """B on the recovery window, q_ref = 0 on every axis (module docstring): the checks, or None without a window."""
    rows = [(first, last) for _, _, first, last, phase in table if phase == "recovery"]
    if not rows:
        return None
    first, last = rows[0]
    segment = {axis: (UNIT, errors) for axis, errors in zip(AXES, segment_errors(runs[1], first, last, (0.0, 0.0, 0.0)))}
    return {"window": [first, last], "B": t4_rule.bounded(segment, SETTLE_FRACTION, segment_e(runs, first, last))}


def evaluate(runs, replays, abs_bm):
    lo, hi = runs[1].dshot_bounds
    p = runs[1].plan
    phases, table = run_l4.acro_phases(p)
    clean = [f for s in runs.values() for f in clean_findings(s)]
    bad = {m: dshot_findings(s.dshot, lo, hi) for m, s in runs.items()}
    behaviour = behaviour_evaluation(runs, table)
    fidelity = {f"m{m}": run_l4.replay_fidelity(s, replays[m]) for m, s in runs.items()}
    sat = saturation_evaluation(replays[1], phases, abs_bm) if fidelity["m1"] is None else {
        "passed": False, "reason": "the m = 1 replay does not reproduce the run"}
    recovery = recovery_evaluation(runs, table)
    stale = {f"m{m}": len(s.window_stale) for m, s in runs.items()}
    commands = [v for s in runs.values() for row in s.dshot for v in row]
    ends = sum(1 for k in range(p.end_execution + 1) if any(v in (lo, hi) for v in runs[1].dshot[p.tick_of(k)]))
    return {
        "passed": (not clean and not any(bad.values()) and behaviour_passed(behaviour) and not any(stale.values())
                   and all(f is None for f in fidelity.values()) and sat["passed"]),
        "segments (number, kind, first execution, last execution, phase)": [list(t) for t in table],
        "(i) clean": clean or "yes", "(ii) DShot bounds": [lo, hi],
        "(ii) DShot range seen": [min(commands), max(commands)],
        "(ii) out of range": {f"m{m}": len(b) for m, b in bad.items()},
        "(iii) behaviour (B, T) per linear segment": behaviour, "(iii) passed": behaviour_passed(behaviour),
        "(iv) replay fidelity (first mismatch)": {k: v or "none" for k, v in fidelity.items()},
        "(v) combined segment (m = 1 replay)": sat,
        "recovery (known failing, decisions 0005 and 0029)": recovery,
        "recovery passed": bool(recovery) and all(c.ok for c in recovery["B"]) and not any(stale.values()),
        "cross axes passed (known failing, decision 0030)": cross_axis_passed(behaviour),
        "stale reads": stale,
        "executions with a motor at a DShot end (m = 1)": ends,
        "gz wall (s)": sum(s.run.wall_s for s in runs.values()),
    }


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Acro:
    runs: dict
    replays: dict
    evaluation: dict
    report_path: str
    out_dir: Path


def design_bound(plan, defaults):
    """P_a, TOL_a, H_a, F_a and the 17 x 17 envelope (lo, hi) and Z_a = rest_bound(lo, hi) per axis (module
    docstring), from the build's parameter table `defaults`."""
    with tempfile.TemporaryDirectory() as d:
        path = str(Path(d) / "rate_t3_inputs.txt")
        oracle.refresh_inputs(str(defaults), path)
        p = oracle.read_inputs(path)
    su = oracle.Setup(p)
    out = {}
    for a, axis in enumerate(AXES):
        law = (p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_kd_{axis}"], p[f"rate_d_filter_tau_{axis}"],
               p[f"rate_tau_ref_{axis}"], (su.divisor, su.lowpass, su.lowpass_error))
        inertia, tau = p[oracle.INERTIA[axis]], p["motor_tau"]
        sps, stamps = run_l4.acro_setpoints(plan, a)
        args = (*law, su.tick_map, inertia, tau, p["inertia_robustness_band"], p["tau_robustness_band"], sps, stamps)
        lo_c, hi_c = run_l4.script_envelope(*args, oracle.GRID)
        lo, hi = run_l4.script_envelope(*args, HALVED)
        halving = max(max(abs(x - y) for x, y in zip(lo_c, lo)), max(abs(x - y) for x, y in zip(hi_c, hi)))
        peak = max(max(abs(x) for x in lo), max(abs(x) for x in hi))
        _, rho = run_l4.script_response(*law, su.tick_map(inertia, tau), sps, stamps, want_rho=True)
        n = su.executions(axis)
        l1 = {node: max(oracle.l1_norm(su, axis, node, k, n) for k in PHASES) for node in oracle.NODES}
        tol = sum(l1[node] * rho[node] for node in oracle.NODES)
        out[axis] = {"P": peak, "P / rate_max": peak / p[f"rate_max_{axis}"], "TOL": tol, "H": halving,
                     "F": tol + halving, "lo": lo, "hi": hi, "Z": run_l4.rest_bound(lo, hi)}
    return out


def run_params(s):
    """The parameters a run flew: the build's table with the run's sil_overrides (float32 / i32 values)."""
    params = run_l4.read_param_defaults(s.params_path)
    for name, (kind, text) in s.overrides.items():
        params[name] = run_l4.r32(float(text)) if kind == run_l4.F32 else int(text)
    return params


def fly(tmp_path_factory, name, overrides=None):
    out = tmp_path_factory.mktemp(name)
    runs = {m: run_l4.run_acro(CARD, SCENARIO, m, out, PLUGIN_DIR, overrides=overrides) for m in (2, 1)}
    tool = run_l4.replay_tool(PLUGIN_DIR)
    replays = {m: run_l4.replay_acro(s, tool, out) for m, s in runs.items()}
    ev = evaluate(runs, replays, run_l4.mixer_abs_bm(run_params(runs[1])))
    path = str(out / f"{name}_{run_l4.NAME_LABEL}_sequence.report.txt")
    run_l4.write_t4_report([runs[2], runs[1]], "t4 acro", ev, path)
    return Acro(runs, replays, ev, path, out)


@pytest.fixture(scope="module")
def acro(tmp_path_factory):
    return fly(tmp_path_factory, "acro")


@pytest.fixture(scope="module")
def loop_disabled(tmp_path_factory):
    over = {f"rate_{q}_{axis}": (run_l4.F32, "0.0") for axis in AXES for q in ("kp", "ki")}
    return fly(tmp_path_factory, "acro_loop_disabled", over)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@needs_gz
def test_acro_meets_the_predicate(acro, capsys):
    ev = acro.evaluation
    sat = ev["(v) combined segment (m = 1 replay)"]
    lines = [f"acro {'PASS' if ev['passed'] else 'FAIL'}  DShot {ev['(ii) DShot range seen']} in "
             f"{ev['(ii) DShot bounds']}, stale {ev['stale reads']}, executions at a DShot end "
             f"{ev['executions with a motor at a DShot end (m = 1)']}, gz wall {ev['gz wall (s)']:.2f} s",
             f"    segments {ev['segments (number, kind, first execution, last execution, phase)']}"]
    for name, e in ev["(iii) behaviour (B, T) per linear segment"].items():
        lines.append(f"    (iii) {name}: B " + ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}"
                                                         for c in e["B"])
                     + (f"; T {e['T']['axis']} {'ok' if e['T']['ok'] else 'FAIL'} |q| {e['T']['first |q|']:.4f} -> "
                        f"{e['T']['last |q|']:.4f}" if e["T"] else ""))
    lines.append(f"    (iv) replay first mismatch {ev['(iv) replay fidelity (first mismatch)']}")
    lines.append(f"    (v) combined executions {sat.get('combined executions')}, per axis {sat.get('per axis')}, flag "
                 f"findings {sat.get('flag findings')}, s or t not 1 {sat.get('executions with s or t not 1')}")
    _say(capsys, *lines, f"    report: {acro.report_path}")
    for s in acro.runs.values():
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
        assert s.harness_overrides == {}
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
    assert ev["passed"], ev


@needs_gz
def test_acro_replay_reproduces_the_run(acro, capsys):
    fidelity = {m: run_l4.replay_fidelity(s, acro.replays[m]) for m, s in acro.runs.items()}
    _say(capsys, "acro replay: " + ", ".join(f"m = {m}: {len(acro.replays[m])} executions, first mismatch {f}"
                                             for m, f in fidelity.items()))
    assert all(f is None for f in fidelity.values()), fidelity
    assert all(len(acro.replays[m]) == len(s.executions) for m, s in acro.runs.items())


@needs_gz
@pytest.mark.xfail(condition=not XFAIL_GATE.exists(), reason=COUPLING_KNOWN_FAILING, strict=True, raises=AssertionError)
def test_acro_rejects_cross_axis_coupling(acro, capsys):
    ev = acro.evaluation
    rec = ev["recovery (known failing, decisions 0005 and 0029)"]
    lines = [f"acro cross axes, {name}: B " + ", ".join(
        f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.6f} <= {c.bound:.6f}" for c in e["B cross"])
        for name, e in ev["(iii) behaviour (B, T) per linear segment"].items() if e["B cross"]]
    lines.append(f"acro recovery window {rec['window']}: B " + ", ".join(
        f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.6f} <= {c.bound:.6f}" for c in rec["B"]))
    _say(capsys, *lines)
    assert ev["cross axes passed (known failing, decision 0030)"] and ev["recovery passed"], (
        [(n, e["B cross"]) for n, e in ev["(iii) behaviour (B, T) per linear segment"].items()], rec)


@needs_gz
def test_acro_script_is_the_scenario_on_the_live_parameters(acro):
    s = acro.runs[1]
    p = s.plan
    assert len(p.segments) <= p.capacity
    for i, (k, t_us, sp, stick) in enumerate(p.segments, start=1):
        assert s.overrides[f"l4_seg{i}_t_us"] == (run_l4.I32, str(t_us))
        assert p.stamp_us(k - 1) < t_us == p.stamp_us(k)
        assert sp == tuple(run_l4.r32(x * r) for x, r in zip(stick, p.rate_max))
    assert s.overrides["l4_seg_count"] == (run_l4.I32, str(len(p.segments)))
    assert s.overrides["l4_thrust_n"] == (run_l4.F32, repr(p.thrust_n))


# ---- negative controls ----------------------------------------------------------------------------------------------

@needs_gz
def test_control_loop_disabled_fails_the_behaviour_predicates(loop_disabled, capsys):
    ev = loop_disabled.evaluation
    lines = [f"acro control kp = ki = 0 on every axis: (iii) {'PASS' if ev['(iii) passed'] else 'FAIL'} (must fail)"]
    for name, e in ev["(iii) behaviour (B, T) per linear segment"].items():
        lines.append(f"    {name}: B " + ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'}" for c in e["B"])
                     + (f"; T {e['T']['axis']} {'ok' if e['T']['ok'] else 'FAIL'} |q| {e['T']['first |q|']:.6f} -> "
                        f"{e['T']['last |q|']:.6f}" if e["T"] else ""))
    _say(capsys, *lines)
    for s in loop_disabled.runs.values():
        assert set(s.harness_overrides) == {f"rate_{q}_{axis}" for axis in AXES for q in ("kp", "ki")}
        assert all(v == (run_l4.F32, "0.0") for v in s.harness_overrides.values())
        for name in s.harness_overrides:
            assert f'<sil_override param="{name}" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
    assert not clean_findings(loop_disabled.runs[1]) and not clean_findings(loop_disabled.runs[2]), "must fail on (iii) only"
    assert not any(ev["stale reads"].values()), "the control must fail on the behaviour, not on a stale read"
    assert not ev["(iii) passed"] and not ev["passed"], ev


@needs_gz
def test_control_dshot_check_is_not_vacuous(acro):
    lo, hi = acro.runs[1].dshot_bounds
    rows = [list(row) for row in acro.runs[1].dshot]
    assert dshot_findings(rows, lo, hi) == []
    rows[len(rows) // 2][0] = lo - 1
    assert dshot_findings(rows, lo, hi) == [(len(rows) // 2, 1, lo - 1)]


@needs_gz
def test_control_flag_and_saturation_checks_are_not_vacuous(acro):
    p = acro.runs[1].plan
    phases, _ = run_l4.acro_phases(p)
    abs_bm = run_l4.mixer_abs_bm(run_params(acro.runs[1]))
    rows = [dict(r) for r in acro.replays[1]]
    assert saturation_evaluation(rows, phases, abs_bm)["passed"]
    k = next(r["k"] for r in rows if phases[r["k"]] == "combined" and r["req_pitch"] != 0.0)
    base = dict(rows[k])

    def planted(**change):
        out = [dict(r) for r in rows]
        out[k] = {**base, **change}
        return saturation_evaluation(out, phases, abs_bm)

    # a flag set with achieved equal to requested
    ev = planted(flag_pitch=1)
    assert ev["flag findings"] == 1 and not ev["passed"], ev
    # a reduction far beyond b_a (half the request) with the flag clear, s kept at 1
    ev = planted(ach_pitch=base["req_pitch"] / 2)
    assert ev["flag findings"] == 1 and not ev["passed"], ev
    # a consistent flagged reduction: flag consistency holds, s < 1 fails
    half = {"ach_roll": base["req_roll"] / 2, "ach_pitch": base["req_pitch"] / 2, "flag_roll": int(base["req_roll"] != 0),
            "flag_pitch": 1, "s": 0.5}
    ev = planted(**half)
    assert ev["flag findings"] == 0 and ev["executions with s or t not 1"] == 1 and not ev["passed"], ev
