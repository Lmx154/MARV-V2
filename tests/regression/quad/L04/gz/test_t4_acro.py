"""T4 acro of the L4 rate loop in Gazebo on truth gyro (quad spec 4 L4 pass bar, T4: "acro with a scripted stick sequence
completes without saturation beyond the mixer's documented behaviour"; decision 0005 "T4" acro predicate, owner decision
9; decision 0004 items 3 and 5; Luis's Option 1 and recovery ruling, 2026-09-30).

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

Predicate. PASS iff
  (i)   each run ends cleanly: gz accepted the run (run_scenario.check_exit), no hal_panic, and the trailer counts equal
        the run's (every host step, tick and applied record logged);
  (ii)  every DShot command of every tick of both runs lies in [ceil(D(omega_idle)), kDshotThrottleMax], the idle bound
        computed from the build's parameters as thrust_to_dshot computes it (run_l4.dshot_idle_bound, decision 0004);
  (iii) no stale read in the evaluation window of either run (0003 item 11, as the step test), and
        |w_a(n)| <= P_a + F_a + E_a(n) at every linear execution n whose host-step read is fresh in both runs, w_a the
        gyro sample the firmware received (m = 1), E_a(n) = |w_a,m=1(n) - w_a,m=2(n)| (decision 0003 E):
          P_a  the largest |w_a| over every execution and over the 17 x 17 tau x J grid of the band box (the T3 oracle's
               band_envelope grid, corners included) of the LINEAR design model driven by acro.yaml's exact setpoint
               sequence at the run's stamps: run_l4.script_response, the oracle's closed_loop law and plant map (the
               oracle's Setup.plant_map, the build's parameters through the oracle's inputs-refresh path) with a
               setpoint per execution (run_l4.acro_setpoints, the composition's setpoint_at);
          F_a  = TOL_a + H_a as in the step test: TOL_a = sum over the nodes r, e, I, u of l1 x rho, l1 the oracle's
               impulse-response norms (rate_t3_oracle.l1_norm, both dt phases) and rho the oracle's first-order float32
               rounding injections along the nominal script trajectory; H_a the envelope's last-halving change, the
               largest change of lo(n), hi(n) between the 9 x 9 and the 17 x 17 grids;
  (iv)  the replay reproduces both runs: per execution the log's tick, stamp and DShot, bit for bit
        (run_l4.replay_fidelity; its own test too);
  (v)   over the combined segment, from the m = 1 replay: flag consistency (run_l4.flag_findings: a set flag has
        |achieved| < |requested|, and |requested| - |achieved| > b_a, the decision 0005 anti-windup bound, has the flag
        set), and s = t = 1 at every execution (nan only where the requests it divides are exactly zero): no saturation
        is a checked fact of the run, not an assumption.
Recovery, its own test, a known failing item (strict xfail until tests/regression/quad/L06 exists; decision 0005):
  no stale read in the evaluation window, and |w_a(n)| <= Z_a(n) + F_a + E_a(n) at every fresh execution n of the
  recovery window, Z_a(n) the largest |w_a(n)| over
  the same 17 x 17 grid of the same script response from rest (run_l4.rest_bound). A vehicle at rest passes; the model
  is not re-seeded from the measured state. The run fails it: the per-axis PI cannot reject the gyroscopic coupling of
  the combined segment, and the roll integrator releases what it absorbed as an uncommanded roll with the stick centred
  (tests/regression/quad/L04/results/acro_cause/cause.txt).
Saturation during transients would be expected (air mode, owner decision 9); altitude is not checked.

Negative controls, harness only (0003 item 9). Each test passes only if the check fails:
  (a) the first segment's roll setpoint overridden to 2 rate_max_roll through sil_override: (iii) fails;
  (b) (ii): no harness-side control exists. thrust_to_dshot clamps every command to [ceil(D(omega_idle)),
      kDshotThrottleMax] with the parameters in effect, and the build's idle speed is the ESC map's minimum, so the bound
      is kDshotThrottleMin itself; no parameter override moves a command below it without moving the bound. The check is
      instead shown not to be vacuous on the run's own data: one command set to the bound minus one must fail it;
  (c) (iv): the m = 1 run replayed with l4_thrust_n one float32 ulp higher must not reproduce the run;
  (d) (v): planted replay rows, one per rule (a flag set with achieved equal to requested, a reduction beyond b_a with the
      flag clear, s < 1 with the flag consistent), each must fail its check;
  (e) recovery: the window planted at rest must pass and planted stuck at rate_max on every axis must fail; the real run
      failing it is the strict xfail itself.
"""

import dataclasses
import math
import struct
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

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L04" / "acro.yaml"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
AXES = ("roll", "pitch", "yaw")
HALVED = 2 * (oracle.GRID - 1) + 1  # the oracle's halved grid, 17 x 17, as the step test
PHASES = (2, 3)  # the oracle's two dt phases of an injection (its main()), as the step test
CONTROL_RATE_FACTOR = 2  # the control's first-segment roll setpoint, in rate_max_roll
L06 = ROOT / "tests" / "regression" / "quad" / "L06"
RECOVERY_KNOWN_FAILING = (
    "decision 0005, known failing: the PI baseline's bandwidth (crossover 8.3 rad/s) cannot reject the gyroscopic "
    "coupling of the combined full-stick segment; the roll integrator absorbs it and then releases it, an uncommanded "
    "roll with the stick centred (tests/regression/quad/L04/results/acro_cause/cause.txt). It must pass before L6 passes "
    "and before L8.")

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


def summary(b):
    return {k: b[k] for k in ("P", "P / rate_max", "TOL", "H", "F")}


def rate_bound_evaluation(runs, bound, phases, table):
    """(iii) on a {1: run, 2: run} pair over the linear executions: per axis the largest |w|, the bound at that
    execution, the smallest slack overall and per linear segment, and the violations."""
    s1, s2 = runs[1], runs[2]
    p = s1.plan
    out = {}
    for a, axis in enumerate(AXES):
        base = bound[axis]["P"] + bound[axis]["F"]
        peak = {"w": 0.0, "k": None, "bound": base, "E": 0.0}
        violations, closest, checked, per_segment = [], None, 0, {}
        for x1, x2 in zip(s1.executions[:p.end_execution + 1], s2.executions[:p.end_execution + 1]):
            if phases[x1["k"]] != "linear" or not (x1["fresh"] and x2["fresh"]):
                continue
            checked += 1
            w, e = abs(x1["gyro"][a]), abs(x1["gyro"][a] - x2["gyro"][a])
            slack = base + e - w
            if w > peak["w"]:
                peak = {"w": w, "k": x1["k"], "bound": base + e, "E": e}
            if closest is None or slack < closest["slack"]:
                closest = {"k": x1["k"], "w": w, "bound": base + e, "slack": slack}
            for number, kind, first, last, _ in table:
                if first <= x1["k"] <= last:
                    key = f"segment {number} ({kind})"
                    per_segment[key] = min(per_segment.get(key, math.inf), slack)
            if slack < 0:
                violations.append(x1["k"])
        out[axis] = {"rate_max": p.rate_max[a], "design bound": summary(bound[axis]), "bound without E": base,
                     "max |w|": peak["w"], "at execution": peak["k"], "bound there": peak["bound"],
                     "max |w| / rate_max": peak["w"] / p.rate_max[a], "smallest slack": closest,
                     "smallest slack per linear segment": per_segment, "violations": len(violations),
                     "first violation": violations[0] if violations else None, "fresh executions checked": checked}
    return out


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


def recovery_evaluation(ex1, ex2, bound, phases):
    """Recovery per axis from the executions of the m = 1 and m = 2 runs (lists of execution dicts)."""
    window = [k for k, ph in enumerate(phases) if ph == "recovery"]
    fresh = [a["fresh"] and b["fresh"] for a, b in zip(ex1, ex2)]
    out = {}
    for a, axis in enumerate(AXES):
        w = [x["gyro"][a] for x in ex1]
        e = [abs(x["gyro"][a] - y["gyro"][a]) for x, y in zip(ex1, ex2)]
        out[axis] = run_l4.recovery_evaluation(window, w, e, bound[axis]["Z"], bound[axis]["F"], fresh)
    return out


def evaluate(runs, bound, replays, abs_bm):
    lo, hi = runs[1].dshot_bounds
    p = runs[1].plan
    phases, table = run_l4.acro_phases(p)
    clean = [f for s in runs.values() for f in clean_findings(s)]
    bad = {m: dshot_findings(s.dshot, lo, hi) for m, s in runs.items()}
    rates = rate_bound_evaluation(runs, bound, phases, table)
    fidelity = {f"m{m}": run_l4.replay_fidelity(s, replays[m]) for m, s in runs.items()}
    sat = saturation_evaluation(replays[1], phases, abs_bm) if fidelity["m1"] is None else {
        "passed": False, "reason": "the m = 1 replay does not reproduce the run"}
    recovery = recovery_evaluation(runs[1].executions, runs[2].executions, bound, phases)
    stale = {f"m{m}": len(s.window_stale) for m, s in runs.items()}
    commands = [v for s in runs.values() for row in s.dshot for v in row]
    ends = sum(1 for k in range(p.end_execution + 1) if any(v in (lo, hi) for v in runs[1].dshot[p.tick_of(k)]))
    return {
        "passed": (not clean and not any(bad.values()) and not any(r["violations"] for r in rates.values())
                   and not any(stale.values()) and all(f is None for f in fidelity.values()) and sat["passed"]),
        "segments (number, kind, first execution, last execution, phase)": [list(t) for t in table],
        "(i) clean": clean or "yes", "(ii) DShot bounds": [lo, hi],
        "(ii) DShot range seen": [min(commands), max(commands)],
        "(ii) out of range": {f"m{m}": len(b) for m, b in bad.items()}, "(iii) per axis (linear executions)": rates,
        "(iv) replay fidelity (first mismatch)": {k: v or "none" for k, v in fidelity.items()},
        "(v) combined segment (m = 1 replay)": sat,
        "recovery (known failing, decision 0005)": recovery,
        "recovery passed": all(r["passed"] for r in recovery.values()) and not any(stale.values()),
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
        kp, ki, tau_ref = p[f"rate_kp_{axis}"], p[f"rate_ki_{axis}"], p[f"rate_tau_ref_{axis}"]
        inertia, tau = p[oracle.INERTIA[axis]], p["motor_tau"]
        sps, stamps = run_l4.acro_setpoints(plan, a)
        args = (kp, ki, tau_ref, su.plant_map, inertia, tau, p["inertia_robustness_band"], p["tau_robustness_band"],
                sps, stamps)
        lo_c, hi_c = run_l4.script_envelope(*args, oracle.GRID)
        lo, hi = run_l4.script_envelope(*args, HALVED)
        halving = max(max(abs(x - y) for x, y in zip(lo_c, lo)), max(abs(x - y) for x, y in zip(hi_c, hi)))
        peak = max(max(abs(x) for x in lo), max(abs(x) for x in hi))
        _, rho = run_l4.script_response(kp, ki, tau_ref, su.plant_map(inertia, tau), sps, stamps, want_rho=True)
        n = su.executions(axis)
        l1 = {node: max(oracle.l1_norm(su, axis, node, k, n) for k in PHASES) for node in oracle.NODES}
        tol = sum(l1[node] * rho[node] for node in oracle.NODES)
        out[axis] = {"P": peak, "P / rate_max": peak / p[f"rate_max_{axis}"], "TOL": tol, "H": halving,
                     "F": tol + halving, "lo": lo, "hi": hi, "Z": run_l4.rest_bound(lo, hi)}
    return out


@pytest.fixture(scope="module")
def bound():
    _, defaults = run_l4.build_parameters(PLUGIN_DIR)
    plan = run_l4.plan_acro(run_l4.l4s.load_acro(SCENARIO), run_l4.read_param_defaults(defaults), CARD)
    return design_bound(plan, defaults)


def run_params(s):
    """The parameters a run flew: the build's table with the run's sil_overrides (float32 / i32 values)."""
    params = run_l4.read_param_defaults(s.params_path)
    for name, (kind, text) in s.overrides.items():
        params[name] = run_l4.r32(float(text)) if kind == run_l4.F32 else int(text)
    return params


def fly(tmp_path_factory, name, bound, overrides=None):
    out = tmp_path_factory.mktemp(name)
    runs = {m: run_l4.run_acro(CARD, SCENARIO, m, out, PLUGIN_DIR, overrides=overrides) for m in (2, 1)}
    tool = run_l4.replay_tool(PLUGIN_DIR)
    replays = {m: run_l4.replay_acro(s, tool, out) for m, s in runs.items()}
    ev = evaluate(runs, bound, replays, run_l4.mixer_abs_bm(run_params(runs[1])))
    path = str(out / f"{name}_{run_l4.NAME_LABEL}_sequence.report.txt")
    run_l4.write_t4_report([runs[2], runs[1]], "t4 acro", ev, path)
    return Acro(runs, replays, ev, path, out)


@pytest.fixture(scope="module")
def acro(tmp_path_factory, bound):
    return fly(tmp_path_factory, "acro", bound)


@pytest.fixture(scope="module")
def double_rate(tmp_path_factory, bound):
    rate_max = run_l4.read_param_defaults(run_l4.build_parameters(PLUGIN_DIR)[1])["rate_max_roll"]
    over = {"l4_seg1_roll": (run_l4.F32, repr(run_l4.r32(CONTROL_RATE_FACTOR * rate_max)))}
    return fly(tmp_path_factory, "acro_double_rate", bound, over)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@needs_gz
def test_acro_meets_the_predicate(acro, capsys):
    ev = acro.evaluation
    sat = ev["(v) combined segment (m = 1 replay)"]
    lines = [f"acro {'PASS' if ev['passed'] else 'FAIL'}  DShot {ev['(ii) DShot range seen']} in "
             f"{ev['(ii) DShot bounds']}, stale {ev['stale reads']}, executions at a DShot end "
             f"{ev['executions with a motor at a DShot end (m = 1)']}, gz wall {ev['gz wall (s)']:.2f} s",
             f"    segments {ev['segments (number, kind, first execution, last execution, phase)']}"]
    for axis, r in ev["(iii) per axis (linear executions)"].items():
        lines.append(f"    (iii) {axis:<5} max |w| {r['max |w|']:.6f} rad/s ({r['max |w| / rate_max']:.4f} rate_max) at "
                     f"execution {r['at execution']}, bound there {r['bound there']:.6f} (P "
                     f"{r['design bound']['P']:.6f}, F {r['design bound']['F']:.3e}, + E), smallest slack "
                     f"{r['smallest slack']['slack']:.6f} rad/s at {r['smallest slack']['k']}, violations "
                     f"{r['violations']}; per segment "
                     + ", ".join(f"{k} {v:.6f}" for k, v in r["smallest slack per linear segment"].items()))
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
@pytest.mark.xfail(condition=not L06.exists(), reason=RECOVERY_KNOWN_FAILING, strict=True, raises=AssertionError)
def test_acro_recovers_after_the_combined_segment(acro, capsys):
    rec = acro.evaluation["recovery (known failing, decision 0005)"]
    p = acro.runs[1].plan
    lines = []
    for axis, r in rec.items():
        lines.append(f"acro recovery {axis:<5} {'PASS' if r['passed'] else 'FAIL'}  worst margin "
                     f"{r['worst margin']:.6f} rad/s at execution {r['at execution']} "
                     f"(t = {p.stamp_us(r['at execution']) / 1e6:.6f} s, |w| {abs(r['w there']):.6f}, Z + F + E "
                     f"{r['Z + F + E there']:.6f}), violations {r['violations']} ({r['first violation']}.."
                     f"{r['last violation']}) of {r['fresh executions checked']}")
    _say(capsys, *lines)
    assert acro.evaluation["recovery passed"], rec


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
def test_control_double_rate_segment_fails_the_rate_bound(double_rate, capsys):
    ev = double_rate.evaluation
    r = ev["(iii) per axis (linear executions)"]["roll"]
    _say(capsys, f"acro control l4_seg1_roll = {CONTROL_RATE_FACTOR} rate_max: roll max |w| {r['max |w|']:.6f} rad/s, "
                 f"bound {r['bound there']:.6f}, violations {r['violations']} "
                 f"({'PASS' if ev['passed'] else 'FAIL'}, must fail)")
    for s in double_rate.runs.values():
        assert list(s.harness_overrides) == ["l4_seg1_roll"]
        assert f'<sil_override param="l4_seg1_roll" type="f32">{s.harness_overrides["l4_seg1_roll"][1]}</sil_override>' \
            in Path(s.run.world_path).read_text()
    assert not clean_findings(double_rate.runs[1]) and not clean_findings(double_rate.runs[2]), "must fail on (iii) only"
    assert not any(ev["stale reads"].values()), "the control must fail on the rate bound, not on a stale read"
    assert r["violations"] > 0 and not ev["passed"], ev


@needs_gz
def test_control_dshot_check_is_not_vacuous(acro):
    lo, hi = acro.runs[1].dshot_bounds
    rows = [list(row) for row in acro.runs[1].dshot]
    assert dshot_findings(rows, lo, hi) == []
    rows[len(rows) // 2][0] = lo - 1
    assert dshot_findings(rows, lo, hi) == [(len(rows) // 2, 1, lo - 1)]


@needs_gz
def test_control_replay_of_a_one_ulp_thrust_change_does_not_reproduce_the_run(acro, capsys):
    s = acro.runs[1]
    bits = struct.unpack("<I", struct.pack("<f", s.plan.thrust_n))[0]
    up = struct.unpack("<f", struct.pack("<I", bits + 1))[0]  # the next float32 above l4_thrust_n
    rows = run_l4.replay_acro(s, run_l4.replay_tool(PLUGIN_DIR), acro.out_dir,
                              overrides={"l4_thrust_n": (run_l4.F32, repr(up))}, tag="_thrust_ulp_control")
    mismatch = run_l4.replay_fidelity(s, rows)
    _say(capsys, f"acro replay control l4_thrust_n {s.plan.thrust_n!r} -> {up!r}: first mismatch {mismatch} (must exist)")
    assert mismatch is not None


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


@needs_gz
def test_control_recovery_check_passes_rest_and_fails_a_stuck_rate(acro, bound, capsys):
    s1, s2 = acro.runs[1], acro.runs[2]
    phases, _ = run_l4.acro_phases(s1.plan)
    window = [k for k, ph in enumerate(phases) if ph == "recovery"]

    def planted(gyro):
        ex1 = [dict(x) for x in s1.executions]
        ex2 = [dict(x) for x in s2.executions]
        for k in window:
            ex1[k]["gyro"] = ex2[k]["gyro"] = gyro
        return recovery_evaluation(ex1, ex2, bound, phases)

    rest = planted((0.0, 0.0, 0.0))
    stuck = planted(tuple(s1.plan.rate_max))
    _say(capsys, "acro recovery controls: rest " + ", ".join(f"{a} {r['passed']} (worst {r['worst margin']:.6f})"
                                                              for a, r in rest.items())
         + "; stuck at rate_max " + ", ".join(f"{a} {r['passed']} (worst {r['worst margin']:.6f})"
                                              for a, r in stuck.items()))
    assert all(r["passed"] for r in rest.values()), rest
    assert not any(r["passed"] for r in stuck.values()), stuck
