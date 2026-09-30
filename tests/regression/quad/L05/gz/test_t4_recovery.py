"""T4 large-angle recovery of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4; decision 0006 F
"T4 large-angle recovery", "T3 envelopes" and "Tolerance terms at T4"; owner decisions 5, 15 and 19; decision 0005 owner decision 12).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/recover_inverted.yaml (R1: inverted, at rest) and recover_tumble.yaml (R2: inverted, body rates
(rate_max_roll, rate_max_pitch, rate_max_yaw)), one gz process per m of [2, 1] (tools/sim/run_l5.py run_sequence). The setpoint is
level, locked at angle mode's initial heading; no sticks. The run starts at the scenario's initial state (settle_s is one attitude
period): the firmware's attitude execution n is the design model's execution n.

Envelope. The 3-axis design model of the MODEL section below (the T3 oracle's pieces; no w x Jw): box envelope over the 17 x 17
tau x J grid of the model driven from the exact initial attitude and rates, never re-seeded, for six channels per attitude
execution: the body-frame rotation vector of the attitude error q_e = canonical(conj(q) (x) q_sp) (err_x, err_y the tilt error,
err_z the heading error) and the body rates (w_x, w_y, w_z). The recorded T3 envelope file has no recovery script (t3/ is not this
packet's to extend), so the envelope is computed here for the live parameters, which must equal the recorded fixture
attitude_t3_inputs.txt (as the step test requires).

Predicate (decision 0006 F; owner decision 19). From the TRUTH record of each attitude execution n = 0 .. end (the float cast of the
body state the firmware read): y_c(n) = the six channels of (q, omega), E_c(n) = |y_c,m=1(n) - y_c,m=2(n)|, Q_c the quantisation
term of the channel (the oracle's Quantiser on the full 3-vector: largest |quantised - unquantised| over time at the corners and the
9 x 9 grid), F_c = T3 + H_c + K_c with H_c the envelope's last-halving change and K_c the kinematics halving change (the run against
the run with every tick in two sub-steps, at the nominal and the four corners); T3 stands for the first-order rounding tolerance
of the T3 oracle, which has no recovery script: the largest F - halving of the recorded angle channels (theta, heading_lock) and of
the recorded rate channels (omega) of attitude_t3_envelope.txt, INFERRED to bound the float32 rounding of a recovery (unverified).
PASS iff every run is clean (complete trailer, a TRUTH record for every tick, no stale host step in the window, every DShot of every
tick within [idle, 2047]) and at every execution and channel
    lo_c(n) - (E_c(n) + F_c + Q_c) <= y_c,1(n) <= hi_c(n) + (E_c(n) + F_c + Q_c).
The T3 property is asserted too: every envelope end value is below its F_c at the scenario's last execution.

R1 must pass. R2's envelope test is a strict xfail (raises=AssertionError) while tests/regression/quad/L06/ does not exist, exactly as
the L4 acro recovery (docs/handoff.md, known failing item), ONLY if its cause is the gyroscopic coupling: it blocks L6 and must pass
before L8. The cause file is tests/regression/quad/L05/results/recovery_cause/.

Negative controls, harness only (0003 item 9), each passes only if the check fails:
  (a) a planted trace held at the initial attitude and rates for the whole run;
  (b) att_kp = 0 through sil_override (the metric control of core 7.2), with no stale read.
"""

import dataclasses
import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import l5_scenario as l5s  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
L06 = ROOT / "tests" / "regression" / "quad" / "L06"
R1, R2 = "recover_inverted", "recover_tumble"
ALL = [R1, R2]
RECOVERY_KNOWN_FAILING = (
    "known failing item (decision 0006 F, owner decision 15; decision 0005 owner decision 12): the R2 recovery leaves the design-model "
    "envelope; cause in tests/regression/quad/L05/results/recovery_cause/. It blocks L6 (this becomes a normal test when "
    "tests/regression/quad/L06/ exists) and must pass before L8."
)
CH = rm.CHANNELS


# ---- the recorded T3 tolerance terms and the design ------------------------------------------------------------------------

def recorded_inputs():
    out = {}
    for line in (T3_REFERENCE / "attitude_t3_inputs.txt").read_text().splitlines():
        line = line.split("#")[0].split()
        if line:
            out[line[0]] = int(line[1]) if line[1].lstrip("-").isdigit() else float.fromhex(line[1])
    return out


def recorded_t3_terms():
    """{"angle": max of F - halving over the recorded theta and heading_lock channels, "rate": over the omega channels} of the
    recorded envelope file's settle lines (F = T3 tolerance + halving + kinematics of the script)."""
    text = (T3_REFERENCE / "attitude_t3_envelope.txt").read_text(encoding="utf-8").splitlines()
    settle = re.compile(r"^#\s+(\w+) (\w+): end \S+ < F (\S+)$")
    halving, script, terms = {}, None, {"angle": 0.0, "rate": 0.0}
    for ln in text:
        if ln.startswith("scenario "):
            script = ln.split()[1]
        m = re.match(r"^channel (\w+) first \d+ count \d+ halving_max_change (\S+)$", ln)
        if m:
            halving[(script, m.group(1))] = float(m.group(2))
    for ln in text:
        m = settle.match(ln)
        if m:
            key = "rate" if m.group(2) == "omega" else "angle"
            terms[key] = max(terms[key], float(m.group(3)) - halving[(m.group(1), m.group(2))])
    assert terms["angle"] > 0 and terms["rate"] > 0
    return terms


@dataclasses.dataclass(frozen=True)
class Design:
    name: str
    count: int
    lo: list
    hi: list
    F: list
    Q: list
    halving: list
    kin: list
    saturated: int
    end_ok: list


@pytest.fixture(scope="module", params=ALL)
def name(request):
    return request.param


@pytest.fixture(scope="module")
def design(name, live_params):
    recorded = recorded_inputs()
    for key, want in recorded.items():
        if key in live_params:
            assert live_params[key] == want, f"{key}: build {live_params[key]!r} differs from the recorded {want!r}"
    su = rm.oracle.Setup(recorded)
    vals = l5s.values(l5s.load(SCEN / f"{name}.yaml"))
    st = vals["initial_state"]
    count = vals["script"]["end_attitude_execution"] + 1
    env = rm.envelope(su, rm.oracle.read_q_inputs(T3_REFERENCE / "attitude_t3_q_inputs.txt"), st["attitude_q_wxyz"],
                      st["body_rates_frd_rad_s"], count)
    terms = recorded_t3_terms()
    F = [(terms["angle"] if c < 3 else terms["rate"]) + env["halving"][c] + env["kin"][c] for c in range(len(CH))]
    return Design(name, count, env["lo"], env["hi"], F, env["q"], env["halving"], env["kin"], env["saturated"],
                  rm.settle_ok(env, F))


# ---- quantities and predicate ---------------------------------------------------------------------------------------------

def series(s, n_exec, reflect=False):
    """Per attitude execution n = 0 .. n_exec - 1 (the run's execution n) the six channels of the TRUTH record. `reflect`
    (R1 only) negates the roll channels err_x and w_x when the recovery went the other way round the roll axis: R1 starts exactly
    on the singular set (error pi about +x or -x: the same rotation), so which shortest rotation the law takes is decided by the
    sign of the rounding noise of the plant's first step, and the single-axis design model is symmetric under x -> -x (the
    other four channels are 0 in the model). The direction is the sign of err_x at execution 1."""
    out = []
    for n in range(n_exec):
        t = s.executions[n]["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {n}"
        out.append(rm.channels(t["q_wxyz"], t["omega_frd"]))
    if reflect and out[1][0] < 0:
        out = out[:1] + [(-a, b, c, -d, e, f) for a, b, c, d, e, f in out[1:]]
    return out


def evaluate(runs, d, y1=None, y2=None):
    """The predicate of the module docstring on a [m = 2, m = 1] sequence; `y1` replaces the m = 1 series (the planted trace)."""
    by_m = {s.run.m: s for s in runs}
    reflect = d.name == R1
    y1 = series(by_m[1], d.count, reflect) if y1 is None else y1
    y2 = series(by_m[2], d.count, reflect) if y2 is None else y2
    stats = {c: {"max_out": -math.inf, "max_E": 0.0, "violations": 0, "first": None, "worst": None} for c in CH}
    for n in range(d.count):
        for ci, c in enumerate(CH):
            y = y1[n][ci]
            e = abs(y - y2[n][ci])
            out = max(d.lo[ci][n] - y, y - d.hi[ci][n])
            slack = out - (e + d.F[ci] + d.Q[ci])
            st = stats[c]
            st["violations"] += slack > 0
            if slack > 0 and st["first"] is None:
                st["first"] = n
            st["max_out"], st["max_E"] = max(st["max_out"], out), max(st["max_E"], e)
            if st["worst"] is None or slack > st["worst"]["slack"]:
                st["worst"] = {"n": n, "slack": slack, "outside": out, "E": e, "y": y, "lo": d.lo[ci][n], "hi": d.hi[ci][n]}
    violations = sum(st["violations"] for st in stats.values())
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {"passed": violations == 0 and not any(stale.values()), "violations": violations,
            "stale_reads_in_window": stale, "F": d.F, "Q": d.Q, "halving": d.halving, "kin": d.kin, "N": d.count,
            "design_saturated_executions": d.saturated, "components": stats}


def verdict_line(name, label, ev, wall):
    parts = "; ".join(f"{c} out {st['max_out']:+.4e} E {st['max_E']:.2e} viol {st['violations']}"
                      + (f" first n {st['first']}" if st["first"] is not None else "")
                      for c, st in ev["components"].items())
    worst = max(ev["components"].values(), key=lambda st: st["worst"]["slack"])["worst"]
    return (f"{name} {label:<12} {'PASS' if ev['passed'] else 'FAIL'}  {parts}; worst slack {worst['slack']:+.4e} at n {worst['n']}, "
            f"violations {ev['violations']}, stale {ev['stale_reads_in_window']}, N {ev['N']}, gz wall {wall:.2f} s")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


@dataclasses.dataclass
class Sequence:
    runs: list
    report_path: str
    wall_s: float
    evaluation: dict


def fly(tmp_path_factory, label, name, d, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"{name}.yaml", tmp_path_factory.mktemp(f"{name}_{label}"), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, d)
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def recovery(tmp_path_factory, name, design):
    return fly(tmp_path_factory, "recovery", name, design)


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, name, design):
    return fly(tmp_path_factory, "kp_zero", name, design, {"att_kp": (run_l5.F32, "0.0")})


# ---- the pass bar ---------------------------------------------------------------------------------------------------------

def test_design_envelope_has_settled(design, name, capsys):
    _say(capsys, f"{name}: design envelope N {design.count}, Q {['%.3e' % q for q in design.Q]}, F {['%.3e' % f for f in design.F]}, "
                 f"halving {['%.3e' % h for h in design.halving]}, kin {['%.2e' % k for k in design.kin]}, quantised-model "
                 f"saturated member executions {design.saturated}")
    assert all(design.end_ok), f"{name}: an envelope end value is not below F (T3 property): {design.end_ok}"


def test_scenario_initial_state_is_the_labelled_test_value(name, live_params):
    st = l5s.values(l5s.load(SCEN / f"{name}.yaml"))["initial_state"]
    assert st["attitude_q_wxyz"] == [0.0, 1.0, 0.0, 0.0]
    rates = [live_params["rate_max_roll"], live_params["rate_max_pitch"], live_params["rate_max_yaw"]]
    assert st["body_rates_frd_rad_s"] == ([0.0, 0.0, 0.0] if name == R1 else rates)


def test_run_is_clean_on_truth_and_in_the_dshot_range(recovery, name, live_params):
    idle, top = run_l5.l4.dshot_idle_bound(live_params)
    assert [s.run.m for s in recovery.runs] == [2, 1]
    for s in recovery.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert len(s.run.log["truths"]) == len(s.run.log["ticks"]), "a tick without a TRUTH record"
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        lo, hi = min(min(t["dshot"]) for t in s.run.log["ticks"]), max(max(t["dshot"]) for t in s.run.log["ticks"])
        assert idle <= lo and hi <= top, f"DShot [{lo}, {hi}] outside [{idle}, {top}]"


def test_recovery_meets_the_design_envelope(recovery, design, name, request, capsys):
    ev = recovery.evaluation
    _say(capsys, verdict_line(name, "recovery", ev, recovery.wall_s), f"    report: {recovery.report_path}")
    if name == R2:
        request.applymarker(pytest.mark.xfail(condition=not L06.exists(), reason=RECOVERY_KNOWN_FAILING, strict=True,
                                              raises=AssertionError))
    assert ev["passed"], ev["components"]


# ---- negative controls ----------------------------------------------------------------------------------------------------

def test_control_planted_trace_held_at_the_initial_state_fails(recovery, design, name):
    first = series(recovery.runs[1], 1)[0]
    planted = [first] * design.count
    ev = evaluate(recovery.runs, design, y1=planted, y2=planted)
    assert not ev["passed"] and ev["violations"] > 0, ev["components"]


def test_control_att_kp_zero_fails_the_predicate(kp_zero, name, capsys):
    ev = kp_zero.evaluation
    _say(capsys, verdict_line(name, "control kp=0", ev, kp_zero.wall_s))
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ev["violations"] > 0, ev["components"]
