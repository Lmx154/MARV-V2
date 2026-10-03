"""T4 large-angle recovery of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4; decision 0006 F
"T4 large-angle recovery", "T3 envelopes" and "Tolerance terms at T4"; owner decisions 5, 15 and 19; decision 0005 owner decision 12).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/recover_inverted.yaml (R1: a roll of pi - delta from level, at rest, delta = 0.01 rad, owner decision 22:
far above the rounding noise in (w, z) of the plant's first step, so the model and gz take the same branch of the tilt-yaw split) and
recover_tumble.yaml (R2: inverted, body rates (rate_max_roll, rate_max_pitch, rate_max_yaw)), R1 with the rotors at the card's hover
speed (owner decision 23; rotor_speed_rad_s: hover), R2 at its steady tumble (decision 0015; rotor_speed_rad_s: steady_tumble), one gz
process per m of [2, 1] (tools/sim/run_l5.py run_sequence). The setpoint is
level, locked at angle mode's initial heading; no sticks. The run starts at the scenario's initial state (settle_s is one attitude
period): the firmware's attitude execution n is the design model's execution n.

Envelope. The 3-axis design model of the MODEL section below (the T3 oracle's pieces; no w x Jw): box envelope over the 17 x 17
tau x J grid of the model driven from the exact initial attitude and rates, never re-seeded, for six channels per attitude
execution: the body-frame rotation vector of the attitude error q_e = canonical(conj(q) (x) q_sp) (err_x, err_y the tilt error,
err_z the heading error) and the body rates (w_x, w_y, w_z). The design model's motor state starts at 0, the deviation from the trim
of the initial rotor speeds (R2: the steady tumble, decision 0015). The recorded T3 envelope file has no recovery script (t3/ is not this
packet's to extend), so the envelope is computed here for the live parameters, which must equal the recorded fixture
attitude_t3_inputs.txt (as the step test requires).

Rate origin (harness fact, 0003 item 11; tests/regression/quad/L05/results/recovery_cause/README.md finding 1). The plugin reads the
body state and runs the SIL for host step 0 before it sets the scenario's initial rates, so the TRUTH record and the gyro of execution
0 read omega = 0 while the body moves from its initial rates. The rate channels (w_x, w_y, w_z) are therefore not compared at
execution 0; the attitude channels at execution 0 are. This is the plugin's documented timing, not a tolerance term.

Side check (owner decision 22). recover_inverted_exact.yaml starts at exactly [0, 1, 0, 0]: there the tilt axis is chosen by rounding
noise, so only the tilt angle alpha (the angle between body z and NED down, axis-invariant) is checked, against the design model's
alpha envelope over the same box from exactly 180 degrees, with the same E + F + Q rule (E, F, Q of the alpha channel, same
construction).

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

R1 must pass. R2's envelope test is a strict xfail (raises=AssertionError) while tests/regression/quad/L06/XFAIL_GATE_CLOSED does not exist (decision 0009), exactly as
the L4 acro recovery (docs/handoff.md, known failing item): its cause is the gyroscopic coupling (no L3 flag, s = t = 1, |w x Jw| 33-44 %
of tau_held; tests/regression/quad/L05/results/recovery_cause/). It extends the L4 item: blocks L6, must pass before L8.
The side check's envelope predicate is a strict xfail on the same gate (decision 0014, owner decisions third round, item 3): its
cause is the same coupling on the branch gz's first step picks (tests/regression/quad/L06/results/r1x_coupling/); it must pass at L6
stage (e) with the feed-forward live. The side check's clean run, DShot range and settled end value stay normal tests.
Helper module: recovery_model.py, next to this file (the cause script imports it too).

Negative controls, harness only (0003 item 9), each passes only if the check fails:
  (a) a planted trace held at the initial attitude and rates for the whole run;
  (b) att_kp = 0 through sil_override (the metric control of core 7.2), with no stale read;
  (c) att_kp x 1.1 (f32(att_kp f32(1.1)), as the step test), R1 only: the fine gain control.
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
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L05/t3")  # the generated golden, envelope and q (decision 0011)
XFAIL_GATE = ROOT / "tests" / "regression" / "quad" / "L06" / "XFAIL_GATE_CLOSED"  # created by L6 stage (e), decision 0009
R1, R1X, R2 = "recover_inverted", "recover_inverted_exact", "recover_tumble"
GAIN_SCALE = 1.1  # scenario test value: the spec's fine gain control (t3_test.cpp kGainScale)
ALL = [R1, R2]
RECOVERY_KNOWN_FAILING = (
    "known failing item (decision 0006 F, owner decision 15; extends the L4 acro item, decision 0005 owner decision 12, docs/handoff.md): "
    "the R2 recovery leaves the design-model envelope by the gyroscopic coupling w x Jw (no L3 flag, s = t = 1; cause in "
    "tests/regression/quad/L05/results/recovery_cause/cause.txt). Blocks L6 (a normal test once tests/regression/quad/L06/XFAIL_GATE_CLOSED exists, decision 0009); "
    "must pass before L8."
)
EXACT_KNOWN_FAILING = (
    "known failing item (decision 0014, owner decisions third round, item 3; docs/handoff.md): the exact-180 degree recovery's tilt "
    "angle leaves the design model's alpha envelope by the gyroscopic coupling w x Jw on the branch of the tilt-yaw split that gz's "
    "first-step rounding picks; the design model plus w x Jw reproduces gz and its excess under the stage (c) and the old gains "
    "(tests/regression/quad/L06/results/r1x_coupling/coupling.txt). Only the envelope predicate; a normal test once "
    "tests/regression/quad/L06/XFAIL_GATE_CLOSED exists (decision 0009); must pass at L6 stage (e) with the feed-forward live."
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
    text = (T3_GENERATED / "attitude_t3_envelope.txt").read_text(encoding="utf-8").splitlines()
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


_DESIGNS = {}


def make_design(name, live_params, fn=None, label=None):
    """The design envelope of scenario `name` (cached per (name, channel set)): six channels, or `fn`'s (the alpha side check)."""
    key = (name, label)
    if key in _DESIGNS:
        return _DESIGNS[key]
    recorded = recorded_inputs()
    for k, want in recorded.items():
        if k in live_params:
            assert live_params[k] == want, f"{k}: build {live_params[k]!r} differs from the recorded {want!r}"
    su = rm.oracle.Setup(recorded)
    vals = l5s.values(l5s.load(SCEN / f"{name}.yaml"))
    st = vals["initial_state"]
    count = vals["script"]["end_attitude_execution"] + 1
    env = rm.envelope(su, rm.oracle.read_q_inputs(T3_REFERENCE / "attitude_t3_q_inputs.txt"), st["attitude_q_wxyz"],
                      st["body_rates_frd_rad_s"], count, fn=fn)
    terms = recorded_t3_terms()
    n = len(env["lo"])
    F = [(terms["angle"] if (fn is not None or c < 3) else terms["rate"]) + env["halving"][c] + env["kin"][c] for c in range(n)]
    _DESIGNS[key] = Design(name, count, env["lo"], env["hi"], F, env["q"], env["halving"], env["kin"], env["saturated"],
                           rm.settle_ok(env, F))
    return _DESIGNS[key]


@pytest.fixture(scope="module")
def design(name, live_params):
    return make_design(name, live_params)


# ---- quantities and predicate ---------------------------------------------------------------------------------------------

def series(s, n_exec):
    """Per attitude execution n = 0 .. n_exec - 1 (the run's execution n) the six channels of the TRUTH record."""
    out = []
    for n in range(n_exec):
        t = s.executions[n]["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {n}"
        out.append(rm.channels(t["q_wxyz"], t["omega_frd"]))
    return out


def evaluate(runs, d, y1=None, y2=None, chs=CH, ser=None):
    """The predicate of the module docstring on a [m = 2, m = 1] sequence; `y1` replaces the m = 1 series (the planted trace)."""
    by_m = {s.run.m: s for s in runs}
    ser = ser or series
    y1 = ser(by_m[1], d.count) if y1 is None else y1
    y2 = ser(by_m[2], d.count) if y2 is None else y2
    stats = {c: {"max_out": -math.inf, "max_E": 0.0, "violations": 0, "first": None, "worst": None} for c in chs}
    for n in range(d.count):
        for ci, c in enumerate(chs):
            if n == 0 and c.startswith("w_"):
                continue  # rate origin (module docstring): the plugin's execution-0 gyro is 0
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
    parts = "; ".join(f"{c} out {st['max_out']:+.4e} E {st['max_E']:.2e} slack(out-(E+F+Q)) {st['worst']['slack']:+.3e} viol {st['violations']}"
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


def alpha_series(s, n_exec):
    """Per attitude execution the tilt angle alpha (one channel) of the TRUTH record."""
    return [rm.alpha_channel(s.executions[n]["truth"]["q_wxyz"], None) for n in range(n_exec)]


def fly(tmp_path_factory, label, name, d, overrides=None, **kw):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"{name}.yaml", tmp_path_factory.mktemp(f"{name}_{label}"), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, d, **kw)
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
    exact = l5s.values(l5s.load(SCEN / f"{R1X}.yaml"))["initial_state"]
    assert exact["attitude_q_wxyz"] == [0.0, 1.0, 0.0, 0.0] and exact["rotor_speed_rad_s"] == "hover"
    assert st["rotor_speed_rad_s"] == ("steady_tumble" if name == R2 else "hover")
    if name == R1:
        w, x = st["attitude_q_wxyz"][:2]
        assert st["attitude_q_wxyz"][2:] == [0.0, 0.0] and abs(w * w + x * x - 1.0) < 1e-15
        assert abs(2 * math.atan2(x, w) - (math.pi - 2 * math.asin(w))) < 1e-15 and w > 1e-9  # delta far above the ~1e-16 noise
    else:
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
        request.applymarker(pytest.mark.xfail(condition=not XFAIL_GATE.exists(), reason=RECOVERY_KNOWN_FAILING, strict=True,
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


@pytest.fixture(scope="module")
def r1_design(live_params):
    return make_design(R1, live_params)


@pytest.fixture(scope="module")
def att_gain_up(tmp_path_factory, r1_design, live_params):
    k = run_l5.l4.r32(live_params["att_kp"] * run_l5.l4.r32(GAIN_SCALE))
    assert k != live_params["att_kp"]
    return fly(tmp_path_factory, "att_gain_up", R1, r1_design, {"att_kp": (run_l5.F32, repr(k))})


def test_control_att_kp_times_1_1_fails_r1(att_gain_up, live_params, capsys):
    ev = att_gain_up.evaluation
    _say(capsys, verdict_line(R1, "control kp*1.1", ev, att_gain_up.wall_s))
    k = run_l5.l4.r32(live_params["att_kp"] * run_l5.l4.r32(GAIN_SCALE))
    for s in att_gain_up.runs:
        assert s.harness_overrides == {"att_kp": ("f32", repr(k))}
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ev["violations"] > 0, ev["components"]


# ---- the exact-180 degree side check (owner decision 22) ---------------------------------------------------------------------

@pytest.fixture(scope="module")
def exact_design(live_params):
    return make_design(R1X, live_params, fn=rm.alpha_channel, label="alpha")


@pytest.fixture(scope="module")
def exact(tmp_path_factory, exact_design):
    return fly(tmp_path_factory, "exact", R1X, exact_design, chs=("alpha",), ser=alpha_series)


def test_exact_180_design_alpha_envelope_has_settled(exact_design, capsys):
    _say(capsys, f"{R1X}: alpha envelope N {exact_design.count}, Q {exact_design.Q}, F {exact_design.F}, halving {exact_design.halving}, "
                 f"kin {exact_design.kin}")
    assert all(exact_design.end_ok)


def test_exact_180_run_is_clean_on_truth_and_in_the_dshot_range(exact, live_params):
    test_run_is_clean_on_truth_and_in_the_dshot_range(exact, R1X, live_params)


def test_exact_180_tilt_angle_is_inside_the_alpha_envelope(exact, exact_design, request, capsys):
    ev = exact.evaluation
    _say(capsys, verdict_line(R1X, "alpha", ev, exact.wall_s), f"    report: {exact.report_path}")
    request.applymarker(pytest.mark.xfail(condition=not XFAIL_GATE.exists(), reason=EXACT_KNOWN_FAILING, strict=True,
                                          raises=AssertionError))
    assert ev["passed"], ev["components"]


def test_control_exact_180_alpha_planted_at_the_initial_angle_fails(exact, exact_design):
    planted = [(math.pi,)] * exact_design.count
    ev = evaluate(exact.runs, exact_design, y1=planted, y2=planted, chs=("alpha",), ser=alpha_series)
    assert not ev["passed"] and ev["violations"] > 0, ev["components"]
