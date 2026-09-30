"""T4 angle-mode steps of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4; decision 0006
F "T4 angle-mode steps" and "Tolerance terms at T4"; decision 0005 "T4 envelope widening").

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/step_<axis>.yaml (roll, pitch), one gz process per m of its m_sequence [2, 1]
(tools/sim/run_l5.py run_sequence). Both m divide the attitude divisor, so every attitude sample is fresh.

Envelope. The RECORDED T3 envelope, tests/regression/quad/L05/t3/reference/attitude_t3_envelope.txt (the box envelope of
the design model over the 17 x 17 tau x J grid, driven by the exact script from level and at rest, never re-seeded): the
channel `theta` of the script, one lo / hi pair per attitude execution n = 0 .. 2 H, n = 0 the seed execution (sticks 0)
and n the run's attitude execution a0 + n (tools/sim/run_l5.py plan). The recorded live-parameter fixture
(attitude_t3_inputs.txt) must equal the build's parameter table, else the envelope does not belong to the build.

Quantities, from the TRUTH record of each attitude execution (the float cast of the body state the firmware read, decision
0006 B). q is canonical (w >= 0); its rotation vector v = 2 atan2(|q_v|, w) q_v / |q_v| has the tilt components v_x (roll)
and v_y (pitch), equal for the script's pure single-axis rotation to the T3 oracle's unwrapped angle theta; the heading
relative to the lock is psi(n) - psi(0), psi = atan2(2 (w z + x y), 1 - 2 (y^2 + z^2)), wrapped (the heading lock is taken at
the seed execution, where the sticks are 0). The stepped axis's component is compared with the envelope; the other tilt
component and the heading relative to the lock have the envelope 0 (a pure single-axis script).

Predicate (decision 0006 F; owner decision 19). E_c(n) = |y_c,1(n) - y_c,2(n)| (m = 1 against m = 2), F the recorded T3
tolerance term of the script's header line (T3 tolerance + the envelope's last-halving change + the kinematics halving
change), Q_c the recorded DShot quantisation term of the script's `theta` channel (tests/regression/quad/L05/t3/reference/
attitude_t3_q.txt, rule in t3/README.md "The quantisation term Q") for the stepped component and 0 for the other two (the
quantised design model is single-axis: it carries no off-axis torque of the rounding, so it gives no Q for them). PASS iff
every run is clean (complete trailer, a TRUTH record for every tick, no stale host step in the window) and every DShot of
every tick lies within [idle, 2047], and at every attitude execution n = 0 .. 2 H and for each of the three components c:
    lo_c(n) - (E_c(n) + F + Q_c) <= y_c,1(n) <= hi_c(n) + (E_c(n) + F + Q_c).
Altitude drift is allowed (0005 decision 9; the world has no ground plane).

Negative controls, harness only (0003 item 9). Each passes only if the check fails:
  (a) att_kp = 0 through sil_override: the metric control of core 7.2; the predicate fails, with no stale read;
  (b) alignment: the exact alignment check with the origin execution one earlier or later fails (the segment stamps lie
      outside the allowed interval). With F about the size of one attitude execution's change of theta at its steepest, the
      predicate alone does not detect a one-execution shift; the exact check does, as at L4;
  (c) att_kp x 1.1 through sil_override (f32(att_kp f32(1.1)), as t3_test.cpp scales it): the spec's fine gain control,
      which fails the predicate widened by Q (owner decision 19; measured in tests/regression/quad/L05/results/
      step_controls/). The fine delay control does not fail it and is enforced at T3 only (same record).
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
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
AXES = ("roll", "pitch")
GAIN_SCALE = 1.1  # scenario test value: control (c), the spec's fine gain control (t3_test.cpp kGainScale)
AXIS_INDEX = {"roll": 0, "pitch": 1}
SHIFTS = (-1, 1)
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
COMPONENTS = ("roll", "pitch", "heading")  # tilt x, tilt y, heading relative to the lock


# ---- the recorded envelope ------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Envelope:
    scenario: str
    first: int
    count: int
    lo: list
    hi: list
    F: float  # T3 tolerance + last-halving change + kinematics halving change (the header's settle line)
    halving: float
    Q: float  # the DShot quantisation term of the script's theta channel (attitude_t3_q.txt)


def parse_envelope(path, scenario):
    """The `theta` channel of `scenario` from the recorded envelope file, and the header's F of that script."""
    text = Path(path).read_text(encoding="utf-8").splitlines()
    f = re.compile(rf"^#\s+{scenario} theta: end \S+ < F (\S+)$")
    F = next(float(m.group(1)) for m in map(f.match, text) if m)
    start = text.index(f"scenario {scenario}")
    i = next(k for k in range(start, len(text)) if text[k].startswith("channel theta "))
    m = re.match(r"channel theta first (\d+) count (\d+) halving_max_change (\S+)$", text[i])
    first, count, halving = int(m.group(1)), int(m.group(2)), float(m.group(3))
    assert text[i + 1] == "min_max"
    rows = [tuple(map(float, ln.split())) for ln in text[i + 2:i + 2 + count]]
    assert len(rows) == count and all(len(r) == 2 for r in rows)
    return Envelope(scenario, first, count, [r[0] for r in rows], [r[1] for r in rows], F, halving,
                    parse_q(T3_REFERENCE / "attitude_t3_q.txt", scenario, "theta"))


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
    for line in (T3_REFERENCE / "attitude_t3_inputs.txt").read_text().splitlines():
        line = line.split("#")[0].split()
        if line:
            out[line[0]] = int(line[1]) if line[1].lstrip("-").isdigit() else float.fromhex(line[1])
    return out


# ---- quantities -----------------------------------------------------------------------------------------------------

def rotation_vector(q):
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    s = 2.0 * math.atan2(n, w) / n
    return (x * s, y * s, z * s)


def heading(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def series(s, n_exec):
    """Per oracle execution n = 0 .. n_exec - 1: (tilt x, tilt y, heading relative to the lock) from the TRUTH record."""
    p = s.plan
    out, psi0 = [], None
    for n in range(n_exec):
        t = s.executions[p.run_execution(n)]["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {p.run_execution(n)}"
        v, psi = rotation_vector(t["q_wxyz"]), heading(t["q_wxyz"])
        psi0 = psi if psi0 is None else psi0
        out.append((v[0], v[1], wrap(psi - psi0)))
    return out


def evaluate(runs, env, axis, shift=0):
    """The predicate of the module docstring on a [m=2, m=1] sequence; `shift` compares y(n) with the envelope at n + shift
    (reported only, the alignment control is exact)."""
    by_m = {s.run.m: s for s in runs}
    y1, y2 = series(by_m[1], env.count), series(by_m[2], env.count)
    k = AXIS_INDEX[axis]
    worst = {c: None for c in COMPONENTS}
    stats = {c: {"max_out": -math.inf, "max_E": 0.0, "violations": 0} for c in COMPONENTS}
    for n in range(env.count):
        j = n + shift
        if not 0 <= j < env.count:
            continue
        for ci, c in enumerate(COMPONENTS):
            lo, hi = (env.lo[j], env.hi[j]) if ci == k else (0.0, 0.0)
            y = y1[n][ci]
            e = abs(y - y2[n][ci])
            out = max(lo - y, y - hi)
            slack = out - (e + env.F + (env.Q if ci == k else 0.0))
            st = stats[c]
            st["violations"] += slack > 0
            st["max_out"], st["max_E"] = max(st["max_out"], out), max(st["max_E"], e)
            if worst[c] is None or slack > worst[c]["slack"]:
                worst[c] = {"n": n, "slack": slack, "outside": out, "E": e, "y": y, "lo": lo, "hi": hi}
    violations = sum(st["violations"] for st in stats.values())
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {"passed": violations == 0 and not any(stale.values()), "shift": shift, "violations": violations,
            "stale_reads_in_window": stale, "F": env.F, "Q": env.Q, "halving": env.halving, "N": env.count,
            "components": stats,
            "worst": worst}


def alignment_findings(s, origin):
    """Why the mapping n -> attitude execution `origin` + n does not put oracle execution n on the run's execution; empty
    when it does (module docstring)."""
    ex, p = s.executions, s.plan
    out = []
    for i, (start, _, _) in enumerate(p.segments, 1):
        seg = int(s.overrides[f"l5_seg{i}_t_us"][1])
        a = origin + start
        if not (ex[a - 1]["t_us"] < seg <= ex[a]["t_us"]):
            out.append(f"segment {i} stamp {seg} is not in (t({a - 1}), t({a})] = ({ex[a - 1]['t_us']}, {ex[a]['t_us']}]")
    if origin + p.end >= len(ex):
        out.append(f"the window's last execution {origin + p.end} is beyond the run's {len(ex) - 1}")
    want = int(p.period_s * 10 ** 6)
    bad = [n for n in range(1, min(p.end, len(ex) - origin - 1) + 1)
           if ex[origin + n]["t_us"] - ex[origin + n - 1]["t_us"] != want]
    if bad:
        out.append(f"{len(bad)} window dt differ from the oracle's {want} us (first at n = {bad[0]})")
    if any(x["tick"] != p.tick_of(x["a"]) for x in ex):
        out.append("an execution's logged tick is not D_a a")
    return out


def dshot_changes(s):
    ticks = s.run.log["ticks"]
    return [t["tick"] for a, t in zip(ticks, ticks[1:]) if a["dshot"] != t["dshot"]]


def verdict_line(axis, label, ev, wall):
    c = ev["components"]
    parts = "; ".join(f"{k} max outside {c[k]['max_out']:+.6e} max E {c[k]['max_E']:.3e} viol {c[k]['violations']}"
                      for k in COMPONENTS)
    w = ev["worst"][axis]
    return (f"{axis:<5} {label:<14} {'PASS' if ev['passed'] else 'FAIL'}  F {ev['F']:.6e} (halving {ev['halving']:.6e}), "
            f"Q {ev['Q']:.6e}, "
            f"{parts}; worst {axis} n {w['n']} (outside {w['outside']:+.3e}, E {w['E']:.3e}, slack {w['slack']:+.6e}), "
            f"violations {ev['violations']}, stale {ev['stale_reads_in_window']}, N {ev['N']}, gz wall {wall:.2f} s")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Sequence:
    runs: list
    report_path: str
    wall_s: float
    evaluation: dict


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


@pytest.fixture(scope="module")
def envelope(axis):
    return parse_envelope(T3_REFERENCE / "attitude_t3_envelope.txt", f"step_{axis}")


def fly(tmp_path_factory, name, axis, env, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"step_{axis}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, env, axis)
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def step(tmp_path_factory, axis, envelope):
    return fly(tmp_path_factory, f"step_{axis}", axis, envelope)


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, axis, envelope):
    return fly(tmp_path_factory, f"step_{axis}_kp_zero", axis, envelope, {"att_kp": (run_l5.F32, "0.0")})


@pytest.fixture(scope="module")
def att_gain_up(tmp_path_factory, axis, envelope, live_params):
    k = run_l5.l4.r32(live_params["att_kp"] * run_l5.l4.r32(GAIN_SCALE))
    return fly(tmp_path_factory, f"step_{axis}_att_gain_up", axis, envelope, {"att_kp": (run_l5.F32, repr(k))})


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_step_meets_the_angle_mode_envelope(step, envelope, axis, live_params, capsys):
    ev = step.evaluation
    _say(capsys, verdict_line(axis, "step", ev, step.wall_s), f"    report: {step.report_path}",
         *[f"    m = {s.run.m}: run stale report {s.run.stale}; window host steps {s.window_steps.start}.."
           f"{s.window_steps.stop - 1}, stale in window {len(s.window_stale)}" for s in step.runs])
    idle, top = run_l5.l4.dshot_idle_bound(live_params)
    assert [s.run.m for s in step.runs] == [2, 1]
    for s in step.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert len(s.run.log["truths"]) == len(s.run.log["ticks"]), "a tick without a TRUTH record"
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
        lo, hi = min(min(t["dshot"]) for t in s.run.log["ticks"]), max(max(t["dshot"]) for t in s.run.log["ticks"])
        assert idle <= lo and hi <= top, f"DShot [{lo}, {hi}] outside [{idle}, {top}]"
    assert ev["passed"], ev


def test_step_runs_on_the_recorded_parameters_and_both_truth_sources(step, envelope, axis, live_params):
    recorded = recorded_inputs()
    assert set(recorded) - set(live_params) == {"tau_held_yaw_nm"}, "the recorded fixture has keys the build lacks"
    for key, want in recorded.items():
        if key in live_params:
            assert live_params[key] == want, f"{key}: build {live_params[key]!r} differs from the recorded {want!r}"
    assert envelope.first == 0 and envelope.count == step.runs[0].plan.end + 1
    for s in step.runs:
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        assert s.plan.att_divisor % s.run.m == 0
        for t in s.run.log["ticks"][:: s.plan.att_divisor]:
            assert struct_flags(t["imu"]) == 1 << GYRO_VALID_BIT, "the sample is not the truth gyro"


def struct_flags(imu):
    return run_l5.l4.IMU.unpack(imu)[7]


def test_alignment_is_exact(step, axis, capsys):
    for s in step.runs:
        assert alignment_findings(s, s.plan.origin) == []
        changes = dshot_changes(s)
        assert changes, "the DShot command never changed: the check below would be vacuous"
        assert all(t % s.plan.divisor == 0 for t in changes), "a DShot change off a rate-loop tick"
    p = step.runs[0].plan
    _say(capsys, f"alignment {axis}: origin a0 = {p.origin}, segment stamps {[x[1] for x in p.segments]} us, "
                 f"{len(dshot_changes(step.runs[1]))} DShot changes (m = 1), all on rate-loop ticks")


# ---- negative controls ----------------------------------------------------------------------------------------------

def test_control_att_kp_zero_fails_the_predicate(kp_zero, axis, capsys):
    ev = kp_zero.evaluation
    _say(capsys, verdict_line(axis, "control kp = 0", ev, kp_zero.wall_s))
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ev["violations"] > 0, ev


def test_control_att_kp_times_1_1_fails_the_predicate(att_gain_up, axis, live_params, capsys):
    ev = att_gain_up.evaluation
    _say(capsys, verdict_line(axis, "control kp x1.1", ev, att_gain_up.wall_s))
    k = run_l5.l4.r32(live_params["att_kp"] * run_l5.l4.r32(GAIN_SCALE))
    assert k != live_params["att_kp"]
    for s in att_gain_up.runs:
        assert s.harness_overrides == {"att_kp": ("f32", repr(k))}
        assert f'<sil_override param="att_kp" type="f32">{k!r}</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ev["violations"] > 0, ev


@pytest.mark.parametrize("off", SHIFTS)
def test_control_alignment_off_by_one_execution_fails(step, off):
    for s in step.runs:
        assert alignment_findings(s, s.plan.origin + off) != []
