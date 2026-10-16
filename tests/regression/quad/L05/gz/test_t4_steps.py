"""T4 angle-mode steps of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4: "in Gazebo
... settle by the deadline (S) and stay bounded (B) on roll, pitch and heading", predicates S and B of decision 0029,
loop-disabled control; decision 0006 F "T4 angle-mode steps", decision 0005 "T4 envelope widening"; decision 0031).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/step_<axis>.yaml (roll, pitch), one gz process per m of its m_sequence [2, 1]
(tools/sim/run_l5.py run_sequence). Both m divide the attitude divisor, so every attitude sample is fresh.

Quantities, from the TRUTH record of each attitude execution (the float cast of the body state the firmware read, decision
0006 B). q is canonical (w >= 0); its rotation vector v = 2 atan2(|q_v|, w) q_v / |q_v| has the tilt components v_x (roll)
and v_y (pitch); the heading relative to the lock is psi(n) - psi(0), psi = atan2(2 (w z + x y), 1 - 2 (y^2 + z^2)), wrapped
(the heading lock is taken at the seed execution, where the sticks are 0). The reference of the tilt component of the
stepped axis is the segment's stick times angle_tilt_max (the angle-mode setpoint of a single full stick, decision 0006 C),
read from the build's parameter table; the other tilt component and the heading relative to the lock have reference 0.

Predicates (decision 0029, tools/sim/t4_rule.py, on the m = 1 run). The two segments of the script are the step, executions
1 .. a_r - 1, and the release, a_r .. end (oracle executions; the deadline of each is the scenario's horizon, ten attitude
time constants 1/att_kp doubled once, H executions). Error q = y - q_ref on the three components (roll, pitch, heading; all
rad, so one Delta per segment). s is the budget's t4_settle_fraction. E_c = |y_c,1 - y_c,2| (decision 0003 E).
  S  at the segment's last execution |error| <= s Delta + E, Delta the largest |error| in the segment over the three
     components, E that of the last execution;
  B  at every execution |error(n)| <= |error(n_0)| + s Delta + E, E the largest over the segment (the predicate takes one
     E per quantity).
PASS iff every run is clean (complete trailer, a TRUTH record for every tick, no stale host step in the window) and every
DShot of every tick lies within [idle, 2047], and S and B hold on every component of both segments. Altitude drift is
allowed (0005 decision 9; the world has no ground plane).

Roles (decision 0031): the stepped axis's tilt component is STEPPED (the stick, then the offset left at the release); the other tilt
component and the heading are HELD, with a STEPPED quantity of the same unit in the segment, so S and B are as above.

Negative control, harness only (0003 item 9). The test passes only if the conjunction fails: att_kp = 0 through
sil_override, no stale read. This is the metric test's negative control of core 7.2: the loop disabled (decision 0031).
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
AXES = ("roll", "pitch")
AXIS_INDEX = {"roll": 0, "pitch": 1}
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
COMPONENTS = ("roll", "pitch", "heading")  # tilt x, tilt y, heading relative to the lock
UNIT = "rad"
SEGMENTS = ("step", "release")


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


def segment_ranges(plan):
    """name -> (first, last) oracle executions: the step (the first stick segment) and the release (the second)."""
    (a_s, _, _), (a_r, _, _) = plan.segments
    return {"step": (a_s, a_r - 1), "release": (a_r, plan.end)}


def reference(plan, name, tilt_max):
    """The reference of the three components in segment `name`: the stick times angle_tilt_max on the tilt axes, 0 on heading."""
    stick = plan.segments[SEGMENTS.index(name)][2]
    return (stick[0] * tilt_max, stick[1] * tilt_max, 0.0)


def evaluate(runs, tilt_max):
    """S and B (module docstring) on a [m=2, m=1] sequence: the checks per segment and component, and the verdict."""
    by_m = {s.run.m: s for s in runs}
    p = by_m[1].plan
    y1, y2 = series(by_m[1], p.end + 1), series(by_m[2], p.end + 1)
    out = {"S": {}, "B": {}}
    for name, (first, last) in segment_ranges(p).items():
        ns = range(first, last + 1)
        ref = reference(p, name, tilt_max)
        segment = {c: (UNIT, [y1[n][k] - ref[k] for n in ns]) for k, c in enumerate(COMPONENTS)}
        diff = {c: [abs(y1[n][k] - y2[n][k]) for n in ns] for k, c in enumerate(COMPONENTS)}
        out["S"][name] = t4_rule.settles(segment, SETTLE_FRACTION, {c: diff[c][-1] for c in COMPONENTS})
        out["B"][name] = t4_rule.bounded(segment, SETTLE_FRACTION, {c: max(diff[c]) for c in COMPONENTS})
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    checks = [c for part in ("S", "B") for cs in out[part].values() for c in cs]
    return dict(out, passed=all(c.ok for c in checks) and not any(stale.values()), stale_reads_in_window=stale, N=p.end + 1,
                s=SETTLE_FRACTION, tilt_max=tilt_max)


def verdict_line(axis, label, ev, wall):
    def part(checks):
        return ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}" for c in checks)
    seg = "; ".join(f"{n}: S [{part(ev['S'][n])}] B [{part(ev['B'][n])}]" for n in SEGMENTS)
    return (f"{axis:<5} {label:<14} {'PASS' if ev['passed'] else 'FAIL'}  {seg}; stale {ev['stale_reads_in_window']}, "
            f"N {ev['N']}, gz wall {wall:.2f} s")


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


def fly(tmp_path_factory, name, axis, tilt_max, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"step_{axis}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, tilt_max)
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def step(tmp_path_factory, axis, live_params):
    return fly(tmp_path_factory, f"step_{axis}", axis, live_params["angle_tilt_max"])


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, axis, live_params):
    return fly(tmp_path_factory, f"step_{axis}_kp_zero", axis, live_params["angle_tilt_max"],
               {"att_kp": (run_l5.F32, "0.0")})


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_step_settles_and_stays_bounded(step, axis, live_params, capsys):
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


def test_step_runs_on_the_recorded_parameters_and_both_truth_sources(step, axis, live_params):
    recorded = recorded_inputs()
    assert set(recorded) - set(live_params) == {"tau_held_yaw_nm"}, "the recorded fixture has keys the build lacks"
    for key, want in recorded.items():
        if key in live_params:
            assert live_params[key] == want, f"{key}: build {live_params[key]!r} differs from the recorded {want!r}"
    for s in step.runs:
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        assert s.plan.att_divisor % s.run.m == 0
        for t in s.run.log["ticks"][:: s.plan.att_divisor]:
            assert struct_flags(t["imu"]) == 1 << GYRO_VALID_BIT, "the sample is not the truth gyro"


def struct_flags(imu):
    return run_l5.l4.IMU.unpack(imu)[7]


# ---- negative control -----------------------------------------------------------------------------------------------

def test_control_att_kp_zero_fails_the_predicates(kp_zero, axis, capsys):
    ev = kp_zero.evaluation
    _say(capsys, verdict_line(axis, "control kp = 0", ev, kp_zero.wall_s))
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the predicates, not on a stale read"
    assert not ev["passed"], ev
