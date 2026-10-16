"""T4 per-axis steps of the L4 rate loop in Gazebo on truth gyro (quad spec 4 L4 pass bar, T4: "in Gazebo, on truth
gyro, per-axis steps settle and stay bounded", predicates S and B of decision 0029, loop-disabled control; decision 0005
"QF-2", "T4", owner decisions 7 and 9; decision 0030).

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l4 build of libmarv_gz_lockstep.so is
absent (`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`); the gz CI step fails on a skip. Every run is
truth-fed, perfect-model (tools/sim/run_l4.py): not a validation run.

Runs. scenarios/quad/L04/step_<axis>.yaml, one gz process per m of its m_sequence [2, 1] (tools/sim/run_l4.py
run_sequence). y_m(n) is the gyro sample the firmware received at oracle execution n (the logged TICK IMU bytes are the
sample passed to the SIL, decision 0005 "Truth gyro"), at execution k = k0 + n - 2 of the run (run_l4 plan: k0 is the
step's first execution, n = 1 the last one before it).

Predicates (decision 0029, tools/sim/t4_rule.py, on the m = 1 run). The segment is the step's executions k0 .. k0 + N - 2
(oracle n = 2..N, the setpoint segment from its stamp to the horizon of ten reference time constants, the scenario's
horizon_tau_ref), on all three gyro axes, error q = y - q_ref with q_ref the stepped axis's step rate and 0 on the other
two. s is the budget's t4_settle_fraction. E_a = |y_1,a - y_2,a| (decision 0003 E).
  S  the error at the segment's last execution is within s Delta + E of zero, Delta the largest error in the segment over
     the three axes (all rad/s), E that of the last execution;
  B  at every execution |error(n)| <= |error(n_0)| + s Delta + E, E the largest over the segment (the predicate takes one
     E per quantity).
PASS iff no host step read in the window is stale (0003 item 11: the raw gz read bitwise equal to the previous step's) and
S and B hold on every axis. Altitude drift is allowed (Luis, owner decision 9) and not checked; marv_plant v0 has no drag
and no position-dependent torque.

Negative control, harness only (0003 item 9). The test passes only if the check fails: kp = ki = 0 on the stepped axis
through sil_override, no stale read. This is the metric test's negative control of core 7.2: the loop disabled.
"""

import dataclasses
import sys
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
SCEN = ROOT / "scenarios" / "quad" / "L04"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
AXES = ("roll", "pitch", "yaw")
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
UNIT = "rad/s"

needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l4 build of libmarv_gz_lockstep.so is absent")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the predicates -------------------------------------------------------------------------------------------------

def segment_gyro(s):
    """Per gyro axis the m run's samples over the step segment, executions k0 .. k0 + N - 2."""
    p = s.plan
    ks = range(p.execution_of(2), p.execution_of(p.n_exec) + 1)
    return [[s.executions[k]["gyro"][a] for k in ks] for a in range(len(AXES))]


def evaluate(runs):
    """S and B (module docstring) on a [m=2, m=1] sequence: the checks per axis and the verdict."""
    by_m = {s.run.m: s for s in runs}
    p = by_m[1].plan
    y1, y2 = segment_gyro(by_m[1]), segment_gyro(by_m[2])
    sp = [p.rate_rad_s if a == p.axis_index else 0.0 for a in range(len(AXES))]
    segment = {axis: (UNIT, [y - sp[a] for y in y1[a]]) for a, axis in enumerate(AXES)}
    diff = {axis: [abs(u - v) for u, v in zip(y1[a], y2[a])] for a, axis in enumerate(AXES)}
    settles = t4_rule.settles(segment, SETTLE_FRACTION, {axis: diff[axis][-1] for axis in AXES})
    bounded = t4_rule.bounded(segment, SETTLE_FRACTION, {axis: max(diff[axis]) for axis in AXES})
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {"passed": all(c.ok for c in (*settles, *bounded)) and not any(stale.values()), "S": settles, "B": bounded,
            "stale_reads_in_window": stale, "N": p.n_exec, "s": SETTLE_FRACTION}


def verdict_line(axis, label, ev, wall):
    def part(checks):
        return ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}" for c in checks)
    return (f"{axis:<5} {label:<22} {'PASS' if ev['passed'] else 'FAIL'}  S [{part(ev['S'])}]  B [{part(ev['B'])}]  "
            f"stale {ev['stale_reads_in_window']}, N {ev['N']}, gz wall {wall:.2f} s")


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Sequence:
    runs: list
    report_path: str
    wall_s: float
    evaluation: dict


@pytest.fixture(scope="module")
def live(tmp_path_factory):
    """The oracle's inputs from the plugin build's parameter table, through the oracle's refresh path."""
    _, defaults = run_l4.build_parameters(PLUGIN_DIR)
    path = tmp_path_factory.mktemp("t3_inputs") / "rate_t3_inputs.txt"
    oracle.refresh_inputs(str(defaults), str(path))
    return oracle.read_inputs(str(path)), run_l4.read_param_defaults(defaults)


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


def fly(tmp_path_factory, name, axis, overrides=None):
    runs, path = run_l4.run_sequence(CARD, SCEN / f"step_{axis}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs)
    wall = sum(s.run.wall_s for s in runs)
    run_l4.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def step(tmp_path_factory, axis):
    return fly(tmp_path_factory, f"step_{axis}", axis)


@pytest.fixture(scope="module")
def gains_zero(tmp_path_factory, axis):
    zero = {f"rate_kp_{axis}": (run_l4.F32, "0.0"), f"rate_ki_{axis}": (run_l4.F32, "0.0")}
    return fly(tmp_path_factory, f"step_{axis}_gains_zero", axis, zero)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@needs_gz
def test_step_settles_and_stays_bounded(step, axis, capsys):
    ev = step.evaluation
    _say(capsys, verdict_line(axis, "step", ev, step.wall_s), f"    report: {step.report_path}",
         *[f"    m = {s.run.m}: run stale report {s.run.stale}; window host steps {s.window_steps.start}.."
           f"{s.window_steps.stop - 1}, stale in window {len(s.window_stale)}" for s in step.runs])
    assert [s.run.m for s in step.runs] == [2, 1]
    for s in step.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
    assert ev["passed"], ev


@needs_gz
def test_step_runs_on_the_live_parameters_and_truth_gyro(step, live, axis):
    p_oracle, p_build = live
    for key in oracle.INPUT_KEYS:
        assert p_build[key] == p_oracle[key], key
    for s in step.runs:
        p = s.plan
        assert p.n_exec == oracle.Setup(p_oracle).executions(axis)
        assert p.divisor == p_oracle["rate_loop_divisor"] and p.tau_ref_s == p_oracle[f"rate_tau_ref_{axis}"]
        assert p.rate_rad_s == p_oracle[f"rate_max_{axis}"]
        for k in p.window:
            assert s.executions[k]["flags"] == 1 << GYRO_VALID_BIT, "the sample is not the truth gyro"


# ---- negative control -----------------------------------------------------------------------------------------------

@needs_gz
def test_control_gains_zero_fails_the_predicates(gains_zero, axis, capsys):
    ev = gains_zero.evaluation
    _say(capsys, verdict_line(axis, "control kp = ki = 0", ev, gains_zero.wall_s))
    for s in gains_zero.runs:
        assert s.harness_overrides == {f"rate_kp_{axis}": ("f32", "0.0"), f"rate_ki_{axis}": ("f32", "0.0")}
        assert f'<sil_override param="rate_kp_{axis}" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the predicates, not on a stale read"
    assert not ev["passed"], ev
