"""The L5 hover predicate with the attitude estimator in authority (quad spec 4.0 M3 done-when: "the L5 Gazebo hover predicate
passes on the estimate"; docs/decisions/0042 item 4, predicates of decisions 0029 and 0031).

Skipped only as tests/regression/quad/L07/gz/conftest.py says; the gz CI step fails on a skip. The runs are on the host-gz-l7
build: the l7_attitude_estimated composition (angle mode, the attitude law, the rate loop in bypass) takes its attitude from the
Mahony estimator on the model IMU (gyro and accelerometer, nominal corner); the world has no <attitude_source> and the SIL
library no truth entry. Not a validation run: truth and firmware share one card (core section 6).

Run. scenarios/quad/L07/hover.yaml: level at hover rotor speed, a body-rate kick [0.5, 0.5, 0] rad/s, sticks centred, L5's
horizon; one gz process per m of [2, 1] (tools/sim/run_l5.py run_step, through l7_gz.py).

Quantity. The truth tilt (the angle between body z and NED z) from the log's STEP body records, at each attitude execution. It
is DRIVEN under decision 0031 (zero at the start, set moving by the kick), so only S applies. S (tools/sim/t4_rule.py): at the
last execution |tilt| <= s Delta + E + attitude_tilt_error_max, s the budget's t4_settle_fraction, Delta the largest tilt of the
m = 1 run, E = |tilt(m = 1) - tilt(m = 2)| at the last execution, attitude_tilt_error_max the budget's tilt share of the
estimator (a PLACEHOLDER there). Heading is not checked (no magnetometer).

Control, the loop disabled: att_kp = 0 and rate_ki_roll = rate_ki_pitch = 0 through sil_override, so no gain returns the
tilt to level. It must fail S. att_kp = 0 alone passes (the rate loop's integral restores a kick's tilt), so it is not a
control (decision 0042 item 4; measurements in tests/regression/quad/L07/results/hover_kick).
"""

import dataclasses
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import l7_gz  # noqa: E402

run_l5 = l7_gz.run_l5
run_scenario = l7_gz.run_scenario


@dataclasses.dataclass
class Sequence:
    runs: list
    evaluation: dict
    wall_s: float


def _fly(tmp_path_factory, name, overrides=None):
    runs = l7_gz.fly(tmp_path_factory.mktemp(name), overrides)
    return Sequence(runs, l7_gz.evaluate(runs), sum(s.run.wall_s for s in runs))


@pytest.fixture(scope="module")
def estimated(tmp_path_factory):
    return _fly(tmp_path_factory, "hover_estimate")


@pytest.fixture(scope="module")
def loop_disabled(tmp_path_factory):
    return _fly(tmp_path_factory, "hover_loop_disabled", l7_gz.OVERRIDES_LOOP_DISABLED)


def _say(capsys, line):
    with capsys.disabled():
        print()
        print(line)


def test_hover_on_the_estimate_settles(estimated, capsys):
    ev = estimated.evaluation
    _say(capsys, l7_gz.verdict_line("estimate in authority", ev, estimated.wall_s))
    idle, top = run_l5.l4.dshot_idle_bound(l7_gz.live_params())
    assert [s.run.m for s in estimated.runs] == [2, 1]
    for s in estimated.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        lo, hi = min(min(t["dshot"]) for t in s.run.log["ticks"]), max(max(t["dshot"]) for t in s.run.log["ticks"])
        assert idle <= lo and hi <= top, f"DShot [{lo}, {hi}] outside [{idle}, {top}]"
    assert ev["passed"], ev


def test_hover_runs_on_the_estimate_alone(estimated):
    """The only attitude source is the estimator: no <attitude_source>, no truth gyro, no TRUTH record in the log, the model
    IMU in the world, and the build is the host-gz-l7 one."""
    for s in estimated.runs:
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert "<attitude_source>" not in text and "<gyro_source>truth</gyro_source>" not in text
        assert text.count("<gyro_source>model</gyro_source>") == 1 and text.count("<imu_model>") == 1
        assert len(s.run.log["truths"]) == 0
        assert s.harness_overrides == {}
        assert "estimator_fed_imu_model" in Path(s.run.log_path).name
    assert l7_gz.PARAMS in str(estimated.runs[0].params_path)


def test_control_loop_disabled_fails_the_predicate(loop_disabled, capsys):
    ev = loop_disabled.evaluation
    _say(capsys, l7_gz.verdict_line("control, loop disabled", ev, loop_disabled.wall_s))
    for s in loop_disabled.runs:
        assert s.harness_overrides == l7_gz.OVERRIDES_LOOP_DISABLED
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        for name in l7_gz.OVERRIDES_LOOP_DISABLED:
            assert f'<sil_override param="{name}" type="f32">0.0</sil_override>' in text
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the predicate, not on a stale read"
    assert not ev["S"].ok, ev
