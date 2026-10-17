"""The plumbing of the quad_fc flight composition into the gz-sim 8 lockstep plugin (HIL S2d-2a, decision 0046 stage S2, item 4).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz-quadfc build holds libmarv_gz_lockstep.so
(`cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc`); the gz CI step fails on a skip. Every gz
process gets its own GZ_PARTITION and GZ_IP=127.0.0.1.

The plugin of this build links marv_sil_quad_fc (the flight composition with the FC3v2 parameter set) and stages a
<manual_control_script> into it through marv_sil_stage_manual_control before every tick. The world is the L2 hover world of the
UZH card (member lo, m = 1, a whole-ns tick of 156250 ns) with the model gyro, without the open-loop DShot overrides (the
FC3v2 set has no ol_dshot_m*), with rc_channel_angle = 6 through <sil_override> (the register's default is 0, acro) and the
script. There is no ground in the world, so the vehicle falls from tick 0 with a specific force of zero; the estimator would
never initialise from that. Two harness values of this file only (labelled below) make the free-falling sample usable for the
arming check: estimator_accel_healthy_min = 0 and the model accelerometer's turn-on bias at -1 x its bound on z (the FRD
specific force of a level vehicle at rest is -z). Nothing here is a flight result; arming is read from the DShot values as
tests/regression/quad/L09/unit/sil_quad_fc_test.cpp reads it: 0 is stop, a value of 48 or more is a throttle command.

What is checked:
  - A script whose arm channel stays low for the whole run: DShot is stop on every tick, and the log holds the staged frames
    (record 11): one per 8 ms of SIL time from the first slot, frames counting from 1, t_us = the SIL stamp of the tick the
    frame was staged before, the sticks and the arm and angle channels as scripted, every other channel 0.
  - The arm channel raised after the link has been valid for the gyro calibration and the link recovery time (the register's
    values plus a margin): stop before the raise, a throttle command on every motor from the first rate tick after it on.
  - Controls: the same raise before the calibration has finished (and never cycled after it) does not arm; the same raise in a
    link-off segment stages no frame and does not arm; after arming, a link-off segment stages nothing, the last staged frame
    persists (the output stays a throttle command) and its age brings the failsafe stop at failsafe_disarm_us; a run with no
    script has no record 11 and is stop throughout, and a script whose only segment is link off writes the same log as no script.
  - Planted refusals of a malformed <manual_control_script>.
"""

import re
import struct
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
HOVER = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
REGISTER = ROOT / "fw" / "compositions" / "quad_fc" / "params" / "quad_fc_register.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz-quadfc" / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
TIMEOUT_S = 300
SEED = 1
NUM_US, DEN = 625, 4  # the scenario's tick period, us (scenarios/quad/L02/hover.yaml)
FRAME_PERIOD_US = 8000  # 125 frames/s, measured on Luis's ExpressLRS link (docs/m2-bench-card.md)
ARM_CHANNEL = 5  # the register's rc_channel_arm (read below and asserted equal)
ANGLE_CHANNEL = 6  # labelled harness value: written to rc_channel_angle through <sil_override>
DSHOT_THROTTLE_MIN = 48  # DShot protocol: 0 is stop, 1..47 are commands, 48..2047 are throttle values
SWITCH_LOW, SWITCH_HIGH = -1.0, 1.0  # normalised channel values, [-1, 1] (fw/types manual_control.hpp)
MARGIN_US = 1_000_000  # labelled harness value: the margin after calibration + recovery before the raise
SETTLE_US = 250_000  # labelled harness value: sim time run after the raise before the output is read
SLACK_US = 10_000  # labelled harness value: allowance on a failsafe edge, a few rate periods
# Labelled harness values making the free-falling model-IMU sample usable (module docstring).
ACCEL_HEALTHY_MIN_G = "0.0"
BIAS_SIGNS = (0, 0, 0, 0, 0, -1)
# The sticks of the scripted frames in the control that checks the log: exactly representable in binary32.
STICKS = {"roll": 0.5, "pitch": -0.25, "yaw": 0.125, "throttle": 0.0}

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz-quadfc build of libmarv_gz_lockstep.so is absent")


def register_value(name):
    return yaml.safe_load(REGISTER.read_text(encoding="utf-8"))[name]["value"]


CAL_US = register_value("gyro_cal_duration_us")
RECOVERY_US = register_value("failsafe_recovery_us")
LOSS_US = register_value("failsafe_loss_us")
DISARM_US = register_value("failsafe_disarm_us")
READY_US = CAL_US + RECOVERY_US + MARGIN_US
assert register_value("rc_channel_arm") == ARM_CHANNEL
assert register_value("rc_channel_angle") == 0, "the world sets it: the test relies on the register default being acro"


def stamp(tick):
    return (tick // DEN) * NUM_US + ((tick % DEN) * NUM_US) // DEN


def iterations(duration_us):
    return -(-duration_us * DEN // NUM_US)  # ticks of m = 1 that reach duration_us


def seg(start_us, duration_us, arm, link="on", **sticks):
    s = {**{"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}, **sticks}
    return (f"<segment><start_us>{start_us}</start_us><duration_us>{duration_us}</duration_us><link>{link}</link>"
            f"<roll>{s['roll']}</roll><pitch>{s['pitch']}</pitch><yaw>{s['yaw']}</yaw><throttle>{s['throttle']}</throttle>"
            f"<arm>{arm}</arm><angle>1.0</angle></segment>")


def script(segments, period=FRAME_PERIOD_US):
    return (f"<manual_control_script><frame_period_us>{period}</frame_period_us><arm_channel>{ARM_CHANNEL}</arm_channel>"
            f"<angle_channel>{ANGLE_CHANNEL}</angle_channel>{''.join(segments)}</manual_control_script>")


def world(log, script_text):
    text = gen_world.generate(CARD, HOVER, "test", 1, "lo", str(log),
                              sensors=gen_world.SensorSet(gyro="model", bias_signs=BIAS_SIGNS))[1]
    text = re.sub(r'\s*<sil_override param="ol_dshot_m\d" type="i32">\d+</sil_override>', "", text)
    add = (f'<sil_override param="rc_channel_angle" type="i32">{ANGLE_CHANNEL}</sil_override>\n'
           f'<sil_override param="estimator_accel_healthy_min" type="f32">{ACCEL_HEALTHY_MIN_G}</sil_override>\n')
    add += (script_text + "\n") if script_text else ""
    assert text.count("<gyro_source>") == 1
    return text.replace("<gyro_source>", add + "<gyro_source>", 1)


def run(tmp_path_factory, name, script_text, duration_us):
    tmp = tmp_path_factory.mktemp(name)
    log = tmp / "run.bin"
    sdf = tmp / "world.sdf"
    sdf.write_text(world(log, script_text), encoding="utf-8")
    n = iterations(duration_us)
    r = run_scenario.run_gz_process(sdf, n, SEED, PLUGIN_DIR, None, TIMEOUT_S)
    return r, log, n


def read_ok(tmp_path_factory, name, script_text, duration_us):
    r, log, n = run(tmp_path_factory, name, script_text, duration_us)
    assert r.returncode == 0 and "REFUSED" not in r.stderr, run_scenario.stderr_tail(r.stderr)
    d = lockstep_log.read(log)
    assert len(d["ticks"]) == n and d["trailer"] == {"steps": n, "ticks": n, "applied": n}
    assert [t["sil_t_us"] for t in d["ticks"]] == [stamp(j) for j in range(n)]
    return d


def stopped(t):
    return all(v == 0 for v in t["dshot"])


def throttle(t):
    return all(v >= DSHOT_THROTTLE_MIN for v in t["dshot"])


DURATION_US = READY_US + SETTLE_US


@pytest.fixture(scope="module")
def low_run(tmp_path_factory):
    return read_ok(tmp_path_factory, "low", script([seg(0, DURATION_US, SWITCH_LOW, **STICKS)]), DURATION_US)


@pytest.fixture(scope="module")
def raised_run(tmp_path_factory):
    segs = [seg(0, READY_US, SWITCH_LOW), seg(READY_US, SETTLE_US, SWITCH_HIGH)]
    return read_ok(tmp_path_factory, "raised", script(segs), DURATION_US)


def test_switch_low_throughout_dshot_stays_stop(low_run):
    assert all(stopped(t) for t in low_run["ticks"])


def test_staged_frames_are_logged_at_the_link_rate_with_the_scripted_content(low_run):
    ms = low_run["manuals"]
    ticks = low_run["ticks"]
    last_stamp = ticks[-1]["sil_t_us"]
    assert len(ms) == last_stamp // FRAME_PERIOD_US  # slot 0 carries no frame
    for k, m in enumerate(ms, start=1):
        t = ticks[m["tick"]]
        assert m["frames"] == k and m["t_us"] == t["sil_t_us"]
        # the first tick whose stamp reaches the frame's slot: the tick before it is in the slot before
        assert t["sil_t_us"] // FRAME_PERIOD_US == k and ticks[m["tick"] - 1]["sil_t_us"] // FRAME_PERIOD_US == k - 1
        assert {n: m[n] for n in STICKS} == STICKS
        assert m["channels"][ARM_CHANNEL - 1] == SWITCH_LOW and m["channels"][ANGLE_CHANNEL - 1] == 1.0
        assert [c for i, c in enumerate(m["channels"]) if i not in (ARM_CHANNEL - 1, ANGLE_CHANNEL - 1)] == [0.0] * 14


def test_switch_raised_after_calibration_and_recovery_arms(raised_run):
    ticks = raised_run["ticks"]
    assert all(stopped(t) for t in ticks if t["sil_t_us"] < READY_US)
    armed = [t for t in ticks if throttle(t)]
    assert armed, "the output never left stop"
    first = armed[0]["sil_t_us"]
    assert READY_US <= first < READY_US + SETTLE_US
    assert all(throttle(t) for t in ticks if t["sil_t_us"] >= first)
    assert not any(not (stopped(t) or throttle(t)) for t in ticks)
    raised = [m for m in raised_run["manuals"] if m["channels"][ARM_CHANNEL - 1] == SWITCH_HIGH]
    assert raised and raised[0]["t_us"] >= READY_US and raised[0]["t_us"] <= first


def test_control_raise_before_the_calibration_has_finished_does_not_arm(tmp_path_factory):
    early = CAL_US // 2
    segs = [seg(0, early, SWITCH_LOW), seg(early, DURATION_US - early, SWITCH_HIGH)]
    d = read_ok(tmp_path_factory, "early", script(segs), DURATION_US)
    assert any(m["channels"][ARM_CHANNEL - 1] == SWITCH_HIGH for m in d["manuals"])
    assert all(stopped(t) for t in d["ticks"])


def test_control_link_off_stages_nothing_and_does_not_arm(tmp_path_factory):
    segs = [seg(0, READY_US, SWITCH_LOW), seg(READY_US, SETTLE_US, SWITCH_HIGH, link="off")]
    d = read_ok(tmp_path_factory, "linkoff", script(segs), DURATION_US)
    assert d["manuals"] and all(m["t_us"] < READY_US and m["channels"][ARM_CHANNEL - 1] == SWITCH_LOW for m in d["manuals"])
    assert [m["frames"] for m in d["manuals"]] == list(range(1, len(d["manuals"]) + 1))
    assert all(stopped(t) for t in d["ticks"])


def test_after_arming_a_link_off_segment_keeps_the_last_frame_and_its_age_stops_the_output(tmp_path_factory):
    off = READY_US + SETTLE_US
    duration = off + DISARM_US + 2 * SETTLE_US
    segs = [seg(0, READY_US, SWITCH_LOW), seg(READY_US, SETTLE_US, SWITCH_HIGH), seg(off, duration - off, SWITCH_HIGH, link="off")]
    d = read_ok(tmp_path_factory, "persist", script(segs), duration)
    last = d["manuals"][-1]["t_us"]
    assert off - FRAME_PERIOD_US - NUM_US <= last < off
    ticks = d["ticks"]
    armed_from = next(t["sil_t_us"] for t in ticks if throttle(t))
    assert armed_from < off
    assert all(throttle(t) for t in ticks if armed_from <= t["sil_t_us"] < last + DISARM_US - SLACK_US)
    assert all(stopped(t) for t in ticks if t["sil_t_us"] >= last + DISARM_US + SLACK_US)
    assert last + DISARM_US + SLACK_US < ticks[-1]["sil_t_us"]


def test_a_world_without_a_script_has_no_frame_records_and_is_stop_throughout(tmp_path_factory):
    d = read_ok(tmp_path_factory, "noscript", None, DURATION_US)
    assert d["manuals"] == []
    assert all(stopped(t) for t in d["ticks"])


def test_a_script_that_stages_nothing_writes_the_log_of_no_script(tmp_path_factory):
    none = read_ok(tmp_path_factory, "noscript2", None, DURATION_US)
    nothing = read_ok(tmp_path_factory, "allinfo", script([seg(0, DURATION_US, SWITCH_HIGH, link="off")]), DURATION_US)
    assert nothing["manuals"] == []
    assert nothing["raw_records"] == none["raw_records"]


def mutate(pattern, repl):
    return lambda t: re.sub(pattern, repl, t, count=1, flags=re.S)


GOOD_SEGS = [seg(0, 1000, SWITCH_LOW), seg(2000, 1000, SWITCH_HIGH)]
REFUSALS = [
    ("overlap", [seg(0, 1000, SWITCH_LOW), seg(999, 1000, SWITCH_HIGH)], None,
     "<segment> at start_us 999 overlaps or precedes the segment before it"),
    ("out_of_order", [seg(2000, 1000, SWITCH_LOW), seg(0, 1000, SWITCH_HIGH)], None,
     "<segment> at start_us 0 overlaps or precedes the segment before it"),
    ("duration_0", [seg(0, 0, SWITCH_LOW)], None, "<segment> duration_us is 0"),
    ("link_bogus", GOOD_SEGS, mutate(r"<link>on</link>", "<link>maybe</link>"), "<segment> link is 'maybe', not 'on' or 'off'"),
    ("arm_out_of_range", [seg(0, 1000, 1.5)], None, "<arm> in <manual_control_script> is 1.5, not a finite value in [-1, 1]"),
    ("throttle_negative", [seg(0, 1000, SWITCH_LOW, throttle=-0.5)], None,
     "<throttle> in <manual_control_script> is -0.5, not a finite value in [0, 1]"),
    ("roll_nan", GOOD_SEGS, mutate(r"<roll>0.0</roll>", "<roll>nan</roll>"),
     "<roll> in <manual_control_script> is nan, not a finite value in [-1, 1]"),
    ("channel_17", GOOD_SEGS, mutate(r"<arm_channel>5</arm_channel>", "<arm_channel>17</arm_channel>"),
     "<arm_channel> is not an RC channel 1..16"),
    ("channel_0", GOOD_SEGS, mutate(r"<angle_channel>6</angle_channel>", "<angle_channel>0</angle_channel>"),
     "<angle_channel> is not an RC channel 1..16"),
    ("same_channel", GOOD_SEGS, mutate(r"<angle_channel>6</angle_channel>", "<angle_channel>5</angle_channel>"),
     "<manual_control_script> arm_channel and angle_channel are the same channel"),
    ("period_0", GOOD_SEGS, mutate(r"<frame_period_us>8000</frame_period_us>", "<frame_period_us>0</frame_period_us>"),
     "<frame_period_us> is 0"),
    ("no_period", GOOD_SEGS, mutate(r"<frame_period_us>8000</frame_period_us>", ""),
     "<manual_control_script> needs <frame_period_us>, <arm_channel>, <angle_channel> and at least one <segment>"),
    ("no_segment", [], None,
     "<manual_control_script> needs <frame_period_us>, <arm_channel>, <angle_channel> and at least one <segment>"),
    ("period_twice", GOOD_SEGS, mutate(r"(<frame_period_us>8000</frame_period_us>)", r"\g<1>\g<1>"),
     "<manual_control_script> element <frame_period_us> appears twice"),
    ("unknown_child", GOOD_SEGS, mutate(r"(<manual_control_script>)", r"\g<1><bogus>1</bogus>"),
     "unknown element <bogus> in <manual_control_script>"),
    ("segment_lacks_roll", GOOD_SEGS, mutate(r"<roll>0.0</roll>", ""), "<segment> lacks <roll>"),
    ("segment_unknown_child", GOOD_SEGS, mutate(r"(<segment>)", r"\g<1><bogus>1</bogus>"), "unknown element <bogus> in <segment>"),
    ("script_twice", GOOD_SEGS, lambda t: t.replace("</manual_control_script>", "</manual_control_script>" + script(GOOD_SEGS), 1),
     "plugin element <manual_control_script> appears twice"),
]


def test_control_the_good_script_runs(tmp_path_factory):
    span = 100_000  # labelled harness value: long enough for a dozen frames
    d = read_ok(tmp_path_factory, "good", script([seg(0, span, SWITCH_LOW)]), span)
    assert len(d["manuals"]) == (span - 1) // FRAME_PERIOD_US


@pytest.mark.parametrize("name,segs,edit,reason", REFUSALS, ids=[r[0] for r in REFUSALS])
def test_planted_malformed_script_is_refused(tmp_path, name, segs, edit, reason):
    log = tmp_path / "run.bin"
    good = world(log, script(segs))
    planted = edit(good) if edit else good
    if edit:
        assert planted != good, "the planted edit did not change the world"
    sdf = tmp_path / "world.sdf"
    sdf.write_text(planted, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, 8, SEED, PLUGIN_DIR, None, TIMEOUT_S)
    assert r.returncode != 0, "the run was not refused"
    assert f"marv_gz_lockstep: REFUSED: {reason}" in r.stderr, run_scenario.stderr_tail(r.stderr)
    assert not log.exists(), "a refused run wrote a log"
