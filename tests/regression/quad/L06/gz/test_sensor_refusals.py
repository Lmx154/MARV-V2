"""Planted refusals of the plugin's sensor elements (decision 0019, W1 C1, test f).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so. Every gz process
gets its own GZ_PARTITION and GZ_IP=127.0.0.1. The good world is the L2 hover world (member lo, m = 1) from
tools/card/gen_world.py with SensorSet(gyro="model", rotor=True, signs, clock_corner=+1); each planted world is that text
with one edit, and each must be refused (non-zero exit, `marv_gz_lockstep: REFUSED: <reason>` on stderr, no log).

Planted: the model without its <imu_model>; an <imu_model> without the model; a sign of +2 and of -2; a rotor latency above
the plant's delay-line capacity; pole_count 0 with the rotor model; a malformed profile hash (short, uppercase); the rotor
model without the model gyro; <clock_corner> without <odr_error>; a corner of 2; an unknown child; a gyro source that is
neither truth nor model; and a clock corner without the model gyro and without the test-only allow (ruling on Q6).

Negative control: the good world itself runs and logs.
"""

import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
HOVER = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz" / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
ITERATIONS = 8  # labelled harness value: a refusal happens at Configure or at the first PreUpdate
TIMEOUT_S = 180
SEED = 1
SENSORS = gen_world.SensorSet(gyro="model", rotor=True, bias_signs=(1, -1, 0, 1, 1, -1), clock_corner=1)

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def world(log, sensors=SENSORS):
    return gen_world.generate(CARD, HOVER, "test", 1, "lo", str(log), sensors=sensors)[1]


def run_gz(text, tmp_path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    return run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, None, TIMEOUT_S)


def drop(pattern):
    return lambda t: re.sub(pattern, "", t, count=1, flags=re.S)


def sub(pattern, repl):
    return lambda t: re.sub(pattern, repl, t, count=1, flags=re.S)


MUTATIONS = [
    ("model_without_block", drop(r"\s*<imu_model>.*?</imu_model>"),
     "<gyro_source>model</gyro_source> needs an <imu_model> element"),
    ("block_without_model", drop(r"\s*<gyro_source>model</gyro_source>"),
     "<imu_model> needs <gyro_source>model</gyro_source>"),
    ("sign_plus_2", sub(r"(<turn_on_bias_signs>)1 ", r"\g<1>2 "),
     "<turn_on_bias_signs> component 0 is 2, not -1, 0 or +1"),
    ("sign_minus_2", sub(r"(<turn_on_bias_signs>1) -1 ", r"\g<1> -2 "),
     "<turn_on_bias_signs> component 1 is -2, not -1, 0 or +1"),
    ("rotor_latency_above_cap", sub(r"(<latency_ticks>)[^<]*", r"\g<1>65"),
     "<rotor_speed_model> latency_ticks 65 is above the plant's delay-line capacity "
     "MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS = 64"),
    ("pole_count_0", sub(r"(<pole_count>)[^<]*", r"\g<1>0"), "<rotor_speed_model> needs a nonzero <pole_count>"),
    ("hash_short", sub(r"(<profile_sha256>)[0-9a-f]", r"\g<1>"), "<profile_sha256> is not 64 lowercase hexadecimal digits"),
    ("hash_uppercase", lambda t: re.sub(r"<profile_sha256>([^<]*)", lambda mo: "<profile_sha256>" + mo.group(1).upper(),
                                        t, count=1),
     "<profile_sha256> is not 64 lowercase hexadecimal digits"),
    ("rotor_without_model", lambda t: drop(r"\s*<imu_model>.*?</imu_model>")(drop(r"\s*<gyro_source>model</gyro_source>")(t)),
     "<rotor_speed_model> needs <gyro_source>model</gyro_source>"),
    ("corner_without_odr_error", drop(r"\s*<odr_error>[^<]*</odr_error>"),
     "<clock_corner> and <odr_error> go together"),
    ("corner_2", sub(r"(<clock_corner>)[^<]*", r"\g<1>2"), "<clock_corner> is 2, not -1, 0 or +1"),
    ("unknown_child", sub(r"(<imu_model>)", r"\g<1><bogus>1</bogus>"), "unknown element <bogus> in <imu_model>"),
    ("gyro_source_bogus", sub(r"<gyro_source>model</gyro_source>", "<gyro_source>bogus</gyro_source>"),
     "<gyro_source> is 'bogus', not 'truth' or 'model'"),
]


def test_control_the_good_world_runs_and_logs(tmp_path):
    log = tmp_path / "run.bin"
    r = run_gz(world(log), tmp_path)
    assert r.returncode == 0 and "REFUSED" not in r.stderr, run_scenario.stderr_tail(r.stderr)
    d = lockstep_log.read(log)
    assert d["sensors"]["gyro_source"] == "model" and len(d["rotors"]) == ITERATIONS


@pytest.mark.parametrize("name,edit,reason", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_planted_sensor_element_is_refused(tmp_path, name, edit, reason):
    log = tmp_path / "run.bin"
    good = world(log)
    planted = edit(good)
    assert planted != good, "the planted edit did not change the world"
    r = run_gz(planted, tmp_path)
    assert r.returncode != 0, "the run was not refused"
    assert f"marv_gz_lockstep: REFUSED: {reason}" in r.stderr, run_scenario.stderr_tail(r.stderr)
    assert not log.exists(), "a refused run wrote a log"


def test_a_clock_corner_without_the_model_gyro_or_the_allow_is_refused(tmp_path):
    log = tmp_path / "run.bin"
    text = world(log, gen_world.SensorSet(clock_corner=1))
    assert "<gyro_source>" not in text
    r = run_gz(text, tmp_path)
    assert r.returncode != 0
    assert "marv_gz_lockstep: REFUSED: <clock_corner> needs <gyro_source>model</gyro_source>" in r.stderr, \
        run_scenario.stderr_tail(r.stderr)
    assert not log.exists()
