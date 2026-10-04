"""The plugin's sensor elements on the L5 composition (decision 0019, W1 C3): the run_l5 passthrough and the accelerometer
tie of ruling 11 (test e). It runs in the gz-l5 job (ci/run_ci_gz.sh, step gz_sensors_l5) on the host-gz-l5 build.

Skipped unless the host-gz-l5 build exists (its CMakeCache.txt, parameter table and G3 manifest); test e also needs
`gz sim --force-version 8` reporting 8.x and that build's libmarv_gz_lockstep.so. The gz step fails on a skip.

(a)(ii) on the L5 worlds: test_l4_sensors.py's checks, on every committed L5 scenario (scenarios/quad/L05) and every m of
its m_sequence, through run_l5.run_chirp for the chirp scenarios and run_l5.run_step for the others. The default world
holds one <gyro_source>truth</gyro_source> and one <attitude_source>truth</attitude_source>; with the model gyro only the
truth-gyro element is removed (the attitude source stays the truth: L5 has no estimator before L7). run_sequence passes
the sensors to run_step. Controls as in test_l4_sensors.py.

Test e (ruling 11), as test_l4_sensors.py's, on recover_tumble (R2) at m = 1, the scenario's seed 1, the model gyro and
the rotor-speed model on, no clock element: SIGNS = (g, a) against ACCEL_FLIPPED = (g, -a). The masked comparison is
test_l4_sensors.py's (record 6's accel turn-on biases and signs and every TICK's 12 accel_m_s2 bytes zeroed, every other
byte equal), and here it also covers every TRUTH record, the truth state passed to the SIL's marv_truth_state_set each
tick, which the L5 composition reads. Same guards and gyro-sign control; the static guard scans the plugin build's SIL
library (MARV_GZ_SIL = marv_sil_l5_attitude_scripted), with the L0 SIL library as its control.
At L7 the estimator reads the accelerometer: then this test and its static guard are expected to fail, and the tie
breaks by itself (ruling 11).
"""

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import composition_sensors as cs  # noqa: E402
import gen_world  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import lockstep_log  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

ROOT = cs.ROOT
CARD = cs.CARD
SCEN = ROOT / "scenarios" / "quad" / "L05"
BUILD = ROOT / "build" / "host-gz-l5"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
SIL = "marv_sil_l5_attitude_scripted"
COMPOSITION_SOURCE = "fw/compositions/l5_attitude_scripted/src/l5_attitude_scripted.cpp"
CONTROL_SIL, CONTROL_READER = "marv_sil_l0", "fw/compositions/l0/src/l0.cpp"
ATTITUDE_LINE = run_l5.ATTITUDE_ELEMENT
TIE_SCENARIO = SCEN / "recover_tumble.yaml"
TIE_M = 1
# Labelled harness values: turn-on corners, gyro x y z then accel x y z, every sign nonzero.
SIGNS = (1, -1, 1, -1, 1, 1)
ACCEL_FLIPPED = SIGNS[:3] + tuple(-s for s in SIGNS[3:])
GYRO_FLIPPED = (-SIGNS[0],) + SIGNS[1:]

has_build = pytest.mark.skipif(not ((BUILD / "CMakeCache.txt").exists() and (BUILD / "g3_manifest.json").exists()),
                               reason="the host-gz-l5 build is absent")
needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l5 build of libmarv_gz_lockstep.so is absent")


def model(signs):
    return gen_world.SensorSet(gyro="model", rotor=True, bias_signs=signs)


def scenarios():
    out = []
    for path in sorted(SCEN.glob("*.yaml")):
        vals = l5s.values(l5s.load(path))
        drive = run_l5.run_chirp if "chirp" in vals["script"] else run_l5.run_step
        out.extend((path, drive, m) for m in vals["m_sequence"])
    return out


# ---- (a)(ii) on the L5 worlds ----------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def worlds(tmp_path_factory):
    """(scenario, m) -> {"default", "none", set_id -> text, "doc"}: every world in its own directory per (scenario, m),
    so that the log paths agree."""
    out = {}
    for path, drive, m in scenarios():
        d = tmp_path_factory.mktemp(f"{path.stem}_m{m}")
        w = {"default": cs.capture_world(lambda: drive(CARD, path, m, d)),
             "none": cs.capture_world(lambda: drive(CARD, path, m, d, sensors=None))}
        for s in cs.SENSOR_SETS:
            w[cs.set_id(s)] = cs.capture_world(lambda s=s: drive(CARD, path, m, d, sensors=s))
        w["doc"] = cs.l2_doc(d)
        out[(path.stem, m)] = w
    return out


@has_build
def test_every_committed_l5_scenario_is_covered(worlds):
    assert {k for k, _ in worlds} == {p.stem for p in SCEN.glob("*.yaml")}
    assert len(worlds) == len(scenarios())
    assert any(drive is run_l5.run_chirp for _, drive, _ in scenarios())


@has_build
def test_sensors_none_is_the_default_world_byte_for_byte(worlds):
    for key, w in worlds.items():
        assert w["none"] == w["default"], key
        assert w[cs.set_id(gen_world.SensorSet())] == w["default"], key
        assert cs.default_world_problems(w["default"]) == [], key
        assert w["default"].count(ATTITUDE_LINE) == 1, key


@has_build
@pytest.mark.parametrize("s", cs.SENSOR_SETS, ids=cs.set_id)
def test_a_sensors_world_differs_only_by_the_inserted_elements(worlds, s):
    for (name, m), w in worlds.items():
        assert cs.world_problems(w["default"], w[cs.set_id(s)], s, w["doc"], m) == [], (name, m)
        assert w[cs.set_id(s)].count(ATTITUDE_LINE) == 1, (name, m)


@has_build
def test_run_sequence_passes_the_sensors(tmp_path):
    s = cs.SENSOR_SETS[2]
    first = l5s.values(l5s.load(TIE_SCENARIO))["m_sequence"][0]
    seq = cs.capture_world(lambda: run_l5.run_sequence(CARD, TIE_SCENARIO, tmp_path, sensors=s))
    assert seq == cs.capture_world(lambda: run_l5.run_step(CARD, TIE_SCENARIO, first, tmp_path, sensors=s))
    assert "<gyro_source>model</gyro_source>" in seq


@has_build
def test_control_a_stray_change_or_a_corner_checked_as_nominal_is_flagged(worlds):
    w = worlds[(TIE_SCENARIO.stem, TIE_M)]
    s = cs.SENSOR_SETS[2]  # model and rotor, no clock element
    text = w[cs.set_id(s)]
    head, _, tail = text.rpartition("</plugin>")  # the lockstep plugin element closes last
    kept = f"{head}  {cs.TRUTH_GYRO_LINE}\n</plugin>{tail}"  # the truth gyro not removed
    assert cs.world_problems(w["default"], kept, s, w["doc"], TIE_M) != []
    assert cs.world_problems(w["default"], text.replace("<seed>1</seed>", "<seed>2</seed>"), s, w["doc"], TIE_M) != []
    c1, c0 = cs.SENSOR_SETS[5], cs.SENSOR_SETS[4]
    assert (c1.clock_corner, c0.clock_corner) == (1, 0)
    assert cs.world_problems(w["default"], w[cs.set_id(c1)], c0, w["doc"], TIE_M) != []


# ---- test e: the accelerometer tie (ruling 11) -----------------------------------------------------------------------

@pytest.fixture(scope="module")
def tie_logs(tmp_path_factory):
    out = {}
    for tag, signs in (("base", SIGNS), ("accel", ACCEL_FLIPPED), ("gyro", GYRO_FLIPPED)):
        s = run_l5.run_step(CARD, TIE_SCENARIO, TIE_M, tmp_path_factory.mktemp(f"tie_{tag}"), sensors=model(signs))
        assert s.run.log["sensors"]["imu"]["turn_on_bias_signs"] == signs
        assert len(s.run.log["truths"]) == len(s.run.log["ticks"])
        out[tag] = Path(s.run.log_path).read_bytes()
    return out


@needs_gz
def test_e_accel_sign_corners_give_identical_logs_but_the_accel_bytes(tie_logs):
    a, b = tie_logs["base"], tie_logs["accel"]
    assert len(a) == len(b)
    kinds = [k for k, _ in cs.records(a)]
    assert kinds == [k for k, _ in cs.records(b)]
    assert lockstep_log.TRUTH in kinds and lockstep_log.ROTOR in kinds
    ma, mb = cs.masked_log(a), cs.masked_log(b)
    first = run_scenario.first_difference(ma, mb)
    assert first is None, cs.where(a, first)


@needs_gz
def test_e_guard_the_accel_bytes_differ_on_every_tick_and_stay_valid(tie_logs):
    a, b = tie_logs["base"], tie_logs["accel"]
    n, same = cs.accel_differs_every_tick(a, b)
    assert n > 0 and same == 0, (n, same)
    assert cs.accel_valid_and_finite(a) == 0 and cs.accel_valid_and_finite(b) == 0


@needs_gz
def test_e_control_a_gyro_sign_changes_the_log_and_the_dshot(tie_logs):
    a, g = tie_logs["base"], tie_logs["gyro"]
    assert cs.masked_log(a) != cs.masked_log(g)
    n, _ = cs.dshot_differs(a, g)
    assert n > 0, "flipping the gyro x turn-on sign left every DShot command unchanged"


@has_build
def test_e_static_guard_no_composition_file_reads_the_accelerometer():
    assert cs.cache_value(BUILD, "MARV_GZ_SIL") == SIL
    closure, readers = cs.accel_readers(BUILD, SIL)
    assert COMPOSITION_SOURCE in closure
    assert set(readers) == cs.BOUNDARY, readers


@has_build
def test_e_control_the_scan_flags_the_l0_composition():
    closure, readers = cs.accel_readers(BUILD, CONTROL_SIL)
    assert set(readers) - cs.BOUNDARY == {CONTROL_READER}, readers
