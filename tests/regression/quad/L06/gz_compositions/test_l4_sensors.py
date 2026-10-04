"""The plugin's sensor elements on the L4 composition (decision 0019, W1 C3): the run_l4 passthrough and the accelerometer
tie of ruling 11 (test e). It runs in the gz-l4 job (ci/run_ci_gz.sh, step gz_sensors_l4) on the host-gz-l4 build.

Skipped unless the host-gz-l4 build exists (its CMakeCache.txt, parameter table and G3 manifest); test e also needs
`gz sim --force-version 8` reporting 8.x and that build's libmarv_gz_lockstep.so. The gz step fails on a skip.

(a)(ii) on the L4 worlds. For every committed L4 scenario (scenarios/quad/L04: step, chirp, acro) and every m of its
m_sequence, the world run_l4 writes (run_step, run_chirp, run_acro on the build's parameter table, taken before the gz
process starts: none is started) with sensors=None is the world written without the argument, byte for byte, and with
SensorSet() too; it holds one <gyro_source>truth</gyro_source> and no sensor element. With each SensorSet of test
(a)(ii) (tests/regression/quad/L06/tools/test_world_sensors.py SENSOR_SETS) the world differs from that default world
only by the inserted elements (children of the lockstep plugin element), by the truth-gyro element removed iff the gyro
is the model (gen_world writes <gyro_source>model</gyro_source> instead; the plugin takes one <gyro_source>), and by
max_step_size iff the clock corner is nonzero (fl(n_true / 1e9)). run_sequence passes the sensors to run_step. Controls:
a model world that still carries the truth-gyro element, a seed edit, and a nonzero-corner world checked as nominal are
flagged.

Test e (ruling 11: "Each tie needs a per-push test proving it bit-identical. For example, two corners differing only in
accel bias give identical logs in L4 and L5"). Scenario step_roll at m = 1, the scenario's seed 1, the model gyro and the
rotor-speed model on, no clock element. Two runs differ only in the accelerometer's turn-on signs: SIGNS = (g, a) and
ACCEL_FLIPPED = (g, -a), every sign nonzero.
  Compared: the two log files byte for byte, after zeroing in both (composition_sensors.masked_log) record 6's accel
  turn-on biases (3 f64) and accel signs (3 i8), which describe the corner, and the 12 accel_m_s2 bytes of every TICK
  record's IMU sample, the accelerometer reading passed to the SIL. Everything else must be equal: the header; the rest
  of record 6; every STEP (the gz read and the plant body); every TICK's tick, stamp, gyro bytes, temperature, flags
  word (the accel valid and saturation bits included), DShot, eRPM bits and plant outputs; every APPLIED wrench, every
  ROTOR sample and the trailer.
  Why all of it: per tick the SIL is given the stamp, the IMU sample (gyro, accel, temperature, flags) and the rotor
  sample, and gives back DShot. If the composition does not read the accelerometer, equal inputs but the accel give
  equal DShot, so the plant moves the same and every later input but the accel is equal again: the whole log is the
  claim, not the DShot alone.
  Guards: the accel bytes differ on every tick and every axis (the corners are two turn-on bounds apart, about 655
  LSB), and in both runs every tick's sample has AccelValid set and a finite accel, so the SIL's boundary check of the accelerometer (fw/sil/src/marv_sil.cpp sample_valid, the only
  reader in the library) passes alike.
  Control: GYRO_FLIPPED, SIGNS with the gyro x sign flipped, changes the masked log and the DShot.
  Static guard: no file compiled into the plugin build's SIL library (MARV_GZ_SIL = marv_sil_l4_rate_scripted; its
  compile closure from ninja: the sources of its compile commands and their recorded header dependencies) names an
  accelerometer field or flag (accel_m_s2, AccelSat*, AccelValid, MARV_IMU_ACCEL_*), except the SIL boundary: marv_sil.h,
  imu_sample.hpp and marv_sil.cpp (the validity check and the copy into ImuSample). The HAL has no accelerometer read:
  the sample reaches the composition only through the SIL's ImuSample. Control: the same scan on the L0 SIL library of
  the same build tree flags fw/compositions/l0/src/l0.cpp, whose composition low-passes the accelerometer.
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
import run_l4  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402

ROOT = cs.ROOT
CARD = cs.CARD
SCEN = ROOT / "scenarios" / "quad" / "L04"
BUILD = ROOT / "build" / "host-gz-l4"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
SIL = "marv_sil_l4_rate_scripted"
COMPOSITION_SOURCE = "fw/compositions/l4_rate_scripted/src/l4_rate_scripted.cpp"
CONTROL_SIL, CONTROL_READER = "marv_sil_l0", "fw/compositions/l0/src/l0.cpp"
DRIVERS = {"step": run_l4.run_step, "chirp": run_l4.run_chirp, "acro": run_l4.run_acro}
TIE_SCENARIO = SCEN / "step_roll.yaml"
TIE_M = 1
# Labelled harness values: turn-on corners, gyro x y z then accel x y z, every sign nonzero.
SIGNS = (1, -1, 1, -1, 1, 1)
ACCEL_FLIPPED = SIGNS[:3] + tuple(-s for s in SIGNS[3:])
GYRO_FLIPPED = (-SIGNS[0],) + SIGNS[1:]

has_build = pytest.mark.skipif(not ((BUILD / "CMakeCache.txt").exists() and (BUILD / "g3_manifest.json").exists()),
                               reason="the host-gz-l4 build is absent")
needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l4 build of libmarv_gz_lockstep.so is absent")


def model(signs):
    return gen_world.SensorSet(gyro="model", rotor=True, bias_signs=signs)


def scenarios():
    out = []
    for path in sorted(SCEN.glob("*.yaml")):
        kind = path.stem.split("_")[0]
        assert kind in DRIVERS, f"{path.name}: no driver for the kind {kind!r}"
        out.extend((path, kind, m) for m in schema.load_yaml(path)["m_sequence"]["value"])
    return out


# ---- (a)(ii) on the L4 worlds ----------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def worlds(tmp_path_factory):
    """(scenario, m) -> {"default", "none", set_id -> text, "doc"}: every world in its own directory per (scenario, m),
    so that the log paths agree."""
    out = {}
    for path, kind, m in scenarios():
        d = tmp_path_factory.mktemp(f"{path.stem}_m{m}")
        drive = DRIVERS[kind]
        w = {"default": cs.capture_world(lambda: drive(CARD, path, m, d)),
             "none": cs.capture_world(lambda: drive(CARD, path, m, d, sensors=None))}
        for s in cs.SENSOR_SETS:
            w[cs.set_id(s)] = cs.capture_world(lambda s=s: drive(CARD, path, m, d, sensors=s))
        w["doc"] = cs.l2_doc(d)
        out[(path.stem, m)] = w
    return out


@has_build
def test_every_committed_l4_scenario_kind_is_covered(worlds):
    assert {k for k, _ in worlds} == {p.stem for p in SCEN.glob("*.yaml")}
    assert len(worlds) == len(scenarios())


@has_build
def test_sensors_none_is_the_default_world_byte_for_byte(worlds):
    for key, w in worlds.items():
        assert w["none"] == w["default"], key
        assert w[cs.set_id(gen_world.SensorSet())] == w["default"], key
        assert cs.default_world_problems(w["default"]) == [], key


@has_build
@pytest.mark.parametrize("s", cs.SENSOR_SETS, ids=cs.set_id)
def test_a_sensors_world_differs_only_by_the_inserted_elements(worlds, s):
    for (name, m), w in worlds.items():
        assert cs.world_problems(w["default"], w[cs.set_id(s)], s, w["doc"], m) == [], (name, m)


@has_build
def test_run_sequence_passes_the_sensors(tmp_path):
    s = cs.SENSOR_SETS[2]
    first = schema.load_yaml(TIE_SCENARIO)["m_sequence"]["value"][0]
    seq = cs.capture_world(lambda: run_l4.run_sequence(CARD, TIE_SCENARIO, tmp_path, sensors=s))
    assert seq == cs.capture_world(lambda: run_l4.run_step(CARD, TIE_SCENARIO, first, tmp_path, sensors=s))
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
        s = run_l4.run_step(CARD, TIE_SCENARIO, TIE_M, tmp_path_factory.mktemp(f"tie_{tag}"), sensors=model(signs))
        assert s.run.log["sensors"]["imu"]["turn_on_bias_signs"] == signs
        out[tag] = Path(s.run.log_path).read_bytes()
    return out


@needs_gz
def test_e_accel_sign_corners_give_identical_logs_but_the_accel_bytes(tie_logs):
    a, b = tie_logs["base"], tie_logs["accel"]
    assert len(a) == len(b)
    assert [k for k, _ in cs.records(a)] == [k for k, _ in cs.records(b)]
    ma, mb = cs.masked_log(a), cs.masked_log(b)
    first = run_scenario.first_difference(ma, mb)
    assert first is None, cs.where(a, first)


@needs_gz
def test_e_guard_the_accel_bytes_differ_on_every_tick_and_stay_valid(tie_logs):
    a, b = tie_logs["base"], tie_logs["accel"]
    n, same = cs.accel_differs_every_tick(a, b)
    assert n > 0 and same == 0, (n, same)
    assert a != b
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
