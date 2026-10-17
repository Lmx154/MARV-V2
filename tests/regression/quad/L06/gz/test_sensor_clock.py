"""The clock corner of the gz-sim 8 lockstep plugin (decision 0019, W1 C1, test d; ruling 10: outward rounding).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so. Every gz process
gets its own GZ_PARTITION and GZ_IP=127.0.0.1. The worlds are the L2 hover world (member lo) from tools/card/gen_world.py
with SensorSet(clock_corner=c) and no IMU model: the zeroed sample, as the default L2 world. The plugin refuses a clock
corner without <gyro_source>model</gyro_source> unless MARV_GZ_TEST_CLOCK_CORNER_ANY_GYRO is set, and this file is that
variable's only user (ruling on Q6; tests/regression/quad/L06/tools/test_world_sensors.py checks it).

What is checked, at c in {-1, 0, +1} and m in {1, 2}: every simTime increment is n_true (the design's table:
156261/156250/156239 ns at m = 1, 312521/312500/312479 ns at m = 2); record 6 holds n_true, t_true = fl(n_true / (m 1e9)),
the corner and the profile's ODR error bitwise; e_r = n_nom / n_true - 1 is the exact rational (+-11/156239..., the ppm
figures of the table); the SIL stamps stay floor(j 625 / 4) us whatever c is (e never reaches the SIL); and the c = 0 log
is the default log byte for byte once record 6 is removed.

Negative controls: at c = +1 a world whose max_step_size is the nearest step (156240 / 312480 ns) or the nominal one is
refused; and the c = +1 log without record 6 is not the default log.

The rounded host step (decision 0046 item 1), at the board's tick 1953125/14649 us = 133328.2135 ns (not a whole number of
ns), m in {1, 2}: the host step is round(m tick) = 133328 / 266656 ns, record 6 holds it with t_true = n / (m 1e9) and
no clock, record 9 holds the exact m tick and the error -1.601536 ppm, and the SIL stamps stay floor(j 1953125 / 14649) us.
A whole-ns tick (625/4 us) with an <odr_error> element alone is the default log byte for byte. Negative controls: a bound
just under the error (1.6 ppm) is refused and one just over it (1.61 ppm) runs; no <odr_error> is refused; a max_step_size
of the next ns is refused.
"""

import re
import struct
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_imu_config as gic  # noqa: E402
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
HOVER = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz" / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
ALLOW = {"MARV_GZ_TEST_CLOCK_CORNER_ANY_GYRO": "1"}  # the plugin's test-only allow of a corner without the IMU model
ITERATIONS = 512  # labelled harness value: host steps per run, far more than the checks need
TIMEOUT_S = 180
SEED = 1
NUM_US, DEN = 625, 4  # the scenario's tick period, us (scenarios/quad/L02/hover.yaml)
# The W1 design's table (decision 0019): n_true ns, and e_r = n_nom / n_true - 1 exactly.
TABLE = {
    1: {-1: (156261, Fraction(-11, 156261)), 0: (156250, Fraction(0)), 1: (156239, Fraction(11, 156239))},
    2: {-1: (312521, Fraction(-21, 312521)), 0: (312500, Fraction(0)), 1: (312479, Fraction(21, 312479))},
}
PPM = {1: {-1: -70.395044, 1: 70.404957}, 2: {-1: -67.195484, 1: 67.204516}}
NEAREST = {1: 156240, 2: 312480}

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def bits(x):
    return struct.pack("<d", x)


def world(log, m, corner):
    sensors = None if corner is None else gen_world.SensorSet(clock_corner=corner)
    return gen_world.generate(CARD, HOVER, "test", m, "lo", str(log), sensors=sensors)[1]


def run(tmp_path_factory, m, corner, edit=None):
    tmp = tmp_path_factory.mktemp(f"clock_m{m}_c{corner}")
    log = tmp / "run.bin"
    text = world(log, m, corner)
    if edit is not None:
        text = edit(text)
    sdf = tmp / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, ALLOW if corner is not None else None, TIMEOUT_S)
    return r, log


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    out = {}
    for m in TABLE:
        for corner in (None, -1, 0, 1):
            r, log = run(tmp_path_factory, m, corner)
            assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
            out[(m, corner)] = log.read_bytes(), lockstep_log.read(log)
    return out


def without_record_6(data):
    hs = lockstep_log._header(data)["header_size"]
    assert data[hs] == lockstep_log.SENSORS
    return data[:hs] + data[hs + 1 + lockstep_log._SENSORS.size:]


@pytest.mark.parametrize("m", sorted(TABLE))
@pytest.mark.parametrize("corner", (-1, 0, 1))
def test_every_host_step_is_n_true_and_record_6_holds_the_clock(runs, m, corner):
    _, d = runs[(m, corner)]
    n, e_r = TABLE[m][corner]
    assert [s["sim_time_ns"] for s in d["steps"]] == [(i + 1) * n for i in range(ITERATIONS)]
    s6 = d["sensors"]
    assert s6["host_step_ns"] == n and s6["clock"] == 1 and s6["clock_corner"] == corner
    assert bits(s6["tick_s"]) == bits(n / (m * 1e9)) == bits(float(Fraction(n, m * 10 ** 9)))
    assert bits(s6["odr_error"]) == bits(gic.imu_config(PROFILE)[0]["odr_error"])
    assert s6["gyro_source"] == "none" and s6["rotor_speed"] == 0 and s6["seed"] == SEED
    assert s6["imu_stream_id"] == 0 and s6["imu_counter_base"] == 0 and s6["imu"]["latency_samples"] == 0
    assert Fraction(m * NUM_US * 1000, DEN * n) - 1 == e_r
    if corner:
        assert round(float(e_r) * 1e6, 6) == PPM[m][corner]
    else:
        assert bits(s6["tick_s"]) == bits(NUM_US / (DEN * 1e6))  # c = 0: the nominal tick, bitwise


@pytest.mark.parametrize("m", sorted(TABLE))
@pytest.mark.parametrize("corner", (None, -1, 0, 1))
def test_the_sil_stamps_stay_nominal(runs, m, corner):
    _, d = runs[(m, corner)]
    assert [t["sil_t_us"] for t in d["ticks"]] == [j * NUM_US // DEN for j in range(ITERATIONS * m)]


@pytest.mark.parametrize("m", sorted(TABLE))
def test_the_nominal_corner_log_is_the_default_log_without_record_6(runs, m):
    data0, d0 = runs[(m, 0)]
    default, dd = runs[(m, None)]
    assert dd["sensors"] is None
    assert without_record_6(data0) == default


@pytest.mark.parametrize("m", sorted(TABLE))
def test_control_a_nonzero_corner_log_is_not_the_default(runs, m):
    assert without_record_6(runs[(m, 1)][0]) != runs[(m, None)][0]
    assert runs[(m, 1)][1]["ticks"][-1]["rotor_speed"] != runs[(m, None)][1]["ticks"][-1]["rotor_speed"]


def step_edit(n):
    def edit(text):
        out, k = re.subn(r"(<max_step_size>)[^<]*", rf"\g<1>{float(Fraction(n, 10 ** 9))!r}", text, count=1)
        assert k == 1
        return out
    return edit


@pytest.mark.parametrize("m", sorted(TABLE))
@pytest.mark.parametrize("which", ("nearest", "nominal"))
def test_control_the_nearest_or_the_nominal_step_at_c_plus_1_is_refused(tmp_path_factory, m, which):
    n = NEAREST[m] if which == "nearest" else TABLE[m][0][0]
    r, log = run(tmp_path_factory, m, 1, step_edit(n))
    assert r.returncode != 0
    assert (f"marv_gz_lockstep: REFUSED: physics max_step_size is not the realised host step n_true = {TABLE[m][1][0]} ns"
            in r.stderr), run_scenario.stderr_tail(r.stderr)
    assert not log.exists()


# ---- the rounded host step (decision 0046 item 1) ---------------------------------------------------------------------

FC3_NUM_US, FC3_DEN = 1953125, 14649  # the board's tick period, us (fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml)
ROUNDED = {1: 133328, 2: 266656}  # round(m * 1953125000 / 14649) ns
ROUNDED_PPM = -1.601536  # (n - m tick) / (m tick) * 1e6 at m = 1 and m = 2, exact rationals -3128/1953125000 and -6256/3906250000


def fc3_scenario(tmp):
    text = HOVER.read_text(encoding="utf-8")
    text, k1 = re.subn(r"(tick_period_num_us:\n  value: )625", rf"\g<1>{FC3_NUM_US}", text)
    text, k2 = re.subn(r"(tick_period_den:\n  value: )4\n", rf"\g<1>{FC3_DEN}\n", text)
    assert (k1, k2) == (1, 1)
    path = tmp / "hover.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def run_rounded(tmp_path_factory, m, edit=None):
    tmp = tmp_path_factory.mktemp(f"rounded_m{m}")
    log = tmp / "run.bin"
    text = gen_world.generate(CARD, fc3_scenario(tmp), "test", m, "lo", str(log))[1]
    if edit is not None:
        text = edit(text)
    sdf = tmp / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    return run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, None, TIMEOUT_S), log


def odr_edit(value):
    def edit(text):
        out, k = re.subn(r"(<odr_error>)[^<]*", rf"\g<1>{value!r}", text, count=1)
        assert k == 1
        return out
    return edit


@pytest.fixture(scope="module")
def rounded_runs(tmp_path_factory):
    out = {}
    for m in ROUNDED:
        r, log = run_rounded(tmp_path_factory, m)
        assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
        out[m] = lockstep_log.read(log)
    return out


@pytest.mark.parametrize("m", sorted(ROUNDED))
def test_the_host_step_is_the_rounded_step_and_the_logs_state_the_error(rounded_runs, m):
    d = rounded_runs[m]
    n = ROUNDED[m]
    assert [s["sim_time_ns"] for s in d["steps"]] == [(i + 1) * n for i in range(ITERATIONS)]
    s6 = d["sensors"]
    assert s6["host_step_ns"] == n and s6["clock"] == 0 and s6["clock_corner"] == 0 and s6["gyro_source"] == "none"
    assert bits(s6["tick_s"]) == bits(n / (m * 1e9)) == bits(float(Fraction(n, m * 10 ** 9)))  # the plant's t_true
    assert bits(s6["odr_error"]) == bits(gic.imu_config(PROFILE)[0]["odr_error"])
    r9 = d["rounding"]
    assert bits(r9["exact_host_step_ns"]) == bits(m * FC3_NUM_US * 1000 / FC3_DEN)
    assert round(r9["error_ppm"], 6) == ROUNDED_PPM
    assert abs(r9["error_ppm"] * 1e-6) <= s6["odr_error"]
    assert Fraction(n * FC3_DEN, m * FC3_NUM_US * 1000) - 1 == Fraction(-3128 * m, m * 1953125000)


@pytest.mark.parametrize("m", sorted(ROUNDED))
def test_the_sil_stamps_stay_the_exact_rational(rounded_runs, m):
    d = rounded_runs[m]
    assert (d["header"]["tick_period_num_us"], d["header"]["tick_period_den"]) == (FC3_NUM_US, FC3_DEN)
    assert [t["sil_t_us"] for t in d["ticks"]] == [j * FC3_NUM_US // FC3_DEN for j in range(ITERATIONS * m)]


@pytest.mark.parametrize("m", sorted(TABLE))
def test_a_whole_ns_tick_with_an_odr_error_element_alone_is_the_default_log_byte_for_byte(runs, tmp_path_factory, m):
    def add_odr_error(text):
        out, k = re.subn(r"(\n\s*)(<seed>)", rf"\g<1><odr_error>{gic.imu_config(PROFILE)[0]['odr_error']!r}</odr_error>\g<1>\g<2>",
                         text, count=1)
        assert k == 1
        return out
    r, log = run(tmp_path_factory, m, None, add_odr_error)
    assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
    default = runs[(m, None)][0]
    assert log.read_bytes() == default
    assert lockstep_log.read(log)["sensors"] is None and lockstep_log.read(log)["rounding"] is None


def test_control_a_bound_under_the_rounding_error_is_refused_and_one_over_it_runs(tmp_path_factory):
    r, log = run_rounded(tmp_path_factory, 1, odr_edit(1.6e-6))
    assert r.returncode != 0
    assert ("marv_gz_lockstep: REFUSED: the host step 133328 ns rounds m*tick by -1.601536 ppm, above the profile's "
            "odr_error 1.600000 ppm") in r.stderr, run_scenario.stderr_tail(r.stderr)
    assert not log.exists()
    r, log = run_rounded(tmp_path_factory, 1, odr_edit(1.61e-6))
    assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
    assert lockstep_log.read(log)["rounding"] is not None


def test_control_a_rounded_tick_without_odr_error_is_refused(tmp_path_factory):
    r, log = run_rounded(tmp_path_factory, 1, lambda t: re.sub(r"\s*<odr_error>[^<]*</odr_error>", "", t, count=1))
    assert r.returncode != 0
    assert "marv_gz_lockstep: REFUSED: tick_ns = num*1000/den is not an integer and <odr_error>" in r.stderr, \
        run_scenario.stderr_tail(r.stderr)
    assert not log.exists()


@pytest.mark.parametrize("n", (ROUNDED[1] + 1, ROUNDED[1] - 1))
def test_control_a_step_of_the_next_ns_is_refused(tmp_path_factory, n):
    r, log = run_rounded(tmp_path_factory, 1, step_edit(n))
    assert r.returncode != 0
    assert "marv_gz_lockstep: REFUSED: physics max_step_size is not the rounded host step 133328 ns" in r.stderr, \
        run_scenario.stderr_tail(r.stderr)
    assert not log.exists()
