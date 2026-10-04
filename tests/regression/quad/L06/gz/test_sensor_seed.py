"""Seed determinism of the plugin's IMU model run (decision 0019, W1 C1, test c; SIM-2 with the gz plugin).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so. Every gz process
gets its own GZ_PARTITION and GZ_IP=127.0.0.1. The world is the L2 hover world (member lo, m = 1) from
tools/card/gen_world.py with SensorSet(gyro="model", rotor=True, signs, clock_corner=+1): the SIL gets the plant IMU
model's bytes and the rotor-speed samples, at a clock corner.

What is checked: two processes with the same seed and corner write identical files, byte for byte. With seed 2 the files
differ only where the seed reaches: the header's and record 6's seed fields, and then, first, inside tick 0's IMU bytes
(the IMU draws from noise stream 0 of the seed): every earlier byte is equal.

Negative control: the seed 2 run is that control for the equality (a different seed must change the file), and its IMU
bytes differ on most ticks.
"""

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
ITERATIONS = 512  # labelled harness value: host steps per run
TIMEOUT_S = 180
M = 1
SENSORS = gen_world.SensorSet(gyro="model", rotor=True, bias_signs=(1, -1, 0, -1, 1, 0), clock_corner=1)
SEED_OFFSET_HEADER = 8 + 4 * 5  # magic, version, header_size, m, num_us, den: lockstep_log._header's layout
SEED_OFFSET_SENSORS = 4 + 8 + 8 + 8  # four u8, odr_error, host_step_ns, tick_s: lockstep_log._SENSORS's layout
STEP_SIZE = 1 + lockstep_log._STEP.size
TICK_IMU = (1 + 8 + 8, 1 + 8 + 8 + 32)  # the TICK record's IMU bytes, after the type byte, tick and sil_t_us

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def run(tmp_path_factory, seed, tag):
    tmp = tmp_path_factory.mktemp(f"seed{seed}_{tag}")
    log = tmp / "run.bin"
    text = gen_world.generate(CARD, HOVER, "test", M, "lo", str(log), sensors=SENSORS)[1]
    text = run_scenario._with_seed(text, seed, 1)
    sdf = tmp / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, ITERATIONS, seed, PLUGIN_DIR, None, TIMEOUT_S)
    assert r.returncode == 0, run_scenario.stderr_tail(r.stderr)
    return log.read_bytes()


@pytest.fixture(scope="module")
def logs(tmp_path_factory):
    return {"1a": run(tmp_path_factory, 1, "a"), "1b": run(tmp_path_factory, 1, "b"), "2": run(tmp_path_factory, 2, "a")}


def test_same_seed_and_corner_give_identical_files(logs):
    assert logs["1a"] == logs["1b"]


def masked(data):
    """The file with the header's and record 6's seed fields zeroed; also the offset of tick 0's TICK record."""
    hs = lockstep_log._header(data)["header_size"]
    assert data[hs] == lockstep_log.SENSORS
    b = bytearray(data)
    b[SEED_OFFSET_HEADER:SEED_OFFSET_HEADER + 8] = bytes(8)
    s = hs + 1 + SEED_OFFSET_SENSORS
    b[s:s + 8] = bytes(8)
    tick0 = hs + 1 + lockstep_log._SENSORS.size + STEP_SIZE
    assert b[tick0] == lockstep_log.TICK
    return bytes(b), tick0


def test_a_different_seed_first_differs_inside_tick_zeros_imu_bytes(logs):
    a, b = logs["1a"], logs["2"]
    assert a != b
    first = run_scenario.first_difference(a, b)
    assert first == SEED_OFFSET_HEADER  # the header's seed field comes first
    ma, tick0 = masked(a)
    mb, _ = masked(b)
    first = run_scenario.first_difference(ma, mb)
    assert tick0 + TICK_IMU[0] <= first < tick0 + TICK_IMU[1], (first, tick0)


def test_control_the_imu_bytes_differ_on_most_ticks(logs, tmp_path):
    pa, pb = tmp_path / "a.bin", tmp_path / "b.bin"
    pa.write_bytes(logs["1a"])
    pb.write_bytes(logs["2"])
    a, b = lockstep_log.read(pa), lockstep_log.read(pb)
    differing = sum(x["imu"] != y["imu"] for x, y in zip(a["ticks"], b["ticks"]))
    assert differing > len(a["ticks"]) // 2
    assert a["sensors"]["seed"] == 1 and b["sensors"]["seed"] == 2
    assert {k: v for k, v in a["sensors"].items() if k not in ("seed", "raw")} == \
        {k: v for k, v in b["sensors"].items() if k not in ("seed", "raw")}
    assert len(a["rotors"]) == len(a["ticks"]) == ITERATIONS * M
