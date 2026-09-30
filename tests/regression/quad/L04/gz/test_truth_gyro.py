"""The gz-sim 8 lockstep plugin with the optional <gyro_source>truth</gyro_source> element (quad L4, sim/gz/plugin).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so (as
tests/regression/quad/L02/gz). One short run on the L02 rotation world (a nonzero, evolving body rate), m = 4 so that one
sample is held over the ticks of a host step, with the element inserted. The log's TICK IMU bytes must be the sample
passed to the SIL: gyro = the float cast of the STEP record's body omega FRD (of the step the tick belongs to), flags
exactly GyroValid, accel and temperature zero.

Negative control: the same world without the element logs the zeroed sample (L2 behaviour), which fails the same check.
"""

import re
import struct
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
SCENARIO = ROOT / "scenarios" / "quad" / "L02" / "rotation.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz" / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
ITERATIONS = 16
M = 4
SEED = 1
TIMEOUT_S = 180
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
ELEMENT = "<gyro_source>truth</gyro_source>"

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def run(tmp_path, with_element):
    log = tmp_path / "run.bin"
    text = gen_world.generate(CARD, SCENARIO, "test", M, None, str(log))[1]
    if with_element:
        block = re.search(r'<plugin filename="marv_gz_lockstep".*?</plugin>', text, flags=re.S).group(0)
        text = text.replace(block, block.replace("</plugin>", f"  {ELEMENT}\n    </plugin>"), 1)
        assert text.count(ELEMENT) == 1
    sdf = tmp_path / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, None, TIMEOUT_S)
    assert r.returncode == 0, r.stderr
    assert "REFUSED" not in r.stderr
    return lockstep_log.read(log)


def f32_bits(x):
    return struct.pack("<f", x)


def imu_mismatches(d):
    """Ticks whose logged IMU is not the truth-gyro sample of the tick's step; empty means every tick agrees."""
    bad = []
    for t in d["ticks"]:
        step = d["steps"][t["tick"] // M]
        gx, gy, gz, ax, ay, az, temp, flags = struct.unpack("<7fI", t["imu"])
        want = [struct.unpack("<f", f32_bits(w))[0] for w in step["body_omega_frd"]]
        same = all(f32_bits(a) == f32_bits(b) for a, b in zip((gx, gy, gz), want))
        clean = (ax, ay, az, temp) == (0.0, 0.0, 0.0, 0.0) and flags == 1 << GYRO_VALID_BIT
        if not (same and clean):
            bad.append(t["tick"])
    return bad


def test_logged_imu_is_the_float_cast_of_the_step_body_rate(tmp_path):
    d = run(tmp_path, True)
    assert len(d["steps"]) == ITERATIONS and len(d["ticks"]) == ITERATIONS * M
    assert any(any(w != 0.0 for w in s["body_omega_frd"]) for s in d["steps"]), "the world has no body rate"
    assert imu_mismatches(d) == []


def test_negative_control_without_the_element_the_zero_sample_fails_the_check(tmp_path):
    d = run(tmp_path, False)
    assert len(d["ticks"]) == ITERATIONS * M
    assert all(t["imu"] == bytes(len(t["imu"])) for t in d["ticks"])
    assert imu_mismatches(d) == [t["tick"] for t in d["ticks"]]
