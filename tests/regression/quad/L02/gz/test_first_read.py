"""The first state read of the gz-sim 8 lockstep plugin carries the scenario's starting body rates (quad spec 3.1, docs/decisions/0016,
0003 item 11).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so
(`cmake --preset host-gz && cmake --build --preset host-gz`). Every gz process gets its own GZ_PARTITION and
GZ_IP=127.0.0.1. The world comes from tools/card/gen_world.py (rotation scenario, m = 1, the world of the smoke test).

Cause. PreUpdate reads before gz's physics step, and gz-sim 8.15 has no initial velocity, so the plugin applies the
scenario's initial rates (Link::SetAngularVelocity) at the end of the step-0 PreUpdate. The step-0 read of gz is therefore
exactly zero (0003 item 11) and the SIL's gyro and the TRUTH state would see a step from zero to the starting rates at
the next execution. Physically the body moves from (q0, w0) at t = 0, so the plugin feeds step 0 the scenario's rates.

What is checked: (a) gz's raw step-0 angular velocity (gz_ang_vel_enu) is still exactly zero, which documents the cause;
(b) the logged plant body rates at step 0 (body_omega_frd, the bytes the plant and the SIL saw) are bitwise the
scenario's initial FRD rates; (c) only step 0 is changed: gz reads nonzero rates at step 1 and the step-1 logged rates
are not the scenario's bit pattern (they come from gz's read after one physics step). Negative control: MARV_GZ_TEST_ZERO_FIRST_READ=1 makes the plugin pass gz's
read through at step 0 (the old behaviour), and check (b) must then report mismatches.
"""

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
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
PLUGIN_DIR = ROOT / "build" / "host-gz" / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
ITERATIONS = 2  # steps 0 and 1 are the only ones read
TIMEOUT_S = 180  # the smoke test's gz process timeout
SEED = 1
ZERO_FIRST_READ = {"MARV_GZ_TEST_ZERO_FIRST_READ": "1"}  # negative control only: the plugin passes gz's step-0 read through

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def same(a, b):
    return struct.pack("<d", a) == struct.pack("<d", b)


def run(tmp_path_factory, extra_env=None):
    tmp = tmp_path_factory.mktemp("first_read")
    log = tmp / "run.bin"
    text = gen_world.generate(CARD, SCEN / "rotation.yaml", "test", 1, None, str(log))[1]
    sdf = tmp / "world.sdf"
    sdf.write_text(text, encoding="utf-8")
    r = run_scenario.run_gz_process(sdf, ITERATIONS, SEED, PLUGIN_DIR, extra_env, TIMEOUT_S)
    assert r.returncode == 0, r.stderr
    return lockstep_log.read(log)


def scenario_rates():
    st = schema.load_yaml(SCEN / "rotation.yaml")["initial_state"]
    return [float(c) for c in st["body_rates_frd_rad_s"]["value"]]


def first_read_mismatches(d, w0):
    """What differs between the logged step-0 plant body rates and the scenario's initial FRD rates; empty means bitwise equal."""
    return [k for k, (a, b) in enumerate(zip(d["steps"][0]["body_omega_frd"], w0)) if not same(a, b)]


@pytest.fixture(scope="module")
def first_read(tmp_path_factory):
    return run(tmp_path_factory)


@pytest.fixture(scope="module")
def first_read_zero(tmp_path_factory):
    return run(tmp_path_factory, ZERO_FIRST_READ)


def test_scenario_has_nonzero_rates():
    assert all(c != 0.0 for c in scenario_rates())


def test_gz_step_zero_read_is_zero(first_read):
    assert all(c == 0.0 for c in first_read["steps"][0]["gz_ang_vel_enu"])


def test_step_zero_plant_rates_are_the_scenarios(first_read):
    assert first_read_mismatches(first_read, scenario_rates()) == []


def test_step_one_is_not_overridden(first_read):
    w0 = scenario_rates()
    assert first_read_mismatches({"steps": [first_read["steps"][1]]}, w0) != []
    assert any(c != 0.0 for c in first_read["steps"][1]["gz_ang_vel_enu"])


def test_first_read_check_negative_control(first_read_zero):
    """With gz's read passed through at step 0 the plant rates are zero and the check must report mismatches."""
    assert all(c == 0.0 for c in first_read_zero["steps"][0]["body_omega_frd"])
    assert first_read_mismatches(first_read_zero, scenario_rates()) != []
