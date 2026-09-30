"""SIM-2 determinism of the gz-sim 8 lockstep plugin (quad spec 3.3 SIM-2 and 4 L2, docs/decisions/0003 item 10).

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz build of libmarv_gz_lockstep.so is absent
(the gz CI step fails on a skip). Each run is a separate gz process from tools/sim/run_scenario.py: the determinism scenario,
seed 1, m = 1, its full 6400 host steps, test mode.

  test_two_runs_are_byte_identical   two processes, the same world: the plugin logs are equal byte for byte (both SHA-256
                                     are printed).
  test_control_differs_in_gz_state   negative control: the second world has the SDF ixx one ulp higher. Its log must
                                     differ from the first, and the first differing byte must lie in a STEP record's gz
                                     read (position, attitude or velocity fields), not in the header.
  test_control_rerun_is_identical    two processes on the control world give byte-identical logs, so the control's
                                     difference is the ixx edit and not run-to-run noise.

The log holds no wall-clock, host or path bytes (lockstep_log.py), so the four runs, in four different directories,
are comparable byte for byte.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L02" / "determinism.yaml"
PLUGIN_DIR = run_scenario.DEFAULT_PLUGIN_DIR
SEED = 1
M = 1
GZ_STATE_FIELDS = ("gz_pos_enu", "gz_q_wxyz", "gz_lin_vel_enu", "gz_ang_vel_enu")

pytestmark = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def _run(tmp_path_factory, name, sdf_edit=None):
    return run_scenario.run(CARD, SCENARIO, SEED, M, "test", None, tmp_path_factory.mktemp(name), PLUGIN_DIR, sdf_edit)


@pytest.fixture(scope="module")
def plain(tmp_path_factory):
    return _run(tmp_path_factory, "a"), _run(tmp_path_factory, "b")


@pytest.fixture(scope="module")
def control(tmp_path_factory):
    return (_run(tmp_path_factory, "c1", run_scenario.ixx_one_ulp),
            _run(tmp_path_factory, "c2", run_scenario.ixx_one_ulp))


def _bytes(r):
    return Path(r.log_path).read_bytes()


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


def test_two_runs_are_byte_identical(plain, capsys):
    a, b = plain
    _say(capsys, f"SIM-2 run A sha256 {a.log_sha256} ({len(_bytes(a))} bytes)",
         f"SIM-2 run B sha256 {b.log_sha256} ({len(_bytes(b))} bytes)")
    assert a.log_path != b.log_path
    for r in plain:
        assert r.iterations == 6400 and len(r.log["steps"]) == 6400
        assert r.log["trailer"] == {"steps": 6400, "ticks": 6400, "applied": 6400}
    assert _bytes(a) == _bytes(b), f"logs differ at byte {run_scenario.first_difference(_bytes(a), _bytes(b))}"
    assert a.log_sha256 == b.log_sha256


def test_control_differs_in_gz_state(plain, control, capsys):
    a, _ = plain
    c1, _ = control
    assert c1.sdf_edited and not a.sdf_edited
    da, dc = _bytes(a), _bytes(c1)
    off = run_scenario.first_difference(da, dc)
    assert off is not None, "the ixx control produced a byte-identical log: the determinism check has no teeth"
    where = run_scenario.locate_offset(da, off)
    _say(capsys, f"SIM-2 control (ixx one ulp higher) sha256 {c1.log_sha256}",
         f"SIM-2 control first differing byte: offset {off} (header is {a.log['header']['header_size']} bytes), {where}")
    assert a.log_sha256 != c1.log_sha256
    assert off >= a.log["header"]["header_size"], "the header differs"
    assert where["region"] == "STEP", where
    assert where["field"].split("[")[0] in GZ_STATE_FIELDS, where
    assert lockstep_log.read(c1.log_path)["header"] == a.log["header"]


def test_control_rerun_is_identical(control, capsys):
    c1, c2 = control
    _say(capsys, f"SIM-2 control run 1 sha256 {c1.log_sha256}", f"SIM-2 control run 2 sha256 {c2.log_sha256}")
    assert c1.log_path != c2.log_path
    assert _bytes(c1) == _bytes(c2), f"control logs differ at byte {run_scenario.first_difference(_bytes(c1), _bytes(c2))}"
    assert c1.log_sha256 == c2.log_sha256
