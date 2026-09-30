"""T4 truth plumbing of the L5 plugin (quad L5; decision 0006 B "Plugin" and "Log", F "T4 truth plumbing").

Skipped only as tests/regression/quad/L05/gz/conftest.py says. One short run of the l5_attitude_scripted composition on
the host-gz-l5 build from a NON-LEVEL initial attitude with a nonzero stick, so q and omega evolve
(a level, at-rest world would make every cast trivially equal). The scenario is the step_roll document with only those
values changed (written into the test's temporary directory).

Claim. For every tick the log holds one TRUTH record directly after the TICK record, and it equals the float cast of the body
state (q wxyz, omega FRD) of the host step the tick belongs to, held over that step's m ticks, bit for bit: q rounded to
binary32 and made canonical (w >= 0 after the cast), omega rounded to binary32; its flags are the valid bit only; its tick
fields are the tick.

Negative controls (each must fail the same check or be refused):
  (a) the expectation taken from the host step one earlier fails on a world whose state evolves (a held or late state);
  (b) the expectation left in binary64 (no float cast) fails;
  (c) the same world without <attitude_source>: the composition's attitude latch is never filled, the firmware panics at
      tick 0 (gz aborts, the log is incomplete) and no TRUTH record is written. The L5 composition cannot run without the
      element, so "the log is byte-identical to an L4 run" is checked by the L4 suites, not here;
  (d) the same world with <attitude_source> but without <gyro_source>: the plugin refuses it.
"""

import math
import struct
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import lockstep_log  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SOURCE = ROOT / "scenarios" / "quad" / "L05" / "step_roll.yaml"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
VALID = 1  # bit MARV_TRUTH_ATTITUDE_VALID (fw/sil/include/marv_truth.h)
TRUTH_STATE_SIZE = 48
M = 2
# Scenario values of this test (labelled here, not physics): an initial rotation of 2 rad about the axis (1, 2, 3)/sqrt(14)
# (canonical, w = cos 1 > 0, a non-level attitude) and a stick on every axis; the attitude loop's response to the
# attitude error moves every component of q and omega (the initial body rates stay 0: the L2 schema asks a separatrix
# check of nonzero rates), and a run of 400 attitude executions after the origin.
ANGLE_RAD = 2.0
AXIS = (1.0, 2.0, 3.0)
STICK = [0.5, -0.5, 0.3]
END = 400
GZ_TIMEOUT_S = 180
PANIC_TEXT = "the latched attitude stamp differs from the sample stamp"


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def scenario_file(tmp_path):
    doc = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    n = math.sqrt(sum(a * a for a in AXIS))
    q = [math.cos(ANGLE_RAD / 2)] + [math.sin(ANGLE_RAD / 2) * a / n for a in AXIS]
    doc["scenario"] = "plumbing"
    st = doc["initial_state"]
    st["attitude_q_wxyz"].update(value=q, rationale="a non-level canonical attitude (w = cos 1 > 0): 2 rad about (1, 2, 3)/sqrt(14)")
    seg = doc["script"]["segments"]
    del seg[1:]
    seg[0]["stick"].update(value=STICK, rationale="a stick on every axis so the commanded attitude moves")
    doc["script"]["end_attitude_execution"].update(value=END, label="scenario",
                                                   rationale="a short run: the plumbing check needs evolving state, not a settled step")
    doc["script"]["end_attitude_execution"].pop("rule", None)
    path = tmp_path / "plumbing.yaml"
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return path


def canonical_cast(q):
    c = [r32(x) for x in q]
    return [-x for x in c] if c[0] < 0 else c


def mismatches(d, m, shift=0, cast=True):
    """Ticks whose TRUTH record is not (bit for bit) the expected state of step tick // m + shift; also the structural
    failures (missing record, tick fields, flags, size)."""
    truths = {t["tick"]: t for t in d["truths"]}
    bad = []
    for t in d["ticks"]:
        tick = t["tick"]
        r = truths.get(tick)
        j = min(max(tick // m + shift, 0), len(d["steps"]) - 1)
        body = d["steps"][j]
        if cast:
            q, w = canonical_cast(body["body_q_wxyz"]), [r32(x) for x in body["body_omega_frd"]]
        else:
            q, w = list(body["body_q_wxyz"]), list(body["body_omega_frd"])
        ok = (r is not None and r["state_tick"] == tick and r["flags"] == VALID and r["struct_size"] == TRUTH_STATE_SIZE
              and list(r["q_wxyz"]) == q and list(r["omega_frd"]) == w)
        if not ok:
            bad.append(tick)
    return bad


@pytest.fixture(scope="module")
def flown(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("plumbing")
    s = run_l5.run_step(CARD, scenario_file(tmp), M, tmp / "run", PLUGIN_DIR, timeout_s=GZ_TIMEOUT_S)
    return s, tmp


def test_truth_record_is_the_float_cast_of_the_step_body_state_every_tick(flown):
    s, _ = flown
    d = s.run.log
    assert run_scenario.complete_trailer(d, s.run.iterations, M)
    assert len(d["truths"]) == len(d["ticks"]) == s.run.iterations * M
    assert s.plan.att_divisor % M == 0
    assert mismatches(d, M) == []


def test_the_world_evolves_and_exercises_the_cast(flown):
    s, _ = flown
    steps = s.run.log["steps"]
    for k in range(4):
        assert len({st["body_q_wxyz"][k] for st in steps}) > 1, f"q[{k}] never changes"
    for k in range(3):
        assert len({st["body_omega_frd"][k] for st in steps}) > 1, f"omega[{k}] never changes"
    assert any(r32(x) != x for st in steps for x in st["body_q_wxyz"]), "every q component is already binary32"
    assert s.run.log["truths"][0]["q_wxyz"][0] > 0.5, "the run does not start from the non-level canonical attitude"


@pytest.mark.parametrize("shift", (-1, 1))
def test_control_expectation_from_another_step_fails(flown, shift):
    assert mismatches(flown[0].run.log, M, shift=shift) != []


def test_control_expectation_without_the_float_cast_fails(flown):
    d = flown[0].run.log
    assert len(mismatches(d, M, cast=False)) > len(d["ticks"]) // 2


def test_control_world_without_the_attitude_element_fails_at_tick_zero(flown, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("no_attitude")
    with pytest.raises(run_scenario.RunError, match="exited"):
        run_l5.run_step(CARD, scenario_file(tmp), M, tmp / "run", PLUGIN_DIR, timeout_s=GZ_TIMEOUT_S,
                        attitude_source=False)
    world = next((tmp / "run").glob("*.sdf"))
    assert "<attitude_source>" not in world.read_text(encoding="utf-8")
    r = run_scenario.run_gz_process(world, 4, 1, PLUGIN_DIR, None, GZ_TIMEOUT_S)
    assert r.returncode != 0 and PANIC_TEXT in r.stderr, run_scenario.stderr_tail(r.stderr)
    try:
        log = lockstep_log.read(next((tmp / "run").glob("*.bin")))
    except lockstep_log.LogError:
        return
    assert log["truths"] == [] and log["trailer"] is None


def test_control_attitude_source_without_gyro_source_is_refused(flown, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("no_gyro")
    with pytest.raises(run_scenario.RunError, match="the plugin refused the world"):
        run_l5.run_step(CARD, scenario_file(tmp), M, tmp / "run", PLUGIN_DIR, timeout_s=GZ_TIMEOUT_S, gyro_source=False)
