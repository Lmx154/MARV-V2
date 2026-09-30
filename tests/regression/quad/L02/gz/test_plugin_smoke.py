"""Smoke tests of the gz-sim 8 lockstep plugin (sim/gz/plugin, quad spec 3.1, docs/decisions/0003 items 5, 6 and 10).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz build holds libmarv_gz_lockstep.so
(`cmake --preset host-gz && cmake --build --preset host-gz`). Every gz process gets its own GZ_PARTITION and
GZ_IP=127.0.0.1. Worlds come from tools/card/gen_world.py; the refusal worlds are that text with one edit.

What is checked: the determinism world runs 64 host steps and its log parses to 64 step and 64 tick records with the
overridden DShot values; each planted world is refused (non-zero exit, `marv_gz_lockstep: REFUSED: <reason>` on stderr,
no log written); and the state of step 0.

Initial velocity. The Physics system keeps a link velocity command component and resets it to zero rather than
removing it (measured, gz-sim 8.15), so the plugin removes the LinearVelocityCmd and AngularVelocityCmd components at the
PreUpdate after the one that set them. Two checks show the velocity is not pinned afterwards. Rotation world, m = 1: the
angular speed |w| of the torque-free body stays within the bound derived at omega_norm_mismatches. Determinism world:
the linear velocity evolves as v_i = v_1 + sum (F_bar_j / m) dt, within the rounding bound derived at
velocity_evolution_mismatches. Negative control: MARV_GZ_TEST_KEEP_VEL_CMD=1 makes the plugin skip the removal, and
both checks must then fail.

Step 0. The position is bitwise the scenario's after the exact map. The attitude is within the round-trip bound of
0003 item 6, (3 + 3 sqrt 2) u ||q|| per component, plus u ||q|| for gz normalising the quaternion it parses. The
velocity cannot be bitwise at step 0: PreUpdate reads before gz's physics step, and gz-sim 8.15 has no initial
velocity, so the plugin applies it (Link::SetLinearVelocity, whose vector is in the LINK frame, measured) in the same
PreUpdate. What the test checks is (a) the step-0 read is exactly zero, and (b) the step-1 read is the scenario's
velocity plus the applied wrench's acceleration over one step, v1 = v0_enu + (F_bar / m) dt, within the rounding
bound 2 (5 sqrt 3 + 3) u ||v0|| + 4 u ||v1|| (one rotation into the link frame by the plugin and one back by gz, each
(5 sqrt 3 + 3) u ||v|| per component, frames.hpp; the velocity update is a product and a sum).
"""

import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
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
U = 2.0 ** -53
ITERATIONS = 64
TIMEOUT_S = 180
SEED = 1


pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz build of libmarv_gz_lockstep.so is absent")


def same(a, b):
    return struct.pack("<d", a) == struct.pack("<d", b)


def world(scenario, log_path, mode="test", m=1):
    return gen_world.generate(CARD, SCEN / f"{scenario}.yaml", mode, m, None, str(log_path))[1]


def run_gz(sdf_text, tmp_path, iterations=ITERATIONS, extra_env=None):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(sdf_text, encoding="utf-8")
    return run_scenario.run_gz_process(sdf, iterations, SEED, PLUGIN_DIR, extra_env, TIMEOUT_S)


@pytest.fixture(scope="module")
def determinism(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("determinism")
    log = tmp / "run.bin"
    r = run_gz(world("determinism", log), tmp)
    return r, log


def test_determinism_world_runs_and_logs(determinism):
    r, log = determinism
    assert r.returncode == 0, r.stderr
    assert "REFUSED" not in r.stderr
    d = lockstep_log.read(log)
    assert d["header"]["m"] == 1 and d["header"]["tick_period_num_us"] == 625 and d["header"]["tick_period_den"] == 4
    assert d["header"]["seed"] == 1 and d["header"]["gz_sim_version"].startswith("8.")
    assert len(d["steps"]) == ITERATIONS and len(d["ticks"]) == ITERATIONS and len(d["applied"]) == ITERATIONS
    assert d["trailer"] == {"steps": ITERATIONS, "ticks": ITERATIONS, "applied": ITERATIONS}
    assert [t["tick"] for t in d["ticks"]] == list(range(ITERATIONS))
    assert all(t["dshot"] == (1100, 700, 900, 800) for t in d["ticks"])


def _mutations():
    def gravity(t):
        return t.replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>")

    def inertial_pose(t):
        return re.sub(r"(<inertial>\s*<pose>)[^<]*", r"\g<1>0 0 0.001 0 0 0", t, count=1)

    def link_origin(t):
        return re.sub(r'(<link name="base_link">\s*<pose>)[^<]*', r"\g<1>0.001 0 0 0 0 0", t, count=1)

    def mass_ulp(t):
        return re.sub(r"(<inertial>.*?<mass>)([^<]*)",
                      lambda mo: mo.group(1) + repr(math.nextafter(float(mo.group(2)), math.inf)), t, count=1,
                      flags=re.S)

    def product_of_inertia(t):
        return re.sub(r"<ixy>[^<]*", "<ixy>1e-9", t, count=1)

    def duplicate_plugin(t):
        block = re.search(r'\s*<plugin filename="marv_gz_lockstep".*?</plugin>', t, flags=re.S).group(0)
        return t.replace(block, block + block, 1)

    def step_off_by_one_tick_ns(t):
        tick_ns = 625 * 1000 // 4
        return re.sub(r"(<max_step_size>)[^<]*", rf"\g<1>{repr((tick_ns + 1) / 1e9)}", t, count=1)

    return [
        ("gravity", gravity, "world gravity is not zero"),
        ("inertial_pose", inertial_pose, "the link's inertial pose is not identity"),
        ("link_origin", link_origin, "the link origin is not the model frame"),
        ("mass_one_ulp", mass_ulp, "SDF mass is not the plant mass_kg"),
        ("product_of_inertia", product_of_inertia, "SDF inertia has nonzero products of inertia"),
        ("duplicate_plugin", duplicate_plugin, "a second marv::gz::Lockstep instance"),
        ("max_step_size_one_tick_ns", step_off_by_one_tick_ns, "physics max_step_size is not m*tick"),
    ]


@pytest.mark.parametrize("name,edit,reason", _mutations(), ids=[m[0] for m in _mutations()])
def test_planted_world_is_refused(tmp_path, name, edit, reason):
    log = tmp_path / "run.bin"
    good = world("determinism", log)
    planted = edit(good)
    assert planted != good, "the planted edit did not change the world"
    r = run_gz(planted, tmp_path)
    assert r.returncode != 0, "the run was not refused"
    assert f"marv_gz_lockstep: REFUSED: {reason}" in r.stderr, r.stderr
    assert not log.exists(), "a refused run wrote a log"


def ned_to_enu(v):
    return (v[1], v[0], -v[2])


def step_zero_mismatches(d, mass, pos_ned, q_ned, vel_ned):
    """What differs between the log's steps 0 and 1 and the scenario's initial state; empty means they agree."""
    bad = []
    s0, s1 = d["steps"][0], d["steps"][1]
    pos = ned_to_enu(pos_ned)
    if not all(same(a, b) for a, b in zip(s0["gz_pos_enu"], pos)):
        bad.append("gz position at step 0 is not bitwise the scenario's after the exact map")
    if not all(same(a, b) for a, b in zip(s0["body_pos_ned"], pos_ned)):
        bad.append("plant body position at step 0 is not bitwise the scenario's")
    norm = math.sqrt(sum(c * c for c in q_ned))
    got = list(s0["body_q_wxyz"])
    if sum(a * b for a, b in zip(got, q_ned)) < 0:
        got = [-c for c in got]
    bound = ((3 + 3 * math.sqrt(2)) + 1) * U * norm
    if not all(abs(a - b) <= bound for a, b in zip(got, q_ned)):
        bad.append("plant body attitude at step 0 is outside the 0003 item 6 round-trip bound")
    if any(c != 0.0 for c in s0["gz_lin_vel_enu"]):
        bad.append("gz reads a velocity at step 0 (the recorded finding has changed)")
    v0 = ned_to_enu(vel_ned)
    dt = 625 * 1e-6 / 4
    f = d["applied"][0]["force_enu"]
    v1 = s1["gz_lin_vel_enu"]
    n0 = math.sqrt(sum(c * c for c in v0))
    n1 = math.sqrt(sum(c * c for c in v1))
    bound_v = 2 * (5 * math.sqrt(3) + 3) * U * n0 + 4 * U * n1
    if not all(abs(a - (b + fc / mass * dt)) <= bound_v for a, b, fc in zip(v1, v0, f)):
        bad.append("gz velocity at step 1 is not v0 + F_bar/m dt within the rounding bound")
    return bad


def _step_zero_inputs(log):
    doc = schema.load_yaml(SCEN / "determinism.yaml")
    st = doc["initial_state"]
    mass = float(ET.fromstring(world("determinism", "x")).find("world/model/plugin/mass_kg").text)
    return (lockstep_log.read(log), mass, [float(c) for c in st["position_ned_m"]["value"]],
            [float(c) for c in st["attitude_q_wxyz"]["value"]], [float(c) for c in st["velocity_ned_m_s"]["value"]])


def test_step_zero_state_is_the_scenarios(determinism):
    r, log = determinism
    assert r.returncode == 0, r.stderr
    assert step_zero_mismatches(*_step_zero_inputs(log)) == []


@pytest.mark.parametrize("what", ["position axes", "attitude 1e-6", "velocity 1e-6"])
def test_step_zero_check_has_teeth(determinism, what):
    """Negative control: a scenario state that differs in one field must be reported."""
    _, log = determinism
    d, mass, pos, q, vel = _step_zero_inputs(log)
    if what == "position axes":
        pos = [pos[1], pos[0], pos[2]]
    elif what == "attitude 1e-6":
        q = [q[0] + 1e-6] + q[1:]
    else:
        vel = [vel[0] + 1e-6] + vel[1:]
    assert step_zero_mismatches(d, mass, pos, q, vel) != []


ixx_one_ulp = run_scenario.ixx_one_ulp  # the determinism control of 0003 item 10; not a refusal (no reference inertia)


def test_ixx_one_ulp_world_runs(tmp_path):
    log = tmp_path / "run.bin"
    good = world("determinism", log)
    planted = ixx_one_ulp(good)
    assert planted != good, "the edit did not change the world"
    r = run_gz(planted, tmp_path)
    assert r.returncode == 0, r.stderr
    assert "REFUSED" not in r.stderr
    d = lockstep_log.read(log)
    assert len(d["steps"]) == ITERATIONS and d["trailer"] == {"steps": ITERATIONS, "ticks": ITERATIONS, "applied": ITERATIONS}


KEEP = {"MARV_GZ_TEST_KEEP_VEL_CMD": "1"}  # negative control only: the plugin skips the removal of the velocity commands


def _run_scenario(tmp_path_factory, scenario, extra_env=None):
    tmp = tmp_path_factory.mktemp(scenario)
    log = tmp / "run.bin"
    text = world(scenario, log)
    r = run_gz(text, tmp, extra_env=extra_env)
    assert r.returncode == 0, r.stderr
    root = ET.fromstring(text)
    mass = float(root.find("world/model/plugin/mass_kg").text)
    inertia = [float(root.find(f"world/model/link/inertial/inertia/{a}").text) for a in ("ixx", "iyy", "izz")]
    return lockstep_log.read(log), mass, inertia


@pytest.fixture(scope="module")
def rotation_run(tmp_path_factory):
    return _run_scenario(tmp_path_factory, "rotation")


@pytest.fixture(scope="module")
def rotation_keep(tmp_path_factory):
    return _run_scenario(tmp_path_factory, "rotation", KEEP)


@pytest.fixture(scope="module")
def determinism_keep(tmp_path_factory):
    return _run_scenario(tmp_path_factory, "determinism", KEEP)


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def omega_norm_mismatches(d, inertia):
    """Steps whose angular speed is outside the bound of torque-free motion around the step-1 value.

    Euler's equations without torque, I w' = -w x I w, give |w'| <= |I^-1| |w| |I w| <= kappa |w|^2 with kappa =
    I_max / I_min, hence d|w|/dt in [-kappa |w|^2, kappa |w|^2]. Integrating d(1/|w|)/dt = -d|w|/dt / |w|^2, which lies
    in [-kappa, kappa], gives |w_1| / (1 + x) <= |w_i| <= |w_1| / (1 - x) with x = kappa |w_1| (i - 1) dt, for x < 1.
    The bound is that of the exact dynamics; the integrator's own error is far below it (the margin is checked by the
    run below staying inside, and a pinned velocity leaves it by orders of magnitude). Step 0 is read before the
    command applies, so the reference is step 1."""
    dt = 625 * 1e-6 / 4
    kappa = max(inertia) / min(inertia)
    w1 = norm(d["steps"][1]["gz_ang_vel_enu"])
    bad = []
    for i in range(1, len(d["steps"])):
        x = kappa * w1 * (i - 1) * dt
        assert x < 1
        wi = norm(d["steps"][i]["gz_ang_vel_enu"])
        if not (w1 / (1 + x) <= wi <= w1 / (1 - x)):
            bad.append(i)
    return bad


def velocity_evolution_mismatches(d, mass):
    """Steps whose gz linear velocity is not v_1 + sum_{j=1}^{i-1} (F_bar_j / m) dt within the rounding bound.

    The read at step i is the state after the physics step that applied the wrench of step i - 1, so step 1 holds
    the first wrench and v_i = v_1 + sum_{j=1..i-1} F_bar_j dt / m. Each accumulated step costs the rotation of
    the velocity into the body frame and back by gz (each (5 sqrt 3 + 3) u |v| per component, frames.hpp), one
    product and one sum in gz (2 u |v| each) and the same product and sum in the expectation: (2 (5 sqrt 3 + 3) + 4 + 4) u
    |v|max per step, times the number of steps (errors add at most linearly). The Coriolis term of a rotating body
    is not rounding, but at the small body rates of this world it is far below the bound (observed worst deviation about
    1e-15 against the bound). A pinned velocity deviates by the whole accumulated velocity change, about 1 m/s."""
    dt = 625 * 1e-6 / 4
    steps, applied = d["steps"], d["applied"]
    vmax = max(norm(s["gz_lin_vel_enu"]) for s in steps)
    per_step = (2 * (5 * math.sqrt(3) + 3) + 8) * U * vmax
    expected = list(steps[1]["gz_lin_vel_enu"])
    bad = []
    for i in range(2, len(steps)):
        expected = [e + f / mass * dt for e, f in zip(expected, applied[i - 1]["force_enu"])]
        if any(abs(a - e) > (i - 1) * per_step for a, e in zip(steps[i]["gz_lin_vel_enu"], expected)):
            bad.append(i)
    return bad


def test_rotation_speed_does_not_decay(rotation_run):
    d, _, inertia = rotation_run
    assert norm(d["steps"][1]["gz_ang_vel_enu"]) > 1.0
    assert omega_norm_mismatches(d, inertia) == []


def test_determinism_velocity_evolves(determinism):
    r, log = determinism
    assert r.returncode == 0, r.stderr
    d, mass, *_ = _step_zero_inputs(log)
    assert norm(d["steps"][1]["gz_lin_vel_enu"]) > 0.5
    assert velocity_evolution_mismatches(d, mass) == []


def test_velocity_check_negative_control_rotation(rotation_keep):
    """With the removal skipped the command pins the angular velocity to zero and the check must fail."""
    d, _, inertia = rotation_keep
    assert omega_norm_mismatches(d, inertia) != []


def test_velocity_check_negative_control_determinism(determinism_keep):
    """With the removal skipped the command pins the linear velocity at v_1 and the check must fail."""
    d, mass, _ = determinism_keep
    assert velocity_evolution_mismatches(d, mass) != []
