"""World generator (tools/card/gen_world.py). L02 tests.

The round trip reads every generated value back from the SDF with xml.etree and compares it bit-exactly (struct-packed
binary64) with the value read from the card and scenario YAML by literal path, independently of the generator. The
attitude is compared under the rounding bound of docs/decisions/0003 item 6 against an independent Hamilton-product
route, and by rotating vectors. check_world() is the round trip; each negative control mutates one generated value and
must make it report a mismatch.
"""

import math
import re
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import gen_sdf  # noqa: E402
import gen_world  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
VEHICLE = "uzh_neurobem_5in"
U = 2.0 ** -53
MOTORS = ("m1", "m2", "m3", "m4")
CASES = [(sc, hv, mode, m)
         for sc, hv, ms in (("free_fall", None, (4, 2, 1)), ("rotation", None, (4, 2, 1)),
                            ("hover", "lo", (4, 2, 1)), ("hover", "hi", (4, 2, 1)), ("determinism", None, (1,)))
         for mode in ("pilot", "test") for m in ms]
CARD_SCALARS = (
    ("mass_kg", ("mass", "value")), ("thrust_coeff", ("rotors", "thrust_coeff", "value")),
    ("torque_ratio_m", ("rotors", "torque_ratio", "value")), ("motor_tau_s", ("rotors", "motor_lag", "tau", "value")),
)


def same(a, b):
    return struct.pack("<d", a) == struct.pack("<d", b)


def dig(doc, path):
    for k in path:
        doc = doc[k]
    return doc


def floats(text):
    return [float(x) for x in text.split()]


def qmul(a, b):
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    return (a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3, a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
            a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1, a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0)


def rotate(q, v):
    w, x, y, z = q
    r = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def generate(scenario, mode, m, hover_member=None, log_path=None):
    name, text = gen_world.generate(CARD, SCEN / f"{scenario}.yaml", mode, m, hover_member, log_path)
    return name, text


def check_world(text, scenario, mode, m, hover_member=None, log_path=None):
    """Mismatches between the generated `text` and the card and scenario read independently; empty means round trip."""
    bad = []
    root = ET.fromstring(text)
    card = schema.load_yaml(CARD)
    doc = schema.load_yaml(SCEN / f"{scenario}.yaml")
    if root.tag != "sdf" or root.get("version") != "1.11":
        bad.append("sdf version")
    world = root.find("world")
    phys = world.find("physics")
    if phys.get("type") != "dart":
        bad.append("physics type")
    step = float(phys.find("max_step_size").text)
    if not same(step, (m * doc["tick_period_num_us"]["value"]) / (doc["tick_period_den"]["value"] * 10 ** 6)):
        bad.append("max_step_size")
    if float(phys.find("real_time_factor").text) != {"pilot": 1.0, "test": 0.0}[mode]:
        bad.append("real_time_factor")
    if floats(world.find("gravity").text) != [0.0, 0.0, 0.0]:
        bad.append("gravity")
    system = [p for p in world.findall("plugin") if p.get("name") == "gz::sim::systems::Physics"]
    if len(system) != 1 or system[0].get("filename") != "gz-sim-physics-system" or \
            system[0].find("engine/filename").text != "gz-physics-dartsim-plugin":
        bad.append("physics system / engine")
    if world.find(".//collision") is not None or world.find(".//visual") is not None or \
            world.find(".//plane") is not None:
        bad.append("contact geometry present")

    model = world.find("model")
    plugin = model.find("plugin")
    if plugin.get("filename") != "marv_gz_lockstep" or plugin.get("name") != "marv::gz::Lockstep":
        bad.append("plugin name")

    def get(tag):
        return plugin.find(tag).text

    for tag, path in CARD_SCALARS:
        if not same(float(get(tag)), float(dig(card, path))):
            bad.append(tag)
    speed = dig(card, ("rotors", "speed_range", "value"))
    if not same(float(get("omega_min_rad_s")), float(speed[0])) or not same(float(get("omega_max_rad_s")), float(speed[1])):
        bad.append("speed range")
    if not same(float(model.find("link/inertial/mass").text), float(card["mass"]["value"])):
        bad.append("model mass")
    for tag, v in zip(("ixx", "iyy", "izz"), card["inertia_diag"]["value"]):
        if not same(float(model.find(f"link/inertial/inertia/{tag}").text), float(v)):
            bad.append(tag)
    for i, mk in enumerate(MOTORS):
        rotor = plugin.findall("rotor")[i]
        pos = floats(rotor.find("position_frd_m").text)
        if rotor.get("motor") != str(i + 1) or not all(same(a, float(b)) for a, b in
                                                       zip(pos, card["rotors"][mk]["position"]["value"])):
            bad.append(f"rotor {i + 1} position")
        if int(rotor.find("yaw_sign").text) != {"ccw": 1, "cw": -1}[card["rotors"][mk]["spin"]["value"]]:
            bad.append(f"rotor {i + 1} yaw_sign")

    if not same(float(get("site_lat_rad")), float(doc["site_latitude_rad"]["value"])):
        bad.append("site_lat_rad")
    if not same(float(get("site_height_m")), float(doc["site_height_m"]["value"])):
        bad.append("site_height_m")
    if not same(float(get("motor_substep_s")),
                doc["tick_period_num_us"]["value"] / (doc["tick_period_den"]["value"] * 10 ** 6)):
        bad.append("motor_substep_s")
    if int(get("ticks_per_step")) != m:
        bad.append("ticks_per_step")
    if int(get("tick_period_num_us")) != doc["tick_period_num_us"]["value"]:
        bad.append("tick_period_num_us")
    if int(get("tick_period_den")) != doc["tick_period_den"]["value"]:
        bad.append("tick_period_den")
    if int(get("seed")) != doc["seed"]["value"]:
        bad.append("seed")

    cmd = doc["command"]
    if "dshot" in cmd:
        want = cmd["dshot"]["value"]
    else:
        lo = doc["command"]["hover"]["value"]
        assert hover_member in lo
        import hover
        cfg = gpc.plant_config(CARD)
        _, _, d_lo, d_hi = hover.bracket_for_card(cfg, doc["site_latitude_rad"]["value"], doc["site_height_m"]["value"])
        want = [d_lo if hover_member == "lo" else d_hi] * 4
    ov = plugin.findall("sil_override")
    got = [(o.get("param"), o.get("type"), int(o.text)) for o in ov]
    if got != [(f"ol_dshot_m{i + 1}", "i32", want[i]) for i in range(4)]:
        bad.append("sil_override")

    st = doc["initial_state"]
    v = plugin.find("initial_velocity_ned_m_s")
    if any(st["velocity_ned_m_s"]["value"]) != (v is not None) or (
            v is not None and not all(same(a, float(b)) for a, b in zip(floats(v.text), st["velocity_ned_m_s"]["value"]))):
        bad.append("initial velocity")
    r = plugin.find("initial_body_rates_frd")
    if any(st["body_rates_frd_rad_s"]["value"]) != (r is not None) or (
            r is not None and not all(same(a, float(b)) for a, b in zip(floats(r.text), st["body_rates_frd_rad_s"]["value"]))):
        bad.append("initial body rates")
    lp = plugin.find("log_path")
    if (lp is None) != (log_path is None) or (lp is not None and lp.text != log_path):
        bad.append("log_path")

    pose = model.find("pose")
    if pose.get("rotation_format") != "quat_xyzw":
        bad.append("pose format")
    x, y, z, qx, qy, qz, qw = floats(pose.text)
    n = st["position_ned_m"]["value"]
    if not (same(x, float(n[1])) and same(y, float(n[0])) and same(z, -float(n[2]))):
        bad.append("pose position")
    q_ned = tuple(float(c) for c in st["attitude_q_wxyz"]["value"])
    norm = math.sqrt(sum(c * c for c in q_ned))
    ref = qmul(qmul((0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0), q_ned), (0.0, 1.0, 0.0, 0.0))
    if ref[0] < 0:
        ref = tuple(-c for c in ref)
    got_q = (qw, qx, qy, qz)
    if not all(abs(a - b) <= 6 * U * norm for a, b in zip(got_q, ref)):
        bad.append("pose attitude vs Hamilton route")
    for v_frd in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.3, -0.5, 0.7)):
        ned = rotate(q_ned, v_frd)
        want = (ned[1], ned[0], -ned[2])
        have = rotate(got_q, (v_frd[0], -v_frd[1], -v_frd[2]))
        if not all(abs(a - b) <= 8 * U * 4 for a, b in zip(have, want)):
            bad.append("pose attitude rotates vectors wrongly")
            break
    return bad


@pytest.mark.parametrize("scenario, hover_member, mode, m", CASES)
def test_round_trip(scenario, hover_member, mode, m):
    log = "/run/marv/run.log"
    name, text = generate(scenario, mode, m, hover_member, log)
    stem = f"{scenario}_{hover_member}" if hover_member else scenario
    assert name == f"{VEHICLE}_{stem}_{mode}_m{m}.sdf"
    assert check_world(text, scenario, mode, m, hover_member, log) == []
    no_log = generate(scenario, mode, m, hover_member)[1]
    assert check_world(no_log, scenario, mode, m, hover_member) == []
    assert ET.fromstring(text).find("world").get("name") == name[:-4]


def canon(e):
    return (e.tag, dict(e.attrib), (e.text or "").strip(), [canon(c) for c in e if isinstance(c.tag, str)])


def test_the_model_is_the_gen_sdf_model_and_the_card_children_are_the_generated_ones():
    _, text = generate("free_fall", "test", 1)
    world_model = ET.fromstring(text).find("world/model")
    card, profile = gpc.load_linted(CARD)
    cfg, units = gpc.config_from(card, profile, CARD)
    ref_model = gen_sdf.sdf_element(card, cfg, units).find("model")
    assert world_model.find("pose") is not None and ref_model.find("pose") is None
    assert canon(world_model.find("link")) == canon(ref_model.find("link"))
    ref_plugin = gpc.plugin_element(cfg, units, card)
    ref_children = [canon(c) for c in ref_plugin if isinstance(c.tag, str)]
    got = [canon(c) for c in world_model.find("plugin")][:len(ref_children)]
    assert ref_children and got == ref_children


def test_pose_of_the_identity_attitude_and_of_the_origin():
    _, text = generate("free_fall", "test", 1)
    x, y, z, qx, qy, qz, qw = floats(ET.fromstring(text).find("world/model/pose").text)
    assert (x, y, z) == (0.0, 0.0, 0.0) and math.copysign(1.0, z) == -1.0  # -0.0: unary negation of +0.0
    s = math.sqrt(0.5)
    assert (qx, qy, qz, qw) == (0.0, 0.0, s, s)  # FLU x is ENU y: 90 degrees about z


def test_optional_elements_only_when_nonzero():
    rot = ET.fromstring(generate("rotation", "test", 4)[1]).find("world/model/plugin")
    assert rot.find("initial_body_rates_frd") is not None and rot.find("initial_velocity_ned_m_s") is None
    ff = ET.fromstring(generate("free_fall", "test", 4)[1]).find("world/model/plugin")
    assert ff.find("initial_body_rates_frd") is None and ff.find("initial_velocity_ned_m_s") is None
    det = ET.fromstring(generate("determinism", "test", 1)[1]).find("world/model/plugin")
    assert det.find("initial_velocity_ned_m_s") is not None and det.find("initial_body_rates_frd") is None


def mutate_text(text, pattern, repl):
    out, n = re.subn(pattern, repl, text, count=1)
    assert n == 1, pattern
    return out


def one_ulp_up(m):
    return repr(math.nextafter(float(m.group(2)), math.inf)).join((m.group(1), m.group(3)))


@pytest.mark.parametrize("what, pattern, repl", [
    ("step size", r"(<max_step_size>)([^<]*)(</max_step_size>)", one_ulp_up),
    ("rtf", r"(<real_time_factor>)([^<]*)(</real_time_factor>)", r"\g<1>1.0\3"),
    ("gravity", r"(<gravity>)([^<]*)(</gravity>)", r"\g<1>0 0 -9.8\3"),
    ("engine", r"(<filename>)(gz-physics-dartsim-plugin)(</filename>)", r"\g<1>gz-physics-bullet-plugin\3"),
    ("physics type", r'type="dart"', 'type="ode"'),
    ("mass", r"(<mass_kg unit=\"kg\">)([^<]*)(</mass_kg>)", one_ulp_up),
    ("thrust coeff", r"(<thrust_coeff unit=\"N/\(rad/s\)\^2\">)([^<]*)(</thrust_coeff>)", one_ulp_up),
    ("site latitude", r"(<site_lat_rad unit=\"rad\">)([^<]*)(</site_lat_rad>)", one_ulp_up),
    ("substep", r"(<motor_substep_s unit=\"s\">)([^<]*)(</motor_substep_s>)", one_ulp_up),
    ("ticks per step", r"(<ticks_per_step>)([^<]*)(</ticks_per_step>)", r"\g<1>3\3"),
    ("dshot", r'(param="ol_dshot_m2" type="i32">)([^<]*)(</sil_override>)', r"\g<1>701\3"),
    ("ixx", r"(<ixx>)([^<]*)(</ixx>)", one_ulp_up),
    ("position swap", r'(<pose rotation_format="quat_xyzw">)-2.5 1.5 3.5( [^<]*)(</pose>)', r"\g<1>1.5 -2.5 3.5\2\3"),
])
def test_negative_control_round_trip_detects_a_perturbed_value(what, pattern, repl):
    if what in ("position swap",):
        scenario, mode, m = "determinism", "test", 1
    else:
        scenario, mode, m = "free_fall", "test", 4
    text = generate(scenario, mode, m)[1]
    assert check_world(text, scenario, mode, m) == []
    bad = check_world(mutate_text(text, pattern, repl), scenario, mode, m)
    assert bad, what


def test_negative_control_attitude_swap_and_pose_sign_detected():
    scenario, mode, m = "determinism", "test", 1
    text = generate(scenario, mode, m)[1]
    pose = re.search(r'(<pose rotation_format="quat_xyzw">)([^<]*)(</pose>)', text)
    x, y, z, qx, qy, qz, qw = floats(pose.group(2))
    for vals in ((x, y, z, qy, qx, qz, qw), (x, y, -z, qx, qy, qz, qw), (x, y, z, qx, qy, -qz, qw),
                 (x, y, z, qx, qy, qz, qw + 1e-12)):
        swapped = text.replace(pose.group(2), " ".join(repr(v) for v in vals))
        assert check_world(swapped, scenario, mode, m), vals


@pytest.mark.parametrize("scenario, mode, m, hv, why", [
    ("free_fall", "test", 3, None, "m not in the sequence"),
    ("free_fall", "test", 2.5, None, "m not an integer"),
    ("free_fall", "test", True, None, "m a bool"),
    ("determinism", "test", 2, None, "m outside [1]"),
    ("free_fall", "flight", 1, None, "mode"),
    ("hover", "test", 4, None, "hover member missing"),
    ("hover", "test", 4, "mid", "hover member unknown"),
    ("free_fall", "test", 4, "lo", "hover given for a dshot scenario"),
])
def test_refusals(scenario, mode, m, hv, why):
    with pytest.raises(gpc.GenError):
        gen_world.generate(CARD, SCEN / f"{scenario}.yaml", mode, m, hv)


def test_invalid_scenario_and_unlinted_card_generate_nothing(tmp_path):
    bad = tmp_path / "free_fall.yaml"
    bad.write_text((SCEN / "free_fall.yaml").read_text(encoding="utf-8").replace("value: [0, 0, 0, 0]",
                                                                                 "value: [0, 0, 0, 5]"), encoding="utf-8")
    assert gen_world.main(["--card", str(CARD), "--scenario", str(bad), "--mode", "test", "--m", "1",
                           "--out-dir", str(tmp_path / "out")]) == 1
    assert not (tmp_path / "out").exists()
    card = tmp_path / f"{VEHICLE}.yaml"
    card.write_text(CARD.read_text(encoding="utf-8").replace("sigma: 0.010", "sigma: 0.0", 1), encoding="utf-8")
    assert gen_world.main(["--card", str(card), "--scenario", str(SCEN / "free_fall.yaml"), "--mode", "test",
                           "--m", "1", "--out-dir", str(tmp_path / "out2")]) == 1
    assert not (tmp_path / "out2").exists()


def test_cli_writes_the_named_file(tmp_path):
    rc = gen_world.main(["--card", str(CARD), "--scenario", str(SCEN / "hover.yaml"), "--mode", "test", "--m", "2",
                         "--hover", "hi", "--log-path", "/x/y.bin", "--out-dir", str(tmp_path)])
    assert rc == 0
    f = tmp_path / f"{VEHICLE}_hover_hi_test_m2.sdf"
    assert check_world(f.read_text(encoding="utf-8"), "hover", "test", 2, "hi", "/x/y.bin") == []


def sdformat14_checker():
    gz = shutil.which("gz")
    if gz is None:
        return None
    out = subprocess.run([gz, "sdf", "--versions"], capture_output=True, text=True, check=False)
    for line in out.stdout.split():
        if line.startswith("14."):
            return [gz, "sdf", "--force-version", line, "-k"]
    return None


def test_gz_sdf_check_with_sdformat14(tmp_path):
    cmd = sdformat14_checker()
    if cmd is None:
        pytest.skip("UNKNOWN: no sdformat14 'gz sdf' on this host; the check runs inside marv-ci-gz")
    for scenario, hv, mode, m in CASES:
        name, text = generate(scenario, mode, m, hv, "/x/run.log")
        f = tmp_path / name
        f.write_text(text, encoding="utf-8")
        out = subprocess.run(cmd + [str(f)], capture_output=True, text=True, check=False)
        assert out.returncode == 0 and "Valid." in out.stdout, (name, out.stdout, out.stderr)
        assert "Warning" not in out.stdout + out.stderr and "Error" not in out.stdout + out.stderr, (name, out.stderr)
    # negative control: an element SDFormat does not define is reported
    bad = tmp_path / "bad.sdf"
    bad.write_text('<sdf version="1.11"><world name="w"><physics type="dart"><bogus_elem/></physics></world></sdf>',
                   encoding="utf-8")
    out = subprocess.run(cmd + [str(bad)], capture_output=True, text=True, check=False)
    assert "Warning" in out.stdout + out.stderr
