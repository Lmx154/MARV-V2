"""The plugin's sensor elements in the generated world (decision 0019, W1 C1; tools/card/gen_world.py SensorSet,
tools/card/gen_imu_config.py imu_model_element). No simulator: this runs per push in the core image.

What is checked:
  (a) defaults. For every committed L2 scenario (each hover member) and every m of its m_sequence, generate(...,
      sensors=None) is the world generated without the argument, byte for byte, and holds none of the sensor elements. A
      world with sensors differs from that default world only by the inserted elements (exactly the ones the SensorSet
      asks for, inside the lockstep plugin element), plus the max_step_size line iff the clock corner is nonzero, where it
      is fl(n_true / 1e9).
  The emitted values read back as the profile's SI values bitwise (imu_config(), the profile's rotor-speed grid times the
      register's rate_loop_divisor, the ODR error), with the signs and the profile's SHA-256.
  The realised host step: n_true and e_r = n_nom / n_true - 1 at c in {-1, 0, +1} and m in {1, 2} are the table of the W1
      design (outward rounding, ruling 10 of decision 0019), max_step_size maps back to n_true through gz-sim's
      duration_cast<ns> truncation (INFERRED from gz-sim 8.15; the plugin's UpdateInfo dt check is the guard), and at c = 0
      t_true = n_true / (m 1e9) is the nominal tick bitwise.
  The generator refuses a malformed SensorSet.
  The test-only clock allow of the plugin (ruling on Q6) is named by exactly two files: the plugin, which reads it, and the
      clock T1 test, its only user.

Negative controls: a sensors world with one stray change (a seed edit, or an unexpected element) and a nonzero-corner
world checked as if nominal are flagged; perturbed emitted values are flagged; nearest rounding misses the table; a file
planted with the allow's name is found by the scan.
"""

import hashlib
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from difflib import SequenceMatcher
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_imu_config as gic  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gen_world  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
REGISTER = ROOT / "design" / "scenario_values.yaml"
LOG = "/x/run.bin"
WORLDS = [("determinism", None), ("free_fall", None), ("rotation", None), ("hover", "lo"), ("hover", "hi")]
NEW_TAGS = ("gyro_source", "imu_model", "rotor_speed_model", "clock_corner", "odr_error")
SIGNS = (1, -1, 0, 1, 1, -1)  # labelled: a corner with every sign value on both sensors
SENSOR_SETS = [
    gen_world.SensorSet(),
    gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6),
    gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS),
    *[gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS, clock_corner=c) for c in (-1, 0, 1)],
    *[gen_world.SensorSet(clock_corner=c) for c in (-1, 1)],
]
# The W1 design's table (decision 0019, W1 memo section 2): n_true ns at c = -1, 0, +1, and e_r in ppm to 6 decimals.
TABLE = {
    1: {-1: (156261, -70.395044), 0: (156250, 0.0), 1: (156239, 70.404957)},
    2: {-1: (312521, -67.195484), 0: (312500, 0.0), 1: (312479, 67.204516)},
}
ALLOW = "MARV_GZ_TEST_" + "CLOCK_CORNER_ANY_GYRO"  # split so that this file does not name it
ALLOW_USERS = {"sim/gz/plugin/src/lockstep.cpp", "tests/regression/quad/L06/gz/test_sensor_clock.py"}
SCAN_SKIP_DIRS = {".git", ".venv", "build", "docs", "__pycache__"}
SCAN_SUFFIXES = {".py", ".sh", ".c", ".cc", ".cpp", ".h", ".hpp", ".cmake", ".txt", ".yml", ".yaml", ".json", ".toml"}


def world(name, member, m, sensors=None, **kw):
    path = SCEN / f"{name}.yaml"
    if sensors is None and not kw:
        return gen_world.generate(CARD, path, "test", m, member, LOG)[1]
    return gen_world.generate(CARD, path, "test", m, member, LOG, sensors=sensors, **kw)[1]


def m_sequence(name):
    return scn.values(scn.load(SCEN / f"{name}.yaml", CARD))["m_sequence"]


def expected_tags(s):
    return ([*(["gyro_source", "imu_model"] if s.gyro == "model" else []), *(["rotor_speed_model"] if s.rotor else []),
             *(["clock_corner", "odr_error"] if s.clock_corner is not None else [])])


def odr_error():
    return gic.imu_config(PROFILE)[0]["odr_error"]


def insertion_problems(default, text, s, name, m):
    """What differs between a sensors world and its default world besides what SensorSet s inserts; [] if nothing."""
    a, b = default.splitlines(), text.splitlines()
    problems, inserted = [], []
    for op, i1, i2, j1, j2 in SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        if op == "equal":
            continue
        if op == "insert":
            inserted.extend(b[j1:j2])
            continue
        if op == "replace" and i2 - i1 == 1 and j2 - j1 == 1 and "<max_step_size>" in a[i1] and s.clock_corner:
            n = gen_world.realised_host_step_ns(scn.load(SCEN / f"{name}.yaml", CARD), m, s.clock_corner, odr_error())
            if b[j1].strip() != f"<max_step_size>{float(Fraction(n, 10 ** 9))!r}</max_step_size>":
                problems.append(f"max_step_size is not fl(n_true / 1e9): {b[j1].strip()}")
            continue
        problems.append(f"{op} of default lines {i1}..{i2} by {b[j1:j2]!r}")
    got = [e.tag for e in ET.fromstring("<x>" + "\n".join(inserted) + "</x>")] if inserted else []
    if got != expected_tags(s):
        problems.append(f"inserted elements {got}, expected {expected_tags(s)}")
    plugin = ET.fromstring(text).find(".//plugin[@filename='marv_gz_lockstep']")
    if [c.tag for c in plugin if c.tag in NEW_TAGS] != expected_tags(s):
        problems.append("the inserted elements are not the lockstep plugin's children")
    return problems


@pytest.fixture(scope="module")
def defaults():
    return {(n, h, m): world(n, h, m) for n, h in WORLDS for m in m_sequence(n)}


def test_sensors_none_is_the_default_world_byte_for_byte(defaults):
    for (n, h, m), text in defaults.items():
        assert world(n, h, m, None, attitude_source=False) == text, (n, h, m)
        assert world(n, h, m, gen_world.SensorSet()) == text, (n, h, m)
        assert not any(f"<{t}" in text for t in NEW_TAGS), (n, h, m)


@pytest.mark.parametrize("s", SENSOR_SETS, ids=lambda s: f"{s.gyro}-{s.rotor}-{s.clock_corner}")
def test_a_sensors_world_differs_only_by_the_inserted_elements(defaults, s):
    for (n, h, m), default in defaults.items():
        assert insertion_problems(default, world(n, h, m, s), s, n, m) == [], (n, h, m)


def test_control_a_stray_change_or_a_corner_checked_as_nominal_is_flagged(defaults):
    s = SENSOR_SETS[2]  # model and rotor, no clock element
    default, text = defaults[("hover", "lo", 1)], world("hover", "lo", 1, s)
    assert insertion_problems(default, text.replace("<seed>1</seed>", "<seed>2</seed>"), s, "hover", 1) != []
    stray = text.replace("</imu_model>", "</imu_model>\n<bogus>1</bogus>")
    assert insertion_problems(default, stray, s, "hover", 1) != []
    c1 = gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS, clock_corner=1)
    c0 = gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS, clock_corner=0)
    assert insertion_problems(default, world("hover", "lo", 1, c1), c0, "hover", 1) != []


# ---- the emitted values ---------------------------------------------------------------------------------------------

def emitted(text):
    plugin = ET.fromstring(text).find(".//plugin[@filename='marv_gz_lockstep']")
    imu, rotor = plugin.find("imu_model"), plugin.find("rotor_speed_model")
    out = {"latency_samples": int(imu.find("latency_samples").text), "odr_error": float(plugin.find("odr_error").text),
           "signs": tuple(int(v) for v in imu.find("turn_on_bias_signs").text.split()),
           "sha256": imu.find("profile_sha256").text, "corner": int(plugin.find("clock_corner").text),
           "rotor": {k: (float if k == "period_unit_s" else int)(rotor.find(k).text)
                     for k in ("latency_ticks", "exponent_bits", "mantissa_bits", "period_unit_s")}}
    for s in ("gyro", "accel"):
        for f in ("noise_density", "bias_instability", "lsb", "full_scale", "turn_on_bias_bound"):
            out[f"{s}_{f}"] = float(imu.find(f"{s}/{f}").text)
    return out


def value_problems(got, si, sha, rotor):
    bad = [k for k in si if k in got and k != "latency_samples" and float(got[k]).hex() != float(si[k]).hex()]
    if got["latency_samples"] != si["latency_samples"]:
        bad.append("latency_samples")
    if got["sha256"] != sha:
        bad.append("sha256")
    if {k: (v.hex() if isinstance(v, float) else v) for k, v in got["rotor"].items()} != \
            {k: (v.hex() if isinstance(v, float) else v) for k, v in rotor.items()}:
        bad.append("rotor")
    return bad


def expected_rotor():
    e = schema.load_yaml(PROFILE)["classes"]["rotor_speed"]["entries"]
    divisor = schema.load_yaml(REGISTER)["rate_loop_divisor"]["value"]
    assert e["telemetry_period_unit"]["unit"] == "us"
    return {"latency_ticks": e["latency_rate_periods"]["value"] * divisor,
            "exponent_bits": e["telemetry_exponent_bits"]["value"], "mantissa_bits": e["telemetry_mantissa_bits"]["value"],
            "period_unit_s": e["telemetry_period_unit"]["value"] * 1e-6}


def test_emitted_values_are_the_profiles_bitwise():
    si, _, meta = gic.imu_config(PROFILE)
    got = emitted(world("hover", "lo", 1, gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS,
                                                               clock_corner=1)))
    assert meta["sha256"] == hashlib.sha256(PROFILE.read_bytes()).hexdigest()
    assert value_problems(got, si, meta["sha256"], expected_rotor()) == []
    assert got["signs"] == SIGNS and got["corner"] == 1
    assert expected_rotor() == {"latency_ticks": 2, "exponent_bits": 3, "mantissa_bits": 9, "period_unit_s": 1e-6}
    assert got["odr_error"] == 65 * 1e-6 and got["latency_samples"] == 1


def test_control_perturbed_values_are_flagged():
    si, _, meta = gic.imu_config(PROFILE)
    got = emitted(world("hover", "lo", 1, gen_world.SensorSet(gyro="model", rotor=True, bias_signs=SIGNS,
                                                               clock_corner=1)))
    for k in ("gyro_noise_density", "accel_turn_on_bias_bound", "odr_error"):
        bad = dict(si, **{k: math.nextafter(si[k], math.inf)})
        assert value_problems(got, bad, meta["sha256"], expected_rotor()) == [k]
    assert value_problems(got, si, "0" * 64, expected_rotor()) == ["sha256"]
    assert value_problems(got, si, meta["sha256"], dict(expected_rotor(), latency_ticks=1)) == ["rotor"]


# ---- the realised host step -----------------------------------------------------------------------------------------

def test_the_realised_host_step_is_the_designs_table():
    doc = scn.load(SCEN / "hover.yaml", CARD)
    e = odr_error()
    for m, row in TABLE.items():
        n_nom = row[0][0]
        for c, (n_want, ppm) in row.items():
            n = gen_world.realised_host_step_ns(doc, m, c, e)
            assert n == n_want, (m, c)
            e_r = Fraction(n_nom, n) - 1
            assert round(float(e_r * 10 ** 6), 6) == ppm, (m, c)
            assert e_r == Fraction(n_nom - n, n)
            step = float(Fraction(n, 10 ** 9))
            assert int(step * 1e9) == n, (m, c)  # duration_cast<ns> truncation (INFERRED) gives n back
            t_true = n / (m * 1e9)
            assert t_true == float(Fraction(n, m * 10 ** 9))
            if c == 0:
                assert t_true.hex() == (625 / (4 * 1e6)).hex()
            else:
                assert abs(e_r) >= Fraction(e)  # outward: the realised error covers the bound e
    # Control: nearest rounding misses the outward table at both nonzero corners.
    for m, row in TABLE.items():
        for c in (-1, 1):
            nearest = round(Fraction(m * 156250) / (1 + c * Fraction(e)))
            assert nearest != row[c][0]
    assert {m: round(Fraction(m * 156250) / (1 + Fraction(e))) for m in (1, 2)} == {1: 156240, 2: 312480}


def test_the_generator_refuses_a_malformed_sensor_set():
    for s in (gen_world.SensorSet(gyro="truth"), gen_world.SensorSet(rotor=True),
              gen_world.SensorSet(gyro="model"), gen_world.SensorSet(bias_signs=(0,) * 6),
              gen_world.SensorSet(clock_corner=2), gen_world.SensorSet(gyro="model", bias_signs=(2, 0, 0, 0, 0, 0)),
              gen_world.SensorSet(gyro="model", bias_signs=(0,) * 5)):
        with pytest.raises(gpc.GenError):
            world("hover", "lo", 1, s)


# ---- the test-only clock allow (ruling on Q6) -----------------------------------------------------------------------

def files_naming(root, name):
    out = set()
    for d, dirs, files in os.walk(root):
        dirs[:] = [x for x in dirs if x not in SCAN_SKIP_DIRS]
        for f in files:
            p = Path(d) / f
            if p.suffix in SCAN_SUFFIXES or f.startswith("Dockerfile") or f == "CMakeLists.txt":
                try:
                    if name in p.read_text(encoding="utf-8", errors="replace"):
                        out.add(p.relative_to(root).as_posix())
                except OSError:
                    continue
    return out


def test_the_clock_allow_is_named_only_by_the_plugin_and_the_clock_test():
    assert files_naming(ROOT, ALLOW) == ALLOW_USERS


def test_control_the_scan_finds_a_planted_user(tmp_path):
    (tmp_path / "ci").mkdir()
    (tmp_path / "ci" / "x.sh").write_text(f"export {ALLOW}=1\n", encoding="utf-8")
    (tmp_path / "build").mkdir()
    (tmp_path / "build" / "y.sh").write_text(f"export {ALLOW}=1\n", encoding="utf-8")
    assert files_naming(tmp_path, ALLOW) == {"ci/x.sh"}
    assert re.search(re.escape(ALLOW), (ROOT / "sim" / "gz" / "plugin" / "src" / "lockstep.cpp").read_text())
