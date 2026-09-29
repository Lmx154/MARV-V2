"""SDF and plant-configuration generators (tools/card/gen_plant_config.py, gen_sdf.py). Frozen L01 tests.

The round-trip check reads every generated value back and compares it bit-exactly (==) with the value read from the
card YAML here, independently of the generators: the card is read with the project YAML loader and navigated by
literal paths, the generated SDF and header are parsed with xml.etree and float.fromhex.
"""

import copy
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
GEN_PLANT = ROOT / "tools" / "card" / "gen_plant_config.py"
GEN_SDF = ROOT / "tools" / "card" / "gen_sdf.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
PLANT_HEADER = ROOT / "sim" / "plant" / "include" / "marv_plant.h"
VEHICLE = "uzh_neurobem_5in"
SCENARIO_FIELDS = {"site_lat_rad", "site_height_m", "motor_substep_s", "rng_seed"}
MOTORS = ("m1", "m2", "m3", "m4")

sys.path.insert(0, str(ROOT / "tools" / "card"))
import schema  # noqa: E402


def load(path):
    return copy.deepcopy(schema.load_yaml(path))


def write_yaml(path, doc):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


def run(script, card, out_dir, root=ROOT):
    return subprocess.run(
        [sys.executable, str(script), "--card", str(card), "--root", str(root), "--out-dir", str(out_dir)],
        capture_output=True, text=True, check=False,
    )


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    out = tmp_path_factory.mktemp("gen")
    for script in (GEN_PLANT, GEN_SDF):
        r = run(script, CARD, out)
        assert r.returncode == 0, r.stderr
    return out


def expected_from_card(card, profile):
    """Every vehicle value, read from the YAML documents by literal path."""
    rotors = card["rotors"]
    return {
        "mass_kg": float(card["mass"]["value"]),
        "inertia": [float(v) for v in card["inertia_diag"]["value"]],
        "thrust_coeff": float(rotors["thrust_coeff"]["value"]),
        "torque_ratio_m": float(rotors["torque_ratio"]["value"]),
        "omega_min_rad_s": float(rotors["speed_range"]["value"][0]),
        "omega_max_rad_s": float(rotors["speed_range"]["value"][1]),
        "motor_tau_s": float(rotors["motor_lag"]["tau"]["value"]),
        "position": [[float(v) for v in rotors[m]["position"]["value"]] for m in MOTORS],
        "spin": [rotors[m]["spin"]["value"] for m in MOTORS],
        "pole_count": profile["classes"]["rotor_speed"]["entries"]["pole_count"]["value"],
    }


def yaw_sign_of(spin):
    assert spin in ("ccw", "cw")
    return 1 if spin == "ccw" else -1


def esc_enum_value():
    m = re.search(r"MARV_PLANT_ESC_LINEAR_IN_OMEGA\s*=\s*(\d+)", PLANT_HEADER.read_text(encoding="utf-8"))
    return int(m.group(1))


def plugin_of(sdf_path):
    return ET.parse(sdf_path).getroot().find("model").find("plugin")


def read_plugin(plugin):
    """(scalars, per-motor) of a parsed <plugin>, values converted with float() / int()."""
    scalars = {}
    for tag in ("mass_kg", "thrust_coeff", "torque_ratio_m", "omega_min_rad_s", "omega_max_rad_s", "motor_tau_s"):
        scalars[tag] = float(plugin.find(tag).text)
    scalars["esc_map"] = int(plugin.find("esc_map").text)
    scalars["pole_count"] = int(plugin.find("pole_count").text)
    rotors = {}
    for r in plugin.findall("rotor"):
        n = int(r.get("motor"))
        rotors[n] = ([float(t) for t in r.find("position_frd_m").text.split()], int(r.find("yaw_sign").text))
    return scalars, rotors


def header_assignments(text):
    """{'mass_kg': value, 'rotor_position_frd_m[0][1]': value, ...} with hex floats via float.fromhex and ints."""
    out = {}
    for m in re.finditer(r"^\s*c->([A-Za-z_]+(?:\[\d+\])*)\s*=\s*([^;]+);", text, re.MULTILINE):
        name, rhs = m.group(1), m.group(2).strip()
        if re.fullmatch(r"-?0x[0-9a-f.]+p[+-]\d+", rhs):
            out[name] = float.fromhex(rhs)
        elif re.fullmatch(r"-?\d+u?", rhs):
            out[name] = int(rhs.rstrip("u"))
        else:
            out[name] = rhs
    return out


def check_round_trip(card, profile, sdf_path, header_path):
    exp = expected_from_card(card, profile)
    root = ET.parse(sdf_path).getroot()
    assert root.tag == "sdf" and root.get("version") == "1.11"
    models = root.findall("model")
    assert len(models) == 1 and models[0].get("name") == card["vehicle"]
    links = models[0].findall("link")
    assert len(links) == 1 and links[0].get("name") == "base_link"
    link = links[0]
    assert [float(t) for t in link.find("pose").text.split()] == [0.0] * 6
    inertial = link.find("inertial")
    assert [float(t) for t in inertial.find("pose").text.split()] == [0.0] * 6
    assert float(inertial.find("mass").text) == exp["mass_kg"]
    inertia = inertial.find("inertia")
    assert [float(inertia.find(t).text) for t in ("ixx", "iyy", "izz")] == exp["inertia"]
    assert [float(inertia.find(t).text) for t in ("ixy", "ixz", "iyz")] == [0.0, 0.0, 0.0]
    assert link.find("visual") is None and link.find("collision") is None

    plugin = models[0].find("plugin")
    assert plugin.get("filename") == "marv_gz_lockstep" and plugin.get("name") == "marv::gz::Lockstep"
    scalars, rotors = read_plugin(plugin)
    assert scalars["mass_kg"] == exp["mass_kg"]
    for tag in ("thrust_coeff", "torque_ratio_m", "omega_min_rad_s", "omega_max_rad_s", "motor_tau_s"):
        assert scalars[tag] == exp[tag], tag
    assert scalars["esc_map"] == esc_enum_value()
    assert scalars["pole_count"] == exp["pole_count"]
    assert sorted(rotors) == [1, 2, 3, 4]
    for i in range(4):
        position, yaw = rotors[i + 1]
        assert position == exp["position"][i], f"motor {i + 1}"
        assert yaw == yaw_sign_of(exp["spin"][i]), f"motor {i + 1}"
    units = {c.tag: c.get("unit") for c in plugin if c.get("unit")}
    assert units == {
        "mass_kg": card["mass"]["unit"],
        "thrust_coeff": card["rotors"]["thrust_coeff"]["unit"],
        "torque_ratio_m": card["rotors"]["torque_ratio"]["unit"],
        "omega_min_rad_s": card["rotors"]["speed_range"]["unit"],
        "omega_max_rad_s": card["rotors"]["speed_range"]["unit"],
        "motor_tau_s": card["rotors"]["motor_lag"]["tau"]["unit"],
    }
    assert all(r.find("position_frd_m").get("unit") == card["rotors"]["m1"]["position"]["unit"]
               for r in plugin.findall("rotor"))

    h = header_assignments(header_path.read_text(encoding="utf-8"))
    for tag in ("mass_kg", "thrust_coeff", "torque_ratio_m", "omega_min_rad_s", "omega_max_rad_s"):
        assert h[tag] == exp[tag], tag
    assert h["motor_tau_s"] == exp["motor_tau_s"]
    assert h["pole_count"] == exp["pole_count"]
    for i in range(4):
        assert h[f"yaw_sign[{i}]"] == yaw_sign_of(exp["spin"][i])
        for k in range(3):
            assert h[f"rotor_position_frd_m[{i}][{k}]"] == exp["position"][i][k]
    return h, plugin


def test_round_trip_of_the_product_card(generated):
    card, profile = load(CARD), load(PROFILE)
    check_round_trip(card, profile, generated / f"{VEHICLE}.sdf", generated / f"marv_plant_card_{VEHICLE}.h")


def test_standalone_plugin_configuration_equals_the_one_in_the_sdf(generated):
    standalone = ET.parse(generated / f"marv_plant_card_{VEHICLE}.plugin.xml").getroot()
    embedded = plugin_of(generated / f"{VEHICLE}.sdf")

    def flat(e):
        return [(x.tag, dict(x.attrib), (x.text or "").strip()) for x in e.iter()]

    assert standalone.tag == "plugin"
    assert flat(standalone) == flat(embedded)


def test_plugin_and_header_carry_exactly_the_vehicle_fields_of_the_plant_config(generated):
    text = PLANT_HEADER.read_text(encoding="utf-8")
    body = re.search(r"typedef struct marv_plant_config \{(.*?)\} marv_plant_config;", text, re.DOTALL).group(1)
    fields = set()
    for line in re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL).split(";"):
        line = line.strip()
        if line:
            fields.add(re.search(r"([A-Za-z_]+)(?:\[[^\]]*\])*$", line).group(1))
    vehicle_fields = fields - {"struct_size"} - SCENARIO_FIELDS
    assert SCENARIO_FIELDS <= fields

    plugin = plugin_of(generated / f"{VEHICLE}.sdf")
    plugin_fields = {c.tag for c in plugin if c.tag != "rotor"} | {"rotor_position_frd_m", "yaw_sign"}
    assert plugin_fields == vehicle_fields

    header = header_assignments((generated / f"marv_plant_card_{VEHICLE}.h").read_text(encoding="utf-8"))
    assigned = {re.sub(r"\[.*", "", k) for k in header}
    assert assigned == vehicle_fields | {"struct_size"}
    for scenario in SCENARIO_FIELDS:
        assert scenario in (generated / f"marv_plant_card_{VEHICLE}.h").read_text(encoding="utf-8")  # named in the comment
        assert scenario not in assigned


def test_header_states_the_decimal_repr_beside_each_hex_literal(generated):
    text = (generated / f"marv_plant_card_{VEHICLE}.h").read_text(encoding="utf-8")
    pairs = re.findall(r"= (-?0x[0-9a-f.]+p[+-]\d+);\s*/\* (\S+) \*/", text)
    assert pairs
    for hexa, dec in pairs:
        assert float(dec) == float.fromhex(hexa)


def test_changed_mass_changes_every_copy(tmp_path):
    card = load(CARD)
    card["mass"]["value"] = 0.9
    card_path = write_yaml(tmp_path / "in" / f"{VEHICLE}.yaml", card)
    out = tmp_path / "out"
    for script in (GEN_PLANT, GEN_SDF):
        assert run(script, card_path, out).returncode == 0
    h, plugin = check_round_trip(load(card_path), load(PROFILE), out / f"{VEHICLE}.sdf",
                                 out / f"marv_plant_card_{VEHICLE}.h")
    assert h["mass_kg"] == 0.9
    assert float(plugin.find("mass_kg").text) == 0.9
    assert float(ET.parse(out / f"{VEHICLE}.sdf").getroot().find("model/link/inertial/mass").text) == 0.9


def test_changed_inertia_and_spin_change_the_outputs(tmp_path):
    card = load(CARD)
    card["inertia_diag"]["value"] = [0.003, 0.0031, 0.005]
    card["rotors"]["m1"]["spin"]["value"] = "cw"
    card["rotors"]["m3"]["spin"]["value"] = "ccw"
    card_path = write_yaml(tmp_path / "in" / f"{VEHICLE}.yaml", card)
    out = tmp_path / "out"
    for script in (GEN_PLANT, GEN_SDF):
        assert run(script, card_path, out).returncode == 0
    _, plugin = check_round_trip(load(card_path), load(PROFILE), out / f"{VEHICLE}.sdf",
                                 out / f"marv_plant_card_{VEHICLE}.h")
    _, rotors = read_plugin(plugin)
    assert (rotors[1][1], rotors[3][1]) == (-1, 1)


def test_pole_count_unknown_becomes_zero(tmp_path):
    root = tmp_path / "root"
    profile = load(PROFILE)
    entry = profile["classes"]["rotor_speed"]["entries"]["pole_count"]
    entry["value"] = "UNKNOWN"
    entry["sigma"] = "UNKNOWN"
    write_yaml(root / "sensors" / "profiles" / PROFILE.name, profile)
    (root / "sim" / "plant" / "include").mkdir(parents=True)
    shutil.copy(PLANT_HEADER, root / "sim" / "plant" / "include" / PLANT_HEADER.name)
    card_path = root / "vehicles" / f"{VEHICLE}.yaml"
    card_path.parent.mkdir()
    shutil.copy(CARD, card_path)
    out = tmp_path / "out"
    for script in (GEN_PLANT, GEN_SDF):
        r = run(script, card_path, out, root=root)
        assert r.returncode == 0, r.stderr
    scalars, _ = read_plugin(plugin_of(out / f"{VEHICLE}.sdf"))
    assert scalars["pole_count"] == 0
    assert header_assignments((out / f"marv_plant_card_{VEHICLE}.h").read_text(encoding="utf-8"))["pole_count"] == 0


def unlint_mass(card):
    del card["mass"]["source"]


def unknown_thrust_coeff(card):
    card["rotors"]["thrust_coeff"]["value"] = "UNKNOWN"
    del card["rotors"]["thrust_coeff"]["status"]
    del card["rotors"]["thrust_coeff"]["note"]


def unknown_position(card):
    card["rotors"]["m2"]["position"]["value"] = "UNKNOWN"
    del card["rotors"]["m2"]["position"]["shape"]


def missing_spin(card):
    del card["rotors"]["m3"]["spin"]


@pytest.mark.parametrize("script", [GEN_PLANT, GEN_SDF], ids=["gen_plant_config", "gen_sdf"])
@pytest.mark.parametrize("mutate,needle", [
    (unlint_mass, "mass"),
    (unknown_thrust_coeff, "UNKNOWN"),
    (unknown_position, "UNKNOWN"),
    (missing_spin, "spin"),
])
def test_broken_card_exits_1_and_writes_nothing(tmp_path, script, mutate, needle):
    card = load(CARD)
    mutate(card)
    card_path = write_yaml(tmp_path / "in" / f"{VEHICLE}.yaml", card)
    out = tmp_path / "out"
    r = run(script, card_path, out)
    assert r.returncode == 1
    assert needle in r.stderr
    assert not out.exists() or not any(out.iterdir())


def test_unmodified_card_is_the_control_of_the_refusals(tmp_path):
    card_path = write_yaml(tmp_path / "in" / f"{VEHICLE}.yaml", load(CARD))
    out = tmp_path / "out"
    assert run(GEN_SDF, card_path, out).returncode == 0
    assert (out / f"{VEHICLE}.sdf").is_file()


@pytest.mark.parametrize("script", [GEN_PLANT, GEN_SDF], ids=["gen_plant_config", "gen_sdf"])
def test_card_file_name_must_be_the_vehicle_id(tmp_path, script):
    card_path = write_yaml(tmp_path / "in" / "other_name.yaml", load(CARD))
    out = tmp_path / "out"
    r = run(script, card_path, out)
    assert r.returncode == 1
    assert VEHICLE + ".yaml" in r.stderr
    assert not out.exists() or not any(out.iterdir())


GZ = shutil.which("gz")


@pytest.mark.skipif(GZ is None, reason="gz is not installed here (the CI image has no Gazebo), so gz sdf -k cannot run")
def test_gz_sdf_check_accepts_the_generated_sdf_and_rejects_a_broken_one(generated, tmp_path):
    r = subprocess.run([GZ, "sdf", "-k", str(generated / f"{VEHICLE}.sdf")], capture_output=True, text=True,
                       check=False)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "Valid" in r.stdout and "Error" not in r.stdout + r.stderr
    broken = tmp_path / "broken.sdf"
    broken.write_text((generated / f"{VEHICLE}.sdf").read_text(encoding="utf-8").replace("<link ", "<lynk "),
                      encoding="utf-8")
    r = subprocess.run([GZ, "sdf", "-k", str(broken)], capture_output=True, text=True, check=False)
    assert "Valid" not in r.stdout + r.stderr  # gz sdf -k prints its error and still exits 0 on an unreadable file
