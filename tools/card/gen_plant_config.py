#!/usr/bin/env python3
"""Plant configuration generator (core contracts 2.1 rule 2, quad spec L1).

  gen_plant_config.py --card <vehicle>.yaml --out-dir <dir> [--root <repo root>]

Reads a vehicle card and the sensor profile it names, lints both with the same code as lint.py (an unlinted card
generates nothing), and writes into <dir>:

  marv_plant_card_<vehicle>.h           C header (build directory only, never committed): the function
                                        marv_plant_card_<vehicle>_fill(marv_plant_config*) sets every vehicle field of
                                        marv_plant_config; floating values are C hexadecimal floating literals (exact).
  marv_plant_card_<vehicle>.plugin.xml  the plugin configuration as an SDF <plugin> element, consumed by gen_sdf.py;
                                        decimal values are Python repr (float() reads the identical double).

Vehicle fields are exactly those of marv_plant_config that the card defines. Scenario fields (site_lat_rad,
site_height_m, motor_substep_s, rng_seed) are not vehicle data and are never emitted; the consumer supplies them.
struct_size is ABI plumbing, set by the generated fill function from the caller's sizeof.

The card file must be named <vehicle>.yaml so that a build can name the outputs without reading the card.
Exit 0 on success. Exit 1 with one line per finding on stderr, and nothing written, when the card or profile does not
lint clean, a needed value is UNKNOWN, or the card file name is not <vehicle>.yaml. A pole_count of UNKNOWN is not a
refusal: it becomes 0, which the plant ABI defines as unknown.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import lint
import schema

ROOT = lint.ROOT
PLANT_HEADER = Path("sim") / "plant" / "include" / "marv_plant.h"
MOTORS = ("m1", "m2", "m3", "m4")
SPIN_SIGN = {"ccw": 1, "cw": -1}
ESC_ENUM = {"linear_in_omega": "MARV_PLANT_ESC_LINEAR_IN_OMEGA"}
VEHICLE_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
PLUGIN_FILENAME = "marv_gz_lockstep"
PLUGIN_NAME = "marv::gz::Lockstep"
SCENARIO_FIELDS = ("site_lat_rad", "site_height_m", "motor_substep_s", "rng_seed")
# The header comment also names the optional initial rotor speed (decision 0007; 0 = at rest). The plugin element's
# comment keeps SCENARIO_FIELDS so that every generated world stays byte-identical.
HEADER_SCENARIO_FIELDS = (*SCENARIO_FIELDS, "initial_omega_rad_s")
SCALAR_FIELDS = ("mass_kg", "thrust_coeff", "torque_ratio_m", "omega_min_rad_s", "omega_max_rad_s", "motor_tau_s")


class GenError(Exception):
    """Refusal to generate; `lines` are the findings, one per line."""

    def __init__(self, lines):
        super().__init__("; ".join(lines))
        self.lines = list(lines)


def refuse(card_path, path, reason):
    raise GenError([f"{card_path}: {path}: {reason}"])


def load_linted(card_path, root=ROOT):
    """Return (card, profile) documents, or raise GenError with the linter's findings."""
    out = schema.Findings()
    lint.lint_card(card_path, out, root=root)
    if out:
        raise GenError(out.lines())
    card = schema.load_yaml(card_path)
    pfile = Path(root) / lint.PROFILE_DIR / f"{card['sensor_profile']}.yaml"
    profile = schema.load_yaml(pfile)
    vehicle = card["vehicle"]
    if not VEHICLE_ID.fullmatch(vehicle):
        refuse(card_path, "vehicle", f"{vehicle!r} is not an identifier [A-Za-z][A-Za-z0-9_]*")
    if Path(card_path).stem != vehicle:
        refuse(card_path, "vehicle", f"the card file must be named {vehicle}.yaml, not {Path(card_path).name}")
    return card, profile


def _known(entry, path, card_path):
    if entry["value"] == schema.UNKNOWN:
        refuse(card_path, path, f"value is {schema.UNKNOWN}; the plant configuration needs it")
    return entry["value"]


def _esc_enum_value(root, model):
    header = (Path(root) / PLANT_HEADER).read_text(encoding="utf-8")
    m = re.search(rf"\b{ESC_ENUM[model]}\s*=\s*(\d+)", header)
    if not m:
        raise GenError([f"{PLANT_HEADER}: {ESC_ENUM[model]} not found"])
    return int(m.group(1))


def config_from(card, profile, card_path, root=ROOT):
    """(cfg, units): cfg is the ordered dict of every vehicle field of marv_plant_config in the struct's order,
    per-motor lists indexed by logical motor - 1; units maps a field name to its unit text from the card."""
    rotors = card["rotors"]
    units = {}
    cfg = {}
    cfg["esc_map"] = _esc_enum_value(root, rotors["esc_map"]["model"])
    poles = profile["classes"]["rotor_speed"]["entries"].get("pole_count")
    if poles is None:
        refuse(card_path, "profile rotor_speed.entries.pole_count", "missing")
    cfg["pole_count"] = 0 if poles["value"] == schema.UNKNOWN else poles["value"]
    cfg["yaw_sign"] = [SPIN_SIGN[rotors[m]["spin"]["value"]] for m in MOTORS]
    cfg["mass_kg"] = float(_known(card["mass"], "mass", card_path))
    units["mass_kg"] = card["mass"]["unit"]
    cfg["rotor_position_frd_m"] = [
        [float(v) for v in _known(rotors[m]["position"], f"rotors.{m}.position", card_path)] for m in MOTORS
    ]
    units["rotor_position_frd_m"] = rotors["m1"]["position"]["unit"]
    cfg["thrust_coeff"] = float(_known(rotors["thrust_coeff"], "rotors.thrust_coeff", card_path))
    units["thrust_coeff"] = rotors["thrust_coeff"]["unit"]
    cfg["torque_ratio_m"] = float(_known(rotors["torque_ratio"], "rotors.torque_ratio", card_path))
    units["torque_ratio_m"] = rotors["torque_ratio"]["unit"]
    speed = _known(rotors["speed_range"], "rotors.speed_range", card_path)
    cfg["omega_min_rad_s"] = float(speed[0])
    cfg["omega_max_rad_s"] = float(speed[1])
    units["omega_min_rad_s"] = units["omega_max_rad_s"] = rotors["speed_range"]["unit"]
    cfg["motor_tau_s"] = float(_known(rotors["motor_lag"]["tau"], "rotors.motor_lag.tau", card_path))
    units["motor_tau_s"] = rotors["motor_lag"]["tau"]["unit"]
    return cfg, units


def plant_config(card_path, root=ROOT):
    """Every vehicle field of marv_plant_config, taken from the linted card, as an ordered dict."""
    card, profile = load_linted(card_path, root)
    return config_from(card, profile, card_path, root)[0]


def _hex(x):
    return float.hex(float(x))


def render_header(cfg, card, card_path):
    v = card["vehicle"]
    digest = hashlib.sha256(Path(card_path).read_bytes()).hexdigest()
    guard = f"MARV_PLANT_CARD_{v.upper()}_H"
    lines = [
        f"/* Generated by tools/card/gen_plant_config.py from {Path(card_path).name} (sha256 {digest}).",
        " * Build directory only, never committed. Edit the card, not this file.",
        " *",
        f" * marv_plant_card_{v}_fill sets every VEHICLE field of marv_plant_config (and struct_size). The caller",
        " * must set the scenario fields, which are not vehicle data: " + ", ".join(HEADER_SCENARIO_FIELDS) + ".",
        " * Floating values are C hexadecimal floating literals (exact); the decimal repr is in the trailing comment. */",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        '#include "marv_plant.h"',
        "",
        f"static inline void marv_plant_card_{v}_fill(marv_plant_config* c) {{",
        "  c->struct_size = (uint32_t)sizeof(*c);",
        f"  c->esc_map = {ESC_ENUM[card['rotors']['esc_map']['model']]};",
        f"  c->pole_count = {cfg['pole_count']}u;",
    ]
    for i, s in enumerate(cfg["yaw_sign"]):
        lines.append(f"  c->yaw_sign[{i}] = {s};")
    lines.append(f"  c->mass_kg = {_hex(cfg['mass_kg'])};  /* {cfg['mass_kg']!r} */")
    for i, pos in enumerate(cfg["rotor_position_frd_m"]):
        for k, x in enumerate(pos):
            lines.append(f"  c->rotor_position_frd_m[{i}][{k}] = {_hex(x)};  /* {x!r} */")
    for name in SCALAR_FIELDS[1:]:
        lines.append(f"  c->{name} = {_hex(cfg[name])};  /* {cfg[name]!r} */")
    lines += ["}", "", f"#endif /* {guard} */", ""]
    return "\n".join(lines)


def text_element(parent, tag, text, unit=None, **attrs):
    e = ET.SubElement(parent, tag, {**({"unit": unit} if unit else {}), **attrs})
    e.text = text
    return e


def plugin_element(cfg, units, card):
    plugin = ET.Element("plugin", {"filename": PLUGIN_FILENAME, "name": PLUGIN_NAME})
    plugin.append(ET.Comment(
        f" Generated from the vehicle card {card['vehicle']}. filename and name are provisional names for the L2 "
        "plugin. Vehicle fields of marv_plant_config only; the scenario fields (" + ", ".join(SCENARIO_FIELDS) +
        ") are not vehicle data and are supplied by the consumer. Motor numbers are logical, 1 to 4. "))
    text_element(plugin, "esc_map", str(cfg["esc_map"]), model=card["rotors"]["esc_map"]["model"])
    text_element(plugin, "pole_count", str(cfg["pole_count"]))
    for name in SCALAR_FIELDS:
        text_element(plugin, name, repr(cfg[name]), units[name])
    for i in range(len(cfg["yaw_sign"])):
        rotor = ET.SubElement(plugin, "rotor", {"motor": str(i + 1)})
        text_element(rotor, "position_frd_m", " ".join(repr(x) for x in cfg["rotor_position_frd_m"][i]),
                      units["rotor_position_frd_m"])
        text_element(rotor, "yaw_sign", str(cfg["yaw_sign"][i]))
    return plugin


def to_text(element):
    ET.indent(element)
    return ET.tostring(element, encoding="unicode") + "\n"


def generate(card_path, root=ROOT):
    """Return {file name: text} for the two outputs, or raise GenError."""
    card, profile = load_linted(card_path, root)
    cfg, units = config_from(card, profile, card_path, root)
    v = card["vehicle"]
    return {
        f"marv_plant_card_{v}.h": render_header(cfg, card, card_path),
        f"marv_plant_card_{v}.plugin.xml": to_text(plugin_element(cfg, units, card)),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True, metavar="FILE")
    ap.add_argument("--out-dir", required=True, metavar="DIR")
    ap.add_argument("--root", default=str(ROOT), help="repository root for resolving sensor_profile")
    args = ap.parse_args(argv)
    try:
        files = generate(args.card, args.root)
    except GenError as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, text in files.items():
        (out_dir / name).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
