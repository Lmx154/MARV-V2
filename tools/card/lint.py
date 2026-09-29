#!/usr/bin/env python3
"""Linter for vehicle cards, sensor profiles and the design-budget register (core contracts 2.1, 2.2, 5).

  lint.py --card <file> | --profile <file> | --budget <file>   (each may be repeated; --root sets the repository
                                                                root used to resolve a card's sensor_profile)

Exit 0 when every file is clean. Exit 1 otherwise, with one line per finding on stderr:

  <file>: <entry path>: <reason>

The entry schema and the sigma policy live in schema.py; this file only knows the structure of each file kind.

Card structure (top level): vehicle, mass, inertia_diag, rotors, sensor_profile. rotors holds layout: quad_x,
thrust_coeff, torque_ratio, speed_range, motor_lag {model: first_order, tau}, esc_map (a model entry:
model: linear_in_omega, with method, source and sigma), and m1..m4, each
{position, spin}. Position is FRD metres and must lie in the quadrant of the logical motor number (core 3):
m1 front-right (+x, +y), m2 rear-left (-x, -y), m3 front-left (+x, -y), m4 rear-right (-x, +y). Spin is ccw or cw
(ccw = counter-clockwise viewed from above); a card without spin directions is rejected.

sensor_profile names sensors/profiles/<id>.yaml under the repository root; the file must exist and lint clean.

Profile structure: profile (the id, equal to the file name without .yaml) and classes, a mapping from sensor class
to {part, entries: {name: entry}}. imu, high_g_accel, barometer and rotor_speed are required.

Budget structure: a mapping from entry name to a register entry (schema.py).
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

import schema

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIR = Path("sensors") / "profiles"
IDENTIFIER = re.compile(r"[a-z][a-z0-9_]*")
NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")

CARD_KEYS = ("vehicle", "mass", "inertia_diag", "rotors", "sensor_profile")
ROTOR_SCALARS = {"thrust_coeff": "N/(rad/s)^2", "torque_ratio": "m", "speed_range": "rad/s"}
ESC_MODELS = ("linear_in_omega",)
TOP_UNITS = {"mass": "kg", "inertia_diag": "kg m^2"}
MOTOR_SHAPES = {"position": "frd3"}
QUADRANT = {"m1": (1, 1), "m2": (-1, -1), "m3": (1, -1), "m4": (-1, 1)}
QUADRANT_NAME = {"m1": "front-right (+x, +y)", "m2": "rear-left (-x, -y)", "m3": "front-left (+x, -y)",
                 "m4": "rear-right (-x, +y)"}
SPINS = ("ccw", "cw")
PROFILE_CLASSES_REQUIRED = ("imu", "high_g_accel", "barometer", "rotor_speed")
PROFILE_CLASSES_OPTIONAL = ("gnss", "magnetometer", "range", "battery", "cv_module", "pyro_channel")


def load(path, out):
    try:
        doc = schema.load_yaml(path)
    except OSError as e:
        out.add(path, "(file)", f"cannot read: {e.strerror}")
        return None
    except yaml.YAMLError as e:
        out.add(path, "(file)", str(e).replace("\n", " "))
        return None
    if not isinstance(doc, dict):
        out.add(path, "(file)", "top level must be a mapping")
        return None
    return doc


def _unit_of(entry, expected, path, out, file):
    if isinstance(entry, dict) and schema.is_text(entry.get("unit")) and entry["unit"] != expected:
        out.add(file, path, f"unit: must be {expected!r}, got {entry['unit']!r}")


def _entry(entry, path, out, file, unit=None, shape=None, **kw):
    schema.check_entry(entry, path, out, file, **kw)
    if unit is not None:
        _unit_of(entry, unit, path, out, file)
    if shape is not None and isinstance(entry, dict) and entry.get("value") not in (None, schema.UNKNOWN) \
            and entry.get("shape") != shape \
            and (entry.get("shape") is not None or not isinstance(entry.get("value"), list)):
        out.add(file, path, f"shape: this entry must be a {shape} list, got shape {entry.get('shape')!r}")


def _position_quadrant(motor, entry, path, out, file):
    value = entry.get("value") if isinstance(entry, dict) else None
    if not (isinstance(value, list) and len(value) >= 2 and all(schema.is_number(v) for v in value)):
        return
    sx, sy = QUADRANT[motor]
    if value[0] * sx <= 0 or value[1] * sy <= 0:
        out.add(file, path, f"value: {motor} is {QUADRANT_NAME[motor]} in FRD (core 3); position "
                            f"[{value[0]}, {value[1]}, ...] is in the wrong quadrant")


def _check_rotors(rotors, out, file):
    if not isinstance(rotors, dict):
        out.add(file, "rotors", "must be a mapping")
        return
    allowed = ("layout", "motor_lag", "esc_map", *ROTOR_SCALARS, *QUADRANT)
    for k in rotors:
        if k not in allowed:
            out.add(file, "rotors", f"unknown field {k!r}")
    if rotors.get("layout") != "quad_x":
        out.add(file, "rotors.layout", f"must be quad_x, got {rotors.get('layout')!r}")
    for name, unit in ROTOR_SCALARS.items():
        p = f"rotors.{name}"
        if name not in rotors:
            out.add(file, p, "missing")
            continue
        _entry(rotors[name], p, out, file, unit=unit, shape="range" if name == "speed_range" else None)
    if "esc_map" not in rotors:
        out.add(file, "rotors.esc_map", "missing")
    else:
        _entry(rotors["esc_map"], "rotors.esc_map", out, file, models=ESC_MODELS)
    lag = rotors.get("motor_lag")
    if "motor_lag" not in rotors:
        out.add(file, "rotors.motor_lag", "missing")
    elif not isinstance(lag, dict):
        out.add(file, "rotors.motor_lag", "must be a mapping {model: first_order, tau: <entry>}")
    else:
        for k in lag:
            if k not in ("model", "tau"):
                out.add(file, "rotors.motor_lag", f"unknown field {k!r}")
        if lag.get("model") != "first_order":
            out.add(file, "rotors.motor_lag.model", f"must be first_order, got {lag.get('model')!r}")
        if "tau" not in lag:
            out.add(file, "rotors.motor_lag.tau", "missing")
        else:
            _entry(lag["tau"], "rotors.motor_lag.tau", out, file, unit="s")
    for motor in QUADRANT:
        mp = f"rotors.{motor}"
        m = rotors.get(motor)
        if motor not in rotors:
            out.add(file, mp, "missing (every motor m1..m4 needs a position and a spin direction)")
            continue
        if not isinstance(m, dict):
            out.add(file, mp, "must be a mapping {position, spin}")
            continue
        for k in m:
            if k not in ("position", "spin"):
                out.add(file, mp, f"unknown field {k!r}")
        if "position" not in m:
            out.add(file, f"{mp}.position", "missing")
        else:
            _entry(m["position"], f"{mp}.position", out, file, unit="m", shape="frd3")
            _position_quadrant(motor, m["position"], f"{mp}.position", out, file)
        if "spin" not in m:
            out.add(file, f"{mp}.spin", "missing: a card without spin directions is rejected (core 3)")
        else:
            _entry(m["spin"], f"{mp}.spin", out, file, categories=SPINS)


def lint_card(path, out, root=ROOT):
    doc = load(path, out)
    if doc is None:
        return
    for k in doc:
        if k not in CARD_KEYS:
            out.add(path, k, "unknown top-level field")
    if not schema.is_text(doc.get("vehicle")):
        out.add(path, "vehicle", "missing or empty (the vehicle id)")
    for name, unit in TOP_UNITS.items():
        if name not in doc:
            out.add(path, name, "missing")
        else:
            _entry(doc[name], name, out, path, unit=unit, shape="diag3" if name == "inertia_diag" else None)
    if "rotors" not in doc:
        out.add(path, "rotors", "missing")
    else:
        _check_rotors(doc["rotors"], out, path)
    pid = doc.get("sensor_profile")
    if pid is None:
        out.add(path, "sensor_profile", "missing (a profile id)")
    elif not (isinstance(pid, str) and IDENTIFIER.fullmatch(pid)):
        out.add(path, "sensor_profile", f"must be a profile id [a-z][a-z0-9_]*, got {pid!r}")
    else:
        pfile = Path(root) / PROFILE_DIR / f"{pid}.yaml"
        if not pfile.is_file():
            out.add(path, "sensor_profile", f"profile file {PROFILE_DIR / (pid + '.yaml')} not found under {root}")
        else:
            lint_profile(pfile, out, expect_id=pid)


def lint_profile(path, out, expect_id=None):
    doc = load(path, out)
    if doc is None:
        return
    for k in doc:
        if k not in ("profile", "classes"):
            out.add(path, k, "unknown top-level field")
    pid = doc.get("profile")
    if not schema.is_text(pid):
        out.add(path, "profile", "missing or empty (the profile id)")
    elif pid != Path(path).stem or (expect_id is not None and pid != expect_id):
        out.add(path, "profile", f"id {pid!r} must equal the file name {Path(path).stem!r}")
    classes = doc.get("classes")
    if not isinstance(classes, dict):
        out.add(path, "classes", "missing or not a mapping of sensor class -> {part, entries}")
        return
    for c in PROFILE_CLASSES_REQUIRED:
        if c not in classes:
            out.add(path, f"classes.{c}", "missing (required sensor class)")
    for c, body in classes.items():
        cp = f"classes.{c}"
        if c not in PROFILE_CLASSES_REQUIRED + PROFILE_CLASSES_OPTIONAL:
            out.add(path, cp, "unknown sensor class")
            continue
        if not isinstance(body, dict):
            out.add(path, cp, "must be a mapping {part, entries}")
            continue
        for k in body:
            if k not in ("part", "entries"):
                out.add(path, cp, f"unknown field {k!r}")
        if not schema.is_text(body.get("part")):
            out.add(path, f"{cp}.part", "missing or empty (the part name)")
        entries = body.get("entries")
        if not isinstance(entries, dict) or not entries:
            out.add(path, f"{cp}.entries", "missing or empty (a mapping of entry name -> entry)")
            continue
        for name, entry in entries.items():
            if not (isinstance(name, str) and NAME.fullmatch(name)):
                out.add(path, f"{cp}.entries", f"entry name {name!r} is not an identifier")
                continue
            schema.check_entry(entry, f"{cp}.entries.{name}", out, path)


def lint_budget(path, out):
    doc = load(path, out)
    if doc is None:
        return
    for name, entry in doc.items():
        if not (isinstance(name, str) and NAME.fullmatch(name)):
            out.add(path, str(name), "entry name is not an identifier")
            continue
        schema.check_entry(entry, name, out, path, budget=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", action="append", default=[], metavar="FILE")
    ap.add_argument("--profile", action="append", default=[], metavar="FILE")
    ap.add_argument("--budget", action="append", default=[], metavar="FILE")
    ap.add_argument("--root", default=str(ROOT), help="repository root for resolving sensor_profile")
    args = ap.parse_args(argv)
    if not (args.card or args.profile or args.budget):
        ap.error("give at least one of --card, --profile, --budget")
    out = schema.Findings()
    for f in args.card:
        lint_card(f, out, root=args.root)
    for f in args.profile:
        lint_profile(f, out)
    for f in args.budget:
        lint_budget(f, out)
    if out:
        for line in out.lines():
            print(line, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
