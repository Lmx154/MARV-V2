#!/usr/bin/env python3
"""Flatten a vehicle card and the design-budget register into params_gen input (core contracts 2.1, 2.2, 4).

  flatten.py --card <card> --budget <budget> --out-card <file> --out-register <file> [--out-mixer <file>]
             [--root <repo>]

The card and the budget are linted first (lint.py's checks, which use schema.py); on any finding nothing is written
and the exit status is 1. Otherwise two params_gen source files are written, deterministically and byte-stable:
--out-card (params_gen --card, origin default-from-card) and --out-register (params_gen --register). Each entry is
one line of JSON, which is YAML flow syntax; floats are written with repr, so params_gen reads the identical value.

Card entries become parameters named <quantity>[_m<n>]:

  mass                      mass                    f32
  inertia_diag              inertia                 diag3 -> inertia_xx _yy _zz
  rotors.thrust_coeff       rotor_thrust_coeff      f32
  rotors.torque_ratio       rotor_torque_ratio      f32
  rotors.speed_range        rotor_speed             range -> rotor_speed_min _max
  rotors.motor_lag.tau      motor_tau               f32
  rotors.m<n>.position      rotor_position_m<n>     frd3 -> rotor_position_m<n>_x _y _z
  rotors.m<n>.spin          rotor_yaw_sign_m<n>     i32, +1 for ccw, -1 for cw: the sign of the reaction torque on
                                                    the body about +z_FRD; derived, sigma exact

method, source, unit, sigma, lock, shape, status, conflict and check are carried through verbatim; sigma_rule and
note are dropped (params_gen has no field for them). A card entry whose value is UNKNOWN is not emitted, and neither
are the model entries (rotors.esc_map, rotors.motor_lag.model); the header comment of --out-card lists them. Sensor
profile entries are not flattened at L1; they join the firmware with L6.

--out-mixer (optional) adds a third params_gen --card file: idle_speed and the 16 L3 mixer parameters
mixer_m<i>_{thrust,roll,pitch,yaw} (mixer.py, decision 0004). When mixer.py refuses the card nothing is written and the
exit status is 1.

Every budget entry with a numeric value becomes a register entry (sigma choice, method design-budget, unit carried).
A budget entry whose value is UNKNOWN is not emitted, so a consumer of it fails to compile.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import gen_plant_config as gpc
import lint
import mixer
import schema

FORWARD = ("unit", "method", "source", "sigma", "lock", "shape", "status", "conflict", "check")
SPIN_SIGN = {"ccw": 1, "cw": -1}
YAW_METHOD = "derived(ccw → +1, cw → −1: reaction torque sign about +z_FRD)"
MOTORS = ("m1", "m2", "m3", "m4")


def _rel(path, root):
    p = Path(path).resolve()
    try:
        return p.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return p.name


def _lock(lock):
    return {k: (v.isoformat() if hasattr(v, "isoformat") else v) for k, v in lock.items()}


def _carry(src, out):
    for k in FORWARD:
        if k in src:
            out[k] = _lock(src[k]) if k == "lock" else src[k]
    return out


def flatten_card(doc, skipped):
    """Return the ordered list of (parameter name, params_gen entry) for a linted card."""
    out = []

    def add(name, entry, path):
        if entry["value"] == schema.UNKNOWN:
            skipped.append(f"{path}: value UNKNOWN")
            return
        ptype = "i32" if entry["sigma"] == schema.EXACT else "f32"
        out.append((name, _carry(entry, {"type": ptype, "value": entry["value"]})))

    add("mass", doc["mass"], "mass")
    add("inertia", doc["inertia_diag"], "inertia_diag")
    rotors = doc["rotors"]
    add("rotor_thrust_coeff", rotors["thrust_coeff"], "rotors.thrust_coeff")
    add("rotor_torque_ratio", rotors["torque_ratio"], "rotors.torque_ratio")
    add("rotor_speed", rotors["speed_range"], "rotors.speed_range")
    add("motor_tau", rotors["motor_lag"]["tau"], "rotors.motor_lag.tau")
    skipped.append("rotors.motor_lag.model: model entry (first_order), not a number")
    skipped.append(f"rotors.esc_map: model entry ({rotors['esc_map']['model']}), not a number")
    for m in MOTORS:
        add(f"rotor_position_{m}", rotors[m]["position"], f"rotors.{m}.position")
        spin = rotors[m]["spin"]
        entry = {"type": "i32", "value": SPIN_SIGN[spin["value"]], "unit": "1", "method": YAW_METHOD,
                 "source": f"vehicle card rotors.{m}.spin = {spin['value']} (ccw = counter-clockwise viewed from "
                           f"above); {spin['source']}",
                 "sigma": schema.EXACT}
        for k in ("lock", "status", "check"):
            if k in spin:
                entry[k] = _lock(spin[k]) if k == "lock" else spin[k]
        out.append((f"rotor_yaw_sign_{m}", entry))
    return out


def flatten_budget(doc, root, budget_path, skipped):
    out = []
    where = _rel(budget_path, root)
    for name, entry in doc.items():
        if not schema.is_number(entry["value"]):
            skipped.append(f"{name}: value UNKNOWN")
            continue
        e = {"type": "f32", "value": entry["value"], "unit": entry["unit"], "method": entry["method"],
             "source": f"design-budget register {where}, entry {name} (rationale there)", "sigma": entry["sigma"]}
        for k in ("status", "check"):
            if k in entry:
                e[k] = entry[k]
        out.append((name, e))
    return out


def render(header, entries):
    lines = [f"# {h}" if h else "#" for h in header]
    for name, entry in entries:
        lines.append(f"{name}: {json.dumps(entry, ensure_ascii=True)}")
    return "\n".join(lines) + "\n"


def lint_all(card, budget, root):
    out = schema.Findings()
    lint.lint_card(card, out, root=root)
    lint.lint_budget(budget, out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True)
    ap.add_argument("--budget", required=True)
    ap.add_argument("--out-card", required=True)
    ap.add_argument("--out-register", required=True)
    ap.add_argument("--out-mixer")
    ap.add_argument("--root", default=str(lint.ROOT), help="repository root (resolves sensor_profile)")
    args = ap.parse_args(argv)

    findings = lint_all(args.card, args.budget, args.root)
    if findings:
        for line in findings.lines():
            print(line, file=sys.stderr)
        return 1

    card = schema.load_yaml(args.card)
    budget = schema.load_yaml(args.budget)
    card_skipped, budget_skipped = [], []
    card_entries = flatten_card(card, card_skipped)
    register_entries = flatten_budget(budget, args.root, args.budget, budget_skipped)

    mixer_entries = None
    if args.out_mixer:
        try:
            mixer_entries = mixer.mixer_entries(card, args.card)
        except gpc.GenError as e:
            for line in e.lines:
                print(line, file=sys.stderr)
            return 1

    card_header = [
        f"params_gen input flattened from vehicle card {_rel(args.card, args.root)} by tools/card/flatten.py. "
        "Generated; do not edit.",
        f"Vehicle: {card['vehicle']}.",
        f"Sensor profile {card['sensor_profile']} is not flattened at L1; its entries join the firmware with L6.",
        "Not emitted:",
        *[f"  {s}" for s in card_skipped],
        *[f"  rotors.{m}.spin: carried by rotor_yaw_sign_{m}" for m in MOTORS],
    ]
    register_header = [
        f"params_gen input flattened from design-budget register {_rel(args.budget, args.root)} by "
        "tools/card/flatten.py. Generated; do not edit.",
        "Not emitted (a consumer of these does not compile until the value is set):",
        *[f"  {s}" for s in budget_skipped],
    ]
    card_text = render(card_header, card_entries)
    register_text = render(register_header, register_entries)
    outputs = [(args.out_card, card_text), (args.out_register, register_text)]
    if mixer_entries is not None:
        mixer_header = [
            f"params_gen input for the L3 mixer, computed from vehicle card {_rel(args.card, args.root)} by "
            "tools/card/mixer.py through tools/card/flatten.py. Generated; do not edit.",
            "idle_speed and the mixer M = B^-1 (row i of M is motor i); see mixer.py for B and the refusals.",
        ]
        outputs.append((args.out_mixer, render(mixer_header, mixer_entries)))
    for path, text in outputs:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
