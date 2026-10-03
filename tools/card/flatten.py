#!/usr/bin/env python3
"""Flatten a vehicle card and the design-budget register into params_gen input (core contracts 2.1, 2.2, 4).

  flatten.py --card <card> --budget <budget> --out-card <file> --out-register <file> [--out-mixer <file>]
             [--scenario <file> --out-scenario <file>] [--out-rate <file>] [--out-attitude <file>]
             [--out-gyro-chain <file>] [--out-rate-lead <file>] [--out-attitude-lead <file>] [--procs <n>] [--root <repo>]

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

--scenario <register> with --out-scenario <file> (optional, given together) lints the scenario register
(design/scenario_values.yaml, lint.py --scenario) with the rest and writes a fourth params_gen --card file: every entry
with a numeric value becomes a parameter of the same name (method scenario, sigma choice, unit carried); a Python
integer value is type i32, any other number f32 (so 625 is i32 and 625.0 is f32). Without the flags nothing else
changes: the other outputs are written by the same code from the same inputs.

--out-rate <file> (optional, needs --scenario) adds a fifth params_gen --card file: the 12 L4 rate-loop parameters
rate_{kp,ki,kd,tau_ref}_{roll,pitch,yaw} (rate.py, decision 0005), and next to it the derivation report
<file stem>_report.txt. When rate.py refuses the card, budget or scenario register nothing is written and the exit
status is 1. Without the flag the other outputs are byte-identical.

--out-attitude <file> (optional, needs --scenario) adds a sixth params_gen --card file: att_kp, att_yaw_weight,
att_loop_ratio (= 1, the parent-rate rule of core 7.5), att_yaw_alpha_min and att_yaw_t_cross (attitude.py, decision 0006
E), and next to it the derivation report <file stem>_report.txt. The rate loop is designed in memory for it (rate.py), so
--out-rate is not needed. When attitude.py or rate.py refuses, nothing is written and the exit status is 1. Without the flag
the other outputs are byte-identical.

--out-gyro-chain <file> (optional, needs --scenario) adds a seventh params_gen --card file: gyro_lpf_cutoff_hz,
gyro_notch_q_h1, gyro_notch_q_h2, gyro_notch_q_h3 and gyro_notch_omega_min (gyro_chain_params.py, which applies
gyro_chain_design.py's rules, decision 0013), and next to it the derivation report <file stem>_report.txt and the C++ header
<file stem>_design.hpp (the relative tracking error eps and its three terms, for tests; not parameters). The inputs
include the sensor profile the card names (imu odr_error, rotor_speed esc_clock_error). When gyro_chain_params.py refuses
(an UNKNOWN input, an esc_clock_error above the budget's esc_clock_error_max), nothing is written and the exit status is
1. Without the flag the other outputs are byte-identical.

--out-rate-lead <file> and --out-attitude-lead <file> (optional, need --scenario; each refuses to be given with its L4/L5
twin --out-rate / --out-attitude, which write the same parameter names) are the stage (c) switch of decision 0014: the same
parameter files and reports from the stage (c) rules instead of rate.py and attitude.py. --out-rate-lead writes the 16
rate-loop parameters rate_{kp,ki,kd,d_filter_tau,tau_ref}_{roll,pitch,yaw} and rate_ff_filter_tau (rate_lead.py: PI x lead
with the gyro chain; tau_ref by rate.py rule step 9 with rate_lead's tau_cl; T_ff by rule step 9) and the report <file stem>_report.txt. --out-attitude-lead writes the
same five parameters as --out-attitude (att_kp, att_yaw_weight, att_loop_ratio, att_yaw_alpha_min, att_yaw_t_cross) from
attitude_lead.py on the stage (c) rate loop, and its report (which sets the new design against today's attitude.py one); the
rate loop is designed in memory for it, once for both flags. The sensor profile the card names feeds the gyro chain.
--procs <n> (default tools/card/cpu_quota.py usable_cpus()) is the worker count of rate_lead's N search; the outputs do not
depend on it. rate.py and attitude.py stay the L4/L5 PI reference. When rate_lead.py or attitude_lead.py refuses, nothing is
written and the exit status is 1. Without the flags the other outputs are byte-identical.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import attitude
import attitude_lead
import cpu_quota
import gen_plant_config as gpc
import gyro_chain_design as gcd
import gyro_chain_params
import lint
import mixer
import rate
import rate_lead
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


def flatten_scenario(doc, root, scenario_path, skipped):
    out = []
    where = _rel(scenario_path, root)
    for name, entry in doc.items():
        if not schema.is_number(entry["value"]):
            skipped.append(f"{name}: value UNKNOWN")
            continue
        ptype = "i32" if isinstance(entry["value"], int) else "f32"
        out.append((name, {"type": ptype, "value": entry["value"], "unit": entry["unit"], "method": entry["method"],
                           "source": f"scenario-values register {where}, entry {name} (rationale there)",
                           "sigma": entry["sigma"]}))
    return out


def render(header, entries):
    lines = [f"# {h}" if h else "#" for h in header]
    for name, entry in entries:
        lines.append(f"{name}: {json.dumps(entry, ensure_ascii=True)}")
    return "\n".join(lines) + "\n"


def lint_all(card, budget, root, scenario=None):
    out = schema.Findings()
    lint.lint_card(card, out, root=root)
    lint.lint_budget(budget, out)
    profile = lint.profile_of_card(card, root)
    if profile is not None:
        lint.lint_profile_budget(profile, budget, out)
    if scenario:
        lint.lint_scenario(scenario, out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True)
    ap.add_argument("--budget", required=True)
    ap.add_argument("--out-card", required=True)
    ap.add_argument("--out-register", required=True)
    ap.add_argument("--out-mixer")
    ap.add_argument("--scenario")
    ap.add_argument("--out-scenario")
    ap.add_argument("--out-rate")
    ap.add_argument("--out-attitude")
    ap.add_argument("--out-gyro-chain")
    ap.add_argument("--out-rate-lead")
    ap.add_argument("--out-attitude-lead")
    ap.add_argument("--procs", type=int, default=cpu_quota.usable_cpus(),
                    help="worker processes of rate_lead's N search (the outputs do not depend on it)")
    ap.add_argument("--root", default=str(lint.ROOT), help="repository root (resolves sensor_profile)")
    args = ap.parse_args(argv)

    if bool(args.scenario) != bool(args.out_scenario):
        ap.error("--scenario and --out-scenario are given together")
    if args.out_rate and not args.scenario:
        ap.error("--out-rate needs --scenario (the rate loop period and maximum rates are scenario values)")
    if args.out_attitude and not args.scenario:
        ap.error("--out-attitude needs --scenario (the loop period is a scenario value)")
    if args.out_gyro_chain and not args.scenario:
        ap.error("--out-gyro-chain needs --scenario (the tick period and the rate-loop divisor are scenario values)")

    if args.out_rate_lead and not args.scenario:
        ap.error("--out-rate-lead needs --scenario (the rate loop period and maximum rates are scenario values)")
    if args.out_attitude_lead and not args.scenario:
        ap.error("--out-attitude-lead needs --scenario (the loop period is a scenario value)")
    if args.out_rate_lead and args.out_rate:
        ap.error("--out-rate-lead and --out-rate write the same parameters")
    if args.out_attitude_lead and args.out_attitude:
        ap.error("--out-attitude-lead and --out-attitude write the same parameters")

    findings = lint_all(args.card, args.budget, args.root, args.scenario)
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

    rate_entries = None
    rate_result = None
    if args.out_rate:
        try:
            rate_entries, rate_report, rate_result = rate.rate_entries(card, budget, schema.load_yaml(args.scenario),
                                                                       args.card, _rel(args.card, args.root))
        except gpc.GenError as e:
            for line in e.lines:
                print(line, file=sys.stderr)
            return 1

    attitude_entries = None
    if args.out_attitude:
        try:
            attitude_entries, attitude_report, _ = attitude.attitude_entries(
                card, budget, schema.load_yaml(args.scenario), args.card, _rel(args.card, args.root), rate_result)
        except gpc.GenError as e:
            for line in e.lines:
                print(line, file=sys.stderr)
            return 1

    lead_rate_entries = lead_attitude_entries = None
    if args.out_rate_lead or args.out_attitude_lead:
        profile_path = Path(args.root) / lint.PROFILE_DIR / f"{card['sensor_profile']}.yaml"
        where = _rel(args.card, args.root)
        try:
            scenario_doc = schema.load_yaml(args.scenario)
            lead = rate_lead.design(card, budget, scenario_doc, schema.load_yaml(profile_path), args.card, profile_path,
                                    args.procs)
            if args.out_rate_lead:
                lead_rate_entries = rate_lead.entries_from(lead)
                lead_rate_report = rate_lead.report_lines(lead)
            if args.out_attitude_lead:
                today = attitude.design(card, budget, scenario_doc, args.card, lead["pi_l4"])
                att = attitude_lead.design(card, budget, scenario_doc, args.card, attitude_lead.lead_inner(lead),
                                           lead["pi_l4"])
                lead_attitude_entries = attitude_lead.entries_from(att)
                lead_attitude_report = attitude_lead.report_lines(att, today, lead)
        except (gpc.GenError, gcd.DesignError) as e:
            for line in getattr(e, "lines", None) or [str(e)]:
                print(line, file=sys.stderr)
            return 1

    gyro_chain_entries = None
    if args.out_gyro_chain:
        profile_path = Path(args.root) / lint.PROFILE_DIR / f"{card['sensor_profile']}.yaml"
        try:
            gyro_chain_entries, gyro_chain_report, gyro_chain_design = gyro_chain_params.gyro_chain_entries(
                card, budget, schema.load_yaml(args.scenario), schema.load_yaml(profile_path), profile_path,
                _rel(args.card, args.root))
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
    if args.scenario:
        scenario_skipped = []
        scenario_entries = flatten_scenario(schema.load_yaml(args.scenario), args.root, args.scenario,
                                            scenario_skipped)
        scenario_header = [
            f"params_gen input flattened from scenario-values register {_rel(args.scenario, args.root)} by "
            "tools/card/flatten.py. Generated; do not edit.",
            "Not emitted (a consumer of these does not compile until the value is set):",
            *[f"  {s}" for s in scenario_skipped],
        ]
        outputs.append((args.out_scenario, render(scenario_header, scenario_entries)))
    if rate_entries is not None:
        rate_header = [
            f"params_gen input for the L4 rate loop, computed from vehicle card {_rel(args.card, args.root)}, the "
            "design budget and the scenario register by tools/card/rate.py through tools/card/flatten.py. Generated; "
            "do not edit.",
            "rate_kp, rate_ki, rate_kd (= 0) and rate_tau_ref per axis (decision 0005); the derivation is in the "
            "report next to this file.",
        ]
        outputs.append((args.out_rate, render(rate_header, rate_entries)))
        report_path = Path(args.out_rate).with_name(Path(args.out_rate).stem + "_report.txt")
        outputs.append((str(report_path), "\n".join(rate_report) + "\n"))
    if attitude_entries is not None:
        attitude_header = [
            f"params_gen input for the L5 attitude loop, computed from vehicle card {_rel(args.card, args.root)}, the "
            "design budget and the scenario register by tools/card/attitude.py through tools/card/flatten.py. Generated; "
            "do not edit.",
            "att_kp, att_yaw_weight, att_loop_ratio, att_yaw_alpha_min and att_yaw_t_cross (decision 0006 E); the derivation "
            "is in the report next to this file.",
        ]
        outputs.append((args.out_attitude, render(attitude_header, attitude_entries)))
        attitude_report_path = Path(args.out_attitude).with_name(Path(args.out_attitude).stem + "_report.txt")
        outputs.append((str(attitude_report_path), "\n".join(attitude_report) + "\n"))
    if lead_rate_entries is not None:
        lead_rate_header = [
            f"params_gen input for the stage (c) rate loop, computed from vehicle card {_rel(args.card, args.root)}, the "
            "design budget, the scenario register and the sensor profile by tools/card/rate_lead.py through "
            "tools/card/flatten.py. Generated; do not edit.",
            "rate_kp, rate_ki, rate_kd, rate_d_filter_tau and rate_tau_ref per axis (decision 0014); the derivation is in the "
            "report next to this file.",
        ]
        outputs.append((args.out_rate_lead, render(lead_rate_header, lead_rate_entries)))
        report_path = Path(args.out_rate_lead).with_name(Path(args.out_rate_lead).stem + "_report.txt")
        outputs.append((str(report_path), "\n".join(lead_rate_report) + "\n"))
    if lead_attitude_entries is not None:
        lead_attitude_header = [
            f"params_gen input for the stage (c) attitude loop, computed from vehicle card {_rel(args.card, args.root)}, the "
            "design budget, the scenario register and the sensor profile by tools/card/attitude_lead.py through "
            "tools/card/flatten.py. Generated; do not edit.",
            "att_kp, att_yaw_weight, att_loop_ratio, att_yaw_alpha_min and att_yaw_t_cross on the stage (c) rate loop "
            "(decisions 0006 E, 0014 D7); the derivation is in the report next to this file.",
        ]
        outputs.append((args.out_attitude_lead, render(lead_attitude_header, lead_attitude_entries)))
        report_path = Path(args.out_attitude_lead).with_name(Path(args.out_attitude_lead).stem + "_report.txt")
        outputs.append((str(report_path), "\n".join(lead_attitude_report) + "\n"))
    if gyro_chain_entries is not None:
        gyro_chain_header = [
            f"params_gen input for the L6 gyro chain, computed from vehicle card {_rel(args.card, args.root)}, the design "
            "budget, the scenario register and the sensor profile by tools/card/gyro_chain_params.py through "
            "tools/card/flatten.py. Generated; do not edit.",
            "gyro_lpf_cutoff_hz, gyro_notch_q_h1..h3 and gyro_notch_omega_min (decision 0013); the derivation is in the "
            "report next to this file.",
        ]
        outputs.append((args.out_gyro_chain, render(gyro_chain_header, gyro_chain_entries)))
        gyro_chain_report_path = Path(args.out_gyro_chain).with_name(Path(args.out_gyro_chain).stem + "_report.txt")
        outputs.append((str(gyro_chain_report_path), "\n".join(gyro_chain_report) + "\n"))
        gyro_chain_design_path = Path(args.out_gyro_chain).with_name(Path(args.out_gyro_chain).stem + "_design.hpp")
        outputs.append((str(gyro_chain_design_path),
                        gyro_chain_params.design_header_text(gyro_chain_design, _rel(args.card, args.root))))
    for path, text in outputs:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
