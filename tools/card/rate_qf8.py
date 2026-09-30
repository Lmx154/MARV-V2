#!/usr/bin/env python3
"""QF-8 crossover-vs-loop-rate curve for the L4 rate-loop gain rule (quad spec QF-8 section 6.2, decision 0005).

  rate_qf8.py --card <card> --budget <budget> --scenario <scenario register> --output <file> [--root <repo>]
  rate_qf8.py --from-header <table> --card <card> --output <file>

For the loop period T_n = 2^n t_tick, n = 0..9, runs tools/card/rate.py design() (the committed gain generator,
imported, not copied) with rate_loop_divisor overridden to 2^n and writes a plain-text table. The output has no
timestamps; the header carries the exact input values the rule consumes (repr), each with its source file and entry, and the
command line (without --output and --root). There is no file hash: an unrelated edit to a shared input must not change
the table.

--from-header regenerates a table from the input values recorded in the header of an earlier table (its fixed-input
golden), not from the live budget and scenario register: the values the rule consumes are parsed from the header and
substituted into the card's document. The card is still read for what the table does not consume (the mixer and rotor
entries design() evaluates after the crossover), and the scenario's rate_max_<axis> take the placeholder RATE_MAX_UNUSED
(design() uses them only for the authority term, no column of the table). The output for the same values is
byte-identical to the live-file mode.
"""

from __future__ import annotations

import argparse
import ast
import copy
import math
import re
import sys
from pathlib import Path

import flatten
import gen_plant_config as gpc
import lint
import rate
import schema

LOG2_DIVISORS = range(10)
PROFILE = "sensors/profiles/marv_v2_board_default.yaml"
COLUMNS = ("n", "rate_Hz", "T_s", "w_c_nom", "w_c_min", "w_c_max", "PM_nom_deg", "PM_worst_deg", "kappa_p", "kappa_i",
           "tau_cl_s", "condition", "dw_c_rel")
CORNERS = ("J-,tau-", "J-,tau+", "J+,tau-", "J+,tau+")
RATE_MAX_UNUSED = 1.0  # rad/s, placeholder: no column of the table depends on rate_max (scenario value, unused)
AXES = ("roll", "pitch", "yaw")
INPUT_LINE = re.compile(r"^  (?P<label>.+) = (?P<value>.+)   \[(?P<file>[^:\]]+): (?P<entry>[^\]]+)\]$")


def _num(x):
    return f"{x:.9g}"


def _rel(path, root):
    p = Path(path).resolve()
    try:
        return p.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return p.name


def table_lines(card_path, budget_path, scenario_path, root):
    findings = flatten.lint_all(card_path, budget_path, root, scenario_path)
    if findings:
        raise gpc.GenError(findings.lines())
    return _rows(schema.load_yaml(card_path), schema.load_yaml(budget_path), schema.load_yaml(scenario_path), card_path)


def _rows(card, budget, scenario, card_path):
    rows, previous = [], None
    for n in LOG2_DIVISORS:
        override = copy.deepcopy(scenario)
        override["rate_loop_divisor"]["value"] = 2 ** n
        try:
            r = rate.design(card, budget, override, card_path)
        except gpc.GenError as e:
            why = e.lines[0].split(": rate: ", 1)[-1]
            rows.append([str(n), _num(1 / (override["tick_period_num_us"]["value"] * 2 ** n / override["tick_period_den"]["value"] * 1e-6))] + [f"refused: {why}"])
            previous = None
            continue
        crossovers = [m[2] for m in r["margins"] if m[0] in CORNERS]
        delta = "-" if previous is None else _num((r["omega_c"] - previous) / previous)
        rows.append([str(n), _num(1 / r["T"]), _num(r["T"]), _num(r["omega_c"]), _num(min(crossovers)),
                     _num(max(crossovers)), _num(math.degrees(r["pm_nom"])), _num(math.degrees(r["pm_worst"])),
                     _num(r["kappa_p"]), _num(r["kappa_i"]), _num(r["tau_cl"]), _num(r["condition"]), delta])
        previous = r["omega_c"]
    return rows


def render(card_path, budget_path, scenario_path, root):
    rows = table_lines(card_path, budget_path, scenario_path, root)
    card = schema.load_yaml(card_path)
    budget = schema.load_yaml(budget_path)
    scenario = schema.load_yaml(scenario_path)
    profile = schema.load_yaml(Path(root) / PROFILE)
    card_file, budget_file, scenario_file = (_rel(p, root) for p in (card_path, budget_path, scenario_path))
    values = [
        ("J_roll, J_pitch, J_yaw (kg m^2)", repr(tuple(card["inertia_diag"]["value"])), card_file, "inertia_diag"),
        ("motor tau (s)", repr(card["rotors"]["motor_lag"]["tau"]["value"]), card_file, "rotors.motor_lag.tau"),
    ]
    values += [(f"{k} ({budget[k]['unit']})", repr(budget[k]["value"]), budget_file, k)
               for k in ("PM_min", "tau_robustness_band", "inertia_robustness_band")]
    values += [(f"{k} ({scenario[k]['unit']})", repr(scenario[k]["value"]), scenario_file, k)
               for k in ("tick_period_num_us", "tick_period_den")]
    values.append(("IMU ODR range [min, max] (Hz)", repr(tuple(profile["classes"]["imu"]["entries"]["odr"]["value"])),
                   PROFILE, "classes.imu.entries.odr"))
    command = ("uv run python tools/card/rate_qf8.py --card " + _rel(card_path, root) + " --budget "
               + _rel(budget_path, root) + " --scenario " + _rel(scenario_path, root) + " --output <file>")
    return _text(rows, values, command)


def parse_header(text):
    """(values, command) recorded in a table's header: values = [(label, value repr, file, entry)], command the header's
    command line (without the "command: " prefix)."""
    values, command = [], None
    for line in text.splitlines():
        m = INPUT_LINE.match(line)
        if m:
            values.append((m["label"], m["value"], m["file"], m["entry"]))
        elif line.startswith("command: "):
            command = line[len("command: "):]
            break
    if not values or command is None:
        raise gpc.GenError(["rate_qf8: no input values or command line in the header"])
    return values, command


def docs_from_header(values, card):
    """(card, budget, scenario) documents for rate.design() with the header's values: `card` (a loaded document, deep
    copied) gets inertia_diag and rotors.motor_lag.tau; the budget and the scenario hold only what design() reads."""
    by_entry = {entry: ast.literal_eval(value) for _, value, _, entry in values}
    needed = ("inertia_diag", "rotors.motor_lag.tau", "PM_min", "tau_robustness_band", "inertia_robustness_band",
              "tick_period_num_us", "tick_period_den")
    missing = [k for k in needed if k not in by_entry]
    if missing:
        raise gpc.GenError([f"rate_qf8: the header lacks {missing}"])
    card = copy.deepcopy(card)
    card["inertia_diag"]["value"] = list(by_entry["inertia_diag"])
    card["rotors"]["motor_lag"]["tau"]["value"] = by_entry["rotors.motor_lag.tau"]
    budget = {k: {"value": by_entry[k]} for k in needed[2:5]}
    scenario = {k: {"value": by_entry[k]} for k in needed[5:]}
    scenario["rate_loop_divisor"] = {"value": 1}
    scenario.update({f"rate_max_{a}": {"value": RATE_MAX_UNUSED} for a in AXES})
    return card, budget, scenario


def render_from_header(header_path, card_path):
    values, command = parse_header(Path(header_path).read_text(encoding="utf-8"))
    card, budget, scenario = docs_from_header(values, schema.load_yaml(card_path))
    return _text(_rows(card, budget, scenario, card_path), values, command)


def _text(rows, values, command):
    lines = [
        "MARV QF-8 crossover-vs-loop-rate curve (tools/card/rate_qf8.py, decision 0005, quad spec section 6.2 QF-8)",
        "inputs the rule consumes (value [source file: entry]); rate_loop_divisor is overridden to 2^n per row:",
    ]
    lines += [f"  {label} = {value}   [{file}: {entry}]" for label, value, file, entry in values]
    lines += [
        "command: " + command,
        "QF-8 verdict: UNKNOWN - needs the motor tau sigma (card sigma UNKNOWN); the convergence threshold is that "
        "uncertainty (quad section 6.2 QF-8)",
        "Spec gap: QF-8 ignores gyro filtering and aliasing; revisit at L6 (decision 0005)",
        "The listed rates are integer divisions of the 6.4 kHz tick; that each is an available ODR of the IMU is "
        "INFERRED (the profile records only the range)",
        "",
        "Each row is the L4 gain rule (rate.py design(), PM_min and the tau x J band box from the budget) with "
        "rate_loop_divisor = 2^n.",
        "rate_Hz = 1/T; w_c_nom, w_c_min, w_c_max in rad/s (w_c_nom the nominal crossover of the rule, w_c_min and "
        "w_c_max the smallest and largest",
        "crossover over the four box corners J in {1 -+ b_J} x tau in {(1 -+ b_tau) tau}); PM in degrees "
        "(PM_worst = minimum over nominal and corners); kappa_p, kappa_i for J = 1;",
        "tau_cl_s = t63 worst over the box; condition = w_c^2 T tau_lo (rate.py step 7, must be < 1/2); dw_c_rel = "
        "(w_c_nom - w_c_nom of the next-faster rate) / (w_c_nom of the next-faster rate).",
        "Floats are printed with 9 significant digits (%.9g).",
        "",
        "  ".join(COLUMNS),
    ]
    lines += ["  ".join(row) for row in rows]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True)
    ap.add_argument("--budget")
    ap.add_argument("--scenario")
    ap.add_argument("--from-header", help="a table whose header records the input values to use (module docstring)")
    ap.add_argument("--output", required=True)
    ap.add_argument("--root", default=str(lint.ROOT), help="repository root (paths in the header are relative to it)")
    args = ap.parse_args(argv)
    if bool(args.from_header) == bool(args.budget or args.scenario) or bool(args.budget) != bool(args.scenario):
        ap.error("give --from-header, or --budget and --scenario together, not both")
    try:
        if args.from_header:
            text = render_from_header(args.from_header, args.card)
        else:
            text = render(args.card, args.budget, args.scenario, args.root)
    except gpc.GenError as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
