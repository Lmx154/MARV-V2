#!/usr/bin/env python3
"""N_max of the HIL/SIL command delay (decision 0046 item 2): the largest N with the rate loop's PM at IMU-equivalent latency
L = profile latency_samples + N still at or above PM_min, the flight gains held fixed.

  hil_delay_margin.py [--card <card>] [--profile <profile>] [--scenario <register>] [--budget <budget>] [--extra N]
                      [--procs <n>]

Method (nothing here changes a design rule or a threshold; the evaluation is tools/card/rate_lead.py's own):
  1. rate_lead.design(..., rate_guard="mean_period") on the flight card, profile and scenario register (the arguments of
     marv_params_fc3v2, fw/params/CMakeLists.txt) gives the flown f32 gains (r["axes32"]), PM_min, Ms_max, the chain and the
     firmware dt (the board's mean period).
  2. For each L = latency .. latency + extra (latency = the profile's latency_samples): the model of rule step 1 with latency L
     and rate_lead.configuration_rows (rule step 13) on those gains: the notch grid resolved at L by step 13's own halving
     rule, with every notch bypassed, each at the f32 firmware loop (dt, f32 alpha, expf -+LIBM_EXP_ULPS) at nominal and the
     four band corners (the robustness box of rate.corner_list). Worst PM and Ms over the flight configurations (latency L);
     step 13's latency-0 configurations (the truth gyro) do not depend on L and are left out of the table.
  3. A delay of N ticks on the DShot command is a pure delay of N T_s in the loop, as a latency of N samples is (the adapter's
     delay line applies the command of tick j - N at tick j; the IMU latency is the plant's sample delay line, both counted in
     ticks), so the loop at command delay N and latency l is the loop at latency l + N. N_max is the largest N with every
     N' <= N passing (worst PM >= PM_min); it is None if N = 0 already fails. The Ms column is information: Ms_max is not part
     of decision 0046's N_max.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cpu_quota  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import rate_lead  # noqa: E402

ROOT = HERE.parents[1]
FLIGHT_CARD = ROOT / "vehicles" / "marv_5in_fc3v2.yaml"
FLIGHT_SCENARIO = ROOT / "fw" / "compositions" / "quad_fc" / "params" / "fc3v2_scenario_values.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
EXTRA_DEFAULT = 20  # labelled: delays 0..20 ticks, twice the HIL S1 round-trip requirement of 10 (decision 0046 item 2)


def flight_profile(card_path):
    card = rate_lead.schema.load_yaml(card_path)
    return ROOT / "sensors" / "profiles" / f"{card['sensor_profile']}.yaml"


def margins_at(result, latency, procs, card_path):
    """(level, worst PM, label, axis, corner), (worst Ms, ...) over the flight configurations at IMU-equivalent latency."""
    base = result["model"]
    model = rate_lead.Model(base.t_s, base.divisor, latency, base.stages, base.corners, base.a, base.inertia)
    cset = rate_lead.configuration_rows(model, result["chain"], result["axes32"], result["dts"], procs, card_path)
    flight = [row for row in cset["rows"] if row[0][1] == latency]
    pm, ms = rate_lead.set_worst(flight)
    return cset["level"], pm, ms, len(flight)


def table(card_path, profile_path, scenario_path, budget_path, extra, procs):
    card, budget, scenario, profile = rate_lead.load(card_path, budget_path, scenario_path, profile_path)
    result = rate_lead.design(card, budget, scenario, profile, str(card_path), str(profile_path), procs, "mean_period")
    inp = result["inputs"]
    base_latency = inp["latency"]
    rows = []
    for n in range(extra + 1):
        level, pm, ms, count = margins_at(result, base_latency + n, procs, str(card_path))
        rows.append({"n": n, "latency": base_latency + n, "level": level, "configs": count, "pm": pm, "ms": ms})
    n_max = None
    for r in rows:
        if r["pm"][0] < inp["PM_min"]:
            break
        n_max = r["n"]
    return result, rows, n_max


def lines(result, rows, n_max, card_path, profile_path, scenario_path):
    deg = math.degrees
    inp, c = result["inputs"], result["chain"]
    out = [
        "HIL command delay N_max (decision 0046 item 2), tools/card/hil_delay_margin.py",
        f"card {Path(card_path).relative_to(ROOT)}, profile {Path(profile_path).relative_to(ROOT)}, "
        f"scenario register {Path(scenario_path).relative_to(ROOT)}, rate guard mean_period (dt {result['dts']} us)",
        f"T_s {inp['T_s']!r} s, D {c['divisor']}, T {inp['T']!r} s, profile latency {inp['latency']} samples, "
        f"PM_min {deg(inp['PM_min']):.6f} deg ({inp['PM_min']!r} rad), Ms_max {inp['Ms_max']!r}",
        f"flight gains held fixed (rate_lead.design N* {result['n_star']!r}, w_c,nom {result['omega_c']!r} rad/s), f32 per axis:",
    ]
    for ax, j, g in zip(rate_lead.AXES, inp["inertia"], result["axes32"]):
        out.append(f"  {ax:<5} J {j!r}: kp {g[0]!r}  ki {g[1]!r}  kd {g[2]!r}  T_f {g[3]!r}")
    out += [
        "N = command delay in ticks; latency = profile latency + N samples; PM and Ms are the worst over the flight",
        "configuration set (step 13: notch grid at the level resolved there, bypassed) x axes x the band corners x libm variants",
        f"{'N':>3} {'latency':>7} {'level':>5} {'configs':>7} {'worst PM deg':>13} {'PM - PM_min deg':>16} {'worst Ms':>9}  "
        "binding PM (configuration; axis; corner)",
    ]
    for r in rows:
        pm, ms = r["pm"], r["ms"]
        out.append(f"{r['n']:>3} {r['latency']:>7} {r['level']:>5} {r['configs']:>7} {deg(pm[0]):>13.6f} "
                   f"{deg(pm[0] - inp['PM_min']):>16.6f} {ms[0]:>9.6f}  {pm[1]}; {pm[2]}; {pm[3]}"
                   f"{'' if pm[0] >= inp['PM_min'] else '   FAILS PM_min'}"
                   f"{'' if ms[0] <= inp['Ms_max'] else '   (Ms above Ms_max)'}")
    monotone = all(b["pm"][0] <= a["pm"][0] for a, b in zip(rows, rows[1:]))
    out += [
        f"worst PM non-increasing in N over the table: {monotone}",
        f"N_max = {n_max if n_max is not None else 'NONE: N = 0 already fails PM_min'}"
        + ("" if n_max is None or n_max < rows[-1]["n"] else f" (the table's last row; extend --extra)"),
    ]
    first_ms = next((r["n"] for r in rows if r["ms"][0] > inp["Ms_max"]), None)
    out.append(f"first N with Ms above Ms_max (information, not part of N_max): {first_ms if first_ms is not None else 'none in the table'}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", default=str(FLIGHT_CARD))
    ap.add_argument("--profile", help="default: the card's sensor_profile")
    ap.add_argument("--scenario", default=str(FLIGHT_SCENARIO))
    ap.add_argument("--budget", default=str(BUDGET))
    ap.add_argument("--extra", type=int, default=EXTRA_DEFAULT, help="delays 0..extra ticks")
    ap.add_argument("--procs", type=int, default=cpu_quota.usable_cpus(), help="worker processes (the output does not depend on it)")
    args = ap.parse_args(argv)
    profile = args.profile or str(flight_profile(args.card))
    try:
        result, rows, n_max = table(args.card, profile, args.scenario, args.budget, args.extra, args.procs)
    except (gpc.GenError, gcd.DesignError) as e:
        print(e, file=sys.stderr)
        return 1
    for line in lines(result, rows, n_max, args.card, profile, args.scenario):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
