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
     rule, with every notch bypassed, each at the f32 firmware loop (dt, f32 alpha, expf -+LIBM_EXP_ULPS). Worst PM and Ms over
     the flight configurations (latency L); step 13's latency-0 configurations (the truth gyro) do not depend on L and are left
     out. Two sets of rows: the NOMINAL rows (the card's nominal J and tau only: the plant of a HIL or SIL run is the card's
     nominal model, core section 6) and the worst-corner rows (nominal and the four band corners of rate.corner_list).
  3. A delay of N ticks on the DShot command is a pure delay of N T_s in the loop, as a latency of N samples is (the adapter's
     delay line applies the command of tick j - N at tick j; the IMU latency is the plant's sample delay line, both counted in
     ticks), so the loop at command delay N and latency l is the loop at latency l + N.
  4. N_max (decision 0046 item 2, as re-ruled): the largest N such that every N' <= N has every NOMINAL row meeting QF-3, worst
     PM >= PM_min AND worst Ms <= Ms_max; None if N = 0 already fails. The worst-corner table is printed beside it, labelled
     "worst corner: not a margin result for HIL": the gain rule puts the worst corner at PM_min, so its N_max is 0 by
     construction.
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
EXTRA_DEFAULT = 20  # labelled: the worst-corner table covers N = 0..20, twice the HIL S1 requirement of 10 (decision 0046 item 2)
EXTRA_LISTED = 20  # labelled: rows listed one by one up to here
NOMINAL_STRIDE = 25  # labelled: then every 25th row
NOMINAL_SCAN_MAX = 4096  # labelled: the nominal scan stops at the first failing N; refused if none fails by here (a runaway guard)


def flight_profile(card_path):
    card = rate_lead.schema.load_yaml(card_path)
    return ROOT / "sensors" / "profiles" / f"{card['sensor_profile']}.yaml"


def margins_at(result, latency, procs, card_path, nominal_only):
    """(level, worst PM, worst Ms, configuration count): PM and Ms are (value, configuration label, axis, corner) over the flight
    configurations at IMU-equivalent latency; nominal_only keeps the nominal corner (card J and tau) alone."""
    base = result["model"]
    corners = base.corners[:1] if nominal_only else base.corners
    assert not nominal_only or corners[0][0] == "nominal"
    model = rate_lead.Model(base.t_s, base.divisor, latency, base.stages, corners, base.a, base.inertia)
    cset = rate_lead.configuration_rows(model, result["chain"], result["axes32"], result["dts"], procs, card_path)
    flight = [row for row in cset["rows"] if row[0][1] == latency]
    pm, ms = rate_lead.set_worst(flight)
    return cset["level"], pm, ms, len(flight)


def table(card_path, profile_path, scenario_path, budget_path, extra, procs):
    card, budget, scenario, profile = rate_lead.load(card_path, budget_path, scenario_path, profile_path)
    result = rate_lead.design(card, budget, scenario, profile, str(card_path), str(profile_path), procs, "mean_period")
    inp = result["inputs"]
    base_latency = inp["latency"]
    rows, n_max, n = [], None, 0
    while True:
        row = {"n": n, "latency": base_latency + n, "nom": margins_at(result, base_latency + n, procs, str(card_path), True)}
        row["pass"] = row["nom"][1][0] >= inp["PM_min"] and row["nom"][2][0] <= inp["Ms_max"]
        if n <= extra:
            row["worst"] = margins_at(result, base_latency + n, procs, str(card_path), False)
        rows.append(row)
        if not row["pass"]:
            break
        n_max = n
        n += 1
        if n > NOMINAL_SCAN_MAX:
            raise gcd.DesignError(f"the nominal rows still pass at N = {NOMINAL_SCAN_MAX}; raise NOMINAL_SCAN_MAX")
    while len(rows) <= extra:  # the nominal scan ended inside the worst-corner table
        n = len(rows)
        rows.append({"n": n, "latency": base_latency + n, "pass": False,
                     "nom": margins_at(result, base_latency + n, procs, str(card_path), True),
                     "worst": margins_at(result, base_latency + n, procs, str(card_path), False)})
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
        "N = command delay in ticks; latency = profile latency + N samples. Worst over the flight configuration set (step 13:",
        "notch grid at the level that step resolves, bypassed) x axes x libm variants.",
        "NOMINAL rows (the card's nominal J and tau): the HIL/SIL plant. N_max is decided here: PM >= PM_min AND Ms <= Ms_max.",
        "Worst corner (nominal and the four band corners): not a margin result for HIL.",
        f"{'N':>3} {'latency':>7} | {'nominal PM deg':>14} {'PM-PM_min deg':>13} {'Ms':>9} {'level':>5} {'cfgs':>4} {'pass':>4} | "
        f"{'worst-corner PM deg':>19} {'PM-PM_min deg':>13} {'Ms':>9} {'level':>5}  binding worst corner (configuration; axis; corner)",
    ]
    last = rows[-1]["n"]
    for r in rows:
        if r["n"] > EXTRA_LISTED and r["n"] % NOMINAL_STRIDE and r["n"] < last - 1:
            continue  # beyond the worst-corner table only every NOMINAL_STRIDE-th nominal row and the last two are listed
        nl, npm, nms, ncfg = r["nom"]
        head = (f"{r['n']:>3} {r['latency']:>7} | {deg(npm[0]):>14.6f} {deg(npm[0] - inp['PM_min']):>13.6f} {nms[0]:>9.6f} "
                f"{nl:>5} {ncfg:>4} {'yes' if r['pass'] else 'NO':>4} | ")
        if "worst" in r:
            wl, wpm, wms, _ = r["worst"]
            head += (f"{deg(wpm[0]):>19.6f} {deg(wpm[0] - inp['PM_min']):>13.6f} {wms[0]:>9.6f} {wl:>5}  "
                     f"{wpm[1]}; {wpm[2]}; {wpm[3]}")
        else:
            head += "(not evaluated beyond --extra)"
        out.append(head)
    nominal = [r["nom"] for r in rows]
    monotone = all(b[1][0] <= a[1][0] and b[2][0] >= a[2][0] for a, b in zip(nominal, nominal[1:]))
    fail = rows[-1]
    why = []
    if fail["nom"][1][0] < inp["PM_min"]:
        why.append("PM below PM_min")
    if fail["nom"][2][0] > inp["Ms_max"]:
        why.append("Ms above Ms_max")
    out += [
        f"nominal worst PM non-increasing and worst Ms non-decreasing in N over the rows computed: {monotone}",
        f"N_max (nominal rows, PM >= PM_min and Ms <= Ms_max) = {n_max if n_max is not None else 'NONE: N = 0 already fails'}"
        f"; N = {fail['n']} is the first failure ({' and '.join(why)})",
    ]
    s1 = 10  # labelled: HIL S1 requirement, ceil(1.26 ms / 133.3 us = 9.5 ticks), branch hil-s1 tests/regression/quad/L09/results
    out.append(f"HIL S1 requirement N >= {s1}: " + ("MET" if n_max is not None and n_max >= s1 else
                                                      "NOT MET: N_max is below the link's required N; the choice is Luis's"))
    first = next((r["n"] for r in rows if "worst" in r and r["worst"][1][0] < inp["PM_min"]), None)
    out.append(f"worst corner (not a margin result for HIL): first N below PM_min {first if first is not None else 'none in the table'}"
               f"; its N_max would be {first - 1 if first else 'beyond the table'}")
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
