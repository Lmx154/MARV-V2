#!/usr/bin/env python3
"""QF-3 margins of a parameter table's rate loop (ground-segment unit 9, decision 0053 G7 check 3; ctest qf3_table_fc3v2,
label config-validation).

  qf3_table.py --manifest <params_manifest.json> --factory <factory .mcfg> --card <card> --budget <budget>
               --scenario <scenario register> [--config <.mcfg>] [--gain-factor <f>] [--procs <n>]

The table: --config, else the file named by the environment variable MARV_VALIDATE_CONFIG (the validator,
tools/validate/validate_config.py, injects the candidate config so), else --factory (the build's factory config, the frozen
behaviour). The manifest is the image's (its schema hash must be the table's).

The check is the generator's own (tools/card/rate_lead.py rule steps 8 and 13, the rule that accepted the factory gains), run
on the table's values instead of the designed ones: worst PM >= PM_min and worst Ms <= Ms_max per axis, over nominal and the
four (J, tau) corners, on the firmware loop (the f32 gains, the f32 alpha with expf moved by -+LIBM_EXP_ULPS), over the
configuration set (the notches of all four motors at one speed on the resolved notch grid, and every notch bypassed, each
at the profile's latency and at latency 0).
  - From the table (what the firmware reads): rate_kp_<a>, rate_ki_<a>, rate_kd_<a>, rate_d_filter_tau_<a> (T_f per axis),
    gyro_lpf_cutoff_hz, gyro_notch_q_h1..h3, gyro_notch_omega_min, rate_loop_divisor, tick_period_num_us, tick_period_den.
    A chain value equal to the f32 of the generator's design value is evaluated at the design value (the generator's
    guard ran on the double chain); any other is evaluated at the table's f32.
  - From the card and its sensor profile (the plant): the inertia and the motor tau (rate.design's inputs), the rotor speed
    range (the notch grid's top), the latency.
  - Thresholds and bands, never from the table (decision 0053, lead 2026-10-18: a candidate cannot loosen its own bound):
    PM_min, Ms_max, tau_robustness_band and inertia_robustness_band from the budget; each must equal, rounded to f32, the
    image's compiled value (the manifest's default), else the budget is not this image's and the tool refuses.
  - The executions' dt: the board's mean period, one dt = D num/den (marv_params_fc3v2 builds with --rate-guard mean_period,
    decision 0014 Amendment 2026-10-16).

--gain-factor multiplies rate_kp, rate_ki and rate_kd of every axis (rounded to f32) before the check: the negative control
(qf3_table_fc3v2_negative_control, gain factor 1.1 per core 7.2 item 3) must fail.

Prints one line per axis (worst PM in rad and degrees against PM_min, worst Ms against Ms_max, where each binds) and a last
line PASS or FAIL. Exit 0 on PASS, 1 on FAIL, 2 when the table or an input cannot be read.

The attitude loop: attitude_lead.py rule steps 2', 5' and 6' at the table's att_kp (att_yaw_weight for yaw, att_loop_ratio must
be 1) on the table's rate loop as above: worst PM >= PM_min over nominal and the four corners of each axis and every loop of
the configuration set (notch-grid level L, then the configurations level L + 1 adds, until they lower nothing) certified
stable by the Stein certificate.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "config"))
import cpu_quota  # noqa: E402
import multiprocessing  # noqa: E402
import gen_imu_config as gic  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import attitude  # noqa: E402
import attitude_lead  # noqa: E402
import gyro_chain_params as gcp  # noqa: E402
import lint  # noqa: E402
import mcfg  # noqa: E402
import rate  # noqa: E402
import rate_lead  # noqa: E402
import schema  # noqa: E402

ENV = "MARV_VALIDATE_CONFIG"
AXES = rate.AXES
THRESHOLDS = (("PM_min", "PM_min"), ("Ms_max", "Ms_max"), ("tau_robustness_band", "tau_robustness_band"),
              ("inertia_robustness_band", "inertia_robustness_band"))


class InputError(Exception):
    pass


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def table_values(config_path, manifest):
    """{name: value} of the config's section of the manifest's schema (f32 as the exact double, i32 as int)."""
    try:
        cfg = mcfg.decode(Path(config_path).read_bytes())
    except (OSError, mcfg.McfgError) as e:
        raise InputError(f"{config_path}: {e}") from None
    schema_hash = mcfg.manifest_schema_hash(manifest)
    section = next((s for s in cfg.sections if s.schema_hash == schema_hash), None)
    if section is None:
        raise InputError(f"{config_path}: no section has the image's schema hash {manifest['schema_hash']}")
    records = mcfg.section_records(section)
    names = list(manifest["params"])
    if len(records) != len(names):
        raise InputError(f"{config_path}: {len(records)} records, the manifest has {len(names)}")
    return {n: (mcfg.f32_of(r.value_bits) if r.type == "f32" else mcfg.i32_of(r.value_bits))
            for n, r in zip(names, records, strict=True)}


def compiled_value(manifest, name):
    e = manifest["params"][name]
    bits = int(e["default_bits"], 16)
    return mcfg.f32_of(bits) if e["type"] == "f32" else mcfg.i32_of(bits)


def thresholds(budget, manifest, card_path):
    out = {}
    for key, param in THRESHOLDS:
        v = float(rate._value(budget, key, "budget entry", card_path))
        if param in manifest["params"] and f32(v) != compiled_value(manifest, param):
            raise InputError(f"budget {key} = {v!r} is not the image's compiled {param} = "
                             f"{compiled_value(manifest, param)!r} (rounded to f32): the budget is not this image's")
        out[key] = v
    return out


def evaluate(values, card_path, budget_path, scenario_path, gain_factor=1.0, procs=1, manifest=None):
    """The check's result: dict(rows per axis, set worst, pass, inputs)."""
    card = schema.load_yaml(card_path)
    budget = schema.load_yaml(budget_path)
    scenario = schema.load_yaml(scenario_path)
    profile_path = ROOT / lint.PROFILE_DIR / f"{card['sensor_profile']}.yaml"
    profile = schema.load_yaml(profile_path)
    th = thresholds(budget, manifest, card_path) if manifest is not None else {
        k: float(rate._value(budget, k, "budget entry", card_path)) for k, _ in THRESHOLDS}
    pm_min, ms_max = th["PM_min"], th["Ms_max"]
    pi = rate.design(card, budget, scenario, card_path)
    inp = pi["inputs"]
    chain = dict(gcp.design(card, budget, scenario, profile, profile_path, card_path))
    si, _, _ = gic.imu_config(profile_path)

    num, den, divisor = int(values["tick_period_num_us"]), int(values["tick_period_den"]), int(values["rate_loop_divisor"])
    if num <= 0 or den <= 0 or divisor <= 0:
        raise InputError(f"tick {num}/{den} us or divisor {divisor} is not positive")
    t_s = num / den / rate_lead.MICRO_PER_UNIT

    def chain_value(design_value, table_value):
        return design_value if f32(design_value) == table_value else table_value

    same_tick = (num, den, divisor) == (scenario["tick_period_num_us"]["value"], scenario["tick_period_den"]["value"],
                                        chain["divisor"])
    chain["t_s"] = chain["t_s"] if same_tick else t_s
    chain["divisor"] = divisor
    chain["f_c"] = chain_value(chain["f_c"], values["gyro_lpf_cutoff_hz"])
    chain["q"] = tuple(chain_value(q, values[f"gyro_notch_q_h{h}"]) for q, h in zip(chain["q"], gcd.HARMONICS, strict=True))
    chain["omega_th"] = chain_value(chain["omega_th"], values["gyro_notch_omega_min"])

    corners = rate.corner_list(inp["tau"], th["tau_robustness_band"], th["inertia_robustness_band"])
    stages = gcd.chain_stages(chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"], [chain["omega_th"]] * rate_lead.MOTORS)
    model = rate_lead.Model(chain["t_s"], divisor, si["latency_samples"], stages, corners, rate.spacing(pm_min), inp["inertia"])
    dts = [divisor * num / den]

    axes32 = []
    for a in AXES:
        kp, ki, kd = (f32(values[f"rate_{g}_{a}"] * gain_factor) if gain_factor != 1.0 else values[f"rate_{g}_{a}"]
                      for g in ("kp", "ki", "kd"))
        axes32.append((kp, ki, kd, values[f"rate_d_filter_tau_{a}"]))
    for a, ax in zip(AXES, axes32, strict=True):
        if not all(math.isfinite(x) for x in ax):
            raise InputError(f"axis {a}: a gain or T_f is not finite: {ax}")
        if not ax[3] > 0:
            raise InputError(f"axis {a}: rate_d_filter_tau_{a} = {ax[3]!r} is not positive")
    for _, _, tau in corners:
        model.plant_cache(tau)
    cset = rate_lead.configuration_rows(model, chain, axes32, dts, max(1, procs), card_path)
    per_axis = []
    for i, a in enumerate(AXES):
        pm = min((g[i][0], label, g[i][1]) for _, label, g in cset["rows"])
        ms = max((g[i][2], label, g[i][3]) for _, label, g in cset["rows"])
        per_axis.append({"axis": a, "pm": pm[0], "pm_at": f"{pm[1]}, {pm[2]}", "ms": ms[0], "ms_at": f"{ms[1]}, {ms[2]}",
                         "pass": pm[0] >= pm_min and ms[0] <= ms_max})
    att = attitude_eval(values, pi, chain, si["latency_samples"], axes32, pm_min, max(1, procs))
    return {"axes": per_axis, "pm_min": pm_min, "ms_max": ms_max, "level": cset["level"],
            "configurations": len(cset["rows"]), "attitude": att,
            "pass": all(r["pass"] for r in per_axis) and att["pass"],
            "chain": {"f_c": chain["f_c"], "q": chain["q"], "omega_th": chain["omega_th"], "t_s": chain["t_s"],
                      "divisor": divisor}}


def _att_worst(rr, inner, cfgs, k32, w32, procs):
    loops, units = attitude_lead.set_loops(rr, inner, cfgs, attitude_lead.SetLoop)
    table = attitude_lead.CertTable(units)
    if procs <= 1:
        return attitude.pm_worst(loops, k32, w32)
    attitude_lead._UNITS[:] = units
    try:
        with multiprocessing.get_context("fork").Pool(procs) as pool:
            table.map = lambda jobs: pool.map(attitude_lead._unit_job, jobs, chunksize=1)
            return attitude.pm_worst(loops, k32, w32)
    finally:
        table.map = None
        attitude_lead._UNITS[:] = []


def attitude_eval(values, rr, chain, latency, axes32, pm_min, procs):
    """attitude_lead rule 6' at the table's att_kp: dict(pm, at, level, configurations, unstable, pass)."""
    if int(values["att_loop_ratio"]) != attitude.LOOP_RATIO:
        raise InputError(f"att_loop_ratio {values['att_loop_ratio']}: the attitude loop's closed form is exact at "
                         f"{attitude.LOOP_RATIO} only (attitude_lead.py step 2')")
    k32, w32 = values["att_kp"], values["att_yaw_weight"]
    if not (math.isfinite(k32) and math.isfinite(w32) and k32 > 0 and w32 > 0):
        raise InputError(f"att_kp {k32!r} or att_yaw_weight {w32!r} is not finite and positive")

    def configs(level):
        return [(key, name, cl, [s for s in st if s != gcd.IDENTITY])
                for key, name, cl, st in rate_lead.configuration_set(chain, latency, level)]

    stages = gcd.chain_stages(chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"],
                              [chain["omega_th"]] * rate_lead.MOTORS)
    inner = attitude_lead.Inner("the table's rate loop", axes32, chain["t_s"], chain["divisor"], latency, stages, configs)
    level = 0
    while True:
        cfgs = inner.configs(level)
        worst, detail = _att_worst(rr, inner, cfgs, k32, w32, procs)
        known = {c[0] for c in cfgs}
        added = [c for c in inner.configs(level + 1) if c[0] not in known]
        worst_new, detail_new = _att_worst(rr, inner, added, k32, w32, procs)
        if worst_new >= worst or level >= rate_lead.NOTCH_GRID_HALVINGS_MAX:
            break
        level += 1
    rows = [d for d in detail + detail_new]
    unstable = [f"{a} {name}: {why}" for a, name, _, _, _, why in rows if why]
    finite = [(pm, f"{a} {name}") for a, name, pm, _, _, why in rows if pm is not None and not why]
    pm, at = min(finite) if finite else (-math.inf, "no loop")
    resolved = worst_new >= worst
    return {"pm": pm if not unstable else -math.inf, "at": at, "level": level, "configurations": len(cfgs) + len(added),
            "unstable": unstable, "resolved": resolved, "k32": k32, "w32": w32,
            "pass": resolved and not unstable and pm >= pm_min}


def report_lines(r, table, gain_factor):
    deg = 180.0 / math.pi
    lines = [f"qf3_table: table {table}" + (f", rate kp/ki/kd x {gain_factor!r} (negative control)" if gain_factor != 1.0
                                           else ""),
             f"qf3_table: configuration set level {r['level']} ({r['configurations']} configurations); thresholds from the "
             f"budget, equal to the image's compiled values: PM_min {r['pm_min']!r} rad, Ms_max {r['ms_max']!r}"]
    for a in r["axes"]:
        lines.append(f"qf3_table: rate {a['axis']} worst PM {a['pm'] * deg:.4f} deg ({a['pm']!r} rad) at {a['pm_at']}, "
                     f"limit {r['pm_min'] * deg:.4f} deg, margin {(a['pm'] - r['pm_min']) * deg:+.3e} deg; worst Ms "
                     f"{a['ms']:.6f} at {a['ms_at']}, limit {r['ms_max']!r}, margin {r['ms_max'] - a['ms']:+.3e}; "
                     f"{'PASS' if a['pass'] else 'FAIL'}")
    t = r["attitude"]
    pm = t["pm"]
    lines.append(f"qf3_table: attitude att_kp {t['k32']!r} 1/s (yaw weight {t['w32']!r}) worst PM "
                 + (f"{pm * deg:.4f} deg ({pm!r} rad) at {t['at']}, limit {r['pm_min'] * deg:.4f} deg, margin "
                    f"{(pm - r['pm_min']) * deg:+.3e} deg" if math.isfinite(pm) else "-inf")
                 + f"; configuration set level {t['level']} ({t['configurations']} configurations)"
                 + ("" if t["resolved"] else ", NOT resolved")
                 + (f"; not certified stable: {'; '.join(t['unstable'][:4])}" if t["unstable"] else "; every loop certified stable")
                 + f"; {'PASS' if t['pass'] else 'FAIL'}")
    lines.append(f"qf3_table: {'PASS' if r['pass'] else 'FAIL'}")
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description="QF-3 rate-loop margins of a parameter table (the generator's guard)")
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--factory", type=Path, required=True, help="the build's factory .mcfg (used without --config or the env)")
    ap.add_argument("--config", type=Path, help=f"the table to check (default: ${ENV}, else --factory)")
    ap.add_argument("--card", type=Path, required=True)
    ap.add_argument("--budget", type=Path, required=True)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--gain-factor", type=float, default=1.0)
    ap.add_argument("--procs", type=int, default=0, help="worker processes (default: usable CPUs); the result does not depend on it")
    args = ap.parse_args(argv)
    table = args.config or (Path(os.environ[ENV]) if os.environ.get(ENV) else args.factory)
    try:
        manifest = mcfg.load_manifest(args.manifest)
        values = table_values(table, manifest)
        r = evaluate(values, args.card, args.budget, args.scenario, args.gain_factor,
                     args.procs or cpu_quota.usable_cpus(), manifest)
    except (InputError, mcfg.McfgError, gpc.GenError, gcd.DesignError, KeyError, ValueError) as e:
        lines = getattr(e, "lines", None) or [str(e)]
        for line in lines:
            print(f"qf3_table: cannot check: {line}", file=sys.stderr)
        print("qf3_table: FAIL")
        return 2
    for line in report_lines(r, table, args.gain_factor):
        print(line)
    return 0 if r["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
