#!/usr/bin/env python3
"""Rewrites scenarios/quad/L05/chirp_{roll,pitch,yaw}.yaml from the live design (decision 0006 F "T4 attitude chirp"; owner
decisions 7, 20 and 21).

  uv run python tools/sim/gen_l5_chirp.py [--plugin-dir build/host-gz-l5/sim/gz/plugin] [--duration-s 8] [--tail-s 15]

The band, the amplitude A_env and the window end are derived values: run_l5.chirp_design on the host-gz-l5 build's parameter table
(the live gains and N), so they regenerate cleanly when the rate loop changes (L6). Only the `script:` section of each file is
rewritten; everything above it (the world, the initial state, the hover thrust) is kept as committed. Run it after the product
parameters are regenerated; tests/regression/quad/L05/gz/test_t4_chirp.py fails with this command when the committed files differ.

Scenario test values (labelled in the files): duration D_0 = DURATION_S, a starting value that core 7.5 doubles until |PM(D) -
PM(D/2)| < E_H + U_A (the gz test re-checks it; pass the doubled value with --duration-s); tail TAIL_S, checked against the slowest
closed-loop pole of the design model (refused if it leaves more than 2^-24 of it).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import l5_scenario as l5s  # noqa: E402
import run_l5  # noqa: E402

l4 = run_l5.l4
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
DURATION_S = 8.0  # scenario test value, D_0 (see the module docstring)
TAIL_S = 15.0  # scenario test value (see the module docstring)
COMMAND = "uv run python tools/sim/gen_l5_chirp.py"
ROLL_PITCH_NOTE = ("; this is A_env, the torque-envelope amplitude: at A_env the vehicle tumbles (peak attitude error 2, no "
                   "crossover), so owner decision 20 has the test run A_env/2^k, k = 1..5, and take the margin as the minimum "
                   "PM over those that give a unique crossover and U_A as their spread (run_l5.u_terms)")


def live_design(plugin_dir=run_l5.DEFAULT_PLUGIN_DIR, card=CARD, root=ROOT):
    """chirp_design on the build's parameter table and the scenario's hover thrust."""
    _, defaults = run_l5.build_parameters(plugin_dir)
    params = l4.read_param_defaults(defaults)
    vals = l5s.values(l5s.load(SCEN / "chirp_yaw.yaml"))
    return run_l5.chirp_design(card, params, l4.r32(l4.hover_thrust(card, vals, root)), root)


def script_section(axis, d, duration_s=DURATION_S, tail_s=TAIL_S):
    """The text of the `script:` section of chirp_<axis>.yaml for the design `d` (run_l5.chirp_design)."""
    res, ax, ta, n = d["result"], d["axes"][axis], d["T_a"], d["N"]
    end = round((duration_s + tail_s) / ta)
    lo, hi = d["band"]
    detail = res["final"]["detail"]
    radius = max(x[4] for x in detail)
    worst = next(x for x in detail if x[4] == radius)
    tau_slow = ta / (1 - radius)
    if not math.exp(-tail_s / tau_slow) < 2.0 ** -24:
        raise SystemExit(f"tail_s {tail_s} s leaves e^-{tail_s / tau_slow:.3g} of the slowest pole ({tau_slow:.3g} s), "
                         "not below 2^-24: lengthen it")
    xs = [(th / ta, a, name) for a, name, pm, th, *_ in detail]
    cmin, cmax = min(xs), max(xs)
    note = ROLL_PITCH_NOTE if axis != "yaw" else ""
    regen = f"regenerate with `{COMMAND}`"
    return f"""script:
  settle_s:
    value: 1.0
    unit: s
    label: scenario
    rationale: "as L4 chirp_{axis}: the motors spin up from rest and the chirp must start at the design model's operating point; 1 s is 30.3 motor time constants of the card's tau = 0.033 s"
  segments: []
  end_attitude_execution:
    value: {end}
    unit: "1"
    label: derived
    rule: "the chirp's last execution plus the tail: (duration_s + tail_s) / T_a = ({duration_s:g} s + {tail_s:g} s) / {ta!r} s, with tail_s = {tail_s:g} s a scenario value (the identification window runs past the chirp's end until the response has decayed: the slowest closed-loop pole of the lifted design model at N = {n} has spectral radius {radius:.6f} per T_a (Jury radius, tools/card/attitude.py, axis {worst[0]} corner {worst[1]}), a time constant of {tau_slow:.3g} s, so {tail_s:g} s leaves e^-{tail_s / tau_slow:.3g} of it, below the float32 resolution 2^-24 = 6e-8 of the TRUTH quaternion); {regen}"
  chirp:
    axis:
      value: {axis}
      unit: "1"
      label: scenario
      rationale: "one axis per scenario: quad spec 4 L5 T4, the margins are measured per axis"
    amp_rad_s:
      value: {ax['amplitude_rad_s']!r}
      unit: rad/s
      label: derived
      rule: "decision 0006 F 'T4 attitude chirp': A = tau_held,{axis} / max over the band and the tau x J box of G_tau, G_tau the design model's peak torque request per rad/s of a steady sinusoid added to the rate setpoint, in closed loop with the attitude law at the build's gain and N (tools/sim/run_l5.py chirp_design, steady_torque_peak): tau_held,{axis} = {ax['tau_held_nm']!r} N m, max G_tau = {ax['peak_torque_per_rad_s']!r} N m per rad/s at {ax['peak_at_rad_s']!r} rad/s ({ax['peak_loop']}){note}; the test re-derives it from the live build; {regen}"
    w_lo_rad_s:
      value: {lo!r}
      unit: rad/s
      label: derived
      rule: "min over the box loops of the design crossover / a, the crossover |L| = 1 of the nominal loop and the four tau x J corners of every axis (tools/card/attitude.py, {cmin[0]:.4g} rad/s at {cmin[2]} on {cmin[1]}), a = {d['a']!r} (rate.py, the loop-shaping spacing of PM_min); the test re-derives it from the live build; {regen}"
    w_hi_rad_s:
      value: {hi!r}
      unit: rad/s
      label: derived
      rule: "a x max over the box loops of the design crossover ({cmax[0]:.5g} rad/s at {cmax[2]} on {cmax[1]}), a = {d['a']!r}; the test re-derives it from the live build; {regen}"
    start_attitude_execution:
      value: 0
      unit: "1"
      label: derived
      rule: "the origin execution itself: the first attitude execution at or after settle_s that keeps the stamp phase (tools/sim/run_l5.py plan)"
    duration_s:
      value: {duration_s:g}
      unit: s
      label: derived
      rule: "core 7.5 convergence from D_0 = {DURATION_S:g} s, a starting value above one period 2 pi / w_lo = {2 * math.pi / lo:.3g} s of the band's lowest frequency; doubled until |PM(D) - PM(D/2)| < E_H + U_A, both at D (tests/regression/quad/L05/gz/test_t4_chirp.py re-checks the last doubling on every run)"
"""


def render(axis, d, duration_s=DURATION_S, tail_s=TAIL_S, scen_dir=SCEN):
    """The whole text of chirp_<axis>.yaml: the committed text above `script:` and the derived script section."""
    old = (Path(scen_dir) / f"chirp_{axis}.yaml").read_text(encoding="utf-8")
    return old[:old.index("\nscript:\n") + 1] + script_section(axis, d, duration_s, tail_s)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--plugin-dir", default=str(run_l5.DEFAULT_PLUGIN_DIR))
    ap.add_argument("--duration-s", type=float, default=DURATION_S)
    ap.add_argument("--tail-s", type=float, default=TAIL_S)
    args = ap.parse_args(argv)
    d = live_design(args.plugin_dir)
    for axis in l5s.AXES:
        (SCEN / f"chirp_{axis}.yaml").write_text(render(axis, d, args.duration_s, args.tail_s), encoding="utf-8")
        print(f"wrote scenarios/quad/L05/chirp_{axis}.yaml (N = {d['N']}, A_env {d['axes'][axis]['amplitude_rad_s']!r} rad/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
