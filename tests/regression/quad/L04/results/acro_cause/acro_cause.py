#!/usr/bin/env python3
"""Cause of the acro pitch excursion and roll recovery failure (decision 0005; Luis's ruling of 2026-09-30).

  uv run python tests/regression/quad/L04/results/acro_cause/acro_cause.py \
      --out tests/regression/quad/L04/results/acro_cause/cause.txt [--plugin-dir <dir>] [--work-dir <dir>]

Needs gz-sim 8 and the host-gz-l4 build (`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`): it flies
scenarios/quad/L04/acro.yaml at m = 2 and m = 1 exactly as tests/regression/quad/L04/gz/test_t4_acro.py does
(run_l4.run_acro), replays both runs (run_l4.replay_acro, tests/regression/quad/L04/replay/l4_acro_replay), and refuses
to report unless the replay reproduces both runs bit for bit (run_l4.replay_fidelity). The rate bound is the test's
own (test_t4_acro.design_bound, (iii)); nothing here is a new number. Written to --out:
  - the inputs: scenario, card, the build's parameter table (paths and SHA-256), the replay tool;
  - the pitch excursion: every fresh execution with |w_pitch| > P + F + E, its interval and phase, and per execution the
    flags, s, t, requested and achieved pitch torque and the gyroscopic pitch torque (J_zz - J_xx) w_x w_z from the
    logged rates (Euler's equation, J_yy w_y' = tau_y + (J_zz - J_xx) w_x w_z; J the build's inertia_* values);
  - the flag and s/t counts over the combined segment and the whole run;
  - the roll collapse and release: w_roll at the end of the combined segment, its peak in the recovery window and at
    the window's end, with the gyroscopic roll torque -(J_zz - J_yy) w_y w_z at the end of the combined segment, and a
    roll trace every TRACE_STRIDE executions over the combined segment and the recovery window;
  - the verdict by Luis's rule: "documented desaturation" if a flag was set during the excursion, else gyroscopic
    coupling.
The gz logs and replay files go to --work-dir (default: a temporary directory, removed); they are not committed and a
rerun regenerates them (the runs are deterministic, decision 0003).
"""

import argparse
import hashlib
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(ROOT / "tests" / "regression" / "quad" / "L04" / "gz"))
import test_t4_acro as t4  # noqa: E402  (also puts tools/sim, tools/card and the T3 oracle on the path)
import run_l4  # noqa: E402

TRACE_STRIDE = 100  # executions between roll-trace lines (31.25 ms at 3.2 kHz): a resolution choice of the report


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def f(x):
    return "nan" if isinstance(x, float) and math.isnan(x) else repr(x)


def analyse(runs, replays, bound):
    s1, s2 = runs[1], runs[2]
    p = s1.plan
    phases, table = run_l4.acro_phases(p)
    params = t4.run_params(s1)
    jxx, jyy, jzz = params["inertia_xx"], params["inertia_yy"], params["inertia_zz"]
    ex1, ex2, rows = s1.executions, s2.executions, replays[1]
    lines = []
    out = lines.append

    def t(k):
        return ex1[k]["t_us"] / 1e6

    def gyro_pitch(k):
        wx, _, wz = ex1[k]["gyro"]
        return (jzz - jxx) * wx * wz

    def gyro_roll(k):
        _, wy, wz = ex1[k]["gyro"]
        return -(jzz - jyy) * wy * wz

    base = bound["pitch"]["P"] + bound["pitch"]["F"]
    exc = [k for k in range(p.end_execution + 1) if ex1[k]["fresh"] and ex2[k]["fresh"]
           and abs(ex1[k]["gyro"][1]) > base + abs(ex1[k]["gyro"][1] - ex2[k]["gyro"][1])]
    out("segments (number, kind, first execution, last execution, phase): " + str([list(x) for x in table]))
    out(f"inertia (build, kg m^2): J_xx {jxx!r}, J_yy {jyy!r}, J_zz {jzz!r}")
    out(f"pitch rate bound (iii): P {bound['pitch']['P']!r} + F {bound['pitch']['F']!r} + E(n)")
    out("")
    out("== pitch excursion: fresh executions with |w_pitch| > P + F + E")
    if not exc:
        out("none")
        return lines, None
    out(f"executions {exc[0]}..{exc[-1]} ({len(exc)}, contiguous {exc == list(range(exc[0], exc[-1] + 1))}), "
        f"t = {t(exc[0]):.6f}..{t(exc[-1]):.6f} s, phases {sorted({phases[k] for k in exc})}")
    worst = max(exc, key=lambda k: abs(ex1[k]["gyro"][1]) - base - abs(ex1[k]["gyro"][1] - ex2[k]["gyro"][1]))
    out(f"largest excess at execution {worst}")
    head = ("k t_s w_roll w_pitch w_yaw bound_pitch flag_r flag_p flag_y s t req_pitch ach_pitch "
            "gyro_torque_pitch")
    out("-- summary rows (first, largest excess, last)")
    out(head)

    def row_line(k):
        r, w = rows[k], ex1[k]["gyro"]
        e = abs(w[1] - ex2[k]["gyro"][1])
        return " ".join([str(k), f"{t(k):.6f}", *(f(v) for v in w), f(base + e), str(r["flag_roll"]),
                         str(r["flag_pitch"]), str(r["flag_yaw"]), f(r["s"]), f(r["t"]), f(r["req_pitch"]),
                         f(r["ach_pitch"]), f(gyro_pitch(k))])

    for k in sorted({exc[0], worst, exc[-1]}):
        out(row_line(k))
    flags = {a: sum(rows[k][f"flag_{a}"] for k in exc) for a in t4.AXES}
    reduced = sum(1 for k in exc if rows[k]["ach_pitch"] != rows[k]["req_pitch"])
    gt = [gyro_pitch(k) for k in exc]
    rq = [rows[k]["req_pitch"] for k in exc]
    out(f"flags set during the excursion: {flags}; executions with ach_pitch != req_pitch: {reduced}; s or t not 1: "
        f"{len(t4.s_t_findings([rows[k] for k in exc]))}")
    out(f"gyroscopic pitch torque over the excursion: {min(gt)!r} .. {max(gt)!r} N m; requested pitch torque "
        f"{min(rq)!r} .. {max(rq)!r} N m")
    out("")
    out("== flags and s, t (replay)")
    for name, ks in (("combined segment", [k for k in range(p.end_execution + 1) if phases[k] == "combined"]),
                     ("whole run", range(len(rows)))):
        sel = [rows[k] for k in ks]
        counts = {a: sum(r[f"flag_{a}"] for r in sel) for a in t4.AXES}
        first_last = {a: [k for k in ks if rows[k][f"flag_{a}"]] for a in t4.AXES}
        out(f"{name}: {len(sel)} executions, flags set {counts}, first/last set "
            f"{ {a: (v[0], v[-1]) if v else None for a, v in first_last.items()} }, s or t not 1: "
            f"{len(t4.s_t_findings(sel))}, executions with achieved != requested on some axis: "
            f"{sum(1 for r in sel if any(r[f'ach_{a}'] != r[f'req_{a}'] for a in t4.AXES))}")
    out("")
    out("== roll collapse and release")
    comb = [k for k in range(p.end_execution + 1) if phases[k] == "combined"]
    win = [k for k in range(p.end_execution + 1) if phases[k] == "recovery"]
    k_end, k_peak, k_last = win[0], max(win, key=lambda k: abs(ex1[k]["gyro"][0])), win[-1]
    sp_roll = rows[comb[0]]["sp_roll"]
    out(f"combined segment setpoint roll {sp_roll!r} rad/s; roll peak in the combined segment "
        f"{max(ex1[k]['gyro'][0] for k in comb)!r} rad/s")
    for label, k in (("end of the combined segment (first recovery execution)", k_end),
                     ("largest |w_roll| in the recovery window (stick centred)", k_peak),
                     ("end of the recovery window", k_last)):
        out(f"{label}: execution {k}, t = {t(k):.6f} s, w_roll {ex1[k]['gyro'][0]!r} rad/s, req_roll "
            f"{rows[k]['req_roll']!r} N m, sp_roll {rows[k]['sp_roll']!r}")
    out(f"gyroscopic roll torque -(J_zz - J_yy) w_y w_z at execution {k_end}: {gyro_roll(k_end)!r} N m "
        f"(req_roll {rows[k_end]['req_roll']!r} N m)")
    out(f"-- roll trace every {TRACE_STRIDE} executions: k t_s sp_roll w_roll w_pitch w_yaw req_roll "
        "gyro_torque_roll")
    for k in list(range(comb[0], win[-1] + 1, TRACE_STRIDE)) + ([win[-1]] if (win[-1] - comb[0]) % TRACE_STRIDE else []):
        w = ex1[k]["gyro"]
        out(" ".join([str(k), f"{t(k):.6f}", f(rows[k]["sp_roll"]), *(f(v) for v in w), f(rows[k]["req_roll"]),
                      f(gyro_roll(k))]))
    out("")
    out("== pitch excursion, every execution")
    out(head)
    for k in exc:
        out(row_line(k))
    return lines, any(flags.values())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, metavar="FILE")
    ap.add_argument("--plugin-dir", default=str(run_l4.DEFAULT_PLUGIN_DIR), metavar="DIR")
    ap.add_argument("--work-dir", metavar="DIR")
    args = ap.parse_args(argv)
    _, defaults = run_l4.build_parameters(args.plugin_dir)
    tool = run_l4.replay_tool(args.plugin_dir)
    plan = run_l4.plan_acro(run_l4.l4s.load_acro(t4.SCENARIO), run_l4.read_param_defaults(defaults), t4.CARD)
    bound = t4.design_bound(plan, defaults)
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(args.work_dir or tmp)
        work.mkdir(parents=True, exist_ok=True)
        runs = {m: run_l4.run_acro(t4.CARD, t4.SCENARIO, m, work, args.plugin_dir) for m in (2, 1)}
        replays = {m: run_l4.replay_acro(s, tool, work) for m, s in runs.items()}
        fidelity = {m: run_l4.replay_fidelity(s, replays[m]) for m, s in runs.items()}
        if any(v is not None for v in fidelity.values()):
            print(f"acro_cause: the replay does not reproduce the run: {fidelity}", file=sys.stderr)
            return 1
        body, flagged = analyse(runs, replays, bound)
    rel = lambda x: Path(x).resolve().relative_to(ROOT)  # noqa: E731
    head = [
        "MARV quad L4 acro: cause of the pitch excursion and of the roll recovery failure (decision 0005)",
        "command: uv run python tests/regression/quad/L04/results/acro_cause/acro_cause.py --out <file>",
        f"label: {run_l4.LABEL} (truth gyro, one card for truth and firmware): not a validation run",
        f"scenario: {rel(t4.SCENARIO)} sha256 {sha256(t4.SCENARIO)}",
        f"card: {rel(t4.CARD)} sha256 {sha256(t4.CARD)}",
        f"parameter table (build): {rel(defaults)} sha256 {sha256(defaults)}",
        f"replay tool: {rel(tool)} (tests/regression/quad/L04/replay); replay fidelity: bit-exact DShot at every "
        f"execution of both runs ({', '.join(f'm = {m}: {len(replays[m])}' for m in replays)} executions)",
        "",
    ]
    verdict = ("documented desaturation (a saturation flag was set during the excursion)" if flagged else
               "gyroscopic coupling: no saturation flag was set during the excursion and pitch authority was not "
               "reduced; a known limit of the per-axis PI, revisit at L6 (D) and B3 (INDI)")
    text = "\n".join(head + body + ["", f"verdict: {verdict}"]) + "\n"
    Path(args.out).write_text(text, encoding="utf-8", newline="\n")
    print(f"acro_cause: wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
