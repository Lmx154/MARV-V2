#!/usr/bin/env python3
"""The hover-rotor tumble, run and reported (decision 0015; decision 0014 owner decision 5, condition 4).

    uv run python tests/regression/quad/L06/results/r2_lower_bound/hover_tumble.py --work <scratch dir> --out <file>

Needs gz-sim 8, the host-gz-l5 build and the generated L5 T3 reference, as measure_e.py.

What is run. scenarios/quad/L05/recover_tumble_prop_strike.yaml: R2's tumble (inverted, rate_max on every axis) with every
rotor at the card's hover speed, so w x Jw kicks the body at t = 0: the collision or prop-strike case. It is R2 as specified
before its steady-tumble setup, and the scenario the lower-bound proof (r2_lower_bound.py) and the E measurement
(measure_e.py) read. The runs, the design envelope and the predicate are the R2 test's own
(tests/regression/quad/L05/gz/test_t4_recovery.py: fly, make_design, evaluate, verdict_line), and its clean-run check is
applied to the runs.

Not asserted. The case has no pass bar until L8 (owner decision 5, condition 4): this script prints the excess of every
channel over the R2 predicate's tolerance, envelope +- (E + F + Q), and fails only on a run that is not clean.
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import measure_e as me  # noqa: E402  (its import block puts tools/ and the L5 gz directory on the path)
import r2_lower_bound as lb  # noqa: E402

NAME = lb.SCEN_NAME
SCEN = me.t4r.SCEN / f"{NAME}.yaml"


def run(work):
    if not (me.run_scenario.plugin_present(me.t4r.PLUGIN_DIR) and me.run_scenario.gz8_available()):
        raise SystemExit(f"hover_tumble: gz-sim 8 or {me.rel(me.t4r.PLUGIN_DIR)}/{me.run_scenario.PLUGIN_FILE} is absent")
    params, defaults = me.live_params()
    design = me.t4r.make_design(NAME, params)
    seq = me.t4r.fly(me.Dirs(work), "recovery", NAME, design)
    me.t4r.test_run_is_clean_on_truth_and_in_the_dshot_range(seq, NAME, params)
    headers = {s.run.log["header"]["gz_sim_version"] for s in seq.runs}
    if len(headers) != 1:
        raise SystemExit(f"hover_tumble: the runs report different gz-sim versions {sorted(headers)}")
    versions = {k: v for k, v in me.run_scenario.versions(headers.pop(), me.t4r.PLUGIN_DIR).items()
                if k not in me.OMITTED_VERSION_KEYS}
    return {"design": design, "seq": seq, "defaults": defaults, "versions": versions}


def render(r):
    d, seq = r["design"], r["seq"]
    ev = seq.evaluation
    s1 = {s.run.m: s for s in seq.runs}[1]
    inputs = [("card", me.t4r.CARD), ("scenario", SCEN), ("test_t4_recovery.py", me.t4r.__file__),
              ("recovery_model.py", me.rm.__file__), ("run_l5.py", me.run_l5.__file__),
              ("attitude_t3_inputs.txt", me.t4r.T3_REFERENCE / "attitude_t3_inputs.txt"),
              ("attitude_t3_q_inputs.txt", me.t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"),
              ("attitude_t3_envelope.txt (generated, decision 0011)", me.t4r.T3_GENERATED / "attitude_t3_envelope.txt"),
              ("parameter table (build) " + me.rel(r["defaults"]), r["defaults"])]
    L = ["MARV quad L6 stage (c): the hover-rotor tumble, run and reported, not asserted (decision 0015; decision 0014 owner "
         "decision 5, condition 4: the collision or prop-strike case, no pass bar until L8)",
         "command: uv run python tests/regression/quad/L06/results/r2_lower_bound/hover_tumble.py --work <dir> --out <file>",
         f"label: {me.run_l5.LABEL}: not a validation run",
         "inputs sha256: " + "; ".join(f"{k} {me.sha(p)}" for k, p in inputs),
         "versions: " + "; ".join(f"{k} {v}" for k, v in r["versions"].items()),
         "",
         f"scenario {me.rel(SCEN)}: m_sequence {list(s1.plan.m_sequence)}; initial rotor speed "
         f"{[repr(x) for x in s1.plan.rotor_speed_rad_s]} rad/s; design executions N {d.count}; the test's clean-run check "
         "passed",
         f"the R2 predicate on these runs: passed {ev['passed']}; violations {ev['violations']}; stale "
         f"{ev['stale_reads_in_window']}",
         "per channel: max outside the envelope; worst slack = outside - (E + F + Q) at its execution; violations; first "
         "violating execution"]
    for ci, (c, st) in enumerate(ev["components"].items()):
        w = st["worst"]
        L.append(f"  {c}: max outside {st['max_out']!r}; worst slack {w['slack']!r} at n {w['n']} (outside {w['outside']!r}, "
                 f"E {w['E']!r}, F {d.F[ci]!r}, Q {d.Q[ci]!r}); violations {st['violations']}; first n {st['first']}")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    r = run(a.work)
    text = render(r)
    Path(a.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return r


if __name__ == "__main__":
    main()
