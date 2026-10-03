#!/usr/bin/env python3
"""R2's E, measured on R2's own two runs (decision 0014, owner decisions second round, item 2; order step 1).

    uv run python tests/regression/quad/L06/results/r2_lower_bound/measure_e.py --work <scratch dir> --out <file>

Needs gz-sim 8, the host-gz-l5 build (cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5) and the generated L5 T3
reference (uv run python tools/refdata/refdata.py ensure quad/L05/t3, decision 0011).

What is measured. E is the R2 predicate's run term (tests/regression/quad/L05/gz/test_t4_recovery.py, module docstring and
evaluate): E_c(n) = |y_c,m=1(n) - y_c,m=2(n)|, y_c(n) channel c of the TRUTH record of attitude execution n (the test's
series), over the two runs of the m_sequence of scenarios/quad/L05/recover_tumble_prop_strike.yaml (R2 with hover rotors, as
the proof reads it; decision 0015). The runs are the test's own: its
fly(..., "recovery", "recover_tumble_prop_strike", make_design(...)) (run_l5.run_sequence, then the test's evaluate), and the test's
clean-run check (test_run_is_clean_on_truth_and_in_the_dshot_range) is applied to them. E is evaluated with the expression of
evaluate, and the script refuses unless its maximum per channel equals evaluate's max_E. The rate channels are taken from
execution 1, as the predicate takes them (rate origin, test module docstring). Setup: R2 with hover rotors and the
gains of the host-gz-l5 build; make_design refuses a build whose parameters differ from the recorded T3 fixture.

Against the proof (r2_lower_bound.py; bound.txt). Its compute() gives per rate axis the binding execution n*, the forced
excess X* and the design tolerance F + Q. The predicate can hold at n* only if E(n*) >= X* - (F + Q): the stop condition
(decision 0014, second round, lead note). Per axis the script also lists the executions n of the proof's window at which
X(n) - (F + Q) > E(n), where the measured E does not cover the forced excess.

Not printed (environment-specific): wall times (stderr only) and the plugin binary's sha256, which differs between build hosts
for the same source. The gz logs go to --work; a rerun regenerates them (the runs are deterministic, decision 0003).
"""
import argparse
import hashlib
import statistics
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
for _p in (ROOT / "tools" / "card", ROOT / "tools" / "sim", ROOT / "tests" / "regression" / "quad" / "L05" / "gz", HERE):
    sys.path.insert(0, str(_p))
import r2_lower_bound as lb  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import test_t4_recovery as t4r  # noqa: E402

NAME = lb.SCEN_NAME
SCEN = t4r.SCEN / f"{NAME}.yaml"
RATE = lb.RATE_CH
RATE_IDX = tuple(t4r.CH.index(c) for c in RATE)
FIRST = 1  # the predicate's first rate execution (test_t4_recovery.evaluate skips the rate channels at execution 0)
# Report choices (labelled; neither enters E):
FULL_EXEC = 40  # executions printed in full, FIRST .. FULL_EXEC; must cover the proof's window (lb.WINDOW_EXEC)
SAMPLE_EXEC = (80, 160, 400, 800, 1600)  # sample executions of the summary: cause.txt section 2's, plus the last
OMITTED_VERSION_KEYS = ("plugin sha256",)  # run_scenario.versions entries that name the build host, not the source


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path):
    return str(Path(path).resolve().relative_to(ROOT))


class Dirs:
    """The tmp_path_factory that test_t4_recovery.fly takes: a fresh directory under --work per call."""

    def __init__(self, work):
        self.work = Path(work)

    def mktemp(self, basename):
        self.work.mkdir(parents=True, exist_ok=True)
        return Path(tempfile.mkdtemp(prefix=f"{basename}_", dir=self.work))


def live_params():
    """tests/regression/quad/L05/gz/conftest.py live_params: the plugin build's parameter table, and its path."""
    _, defaults = run_l5.build_parameters(t4r.PLUGIN_DIR)
    return run_l5.l4.read_param_defaults(defaults), defaults


def ranges(ns):
    """'a-b, c, ...' of a sorted list of integers."""
    out, start, prev = [], None, None
    for n in ns:
        if start is None:
            start = prev = n
        elif n == prev + 1:
            prev = n
        else:
            out.append(f"{start}-{prev}" if prev > start else f"{start}")
            start = prev = n
    if start is not None:
        out.append(f"{start}-{prev}" if prev > start else f"{start}")
    return ", ".join(out) if out else "none"


def measure(work):
    if not (run_scenario.plugin_present(t4r.PLUGIN_DIR) and run_scenario.gz8_available()):
        raise SystemExit(f"measure_e: gz-sim 8 or {rel(t4r.PLUGIN_DIR)}/{run_scenario.PLUGIN_FILE} is absent")
    if FULL_EXEC < lb.WINDOW_EXEC:
        raise SystemExit("measure_e: FULL_EXEC does not cover the proof's window")
    clock = time.monotonic()
    params, defaults = live_params()
    design = t4r.make_design(NAME, params)
    t_design = time.monotonic() - clock
    seq = t4r.fly(Dirs(work), "recovery", NAME, design)
    t4r.test_run_is_clean_on_truth_and_in_the_dshot_range(seq, NAME, params)
    by_m = {s.run.m: s for s in seq.runs}
    y1, y2 = t4r.series(by_m[1], design.count), t4r.series(by_m[2], design.count)
    e = {c: [abs(y1[n][ci] - y2[n][ci]) for n in range(design.count)] for c, ci in zip(RATE, RATE_IDX)}
    ev = seq.evaluation
    for c in RATE:
        if max(e[c][FIRST:]) != ev["components"][c]["max_E"]:
            raise SystemExit(f"measure_e: {c}: max E {max(e[c][FIRST:])!r} differs from the test's evaluate max_E "
                             f"{ev['components'][c]['max_E']!r}")
    clock = time.monotonic()
    proof = lb.compute()
    if proof["design"] is not design:
        raise SystemExit("measure_e: the proof did not use the measured runs' design")
    t_proof = time.monotonic() - clock
    headers = {s.run.log["header"]["gz_sim_version"] for s in seq.runs}
    if len(headers) != 1:
        raise SystemExit(f"measure_e: the runs report different gz-sim versions {sorted(headers)}")
    versions = {k: v for k, v in run_scenario.versions(headers.pop(), t4r.PLUGIN_DIR).items() if k not in OMITTED_VERSION_KEYS}
    plugin = Path(t4r.PLUGIN_DIR) / run_scenario.PLUGIN_FILE
    print(f"measure_e: wall: design {t_design:.1f} s, gz {seq.wall_s:.1f} s, proof {t_proof:.1f} s; plugin sha256 {sha(plugin)} "
          "(not in the output)", file=sys.stderr)
    return {"design": design, "seq": seq, "by_m": by_m, "y1": y1, "y2": y2, "E": e, "ev": ev, "proof": proof,
            "defaults": defaults, "versions": versions}


def render(r):
    d, ev, proof, by_m = r["design"], r["ev"], r["proof"], r["by_m"]
    e, y1, y2 = r["E"], r["y1"], r["y2"]
    last = d.count - 1
    s1 = by_m[1]
    t_a = s1.plan.period_s
    inputs = [("card", t4r.CARD), ("scenario", SCEN), ("test_t4_recovery.py", t4r.__file__), ("recovery_model.py", rm.__file__),
              ("run_l5.py", run_l5.__file__), ("attitude_t3_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_inputs.txt"),
              ("attitude_t3_q_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"),
              ("attitude_t3_envelope.txt (generated, decision 0011)", t4r.T3_GENERATED / "attitude_t3_envelope.txt"),
              ("parameter table (build) " + rel(r["defaults"]), r["defaults"]), ("r2_lower_bound.py", lb.__file__),
              ("bound.txt", HERE / "bound.txt")]
    L = ["MARV quad L6 stage (c): R2's E measured on its own two runs (decision 0014, second round item 2: E first)",
         "command: uv run python tests/regression/quad/L06/results/r2_lower_bound/measure_e.py --work <dir> --out <file>",
         f"label: {run_l5.LABEL}: not a validation run",
         "inputs sha256: " + "; ".join(f"{k} {sha(p)}" for k, p in inputs),
         "versions: " + "; ".join(f"{k} {v}" for k, v in r["versions"].items()),
         "",
         "== 1. the runs (test_t4_recovery.fly, 'recovery', on make_design's design; the test's clean-run check passed)",
         f"scenario {rel(SCEN)}: m_sequence {list(s1.plan.m_sequence)}; attitude period T_a {t_a} s; initial rotor speed "
         f"{[repr(x) for x in s1.plan.rotor_speed_rad_s]} rad/s; design executions N {d.count} (n = 0 .. {last})"]
    for s in r["seq"].runs:
        dshot = [x for t in s.run.log["ticks"] for x in t["dshot"]]
        L.append(f"m = {s.run.m}: host steps {s.run.iterations}; ticks {len(s.run.log['ticks'])}; TRUTH records "
                 f"{len(s.run.log['truths'])}; window host steps {s.window_steps.start}..{s.window_steps.stop - 1}; stale reads "
                 f"in the window {len(s.window_stale)}; DShot [{min(dshot)}, {max(dshot)}]")
    L.append(f"the test's evaluate on these runs: passed {ev['passed']}; violations {ev['violations']} (all six channels); "
             f"stale {ev['stale_reads_in_window']}")
    for c, ci in zip(RATE, RATE_IDX):
        st = ev["components"][c]
        w = st["worst"]
        L.append(f"  {c}: F {d.F[ci]!r}; Q {d.Q[ci]!r}; violations {st['violations']}; first n {st['first']}; worst n {w['n']} "
                 f"slack {w['slack']!r} (outside {w['outside']!r}, E {w['E']!r}); max_E {st['max_E']!r}")
    L += ["",
          f"== 2. E per execution, n = {FIRST} .. {FULL_EXEC} (rad/s): per channel y1 = y(m = 1), y2 = y(m = 2), E = |y1 - y2|, "
          "y the TRUTH record's body rate (test_t4_recovery.series)"]
    for n in range(FIRST, FULL_EXEC + 1):
        L.append(f"exec {n}: " + "; ".join(f"{c} {y1[n][ci]!r} {y2[n][ci]!r} {e[c][n]!r}" for c, ci in zip(RATE, RATE_IDX)))
    L += ["",
          f"== 3. E over the predicate's rate window, n = {FIRST} .. {last} (rad/s)"]
    for c in RATE:
        xs = e[c][FIRST:]
        n_max = FIRST + max(range(len(xs)), key=xs.__getitem__)
        win = e[c][FIRST:lb.WINDOW_EXEC + 1]
        n_win = FIRST + max(range(len(win)), key=win.__getitem__)
        L.append(f"{c}: executions {len(xs)}; max E {e[c][n_max]!r} at n {n_max}; mean E {statistics.fmean(xs)!r}; median E "
                 f"{statistics.median(xs)!r}; max over the proof's window n = {FIRST} .. {lb.WINDOW_EXEC} {e[c][n_win]!r} at n "
                 f"{n_win}; at n = " + ", ".join(f"{n}: {e[c][n]!r}" for n in (*SAMPLE_EXEC, last))
                 + f"; equals the test's evaluate max_E: {e[c][n_max] == ev['components'][c]['max_E']}")
    L += ["",
          "== 4. against the proof (r2_lower_bound.compute(), as bound.txt): the predicate can hold at n* only if "
          "E(n*) >= X* - (F + Q)"]
    stops = {}
    for a, c in enumerate(RATE):
        x = proof["ax"][a]
        n_star, need = x["n_star"], x["X_star"] - x["tol"]
        forced = x["X_star"] > x["tol"]
        stops[c] = forced and e[c][n_star] >= need
        uncovered = [n for n in range(FIRST, lb.WINDOW_EXEC + 1) if x["X"][n] - x["tol"] > e[c][n]]
        L.append(f"{c}: n* {n_star}; X* {x['X_star']!r}; F + Q {x['tol']!r}; X* - (F + Q) {need!r}; forced (X* > F + Q): {forced}; "
                 f"E(n*) {e[c][n_star]!r}; E(n*) >= X* - (F + Q): {e[c][n_star] >= need}; X(n) - (F + Q) > E(n) at n = "
                 f"{ranges(uncovered)} ({len(uncovered)} of {FIRST} .. {lb.WINDOW_EXEC})")
    b = RATE[proof["binding"]]
    L.append(f"binding axis: {b}; stop (E(n*) >= X* - (F + Q) on the binding axis): {stops[b]}; on any forced axis: "
             f"{any(stops.values())}")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    r = measure(a.work)
    text = render(r)
    Path(a.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return r


if __name__ == "__main__":
    main()
