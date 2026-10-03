#!/usr/bin/env python3
"""R1X in gz, reduced to the compact summary the coupling counterfactual reads (decision 0014, owner decisions third round,
item 3). The generating command of gz_new.txt and gz_old.txt; r1x_coupling.py reads them and never runs gz.

    uv run python tests/regression/quad/L06/results/r1x_coupling/capture_gz.py --tag new --work <dir> --out <file>
    uv run python tests/regression/quad/L06/results/r1x_coupling/capture_gz.py --tag old --root <tree> --rev 69c62f2 \
        --work <dir> --out <file>

Needs gz-sim 8, the tree's host-gz-l5 build (cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5, in the tree)
and the tree's generated L5 T3 reference (uv run python tools/refdata/refdata.py ensure quad/L05/t3, in the tree; decision
0011). --root defaults to this repository; with --root another checkout, --rev names its commit and the script checks the
checkout against this repository's `git ls-tree -r <rev>`, blob for blob, before it runs anything.

The tree's own code does everything (r1x_model.load_tree): its test_t4_recovery.make_design(R1X, live params, alpha) is the
test's exact_design fixture; its fly(..., "exact", R1X, design, chs=("alpha",), ser=alpha_series) is the test's exact fixture
(run_l5.run_sequence, m = 2 then m = 1); its test_run_is_clean_on_truth_and_in_the_dshot_range is applied to the runs
(complete trailer, a TRUTH record per tick, no stale read in the window, truth sources, DShot within [idle, 2047]); the
script's predicate on gz's alpha with gz's E must equal the test's evaluate, or it refuses. The design model's runs on gz's
branch (r1x_model) use the tree's recovery_model and its recorded T3 fixture.

Output (raw data; `key = value` lines, Python literals, read back with ast.literal_eval):
  1. design   the design's N, F, Q, halving, kin, end_ok, sha256 of (lo, hi) per execution, the fixture's gains;
  2. gz       the runs' counts, the test's verdict, q(1) and w(1) of both runs, sha256 of each run's TRUTH (q, w) and channel
              series, gz's E, and the reductions of r1x_model.summary on the m = 1 run with the predicate's E;
  3. model    r1x_model.model_facts of the model runs seeded with the m = 1 run's q(1) (reproduced by r1x_coupling.py);
  4. dist     per run and channel: max_n |gz - run| over every execution n = 0 .. N - 1, its n, gz's and the run's value
              there (the full gz series exists only here; the gz logs go to --work and a rerun regenerates them, decision
              0003).
Not in the output (environment-specific): wall times and the plugin binary's sha256 (stderr only).
"""

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
sys.path.insert(0, str(HERE))
import r1x_model as mdl  # noqa: E402

OMITTED_VERSION_KEYS = ("plugin sha256",)  # run_scenario.versions entries that name the build host, not the source
DIST_RUNS = ("D", "C", "CF", "C0")


class Dirs:
    """The tmp_path_factory that test_t4_recovery.fly takes: a fresh directory under --work per call."""

    def __init__(self, work):
        self.work = Path(work)

    def mktemp(self, basename):
        self.work.mkdir(parents=True, exist_ok=True)
        return Path(tempfile.mkdtemp(prefix=f"{basename}_", dir=self.work))


def check_tree(root, rev):
    """The checkout at root holds exactly the blobs of this repository's commit rev."""
    out = subprocess.run(["git", "-C", str(ROOT), "ls-tree", "-r", rev], capture_output=True, text=True, check=True).stdout
    entries = [ln.split(None, 3) for ln in out.splitlines()]
    paths = [e[3] for e in entries]
    got = subprocess.run(["git", "hash-object", "--stdin-paths", "--no-filters"], input="\n".join(str(root / p) for p in paths),
                         capture_output=True, text=True, check=True).stdout.split()
    bad = [p for (_, _, blob, p), h in zip(entries, got) if h != blob]
    if bad or len(got) != len(paths):
        raise SystemExit(f"capture_gz: {root} is not a checkout of {rev}: {bad[:8]}")
    return len(paths)


def capture(tree, work, procs):
    t4r, rm, run_l5, run_scenario, l5s = tree.t4r, tree.rm, tree.run_l5, tree.run_scenario, tree.l5s
    if not (run_scenario.plugin_present(t4r.PLUGIN_DIR) and run_scenario.gz8_available()):
        raise SystemExit(f"capture_gz: gz-sim 8 or {t4r.PLUGIN_DIR}/{run_scenario.PLUGIN_FILE} is absent")
    clock = time.monotonic()
    _, defaults = run_l5.build_parameters(t4r.PLUGIN_DIR)
    live = run_l5.l4.read_param_defaults(defaults)
    design = t4r.make_design(t4r.R1X, live, fn=rm.alpha_channel, label="alpha")
    t_design = time.monotonic() - clock
    seq = t4r.fly(Dirs(work), "exact", t4r.R1X, design, chs=("alpha",), ser=t4r.alpha_series)
    t4r.test_run_is_clean_on_truth_and_in_the_dshot_range(seq, t4r.R1X, live)
    by_m = {s.run.m: s for s in seq.runs}
    n_exec = design.count
    truth = {m: [(tuple(s.executions[n]["truth"]["q_wxyz"]), tuple(s.executions[n]["truth"]["omega_frd"]))
                 for n in range(n_exec)] for m, s in by_m.items()}
    gz = {m: [(*rm.channels(q, w), rm.alpha_channel(q, None)[0]) for q, w in tr] for m, tr in truth.items()}
    e = [abs(a[mdl.ALPHA] - b[mdl.ALPHA]) for a, b in zip(gz[1], gz[2])]
    ev = seq.evaluation["components"]["alpha"]
    mine = mdl.verdict([r[mdl.ALPHA] for r in gz[1]], design.lo[0], design.hi[0], design.F[0], design.Q[0], e)
    if (mine["violations"], mine["first"], mine["worst_n"], mine["worst_slack"]) != (
            ev["violations"], ev["first"], ev["worst"]["n"], ev["worst"]["slack"]) or max(e) != ev["max_E"]:
        raise SystemExit(f"capture_gz: the alpha predicate here {mine} differs from the test's evaluate {ev}")
    st = l5s.values(l5s.load(t4r.SCEN / f"{t4r.R1X}.yaml"))["initial_state"]
    q0, w0 = tuple(st["attitude_q_wxyz"]), tuple(st["body_rates_frd_rad_s"])
    su = rm.oracle.Setup(t4r.recorded_inputs())
    q1 = truth[1][1][0]
    clock = time.monotonic()
    runs = mdl.model_runs(tree, su, q0, w0, q1, n_exec, procs)
    t_model = time.monotonic() - clock
    gz_summary = mdl.summary(gz[1], design, su, tree)
    gz_summary["verdict"] = mine
    dist = {}
    for k in DIST_RUNS:
        dist[k] = {}
        for ci, c in enumerate(mdl.CHANNELS):
            d = [abs(a[ci] - b[ci]) for a, b in zip(gz[1], runs[k])]
            n = max(range(n_exec), key=d.__getitem__)
            dist[k][c] = [d[n], n, gz[1][n][ci], runs[k][n][ci]]
    headers = {s.run.log["header"]["gz_sim_version"] for s in seq.runs}
    if len(headers) != 1:
        raise SystemExit(f"capture_gz: the runs report different gz-sim versions {sorted(headers)}")
    versions = {k: v for k, v in run_scenario.versions(headers.pop(), t4r.PLUGIN_DIR).items() if k not in OMITTED_VERSION_KEYS}
    plugin = Path(t4r.PLUGIN_DIR) / run_scenario.PLUGIN_FILE
    print(f"capture_gz: wall: design {t_design:.1f} s, gz {seq.wall_s:.1f} s, model {t_model:.1f} s; plugin sha256 "
          f"{mdl.sha_file(plugin)} (not in the output)", file=sys.stderr)
    runs_info = {}
    for s in seq.runs:
        dshot = [x for t in s.run.log["ticks"] for x in t["dshot"]]
        runs_info[s.run.m] = {"host_steps": s.run.iterations, "ticks": len(s.run.log["ticks"]),
                              "truth_records": len(s.run.log["truths"]), "stale_in_window": len(s.window_stale),
                              "dshot": [min(dshot), max(dshot)]}
    return {"design": design, "seq": seq, "ev": ev, "truth": truth, "gz": gz, "E": e, "gz_summary": gz_summary, "su": su,
            "q0": q0, "w0": w0, "q1": q1, "runs": runs, "dist": dist, "versions": versions, "defaults": defaults,
            "runs_info": runs_info}


def render(tree, r, tag, tree_label):
    t4r, rm = tree.t4r, tree.rm
    root = tree.root

    def rel(p):
        return str(Path(p).resolve().relative_to(root))

    inputs = [("card", t4r.CARD), ("scenario", t4r.SCEN / f"{t4r.R1X}.yaml"), ("test_t4_recovery.py", t4r.__file__),
              ("recovery_model.py", rm.__file__), ("attitude_t3_oracle.py", rm.oracle.__file__),
              ("attitude_t3_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_inputs.txt"),
              ("attitude_t3_q_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"),
              ("attitude_t3_envelope.txt (generated, decision 0011)", t4r.T3_GENERATED / "attitude_t3_envelope.txt"),
              ("run_l5.py", tree.run_l5.__file__), ("parameter table (build) " + rel(r["defaults"]), r["defaults"]),
              ("capture_gz.py", __file__), ("r1x_model.py", mdl.__file__)]
    ev = r["ev"]
    L = ["MARV quad L6 stage (c): R1X in gz, the compact summary of the coupling counterfactual (decision 0014, third round, "
         "item 3)",
         "command: uv run python tests/regression/quad/L06/results/r1x_coupling/capture_gz.py --tag <tag> [--root <tree> --rev "
         "<rev>] --work <dir> --out <file>",
         f"label: {tree.run_l5.LABEL}: not a validation run",
         mdl.kv("tag", tag),
         mdl.kv("tree", tree_label),
         "inputs sha256: " + "; ".join(f"{k} {mdl.sha_file(p)}" for k, p in inputs),
         "versions: " + "; ".join(f"{k} {v}" for k, v in r["versions"].items()),
         "",
         "# == 1. design: test_t4_recovery.make_design(R1X, live params, alpha channel), the test's exact_design fixture"]
    L += mdl.facts_lines("design", mdl.design_facts(r["design"], tree))
    L += ["",
          "# == 2. gz: test_t4_recovery.fly(..., 'exact', R1X, design, alpha), the test's exact fixture; the test's clean-run "
          "check passed",
          mdl.kv("gz.scenario_initial_state", {"q": list(r["q0"]), "w": list(r["w0"])}),
          mdl.kv("gz.runs", r["runs_info"]),
          mdl.kv("gz.test_verdict", {"passed": r["seq"].evaluation["passed"], "violations": ev["violations"],
                                     "first": ev["first"], "worst": ev["worst"], "max_E": ev["max_E"],
                                     "max_out": ev["max_out"]}),
          mdl.kv("gz.q1", {m: list(r["truth"][m][1][0]) for m in (1, 2)}),
          mdl.kv("gz.w1", {m: list(r["truth"][m][1][1]) for m in (1, 2)}),
          mdl.kv("gz.truth_sha256", {m: mdl.series_sha(q + w for q, w in r["truth"][m]) for m in (1, 2)}),
          mdl.kv("gz.series_sha256", {m: mdl.series_sha(r["gz"][m]) for m in (1, 2)}),
          mdl.kv("gz.E", {"max": max(r["E"]), "n": max(range(len(r["E"])), key=r["E"].__getitem__)})]
    L += mdl.facts_lines("gz.summary", r["gz_summary"])
    L += ["",
          "# == 3. model: r1x_model.model_facts, the design model's nominal member seeded with gz.q1[1] (model only: "
          "r1x_coupling.py re-runs it)",
          mdl.kv("model.seed_q1", list(r["q1"]))]
    L += mdl.facts_lines("model", mdl.model_facts(r["runs"], r["design"], r["su"], tree))
    L += ["",
          "# == 4. dist: gz's m = 1 run against each model run, per channel [max_n |gz - run|, n, gz(n), run(n)], "
          "n = 0 .. N - 1"]
    L += [mdl.kv(f"dist.{k}", v) for k, v in r["dist"].items()]
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, choices=("new", "old"))
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--rev")
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=len(mdl.RUNS))
    a = ap.parse_args(argv)
    root = Path(a.root).resolve()
    if root == ROOT:
        label = "this repository's tree (the inputs' sha256 pin it)"
    else:
        if not a.rev:
            raise SystemExit("capture_gz: --root another checkout needs --rev")
        label = f"a checkout of {a.rev} (checked blob for blob against git ls-tree -r {a.rev}: {check_tree(root, a.rev)} files)"
    tree = mdl.load_tree(root)
    r = capture(tree, a.work, a.jobs)
    text = render(tree, r, a.tag, label)
    Path(a.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return r


if __name__ == "__main__":
    main()
