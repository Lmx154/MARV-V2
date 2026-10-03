#!/usr/bin/env python3
"""The FF-on report runs of L6 stage (c): one T4 run each of L4 acro, L5 R2 and L5 R1X with the rate loop's
lag-compensated omega x J omega feed-forward switched on through the test harness, against the same run with it off
(decision 0014, owner decision 2: "stage (c) records FF-on evidence: ... one T4 run of the acro and R2 scenarios with FF
switched on through the test harness. These are reported, not asserted"; third round, item 3 adds R1X). The generating
command of gz_acro.txt, gz_r2.txt and gz_r1x.txt; test_ff_on.py reads them and never runs gz.

    uv run python tests/regression/quad/L06/results/ff_on/capture_gz.py --case acro --work <dir> --out <file>
    uv run python tests/regression/quad/L06/results/ff_on/capture_gz.py --case r2 --work <dir> --out <file>
    uv run python tests/regression/quad/L06/results/ff_on/capture_gz.py --case r1x --work <dir> --out <file>

Needs gz-sim 8, the host-gz-l4 build (acro) or the host-gz-l5 build (r2, r1x), and for r2 and r1x the generated L5 T3
reference (uv run python tools/refdata/refdata.py ensure quad/L05/t3; decision 0011).

The switch. The product parameter rate_ff_enable (design/scenario_values.yaml) is 0 in every build. The FF-on runs set it
to 1 by name, as a harness sil_override (FF_ON); fw/rate's from_params then reads J (inertia_xx/yy/zz), tau_m (motor_tau)
and T_ff (rate_ff_filter_tau). The FF-off runs carry no harness override: they are the default runs of the suites.

The tests' own code does everything; nothing of a predicate is restated here:
  acro  tests/regression/quad/L04/gz/test_t4_acro.py: design_bound (the module's bound fixture), fly (run_l4.run_acro at
        m = 2 then m = 1, the replays, evaluate). Both predicates of its evaluate are recorded: the pass bar (i)-(v) and the
        recovery predicate over the window after the combined full-stick segment (the strict xfail).
  r2    tests/regression/quad/L05/gz/test_t4_recovery.py: make_design(R2, live params) (its design fixture), fly (run_l5
        run_sequence, m = 2 then m = 1, evaluate on the six channels); the R2 envelope predicate (the strict xfail).
  r1x   the same module: make_design(R1X, live params, alpha channel) (its exact_design fixture), fly(..., chs=("alpha",),
        ser=alpha_series) (its exact fixture); the alpha envelope predicate (the strict xfail).
For r2 and r1x the module's clean-run check (test_run_is_clean_on_truth_and_in_the_dshot_range) is applied to each run and
its outcome recorded (it does not stop the capture).

Output (raw data; `key = value` lines, Python literals, read back with ast.literal_eval):
  inputs and versions  the sha256 of every input read (the build's parameter table included) and the tool versions;
  params               the build's feed-forward parameters (the defaults the runs start from);
  design               the predicate's fixed terms (acro: P, F per axis; r2, r1x: N, F, Q per channel);
  off.*, on.*          per variant: the runs (host steps, ticks, DShot range, stale reads, harness overrides), the sha256 of
                       each run's series (acro: the gyro sample per execution; r2, r1x: TRUTH (q, w) per attitude
                       execution, as r1x_coupling/capture_gz.py hashes it), and the predicate's verdict and per-channel
                       statistics as the test's evaluate returns them;
  compare.*            per channel: the worst excess over the tolerance (positive) or the slack (negative), the violation
                       count and the first violation, off and on, and on - off.
The gz logs go to --work; a rerun regenerates them (decision 0003). Not in the output (environment-specific): wall times
and the plugin binary's sha256 (stderr only).
"""

import argparse
import hashlib
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
for p in (ROOT / "tools" / "card", ROOT / "tools" / "sim"):
    sys.path.insert(0, str(p))
import run_scenario  # noqa: E402

FF_ON = {"rate_ff_enable": ("i32", "1")}
FF_PARAMS = ("rate_ff_enable", "rate_ff_filter_tau", "inertia_xx", "inertia_yy", "inertia_zz", "motor_tau")
VARIANTS = (("off", None), ("on", FF_ON))
OMITTED_VERSION_KEYS = ("plugin sha256",)  # run_scenario.versions entries that name the build host, not the source


class Dirs:
    """The tmp_path_factory the tests' fly functions take: a fresh directory under --work per call."""

    def __init__(self, work):
        self.work = Path(work)

    def mktemp(self, basename):
        self.work.mkdir(parents=True, exist_ok=True)
        return Path(tempfile.mkdtemp(prefix=f"{basename}_", dir=self.work))


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def series_sha(rows):
    """sha256 of the rows' repr, one per line (r1x_coupling/r1x_model.py series_sha): pins a series bit for bit."""
    return hashlib.sha256(("\n".join(repr(tuple(r)) for r in rows) + "\n").encode("utf-8")).hexdigest()


def kv(key, value):
    return f"{key} = {value!r}"


def rel(path):
    return str(Path(path).resolve().relative_to(ROOT))


def import_test(layer, module):
    d = ROOT / "tests" / "regression" / "quad" / layer / "gz"
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))
    mod = __import__(module)
    if Path(mod.__file__).resolve() != (d / f"{module}.py").resolve():
        raise SystemExit(f"capture_gz: {module} was imported from {mod.__file__}")
    return mod


def versions(runs, plugin_dir):
    headers = {r.log["header"]["gz_sim_version"] for r in runs}
    if len(headers) != 1:
        raise SystemExit(f"capture_gz: the runs report different gz-sim versions {sorted(headers)}")
    return {k: v for k, v in run_scenario.versions(headers.pop(), plugin_dir).items() if k not in OMITTED_VERSION_KEYS}


def run_facts(s, rows):
    """`rows`: the run's DShot per tick (run_l4 keeps them as AcroRun.dshot and reduces the log to counts)."""
    dshot = [x for row in rows for x in row]
    return {"host_steps": s.run.iterations, "ticks": len(rows), "dshot": [min(dshot), max(dshot)],
            "stale_in_window": len(s.window_stale), "harness_overrides": dict(s.harness_overrides),
            "sil_override_rate_ff_enable": s.overrides.get("rate_ff_enable")}


# ---- acro -------------------------------------------------------------------------------------------------------------

def capture_acro(work):
    t4a = import_test("L04", "test_t4_acro")
    run_l4 = t4a.run_l4
    if not (run_scenario.plugin_present(t4a.PLUGIN_DIR) and run_scenario.gz8_available()):
        raise SystemExit(f"capture_gz: gz-sim 8 or {t4a.PLUGIN_DIR}/{run_scenario.PLUGIN_FILE} is absent")
    _, defaults = run_l4.build_parameters(t4a.PLUGIN_DIR)
    params = run_l4.read_param_defaults(defaults)
    plan = run_l4.plan_acro(run_l4.l4s.load_acro(t4a.SCENARIO), params, t4a.CARD)
    clock = time.monotonic()
    bound = t4a.design_bound(plan, defaults)
    print(f"capture_gz: design bound {time.monotonic() - clock:.1f} s", file=sys.stderr)
    out = {"defaults": defaults, "params": params, "plugin_dir": t4a.PLUGIN_DIR,
           "inputs": [("card", t4a.CARD), ("scenario", t4a.SCENARIO), ("test_t4_acro.py", t4a.__file__),
                      ("rate_t3_oracle.py", t4a.oracle.__file__), ("run_l4.py", run_l4.__file__),
                      ("parameter table (build) " + rel(defaults), defaults), ("capture_gz.py", __file__)],
           "design": {a: {"P": bound[a]["P"], "F": bound[a]["F"], "TOL": bound[a]["TOL"], "H": bound[a]["H"]}
                      for a in t4a.AXES}}
    runs = []
    for tag, over in VARIANTS:
        acro = t4a.fly(Dirs(work), f"acro_ff_{tag}", bound, over)
        print(f"capture_gz: acro {tag}: gz wall {acro.evaluation['gz wall (s)']:.1f} s", file=sys.stderr)
        ev = acro.evaluation
        rates = ev["(iii) per axis (linear executions)"]
        sat = ev["(v) combined segment (m = 1 replay)"]
        rec = ev["recovery (known failing, decision 0005)"]
        out[tag] = {
            "runs": {m: run_facts(s, s.dshot) for m, s in acro.runs.items()},
            "gyro_sha256": {m: series_sha(x["gyro"] for x in s.executions) for m, s in acro.runs.items()},
            "pass_bar": {
                "passed": ev["passed"], "clean": ev["(i) clean"], "dshot_bounds": ev["(ii) DShot bounds"],
                "dshot_seen": ev["(ii) DShot range seen"], "dshot_out_of_range": ev["(ii) out of range"],
                "stale_reads": ev["stale reads"],
                "replay_first_mismatch": ev["(iv) replay fidelity (first mismatch)"],
                "saturation": {k: sat.get(k) for k in ("passed", "combined executions", "flag findings",
                                                       "executions with s or t not 1")},
                "rate_bound": {a: {"violations": r["violations"], "first": r["first violation"],
                                   "smallest_slack": r["smallest slack"]["slack"], "at": r["smallest slack"]["k"],
                                   "max_w": r["max |w|"]} for a, r in rates.items()}},
            "recovery": {
                "passed": ev["recovery passed"],
                "axes": {a: {k: r[k] for k in ("passed", "violations", "first violation", "last violation", "worst margin",
                                               "at execution", "w there", "Z + F + E there", "fresh executions checked")}
                         for a, r in rec.items()}}}
        runs += [s.run for s in acro.runs.values()]
    out["versions"] = versions(runs, t4a.PLUGIN_DIR)
    # compare: the recovery predicate's excess (minus the worst margin) and the pass bar's (minus the smallest slack)
    out["compare"] = {}
    for a in t4a.AXES:
        rows = {}
        for tag in ("off", "on"):
            r = out[tag]["recovery"]["axes"][a]
            b = out[tag]["pass_bar"]["rate_bound"][a]
            rows[tag] = {"recovery_excess": -r["worst margin"], "recovery_violations": r["violations"],
                         "recovery_first": r["first violation"], "pass_bar_excess": -b["smallest_slack"],
                         "pass_bar_violations": b["violations"], "pass_bar_first": b["first"]}
        rows["on - off"] = {k: rows["on"][k] - rows["off"][k] for k in ("recovery_excess", "pass_bar_excess")}
        out["compare"][a] = rows
    return out


# ---- r2 and r1x -------------------------------------------------------------------------------------------------------

def capture_l5(case, work):
    t4r = import_test("L05", "test_t4_recovery")
    run_l5, rm = t4r.run_l5, t4r.rm
    if not (run_scenario.plugin_present(t4r.PLUGIN_DIR) and run_scenario.gz8_available()):
        raise SystemExit(f"capture_gz: gz-sim 8 or {t4r.PLUGIN_DIR}/{run_scenario.PLUGIN_FILE} is absent")
    _, defaults = run_l5.build_parameters(t4r.PLUGIN_DIR)
    live = run_l5.l4.read_param_defaults(defaults)
    name = t4r.R2 if case == "r2" else t4r.R1X
    clock = time.monotonic()
    if case == "r2":
        design = t4r.make_design(name, live)
        kw = {}
    else:
        design = t4r.make_design(name, live, fn=rm.alpha_channel, label="alpha")
        kw = {"chs": ("alpha",), "ser": t4r.alpha_series}
    print(f"capture_gz: design {time.monotonic() - clock:.1f} s", file=sys.stderr)
    scen = t4r.SCEN / f"{name}.yaml"
    out = {"defaults": defaults, "params": live, "plugin_dir": t4r.PLUGIN_DIR,
           "inputs": [("card", t4r.CARD), ("scenario", scen), ("test_t4_recovery.py", t4r.__file__),
                      ("recovery_model.py", rm.__file__), ("attitude_t3_oracle.py", rm.oracle.__file__),
                      ("attitude_t3_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_inputs.txt"),
                      ("attitude_t3_q_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"),
                      ("attitude_t3_envelope.txt (generated, decision 0011)", t4r.T3_GENERATED / "attitude_t3_envelope.txt"),
                      ("run_l5.py", run_l5.__file__), ("parameter table (build) " + rel(defaults), defaults),
                      ("capture_gz.py", __file__)],
           "design": {"scenario": name, "N": design.count, "channels": list(kw.get("chs", t4r.CH)), "F": list(design.F),
                      "Q": list(design.Q)}}
    runs = []
    for tag, over in VARIANTS:
        seq = t4r.fly(Dirs(work), f"ff_{tag}", name, design, over, **kw)
        print(f"capture_gz: {case} {tag}: gz wall {seq.wall_s:.1f} s", file=sys.stderr)
        try:
            t4r.test_run_is_clean_on_truth_and_in_the_dshot_range(seq, name, live)
            clean = True
        except AssertionError as e:
            clean = f"FAIL: {str(e).splitlines()[0] if str(e) else 'assertion'}"
        ev = seq.evaluation
        truth = {s.run.m: series_sha(tuple(s.executions[n]["truth"]["q_wxyz"]) + tuple(s.executions[n]["truth"]["omega_frd"])
                                     for n in range(design.count)) for s in seq.runs}
        out[tag] = {
            "runs": {s.run.m: run_facts(s, [t["dshot"] for t in s.run.log["ticks"]]) for s in seq.runs},
            "truth_sha256": truth,
            "clean": clean,
            "predicate": {"passed": ev["passed"], "violations": ev["violations"], "stale": ev["stale_reads_in_window"],
                          "channels": {c: {"violations": st["violations"], "first": st["first"], "worst": st["worst"],
                                           "max_out": st["max_out"], "max_E": st["max_E"]}
                                       for c, st in ev["components"].items()}}}
        runs += [s.run for s in seq.runs]
    out["versions"] = versions(runs, t4r.PLUGIN_DIR)
    out["compare"] = {}
    for c in out["design"]["channels"]:
        rows = {tag: {"excess": out[tag]["predicate"]["channels"][c]["worst"]["slack"],
                      "at": out[tag]["predicate"]["channels"][c]["worst"]["n"],
                      "violations": out[tag]["predicate"]["channels"][c]["violations"],
                      "first": out[tag]["predicate"]["channels"][c]["first"]} for tag in ("off", "on")}
        rows["on - off"] = {"excess": rows["on"]["excess"] - rows["off"]["excess"]}
        out["compare"][c] = rows
    return out


# ---- output -----------------------------------------------------------------------------------------------------------

TITLES = {
    "acro": "L4 acro (scenarios/quad/L04/acro.yaml): the pass bar (i)-(v) and the recovery predicate after the combined "
            "full-stick segment, tests/regression/quad/L04/gz/test_t4_acro.py",
    "r2": "L5 R2, the steady tumble (scenarios/quad/L05/recover_tumble.yaml): the six-channel envelope predicate, "
          "tests/regression/quad/L05/gz/test_t4_recovery.py",
    "r1x": "L5 R1X, exactly 180 degrees (scenarios/quad/L05/recover_inverted_exact.yaml): the alpha envelope predicate, "
           "tests/regression/quad/L05/gz/test_t4_recovery.py",
}


def render(case, r):
    L = ["MARV quad L6 stage (c): FF-on report run, reported, not asserted (decision 0014, owner decision 2; third round, "
         "item 3)",
         "command: uv run python tests/regression/quad/L06/results/ff_on/capture_gz.py --case <case> --work <dir> --out <file>",
         "label: truth-fed, perfect-model gz runs: not a validation run",
         kv("case", case),
         kv("title", TITLES[case]),
         kv("ff_on_override", FF_ON),
         "inputs sha256: " + "; ".join(f"{k} {sha_file(p)}" for k, p in r["inputs"]),
         "versions: " + "; ".join(f"{k} {v}" for k, v in r["versions"].items()),
         kv("params", {k: r["params"][k] for k in FF_PARAMS}),
         "",
         "# == design: the predicate's fixed terms (the same for both variants)"]
    L += [kv(f"design.{k}", v) for k, v in r["design"].items()]
    for tag in ("off", "on"):
        L += ["", f"# == {tag}: rate_ff_enable {'= 1 by harness sil_override' if tag == 'on' else 'at its default (no harness override)'}"]
        L += [kv(f"{tag}.{k}", v) for k, v in r[tag].items()]
    L += ["", "# == compare: excess = the worst excess over the tolerance (> 0) or minus the slack (<= 0), rad or rad/s; "
              "on - off"]
    L += [kv(f"compare.{k}", v) for k, v in r["compare"].items()]
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True, choices=tuple(TITLES))
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    r = capture_acro(a.work) if a.case == "acro" else capture_l5(a.case, a.work)
    plugin = Path(r["plugin_dir"]) / run_scenario.PLUGIN_FILE
    print(f"capture_gz: plugin sha256 {sha_file(plugin)} (not in the output)", file=sys.stderr)
    text = render(a.case, r)
    Path(a.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return r


if __name__ == "__main__":
    main()
