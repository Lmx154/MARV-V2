#!/usr/bin/env python3
"""R1X's alpha excess in gz is the gyroscopic coupling w x Jw, under the stage (c) gains and the old ones (decision 0014, owner
decisions third round, item 3, Luis, 2026-10-02: "Evidence first, as with R2: commit the counterfactual showing that Gazebo's
excess is the coupling. That means the design model plus w x Jw reproduces Gazebo, as for R2's 7.96 -> 1.46e-2, and that it
is identical under the old and new gains.").

    uv run python tests/regression/quad/L06/results/r1x_coupling/r1x_coupling.py --out <file> [--jobs <n>]

R1X (scenarios/quad/L05/recover_inverted_exact.yaml) starts at exactly [0, 1, 0, 0], at rest, rotors at hover. Its check
(tests/regression/quad/L05/gz/test_t4_recovery.py, the exact-180 degree side check) compares gz's tilt angle alpha with the
design model's alpha envelope from the exact start, by E + F + Q. At exactly 180 degrees the tilt axis is chosen by rounding
noise: gz's first plant step leaves the body at q_gz(1), and that picks the branch of the tilt-yaw split. On that branch the
body rates are not about one axis, so w x Jw is not zero; on the design model's exact branch it is (a pure roll).

Inputs, read and never retyped (the output lists each with its sha256):
  gz_new.txt, gz_old.txt  the compact gz summaries, this directory, written by capture_gz.py in the tree of each gain set
                          (new: this repository; old: a checkout of 69c62f2, its own build and T3 reference). gz is not run
                          here;
  this tree's design model and fixture: test_t4_recovery.py (make_design), recovery_model.py, attitude_t3_oracle.py,
                          attitude_t3_inputs.txt, attitude_t3_q_inputs.txt, the generated attitude_t3_envelope.txt (decision
                          0011) and the R1X scenario; r1x_model.py (the model runs, shared with capture_gz.py).

Method.
  1. Re-run (new gains). make_design(R1X, alpha) from the recorded fixture (the test's exact_design; no build is read, so
     the live-parameter check is the capture's) and r1x_model.model_runs seeded with gz_new.txt's q_gz(1). The design and
     model sections this renders must equal gz_new.txt's line for line, or the script refuses: the capture's distances were
     computed against these exact series (sha256 of every series). The old gains' model needs the 69c62f2 tree's code, so
     gz_old.txt's model section is quoted, as captured there by the same r1x_model.py (its sha256 must be today's).
  2. Distances (capture, the full gz series): max_n |gz - run| per channel over n = 0 .. N - 1; each row is checked here:
     |gz - run| at its n equals its value, and (new) the run's value there equals the re-run's.
  3. Claims, per gain set:
     (i, ii) gz's verdict is the test's; D (no w x Jw) on gz's branch stays inside the envelope at F + Q; C (+ w x Jw) on the
             same branch does what gz does (leaves it, or stays inside); C is closer to gz than D on alpha, w_x, w_y, w_z
             and err_z (the packet's channels), and its alpha distance is within the test's F + Q while D's is not.
             Control: C0 (coupled, from the exact start, not on gz's branch) is not closer than D on any of them.
     (iii)   old against new: q_gz(1) of both runs bit-identical; w_y / w_x at execution 300, the coupling integral
             sum_n |w x Jw| T_a on gz's rates (the fixture's J) and the coupling's alpha effect max_n |alpha_C - alpha_D| are
             reported with their relative change. No threshold is set on them: none is sourced (owner, third round item 2:
             a threshold picked after seeing a table is tuned to it).
     (iv)    CF (C with the ideal lag-compensated feed-forward) equals D within K + R on every channel (r2_envelope_a's
             rule; V = 0 here: R1X starts at rest at the hover trim) and stays inside the envelope. Control: C is beyond
             K + R from D on every channel.
Run times go to stderr; the output does not depend on --jobs.
"""

import argparse
import ast
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
sys.path.insert(0, str(HERE))
import r1x_model as mdl  # noqa: E402

GZ = {"new": HERE / "gz_new.txt", "old": HERE / "gz_old.txt"}
CLAIM = ("alpha", "w_x", "w_y", "w_z", "err_z")  # the packet's distance channels (decision 0014 third round item 3)
FIELD = re.compile(r"^([A-Za-z_][\w./]*) = (.*)$")
SECTION = re.compile(r"^# == (\d+)\.")


def read_gz(path):
    text = Path(path).read_text(encoding="utf-8")
    facts, blocks, sec, inputs = {}, {}, None, {}
    for ln in text.splitlines():
        m = SECTION.match(ln)
        if m:
            sec = int(m.group(1))
            blocks[sec] = []
            continue
        if ln.startswith("inputs sha256: "):
            for item in ln[len("inputs sha256: "):].split("; "):
                name, digest = item.rsplit(" ", 1)
                inputs[name] = digest
            continue
        m = FIELD.match(ln)
        if m:
            facts[m.group(1)] = ast.literal_eval(m.group(2))
            if sec is not None:
                blocks[sec].append(ln)
    return {"path": Path(path), "text": text, "facts": facts, "blocks": blocks, "inputs": inputs}


def rerun(gz_new, procs):
    """This tree's design and model runs seeded with gz_new.txt's q_gz(1), rendered as the capture renders them."""
    tree = mdl.load_tree(ROOT)
    t4r, rm = tree.t4r, tree.rm
    design = t4r.make_design(t4r.R1X, {}, fn=rm.alpha_channel, label="alpha")
    st = tree.l5s.values(tree.l5s.load(t4r.SCEN / f"{t4r.R1X}.yaml"))["initial_state"]
    su = rm.oracle.Setup(t4r.recorded_inputs())
    q1 = tuple(gz_new["facts"]["gz.q1"][1])
    runs = mdl.model_runs(tree, su, tuple(st["attitude_q_wxyz"]), tuple(st["body_rates_frd_rad_s"]), q1, design.count, procs)
    blocks = {1: mdl.facts_lines("design", mdl.design_facts(design, tree)),
              3: [mdl.kv("model.seed_q1", list(q1))] + mdl.facts_lines("model", mdl.model_facts(runs, design, su, tree))}
    return {"tree": tree, "design": design, "runs": runs, "blocks": blocks}


def first_difference(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return f"line {i}: re-run {x[:160]!r} against captured {y[:160]!r}"
    return None if len(a) == len(b) else f"{len(a)} lines re-run against {len(b)} captured"


def check_dist(gz, runs=None):
    """Every dist row: value == |gz - run| at its n; with runs (the re-run), the run's value there is the re-run's."""
    for k in ("D", "C", "CF", "C0"):
        for ci, c in enumerate(mdl.CHANNELS):
            d, n, y, x = gz["facts"][f"dist.{k}"][c]
            if d != abs(y - x):
                return f"dist.{k} {c}: {d!r} != |{y!r} - {x!r}|"
            if runs is not None and runs[k][n][ci] != x:
                return f"dist.{k} {c} at n {n}: captured run value {x!r}, re-run {runs[k][n][ci]!r}"
    return None


def evaluate(gz):
    f = gz["facts"]
    s, tv = f["model.summary"], f["gz.test_verdict"]
    tol = f["design.F"] + f["design.Q"]
    dist = {k: f[f"dist.{k}"] for k in ("D", "C", "CF", "C0")}
    gz_out = not tv["passed"]
    closer = {c: dist["C"][c][0] < dist["D"][c][0] for c in CLAIM}
    control = {c: dist["C0"][c][0] < dist["D"][c][0] for c in CLAIM}
    ff = f["model.ideal_ff"]
    return {
        "tol": tol,
        "gz_outside": gz_out,
        "verdict_consistent": (f["gz.summary.verdict"]["violations"], f["gz.summary.verdict"]["worst_slack"]) == (
            tv["violations"], tv["worst"]["slack"]),
        "D_inside": s["D"]["verdict"]["violations"] == 0,
        "C_outside": s["C"]["verdict"]["violations"] > 0,
        "C_does_what_gz_does": (s["C"]["verdict"]["violations"] > 0) == gz_out,
        "closer": closer,
        "C_closer_on_all": all(closer.values()),
        "alpha_C_within_tol": dist["C"]["alpha"][0] <= tol,
        "alpha_D_beyond_tol": dist["D"]["alpha"][0] > tol,
        "control_C0_closer": control,
        "control_C0_closer_on_any": any(control.values()),
        "ratio": {c: dist["D"][c][0] / dist["C"][c][0] for c in CLAIM},
        "largest": {k: max(CLAIM, key=lambda c, k=k: dist[k][c][0]) for k in ("D", "C")},
        "ff_within": all(ff["within_K_plus_R"]),
        "ff_inside": s["CF"]["verdict"]["violations"] == 0,
        "ff_control": all(ff["control_beyond_K_plus_R"]),
    }


def identity(old, new):
    fo, fn = old["facts"], new["facts"]

    def rel(a, b):
        return (b - a) / a

    eff_o, eff_n = fo["model.coupling_alpha_effect"], fn["model.coupling_alpha_effect"]
    return {
        "q1_identical": {m: fo["gz.q1"][m] == fn["gz.q1"][m] for m in (1, 2)},
        "w1_identical": {m: fo["gz.w1"][m] == fn["gz.w1"][m] for m in (1, 2)},
        "ratio": (fo["gz.summary.w_y_over_w_x"], fn["gz.summary.w_y_over_w_x"],
                  rel(fo["gz.summary.w_y_over_w_x"], fn["gz.summary.w_y_over_w_x"])),
        "cint": (fo["gz.summary.coupling_integral"], fn["gz.summary.coupling_integral"],
                 [rel(a, b) for a, b in zip(fo["gz.summary.coupling_integral"], fn["gz.summary.coupling_integral"])]),
        "cint_C": (fo["model.summary"]["C"]["coupling_integral"], fn["model.summary"]["C"]["coupling_integral"]),
        "effect": (eff_o, eff_n, rel(eff_o["value"], eff_n["value"])),
        "tol": (fo["design.F"] + fo["design.Q"], fn["design.F"] + fn["design.Q"]),
        "excess": {k: (fo["model.summary"][k]["alpha_minus_hi"], fn["model.summary"][k]["alpha_minus_hi"]) for k in ("D", "C")}
        | {"gz": (fo["gz.summary.alpha_minus_hi"], fn["gz.summary.alpha_minus_hi"])},
    }


def g(x):
    return f"{x:.4g}"


def e3(x):
    return f"{x:.3e}"


def gains_line(f):
    p = f["design.gains"]
    keys = [k for k in mdl.GAIN_KEYS if k in p and not k.startswith(("inertia", "motor"))]
    return ", ".join(f"{k} {g(p[k])}" for k in keys)


def verdict_text(v, tol_label):
    if v["violations"] == 0:
        return f"0 violations at {tol_label}; worst slack {e3(v['worst_slack'])} at n {v['worst_n']}"
    return (f"{v['violations']} violations at {tol_label}, first n {v['first']}, worst n {v['worst_n']} slack "
            f"{e3(v['worst_slack'])} (alpha {g(v['worst_alpha'])}, hi {g(v['worst_hi'])})")


def section(gz, ev, title):
    f = gz["facts"]
    s, tv = f["model.summary"], f["gz.test_verdict"]
    w = tv["worst"]
    L = [title,
         f"gains: {gains_line(f)}; design N {f['design.N']}, F {e3(f['design.F'])}, Q {e3(f['design.Q'])}, F + Q {e3(ev['tol'])}",
         f"gz (the test's evaluate, E + F + Q): {'PASS' if tv['passed'] else 'FAIL'}, {tv['violations']} violations"
         + (f", first n {tv['first']}" if tv["first"] is not None else "")
         + f", worst n {w['n']} slack {e3(w['slack'])} (alpha {g(w['y'])}, hi {g(w['hi'])}, E {e3(w['E'])}); max E "
         f"{e3(tv['max_E'])}; max(alpha - hi) {e3(f['gz.summary.alpha_minus_hi']['value'])} at n "
         f"{f['gz.summary.alpha_minus_hi']['n']}",
         f"D  design model, no w x Jw, gz's branch: {verdict_text(s['D']['verdict'], 'F + Q')}; max(alpha - hi) "
         f"{e3(s['D']['alpha_minus_hi']['value'])} at n {s['D']['alpha_minus_hi']['n']}",
         f"C  + w x Jw, gz's branch:              {verdict_text(s['C']['verdict'], 'F + Q')}; max(alpha - hi) "
         f"{e3(s['C']['alpha_minus_hi']['value'])} at n {s['C']['alpha_minus_hi']['n']}",
         f"C0 + w x Jw, exact start (control):    {verdict_text(s['C0']['verdict'], 'F + Q')}",
         "alpha below pi/2 first at n: gz " + str(f["gz.summary.alpha_below_half_pi_n"]) + ", D "
         + str(s["D"]["alpha_below_half_pi_n"]) + ", C " + str(s["C"]["alpha_below_half_pi_n"]),
         "distance to gz, max_n |gz - run| over n = 0 .. N - 1 (rad, rad/s), at n:"]
    for k, label in (("D", "D "), ("C", "C "), ("C0", "C0")):
        L.append(f"  {label} " + "; ".join(f"{c} {e3(f[f'dist.{k}'][c][0])} (n {f[f'dist.{k}'][c][1]})" for c in CLAIM))
    L.append("  D / C " + "; ".join(f"{c} {g(ev['ratio'][c])}" for c in CLAIM))
    lc, ld = ev["largest"]["C"], ev["largest"]["D"]
    L.append(f"largest of these: D {e3(f['dist.D'][ld][0])} ({ld}) -> C {e3(f['dist.C'][lc][0])} ({lc})")
    L.append(f"alpha distance against the test's F + Q {e3(ev['tol'])}: C {e3(f['dist.C']['alpha'][0])} within: "
             f"{ev['alpha_C_within_tol']}; D {e3(f['dist.D']['alpha'][0])} beyond: {ev['alpha_D_beyond_tol']}")
    return L


def render(r, ev):
    new, old = r["gz"]["new"], r["gz"]["old"]
    fn, fo = new["facts"], old["facts"]
    idt = ev["identity"]
    inputs = [("gz_new.txt", GZ["new"]), ("gz_old.txt", GZ["old"]), ("r1x_model.py", Path(mdl.__file__)),
              ("capture_gz.py", HERE / "capture_gz.py")] + r["tree_inputs"]
    L = ["MARV quad L6 stage (c): R1X's alpha excess in gz is the coupling w x Jw, under the stage (c) gains and the old ones "
         "(decision 0014, third round, item 3)",
         "command: uv run python tests/regression/quad/L06/results/r1x_coupling/r1x_coupling.py --out <file>",
         "label: the design model (T3) on gz's branch against gz's truth-fed, perfect-model runs: not a validation run",
         "inputs sha256: " + "; ".join(f"{k} {mdl.sha_file(p)}" for k, p in inputs),
         "",
         "== 0. the two gain sets and the re-run",
         f"new: {fn['tree']} (gz_new.txt)",
         f"old: {fo['tree']} (gz_old.txt; its model section is quoted: the 69c62f2 design model is not in this tree)",
         f"r1x_model.py as captured (sha256 in both summaries equals today's): {r['model_sha_ok']}",
         f"re-run here, new gains: the design and every model series equal gz_new.txt's, line for line: {r['rerun_equal']}",
         f"dist rows: |gz - run| at the stated n (new: and the run's value is the re-run's): new {r['dist_new_ok']}, old "
         f"{r['dist_old_ok']}",
         f"gz's own verdict equals the capture's predicate on gz's alpha with E: new {ev['new']['verdict_consistent']}, old "
         f"{ev['old']['verdict_consistent']}",
         ""]
    L += section(new, ev["new"], "== 1. (i) new gains: gz against the design model's nominal member on gz's branch")
    L += [""]
    L += section(old, ev["old"], "== 2. (ii) old gains (69c62f2): the same")
    eo, en, er = idt["effect"]
    L += ["",
          "== 3. (iii) old against new (gz: the m = 1 run unless stated; model: gz's branch, nominal member)",
          f"q_gz(1): m = 1 old {fo['gz.q1'][1]} new {fn['gz.q1'][1]}: bit-identical {idt['q1_identical'][1]}; m = 2 "
          f"bit-identical {idt['q1_identical'][2]}",
          f"w_gz(1): bit-identical m = 1 {idt['w1_identical'][1]}, m = 2 {idt['w1_identical'][2]}",
          f"w_y / w_x at n {mdl.RATIO_EXEC} (gz, the branch's tilt-axis split): old {g(idt['ratio'][0])} new {g(idt['ratio'][1])} "
          f"(change {idt['ratio'][2]:+.2%})",
          "coupling integral sum_n |w x Jw| T_a (gz, N m s, roll pitch yaw): old " + str([g(x) for x in idt["cint"][0]])
          + " new " + str([g(x) for x in idt["cint"][1]]) + " (change " + ", ".join(f"{x:+.1%}" for x in idt["cint"][2]) + ")",
          "  the same on C's rates: old " + str([g(x) for x in idt["cint_C"][0]]) + " new " + str([g(x) for x in idt["cint_C"][1]]),
          f"the coupling's alpha effect max_n |alpha_C - alpha_D|: old {g(eo['value'])} at n {eo['n']} new {g(en['value'])} at n "
          f"{en['n']} (change {er:+.1%})",
          f"the envelope's F + Q: old {e3(idt['tol'][0])} new {e3(idt['tol'][1])}",
          "max(alpha - hi), old -> new: " + "; ".join(
              f"{k} {e3(a['value'])} (n {a['n']}) -> {e3(b['value'])} (n {b['n']})" for k, (a, b) in idt["excess"].items()),
          "no threshold is set on these changes (none is sourced); they are reported"]
    L += ["",
          "== 4. (iv) the ideal lag-compensated feed-forward on the coupled model (CF), gz's branch, nominal member"]
    for tag, gz in (("new", new), ("old", old)):
        ff = gz["facts"]["model.ideal_ff"]
        L.append(f"{tag}: max_n |CF - D| " + str([e3(x) for x in ff["CF_minus_D"]]) + " <= K + R "
                 + str([e3(k + rr) for k, rr in zip(ff["K_CF"], ff["R"])]) + f": {ev[tag]['ff_within']}; CF "
                 + verdict_text(gz["facts"]["model.summary"]["CF"]["verdict"], "F + Q"))
        L.append(f"  control, max_n |C - D| " + str([e3(x) for x in ff["control_C_minus_D"]]) + " beyond K + R "
                 + str([e3(k + rr) for k, rr in zip(ff["K_C"], ff["R"])]) + f" on every channel: {ev[tag]['ff_control']}")
    L.append("  channels " + ", ".join(mdl.CHANNELS))
    L += ["",
          "== 5. claims"]
    for tag in ("new", "old"):
        e = ev[tag]
        L.append(f"{tag}: gz leaves the envelope {e['gz_outside']}; D inside {e['D_inside']}; C leaves it {e['C_outside']} "
                 f"(as gz: {e['C_does_what_gz_does']}); C closer to gz than D on {', '.join(CLAIM)}: {e['C_closer_on_all']}; "
                 f"C's alpha distance within F + Q {e['alpha_C_within_tol']}, D's beyond {e['alpha_D_beyond_tol']}; control C0 "
                 f"closer than D on any: {e['control_C0_closer_on_any']}; CF within K + R of D {e['ff_within']} and inside "
                 f"{e['ff_inside']}; control C beyond K + R {e['ff_control']}")
    L.append(f"old and new: q_gz(1) bit-identical (both runs) {all(idt['q1_identical'].values())}")
    L.append(f"the excess is the coupling: {ev['claim']}")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=len(mdl.RUNS))
    a = ap.parse_args(argv)
    gz = {k: read_gz(p) for k, p in GZ.items()}
    model_sha = mdl.sha_file(mdl.__file__)
    model_sha_ok = all(x["inputs"].get("r1x_model.py") == model_sha for x in gz.values())
    if not model_sha_ok:
        raise SystemExit("r1x_coupling: r1x_model.py differs from the one the gz summaries were captured with; re-capture")
    clock = time.monotonic()
    re_ = rerun(gz["new"], a.jobs)
    print(f"r1x_coupling: wall: re-run {time.monotonic() - clock:.1f} s", file=sys.stderr)
    diffs = {s: first_difference(re_["blocks"][s], gz["new"]["blocks"][s]) for s in (1, 3)}
    if any(diffs.values()):
        raise SystemExit(f"r1x_coupling: the re-run differs from gz_new.txt: {diffs}")
    dist_new, dist_old = check_dist(gz["new"], re_["runs"]), check_dist(gz["old"])
    if dist_new or dist_old:
        raise SystemExit(f"r1x_coupling: dist rows inconsistent: new {dist_new}; old {dist_old}")
    t4r, rm = re_["tree"].t4r, re_["tree"].rm
    tree_inputs = [("test_t4_recovery.py", Path(t4r.__file__)), ("recovery_model.py", Path(rm.__file__)),
                   ("attitude_t3_oracle.py", Path(rm.oracle.__file__)),
                   ("attitude_t3_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_inputs.txt"),
                   ("attitude_t3_q_inputs.txt", t4r.T3_REFERENCE / "attitude_t3_q_inputs.txt"),
                   ("attitude_t3_envelope.txt (generated, decision 0011)", t4r.T3_GENERATED / "attitude_t3_envelope.txt"),
                   ("scenario", t4r.SCEN / f"{t4r.R1X}.yaml")]
    tree_inputs = [(k, Path(p)) for k, p in tree_inputs]
    r = {"gz": gz, "rerun": re_, "model_sha_ok": model_sha_ok, "rerun_equal": not any(diffs.values()),
         "dist_new_ok": dist_new is None, "dist_old_ok": dist_old is None, "tree_inputs": tree_inputs}
    ev = {k: evaluate(x) for k, x in gz.items()}
    ev["identity"] = identity(gz["old"], gz["new"])
    ev["claim"] = (all(ev[k]["verdict_consistent"] and ev[k]["D_inside"] and ev[k]["C_does_what_gz_does"]
                       and ev[k]["C_closer_on_all"] and ev[k]["alpha_C_within_tol"] and ev[k]["alpha_D_beyond_tol"]
                       and not ev[k]["control_C0_closer_on_any"] and ev[k]["ff_within"] and ev[k]["ff_inside"]
                       and ev[k]["ff_control"] for k in ("new", "old"))
                   and ev["new"]["gz_outside"] and all(ev["identity"]["q1_identical"].values()))
    text = render(r, ev)
    Path(a.out).write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return r, ev, text


if __name__ == "__main__":
    main()
