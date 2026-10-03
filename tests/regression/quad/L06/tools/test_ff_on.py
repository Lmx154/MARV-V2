"""The FF-on report runs of L6 stage (c) (decision 0014, owner decision 2; third round, item 3):
tests/regression/quad/L06/results/ff_on/. Reported, not asserted: no verdict of a run is checked here.

gz is not run. gz_acro.txt, gz_r2.txt and gz_r1x.txt, the compact summaries capture_gz.py wrote, are read back and checked
for internal consistency only:
  - the switch: the FF-on runs carry exactly the harness override rate_ff_enable = 1 and the FF-off runs none, the build's
    default is 0, and each FF-on run's series differs from the FF-off run's (the override reached the firmware);
  - each recorded verdict follows from its recorded statistics by the test's own rule (acro: the pass bar (i)-(v) and the
    recovery predicate, per axis; R2 and R1X: the envelope predicate, per channel), and each recorded worst execution
    re-computes from its recorded terms (acro recovery: margin = Z + F + E - |w|; R2, R1X: slack = out - (E + F + Q),
    out = max(lo - y, y - hi));
  - the compare section equals the variants' records, with on - off recomputed.
Controls (core 7.2): a planted inconsistency of each kind is caught.
"""

import ast
import copy
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "ff_on"
CASES = ("acro", "r2", "r1x")
FF_ON = {"rate_ff_enable": ("i32", "1")}
KV = re.compile(r"^([A-Za-z_][A-Za-z0-9_.]*) = (.*)$")


def read(case):
    out = {}
    for line in (DIR / f"gz_{case}.txt").read_text(encoding="utf-8").splitlines():
        m = KV.match(line)
        if m:
            out[m.group(1)] = ast.literal_eval(m.group(2))
    return out


def tree(flat):
    """{"off": {...}, "on": {...}, "design": {...}, "compare": {...}, ...} from the dotted keys."""
    out = {}
    for k, v in flat.items():
        head, _, rest = k.partition(".")
        if rest:
            out.setdefault(head, {})[rest] = v
        else:
            out[head] = v
    return out


@pytest.fixture(scope="module", params=CASES)
def summary(request):
    return request.param, tree(read(request.param))


# ---- the checks (each returns its findings) -----------------------------------------------------------------------------

def switch_findings(t):
    bad = []
    if t["ff_on_override"] != FF_ON:
        bad.append(f"ff_on_override {t['ff_on_override']}")
    if t["params"]["rate_ff_enable"] != 0:
        bad.append(f"the build's rate_ff_enable is {t['params']['rate_ff_enable']}, not 0")
    for tag, want, sil in (("off", {}, None), ("on", FF_ON, FF_ON["rate_ff_enable"])):
        runs = t[tag]["runs"]
        if sorted(runs) != [1, 2]:
            bad.append(f"{tag}: runs m {sorted(runs)}")
        for m, r in runs.items():
            if r["harness_overrides"] != want or r["sil_override_rate_ff_enable"] != sil:
                bad.append(f"{tag} m = {m}: overrides {r['harness_overrides']}, rate_ff_enable {r['sil_override_rate_ff_enable']}")
    key = "gyro_sha256" if "gyro_sha256" in t["off"] else "truth_sha256"
    for m in (1, 2):
        if t["off"][key][m] == t["on"][key][m]:
            bad.append(f"m = {m}: the FF-on series equals the FF-off series")
    return bad


def acro_findings(v):
    bad = []
    pb = v["pass_bar"]
    rb = pb["rate_bound"]
    for a, r in rb.items():
        if (r["violations"] > 0) != (r["smallest_slack"] < 0) or (r["first"] is None) != (r["violations"] == 0):
            bad.append(f"pass bar {a}: violations {r['violations']}, slack {r['smallest_slack']}, first {r['first']}")
        if r["violations"] and not r["first"] <= r["at"]:
            bad.append(f"pass bar {a}: first {r['first']} after the worst execution {r['at']}")
    passed = (pb["clean"] == "yes" and not any(pb["dshot_out_of_range"].values())
              and not any(r["violations"] for r in rb.values()) and not any(pb["stale_reads"].values())
              and all(x == "none" for x in pb["replay_first_mismatch"].values()) and pb["saturation"]["passed"])
    if pb["passed"] != passed:
        bad.append(f"pass bar: recorded {pb['passed']}, the rule gives {passed}")
    rec = v["recovery"]
    for a, r in rec["axes"].items():
        n = r["violations"]
        if r["passed"] != (n == 0) or (n > 0) != (r["worst margin"] < 0):
            bad.append(f"recovery {a}: passed {r['passed']}, violations {n}, worst margin {r['worst margin']}")
        if (r["first violation"] is None) != (n == 0) or (r["last violation"] is None) != (n == 0):
            bad.append(f"recovery {a}: first/last {r['first violation']}/{r['last violation']} with {n} violations")
        if n and not r["first violation"] <= r["at execution"] <= r["last violation"]:
            bad.append(f"recovery {a}: the worst execution {r['at execution']} is outside its violations")
        if r["Z + F + E there"] - abs(r["w there"]) != r["worst margin"]:
            bad.append(f"recovery {a}: Z + F + E - |w| does not give the worst margin")
    want = all(r["passed"] for r in rec["axes"].values()) and not any(pb["stale_reads"].values())
    if rec["passed"] != want:
        bad.append(f"recovery: recorded {rec['passed']}, the rule gives {want}")
    return bad


def l5_findings(v, design):
    bad = []
    p = v["predicate"]
    for ci, c in enumerate(design["channels"]):
        st = p["channels"][c]
        w, n = st["worst"], st["violations"]
        if (n > 0) != (w["slack"] > 0) or (st["first"] is None) != (n == 0):
            bad.append(f"{c}: violations {n}, worst slack {w['slack']}, first {st['first']}")
        if n and not st["first"] <= w["n"]:
            bad.append(f"{c}: first {st['first']} after the worst execution {w['n']}")
        if max(w["lo"] - w["y"], w["y"] - w["hi"]) != w["outside"]:
            bad.append(f"{c}: the worst execution's outside does not re-compute")
        if w["outside"] - (w["E"] + design["F"][ci] + design["Q"][ci]) != w["slack"]:
            bad.append(f"{c}: the worst execution's slack does not re-compute from out, E, F and Q")
        if st["max_out"] < w["outside"] or st["max_E"] < w["E"]:
            bad.append(f"{c}: max_out or max_E below the worst execution's")
    total = sum(st["violations"] for st in p["channels"].values())
    if p["violations"] != total or p["passed"] != (total == 0 and not any(p["stale"].values())):
        bad.append(f"predicate: passed {p['passed']}, violations {p['violations']} against the channels' {total}")
    return bad


def compare_findings(case, t):
    bad = []
    for ch, rows in t["compare"].items():
        for tag in ("off", "on"):
            v = t[tag]
            if case == "acro":
                r, b = v["recovery"]["axes"][ch], v["pass_bar"]["rate_bound"][ch]
                want = {"recovery_excess": -r["worst margin"], "recovery_violations": r["violations"],
                        "recovery_first": r["first violation"], "pass_bar_excess": -b["smallest_slack"],
                        "pass_bar_violations": b["violations"], "pass_bar_first": b["first"]}
            else:
                st = v["predicate"]["channels"][ch]
                want = {"excess": st["worst"]["slack"], "at": st["worst"]["n"], "violations": st["violations"],
                        "first": st["first"]}
            if rows[tag] != want:
                bad.append(f"compare {ch} {tag}: {rows[tag]} against the record {want}")
        delta = {k: rows["on"][k] - rows["off"][k] for k in rows["on - off"]}
        if rows["on - off"] != delta or not rows["on - off"]:
            bad.append(f"compare {ch}: on - off {rows['on - off']} against {delta}")
    return bad


def findings(case, t):
    out = switch_findings(t) + compare_findings(case, t)
    for tag in ("off", "on"):
        out += [f"{tag}: {f}" for f in (acro_findings(t[tag]) if case == "acro" else l5_findings(t[tag], t["design"]))]
    return out


# ---- the tests ----------------------------------------------------------------------------------------------------------

def test_summary_is_internally_consistent(summary):
    case, t = summary
    assert t["case"] == case
    assert findings(case, t) == []


def test_control_a_wrong_override_is_caught(summary):
    case, t = summary
    planted = copy.deepcopy(t)
    planted["on"]["runs"][1]["harness_overrides"] = {}
    assert switch_findings(planted)
    planted = copy.deepcopy(t)
    planted["params"]["rate_ff_enable"] = 1
    assert switch_findings(planted)


def test_control_an_on_series_equal_to_off_is_caught(summary):
    case, t = summary
    planted = copy.deepcopy(t)
    key = "gyro_sha256" if case == "acro" else "truth_sha256"
    planted["on"][key][1] = planted["off"][key][1]
    assert switch_findings(planted)


def test_control_a_flipped_verdict_is_caught(summary):
    case, t = summary
    for tag in ("off", "on"):
        planted = copy.deepcopy(t)
        if case == "acro":
            planted[tag]["recovery"]["passed"] = not planted[tag]["recovery"]["passed"]
            assert acro_findings(planted[tag])
            planted = copy.deepcopy(t)
            planted[tag]["pass_bar"]["passed"] = not planted[tag]["pass_bar"]["passed"]
            assert acro_findings(planted[tag])
        else:
            planted[tag]["predicate"]["passed"] = not planted[tag]["predicate"]["passed"]
            assert l5_findings(planted[tag], planted["design"])


def test_control_a_changed_worst_execution_is_caught(summary):
    case, t = summary
    for tag in ("off", "on"):
        planted = copy.deepcopy(t)
        if case == "acro":
            r = next(iter(planted[tag]["recovery"]["axes"].values()))
            r["worst margin"] = r["worst margin"] * 2 if r["worst margin"] else 1.0
            assert acro_findings(planted[tag])
        else:
            st = next(iter(planted[tag]["predicate"]["channels"].values()))
            st["worst"]["slack"] = st["worst"]["slack"] * 2 if st["worst"]["slack"] else 1.0
            assert l5_findings(planted[tag], planted["design"])


def test_control_a_changed_comparison_is_caught(summary):
    case, t = summary
    ch = next(iter(t["compare"]))
    planted = copy.deepcopy(t)
    k = next(iter(planted["compare"][ch]["on - off"]))
    planted["compare"][ch]["on - off"][k] += 1.0
    assert compare_findings(case, planted)
    planted = copy.deepcopy(t)
    k = "recovery_violations" if case == "acro" else "violations"
    planted["compare"][ch]["on"][k] += 1
    assert compare_findings(case, planted)
