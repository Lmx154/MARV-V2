"""R2's E, measured on its own two runs (decision 0014, second round item 2): tests/regression/quad/L06/results/r2_lower_bound/
e_measured.txt, the raw output of measure_e.py next to it (README: e_measured.md).

The measurement needs gz, so this test does not re-run it: it parses the committed raw output and checks
  - internal consistency: the printed executions are contiguous and cover the proof's window, and every printed E is
    |y1 - y2| of its two printed rates, exactly; the summary's window maximum is the rows' maximum; the stop-condition lines
    quote the rows' E(n*) and X* - (F + Q) exactly; control: a row whose y2 moves by one ulp fails;
  - the proof the comparison used is the committed one: n*, X*, F + Q and X* - (F + Q) of every rate axis and the binding axis
    render as bound.txt prints them; control: X* planted at F + Q renders differently;
  - the comparison (the stop condition of decision 0014, second round, lead note): on the binding axis and on every forced
    axis E(n*) < X* - (F + Q), so the measured E does not close the gap and the proof stands; control: E(n*) planted at
    X* - (F + Q) trips the stop condition.
"""

import copy
import math
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "r2_lower_bound"
RATE = ("w_x", "w_y", "w_z")
BOOL = {"True": True, "False": False}

ROW = re.compile(r"^exec (\d+): " + "; ".join(rf"{c} (\S+) (\S+) (\S+)" for c in RATE) + "$", re.M)
SUMMARY = re.compile(r"^(w_[xyz]): executions (\d+); max E (\S+) at n (\d+); mean E \S+; median E \S+; max over the proof's window "
                     r"n = (\d+) \.\. (\d+) (\S+) at n (\d+); at n = .*; equals the test's evaluate max_E: (True|False)$", re.M)
STOP = re.compile(r"^(w_[xyz]): n\* (\d+); X\* (\S+); F \+ Q (\S+); X\* - \(F \+ Q\) (\S+); forced \(X\* > F \+ Q\): (True|False); "
                  r"E\(n\*\) (\S+); E\(n\*\) >= X\* - \(F \+ Q\): (True|False); X\(n\) - \(F \+ Q\) > E\(n\) at n = .* "
                  r"\((\d+) of (\d+) \.\. (\d+)\)$", re.M)
BINDING = re.compile(r"^binding axis: (w_[xyz]); stop \(E\(n\*\) >= X\* - \(F \+ Q\) on the binding axis\): (True|False); "
                     r"on any forced axis: (True|False)$", re.M)
BOUND_AXIS = re.compile(r"^(w_[xyz]): F \S+ \(its halving term \S+\) Q \S+ F \+ Q (\S+) rad/s; .*; X\* = max_n X\(n\) (\S+) rad/s "
                        r"at n (\d+) \(t \S+ s\); X\*/\(F \+ Q\) \S+; E needed at n\*: >= (\S+) rad/s$", re.M)
BOUND_BINDING = re.compile(r"^binding axis \(largest X\* - \(F \+ Q\)\): (w_[xyz]);", re.M)
BOUND_WINDOW = re.compile(r"; window (\d+) executions = ")


def g(x):
    """bound.txt's number format (r2_lower_bound.py g)."""
    return f"{x:.5g}"


def parse(text):
    rows = {int(m.group(1)): {c: tuple(float(v) for v in m.groups()[1 + 3 * i:4 + 3 * i]) for i, c in enumerate(RATE)}
            for m in ROW.finditer(text)}
    summary = {m.group(1): {"count": int(m.group(2)), "max": float(m.group(3)), "n_max": int(m.group(4)),
                            "win": (int(m.group(5)), int(m.group(6))), "win_max": float(m.group(7)), "n_win": int(m.group(8)),
                            "eval_equal": BOOL[m.group(9)]} for m in SUMMARY.finditer(text)}
    stop = {m.group(1): {"n_star": int(m.group(2)), "X": float(m.group(3)), "tol": float(m.group(4)), "need": float(m.group(5)),
                         "forced": BOOL[m.group(6)], "E": float(m.group(7)), "covers": BOOL[m.group(8)],
                         "win": (int(m.group(10)), int(m.group(11)))} for m in STOP.finditer(text)}
    b = BINDING.search(text)
    return {"rows": rows, "summary": summary, "stop": stop,
            "binding": None if b is None else {"axis": b.group(1), "stop": BOOL[b.group(2)], "any": BOOL[b.group(3)]}}


def parse_bound(text):
    axes = {m.group(1): {"tol": m.group(2), "X": m.group(3), "n_star": int(m.group(4)), "need": m.group(5)}
            for m in BOUND_AXIS.finditer(text)}
    return {"axes": axes, "binding": BOUND_BINDING.search(text).group(1), "window": int(BOUND_WINDOW.search(text).group(1))}


def first_max(rows, c, lo, hi):
    """(the largest E of channel c over rows lo .. hi, the first n where it occurs), as measure_e.py picks it."""
    n = max(range(lo, hi + 1), key=lambda k: rows[k][c][2])
    return rows[n][c][2], n


def consistency_failures(p, window):
    """Internal consistency of a parsed e_measured.txt (module docstring), as a list of failures."""
    out = []
    rows = p["rows"]
    ns = sorted(rows)
    if not ns or ns != list(range(ns[0], ns[-1] + 1)) or ns[0] > 1 or ns[-1] < window:
        return [f"rows {ns[:1]} .. {ns[-1:]} are not contiguous or do not cover the proof's window 1 .. {window}"]
    for n in ns:
        for c in RATE:
            y1, y2, e = rows[n][c]
            if e != abs(y1 - y2):
                out.append(f"exec {n} {c}: E {e!r} is not |y1 - y2| = {abs(y1 - y2)!r}")
    for c in RATE:
        s = p["summary"].get(c)
        if s is None:
            out.append(f"{c}: no summary line")
            continue
        lo, hi = s["win"]
        if (lo, hi) != (ns[0], window):
            out.append(f"{c}: summary window {lo} .. {hi} is not {ns[0]} .. {window}")
        elif (s["win_max"], s["n_win"]) != first_max(rows, c, lo, hi):
            out.append(f"{c}: summary window max {s['win_max']!r} at n {s['n_win']} is not the rows' {first_max(rows, c, lo, hi)}")
        if s["max"] < max(rows[n][c][2] for n in ns) or not s["eval_equal"]:
            out.append(f"{c}: summary max {s['max']!r} below a row's E, or not the test's evaluate max_E")
        st = p["stop"].get(c)
        if st is None:
            out.append(f"{c}: no stop-condition line")
            continue
        if st["n_star"] not in rows or st["E"] != rows[st["n_star"]][c][2]:
            out.append(f"{c}: E(n*) {st['E']!r} is not the row's E at n* {st['n_star']}")
        if st["need"] != st["X"] - st["tol"] or st["forced"] != (st["X"] > st["tol"]) or st["covers"] != (st["E"] >= st["need"]):
            out.append(f"{c}: the stop-condition line is inconsistent: {st}")
        if st["win"] != (ns[0], window):
            out.append(f"{c}: the stop-condition window {st['win']} is not {ns[0]} .. {window}")
    b = p["binding"]
    if b is None or b["axis"] not in p["stop"]:
        return out + ["no binding-axis line"]
    stops = {c: st["forced"] and st["covers"] for c, st in p["stop"].items()}
    if b["stop"] != stops[b["axis"]] or b["any"] != any(stops.values()):
        out.append(f"the binding-axis line {b} is inconsistent with the axes {stops}")
    return out


def bound_failures(p, bound):
    """The proof the comparison used against bound.txt, as a list of failures."""
    out = []
    if p["binding"] is None or p["binding"]["axis"] != bound["binding"]:
        out.append(f"binding axis {p['binding']} is not bound.txt's {bound['binding']}")
    for c in RATE:
        st, bd = p["stop"][c], bound["axes"][c]
        got = {"n_star": st["n_star"], "X": g(st["X"]), "tol": g(st["tol"]), "need": g(st["need"])}
        if got != bd:
            out.append(f"{c}: {got} is not bound.txt's {bd}")
    return out


def stop_failures(p):
    """The comparison: on the binding axis and on every forced axis, E(n*) < X* - (F + Q) (the proof stands)."""
    b = p["binding"]["axis"]
    out = [] if p["stop"][b]["forced"] else [f"the binding axis {b} is not forced: {p['stop'][b]}"]
    for c, st in p["stop"].items():
        if (c == b or st["forced"]) and not st["E"] < st["need"]:
            out.append(f"{c}: E(n*) {st['E']!r} >= X* - (F + Q) {st['need']!r} at n* {st['n_star']}: the measured E closes the gap")
    if p["binding"]["stop"] or p["binding"]["any"]:
        out.append(f"the file reports the stop condition: {p['binding']}")
    return out


@pytest.fixture(scope="module")
def measured():
    return parse((DIR / "e_measured.txt").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def bound():
    return parse_bound((DIR / "bound.txt").read_text(encoding="utf-8"))


def test_raw_output_is_internally_consistent(measured, bound):
    assert set(measured["stop"]) == set(RATE) and set(measured["summary"]) == set(RATE)
    assert consistency_failures(measured, bound["window"]) == []


def test_control_a_row_moved_by_one_ulp_fails_the_consistency_check(measured, bound):
    planted = copy.deepcopy(measured)
    c = measured["binding"]["axis"]
    n = measured["stop"][c]["n_star"]
    y1, y2, e = planted["rows"][n][c]
    planted["rows"][n][c] = (y1, math.nextafter(y2, math.inf), e)
    assert consistency_failures(planted, bound["window"]) != []


def test_comparison_used_the_committed_proof(measured, bound):
    assert bound_failures(measured, bound) == []


def test_control_a_changed_proof_value_fails_the_bound_check(measured, bound):
    planted = copy.deepcopy(measured)
    c = measured["binding"]["axis"]
    planted["stop"][c]["X"] = planted["stop"][c]["tol"]
    assert bound_failures(planted, bound) != []


def test_measured_E_does_not_close_the_gap_the_proof_stands(measured):
    assert stop_failures(measured) == []


def test_control_E_planted_at_the_needed_value_trips_the_stop_condition(measured):
    planted = copy.deepcopy(measured)
    c = measured["binding"]["axis"]
    planted["stop"][c]["E"] = planted["stop"][c]["need"]
    assert stop_failures(planted) != []
