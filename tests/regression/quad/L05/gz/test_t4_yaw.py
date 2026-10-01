"""T4 yaw release and yaw-lock fallback of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4;
decision 0006 F "T4 yaw release (owner decisions 14 and 17)", "T4 yaw-lock fallback (owner decision 18)" and "Tolerance
terms at T4"; D for the lock logic).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/yaw_release.yaml and yaw_fallback.yaml, one gz process per m of the m_sequence [2, 1]
(run_l5.run_sequence). The scripts are the T3 scripts (test_l5_scenario.py cross-checks them against the envelope header).

Envelope. The RECORDED T3 envelope, tests/regression/quad/L05/t3/reference/attitude_t3_envelope.txt, the blocks
`yaw_release` and `yaw_fallback`: the design model over the tau x J box with D's braking, crossing, fallback and lock logic,
never re-seeded, one lo / hi pair per attitude execution n of the channels
  omega            the world-down yaw rate, n = 0 .. 2 H (n = 0 the seed execution, n = n_r the release execution);
  heading_release  the heading relative to the release, n = n_r ..;
  heading_lock     the heading relative to the member's own lock, n = the smallest lock execution .. (a member's value is
                   in the envelope from its own lock).
Oracle execution n is the run's attitude execution a0 + n (tools/sim/run_l5.py plan).

Quantities, from the TRUTH record of each attitude execution (the float cast of the body state the firmware read, decision
0006 B): psi(n) = wrap(2 atan2(q.z, q.w)) of the canonical q (D's heading); the yaw rate w(n) = rotate(q, omega_frd).z, the
world-down component (D "Yaw rate for the lock"); sigma_r = sign w(n_r), omega_r = |w(n_r)|, t_r = the stamp of n_r.

Lock execution. The log does not carry it (the TRUTH record holds q and omega only). It is RECOMPUTED from the TRUTH
records by D's rule, exactly as D states it: the first braking execution n >= n_r where sigma_r w(n) <= 0 (the crossing) or
where both dt >= t_cross and dt alpha_min >= omega_r (the fallback, n_fb the first such execution), dt = float32(t - t_r) /
float32(1e6) from the logged stamps, the products in float32, t_cross and alpha_min the emitted parameters of the run (the
build's table and the harness overrides). The crossing wins when both hold at n_fb.

Predicate (decision 0006 F; owner decisions 19 and 21). E_c(n) = |y_c,1(n) - y_c,2(n)| (m = 1 against m = 2), F_c the
recorded T3 tolerance term of channel c (the header's settle line F of omega and of heading_lock of the script; the
heading_release channel has no line: its F is the heading_lock line's F with that channel's halving change in place of
heading_lock's, the T3 tolerance and the kinematics change being the same angle terms), Q_c the recorded DShot
quantisation term of the script and channel (t3/reference/attitude_t3_q.txt). Every band check is
    lo_c(n) - (E_c(n) + F_c + Q_c) <= y_c,1(n) <= hi_c(n) + (E_c(n) + F_c + Q_c),
and "at zero" is the same with lo = hi = 0.
Yaw release: (i) omega for n < n_r; (ii) omega for n >= n_r (sigma_r = +1, so sigma_r w is the envelope's omega, whose
most negative value is the reversal bound); (iii) n_l within [lock_execution_min, lock_execution_max] of the header, the
lock by the crossing (and before the fallback execution), and the heading relative to the lock for n >= n_l; (iv) the heading
relative to the release for n >= n_r; (v) at the last execution the yaw rate and the heading relative to the lock at zero.
Yaw fallback: (i) sigma_r w > 0 at every execution from n_r up to and including n_fb (T3's min_sigma_omega_to_fallback
convention); (ii) the lock is the fallback, n_l = n_fb, within the header's lock range; heading relative to the lock and the
yaw rate for n >= n_fb; at the last execution both at zero. The heading relative to the release is reported, not asserted.
Both: a clean run (complete trailer, a TRUTH record for every tick, no stale host step in the window) and every DShot within
[idle, 2047].

Negative controls (harness only; each passes only if the check fails).
Release: (a) a planted trace whose heading goes back to the heading where the yaw input started (shortest way, as the law
would take it), exponentially at the attitude gain, the yaw rate its derivative; (b) att_kp = 0 through sil_override (no pull
back to the lock); (c) the release moved one attitude execution off its stamp (the exact alignment check with the origin
one earlier or later).
Fallback: (d) att_yaw_t_cross overridden to the run duration, taken from the plan, so the fallback cannot fire within the
run: its heading items, with the lock taken where the build's parameters put it, fail (and with the emitted parameters the
fallback item fails); (e) a planted trace whose yaw rate crosses zero before n_fb must fail (i).
"""

import dataclasses
import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L05/t3")  # the generated golden, envelope and q (decision 0011)
ENVELOPE = T3_GENERATED / "attitude_t3_envelope.txt"
SCENARIOS = ("yaw_release", "yaw_fallback")
CHANNELS = ("omega", "heading_release", "heading_lock")
SHIFTS = (-1, 1)
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID
US_PER_S = 10 ** 6
r32 = run_l5.l4.r32


# ---- the recorded envelope ------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Channel:
    first: int
    count: int
    halving: float
    lo: list
    hi: list
    F: float
    Q: float


@dataclasses.dataclass(frozen=True)
class YawEnvelope:
    scenario: str
    release: int
    lock_min: int
    lock_max: int
    fallback_min: int
    fallback_max: int
    header: dict
    channels: dict

    @property
    def end(self):
        return self.channels["omega"].count - 1


def parse_q(path, scenario, channel):
    text = Path(path).read_text(encoding="utf-8").splitlines()
    start = text.index(f"scenario {scenario}")
    block = next((text[start + 1:start + 1 + k] for k, ln in enumerate(text[start + 1:]) if ln.startswith("scenario ")),
                 text[start + 1:])
    found = [float(ln.split()[3]) for ln in block if ln.startswith(f"channel {channel} q ")]
    assert len(found) == 1, f"{path}: {len(found)} q lines for {scenario} {channel}"
    return found[0]


def parse_envelope(path, scenario):
    text = Path(path).read_text(encoding="utf-8").splitlines()
    fs = {}
    for ln in text:
        m = re.match(rf"^#\s+{scenario} (omega|heading_lock): end \S+ < F (\S+)$", ln)
        if m:
            fs[m.group(1)] = float(m.group(2))
    assert set(fs) == {"omega", "heading_lock"}, f"{path}: settle lines of {scenario}: {sorted(fs)}"
    start = text.index(f"scenario {scenario}")
    header, pos = {}, start + 1
    while not text[pos].startswith("channel "):
        k, v = text[pos].split(" ", 1)
        header[k] = v
        pos += 1
    raw = {}
    while pos < len(text) and not text[pos].startswith("scenario "):
        m = re.match(r"channel (\w+) first (\d+) count (\d+) halving_max_change (\S+)$", text[pos])
        assert m, text[pos]
        assert text[pos + 1] == "min_max"
        first, count = int(m.group(2)), int(m.group(3))
        rows = [tuple(map(float, ln.split())) for ln in text[pos + 2:pos + 2 + count]]
        assert len(rows) == count and all(len(r) == 2 for r in rows)
        raw[m.group(1)] = (first, count, float(m.group(4)), [r[0] for r in rows], [r[1] for r in rows])
        pos += 2 + count
    assert set(raw) == set(CHANNELS), sorted(raw)
    angle_base = fs["heading_lock"] - raw["heading_lock"][2]  # T3 tolerance + kinematics halving change of the angle channels
    f = {"omega": fs["omega"], "heading_lock": fs["heading_lock"], "heading_release": angle_base + raw["heading_release"][2]}
    channels = {c: Channel(*raw[c], f[c], parse_q(T3_GENERATED / "attitude_t3_q.txt", scenario, c)) for c in CHANNELS}
    h = header
    return YawEnvelope(scenario, int(h["release_execution"]), int(h["lock_execution_min"]), int(h["lock_execution_max"]),
                       int(h["fallback_execution_min"]), int(h["fallback_execution_max"]), header, channels)


def recorded_inputs():
    out = {}
    for line in (T3_REFERENCE / "attitude_t3_inputs.txt").read_text().splitlines():
        line = line.split("#")[0].split()
        if line:
            out[line[0]] = int(line[1]) if line[1].lstrip("-").isdigit() else float.fromhex(line[1])
    return out


# ---- quantities -----------------------------------------------------------------------------------------------------

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def heading(q):
    w, x, y, z = q
    if w < 0:
        w, z = -w, -z
    return wrap(2.0 * math.atan2(z, w))


def world_down_rate(q, om):
    w, x, y, z = q
    return 2.0 * (x * z - w * y) * om[0] + 2.0 * (y * z + w * x) * om[1] + (1.0 - 2.0 * (x * x + y * y)) * om[2]


@dataclasses.dataclass
class Trace:
    t_us: list
    psi: list
    omega: list


def trace_of(s, n_exec):
    p = s.plan
    out = Trace([], [], [])
    for n in range(n_exec):
        e = s.executions[p.run_execution(n)]
        t = e["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {p.run_execution(n)}"
        out.t_us.append(e["t_us"])
        out.psi.append(heading(t["q_wxyz"]))
        out.omega.append(world_down_rate(t["q_wxyz"], t["omega_frd"]))
    return out


def lock_rule(tr, n_r, t_cross, alpha_min, end):
    """D's braking rule recomputed on the trace: dict(sigma, omega_r, n_l, by, n_fb); the crossing wins over the fallback."""
    w_r = tr.omega[n_r]
    sigma = (w_r > 0) - (w_r < 0)
    omega_r = abs(w_r)
    n_l = by = n_fb = None
    for n in range(n_r, end + 1):
        dt = r32(r32(float(tr.t_us[n] - tr.t_us[n_r])) / r32(float(US_PER_S)))
        crossing = sigma * tr.omega[n] <= 0
        fallback = dt >= t_cross and r32(dt * alpha_min) >= omega_r
        if fallback and n_fb is None:
            n_fb = n
        if n_l is None and (crossing or fallback):
            n_l, by = n, ("crossing" if crossing else "fallback")
        if n_l is not None and n_fb is not None:
            break
    return {"sigma": sigma, "omega_r": omega_r, "n_l": n_l, "by": by, "n_fb": n_fb}


def band(ch, ns, f1, f2, zero=False):
    """The band check of the module docstring over the executions ns: worst slack and the largest distance outside."""
    worst, max_out, missing, checked = None, -math.inf, [], 0
    for n in ns:
        if zero:
            lo = hi = 0.0
        elif ch.first <= n < ch.first + ch.count:
            lo, hi = ch.lo[n - ch.first], ch.hi[n - ch.first]
        else:
            missing.append(n)
            continue
        y = f1(n)
        e = abs(y - f2(n))
        out = max(lo - y, y - hi)
        tol = e + ch.F + ch.Q
        rec = {"n": n, "outside": out, "E": e, "tol": tol, "slack": out - tol, "y": y, "lo": lo, "hi": hi}
        checked += 1
        max_out = max(max_out, out)
        if worst is None or rec["slack"] > worst["slack"]:
            worst = rec
    return {"passed": worst is not None and worst["slack"] <= 0 and not missing, "worst": worst, "max_out": max_out,
            "missing": missing[:4], "checked": checked, "F": ch.F, "Q": ch.Q}


def evaluate(scenario, env, tr1, tr2, rule_params):
    """Every predicate item of the module docstring on the traces of m = 1 and m = 2; rule_params = (t_cross, alpha_min)."""
    C, n_r, end = env.channels, env.release, env.end
    t_cross, alpha = rule_params
    k1, k2 = (lock_rule(t, n_r, t_cross, alpha, end) for t in (tr1, tr2))
    w1, w2 = tr1.omega.__getitem__, tr2.omega.__getitem__
    items = {}
    in_range = all(k["n_l"] is not None and env.lock_min <= k["n_l"] <= env.lock_max for k in (k1, k2))
    n_l = max(k["n_l"] for k in (k1, k2)) if in_range else None

    def hl(tr, k):
        return lambda n: wrap(tr.psi[n] - tr.psi[k["n_l"]])
    if scenario == "yaw_release":
        items["hold_rate"] = band(C["omega"], range(0, n_r), w1, w2)
        items["lock_in_range_by_crossing"] = {
            "passed": in_range and all(k["by"] == "crossing" and k["sigma"] == 1 for k in (k1, k2))
            and all(k["n_fb"] is None or k["n_l"] < k["n_fb"] for k in (k1, k2)),
            "lock": [k1["n_l"], k2["n_l"]], "by": [k1["by"], k2["by"]], "fb": [k1["n_fb"], k2["n_fb"]],
            "range": [env.lock_min, env.lock_max], "fb_range": [env.fallback_min, env.fallback_max]}
        items["rate_after_release"] = band(C["omega"], range(n_r, end + 1), w1, w2)
        items["heading_release"] = band(C["heading_release"], range(n_r, end + 1),
                                        lambda n: wrap(tr1.psi[n] - tr1.psi[n_r]), lambda n: wrap(tr2.psi[n] - tr2.psi[n_r]))
    else:
        nf1, nf2 = k1["n_fb"], k2["n_fb"]
        ok = nf1 is not None and nf2 is not None
        items["no_crossing_before_fallback"] = {
            "passed": ok and all(min(k_sig * t.omega[n] for n in range(n_r, nf + 1)) > 0
                                 for t, k_sig, nf in ((tr1, k1["sigma"], nf1), (tr2, k2["sigma"], nf2))),
            "min_sigma_omega": [min(k["sigma"] * t.omega[n] for n in range(n_r, (k["n_fb"] if ok else end) + 1))
                                for t, k in ((tr1, k1), (tr2, k2))], "n_fb": [nf1, nf2]}
        items["fallback_locks"] = {
            "passed": ok and in_range and all(k["by"] == "fallback" and k["n_l"] == k["n_fb"] and k["sigma"] == 1
                                              for k in (k1, k2)),
            "lock": [k1["n_l"], k2["n_l"]], "by": [k1["by"], k2["by"]], "fb": [nf1, nf2],
            "range": [env.lock_min, env.lock_max]}
        items["heading_release_reported"] = band(C["heading_release"], range(n_r, end + 1),
                                                 lambda n: wrap(tr1.psi[n] - tr1.psi[n_r]),
                                                 lambda n: wrap(tr2.psi[n] - tr2.psi[n_r]))
        items["heading_release_reported"]["reported_only"] = True
        n_fb = max(nf1, nf2) if ok else None
    ns_lock = range(n_l, end + 1) if n_l is not None and all(k["n_l"] is not None for k in (k1, k2)) else range(0)
    items["heading_lock"] = band(C["heading_lock"], ns_lock, hl(tr1, k1) if k1["n_l"] is not None else None,
                                 hl(tr2, k2) if k2["n_l"] is not None else None) if len(ns_lock) else \
        {"passed": False, "worst": None, "max_out": math.nan, "missing": [], "checked": 0, "F": C["heading_lock"].F,
         "Q": C["heading_lock"].Q}
    if scenario == "yaw_fallback":
        first_rate = n_fb if n_fb is not None else end + 1
        items["rate_after_fallback"] = band(C["omega"], range(first_rate, end + 1), w1, w2) if n_fb is not None else \
            {"passed": False, "worst": None, "max_out": math.nan, "missing": [], "checked": 0, "F": C["omega"].F,
             "Q": C["omega"].Q}
    last = range(end, end + 1)
    items["end_rate_at_zero"] = band(C["omega"], last, w1, w2, zero=True)
    items["end_heading_lock_at_zero"] = band(C["heading_lock"], last,
                                             (lambda n: wrap(tr1.psi[n] - tr1.psi[k1["n_l"]])) if k1["n_l"] is not None else None,
                                             (lambda n: wrap(tr2.psi[n] - tr2.psi[k2["n_l"]])) if k2["n_l"] is not None else None,
                                             zero=True) if k1["n_l"] is not None and k2["n_l"] is not None else \
        {"passed": False, "worst": None, "max_out": math.nan, "missing": [], "checked": 0, "F": C["heading_lock"].F,
         "Q": C["heading_lock"].Q}
    asserted = {k: v for k, v in items.items() if not v.get("reported_only")}
    return {"passed": all(v["passed"] for v in asserted.values()), "items": items, "rule": [k1, k2],
            "failed": [k for k, v in asserted.items() if not v["passed"]]}


# ---- the flown runs -------------------------------------------------------------------------------------------------

def emitted_params(live_params, s_overrides):
    out = dict(live_params)
    for k, (_, v) in s_overrides.items():
        if k in out:
            out[k] = float(v)
    return out


@dataclasses.dataclass
class Flown:
    runs: list
    traces: dict  # m -> Trace
    wall_s: float
    rule_params: tuple
    evaluation: dict


def fly(tmp_path_factory, name, scenario, env, live_params, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"{scenario}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    by_m = {s.run.m: s for s in runs}
    traces = {m: trace_of(by_m[m], env.end + 1) for m in (1, 2)}
    e = emitted_params(live_params, by_m[1].harness_overrides)
    rule_params = (e["att_yaw_t_cross"], e["att_yaw_alpha_min"])
    ev = evaluate(scenario, env, traces[1], traces[2], rule_params)
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, {"passed": ev["passed"], "failed": ev["failed"], "gz_wall_s": wall,
                               "lock_rule": ev["rule"], "rule_params (t_cross, alpha_min)": list(rule_params)}, path)
    return Flown(runs, traces, wall, rule_params, ev)


@pytest.fixture(scope="module", params=SCENARIOS)
def scenario(request):
    return request.param


@pytest.fixture(scope="module")
def envelope(scenario):
    return parse_envelope(ENVELOPE, scenario)


@pytest.fixture(scope="module")
def nominal(tmp_path_factory, scenario, envelope, live_params):
    return fly(tmp_path_factory, scenario, scenario, envelope, live_params)


@pytest.fixture(scope="module")
def release_env():
    return parse_envelope(ENVELOPE, "yaw_release")


@pytest.fixture(scope="module")
def fallback_env():
    return parse_envelope(ENVELOPE, "yaw_fallback")


@pytest.fixture(scope="module")
def release_flown(tmp_path_factory, release_env, live_params):
    return fly(tmp_path_factory, "yaw_release", "yaw_release", release_env, live_params)


@pytest.fixture(scope="module")
def fallback_flown(tmp_path_factory, fallback_env, live_params):
    return fly(tmp_path_factory, "yaw_fallback", "yaw_fallback", fallback_env, live_params)


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, release_env, live_params):
    return fly(tmp_path_factory, "yaw_release_kp_zero", "yaw_release", release_env, live_params,
               {"att_kp": (run_l5.F32, "0.0")})


@pytest.fixture(scope="module")
def t_cross_run(tmp_path_factory, fallback_env, live_params):
    plan = run_l5.plan(run_l5.l5s.load(SCEN / "yaw_fallback.yaml"), live_params, CARD)
    seconds = float(plan.duration_ticks * plan.tick_s)  # the run duration, from the plan (no new number)
    return fly(tmp_path_factory, "yaw_fallback_t_cross_run", "yaw_fallback", fallback_env, live_params,
               {"att_yaw_t_cross": (run_l5.F32, repr(r32(seconds)))}), seconds


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


def item_line(tag, name, it):
    if "worst" not in it:
        return f"{tag} {name:<28} {'PASS' if it['passed'] else 'FAIL'}  " + ", ".join(f"{k} {v}" for k, v in it.items() if k != "passed")
    w = it["worst"]
    if w is None:
        return f"{tag} {name:<28} {'PASS' if it['passed'] else 'FAIL'}  (nothing checked)"
    return (f"{tag} {name:<28} {'PASS' if it['passed'] else 'FAIL'}  n_checked {it['checked']}, max outside {it['max_out']:+.4e}; "
            f"worst n {w['n']}: outside {w['outside']:+.4e}, E {w['E']:.3e}, F {it['F']:.3e}, Q {it['Q']:.3e}, "
            f"E+F+Q {w['tol']:.4e}, slack {w['slack']:+.4e}")


def report_lines(tag, ev):
    return [item_line(tag, k, v) for k, v in ev["items"].items()] + [f"{tag} lock rule m=1 {ev['rule'][0]}", f"{tag} lock rule m=2 {ev['rule'][1]}"]


def struct_flags(imu):
    return run_l5.l4.IMU.unpack(imu)[7]


def alignment_findings(s, origin):
    """Why the mapping n -> attitude execution `origin` + n does not put oracle execution n on the run's execution (as
    test_t4_steps.py); empty when it does. The disturbance stamp is part of the plan's segments' alignment of the fallback."""
    ex, p = s.executions, s.plan
    out = []
    for i, (start, _, _) in enumerate(p.segments, 1):
        seg = int(s.overrides[f"l5_seg{i}_t_us"][1])
        a = origin + start
        if not (ex[a - 1]["t_us"] < seg <= ex[a]["t_us"]):
            out.append(f"segment {i} stamp {seg} is not in (t({a - 1}), t({a})] = ({ex[a - 1]['t_us']}, {ex[a]['t_us']}]")
    if p.disturbance:
        seg = int(s.overrides["l5_dist_t0_us"][1])
        a = origin + p.disturbance["start_attitude_execution"]
        if not (ex[a - 1]["t_us"] < seg <= ex[a]["t_us"]):
            out.append(f"disturbance stamp {seg} is not in (t({a - 1}), t({a})]")
    if origin + p.end >= len(ex):
        out.append(f"the window's last execution {origin + p.end} is beyond the run's {len(ex) - 1}")
    bad = [n for n in range(1, min(p.end, len(ex) - origin - 1) + 1)
           if ex[origin + n]["t_us"] - ex[origin + n - 1]["t_us"] != p.stamp_us(origin + n) - p.stamp_us(origin + n - 1)]
    if bad:
        out.append(f"{len(bad)} window dt differ from the oracle's (first at n = {bad[0]})")
    if any(x["tick"] != p.tick_of(x["a"]) for x in ex):
        out.append("an execution's logged tick is not D_a a")
    return out


def dshot_changes(s):
    ticks = s.run.log["ticks"]
    return [t["tick"] for a, t in zip(ticks, ticks[1:]) if a["dshot"] != t["dshot"]]


def check_clean(flown, live_params):
    idle, top = run_l5.l4.dshot_idle_bound(live_params)
    assert [s.run.m for s in flown.runs] == [2, 1]
    for s in flown.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert len(s.run.log["truths"]) == len(s.run.log["ticks"]), "a tick without a TRUTH record"
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
        lo, hi = min(min(t["dshot"]) for t in s.run.log["ticks"]), max(max(t["dshot"]) for t in s.run.log["ticks"])
        assert idle <= lo and hi <= top, f"DShot [{lo}, {hi}] outside [{idle}, {top}]"


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_yaw_meets_the_envelope(nominal, envelope, scenario, live_params, capsys):
    ev = nominal.evaluation
    _say(capsys, *report_lines(scenario, ev), f"    gz wall {nominal.wall_s:.1f} s; rule params (t_cross, alpha_min) "
         f"{nominal.rule_params}; design lock range [{envelope.lock_min}, {envelope.lock_max}], fallback range "
         f"[{envelope.fallback_min}, {envelope.fallback_max}]")
    check_clean(nominal, live_params)
    assert ev["passed"], ev["failed"]


def test_yaw_runs_on_the_recorded_parameters_and_both_truth_sources(nominal, envelope, scenario, live_params):
    recorded = recorded_inputs()
    assert set(recorded) - set(live_params) == {"tau_held_yaw_nm"}, "the recorded fixture has keys the build lacks"
    for key, want in recorded.items():
        if key in live_params:
            assert live_params[key] == want, f"{key}: build {live_params[key]!r} differs from the recorded {want!r}"
    assert nominal.runs[0].plan.end == envelope.end
    if scenario == "yaw_fallback":
        d = float(envelope.header["disturbance_nm"])
        assert r32(d) == recorded["tau_held_yaw_nm"]
        for s in nominal.runs:
            assert s.overrides["l5_dist_yaw_nm"] == (run_l5.F32, repr(r32(d)))
            assert s.overrides["l5_dist_t0_us"] == s.overrides["l5_seg2_t_us"]
    for s in nominal.runs:
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        assert s.harness_overrides == {}
        for t in s.run.log["ticks"][:: s.plan.att_divisor]:
            assert struct_flags(t["imu"]) == 1 << GYRO_VALID_BIT, "the sample is not the truth gyro"


def test_alignment_is_exact(nominal, scenario, capsys):
    for s in nominal.runs:
        assert alignment_findings(s, s.plan.origin) == []
        changes = dshot_changes(s)
        assert changes and all(t % s.plan.divisor == 0 for t in changes)
    p = nominal.runs[0].plan
    _say(capsys, f"alignment {scenario}: origin a0 = {p.origin}, segment stamps {[x[1] for x in p.segments]} us")


# ---- negative controls: yaw release ---------------------------------------------------------------------------------

def planted_heading_back(env, tr, att_kp):
    """A trace like `tr` until the release whose heading then returns, exponentially at att_kp, to the heading where the yaw
    input started (the shortest way, wrap(psi_0 - psi_r)); the yaw rate is its derivative."""
    n_r = env.release
    psi0, psi_r = tr.psi[0], tr.psi[n_r]
    target = wrap(psi0 - psi_r)
    out = Trace(list(tr.t_us), list(tr.psi), list(tr.omega))
    for n in range(n_r, len(tr.psi)):
        rel = wrap(tr.psi[n] - psi_r)
        c = math.exp(-att_kp * (tr.t_us[n] - tr.t_us[n_r]) / US_PER_S)
        out.psi[n] = wrap(psi_r + target + (rel - target) * c)
        out.omega[n] = c * tr.omega[n] - att_kp * c * (rel - target)
    return out, target


def test_control_planted_heading_back_to_the_input_start_fails(release_flown, release_env, live_params, capsys):
    planted = {m: planted_heading_back(release_env, release_flown.traces[m], live_params["att_kp"]) for m in (1, 2)}
    target = planted[1][1]
    ev = evaluate("yaw_release", release_env, planted[1][0], planted[2][0], release_flown.rule_params)
    _say(capsys, f"planted heading-back target relative to the release {target:+.6f} rad",
         *report_lines("control planted", ev), f"    failed items {ev['failed']}")
    assert release_flown.evaluation["passed"], "the unplanted trace must pass (control of the control)"
    assert not ev["passed"] and "heading_release" in ev["failed"], ev["failed"]


def test_control_att_kp_zero_fails(kp_zero, release_env, live_params, capsys):
    ev = kp_zero.evaluation
    _say(capsys, *report_lines("control kp = 0", ev), f"    failed items {ev['failed']}")
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ("heading_lock" in ev["failed"] or "end_heading_lock_at_zero" in ev["failed"]), ev["failed"]


@pytest.mark.parametrize("off", SHIFTS)
def test_control_release_moved_one_execution_fails(release_flown, off):
    for s in release_flown.runs:
        assert alignment_findings(s, s.plan.origin) == []
        assert alignment_findings(s, s.plan.origin + off) != []


# ---- negative controls: yaw-lock fallback ---------------------------------------------------------------------------

def test_control_t_cross_at_the_run_duration_fails(t_cross_run, fallback_env, live_params, capsys):
    flown, seconds = t_cross_run
    nominal_rule = (live_params["att_yaw_t_cross"], live_params["att_yaw_alpha_min"])
    ev_built = evaluate("yaw_fallback", fallback_env, flown.traces[1], flown.traces[2], nominal_rule)
    _say(capsys, f"att_yaw_t_cross overridden to {r32(seconds)!r} s (the run duration)",
         *report_lines("control emitted-params", flown.evaluation), f"    failed items {flown.evaluation['failed']}",
         *report_lines("control build-params ", ev_built), f"    failed items {ev_built['failed']}")
    for s in flown.runs:
        assert s.harness_overrides == {"att_yaw_t_cross": ("f32", repr(r32(seconds)))}
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not flown.evaluation["passed"] and "fallback_locks" in flown.evaluation["failed"]
    assert not ev_built["passed"] and ("heading_lock" in ev_built["failed"] or "end_heading_lock_at_zero" in ev_built["failed"])


def planted_early_crossing(env, tr, n_fb):
    n_r = env.release
    half = max(1, (n_fb - n_r) // 2)
    out = Trace(list(tr.t_us), list(tr.psi), list(tr.omega))
    for n in range(n_r, n_fb + 1):
        out.omega[n] = tr.omega[n] * (1.0 - 2.0 * min(1.0, (n - n_r) / half))
    return out


def test_control_planted_early_crossing_fails(fallback_flown, fallback_env, capsys):
    k = fallback_flown.evaluation["rule"]
    assert fallback_flown.evaluation["passed"], "the unplanted trace must pass (control of the control)"
    planted = {m: planted_early_crossing(fallback_env, fallback_flown.traces[m], k[0]["n_fb"]) for m in (1, 2)}
    ev = evaluate("yaw_fallback", fallback_env, planted[1], planted[2], fallback_flown.rule_params)
    _say(capsys, *report_lines("control planted", ev), f"    failed items {ev['failed']}")
    assert not ev["passed"] and "no_crossing_before_fallback" in ev["failed"], ev["failed"]
