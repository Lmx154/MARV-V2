"""T4 yaw release and yaw-lock fallback of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4:
S and B on yaw rate, heading relative to the lock and tilt, predicates of decision 0029, loop-disabled control; decision 0006 F
"T4 yaw release (owner decisions 14 and 17)" and "T4 yaw-lock fallback (owner decision 18)"; D for the lock logic; decision
0031).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/yaw_release.yaml and yaw_fallback.yaml, one gz process per m of the m_sequence [2, 1]
(run_l5.run_sequence). The scripts are the T3 scripts (test_l5_scenario.py cross-checks them against the envelope header and
binds the release and end executions, so the deadlines).

Quantities, from the TRUTH record of each attitude execution (the float cast of the body state the firmware read, decision
0006 B): psi(n) = wrap(2 atan2(q.z, q.w)) of the canonical q (D's heading); the yaw rate w(n) = rotate(q, omega_frd).z, the
world-down component (D "Yaw rate for the lock"); the tilt alpha(n) = 2 atan2(|(q.x, q.y)|, |(q.w, q.z)|), the angle between
body z and down; sigma_r = sign w(n_r), omega_r = |w(n_r)|, t_r = the stamp of n_r.

Lock execution. The log does not carry it (the TRUTH record holds q and omega only). It is RECOMPUTED from the TRUTH
records by D's rule, exactly as D states it: the first braking execution n >= n_r where sigma_r w(n) <= 0 (the crossing) or
where both dt >= t_cross and dt alpha_min >= omega_r (the fallback, n_fb the first such execution), dt = float32(t - t_r) /
float32(1e6) from the logged stamps, the products in float32, t_cross and alpha_min the emitted parameters of the run (the
build's table and the harness overrides). The crossing wins when both hold at n_fb.

Predicates (decision 0029 and the roles of decision 0031, tools/sim/t4_rule.py scenario_checks, on the m = 1 run). Two
segments: the hold, oracle executions 1 .. a_r - 1 (the yaw stick on), and the release, a_r .. end (the deadline of each is
the scenario's horizon, H executions). Errors q - q_ref: the yaw rate against the stick times rate_max_yaw in the hold and 0
in the release; the tilt against 0; the heading relative to the lock against 0, from the member's own lock execution n_l (the
lock exists from there) to the end, in the release segment only. Delta is the largest |error| of the segment among the
quantities of the same unit (rad/s alone; the tilt and the heading, rad). s is the budget's t4_settle_fraction, E = |y_1 - y_2|
of the quantity (the heading of each run relative to its own lock). Roles, static from the scenario and the firmware logic:
the yaw rate is STEPPED in both segments (the stick, then its release), the tilt is HELD, the heading relative to the lock is
DRIVEN (its reference is set by the lock event inside the yaw response, so its error is 0 at n_l by construction and decision
0006 line 458 predicts 0.32 to 1.07 rad of swing). Every quantity of the hold's rad unit is HELD, so the hold's Delta for the
tilt is the largest rad Delta of the release, which has the DRIVEN heading (decision 0031).
  S  every quantity, at the segment's last execution |error| <= s Delta + E;
  B  the STEPPED and HELD quantities, at every execution |error(n)| <= |error(n_0)| + s Delta + E (E the largest over the
     segment); not applied to the release segment of yaw_fallback, which has a disturbance onset (the disturbance is on from
     the release stamp), nor to the DRIVEN heading (its excursion after the lock has no bound at T4; the T3 golden L5T3Envelope
     keeps the design bound).
The tilt cannot be falsified by the att_kp = 0 control (the vehicle stays level without the attitude loop); it is covered by the
known-answer cases in tests/regression/quad/L02/tools/test_t4_rule.py and the mixer identity L3MixerIdentity
(tests/regression/quad/L03/unit/mixer/mixer_test.cpp:147).
Items kept besides S and B: yaw release, the lock is by the crossing (sigma_r = +1, and before the fallback execution when
there is one); yaw fallback, no crossing before the fallback (sigma_r w > 0 at every execution from n_r up to and including
n_fb, T3's min_sigma_omega_to_fallback convention) and the lock is the fallback, n_l = n_fb. The recorded lock range and the
heading relative to the release are not checked (decision 0031).
Both: a clean run (complete trailer, a TRUTH record for every tick, no stale host step in the window) and every DShot within
[idle, 2047].

Negative control, one per scenario (harness only, 0003 item 9; the test passes only if the conjunction fails and S fails on the
heading relative to the lock): att_kp = 0 through sil_override, the loop disabled (decision 0031), no stale read. Plus the
planted early crossing of the fallback item.
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
T3_REFERENCE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference"
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
SCENARIOS = ("yaw_release", "yaw_fallback")
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID
US_PER_S = 10 ** 6
RATE_UNIT, ANGLE_UNIT = "rad/s", "rad"
r32 = run_l5.l4.r32


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


def tilt(q):
    w, x, y, z = q
    return 2.0 * math.atan2(math.hypot(x, y), math.hypot(w, z))


def world_down_rate(q, om):
    w, x, y, z = q
    return 2.0 * (x * z - w * y) * om[0] + 2.0 * (y * z + w * x) * om[1] + (1.0 - 2.0 * (x * x + y * y)) * om[2]


@dataclasses.dataclass
class Trace:
    t_us: list
    psi: list
    omega: list
    alpha: list


def trace_of(s, n_exec):
    p = s.plan
    out = Trace([], [], [], [])
    for n in range(n_exec):
        e = s.executions[p.run_execution(n)]
        t = e["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {p.run_execution(n)}"
        out.t_us.append(e["t_us"])
        out.psi.append(heading(t["q_wxyz"]))
        out.omega.append(world_down_rate(t["q_wxyz"], t["omega_frd"]))
        out.alpha.append(tilt(t["q_wxyz"]))
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


def evaluate(scenario, plan, tr1, tr2, rule_params, rate_max_yaw):
    """S, B and the kept lock items (module docstring) on the traces of m = 1 and m = 2; rule_params = (t_cross, alpha_min)."""
    (a_s, _, stick), (n_r, _, _) = plan.segments
    end = plan.end
    t_cross, alpha = rule_params
    k1, k2 = (lock_rule(t, n_r, t_cross, alpha, end) for t in (tr1, tr2))
    disturbed = plan.disturbance is not None
    out = {"S": {}, "B": {}, "items": {}, "rule": [k1, k2], "failed": []}

    def segment_of(first, last, ref_rate, lock):
        ns = range(first, last + 1)
        seg = {"omega": (RATE_UNIT, t4_rule.STEPPED, [tr1.omega[n] - ref_rate for n in ns]),
               "tilt": (ANGLE_UNIT, t4_rule.HELD, [tr1.alpha[n] for n in ns])}
        diff = {"omega": [abs(tr1.omega[n] - tr2.omega[n]) for n in ns],
                "tilt": [abs(tr1.alpha[n] - tr2.alpha[n]) for n in ns]}
        if lock is not None:
            hs = range(lock, last + 1)
            seg["heading"] = (ANGLE_UNIT, t4_rule.DRIVEN, [wrap(tr1.psi[n] - tr1.psi[k1["n_l"]]) for n in hs])
            diff["heading"] = [abs(wrap(tr1.psi[n] - tr1.psi[k1["n_l"]]) - wrap(tr2.psi[n] - tr2.psi[k2["n_l"]])) for n in hs]
        return seg, diff

    both = k1["n_l"] is not None and k2["n_l"] is not None
    segments, diffs = {}, {}
    segments["hold"], diffs["hold"] = segment_of(a_s, n_r - 1, stick[2] * rate_max_yaw, None)
    segments["release"], diffs["release"] = segment_of(n_r, end, 0.0, max(k1["n_l"], k2["n_l"]) if both else None)
    if not both:
        out["failed"].append("release: no lock execution in the run")
    try:
        checks = t4_rule.scenario_checks(
            segments, SETTLE_FRACTION, {g: {c: d[-1] for c, d in diffs[g].items()} for g in segments},
            {g: {c: max(d) for c, d in diffs[g].items()} for g in segments}, {"release"} if disturbed else frozenset())
        for g, c in checks.items():
            out["S"][g], out["B"][g] = c["S"], c["B"]
    except ValueError as err:
        out["failed"].append(f"no scale for the held tilt: {err}")

    if scenario == "yaw_release":
        in_time = all(k["n_fb"] is None or (k["n_l"] is not None and k["n_l"] < k["n_fb"]) for k in (k1, k2))
        out["items"]["lock_by_crossing"] = {
            "passed": both and in_time and all(k["by"] == "crossing" and k["sigma"] == 1 for k in (k1, k2)),
            "lock": [k1["n_l"], k2["n_l"]], "by": [k1["by"], k2["by"]], "fb": [k1["n_fb"], k2["n_fb"]]}
    else:
        nf1, nf2 = k1["n_fb"], k2["n_fb"]
        ok = nf1 is not None and nf2 is not None
        out["items"]["no_crossing_before_fallback"] = {
            "passed": ok and all(min(k["sigma"] * t.omega[n] for n in range(n_r, k["n_fb"] + 1)) > 0
                                 for t, k in ((tr1, k1), (tr2, k2))),
            "min_sigma_omega": [min(k["sigma"] * t.omega[n] for n in range(n_r, (k["n_fb"] if ok else end) + 1))
                                for t, k in ((tr1, k1), (tr2, k2))], "n_fb": [nf1, nf2]}
        out["items"]["fallback_locks"] = {
            "passed": ok and all(k["by"] == "fallback" and k["n_l"] == k["n_fb"] and k["sigma"] == 1 for k in (k1, k2)),
            "lock": [k1["n_l"], k2["n_l"]], "by": [k1["by"], k2["by"]], "fb": [nf1, nf2]}
    out["failed"] += [f"{part} {seg} {c.name}" for part in ("S", "B") for seg, cs in out[part].items() for c in cs if not c.ok]
    out["failed"] += [k for k, v in out["items"].items() if not v["passed"]]
    out["passed"] = not out["failed"]
    return out


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


def fly(tmp_path_factory, name, scenario, live_params, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"{scenario}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    by_m = {s.run.m: s for s in runs}
    plan = by_m[1].plan
    traces = {m: trace_of(by_m[m], plan.end + 1) for m in (1, 2)}
    e = emitted_params(live_params, by_m[1].harness_overrides)
    rule_params = (e["att_yaw_t_cross"], e["att_yaw_alpha_min"])
    ev = evaluate(scenario, plan, traces[1], traces[2], rule_params, live_params["rate_max_yaw"])
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, {"passed": ev["passed"], "failed": ev["failed"], "gz_wall_s": wall, "lock_rule": ev["rule"],
                               "rule_params (t_cross, alpha_min)": list(rule_params)}, path)
    return Flown(runs, traces, wall, rule_params, ev)


@pytest.fixture(scope="module", params=SCENARIOS)
def scenario(request):
    return request.param


@pytest.fixture(scope="module")
def nominal(tmp_path_factory, scenario, live_params):
    return fly(tmp_path_factory, scenario, scenario, live_params)


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, scenario, live_params):
    return fly(tmp_path_factory, f"{scenario}_kp_zero", scenario, live_params, {"att_kp": (run_l5.F32, "0.0")})


@pytest.fixture(scope="module")
def fallback_flown(tmp_path_factory, live_params):
    return fly(tmp_path_factory, "yaw_fallback", "yaw_fallback", live_params)


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


def report_lines(tag, ev):
    def part(checks):
        return ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}" for c in checks)
    lines = [f"{tag} {'PASS' if ev['passed'] else 'FAIL'}"]
    lines += [f"{tag} S {seg}: [{part(cs)}]" for seg, cs in ev["S"].items()]
    lines += [f"{tag} B {seg}: [{part(cs)}]" for seg, cs in ev["B"].items()]
    lines += [f"{tag} item {k}: {'PASS' if v['passed'] else 'FAIL'} " + ", ".join(f"{a} {b}" for a, b in v.items() if a != "passed")
              for k, v in ev["items"].items()]
    return lines + [f"{tag} lock rule m=1 {ev['rule'][0]}", f"{tag} lock rule m=2 {ev['rule'][1]}"]


def struct_flags(imu):
    return run_l5.l4.IMU.unpack(imu)[7]


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

def test_yaw_settles_and_stays_bounded(nominal, scenario, live_params, capsys):
    ev = nominal.evaluation
    _say(capsys, *report_lines(scenario, ev), f"    gz wall {nominal.wall_s:.1f} s; rule params (t_cross, alpha_min) "
         f"{nominal.rule_params}")
    check_clean(nominal, live_params)
    assert ev["passed"], ev["failed"]


def test_yaw_runs_on_the_recorded_parameters_and_both_truth_sources(nominal, scenario, live_params):
    recorded = recorded_inputs()
    assert set(recorded) - set(live_params) == {"tau_held_yaw_nm"}, "the recorded fixture has keys the build lacks"
    for key, want in recorded.items():
        if key in live_params:
            assert live_params[key] == want, f"{key}: build {live_params[key]!r} differs from the recorded {want!r}"
    if scenario == "yaw_fallback":
        d = nominal.runs[0].plan.disturbance["yaw_nm"]
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


# ---- negative controls ----------------------------------------------------------------------------------------------

def test_control_att_kp_zero_fails_the_predicates(kp_zero, scenario, capsys):
    ev = kp_zero.evaluation
    _say(capsys, *report_lines(f"{scenario} control kp = 0", ev), f"    failed {ev['failed']}")
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the predicates, not on a stale read"
    assert not ev["passed"] and "S release heading" in ev["failed"], ev["failed"]


def planted_early_crossing(plan, tr, n_fb):
    n_r = plan.segments[1][0]
    half = max(1, (n_fb - n_r) // 2)
    out = Trace(list(tr.t_us), list(tr.psi), list(tr.omega), list(tr.alpha))
    for n in range(n_r, n_fb + 1):
        out.omega[n] = tr.omega[n] * (1.0 - 2.0 * min(1.0, (n - n_r) / half))
    return out


def test_control_planted_early_crossing_fails(fallback_flown, live_params, capsys):
    k = fallback_flown.evaluation["rule"]
    plan = fallback_flown.runs[0].plan
    assert fallback_flown.evaluation["passed"], "the unplanted trace must pass (control of the control)"
    planted = {m: planted_early_crossing(plan, fallback_flown.traces[m], k[0]["n_fb"]) for m in (1, 2)}
    ev = evaluate("yaw_fallback", plan, planted[1], planted[2], fallback_flown.rule_params, live_params["rate_max_yaw"])
    _say(capsys, *report_lines("control planted", ev), f"    failed {ev['failed']}")
    assert not ev["passed"] and "no_crossing_before_fallback" in ev["failed"], ev["failed"]
