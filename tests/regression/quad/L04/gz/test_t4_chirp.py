"""T4 chirp margins of the L4 rate loop in Gazebo on truth gyro (quad spec 4 L4 pass bar, T4: "chirp injection measures
margins that meet QF-3"; QF-3: phase margin >= PM_min; decision 0005 "T4", "Gain rule").

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l4 build of libmarv_gz_lockstep.so is
absent (`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`); the gz CI step fails on a skip. Every run is
truth-fed, perfect-model (tools/sim/run_l4.py): not a validation run.

Runs, per axis, scenarios/quad/L04/chirp_<axis>.yaml (tools/sim/run_l4.py run_chirp; one gz process each):
  (m = 1, A, D), (m = 2, A, D), (m = 1, A/2, D), (m = 1, A, D/2), and the negative control (m = 1, A, D, gains x k).
The band, the amplitude A = tau_held / max|S|, the stamps and the hover thrust are the runner's derived plan
(run_l4.chirp_design, plan_chirp); the torque chirp is added to the axis's torque request before allocate(), the
setpoint is 0 on every axis.

Measurement. PM(run) = run_l4.chirp_margin: indirect closed-loop identification from the log alone (run_l4 section
"T4 chirp margins": G_m = Y/D, L = C G_m / (1 - C G_m), crossover |L| = 1 inside the band, PM = pi + arg L), with C the
gains the run flew. E_H = |PM(m = 1) - PM(m = 2)| (decision 0003 E), U_A = |PM(A) - PM(A/2)|.

Float input term (lead decision, 2026-09-30). U_d bounds the PM error of recomputing d in binary64 while the composition
computes it in float32: run_l4.float_input_term, the stated rule of run_l4's section "the chirp's float32 input term",
on the run's own samples and crossover (the per-sample difference to a float32 emulation of the composition's chirp,
its sum over the window against |D(w_c)|, propagated through L = C G_m / (1 - C G_m) to the phase and the crossover).

Predicate per axis. PASS iff no host step read in any window is stale (0003 item 11) and, with U = E_H + U_A + U_d,
  PM(m = 1, A) - U >= PM_min   and
  min_box PM_design - U <= PM(m = 1, A) <= max_box PM_design + U,
PM_design at nominal and the four tau x J corners from tools/card/rate.py design() (the build's gains are checked equal
to the design's), PM_min the budget's, U_d that of the (m = 1, A) run.

Duration (core 7.5). The scenario's duration_s is duration_start_s 2^j (the schema); the test re-checks the last
doubling: |PM(D) - PM(D/2)| < E_H + U_A, both at D.

Prefilter. With sp = 0 the reference r is seeded from the first gyro sample and stays there; the test requires that
sample to be exactly 0 on every axis, so r is exactly 0 and the loop is u = -C y (the identification's model).

Negative control, harness only (0003 item 9), the metric control of core 7.2: rate_kp_<axis> and rate_ki_<axis> x k via
sil_override, k the smallest integer >= 2 for which the design-model PM at nominal tau and J (tools/card/rate.py
loop_crossover on the float32 gains) is below PM_min - (E_H + U_A) of the axis's own nominal runs. Its measured PM (C
with the control's gains) must fail the predicate evaluated with those E_H and U_A and its own U_d.

Cross-check (reported, not asserted): the measured nominal PM against the design nominal PM.
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l4_scenario as l4s  # noqa: E402
import rate  # noqa: E402
import run_l4  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L04"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
AXES = ("roll", "pitch", "yaw")
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid
HALF = 0.5  # the amplitude of the U_A run, A/2
FIRST_CONTROL_K = 2

needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l4 build of libmarv_gz_lockstep.so is absent")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


def deg(x):
    return None if x is None else math.degrees(x)


def verdict(pm, e_h, u_a, u_d, design_pms, pm_min):
    """The predicate of the module docstring on one measured PM (rad); pm None (no crossover in the band) fails."""
    u = e_h + u_a + u_d
    lo, hi = min(design_pms) - u, max(design_pms) + u
    if pm is None:
        return {"passed": False, "reason": "no unique crossover in the band", "E_H + U_A + U_d (deg)": deg(u)}
    slack = pm - u - pm_min
    in_range = lo <= pm <= hi
    return {"passed": slack >= 0 and in_range, "PM (deg)": deg(pm), "E_H (deg)": deg(e_h), "U_A (deg)": deg(u_a),
            "U_d (deg)": deg(u_d), "PM - E_H - U_A - U_d - PM_min (deg)": deg(slack), "range (deg)": [deg(lo), deg(hi)],
            "in range": in_range}


def control_k(design, axis, kp, ki, limit):
    """The smallest integer k >= 2 whose float32 gains kp k, ki k give a nominal design-model PM below `limit`
    (rad), with that PM and crossover; tools/card/rate.py loop_crossover, nominal tau and J."""
    T, tau, j = design["T"], design["tau"], design["axes"][axis]["J"]
    pc = rate.plant_constants(tau, T)
    k = FIRST_CONTROL_K
    while True:
        kpk, kik = run_l4.r32(k * kp), run_l4.r32(k * ki)
        theta, pm, why = rate.loop_crossover(kpk / j, kik / j, T, pc)
        if theta is None:
            raise AssertionError(f"gains x {k}: the design model has no unique crossover ({why}) before PM < limit")
        if pm < limit:
            return k, kpk, kik, pm, theta / T
        k += 1


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Chirp:
    axis: str
    runs: dict  # name -> run_l4.ChirpRun
    margins: dict  # name -> identify() result
    e_h: float
    u_a: float
    convergence: float
    evaluation: dict
    report_path: str
    wall_s: float


RUNS = (("m1", 1, 1.0, False), ("m2", 2, 1.0, False), ("half_amplitude", 1, HALF, False),
        ("half_duration", 1, 1.0, True))


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


@pytest.fixture(scope="module")
def chirp(tmp_path_factory, axis):
    out = tmp_path_factory.mktemp(f"chirp_{axis}")
    scenario = SCEN / f"chirp_{axis}.yaml"
    runs = {name: run_l4.run_chirp(CARD, scenario, m, out, PLUGIN_DIR, amp_scale=scale, halved=halved)
            for name, m, scale, halved in RUNS}
    margins = {name: run_l4.chirp_margin(s) for name, s in runs.items()}
    pms = {name: r["pm"] for name, r in margins.items()}
    assert all(v is not None for v in pms.values()), {n: r["reason"] for n, r in margins.items()}
    e_h, u_a = abs(pms["m1"] - pms["m2"]), abs(pms["m1"] - pms["half_amplitude"])
    conv = abs(pms["m1"] - pms["half_duration"])
    design = runs["m1"].plan.design
    float_term = run_l4.float_input_term(runs["m1"], margins["m1"])
    ev = verdict(pms["m1"], e_h, u_a, float_term["U_d"], list(design["design_pm"].values()), design["pm_min"])
    ev.update({
        "U_d terms": float_term,
        "PM per run (deg)": {n: deg(v) for n, v in pms.items()},
        "crossover per run (rad/s)": {n: r["crossover"] for n, r in margins.items()},
        "|PM(D) - PM(D/2)| (deg)": deg(conv), "E_H + U_A (deg)": deg(e_h + u_a),
        "design PM (deg)": {n: deg(v) for n, v in design["design_pm"].items()},
        "measured - design nominal (deg)": deg(pms["m1"] - design["design_pm"]["nominal"]),
        "stale reads in the windows": {n: len(s.window_stale) for n, s in runs.items()},
        "executions with a motor at a DShot end (upper bound of saturation-flagged executions)":
            {n: s.dshot_end_executions for n, s in runs.items()},
    })
    wall = sum(s.run.wall_s for s in runs.values())
    path = str(out / f"chirp_{axis}_{run_l4.NAME_LABEL}_sequence.report.txt")
    run_l4.write_t4_report(list(runs.values()), "t4 chirp", dict(ev, gz_wall_s=wall), path)
    return Chirp(axis, runs, margins, e_h, u_a, conv, ev, path, wall)


@pytest.fixture(scope="module")
def control(tmp_path_factory, axis, chirp):
    s1 = chirp.runs["m1"]
    design = s1.plan.design
    k, kpk, kik, pm_design, wc_design = control_k(design, axis, s1.plan.kp, s1.plan.ki,
                                                  design["pm_min"] - (chirp.e_h + chirp.u_a))
    over = {f"rate_kp_{axis}": (run_l4.F32, repr(kpk)), f"rate_ki_{axis}": (run_l4.F32, repr(kik))}
    out = tmp_path_factory.mktemp(f"chirp_{axis}_control")
    s = run_l4.run_chirp(CARD, SCEN / f"chirp_{axis}.yaml", 1, out, PLUGIN_DIR, overrides=over)
    r = run_l4.chirp_margin(s)
    assert r["pm"] is not None, r["reason"]
    float_term = run_l4.float_input_term(s, r)
    ev = verdict(r["pm"], chirp.e_h, chirp.u_a, float_term["U_d"], list(design["design_pm"].values()), design["pm_min"])
    ev.update({"U_d terms": float_term, "k": k, "control gains kp, ki": [kpk, kik], "design PM at nominal (deg)": deg(pm_design),
               "design crossover (rad/s)": wc_design, "measured crossover (rad/s)": r["crossover"],
               "reason": r["reason"], "stale reads in the window": len(s.window_stale),
               "executions with a motor at a DShot end": s.dshot_end_executions})
    run_l4.write_t4_report([s], "t4 chirp control", dict(ev, gz_wall_s=s.run.wall_s))
    return {"run": s, "margin": r, "evaluation": ev, "k": k, "gains": (kpk, kik)}


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@needs_gz
def test_chirp_margin_meets_qf3(chirp, axis, capsys):
    ev = chirp.evaluation
    _say(capsys, f"{axis:<5} chirp {'PASS' if ev['passed'] else 'FAIL'}  PM {ev['PM (deg)']:.6f} deg, "
                 f"E_H {ev['E_H (deg)']:.3e}, U_A {ev['U_A (deg)']:.6f}, U_d {ev['U_d (deg)']:.6f}, slack "
                 f"{ev['PM - E_H - U_A - U_d - PM_min (deg)']:.6f} deg, range [{ev['range (deg)'][0]:.6f}, "
                 f"{ev['range (deg)'][1]:.6f}], measured - design nominal "
                 f"{ev['measured - design nominal (deg)']:+.6f} deg, gz wall {chirp.wall_s:.2f} s",
         f"    runs (deg): {ev['PM per run (deg)']}", f"    report: {chirp.report_path}")
    for s in chirp.runs.values():
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
    assert ev["passed"], ev


@needs_gz
def test_duration_is_converged(chirp, axis):
    doc = l4s.load_chirp(SCEN / f"chirp_{axis}.yaml")
    c = l4s.chirp_values(doc)["chirp"]
    assert chirp.runs["m1"].plan.duration_s == c["duration_s"]
    assert chirp.runs["half_duration"].plan.duration_s == c["duration_s"] / 2 >= c["duration_start_s"]
    assert chirp.convergence < chirp.e_h + chirp.u_a, chirp.evaluation


@needs_gz
def test_chirp_runs_on_the_live_design_truth_gyro_and_a_zero_reference(chirp, axis):
    for name, s in chirp.runs.items():
        p = s.plan
        assert p.axis == axis and s.harness_overrides == {}
        assert (p.kp, p.ki) == run_l4.flown_gains(s)
        assert s.flags == {1 << GYRO_VALID_BIT}, "the sample is not the truth gyro"
        assert s.gyro_first == (0.0, 0.0, 0.0), "the prefilter seed is not zero: the reference is not exactly 0"
        ax = p.design["axes"][axis]
        assert p.amp_nm == run_l4.r32(run_l4.r32(ax["amplitude_nm"]) * p.amp_scale)
        assert (p.w_lo, p.w_hi) == tuple(run_l4.r32(w) for w in p.design["band"])
        assert s.stamps[0] == p.t0_us and p.stamp_us(p.start_execution - 1) < p.t0_us
        assert any(s.d) and s.d[-1] == 0.0, "the window must cover the chirp and its end"
        assert s.overrides["l4_chirp_amp_nm"] == (run_l4.F32, repr(p.amp_nm)), name


# ---- negative control -----------------------------------------------------------------------------------------------

@needs_gz
def test_control_gains_times_k_fails_the_predicate(control, chirp, axis, capsys):
    ev = control["evaluation"]
    _say(capsys, f"{axis:<5} control gains x {control['k']}: measured PM {ev.get('PM (deg)')} deg, U_d "
                 f"{ev.get('U_d (deg)')} deg (design "
                 f"{ev['design PM at nominal (deg)']:.6f}), crossover {ev['measured crossover (rad/s)']} rad/s, "
                 f"{'PASS' if ev['passed'] else 'FAIL'} (must fail), gz wall {control['run'].run.wall_s:.2f} s")
    s = control["run"]
    assert s.harness_overrides == {f"rate_kp_{axis}": (run_l4.F32, repr(control["gains"][0])),
                                   f"rate_ki_{axis}": (run_l4.F32, repr(control["gains"][1]))}
    assert run_l4.flown_gains(s) == control["gains"]
    assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
    assert s.window_stale == [], "the control must fail on the margin, not on a stale read"
    assert control["margin"]["pm"] is not None, "the control must fail on a measured margin, not on a missing crossover"
    assert not ev["passed"], ev
