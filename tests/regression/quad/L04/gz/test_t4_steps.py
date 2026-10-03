"""T4 per-axis steps of the L4 rate loop in Gazebo on truth gyro (quad spec 4 L4 pass bar, T4 item 1: "in Gazebo, on truth
gyro, per-axis steps meet QF-2"; decision 0005 "QF-2", "T4", "T4 envelope widening", owner decisions 7 and 9).

Skipped only when `gz sim --force-version 8` does not report 8.x or the host-gz-l4 build of libmarv_gz_lockstep.so is
absent (`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`); the gz CI step fails on a skip. Every run is
truth-fed, perfect-model (tools/sim/run_l4.py): not a validation run.

Runs. scenarios/quad/L04/step_<axis>.yaml, one gz process per m of its m_sequence [2, 1] (tools/sim/run_l4.py
run_sequence). y_m(n) is the gyro sample the firmware received at oracle execution n (the logged TICK IMU bytes are the
sample passed to the SIL, decision 0005 "Truth gyro"), at execution k = k0 + n - 2 of the run (run_l4 plan: k0 is the
step's first execution, n = 1 the last one before it, which is the oracle's seed execution).

Envelope. Computed here from the LIVE parameters of the plugin build: the T3 oracle's inputs-refresh path
(rate_t3_oracle.refresh_inputs on the build's param_defaults.cpp), then the oracle's own functions, exactly as its main()
combines them: the band envelope lo(n), hi(n) over the 17 x 17 tau x J grid of the design-model step response with the
prefilter; its last-halving change H (9 x 9 against 17 x 17); the float rounding tolerance TOL = sum(l1 x rho). The
committed T3 envelope (a fixed-input golden) is not used.

Predicate per axis (decision 0005 "T4 envelope widening"). E(n) = |y_1(n) - y_2(n)|, F = TOL + H. PASS iff no host
step read in the window is stale (0003 item 11: the raw gz read bitwise equal to the previous step's) and at every
execution n = 1..N:  lo(n) - (E(n) + F) <= y_1(n) <= hi(n) + (E(n) + F). Altitude drift is allowed (Luis, owner decision
9) and not checked; marv_plant v0 has no drag and no position-dependent torque.

Time resolution of the predicate. With F = TOL + H (about 0.041 rad/s, 98 % of it the envelope's grid-convergence
term H), the smallest shift of the envelope along the executions that the predicate detects is 48 executions (15 ms)
on every axis; the design-model response changes by at most 0.0134 rad/s per execution, below F. The predicate is
therefore not an alignment check; the exact alignment check below is.

Alignment (exact). The oracle's execution n is the run's execution k0 + n - 2: the setpoint segment's stamp lies after
execution k0 - 1's stamp and at execution k0's, every window dt (stamp difference, us) equals the oracle's dt(n), and
every DShot change of the run falls on a rate-loop tick (D k). The runner's N equals the oracle's executions(axis).

Negative controls, harness only (0003 item 9). Each test passes only if the check fails:
  (a) kp = ki = 0 on the stepped axis through sil_override: the predicate fails, with no stale read. This is the metric
      test's negative control of core 7.2;
  (b) alignment: the exact alignment check with k0 one execution early or late fails. This guards the stamp alignment
      exactly (the segment stamp and every window dt, which alternates 312 / 313 us).
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tests" / "regression" / "quad" / "L04" / "t3" / "reference"))
import rate_t3_oracle as oracle  # noqa: E402
import run_l4  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L04"
PLUGIN_DIR = run_l4.DEFAULT_PLUGIN_DIR
AXES = ("roll", "pitch", "yaw")
HALVED = 2 * (oracle.GRID - 1) + 1  # the oracle's halved grid, 17 x 17
PHASES = (2, 3)  # the oracle's two dt phases of an injection (its main())
SHIFTS = (-1, 1)
GYRO_VALID_BIT = 6  # marv_sil.h MARV_IMU_GYRO_VALID: the enum position of GyroValid

needs_gz = pytest.mark.skipif(not (run_scenario.plugin_present(PLUGIN_DIR) and run_scenario.gz8_available()),
                              reason="gz-sim 8 or the host-gz-l4 build of libmarv_gz_lockstep.so is absent")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


# ---- the envelope from the live parameters ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Envelope:
    axis: str
    su: object
    n_exec: int
    lo: list
    hi: list
    nominal: list
    tol: float
    halving: float

    @property
    def F(self):
        return self.tol + self.halving


def oracle_envelope(p, axis):
    """The oracle's main() terms for one axis on the inputs p: envelope, TOL and the halving change H."""
    assert p[f"rate_d_filter_tau_{axis}"] > 0.0, "the stage (c) law: kd from the parameters, D through T_f > 0 (0014)"
    su = oracle.Setup(p)
    n = su.executions(axis)
    sp = p[f"rate_max_{axis}"]
    y, rho = oracle.closed_loop(su, axis, p[oracle.INERTIA[axis]], p["motor_tau"], n, sp, want_rho=True)
    l1 = {node: max(oracle.l1_norm(su, axis, node, k, n) for k in PHASES) for node in oracle.NODES}
    tol = sum(l1[node] * rho[node] for node in oracle.NODES)
    lo_c, hi_c = oracle.band_envelope(su, axis, n, sp, oracle.GRID)
    lo, hi = oracle.band_envelope(su, axis, n, sp, HALVED)
    change = max(max(abs(a - b) for a, b in zip(lo_c, lo)), max(abs(a - b) for a, b in zip(hi_c, hi)))
    return Envelope(axis, su, n, lo, hi, y, tol, change)


# ---- the predicate --------------------------------------------------------------------------------------------------

def samples(s, n_exec):
    p = s.plan
    return [s.executions[p.execution_of(n)]["gyro"][p.axis_index] for n in range(1, n_exec + 1)]


def evaluate(runs, env, shift=0):
    """The predicate of the module docstring on a [m=2, m=1] sequence; `shift` compares y(n) with the envelope at
    n + shift (the alignment control), over the n whose shifted index is in 1..N."""
    by_m = {s.run.m: s for s in runs}
    y1, y2 = samples(by_m[1], env.n_exec), samples(by_m[2], env.n_exec)
    worst = None
    max_out, max_e, violations = -math.inf, 0.0, 0
    for n in range(1, env.n_exec + 1):
        j = n + shift
        if not 1 <= j <= env.n_exec:
            continue
        e = abs(y1[n - 1] - y2[n - 1])
        out = max(env.lo[j - 1] - y1[n - 1], y1[n - 1] - env.hi[j - 1])
        slack = out - (e + env.F)
        violations += slack > 0
        max_out, max_e = max(max_out, out), max(max_e, e)
        if worst is None or slack > worst["slack"]:
            worst = {"n": n, "slack": slack, "outside": out, "E": e, "y": y1[n - 1], "lo": env.lo[j - 1],
                     "hi": env.hi[j - 1]}
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {
        "passed": violations == 0 and not any(stale.values()), "shift": shift, "violations": violations,
        "stale_reads_in_window": stale, "max_distance_outside_envelope": max_out, "max_E": max_e, "F": env.F,
        "TOL": env.tol, "H": env.halving, "N": env.n_exec, "worst": worst,
    }


def alignment_findings(s, env, k0):
    """Why the mapping k = k0 + n - 2 does not put oracle execution n on the run's execution k; empty when it does."""
    ex, p = s.executions, s.plan
    seg = int(s.overrides["l4_seg1_t_us"][1])
    out = []
    if not (ex[k0 - 1]["t_us"] < seg <= ex[k0]["t_us"]):
        out.append(f"segment stamp {seg} is not in (t(k0 - 1), t(k0)] = ({ex[k0 - 1]['t_us']}, {ex[k0]['t_us']}]")
    if k0 + env.n_exec - 2 >= len(ex):
        out.append(f"the window's last execution {k0 + env.n_exec - 2} is beyond the run's {len(ex) - 1}")
    bad_dt = [n for n in range(2, min(env.n_exec, len(ex) - k0 + 1) + 1)
              if ex[k0 + n - 2]["t_us"] - ex[k0 + n - 3]["t_us"] != env.su.stamp_us(n) - env.su.stamp_us(n - 1)]
    if bad_dt:
        out.append(f"{len(bad_dt)} window dt differ from the oracle's (first at n = {bad_dt[0]})")
    if any(x["tick"] != p.tick_of(x["k"]) for x in ex):
        out.append("an execution's logged tick is not D k")
    return out


def dshot_changes(s):
    ticks = s.run.log["ticks"]
    return [t["tick"] for a, t in zip(ticks, ticks[1:]) if a["dshot"] != t["dshot"]]


def verdict_line(axis, label, ev, wall):
    w = ev["worst"]
    return (f"{axis:<5} {label:<22} {'PASS' if ev['passed'] else 'FAIL'}  max distance outside envelope "
            f"{ev['max_distance_outside_envelope']:+.6e} rad/s, max E {ev['max_E']:.3e}, F {ev['F']:.6e} "
            f"(TOL {ev['TOL']:.3e} + H {ev['H']:.6e}), worst n {w['n']} (outside {w['outside']:+.3e}, slack "
            f"{w['slack']:+.6e}), violations {ev['violations']}, stale {ev['stale_reads_in_window']}, N {ev['N']}, "
            f"gz wall {wall:.2f} s")


# ---- fixtures -------------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Sequence:
    runs: list
    report_path: str
    wall_s: float
    evaluation: dict


@pytest.fixture(scope="module")
def live(tmp_path_factory):
    """The oracle's inputs from the plugin build's parameter table, through the oracle's refresh path."""
    _, defaults = run_l4.build_parameters(PLUGIN_DIR)
    path = tmp_path_factory.mktemp("t3_inputs") / "rate_t3_inputs.txt"
    oracle.refresh_inputs(str(defaults), str(path))
    return oracle.read_inputs(str(path)), run_l4.read_param_defaults(defaults)


@pytest.fixture(scope="module", params=AXES)
def axis(request):
    return request.param


@pytest.fixture(scope="module")
def envelope(live, axis):
    return oracle_envelope(live[0], axis)


def fly(tmp_path_factory, name, axis, env, overrides=None):
    runs, path = run_l4.run_sequence(CARD, SCEN / f"step_{axis}.yaml", tmp_path_factory.mktemp(name), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, env)
    wall = sum(s.run.wall_s for s in runs)
    run_l4.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def step(tmp_path_factory, axis, envelope):
    return fly(tmp_path_factory, f"step_{axis}", axis, envelope)


@pytest.fixture(scope="module")
def gains_zero(tmp_path_factory, axis, envelope):
    zero = {f"rate_kp_{axis}": (run_l4.F32, "0.0"), f"rate_ki_{axis}": (run_l4.F32, "0.0")}
    return fly(tmp_path_factory, f"step_{axis}_gains_zero", axis, envelope, zero)


# ---- the pass bar ---------------------------------------------------------------------------------------------------

@needs_gz
def test_step_meets_qf2_envelope(step, envelope, axis, capsys):
    ev = step.evaluation
    _say(capsys, verdict_line(axis, "step", ev, step.wall_s), f"    report: {step.report_path}",
         *[f"    m = {s.run.m}: run stale report {s.run.stale}; window host steps {s.window_steps.start}.."
           f"{s.window_steps.stop - 1}, stale in window {len(s.window_stale)}" for s in step.runs])
    assert [s.run.m for s in step.runs] == [2, 1]
    for s in step.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
    assert ev["passed"], ev


@needs_gz
def test_step_runs_on_the_live_parameters_and_truth_gyro(step, envelope, live, axis):
    p_oracle, p_build = live
    for key in oracle.INPUT_KEYS:
        assert p_build[key] == p_oracle[key], key
    for s in step.runs:
        p = s.plan
        assert p.n_exec == envelope.su.executions(axis) == envelope.n_exec
        assert p.divisor == p_oracle["rate_loop_divisor"] and p.tau_ref_s == p_oracle[f"rate_tau_ref_{axis}"]
        assert p.rate_rad_s == p_oracle[f"rate_max_{axis}"]
        for k in p.window:
            assert s.executions[k]["flags"] == 1 << GYRO_VALID_BIT, "the sample is not the truth gyro"


@needs_gz
def test_alignment_is_exact(step, envelope, capsys):
    for s in step.runs:
        k0 = s.plan.step_execution
        assert alignment_findings(s, envelope, k0) == []
        changes = dshot_changes(s)
        assert changes, "the DShot command never changed: the check below would be vacuous"
        assert all(t % s.plan.divisor == 0 for t in changes), "a DShot change off a rate-loop tick"
    _say(capsys, f"alignment {envelope.axis}: k0 = {step.runs[0].plan.step_execution}, segment stamp "
                 f"{step.runs[0].overrides['l4_seg1_t_us'][1]} us, {len(dshot_changes(step.runs[1]))} DShot changes "
                 "(m = 1), all on rate-loop ticks")


# ---- negative controls ----------------------------------------------------------------------------------------------

@needs_gz
def test_control_gains_zero_fails_the_predicate(gains_zero, axis, capsys):
    ev = gains_zero.evaluation
    _say(capsys, verdict_line(axis, "control kp = ki = 0", ev, gains_zero.wall_s))
    for s in gains_zero.runs:
        assert s.harness_overrides == {f"rate_kp_{axis}": ("f32", "0.0"), f"rate_ki_{axis}": ("f32", "0.0")}
        assert f'<sil_override param="rate_kp_{axis}" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the envelope, not on a stale read"
    assert not ev["passed"] and ev["violations"] > 0, ev


@needs_gz
@pytest.mark.parametrize("off", SHIFTS)
def test_control_alignment_off_by_one_execution_fails(step, envelope, off):
    for s in step.runs:
        assert alignment_findings(s, envelope, s.plan.step_execution + off) != []
