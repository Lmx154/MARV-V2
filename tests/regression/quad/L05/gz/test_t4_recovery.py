"""T4 large-angle recovery of the L5 attitude loop in Gazebo on truth gyro and truth attitude (quad spec 4 L5 T4: R1, R2 and R1X
settle by the deadline (S) and stay bounded (B) on the tilt alpha, |omega| and the heading, predicates of decision 0029,
loop-disabled control; decision 0006 F "T4 large-angle recovery"; owner decisions 5, 15 and 22; decision 0031).

Skipped only as tests/regression/quad/L05/gz/conftest.py says; the gz CI step fails on a skip. Every run is truth-fed,
perfect-model (tools/sim/run_l5.py): not a validation run.

Runs. scenarios/quad/L05/recover_inverted.yaml (R1: a roll of pi - delta from level, at rest, delta = 0.01 rad, owner decision 22:
far above the rounding noise in (w, z) of the plant's first step, so the model and gz take the same branch of the tilt-yaw split),
recover_tumble.yaml (R2: inverted, body rates (rate_max_roll, rate_max_pitch, rate_max_yaw)) and recover_inverted_exact.yaml (R1X:
exactly [0, 1, 0, 0], where the tilt axis is chosen by rounding noise, so only the axis-invariant alpha and |omega| are
checked, not the heading); R1 and R1X with the rotors at the card's hover speed (owner decision 23; rotor_speed_rad_s: hover), R2
at its steady tumble (decision 0015; rotor_speed_rad_s: steady_tumble); one gz process per m of [2, 1] (tools/sim/run_l5.py
run_sequence). The setpoint is level, locked at angle mode's initial heading; no sticks. The run starts at the scenario's
initial state: the firmware's attitude execution n is the run's execution n.

Rate origin (harness fact, 0003 item 11; tests/regression/quad/L05/results/recovery_cause/README.md finding 1). The plugin reads the
body state and runs the SIL for host step 0 before it sets the scenario's initial rates, so the TRUTH record and the gyro of execution
0 read omega = 0 while the body moves from its initial rates. |omega| therefore starts at execution 1; alpha and the heading
start at execution 0. This is the plugin's documented timing, not a tolerance term.

Predicates (decision 0029 and the roles of decision 0031, tools/sim/t4_rule.py scenario_checks, on the m = 1 run). One segment,
executions 0 (1 for |omega|) .. the scenario's last execution (the deadline: ten attitude time constants 1/att_kp doubled once,
scenario end_attitude_execution). Quantities and their errors against level and rest: alpha, the angle between body z and NED
down (rad); the heading error, |err_z| of the body-frame rotation vector of the attitude error (rad; not for R1X); |omega|, the
norm of the body rates (rad/s). Delta is the largest error of the segment among the quantities of the same unit; s is the
budget's t4_settle_fraction; E = |y_1 - y_2|. Roles, static from the scenario file:
  R1   alpha STEPPED (initial offset pi - delta), heading HELD (the split puts none of the offset in it), |omega| DRIVEN (at
       rest at the start, set moving by the recovery of the stepped alpha);
  R1X  alpha STEPPED, |omega| DRIVEN;
  R2   alpha and |omega| STEPPED (the initial rates are nonzero), heading DRIVEN (it starts at 0 and the rate z0 sets it moving).
  S  every quantity, at the last execution |error| <= s Delta + E;
  B  the STEPPED and HELD quantities, at every execution |error(n)| <= |error(n_0)| + s Delta + E (E the largest over the
     segment). The DRIVEN quantities get no B: the R1 and R1X rate transient and the R2 heading excursion are unbounded at T4.
PASS iff every run is clean (complete trailer, a TRUTH record for every tick, no stale host step in the window, every DShot of
every tick within [idle, 2047]) and S and B hold as above.

Negative control, one per scenario (harness only, 0003 item 9; the test passes only if the conjunction fails and S fails on
alpha): att_kp = 0 through sil_override, the loop disabled (decision 0031), no stale read.
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import l5_scenario as l5s  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05"
PLUGIN_DIR = run_l5.DEFAULT_PLUGIN_DIR
BUDGET = ROOT / "design" / "budget.yaml"
SETTLE_FRACTION = schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]
R1, R1X, R2 = "recover_inverted", "recover_inverted_exact", "recover_tumble"
ALL = [R1, R2, R1X]
ANGLE_UNIT, RATE_UNIT = "rad", "rad/s"


# ---- plumbing -------------------------------------------------------------------------------------------------------

@pytest.fixture(scope="module", params=ALL)
def name(request):
    return request.param


# ---- quantities and predicates --------------------------------------------------------------------------------------

def series(s, n_exec):
    """Per attitude execution n = 0 .. n_exec - 1 (the run's execution n): (alpha, |err_z|, |omega|) of the TRUTH record."""
    out = []
    for n in range(n_exec):
        t = s.executions[n]["truth"]
        assert t is not None, f"no TRUTH record at attitude execution {n}"
        ch = rm.channels(t["q_wxyz"], t["omega_frd"])
        out.append((rm.alpha_channel(t["q_wxyz"], None)[0], abs(ch[2]), math.sqrt(sum(c * c for c in ch[3:]))))
    return out


def evaluate(runs, name):
    """S and B (module docstring) on a [m = 2, m = 1] sequence: the checks per quantity and the verdict."""
    by_m = {s.run.m: s for s in runs}
    n_exec = by_m[1].plan.end + 1
    y1, y2 = series(by_m[1], n_exec), series(by_m[2], n_exec)
    roles = {R1: {"alpha": t4_rule.STEPPED, "heading": t4_rule.HELD, "omega": t4_rule.DRIVEN},
             R1X: {"alpha": t4_rule.STEPPED, "omega": t4_rule.DRIVEN},
             R2: {"alpha": t4_rule.STEPPED, "heading": t4_rule.DRIVEN, "omega": t4_rule.STEPPED}}[name]
    columns = {"alpha": (0, ANGLE_UNIT, 0), "heading": (1, ANGLE_UNIT, 0), "omega": (2, RATE_UNIT, 1)}  # (column, unit, first)
    segment, diff = {}, {}
    for q, role in roles.items():
        k, unit, first = columns[q]
        segment[q] = (unit, role, [y1[n][k] for n in range(first, n_exec)])
        diff[q] = [abs(y1[n][k] - y2[n][k]) for n in range(first, n_exec)]
    checks = t4_rule.scenario_checks({"recovery": segment}, SETTLE_FRACTION, {"recovery": {q: d[-1] for q, d in diff.items()}},
                                     {"recovery": {q: max(d) for q, d in diff.items()}})["recovery"]
    settles, bounded = checks["S"], checks["B"]
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {"passed": all(c.ok for c in (*settles, *bounded)) and not any(stale.values()), "S": settles, "B": bounded,
            "stale_reads_in_window": stale, "N": n_exec, "s": SETTLE_FRACTION}


def verdict_line(name, label, ev, wall):
    def part(checks):
        return ", ".join(f"{c.name} {'ok' if c.ok else 'FAIL'} {c.value:.3e} <= {c.bound:.3e}" for c in checks)
    return (f"{name} {label:<12} {'PASS' if ev['passed'] else 'FAIL'}  S [{part(ev['S'])}]  B [{part(ev['B'])}]  "
            f"stale {ev['stale_reads_in_window']}, N {ev['N']}, gz wall {wall:.2f} s")


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


@dataclasses.dataclass
class Sequence:
    runs: list
    report_path: str
    wall_s: float
    evaluation: dict


def fly(tmp_path_factory, label, name, overrides=None):
    runs, path = run_l5.run_sequence(CARD, SCEN / f"{name}.yaml", tmp_path_factory.mktemp(f"{name}_{label}"), PLUGIN_DIR,
                                     overrides=overrides)
    ev = evaluate(runs, name)
    wall = sum(s.run.wall_s for s in runs)
    run_l5.write_report(runs, dict(ev, gz_wall_s=wall), path)
    return Sequence(runs, path, wall, ev)


@pytest.fixture(scope="module")
def recovery(tmp_path_factory, name):
    return fly(tmp_path_factory, "recovery", name)


@pytest.fixture(scope="module")
def kp_zero(tmp_path_factory, name):
    return fly(tmp_path_factory, "kp_zero", name, {"att_kp": (run_l5.F32, "0.0")})


# ---- the pass bar ---------------------------------------------------------------------------------------------------

def test_scenario_initial_state_is_the_labelled_test_value(name, live_params):
    st = l5s.values(l5s.load(SCEN / f"{name}.yaml"))["initial_state"]
    exact = l5s.values(l5s.load(SCEN / f"{R1X}.yaml"))["initial_state"]
    assert exact["attitude_q_wxyz"] == [0.0, 1.0, 0.0, 0.0] and exact["rotor_speed_rad_s"] == "hover"
    assert st["rotor_speed_rad_s"] == ("steady_tumble" if name == R2 else "hover")
    if name == R1:
        w, x = st["attitude_q_wxyz"][:2]
        assert st["attitude_q_wxyz"][2:] == [0.0, 0.0] and abs(w * w + x * x - 1.0) < 1e-15
        assert abs(2 * math.atan2(x, w) - (math.pi - 2 * math.asin(w))) < 1e-15 and w > 1e-9  # delta far above the ~1e-16 noise
    else:
        assert st["attitude_q_wxyz"] == [0.0, 1.0, 0.0, 0.0]
    rates = [live_params["rate_max_roll"], live_params["rate_max_pitch"], live_params["rate_max_yaw"]]
    assert st["body_rates_frd_rad_s"] == ([0.0, 0.0, 0.0] if name != R2 else rates)


def test_run_is_clean_on_truth_and_in_the_dshot_range(recovery, name, live_params):
    idle, top = run_l5.l4.dshot_idle_bound(live_params)
    assert [s.run.m for s in recovery.runs] == [2, 1]
    for s in recovery.runs:
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert len(s.run.log["truths"]) == len(s.run.log["ticks"]), "a tick without a TRUTH record"
        assert s.window_stale == [], f"m = {s.run.m}: stale reads in the window at host steps {s.window_stale[:8]}"
        assert "truth_fed_perfect_model" in Path(s.run.log_path).name
        text = Path(s.run.world_path).read_text(encoding="utf-8")
        assert text.count("<gyro_source>truth</gyro_source>") == 1
        assert text.count("<attitude_source>truth</attitude_source>") == 1
        lo, hi = min(min(t["dshot"]) for t in s.run.log["ticks"]), max(max(t["dshot"]) for t in s.run.log["ticks"])
        assert idle <= lo and hi <= top, f"DShot [{lo}, {hi}] outside [{idle}, {top}]"


def test_recovery_settles_and_stays_bounded(recovery, name, capsys):
    ev = recovery.evaluation
    _say(capsys, verdict_line(name, "recovery", ev, recovery.wall_s), f"    report: {recovery.report_path}")
    assert ev["passed"], ev


# ---- negative control -----------------------------------------------------------------------------------------------

def test_control_att_kp_zero_fails_the_predicates(kp_zero, name, capsys):
    ev = kp_zero.evaluation
    _say(capsys, verdict_line(name, "control kp=0", ev, kp_zero.wall_s))
    for s in kp_zero.runs:
        assert s.harness_overrides == {"att_kp": ("f32", "0.0")}
        assert '<sil_override param="att_kp" type="f32">0.0</sil_override>' in Path(s.run.world_path).read_text()
        assert run_scenario.complete_trailer(s.run.log, s.run.iterations, s.run.m)
        assert s.window_stale == [], "the control must fail on the predicates, not on a stale read"
    assert not ev["passed"] and any(not c.ok and c.name == "alpha" for c in ev["S"]), ev
