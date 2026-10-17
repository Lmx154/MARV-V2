"""Helpers of the L7 gz suite and of the measurement record tests/regression/quad/L07/results/hover_kick (decision 0042 item 4).

The runs are tools/sim/run_l5.py's, on the host-gz-l7 build (the l7_attitude_estimated composition, the estimator the only
attitude source): no <attitude_source>, the model IMU at the nominal corner (<gyro_source>model</gyro_source> and
<imu_model>, turn-on bias signs 0 unless a caller passes others). run_l5 names the host-gz-l5 parameter set and labels its
runs truth-fed; `patched` replaces those module globals for the duration of a call, so run_l5.py itself is unchanged.
Truth is the log's STEP body records (body_q_wxyz): the plant state at the start of the host step, which for an attitude
execution at tick j is step j // m (m divides the attitude divisor).
"""

import contextlib
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_world  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L07" / "hover.yaml"
PLUGIN_DIR = ROOT / "build" / "host-gz-l7" / "sim" / "gz" / "plugin"
PARAMS = "marv_params_l7_attitude_estimated"
BUDGET = ROOT / "design" / "budget.yaml"
NOMINAL = (0, 0, 0, 0, 0, 0)
LABEL = "estimator-fed, IMU model"
LABEL_LINE = ("label: estimator-fed, IMU model. The rate loop's gyro sample and the attitude state (the Mahony estimate) come "
              "from the plant IMU model (<gyro_source>model</gyro_source>, no <attitude_source>, decision 0042 item 4); truth "
              "and firmware share one card (core section 6): not a validation run")
NAME_LABEL = "estimator_fed_imu_model"
F32 = run_l5.F32
OVERRIDES_ATT_KP_ZERO = {"att_kp": (F32, "0.0")}
OVERRIDES_LOOP_DISABLED = {"att_kp": (F32, "0.0"), "rate_ki_roll": (F32, "0.0"), "rate_ki_pitch": (F32, "0.0")}
SEGMENT = "hover"
UNIT = "rad"


def settle_fraction():
    return schema.load_yaml(BUDGET)["t4_settle_fraction"]["value"]


def tilt_share():
    return schema.load_yaml(BUDGET)["attitude_tilt_error_max"]["value"]


@contextlib.contextmanager
def patched():
    names = ("COMPOSITION_PARAMS", "MODEL_LABEL", "MODEL_LABEL_LINE", "NAME_LABEL")
    saved = {n: getattr(run_l5, n) for n in names}
    run_l5.COMPOSITION_PARAMS, run_l5.MODEL_LABEL, run_l5.MODEL_LABEL_LINE, run_l5.NAME_LABEL = (
        PARAMS, LABEL, LABEL_LINE, NAME_LABEL)
    try:
        yield
    finally:
        for n, v in saved.items():
            setattr(run_l5, n, v)


def live_params():
    """The plugin build's parameter table (name -> value)."""
    with patched():
        _, defaults = run_l5.build_parameters(PLUGIN_DIR)
    return run_l5.l4.read_param_defaults(defaults)


def fly(out_dir, overrides=None, signs=NOMINAL, scenario=SCENARIO):
    """[m = 2 run, m = 1 run] (run_l5.StepRun) of the scenario, one gz process each."""
    sensors = gen_world.SensorSet(gyro="model", rotor=False, bias_signs=tuple(signs))
    with patched():
        return [run_l5.run_step(CARD, scenario, m, out_dir, PLUGIN_DIR, overrides=overrides, attitude_source=False,
                                sensors=sensors) for m in (2, 1)]


def tilt_of(q):
    """The angle between body z and NED z of the body -> NED quaternion q = [w, x, y, z], rad."""
    w, x, y, z = q
    return 2.0 * math.atan2(math.hypot(x, y), math.hypot(w, z))


def tilts(s):
    """Truth tilt per attitude execution of the window of the run s."""
    p, m, steps = s.plan, s.run.m, s.run.log["steps"]
    return [tilt_of(steps[p.tick_of(a) // m]["body_q_wxyz"]) for a in p.window]


def evaluate(runs):
    """Predicate S on tilt (a DRIVEN quantity, decision 0031) at the last execution: |tilt| <= s Delta + E +
    attitude_tilt_error_max, Delta the largest tilt of the m = 1 run, E = |tilt(m = 1) - tilt(m = 2)| there."""
    by_m = {s.run.m: s for s in runs}
    t1, t2 = tilts(by_m[1]), tilts(by_m[2])
    e_end, share, s_frac = abs(t1[-1] - t2[-1]), tilt_share(), settle_fraction()
    check = t4_rule.scenario_checks({SEGMENT: {"tilt": (UNIT, t4_rule.DRIVEN, t1)}}, s_frac,
                                    {SEGMENT: {"tilt": e_end + share}}, {SEGMENT: {"tilt": 0.0}})[SEGMENT]["S"][0]
    stale = {f"m{s.run.m}": len(s.window_stale) for s in runs}
    return {"S": check, "passed": check.ok and not any(stale.values()), "stale_reads_in_window": stale,
            "final_tilt_m1": t1[-1], "final_tilt_m2": t2[-1], "peak_tilt": max(t1), "E": e_end, "share": share,
            "s": s_frac, "band": check.bound, "margin": check.bound - check.value, "N": len(t1)}


def verdict_line(label, ev, wall):
    c = ev["S"]
    return (f"{label:<22} {'PASS' if ev['passed'] else 'FAIL'}  S tilt {'ok' if c.ok else 'FAIL'} {c.value:.5f} <= "
            f"{c.bound:.5f} rad (s Delta {ev['s'] * ev['peak_tilt']:.5f} + E {ev['E']:.2e} + share {ev['share']}), "
            f"margin {ev['margin']:+.5f} rad, peak {ev['peak_tilt']:.5f} rad, stale {ev['stale_reads_in_window']}, "
            f"N {ev['N']}, gz wall {wall:.2f} s")
