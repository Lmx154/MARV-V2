#!/usr/bin/env python3
"""Measurement record of decision 0042 item 4: the L7 hover kick on the estimate, and what each control does.

  build/host-gz-l7 built (cmake --preset host-gz-l7 && cmake --build --preset host-gz-l7); then from the repository root
  uv run python tests/regression/quad/L07/results/hover_kick/hover_kick.py > tests/regression/quad/L07/results/hover_kick/hover_kick.txt

Four runs of scenarios/quad/L07/hover.yaml (l7_gz.fly, one gz process per m of [2, 1], seed 1), each reported with predicate S on
the truth tilt (l7_gz.evaluate, the L07 gz test's) and the truth tilt at fixed executions of the m = 1 run:
  estimate                  the estimator in authority, nominal IMU corner (the test's passing run)
  att_kp = 0                att_kp = 0 alone: the rate loop's integral restores the kick's tilt, so S passes (not a control)
  loop disabled             att_kp = rate_ki_roll = rate_ki_pitch = 0 (the test's control)
  estimate, bias corner     the estimate run with the turn-on bias signs all +1 (gyro +0.4 deg/s and accel +20 mg per axis, the
                            profile's bounds): the truth drifts, S fails; why the test flies the nominal corner
Only the printed numbers are the record; the gz logs are written to a temporary directory.
"""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gz"))
import l7_gz  # noqa: E402

CASES = (
    ("estimate", None, l7_gz.NOMINAL),
    ("att_kp = 0", l7_gz.OVERRIDES_ATT_KP_ZERO, l7_gz.NOMINAL),
    ("loop disabled", l7_gz.OVERRIDES_LOOP_DISABLED, l7_gz.NOMINAL),
    ("estimate, bias corner", None, (1, 1, 1, 1, 1, 1)),
)
SAMPLES = (0, 10, 100, 1000, 2000, 5000, 10000, 20292)  # executions of the window


def main():
    print(f"scenario {l7_gz.SCENARIO.relative_to(l7_gz.ROOT)} sha256 {l7_gz.run_scenario.sha256_file(l7_gz.SCENARIO)}")
    print(f"s = t4_settle_fraction {l7_gz.settle_fraction()}, share = attitude_tilt_error_max {l7_gz.tilt_share()} rad")
    params = l7_gz.live_params()
    print("build parameters: " + ", ".join(f"{k} {params[k]!r}" for k in (
        "att_kp", "rate_ki_roll", "rate_ki_pitch", "estimator_kp", "estimator_ki")) + " (host-gz-l7 table)")
    for label, overrides, signs in CASES:
        with tempfile.TemporaryDirectory() as d:
            runs = l7_gz.fly(d, overrides, signs)
        ev = l7_gz.evaluate(runs)
        print()
        print(f"{label}: overrides {sorted(overrides) if overrides else 'none'}, bias_signs {list(signs)}")
        print("  " + l7_gz.verdict_line(label, ev, sum(s.run.wall_s for s in runs)))
        by_m = {s.run.m: s for s in runs}
        t1 = l7_gz.tilts(by_m[1])
        print("  truth tilt (rad) at window executions " + ", ".join(f"{n}: {t1[n]:.5f}" for n in SAMPLES if n < len(t1)))
        print(f"  final tilt m = 1 {ev['final_tilt_m1']:.6f}, m = 2 {ev['final_tilt_m2']:.6f}; peak {ev['peak_tilt']:.6f}")


if __name__ == "__main__":
    main()
