# HIL S2f repeat with the airmode latch (decision 0050), against `../ihold/` (decision 0045 rule)

SIMULATION ON INFERRED / UNVERIFIED / PLACEHOLDER CARD VALUES (vehicles/marv_5in_fc3v2.yaml), NOT A FLIGHT RESULT. Evidence, not a test: nothing tuned. The
disturbance (5 mm rotor-to-centre-of-mass offset) is the PLACEHOLDER of `../ihold/README.md`. Commit of the runs: the reports name it (clean tree).

```sh
cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc -j 6 && uv sync --frozen
uv run python tools/sim/run_quad_fc.py --out-dir <dir> --script ihold --runs both     # dip_below_mincheck/ dip_above_mincheck/
uv run python tests/regression/quad/L10/results/ihold_latch/punch_wrap.py --out-dir <dir> --script ihold --runs both   # punch_dip_*/ (from the repository root)
```
Layout per run dir as `../ihold/` (`world.sdf`, `report.txt`, `analysis.txt`, `traces.csv.xz`, `scenario/hover.yaml`, `SHA256SUMS`).

## 1. The unchanged ihold script: identical to `ihold/`
`dip_below_mincheck/` and `dip_above_mincheck/` have the same log sha256 as `../ihold/` (29fbbdfc..., 4eca27a9...): peak tilt after the dip 26.78 deg and 2.26 deg,
tilt back in band 3.41 s and 1.39 s. Reason: the latch needs the limited throttle at or above `airmode_activate_throttle` = 0.29296875 (300/1024), and this
script never goes above its derived hover stick 0.0716427 (the quad_fc thrust map is linear in the stick between T_min and T_max, hover thrust 7.18 N is
7 % of the way). So the latch is never set in this script, and the rule is exactly decision 0045's. The runs show the change does not alter behaviour below the threshold.

## 2. `punch_wrap.py`: the same script with one 0.05 s throttle punch to stick 0.30 (just above the threshold) at 13.0 s, 10 s before the dip
A scenario value (labelled in the file); everything else as the ihold script (the wrapper only splits its `hold` phase). Pre-dip state is settled (tilt 0.297 deg).
| | dip to stick 0, no latch (`../ihold/`) | dip to stick 0 after the punch (latched) | dip to 0.0625, after the punch |
| --- | --- | --- | --- |
| peak tilt after the dip start | 26.78 deg | **19.32 deg** at 23.404 s | 2.25 deg |
| peak \|body rate\| | 1.170 rad/s | 1.202 rad/s | 0.144 rad/s |
| tilt back in band after the dip end | 3.41 s (+-2.65 deg) | 2.87 s (+-1.90 deg) | 1.40 s (+-0.195 deg) |
So the latch helps (19.3 against 26.8 deg), but does not reach the 2.3 deg of a dip that stays above min_check. In the latched dip the integrator is kept (the dip-start
DShot is 48 / 268 / 199 / 209 against 48 / 59 / 70 / 69 without the latch, `traces.csv.xz` t = 23.001 s) and the mixer keeps applying the learned 0.036 Nm
differential while the thrust falls to about zero; the disturbance it learned is proportional to the thrust and vanishes with it, so the quad is rolled by the
kept correction (body rate 0.4 to 0.9 rad/s during the dip, tilt 15.9 deg inside the dip window) before the thrust returns. The same dip depth (stick 0) is in both
the 26.8 and the 19.3 deg cases; the caveat of `../ihold/README.md` (depth and zeroing are not separated) stands for the 2.3 deg column.

## 3. The pad (armed at idle before lift-off), from `dip_below_mincheck/traces.csv.xz`
| window (SIM time) | DShot min..max | max \|commanded torque\| roll / pitch | body rate |
| --- | --- | --- | --- |
| 2.76 to 2.99 s, armed, stick 0 | 48..104 | 0.00035 / 0.00029 Nm | 0 |
| 3.00 to 3.60 s, hover stick on the pad (pad holds the quad) | 581..585 | 0.0026 / 0.0015 Nm | 0 |
| 3.65 to 4.00 s, lift-off | 555..613 | 0.0455 / 0.0459 Nm | up to 0.73 rad/s |
The gyro is exactly zero on the pad, so the rate error is zero and the integrator has nothing to wind: this run cannot show the props-off windup fix (that is
the unit cases `ArmedAtIdleBeforeTheThresholdTheIntegratorIsStillHeldAtZero` etc. with a gyro bias). What it shows is that armed at idle the commanded torque stays under 0.0004 Nm.
