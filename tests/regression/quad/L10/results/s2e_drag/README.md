# HIL S2e re-run on the drag plant (stick 0.5, flight + control)

SIMULATION ON INFERRED / UNVERIFIED / PLACEHOLDER CARD VALUES (vehicles/marv_5in_fc3v2.yaml), NOT A FLIGHT RESULT. Evidence, not a
test: nothing tuned. The same tool, script, card and firmware as `../s2e/` (see its README for the script, the derived hover stick and
the labelled scenario values); the one difference is the plant: the card now carries the rotor-drag entry (decision 0049, Faessler et
al. 2018 coefficients d = 0.356 / 0.214 / 0.0 kg/s, PLACEHOLDER), `world.sdf` line 59, so the accelerometer sees drag. Repo commit of
the runs: 32e7926 (clean tree; the reports say so). Supersedes `../s2e/` for tilt behaviour.

```sh
cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc -j 6 && uv sync --frozen
uv run python tools/sim/run_quad_fc.py --out-dir <dir> --runs both        # stick 0.5; flight/ and control/
```
Per run dir: `world.sdf`, `report.txt`, `analysis.txt` (all numbers), `traces.csv.xz` (234 Hz), `scenario/hover.yaml`, `SHA256SUMS`. The 80 MB
logs are not committed; their sha256 is in each `analysis.txt` (flight d98f3a8a..., control 8941c69d...). Each run ~6 s wall, 161 632 ticks.

## Result: truth tilt (angle between body z and NED z), deg
| | end of roll_centre (13.0 s) | end of pitch_centre (18.0 s) | end of run (21.55 s) | max overall |
| --- | --- | --- | --- | --- |
| flight (loop closed) | 3.78 | 3.34 | 5.85 | 35.40 (10.05 s) |
| control (att_kp = 0, rate_ki_roll = rate_ki_pitch = 0) | 0.11 | 0.19 | 0.31 | 0.31 |
Values are the last trace row at or before the instant (traces.csv.xz); the last-0.5 s means are in `analysis.txt` (4.32 and 3.44 deg).
Without drag (`../s2e/half_stick/flight`): 13.0, 18.6 and 18.9 deg. With drag the tilt after centring decays (35.4 -> 7.1 deg at 11 s ->
3.8 deg at 13 s after the roll step; 34.2 -> 5.3 -> 3.3 deg after the pitch step) and does not grow; it is not yet zero at the end of the
3 s centre phase. Step response with the loop closed: overshoot +5.14 deg (roll), +4.20 deg (pitch) on the 30 deg setpoint; last-0.5 s error 3.46 and
2.80 deg. Horizontal speed at the end of the steps 6.2 and 3.2 m/s (31.8 m/s without drag).
Hover hold 1 (lift-off 3.6452 s to 8 s): tilt mean 0.140 deg, max 0.242 deg. Dip (0.3 s, stick 0): altitude lost in the dip window 0.728 m, tilt 3.31 deg
in the dip, max 6.23 deg after it (20.48 s), body rate max 0.095 rad/s in the dip and 0.207 rad/s in the first second after.
The control hovers level (tilt max 0.31 deg at the end): the loop is what tracks the 30 deg setpoint; the tilt in flight comes from the commanded steps.

Accelerometer (analysis.txt "Accelerometer"): in the centre phases the accelerometer-implied tilt is now 12.6 deg (roll_centre) and 14.9 deg
(pitch_centre) mean, |f|/g 1.025 and 1.037 (0.26 deg and 1.000 on the drag-free plant): drag makes the specific force leave body z, so the Mahony
estimate is pulled toward the true tilt. Replica estimate versus truth, mean: 6.4 deg (roll_centre), 5.0 deg (pitch_centre). INFERRED: the replica is binary64, not the firmware's binary32.
