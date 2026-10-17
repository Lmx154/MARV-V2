# HIL S2f: what the low-throttle rate-integrator hold (decision 0045) does after a throttle dip

SIMULATION ON INFERRED / UNVERIFIED / PLACEHOLDER CARD VALUES (vehicles/marv_5in_fc3v2.yaml), NOT A FLIGHT RESULT. Evidence, not a test: nothing
tuned (no fw/, plant, plugin, card, gain or threshold change). The disturbance size is a PLACEHOLDER scenario value (see below), so the sizes
below say how the effect behaves, not how big it will be on the real quad. Commit of the runs: d7d6afd (clean tree; the reports say so).

```sh
cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc -j 6 && uv sync --frozen
uv run python tools/sim/run_quad_fc.py --out-dir <dir> --script ihold --runs both     # dip_below_mincheck/ and dip_above_mincheck/
```
Per run dir: `world.sdf`, `report.txt`, `analysis.txt` (all numbers), `traces.csv.xz` (234 Hz), `scenario/hover.yaml`, `SHA256SUMS`. The logs are not
committed; sha256 in `analysis.txt` (below 29fbbdfc..., above 4eca27a9...). Deterministic (a second run gave the same sha256). ~8 s wall each (RTF 3.8).

## Why S2e did not show the hold, and what is done here
On the symmetric perfect-model card nothing pushes the quad steadily, so the rate integrator is about zero before a dip and zeroing it changes
nothing (`../s2e/README.md`). Here a constant disturbance torque makes the integrator carry a real load, then a dip zeroes it (or not).

**The disturbance.** There is no torque-injection feature in the plant or the plugin (the L11 "thrust offset" / "torque pulse" are added by the
`l11_vertical_scripted` firmware composition to its own command, not to the plant, and quad_fc has none). What the plugin does take as input is
the rotor positions of its element, so the tool shifts all four by d = 0.005 m on x and y (`--com-offset-m`; PLACEHOLDER: a 5 mm rotor-to-centre-of-mass
offset, a battery-placement tolerance; replaced by the measured balance of the built quad). The plant's origin is the centre of mass, so this is the
same as a centre of mass 5 mm off the rotor layout: a torque thrust x d on roll and pitch (0.0359 Nm each at hover, T = 7.178 N), proportional to the
thrust, that the firmware's mixer (from the unshifted card) does not know about. `world.sdf` lines 44-56 hold the shifted positions.

**The script** (SIL time): ready 2.75 s, armed idle 0.25 s, hover stick (the derived 0.0716427) from 3.0 s; the pad releases at 3.6452 s; hover 20 s
from the raise (so the lift-off transient and the drift have died away); the dip 23.0 to 23.3 s (0.3 s, the S2e dip length); hover again 6 s; disarm.
Angle mode throughout, sticks centred. The rate integrator is not in the log; its trace is the commanded torque: it settles at +0.03589 / -0.03589 Nm
(roll/pitch) before the dip, equal to the disturbance torque of the plant.

**The two runs.** Same disturbance, same timing, same everything but the dip stick:
- `dip_below_mincheck`: dip stick 0. `throttle_low = throttle < g_throttle_min` (fw/compositions/quad_fc/src/quad_fc.cpp:439) with
  g_throttle_min = arm_throttle_min = 0.060546875 (fw/compositions/quad_fc/params/quad_fc_register.yaml:49-50, Betaflight min_check 1050 us); the hold
  zeroes the integrator after every rate execution (quad_fc.cpp:461-463).
- `dip_above_mincheck`: dip stick 0.0625 = arm_throttle_min + 2/1024 (2 us above min_check on the receive contract throttle = (us - 988)/1024; the
  logged stick is 0.0625); the hold never engages.

## Result
| | dip to stick 0 (hold engaged) | dip to stick 0.0625 (not engaged) |
| --- | --- | --- |
| before the dip (last 1 s): tilt mean (min..max) | 0.278 deg (0.256..0.306) | the same (same run until 23.0 s) |
| before the dip: \|body rate\| mean | 0.00159 rad/s | the same |
| peak tilt after the dip start | **26.78 deg** at 23.981 s (x -17.5, y +20.2 deg) | **2.26 deg** at 23.431 s |
| peak \|body rate\| | **1.170 rad/s** at 23.570 s | 0.144 rad/s at 23.253 s (during the dip) |
| tilt back within the pre-dip band, after the dip end | 3.41 s (band +-2.65 deg) | 1.39 s (band +-0.198 deg) |
| \|body rate\| back within the band, after the dip end | 1.59 s (band +-0.117 rad/s) | 1.61 s (band +-0.0142 rad/s) |
| altitude lost at the dip end / +1 s / end of hold2 | 0.455 / 4.244 / 23.903 m | 0.129 / 0.804 / 4.205 m |
| same, against continuing at the pre-dip descent speed 0.295 m/s | 0.366 / 3.861 / 22.046 m | 0.040 / 0.421 / 2.348 m |
Band rule (a labelled analysis value, `analysis.txt`): the pre-dip mean +- the wider of the pre-dip envelope and 10 % of the peak excursion; the time is
that of the last sample outside it. The quad is flying open-loop in throttle (no altitude loop), so lost altitude keeps growing after any dip by the
descent speed gained (vz 3.9 m/s after the deep dip, 0.68 m/s after the shallow one). The 0.295 m/s pre-dip descent is left over from the lift-off in both runs.

## What this means (plain words)
Before the dip the integrator has learned the quad's steady push (here the 5 mm balance error) and cancels it; the quad sits level with a steady
0.036 Nm of correction. With the throttle below min_check the firmware (decision 0045, Betaflight without airmode) keeps resetting that learned
correction to zero. During the dip itself nothing visible happens: the thrust, and so the disturbance, is almost gone (tilt stays at 0.26 deg).
When the throttle comes back up the disturbance returns at once, but the correction does not: only the proportional part of the loop pushes
against it until the integrator has rebuilt, and in this run the quad rolls and pitches out to about 27 deg and 1.2 rad/s, taking about 3.4 s to
settle back within 2.7 deg. Without the reset (dip just above min_check) the learned correction survives the dip and the same quad only wiggles
by about 2 deg and is back in the pre-dip band in 1.4 s.
This is Betaflight's airmode trade-off. Zeroing the integrator at minimum throttle is what keeps the props from spinning up on the ground from a
gyro bias or a held stick (the reason for decision 0045), and it is the price of that: a chop to idle in the air and back costs the trim, and the
recovery takes seconds. Airmode keeps the integrator (and the motor authority) at low throttle, avoiding that cost; the latch variant (zero below
min_check only until the throttle first reaches 25 percent after arming) in decision 0045 is the middle option and is pending Luis.
How large the effect is on the real quad depends on its real steady torque (balance, motor mismatch), which is unknown (PLACEHOLDER above).

## Caveats
- The two dips differ in depth, not only in the hold: stick 0 takes the thrust to about zero (rotors fall to 20 rad/s), stick 0.0625 only to
  87 % of the hover thrust (rotors stay above 716 rad/s), and the disturbance is proportional to thrust. The difference in the post-dip
  excursion therefore also contains the deeper dip; the packet's control (just above min_check) was run as specified. No run isolates the
  zeroing from the depth (that would need the hold disabled, a firmware change).
- Altitude lost is the open-loop throttle's: the larger loss after the deep dip is mostly the dip itself (thrust ~0 for 0.3 s), not the hold.
- No drag tuning, one disturbance size, one seed, one pair of runs.
