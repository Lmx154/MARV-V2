# 0049: Plant drag pulled forward into M4, before HIL S4

## What changed

Rotor and body drag move from the quad §4.0 deferred list ("plant fidelity") into M4, ahead of HIL stage S4 (Luis,
2026-10-17: "yes pull drag forward"). Rules for the change:

1. **Model:** a body-frame linear drag force on the vehicle, F_drag = −R D Rᵀ v (D = diag(d_x, d_y, d_z), kg/s; R body →
   NED, v the NED velocity relative to still air), the rotor-drag form of Faessler, Franchi and Scaramuzza (IEEE RA-L 2018,
   "Differential flatness of quadrotor dynamics subject to rotor drag"). Coefficients are card entries with source and σ;
   an unsourceable value is a labelled PLACEHOLDER (not a safety number) with what replaces it (a flight or wind-tunnel
   fit after first flight).
2. The drag force enters the translational dynamics **and the IMU specific force** (that is the point: the accelerometer
   must see drag, so a tilted, translating vehicle reads the gravity direction in steady state).
3. **Opt-in:** a card without drag coefficients keeps D = 0, and every existing world and log stays byte-identical (the
   UZH card's gz suites, L2, L4, L5, L7, L11, L10 plumbing). Only `vehicles/marv_5in_fc3v2.yaml` gains coefficients.
4. Frozen plant tests are extended, not loosened (ABI field list, a known answer for the drag force and the specific force).

## Why

HIL S2e (`tests/regression/quad/L10/results/s2e/README.md`): with no drag, the specific force is thrust along body z at
every tilt, so the Mahony estimator (kp 0.25, decision 0042) drifts toward level while the truth stays tilted; after a
30° step the truth stays 13-19° tilted. A real quad has drag; S4 (Luis flying the sim with the radio) needs the sim to
behave like one.

## Evidence

sim/plant/src/marv_plant.cpp:258 (specific force: thrust only); sim/plant/include/marv_plant.h:10, 71 ("drag comes
later"); L10/results/s2e/half_stick/flight/analysis.txt ("Accelerometer").

## Approval

Luis, 2026-10-17 (conversation): "yes pull drag forward". Model choice and opt-in rule: lead.
