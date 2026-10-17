# 0052: M4: the real card's maximum rotor speed is the loaded maximum; the throttle stick is linear in rotor speed

## What changed

1. `vehicles/marv_5in_fc3v2.yaml`, `rotors.speed_range`: `[0, 2991]` becomes `[0, 2297]` rad/s, INFERRED and UNVERIFIED, sigma `[0, 15]`.
   Rule: omega_max = sqrt(T_max / k_T), T_max the 1760 g maximum-load row (17.26 N) of the Emax ECO III 2807 1700KV table at 16.8 V with
   an Avia 7035-3 prop (reseller page cited in the card; Luis's motor is the ECO II, for which Emax publishes no table), k_T the mean of the
   table's seven rows that list an RPM (omega = RPM 2 pi / 60, T = g x 9.80665e-3 N), 3.270e-6 N/(rad/s)^2. The card's `thrust_coeff` stays
   2.8e-6 (its own source); the table's 3.270e-6 is recorded beside it as a conflicting source, both UNVERIFIED. It lies inside the stated range
   2.5e-6..3.4e-6, inside the 0.5 thrust-gain band of the vertical loop (ratio 1.17), and 0.02e-6 above the card's 1-sigma interval (sigma 0.45e-6).
   The gains, mixer parameters and reports are generated from the card at build time; no committed golden reads this card.
2. `fw/compositions/quad_fc/src/quad_fc.cpp`: the collective thrust from the throttle stick s is
   `T = (sqrt(T_min) + s (sqrt(T_max) - sqrt(T_min)))^2` (was `T_min + s (T_max - T_min)`), which equals
   `4 k (omega_idle + s (omega_max - omega_idle))^2` for the mixer's `T_min = 4 f_min`, `T_max = 4 f_max` (`fw/mixer/include/marv/mixer/mixer.hpp:76-84`).
   With the mixer's linear-in-omega ESC map the stick is then linear in rotor speed and DShot, as Betaflight's throttle to motor output
   (4.5.1, INFERRED reference: default `thr_mid` 50, `thr_expo` 0 in `config/controlrate_profile.c`, the throttle curve of `fc/rc.c` is then the
   identity, and `flight/mixer.c` maps the normalised throttle linearly onto the motor output range). The two square roots are taken at init; no
   new parameter, no new literal. The airmode latch threshold (decision 0050) is a stick value and is unchanged.
3. `tools/sim/run_quad_fc.py`: the derived hover stick is `(sqrt(m g) - sqrt(T_min)) / (sqrt(T_max) - sqrt(T_min))`; the docstring chain follows.
4. Frozen `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`: the beside oracle `thrust_of` (and `reference`, which now uses it) applies the new
   mapping; case added `TheThrottleStickIsLinearInRotorSpeedKnownAnswers` (s = 0 gives T_min, s = 1 gives T_max, s = 1/2 the stated formula, the real
   card's hover stick 0.3485, each checked as the DShot value of the ESC map from the mixer configuration alone, within one count). No threshold changed.

## Why

Luis's rulings of 2026-10-17 (the packet's two items). Derived consequence on the real card (m g = 7.178 N, k = 2.8e-6): with the no-load
maximum 2991 rad/s, T_max is 100.2 N (thrust to weight 14.0), and the thrust-linear stick hovers at s = m g / T_max = 0.0716, barely above
`arm_throttle_min` (0.0605) and well below the airmode threshold (0.293). The loaded maximum alone gives T_max = 59.1 N (thrust to weight 8.2)
and the thrust-linear hover stick 0.1215; with the rotor-speed-linear stick the hover stick is 0.3485 (omega_hover 800 rad/s of 2297).

## Evidence

QF-3 on the card, before -> after (build reports): rate worst PM 45.0044957 -> 45.0000674 deg, Ms 1.9994004 -> 1.9912204 (limits 45 deg, 2.0);
attitude loop worst PM 45.000011 -> 45.000007 deg, nominal 62.347593 -> 62.383605 deg; vertical unchanged (PM 45.000003, Ms 1.3411).
`ctest --preset host-debug -R 'Gate|QuadFc|quad_fc|Sil|replay|L07|L09|Mixer|L03|params'`; with the old thrust-linear formula back in the firmware the
new case and the oracle-dependent cases fail. Gazebo: `tools/sim/run_quad_fc.py --runs flight` (numbers in the commit report, not committed).

## Approval

Luis, 2026-10-17: "Yes (Recommended)" to both.
