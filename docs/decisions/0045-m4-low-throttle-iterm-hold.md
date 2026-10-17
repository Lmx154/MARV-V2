# 0045: M4 C1: the rate integrator is held at zero at minimum throttle

## What changed

1. quad_fc (`fw/compositions/quad_fc/src/quad_fc.cpp`): while armed and the (limited) throttle is below `arm_throttle_min`
   (existing parameter, Betaflight min_check 1050 us; no new parameter), the rate loop's integrator is zeroed after each
   rate execution, in acro and in angle mode. That execution's output holds at most one increment over a zero previous
   integrator (the order of Betaflight's `pidResetIterm`, which runs at the end of `pidController`, after the sum is
   formed); P, D, the filters and the prefilter run as before. No latch. New hook: `RateLoop::zero_integrator()`
   (`fw/rate/include/marv/rate/rate_loop.hpp`) and `RateGroupStep::zero_integrator()`
   (`fw/rate_group/include/marv/rate_group/rate_group.hpp`); it zeroes the integrator and no other state. The attitude law
   has no integrator (nothing in `fw/attitude` integrates), so nothing is held there.
2. Frozen `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`: the beside oracle `ArmedGateTest::reference` and
   `reference_angle` apply the same rule (after the beside `ref_` executes, if armed and `want.throttle` < 0.060546875,
   the beside `ref_` integrator is zeroed; the constant is stated in the test as (1050 - 988) / 1024). A second beside group `integ_`
   (same samples, same arming-edge restart, never zeroed) is the control. Equality assertions are exact and no threshold
   changed. Two existing cases' effective expectation changes with the oracle: `ArmedGateTest.ALostLinkHoldsThenCentresThenWritesStopAtTheDisarmTimeOnThatRateTick`
   (from the first rate tick of ArmedFailsafeCentred, throttle 0 with the roll stick's prefilter error still nonzero) and
   `ArmedGateTest.NonFiniteAndOutOfRangeSticksAndThrottleWhileArmedAreLimitedBeforeTheyReachTheRateLoop` (its second loop, throttle
   -2 limited to 0, roll -1): both run armed at minimum throttle with a rate error, where the hold now applies; before the
   oracle change they failed by one to several DShot counts on motors 1 to 3.
   Cases added: `ArmedJustBelowMinCheckTheIntegratorStaysZeroAndTheOutputDiffersFromAnIntegratingGroup`,
   `ArmedAtMinCheckAndAboveTheIntegratorRunsAsBefore`, `TheTransitionToLowThrottleAndBackZeroesAfterTheFirstLowTickOutput`,
   `AngleGateTest.ArmedJustBelowMinCheckInAngleModeTheIntegratorStaysZero`. With the hold disabled in `quad_fc.cpp` these four
   and the two above fail.
3. Frozen `tests/regression/quad/L04/unit/rate/rate_test.cpp`: case added, `L4RateIntegratorHold.ZeroingLeavesEveryOtherStateAndTheNextIncrementStartsFromZero`
   (hand-computed, from the table of `kHand`); no existing case changed.

## Why

Quad spec §4.0 M4 as amended at 21dc067: "the rate integrator held at zero at minimum throttle (Betaflight without airmode)".
Luis 2026-10-16 on the props-off spin-up: "as long as we know the reason for the spin up then that is fine. unless you think it
is absolutely necessary then sure we can add it"; the lead decided to add it before props-on. A bias on the gyro or a held
stick on the ground winds the integrator and spins the motors up with the throttle at minimum.

Betaflight 4.5.1 evidence (tag source, read 2026-10-17): `fc/rc_controls.c:118` throttle is low below `mincheck`;
`pg/rx.c:91` `mincheck = 1050`. `fc/core.c:790-796`: armed and (`airmodeIsActivated` or throttle not low or launch control or fixed
wing) gives `pidSetItermReset(false)`; otherwise `pidSetItermReset(true)` and `pidStabilisationState(pidAtMinThrottle ? ON : OFF)`;
`flight/pid.c:137` `pidAtMinThrottle` defaults to ON, so P, D and F stay active. `flight/pid.c:1148-1149`: with the reset flag set
`pidResetIterm()` (pid.c:270-273, `pidData[axis].I = 0`) runs at the end of `pidController`, every loop, after `pidData[axis].Sum`
is formed. That is the rule implemented: airmode off, no latch. Betaflight's default is airmode ON: `config/feature.c:35` enables
`FEATURE_AIRMODE`, `fc/rc_modes.c:178` `airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOXAIRMODE)`.
With airmode on, `fc/core.c:781-788` latches `airmodeIsActivated` once throttle reaches `airModeActivateThreshold`
(`pg/rx.c:103`, 25 percent) until disarm: the same zeroing applies from arming until that first crossing, and never after it.
The latch variant (zero below min_check only until the throttle first reaches 25 percent after arming) is a spec change and
is pending Luis; this record implements the rule the spec text names.

## Evidence

`ctest --preset host-debug -L 'L04|L05|L08|L09'`; the six hold cases above fail with the call in `quad_fc.cpp` disabled and pass
with it. Betaflight tag 4.5.1 files as cited. Hook and call: `fw/rate/include/marv/rate/rate_loop.hpp` (`zero_integrator`),
`fw/rate_group/include/marv/rate_group/rate_group.hpp`, `fw/compositions/quad_fc/src/quad_fc.cpp` (`throttle_low`).

## Approval

The lead's decision to add the hold, per the spec amendment at 21dc067 and Luis 2026-10-16. The oracle change and the two cases'
effective expectations: the lead, 2026-10-17. The airmode-latch variant: Luis, pending.
