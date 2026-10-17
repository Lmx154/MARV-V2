# 0050: M4: the low-throttle integrator hold ends at Betaflight's airmode latch

## What changed

1. Supersedes the rule of decision 0045 (rate integrator zeroed whenever armed and throttle < `arm_throttle_min`, no latch).
   `fw/compositions/quad_fc/src/quad_fc.cpp`: a latch `g_airmode_active` is set on an armed rate tick whose limited throttle is at or above the
   new parameter `airmode_activate_throttle`, and cleared on every rate tick on which the gate is not Armed (the gate follows the arming
   machine's armed output, so a switch, link-loss, IMU or refusal disarm clears it; the armed failsafe states `ArmedHold` and
   `ArmedFailsafeCentred` keep it). The integrator is zeroed after the output iff armed, throttle < `arm_throttle_min` and the latch is not set.
   Nothing else changes: mixer, PID at minimum throttle, the arming-edge controller restart and the hook `zero_integrator()` are as in 0045.
2. New register parameter `airmode_activate_throttle` = 0.29296875 (`fw/compositions/quad_fc/params/quad_fc_register.yaml`), derived: Betaflight 4.5.1
   `pg/rx.c:103` `airModeActivateThreshold` 25 percent; `fc/core.c:715-744` `calculateThrottlePercentAbs()` =
   `constrain(((rcData[THROTTLE] - mincheck) * 100) / (PWM_RANGE_MAX - mincheck), 0, 100)` in integer arithmetic, `mincheck` 1050 (`pg/rx.c:91`),
   `PWM_RANGE_MAX` 2000 (`rx/rx.h:33`); the first whole microsecond at 25 is 1288 us (1287 us gives 24); normalized (1288 - 988) / 1024 under the
   receive contract. Airmode is on by default: `config/feature.c:35` (`FEATURE_AIRMODE`); the latch: `fc/core.c:776-783`; the reset decision
   `fc/core.c:790-796`.
3. Quad spec 10, section 4.0 M4: "(Betaflight without airmode)" becomes the latch (Luis, 2026-10-17, decision 0050).
4. Frozen `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`: the beside oracle (`hold_integrator`) applies the latch, so the expectations of
   `ALostLinkHoldsThenCentresThenWritesStopAtTheDisarmTimeOnThatRateTick` and
   `NonFiniteAndOutOfRangeSticksAndThrottleWhileArmedAreLimitedBeforeTheyReachTheRateLoop` follow it; the case
   `TheTransitionToLowThrottleAndBackZeroesAfterTheFirstLowTickOutput` winds the integrator at a throttle just below the threshold instead of 0.5 (which
   now latches). Cases added: `TheAirmodeThresholdIsBetaflightsTwentyFivePercentOfMinCheckToTwoThousandMicroseconds`,
   `ArmedAtIdleBeforeTheThresholdTheIntegratorIsStillHeldAtZero`, `ThePassOfTheThresholdLatchesAndALaterDipBelowMinCheckDoesNotZeroTheIntegrator`,
   `AfterTheThresholdAnyLowThrottleEqualsTheGroupThatNeverZeroes`, `ADisarmClearsTheLatchAndTheNextArmZeroesAgain`. No threshold of any test changed.

## Why

Decision 0045 implemented Betaflight with airmode off; its record left the default airmode latch pending Luis. The evidence
`tests/regression/quad/L10/results/ihold/README.md` (simulation, placeholder 5 mm centre-of-mass offset): a 0.3 s throttle dip to stick 0 in hover
gave a peak tilt of 26.8 deg and 3.4 s back to band with the hold, against 2.3 deg and 1.4 s for a dip just above min_check. The latch keeps the
props-off fix (armed at idle on the pad the integrator stays zero) and drops the hold once the throttle has passed 25 percent of Betaflight's span.

## Evidence

`ctest --preset host-debug -R 'Gate|QuadFc|quad_fc|Sil|L09|L04|L05'`; with the latch never set, with `>` for `>=`, and with the clear removed, the latch
cases fail (the first two also fail the failsafe and out-of-range cases whose oracle now latches). Gazebo: `tests/regression/quad/L10/results/ihold_latch/`.

## Approval

Luis, 2026-10-17: "Implement b" (the default latch: the integrator is zeroed only until the throttle first passes 25% after arming; after that it is
never zeroed while armed).
