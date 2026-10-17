# 0043: M4 angle mode on the board: the attitude law's timing faults, failsafe and the mode switch

## What changed

1. `AttitudeLaw` no longer halts on an execution spacing off its period. As the rate loop (decision 0034): refused when
   |den·dt − D_a·num| > den·tol, D_a = rate_loop_divisor × att_loop_ratio, tol = `rate_period_tolerance_us` (existing id;
   valid while T_a·odr_error + 2δ + 1 µs ≤ tol, on fc3v2 att_loop_ratio ≤ 3). A refusal takes the law's fault path
   (zero rate setpoint, latched, counted, a timing counter) and re-anchors on that stamp; `detail::panic_period` and
   `kPeriodToleranceUs` are deleted. Frozen files: `tests/regression/quad/L05/unit/attitude/attitude_test.cpp`, the
   L5AttitudePeriod death tests become refused-and-counted at unchanged boundaries (±1 µs accepted; ±2 µs, repeated,
   backwards, dropped and after-fault refused; 312/313 accepted and 311/314 refused at 312.5 µs; nothing checked after
   reset; a law initialised with an unset or invalid timing config refuses every execution as a counted timing fault,
   so the check never runs silently off); `AttitudeConfig` gains four timing fields (`tick_period_num_us`,
   `tick_period_den`, `period_ticks`, `period_tolerance_us`), so the hand-built configs that run the law also set them,
   fields added and nothing loosened: `tests/regression/quad/L05/unit/angle/angle_test.cpp` (`test_config`: 1000/1 µs,
   one tick, 1 µs) and `tests/regression/quad/L05/t3/t3_test.cpp` (from the fixture's tick period, divisor and ratio,
   tolerance 1 µs as in the SIL register; `attitude_t3_inputs.txt`, its checksums and every threshold and golden are
   unchanged); `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`, the off-period test with angle selected: nothing halts
   and the attitude timing count equals the rate timing count.
2. No failsafe phase forces angle mode; the mode channel holds the newest valid frame through ArmedHold and
   ArmedFailsafeCentred. Decision 0041 is unchanged.
3. quad_fc: `rc_channel_angle` (0..16, 0 = no switch: acro, Betaflight's default of no configured mode) and
   `angle_switch_threshold`; angle iff value ≥ threshold, NaN is acro; init refuses the arm or a stick channel. On entering
   armed-and-angle: `AngleMode::reset()` and `AttitudeLaw::reset()`; the law runs only there; the rate integrator is not
   reset on a mode change.

## Why

Quad spec §4.0 M1: flight timing never halts the firmware after arming; 0034 left the attitude law's halt for later, and
on fc3v2 (spacings 266/267 µs, ±0.79 µs entry jitter) the 1 µs check can fire in flight. Betaflight 4.5.1 forces ANGLE
only when `failsafeIsActive()` (core.c processRxModes), which with Drop becomes true in the pass that disarms
(failsafe.c:213-217, 314, 324-328, 387-388); aux channels hold in stage 1 (rx.c pgResetFn_rxFailsafeChannelConfigs);
no mode is configured by default (rc_modes.c, INFERRED from the parameter-group pattern); pid.c resets no I term on a
mode change.

## Evidence

fw/attitude/include/marv/attitude/attitude_law.hpp:32, 75-84; fw/attitude/src/attitude.cpp:47-49;
fw/rate/include/marv/rate/rate_loop.hpp:197-200, 252-265; fw/compositions/quad_fc/params/fc3v2_rate_timing_register.yaml;
docs/decisions/0034-timing-fault-rule.md; Betaflight tag 4.5.1 source as cited, read 2026-10-17 (architect consult).

## Approval

Item 1 applies Luis's 0034 ruling (a timing anomaly on the target is a counted fault, never a halt) to the attitude law;
boundaries unchanged, nothing loosened. Item 3's channel, switch position and threshold: Luis, pending (the default 0
keeps acro until he chooses).
