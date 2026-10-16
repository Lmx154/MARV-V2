# 0034: The timing-fault rule (M1 P7a)

## What changed

The rate loop no longer halts on a sample spacing off its period. A spacing dt with tick period num/den µs, divisor D
and tolerance tol is refused when |den·dt − D·num| > den·tol (in `uint64`; a dt whose den·dt overflows, such as a stamp
going backwards, is refused). A refusal takes the existing fault path (zero torque, state reset, latched, counted) and
a new timing counter; the loop re-anchors on that stamp, so the next execution's spacing is taken from it and that
execution seeds. `tol` is the new i32 parameter `rate_period_tolerance_us`: 1 µs in the SIL sets
(`design/rate_timing_register.yaml`, the hal_sim truncation bound of decision 0005), and a labelled `PLACEHOLDER` of
1 µs in `marv_params_fc3v2` (`fw/compositions/quad_fc/params/fc3v2_rate_timing_register.yaml`), to be replaced by
ceil(T_rate·odr_error + 2δ) after M1 P8 and P12. `require_valid` refuses a tolerance with 2·den·tol ≥ num.

Frozen files:

- `tests/regression/quad/L04/unit/rate/rate_test.cpp`: the three `EXPECT_DEATH` tests become "refused and counted"
  (zero torque, fault latched and counted, timing count 1, re-anchored) on the same stamps: ±1 µs accepted, ±2 µs,
  repeated, dropped and backwards refused; 312/313 µs accepted and 311/314 µs refused at 625/4 µs; a refusal after a
  fault execution. Added: 260/261 accepted and 259/262 refused at 3125/24 µs, and tolerances 78/79 (625/4) and 65/66
  (3125/24) accepted/refused at init. The config helper sets the check's rational (1000/1 µs, D = 1, tol 1 µs).
- `tests/regression/quad/L05/unit/rate_bypass/rate_bypass_test.cpp`: its `EXPECT_DEATH` period test (not named in the
  packet) halted on the same rule; now refused and counted at the same jitters (±1 accepted; ±2, repeated, dropped
  refused). Its config helper sets the same rational.
- `tests/regression/quad/L06/d_term/rate_loop_test.cpp`: its fixture takes `rate::from_params()` (now the SIL
  rational, 312.5 µs) but runs 1 ms stamps; it sets the rational to its own 1 ms period, as its `period` already was.
- `tests/regression/quad/L06/product_params/marv_params_product_table.txt`: rows 0–100 byte-identical; one new row,
  101 `rate_period_tolerance_us`; the header's count (101 → 102), schema hash and Inputs line (adds
  `design/rate_timing_register.yaml`) follow from it.
- `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`, `tests/regression/quad/L09/unit/CMakeLists.txt`: a T1 test
  feeds quad_fc late, early, repeated, backwards and missing stamps: nothing halts, the gate writes stop on every rate
  tick, and a rate-group step built as quad_fc builds it counts 2, 2, 2, 2 and 1 timing faults.

- `tests/regression/quad/L06/tools/rate_law_tool.cpp`: its `detail::panic_period` stub is deleted; fw/rate no longer
  declares it. `tests/regression/quad/L05/unit/rate_bypass/golden_gen.cpp` keeps its stub: `regenerate.sh` compiles it
  against `fw/` at tag `quad-L4-pass`, whose header still declares it.

No boundary moved: every accepted and refused spacing above is the one the death tests used.

## Why

Quad spec §4.0 M1: "Flight timing never halts the firmware after arming: a sample spacing outside the IMU profile's
clock bound, or a missing sample, is a counted fault handled by the loop's fault path". The half-µs `lround` check was
exact only for half-integer periods; the target tick 3125/24 µs is not one. The tolerance is a parameter with
provenance (G2) so the target can carry its own; a separate register is used because the id must be the same in every
set that links `marv_rate` and `design/scenario_values.yaml` stays unchanged. The attitude law keeps its halt (M3).

## Evidence

`cmake --build --preset host-debug && ctest --preset host-debug`; `cmake --build --preset m33`; mutation checks (spacing
check disabled; no re-anchoring) fail the L04, L05 and L09 tests above; the product table diff is the one row and its
header.

## Approval

Luis, 2026-10-16, verbatim: "P7 approved yes", to: a timing anomaly on the target is a counted fault (zero torque,
latched, counted, re-anchor), never a halt; the L04 death cases become refused and counted with unchanged boundaries;
SIL tolerance stays 1 µs. Board tolerance as a `PLACEHOLDER` until P12: Luis, 2026-10-16, "4. yes".
