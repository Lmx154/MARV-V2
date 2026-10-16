# 0038: The console motor test (M1 P11)

## What changed

The quad_fc motor gate gains a second state, MotorTest, beside Disarmed. A request reaches it through a new HAL pull, `hal_motor_test()` (counters plus the latest request; `fw/hal/include/marv/hal/motor_test.hpp`), and its replies leave through `hal_motor_test_report()`. The gate stays the only place that decides motor values: on a rate tick it applies a cancel, then a request, then the deadline; a valid request writes the requested value to exactly one logical motor and `DshotValue::stop()` to the other three (at most one motor nonzero, by construction of the write). A request outside motor 1..4, value 48..`motor_test_dshot_max` or duration 1 ms..`motor_test_duration_max_ms` is refused, counted and reported with every motor stopped. The test ends at the first rate tick whose stamp is at least the request's stamp plus its duration, or whose count of rate ticks since the request reaches the duration over the nominal rate period (so a stuck stamp cannot extend it; a stamp before the request's also ends it), or at the first rate tick after a cancel. The new parameters in `quad_fc_register.yaml` are `motor_test_dshot_max` = 500 and `motor_test_duration_max_ms` = 3000 (provenance "Luis 2026-10-16"); `init` panics if the cap is outside the DShot range. The gate writes its values to the HAL directly, so the mixer's DShot diffuser carry is not reachable and is not reset.

On the target the console parses and forwards only. `marv_console_decode` (SDK-free, `fw/hal/target/decode`) parses `motor_test <1-4> <dshot> <ms> PROPS-OFF` (the literal token is the stateless unlock) and `stop`. The idle loop in `main.cpp` reads bytes; every byte, `stop` and a USB disconnect is a cancel. Hand-off: the idle loop is the single writer and fills the slot the tick is not reading, then flips an atomic index; the IMU tick reads one consistent snapshot without waiting. Replies (accepted, refused with reason, stopped with reason deadline/cancel/disconnect) go back through a seqlock and are printed from the idle loop; the stats line gains the counters. Constants `kMicrosecondsPerMillisecond` and `kDecimalRadix` join `constants.hpp` with citations.

Frozen paths: `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp` is extended (ten tests; existing tests unchanged); `tests/regression/quad/L09/unit/CMakeLists.txt` gains `unit_console`; new `tests/regression/quad/L09/unit/console_test.cpp` and bench log `tests/regression/quad/L09/results/p11_bench/console.log`. `ci/blast_radius.yaml` ignores the L09 results.

## Why

Quad spec §4.0 M1: with the props off, each logical motor is spun alone to fix the motor-to-pad map (`motor_pad`, decision 0037). The safety numbers are Luis's, not ours; refusing in the composition keeps the console, which is outside the flight path, unable to widen them.

## Evidence

`ctest --preset host-debug -L L09`: 41 passed (new: gate, deadline at the exact tick and at the exact stamp, cancel and disconnect at the next rate tick, exhaustive motors and values for I1, 11 refusals, 6 parser tests). Mutations that fail the tests: cap + 1, tick bound, deadline `>`, two motors, disconnect ignored. Bench (FC3v2, battery not connected, ESC unpowered): `console.log` shows the replies and counters; 7496.3 to 7497.0 ticks/s, 3747.9 to 3749.0 writes/s, `motor_drops` 0, unchanged from decision 0037. The DShot values on the pads were not captured.

## Approval

Luis, 2026-10-16, verbatim: "1. 500 2. 3 seconds 3. yes" (cap 500; maximum duration of one command 3 s; the unlock form `motor_test <1-4> <dshot> <ms> PROPS-OFF` accepted, with any received byte, `stop` or a disconnect cancelling).
