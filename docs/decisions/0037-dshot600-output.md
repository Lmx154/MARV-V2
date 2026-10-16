# 0037: DShot600 motor output and the panic stop frames (M1 P10)

## What changed

`hal_actuators_write` on the target sends DShot600 frames through PIO0: one state machine drives the four motor pads
(GP6..9) together, one 16-bit frame per motor per write (11-bit value, telemetry bit 0, CRC
(v ^ v>>4 ^ v>>8) & 0xF, MSB first). A bit is 8 PIO cycles (3 high, 3 data, 2 low) at 600 kbit/s × 8 = 4.8 MHz; the
divider is derived from clk_sys at init (150 MHz → 31 + 64/256, exact). The state machine stalls with every pad low
when it has no frame and never repeats one: a firmware stall sends nothing, so the ESC's signal-loss stop applies. A
write that arrives while a frame is in flight is dropped and counted (`motor_drops`). The logical motor → pad map is
the parameter set `motor_pad_m1..m4` (default identity, a `PLACEHOLDER` set by the M1 motor test), validated as a
permutation at start-up and applied once; firmware never inverts a direction. The SDK-free encoder, timing, pad map
and stop-frame logic are in `fw/hal/target/decode` (`marv_dshot`) with T1 tests in
`tests/regression/quad/L09/unit/dshot_test.cpp`; the frozen `tests/regression/quad/L09/unit/CMakeLists.txt` gains it.
Nothing is removed or loosened.

`tests/regression/quad/L00/tools/test_lint_g1.py`: the check that `constants.hpp` still holds the L0 literals 3, 48 and
2047 "in order" compared the whole value-filtered list, so the new cited DShot constant 3 (T0H, 3/8 of a bit) failed
it. It now checks that the three appear as an ordered subsequence, which is what its comment states (the file "grows
with each layer's cited constants"); a missing or out-of-order L0 literal still fails it.

On `hal_panic`, the panic loop (interrupts off, USB polled, M1 P5) also sends stop frames (value 0 on every motor) at
the nominal rate-execution interval ceil(rate_loop_divisor × tick_period_num / tick_period_den) µs from the
parameters, each after the previous frame plus one idle frame time; before the motor output starts it sends nothing.

## Why

Quad spec §4.0 M1 ("PIO DShot600 output. eRPM decode and the RPM notches wait for the L6 resume") and Luis's ruling on
what a panic does to the motors, 2026-10-16, verbatim: "take option A" (a panic keeps sending DShot motor-stop
frames; if that fails the ESC's signal-loss stop applies). Betaflight's fault handler compares: `stopMotors()`
(read only).

## Evidence

Bench, FC3v2, battery not connected (no ESC power, no motor turned): a second PIO state machine sampled the motor pads
at 50 MHz in a temporary bench build (removed): T0H 620–640 ns, T1H 1240–1260 ns, bit 1660–1680 ns, frame ≈ 25.6 µs;
every frame carried its value and a correct CRC; pad map {2, 0, 3, 1} routed each motor's frame to its pad; panic stop
frames 61 µs apart at a 60 µs test interval, 29 µs after the last tick frame. Console: 7497 ticks/s, 3748 writes/s,
`motor_drops` 0 over 31 s. `ctest -L L09`: 25 passed (8 new). Independent review: PASS. Not yet done: the ESC arming
on stop frames and the motor test, with Luis present.

## Approval

Luis, 2026-10-16: "take option A"; the M1 motor test answers "1. 500 2. 3 seconds 3. yes" (P11).
