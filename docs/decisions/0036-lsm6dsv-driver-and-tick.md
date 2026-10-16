# 0036: the LSM6DSV driver and the target tick loop (M1 P9)

## What changed

Commit 1cc75fd, merged into `m1-sdk` at 91b00c4. `fw/hal/target/src/lsm6dsv.cpp` drives the LSM6DSV on SPI1
(WHO_AM_I checked at init; gyro ±2000 dps, accel ±16 g, ODR code 1100 high-performance; FIFO continuous; INT1 FIFO
watermark as a level GPIO interrupt). The interrupt reads the k samples in the FIFO and calls
`marv::composition::tick` k times in order, single core. Counters are published by a seqlock and printed once per
second by `main`'s idle loop; nothing prints in the tick. `hal_actuators_write` counts and drops the gate's stop writes
until the DShot packet (P10). The SDK-free decoding (conversion, mounting, flags, stamping) is in
`fw/hal/target/decode` with T1 known-answer tests; the frozen `tests/regression/quad/L09/unit/CMakeLists.txt` gains
`tests/regression/quad/L09/unit/lsm6dsv_decode_test.cpp`. Nothing is removed or loosened.

Choices:
- The datasheet numbers (sensitivities, register map, Ton, SPI clock limit; ST DS13476 Rev 5, with table numbers) are
  in `boards/parts/lsm6dsv.hpp`, outside `fw/`, as decision 0027 placed the board's pin numbers in `boards/`. G1 and
  `check_constants.py` keep datasheet numbers out of `fw/prim` constants. New cited constants in
  `fw/prim/include/marv/prim/constants.hpp`: 180°, g_n and 273.15 K.
- Stamping: t_irq is read at interrupt entry; the j-th of k samples is stamped t_irq − round((k−1−j)·T) with the
  nominal T; a stamp not above the previous one is clamped to previous + 1 µs and counted. Its error δ is a
  `PLACEHOLDER` until the M1 evidence run (P12) measures it.
- The mounting rotation chip → body FRD comes from the board header ((−x, +y, −z), INFERRED from madflight, MIT).
- A bench build option `MARV_FC3V2_COUNT_ONLY` (default OFF) ticks nothing and only counts samples.

## Why

Quad spec §4.0 M1: "One driver for the board's IMU (LSM6DSV over SPI, FIFO read), producing the existing IMU sample
struct, with an LSM6DSV sensor profile sourced from its datasheet", and a tick is one primary-IMU sample (core §3).

## Evidence

Bench, FC3v2 flat, 68 s, count-only image: 7497 samples/s (INTERNAL_FREQ_FINE −18 predicts 7500.3 Hz by DS13476
§9.49), 0 FIFO overflows, 0 SPI errors, 0 clamps; gyro ≈ 0 rad/s, accel (−0.009, 0.034, −9.792) m/s² in FRD. Merged
image (with the P7a timing rule): 7497 ticks/s, 3748 gate writes/s, no panic. `ctest -L L09`: 16 passed. Independent
review: PASS, with two items for later packets (the gyro saturation threshold, a guard against a stuck interrupt line).
Raw logs: `tests/regression/quad/L09/results/p9_bench/` (`tb-count-only-2.log`, the 68 s soak; `tb-quad-fc-70s.head`,
the old-panic run on the P9 branch; `tb-merged-p7a-p9.log`, the merged image), read with `tools/target/readcon.py`.

## Approval

Luis, 2026-10-15: "focus on the madflight fc3v2 rev-c" (quad spec §4.0 M1); 2026-10-16: tick 7.68 kHz nominal, "6. yes".
