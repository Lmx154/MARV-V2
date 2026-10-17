# 0054: HIL S4 bridge: the delay lives in the HIL source; receiver records

## What changed

HIL stage S4a (decision 0046) as built (`ebb642b`, merged into `m4`):
1. The gz plugin talks to the board directly: a `HilCommandSource` selected by a world `<hil>` element (no separate bridge
   process). `<hil>` is refused together with `<command_delay_ticks>`, `<manual_control_script>`, a non-model gyro, truth
   sources or a clock corner.
2. **The N-tick delay of 0046 item 2 lives in the HIL source, not the adapter's delay line** (the adapter needs tick j's
   command at tick j, which the board cannot supply in time). Same rule: at tick j the plant gets the board's command of
   tick j − N (rate tick ⌊(j−N)/d⌋·d), zeros before. In HIL logs the TICK dshot column is the applied command and there is
   no DELAY record; the board's issued commands are in the saved uplink. A missing command is a stated fault that stops the
   run; nothing is reused. Wall-clock pacing absorbs lateness and reports it.
3. Image option `MARV_FC3V2_HIL_RECEIVER` (requires `MARV_FC3V2_HIL`): the flight CRSF receiver at its flight priority
   arms the HIL image (0046 item 4); host stick records are refused; new uplink records MHRB (receiver boot), MHRF (each
   frame with its board stamp, for the S5 replay), MHRS (receiver status).
4. N = 25 for S4 (F = 4 + ⌈2 × 1.26 ms / T⌉ = 23, rounded up to a row of `L09/results/hil_s2/hil_delay_margin.txt`: nominal
   PM 71.47°, Ms 1.4465; N ≤ N_max = 249).

## Why

0046 item 2's wording named the adapter's delay line; the semantics are unchanged.

## Evidence

sim/gz/plugin/src/hil_source.cpp:472-565; lockstep.cpp:1060-1075; fw/hal/target/src/hil_main.cpp:67-72; hil.cpp:335-437.
Board: 10 s at N = 64 all checks OK (0 CRC/sequence/link faults, 0 deadline misses while streaming, command margin min
7.1 ms); 60 s at N = 25: 0 late commands, margin min 1.9 ms, but 44 s of absorbed lag on a host at load 31 (113 deadline
misses) — to be repeated on an idle host. Review PASS (flight ELF unchanged, no HIL symbol).

## Approval

Lead, 2026-10-17 (implementation of 0046; no spec or frozen-test change).
