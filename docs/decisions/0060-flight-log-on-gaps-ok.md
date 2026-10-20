# 0060: The flight image logs now; its log-ring check (L-5) is a warning, and a long SD stall leaves Gap records

## What changed

`MARV_FC3V2_FLIGHT_LOG` is on by default: the flight image `marv_fc3v2` runs the onboard SD logger of unit U4-4 on the
disarmed arena (decision 0059). Every bench and HIL option turns it off.

**Loosened (this record covers only this):** invariant L-5 (core EMB-6, the log ring holds the fast stream for the worst SD
stall) on the flight image. `tools/ci/check_log_ring.py` runs there with `--warn`: a shortfall is printed as `LOG-RING: WARN`
naming this record, and the image is kept instead of removed. The arena ring is 135152 B: 0.430 s at the check's rate (the
32-byte IMU record, 314011.5 B/s), about 0.70 s of the 16-byte raw-count stream the image writes, against the design-budget
entry `sd_log_write_stall_max_s` = 0.809 s (a PLACEHOLDER). An SD stall longer than the ring covers fills it: the tick drops
the newest records and counts them, and the log holds a Gap record at the place of the loss (fw/log format.hpp). Nothing
waits on the card.

**Unchanged:** the 0.809 s entry and the frozen pins of `tests/regression/quad/L09/unit/log_ring_test.cpp`; L-5 as a build
failure on the bench log image (`MARV_FC3V2_LOG_BENCH`); the measurement build `MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN` (header flag,
console trigger, its own warning). No file under `tests/regression/` changes.

**Still holds:** L-5a (every build but the measurement build FORBIDs its emb6_open symbols, with a planted control); L-8 (one
arena owner; the logger never yields while Logging or PostRoll; the tick pushes only through the published ring pointer);
L-8b (no core-1 launch while the lease exists); L-10 (no file holding a data sector of its session is reclaimed); arming never
waits on the logger (decision 0053); the tick hooks never block (L-1); the post-link parameter-table, flash-layout and L-6
checks.

**One parameter set:** the logger's register joins `marv_params_fc3v2`, the set the flight image, the config store, the link,
the ground station (Params page, GX-13 derive) and config validation read; `marv_params_fc3v2_log` is now an alias of it.
Schema hash 0x0fd34ce78b31e33f (155 records) becomes 0xd1eb1dc8320b077d (161), the hash of U4-4's flight-log image.

## Why

Luis, 2026-10-20 (AskUserQuestion, answer "Log now, gaps OK (Recommended)"): "Merge U4-4 and U4-5 and turn the flight logger
on with its RAM check as a warning. If the SD card stalls, the log keeps Gap records instead of refusing to build. Because this
loosens a check, it needs a one-page decision record. U4-6 and U4-7 (soaks and budget) move to after first flight." His
context: "don't get stuck tuning things". The first flights need logs (decision 0059), and a log with counted gaps is better
than none; decision 0026 puts refinements after hardware contact.

**What restores L-5:** unit U4-6 (at least three 30-minute soaks of this image with the link and every telemetry stream at its
cap, EMB-3 with the hooks, the L-3 and L-4 repeats) and unit U4-7 (Luis rules the stall figure; the budget entry, the frozen
pins and L-5 as a failure on the default build follow), both after first flight.

**The gate that remains before flight:** an EMB-3 timing run on this exact image (its hash) with the logger hooks and the
telemetry streams on, folded into Luis's pending armed EMB-3 task. No flight uses this image until that run passes.

## Evidence

U4-4's measurement of this image: `tests/regression/quad/L10/results/arena/` (`flight_log_151552.txt`: arena 151552 B). The
default m33 build of this change (branch u4-log-on): arena 151552 B, free RAM 6228 B, table 161 records with the schema above,
`LOG-RING: WARN ... ring 135152 B (8447 cells, 135136 B usable) vs fast stream 314011.5 B/s x stall 0.809 s = 254035.3 B
(margin -378.6 ms)`; the parameter-table, flash-layout, L-6, L-8b and L-5a checks pass.

## Approval

Luis, 2026-10-20, AskUserQuestion answer "Log now, gaps OK (Recommended)", quoted above.
