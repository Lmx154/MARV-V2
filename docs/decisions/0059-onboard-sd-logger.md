# 0059: The onboard SD flight logger (units U1-U5) and its frozen-test extensions

## What changed

Quad §4.0 M4 gains an onboard SD flight logger (Luis, 2026-10-18): every IMU sample, every rate tick's DShot command and
arming/gate state, every new receiver frame, a periodic slow-state record (the unit-11a telemetry snapshot), status, gap and
end records, in 512-byte self-validating sectors behind a header with the image, config and schema hashes and the parameter
table. The tick only pushes into a wait-free ring; the background loop writes a preallocated contiguous file with no FAT
metadata while armed. Logging runs from arm (plus a derived pre-roll) to disarm plus a post-roll; arming never depends on it
(decision 0053: SD never in the boot or arming path). Design: Serena memory `quad-m4/architect-onboard-logger-2026-10-18`.

This record covers no loosening, deletion or spec change; core EMB-6 holds (ring >= rate x the worst measured stall, a ctest
with negative controls). It names the frozen files the logger extends:
tests/regression/quad/L09/param_ids
tests/regression/quad/L09/unit/CMakeLists.txt
The first gains one appended line for the new design-budget entry `sd_log_write_stall_max_s` (EMB-6's worst stall, 0.809 s
from tests/regression/quad/L10/results/sd/README.md, PLACEHOLDER until the U2 soak); the second appends the registration of
the new logger tests (unit_log_ring, unit_log_logger, log_roundtrip_*). No existing test, threshold or predicate changes.

## Why

Luis, 2026-10-18: "what we should probably work on is creating a logging system for when this happens, because i can't
realistically have a wire dangling connected to the computer while a drone this powerful is spinning up hard". The M4
done-when asks for each test with its log; the strapped-down motor test and the first flights need logs to tune from.

## Evidence

Unit U1 (branch log-core): L-1 writer-never-waits and concurrent stress, L-3 fault injection plus a static no-panic check with
its control, L-4 decoder cuts, torn CRC and stale sessions, L-5 EMB-6 sizing with negative controls, L-9 no metadata call
while logging, and a C++ to Python round trip.

## Approval

Luis, 2026-10-18 (AskUserQuestion): bench switch "ch6 fully up (Recommended)"; motors "One at a time (Recommended)"; profile
"Approve as proposed"; eRPM "Yes, pull it forward (Recommended)" (decision 0057). The logger itself follows his request
quoted above; the lead accepted the architect's design.

Unit U2b (board soak and fault evidence, 2026-10-19) edits two files of the U2a bench record, adding evidence only:
tests/regression/quad/L10/results/log_bench/README.md (the U2b section) and
tests/regression/quad/L10/results/log_bench/log_bench.py (--dump-timeout and the backlog readout). No test or threshold changes.
