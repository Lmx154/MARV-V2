# 0039: the hardware watchdog is deferred until after M4 (a recovery feature, not the stall stop)

## What changed

Quad spec §4.0's deferred list gains the RP2350 hardware watchdog (`watchdog_timeout_ms` 2048 ms, Luis's ruling, fed
by the rate tick). The stopped, unbuilt work is kept on branch `m1-wdt` (8b268a6, local, not merged). Nothing else in
the spec, the code or the tests changes.

## Why

The M1 bench (`tests/regression/quad/L09/results/m1-bench-card.md`, step 3.3) first recorded the SWD-halt check as a
fail (ESC holding throttle) and Luis chose "both a and b" (a watchdog and an AM32 setting) on that finding. The timed
re-run showed the reading was wrong: with the core halted the DShot line is silent and the AM32 v2.20 ESC cuts the
motor about 0.4 s later and resets (consistent with its "half second timeout when armed"; INFERRED from a source read).
So a firmware stall already stops the motors, four times faster than a 2048 ms watchdog would; AM32 v2.20 has no
setting for this (b has nothing to set). What the watchdog still adds is recovery (the board restarts instead of
staying hung, "last reset: watchdog") at the cost of rebooting on any debugger halt longer than 2 s and a flash.sh
change. MVP first (decision 0026): build the least that meets the milestone.

## Evidence

Bench logs `m1_bench_2026-10-16/12-*.log` (first run), `13-halt-pio-diag.log` (PIO stalled, pads low while halted),
`14-*.log` (timed re-run: halt 5.1 s into the test, 15 s halt, motor off at about 5.5 s on Luis's timer, reboot at
20.1 s). AM32 settings and EEPROM backups: `m1_bench_2026-10-16/am32/`. Watchdog sources (for the resumed work):
ArduPilot `libraries/AP_HAL_ChibiOS/hwdef/common/watchdog.c:29` (2048 ms); pico-sdk `hardware/watchdog.h` (RP2350
maximum 16777 ms).

## Approval

Luis, 2026-10-16: "yes use 2048. we can always change this later when we start refining." and, on the recommendation
to stop the watchdog work and defer it as a recovery feature, "your recommendation is fine with me go ahead".
