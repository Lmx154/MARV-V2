# M1 bench card — ESC arming and motor survey (P10/P11 TB), Luis present

Setup: PROPS OFF (all four). FC on USB + debug probe as tonight. Quad on the bench, nothing within reach of the motors.
Console: `python3 tools/target/sendcon.py` (or readcon.py) on /dev/ttyACM1 (check `ls -l /dev/serial/by-id/`).

## 1. ESC arming on stop frames (P10)
1. Image running (console shows `motors ok: DShot600` and ticks/s ≈ 7497). No motor_test active.
2. Connect the 4S pack. Expect AM32's power-up tones, then its "armed" tones within a few seconds (it sees DShot 0).
3. Record: tones heard (yes/no, pattern). If it does not arm: disconnect the pack, stop, report.

## 2. Motor survey (P11) — one motor at a time, low value
For m = 1, 2, 3, 4: `motor_test m 100 2000 PROPS-OFF` (DShot 100 for 2 s; cap 500, max 3 s).
Expected (airframe card, FRD body, PX4 quad-X; view from above, nose = forward):

| logical | position | spin |
|---|---|---|
| 1 | front right | CCW |
| 2 | rear left | CCW |
| 3 | front left | CW |
| 4 | rear right | CW |

Record which physical motor turns and its direction. Wrong position → set `motor_pad_m1..m4` (pad map). Wrong
direction → AM32 reverse setting or swap two motor wires (never inverted in firmware).
If no motor turns at 100: retry at 150, then 200 (still ≤ cap). Note the lowest value that starts each motor.

## 3. Stops
1. Cancel: start a 3 s test, press any key after ~1 s → stops at once.
2. Disconnect: start a 3 s test, close the console → stops (≤ next rate tick).
3. Firmware halt: start a 3 s test, halt the core over SWD
   (`openocd ... -c init -c "targets rp2350.cm0" -c halt`) → no more frames → AM32 signal-loss stop. Time it
   (expected well under 1 s; AM32 timeout INFERRED). This covers the stalled-tick case the P11 review flagged.
4. Panic: (needs a debugger call; optional) → stop frames, motor stops.

Afterwards: disconnect the pack first, then USB. Commit the console log and these results (evidence rule).

## Results, 2026-10-16 09:18–09:53 (Luis observing, 4S pack on, props off)

Image `quad-L2-pass-147-gc52b42b` for steps 1–2, then `gc52b42b-dirty` (this commit's `motor_pad` and
`motor_test_duration_max_ms`) for the map check and step 3. Console logs: `m1_bench_2026-10-16/`.

1. ESC arming: **pass**. Luis: "yup, ESC arms. i can hear it make the arming tone it's 3 beeps that are descending".
   The board did not reset on pack connect (its USB device node was not re-created). The console log of this step was
   lost (logger stopped early).
2. Motor survey at DShot 100 (`02`–`07`), identity map: OUT0 rear left CCW, OUT1 front left CW, OUT2 rear right CW,
   OUT3 front right CCW (Luis). Every direction already matches quad-X, so no AM32 reverse. `motor_pad` set to
   `[3, 0, 1, 2]`, flashed and confirmed in one sequence (`08`; Luis: "yes good to go"). All starts at 100.
3. Stops (DShot 100, 20 s deadline; cap raised to 20 s, decision 0038 amendment):
   1. Cancel (`09`): **pass**, stopped at the keystroke (`stopped: cancel`). The cancelling byte stays in the console
      line buffer, so the next line was refused as "unknown command" (`10`, nothing spun; noticed, not changed).
   2. Disconnect (`11`): **pass**, one continuous run that ended at the port close (`stopped: disconnect`).
   3. SWD halt (`12`): **FAIL**. Core 0 halted 2.0 s into the test for 5 s, then watchdog reboot. The motor ran without
      a break until the reboot (Luis: "it was one continuous go no stopping in between at any point"). With the core
      halted the DShot PIO is stalled on its empty FIFO and all four pads read low (`13`: FDEBUG TXSTALL0 re-set
      within 200 ms, FLEVEL 0, SM0 at 0, GPIO 6–9 low ×20). So the ESC held the last throttle for ≥ 5 s with no
      signal (AM32 signal-loss behaviour INFERRED; version and settings unread). Blocks props-on (M4). Luis: both a
      firmware watchdog and the AM32 setting.
   4. Panic: not run.
