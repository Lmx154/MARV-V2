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
