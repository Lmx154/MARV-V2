# 0057: eRPM pulled forward into M1

## What changed

Bidirectional DShot eRPM decode on the FC3v2 target (the eRPM item of L9) leaves the quad §4.0 deferred list and joins
M1, so the board reads each motor's speed from the ESC. In `docs/spec/10-quad-flight-software.md`:

- §4.0 M1, the DShot bullet: "PIO DShot600 output. eRPM decode and the RPM notches wait for the L6 resume; the gyro chain
  runs its low-pass stages." becomes "PIO DShot600 output with bidirectional DShot eRPM decode on the FC3v2 target (pulled
  forward, decision 0057; checked by L9's pass bar: the decode matches an independent speed reference). The RPM notches
  wait for the L6 resume; the gyro chain runs its low-pass stages."
- §4.0 deferred list: "the rest of L9 (eRPM, core split, SRAM placement if timing holds without them)" becomes "the rest
  of L9 (core split, SRAM placement if timing holds without them; eRPM moved into M1, decision 0057)".

Not changed: the RPM notches and the rotor-speed source for the gyro chain stay deferred; feeding the board's rotor
speed to the notch set re-opens the configuration-set question and needs its own ruling and record. Core EMB-6 already
lists eRPM in the log content, so the logger needs no amendment.

## Why

Luis, 2026-10-18, AskUserQuestion, asked whether the board should read each motor's speed from the ESC (bidirectional
DShot): "Yes, pull it forward (Recommended)".

- The motor time-constant measurement on the strapped-down bench needs it: tau from IMU vibration is at best INFERRED,
  and eRPM is the method that counts (unit E1 of the onboard logger ruling; spec S2, props on, strapped down, needs eRPM).
- RPM filtering later needs it: the board has no rotor-speed source today, so every notch is bypassed (rate-response
  ruling).

## Evidence

Serena memories `quad-m4/architect-onboard-logger-2026-10-18` (unit E1, "eRPM is a separate unit, only after Luis pulls
it forward") and `quad-m4/architect-rate-response-2026-10-18` (measurement 2; step 3, the notch set). Spec: quad §2
(SC-HW1, bidirectional DShot a hard requirement), L9 pass bar and Freezes (DShot and eRPM tests), core EMB-6.

## Approval

Luis, 2026-10-18, AskUserQuestion answer "Yes, pull it forward (Recommended)", quoted above.
