# Bench motor-step profile: props-off TB runbook (unit U3 BENCH-PROFILE)

This is a bench runbook and evidence, not a frozen test. It has not been run yet (host evidence: the L09 unit_quad_fc_bench_profile tests, the eight planted defects killed, the L-6 post-link check and its negative controls). Luis runs it with the lead, with the props
off. The image is `marv_fc3v2_bench_profile` (fw/hal/target/CMakeLists.txt). It is the flight composition with the bench
motor-step profile in place of the mixer's output while Armed: the composition variant `MARV_QUAD_FC_BENCH_PROFILE`
(fw/compositions/quad_fc/src/quad_fc.cpp, header comment).

## The profile and its kills

The values come from fw/compositions/quad_fc/params/bench_profile_register.yaml: Luis's rulings of 2026-10-18, with the
40 s cap "40 s cap, all 4 motors (Recommended)".

- The arm edge is the rate tick on which the arm switch (ch5) arms.
- The profile runs only if the bench switch (ch6) is fully up on that tick: +100, at or above 1700 µs, normalized threshold
  0.390625.
- Timeline after the arm edge:
  - 2 s lead-in: every motor stops.
  - Then motor 1, 2, 3 and 4 in turn (logical quad-X: 1 front right, 2 rear left, 3 front left, 4 rear right). Each motor
    runs three repeats of DShot 200, 450, 200 at 1 s per level (9 s per motor). Only one motor spins at a time.
  - At 38 s every motor stops and stays stopped until the next arm. The profile never repeats within one arm.
  - Hard stop at 40 s at the latest (the stamp clock).
- The bench cap is DShot 450 (init refuses a level above it). The console cap stays 500.
- Kills, each evaluated in the rate tick, stopping every motor on the tick it is seen and latching until the next arm edge
  (the lead, 2026-10-18), even if the cause goes away:
  - ch6 leaves fully up: the middle and down positions are out of position.
  - ch5 off: the existing disarm.
  - The link age reaches failsafe_loss_us = 100 ms (ArmedHold).
  - Any other arming state that is not Armed: centred at 300 ms, disarm (Failsafe) at 1.5 s.
- Kill latency is at most one ELRS frame (8 ms at 125 Hz) plus one rate tick (267 µs).

The image has no config store and no link service. It boots the compiled defaults (console:
`params ok: generated defaults of the bench-profile set (no config store)`), so changing any bench value needs a new image
with a new hash. It has no onboard logger yet; the logger joins it with unit U2.

## Preconditions

- **Props off all four motors.** Luis checks them by eye before the ESC battery is connected.
- The ESC is powered from the 4S pack only with Luis present. The frame is held or strapped.
- For this props-off rehearsal the USB console is connected: it is the evidence. The no-USB use is the later props-on run.
- The exact image is identified by both hashes below. Refuse any other image:
  - Built from the code commit c474f41 (the version string in the image reads `quad-L2-pass-442-gc474f41`; it is part of the
    hash, so a rebuild at any other commit gives other hashes: flash the archived ELF, do not rebuild).
  - `sha256sum marv_fc3v2_bench_profile.elf` = `ed2191fa2856487d68938931d4b8401540876c21b7455861f8f9a5f9b7ba6cf4`
  - `arm-none-eabi-objcopy -O binary marv_fc3v2_bench_profile.elf x.bin; sha256sum x.bin` (143736 bytes) =
    `90c9e15e5de4504b9a46beab53a5ba91ae0c30ad1206766ad36f6d7e4402c29a`
  - The `.bin` hash must equal the console's boot line `image: sha256 <...>`, which is the board's own SHA-256 of the
    flashed image (flash_image.cpp, `__flash_binary_start` .. `__flash_binary_end`).
- Build: `cmake --preset m33 && cmake --build --preset m33 --target marv_fc3v2_bench_profile`. The build fails if the
  image misses its parameter table, its flash layout, or the L-6 control (the image must hold the profile's symbols).
- Flash: `MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh <the archived marv_fc3v2_bench_profile.elf>` (flash.sh prints the ELF's sha256: compare it with the one above).
- Capture the console for the whole session into `raw/`, for example
  `python3 tools/target/readcon.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00 600 > raw/tb.console.log`.
- Radio: RadioMaster Pocket on, ch5 down, ch6 middle, throttle low.
- Expected boot lines:
  - `config: factory (bench-profile image: no config store), status ok`
  - `params ok: generated defaults of the bench-profile set (no config store)`
  - `init ok`
  - `bench_profile status: phase=idle (no arm edge yet) arms=0 ...` once per second
  - `est_valid=1` on the stats line, after the board has been still for the gyro calibration.

Every `bench_profile` line has the form `bench_profile <event|status>: phase=<...> arms=<n> level=<0..35> motor=<1..4>
dshot=<value> stop_state=<arming state on the stop tick> | ch6=<value> <in|out of> position (threshold 0.390625, rc
frames <n>) | arming=<state>`. An `event` line is printed as soon as the phase or the arm count changes; a `status` line
once per second. `motor=0 dshot=0` means the latest rate tick wrote stop.

## Steps

0. **ch6 readout. Do this first, with the ESC battery unplugged.** Read the `bench_profile status` lines with ch6 in each
   position:
   - up: `ch6=+1.000 in position`
   - middle: `ch6=+0.000 out of position`
   - down: `ch6=-1.000 out of position`

   If up reads `-1.000` or `out of position`, **stop and do not arm**. The direction of ch6 is INFERRED from ch8 (up =
   +100). A wrong direction needs a new image (threshold rule) and the lead.
   Then put ch6 in the middle, connect the ESC battery (props off), and wait for `est_valid=1`.
1. **Arm without ch6 up: no spin.** With ch6 in the middle, arm on ch5.
   - Expected: `bench_profile event: phase=killed: bench switch not in position (stop until the next arm) arms=1 ...
     stop_state=armed | ch6=+0.000 out of position ... | arming=armed`.
   - No motor spins.
   - Raise ch6 to up while still armed: still no spin, the phase stays `killed: bench switch not in position`, and
     `motor=0 dshot=0` on every status line.
   - Disarm (ch5 down): `arming=switch_off`.
   - Repeat with ch6 down.
2. **Arm with ch6 up: the profile runs.** ch6 up, then arm on ch5.
   - At the edge: `event: phase=lead-in (stop) arms=<n+1>`. Nothing spins for 2 s.
   - Then `event: phase=running`. Status lines show:
     - motor 1 for 9 s at `dshot=200`, `450`, `200`, three times (levels 0 to 8);
     - then motor 2 (levels 9 to 17), motor 3 (18 to 26) and motor 4 (27 to 35), each the same way.
   - Check by eye that one motor at a time spins, in the order front right, rear left, front left, rear right.
   - At about 38 s: `event: phase=done (stop until the next arm)`. Everything stops and stays stopped while ch5 stays armed.
   - Then disarm.
3. **Arm switch off mid-step: stop.** ch6 up, arm, and wait until a motor spins (for example motor 1 at `dshot=450`,
   about 3 to 4 s after the arm). Put ch5 down.
   - Expected: the motor stops at once.
   - `event: phase=killed: not armed (stop until the next arm) ... stop_state=switch_off ... | arming=switch_off`.
   - Arming again (ch5 up) starts a new profile from the lead-in, with `arms` one higher.
4. **Flip ch6 down: stop.** ch6 up, arm, and wait until a motor spins. Flip ch6 to the middle or down.
   - Expected: stop at once.
   - `event: phase=killed: bench switch not in position ... stop_state=armed | ch6=... out of position | arming=armed`.
   - Flip ch6 back up: still no spin (latched).
   - Disarm.
5. **Transmitter off: stop at about 100 ms, disarm at 1.5 s.** ch6 up, arm, and wait until a motor spins. Switch the
   RadioMaster off.
   - Expected: the motor stops about 100 ms later:
     `event: phase=killed: not armed ... stop_state=armed_hold | ... | arming=armed_hold`.
   - The stats line's `arming=` then reads `armed_failsafe_centred` (from 300 ms), then `failsafe` (from 1.5 s). Nothing
     spins throughout.
   - Transmitter back on: no spin and no arm until ch5 is cycled (down, then up, after 0.5 s of link).
   - Optional 5b: switch the transmitter off, then on again within 1 s. `arming=armed` comes back, but nothing spins
     (latched) until a new arm edge.
6. **Decode the log.** This image has no onboard logger until unit U2 links it, so the TB's evidence is the console
   capture in `raw/`. Once U2 lands, decode the session with `uv run python tools/log/mlog.py` and check:
   - the rate-tick records' DShot and gate state match the console's phase changes;
   - the profile's level edges fall on the ticks the L09 test `TheProfileMatchesTheRulingToTheTickAndStopsForGoodAtTheEnd`
     requires.

After the TB, commit the console capture under `raw/` and the result table to this file, then flash the flight image back
(`build/m33/fw/hal/target/marv_fc3v2.elf`) and check `est_valid=1`.

## What guards this image off the board

- **L-6 (no flight image contains the bench profile).**
  - After every link, `tools/ci/check_bench_profile.py` reads the symbols:
    - The flight image `marv_fc3v2` must have no symbol naming `bench_profile`, or the build fails and the image is removed.
    - The bench image must have one (the negative control).
  - On the host, the L09 ctest entries `bench_profile_symbols` and `bench_profile_symbols_negative_control` do the same for
    the archives and the SIL library.
  - `bench_profile_param_ids` checks the bench set: it keeps every FC3v2 record and adds only `bench_profile_*` ids.
- **L-7 (the gate).** `tests/regression/quad/L09/unit/quad_fc_bench_profile_test.cpp` (unit_quad_fc_bench_profile) holds
  the gate. Mutation check (2026-10-19, one planted defect in quad_fc.cpp at a time, each run of the 18 tests; the clean
  tree passes 18 of 18):

  | Defect | Killed by (failing tests) |
  | --- | --- |
  | ignore_switch (the gate ignores ch6) | 6: the ch6 middle, down, just-below and NaN kill sweeps, the no-ch6 arm test, the ruling's-clock kill test |
  | cap_off (bench cap and console-cap refusals removed) | 2: the validate rejection test, the init-panic death test |
  | repeats (the profile wraps within one arm) | 3: the sequence-to-the-tick test, the once-per-arm test, the stamp test |
  | wrong_order (motors interleaved level by level) | 10: the sequence test, all kill sweeps, once-per-arm, stamp |
  | latency_plus_one (ch6 read one rate tick late) | 5: the four ch6 sweeps, the ruling's-clock kill test |
  | no_latch (the profile resumes when Armed returns) | 2: the link-age sweep, the ruling's-clock kill test |
  | armed_hold_allowed (ArmedHold and centred keep writing) | 2: the link-age sweep, the ruling's-clock kill test |
  | stamp_cap_off (only the tick clock ends the profile) | 1: the stamp test |
