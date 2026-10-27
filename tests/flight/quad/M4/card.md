# M4 flight test card (rulings recorded 2026-10-26)

Quad spec §4.0 M4: "A flight test card (pre-flight checks, tethered hover, free hover, angle, acro, abort criteria)".
Done when: "tethered hover in angle mode, then free hover, then acro, each with its log and card update committed". Core
T7: "the vehicle, tethered first; run from a flight test card" (`docs/spec/00-core-contracts.md:375`).

Status: rulings recorded 2026-10-26 (section 6); nothing flown. Draft written 2026-10-20 from `origin/m4` 6aabd62. Luis
ruled U-1 to U-14 and G-6 on 2026-10-26 (Luis 2026-10-26, "all yes"). A safety number that no ruling answers still reads
`UNKNOWN — Luis` (CLAUDE.md, UNKNOWN rule). Luis fills the card and signs each run. Nobody flies from a line that reads
UNKNOWN.
Rules that apply to every run: no ESC power and no motor spin without Luis present (`docs/handoff.md`, Rules).

Vehicle (card `vehicles/marv_7in_fc3v2.yaml`): HGLRC MY7 7-inch, Emax ECO II 2807 1700KV, HQProp 7x3.5x3, Goku G55M
AM32 ESC, madflight FC3v2 Rev-C, Ovonic 4S1P 1600 mAh pack (card header, lines 4-8). All-up mass 0.732 kg, UNVERIFIED
and PLACEHOLDER (card lines 15-23).

## 1. Gates before any flight

State from the evidence in the repo on 2026-10-20, updated 2026-10-26 with the rulings and the flashed image. DONE needs a committed record; where I found none it is OPEN.

| # | Gate | Needed before | State | Recorded in / evidence |
| --- | --- | --- | --- | --- |
| G-1 | CH8 mix on the RadioMaster (angle switch = CH8, up = angle) | S4b, all flights | OPEN | Luis chose "Radio: CH8 mix" (Serena `session-handoff/2026-10-19`, Owner redirect); `docs/handoff.md:33`. His radio drove ch14 on 2026-10-19. |
| G-2 | HIL S4b flown in ANGLE from the GCS Bench page, verdict saved | (a) first hover | OPEN | `docs/handoff.md:33,42`; runbook `tests/regression/quad/L10/results/gcs/gx10/RUNBOOK.md` (steps 1-11); GX-10 README: "The live S4b run ... is not here". Its "Board CDC" image check is INFERRED until run. Verdict bar = Luis's verdict (U-6, Luis 2026-10-26, "all yes"). |
| G-3 | CG check of the flight-ready quad (balance point against the motor centroid) | (a) first hover | OPEN | `docs/handoff.md:42`; architect memory `quad-m4/architect-rate-response-2026-10-18`: about 3 deg liftoff tilt per mm of offset (INFERRED model). The card assumes CG at the rectangle centre (card lines 129, 147, 165, 183). Ruled: CG within about 1 mm of the motor-rectangle centre (U-6, Luis 2026-10-26, "all yes"). |
| G-4 | Recorded weighing of the flight-ready quad | (a) first hover | OPEN | Card lines 15-23: the 0.732 kg is a sum of 696 g weighed and 36 g added by hand; its `check` asks for a weighing with pack and props on. Ruled: one scale reading with pack and props on, recorded (U-6, Luis 2026-10-26, "all yes"). |
| G-5 | Armed fault-free run on the flight image (radio arm, ESC unpowered, GCS with every telemetry stream at its cap, logger logging, about 2 min) | every flight | OPEN | Decision 0060 amendment 2026-10-26 (Luis: "A"): passes when the board stays armed (no ImuStale disarm, no panic) and rate/attitude `timing_faults`, `motor_drops` and IMU `ovr` do not increase between arming and disarming. Margin INFERRED about 35 % of T (0060 amendment); the measured combined EMB-3 moved to U4-6. The 2026-10-26 flight image showed rate `timing_faults=1` since boot, not growing (old image 0): counts are read at arming. |
| G-6 | Props-off angle-mode motor-direction check on the flight image: tilt the quad by hand, check each motor responds in the right direction, ESC powered, props off. The full bench-profile props-off TB moves to before (c) | (a) first hover (the check); (c) ACRO (the full TB) | OPEN | Ruled (G-6, Luis 2026-10-26, "all yes"). `docs/handoff.md:40`; `tests/regression/quad/L10/results/bench_profile/README.md` (runbook, no `raw/` committed); `tests/regression/quad/L09/results/m4-bench-card.md:18-21` (angle motor response "still to do with Luis present"). |
| G-7 | Idle speed measured, then `tau_max_hover` regenerated | (c) props-on ACRO | OPEN | `docs/handoff.md:22-23`, decision 0058 ("Open: gates the first props-on acro flight"). The card's idle speed is a PLACEHOLDER (f_min = 0), so `tau_max_hover` is non-conservative. |
| G-8 | Bifilar pendulum (inertia), motor tau, DShot-to-thrust map; card updated and gains regenerated | (c) ACRO and the done-when; not before (a) tethered hover nor (b) free hover, which fly on the current card | OPEN | Ruled (U-5, Luis 2026-10-26, "all yes"). Spec §4.0 M4 bullet 1: "A tethered first hover may use the current card." `docs/handoff.md:40`. |
| G-9 | QF-3 margins pass on the updated card | the M4 done-when | OPEN | Passing on the current card per decision 0052 Evidence (rate PM 45.0000674 deg, Ms 1.9912204; limits 45 deg, 2.0), not re-run by me at 6aabd62. Not final until G-8 updates the card. |
| G-10 | ESC set to KV 1700 and 14 poles (AM32) | any props-on spin | OPEN | Serena `hardware/vehicle-build`: the configurator showed KV 820; set to 1700 and 14 poles "affects AM32's low-RPM throttle limiter". Ruled: Luis sets and confirms 1700 KV and 14 poles in the AM32 configurator; screenshot in the run folder (U-2, Luis 2026-10-26, "all yes"). Not yet recorded as done. |
| G-11 | Image and config validated (G7) | every flight | NOT REQUIRED (U-3) | Ruled (U-3, Luis 2026-10-26, "all yes"): the factory config flies; no validation report needed. Console must read "config: source factory, status ok". A stored config not bound to this image refuses arming; the factory config does not (`fw/arming/include/marv/arming/arming.hpp:31-33`). Core G7 (`00-core-contracts.md:442`); the GCS Identity indicator shows a report field ("none" when no report is bound, `gcs/src/lib/state/shell.ts:143-150`). |
| G-12 | Flight image chosen, built, flashed, hashes recorded | every flight | DONE 2026-10-26 | Flight image built from 3531c3f, banner `marv_fc3v2 quad-L2-pass-664-g3531c3f`, elf sha256 dfb643ef662ba9b3e7200eb817d7ba1239c8ff8693142cbdc3f13a4b8268c899, image sha256 (console) 345f95855466d73098cf4280b88799adf6b096ff39a0c67248a2c5ba667b2222, factory config ok, logger ON. Evidence: `docs/handoff.md`. See 2.G. |
| G-13 | Safety equipment and tether | (a) | OPEN | Ruled (U-1, Luis 2026-10-26, "all yes"): about 1 m of non-stretch line to a heavy anchor; flat hard ground; pilot well outside the tether radius; a spotter if one is available. Spec §4.0 M4 bullet 3 says "the stand and safety gear" have not arrived (Luis, 2026-10-17). Nothing in the repo records that they have. |

G-1 and G-2 are Luis's physical tasks; G-5, G-6 and G-7 need him present at the bench.

## 2. Pre-flight checklist

Tick each line on the day, with the date and initials. A line that cannot be ticked is a no-fly.

### A. Image and config identity
1. Connect the GCS (`~/marv-gcs-start.sh`). Read the status bar Identity indicator: image SHA-256 (first characters,
   full value on hover), config SHA-256, schema hash, source, state, last activation, report, staged
   (`gcs/src/lib/state/shell.ts:132-150`). State must read `ok`; a refused config does not arm
   (`gcs/src/lib/state/shell.ts:88`).
2. The USB console banner is `marv_fc3v2 <git describe>` (`fw/hal/target/src/main.cpp:260`) and reads
   `marv_fc3v2 quad-L2-pass-664-g3531c3f` (G-12). Record it and the `image: sha256 ...` line. The version has no `-dirty`.
3. The console `image: sha256` equals 345f95855466d73098cf4280b88799adf6b096ff39a0c67248a2c5ba667b2222 (G-12; elf sha256
   dfb643ef662ba9b3e7200eb817d7ba1239c8ff8693142cbdc3f13a4b8268c899). Config source is `factory`, status `ok`
   (U-3, Luis 2026-10-26; `tests/regression/quad/L10/results/telemetry/README.md:28`).
4. Console `est_valid=1` (`fw/hal/target/src/main.cpp:362`).

### B. Airframe and motors (battery unplugged)
5. Props off. Motor numbering is logical 1-4, array index = number - 1 (CLAUDE.md). Card: 1 front-right, 2 rear-left,
   3 front-left, 4 rear-right; spin 1 and 2 counter-clockwise, 3 and 4 clockwise, viewed from above (card lines 129-189;
   Luis 2026-10-16, PX4 quad-X).
6. Motor test, one motor at a time: `motor_test <1-4> <dshot> <ms> PROPS-OFF`, then `stop` (decision 0038). Check the
   pad and the spin direction by eye. A mismatch is fixed in the ESC setting or the mapping, never in code (card line 135;
   spec L9 pass bar). Do not run it with props on.
7. Props on after step 6: HQProp 7x3.5x3, two clockwise and two counter-clockwise (Serena `hardware/vehicle-build`),
   the clockwise pair on the motors that spin clockwise. Nuts tight. Replace any chipped, cracked or bent prop (U-9, Luis 2026-10-26).
8. Weigh and CG: G-3 and G-4 done and recorded. Frame and arms intact, wiring clear of the props.

### C. Radio
9. RadioMaster Pocket on ExpressLRS; AETR: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw, ch5 arm, ch8 angle
   (`quad_fc_register.yaml:58-77`; `rc_channel_angle` = 8 is Luis's ruling, decision 0048). Check each stick on the GCS
   live view (stick, arm and angle channels are validated distinct, GX-6).
10. Arm switch: off at -100, on at +100; on means at or above 1700 us (`arm_switch_threshold` 0.390625,
    `quad_fc_register.yaml:47`). Angle switch: up = angle, same threshold (`:77`). Confirm CH8 shows angle up and acro
    down (G-1).
11. Receiver failsafe set to "No Pulses", so a lost link reaches the board as no frames
    (`tests/regression/quad/L09/results/m2-bench-card.md:7`). Confirm in the ELRS/EdgeTX setup.

### D. Arming (what the firmware already enforces)
Arming needs, on one rate tick (`fw/arming/include/marv/arming/arming.hpp:12-17`): an off-to-on edge of ch5; a link valid
for `failsafe_recovery_us` = 500000 us (`quad_fc_register.yaml:45`); throttle below `arm_throttle_min` = 0.060546875 (1050 us,
`:49`); no IMU deadline miss since boot; valid, unsaturated IMU samples; gyro calibration complete (`gyro_cal_duration_us`
1250000 us, `:95`; the quad must be still, `m3-bench-card.md:32-34`); estimate valid; tilt below `arm_small_angle` =
0.4363323129985824 rad (25 deg, `:99`). A switch already high at power-up does not arm; cycle it.
12. Power up with the quad level and still. Wait for the gyro calibration. Arm refusals name themselves (`arming=` on the
    console, the Arming indicator in the GCS): `calibrating`, `not_upright`, `throttle_up`, `switch_not_cycled`, `no_link`,
    `estimator_invalid`, `imu_invalid`, `saturated` (`arming.hpp:24-30`, names at `main.cpp:296-324`).
    Clear the cause; never bypass it.
13. Airmode: the rate integrator is held at zero until the throttle first passes `airmode_activate_throttle` = 0.29296875
    (1288 us) after arming (decision 0050, `quad_fc_register.yaml:51`). The INFERRED hover stick is 0.3223 (decision 0052
    item 5; mass and k_T are not final), close above it. Raise the throttle slowly through this region.
14. Armed at zero throttle the motors idle at DShot 48, the mixer floor (decision 0044 Approval; "spins at 48" is INFERRED).
    Props on means props turning at arm.

### E. Link-loss behaviour as implemented
Decisions 0040 and 0041 (Luis: "1. yes 2. yes"); parameters `quad_fc_register.yaml:39-46`; machine `arming.hpp:6-11`.
- 100 ms without a valid frame: link lost, last sticks held (`ArmedHold`).
- 300 ms: roll, pitch, yaw and throttle set to 0 (`ArmedFailsafeCentred`). In ANGLE this is level attitude at throttle 0, so
  the motors fall to the mixer floor and the quad loses lift (my reading of `quad_fc.cpp:41-46` and `arming.hpp:7-8`).
- 1.5 s: disarm (reason Failsafe), stop written to all motors. A link back before 1.5 s returns live sticks at once.
- After a failsafe disarm: re-arm needs 0.5 s of valid link and the ch5 cycled off and on.
- Bench result: 100.4 / 300.5 / 1500.4 ms measured (`m2-bench-card.md:35-36,58`).
- The angle/acro mode channel holds its last value through the hold (decision 0043 item 2).
- No battery monitor is read by the firmware (no INA226 code in `fw/` by grep). There is no low-battery failsafe.
15. The failsafe was walked props-off with the 4S pack on an older image (`m2-bench-card.md:57-59`). Whether to repeat
    it on the chosen flight image before flying: yes, folded into the G-6 props-off angle check (props off, ESC powered,
    arm, transmitter off: motors to idle at about 300 ms, stop at 1.5 s) (Luis 2026-10-26, "1. yes").

### F. Battery
16. Pack: Ovonic 4S1P 1600 mAh 120C, 14.8 V nominal, 23.68 Wh, 172 g (spec §4.0 M4; card header). Measure the voltage
    on a checker before each flight and record it. Start at 4.1 V/cell or more. End the session when a resting reading
    reaches 3.7 V/cell (14.8 V). No swollen or damaged pack (U-7, Luis 2026-10-26).
17. Pack strapped, connector seated, the CG unchanged since G-3 (the card says "battery is mounted on top").

### G. Logging (the done-when asks for a log per run)
The flight logger is ON by default (`docs/decisions/0060-flight-log-on-gaps-ok.md`; `MARV_FC3V2_FLIGHT_LOG` option at
`fw/hal/target/CMakeLists.txt:186`). The flashed image's console reads "tick quad_fc + onboard SD flight log
(MARV_FC3V2_FLIGHT_LOG)". The L-5 ring check is a WARN: ring 135152 B against 254035 B for a 0.809 s stall, so a longer
SD stall leaves Gap records (gaps allowed, decision 0060). U-4 ruled (Luis 2026-10-26, "all yes"): the onboard flight
logger is the per-run log; no cable.

## 3. Sequences

Order: (a) then (b) then (c). A run starts only when its gates and every checklist line are ticked, and the previous run's
log and card update are committed. Mode for (a) and (b) is ANGLE (ch8 up). Tilt is bounded by
`angle_tilt_max` = 1.0471975511965976 rad (60 deg, `fc3v2_scenario_values.yaml:94`). The throttle stick is the collective; there is no altitude hold in this build.

### (a) Tethered hover in ANGLE
Gates: G-1 to G-5, G-6 (the angle-mode motor-direction check only) and G-10 to G-13. G-7 to G-9 are not required for this run
(U-5, Luis 2026-10-26; spec §4.0 M4: "A tethered first hover may use the current card"; G-7 gates props-on ACRO).
Set-up: flat hard ground; area clear; about 1 m of non-stretch line to a heavy anchor; pilot well outside the tether radius;
a spotter if one is available (U-1, Luis 2026-10-26). Calm air only (U-8, Luis 2026-10-26).
Steps:
1. Checklist A to G done. Props on. Quad on the pad, nose direction noted. Radio: ch5 off, ch8 up, throttle low.
2. Power up, wait for the gyro calibration, check the Arming indicator reads `switch_off`.
3. Pilot and observer positions and the abort call-out agreed (pilot well outside the tether radius, U-1).
4. Arm (ch5 on). Idle dwell after arming 5 s (U-8, Luis 2026-10-26). Motors even, no twitch.
5. Raise the throttle slowly through the airmode threshold (0.293) to just past the hover stick (about 0.32), lift off
   inside the tether to 0.3 to 0.5 m (U-8), hold a low hover, sticks centred, then small roll and pitch inputs.
6. Land, throttle down, disarm (ch5 off), then unplug the pack.
Observe: whether the quad lifts at the INFERRED hover stick (0.3223) or not; level drift at centred sticks (CG);
attitude after a small stick input and release; any oscillation or twitch; motor and ESC temperature by hand after the
run; pack voltage after. Runs of 60 s or less; end the session when a resting reading reaches 3.7 V/cell (14.8 V) (U-7, U-8). Stop if a motor or
the ESC is too hot to keep a finger on (U-9).
Evidence to commit: section 5. Pass condition (U-10, Luis 2026-10-26): lifts off near the expected hover stick (about
0.32), holds level at centred sticks without growing drift or oscillation, returns level after small inputs, lands and
disarms cleanly. Nearest sourced references, not flight bars: the SIL angle-hover predicate (spec §4.0 M3 lines 226-229: true tilt settles within s·Delta + E + `attitude_tilt_error_max`,
1.2 deg, `design/budget.yaml:284`) and the HIL S4b verdict. These are simulation, not flight bars.

### (b) Free hover in ANGLE
Gates: all of (a) passed and committed. G-8 is not required (U-5, Luis 2026-10-26).
Set-up: no tether; flat hard ground; a spotter if one is available (U-1); calm air only; height 2 m or less (U-8). Area size
and clearance from people and objects: UNKNOWN — Luis (U-1 rules the tether radius only).
Steps: 1-4 as (a). 5 lift off to a low hover (2 m or less, U-8), hold sticks centred, then gentle inputs in each
axis, then land and disarm.
Observe: as (a) without the tether; drift; return to level after inputs; behaviour as the pack drops (the firmware has no
battery limit).
Pass condition (U-10, Luis 2026-10-26): the same as (a) without the tether. Evidence to commit: section 5.

### (c) ACRO
Gates: (b) passed and committed; G-7 (idle speed measured, `tau_max_hover` regenerated); G-8 (U-5); the full bench-profile
props-off TB (G-6); G-9 for the done-when;
`rate_ff_clamp_k` = 0.5 stays (Luis 2026-10-19, decision 0058). The clamp limits the feed-forward at hover collective;
yaw is the tight axis (`tau_max_hover` yaw 0.0861 N·m, roll 0.9153, pitch 0.8040, decision 0058; INFERRED card inputs).
Set-up: as (a): the first ACRO flight is tethered (U-12, Luis 2026-10-26).
Steps: 1-4 as (a). Take off in ANGLE (ch8 up), hover, then switch to ACRO (ch8 down) at hover with sticks centred. The
switch is continuous (decision 0058) and the rate integrator is not reset (decision 0043 item 3). Small roll and pitch
rate inputs only; return to ANGLE (ch8 up) and land. First ACRO flight (U-13, Luis 2026-10-26): small roll and pitch inputs (about 20 % stick), tiny yaw, no flips (the card's
roll rate is Betaflight's 670 deg/s at full stick, `fc3v2_scenario_values.yaml`).
Observe: response to a small rate step against the SIL values, which are references and not requirements (decision 0058
Evidence: roll step 100 deg/s, t63 118 ms in SIL, overshoot 6.1 %; flick peak 7 ms after release); Luis's HIL ACRO
verdict was "extremely hard to control ... sluggish" before feed-forward (decision 0058, Why): note whether this one
feels different. Compare yaw, which is authority-bound (tau_ref 0.310547 s).
Pass condition (U-10, Luis 2026-10-26): small rate steps stay controllable, back to ANGLE, land. Evidence to commit:
section 5.

## 4. Abort criteria and actions

Actions, as implemented:
- **Disarm: ch5 off.** The machine disarms on that rate tick and writes stop to every motor
  (`arming.hpp:11`, `quad_fc.cpp:46`, decision 0040 I2). This is the kill. The quad then falls, the tether limits it.
- **Throttle to zero is not a kill.** Armed at zero throttle the motors idle at DShot 48 (checklist 14); with the airmode
  latch set the rate integrator is no longer zeroed (decision 0050).
- **TX off** starts the failsafe of section 2.E: the quad falls to idle at 300 ms and disarms at 1.5 s. It is a last resort.
- The mode switch is not an abort action. Do not switch ANGLE to ACRO to recover (U-12, Luis 2026-10-26: no). The first ACRO
  flight is tethered.
- After an abort: disarm, pack out, no re-arm in the same session until Luis has looked at the cause and the log, and has
  signed the card line. Re-arm after a failsafe needs the ch5 cycle (decision 0041).

Abort triggers. The call "abort" is Luis's or the spotter's. The firmware triggers that need no one are the machine's
(link loss, IMU deadline miss, ch5 off). Human triggers are visual calls with no numbers; the action is disarm on ch5
(U-11, Luis 2026-10-26, "all yes"):

| Trigger | Applies to | Threshold |
| --- | --- | --- |
| Lift-off tilt on the pad or first metre (CG offset) | (a) | visual call, no number: the quad tips instead of rising level; throttle down and disarm, then recheck the CG before the next try (Luis 2026-10-26, "2. recommended") |
| Tilt or drift growing with centred sticks | all | visual call: abort (U-11) |
| Oscillation or twitch, or a rate of attitude change that is not the pilot's | all | any: abort (U-11) |
| Tether going taut | (a), (c) | visual call: abort (U-11) |
| Unexpected motor sound, vibration, smell, smoke | all | abort on any (U-11) |
| Loss of sight, or someone entering the area | all | abort (U-11) |
| Low battery: pack voltage in the air (no onboard reading) or elapsed flight time | all | runs of 60 s or less; end the session at a resting reading of 3.7 V/cell (14.8 V) (U-7, U-8) |
| Wind | all | calm air only (U-8) |
| Motor or ESC too hot to keep a finger on | all | stop (U-9) |
| ARM state other than `armed` in flight (`armed_hold`, `armed_failsafe_centred`) | all | any occurrence: abort (U-14, Luis 2026-10-26, "all yes") |
| Airframe contact with the ground in free flight, a bounce, or a prop strike | all | no number; abort and inspect (Luis to confirm) |

## 5. Evidence to commit per run

CLAUDE.md: "The bench (TB) and flight logs are evidence: commit the script, the raw log and the card update." Folder
(U-14, Luis 2026-10-26, "all yes"): `tests/flight/quad/M4/<run>/` with `a_tethered`, `b_free`, `c_acro`. Per run:
1. The filled checklist (this card with dates, initials, pack voltage, banner, image and config hashes, the AM32 configurator screenshot (U-2), G-1 to G-13 state).
2. The raw log(s) and the script that produced any derived number. The log is the onboard flight log (section 2.G, U-4).
3. The GCS ground log or S4b report for the run, if taken (`manifest.json`, report, `verdict.json`, as in the GX-10
   runbook "Where things land").
4. A card update line: what was observed against the pass condition, the pass or fail and who ruled. If a measurement
   (mass, J, tau, idle speed) replaced a PLACEHOLDER, the card file change in its own commit with the script and raw data
   (spec §4.0 M4 bullet 1).
5. The result in the handoff file. Any decision loosening a threshold needs a record (CLAUDE.md, Tests and decisions).

## 6. Luis rulings (2026-10-26)

Luis, 2026-10-26: "all yes start". He accepted every recommendation below.

Gates and equipment
1. U-1 ruled (Luis 2026-10-26, "all yes"): tether and area: about 1 m of non-stretch line to a heavy anchor; flat hard ground; pilot well outside the tether radius; a spotter if one is available (G-13).
2. U-2 ruled (Luis 2026-10-26, "all yes"): Luis sets and confirms 1700 KV and 14 poles in the AM32 configurator; screenshot in the run folder (G-10).
3. U-3 ruled (Luis 2026-10-26, "all yes"): the current m4 flight image with the logger on, flashed and hashes recorded; no validation report needed on the factory config (a stored config not bound to this image refuses arming; the factory config does not: `fw/arming/include/marv/arming/arming.hpp:31-33`) (G-11, G-12).
4. U-4 ruled (Luis 2026-10-26, "all yes"): the onboard flight logger (on by default, decision 0060, gaps allowed); no cable (2.G).
5. U-5 ruled (Luis 2026-10-26, "all yes"): G-8 (pendulum, motor tau, thrust map, card update) is not required before the tethered hover (a) nor the free hover (b), which fly on the current card; it is required before ACRO (c) and for the done-when (G-8, G-9).
6. U-6 ruled (Luis 2026-10-26, "all yes"): CG within about 1 mm of the motor-rectangle centre; one scale reading with pack and props on, recorded; the HIL S4b angle verdict bar is Luis's verdict (G-2, G-3, G-4).
Limits
7. U-7 ruled (Luis 2026-10-26, "all yes"): pack: start at 4.1 V/cell or more; runs of 60 s or less; end the session when a resting reading reaches 3.7 V/cell (14.8 V); no swollen or damaged pack.
8. U-8 ruled (Luis 2026-10-26, "all yes"): calm air only; runs 60 s; idle dwell after arming 5 s; height 0.3 to 0.5 m tethered, 2 m or less in free hover.
9. U-9 ruled (Luis 2026-10-26, "all yes"): replace any chipped, cracked or bent prop; stop if a motor or the ESC is too hot to keep a finger on.
Pass and abort
10. U-10 ruled (Luis 2026-10-26, "all yes"): pass (a): lifts off near the expected hover stick (about 0.32), holds level at centred sticks without growing drift or oscillation, returns level after small inputs, lands and disarms cleanly. Pass (b): the same without the tether. Pass (c): small rate steps stay controllable, back to ANGLE, land.
11. U-11 ruled (Luis 2026-10-26, "all yes"): abort triggers are visual calls, no numbers: tilt or drift growing with sticks centred, any oscillation or twitch, tether going taut, someone entering the area, loss of sight, odd sound, smell or smoke. Action: disarm on ch5.
12. U-12 ruled (Luis 2026-10-26, "all yes"): no, ANGLE to ACRO is not a recovery. The first ACRO flight is tethered.
13. U-13 ruled (Luis 2026-10-26, "all yes"): first ACRO: small roll and pitch inputs (about 20 % stick), tiny yaw, no flips.
14. U-14 ruled (Luis 2026-10-26, "all yes"): yes, evidence folder `tests/flight/quad/M4/<run>/` (`a_tethered`, `b_free`, `c_acro`); abort on any `armed_hold` or `armed_failsafe_centred`.
G-6 ruled (Luis 2026-10-26, "all yes"): only the props-off angle-mode motor-direction check on the flight image gates (a) (tilt the quad by hand, check each motor responds in the right direction, ESC powered, props off); the full bench-profile props-off TB moves to before (c).

G-6 also carries the failsafe re-walk (2.E step 15), and the lift-off tilt trigger is a visual call (section 4) (Luis 2026-10-26: "1. yes 2. recommended").

Not answered by any ruling, still open: the free-hover area size and clearance ((b) set-up; asked before (b)).

Noticed, not changed: decision 0044 and the m2 bench card still say `marv_5in_fc3v2` (the card was renamed
`marv_7in_fc3v2`, decision 0052 item 5); the M2 failsafe trace was measured on an older image than the flight image.
