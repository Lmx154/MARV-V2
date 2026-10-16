# 0040: M2 arming, link-loss failsafe and the stale-IMU path (architect proposal, not approved)

## What changed

Decided 2026-10-16 by the architect, before M2 P1, P3 and P4:

- **Arming** is a new pure state machine (host T1, `tests/regression/quad/L08`), stepped once per rate tick before the gate. It owns Disarmed/Armed and the failsafe phase; it outputs armed, the effective sticks and the reason for a refusal or a disarm. `MotorGate` in `quad_fc.cpp` gains `Armed`; the gate stays the only decider of written values (Disarmed: stop; MotorTest: one motor; Armed: the mixer output) and is Armed iff the machine is. MotorTest is entered only from Disarmed (core §4: the ESC-command path "works only while disarmed"); a request while Armed is refused (new `MotorTestReason`); arming is refused while a test runs. A test cannot run while armed, so no disarm has a test to cancel.
- **Arm** only on an off-to-on arm-switch transition (switch seen off since init and since the last disarm or refusal), with: the link valid for `failsafe_recovery_delay` (boot counts as a loss); received throttle at or below the throttle-minimum parameter; no IMU read-deadline miss since init; GyroValid and AccelValid set and no saturation bit in every sample since the previous rate tick; no motor test running; the estimator valid from M3. The arm channel, its threshold and the throttle minimum are Luis's numbers. **Disarm** on: the switch off in a received frame; Drop; any IMU read-deadline miss (on that rate tick, latched until init). Saturation refuses arming and never disarms.
- **Failsafe clock**: `hal_time_us()` read in the rate tick after the manual-control pull; age = now minus the stamp of the newest valid 0x16 frame; a stamp later than now counts as no valid frame. No tick counting. Age >= 100 ms: lost, last sticks held; >= 400 ms: roll, pitch, yaw 0, throttle 0; >= 1.5 s: disarm (Drop). Integer-microsecond parameters cited to Betaflight 4.5.
- **ManualControl** (HAL pull, double-buffered like `hal_motor_test`, `hal_sim` staging before a tick): `t_us` (`hal_time_us()` when the newest CRC-valid 0x16 frame completed, taken when the decoder's RC-frame counter changes, not from its last event), `frames` (0 = none ever, every field 0), roll, pitch, yaw in [-1, 1] in the FRD sense of core §3 Signs, throttle in [0, 1], aux in [-1, 1] (raw switch positions), link quality in [0, 1] with its own stamp, read by no decision. The producer never decides validity; the consumer owns staleness. The writer runs below the IMU interrupt's priority. Channel order and signs are backend configuration, checked on the bench by stick against console as motor directions were.
- **Stale IMU**: D = FIFO depth in samples / ODR (EMB-6), derived, no parameter. (a) FIFO_OVR_LATCHED at an interrupt entry: the driver increments a deadline-miss counter (new HAL pull, `hal_sim` stage function) before delivering the batch; the machine disarms on that rate tick. (b) No sample delivered for D: `hal_panic` (stop frames, Luis's option A) from a hardware alarm re-armed at every delivery, at a priority not above the IMU interrupt, and from the interrupt's own entry (a stuck-high INT1 line starves the alarm). The AM32 silent-line stop (about 0.4 s, INFERRED, 0039) is the backstop only. SIL cannot stop its tick; it tests (a) through the staged counter.

## Why

Quad spec §4.0 M2 (arming, failsafe, the manual-control HAL pull, the stale-IMU rule EMB-6); the reasons are given per
question above and the rejected alternatives below.

## Invariants and the tests that guard them

I1 gate Armed iff machine armed; Armed and MotorTest exclusive; I2 every disarm writes stop on the same rate tick: `quad_fc_gate_test.cpp` extended (interleavings; the mutation "test while armed" fails). I3 each failsafe threshold on both sides at one rate tick; future stamp not valid; link quality changes no output: L08 T1. I4 first deadline miss disarms on that rate tick and arming stays refused until init: L08 T1, SIL. I5 no sample for D gives PANIC and stop frames: T1 of an SDK-free deadline tracker; TB with INT1 masked. I6 saturation refuses arming, never disarms: L08 T1.

Frozen paths M2 touches (core §7.3; nothing removed or loosened, only test targets added): `tests/regression/quad/L08/unit/CMakeLists.txt` gains each M2 packet's T1 target (CRSF decode, manual-control pull, stick rates, the arming and failsafe machine); `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp` and `tests/regression/quad/L09/unit/CMakeLists.txt` are extended for the gate's Armed state (P4). `tests/regression/quad/L09/unit/lsm6dsv_decode_test.cpp` gains the read-deadline D derivation tests (P7).

## Rejected

Arming inside `MotorGate` alone (not testable alone as M2's done-when asks). Tick counts as the failsafe clock (the cited timings are wall time; the IMU oscillator is the failing part). The sample stamp `s.t_us` (back-dated, so a legitimate frame reads as future). A check in `main`'s idle loop (its `printf` can block up to `PICO_STDIO_USB_STDOUT_TIMEOUT_US`, 500 ms, pico-sdk `stdio_usb.h`; a second writer). Relying on the ESC's stop (INFERRED, ESC-specific, not at once). A new `ImuFlag` bit (changes frozen layout tests). Decoded switch booleans from the producer (hides safety thresholds outside the tested machine).

## Open, for Luis (not resolved here)

1. Stage-1 recovery: §4.0 M2 says control returns only after 0.5 s of valid link; the cited Betaflight 4.5 guide returns control at once during the hold and Stage 1 and applies `failsafe_recovery_delay` in Stage 2 (after Drop: re-arm blocked 0.5 s, switch cycled). He said "yes keep betaflight failsafes".
2. Hold end: the guide says both "a further 300ms" after the 100 ms and "after 300ms with no valid data"; carry 400 ms UNVERIFIED with both readings.
3. A dead IMU halts the firmware through `hal_panic`; M1's "never halts after arming" covers spacing anomalies and missing samples, not a dead sensor.
4. FIFO depth: DS13476 Rev 5 §6.10 gives "1.5 KB of data", not a sample count, so D is INFERRED until measured (INT1 masked, poll DIFF_FIFO_[8:0] until FIFO_OVR_IA; commit the script and log).

## Evidence

Survey of the M1 code (no receiver, manual-control, arming or failsafe code existed; the motor gate at `fw/compositions/quad_fc/src/quad_fc.cpp:48`); the line references above. The Betaflight 4.5.1 source read that closed Q5.1 and Q5.2 is in decision 0041.

## Approval

Pending: the lead owns acceptance; items 1-4 and the three safety numbers are Luis's.
