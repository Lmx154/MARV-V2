# M2 bench card — receiver, arming and failsafe (quad spec §4.0 M2 TB), Luis present

M2 done when: "T1 tests of the arming and failsafe state machine pass; on the bench (TB) a receiver loss disarms at its
stated time, and arming is refused with the throttle up or the IMU stale."

Setup, 2026-10-16: image `g4aeb522` (`build/m33/fw/hal/target/marv_fc3v2.elf`, sha256 `e01df293059c8bca…`) on the
FC3v2. ELRS receiver on SER0 (GPIO 0/1, CRSF 420000 baud), RadioMaster Pocket, receiver failsafe "No Pulses". Arm
switch ch5 (+100 armed, −100 disarmed), throttle ch3. Props off. Battery unplugged (USB power) for steps 1–6; 4S pack
on for step 7. Logs: `m2_bench_2026-10-16/`.

Tools:
- Console (1 line per second, raw sticks of the latest frame and the arming state): `tools/target/readcon.py`.
- SWD trace (`tools/target/arming_trace.py`, this commit): OpenOCD attached without reset or halt (a halted core pauses
  the RP2350 timer), polling the arming state byte every ~350 µs and, on each change, the timer and the latest
  receiver frame's stamp. Times are the firmware's own clock against the firmware's own frame stamp. The resolution is
  the poll interval plus the wait for the next rate tick.

## Results

1. Stick directions (`sticks_raw.log`; Mode 2, one deflection per axis, ch5 off): **pass**. Yaw full right: yaw
   +1.000 (line 24 on); right stick full right: roll +1.000 (31 on); right stick full forward: pitch −0.87 to −1.000,
   forward is nose down (32–40); throttle about half: 0.45–0.49 (18–44), with `arming=throttle_up` while raised. Live
   readings earlier in the session (not logged) agreed: roll +1, pitch −1, yaw +1, throttle 0.667 at the up-right
   corner. The opposite corner was not captured; the conversion's symmetry is covered by the T1 tests.
2. Arm (`failsafe_raw.log` lines 1–48): **pass**. Throttle down, ch5 to +100: `arming=armed`.
3. Arming refused with the throttle up (`throttle_up_raw.log`): **pass**. ch5 off, throttle at full (0.999), ch5 on:
   `arming=throttle_up`. Throttle then brought down with ch5 still on: `arming=switch_not_cycled` (the refused edge
   does not arm later).
4. Receiver loss disarms at its stated time (`failsafe_trace.txt`, raw `failsafe_trace_raw.log`; poll 348 µs):
   **pass**.

   | Stage (decision 0041, Betaflight 4.5.1) | Stated | Measured, frame age at the change |
   |---|---|---|
   | link lost, last sticks held (`armed_hold`) | 100 ms | 100.824 ms |
   | sticks centred, throttle 0 (`armed_failsafe_centred`) | 300 ms | 300.374 ms |
   | disarm (`failsafe`) | 1500 ms | 1500.423 ms |
   | link back: `no_link` until valid for | 500 ms | 500.150 ms, then `switch_not_cycled` |
   | re-arm | after a switch cycle | `switch_off`, then `armed` |

   The 1 Hz console log of the same sequence (`failsafe_raw.log` lines 49–83) agrees: `armed` at age 67 ms,
   `armed_failsafe_centred` at 1.07 s, `failsafe` from 2.07 s; no re-arm with ch5 left on; armed after the cycle.
   No receiver error counter moved during the radio off/on.
5. Arming refused with the IMU stale (`imu_stall_trace.txt`, `imu_stall_console_raw.log.gz`): **pass**, by the target's
   stale-IMU path. Armed, throttle down; the trace cleared IO_IRQ_BANK0 (IRQ 21) in core 0's NVIC (ICER0) over SWD
   without halting, so no IMU sample was delivered. The read-deadline alarm fired and the board panicked:
   `PANIC: IMU read deadline missed: 14532 us without a delivered sample, limit D = 14532 us … found by the alarm`. On
   the target a stale IMU is a panic (interrupts off, stop frames to the motors, nothing runs until a reboot), so no
   arming is possible; the state byte stays at its last value because the composition no longer runs. The arming
   state machine's own `imu_stale` refusal (a FIFO overrun counted as a deadline miss) is not reachable on the bench
   before the alarm and is covered by the T1 tests (`tests/regression/quad/L08/unit`). The panic message repeats on
   the console (about 51 000 lines in 20 s; noticed, not changed). Recovery: `tools/target/flash.sh --probe-only`
   (whole-chip watchdog reboot, nothing written); an OpenOCD `reset run` restarts the cores only and leaves USB down.
6. Boot with ch5 already on (after the step 5 reboot): `arming=switch_not_cycled`, not armed. One rate-loop timing
   fault on this boot (`timing_faults=1`, disarmed); the first boot of the image ran with 0 (`failsafe_raw.log`).
   Noticed, not changed.
7. Motors from the radio, 4S pack on, props off (`motors_trace.txt`): **pass**. All four spun at idle when armed
   (Luis: "all four spun up. they indeed turned off after switching the radio"). Armed three times (2.4 s, 2.0 s,
   4.4 s), then armed and radio off: `armed_hold` 100.379 ms, `armed_failsafe_centred` 300.529 ms, `failsafe`
   1500.430 ms, and the motors stopped. Luis: "they sounded like they started off slow and started getting faster and
   faster gradually, but also some of the motors looked a bit erratic". Expected with props off: the rate loop's
   integrator winds up on gyro bias with no plant to oppose it, and the mixer spreads the resulting torque command
   over the motors. The integrator restarts at arming only; holding it at zero while the throttle is at minimum
   (Betaflight's behaviour without airmode, INFERRED until the 4.5.1 source is checked) is planned before props-on
   (M4), Luis: "as long as we know the reason for the spin up then that is fine".

Done-when: the T1 arming and failsafe tests pass (L08, 121/121 on `4aeb522`); receiver loss disarms at 1500.4 ms
against 1500 ms; arming is refused with the throttle up (step 3) and impossible with the IMU stale (step 5).
