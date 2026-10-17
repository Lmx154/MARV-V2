# HIL S4: the FC3v2 board flies the Gazebo sim (decisions 0046, 0054)

Bench evidence, not a frozen test. Simulation on the real card's INFERRED/PLACEHOLDER values (vehicles/marv_5in_fc3v2.yaml,
with drag, decision 0049); not a flight result.

## S4a: closed loop with the radio off, 2026-10-17 (VERDICT OK)

- Board image: `MARV_FC3V2_HIL=ON`, `MARV_FC3V2_HIL_RECEIVER=ON` build of `d46f7f6` (`quad-L2-pass-328-gd46f7f6`, ELF
  sha256 937cb6be9697f1aa07c146a30004f01e3593ef4a3b77de4ed124f0265c230fe0); ESC battery unplugged; HIL images drive no
  motor pad. Host: load average ≤ 4, output on tmpfs (/dev/shm).
- Command: `MARV_PROBE_SERIAL=900678EF479C3A0E uv run python tools/sim/run_hil_s4.py --port
  /dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00 --out-dir /dev/shm/s4a_60 --seconds 60 --delay-ticks 25
  --expect s4a` → `s4a_60/run_hil_s4.out` (the report), `s4a_60/*.stderr.txt` (the plugin's live lines), the world, and
  the uplink, frame and report timing (xz). The 225 MB gz log is not committed: sha256
  a0b6822a5819423fe82dceefc908dc91f61e35f8ebbdbaf655adff542c11faf7.
- Results: 450 018 ticks (60.000 s), sim/wall rate 0.999908, lag absorbed 5.5 ms (the earlier attempt on a host at load
  31 absorbed 44 s and missed 113 deadlines); 112 504 sample records, 225 008 DShot reports, 0 CRC failures, 0 sequence
  gaps, 0 link faults, 0 rate timing faults; deadline misses 1 (the expected one after the stream stops); turnaround
  median 739 µs, p99 861 µs, max 1842 µs (13.8 ticks); N rule N ≥ 4 + 14 = 18, N = 25 met with 7 ticks to spare;
  command margin min 1912 µs, median 2364 µs; board max tick 37.0 µs, max 4-sample record 102.6 µs; arming boot →
  no_link; every DShot 0; the launch pad held the whole run. QF-3 at N = 25 (nominal rows): PM 71.47°, Ms 1.4465.
- Negative control (`--seconds 10 --delay-ticks 4 --expect late`, `s4a_late/`): the plugin stopped with the late-command
  fault at tick 4; tick 4 was never stepped and no command was reused: VERDICT OK (late).
- Afterwards the flight image `gd46f7f6` was flashed back: console est_valid=1, arming no_link, timing_faults 0.

## S4b runbook (Luis flying; not run yet)

Needs: Luis at the bench with the RadioMaster (the radio must reach the board's receiver); ESC battery unplugged; no
builds or other heavy jobs on the host (real time: check `uptime`, load below ~4).

1. Flash the HIL receiver image: `cmake --preset m33 -B build/m33-hil -DMARV_FC3V2_HIL=ON -DMARV_FC3V2_HIL_RECEIVER=ON &&
   cmake --build build/m33-hil -j 8 --target marv_fc3v2`, then `MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh
   build/m33-hil/fw/hal/target/marv_fc3v2.elf`. Build the plugin: `cmake --preset host-gz-quadfc && cmake --build --preset
   host-gz-quadfc`.
2. Radio on, ch5 (arm) down, ch8 (angle) up, throttle low.
3. Run: `MARV_PROBE_SERIAL=900678EF479C3A0E uv run python tools/sim/run_hil_s4.py --port
   /dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00 --out-dir /dev/shm/s4b --seconds 300 --delay-ticks 25
   --expect none`. The runner reboots the board and waits ~12 s (this host's ModemManager probes the port after a reboot).
4. Watch the live line (board arming state, receiver frames, DShot, height, tilt). Wait for the gyro calibration (keep the
   radio sticks still), then arm on ch5. Raise the throttle past hover (~35 % stick) to lift off the pad, fly gently in
   angle mode, keep it low.
5. To stop: disarm on ch5, then Ctrl-C. (With no samples the board counts a deadline miss and disarms anyway.)
6. Afterwards: flash the flight image back (`cmake --preset m33 && cmake --build --preset m33`, then flash.sh with
   build/m33/fw/hal/target/marv_fc3v2.elf) and check `est_valid=1` on the console. The run's MHRF records (each receiver
   frame with its board stamp) are the input for S5 (S4's sticks replayed in SIL).
