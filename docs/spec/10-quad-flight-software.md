# MARV V2 — Quad flight software

**Status: ACTIVE.** Depends on `00-core-contracts.md`; it never redefines anything the core owns.
Split 1, 2026-09-28.

---

## 0. Scope

**Active now: the spine (§4).** Rate and attitude stabilization, Freestyle (acro, angle) flown in Gazebo, one IMU, one
vehicle card (UZH NeuroBEM 5-inch), carried through to lockstep HIL on the RP2354B.

**Parked branches (§6).** Each starts only after the spine passes, and becomes active only when Luis edits the table
in §6:

- B1 plant realism and log replay;
- B2 mission (navigation estimator, position control, waypoints, RTL, failsafes);
- B3 self-calibration and INDI;
- B4 vehicle zoo;
- B5 avoidance and CV.

**Not in this spec:** the ground segment and the rocket. The spine's pilot input uses a minimal joystick bridge (L8),
not the GCS.

---

## 1. Why the everyday loop is SIL

A loop that needs real hardware in real time (HIL) runs at 1× by construction:

- every experiment costs its flight time plus setup and reset;
- two experiments can never run at once;
- a gain change needs a new run in every condition it must hold for.

So the everyday loop is **software-in-the-loop (SIL)**: the firmware compiled for the PC, driven by a virtual clock
in lockstep with the physics, headless, as fast as the CPU allows, many runs in parallel. HIL stays as the gate
before flight, not the place where tuning happens.

HIL with the real RP2354B can never run faster than real time: the MCU runs on its own crystal. Its job is to verify
real timing on real silicon.

---

## 2. Hardware

**The MCU is fixed:** Raspberry Pi RP2354B, 2 × Cortex-M33 at clk_sys up to 150 MHz, single-precision FPU. Its double
coprocessor (DCP) costs dadd 6, dmul 17, ddiv 51 and dsqrt 49 cycles, against 70–650 in software. 520 kB SRAM in 10
banks; a 16 kB 2-way XIP cache (RP2350 datasheet §3.6.2, §3.6.4, §4.4.1). Hot loops run from SRAM, because an XIP
cache miss is timing jitter.

**The clock budget is 150 MHz**, the datasheet maximum and the SDK default. INAV runs at 192 MHz and ArduPilot at
225 MHz, but neither is supported. An overclock is margin found later, after re-deriving the PIO and QSPI dividers,
never budget.

**Sensors** are classes and profiles (core §5). The spine uses one IMU.

**Actuators:** four ESCs on PIO bidirectional DShot600, with eRPM at the loop rate.

- Bidirectional DShot is a hard requirement (SC-HW1, §6.3): the RPM notch filters and identification need eRPM at
  loop rate, and UART ESC telemetry is too slow for either. ArduPilot quotes about 100 Hz for UART telemetry versus
  400 Hz for bidirectional DShot; Betaflight calls UART telemetry "way too slow for RPM filtering"
  ([Betaflight DShot notes](https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/Dshot.mdx)).
- Betaflight measures a bidirectional DShot300 exchange at about 169 µs, which caps DShot300 below ≈ 5.9 kHz. Plan on
  DShot600, the only mode Betaflight's RP2350 PIO driver supports.

---

## 3. Simulation: Gazebo only

The quad has **one physics host: Gazebo Harmonic (gz-sim)**. `marv_plant` supplies all forces except contact, plus
gravity, environment and sensors (core §6). Gazebo integrates the rigid body and handles contact, geometry and
rendering.

### 3.1 The lockstep plugin

A gz-sim System plugin, the only host-specific code:

1. In `PreUpdate`, read the state: `WorldPose`, `WorldLinearVelocity`, `WorldAngularVelocity`, and
   `WorldLinearAcceleration` (for contact phases).
2. Convert ENU/FLU → NED/FRD.
3. Call `marv_plant`: advance motor states, compute the wrench about the CM and the sensor samples.
4. Step the firmware an integer number of ticks per host step (core §3), in-process (core §4), with one sample set per
   tick; take its actuator outputs.
5. Convert the wrench back to Gazebo's frame and apply it.

The plugin blocks until the firmware's tick completes. Nothing reads the wall clock. (Precedent: ArduPilot's plugin
with `<lock_step>`, [ArduPilotPlugin.cc](https://github.com/ArduPilot/ardupilot_gazebo/blob/main/src/ArduPilotPlugin.cc).)

- It replaces `MulticopterMotorModel`, gz-sensors noise and the wind plugin; world gravity is zero (core §6).
- **Wrench application point.** gz-sim's `AddWorldWrench(ecm, force, torque)` applies at the **link origin**, while
  `AddWorldForce` acts at the CM ([Link.hh, gz-sim8](https://github.com/gazebosim/gz-sim/blob/gz-sim8/include/gz/sim/Link.hh)).
  Generated models put the link origin at the CM, so the two agree. The adapter test (L2) catches it if not.
- **Accelerometer specific force** comes from `marv_plant`'s own forces divided by mass in flight. Only on the ground
  does it include Gazebo's contact force, read from the link acceleration.

### 3.2 Two modes, one code path

| Mode | Command | Real-time factor | Used for |
| --- | --- | --- | --- |
| Pilot (GUI) | `gz sim <world>.sdf` | 1 | Freestyle by a human (T5) |
| Test (headless) | `gz sim -s -r --iterations N --seed S <world>.sdf` | 0 (uncapped) | T4 scenarios and Monte Carlo |

The plugin, the firmware library and `marv_plant` are identical in both modes. How the world gets its real-time
factor (generated variants of one template, or the `set_physics` service at start) is decided at L2 (Q-D4).

### 3.3 Requirements on the simulation (SIM)

| ID | Requirement | Rule or test |
| --- | --- | --- |
| SIM-2 | Determinism. | Same (firmware commit, card, scenario, seed, versions) → bit-identical run on the same machine. Checked at L2; a sample reruns nightly. |
| SIM-3 | Throughput, defined by a rule. | Required real-time factor per core = (simulated seconds in the largest planned batch) / (time window × cores), from the batch plan in the budget register. Measured at L2. If headless Gazebo misses it: decision record (options: profile the plugin, re-derive rates by SIM-7, more parallel processes, or a second host with parity tests per core §6). |
| SIM-4 | Validated against flight logs. | Branch B1. |
| SIM-5 | Provenance. | Core §2; the run report lists every `UNVERIFIED` entry. |
| SIM-6 | Scenario coverage. | Grows with the branches: wind and turbulence; battery sag; ground effect; GNSS denial and multipath; sensor dropout, spike, stuck value, saturation and temperature ramp; CV false positives and misses; GCS link loss. Each failsafe requirement has at least one scenario that trips it. |
| SIM-7 | Step sizes by convergence. | Core §7.5, applied to Gazebo's physics step, `marv_plant`'s motor sub-step and each flight rate group that fails EMB-3 at its parent group's rate (core §7.5, flight rate groups). |

SIM-1 (cross-engine parity) is retired: the quad has one host.

### 3.4 For the record: what gz-sim's `MulticopterMotorModel` computes

([source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc))

- motor lag: zero-order-hold first-order, τ = 1/80 s up and 1/40 s down (the plugin's defaults, which the toolbox
  cites as "Gazebo's rotor lag");
- thrust sgn(ω)·ω²·k; rotor drag −|ω|·c·v⊥; yaw torque −dir·T·k_M; rolling moment −|ω|·c_r·v⊥;
- not modelled: battery, ground effect, noise, any lower speed limit.

This is why `marv_plant` replaces it.

**PX4's `x500` is not a validated reference.** Its motor block (8.54858 × 10⁻⁶, 0.016, 0.0125 / 0.025 s,
8.06428 × 10⁻⁵, 10⁻⁶, slowdown 10) descends from RotorS's 2014 AscTec Firefly values with no identification cited
([451417f](https://github.com/ethz-asl/rotors_simulator/commit/451417febc38792605c1d71e8851d4c637453c44)), copied
through the Iris and Open Robotics' X3 into the x500 in 2022
([0e8e38e](https://github.com/PX4/PX4-Autopilot/commit/0e8e38e698e870f24cc3109c68216be6460ded2b)). Its 2.0 kg and
inertia (0.021667 / 0.021667 / 0.04) are exactly a solid cylinder of radius 0.2 m and height 0.1 m: a placeholder.

---

## 4. The spine

Layers are built bottom-up. Each lists what it builds, the opening it exposes, its pass bar (written before building;
thresholds from rules or the budget register), the test level of each check (core §7.1), and what freezes. Mechanics
of freezing: core §7.2–7.3.

### L0 — Foundations

- **Builds:** the L0 primitives (vectors, small matrices, quaternions; core §9); the conventions of core §3
  (quaternion storage and sign, time type, tick, logical motor numbering and its index mapping); the HAL interface with
  `hal_sim` and its virtual clock; the IMU sample struct and the actuator-output struct (other sensor classes join
  with the layer that first uses them); the rate-group scheduler; the parameter interface and its bootstrap generator,
  run on a test fixture until L1 supplies the card and register (core §4); the SIL entry point (core §4); the CI gates
  G1, G3 and G8; the freezing machinery of core §7.2–7.3.
- **Opening:** `hal_time_us()`; the IMU sample struct (timestamped SI, core §5); the actuator-output struct (DShot 0
  or 48–2047, servo µs; core §4); `param_get(id)` with provenance; the SIL entry point (init, tick, shutdown; C ABI).
- **Pass bar:**
  - T1: quaternion and matrix operations against analytic answers, including composition order, `[w, x, y, z]`
    storage, the canonical sign rule and the body → NED direction; float and double instantiations.
  - T1: the float build for the M33 (`m33` preset) has zero `-Wdouble-promotion` warnings; a planted float → double
    promotion fails it (negative control).
  - T1: the magic-number lint runs over `fw/` and fails on a planted literal (negative control).
  - T1: rate groups fire at exact integer divisions of the tick (EMB-2).
  - T1: the virtual clock's stamp at tick N is ⌊N × 156.25⌋ µs for a 6.4 kHz stream, computed from N and never
    accumulated. Checked in closed form for every N below 10⁶ and for 10⁶-tick windows straddling 2³² and 2⁴⁰.
  - T1: contract tests on both sides of each opening: units and ranges of the structs, the actuator range (1–47
    rejected), `param_get` returning value and provenance for every generated id, the generator refusing an entry
    without provenance.
  - T2: a fixed L0 test composition (scheduler, clock, parameter reads, a deterministic synthetic IMU stream, null
    plant) runs N ticks through the SIL entry point; the hash of its per-tick trace equals a committed golden.
    Negative control: perturbing one sample changes the hash.
  - CI: G3 finds no truth symbol in the firmware library and fails on a planted one (negative control); G8 fails
    when a required `CLAUDE.md` section is missing (negative control).
- **Freezes:** all of the above.

### L1 — Vehicle card and `marv_plant` v0

- **Builds:** the card schema and provenance linter; the sensor-profile schema and its linter (core §5); the NeuroBEM
  5-inch card; the design-budget register file; `marv_plant` v0 with gravity, motor/ESC first-order lag, and rotor
  thrust and torque. No battery, drag, ground effect or noise yet.
- **Opening:** `marv_plant_step(state, actuators, dt) → wrench about CM, motor states, eRPM`.
- **Pass bar (all T1, no host):**
  - Hover: at the card's hover rotor speed, total thrust equals m·g(h) within float tolerance.
  - Motor step response matches the first-order closed form at every sub-step.
  - Yaw torque sign and magnitude per motor match the card's spin directions.
  - The linter rejects an entry without provenance, and rejects the example card until its σ entries are filled
    (negative controls).
  - Generators write the plugin configuration and the SDF from the card, and a round-trip check reads the same values
    back.
- **Freezes:** all of the above.

### L2 — Gazebo host

- **Builds:** the lockstep plugin (§3.1); the generated SDF (link origin at CM, world gravity zero); the headless
  runner script; the choice of how each mode sets its real-time factor (Q-D4).
- **Opening:** a scenario runner: `(card, scenario, seed) → logged run`.
- **Pass bar:**
  - T1: adapter test. For a set of states and seeds, the wrench and every sensor byte through the plugin's conversion
    equal a direct `marv_plant` call. ENU↔NED round-trips are exact.
  - T4: motors off → free fall at g(h) within Gazebo's integration error (measured by halving the step, SIM-7).
  - T4: torque-free rotation conserves |L| and kinetic energy within integration error.
  - T4: open-loop hover command from the card → vertical acceleration within integration error of zero.
  - T4: SIM-2 determinism: two headless runs bit-identical.
  - SIM-3: real-time factor per core measured, recorded, and compared with the required value.
- **Freezes:** adapter test, the three analytic scenarios, determinism.

### L3 — Mixer / allocation

- **Builds:** wrench → DShot values: idle floor at the card's minimum stable rotor speed, saturation, desaturation
  (air mode, QF-6).
- **Opening:** in: thrust (N) and torque (N·m about the FRD CM); out: DShot 48–2047 per motor.
- **Pass bar (T1):**
  - Unsaturated: mixer composed with the card's effectiveness matrix is the identity within float tolerance.
  - Sign contract, in logical motor numbers (core §3): a +roll torque request (right side down) raises motors 2 and 3
    (left side); +pitch (nose up) raises 1 and 3 (front); +yaw (nose right) raises the motors whose props spin
    counter-clockwise viewed from above, per the card's directions.
  - Saturated: the documented priority is preserved (roll and pitch over yaw), and no motor drops below idle.
- **Freezes:** all of the above.

### L4 — Rate loop on truth gyro (PID)

- **Builds:** rate PID with anti-windup; gains generated by the loop-shaping rule (the toolbox's `loopshape.ts`) on
  the card's linear design model at the chosen loop rate. No typed gains.
- **Opening:** in: rate setpoint (rad/s, FRD); out: torque request to L3.
- **Pass bar:**
  - T3: QF-2 reference-model tracking and QF-3 margins (`PM_min`) on the design model.
  - T3: margins still meet QF-3 with motor τ at the card's ±σ.
  - T3: negative controls. Gains × 1.1 and one tick of added delay each move the golden metrics outside tolerance.
  - T4: in Gazebo, on truth gyro, per-axis steps meet QF-2, and chirp injection measures margins that meet QF-3.
  - T4: acro with a scripted stick sequence completes without saturation beyond the mixer's documented behaviour.
- **Freezes:** all of the above.

### L5 — Attitude loop on truth attitude

- **Builds:** attitude controller (quaternion error → rate setpoint), designed on the exact closed rate loop at the
  same phase margin; it runs at the rate loop's rate unless that fails EMB-3 (core §7.5; §5.4 estimates until L9), and
  only then at a rate chosen by SIM-7. Angle mode (self-levelling).
- **Opening:** in: attitude setpoint (quaternion, body → NED); out: rate setpoint to L4.
- **Pass bar:** T3 step response and margins; T4 angle-mode steps and a large-angle recovery scenario; the L4 suite
  still green.
- **Freezes:** the new tests.

### L6 — Sensor models, the gyro chain and the D term

- **Builds:**
  - `marv_plant`'s generic IMU class model driven by the profile (white noise, bias random walk, quantization,
    saturation, rate, latency, ODR error), drawing from seeded streams, one per sensor (core §5). Profile entries the
    datasheet does not give are labelled scenario values until Luis's still-bench dataset replaces them (decision
    0009).
  - `marv_plant`'s rotor vibration at 1×, 2× and 3× each rotor's frequency (§6.3.1). No published log resolves the
    harmonics yet, so the amplitude is swept as a robustness dimension and flagged unsourced.
  - The plant's eRPM to the firmware through `hal_sim`.
  - The firmware's gyro chain: low-pass, and eRPM-tracking notches at the harmonics the vibration model includes
    (QF-7).
  - The rate loop's D term: derivative of the measurement, taken on the chain's output through a first-order D
    low-pass. Gains come from the loop-shaping rule extended to PI × lead. The lead ratio is bounded by
    `d_path_noise_budget` and the sensitivity peak by `Ms_max`. The attitude gains regenerate on the new closed rate
    loop.
  - ω×Jω feed-forward with motor-lag compensation, from the filtered measured rate and the card's J and motor time
    constant τ_m. Its derivative filter T_ff is set by the combined D + FF noise line of the pass bar. Built and tested
    in stage (c), and inert until stage (e) switches it on, in the same change that creates
    `tests/regression/quad/L06/XFAIL_GATE_CLOSED`.
  - DShot error diffusion in L3's thrust → DShot conversion, with its own decision record.
- **Opening:** the IMU sample struct, now noisy; eRPM to the gyro chain; filtered rates to L4.
- **Stages.** Built and closed in order. Each stage has its own decision round and freezes its lines of the pass bar
  when it closes:
  - (a) sensor noise model;
  - (b) gyro chain;
  - (c) D term and ω×Jω;
  - (d) DShot error diffusion;
  - (e) close.

  The L4 acro and L5 R2 known failing items stay strict xfails until stage (e) creates
  `tests/regression/quad/L06/XFAIL_GATE_CLOSED` (decision 0009).
- **Pass bar:**
  - (a) T1: the simulated IMU's Allan deviation matches the profile's noise density and bias instability.
    - The bound is the Allan-deviation confidence interval for the record length (chi-square with the overlapping
      Allan variance's equivalent degrees of freedom: NIST SP 1065, §5.3.2 eq. 45 and §5.4.1 Table 5) at
      `allan_check_confidence`.
    - The record length is the shortest at which noise density × 1.1 and bias instability × 1.1 each fall outside
      the bound. Those two perturbations are the negative controls.
  - (a) T1: SIM-2 with noise. The same seed gives a bit-identical run; distinct streams are independent. The
    adapter's sensor bytes equal a direct `marv_plant` call.
  - (b) T1: each filter's frequency response matches its design formula, and the notches track eRPM across the
    vibration sweep.
  - (b, c) T3: the chain's group delay at crossover and the D low-pass are in the design model's loop delay. QF-3
    (`PM_min` and `Ms_max`) holds over the band box.
  - (c) T3: with the profile's noise, the combined RMS contribution of the D path and the ω×Jω feed-forward path to
    each motor's command is at most `d_path_noise_budget` × the hover DShot step's thrust. It is evaluated at hover and
    at `rate_max` on all three axes, with the feed-forward at the card's J and τ_m. The budget is spent once, on the sum.
  - (c) T3: the L4 and L5 T3 goldens regenerate with the D term; their controls (gains × 1.1, one tick of added
    delay) still break them.
  - (c) T3: the ω×Jω evaluation, run on the design model with ω×Jω in the plant, with the result recorded.
  - (d) T1: the diffusion's carried error stays within one DShot step; the mean applied command over a window matches
    the request within the stated bound; the L03 suite stays green.
  - (e) T3, Monte Carlo over noise seeds: each L4 and L5 scenario meets its predicate for every seed.
    - The seed count is N = ⌈ln(1 − c)/ln p⌉ (Wilks 1941), with p the scenario class's `t4_pass_probability_*` and
      c its `t4_confidence_*`. Recoveries and acro coupling are the safety class; steps, chirps and the other
      scenarios are the tracking class. The seeds are 1…N, committed.
    - Envelopes carry a noise term, derived by propagating the noise model through the design model at the per-seed
      quantile 1 − (1 − `noise_term_confidence`)/N, so that a correct build passes every seed with probability at
      least `noise_term_confidence`. The fine negative controls of 0006 decision 24 still fail with the noise term in
      place.
  - (e) T4: the L4 and L5 scenarios rerun with noise and filters and still meet QF-2 and QF-3.
    - Per push: every scenario at its committed seed.
    - Nightly and before the tag: the confirmation seeds.
    - Both known failing items pass.
- **Freezes:** each stage's lines when that stage closes. **This is where "sluggish" gets explained instead of
  guessed at:** every source of phase lag is now measured.

### L7 — Attitude estimator: shadow, then authority

- **Builds:** Mahony complementary filter, fed from the L6 sensor models; shadow mode first, then authority handoff.
- **Opening:** attitude quaternion, body rates and a validity flag to L5.
- **Pass bar:**
  - T4 shadow: across a Monte Carlo set, attitude error against truth stays within its budget share (a register entry
    derived from QF-3's allowance).
  - T4 authority: with the estimator in the loop, margins still meet QF-3 and L5's scenarios pass.
  - G3 on the link map: the firmware build contains no truth symbol.
- **Freezes:** all of the above.

### L8 — Freestyle with a pilot

- **Builds:** a minimal joystick bridge (a USB joystick on the PC, such as the radio in USB-joystick mode, read once
  per tick and injected into `hal_sim`'s manual-control input); the rates curve (QF-1); arming by a handset switch in
  SIL with the spine's pre-arm checks (sensor alive at profile rate, not saturated or stuck; estimator valid; throttle
  at minimum).
- **Opening:** the manual-control struct (stick axes, switches). The GCS path (IF-8) replaces the bridge later behind
  the same struct.
- **Pass bar:**
  - T5: a pilot flies acro and angle in the Gazebo GUI.
  - T4: a recorded pilot session replayed as a stick script passes QF-2 and QF-6 checks.
  - QF-4: the SIL latency budget (bridge, tick period, filter group delay) is computed and its phase at crossover is
    inside QF-3.
- **Freezes:** the scripted-stick scenario and the latency budget check.

### L9 — Target port

- **Builds:** `hal_target`; one driver per sensor part on the confirmed board (core §2.3), each producing the same
  sample structs; PIO DShot600 with eRPM decode; the core split (§5.2); SRAM placement of the hot chain.
- **Opening:** the same HAL as `hal_sim`.
- **Pass bar (bench):**
  - Per-stage timing: a GPIO toggles at the start and end of each stage (IMU read, filter, estimator, controller,
    mixer, output); a logic analyzer and DWT cycle counts give worst-case execution time per stage; EMB-3 holds with
    measured worst cases.
  - IMU FIFO: read deadline = FIFO depth / ODR (EMB-6); a soak shows no overflow.
  - DShot600 frame timing on the logic analyzer matches the protocol's bit timing; eRPM decode matches an independent
    speed reference.
  - Motor mapping and direction: props off, the motor test spins each logical motor alone; its pad (mapping) and spin
    direction (AM32 setting) match the card. A mismatch is fixed in the mapping or the ESC, never in code.
  - Each driver's samples on a still bench match its profile (bias, noise) within the profile's own bounds.
  - Stack high-water marks and heap-free operation after init (EMB-4).
- **Freezes:** the timing report format and thresholds, driver bench tests, DShot and eRPM tests.

### L10 — HIL lockstep

- **Needs from the ground segment:** phases G0 and G1 (the interface schema and a codec carrying IF-9). Activate them
  when the spine reaches L9. The ground segment's trivial test codec (GS-3) is enough to start.
- **Builds:** HIL mode on the target: tick-stamped sensor injection after the drivers, actuator output returned over
  USB.
- **Pass bar (T6):**
  - SIL and HIL traces of the same scenario agree within float tolerance, with `-ffp-contract=off` on both builds.
  - EMB-3 holds under HIL load.
  - The link-rate cap (core §4) is computed; if the IMU rate is decimated, the report says so.
- **Freezes:** the SIL-vs-HIL comparison scenarios.

**When L10 passes, the spine is done:** the rate and attitude stack, flying Freestyle, proven from unit tests to
real silicon.

---

## 5. Embedded design

### 5.1 Modules

| Module | Inputs → outputs | Rate (rule) | Core |
| --- | --- | --- | --- |
| IMU driver (per part) | the part's FIFO or data-ready interrupt → timestamped body rates, specific force, temperature | ODR from QF-8 | 1 |
| Gyro chain | raw rates → RPM notches, low-pass | every IMU sample | 1 |
| Rate controller (PID; INDI after B3) | rate error → torque request | QF-8 | 1 |
| Allocation + thrust linearization | wrench → rotor-speed targets → DShot values (battery-compensated) | rate loop | 1 |
| DShot + eRPM (PIO) | DShot frames ⇄ eRPM | rate loop | 1 (PIO) |
| Attitude / angle loop | attitude error → rate setpoint | designed on the closed rate loop at the same phase margin; the rate loop's rate unless that fails EMB-3, then SIM-7 (core §7.5) | 1 |
| Attitude estimator (spine) / navigation EKF (B2) | IMU (+ baro, GNSS, mag, ranges, CV in B2) → state, covariance | spine: every rate-loop tick unless that fails EMB-3, then a SIM-7 division (core §7.5); B2: prediction at the IMU down-sample where coning error (ω·Δt)² stays below one step's process noise | 1 (spine), 0 (B2) |
| Position / velocity control, trajectory generation (B2) | mission task → attitude and thrust setpoints | outer-loop rule | 0 |
| Avoidance (B5) | ranges + state → velocity limits | each sensor update | 0 |
| Mission manager, failsafes | events → modes | each event's latency budget | 0 |
| Logger (SD, PIO) | raw samples, commands, eRPM, battery, estimator state | all streams at native rate | 0 + DMA |
| Links (GCS, GNSS, CV, range sensors) | USB, UART + DMA | per link | 0 |

### 5.2 Why core 1 is the real-time core

- The Pico SDK keeps USB and housekeeping on core 0, which runs `main()`.
- The 16 kB XIP cache is shared, so one core's fetches evict the other's. ArduPilot's RP2350 port measured it: moving
  the bidirectional-DShot code into SRAM raised core 1 idle time from 21.9 % to 51.4 %
  ([PR #32995](https://github.com/ArduPilot/ardupilot/pull/32995)). One miss is a 64-bit QSPI read, estimated at
  56–60 cycles (≈ 0.4 µs).
- Betaflight's RP2350 target runs everything from RAM (`RUN_FROM_RAM=1`).

So core 1 holds only SRAM-resident code and data, the IMU DMA interrupt and PIO DShot, and never touches flash.

- Flash writes happen only on the ground: `flash_safe_execute()` stalls the other core.
- Cores exchange data only through lock-free single-producer/single-consumer queues and seqlocks in C11 atomics, so
  the host build runs identical code. The RP2350 hardware spinlocks have an erratum (RP2350-E2), and the SDK defaults
  to software locks.

### 5.3 What others achieve on this chip

- **Betaflight** ships RP2350 since 2025.12: PIO bidirectional DShot600 only; single-core; its ICM-456xx driver runs
  at 6.4 kHz.
- **ArduPilot's draft port** (225 MHz, overclocked): core 1 IMU at 3.2 kHz, rate loop 1.6 kHz, DShot600; core 0 EKF3
  at 200 Hz, navigation, GCS and logging; idle 58 % and 54 % in a 4 kHz gyro / 2 kHz rate build; 13 kB RAM free
  after arming.
- **PX4 does not run on the RP2350**: the port issue was closed as not planned.
- **Open PIO bidirectional-DShot libraries** exist (e.g. `pico-bidir-dshot`). Check each licence (core C-4).

### 5.4 Budget: estimates, replaced by L9 measurements

Cost model: cycles ≈ floating-point operations × 2–4, for compiled SRAM-resident float code at Cortex-M4-class timings.
Arm publishes no per-instruction FPU timings for the M33. This is a compute-only lower bound; ArduPilot's whole stack
scaled to 150 MHz keeps core 1 about 63 % and core 0 about 69 % busy, roughly 10–20× the bottom-up numbers.

| Case | Core | Task | Derivation | Share of a 150 MHz core |
| --- | --- | --- | --- | --- |
| Freestyle | 1 | gyro unpack + 2 low-pass + 12 RPM-notch biquads per axis at 6.4 kHz | ≈ 400 ops | 3–7 % |
| Freestyle | 1 | rate law + mixer + DShot/eRPM decode at 3.2 kHz | 0.8–2.0 k cycles | 2–4 % (3–9 % at 6.4 kHz) |
| Mission | 0 | 24-state EKF (PX4 EKF2 structure): predict at 100 Hz | 1,810 ops (SymForce count) + overhead | 0.3–0.6 % |
| Mission | 0 | EKF fusion, about 6 scalar updates per step | ≈ 4.7 k ops each | 4–8 % |
| Mission | 0 | output predictor 1 kHz, position control 50 Hz, avoidance histogram 50 Hz | — | 1–3 % |

What it says: start Freestyle at a 6.4 kHz gyro and a 3.2 kHz rate loop on core 1, and go to 6.4 kHz only if the
measured worst case still passes EMB-3. Memory is the tighter constraint: budget RAM per module from the start
(EKF2's 24 × 24 float covariance alone is 2.3 kB).

**IMU clock accuracy is a profile item.** The ICM-45686's internal oscillator is ±1.25 %. Integrate with the FIFO's
own timestamps, or drive the IMU from CLKIN (the board wires INT2/FSYNC/CLKIN to GPIO18; Betaflight added gyro CLKIN
in 2026.6.1). The simulator models the ODR error either way.

**SIL host build:** `hal_sim` is plain C++ and nothing above the HAL includes Pico SDK headers (core C-8). The Pico
SDK is used only by `hal_target` (L9).

### 5.5 Embedded requirements (EMB)

EMB-1 to EMB-8 are defined in core §4.1. For the quad, EMB-1's chain is IMU → gyro filter → rate loop → mixer → DShot,
and its worst case is measured at L9.

**EMB-6 worked example with today's IMU** (the rule applies to any part):

- A high-resolution ICM-45686 FIFO packet is 20 bytes (DS-000577 rev 1.0 §6.1: header, accel, gyro, 2-byte temperature, timestamp, 3 extension bytes). At 6.4 kHz that is 128 kB/s for the IMU alone.
- The FIFO holds 2 KB by default, 8 KB with APEX off (DS-000577 rev 1.0 §6): 16 ms (64 ms) of samples at 6.4 kHz. That is the hard deadline
  for a FIFO read on core 1.

---

## 6. Modes, requirements and branches

### 6.1 Modes

```
DISARMED → ARMED ─┬─ FREESTYLE ── ACRO (rate) | ANGLE (optional, self-levelling)
                  └─ MISSION ─┬─ STABILIZED ─┐  tasks: WAYPOINT · ROUTE · HOLD · CV TRACK / LOCK / VERIFY
                              └─ AGILE ──────┴─        · RTL · LAND
FAILSAFES (any mode): GCS link loss · GNSS loss · estimator inconsistency · energy reserve · geofence ·
sensor fault (saturation, stuck, timeout) → HOLD / RTL / LAND / DISARM, by the rules in QX
```

The spine implements DISARMED, ARMED and FREESTYLE. Stabilized and Agile share one controller structure with different
limits and trajectory generators, derived below.

**Full arming (with B2 and the ground segment).** An explicit acknowledged command (IF-3), or the handset's arm
switch in Freestyle. Succeeds only when every pre-arm check passes, each failure reported by name (IF-1):

- the active config file verified and its (image, config) pair carrying a passing report;
- every declared sensor alive at its profile rate, not saturated and not stuck;
- the estimator converged: NIS consistent, attitude σ (and position σ in Mission) within their shares;
- battery above the QR-1 reserve for the uploaded mission (Mission), or the landing reserve (Freestyle);
- throttle at minimum (Freestyle); GNSS quality meeting the estimator's needs (Mission);
- the link present, with heartbeats inside the GS-6 timeout.

### 6.2 Requirements

Each requirement states the rule that produces its number. A generator (`marv-req`) computes every number from the
card, the sensor profile and the budget register, and prints the derivation beside it.

**Freestyle (QF): active in the spine.**

| ID | Requirement | Rule |
| --- | --- | --- |
| QF-1 | Stick → body-rate setpoint through a rates curve (centre sensitivity, maximum rate, expo). | Pilot preference (scenario value), bounded by QF-2. |
| QF-2 | The response to a full-stick step follows a reference model (first order, τ_ref per axis). | τ_ref ≥ the achievable closed-loop time constant for the identified motor lag and loop delay, and ≥ ω_max / α_max, with α_max = τ_max/J (roll/pitch: thrust margin × arm; yaw: rotor torque). |
| QF-3 | Stability margins ≥ design margins. | Phase margin ≥ `PM_min` and sensitivity peak M_s ≤ `Ms_max` (register); verified by chirp injection in SIL, then on the vehicle. |
| QF-4 | Stick-to-motor latency is budgeted and measured. | Sum of the input path (spine: joystick bridge; later: radio USB report period, GCS forwarding, link, IF-8 period, parsing), loop period, filter group delay at crossover, DShot frame (26.7 µs at DShot600) and ESC update. Its phase at crossover, ω_c·τ_d, counts against QF-3. |
| QF-5 | The same stick gives the same response across battery voltage and throttle. | Thrust linearization from the identified thrust curve and measured voltage (plus rotor-speed feedback). Verified by comparing full- and empty-battery responses in SIL; the allowed difference is the identification uncertainty. Needs the battery model (B1). |
| QF-6 | Authority at zero throttle (air mode). | The mixer keeps each motor at or above the measured minimum stable rotor speed. |
| QF-7 | Gyro filtering removes rotor harmonics without eating the phase budget. | Notches follow eRPM at the harmonics seen in the measured vibration spectrum; until one is measured, at the harmonics the B1 vibration model includes (1×, 2×, 3×). The chain's group delay at crossover is part of QF-4. |
| QF-8 | Rate-loop period. | By convergence: the lowest loop rate (among the IMU's ODRs) at which doubling the rate improves the achievable crossover by less than the uncertainty of the identified motor lag. |

**Mission, common (QM): branch B2.**

| ID | Requirement | Rule |
| --- | --- | --- |
| QM-1 | The navigation estimate is consistent. | NEES/NIS tests pass over the Monte Carlo set; innovation gates at `chi2_gate_quantile`. |
| QM-2 | Waypoint acceptance radius. | ≥ √χ²₂(p) · σ_pos, σ_pos the estimator's horizontal position σ. |
| QM-3 | Geofence and altitude limits. | Site rules (scenario values). |
| QM-4 | Missions are uploaded transactionally: sequence numbers, read-back, compare (IF-4). | Protocol conformance tests in SIL and HIL. |

**Mission-Stabilized (QS): branch B5**, for CV identification and smooth holds.

| ID | Requirement | Rule |
| --- | --- | --- |
| QS-1 | Camera angular rate low enough that motion blur stays below b pixels. | ω_max = b · IFOV / t_exp, IFOV = HFOV / (pixels across **at the detector's input size**), t_exp the exposure actually used. |
| QS-2 | Smoothness. | Horizontal jerk ≈ g · θ̇ at small tilt, so j_max = g · ω_max(QS-1). |
| QS-3 | The target stays in frame. | Lateral error ≤ h·tan(HFOV/2) − s/2 − k·σ_pos; yaw error ≤ HFOV/2 minus the same margin in angle. |
| QS-4 | The tracking loop accounts for CV latency. | Crossover ω_c ≤ (CV phase budget) / τ_cv, τ_cv measured end to end. |
| QS-5 | CV lock / verify / abort is an explicit handshake (report → lock request → setpoints → verify → abort or lost), with a timeout on each state (IF-4). | Emulated in SIL with the same message sequence; each timeout derived from the CV module's measured rate. |

**Mission-Agile (QA): branch B2.**

| ID | Requirement | Rule |
| --- | --- | --- |
| QA-1 | Envelope. | θ_max = acos(mg / T_max); a_h = g·tan θ_max; top speed where body and rotor drag equal m·a_h; rates and α from QF-2. |
| QA-2 | Every commanded trajectory is feasible. | Checked against QA-1 before execution; feed-forward from differential flatness. |
| QA-3 | Near obstacles, speed obeys QO-2. | — |

**Obstacle avoidance (QO): branch B5.**

| ID | Requirement | Rule |
| --- | --- | --- |
| QO-1 | No motion into directions no sensor covers, unless explicitly allowed. | Same policy as PX4's collision prevention: 72 × 5° sectors, no-data sectors blocked by default ([PX4 docs](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/computer_vision/collision_prevention.md)). |
| QO-2 | Stop before the obstacle. | v_max = a·(−t + √(t² + 2(R − d)/a)), from v·t + v²/(2a) = R − d. R: sensor range at the target's reflectivity (datasheet). t: sensor period + processing + velocity-loop tracking delay (measured in SIL, confirmed in HIL). a: braking acceleration from QA-1. d: vehicle radius + k·σ_pos + k·σ_range. **The avoidance sensor's range and latency set the agile speed limit.** |
| QO-3 | Ranges are attitude-compensated and time-stamped. | Each range is projected with the attitude at its sample time. |

**Return and energy (QR): branch B2.**

| ID | Requirement | Rule |
| --- | --- | --- |
| QR-1 | RTL triggers with enough energy to get home. | E_rtl = P(v_rtl, wind) · d_home / (v_rtl − w_head) + P_hover · t_descent + landing reserve. Power from the card's measured hover power and momentum theory; remaining energy from the battery model (OCV − I·R). Replaces a fixed-percentage rule. |
| QR-2 | RTL path: climb to the site's safe height, return, land; land detection from thrust / acceleration consistency. | Safe height is a scenario value. |

**Failsafes (QX): branch B2.** Each timeout is derived from the source's own rate and a false-alarm budget (register),
the rule the toolbox's `mission.ts` applies to flight events.

### 6.3 Branches

| Branch | Status | Starts after | Contents |
| --- | --- | --- | --- |
| B1 Plant realism and log replay | PARKED | spine L6 (may run beside L7–L10) | §6.3.1 |
| B2 Mission | PARKED | spine done + ground segment G2 | QM, QA, QR, QX; navigation EKF; position control; trajectory generation |
| B3 Self-calibration and INDI | PARKED | spine done + B1 | §6.3.3 |
| B4 Vehicle zoo | PARKED | B1 | §6.3.4; activates guardrail G4 |
| B5 Avoidance and CV | PARKED | B2 | QO, QS; range sensors and CV emulation in Gazebo |

#### 6.3.1 B1: plant realism and log replay

What `marv_plant` adds, and why:

| Effect | Model | Parameters from | Needed by |
| --- | --- | --- | --- |
| Motor + ESC | DShot value → steady rotor speed (with battery voltage), first-order or back-EMF electrical model, separate spin-up and spin-down, saturation, idle | thrust stand + eRPM step tests | rate loop, freestyle feel, self-calibration |
| Propeller | T = C_T ρ n² D⁴, Q = C_P ρ n² D⁵/2π with C_T, C_P vs rpm (and advance ratio if forward flight matters) | thrust stand; JSBSim FGPropeller definitions (in `rotor.ts`); UIUC / APC data as a cross-check | hover, envelope, energy |
| Rotor drag (H-force) | F = −R D Rᵀ v, per rotor or lumped (Faessler 2018, in `rotor.ts`) | forward-flight log fit | agile tracking, wind estimation |
| Body drag | quadratic, per axis | flight fit or CFD | agile speed limit, RTL energy |
| Ground effect | Cheeseman–Bennett thrust ratio vs height / rotor radius | literature form, verified by a hover-height test | take-off, landing, low hover |
| Battery | OCV vs state of charge, R₀ + one RC branch (Thevenin), I·R sag | measured discharge curve; ArduPilot SITL's equations reimplemented, not copied (GPL) | thrust at a command, RTL reserve |
| Vibration | forces at 1×, 2×, 3× each rotor's frequency | published high-rate IMU logs that resolve the rotor harmonics (the 400 Hz NeuroBEM log does not: 2800 rad/s ≈ 446 Hz; check the TU Delft logs' rate). Otherwise sweep the amplitude as a robustness dimension, flagged unsourced | notch filters, estimator accel noise |
| Atmosphere, gravity, wind | USSA76 + surface deviation; WGS 84 gravity; power-law shear + Dryden (lateral and vertical second order per MIL-F-8785C) | the toolbox, `docs/physics.md`; the day's weather | everything |
| Magnetic field | WMM | NOAA coefficients | heading |
| GNSS, baro, mag, range, CV | generic class models (core §5); GNSS error as Gauss–Markov; range by host raycast; CV at detection level with measured latency | the profiles | B2, B5 |

Borrow equations, cite them in the code, reimplement: aerodynamics and Dryden wind from RotorPy; Gauss–Markov bias
with quantization from gz-sensors' `GaussianNoiseModel.cc`; battery and voltage-dependent thrust from ArduPilot SITL;
asymmetric motor lag from L2F.

**RotorPy is the reference oracle**, never a host ([repo](https://github.com/spencerfolk/rotorpy), MIT): same inputs
to `marv_plant` and RotorPy give the same outputs within RotorPy's model's stated scope.

**Validation of the simulator (SIM-4):**

1. Analytic cases (already at L1–L2).
2. Log replay against published flights: feed logged motor commands and rotor speeds into the plant from a logged
   state; compare predicted accelerations and rates with the IMU over short horizons, as NeuroBEM does. Accept when
   the residual is consistent with sensor noise plus the card's stated uncertainties (a normalized-innovation test),
   not a hand-picked tolerance.

Reference data:

| Vehicle | Data | Where |
| --- | --- | --- |
| NeuroBEM 5-inch | 96 flights, 75 min, 400 Hz: rotor speeds and derivatives, battery voltage, motion-capture state; up to 18 m/s and 46.8 m/s² | [UZH](https://rpg.ifi.uzh.ch/neuro_bem/Readme.html) |
| Crazyflie 2.1 | motor speeds in, 13-state out, ≈ 75 k samples at 100 Hz; Bitcraze thrust-stand tables | [IDSIA](https://github.com/idsia-robotics/nanodrone-sysid-benchmark), [Bitcraze](https://github.com/bitcraze/crazyflie-firmware/tree/master/tools/system_id) |
| TU Delft 5-inch | throw-identification logs with the identified values their paper reports | [4TU](https://doi.org/10.4121/0530be90-cc6c-4029-9774-670657882906) |

**Settling conflicts with data, not by choosing.** Hover and climb segments give k/m from a_z = (k/m)·Σω² − g; angular
accelerations against rotor-speed differences give k·l/I. The NeuroBEM dataset tests which mass (752 g paper vs
772 g dataset), with the config's thrust coefficient, is consistent.

#### 6.3.2 B2: mission

Navigation filter: follow PX4's EKF2 (BSD-3) structure (error state, 24 states, sequential scalar fusion,
delayed-horizon fusion, range-finder and external-vision fusion) and its SymForce-generated equations (Q-D3).
Float robustness practices to copy: error-state attitude; sequential scalar fusion (no matrix inverse); Joseph-form
update with symmetric write-back; per-state variance floors and ceilings; gains zeroed for inhibited states;
innovation gating; latitude and longitude in double outside the filter.

#### 6.3.3 B3: self-calibration and INDI

**What existing stacks do** (checked against source):

- **PX4 multicopter autotune** fits a 2-pole / 2-zero ARX model by recursive least squares from closed-loop
  square-wave excitation and computes gains by GMVC, then applies fixed heuristics (integral ÷ 5, attitude P from a
  60° rule, fixed caps)
  ([mc_autotune_attitude_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.cpp),
  [pid_design.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/pid_design/pid_design.hpp)).
- **ArduCopter AutoTune** twitches each axis, adjusts rate D, rate P and angle P in 5 % steps until the bounce-back is
  within `AUTOTUNE_AGGR`, then backs every gain off 25 %
  ([AC_AutoTune_Multi.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_AutoTune/AC_AutoTune_Multi.cpp)).
- Both assume a vehicle that already flies and end in heuristics. Worth copying is their bookkeeping: PX4's
  hover-thrust EKF ([zero_order_hover_thrust_ekf.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.hpp));
  battery internal resistance by RLS on V = OCV − R·I with a user override
  ([battery.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/battery/battery.cpp)); thermal calibration
  polynomials; precedence rules for learned versus user-set parameters.
- **The closest match is TU Delft's INDIflight** (Betaflight fork, GPL-3.0). Blaha et al. (IROS 2024) identify the
  control-effectiveness matrices and, per motor, the time constant, maximum speed, idle speed and thrust-curve
  nonlinearity: 52 parameters in 450 ms of per-motor excitation after a throw, by RLS on 2 kHz eRPM and IMU data. INDI
  gains follow in closed form: D = 1/(4ζ²τ), and each outer loop K_i = K_{i−1}/(4ζ_i²). In simulation all 1000 random
  quadrotors were stabilized with RMS parameter error "typically below 10 %"; in flight, 57 of 57 throws recovered
  ([paper](https://github.com/tudelft/indiflightSupport/blob/main/Documentation/Papers/IROS2024/IROS_ThrowToFlyQuadrotor_ForArxivFinal_receipt.pdf),
  [learner.c](https://github.com/tudelft/indiflight/blob/master/src/main/flight/learner.c)). Ideas only; write our own
  code (core C-4).
- **Bench and flight values differ.** The same paper found thrust-curve nonlinearity κ ≈ 1.0–1.18 in flight against
  0.46 on the bench, and motor lag 26 ms against 20 ms. Bench values are seeds, not answers.

**What physics lets you identify.** From IMU and rotor speed alone, a_z = (k/m)Σω² − g, so only **k/m** and **k·l/I**
are identifiable, never m or I alone. Absolute mass and inertia need an absolute force reference (scale, pendulum,
thrust stand). That decides which parameters stay manual.

**Hard requirements:**

- **SC-HW1:** bidirectional DShot on the PIO pins, eRPM at loop rate (§2).
- **SC-HW2:** motor pole count is a manual entry (eRPM → rotor speed needs it).
- **SC-CPU:** identification runs on core 0. RLS costs about 3n² multiply-adds per sample; the 52-parameter set is
  ≈ 430 per sample, ≈ 0.9 M/s at 2 kHz. Measure with the DWT counter.

**The pipeline:**

| Stage | Identifies | Needs | Acceptance (in SIL, on every zoo vehicle with known truth) |
| --- | --- | --- | --- |
| S0 Manual | pole count, IMU pose, cell count, **mass**, safety limits; inertia from a pendulum or parts build-up | the user | — |
| S1 Bench, props off | gyro and accel biases; IMU and baro temperature polynomials (thermal soak) | IMU, baro temperature | residual per temperature bin within its noise-derived bound |
| S2 Spin-up, props on, strapped down | per motor: idle speed, κ, ω_max, τ; battery OCV and R; motor order and direction | eRPM, V, I | A1 + A2; values are seeds, re-estimated in S4 |
| S3 First hover (altitude hold on seed gains) | hover thrust, k/m, in-flight R, vibration spectrum (number of notch harmonics) | IMU, baro, eRPM, V/I | \|T̂h − Th\| ≤ 1.96σ̂; A1 |
| S4 In-flight excitation | control effectiveness B₁, B₂; refined motor models; loop delay (from the phase slope); frequency responses | mixer-point chirps or steps, amplitude capped by the angular-impulse rule and by tilt < acos(Th/T_max) | A1 + A2; every B entry larger than 1.96σ has the right sign |
| S5 Synthesis | INDI gains with the identified B, D = 1/(4ζ²τ), outer loops K_i = K_{i−1}/(4ζ_i²), ζ from the overshoot requirement; lower D until the phase-margin formula passes with filter lag in the delay | closed form | A3 |
| S6 Verification | small chirp; revert automatically if measured margins are below spec | IMU, eRPM | A3 |

Acceptance rules:

- **A1, consistency.** Parameter NEES ≤ χ²ₙ,₀.₉₅ in about 95 % of Monte Carlo runs.
- **A2, published parity** (small quads). RMS error ÷ mean true value no worse than the worst motor in Blaha 2024:
  τ 4.3 %, ω_max 8.0 %, κ 23.8 %, B₁ 8.6–11.3 %. For larger classes, A1 and A3 alone.
- **A3, closed loop.** Phase and gain margins on the *true* plant, with the true delay, meet QF-3 in 100 % of runs.

Online learning continues in flight: hover thrust (PX4-style EKF), battery R. Each parameter stores {value, source ∈
identified / manual / default, stage and method, log id, time, σ, which of A1–A3 passed}. Precedence: manual >
identified > default; identification never overwrites a locked manual value.

**Rate-loop choice that follows:** INDI with rotor-speed feedback is the natural inner loop for a self-calibrating
controller. It needs the identified B and motor lag, not tuned gains, and degrades gracefully when the model is off
because it measures the angular acceleration it gets. The spine's PID stays as fallback and for A/B comparison in SIL.

#### 6.3.4 B4: vehicle zoo

Published airframes only, chosen for identified parameters, open data for replay and a usable mesh. The zoo spans
40 g to 2.3 kg (about 55× in mass), enough to catch any gain, limit or threshold that secretly assumes one size.

| # | Vehicle | Size | What is identified and open | Gazebo model | What you must still do |
| --- | --- | --- | --- | --- | --- |
| 1 | **UZH NeuroBEM 5-inch** (Armattan Chameleon 6", 2306 motors, 5" three-blade props) | 0.75–0.77 kg, T/W ≈ 4.5 | τ = 33 ms; k_f 1.5625 × 10⁻⁶ N/(rad/s)², κ 0.022 m, 150–2800 rad/s ([config](https://github.com/uzh-rpg/agile_flight/blob/main/envsim/parameters/quads/blackbird.yaml), MIT); inertia diag(0.0025, 0.0021, 0.0043) ([readme](https://rpg.ifi.uzh.ch/neuro_bem/Readme.html)); dataset above | none: generate the SDF from the card; visual mesh of the frame if one can be found (Q-D1) | Settle 752 vs 772 g; confirm k_f and κ by replay. **The spine's vehicle.** |
| 2 | **Crazyflie 2.1 Brushless** | ≈ 40–45 g | Bitcraze thrust-stand fit ([platform_defaults_cf21bl.h](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/platform/interface/platform_defaults_cf21bl.h)); rise/fall motor model and rpm polynomials ([Crazyflow](https://github.com/learnsyslab/crazyflow), MIT); IDSIA flight data; bidirectional-DShot eRPM (firmware 2026.08) | Crazyflow's MuJoCo meshes, converted to SDF | Two published inertias conflict (Crazyflow diag(25, 28, 49) × 10⁻⁶ vs 33 / 36 / 71.9 × 10⁻⁶); settle by pendulum or by fitting the IDSIA data |
| 3 | **Holybro X500 V2** (2216 KV920, 1045, 4S) | 2.0–2.3 kg | 2.28 kg with a 4S 4300 mAh pack, 12.13 N per motor, attitude-level fit (thrust lag 0.071 s, drag) in Crazyflow ([params](https://github.com/learnsyslab/crazyflow/blob/main/crazyflow/dynamics/so_rpy_rotor_drag/params.toml)); inertia "approximated" | MRS x500 (gz, from Holybro's CAD, BSD-3) | Rotor-level numbers from a **documented proxy**: Tyto's T-Motor 2216 900KV + 1045 on 4S ([test](https://database.tytorobotics.com/tests/k9w/t-motor-2216-900kv-1045prop-4s)), flagged as a proxy (kit motor is KV920). Inertia `UNVERIFIED`. **The held-out vehicle (G4)** and the self-calibration "unknown" case. |
| 4 | *(optional)* Crazyflie 2.x brushed | 27–38 g | Bitcraze thrust-stand cubic and PWM/rpm tables; NanoBench | [bitcraze/crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) (MIT) | Use the firmware's constants, not the SDF's (its rotor drag is the RotorS default) |

**Even "Förster" is not one number:** the Crazyflie thrust constant attributed to the same thesis appears as
1.28192 × 10⁻⁸, 1.71465 × 10⁻⁸ and 1.7965 × 10⁻⁸ in three repositories. The card records a source, not just a value.

**The cautionary example:** CTU MRS's C++ multirotor simulator and their Gazebo model of the same x500 disagree (2.0 vs
1.94 kg; 2.7087 × 10⁻⁷ N/rpm² vs 4.2 × 10⁻⁵ N/(rad/s)²; τ 30 ms vs 12.5 / 25 ms)
([sdf.jinja](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/ros2/models/mrs_robots_description/sdf/drones/x500.sdf.jinja)).
This is what "same parameters in two engines" becomes without shared code, and why the quad has one host.

If MARV flies an airframe of its own, it gets a card by the same rules (thrust stand, pendulum, eRPM step tests,
replay check) before it flies.

---

## 7. Decisions needed (quad)

| ID | Decision |
| --- | --- |
| Q-D1 | NeuroBEM 5-inch visual mesh for Gazebo (Armattan Chameleon), if one exists. Physics comes from the card either way. |
| Q-D2 | Avoidance sensors (B5): their range and latency set the agile speed limit (QO-2). Choose them with that formula in hand. Gazebo's lidar and depth sensors need rendering, so B5's T4 tests need headless GPU rendering in CI; revisit a second host only by decision record. |
| Q-D3 | Navigation filter for B2: PX4 EKF2 structure (proposed). |
| Q-D4 | How each Gazebo mode sets its real-time factor: generated world variants from one template, or `set_physics` at start. Decided at L2. |
| Q-D5 | Rate-loop law after B3: INDI with PID fallback (proposed). |
| Q-D6 | Which IMU the spine flies first: the first part whose measured profile (core §5) passes the L6 Allan check. |
