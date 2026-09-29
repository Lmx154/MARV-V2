# MARV V2 — Core contracts

**Status: ACTIVE. Governs every other MARV V2 spec.**
Split 1, 2026-09-28. With its three siblings, this replaces *"MARV V2: the next simulator, and the requirements it
has to prove"* (Draft 0, 2026-09-27).

Implementation repository: `/home/luis/Projects/CPP/MARV-v2`. Reference repository ("the toolbox"):
`/home/luis/Projects/Web/avionics-toolbox` (Flight Lab, `src/lib/**`, `docs/physics.md`, `docs/sim-architecture.md`).
Board evidence: `/home/luis/Documents/kicad/MARV-V2`.

---

## 0. How the specs fit together

| Spec | Status | Owns |
| --- | --- | --- |
| `00-core-contracts.md` | **ACTIVE** | Everything two products share: number rules, conventions, the firmware boundary and embedded requirements, sensor classes, physics ownership, testing and freezing, the functional interface |
| `10-quad-flight-software.md` | **ACTIVE** | Quad firmware, its Gazebo simulation, the spine build order and its branches |
| `20-ground-segment.md` | PARKED | GCS, link codec, config-file system, flashing |
| `30-rocket.md` | PARKED | Rocket firmware, RocketPy harness, apogee MPC, CFD, recovery |

**Rules:**

1. **Agents work only from ACTIVE specs.** A PARKED spec may be read for context. It is never implemented from. It
   becomes ACTIVE only when Luis edits this table.
2. **The core owns anything two products share.** Product specs reference it and never redefine it. If a product
   spec contradicts the core, the core wins and the contradiction is a bug to fix in the product spec.
3. **Draft 0 is retired.** It is archived at `~/Documents/Notes/marv-archive/`, outside both repositories; agents do
   not read it (decision C-5).
4. **A frozen requirement or test changes only through a decision record** (§7.3).
5. **These four specs are the only planning documents.** No roadmap, plan or milestone file exists beside them. The
   quad spine (`10-quad-flight-software.md` §4) is the build order. A task file, if one is used, names the layer it
   executes and adds no scope.

---

## 1. What MARV V2 is

A new, modular GNC firmware in C++ for the MARV V2 board (RP2354B), plus the software to build, prove, load and
operate it. It follows the Flight Lab's architecture: compose a loop from blocks, prove it in simulation, flash
exactly that composition.

Three products share this core:

- **Quad:** Freestyle (acro, angle) and Mission (Stabilized, Agile) modes.
- **Rocket:** apogee control by airbrakes (estimator + MPC), and recovery events.
- **Ground segment:** the operator's application and the link to the vehicle.

**Authoritative inputs.** Only two:

- **MARV V2:** this firmware and the board it runs on. The board's current sensors are one sensor profile among any
  the system supports (§5).
- **The toolbox:** the Flight Lab's architecture, its physics layer and its rules.

Nothing from earlier projects is assumed or reused: no earlier firmware, simulators, ground software, CV code,
airframes or protocols. External sources are published work, linked where they are used.

---

## 2. Numbers: every one has a provenance

A number in MARV V2 is exactly one of:

- a **cited constant** (physics, a standard);
- a **sourced parameter** (published, measured, identified, datasheet), with its uncertainty;
- a **derived value**, produced by a stated rule from other numbers;
- a **design-budget choice**, recorded in the register (§2.2) with its rationale;
- a **scenario value**, labelled as such (site rules, pilot preference).

Rules:

1. **Unknown numbers are not guessed.** State the rule that will produce the number, or tag it `UNKNOWN` and stop to
   ask. An `UNKNOWN` becomes an explicit task.
2. **Conflicting sources are both kept.** The entry is `UNVERIFIED` until data settles it. The losing value stays,
   marked refuted, with the log that refuted it.
3. **A measurement is a source only if it is reproducible.** The script, its inputs and its raw output are committed.
   Session notes are not a source.
4. **Claims about third-party code** (a bug, an undocumented behaviour) are `INFERRED` until a committed test
   demonstrates them.
5. **Troubleshooting never edits numbers in code.** Experiments inject values through the parameter interface or the
   test harness (sweeps, perturbations) and log the results. The code does not change during an experiment, so
   there is nothing to forget to revert.

### 2.1 The vehicle card

One YAML file per vehicle is the single source of truth for its physics.

```yaml
# vehicles/uzh_neurobem_5in.yaml  (values as published; the simulated sensors are MARV's parts)
mass:          {value: 0.772, unit: kg, method: published, source: "NeuroBEM dataset readme",
                conflict: "0.752 kg in Bauersfeld et al., RSS 2021", status: UNVERIFIED}
inertia_diag:  {value: [0.0025, 0.0021, 0.0043], unit: kg m^2, method: published, source: "NeuroBEM dataset readme"}
rotors:
  thrust_coeff: {value: 1.562522e-6, unit: N/(rad/s)^2, method: published,
                 source: "uzh-rpg/agile_flight envsim/parameters/quads/blackbird.yaml (MIT)", check: "NeuroBEM replay"}
  torque_ratio: {value: 0.022, unit: m, method: published, source: "same file"}
  speed_range:  {value: [150, 2800], unit: rad/s, method: published, source: "same file"}
  motor_lag:    {model: first_order, tau_s: 0.033, method: published, source: "Bauersfeld et al., RSS 2021"}
sensor_profile: marv_v2_board_default    # a profile file (§5); swap it to change sensors
```

Note: this example has no uncertainty (σ) on most entries. It fails rule 1 below until each σ is filled or the entry
is tagged. The card linter's first test is that it rejects this file as written.

Rules:

1. Every entry carries `method` ∈ {published, measured, identified, datasheet, derived(rule), design-budget,
   scenario}, a `source` and an uncertainty. Where sources disagree, the card keeps both and marks the entry
   `UNVERIFIED`.
2. Generators write every downstream copy: the Gazebo SDF and plugin configuration, the RocketPy configuration, the
   linear design models used by controller tests, and the firmware's default parameter set. No number is typed by
   hand into an SDF, a script or firmware.
3. CI rejects a card entry without provenance. Every run report prints the card hash and lists its `UNVERIFIED` and
   design-budget entries.
4. The firmware stores each parameter with its provenance: `identified`, `manual` or `default-from-card`. Manual
   entries are allowed and flagged. Identification never overwrites a parameter the user has locked.

### 2.2 The design-budget register

Design choices are numbers too. They live in one file, `design/budget.yaml`: each entry has a value, a rationale and
the requirements that use it. Requirements reference entries by name and never restate the value.

Entries Draft 0 uses without a value, or with an unrecorded rationale:

| Entry | Used by | Meaning | State |
| --- | --- | --- | --- |
| `PM_min` | QF-3, attitude-loop design | minimum phase margin | Draft 0 uses 45°, "the toolbox's stated rule". Record that rule as the rationale. |
| `chi2_gate_quantile` | QM-1, estimator gates | innovation gate quantile | Draft 0 uses 99.9 % (toolbox rule). Record it. |
| `p` | QM-2, RK-1, RK-9, REC-2, REC-3 | required confidence / probability | no value |
| `k` | QS-3, QO-2 | σ multiplier on position and range margins | no value |
| `b` | QS-1 | allowed motion blur, pixels | no value |
| `E` | RK-1 | apogee error tolerance | derived from accepted points lost (see `30-rocket.md`) |
| false-alarm budgets | QX, GS-6, flight-event detectors | per detector | no value |
| CV phase budget | QS-4 | phase allowed to CV latency | no value |
| batch plan | SIM-3 | vehicles × scenarios × draws × flight length, and the time window | no value |

Each value is filled when the first layer that tests against it is specified, not before.

### 2.3 Draft 0 numbers that fail these rules

Fix these before anything depends on them:

| Item | Problem | Action |
| --- | --- | --- |
| Plant-step costs and RocketPy wall times, "measured here" | Scripts are in session notes, not the repository | Tag `UNVERIFIED`; re-measure with committed scripts |
| RocketPy accelerometer sign error | An agent's reading of RocketPy's source | Tag `INFERRED`; a committed test must demonstrate it before the workaround is justified |
| "PX4 reports 6–10× for Gazebo + x500" | No citation | Cite it or delete it |
| IREC Rules and DTEG 2026 | Cited from a third-party GitHub copy | Replace with the official documents |
| CPU cost model, 2–4 cycles per floating-point op | An estimate (labelled as one) | Replace with DWT cycle counts at quad L9 |
| Default sensor profile (ICM-45686, ADXL375, BMP581) | Datasheet figures only, and not yet checked against the board being flown; an earlier MARV board used BMI088 + LSM6DSV32X | Profiles are rebuilt from Luis's measured sensor datasets (§5). Confirm the board's parts against the KiCad schematic before quad L9 drivers are written. |
| Motor numbering and spin directions | Never defined | Resolved: §3 |
| Example card (§2.1) | σ missing on most entries | Fill or tag before quad L1 closes |

---

## 3. Conventions

Fixed for every product. A convention is a definition, not a tuning value; it is verified against the physical
hardware (tilt the board, spin each motor) before anything depends on it.

- **Frames.** World NED. Body FRD, with the body origin at the centre of mass.
- **Attitude.** Hamilton quaternion rotating body (FRD) → world (NED), stored scalar-first as `[w, x, y, z]` and kept
  normalized. When a unique sign is needed (logs, comparisons), use w ≥ 0; when w = 0, the first nonzero of x, y, z
  is positive.
  - *Rationale:* the same convention as PX4's attitude quaternion and ArduPilot's `Quaternion` (q1 = w), so code
    ported from or compared against either needs no reordering.
  - Eigen stores quaternion coefficients as x, y, z, w in memory even though its constructor takes w first. If Eigen
    is ever used, convert only through named accessors, never by memory layout.
- **Signs.** Body rates in rad/s about FRD axes, right-handed: positive roll is right side down, positive pitch is
  nose up, positive yaw is nose right.
- **Units.** SI at every interface: m, s, kg, N, N·m, rad, rad/s, Pa, T, K. Degrees appear only in user-facing text.
- **Time.** Monotonic `uint64_t` microseconds, since boot on the target and since run start in SIL, read only
  through `hal_time_us()` (§4).
  - *Rationale:* identical to the Pico SDK's `time_us_64()` and PX4's `hrt_abstime`. A 64-bit microsecond counter
    never wraps in practice (about 584,000 years); a 32-bit one wraps after about 71.6 minutes, inside a long session.
  - Intervals used in math (dt) are converted once to `float` seconds. The scheduler counts ticks, not microseconds.
  - **A tick is one sample of the primary IMU.** The tick counter is `uint64_t`. A FIFO read that delivers k samples
    runs k ticks. In SIL, one host step advances an integer number of ticks, with one sample set per tick.
  - When the IMU period is not a whole number of microseconds (6.4 kHz → 156.25 µs), SIL's virtual clock keeps exact
    time internally and stamps each sample by truncating to microseconds, as a hardware timer does. dt comes from
    sample timestamps (or the IMU FIFO's own), so SIL and target compute it the same way.
- **Motors.** Three separate things, so any one can change without touching the others:
  1. **Logical numbering (fixed, in code).** Quad X, viewed from above: 1 front-right, 2 rear-left, 3 front-left,
     4 rear-right, the PX4 and ArduPilot quad-X order. The mixer and every test use logical numbers only. Arrays
     indexed by motor use index = logical number − 1, through one named mapping.
  2. **Spin direction per logical motor (vehicle data).** Lives in the card, with provenance; the mixer's yaw column
     comes from it. There is no default: a card without directions fails the linter. (PX4 and ArduPilot's quad X
     spins 1 and 2 counter-clockwise and 3 and 4 clockwise, a common choice.)
  3. **Output mapping (hardware configuration).** Which board motor pad drives which logical motor, stored as a
     permutation in the configuration (the bootstrap struct until the config file exists, so a remap needs a rebuild
     until then). Physical spin direction is set at the ESC (AM32's reverse setting), never inverted in firmware, so
     the mixer's model of direction always matches the card.
  - **Verification:** the motor test (IF-6), props off, spins each logical motor alone. Its position and direction are
    checked against the card by eye. A mismatch is fixed in the mapping or the ESC setting, never in code.
- **Host frames.** Gazebo (ENU world, FLU body) and RocketPy convert once, in their adapter, with unit tests. Nothing
  above an adapter sees a host frame.
- **Numerics.** Primitives, estimator and control code are templated on the scalar type. Every flight composition
  instantiates `float`; the `double` instantiation exists only for the SIL float64 shadow (EMB-5) and golden-vector
  matching (§9). Builds use `-Wdouble-promotion -Wfloat-conversion`, never `-fsingle-precision-constant` (it would
  silently make the float64 instantiation and deliberate doubles single precision); literals reach code as typed
  constants. Host builds use `-ffp-contract=off`. A deliberate `double` maps to the RP2354B's double coprocessor on
  the target and is used only where float range fails (latitude and longitude).

---

## 4. The firmware boundary

- **Two HALs, one firmware.** `hal_target` (Pico SDK) and `hal_sim`. Everything above the HAL is identical in both
  builds.
- **One clock.** The firmware reads time only through the HAL. In SIL the simulator advances it one tick at a time;
  the scheduler runs its rate groups from it, each an integer division of the master rate. Nothing above the HAL
  reads wall-clock time.
- **Sensors enter at the sensor abstraction** as timestamped SI samples per class (§5), exactly where drivers deliver
  them on the target. Drivers sit below that line and are tested on hardware, one per part.
- **Actuators leave as DShot values** (0 = stop, or throttle 48–2047, captured above the PIO encoder) and servo
  commands in µs. DShot values 1–47 are ESC commands (spin direction, 3D mode, save settings); they never pass through
  the actuator-output struct, only through a separate path that works only while disarmed (motor test and ESC
  configuration, IF-6). Bidirectional DShot eRPM and ESC telemetry come back from the motor model.
- **Transports carrying the boundary:**
  - **In-process** (SIL, both Gazebo modes): the firmware is built as a library and the host's lockstep plugin calls
    it once per host step, advancing an integer number of ticks (§3). The GUI and headless modes use this same path. *Change from Draft 0, which used a socket for
    Gazebo.* The entry point is a C ABI (init, tick, shutdown) so C++ hosts and Python harnesses call the same symbols.
    Firmware state is static (EMB-4), so one process holds one firmware instance; parallel runs are parallel
    processes.
  - **Socket** (SIL): the firmware's link port, for the GCS, using the link codec.
  - **USB** (HIL): the codec's HIL messages (IF-9), tick-stamped (tick number, simulated time) so lockstep is exact.
- **HIL injects at the same point as SIL.** The bench sensors do not move, so HIL messages carry SI samples injected
  after the drivers. HIL adds real timing, CPU load and silicon numerics.
- **The link rate caps the HIL IMU rate.** Bytes per encoded sensor frame × IMU rate must fit the USB link's measured
  throughput each way. If it does not, run HIL at a stated, decimated rate.
- **Determinism.** A SIL run is a pure function of (firmware commit, card hash, scenario, seed, pinned tool versions).
  Two runs on the same machine are bit-identical; CI checks this. Host and M33 are not expected to be bit-identical
  (FMA contraction differs): compare with a tolerance, or build both with `-ffp-contract=off` for comparison runs.
- **Parameters.** The firmware reads tunables only through one parameter interface (get by id, with provenance).
  - **Id and record.** The id is a generated enum. Each record holds value (`float` or `int32`), unit, origin ∈
    {`default-from-card`, `default-from-register`, `manual`, `identified`}, method and source (§2.1), σ, and a lock
    flag. IF-5 carries the same fields.
  - **SIL override.** Only the SIL harness can override a value before a run starts (§2 rule 5). The override
    path is not built into flight compositions.
  - **Final form:** values come from the universal config file (`20-ground-segment.md`).
  - **Bootstrap, until that system exists:** values come from a struct generated from the card and the budget
    register (from quad L1; at L0 the generator runs on a test fixture). SIL and target build the same generated
    source; SIL may then apply the harness override.
  - This is a **declared, time-limited exception to guardrail G2** (§7.4). It ends when ground segment phase G3
    passes. The parameter interface does not change when it ends, so nothing above it changes either.

### 4.1 Embedded requirements (EMB)

These apply to every image on the RP2354B. Product specs add part-specific worked examples, never new rules.

- **EMB-1: the hard real-time chain runs from SRAM.** The sensor → filter → inner loop → output chain runs on core 1,
  placed in SRAM (`__not_in_flash_func`). Its worst-case execution time is measured on the target.
- **EMB-2: one master clock.** The primary IMU's sample clock (FIFO watermark) drives everything; every other rate
  group is an integer division of it.
- **EMB-3: schedulability.** Per core, utilization within the rate-monotonic bound n(2^{1/n} − 1) (Liu & Layland
  1973), or exact response-time analysis passes, using measured worst cases, not averages.
- **EMB-4: memory.** Static allocation only; no heap after initialization; stack high-water marks logged.
- **EMB-5: numerics.** Estimators and optimizers in float32 with covariance symmetrization, Joseph-form updates and
  NaN/Inf guards. SIL runs a float64 shadow of each and reports the divergence.
- **EMB-6: logging for replay.** Every raw sample with its monotonic hardware timestamp, every actuator command,
  eRPM, battery sample and estimator state. The IMU FIFO read deadline is FIFO depth / ODR. The RAM log buffer is at
  least the log rate × the card's worst measured SD write stall.
- **EMB-7: parameters with provenance** through the parameter interface (§4).
- **EMB-8: latency compensation.** Each sample is time-stamped at acquisition; estimators fuse delayed measurements
  at their own time.

---

## 5. Sensors are classes and profiles, not parts

The GNC code sees sensor **classes**. A part is a **driver plus a profile**.

| Class | What the GNC code receives | What the profile states |
| --- | --- | --- |
| IMU (gyro + accel) | timestamped body rates (rad/s) and specific force (m/s²), temperature, per-axis saturation flags | white-noise density, bias (turn-on and random walk), range, resolution, rate, latency, mounting pose |
| High-g accelerometer | specific force (SI), saturation flag | the same |
| Barometer | pressure (Pa), temperature | noise, relative accuracy, temperature coefficient, rate, latency |
| GNSS | position, velocity, their reported accuracies, fix state | latency, rate, error correlation time |
| Magnetometer | field vector (T) | noise, range, rate |
| Range (ToF, lidar) | range, beam direction, validity | range limits, noise, field of view, rate |
| Rotor speed | eRPM → rotor speed | resolution, rate, pole count |
| Battery | voltage, current | resolution, divider tolerance |
| CV module | detections (bearing, size, class, confidence) | detection statistics, latency |
| Pyro channel (actuator class) | arm sense, continuity per channel; output: fire | initiator profile (see `30-rocket.md`) |

- **Estimator noise and priors come from the profile** (the toolbox's `tuning.ts` rules), never from constants.
- **A new part** is a driver and a profile, with provenance. The estimator, controllers and simulator do not change.
- **More than one sensor per class is allowed**; the estimator fuses what the profile set declares.
- **The simulator's sensors are generic models of these classes**, inside `marv_plant`, driven by the same profiles.
  They model at least white noise plus a bias random walk, range and saturation, resolution, rate and latency. Finer
  effects (temperature drift, vibration coupling, FIFO formats, full-scale-dependent noise, barometer prop-wash or
  ram pressure) are added only when validation shows they matter.

**Profiles are built from measured datasets.** Luis records a dataset for every sensor part to be tested, and each
part's profile is fitted from it, with the datasheet kept alongside as the comparison. For the numbers to pass §2:

- **Static logs at every setting the profile will be used at** (ODR, full-scale range, internal filter), long enough
  that the Allan deviation curve's minimum is resolved. The record length is set by where that minimum falls, not
  picked in advance.
- **Temperature logged with every sample**, plus a thermal sweep where the temperature coefficient matters.
- **The recording script, the raw data and the fitting script are committed** with the profile (§2 rule 3).
- **A fitted value outside its datasheet bound is flagged**, not silently used.

**Datasheet priors for today's board parts** (the comparison values; the board check is in §2.3):

| Part | Profile figures | Source |
| --- | --- | --- |
| TDK ICM-45686 IMU | gyro FSR up to ±4000 °/s; 3.8 m°/s/√Hz; zero-rate offset ±0.4 °/s (board), ±0.005 °/s/°C. Accel FSR up to ±32 g; 70 / 80 / 110 µg/√Hz at ≤ 8 / 16 / 32 g; offset ±20 mg (board). ODR 12.5–6400 Hz | DS-000489 rev 1.1, tables 1–2 |
| ADI ADXL375 high-g accel | ±200 g; 49 mg/LSB; 5 mg/√Hz; bandwidth = ODR/2, ODR up to 3200 Hz; offset ±400 mg typical | ADXL375 Rev. B, table 1 |
| Bosch BMP581 barometer | noise 0.78 Pa (OSR ×1) to 0.21 Pa (×16); ODR ≤ 240 Hz; relative accuracy ±6 Pa; TCO ±0.5 Pa/K | BST-BMP581-DS004-13, tables 1, 7, 9 |

The toolbox's own sensor models are other parts (ICM-42688-P, BMP390, MAX-M10S, MMC5983MA): other profiles, not
errors.

---

## 6. Physics ownership and hosts

**`marv_plant`** (C++20, C ABI) is one library that holds every physics model except rigid-body integration and
contact:

- rotor thrust and torque, motor and ESC dynamics, battery, rotor and body drag, ground effect, vibration;
- **gravity** (m·g(h), WGS 84), atmosphere (USSA76), wind (power-law shear + Dryden), magnetic field (WMM);
- every sensor output and all sensor noise, by class (§5).

**The host engine** owns only rigid-body integration, contact, world geometry, raycasts and rendering.

- **World gravity is zero in every host.** `marv_plant` applies it, so no host's constant can differ.
- **Engine-side models are off:** Gazebo's `MulticopterMotorModel`, gz-sensors noise and its wind plugin are not used.
- **One random source.** `marv_plant` owns every random number: seeded, one stream per sensor and disturbance.
- **Motor states live inside `marv_plant`**, sub-stepped at its own fixed rate, independent of the host's step.
- **Link origin at the CM** in every generated model, so an applied wrench means the same thing everywhere.

**One host per vehicle:**

| Vehicle | Host | Modes |
| --- | --- | --- |
| Quad | Gazebo Harmonic (gz-sim) | GUI at real-time factor 1 for a pilot; headless at real-time factor 0 (uncapped) for tests. Same plugin, same code path. |
| Rocket | RocketPy | Headless SIL and Monte Carlo |

- **No cross-engine parity is needed**, because no vehicle runs in two engines. Parity only matters when one vehicle
  must behave identically in two engines.
- **Adapter tests stay.** Each host adapter is tested for exact frame and unit conversion: given the same state and
  seed, the wrench and every sensor byte equal a direct call to `marv_plant`.
- **Linear design models are test fixtures, not simulators.** Controller tests run against the linear model the
  controller was designed on, generated from the card. It makes no claim about full-vehicle behaviour, so there is
  nothing to keep in parity with Gazebo.
- **A second host for a vehicle** (for example MuJoCo for raycast-heavy avoidance) needs a decision record. It brings
  back the Draft 0 parity design: shared `marv_plant`, generated models for both, and adapter, trajectory and
  contact-phase parity tests in CI.
- **Keep knowledge and truth apart.** The flight software's model of the vehicle and the simulator's truth come from
  the same card, but truth is dispersed within the card's stated uncertainties, so they never coincide. A result
  where they share one model is a perfect-model result and is labelled as one.
- **Pin versions.** Gazebo, RocketPy and `marv_plant` versions go into every run report. Determinism is promised only
  within one version set and one CPU architecture.

---

## 7. Testing

### 7.1 Test levels

| Level | What runs | Cost | When |
| --- | --- | --- | --- |
| **T1 unit** | single functions and modules, host-free, against analytic answers | ms | every commit |
| **T2 replay** | recorded sensor streams replayed through the firmware; outputs compared bit-exact with golden vectors | ms | every commit |
| **T3 design-model closed loop** | controller against its linear design model: step response, margins | ms | every commit |
| **T4 host SIL** | full vehicle with firmware in lockstep, headless (Gazebo for the quad, RocketPy for the rocket) | seconds per scenario | smoke set every commit; Monte Carlo nightly |
| **T5 pilot SIL** | Gazebo GUI with a human | real time | by hand, never in CI |
| **T6 HIL** | the real RP2354B, lockstep over USB | real time | before every flight campaign |
| **T7 flight** | the vehicle | — | logs update the card and validate the simulator |

**Placement rule.** A test runs at T4 only if it needs something only the host provides: the full nonlinear vehicle
with the firmware in lockstep, contact, world geometry, or the host integration itself. Ask of every new test: could
it run without the host and still prove the same thing? If yes, it runs at T1–T3.

T2's limit: replayed inputs do not react to the firmware's outputs, so replay catches behaviour changes, not
closed-loop instability. T3 and T4 cover that.

### 7.2 Layers, pass bars and freezing

Each product is built as a stack of **layers**. For each layer the product spec states:

- **Builds:** what it adds.
- **Opening:** the interface it exposes upward, as a contract (types, units, rates, valid ranges, ownership).
- **Pass bar:** measurable acceptance criteria, **written before the layer is built**, with every threshold from a
  derivation or the budget register.
- **Freezes:** which tests become regression tests when it passes.

Mechanics:

1. Frozen tests live in `tests/regression/<product>/Lnn/` (two digits: `L00` … `L10`).
2. Every frozen suite runs on every merge; a merge is blocked if any is red.
3. **Every metric test has a negative control:** a deliberate perturbation (e.g. gains × 1.1, one tick of added
   delay) that must break it. A test that cannot fail is caught by its control. Each control is a CI step that
   passes only if the controlled test fails.
4. **Every opening has contract tests on both sides.**
5. A layer is passed when its suite is green on `master` and the commit is tagged `<product>-L<n>-pass` (e.g.
   `quad-L0-pass`).
6. Passing a layer does not mean it will never be reopened. Integration can reveal problems below; the regression
   suites then say at once whether a fix broke what already worked.

### 7.3 Changing a frozen test

A change to anything under `tests/regression/` needs a decision record, `docs/decisions/NNNN-<slug>.md`, in the same
change: what changed, why, the evidence, and Luis's approval. CI fails a change to a regression file that no decision
record in the same change references. This stops "fixing" a failure by loosening its test.

- A **change** is a pull request into `master`, or a push to `master`; CI checks the whole pushed range. Luis's
  approval is his approval of the pull request, or his push. Branch protection is not enforced (Luis, 2026-09-29:
  "just use master instead i don't feel like changing it and branch protection is not important since we;re the only
  ones messing with it").
- Adding a layer's tests under `tests/regression/` in the change that freezes the layer needs no record.
  Modifying or deleting a frozen file always does.

### 7.4 Guardrails, enforced by CI

Rules in prose do not stop an agent or a deadline, so each of these is a CI check:

| # | Guardrail | How CI enforces it |
| --- | --- | --- |
| G1 | No unexplained numeric literals in GNC code. | clang-tidy `readability-magic-numbers` / `cppcoreguidelines-avoid-magic-numbers` over everything under `fw/`, allow-listing only 0, 1, 2 and ½. Cited constants (π, WGS 84, USSA76, χ² tables…) live in one constants header, each with its citation; it is the only exempt file. |
| G2 | Every tunable is a parameter with provenance. | The generator refuses a parameter without a card, register or rule source. The firmware has no compiled-in tunable numbers, except the declared bootstrap exception (§4). |
| G3 | The flight software never sees the truth. | `truth` kinds and truth-fed modes exist only in SIL harness targets, under the `marv::truth` namespace (`marv_truth_` for C symbols). No flight composition, host or target, may contain such a symbol (checked on each artifact's symbol table), and no flight target may have a harness include directory on its include path (header-only code inlined into a flight translation unit leaves no symbol). A validation run that used one is rejected. |
| G4 | Nothing is tuned to one simulated vehicle. | Once the vehicle zoo is active: Monte Carlo over dispersions and every zoo vehicle, plus a held-out vehicle never used in development. |
| G5 | "It flies" is not a result. | The validation report checks each derived requirement with its margin; a missing requirement is a failure. |
| G6 | The simulator must match reality, not only itself. | Replay against published flight data runs in CI once that branch is active. |
| G7 | Only validated configurations fly. | Once the ground segment exists: the GCS flashes and uploads only (image, config) pairs with a passing report. |
| G8 | Agents get the rules, and CI holds them to them. | `CLAUDE.md` states the number rule, the ACTIVE-spec rule, the `UNKNOWN` rule and these gates; CI fails if any of those sections is missing. CI enforces G1–G7 whether or not anyone reads it. |

`CLAUDE.md` must also say: an agent that needs a number it cannot source tags it `UNKNOWN` and stops to ask.

### 7.5 The convergence rule for steps and rates

Every integration step, sub-step, simulator step and rate group is chosen the same way: halve it until the quantity
being verified (a margin, a tracking error, an apogee) changes by less than its stated uncertainty. The result, and the
halving sequence that produced it, go in the run report. No step or rate is picked by feel.

---

## 8. The functional interface

The interface between vehicle, GCS and simulators belongs to the core because both sides depend on it. The ground
segment spec implements the GCS side; each vehicle spec implements its side.

**Layering:**

```
  flight software: GNC blocks, mission manager,        GCS components
  parameter store, logger, calibration
        │  functional interface: typed messages              │  the same messages, generated types
        ▼                                                    ▼
  link codec: MAVLink v2 + MARV dialect (today; replaceable) link codec: the same
        │                                                    │
  transport: USB CDC · radio UART · SIL socket          transport: serial port · socket
```

- **One schema defines the interface.** It generates the C++ structs and the TypeScript types.
- **Only the codec knows MAVLink.** No flight-software module and no GCS component includes a MAVLink header or names
  a MAVLink message. CI checks this.
- **Functions are fixed before encoding.** This table is signed off before the codec mapping is finalized.
- Where MAVLink has no field for a semantic, it goes in the **MARV dialect**, never into a reinterpreted standard
  field.

MAVLink names were checked in `common.xml` / `standard.xml`
([mavlink/mavlink](https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0)). Generated code is
MIT-licensed when embedded in a binary (the generator is LGPL-3 with that exception, per its `COPYING`).

| ID | Function | What must be carried (semantics) | MAVLink today |
| --- | --- | --- | --- |
| IF-1 | Identity and health | Vehicle kind, image hash, config hash (the sections this image reads), firmware commit, validation report id, mode, arming and failsafe state, pyro arm state and per-channel continuity (rocket), heartbeat | `HEARTBEAT`, `AUTOPILOT_VERSION`, `SYS_STATUS`; dialect for the hashes and report id |
| IF-2 | Telemetry | Named streams: navigation state, estimate with covariance summary, estimator consistency (NIS), sensor health, battery, GNC block signals. Each stream's rate is set by the GCS. | `ATTITUDE_QUATERNION`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `ESTIMATOR_STATUS`, `BATTERY_STATUS`, `NAMED_VALUE_FLOAT`; dialect for block signals |
| IF-3 | Commands | Arm / disarm, mode, hold, RTL, land, abort; each acknowledged by sequence | `COMMAND_LONG` / `COMMAND_ACK` |
| IF-4 | Missions | Waypoints, routes, hold, RTL, geofence, CV lock / verify / abort. Uploads are transactional, with read-back. | `MISSION_COUNT` / `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` / `MISSION_ACK`; geofence as `MAV_MISSION_TYPE_FENCE`; dialect for the CV handshake |
| IF-5 | Parameters | Get / set live, with value, unit, **provenance** (method, source, σ) and lock; a set is written back to the active config file | `PARAM_EXT_REQUEST_READ` / `PARAM_EXT_SET` / `PARAM_EXT_VALUE`; dialect for provenance and lock |
| IF-6 | Configuration | GNC block composition read-back; hardware configuration (IMU pose, ports, motor order and direction, pole and cell count); motor test; calibration stages with results and σ | `MAV_CMD_DO_MOTOR_TEST`; dialect for the rest |
| IF-7 | Logs | List and download the full-rate logs | `LOG_REQUEST_LIST` / `LOG_DATA`, or `FILE_TRANSFER_PROTOCOL` |
| IF-8 | Manual control | Stick axes and switches from the radio connected to the GCS; **accepted only in Freestyle** | `MANUAL_CONTROL` |
| IF-9 | HIL | Tick-stamped sensor injection and actuator output for lockstep (§4) | `HIL_SENSOR`, `HIL_GPS`, `HIL_ACTUATOR_CONTROLS`; dialect for the tick stamp |
| IF-10 | Time | Clock synchronization for aligning ground and vehicle logs | `TIMESYNC`, `SYSTEM_TIME` |
| IF-11 | Config files | List, upload, download, validate and activate config files on the FC; activation only while disarmed | `FILE_TRANSFER_PROTOCOL`; dialect for validate and activate |

**What each image compiles.** Handlers an image does not need are not built into it:

| Interface | Drone image | Rocket image |
| --- | --- | --- |
| IF-1, IF-2 | USB and radio | USB and radio, **downlink only** |
| IF-3, IF-4, IF-8 | USB and radio (IF-8 only in Freestyle) | **not compiled** |
| IF-5 live parameters | USB and radio; while armed, only the in-flight allow-list | **not compiled**: parameters change only through the config file |
| IF-6, IF-7, IF-11 | USB; over the radio only while disarmed | USB only, disarmed |
| IF-9, IF-10 | USB (IF-10 also over the radio) | USB only |

No image, on any port, has a command that fires a charge.

---

## 9. Shared code

- **One primitives layer**, header-only, scalar-templated (§3 Numerics), fixed-size: quaternions, small matrices, RK4,
  table interpolation, biquads, χ² tables, atmosphere and gravity. Every product and harness uses it. Each piece joins
  the layer with the first layer that uses it, and its tests join that layer's pass bar: vectors, matrices and
  quaternions at quad L0; gravity, RK4 and interpolation at L1; biquads at L6; atmosphere in B1; χ² in B2.
- **Language:** C++20 in every product and harness.
- **One estimator core with pluggable measurement models.** Quad and rocket estimators share IMU propagation, fusion,
  gating and covariance hygiene; they differ only in measurement models and states.
- **Compile-time composition.** Only the kinds a structure names are compiled (templates, no virtual dispatch in the
  fast loop). Static allocation sized from the structure. `-fno-exceptions -fno-rtti`.
- **Blocks earn their place.** Each kind carries a qualification level in the registry:

| Level | Meaning | Needs |
| --- | --- | --- |
| `reference` | teaching only | — |
| `sim-qualified` | may be used in SIL | T1 tests + its closed-loop requirement tests |
| `flight-qualified` | the only level allowed in a flashed image | T6 timing within the EMB-3 bound, plus float32-vs-float64 shadow agreement |

  The toolbox's `truth` estimator stays `reference` forever.

- **Porting from the toolbox.** The TypeScript blocks are the reference implementation until the C++ replaces them.
  - Record **block-level golden vectors** from the TypeScript blocks (inputs and outputs per tick). Whole-run goldens
    also depend on the TypeScript plant, so they cannot judge a C++ block alone.
  - A float64 C++ build matches the vectors to round-off; the float32 build matches within a tolerance derived from
    float32 epsilon and the block's conditioning.
  - Apply the audit's fixes (Appendix A) on the way.

---

## 10. Decisions needed (core)

| ID | Decision |
| --- | --- |
| C-1 | **Decided 2026-09-28:** quaternion `[w, x, y, z]`, Hamilton, body → NED; time `uint64_t` µs via `hal_time_us()` (§3). |
| C-2 | **Decided 2026-09-28:** logical quad-X numbering in code; spin direction in the card; pad mapping in configuration; direction set at the ESC (§3). |
| C-3 | **Decided 2026-09-28:** no roadmap or plan file. These four specs are the only reference set (§0 rule 5). |
| C-4 | **Decided 2026-09-29:** BSD-3-Clause. Borrow code only from BSD / MIT / Apache projects (PX4, RocketPy, MuJoCo, Gazebo); use GPL-3.0 projects (Betaflight, ArduPilot, INDIflight) as references for ideas only. |
| C-5 | **Decided 2026-09-29:** Draft 0 archived at `~/Documents/Notes/marv-archive/` (§0 rule 3). |
| C-6 | Values for the design-budget register (§2.2), as their layers are specified. |
| C-7 | **Decided 2026-09-29:** one repository holds the firmware, `marv_plant`, cards, generators, the Gazebo plugin and worlds, and (when active) the RocketPy harness. |
| C-8 | **Decided 2026-09-29:** build and CI. CMake + Ninja with presets `host-debug`, `host-release`, `m33` (arm-none-eabi, compile-only until quad L9); GoogleTest pinned by hash; Python 3.12 + uv lockfile + pytest for generators; GitHub Actions running every job in one pinned Docker image, which is also the only place goldens are regenerated. `hal_sim` is plain C++; nothing above the HAL includes Pico SDK headers. |

---

## Appendix A. The toolbox: take, fix, leave

From a read-only audit of `src/lib/physics`, `src/lib/sim/{lab,sixdof}` and `src/lib/calc`. File:line references are
to the toolbox's `main` at the time of Draft 0.

**Take: port the logic to C++, with these fixes.**

| Piece | Where | Port with these fixes |
| --- | --- | --- |
| 15-state ESKF (δp, δv, δθ, accel bias, gyro bias; NED; Hamilton quaternion; Joseph-form update; χ² 99.9 % gates) | `calc/eskf.ts:423-655`, wired in `sixdof/fsw.ts:211-302` | float32 with scalar sequential updates, symmetrization, variance floors and analytic Jacobians (the magnetometer Jacobian is a finite difference, singular near vertical). Add coning and sculling, the IMU lever arm and delayed-measurement fusion. Bias random walk from the datasheet temperature coefficients: `biasWalkFromTempco` exists (`tuning.ts:66`) but is never called. Recover from rejected updates by inflating covariance, not only counting them. Fuse the magnetometer as a vector, not a direct yaw. |
| Apogee predictor: point mass in the vertical plane [h, w_up, v_h], RK4, apogee at the zero crossing, step by convergence | `physics/coast.ts:49-131` | See `30-rocket.md` §5. |
| Apogee MPC | `sim/lab/blocks/controller.ts:190-257` | See `30-rocket.md` §5. |
| Flight-event detectors (z-score for a false-alarm rate over the pad wait) | `sixdof/fsw.ts:304-320`, `blocks/mission.ts:610-634` | Add debounce and timeouts. Re-derive thresholds including vibration, scale factor and temperature, or a real pad triggers "launch". Use whichever accelerometer is unsaturated. |
| Atmosphere, barometric altitude, WGS 84 gravity, χ² gate table | `physics/atmosphere.ts`, `gravity.ts`, `tuning.ts` | As is. |
| Mixers (clip / shift / yaw-last, DShot 48–2047 with idle floor) | `calc/mixer.ts:58-145` | As is. The Lab's own allocation (`blocks/allocation.ts`) clamps at zero thrust with no idle floor, contrary to `docs/physics.md`. |

**Use on the simulation side** (offline design, or as `marv_plant`'s reference): loop shaping (`calc/loopshape.ts`:
run offline and flash the gains), `tuning.ts`, `rocketAero.ts`, `massprops.ts`, the motor and thrust-curve readers,
`rotor.ts`, `wind.ts`, `sensors.ts`, `constants.ts`, and the Lab engine as a harness for comparing the TypeScript
flight software with the C++ port.

**Fix before any of it informs MARV:**

- **Loop timing.** The presets run IMU, estimator and control at `dt = 0.02` s (`presets.ts:23,44,66,87`); the gains
  are designed against that delay, which caps the rate-loop crossover at 8.8 rad/s. The plant sub-step
  (`sixdof/plant.ts:14`, `SUB = 4`) has no convergence rule.
- **Gain design uses the spin-down lag.** `vehicle.ts:132` sets `motorTau: design.lag.tauDown`, Gazebo's spin-down
  lag. Moving τ from 25 ms to 12.5 ms moves the rate crossover from 8.8 to 12.1 rad/s. Use the measured small-signal
  lag at hover.
- **Process-noise convention.** `tuning.ts:6` states Q = N²Δt, but the code uses σ = N√(f/2) (`sensors.ts:116`),
  i.e. N²Δt/2. Kalibr and PX4 use N√f. Pick one and check it against an Allan variance at the configured ODR and
  filter.
- **Precision.** Tolerances that only work in float64: `1e-12`, `1e-300`, finite-difference `eps = 1e-6`. The UKF's
  α = 10⁻³ gives a centre weight near −10⁶, unusable in float32.
- **Quad take-off and landing profiles have fixed durations.** At a 25 m hover they command 7.4 m/s down and 7.1 m/s
  up against a 3 m/s limit (`mission.ts:137-146, 532`). Derive duration from distance and a speed limit.
- **Sensor figures to re-check:** MAX-M10S CEP (multi-GNSS appears to be 1.5 m, not 2.0); MMC5983MA heading (appears
  ±0.5°, not ±1.0°); BMP390 "×32 at 25 Hz" (×32 limits output to about 12.5 Hz); turbulence in `wind.ts` is first
  order on all axes, while Dryden's lateral and vertical components are second order; fin-lift "fade to 0 by 30°"
  attributed to OpenRocket may only be a clamp at 20°.
- **Design choices without a stated rule:** rocket body, fins, wall and motor-mount sizes; flap size (authority is
  never sized); pad calibration of 30 s (should come from the Allan minimum and the attitude error allowed at coast);
  reporting thresholds outside `diagnose.ts`.
- **Sensors hard-coded as parts.** The figures are baked in for specific parts instead of coming from a profile. The
  sensor model has one accelerometer with no range or saturation (`fsw.ts:35-53`), so a high-g accelerometer needs
  its own channel plus a switch-over rule.

**Leave as teaching material:** `ekf-full.ts`, the vertical-channel ESKF scenarios, `diagnose.ts`, `metrics.ts`,
presets, and the "truth" estimator and model-error knobs.
