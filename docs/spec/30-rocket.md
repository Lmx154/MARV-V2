# MARV V2 — Rocket

**Status: PARKED.** Read for context; do not implement from it until Luis marks it ACTIVE in the core's table.
Depends on `00-core-contracts.md`. Split 1, 2026-09-28.

**What it shares with the quad:** the core's conventions, HAL and clock, embedded requirements EMB-1 to EMB-8 (core
§4.1), primitives layer, sensor classes and `marv_plant`'s sensor models, and the estimator core (core §9). Everything
else here is its own product.

**Host: RocketPy.** The rocket runs in one engine, so there is no parity problem (core §6). Its sensors come from
`marv_plant`'s models evaluated on RocketPy's state, so rocket and quad estimators are tested against the same sensor
code.

**Flagged before activation (core §2.3):**

- The RocketPy accelerometer sign error (§2) is `INFERRED` until a committed test demonstrates it.
- RocketPy wall times (§2) were "measured here" with scripts outside the repository: `UNVERIFIED` until re-measured.
- IREC Rules and DTEG 2026 are cited from a third-party copy; replace with the official documents.

---

## 0. What the rocket image is

- **Apogee control by airbrakes:** an estimator and an MPC; the one control target is the apogee.
- **It controls itself onboard.** It contains no remote-control code: no IF-3, IF-4, IF-5 or IF-8 handler; its radio
  carries telemetry down and nothing up (GS-9). This is safer, and the image is smaller.
- **It fires the recovery charges, as one of two independent recovery systems** (§7). The drone image contains no pyro
  code, and no image has a fire command on any port.
- It reads the universal config file's `rocket` section (target apogee, recovery events) and carries the others
  through untouched.

**Reference rockets:** Camões (RED, EuRoC 2023) and Notre Dame's ACS (NASA Student Launch 2023), plus the Army–Navy
Basic Finner for CFD workflow validation.

---

## 1. Flight phases and airbrake permissions

```
PAD (calibrate: gyro bias, gravity alignment, baro reference, pad temperature)
 → BOOST (launch detected: specific force above the bias bound plus a z-score; `mission.ts` rule)
 → COAST (burnout detected) ── brakes ALLOWED only while every condition holds:
       burnout confirmed (and max-Q passed / above 2,000 m AGL, per the DTEG) · Mach ≤ M_lim ·
       tilt from the launch elevation ≤ 30° (DTEG §7) and inside the model's validity cone ·
       estimator consistent (NIS) · all sensors healthy · power good (retract on abort or power loss)
 → APOGEE (vertical speed below −3.09 σ_vz, the toolbox's rule; brakes already closed, REC-4) → DROGUE event
 → DESCENT → MAIN event (altitude AGL below h_main, REC-3) → LANDED (pyro channels disabled; logging continues)
```

- **Onboard only** (GS-9). Phases advance on the rocket's own detections.
- **Arming.** The rocket is SAFE until the pyro board's physical arm switch goes from off to on **after boot, with the
  FC in PAD** (REC-8). Config activation and flashing happen only while SAFE, on USB. Launch detection needs the
  specific-force test and then a barometric climb consistent with the motor's thrust curve within the time that curve
  predicts; otherwise the FC returns to PAD, so handling on the pad cannot start the flight sequence.
- M_lim comes from CFD and loads, not a round number (RK-6).
- On abort, power loss or any failed condition, the brakes retract. The mechanism must retract with the FC unpowered
  (spring or aerodynamic return); verify on the bench, not in simulation.

---

## 2. RocketPy as the plant, with the firmware in the loop

- **The firmware joins RocketPy as a shared library.** Build the flight software (estimator, MPC, phase logic) as
  `libmarv_fsw` from the same sources as the target; call it from RocketPy's air-brake controller at the firmware's
  rate.
- **The harness models the actuator:** the servo's loaded rate limit and lag, and the brakes set to the *actual*
  position, not the command.
- **Sensors from the active profile.** First step: RocketPy's own sensor classes configured from the profile. Later:
  `marv_plant`'s sensor models on RocketPy's state.
- **Monte Carlo** uses RocketPy's stochastic classes: motor impulse within its certification tolerance, mass and CG,
  drag within its stated uncertainty, weather from soundings or forecast ensembles, rail angle.

**RocketPy facts that shape the harness** (v1.13.0, released 2026-07-21, per Draft 0's reading of the source):

- **API.** `Rocket.add_air_brakes(drag_coefficient_curve, controller_function, sampling_rate, clamp=True,
  reference_area=None, initial_observed_variables=None, override_rocket_drag=False, return_controller=False, …)`
  ([rocket.py](https://github.com/RocketPy-Team/RocketPy/blob/v1.13.0/rocketpy/rocket/rocket.py#L1778)).
- **The controller receives** `(time, sampling_rate, state, state_history, observed_variables, air_brakes[, sensors[,
  environment]])`. `state` is the **truth**: the harness feeds the firmware only from `sensors`, never `state` (G3).
  Use `Flight(time_overshoot=False)` so the controller runs exactly every 1/rate s. Use ≥ 1.13: it fixes fixed-rate
  controllers being called twice per sample (#949), which corrupts a stateful estimator.
- **Deployment is instantaneous** in RocketPy; the harness models the servo.
- **Brake force is axial only:** −½ρV²·A_ref·C_D(deployment, Mach), no lift or moment. An asymmetric-deployment
  failure needs `GenericSurface`.
- **The C_D CSV interpolates by inverse-distance weighting by default.** Load it on a regular grid
  (`Function.from_regular_grid_csv`, or linear) so the plant and the MPC read the same surface.
- **Accelerometer (INFERRED).** `accelerometer.py` appears to form `u_dot[3:6] + (0, 0, −g)` when
  `consider_gravity=True`, evaluating g at `u[3]` (vx, not altitude). A real IMU measures a − g⃗. Until a committed
  test confirms or refutes this, the harness computes specific force from the true state or uses `marv_plant`'s IMU
  model.
- **Sensor gaps.** The barometer is ideal static pressure, with no port lag or Mach error; one team measured about
  0.8 s of barometer lag in flight ([WERT](https://github.com/werocketry/airbrakes-SR/blob/main/flight_sims/README.md)).
  There is no stochastic sensor class.
- **Speed (UNVERIFIED, re-measure):** Draft 0 reported 0.3 s per flight with no controller, and 1.8 / 6.0 / 8.4 s with
  a controller at 10 / 50 / 100 Hz, on a 4-vCPU 2.1 GHz Xeon. Run the firmware's estimator at the IMU rate *inside*
  one controller call, passing the batch of samples since the last call. Pick the call rate by the convergence rule
  (core §7.5).

---

## 3. The reference rockets and the CFD

No open dataset has airbrakes, CFD, wind-tunnel and flight data together. Two published rockets cover it between them.

| | **Camões** (RED, EuRoC 2023) | **Notre Dame ACS** (NASA Student Launch 2023) |
| --- | --- | --- |
| Source | RocketPy example and data, used with RED's permission ([notebook](https://github.com/RocketPy-Team/RocketPy/blob/v1.13.0/docs/examples/camoes_flight_sim.ipynb)) | [EamonTracey/apogee](https://github.com/EamonTracey/apogee), MIT |
| Airframe | 143 mm body, von Kármán nose 0.455 m, four trapezoidal fins, boat tail; SRAD motor "Mariachi"; 22.8 kg with the motor's dry mass lumped in | geometry and mesh inside four Fluent case files (`cfd/models/*.cas.h5`); mass and motor from their MATLAB models and team reports (`UNVERIFIED` until extracted) |
| Brakes | whole-rocket C_D(deployment, Mach) on an 11 × 11 grid, Mach 0.1–1.1; **source not stated; flap geometry not public** | flaps at 0 / 25 / 35 / 40°; Fluent k-ω SST, Mach 0.3–0.6 at 265 and 295 K; their fitted polynomial in `src/acs2024/control.py` |
| Flight data | COTS altimeter altitude and acceleration at 100 Hz (apogee 3,016 m at 23.8 s); GNSS; deployment history 0–19.6 s; ECMWF reanalysis ([data](https://github.com/RocketPy-Team/RocketPy/tree/v1.13.0/data/rockets/camoes)) | 20 Hz log: altitude, BMP altitude, IMU and ADXL acceleration, Euler angles, servo and target servo angle, Kalman states, projected apogee; scoring altimeter 4,908 ft AGL, 20.2 g peak axial ([repo](https://github.com/EamonTracey/apogee/tree/main/matlab)) |
| Sim vs flight | 0.39 % apogee (RocketPy's published run) | not yet simulated |
| Its job | Plant and harness validation; the MPC closed-loop in the transonic regime | The CFD-backed brake model and the CFD-workflow check; a second, subsonic MPC case |

**What the rocket data implies:**

1. The apogee predictor must be two-dimensional (height and horizontal speed, with flight-path angle), as the
   toolbox's `coast.ts` already is.
2. Drag tables must span each airframe's Mach range. Camões' table runs to Mach 1.1; the toolbox's coast model stops
   at 0.9.
3. Sensor ranges come from each airframe's Monte Carlo (RK-7). Check values: Camões' altimeter peaks near 93 m/s²
   (≈ 9.5 g), Notre Dame's at 20.2 g axial.
4. MARV's own logs must be built for replay: raw samples, monotonic hardware timestamps (EMB-6).

**Known limits, recorded in their cards:**

- **Camões:** power-on and power-off drag are the same table; the motor's dry mass and inertia are zero, lumped into
  the rocket; the team's real controller is replaced by a time polynomial of the flown deployment. Asking RED for the
  flap geometry would make Camões CFD-able too.
- **Notre Dame:** the CFD is labelled 2023–24 while the flight is April 2023. Confirm the case geometry is the flown
  airframe (their 2022–23 flight readiness report); if not, the flight validates the method, not the numbers. Their
  published runs had convergence checks off, with a fixed 1,000 iterations.

**Steps on Camões:**

1. Replay the logged deployment open-loop through the harness; match RocketPy's 0.39 % and the altitude trace against
   the COTS altimeter.
2. Close the loop with MARV's estimator and MPC, fed from simulated MARV sensors.
3. Perturb the plant's C_D (`StochasticAirBrakes(drag_coefficient_curve_factor=…)`) while the MPC keeps the nominal
   table: robustness to exactly the error the online k_D / k_B estimates exist to remove.

**Steps on Notre Dame (the CFD):**

1. Validate the workflow first on the Army–Navy Basic Finner, with exactly the solver settings to be used.
2. Rerun their four cases properly: convergence monitoring on; three systematically refined meshes each, with the Grid
   Convergence Index reported (Celik et al., J. Fluids Eng. 130(7), 2008); y⁺ consistent with the wall treatment;
   domain-size sensitivity.
3. Extend the grid to the flight's envelope: Mach, Reynolds number and angle of attack from a RocketPy Monte Carlo of
   the Notre Dame rocket; grid points at the envelope's percentiles.
4. Validate against their flight: map servo angle to flap angle with their `servo_percentage_to_flap_angle`; compare
   measured deceleration with CFD's C_D·q·A/m on low-angle-of-attack segments (RocketPy's `FlightComparator` helps).
5. Cross-check the flap increment against EPFL's semi-empirical model
   ([drag_shuriken.py](https://github.com/EPFLRocketTeam/real_time_simulator/blob/main/scripts/aero/Functions/Models/drag_shuriken.py)).
6. Deliverables: a regular-grid CSV; an uncertainty (GCI plus validation error) feeding the Monte Carlo and RK-4;
   hinge moments C_h = M_h/(q·S·c) from the flap pressures (no public hinge-moment data was found).

**Solver.** Fluent opens their case files directly. OpenFOAM with k-ω SST is the open alternative (`rhoSimpleFoam` /
`rhoPimpleFoam` subsonic and transonic, `rhoCentralFoam` supersonic); the
[TUM toolchain](https://github.com/WyllDuck/OpenFOAM-ToolChain-for-Rocket-Aerodynamic-Analysis), validated against
wind-tunnel data, is a starting template; it needs the mesh exported from Fluent or a re-mesh of the extracted surface.

**Accuracy others have flown** (to calibrate expectations, not targets):

| Flight | Controller | Target → achieved |
| --- | --- | --- |
| RED Camões, EuRoC 2023 | not disclosed | 3000 → 3015 m (+0.5 %) |
| Skyward Pyxis, EuRoC 2022 | reference-trajectory interpolation | 3000 → 3033 m (+1.1 %) (from a summary) |
| Clarkson, IREC 2025 | apogee predictor + PI | 10,000 → 10,352 ft (+3.5 %) ([repo](https://github.com/CU-Rocketry/airbrake_mod)) |
| Notre Dame, NASA SL 2023 | 1-D RK4 predictor + PI | 4,600 → 4,908 ft (+6.7 %, from the team's altimeter file) |

MPC has been published only in simulation; no real-flight MPC result was found.

**If MARV flies IREC** (DTEG 2026 v1.0 §7; official source to be substituted): active control may only brake, and the
rocket must be stable without it; brakes stay retracted until burnout, max-Q, or 2,000 m AGL on 10K flights; they
**must retract on abort, on power loss, or beyond 30° from the launch elevation**. Which DTEG revision applies is not
confirmed. The simulations check these rules too (Camões launched at 84°, 6° off vertical). **Scoring sets the
reference frame:** the official apogee is the COTS barometric altimeter's log, so the MPC aims at *that altimeter's*
barometric AGL reading, and its bias belongs in the error budget.

---

## 4. The airbrake MPC at embedded level

**Prediction model.** The toolbox's `coast.ts` shape: a point mass in the vertical plane with
D = ½ ρ V² (C_D(M)·A + ΔC_DA_brake(u, M)), V the air-relative speed, ρ and g from a pad-referenced atmosphere table,
RK4 at a step chosen by convergence (halving the step moves the apogee by less than 1 cm). Three changes:

- **Tables that span the flight's Mach range** (Camões to 1.1; Notre Dame from the validated CFD).
- **Online drag correction.** A drag-scale state k_D in the rocket estimator, observable from axial deceleration
  during coast (a = −D/m); once the brakes move, a brake-effectiveness scale k_B. This is the rocket's
  self-calibration, and it removes the largest single error source.
- **An uncertain start state.** Predict from the estimator's mean; propagate its covariance through ∂H/∂h, ∂H/∂v and
  ∂H/∂k_D (finite differences) to get σ_H, used in RK-3 and RK-4.

**Decision and solver.** One input and slow dynamics, so no full QP solver at first:

1. **Constant deployment to apogee.** Solve H(u) = H_target with u ∈ [0, 1] by Illinois regula falsi, or
   golden-section when a smoothness cost λ‖Δu‖² is on. Re-solve every MPC period (receding horizon). The servo's rate
   limit and lag are inside each rollout.
2. **If (1) leaves error on the table:** a two- or three-segment schedule, solved by Gauss–Newton on the shooting
   residual.
3. **Only if constraints bite:** a small QP-based MPC on the linearized model.

**Rates.** The estimator runs at the IMU rate, with coning and sculling sized by the airframe's Monte Carlo roll rate
(RK-7). The MPC runs at the lowest rate at which the Monte Carlo apogee error stops improving.

**Cost (estimates, to be replaced by DWT counts).** The shooting solver is ≤ 9 rollouts of ≤ 140 RK4 steps per
decision (measured in the toolbox): 0.3–0.6 M cycles, 2–4 ms at 150 MHz, 4–8 % of core 0 at a 20 Hz MPC. A linear
time-varying QP (3 states, 1 input, horizon 50, 20 iterations at 20 Hz) is estimated at 2–5 %.

| Case | Core | Task | Derivation | Share of a 150 MHz core |
| --- | --- | --- | --- | --- |
| Rocket | 1 | IMU + high-g accelerometer at 3.2 kHz, strapdown integration at 1.6 kHz | ≈ 250 ops per step | 1–2 % + logging |
| Rocket | 0 | 14-state ESKF at 100 Hz | scaled from the 24-state cost | 1.4–2.7 % |
| Rocket | 0 | apogee MPC by shooting at 20 Hz | ≤ 9 rollouts × ≤ 140 RK4 steps × 4 × ≈ 30 ops | 4–8 % |

**If a QP solver is ever needed:**

| Solver | For | Against |
| --- | --- | --- |
| DAQP | single-precision build (`DAQP_SINGLE_PRECISION`); reported faster than TinyMPC on the Crazyflie's STM32F405 at 500 Hz | — |
| TinyMPC ([repo](https://github.com/TinyMPC/TinyMPC)) | reported 500 Hz with 12 states / 4 inputs / horizon 15 on the same MCU | ships `typedef double tinytype`; switch it to float |
| OSQP | — | single-precision builds have had correctness reports on 32-bit MCUs; validate in float |
| acados | — | double-centric; not recommended |

**Code discipline.** The MPC compiles from the same source for RocketPy SIL and the target; its float32 answers are
compared against a float64 shadow in SIL (EMB-5).

---

## 5. Porting from the toolbox (rocket items)

| Piece | Where | Port with these fixes |
| --- | --- | --- |
| Apogee predictor | `physics/coast.ts:49-131` | Extend the C_D table past Mach 0.9 (clamped at 0.89 today). Give table steps and the 20 km ceiling an interpolation-error bound instead of fixed values. Add k_D and k_B. Use air-relative velocity with a wind profile if its apogee sensitivity exceeds its budget share. |
| Apogee MPC | `sim/lab/blocks/controller.ts:190-257` | Model the brake's current position, rate limit and lag inside every rollout (today u is reached instantly). Replace the golden-section tolerance Δu = 10⁻⁶ (≈ 0.5 mm of apogee, 31 rollouts) with Δu = max(servo resolution, 1 cm / \|A₀ − A₁\|). Bound iterations by ⌈log₂(range/tol)⌉. Measured cost: ≤ 9 rollouts (mean 6.2) per tick, each ≤ 140 RK4 steps. |
| 15-state ESKF | core Appendix A | Port the toolbox ESKF with the Appendix A fixes. PX4's EKF2 is not built for high-g, high-roll flight. |

**Knowledge vs truth.** The toolbox's apogee predictor and plant share one flap model, so its 0.12 m apogee error is a
perfect-model result that would not survive a real rocket (core §6).

---

## 6. Requirements

| ID | Requirement | Rule |
| --- | --- | --- |
| RK-1 | Apogee accuracy: P(\|H − H_target\| ≤ E) ≥ p, with H as the **scoring altimeter** will report it. | IREC scoring: Points = 350 − [350/(0.3T)]·\|T − A\|, within ±30 % of target. At T = 10,000 ft that is 0.1167 points/ft (11.67 points per 1 %, at any target). E = (points you accept losing) / 0.1167: 43 ft for 5 points, 86 ft for 10. Camões' +0.5 % would have cost 5.8 points. Verified on the Monte Carlo set. |
| RK-2 | Authority. | H_closed at a low percentile ≥ H_target ≥ H_open at a high percentile, across the dispersions. The operator sets H_target in the config file; the GCS and the validation report refuse a target outside this band. |
| RK-3 | Estimator accuracy at brake enable. | σ_H,state = ‖(∂H/∂x) P_x‖ ≤ its share of E. Without drag, ∂H/∂v = v/g: at 200 m/s, 1 m/s of velocity error is 20 m of apogee. The coast model gives the exact number. |
| RK-4 | Drag-model accuracy. | (∂H/∂k_D)·σ_kD ≤ its share of E. Sets how good the CFD must be before flight and how fast k_D must converge. |
| RK-5 | Actuator. | Full-travel time and rate such that the MPC's reachable ΔH covers RK-2 in the coast time left after M_lim. Hinge moment at the worst allowed dynamic pressure × the structural factor of safety sets servo torque, **at its loaded speed**. |
| RK-6 | Deployment permissions (§1). | Competition rules first (DTEG §7). Then physics: M_lim from CFD, with brake pitching moment small against the restoring moment at the allowed angle of attack and loads within RK-5. The attitude cone is the smaller of the rule's 30° and the angle at which the 2-D model's error stays within its share of E. |
| RK-7 | Sensor ranges. | Full-scale range ≥ the Monte Carlo maximum plus its dispersion; the smallest that satisfies this. A high-g accelerometer covers anything above the IMU's range. |
| RK-8 | Pad calibration. | Gyro bias and gravity alignment from the pad dwell; baro reference and ISA deviation from the pad reading. Allowed pad drift from the datasheet temperature coefficients over the expected soak. |
| RK-9 | Keep the ascent inside the cone. | The fraction of Monte Carlo flights whose tilt at brake enable stays below the RK-6 cone is at least p. Rail angle, weathercocking and static margin become part of the airbrake design. |

**What the rocket estimator must handle:**

- **Barometer error at speed:** modelled from CFD at the static-port location or from flight data (Notre Dame's log
  has BMP and scoring-altimeter altitude side by side), instead of a fixed Mach lockout.
- **Saturation:** switch to the high-g accelerometer while the IMU accelerometer is saturated.
- **Rotation:** carry attitude through the airframe's Monte Carlo roll rates.
- **Never dead reckoning alone:** inertial-only altitude drifts with accelerometer bias and scale factor and breaks at
  every clipping event. Fuse the barometer.

---

## 7. Recovery

**Two independent systems.** MARV fires the charges as one of two completely independent recovery systems. The other
is a COTS altimeter with its own arming switch, battery, charges and initiators; they share nothing and neither reads
the other. IREC requires this (DTEG 2026 §6.9, §6.10, §6.11.3); outside IREC it is still the design.

**The pyro board.** The MARV V2 board powers nothing that moves or fires; it carries only signal pins (MARV-V2-PCB
`README.md`; `DESIGN_SPEC.md`, "Actuators"). Recovery charges need an **external pyro board** on the IO block. GPIO44–47
are spare and ADC-capable (MARV-V2-PCB `PINOUT.md`), suited to continuity and arm sensing; PWM5–8 are signal-only, and
the airbrake servo needs one of them.

- **Two switches in series per channel**, on separate GPIOs: a shared enable and a per-channel fire. Gate pull-downs
  hold both off through power-up, reset, flashing and brownout.
- **Its own pyro battery and a physical arm switch** breaking the pyro supply, operable from outside the airframe
  (DTEG §6.15), with the FC reading its state. Battery chemistry per DTEG §6.17 (no LiPo).
- **Continuity sensing per channel**, with sense current below the initiator's no-fire current.
- **Firing** delivers at least the all-fire current for at least the all-fire time, into the highest initiator
  resistance in the profile, from the pyro battery at its lowest allowed voltage.
- The firmware sees the pyro-channel class (core §5); a board is a driver plus an **initiator profile** from the
  e-match datasheet (resistance, no-fire current, all-fire current and time).

**Events** come from the config file's `rocket` section: single-event (main at apogee) or dual-event (drogue at apogee,
main at h_main on descent), and which channel fires which. The GCS and the validation report check every value. For
IREC, dual-event is required above 457 m AGL, with the main no higher than 457 m (DTEG §6.1.1, §6.4.1).

| ID | Requirement | Rule or test |
| --- | --- | --- |
| REC-1 | Never before it is safe. | No charge fires during BOOST, before burnout is confirmed, while estimated Mach is outside the barometer model's validity, or while vertical speed is positive by more than its z-score. |
| REC-2 | Drogue at apogee. | Apogee detected by the estimator (v_z below −3.09 σ_vz), with two independent fallbacks: barometric altitude falling by more than its noise-derived z-score, and a timer at the Monte Carlo's latest apogee time at quantile p. The first to trip fires. Deployment speed below the hardware limit, ½ρv²·C_D·S·X ≤ rated load / factor of safety, with X and rated loads from the manufacturers (card); the Monte Carlo detection-latency distribution keeps v inside that limit at p. |
| REC-3 | Main at altitude. | Fires while descending when estimated AGL altitude is below h_main by its one-sided z-score. Floor: height lost from fire command to full inflation at the drogue rate, plus height to slow to landing rate, plus the altitude estimate's 3σ. Ceiling: the competition's (IREC 457 m AGL) and the drift the recovery area allows (DTEG §6.2). |
| REC-4 | Brakes closed before the drogue. | Commanded closed when predicted time to apogee falls below their full-travel time at loaded speed (RK-5); the drogue does not fire sooner than one full-travel time after the close command. |
| REC-5 | Descent rates. | Drogue and main C_D·S from manufacturers or drop tests (card); descent rates from RocketPy. IREC: 20–40 m/s under drogue, below 11 m/s at landing, computed for an 890 m MSL site (DTEG §6.3, §6.4). |
| REC-6 | Each channel fires once, correctly. | At least all-fire current for all-fire time. Phase, fired-channel mask, pad reference and event state kept in the POWMAN scratch registers, which survive watchdog chip-level resets (RP2350 datasheet §7.3, §12.9.5); after a watchdog reset the FC resumes and never refires. A power-on or brownout reset loses them; REC-8 then keeps the channels unusable and the COTS system recovers the rocket. |
| REC-7 | Logic power survives a firing. | The FC's supply rides through a firing into a shorted channel with no brownout or reset. Verified on the bench with the flight pyro board. |
| REC-8 | No stray pulse, no remote fire. | Channels usable only when the arm switch goes off → on after boot with the FC in PAD. An FC that boots with the switch on never fires. Procedure: power the FC, then arm on the pad (DTEG §6.14.4). Bench test: power-up, reset, watchdog reset, BOOTSEL and flashing, USB plug and unplug, and brownout produce no pulse on any pyro output (logic analyzer on dummy loads). |
| REC-9 | Arming visible from a distance. | Arm state and per-channel continuity in telemetry and shown in the GCS (DTEG §6.16). The GCS pre-flight check fails on an open channel the config uses. |

**Simulation.** RocketPy parachutes take a trigger function called with pressure, height and state (optionally the
sensors and u̇; `rocketpy/rocket/parachute.py`). The harness's trigger returns the firmware's fire output for that
channel, so RocketPy deploys exactly when MARV fires; the parachute's `lag` is the card's fire-to-inflation time. Monte
Carlo includes recovery faults: late apogee detection; a barometer spike near Mach 1; estimator divergence; a watchdog
reset in every phase; a power-on reset in flight.

**Before any charge is connected:**

1. SIL Monte Carlo: REC-1 to REC-6 at quantile p.
2. HIL with dummy loads at the initiator's resistance: each event's timing against the simulated truth.
3. Ground-test demonstration with the sensor electronics included (DTEG §6.13.4.2): the FC in a vacuum chamber
   following the simulated flight's pressure profile, with the HIL link supplying matching IMU samples. Dummy loads
   first, then real charges in the ejection ground test.
4. Charge sizing: black-powder mass from bay volume and shear-pin break force, confirmed by ground ejection tests
   (card: measured).
5. REC-7 and REC-8 on the bench with the flight pyro board.

**Flight ladder.** First flight: the COTS unit fires the charges; MARV's channels drive dummy loads or LEDs and its
event times are compared with the COTS unit's and the logged flight (shadow recovery). After that, MARV fires one
system and the COTS unit the other.

---

## 8. Plan

| Phase | Work | Exit test |
| --- | --- | --- |
| R0 Harness | `libmarv_fsw` called from RocketPy's controller; servo model; sensor profile; resolve the accelerometer question with a committed test. | A Camões flight runs with the flight software in the loop, deterministically. |
| R1 Reproduce Camões | Replay its deployment open-loop with its reanalysis weather; compare with its COTS altimeter and GNSS. | Apogee and altitude trace within RocketPy's published 0.39 % or better. |
| R2 CFD workflow | Basic Finner with the chosen solver settings; Notre Dame's four cases rerun with convergence monitoring and a three-mesh GCI study. | Basic Finner matches its wind-tunnel data within stated uncertainty; each Notre Dame case's GCI reported. |
| R3 Notre Dame model, CFD vs flight | RocketPy model from its files; CFD grid over its Monte Carlo envelope; deceleration at logged flap angles against CFD. | CFD brake increment agrees with the flight within GCI plus validation uncertainty, once the airframe match is confirmed. |
| R4 Estimator + MPC on both | Port ESKF + coast model + MPC with the §5 fixes; add k_D / k_B; Monte Carlo on Camões (transonic) and Notre Dame (subsonic) with drag-table perturbations. | RK-1 met on both, with CFD uncertainty included (Notre Dame) or the table flagged unsourced (Camões). |
| R5 HIL | Lockstep HIL with the same scenarios. | Worst-case times within the EMB-3 bound. |
| R6 Recovery | Pyro-channel class and driver; event logic; RocketPy parachutes triggered by the firmware; reset handling; pyro board bench tests. | REC-1 to REC-9 pass in SIL, HIL and on the bench; the ground-test demonstration passes with dummy loads, then with charges. |

**First real airframe.** The owner's L1 rocket, once it carries MARV. It gets its own card first (measured, with
provenance). Until its brakes have gone through R2–R4 with their own CFD, it flies **shadow mode**: estimator,
predictor and MPC compute and log; the brakes do not move. Recovery follows its own ladder: shadow recovery first, with
the COTS unit firing.

---

## 9. Decisions needed (rocket)

| ID | Decision |
| --- | --- |
| RD-1 | "MCP" in the brief read as MPC (model predictive control). Confirm. |
| RD-2 | Ask RED for Camões' flap geometry, making it CFD-able instead of relying on an unsourced table. |
| RD-3 | Confirm Notre Dame's CFD geometry is the April 2023 flight airframe (their 2022–23 flight readiness report). |
| RD-4 | Which competition MARV targets: sets RK-1 and whether the DTEG 30° rule applies. |
| RD-5 | CFD resources: Fluent (check the student licence's cell limit against the mesh study) or OpenFOAM plus compute for three meshes × four flap angles × the Mach grid. The Basic Finner comes first either way. |
| RD-6 | Report the RocketPy accelerometer issue upstream, once the committed test confirms it. |
| RD-7 | Pyro hardware: an external pyro board on the IO block, or pyro channels on the next board revision. Either is a hardware design with its own review. |
| RD-8 | The COTS recovery altimeter and its settings. Its backup delays come from MARV's Monte Carlo detection times: late enough that MARV fires first, early enough that REC-2's deployment-speed limit holds. |
