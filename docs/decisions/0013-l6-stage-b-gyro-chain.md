# 0013: quad L6 stage (b), the gyro chain; first item, the per-push time limit enforced by CTest

This record opens L6 stage (b) (quad spec §4 L6). Its first item is Luis's instruction at the stage (a) close: enforce
the per-push time limit instead of measuring it. Later stage (b) decisions, the build and the close are appended here.

## What changed

**The per-push limit as a CTest TIMEOUT.**
- `cmake/marv_test_timeout.cmake` (new) reads `per_push_check_time_max` from `design/budget.yaml` once, at configure
  time, as `MARV_PER_PUSH_CHECK_TIME_MAX`. No other file carries its value. The read uses a regex and stops with
  `FATAL_ERROR` if the entry is missing. The top-level `CMakeLists.txt` includes the module before the host block.
- Discovered googletest tests get it as `PROPERTIES TIMEOUT` from both callers of `gtest_discover_tests`:
  `marv_frozen_suite` in `tests/regression/CMakeLists.txt` and `tests/unit/CMakeLists.txt`.
- Tests registered with `add_test` (scripts, binaries) get it from `marv_apply_per_push_timeout`. This runs deferred
  to the end of the top-level directory and sets TIMEOUT on every test that has none.
- **One exemption:** the reference-set generation steps, the tests labelled `reference` that `marv_reference_set`
  makes (decision 0011). They are generation steps, not per-push checks. On a fresh tree the L5 set takes about 110 s
  to generate, while CI generates it in an earlier step, where these take 0.03 s. They keep CTest's default timeout.
- **CI** (`ci/run_ci.sh`, core job):
  - `per_push_timeout_applied` (positive check) reads the register independently and requires every host-debug test to
    carry TIMEOUT equal to it, with exactly the two reference steps without one.
  - `per_push_timeout_control` (negative control) configures a separate tree with
    `-DMARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE=1`, so that tree's limit is 1 s, and requires the planted
    `per_push_timeout_planted` test (`tests/regression/quad/L06/controls/`, new) to fail ctest as a timeout. The
    override exists only for this control; no preset sets it.

**Frozen file changed: one.** `tests/regression/CMakeLists.txt`, the `marv_frozen_suite` function. Its
`gtest_discover_tests` call adds `TIMEOUT ${MARV_PER_PUSH_CHECK_TIME_MAX}` to the properties it already passes
(`LABELS frozen`, then the caller's own). Nothing else in the file changes. The change only adds a limit; no predicate,
bound, tolerance or test changes.

## Why

Luis, 2026-10-01, stage (a) approval (decision 0012): "After the push, as the first item of stage (b): close the gap
"nothing enforces the 60 s limit". Set CTest's TIMEOUT on the per-push checks from `per_push_check_time_max`, so an
overrun fails CI instead of relying on someone measuring it."

The limit is S9 (decision 0009, second message, item 4; register entry `per_push_check_time_max`, decision 0012). The
first implementation shadowed CMake's own `GoogleTest` module with a same-named file that redefined
`gtest_discover_tests`, so that no frozen file changed. The lead replaced it with the explicit property in the two
callers. A shadowed standard module is fragile across CMake versions and hidden from the reader, and the positive
check fails CI if any registration path ever misses the TIMEOUT.

## Stage (b): owner decisions (Luis, 2026-10-01, verbatim)

> **1. Vibration as motion at the IMU: yes.** Log the spec gap against B1's table. Most of the vibration a real gyro
> sees is frame and mount motion that a rigid-body plant can't represent, so the IMU is the honest place to inject it.
> Gyro only for now. B1 lists vibration as an input to the estimator's accel noise, so log the accel side as an L7 item.
>
> **2. Amplitude ∝ (ω/ω_hover)²: yes, as a labelled scenario value.** Word the rationale carefully. ω² is the law for
> the unbalance force and the aerodynamic load, not for gyro rate. How much of that force shows up as gyro rate depends
> on the frame. In the simple single-mode limits it runs from about ω¹ (mass-dominated) to ω³ (stiffness-dominated);
> that range is INFERRED. Record the exponent 2 as the middle of that range, unsourced, to be replaced by a measured
> spectrum.
>
> **3. gyro_chain_attenuation_min = 0.1: yes.** In the register rationale, say that it sets both the low-pass at the
> rate loop's Nyquist and the notch depth over ±ε, as one rule: the chain attenuates every unwanted component by at
> least a_min. Also record that 0.01 would cost 49° at crossover.
>
> **4. ESC clock error: neither 0 nor an invented value.**
> - 0 is the optimistic side: if the real ESC's clock is off, the notches miss the harmonics on hardware.
> - AM32, the ESC firmware we use, runs on MCUs that often clock from their internal RC oscillator. Take that
>   oscillator's accuracy bound over the operating temperature range from the datasheet of an MCU AM32 supports
>   (STM32F051 or AT32F421, for example), as we did for the crystal. Cite it and label it a scenario value: "candidate
>   part, no ESC chosen".
> - Q derives from it by rule, and it regenerates when the ESC is chosen and its clock is known.
> - Record the cost next to it: at 1 %, Q is about 3 and the lag is 5.7°.
> - Add to the L9 list: prefer an ESC with a crystal, or measure its eRPM error on the bench.
>
> **5. Sweep top = the largest amplitude that can't saturate the IMU: yes.** The no-saturation bound has to hold at the
> top of the rotor-speed range, with the ω² scaling and the largest body rate the scenarios fly, not only at hover. If
> the 0.75 rad/s already comes from that, say so in 0013.
>
> **6. Close (b) on its T1 lines, with the QF-3 line closing at (c): yes.** That line is marked (b, c), so it closes
> with (c) anyway. Two conditions:
> - At (b) close, put the chain's delay and the 1-sample latency into the design model, and record the phase margin
>   they give at today's gains. Expect it to fall below 45°. That number is stage (c)'s starting point and shows the
>   deficit.
> - Until (c), keep the chain out of the L4 and L5 compositions, so they stay bit-identical, as you planned for the
>   rotor-speed path. Otherwise the frozen L4 and L5 checks start failing during (b).
>
> **7. Rotor-speed path, SI through a HAL pull read and an additive SIL entry: yes.** SI rad/s at the interface matches
> core §3 and core §5's rotor-speed class. Two points:
> - Converting eRPM to rad/s with the pole count is the L9 driver's job. Keep the pole count in the profile, and derive
>   the telemetry grid from it.
> - The SIL entry is a sensor path, so keep it out of the `marv_truth` namespace, which keeps G3 meaningful.

## Stage (b): owner decisions, second round (Luis, 2026-10-01, verbatim)

> **ESC clock error: (b), 2 %. The ESC is now chosen: the MicoAir 55A AM32 4in1.**
>
> - **MCU.** A seller lists its firmware as `AM32_F4A_4IN1_F421_2.17`. AM32's `_F421` suffix names the MCU, so this is
>   an AT32F421. Mark that INFERRED until the chip marking (I'll photograph it) or AM32's build files for that target
>   confirm it.
> - **Profile.** Record the ESC in the rotor-speed profile with clock error ±2 %. Cite the AT32F421 datasheet's
>   internal-oscillator figure over −40…105 °C, with status UNVERIFIED.
> - **Register.** Also add the design-budget entry `esc_clock_error_max` = 0.02, claimed in L06/param_ids, as a
>   requirement on any ESC we fly. A replacement ESC either meets it or forces a deliberate profile change. The firmware
>   reads the figure as a parameter, and Q and everything downstream derive from it by rule. Nothing is tied to this
>   part.
> - **0013.** Record the cost: the worst-corner phase margin at today's gains is 35.5°.
> - **One check before P1 relies on 2 %.** Find out whether this AM32 target clocks the AT32F421 from its internal
>   oscillator or from an external crystal. Read AM32's F421 clock setup for that fact only: AM32 is GPL, so copy
>   nothing (C-4). If it's a crystal, ε drops to the ppm level, Q goes back up toward 12/11/9 and the chain lag toward
>   1.9°. Then the profile figure changes, but the 2 % requirement stays.
> - **Pole count.** The motor pole count (SC-HW2) belongs to the motors. It stays UNKNOWN until I name them.
> - **P6.** Re-run it on whichever figure the check gives.
>
> **P6 low end: (a), the resolution rule, with two additions.**
>
> Why (a) is sound: with fixed coefficients, the notch chain is linear, so tracking accuracy depends on frequency, not
> amplitude. At 356 rad/s the 1.5ε control already fails at k = 0–2, so frequency tracking near the threshold is proven.
> The k = 3 point only asks whether IMU rounding breaks it, and the check can't answer that with 7 counts of allowance.
> In that corner the vibration itself is a few tens of counts, so an untracked residual sits near the IMU's own
> resolution anyway.
>
> Not (b): it's INFERRED, as you say. Not (c): it changes the design to suit the test.
>
> Additions:
>
> 1. **An unquantised companion check.** Run the same grid with the IMU's quantisation switched off through the test
>    harness. Assert it down to ω_th at every k, with all four controls. That proves the filter's tracking over the
>    whole sweep. The quantised check then proves robustness to rounding wherever the IMU can resolve it, so nothing is
>    loosened.
> 2. **A guard on the derived floor.** The asserted region is computed, so it could shrink silently when Q or the
>    profile changes, for example after the crystal check above. Assert that it covers every k at the card's hover
>    rotor speed and above. If the floor ever climbs that high, the test fails instead of quietly proving less.
>
> At every asserted point, all four controls must still fail. Points below the floor are run and printed in the report,
> not asserted. Record the limit in 0013 as you worded it: below the floor, tracking isn't proven at the IMU's
> resolution, and it is still bounded by the bypass rule at ω_th.

Lead notes on the second round:
- **Pole count.** The profile's `rotor_speed.pole_count` = 14 is the simulated reference vehicle's motor (Hobbywing
  XRotor 2306, INFERRED; the vehicle card is uzh_neurobem_5in). It stays as the simulation's value. MARV's own motor
  pole count (SC-HW2) is UNKNOWN until Luis names the motors. The firmware does not need it; the conversion is the L9
  driver's (owner decision 7).
- **Which figure Q uses.** Q derives from the profile's ESC clock error (the part flown). The generator refuses a
  profile figure above `esc_clock_error_max`, the requirement on any ESC.
- **The AM32 clock check** (Luis's "one check before P1 relies on 2 %"; read for facts only, nothing copied, C-4). At
  AM32 tag 2.17, commit `9f6fc6d32e864c9a60a0f680279d5c9cf38e3e7b`:
  - `Inc/targets.h` defines the `F4A_4IN1_F421` target in `HARDWARE_GROUP_AT_E` (`MCU_AT421`), and `f421makefile.mk`
    builds it for `AT32F421K8U7`.
  - `Mcu/f421/Src/peripherals.c`, `system_clock_config()`, enables the internal HICK, feeds the PLL from
    `CRM_PLL_SOURCE_HICK` and switches the system clock to the PLL. The external crystal (HEXT) is never enabled in the
    F421 code, and no run-time HICK trim was seen.
  - The commutation timer (TMR17) runs on that clock, so the HICK error carries one-to-one into the eRPM telemetry.
  - **Verdict: the ESC clocks from its internal oscillator, so ±2 % stands** (AT32F421 datasheet V2.02, Table 27,
    −40…105 °C, UNVERIFIED). Q stays 2.007 / 1.822 / 1.537, at a worst-corner PM of 35.5° at today's gains.
  - The MCU of the AM32 target is confirmed by AM32's build files. What remains INFERRED is that the MicoAir 55A runs
    this target, on the seller's firmware name, until the chip marking is photographed.
- **The P6 limit (resolution rule).** Below the derived floor, tracking isn't proven at the IMU's resolution, and it is
  still bounded by the bypass rule at ω_th.

## Stage (b): the design (architect consult, 2026-10-01; accepted with the owner decisions above)

- **Low-pass.** A second-order Butterworth biquad on all three gyro axes, every tick, in float. It sits after the
  notches. It is direct form I, bilinear with prewarp, on the nominal tick period from the parameters.
  - Cutoff rule: the digital gain at the rate loop's Nyquist f_r/2 equals `a_min`, so
    tan(πf_c/f_s) = tan(πf_r/(2f_s))/(1/a_min² − 1)^¼. At a_min = 0.1 this gives 625 Hz and 0.28° of lag at 14 rad/s.
  - The generator refuses a rate-loop divisor below 2.
- **One rule, `gyro_chain_attenuation_min` (a_min) = 0.1.** The chain attenuates every unwanted component by at least
  a_min. That sets both the low-pass gain at the rate loop's Nyquist and the notch depth over ±ε. At a_min = 0.01 the
  chain would cost 49° at crossover.
- **Notches.** One per motor per harmonic (4 × 3 = 12 per axis), on all three axes, in direct form I.
  - Coefficients are shared across axes and recomputed at each rate-loop execution from the latest rotor-speed sample.
  - Q_h is the largest Q that still attenuates by a_min at f0(1 ± ε), evaluated at h·ω_max. ε = the telemetry
    resolution 2⁻⁸ + `odr_error` (65 ppm) + the ESC clock error (owner decision 4).
  - Cost: at ESC error 0, Q = 12.2 / 11.1 / 9.3 and the chain plus the 1-sample latency lags 1.9° at 14 rad/s. At a 1 %
    ESC error, Q is about 3 and the lag 5.7°.
  - A notch is active only if its motor's sample is valid and finite, ω ≥ ω_th = ω_hover·√a_min, and h·f < f_s/2.
    Otherwise it is bypassed: its state keeps running, its output is its input, and a per-motor flag and counter are set.
- **Rotor-speed path** (owner decision 7).
  - **Plant:** an opt-in rotor-speed sensor model in `marv_plant`, like the IMU. ω becomes an electrical period through
    the profile's pole count, quantised on the bidirectional-DShot telemetry grid (`eeemmmmmmmmmcccc`, period µs =
    m << e; the citation is to be pinned to a commit). It returns SI rad/s with a valid flag per motor and is delayed
    by one rate-loop period (a labelled scenario value).
  - **Firmware boundary:** `RotorSpeedSample {t_us, ω[4], flags}` in `fw/types`, a HAL pull read, and an additive SIL
    entry. The entry has no digits in its name (the frozen export check accepts `^marv_sil_[a-z_]+$`) and sits outside
    `marv_truth`. Calling the existing `marv_sil_tick` means rotor speed is invalid.
  - The pole count stays in the profile; converting eRPM to rad/s is the L9 driver's job.
- **Vibration** (owner decisions 1, 2, 5). Motion at the IMU, gyro only, added to the truth before the sensor model, so
  it passes through latency, quantisation and saturation.
  - Model: Σ over motors i and harmonics h of A_h·(ω_i/ω_hover)²·sin(hθ_i + φ_ih). θ is the exact per-substep integral
    of the motor model, kept wrapped.
  - The phases are drawn once per run from a new stream id 1. The model has its own opt-in entry; detached or at zero
    amplitude, the stage (a) bytes are bit-identical.
  - **The sweep:** {0} ∪ {A_max·2⁻ᵏ} down to the first amplitude below σ_d. A_max is the largest reference amplitude
    for which 12·A·(ω_max/ω_hover)² + rate_max ≤ the gyro full scale. Here 12 is all four motors' three harmonics in
    phase, ω_max is the top of the rotor-speed range, and rate_max is the largest body rate the scenarios command. So
    the no-saturation bound holds at the top of the rotor-speed range, as owner decision 5 requires, not only at hover.
    The architect's scratch value is about 0.75 rad/s per gyro axis; P1 derives it by rule from the card and the
    register.
  - The sweep is flagged unsourced in every run report.
- **T1 tests:** coefficients against the design formula; steady-state response at f0, f0(1 ± ε), the crossover band
  and f_r/2; fault injection; tracking across the sweep; the Python design block against the firmware. Controls:
  - Q × 1.1;
  - f0 off by one quantisation step;
  - harmonic + 1;
  - motors permuted;
  - rev/s fed in place of rad/s;
  - one coefficient perturbed;
  - an unguarded notch above Nyquist;
  - a one-ulp amplitude change.
- **Design-model block (toward the (b, c) T3 line).** The chain is put into tools/card as the exact discrete loop,
  including the 1-sample latency (0012 D), at its worst-case operating point. At stage (b) close it records the phase
  margin the chain gives at today's gains (owner decision 6). The chain stays out of the L4 and L5 compositions until
  stage (c).
- **Owner question E** is not needed for stage (b): the T1 tests use the adapter's exact clock map. It is needed
  before stage (e)'s T4 corners.

**Spec gap (owner decision 1).** Quad spec §6.3.1 / the B1 table models vibration as body forces. Stage (b) injects it
as motion at the IMU instead, gyro only. A rigid-body plant cannot represent frame and mount motion: 1 rad/s at 175 Hz
on roll would take 2.75 N·m, more than τ_max = 2.44 N·m. The adapter also averages the wrench over each host step.

**Carried forward:**
- **L7:** the accel side of the vibration, which B1 lists as an input to the estimator's accel noise.
- **L9:** prefer an ESC with a crystal, or measure its eRPM error on the bench (owner decision 4).
- **L9:** the eRPM → rad/s conversion with the pole count in the driver (owner decision 7).
- **L9:** measure the notch coefficient-update cost (12 sin/cos pairs at 3.2 kHz, about 2.5–6 %, not in §5.4).

## Stage (b): the build

**P2, the firmware gyro chain** (`fw/gyro_chain/`, `fw/types/include/marv/types/rotor_speed_sample.hpp`; tests in
`tests/regression/quad/L06/gyro_chain/`).
- **`RotorSpeedSample`:** `{t_us, ω[4] in SI rad/s, flags}`. A per-motor valid bit; an invalid motor reads exactly 0.
- **`GyroChain<T>`, float in flight:** 12 notches (4 motors × 3 harmonics) on 3 axes, with shared coefficients, then a
  2nd-order Butterworth low-pass. Direct form I; bilinear with prewarp on the nominal tick period.
  - `update_notches(RotorSpeedSample)` runs at each rate-loop execution.
  - A notch is active iff its motor is valid, ω is finite, ω ≥ ω_th, and f0 < f_s/2. Otherwise it is bypassed: identity
    coefficients, history running, a per-motor flag and a saturating counter.
  - `validate()` refuses non-finite values, period ≤ 0, divisor < 2, cutoff ≥ f_s/2, Q ≤ 0 and ω_th ≤ 0.
- **Not wired into any composition** (owner decision 6).
- **Q definition.** The firmware notch (RBJ form) is exactly the prewarped bilinear of (s² + 1)/(s² + s/Q + 1) at f0, so
  its Q is the prototype's Q. **The rule must be evaluated on that prewarped form.** The unwarped form would overstate
  Q by 14 % at h = 2 and 36 % at h = 3.
  - **Correction (P5).** The P2 review evaluated only the upper edge f0(1 + ε) and gave 12.277 / 11.116 / 9.328. The
    digital notch is narrower below f0, so the lower edge f0(1 − ε) binds: at ESC error 0 the rule gives Q = 12.231 /
    11.084 / 9.315. The P2 tests use Q 12.2 / 11.1 / 9.3 as labelled test values only; nothing in `fw/` carries a Q.
- **Known limit:** a non-finite gyro input would poison the filter state. The IMU validity rule (invalid gives
  exactly 0) keeps valid flight input finite. Whether the chain guards itself is a stage (c) fault question, when it is
  wired in.
- **Checks:** 22 T1 tests (coefficients, DTFT response, fault injection, and the seven controls); m33 builds with zero
  warnings including `-Wdouble-promotion`; G1 (in the marv-ci image), the constants check and G3 are clean.
  Reviewer: PASS.

**P3, rotor angle and gyro vibration** (`sim/plant/src/plant_model.hpp`, `vibration_model.hpp`, `marv_plant.cpp`; tests
in `tests/regression/quad/L06/vibration/`).
- **Rotor angle.** θ_i per sub-step = c·s + (ω0 − c)·τ·(−expm1(−s/τ)), the exact integral of the first-order motor lag.
  It is wrapped to [0, 2π) after every sub-step. The ω update expression is unchanged, so `marv_plant_step` is
  bit-identical.
- **Vibration attach.** `marv_plant_vibration_attach` takes A_h for h = 1..3, ω_hover, and an integer exponent p from 0
  to 8 (the cap is a labelled capacity bound; the owner value 2 is passed by the caller).
- **Phases.** φ_{i,h,axis} = 2π·U(seed, stream 1, n), with n = (3·motor + (h − 1))·3 + axis. Stream id 1 is
  `kVibration` (ABI).
- **Injection.** v_axis = Σ A_h·(ω_i/ω_hover)^p·sin(hθ_i + φ), with the vendored musl `sin`. It is added to the gyro truth
  before the sensor model, with the same freshness as the specific force.
- **Bit-identity.** Detached, or with every A = 0, the stage (a) bytes are bit-identical: the IMU quantiser maps −0 to +0.
- **Checks:** 17 T1 tests (θ closed form, wrap, bit-identity, a known-answer test, seeds, saturation, controls).
  Reviewer: PASS.

**P5, the design module** (`tools/card/gyro_chain_design.py`; tests in
`tests/regression/quad/L06/tools/test_gyro_chain_design.py` and the firmware coefficient tool
`gyro_chain_coeff_tool.cpp`).
- **Rules:**
  - LPF cutoff: 625.416 Hz at f_s 6400, D 2, a_min 0.1.
  - Notch Q, closed form on the digital notch: Q = a·x/(√(1 − a²)·|1 − x²|), with x = tan(w/2)/tan(w0/2), the smaller of
    the two edges, at f0 = h·ω_max/2π.
  - ω_th = ω_hover·√a_min: 348.04 rad/s, with ω_hover 1100.59 rad/s at standard gravity, labelled. The plant uses site
    WGS84 gravity; the difference is under 1 %.
  - The vibration sweep: A_max = 0.748 rad/s from 12·A·(ω_max/ω_hover)² + rate_max ≤ FSR. Here FSR 69.813 rad/s is the
    profile's ±4000 °/s, rate_max 11.694 rad/s the scenario register's `rate_max_roll`, and ω_max 2800 rad/s the card's
    speed range. So the no-saturation bound holds at the top of the rotor-speed range with the largest commanded body
    rate (owner decision 5).
- **Loop model.** The exact discrete loop L = C·(1/D)·Σ_k F, with F = (Σ_{i<D} z_s⁻ⁱ)·P_s·H·z_s^(−1), uses `rate.py`'s
  plant constants and corner list unchanged, with unwrapped phase.
  - Cross-check 1: with H = 1 and latency 0 it reproduces `rate.py`'s own margins at every band-box corner
    (|ΔPM| ≤ 3.5e-14 rad).
  - Cross-check 2: the Python coefficients match the firmware's float coefficients within P2's bound (at most 0.50 of
    it).
- **Owner decision 6: the phase margin at today's L4 gains.** Worst corner (J−, τ+), a_min 0.1, odr_error 65 ppm, latency
  1 sample. The notches sit at ω_th, the lowest active frequency; at fixed Q a notch's lag below f0 falls as f0 rises,
  so this is the worst operating point.

  | ESC clock error | Q_h1 / Q_h2 / Q_h3 | chain + latency delay at crossover | worst-corner PM |
  | --- | --- | --- | --- |
  | 0 | 12.231 / 11.084 / 9.315 | 2.37 ms | 43.09° |
  | 0.02 (AT32F421, −40…105 °C) | 2.007 / 1.822 / 1.537 | 11.88 ms | 35.45° |
  | 0.038 (STM32F051, −40…105 °C) | 1.136 / 1.033 / 0.875 | 20.56 ms | 28.52° |
  | all notches bypassed | — | 0.51 ms | 44.59° |

  Today's L4 design has PM_min 45.0000° (nominal 52.06°). Every row is below 45°: that deficit is stage (c)'s starting
  point. Reviewer: PASS.

**P4, the rotor-speed path.**
- **Plant** (`sim/plant/src/rotor_speed_model.hpp`, `marv_plant_rotor_speed_attach` / `_sample`).
  - ω → T_e = 2π/(ω·pole_count/2) → p = trunc(T_e/1 µs) → the smallest e with (p >> e) < 2⁹ → period m << e → ω̂ as
    float.
  - Grid: 3 exponent bits, 9 mantissa bits, 1 µs, from Betaflight `src/main/drivers/dshot.c` lines 206–221 and 395 at
    commit `5a09417ee75e91e81003cf7891bc1f4de86f3a83`, cited for the format only. Betaflight is GPL, so no code was
    taken.
  - Truncation is INFERRED.
  - Stopped: a period ≥ 511 << 7 = 65408 µs (ω ≲ 13.7 rad/s at 14 poles) gives a valid +0, since Betaflight reads 0x0fff
    as zero eRPM. p = 0, a non-finite ω or a negative ω gives invalid 0. Latency is a delay line in ticks.
  - The grid's 2⁻⁸ resolution holds for periods ≥ 256 µs, i.e. ω ≤ 3506 rad/s at 14 poles. That covers the card's
    range up to 2800 rad/s, so ε holds across the flown range.
- **Firmware** (`fw/sil`, `fw/hal`).
  - `marv_sil_tick_with_rotor_speed(first_tick, k, imu, rotor, out)` is additive. It is not a truth symbol and passes
    the export check. It refuses a bad rotor sample, including a negative valid speed (INFERRED physical rule), with
    E_INPUT before any tick runs.
  - `marv_sil_tick` shares the internal tick routine. It runs the same checks, the same order, the same return codes and
    the same loop as before, and stages "no rotor sample", so its old path is unchanged.
  - `hal_rotor_speed()` is a HAL pull read; with no sample staged it returns every motor invalid. No composition calls
    it yet (owner decision 6).
- **Adapter.** `AdapterConfig::rotor_speed` is opt-in. Per tick: IMU sample, rotor sample, SIL, plant step. With it off,
  the adapter is bit-identical to before. `RotorSilCommandSource` is in its own object file, so frozen tests with a
  recording `marv_sil_tick` still link.
- **Checks:** 31 tests (KATs, layout, SIL contract, adapter twin equality, controls). ctest 605/605 debug and release;
  frozen 602/602; m33 and the three Gazebo presets build with no warnings; G1 (marv-ci image) and G3 clean.
  - The L4 and L5 bit-exact replays run inside the gz pytest suites, not ctest. They are checked by the gz-l4 and gz-l5
    jobs of `ci/local_ci.sh` at the stage close.
  - Reviewer: PASS.

**P1, the register, profile and derived parameters.**
- **Register** (`design/budget.yaml`), both design-budget with sigma choice:
  - `gyro_chain_attenuation_min` = 0.1 (owner decision 3);
  - `esc_clock_error_max` = 0.02, the requirement on any ESC (owner, second round).
- **Profile** (`rotor_speed` class):
  - `esc_clock_error` = 2 %. Source: the AT32F421 datasheet V2.02, Table 27, HICK over −40…105 °C. UNVERIFIED, and
    INFERRED only for the seller link; the AM32 2.17 clock facts are cited.
  - The telemetry grid: `telemetry_exponent_bits` 3, `telemetry_mantissa_bits` 9, `telemetry_period_unit` 1 µs, with the
    Betaflight citation, format only.
  - `latency_rate_periods` = 1, a labelled scenario value; ticks = value × `rate_loop_divisor`.
  - `pole_count` is unchanged.
- **Generator** (`tools/card/gyro_chain_params.py`, `flatten.py --out-gyro-chain`). It applies P5's rules unchanged and
  derives the telemetry step from the profile's mantissa bits, 2^−(m−1). It refuses, writing nothing:
  - a profile ESC figure above `esc_clock_error_max`;
  - any UNKNOWN or wrong-unit input;
  - bad mantissa bits.
- **Derived parameters**, with provenance (G2), ε = 0.02397125:
  - `gyro_lpf_cutoff_hz` 625.4161 Hz;
  - `gyro_notch_q_h1` 2.0070, `gyro_notch_q_h2` 1.8224, `gyro_notch_q_h3` 1.5373;
  - `gyro_notch_omega_min` 348.037 rad/s.
- **Where the parameters land.** Every numeric register entry is flattened into every parameter set, so the new ids
  also enter the frozen L4 and L5 composition sets and shift the later ids. Every runtime lookup is by name: SIL
  overrides, scenario overrides, the replay tool, the gz runners and the T3 fixtures. So the shift is safe. Nothing in
  any composition reads the new parameters (owner decision 6).
- **Stale records, informational.** `L04/results/acro_cause/cause.txt` and the L5 `step_cause`, `step_controls` and
  `recovery_cause` results record a SHA-256 of the build's `param_defaults.cpp`, which changes now. Nothing reads those
  lines. They are left as historical run records, like the stale envelope hash in 0011.
- **Q follows the flown part (Luis, approval item 2).** Q derives from the profile's `esc_clock_error`; the tracking
  test reads the same ε the generator used; profile lint rejects a part figure above `esc_clock_error_max`, with a
  2.5 % negative control.
- Reviewer: PASS.

**P6, notch tracking across the sweep** (`tests/regression/quad/L06/tracking/`).
- **Setup.**
  - The plant: frozen L1 fixture, body held at rest; IMU with N = B = 0; vibration (equal A_h, p = 2); the rotor-speed
    model (14 poles, the grid, 2 ticks of latency).
  - The firmware `GyroChain`, configured from the generated parameters, which are cross-checked against the values
    above.
  - 15 speed sets (equal, distinct, single-rotor) × the sweep k = 0..3 (A_max = 0.748 rad/s). A joint least-squares fit
    per component frequency.
  - The bound per group: a_min·Σc plus a derived allowance. The allowance is the IMU's half-LSB through the chain and the
    fit, P2's DTFT rounding bound (measured float-vs-double difference asserted ≤ it), the transient, and the phase
    drift.
- **The power rule** (owner's resolution rule, made exact by the lead). A quantised point is asserted iff the check can
  fail there: for some component group, (G_a − a_min)·c − allowance > 0, where G_a is the exact design gain of the chain
  under control (a) ×(1 + 1.5ε). This replaces a picked threshold of 0.5, which admitted single@356 k = 3, where (a)
  reaches only 0.946 of the bound.
  - 19 of 20 single-rotor points are asserted; only single@356 k = 3 is below the floor. It is printed with G_a,
    allowance and margin.
  - At every asserted point, controls (a) ×(1 + 1.5ε), (b) mis-route and (c) ×2/3 fail. The weakest (a) ratios are
    1.140 / 1.108 / 1.048 / 1.256 for k = 0..3; (b) about 8.8, (c) about 5.25.
  - (d), all bypassed, fails at every point of every set. Nominal (≤ 0.096) and twin (≤ 0.71) pass everywhere.
  - (a), (b) and (c) are asserted on single-rotor sets only: with 12 low-Q notches in cascade, a neighbouring rotor's
    notch also attenuates a mistracked component, so those controls cannot reach the bound on equal or distinct sets.
    "Motors permuted" is not a fault, since Q is the same for every motor of a harmonic; it is asserted to pass.
- **The unquantised companion** (owner addition 1). It feeds the plant's own `VibrationModel` with the plant's θ/ω and
  the IMU latency, without the quantiser, and is checked each tick against the plant's IMU output to within half an LSB.
  It is asserted down to ω_th at every k: nominal ≤ 0.097, and every control fails, the weakest being (a) at 1.175.
- **The floor guard** (owner addition 2). Every k is asserted at every single-rotor set whose spinning rotors are all
  ≥ 1100.591 rad/s (the hover speed). It is applied to the single-rotor sets, the only ones that can carry the power
  claim.
- **The limit** (owner's wording). Below the floor, tracking isn't proven at the IMU's resolution, and it is still
  bounded by the bypass rule at ω_th.
- **Run time:** the two sweep tests take 10.8 s each in debug and 2.6 s in release, under the 60 s TIMEOUT.

**Frozen file changed: `tests/regression/quad/L06/CMakeLists.txt`** (committed with stage (a)). Four `add_subdirectory`
lines are appended, for `gyro_chain`, `vibration`, `rotor_speed` and `tracking`, the stage (b) suites. Nothing else in
the file changes.

**Frozen file changed: `tests/regression/quad/L06/param_ids`** (committed with stage (a)). Seven ids are appended:
`gyro_chain_attenuation_min`, `esc_clock_error_max` and the five `gyro_*` derived parameters, plus a comment line.
Nothing is removed.

## Evidence

2026-10-01, on the working tree on top of `d481100`:
- **Fresh configure, build and ctest of host-debug and of host-release:** each 535/535 passed. In each, 533 tests carry
  TIMEOUT = 60 s; the two without are `marv_reference_quad_L04_t3_ensure` and `marv_reference_quad_L05_t3_ensure`.
- **The two CI functions, run from `ci/run_ci.sh` on the host:**
  - `per_push_timeout_applied`: rc 0 ("tests: 535, with TIMEOUT = 60: 533").
  - `per_push_timeout_control`: rc 0. The planted test failed as `per_push_timeout_planted (Timeout)` after 1.00 s.
- **Control of the positive check:** with the TIMEOUT taken off `tests/unit`, `per_push_timeout_applied` failed (rc 1,
  naming `Smoke.HostTestPathWorks`). With it restored, the check passed (rc 0).
- **m33** configures and builds.
- `bash -n ci/run_ci.sh`: ok.
- **`ci/local_ci.sh core`** on `cca4040`, a temporary commit of this change on `d481100`, with `MARV_CI_BASE_REF=d481100`
  (marv-ci, as root, 4 CPUs): ALL STEPS PASSED in 609 s.
  - ctest 535/535 in debug and in release; frozen 534/534.
  - `per_push_timeout_applied` and `per_push_timeout_control` both PASS.
  - Regression-change check: 1 frozen file, 1 decision record.
- **Margins (Luis's first check).** CI runs ctest serially: the presets set no `jobs`, and `run_ci.sh` passes no `-j`.
  So a test's wall time is its own run time. The slowest tests are the three L6 Allan tests, at most:
  - debug 10.02 s: 49.98 s under the 60 s limit, 6.0×;
  - release 4.48 s: 13.4×;
  - the frozen-suite run 4.29 s.

  No margin is thin.
- **Exemptions (Luis's second check).** The two reference setup tests carry no TIMEOUT. In CI they take 0.03 s, because
  the set is generated in an earlier step; on a fresh tree the L5 set takes about 110 s. There are no nightly-only
  checks today: no test carries a nightly label, and `MARV_CI_MODE=full` runs the same steps. When one is added, it is
  labelled and excluded from the per-push TIMEOUT in the same change.

- **Stage (b), `ci/local_ci.sh`** (marv-ci / marv-ci-gz, as root, 4 CPUs, the measured memory limit,
  `MARV_CI_BASE_REF=fb5edff`):
  - **gz-l2, gz-l4, gz-l5** on `cb635b9`, the stage (b) tree before its final record text: PASS.
    - gz-l4: 40 passed, 1 xfailed.
    - gz-l5: 63 passed, 1 xfailed.
    - These suites include the L4 and L5 bit-exact replays, so the shifted parameter ids and the new SIL code change no
      DShot output.
  - **core** on `8324bdf`, the final tree (`cb635b9` plus this record and the handoff): ALL STEPS PASSED in 665 s.
    - ctest 610/610 in debug and in release; frozen 609/609.
    - Tools tests: 963 passed, 1 skipped (the skip is pre-existing).
    - Regression-change check: 2 frozen files (`tests/regression/quad/L06/CMakeLists.txt`,
      `tests/regression/quad/L06/param_ids`), 1 decision record.
    - The per-push timeout steps pass. The slowest checks are the two tracking tests, at 10.7 s in debug.
  - The core job on `cb635b9` had failed only at the regression-change check, because that snapshot's record did not
    yet name `param_ids`.
- **Pre-commit changes (Luis's approval items 2 and 3), after the CI runs above:**
  - **Q follows the flown part.** `flatten.py` writes `marv_params_gyro_chain_design.hpp` into the build tree,
    alongside the derived parameters: ε 0.02397125 and its three terms, with provenance. It adds no parameter ids. The
    tracking test reads ε from it instead of rebuilding it from `esc_clock_error_max`.
  - **The requirement check.** `lint.py` fails a profile whose `esc_clock_error` exceeds `esc_clock_error_max` whenever
    `--budget` is given, and so does `flatten.py`'s lint. CI's L1 lint step already passes the budget.
    `tests/regression/quad/L06/tools/test_profile_budget_lint.py` checks four cases:
    - the committed profile (2 %) passes;
    - a copy at exactly 2 % passes;
    - a copy at 2.5 % is rejected, naming both entries (the negative control);
    - without `--budget`, the check does not run.
  - **The datasheet limits.** `esc_clock_error_datasheet` holds the limits as written, [−2, +1.5] %, citing both
    editions: CN V2.02 2023-10-17 §5.3.7 Table 27, and EN DS_AT32F421 V2.01 2022-06-06 as Luis's cross-check.
    `esc_clock_error` = 2 % is derived from them by max(|−2|, |+1.5|).
  - **The footnote.** The verbatim footnote on the factory-calibrated rows is in the note: (2) "由综合评估得出，不在生产中测试。"
    ("obtained by comprehensive evaluation, not tested in production"). It was read from the rendered page 42 of CN V2.02.
    It corrects the earlier reading: footnote (1), "由设计保证，不在生产中测试。", belongs only to the user-calibration row.
  - **Re-run:** ctest 610/610 in debug and in release; pytest L06/tools + L01: 417 passed; lint exit 0. Generated values
    unchanged.
- **Reviews:** P1, P2, P3, P4, P5 and P6 PASS, each against its packet. P6 was re-reviewed after the power-rule rework.

## Approval

Stage (b) build and close (P1–P6, the two frozen L06 edits, the design and the evidence above): Luis, 2026-10-01:

> **1. Stage (b): approved.** Record my approval in 0013, commit to master, run the core job on that exact commit,
> then push. Don't tag it.
>
> **2. Q derives from the flown part's figure, not from the requirement.**
> - The profile describes the vehicle as built, so the design follows it. If we later fly a better ESC, for example one
>   with a crystal, deriving from the requirement would throw that gain away.
> - The tracking test uses the same figure the generator used, so it tests the Q that actually ships.
> - The requirement becomes a check: profile lint fails if the part's clock error exceeds `esc_clock_error_max`. Add a
>   negative control, a profile at 2.5 % that must be rejected.
> - Record all three in 0013.
>
> **3. The AT32F421 citation: confirmed, with one detail to add.** I checked the English edition, DS_AT32F421 V2.01
> (2022-06-06). §5.3.7, Table 27 "HICK clock characteristics" gives the factory-calibrated accuracy over −40…105 °C as
> −2 % to +1.5 %. The section and table match your Chinese V2.02 reading.
> - Record the asymmetric limits as written. ±2 % is the symmetric bound that contains them, max(|−2|, |1.5|): a derived
>   value with that rule.
> - Cite both editions: the CN V2.02 you read, plus the EN V2.01 as the cross-check.
> - I could not confirm which footnote applies to the accuracy row, because my read went through a summary. Quote the
>   footnote verbatim from your PDF into the profile note.
>
> **Going forward, optional.** Five more result files now carry a stale param_defaults.cpp hash, and each stage will
> add more. New result files could print a "produced at <commit>" line in their header, so a stale hash explains itself
> without a record. Old files stay as they are.

The instruction: Luis, 2026-10-01 (quoted above). The frozen edit to `tests/regression/CMakeLists.txt`: Luis,
2026-10-01:

> **0013, the TIMEOUT: approved.** Before you commit, check two things in the local CI run:
> - Record the slowest frozen test's wall time in debug and in release, and its margin under `per_push_check_time_max`.
>   TIMEOUT measures wall-clock time while ctest runs tests in parallel on 4 CPUs, so a test close to the limit becomes
>   a flaky failure. If any margin is thin, tell me before pushing.
> - Confirm the timeout doesn't apply to the reference setup tests (the L5 set takes about 2 min) or to nightly-only
>   checks.
