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

## Approval

The instruction: Luis, 2026-10-01 (quoted above). The frozen edit to `tests/regression/CMakeLists.txt`: Luis,
2026-10-01:

> **0013, the TIMEOUT: approved.** Before you commit, check two things in the local CI run:
> - Record the slowest frozen test's wall time in debug and in release, and its margin under `per_push_check_time_max`.
>   TIMEOUT measures wall-clock time while ctest runs tests in parallel on 4 CPUs, so a test close to the limit becomes
>   a flaky failure. If any margin is thin, tell me before pushing.
> - Confirm the timeout doesn't apply to the reference setup tests (the L5 set takes about 2 min) or to nightly-only
>   checks.
