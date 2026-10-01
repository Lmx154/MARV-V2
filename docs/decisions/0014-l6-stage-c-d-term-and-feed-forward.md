# 0014: quad L6 stage (c), the D term and ω×Jω feed-forward

This record opens L6 stage (c) (quad spec §4 L6). It holds Luis's stage (c) decisions verbatim, the accepted design,
and the build, evidence and close as they come. The change to R2's initial rotor speeds gets its own record (owner
decision 5, condition 3).

## What changed

Nothing built yet. The plan, in four commits, keeps CI green at each one:
1. **fw:** the D low-pass (T_f = 0 keeps today's unfiltered law), the lag-compensated ω×Jω feed-forward (inert while
   J = 0), and the gyro chain's NaN guard, each with T1 tests. No product change.
2. **tools:** the PI × lead gain rule, the Ms and noise checks, the T_ff rule and the attitude regeneration, with no
   product change.
3. **One atomic switch:** the new product gains, the chain wired into the compositions, every regeneration
   (0011 SHA256SUMS) and frozen edit, plus this record's list of them.
4. **Evaluation results:** FF-on runs of acro and R2 (reported, not asserted), and the `step_cause` re-run.

The known failing items stay strict xfails until stage (e) creates the gate file. FF goes live at (e) with it (owner
decision 2).

## Why

Quad spec §4 L6 stage (c) and decisions 0009 (D1–D7, F1–F3) and 0013 (the chain costs 9.6° at today's gains: the
worst-corner PM is 35.5° against the 45° floor).

## Stage (c): the design (architect consult, 2026-10-01; scratch numbers, the generator recomputes them)

- **Feasibility.** Model: the exact discrete loop of `tools/card/gyro_chain_design.py`, with the chain at ESC 2 %, all
  notches at ω_th (the worst operating point, confirmed on a 9×9 J×τ grid), the 1-sample latency, and the firmware law.
  PI × lead meets the floors at every band-box corner.
- **Gain rule (D3/D4).**
  - Prototype: C = K(1 + ω_i/s)(1 + s/ω_z)/(1 + s/ω_p), with ω_i = ω/2.414, ω_z = ω/√N, ω_p = ω√N, and K from |L_nom| = 1.
  - ω_c(N) by `rate.py`'s sup-PM rule.
  - N* = the largest N within `Ms_max` and the D-path noise budget.
  - At the flown ESC, Ms binds, not noise: N* = 3.878, nominal ω_c 12.13 rad/s.
    - Worst PM 45.00° at (J−, τ+); worst Ms 2.000 at (J−, τ−).
    - D-path RMS 0.494 mN against 2.281 mN.
    - ω_c²Tτ_lo = 0.0071.
  - For comparison, PI alone with the chain reaches 6.02 rad/s.
- **D low-pass (D2).** T_f = 1/ω_p = 1/(√N*·ω_c,nom), the lead's own pole (41.9 ms at N*). T_f = 0 keeps the unfiltered
  L4 law, so the frozen L4 hand tables and the L5 rate-bypass golden stay valid.
- **ω×Jω evaluation (F1–F3).** Variants: PID; plain FF; lag-compensated FF (adds τ_m·d/dt(ω×Jω), with ω̇ from the chain
  output through a first-order T_ff). The input is the chain output at the rate execution. Plants: the card plant and
  the J corners.
  - Scratch predictions on the card plant:
    - D alone fails acro;
    - plain FF still fails;
    - lag-compensated FF passes.
  - Over the J band every variant fails: FF cancels the coupling only where J is known.
- **R2 as specified is infeasible for any realisable controller.** It starts with zero differential thrust while the
  pitch coupling is 0.246 N·m. The motor lag forces an excess over the envelope.
  - **Correction (c3, 2026-10-01).** The "about 1.3e-3 rad/s" tolerance the architect quoted is R1's predicate line
    (`L05/results/recovery_cause/cause.txt:55`), not R2's. R2's tolerance on ω_y is F + Q = 0.1151 rad/s.
  - c3's bound: X* = 0.283 rad/s on ω_y (ratio 2.46) and 0.222 on ω_x (ratio 2.48); ω_z is not forced.
  - Unproven until the condition 1 evidence is committed and reviewed (see the build).
- **Wiring.**
  - One shared rate-group step: `chain.filter` every tick, `update_notches(hal_rotor_speed())` at the rate execution
    (before that tick's filter), and the rate loop on the chain output. It is used by both compositions and the replay
    tools.
  - The Gazebo plugin's rotor-speed and IMU-model wiring stays at stage (e). Stage (c) T4 flies the truth gyro with
    notches bypassed, and its envelopes model that configuration. T3 carries the ω_th worst case.
- **NaN guard (0013 known limit).** The chain resets an axis to the steady state of the next valid finite sample, with
  a flag and a counter, and seeds at the first valid sample, so D sees no start-up kick.
- **Findings for Luis.**
  - At ESC 2 % the closed-loop time constant at the J+ corner is about 0.161 s, barely changed from PI.
  - A crystal ESC would give N* 4.06, ω_c 23.6 rad/s, τ_cl about 0.086 s, with noise then binding.

## Owner decisions (Luis, 2026-10-01, verbatim)

> Good report. Feasibility at 45.0° and Ms 2.000 with the chain in the loop is the result we needed. The answers
> follow, in your order. Two differ from your recommendations: 3 and part of 5.
>
> **1. FF form: lag-compensated.** Plain FF leaves acro failing, so it doesn't meet the bar.
>
> **2. FF goes live at stage (e), with the gate file: yes.** That avoids the unexpected-pass trap. So that (e) is a
> confirmation rather than a first try, stage (c) records FF-on evidence: the T3 evaluation and one T4 run of the acro
> and R2 scenarios with FF switched on through the test harness. These are reported, not asserted.
>
> **3. J corners: physical 3-D corners, not common-scale.** ω×Jω depends on the differences between the principal
> moments, for example (J_yy − J_zz)·ω_y·ω_z. A common scale moves all of J together and never tests those differences,
> which is exactly where FF mis-cancels. Use the vertices of the ±50 % box intersected with the triangle inequalities:
> each moment at most the sum of the other two. That drops the corners that aren't real inertias. The common-scale
> corners stay as a subset, and rate.py's per-axis convention is unchanged for the loop margins. This is a T3
> design-model run, so the extra corners are cheap.
>
> **4. Acro closes at the card plant: yes, with the corner excess carried to a pre-L8 gate rather than to B1/B3.** The
> original ruling (0005 decision 12) was that it must pass before L8. Record the excess at the physical corners from
> item 3. Add to the handoff: before L8, measure J with its σ (S0, pendulum), then re-run the acro check over the
> measured band. The pilot doesn't fly until it passes there.
>
> **5. R2: (a), steady-tumble rotor speeds, under four conditions.**
>
> Why (a) is a setup fix and not a re-seed: R2's own scenario file calls it "the acro limit", a pilot at full stick on
> every axis. A body held at constant ω needs τ = ω×Jω (Euler's equation). So hover rotors don't describe a steady
> tumble: they describe a body that is already being kicked at t = 0. Hover rotors came from 0006 decision 23 as a
> spin-up fix (0 → hover), not as a choice to unbalance the torque. The envelope still comes from the design model,
> never from the observed run.
>
> Conditions:
> 1. **Evidence first.** Commit the lower-bound proof (the 0.13 rad/s motor-lag excess against the 1.3e-3 rad/s
>    tolerance) as a script with its inputs and raw output, with a negative control, and have it independently
>    reviewed. This happens before the predicate changes, not after.
> 2. **Derive the rotor speeds by rule.** Hover collective plus the mixer inverse of τ = ω×Jω at the scenario's ω, from
>    the card. Add a T1 check that the plant's ω̇ at t = 0 is zero within its derived tolerance. Its negative control is
>    hover rotors, which must fail it.
> 3. **Its own decision record.** This changes a known safety item: the frozen check that R2 starts at "hover"
>    (`test_t4_recovery.py:275`) and the scenario. So it gets its own record, not a section of the stage (c) record.
>    The strict xfail and its gating stay exactly as they are.
> 4. **Keep the hover-rotor tumble.** Keep it as a run-and-report evaluation. It is the collision or prop-strike case,
>    and it doesn't disappear. Record its excess, and log in the handoff that this case has no pass bar yet. Propose one
>    at L8, from an absolute recovery requirement rather than the linear envelope.
>
> Not (b): it puts the coupling inside the envelope, so the check would stop asking whether the controller rejects it.
> Not (c): the motor-lag excess persists in the attitude after any start window.
>
> **6. The N\* rule: yes.** The floors (`PM_min`, `Ms_max`) are the margin, so the design sitting exactly on them is
> intended. Don't add a second margin on top. Keep the numeric guard.
>
> **7. Crystal ESC as a requirement: no.** The flown ESC would fail it the day it's written. Record it in the handoff as
> a finding: the ESC clock error is the largest single limit on crossover. Also record a B3 candidate: estimate each
> ESC's clock scale in flight from the gyro's harmonic peaks, which tightens ε without new hardware (INFERRED, an idea
> only).
>
> **8. T_ff rule: reuse `d_path_noise_budget`, no new entry.** Apply it to the combined RMS of the D path plus the FF
> path, at rate_max, in units of the hover DShot step. Rate_max is where FF noise peaks; at hover it is about zero,
> because the term is quadratic in ω. The hover step is the conservative unit. Applying it to the combined total stops
> the budget being spent twice. That is a new line in the spec's (c) pass bar, so send me its text to approve.
>
> **One more finding to log.** A closed-loop time constant of 0.161 s at the J+ corner is the "sluggish" question
> again, and the ±50 % J band drives it as much as the ESC does. Measuring J (S0) and the motor τ_m on the bench shrinks
> the band box and raises the achievable crossover by rule. Put both on the pre-L8 hardware list.

## Stage (c): the build

**c1, the inert firmware pieces (commit 1).**
- **`fw/rate`.** `RateConfig` gains `d_filter_tau` (T_f), `inertia` (J), `motor_tau` (τ_m) and `ff_filter_tau` (T_ff),
  all defaulting to 0, and `ConfigError` gains `DFilter` and `Feedforward` (appended). `from_params()` leaves the new
  fields at 0.
  - **D low-pass:** Df_n = Df_(n−1) + (1 − e^(−dt/T_f))(D_n − Df_(n−1)), behind a branch, so T_f = 0 runs today's exact
    expression.
  - **Feed-forward:** u += g + τ_m·ġ, with g = ω×(Jω) on the loop's measured y and ġ the backward difference through a
    T_ff filter. It is skipped entirely when all J = 0. Sign: Euler, J ω̇ = τ − ω×(Jω), so holding ω takes
    τ = +ω×(Jω); the lag inversion u_req = u + τ_m·u̇ is derived inline.
- **`fw/gyro_chain`.**
  - **Input guard:** a non-finite input holds that axis's last output (+0 before any), keeps its states, and sets a
    per-axis flag and a saturating counter. The next finite sample reseeds the axis to its steady state.
  - **Seeding:** x = y = c is a fixed point because Σb = 1 + a1 + a2 for the notch and the low-pass. First-sample seeding
    sits behind `GyroChainConfig::seed_first_sample`, default false. The frozen `gyro_chain/fault_test.cpp` compares
    against a zero-state reference, so seeding by default would break it; commit 3 sets the flag true in the wiring (lead
    decision).
- **Tests** (`tests/regression/quad/L06/d_term/`): 21. They check:
  - bit-identity to today's law over 3000 ticks with the new parameters at their defaults;
  - the D-filter closed form;
  - the FF known answer (ω = (1, −2, 3), J = (½, ¼, ⅛) → (¾, 9⁄8, ½), exact) and the lag term against a double
    reference;
  - the chain's seeding and guard.

  Negative controls: T_f and τ_m ×1.1 and ÷1.1, permuted J, a flipped sign, T_ff ±10 %, an unseeded chain, and an
  unguarded chain.
- **Checks:** ctest 631/631 in debug and release; m33 with zero warnings; G1 (marv-ci image), the constants check and G3
  are clean. Reviewer: PASS, confirming bit-identity both from the diff and by test, and the FF sign. The two formula
  citations were tightened (Goldstein §5.5 without an unverified equation number; the lag identity derived inline).

**c2, the PI × lead gain rule as a tool** (`tools/card/rate_lead.py`; tests in
`tests/regression/quad/L06/tools/test_rate_lead.py`). Not yet wired into the product parameters (commit 3).
- **The design:**
  - N* 3.877 (Ms binds); nominal ω_c 12.129 rad/s.
  - Worst PM 45.000002° at (J−, τ+); worst Ms 1.99995 at (J−, τ−).
  - T_f 41.87 ms; T_ff 12.31 ms.
  - Combined D + FF RMS at rate_max equals the budget (2.281 mN); at hover the D path is 0.494 mN.
  - t63 at the J+ corner 0.151 s.
  - 12 physical J vertices.
- **Cross-check:** with N = 1, H = 1 and latency 0 it reproduces `rate.py`'s PI to 2.4e-14. A time-domain Monte Carlo
  matches the noise integral to 0.3 %.
- **Rate_max noise operating point (lead decision).** At ω = rate_max on all axes the steady-tumble rotor speeds are
  undefined (one motor would need −0.301 N). So the noise uses the notch-free chain, an upper bound because every notch
  has |H| ≤ 1. That gives T_ff 12.3 ms; the hover-notch convention would give 1.8 ms.
- A guard fix is in progress (α in f32, as the firmware computes it).

**c3, the R2 lower bound** (owner decision 5, condition 1; `tests/regression/quad/L06/results/r2_lower_bound/`,
pytest `tests/regression/quad/L06/tools/test_r2_lower_bound.py`).
- **Proven, given the plant model.** From R2's start (ω0 = rate_max on all axes, hover rotors, zero torque), every
  command history in [0, W_max] forces the truth rate past the design envelope:
  - ω_y by at least 0.28283 rad/s at execution 16;
  - ω_x by at least 0.22176 at execution 13.

  R2's tolerances there are F + Q = 0.1151 (ω_y) and 0.0895 (ω_x); ω_z is not forced.
- **Not proven:** that the predicate fails. The predicate's tolerance is E + F + Q, and E (the m = 1 vs m = 2 run
  difference) is not recorded at execution 16. The check could still pass only if E(16) ≥ 0.1677 rad/s on ω_y.
- **Negative controls:** equal moments, and steady-tumble rotors at the minimal feasible collective. Both collapse the
  bound to 0.
- **Cross-check:** the committed R2 trace's excess is at or above the bound (ω_y 0.563 at execution 16).
- **Independent review: PASS** (2026-10-01). No non-conservative step; the coupling was recomputed by hand; the raw
  output reproduces byte for byte.
- **Condition 2 as worded is infeasible.** Hover collective plus the mixer inverse of ω0×Jω0 needs motor 1 at
  −0.30146 N. The smallest collective that keeps every rotor at or above its minimum is 8.917 N (118 % of hover).
  Question to Luis, pending.

**c2, follow-ups.**
- **f32 guard on the firmware's exact run-time coefficients.** It computes α in float, with dt = float(dt_us)/1e6, and
  analyses the loop as 2-periodic (312/313 µs) by a lifted harmonic transfer, which reduces exactly to the LTI loop at
  equal dt. It checks `expf` at 0 and ±2 ulp (INFERRED bound).
  - Result after one step-down: N* 3.877369, ω_c 12.129191 rad/s, guard PM 45.000042°, Ms 1.999943, T_ff 12.3116 ms.
- **Cross-check tests.** Each has a control that fails by a wide margin.
  - The equal-dt reduction.
  - The tool's controller against the compiled firmware `RateLoop<float>`: within 0.10 of the f32 error bound, with α
    bit for bit.
  - The coherent D + FF noise formula against a 120k-execution seeded brute-force simulation: within 0.04–0.24 %
    against a 4σ bound of 1.1 %.
- **Parallel N search** (`--procs`, the `attitude_t3_oracle.py` pattern). Byte-identical to the serial run for every
  process count. The design takes about 30 s on 4 cores, down from 56 s serial.
- Reviewer: PASS. The loop definition (D on the measurement) and the periodic analysis were re-derived.

**c4, the attitude gains on the new rate loop (D7)** (`tools/card/attitude_lead.py`; `attitude.py` refactor; tests
`tests/regression/quad/L06/tools/{conftest.py,test_attitude_lead.py}`).
- **The `attitude.py` refactor** (lead-approved, minimal): `Loop._evaluate_grid`, `Loop.radius`, `Loop.release_crossing`
  and `yaw_terms`. The PI path is byte-identical: the attitude and rate yaml/report files compare equal against HEAD's
  `attitude.py`, and the L05 tools pass 149. Only the order of the `alpha_min` refusal changes, on the refusal path.
- **The design on the stage (c) inner loop** (rate_lead's N* gains, the chain at ω_th, 1-sample latency):
  - k = 3.2277713 (today 3.0872879);
  - PM nominal 61.268°, worst 45.000016° at yaw (J+, τ+);
  - crossover 3.837–4.376 rad/s; t_cross 0.274375 s;
  - w and α_min unchanged; att_loop_ratio 1.
- **Stability (0006 E step 5) for the 34-state loop.** The Jury table fails numerically at that order, and a forward
  error bound on repeated squaring stalls above 1. So stability is proved by a Lyapunov certificate:
  - P from Smith's doubling;
  - rigorous Gershgorin lower bounds with Higham rounding bounds on λ_min(P − AᵀPA) and on λ_min(TᵀPT), with T = L⁻ᵀ
    from P's Cholesky factor (an invertible congruence);
  - both positive ⇒ ρ(A) < 1 for the double matrix with f32 gains. That is the same scope as the Jury check had for the
    PI loop.

  All 15 loops are certified (margins 0.93–1.0). The lead added a non-finite guard (a NaN row fails the certificate).
- **Checks:**
  - Cross-check: today's PI reproduces `attitude.py` within 2⁻³⁰.
  - The squaring radius agrees with Jury on the PI loop (≤ 1.4e-6).
  - Controls: latency +1; gains 10 % above the stability limit are not certified, 10 % below are. In an independent
    stress test the certificate never passed an unstable matrix (300 random matrices).
  - Reviewer: PASS on soundness.

**c5, the ω×Jω evaluation, L4 acro** (`tools/sim/l6_ff_eval.py`; results and README in
`tests/regression/quad/L06/results/ff_eval/`; pytest `tests/regression/quad/L06/tools/test_l6_ff_eval.py`).
- **The model** reproduces the committed acro gz cause analysis for today's PI: 1378/1036/352 executions outside against
  1377/1037/350, and trace residuals ≤ 0.05 % of peak. Without ω×Jω it gives 0 outside.
- **Lag-compensated FF at the card plant**, by sensor configuration (reported, not asserted; owner decision 2):

  | Sensor configuration | Result |
  | --- | --- |
  | flight, as stage (e) flies: notches tracking the rotor-speed telemetry | PASS by 24.1 mrad/s (0 outside) |
  | stage (c) T4: truth gyro, notches bypassed | PASS by 3.2 mrad/s |
  | no chain, no latency | FAIL by 1.3 mrad/s |
  | T3 worst case: notches pinned at ω_th | FAIL by 58.1 mrad/s |

- PID alone and plain FF fail in every configuration.
- **Physical J corners (owner decisions 3 and 4):** no variant passes at any of the 12 corners. The worst is corner 11,
  J/J0 = (1.5, 1.5, 0.5): lag-compensated FF exceeds by 9.46 rad/s on pitch, and FF worsens some corners against PID.
  This is carried to the pre-L8 gate (measured J).
- **UNKNOWNs:** E is not modelled (E = 0, stricter); τ ±30 % is not swept at the corners. The acro test's
  `script_response` and oracle are PI-only and are extended in commit 3. tau_ref uses `rate.py` rule step 9 with
  rate_lead's τ_cl (lead decision).

**Frozen file changed: `tests/regression/quad/L06/CMakeLists.txt`.** One `add_subdirectory(d_term)` line is appended.

## Evidence

To be added as stage (c) is built.

## Approval

Owner decisions: Luis, 2026-10-01, as quoted above. The spec line for the combined D + FF noise budget: pending Luis.
The stage (c) build and close: pending.
