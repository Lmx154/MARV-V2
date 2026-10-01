# 0012: quad L6 stage (a), the sensor noise model: sources, the clock error, and the ICM-45686 citation

This record opens L6 stage (a) (quad spec §4 L6). It holds Luis's stage (a) decisions verbatim, the spec gap and the
spec fix he approved, and the stage's choices. The stage appends its build, its frozen edits and its evidence here, and
closes in this record.

## What changed

**Spec (both ACTIVE; Luis approved the text, 2026-10-01).**
- `docs/spec/00-core-contracts.md` §5, the ICM-45686 row. It now cites DS-000577 rev 1.0 §3.1–3.2, tables 1–2. Each
  offset gives both figures, marked UNVERIFIED: ±0.4 °/s (board) / ±0.3 °/s (component, 25 °C, §3.1), and ±20 mg
  (board) / ±10 mg (component, 25 °C, §3.2). The board figure comes from the earlier spec, and its cited document
  (DS-000489 rev 1.1) was not located. The ODR range is marked low-noise mode.
- `docs/spec/10-quad-flight-software.md` §5.5:
  - The 20-byte FIFO packet now cites DS-000577 rev 1.0 §6.1, with its layout.
  - The FIFO-size bullet (2 KB by default, 8 KB with APEX off) now cites DS-000577 rev 1.0 §6. It had no citation
    before.

**Sensor profile** (`sensors/profiles/marv_v2_board_default.yaml`, owner's second message below).
- **Sources.** Its eight ICM-45686 entries cited "DS-000489 rev 1.1, tables 1–2". The six that DS-000577 confirms now
  cite it: `gyro_fsr`, `gyro_noise_density` and `gyro_offset_tempco` cite §3.1 Table 1, `accel_fsr` and
  `accel_noise_density` cite §3.2 Table 2, and `odr` cites §3.1 Table 1, low-noise mode. Their values, status
  (UNVERIFIED) and σ (UNKNOWN) are unchanged.
- **The two offsets** fly the board figure as a labelled scenario value (`method: scenario`, `sigma: choice`,
  UNVERIFIED), with Luis's rationale in the note.
  - `gyro_zero_rate_offset`: 0.4 °/s. Its `source` says the board figure is from the earlier spec table and that its
    cited document (DS-000489 rev 1.1) was not located. A `conflict` entry (schema field, `tools/card/schema.py:280`)
    holds 0.3 °/s from DS-000577 §3.1 Table 1 (component, 25 °C), state open.
  - `accel_offset`: 20 mg, the same treatment, with the conflict entry holding 10 mg from §3.2 Table 2.
- **Quantisation.** `gyro_fsr` and `accel_fsr` carry the note that the quantisation step assumes the 20-byte
  high-resolution FIFO packet (DS-000577 §6.1: 131.1 LSB/(°/s), 16384 LSB/g, whatever full scale is set), and that the
  step changes if the L9 driver reads another format. The profile has no quantisation entry.

**Frozen file changed: one.** `tests/regression/quad/L01/tools/test_card_lint.py`,
`test_profile_plus_minus_figure_must_not_become_sigma_without_rule`. The test's example entry changes from
`gyro_zero_rate_offset` (σ set to 0.4) to `gyro_offset_tempco` (σ set to 0.005), in the mutation and in the expected
message.
- **Why.** The test checks that a datasheet ± figure cannot become a numeric σ without a `sigma_rule`. Its example
  entry, the gyro offset, is now a scenario entry, so the linter rejects the mutation by the scenario rule ("sigma: a
  scenario entry is a choice …") instead, and the test's needle no longer matches.
- **Not loosened.** `gyro_offset_tempco` is still a datasheet entry carrying a ± figure, so the same rule is checked
  with the same assertion. The offset with a numeric σ is still rejected, by the scenario rule; this was checked
  directly (lint exit 1, message above).

## Why

**Spec gap: the ICM-45686 citation.** Both ACTIVE specs cited "DS-000489 rev 1.1" for the ICM-45686: the core §5
table and the quad §5.5 FIFO packet line. The profile copied the citation from the spec table, in 8 entries. The
datasheet in Luis's possession reads "Document Number: DS-000577, Revision: 1.0, Rev. Date: 07/25/2024". Nobody has
read a DS-000489, and it is not the ICM-45686 datasheet's number. The figures were checked against DS-000577:
- **Match:** gyro FSR, gyro noise density, the gyro offset tempco, accel FSR, accel noise density, the ODR range
  (low-noise mode), and the 20-byte high-resolution FIFO packet.
- **Differ:**
  - Gyro zero-rate offset: ±0.4 °/s (board), from the spec, against ±0.3 °/s (component, 25 °C, DS-000577 §3.1, Table 1).
  - Accel offset: ±20 mg (board), from the spec, against ±10 mg (component, 25 °C, DS-000577 §3.2, Table 2).
- **Wrong section:** the FIFO packet line cited §5; the packet structure is §6.1.

Per core §2, the conflicting values are both kept and marked UNVERIFIED.

## Owner decisions (Luis, verbatim)

2026-10-01, first message:

> 1. **Datasheet.** Cite DS-000577 rev 1.0. Both ACTIVE specs still cite DS-000489, in the core §5 table and the quad
>    §5.5 FIFO packet line. Log this as a spec gap in the stage (a) record. Check core §5's ICM-45686 figures and the
>    20-byte FIFO packet against DS-000577, then send me the two-line spec fix to approve. If any figure differs, keep
>    both values and mark them UNVERIFIED.
> 2. **Crystal.** Use the worst-case linear sum, ±65 ppm: ±30 tolerance, ±30 stability and ±5 first-year aging, from
>    the ABM8-272-T3 spec. Don't use RSS. Datasheet limits are bounds, not σ, and no source says the terms are
>    independent. Record that the aging term covers the first year only, and that these are the Pico 2 test setup's
>    values until the MARV V2 BOM crystal is known.
> 3. **Clock model.** Yes, apply the error only against the plant's clock. With CLKIN, the IMU's ODR and hal_time_us
>    share one error e, so draw e once per run, never separately for each. Keep the ±1.25 % / ±1 % internal-oscillator
>    figures (DS-000577 §3.3.2) in the profile as the non-CLKIN case, but don't fly them. "The ODR follows CLKIN
>    exactly" stays INFERRED until a committed test or measurement shows it. Exercise e at the corners −65, 0 and +65
>    ppm, not as a sampled distribution.
> 4. **S2 and S5–S8.** That list is lost, so don't reconstruct it. Check stage (a)'s pass-bar lines against core §5's
>    profile fields, and send me only the questions stage (a) is actually blocked on, each one answerable in one word.

2026-10-01, second message:

> **Spec fixes: approved, with one change.**
>
> - **Quad §5.5:441:** approved as written. In the same edit, cite the next bullet (FIFO 2 KB by default, 8 KB with
>   APEX off) from DS-000577 as well. It has no citation today.
> - **Core §5:308:** approved, except don't cite DS-000489 as the source of the board offsets. Nobody has read that
>   document, and DS-000489 isn't the ICM-45686 datasheet's number. The profile copied that citation from the spec
>   table, so the profile doesn't confirm it either. Write the gyro offset as: "±0.4 °/s (board) / ±0.3 °/s
>   (component, 25 °C, DS-000577 §3.1), UNVERIFIED; the board figure comes from the earlier spec, and its cited
>   document (DS-000489 rev 1.1) was not located." Write the accel offset the same way: ±20 mg (board) / ±10 mg
>   (component, §3.2).
>
> **Sensor profile: approved.** Make the correction in the stage (a) change, under its record, and treat it the same
> way:
>
> - Cite DS-000577 rev 1.0 on the sourced entries.
> - Give each of the two offsets both figures, saying that the board figure's source was not located.
> - If the profile format can't hold two figures for one entry, put the flown value in `value` and the other in `note`.
> - Frozen L01 tests read the profile (test_card_lint.py, test_flatten_report.py, test_gen_sdf_plant_config.py). Run
>   them. Any frozen edit they need goes in the same record.
>
> **Which offset to fly:** the board figures (±0.4 °/s, ±20 mg), as labelled scenario values. The rationale: they are
> the larger of the two, and a soldered part sees mounting stress that the 25 °C component figure leaves out. My
> still-bench dataset replaces them later.
>
> **Quantisation and sample rate: agreed.** Record in the profile that the quantisation step assumes the 20-byte
> high-resolution FIFO packet (DS-000577 §6.1). If the L9 driver reads another format, the step changes.
>
> **Turn-on bias: corners.** The figures are limits, not σ, and no source gives a distribution, which is the same
> reasoning as the clock error. Each axis takes +bound or −bound, plus the nominal 0. Six axes give 64 corners, so run
> the full set at T3. At T4, fly the nominal plus the worst corner T3 finds for each scenario. Propose that count, with
> its cost, together with the stage (e) seed count. Stage (a) doesn't wait on this, because a constant bias doesn't
> change the Allan deviation.

2026-10-01, third message (on the architect's questions A–C and the fdlibm licence):

> P2, P3, P6 and P7 are accepted. The clean-up is right. Move the SHA-256 helper to sim/ before anything under
> tests/regression/quad/L06/ is frozen, and add the missing reason comments.
>
> **A: Yes, τ* = 1 s.** I read the rule as: bias instability = the white-noise Allan deviation at τ*. That gives
> 3.8 m°/s/√Hz → 13.68 °/h and 110 µg/√Hz → 110 µg. If that's right, write the rationale into the labelled scenario
> value:
> - At short times it is pessimistic, which is the conservative side for control.
> - It keeps the derived Allan record short.
> - My still-bench dataset replaces it.
>
> The accel figure is the ±32 g noise density, the largest of the three. Record which full-scale range the profile
> assumes. Noise density follows the configured range, not the high-resolution FIFO's fixed ±32 g scaling.
>
> **B: Yes, family-wise, but use Bonferroni, not Šidák.** Without a correction, a correct model fails at least one of
> the 12 checks about 11 % of the time (1 − 0.99¹²). Šidák is exact only for independent checks. The noise-density
> check and the bias-instability check on one axis come from the same record, so they are not independent.
> Bonferroni (0.01/12 = 8.33e-4 per check) holds under any dependence and differs from Šidák (8.37e-4) only in the
> third significant figure. The record length grows by rule. If it passes 60 s, that check runs nightly, as already
> agreed.
>
> **C: Yes, family-wise, but under its own register entry.** Use `stream_independence_confidence` = 0.99, claimed in
> L06/param_ids. Don't borrow `allan_check_confidence`: each register entry has one purpose, as with the class split
> in 0009. Record it in 0012.
>
> **Licence: accept fdlibm.** Sun's notice only asks that the notice be preserved, which is compatible with our BSD-3
> licence and with C-4. Keep the notice in each file that carries the code, and add a third-party notices entry. Keep
> that code out of fw/. Inverse-CDF normals still need a log, so if fdlibm is there to make log bit-exact, switching
> gains nothing. Record the C-4 reading in 0012.

Lead notes on the third message:
- **A, the rule as recorded.** B is the minimum of the Allan curve. The profile's B is N × 1 √Hz, numerically equal to
  N (3.8 m°/s = 13.68 °/h; 110 µg). Under the model's white-noise plus random-walk rule this puts τ* = (N/B)² at
  exactly 1 s. At τ* the two terms are equal, so B is √2 × the white-noise Allan deviation at τ* (N/√2 = 2.69 m°/s
  there), not equal to it. Luis's values and rationale stand unchanged.
- **The accel figure.** The profile assumes the ±32 g configured range for the accel noise density (110 µg/√Hz, DS-000577
  §3.2 Table 2). The density follows the configured range; the 20-bit FIFO scaling (±32 g) sets only the quantisation
  step.
- **The licence.** fdlibm covers only `sin`, `cos` and their helpers (`__sin`, `__cos`, `__rem_pio2*`). The musl `log`
  is Arm's, under MIT. Accepted as Luis ruled.
- **The C-4 reading.** Sun's fdlibm notice ("Permission to use, copy, modify, and distribute this software is freely
  granted, provided that this notice is preserved") is a permissive licence, compatible with the project's
  BSD-3-Clause and allowed under C-4. The notice stays in every vendored file that carries the code, a third-party
  notices entry lists it, and the code stays out of `fw/` (Luis, 2026-10-01).
- **Register.** New entry `stream_independence_confidence` = 0.99, claimed in `tests/regression/quad/L06/param_ids`.
  The Allan checks use Bonferroni over the 12 checks at `allan_check_confidence`.

2026-10-01, fourth message:

> P1 and the clean-up are accepted. Your Bonferroni change on the independence and moments tests is right, for the
> same reason as B. Record it in 0012. musl's libm files carry Sun's fdlibm notice as well as musl's MIT licence, so
> keep both in the vendored files and in THIRD_PARTY_NOTICES.md.
>
> **1. Moments test: its own entry, `normal_moments_confidence` = 0.99.** One register entry, one purpose. Claim it in
> L06/param_ids and record it in 0012.
>
> **2. D, latency: 1 sample, as a labelled scenario value, unless the datasheet lets you derive it.**
> - Check first: if DS-000577 gives the on-chip filter's order and bandwidth for the setting the profile uses, derive
>   the filter's group delay from them, as a derived value with its rule. Then only the FIFO and SPI transport is a
>   scenario value.
> - Otherwise, use 1 sample, with this rationale: 0 is the one value a real sensor chain cannot have, and 1 sample is
>   the smallest nonzero delay the tick model can represent. My bench measurement replaces it.
> - It is not a proven upper bound. If the derived filter delay comes out longer than 1 sample, the derived value wins.
> - Stages (b) and (c) must carry this latency in the design model's loop delay, so the D-term design and the stage (c)
>   regeneration of L4 and L5 include it.

Lead notes on the fourth message:
- **D, the check.** DS-000577 rev 1.0 does not give enough to derive the delay. For the UI LPF it gives only the cut-off
  choices: bypass, ODR/4 … ODR/128 (`GYRO_UI_LPFBW_SEL`, `ACCEL_UI_LPFBW_SEL`). It calls the AAF "an FIR filter with
  fixed coefficients", with no order, tap count or group delay. For the combined AAF and UI LPF bandwidth it refers to
  AN-000365 (ICM-456xx User Guide), which has not been read. So `latency_samples` = 1, a labelled scenario value with
  Luis's rationale. AN-000365 is the document that could let the filter delay be derived.
- **Moments entry.** `normal_moments_confidence` = 0.99 is in `design/budget.yaml` and claimed in
  `tests/regression/quad/L06/param_ids`. The moments test reads it in place of `allan_check_confidence`.
- **Licence.** Both notices stay in the vendored files and in `THIRD_PARTY_NOTICES.md`.

Standing from 2026-09-30 (handoff):
- **S3:** "labelled scenario values, each with a written rationale, flagged in every run report; replaced by my bench
  dataset later".
- **S4:** the IMU is clocked from the MCU via CLKIN on GPIO18 (EMB-2, single master clock).
- **S9:** "60 s is fine as the per-push threshold; record it as a labelled convenience value".

## Sources

- **TDK InvenSense ICM-45686 datasheet:** DS-000577 rev 1.0, 07/25/2024.
  - §3.1 Table 1 (gyro), §3.2 Table 2 (accel), §3.3.2 (internal clock ±1.25 % initial, ±1 % over temperature with the
    gyro active).
  - §4.14 (CLKIN 20–40 kHz).
  - §6, §6.1 (FIFO size, packet structure; the 20-bit high-resolution format is always scaled to ±4000 dps,
    131.1 LSB/dps, and ±32 g, 16384 LSB/g).
- **Raspberry Pi Pico 2 datasheet:** RP-008299-DS, release 5. It names the crystal, Abracon ABM8-272-T3.
- **Abracon ABM8-272-T3 specification:** Drawing #456603, revision IR, issued 2023-11-16.
  - 12.000 MHz; ±30 ppm tolerance at +25 °C.
  - ±30 ppm stability over −40…+85 °C.
  - ±5 ppm aging, first year, 25 ± 3 °C.
  - The ±65 ppm sum holds for the first year only. These are the Pico 2 test setup's values until the MARV V2 BOM
    crystal is known.
- The PDFs stay local (`datasheets/`, gitignored) and are cited by document number and revision.

## Stage (a) choices

- **The clock error e.** One draw per run, shared by the IMU ODR and `hal_time_us`, applied against the plant clock
  only. Flown at −65, 0 and +65 ppm (owner decisions 2 and 3).
- **Turn-on bias.** At the corners: ±0.4 °/s per gyro axis and ±20 mg per accel axis, plus the nominal 0. All 64 at T3,
  and at T4 the nominal plus each scenario's worst T3 corner (second message). The T4 count and its cost go to Luis with
  the stage (e) seed count.
- **Quantisation.** The 20-bit high-resolution FIFO step (DS-000577 §6.1). The sample rate is the tick, 6.4 kHz
  (`design/scenario_values.yaml`, `tick_period_num_us` / `tick_period_den`).
- **Not in DS-000577.** Bias instability, random walk, latency and group delay. These are labelled scenario values
  with a rationale (S3; 0009 owner decision 4).

## Stage (a) build

**Design.** Architect consult, 2026-10-01; the lead accepted it. The IMU model is an opt-in C entry in `marv_plant`, so
every existing plant path stays bit-identical. The per-sample model is:
- truth delayed by L samples;
- plus a bias random walk from the turn-on corner, K = (√6/2)·B²/N, with the Allan minimum B at τ* = (N/B)²;
- plus white noise, σ_d = N·√(f_s/2) (one-sided density, DS-000577 Table 1 note 5);
- quantised to the 20-bit FIFO step, saturated with a flag per axis.

The clock error e is applied once, to the simulator's true tick period, and never reaches `fw/`. The Allan check runs
on the model alone and is computed in the test from the profile, so no reference data is generated for it.

**P2, the noise source** (`sim/plant/src/noise/`, tests in `tests/regression/quad/L06/noise/`):
- **Generator.** Counter-mode SplitMix64: draw(seed, id, n) = mix64(mix64(seed) + (id·2^48 + n + 1)·γ), the mixer of
  `sim/null_plant/include/marv/null_plant/prng.hpp`. Stream ids are ABI, in `stream_ids.hpp`; id 0 is the primary IMU.
- **Normals.** Box–Muller, cosine branch, 2 draws per normal, with u1 in (0, 1] and u2 in [0, 1). A sample takes 12
  normals at counters 12s + j.
- **Vendored maths.** `log`, `sin` and `cos` come from musl 1.2.5, vendored unmodified in
  `sim/plant/third_party/musl/` with its `COPYRIGHT` and `PROVENANCE.txt` (tarball and per-file SHA-256). They are
  renamed `marv_musl_*` and built with `-ffp-contract=off -U__FP_FAST_FMA -fno-fast-math`. No platform libm call is
  reachable from the stream path.
- **Pin.** SHA-256 `3abc7725a20377a80f385013cab5662c86f5bbcdfe3d96a8cb44b0154ed80852`, over the first 65536 normals
  (seed 1, id 0, 8 little-endian bytes each), equal in host-debug and host-release. Changing it needs a decision record.

**P3, the IMU model** (`sim/plant/include/marv_plant.h`, `sim/plant/src/imu_model.hpp`, `marv_plant.cpp`; tests in
`tests/regression/quad/L06/imu/`):

API, added after the existing declarations:
- `marv_plant_imu_attach(plant, const marv_plant_imu_config*)`;
- `marv_plant_imu_sample(plant, const marv_plant_body*, dt_s, marv_plant_imu_out*)`;
- a new status, `MARV_PLANT_E_STATE`, appended after `E_ALLOC`.

`marv_plant_config`, `marv_plant_out` and `marv_plant_step` are unchanged. The seed is `rng_seed` with stream id 0.
`marv_plant_imu_out` has the layout of `marv_imu_meas`, checked by a `static_assert` in a test.

The behaviour:
- **Specific force.** f = Σ k·ω²/m along −z_FRD, from the rotor state after the last step (before any step, the
  initial rotor speed). It reads −g at hover, and exactly 0 at zero thrust.
- **Quantiser.** Round half away from zero. The count is clamped to the tighter of ±⌊FSR/LSB⌋ and the 20-bit word
  [−2^19, 2^19 − 1] (DS-000577 §6.1). The saturation flag is set iff the count was clamped.
- **Labelled bounds.** `MARV_PLANT_IMU_MAX_LATENCY_SAMPLES` = 64 is a labelled capacity bound, not a sensor figure;
  above it, attach refuses with `E_CONFIG`. Attach also refuses `full_scale` above `FLT_MAX`/2 and a non-finite K.
- **Known limit.** A finite but non-physical dt (about 1e300) can drive the bias state to inf or NaN. The output stays
  finite and flagged. It is unreachable at physical sample rates, and not guarded.

Reviewer: PASS, at 502/502 in host-debug and host-release.

**P6, the adapter IMU path and the clock map** (`sim/gz/adapter/`, tests in `tests/regression/quad/L06/adapter/`):
- **Clock map.** `clock_error(corner, odr_error)` = corner × odr_error, with corner ∈ {−1, 0, +1}.
  `true_tick_period_s(t_nom, e)` = t_nom/(1 + e), which is t_nom bitwise at e = 0. That one value is the dt of both
  `marv_plant_step` and `marv_plant_imu_sample`. Nothing in `fw/` changed, and e reaches no SIL call.
- **Adapter config.** `AdapterConfig` holds the nominal tick, the clock corner, `odr_error` and an optional
  `marv_plant_imu_config`. The old constructor delegates to it with corner 0 and no IMU, which is bit-identical to
  before.
- **Firmware input.** `CommandSource` gains a non-pure 3-argument `dshot`. `ImuSilCommandSource` passes the plant's IMU
  bytes verbatim.
- **Order per tick.** IMU sample, then the source, then the plant step. The body has the same freshness as the truth
  gyro (`truth_gyro.hpp:7-8`). Sample j sees the rotor state after step j − 1.
- **Clock test.** At −65, 0 and +65 ppm the firmware stamps equal ⌊n·625/4⌋ µs, and the control (e in the SIL period)
  changes them.

Reviewer: PASS.

**P7, SIM-2 with noise** (`sim/l6_noise_run/`, tests in `tests/regression/quad/L06/sim2/`):
- **The driver.** It closes the adapter, `marv_plant`, the IMU and `marv_sil_l4_rate_scripted` (not a truth build), with
  a minimal rigid-body stand-in for Gazebo, for 4096 ticks. It logs per tick the tick, the firmware stamp, the IMU
  bytes, the DShot and the body state bits, and prints the log's SHA-256.
- **The tests.** Two separate processes with the same seed write byte-identical logs at corners −1, 0 and +1. Seed + 1
  first differs in tick 0's IMU bytes. Corner −1 against +1 gives identical firmware stamps on every tick and a
  different plant state.
- **Placeholders.** The IMU figures sit in one struct, which P4 replaces with the generated profile config. B is at
  τ* = 1 s, pending owner question A.

Reviewer: PASS (P7).

**P1, the profile entries for the model** (`sensors/profiles/marv_v2_board_default.yaml`, IMU class):
- **`gyro_bias_instability`** 3.8e-3 °/s (13.68 °/h) and **`accel_bias_instability`** 110 µg. Labelled scenario values
  (sigma choice, UNVERIFIED), by the rule B = N × 1 √Hz (owner decision A), with Luis's rationale. The accel entry
  records the ±32 g configured range.
- **`odr_error`** 65 ppm, derived as the worst-case linear sum, UNVERIFIED. The note holds the internal-oscillator figures
  (12500 / 10000 / 30000 ppm, DS-000577 §3.3.2) as the non-CLKIN case, not flown. They are in the note, not in
  `conflict`: they describe a different clock source, not a rival value.
- **`latency_samples`** 1, a labelled scenario value (owner decision D, fourth message; not derivable from DS-000577).
- **`gyro_fifo_sensitivity`** 131.1 LSB/(°/s), **`accel_fifo_sensitivity`** 16384 LSB/g and **`fifo_word_bits`** 20, from
  DS-000577 §6.1, UNVERIFIED. "g" read as standard gravity is INFERRED.

**Clean-up and register** (owner, third message):
- **SHA-256 helper.** Moved to `sim/common/include/marv/sim/sha256.hpp` (`marv::sim::Sha256`, INTERFACE library
  `marv_sim_sha256`), so `sim/` no longer includes from `tests/`.
- **Reason comments.** Added to the unlabelled test numbers in `L06/noise` and `L06/adapter`. `kOdrError` cites this
  record.
- **Third-party notices.** `THIRD_PARTY_NOTICES.md` lists musl 1.2.5 with its MIT, Arm MIT and Sun fdlibm notices.
- **Register entries in `design/budget.yaml`**, both `method: design-budget`, `sigma: choice`:
  - `stream_independence_confidence` = 0.99;
  - `per_push_check_time_max` = 60 s, the S9 labelled convenience value. The schema has no convenience kind, and its
    rationale says so.
- **Family-wise corrections.** The independence test reads `stream_independence_confidence`. Both family-wise noise
  tests now use Bonferroni, α = (1 − c)/m, in place of Šidák. This is the lead's reading of owner decision B, whose
  reason applies to them too: lags of one pair and moments of one sample are not independent statistics.

**P4, the profile → IMU config generator** (`tools/card/gen_imu_config.py`, `cmake/marv_card_gen.cmake`, tests in
`tests/regression/quad/L06/tools/`):
- **Output.** It generates `marv/sim/imu_profile_config.hpp` into the build tree. The header gives
  `imu_profile_config()` (every field SI, nominal turn-on bias 0), `imu_corner_config(s)` for s ∈ {−1, 0, +1}^6, the
  turn-on bounds, and `kImuOdrError` = 65e-6. Fields are hex floats, each with a provenance comment, plus the profile's
  SHA-256.
- **Refusals.** It refuses, naming the entry, any flown field that is UNKNOWN, missing or in the wrong unit. It also
  refuses an accel range outside (8, 16, 32) g, which it cites to DS-000577 §3.2 Table 2.
- **Report.** gyro N 6.632e-5 rad/s/√Hz, σ_d 3.752e-3 rad/s, K 8.123e-5; accel N 1.0787e-3 m/s²/√Hz, σ_d 6.102e-2
  m/s², K 1.321e-3; τ* = 1 s for both. `sim/l6_noise_run` uses the generated config.
- Reviewer: PASS.

**P5, the Allan check** (`tests/regression/quad/L06/allan/`):
- **Method.** The model alone through the C ABI, at rest, with the profile config and seed 1 (fixed before the first
  run). It uses the overlapping Allan deviation on integer counts, at m = 1 (noise density) and m* = (N/B)²/τ0 (bias
  instability).
- **Expected value.** σ²(m) = (σ_d² + LSB²/12)/m + K²τ0(2m² + 1)/(6m). The quantisation term is INFERRED (uniform
  rounding noise), and below 1.1e-4 of σ_d².
- **Bound.** Eq. 45 with ν the smaller of the Table 5 white-FM and random-walk-FM edf. The χ² quantiles are computed in
  the test and checked against NIST/SEMATECH table values.
- **Family and record.** Bonferroni over 12 checks at `allan_check_confidence`. The record comes from the strict rule:
  the ×1.1 model's interval wholly outside the nominal one.
- Reviewer: PASS (Table 5 transcription, eq. 45 direction, the closed form and the estimator checked).

**Frozen file changed: `tests/regression/quad/L06/param_ids`** (committed by 0009). It gains `stream_independence_confidence`,
`per_push_check_time_max` and `normal_moments_confidence`, because flatten.py emits every numeric register entry and decision 0004 requires a
manifest to claim each id. Nothing is removed.

## Evidence

- **The profile and the frozen edit**, 2026-10-01, on the working tree:
  - `uv run python tools/card/lint.py --profile sensors/profiles/marv_v2_board_default.yaml`: exit 0.
  - The L1 card lint (`--card vehicles/uzh_neurobem_5in.yaml --budget design/budget.yaml`): exit 0.
  - `uv run pytest tests/regression/quad/L01 -q`: 310 passed. Before the frozen edit: 1 failed, 309 passed.
  - L00, L02, L03, L04 and L05 tools suites: 823 passed. `run_scenario.py` hashes the profile into reports, and no
    committed golden holds the old profile hash.

- **The pass bar (a), T1 Allan check** (P5, `tests/regression/quad/L06/allan/`):
  - Nominal at seed 1: 12 of 12 inside. The thinnest margin is gyro_z bias instability, σ̂/σ 0.9659, margin 1.0187.
  - Noise density × 1.1: all 6 noise-density checks fail (margin about 0.91).
  - Bias instability × 1.1: all 6 bias-instability checks fail (thinnest gyro_z, margin 0.9814).
  - Record: 13,159,601 samples (2056.19 s), derived in the test. It is asserted shortest (it separates at n, not at
    n − 1).
  - ν: 8.77307e6 at m = 1 and 2053.19 at m* = 6400.
  - Run time, read from the core job of the `ci/local_ci.sh` run below (marv-ci image, 4 CPUs):
    - host-debug: nominal 10.00 s, N × 1.1 control 10.03 s, B × 1.1 control 10.04 s;
    - host-release: 4.27, 4.23 and 4.23 s;
    - the frozen-suite run: 4.27, 4.24 and 4.25 s.

    Every figure is under `per_push_check_time_max` (60 s) in both builds, so the check runs per push and the nightly
    rule does not apply.
- **The 64-corner builder** (`tests/regression/quad/L06/imu/corner_test.cpp`, added at Luis's stage (a) approval):
  - `imu_corner_config` gives 2^6 = 64 distinct corners, each axis exactly at +bound or −bound (never 0), with every
    other field equal to the profile config.
  - The nominal (all axes 0) is separate from them.
  - A sign of ±2 on any axis is refused.
  - Control: changing the generated builder so that the accel axes take the gyro signs fails the 64-corner test. The
    header was regenerated afterwards, byte-identical.
- **The pass bar (a), T1 SIM-2 with noise.**
  - Two separate processes give byte-identical logs at corners −1, 0 and +1 (P7). Seed + 1 first differs in tick 0's
    IMU bytes.
  - Distinct streams are independent: Bonferroni over 3332 tests at `stream_independence_confidence`, worst |ρ| 0.0156
    against a bound of 0.0182. The 12-draw-shift and duplicated-key controls fail (P2).
  - The adapter's sensor bytes equal a direct `marv_plant` call over states × seeds × m ∈ {1, 3, 4}. The index-shift,
    other-seed and one-ulp controls fail (P6).
- **`ci/local_ci.sh`**, all four jobs, `MARV_CI_BASE_REF=master`, on `8370c61`. That is a temporary commit of this
  change on top of `3cf3e98`; the record's own text was finished after the run. Fresh clone, as root,
  `--cpus 4 --memory 15740260352`. LOCAL CI PASSED:
  - **core:** ALL STEPS PASSED in 597 s.
    - ctest 532/532 (debug), 532/532 (release), frozen 531/531.
    - Tools tests: 921 passed, 1 skipped. The skip is pre-existing: the same step on `3cf3e98` gives 865 passed, 1
      skipped.
    - Regression-change check: 2 frozen files changed, 1 decision record.
  - **gz-l2:** passed in 133 s.
  - **gz-l4:** 40 passed, 1 xfailed, in 179 s.
  - **gz-l5:** 63 passed, 1 xfailed, in 565 s.
- **Reviews:** each packet was reviewed independently against its packet: P2, P3, P4, P5, P6 and P7 PASS. P1 and the
  clean-up were checked by the lead (lint, L01 310 passed).
- **Known gaps:**
  - The 60 s per-push rule is applied by measurement, not enforced by a test.
  - The Gazebo-side items below remain.
  - Gazebo plugin wiring of the IMU path and the clock corners is stage (e), along with owner question E.
  - The rigid body in `sim/l6_noise_run` restates the card's inertia as a labelled value, because the card header
    carries none.

## Approval

Spec text and owner decisions: Luis, 2026-10-01, as quoted above.

Stage (a) build and close: Luis, 2026-10-01:

> **Stage (a): approved.** Record my approval in 0012, then commit and push to master. Don't tag it. 0011's push and
> last night's run were both green on Actions, so this lands on a green base.
>
> Before you commit:
>
> 1. **The 64-corner bias builder.** Don't commit it untested. Either add a small T1 test now, or take the builder
>    out of this commit and add it with T3, its first user. The test checks that there are 64 distinct corners, each
>    axis at +bound or −bound, with the nominal kept separate.
> 2. **Allan check time.** The 4.3 s is a release figure, but CI runs ctest in debug too. Read the check's debug and
>    release times from the core job of your local_ci.sh run and record both in 0012. If debug is over 60 s, say so,
>    and the nightly rule applies.
>
> After the push, as the first item of stage (b): close the gap "nothing enforces the 60 s limit". Set CTest's TIMEOUT
> on the per-push checks from `per_push_check_time_max`, so an overrun fails CI instead of relying on someone
> measuring it.

Both pre-commit items are done (Evidence above). The 60 s enforcement is the first item of stage (b).
