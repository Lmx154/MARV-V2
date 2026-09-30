# 0006: quad L5 attitude-loop choices

**Revised 2026-09-30** after owner decisions 13–25 and the L5 build. Not yet approved by Luis. Items marked PENDING
are still to come.

## What changed

**Frozen files changed.** One. Files L5 adds under `tests/regression/quad/L05/` need no record (core §7.3).
- **`tests/regression/quad/L01/tools/test_gen_sdf_plant_config.py`.** The `SCENARIO_FIELDS` line classifies the new
  plant field `initial_omega_rad_s`, and a new negative control makes an unclassified planted field fail.
- **Record.** 0007 (`docs/decisions/0007-l2-initial-rotor-speed.md`) records the change, with Luis's approval (owner
  decision 25).
- **The test stays a pinned set, deliberately.** It forces a vehicle-versus-scenario classification of every new plant
  field. It is not one of the snapshot tests to generalise.
- **Nothing else.** No other file under `tests/regression/quad/L00`–`L04` changes. The regression-change check in full
  CI confirms this (pending).

**Interfaces changed (all additive).**
- L4 opening: `RateLoop::execute_bypass` (owner decision 8). There is no other L4 change (owner decision 14: no new L4
  accessor).
- SIL: a test-only truth-state entry, `marv_truth_state_set`, exported only by SIL libraries built with
  `TRUTH_STATE` (owner decision 11, which overrides decision 4). Product SIL libraries are unchanged.
- G3: the exports check distinguishes product and truth-state SIL libraries, with negative controls.
- Gazebo plugin: an optional `<attitude_source>truth</attitude_source>` element and a log record type 5 (TRUTH).
- A new firmware type, `AttitudeState<T>`, which is the L5 input now and L7's opening later.
- L2: `marv_plant_config` and the scenario gain an initial rotor speed. It defaults to 0, so every existing scenario
  and frozen golden stays bit-identical (owner decision 23; record 0007).

**Specs changed (owner decision 21).** Commit `b194ea3`:
- core §7.5 gains a "Flight rate groups" paragraph;
- quad §3.3 SIM-7, §4 L5 Builds and §5.1 (the attitude and estimator rows) follow it.

The lead scoped the core rule (H).

**Superseded owner decisions.** Decisions 3 and 10, and the SIM-7 half of decision 20, are superseded by decision 21.
Their text is kept below.

**Known failing items.** Both are strict xfails on "`tests/regression/quad/L06/` does not exist". Each blocks L6 and
must pass before L8.
- **L4 acro recovery** (0005 owner decision 12). Unchanged.
- **L5 recovery R2** (new; owner decision 15). The cause is of the same class, gyroscopic coupling that the per-axis
  design cannot reject, with no L3 saturation. The gating is the same. The measurements are in F.

This record holds the owner's choices for quad L5 (quad spec §4 L5) where the spec left a choice, and the lead's
choices that follow from them.

## Owner decisions (Luis, 2026-09-30, verbatim)

1. Band box: rate gains fixed at nominal; at each τ×J corner, close the rate loop on that corner's plant, then check
   attitude-loop margin around it. Worst corner ≥ PM_min.
2. Design model: exact sampled-data closed rate loop (tools/card/rate.py).
3. **[Superseded by decision 21.]** Loop rate: SIM-7.
4. **[Overridden by decision 11.]** Truth attitude: same pattern as truth gyro, covered by G3, no SIL ABI change.
5. Large-angle recovery: shortest-rotation quaternion error. If the coupling limit appears, report it and extend the
   open safety item; don't tune.
6. Angle-mode tilt limit: scenario value cited from Betaflight at a pinned commit.
7. L6 will change the rate loop (D term), so L5 must regenerate cleanly: gains by rule only, and frozen tests check
   rule properties (margins, envelopes), not specific gain values or traces. Goldens only where the test owns fixed
   inputs and the gains are part of what's being checked.
8. "Prefilter: Option 1 (bypass input for the attitude loop; acro keeps the prefilter). Record the additive L4
   interface change in the L5 decision record."
9. "Yaw: Keep the full-quaternion setpoint and heading from the yaw stick, but use tilt-prioritized error: correct the
   reduced attitude (tilt) at full authority, and handle the yaw error separately at lower weight. References: PX4
   AttitudeControl (reduced attitude + yaw weight, BSD-3); Brescianini & D'Andrea, "Tilt-prioritized quadrocopter
   attitude control", IEEE TCST 2020. The yaw weight needs a rule or a register entry, not a literal. Large-angle
   recovery uses the same law."
10. **[Superseded by decision 21.]** SIM-7 quantity and uncertainty (the option Luis selected, "PM loss vs meas.
    U"): quantity = the worst-corner attitude PM of the gains designed at the rate-loop rate (3.2 kHz), evaluated at
    each lower candidate rate 3.2 kHz/2^k; uncertainty = the chirp margin-measurement uncertainty E_H + U_A + U_d,
    derived as at L4 (0.056° there, 0005 lines 253-268). No new number.
11. "Truth attitude: Option 1. Test-only, optional truth-state entry, exported only by test SIL libraries; G3 extended
    so product SIL libraries cannot export it, with a negative control. Record in 0006 as overriding my decision 4."
12. "Yaw weight: Option 1 (w = α_max,yaw / min(α_max,roll, α_max,pitch)), clamped to at most 1. Report the resulting
    value in 0006." (α_max as in 0005 QF-2: τ_max,a / (J_a·(1 + inertia_robustness_band)).)
13. "Yaw gain: Option 2 (PX4 compensation). w shapes large combined errors only; linear yaw response unchanged."
14. "Heading hold: neither. Use heading lock on stick release: while the yaw stick is outside the deadband, yaw is a
    rate command and the heading setpoint tracks the current heading; when centred, lock the heading there. No
    runaway, no post-release catch-up yaw, no new L4 accessor. Deadband = scenario value cited from Betaflight at a
    pinned commit. Add a T4 test: after a sustained yaw input is released, yaw rate returns to zero without
    overshooting the heading at release by more than the band envelope."
15. "Recovery: Option 1 (R1 must pass; R2 strict xfail with cause file if it hits the coupling limit, same gating as
    the L4 acro item: blocks L6, must pass before L8)."
16. T4 attitude chirp: "Add T4 chirp (Rec.)"
17. "Option 1 (lock at first world-down yaw-rate zero crossing after release). Two guards: (a) one lock per release;
    no re-lock until the stick leaves the deadband again, so gyro noise at L6+ can't chatter it; (b) fallback lock at
    the derived maximum stopping time ω_release / α_max,yaw (worst band corner) if no zero crossing occurs. T1 tests
    for both guards; T4 as you described."
18. Yaw-lock fallback (the option Luis chose: fallback = max(t_cross, ω_r/α_min), with t_cross the latest first
    zero-crossing time over the band box of the design-model yaw rate after release; in the linear regime it does not
    depend on ω_r, and the generator emits it as a derived parameter). "Option 1. You're right: my ω_r/α_min term was
    the fastest stop, not the maximum. t_cross is a derived, generated parameter, so it regenerates when L6 changes the
    rate loop. Add a T4 case with a constant yaw disturbance (no zero crossing) to exercise the fallback directly."
19. DShot quantisation in the T4 step tolerance.
    - **The option Luis chose.** Q = the maximum over the box (corners plus the grid check) of |quantised −
      unquantised| design-model trajectory for the same script, quantised by the firmware's `thrust_to_dshot` and the
      card's ESC map. The T3 envelope is unchanged; T4 checks within envelope ± (E + F + Q).
    - **His words.** "Option 1. Before recording it, confirm the fine negative controls (gains × 1.1, one tick of added
      delay) still fail the widened T4 predicate. If they don't, record that T4 angle steps now catch gross failures
      only and the fine metric is enforced at T3.

      Log for L6: evaluate DShot error diffusion (carry the rounding residual to the next tick) in the actuator-chain
      work. It would remove the hover limit cycle without relying on noise dither. It changes L3 output, so it needs
      its own decision record there."
20. **[The SIM-7 half is superseded by decision 21.]** "Chirp amplitude: not the tilt-limit rule (60° is a pilot
    envelope, not a linearity bound). Use the sweep you already ran: measured roll/pitch margin = min over the
    amplitudes A_env/2^k that produce a crossover (k = 1..5), and U_A = spread of that plateau. Pass: min − U_A ≥
    PM_min. Include a gains × 2 negative control on roll and pitch, as for yaw.

    SIM-7: apply the recorded rule (re-measure at 800 Hz; if it flips back, take the higher rate and report it).

    Separately, draft a spec amendment for my review (don't apply it yet): SIM-7 minimisation applies only where the
    rate choice has a material cost against the EMB-3 CPU budget; otherwise a loop runs at its parent loop's rate.
    Include what it would change for L5."
21. "Adopt the SIM-7 amendment, with this definition: a loop runs at its parent loop's rate unless that fails EMB-3
    schedulability; only then does SIM-7 minimisation choose a lower rate. Use the §5.4 CPU estimates until L9
    measures WCET, and re-check at L9. Record in 0006 and apply the text to the spec.

    Attitude loop = 3.2 kHz. Stop the 800 Hz re-measure; it's moot. Regenerate the T3 fixture and envelopes at
    3.2 kHz, then start yaw release, fallback and recovery R1/R2."
22. "R1: Option 1. δ = labelled scenario test value. Keep the exact-180° α-only side check." (R1 starts at 180° − δ
    about roll, with the full envelope predicate, plus an α-only side check at exactly 180°.)
23. "Spin-up: Option 2. Add an initial rotor speed to marv_plant_config and the scenario, defaulting to 0 so all
    existing scenarios and frozen goldens stay bit-identical (prove it: full L00–L04 suites green, goldens unchanged).
    Recovery scenarios start at the card's hover rotor speed. Decision record for the additive L2 interface change."
    (The record is `docs/decisions/0007-l2-initial-rotor-speed.md`.)
24. "General: the T4 tolerance is now E + F + Q. Prefer fixing harness/setup limitations over adding derived terms,
    and after any new term re-confirm the fine negative controls still fail."
25. The frozen L01 edit, approved and recorded in 0007: "Option 1, approved, recorded in 0007. Add the negative
    control (an unclassified planted field fails). Note in 0007 that this test is kept as a pinned set deliberately:
    it forces a vehicle-vs-scenario classification for every new plant field, and plant-config changes already need a
    decision record. It is not one of the snapshot tests to generalise."

## Lead decisions

### A. Rate-loop bypass (owner decision 8; additive L4 change)

- **API.** `RateOutput<T> RateLoop<T>::execute_bypass(const ImuSample& imu, const prim::Vec3<T>& reference) noexcept`.
  - `execute` and `execute_bypass` both call one private `step<bool kPrefilter>`.
  - `execute` is `step<true>`, whose body is today's `execute` body (`rate_loop.hpp:154-209`) unchanged.
  - `step<false>` differs only in the prefilter line (`rate_loop.hpp:190-191`): r_n = reference, and no α is computed.
- **Prefilter state.**
  - At a seed execution both entry points do the same: r := y, e := 0, I := 0, zero torque (`rate_loop.hpp:182-186`).
  - **Which seeds are period-checked.** The spacing check depends only on `have_t_`. `execute` sets it before any
    validity check (`rate_loop.hpp:168-169`), and `fault()` does not clear it (`:251-258`). So the seed execution after
    a fault is still period-checked. Only `init()` (`:139`) and `reset()` (`:146-149`) clear it, so only the first
    execution after them is unchecked. This record describes the code as it is and proposes no change.
  - After a non-seed bypassed execution, r_ = reference. A later `execute` therefore continues its prefilter from the
    last applied reference, so the switch from angle to acro is bumpless.
  - The switch from acro to angle needs nothing: the bypass does not read r_.
- **Unchanged.** These are identical for both entry points:
  - the finiteness checks, with the reference in place of the setpoint;
  - GyroValid and the output finiteness check;
  - the fault reset, latch and counter;
  - the period panic (`rate_loop.hpp:158-167`);
  - `record_allocation` and the anti-windup predicate.
- **Invariants and their T1 tests** (`tests/regression/quad/L05/unit/rate_bypass/`).
  - **I-A1, acro is bit-identical.** `execute` must reproduce, bit for bit, a golden of torque bit patterns.
    - The golden is generated against `fw/rate` at tag `quad-L4-pass`, in `marv-ci`, with the command recorded.
    - Its fixture is the L04 T3 inputs plus a scripted allocation sequence that sets flags and freezes, with kd = 0.
    - Control: the fixture with kp one ulp up must not reproduce the golden.
    - This fits owner decision 7: the input is fixed and owned by the test, and the gains are part of what is checked.
      It survives L6 unless L6 changes the prefilter or PI arithmetic, and that change then needs its own record.
  - **I-A2, the bypass skips only the prefilter.** Two loops share one config and one input stream: random inputs,
    saturating allocations and nonzero kd.
    - Loop A runs `execute(imu, sp)`.
    - Loop B runs `execute_bypass(imu, r)`, with r from the header's prefilter formula in the same float operations
      and seeded the same way.
    - Torque, fault flags, counter and integrator are bit-equal at every execution.
    - Control: r is perturbed by one ulp at one execution. The control scans every execution and requires at least one
      break. The change can first appear one execution later, through the deferred integrator (e_ feeds the next
      increment).
  - **I-A3, the switch back is bumpless.** After n bypassed executions ending at reference r_last, the first
    `execute` uses r = r_last + α(sp − r_last). Control: a reset in between (r seeded from y) differs.
  - **I-A4, faults and the period check behave the same.** Each case runs through `execute_bypass`:
    - a NaN reference, a cleared GyroValid or a non-finite output gives zero torque, fault_active, the latch, the
      count and a seed at the next execution. That seed is still period-checked, because `have_t_` stays set;
    - after `reset()` the next execution is a seed with no period check;
    - |dt − T| > 1 µs panics (a death test).
  - The L04 frozen suites run unchanged.
- **Rejected.**
  - A config-time bypass: switching between angle and acro is a per-call choice from L8.
  - A default argument on `execute`: it hides the intent at call sites.
  - τ_ref → 0: α = 1 does not give r = sp exactly in float, and `validate` requires τ_ref > 0 (`rate_loop.hpp:79`).
  - The prefilter inside the attitude path: the attitude plant would become T·F, not the T = L/(1+L) the spec names.

### B. Truth-attitude channel (owner decision 11, overriding 4)

- **Entry.** A new pure-C header, `fw/sil/include/marv_truth.h`:
  ```c
  enum { MARV_TRUTH_ATTITUDE_VALID = 0, MARV_TRUTH_FLAG_COUNT };
  typedef struct marv_truth_state {
    uint32_t struct_size;        /* sizeof(marv_truth_state): the version check */
    uint32_t flags;              /* bit MARV_TRUTH_ATTITUDE_VALID only */
    uint64_t tick;               /* the tick of the next marv_sil_tick */
    float q_wxyz[4];             /* body FRD -> NED, Hamilton, [w x y z] */
    marv_vec3f omega_frd_rad_s;  /* body rates, FRD */
  } marv_truth_state;
  marv_sil_status marv_truth_state_set(const marv_truth_state* s);  /* READY; TRUTH_STATE libraries only */
  ```
- **Status codes.**
  - E_STATE unless the library is READY; E_NULL for a null pointer.
  - E_ABI when `struct_size` ≠ sizeof.
  - E_TICK unless `tick` equals the ticks already run, i.e. the next `marv_sil_tick`'s first_tick.
  - E_INPUT in three cases:
    - a reserved flag bit is set;
    - the valid bit is clear and any field is nonzero (the IMU rule, `marv_sil.cpp:81-99`);
    - the valid bit is set and a field is non-finite or q has zero norm.
  - A second set for the same tick replaces the first.
- **Call order.** For each tick j: `marv_truth_state_set(tick = j)`, then `marv_sil_tick(j, 1, …)` (the loop of 0003
  item 8).
- **Firmware side.**
  - **Hand-over.** On OK, `fw/sil/src/truth_state.cpp` builds `AttitudeState<float>{t_us = the SIL stamp of tick j, q,
    omega, valid}`. It calls `marv::composition::attitude_input(const AttitudeState<float>&) noexcept`.
  - **Where the hook is declared.** In the L5 test composition's own `include/marv/composition.hpp`, not in the HAL
    contract (`fw/hal/include/marv/hal/tick.hpp:5-10`). A composition without it cannot be built into a TRUTH_STATE
    library: the build fails loudly.
  - **Freshness.** The composition latches the state. At each attitude execution it panics unless the latched stamp
    equals the sample stamp. This is a harness-contract panic, like `l4_rate_scripted.cpp:142-144`.
  - **G3.** The call goes from the SIL to the composition. The composition, a flight target under `fw/`
    (`cmake/flight_targets.cmake:14-21`), therefore references no `marv::truth`, `marv_truth_` or `marv_plant_`
    symbol. G3's symbol check proves this on every host and m33 build.
  - **m33.** The composition builds on m33 (`attitude_input` is plain code); `fw/sil` is host-only
    (`fw/sil/CMakeLists.txt:18-20`).
- **The opening type for L7.**
  - `fw/types/include/marv/types/attitude_state.hpp`:
    `template <class T> struct AttitudeState { TimeUs t_us; prim::Quat<T> q; prim::Vec3<T> omega_frd; bool valid; };`
  - This is quad §4 L7's opening: a quaternion, body rates and a validity flag. The attitude module takes it by
    const reference.
  - At L7 the estimator fills it and the attitude module is untouched. The attitude law does not read `omega_frd`;
    angle mode reads it for the heading lock (D).
- **Build.** `marv_add_sil_library(<name> … TRUTH_STATE)`:
  - adds `truth_state.cpp`;
  - links with `marv_sil_truth.map`, which exports `marv_sil_*` and exactly `marv_truth_state_set`;
  - sets the target property `MARV_SIL_TRUTH_STATE`.
  Without the option nothing changes.
- **Plugin.**
  - **Element.** An optional `<attitude_source>truth</attitude_source>`. The plugin refuses it in two cases:
    - without `<gyro_source>truth</gyro_source>`;
    - when the linked SIL library lacks `MARV_SIL_TRUTH_STATE`. The plugin's CMake reads the property. The sim-side
      `marv::truth::TruthAttitude` is a separate static target under `sim/gz/adapter`, linked only in that case.
  - **Per tick.** q and omega are the float casts of the host step's `marv_plant_body`, held over the step's m ticks
    (the truth-gyro freshness rule, `truth_gyro.hpp:7-8`), with the valid bit set. `marv_truth_state_set` is called,
    then `marv_sil_tick`. A non-OK status is refused.
  - **Runner.** m must divide the attitude divisor, so that every attitude sample is fresh.
- **Log.**
  - Record type 5, TRUTH: u64 tick, then the bytes of the `marv_truth_state` passed. It follows that tick's TICK
    record and is written only when the element is present.
  - Without the element the log is byte-identical to L4. The version stays 1. `lockstep_log.hpp` and
    `tools/sim/lockstep_log.py` learn type 5.
- **G3 exports.**
  - **Manifest.** Each `sil_libraries` entry gains `"truth_state"`.
  - **`check_g3.py`.** Two rules:
    - absent or false: today's rule. `non_sil_exports` is unchanged, so the frozen L00 tests
      (`test_check_g3.py:82-89`) still hold;
    - true: the exports must be a subset of `marv_sil_*` ∪ {`marv_truth_state_set`} and must include
      `marv_truth_state_set`. A flag without the export is a G3-ERROR.
  - **Product library.** A product SIL library is one built without TRUTH_STATE, which is the default.
  - **Control exclusion.** It is generalised from `L00/controls` (`cmake/flight_targets.cmake:56`) to every
    `tests/regression/<product>/Lnn/controls`.
  - **Lead (Q6, accepted).** The TRUTH_STATE flag goes in now. The guard that only test compositions may carry it is
    deferred to L8, where the first flight composition appears (carried forward).
- **Negative controls.** They live in `tests/regression/quad/L05/controls`, each with its own manifest, and run as CI
  steps like `g3_export_control`:
  - an unflagged shared library that exports `marv_truth_state_set` must give G3-EXPORT;
  - a flagged library that also exports `marv_truth_planted` must give G3-EXPORT;
  - (pytest) a flagged manifest entry without the export must give G3-ERROR.
- **Label.** L5 T4 runs are labelled "truth-fed (gyro and attitude), perfect-model". G3 rejects them as validation
  runs.
- **Rejected.**
  - The truth-gyro pattern through `marv_imu_meas`: the struct has no attitude field (decision 4, overridden).
  - An attitude input in the HAL: it would put a truth path into the flight HAL, which `hal_target` would have to stub.
  - SIL parameter overrides: they are static, not per tick.
  - A `marv_sil_`-named entry: it would pass the product rule silently.
  - A weak-symbol or dlsym lookup in the plugin: a runtime fact where a build-time one is available.

### C. Attitude law (`fw/attitude`, library `marv_attitude`; owner decisions 5, 9, 12 and 13)

PX4's reduced-attitude recomposition with yaw-gain compensation, verified against
`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` at commit `3b828e157a6d9b7d811c80c73d95fd39fca5e181`
(BSD-3). It is computed in a closed form of the same rotations.

Inputs per attitude execution: the measured a = (t_us, q, ω, valid), the setpoint q_sp, and a feed-forward ω_ff (NED,
rad/s). Gains: k (1/s) and the yaw weight w ∈ (0, 1].

1. **Validity.** The input is valid iff a.valid holds, q, q_sp and ω_ff are finite, ‖q‖ > 0 and ‖q_sp‖ > 0. Then
   q̂ = q/‖q‖ and q̂_sp = q_sp/‖q_sp‖.
2. **Error.** q_e = canonical(q̂* ⊗ q̂_sp), with the core §3 sign rule (`quat.hpp:65-67`). It is the rotation from the
   current body frame to the desired one, in current body axes (PX4 `:88`). w ≥ 0 makes it the shortest rotation
   (owner decision 5).
3. **Split into reduced attitude and yaw.** q_e = q_xy ⊗ q_z. With ρ = √(w² + z²):
   - **ρ > 0:**
     - q_xy = (ρ, a, b, 0), with a = (w·x − y·z)/ρ and b = (w·y + x·z)/ρ;
     - q_z = canonical((w, 0, 0, z)/ρ);
     - ψ = 2·atan2(q_z.z, q_z.w) ∈ [−π, π].
   - **ρ = 0 exactly:** q_xy = q_e and ψ = 0.
   - **Correspondence with PX4.**
     - PX4 forms the reduced desired attitude as the shortest world rotation between the thrust axes, applied to q
       (`:60-73`, via `Quaternion(src, dst)`, `matrix/Quaternion.hpp:189-235`).
     - It then takes q_dyaw = qd_red⁻¹ ⊗ qd, canonicalised (`:78-79`).
     - q⁻¹ ⊗ qd_red is MARV's q_xy, and q_dyaw is MARV's q_z. The split is unique for ρ > 0.
4. **Recomposition with the yaw weight** (PX4 `:84-85`). q_e,w = q_xy ⊗ (cos(wψ/2), 0, 0, sin(wψ/2)). With
   c = cos(wψ/2) and s = sin(wψ/2), q_e,w = (ρc, c·a + s·b, c·b − s·a, ρs).
   - PX4 writes the scaled yaw as cos(w·acos(q_dyaw.w)) and sin(w·asin(q_dyaw.z)), with clamps to [−1, 1] (`:80-85`).
   - For a canonical q_dyaw these equal cos(wψ/2) and sin(wψ/2). MARV computes ψ once by atan2, so no domain clamp
     is needed.
5. **Error vector.** eq = 2·imag(canonical(q_e,w)) (PX4 `:90-92`). Since |wψ| ≤ π, ρc ≥ 0; canonical acts only on the
   w = 0 tie.
6. **Command.** r = (k·eq_x, k·eq_y, (k/w)·eq_z) + rotate(q̂*, ω_ff).
   - This is PX4's proportional law (`:95`), its yaw-gain compensation (`setProportionalGain`, `:44-53`, which divides
     the yaw gain by the weight) and its world-z feed-forward rotated into body axes (`:97-106`).
   - Explicitly: r_xy = 2k·Rot(−wψ/2)·(a, b) and r_z = (2k/w)·ρ·sin(wψ/2).
   - The firmware computes k/w once at config load, as PX4 does in `setProportionalGain`.
7. **Clamp.** The roll-pitch pair is scaled by one factor s ≤ 1, the largest with |s·r_x| ≤ `rate_max_roll` and
   |s·r_y| ≤ `rate_max_pitch`. It is written as comparisons, with no division unless a bound is exceeded, and it
   keeps the tilt direction. Yaw is clamped to ±`rate_max_yaw`. PX4 clamps each axis separately (`:108-111`); MARV
   scales the pair so the tilt direction survives.
8. **Output.** The rate setpoint (FRD) goes to `execute_bypass`. It is held until the next attitude execution.

- **Linear regime (owner decision 13).**
  - For small errors, eq ≈ (θ_x, θ_y, w·θ_z), so r ≈ diag(k, k, k)·θ_err: w cancels.
  - The J-normalised rate design gives the same closed rate loop on every axis (0005, "Scaling"). The yaw loop's
    margin therefore equals the tilt loop's, up to the per-axis f32 rate gains and the float rounding of k/w. E
    evaluates both.
- **What w does (large combined errors only).**
  - It bends the tilt command by −wψ/2 about body z. As w → 0 this is the pure shortest-tilt axis; at w = 1 it is the
    full-error axis, bent by −ψ/2.
  - It shapes the yaw command (2k/w)·ρ·sin(wψ/2). At w = 0.143 this is within 0.9 % of k·ρ·ψ for |ψ| ≤ π.
- **Configuration check.** k ≥ 0 and finite, and 0 < w ≤ 1 and finite; else `hal_panic`. PX4 clamps w into [0, 1]
  (`:47`) and skips the compensation below 1e-4 (`:50`). MARV refuses instead, so no threshold number is needed.
- **Faults (0005 owner decision 3, carried over).**
  - An invalid input or a non-finite output gives a zero rate setpoint, fault_active for that execution,
    fault_latched until init, and a fault count.
  - There is no state to reset: the law is a P law.
  - There is no L4 change: L4 then holds zero body rate.
- **Period.** Executions must be spaced N·T within ±1 µs (the L4 tolerance, `rate_loop.hpp:40-41`), else `hal_panic`.
  Nothing is checked at the first execution after init or reset.
- **At and near 180° tilt.**
  - **Exactly (ρ = 0).** eq = 2(x, y, 0), with |eq| = 2, so |r_xy| = 2k about the canonical axis: never zero. The yaw
    command is zero. This equals PX4's own result there: its near-opposite fallback uses the unweighted full error
    qd_red = qd (`:64-69`), which at exactly 180° is also 2(x, y, 0).
  - **Near (ρ > 0, small).** MARV stays on the split. PX4's switch threshold (`fabsf(qd_red(1 or 2)) > 1 − 1e-5`,
    `:64`) is an unsourced number under the number rule, so MARV does not import it. On the split:
    - |a|, |b| ≤ 1 by Cauchy–Schwarz, so every term is finite;
    - |r_xy| = 2k·√(1 − ρ²) stays near 2k;
    - r_z = (2k/w)·ρ·sin(wψ/2) → 0 with ρ;
    - the ill-conditioned yaw split bends the tilt command by at most wπ/2 (about 12.9° at w = 0.143), toward another
      near-shortest tilt axis.
    The result is a pure function of the input bits, so it is deterministic.
  - **Through 180° in pure roll.** ψ = 0 on both sides and the sign stays the same, so the command does not chatter.
- **Differences from PX4, stated.**
  - The canonical form is core §3's exact rule. PX4's `canonical()` uses a FLT_EPSILON threshold
    (`Quaternion.hpp:454-465`).
  - There is no near-opposite switch.
  - The roll-pitch clamp is scaled rather than per axis.
- **T1 tests** (`unit/attitude/`):
  - pure tilt gives eq_z = 0 with the tilt axis ⟂ body z; pure yaw gives r_xy = 0 and r_z = (2k/w)·sin(wψ/2);
  - q_e and −q_e give identical outputs;
  - **the Jacobian does not depend on w.** The small-angle Jacobian is diag(k, k, k), by finite differences with a
    stated tolerance, for the configured w and for w′ = 1. Control: an uncompensated law (yaw gain k instead of k/w)
    gives a yaw entry of w·k and fails;
  - **w changes large combined errors.** For a combined error with large tilt and large yaw, the outputs with w and
    with w′ = 1 differ by more than the float bound. Control: w′ = w gives equal outputs, so the "differs" assertion
    fails;
  - **agreement with PX4.** A double re-implementation of `AttitudeControl::update`'s steps (`:59-95`, BSD-3) agrees
    with MARV's closed form within a derived rounding bound, on random inputs outside PX4's near-opposite region;
  - at ρ = 0 exactly the output is 2k·(x, y, 0);
  - a near-singular sweep stays finite and ≤ rate_max;
  - scaling q or q_sp by a positive factor leaves the output unchanged;
  - the clamp keeps both the direction and the bound;
  - each fault input fails; control: its valid twin passes;
  - the configuration check (w = 0, w > 1 and a NaN each panic);
  - a period death test;
  - the `from_params`/`load_config` wiring.
- **Rejected.**
  - The decomposed metric with gain ratio k_yaw = w·k (the first draft): owner decision 13 chose compensation.
  - Axis-angle α·n.
  - The unweighted full error (w = 1): it couples a large yaw error into the tilt axis, which decision 9 rejects.
  - PX4's code path taken literally: its near-opposite threshold (`:64`) and the `Quaternion(src, dst)` eps
    (`Quaternion.hpp:197`) are unsourced numbers. The closed form computes the same rotations with an exact-zero
    branch.
  - Per-axis Euler errors: singular at ±90° pitch.

### D. Angle mode (`fw/attitude`, the same library; owner decisions 6, 9, 14 and 17)

Inputs per attitude execution: the sticks s = (s_r, s_p, s_ψ) and the measured AttitudeState. It uses no L4 accessor
(owner decision 14).

- **Tilt.** t = θ_max·(s_r, s_p)/max(1, ‖(s_r, s_p)‖).
  - It is linear per axis, as in Betaflight (`pid.c:389`). The stick vector is clamped to the unit disc, so the total
    tilt never exceeds θ_max.
  - Betaflight instead limits each Euler angle (`pid.c:395`), which allows about 75.5° of total tilt on the diagonal
    at 60°. MARV's limit is the tighter.
  - q_xy(t) = [cos(‖t‖/2), sin(‖t‖/2)·t/‖t‖, 0], and the identity when ‖t‖ = 0.
- **Heading.** ψ_m = wrap(2·atan2(q.z, q.w)) into (−π, π]. This is the world-first split q = q_z(ψ) ⊗ q_xy. On its
  singular set (q.w = q.z = 0), ψ_m = 0, the same q_z = identity branch as in C.
- **Deadband and yaw command.** Let d = `yaw_deadband`.
  - The yaw input is active iff |s_ψ| > d. This is Betaflight's strict comparison (`rc.c:700`).
  - ψ̇_cmd = sign(s_ψ)·(|s_ψ| − d)/(1 − d)·`rate_max_yaw`. Betaflight subtracts the deadband (`rc.c:701`) and rescales
    so that full stick is still full rate (`rc.c:778`).
  - With d = 0, ψ̇_cmd = s_ψ·`rate_max_yaw`.
- **Yaw-rate mode (|s_ψ| > d): the heading setpoint tracks the current heading.**
  - q_des = q_z(ψ_m) ⊗ q_xy(t), and q_sp = q̂ ⊗ q_xy(q̂* ⊗ q_des), where q_xy(·) is C's split. q_sp is the current
    attitude tilted by the shortest rotation to the desired thrust axis.
  - So C's split of q̂* ⊗ q_sp has ψ = 0 to rounding, and the yaw error is zero. The bound is derived and checked at
    T1.
  - ω_ff = (0, 0, ψ̇_cmd) in NED.
- **Frame of the yaw-rate command: world down, rotated to body by C** (rotate(q̂*, ω_ff)).
  - The tilt setpoint is defined in the heading frame, which a rotation about world down leaves unchanged. A world-down
    yaw rate therefore creates no tilt error for the P loop to fight.
  - A body-z rate at nonzero tilt swings the thrust axis around the vertical and creates one. Betaflight adds a
    per-axis correction for exactly that (`pid.c:399-405`, "make co-ordinated yaw turns").
  - PX4 feeds its yaw rate about world z the same way (`AttitudeControl.cpp:97-106`).
  - ψ_m is the integral of the world-down rotation, so the lock uses the same quantity the command moved.
- **Yaw rate for the lock.** ψ̇_m = rotate(q̂, a.omega_frd).z, the world-down component of the measured body rates
  (truth at L5, the estimator's at L7), taken from the same `AttitudeState` as q.
- **Braking (owner decision 17).**
  - **Entry.** At the first execution with |s_ψ| ≤ d after yaw-rate mode (the release execution n_r), angle mode
    records three values from that execution:
    - σ_r = sign(ψ̇_m), which is 0 when ψ̇_m = 0 exactly;
    - ω_r = |ψ̇_m|;
    - t_r = the sample stamp (µs).
  - **Setpoint.** While braking, the setpoint is yaw-rate mode's with ψ̇_cmd = 0: the heading tracks the current
    heading, the yaw error is zero and ω_ff = 0. The rate loop brakes the yaw rate, and no heading target pulls on it.
  - **Lock on the crossing.** The heading locks at the first braking execution, n_r included, where σ_r·ψ̇_m ≤ 0: the
    rate has reached zero or changed sign against the release sign. With σ_r = 0 it locks at n_r.
  - **Fallback (guard b; owner decisions 17 and 18).** Otherwise it locks at the first braking execution where both
    hold: Δt ≥ t_cross AND Δt·α_min ≥ ω_r. Both must hold, so the fallback fires at max(t_cross, ω_r/α_min).
    - Δt = static_cast<T>(t − t_r)/`kMicrosecondsPerSecond`, computed from the µs stamps (`hal_time_us`, equal to the
      sample stamp). The constant is the existing one in `constants.hpp`, so there is no literal.
    - Both are comparisons, one of them a product, so nothing divides.
    - t_cross = `att_yaw_t_cross` (E): the latest linear-regime braking time over the box.
    - α_min = `att_yaw_alpha_min` (E): the smallest α_max,yaw over the box. Its term covers releases so fast that
      torque saturation, not the linear loop, sets the stopping time.
    - In the linear regime every box member crosses zero no later than t_cross, so the fallback fires only when no
      crossing comes: a disturbance, or a regime the design model does not cover. F exercises it directly.
  - **Locking.** In both cases ψ_lock := ψ_m of the locking execution. Then q_sp = q_z(ψ_lock) ⊗ q_xy(t), decision 9's
    full-quaternion setpoint, and ω_ff = 0. The heading setpoint never runs ahead of the vehicle, so there is no
    runaway.
- **Guard (a): one lock per release.** Once locked, the lock holds and the crossing and fallback tests are disarmed
  until the stick leaves the deadband. Later sign changes of ψ̇_m cannot re-lock, so gyro noise at L6 and later cannot
  chatter it.
- **Re-arm.** At the first execution with |s_ψ| > d, from braking or from the lock, angle mode returns to yaw-rate
  mode. A later release starts a new braking phase.
- **Initialisation.** At the first valid execution after init, a reset or a fault: yaw-rate mode if |s_ψ| > d, else
  lock at ψ_m at once. The rate history of the fault is not trusted.
- **Faults.** Any of these marks the setpoint invalid, and C's fault path applies (zero rate setpoint):
  - an invalid measured attitude, which also covers a non-finite `omega_frd`, since the lock reads it;
  - non-finite sticks;
  - |s_i| > 1.
  The braking state and the lock are dropped, and the next valid execution re-initialises.
- **Time-dependent state.** The only one is the fallback timer. It is measured from stamps, and C's period check
  applies to the execution.
- **Consequence (architect scratch, the linear design model at N = 2, no torque limit, full-rate release after a 1 s
  hold).** This is from before decision 21; the built T4 results at N = 1 are in F.
  - The crossing sets the lock at every corner, 0.112–0.249 s after release. The fallback, max(0.256 s, 0.140 s), is
    never reached.
  - Travel from release to lock is 0.66–1.51 rad.
  - After the lock, the rate loop's PI response (undershoot, and the integrator releasing) moves the heading a further
    0.32–1.07 rad from ψ_lock before the lock pulls it back. The worst case is the heavy-inertia corners.
  - The T4 tests bound this by the band envelope. Nothing is tuned.
- **Stick source at L5.** The L5 test composition's own register holds the stick script, the chirp (F), the constant
  yaw disturbance of the fallback case (F) and the collective thrust.
  - The script is K segments (t_us, s_r, s_p, s_ψ). At init, stamps must be ≥ 0 and strictly increasing, and sticks
    finite and within [−1, 1]; otherwise `hal_panic`.
  - There is no pilot until L8. There the manual-control struct replaces the script behind the same input, and the
    rates curve (QF-1) arrives, so at L5 the script supplies the shaped stick directly.
  - Sign convention: positive s_r rolls right side down, positive s_p pitches nose up, positive s_ψ yaws nose right.
    The mapping of physical sticks is L8's.
- **Registers** (`design/scenario_values.yaml`, method scenario, σ choice, used_by angle mode):
  - **`angle_tilt_max`** = 1.0471975511965976 rad (60° × π/180). Betaflight 4.5.2, commit
    `024f8e13d4e642eb6a380308685b9ea3aa3ef1a2`:
    - `src/main/flight/pid.c:138` `.angle_limit = 60` (the default, in degrees);
    - `src/main/cli/settings.c:1169` (configurable from 10 to 85);
    - `pid.c:389` (the linear map).
    The geometry differs from Betaflight's, as described under Tilt.
  - **`yaw_deadband`** = 0.0 (0/500, a stick fraction). It is written 0.0 so that it is an f32 parameter (the
    register header's rule). Same commit:
    - `src/main/fc/rc_controls.c:79` `.yaw_deadband = 0` (the default, in rc units of the 500-unit half range);
    - `src/main/cli/settings.c:1084` (range 0–100);
    - applied at `src/main/fc/rc.c:700-701` (subtracted) and `rc.c:778` (divider 500 − yaw_deadband).
    Consequence: the lock engages only at exactly zero stick. With real sticks at L8 this value will need revisiting.
    The generator refuses d outside [0, 1).
  - **Frozen L04 tests stay green.** They test membership and per-entry method (`test_scenario_values.py:80-95,
    193-201`).
- **T1 tests** (`unit/angle/`):
  - tilt(q_sp) ≤ θ_max over a stick sweep, and on-axis full stick gives θ_max;
  - **deadband.** It is strict: with d = 0, s_ψ = 0 locks and the smallest positive float stick is yaw-rate mode. The
    rescale gives full rate at full stick for every d in [0, 1);
  - **yaw-rate mode.** C's split of the setpoint error has ψ = 0 within the derived float bound, and
    ω_ff = (0, 0, ψ̇_cmd);
  - **braking and the crossing.** On a planted ψ̇_m sequence, the lock happens at the first execution with
    σ_r·ψ̇_m ≤ 0, and ψ_lock equals that execution's ψ_m. A release with ψ̇_m = 0 locks at n_r. While braking,
    ω_ff = 0 and the yaw error is zero. Control: a planted lock at the heading where the yaw input started (the
    catch-up behaviour) fails;
  - **guard (a).** After the lock, a ψ̇_m sequence with repeated sign changes and the stick centred leaves ψ_lock and
    q_sp unchanged. Control: the same sequence with the stick leaving the deadband and returning between two changes
    must produce a second, different lock, which shows the test can see a re-lock;
  - **guard (b).** A same-sign ψ̇_m that never crosses locks exactly at the first execution with Δt ≥ t_cross and
    Δt·α_min ≥ ω_r. That execution index is computed from the stamps, including a stamp spacing of half-µs periods.
    The test covers both regimes: ω_r small (t_cross binds) and ω_r large (ω_r/α_min binds). Controls:
    - the execution before the fallback is still braking;
    - AND, not OR: with ω_r large, the first execution past t_cross, where only the t_cross test holds, is still
      braking; with ω_r small, the first execution past ω_r/α_min, where only the α test holds, is still braking;
    - a sequence that crosses zero earlier locks at the crossing, not at the fallback;
  - re-arm on leaving the deadband, from braking and from the lock;
  - initialisation both ways, including the singular set;
  - faults drop the braking state and the lock, including a non-finite `omega_frd`.
- **Rejected.**
  - A heading integrated from the stick (the first draft): without FF it runs away, and it needs a wind-up guard (owner
    decision 14).
  - A heading-hold accessor on L4 (owner decision 14).
  - A lock at the release execution (decision 14's first form): it pulls the heading back by the whole stopping
    distance (decision 17).
  - Re-locking on every zero crossing: gyro noise would chatter it (guard a).
  - Computing α_min in firmware from the mixer: it would duplicate `rate.py`'s air-mode envelope (`rate.py:220-242`).
  - A body-z yaw-rate command: it causes tilt wobble.
  - An Euler setpoint: 75.5° on the diagonal, and singular at 90° pitch.
  - Betaflight's yaw as a pure rate axis in angle mode (`pid.c:899-916`): it has no heading lock, against decision 9.

### E. Gain rule, loop rate and design model (`tools/card/attitude.py`; `flatten.py --out-attitude`)

- **Rate-loop model at T.**
  - **Scope.** Per axis a and per corner (j, τ_c) of `rate.py`'s `corner_list` (J_true/J_a = j, J-normalised).
  - **Gains.** The axis's firmware gains, κ_p = f32(J_a κ_p)/J_a and κ_i likewise, with kd = 0, and the law of
    `rate_loop.hpp` in bypass.
  - **State** at the start of rate execution n: s = [m, ω, θ, I, e_prev], i.e. lagged torque, rate, angle,
    integrator and previous error.
  - **Update.** With e = exp(−T/τ_c) and g = T − τ_c(1 − e):
    - I⁺ = I + κ_i·T·e_prev; e_n = r_n − ω; u = κ_p·e_n + I⁺;
    - m′ = e·m + (1 − e)·u;
    - ω′ = ω + (τ_c(1 − e)·m + g·u)/j;
    - θ′ = θ + T·ω + (τ_c·g·m + (T²/2 − τ_c·g)·u)/j.
  - **Exactness.** So s_{n+1} = A·s_n + B·r_n exactly. These are the ZOH closed forms of `rate.py:13-15, 212-216`,
    plus the angle.
- **Attitude at T_a = N·T, by lifting.**
  - N = `att_loop_ratio` = 1 by the parent-rate rule (below). The lifting is kept for the case where EMB-3 fails and
    a lower rate is chosen; at N = 1 it is the plain closed rate loop.
  - The attitude law samples θ at n = kN, and its output is held for N rate executions.
  - The computation delay is zero. The composition runs the attitude group before the rate group in the same tick,
    and the output acts at that tick, as L4's torque does.
  - s_{(k+1)N} = A^N·s_{kN} + (Σ_{i<N} A^i·B)·r_k. So G_N(z) = C_θ (zI − A^N)⁻¹ Σ A^i B is exact and LTI at T_a: the
    samplers are synchronous and the inner loop is exact at T.
  - The loop is L_a = k·G_N, the same on every axis in the linear regime (C).
- **Margin.**
  - **Crossover.** |L_a(e^{jωT_a})| = 1, unique on `rate.py`'s 1024-point log grid, then bisected.
  - **Phase margin.** PM = π + arg L_a on the continuous branch, anchored at −π/2 at low frequency (G_N's single pole
    at z = 1). It is unwrapped on the grid and confirmed by one grid doubling: the branch must not change.
  - **Stability.** The Jury test on A^N − k·(ΣA^iB)·C_θ at every corner. A failure refuses.
- **Box (owner decision 1).**
  - The rate gains are fixed at the nominal design. Each corner closes the rate loop on its own plant.
  - PM_worst = the minimum over nominal plus the four corners and over the three axes.
    - Roll and pitch are evaluated at f32(k).
    - Yaw is evaluated at its effective linear gain, f32(f32(k)/f32(w))·f32(w): the product the float law realises.
    - Every axis uses its own f32 rate gains.
  - The rule uses the corners. T3 checks a grid (F); if the grid falls below PM_min − δ_num, T3 fails and the finding
    goes to Luis.
- **Gain rule: 0005's sup rule, transposed to the P gain.**
  - k = sup{k′ : PM_worst(k″) ≥ PM_min for all k″ ∈ (0, k′]}, at T_a.
  - **Search.** A geometric scan from (π/T_a)·2⁻¹⁷ in steps of 2^(1/16) up to the first infeasible point, then
    bisection until f32(k) stops changing. The method constants are shared with `rate.py`.
  - **Guard.** PM_worst(f32 gains) ≥ PM_min + δ_num, with δ_num = max|dPM/dk| × the final bracket width. A failing
    candidate falls back to the previous feasible point.
  - **Relation to the toolbox.** This is loopshape's `outerLoop` (`loopshape.ts:88-107`: the P gain at the phase
    margin around T = L/(1+L)), made robust and exactly sampled.
- **Yaw weight (owner decisions 12 and 13).**
  - w = α_yaw/min(α_roll, α_pitch), with α_a = τ_max,a/(J_a·(1 + `inertia_robustness_band`)) from `rate.py` (`:379`;
    the band cancels). w is clamped to at most 1.
  - The generator refuses w ≤ 0 or a non-finite w. w is emitted and reported.
  - By the compensation, w does not enter the linear loop.
- **Loop rate (owner decision 21; supersedes decisions 3 and 10).** `att_loop_ratio` = 1: the attitude loop runs at
  its parent group's rate, the rate loop's 3.2 kHz (core §7.5, "Flight rate groups"; quad §4 L5 Builds).
  - **EMB-3.** It holds on §5.4's estimates. By §5.4's cost model (`10-quad-flight-software.md:364`), the attitude law
    adds under 1 % of core 1 at 3.2 kHz, on top of §5.4's core-1 Freestyle rows (3–7 % and 2–4 %). This is the lead's
    estimate; L9 re-checks it with measured WCET.
  - **What is removed.** SIM-7 minimisation, its uncertainty U, the fixed point and `design/measured/sim7_u/` are all
    gone. The 800 Hz re-measure was stopped as moot.
  - **Consequence for the chirp.** It still measures its own U per axis (F), but that U no longer chooses a rate.
- **Parameters emitted** (the product set; `tests/regression/quad/L05/param_ids`):
  - `att_kp` (f32, 1/s, derived, σ UNKNOWN);
  - `att_yaw_weight` (f32, unit "1", derived by owner decision 12's rule, σ UNKNOWN);
  - `att_loop_ratio` (i32, unit "1", derived by the flight-rate-group rule of core §7.5, σ exact);
  - `angle_tilt_max` (f32, rad, scenario, σ choice);
  - `yaw_deadband` (f32, unit "1", scenario, σ choice);
  - `att_yaw_alpha_min` (f32, rad/s², derived, σ UNKNOWN), for the fallback of owner decision 17.
    - It equals τ_max,yaw/(J_yaw·(1 + `inertia_robustness_band`)): 0005 QF-2's α_max,yaw, with J at the top of its
      band and `rate.py`'s air-mode τ_max,yaw.
    - τ_max depends on the mixer and rotor limits, not on the motor τ or J, so J at the top of its band gives the
      smallest α over the box.
    - It is stored as the largest f32 at or below the value, so that the fallback never fires before the stopping
      time the rule derives.
    - The firmware validates α_min > 0 and finite.
  - `att_yaw_t_cross` (f32, s, derived, σ UNKNOWN), for the fallback of owner decision 18.
    - **Model.** The yaw axis of the lifted model: that axis's f32 rate gains, the bypass law and attitude period
      T_a = `att_loop_ratio`·T. It starts from the steady state of tracking a unit yaw rate (ω = 1, m = 0, I = 0,
      e_prev = 0), and the reference is zero from the release execution on. This is D's braking: yaw error zero,
      ω_ff = 0.
    - **Crossing time.** t_c = n·T_a, with n the first attitude execution n ≥ 1 at which ω ≤ 0. The model is linear
      from a unit rate, so t_c does not depend on ω_r.
    - **Value.** t_cross = the maximum of t_c over nominal and the four corners, stored as the smallest f32 at or above
      it (rounded up, so the fallback never fires before a linear-regime crossing).
    - **Refusals.** The generator refuses if some member does not cross within its scan bound (`MAX_STEP_SAMPLES`, the
      method constant `rate.py` uses for t63).
    - It regenerates whenever the rate loop or `att_loop_ratio` changes (owner decision 18).
    - The firmware validates t_cross > 0 and finite.
  - The firmware also reads `rate_max_*` (the clamp and the yaw-rate command), `rate_loop_divisor` and the tick
    period. The attitude divisor is `rate_loop_divisor` × `att_loop_ratio` ticks.
- **Report** (`marv_params_attitude_report.txt`). It contains:
  - the inputs and the f32 rate gains used;
  - the model statement;
  - per-corner PM, crossover and Jury radius at T_a, per axis;
  - the loop-rate rule and its EMB-3 basis;
  - w, with the three α;
  - k in f32 and double, and the yaw axis's effective linear gain;
  - `att_yaw_alpha_min`, with its τ_max,yaw and J inputs;
  - t_c per corner, and `att_yaw_t_cross`;
  - δ_num, the bracket, the bisections and the stepped-down flag.
- **Live values** (generator, 2026-09-30, `att_loop_ratio` = 1).

  | Quantity | Value |
  | --- | --- |
  | k (`att_kp`, f32) | 3.0872879 s⁻¹ |
  | PM, every axis | 69.390° nominal; worst 45.000018° at (J+, τ+) |
  | Crossover, every axis | 3.477–4.610 rad/s |
  | w (`att_yaw_weight`) | 0.143255815 |
  | `att_yaw_alpha_min` | 83.327 rad/s² |
  | `att_yaw_t_cross` | 0.25625 s |

  - Under the compensated law the yaw loop is the tilt loop, up to f32 rounding.
  - The attitude loop's worst corner is (J+, τ+); the rate loop's is (J−, τ+).
- **Rejected.**
  - loopshape's e^{−sT} model (owner decision 2).
  - SIM-7 minimisation of the attitude rate (decisions 3, 10 and 20; superseded by 21). The saving was under 1 % of
    core 1, and the rule needed a rate-dependent uncertainty closed by a fixed point.
  - A separate yaw gain w·k (the first draft; owner decision 13).

### F. Tests and controls (core §7.2; owner decisions 7, 14–21)

- **Fixed-input references.** Each is on a fixed input the test owns:
  - the T3 step golden with its envelope;
  - the Q table (`t3/reference/attitude_t3_q_inputs.txt` → `attitude_t3_q.txt`);
  - the acro bit-identity golden (A).
  Every other test asserts a rule on the live parameters.
- **T1.**
  - A: `unit/rate_bypass`.
  - B: `unit/truth_state`. The status codes and the valid-bit rule. Also, the float-cast source keeps |‖q‖ − 1| within
    the derived rounding bound of a binary64 unit quaternion rounded to binary32, derived as in 0003 item 6.
  - C: `unit/attitude`.
  - D: `unit/angle`.
  - Composition: `unit/composition`.
    - attitude ticks are a subset of rate ticks;
    - the attitude group runs before the rate group;
    - the stale-attitude panic;
    - the rate setpoint is held between attitude executions;
    - the product ids are a prefix of the composition's set, as `check_param_prefix.py` checks at L4.
- **T3 margins** (`tools/`, on the live parameters).
  - PM_worst ≥ PM_min − δ_num at nominal and at the corners, on the f32 gains, for all three axes.
  - The yaw axis's effective linear gain equals f32(k) within the derived f32 bound: the compensation holds.
  - A 17×17 box grid, refined by halving with the last-halving change recorded; its minimum ≥ PM_min − δ_num.
  - Tightness: the next f32 k above the rule's gives PM_worst < PM_min.
  - The Jury test.
  - w follows its rule.
  - `att_yaw_alpha_min` follows its rule: rounded down, and equal to the L4 report's α_max,yaw rounded down.
  - `att_yaw_t_cross` follows its rule, recomputed from the live parameters and rounded up.
    - Grid check: on the 17×17 box grid (halving), every member's t_c ≤ `att_yaw_t_cross`, else T3 fails and the
      finding goes to Luis.
    - Control: `att_yaw_t_cross` one f32 step below the corner maximum fails the rule.
  - `att_loop_ratio` = 1, the parent-rate rule.
  - Controls: k × 1.1 fails the margin check, and so does one added tick of delay in the lifted model.
- **T3 step** (`t3/`, C++, the 0005 pattern).
  - **Setup.** The float firmware path (angle mode, then the attitude law, then `execute_bypass`, each at its period)
    runs against the double design plant at tick resolution: exact per-axis ZOH rate dynamics and exact quaternion
    kinematics, without ω×Jω.
  - **Inputs.** Full-stick roll and pitch steps from level and their release, and a sustained full yaw stick and its
    release.
  - **Golden.** From an independent double oracle (`t3/reference/attitude_t3_oracle.py`) on the fixture
    `t3/reference/attitude_t3_inputs.txt` (the product values on the day). The tolerance is derived from the float
    law, as a first-order rounding bound, as in 0005.
  - **CI.** Reproduces the golden byte for byte, with a perturbed-input control.
  - **Controls.** Attitude gain × 1.1, rate gains × 1.1 and one tick of added delay must each leave the tolerance.
  - **Kinematics.** Integrated with sub-steps halved until the trajectory changes by less than the tolerance
    (core §7.5).
  - **Result at N = 1 (regenerated after decision 21).**
    - **Tolerances.** θ 1.78e-5 rad and ω 6.78e-5 rad/s on roll and pitch; 2.80e-4 rad and 1.27e-3 rad/s on yaw. The
      derivation of the yaw bound was tightened with per-execution ρ weighting.
    - **Controls.** On roll and pitch, attitude gain × 1.1 leaves the tolerance by ×3946, rate gains × 1.1 by ×3338
      and one added tick by ×46.4. On yaw: ×192, ×492 and ×8.2.
    - **Size.** The reference files (golden plus envelope) are 8.6 MB.
    - **Reviewer caveat.** The frozen-gain first-order rounding bound relies on about 100× of observed slack.
- **T3 envelopes** (recorded, used at T4).
  - For each step script, the yaw-release script and each recovery scenario: the box envelope (17×17 grid, halving)
    of the design model, driven by the exact script from the exact initial state and never re-seeded (Luis's L4 rule).
  - **Property.** Each envelope's end value (tilt, heading relative to release, yaw rate) is below the T4 tolerance,
    so a trace inside the envelope has settled. Otherwise the duration doubles (core §7.5).
- **Tolerance terms at T4.** As at L4, plus Q (owner decision 19):
  - E is from 0003 item 9 (m = 1 against m = 2);
  - F = the T3 tolerance + the envelope's last-halving change + the kinematics halving change;
  - Q = the maximum over the box of |quantised − unquantised| design-model trajectory for the same script. It is
    quantised by the firmware's `thrust_to_dshot` and the card's ESC map, computed on the fixture
    `attitude_t3_q_inputs.txt`, and is a grid maximum, because the corners do not bound it.

    | Script | Q |
    | --- | --- |
    | `step_roll` | θ 8.28e-3 rad |
    | `step_pitch` | θ 7.40e-3 rad |
    | `yaw_release` | ω 5.83e-3 rad/s |
    | `yaw_fallback` | ω 4.95e-3 rad/s |

  - The T3 envelope itself is unchanged. Every T4 predicate below uses envelope ± (E + F + Q). Owner decision 24:
    fix a harness or setup limitation before adding a derived term, and after any new term re-confirm that the fine
    negative controls still fail.
- **T4 angle-mode steps** (gz).
  - **Script.** Full-stick roll and pitch segments, and their release.
  - **Predicate.** At every attitude execution, the tilt components and the heading relative to the lock lie inside
    the envelope ± (E + F + Q) (owner decision 19).
  - **Other checks.** A clean run and every DShot within [idle, 2047].
  - **Altitude.** Drift is allowed (0005 decision 9). The generated world has no ground plane (`gen_world.py:32`), so
    there is no contact.
  - **Controls.** `att_kp` = 0 through a SIL override (the §7.2 metric control); the step moved one attitude execution
    off its stamp.
  - **Result.**
    - Roll is 6.52e-3 rad outside the envelope, against F + Q = 1.127e-2; pitch is 5.94e-3 rad outside, against
      1.038e-2.
    - **Cause.** DShot quantisation at hover, measured by bit-exact replay and a counterfactual
      (`tests/regression/quad/L05/results/step_cause/`).
  - **Fine controls against the widened predicate** (owner decision 19; `results/step_controls/`).
    - `att_kp` = 0 and `att_kp` × 1.1 fail in gz: slack +3.07e-2 / +3.21e-2 (roll / pitch).
    - Rate gains × 1.1 pass: that plant is inside the box.
    - One added tick passes at T4, with counterfactual slack −4.77e-3 / −4.45e-3; its design shift, 3.69e-4, is below
      F.
    - **Recorded as decision 19 asks.** T4 angle steps now catch gross failures and attitude-gain errors only. The
      delay metric is enforced at T3 (×46.4 on roll and pitch, ×8.2 on yaw).
- **T4 yaw release (owner decisions 14 and 17; gz).**
  - **Script.** Level tilt, full yaw stick held for a sustained segment, then s_ψ = 0 at a stamped release execution
    n_r.
  - **Quantities.** Taken from the truth attitude at each attitude execution:
    - the world-down yaw rate ψ̇_m;
    - the heading relative to the release, ψ_m(n) − ψ_m(n_r);
    - after the lock execution n_l, the heading relative to the lock, ψ_m(n) − ψ_m(n_l).
    - n_l is recomputed from the log by D's rule: the crossing or the fallback, with the emitted α_min.
  - **Envelope.** The design model over the box, including D's braking, crossing, fallback and lock logic, driven by
    the exact script from the exact initial state and never re-seeded. Each member's headings are taken relative to its
    own release and its own lock.
  - **Predicate.**
    - (i) During the sustained segment, the yaw rate lies inside its envelope ± (E + F + Q). This is the angle-mode yaw
      step.
    - (ii) **The yaw rate reaches zero with no reversal beyond the band envelope.** After release, σ_r·ψ̇_m lies inside
      its envelope ± (E + F + Q) at every attitude execution. This includes the envelope's most negative value, which is
      the reversal bound.
    - (iii) **The heading holds after the lock.** For n ≥ n_l, ψ_m(n) − ψ_m(n_l) lies inside its envelope ± (E + F + Q).
      Also, n_l lies within the envelope's range of lock executions.
    - (iv) After release, the heading relative to the release lies inside its envelope ± (E + F + Q).
    - (v) At the end, the yaw rate and the heading relative to the lock are within E + F + Q of zero. By the T3
      property, the envelope has closed there.
  - **Other checks.** A clean run and the DShot range.
  - **Controls.**
    - A planted trace whose heading goes back to the heading where the yaw input started (the catch-up behaviour
      decision 14 excludes) must fail.
    - `att_kp` = 0 through a SIL override (no pull-back to the lock) must fail.
    - The release moved one attitude execution off its stamp must fail.
  - **Result.**
    - The crossing set the lock at execution 21346. That is inside the design range [21087, 21552] and before the
      fallback at 21553.
    - Every channel is within 7.7e-6 to 5.8e-4 of the envelope, against E + F + Q.
    - All the controls fail as required.
    - The lock execution is recomputed from the TRUTH records with D's rule, in f32.
- **T4 yaw-lock fallback (owner decision 18; gz).** It exercises guard (b) directly.
  - **Injection.** A constant yaw torque d is added to the rate loop's torque output before `allocate()`. This is the
    L4 chirp's injection point (`l4_rate_scripted.cpp:151`), and the anti-windup sees the request including d, as at
    L4.
    - It is on from the release stamp. The L5 register holds `l5_dist_yaw_nm` and `l5_dist_t0_us`, with neutral
      default 0.
    - Starting it at release matters: a disturbance present during the hold would be cancelled by the rate integrator
      before release, and the braking would cross zero as usual.
  - **Script.** The yaw-release script: level tilt, full yaw stick held, released at n_r.
  - **Amplitude (derived, no literal).** d = σ_r·τ_held,yaw, the collective-held yaw torque envelope of 0005's chirp
    rule, so the mixer stays linear.
    - T3 property: every box member of the design model, with the same script and d, keeps σ_r·ψ̇ > 0 up to its
      fallback execution.
    - If some member crosses, the held stick halves (core §7.5) until none does. Scratch: full stick suffices.
  - **Predicate.**
    - (i) **No crossing.** σ_r·ψ̇_m > 0 at every attitude execution from n_r up to n_fb, the first execution with
      Δt ≥ t_cross and Δt·α_min ≥ ω_r. n_fb is recomputed from the log with the emitted parameters and the logged
      ω_r.
    - (ii) **The fallback locks, and the heading holds against d.** For n ≥ n_fb, ψ_m(n) − ψ_m(n_fb) and ψ̇_m lie
      inside the envelope ± (E + F + Q). The envelope is the design model with the same script, d and lock logic, never
      re-seeded. At the end both are within E + F + Q of zero: the lock held, and the integrator rejected d.
    - (iii) A clean run and the DShot range.
  - **Controls.**
    - `att_yaw_t_cross` overridden through the SIL to the run duration (a value taken from the scenario, not a new
      number), so the fallback cannot fire within the run: the heading is never pulled back to ψ_m(n_fb), and (ii)
      must fail.
    - A planted trace that crosses zero before n_fb must fail (i).
  - **Result.**
    - No crossing: the minimum of σ_r·ψ̇ is 1.406 rad/s.
    - The lock came at n_fb = 21553.
    - The worst excesses are 7.4e-4 rad/s (yaw rate from the fallback on, allowance 0.092) and 6.2e-4 rad (heading
      relative to the lock, allowance 4.4e-3).
    - All the controls fail as required.
- **T4 attitude chirp (owner decisions 16 and 20; gz).** QF-3 needs it.
  - **Injection.** At the attitude output, in rad/s, added to the rate setpoint before `execute_bypass`, one axis at a
    time (roll, pitch and yaw, all at k).
  - **Scenarios.** Derived by the committed `tools/sim/gen_l5_chirp.py` from the live design; a test compares them
    byte for byte.
  - **Band.** [min corner ω_c/a, a·max corner ω_c] (0005).
  - **Amplitude (owner decision 20).** A sweep of A_env/2^k, k = 1..5.
    - The measured margin is the minimum over the amplitudes that produce a crossover; U_A is the spread of that
      plateau.
    - A_env is the amplitude of this record's first rule (the torque envelope τ_held). At full size it made the vehicle
      tumble, so that rule is superseded.
    - Decision 20 names roll and pitch. Yaw keeps this record's first rule (lead): at A_env (1.467 rad/s) the yaw
      excursion stays small (peak error 0.44 rad), and U_A = |PM(A) − PM(A/2)| as in 0005.
  - **Duration.** Doubled until |ΔPM| < E_H + U_A.
  - **Estimator and reconstruction.** The DTFT ratio. L_a = C_a·G_m/(1 − C_a·G_m), with the implemented f32 gain.
  - **Pass.** PM − U ≥ PM_min, with U = E_H + U_A + U_d per axis (0005's terms). This contains decision 20's
    min − U_A ≥ PM_min. It no longer checks a SIM-7 fixed point (decision 21).
  - **Control.** Gains × 2 on every axis must fail.
  - **Result at N = 1.**

    | Axis | PM | U | Slack | Gains × 2 control |
    | --- | --- | --- | --- | --- |
    | Roll | 68.904° | 2.80° | 21.11° | 33.6°, fails |
    | Pitch | 68.896° | 2.25° | 21.65° | 33.6°, fails |
    | Yaw | 69.397° | 0.085° | 24.31° | 33.4°, fails |

- **T4 large-angle recovery (owner decisions 15, 22, 23 and 24; gz)**, with labelled scenario test values.
  - **Setup (owner decision 23; 0007).**
    - Every recovery scenario starts with the rotors at the card's hover speed, 1100.58 rad/s
      (`run_l5.hover_rotor_speeds`: ω_i = √(M[i,thrust]·m·g/k)).
    - This fixes the harness limitation of rotors starting at rest.
    - The earlier rest-rotor measurements are superseded; they are in commit `7028cf8`.
  - **Predicate.** A clean run, the DShot range, and every channel inside the design-model envelope ± (E + F + Q),
    where the envelope comes from the exact initial state and is never re-seeded. The design model omits ω×Jω, as at
    L4, so a coupling limit shows up as an envelope exit.
  - **R1: near-inverted at rest** (`recover_inverted.yaml`; owner decision 22).
    - **Start.** Roll π − δ, with δ = 0.01 rad, a labelled scenario test value.
    - **Why δ.** The plant's rounding noise in (w, z) is about 4e-17, against w = 5e-3 at that start. So the design
      model and Gazebo take the same tilt/yaw split branch.
    - **Result.** 0 violations of envelope ± (E + F + Q) on six channels (err_x, err_y, err_z; ω_x, ω_y, ω_z).
      - The worst is ω_x: 6.4e-3 outside the envelope, slack −0.101.
      - err_x is 5.21e-3 outside, slack −8.86e-3.
  - **Exact-180° α side check** (`recover_inverted_exact.yaml`; owner decision 22).
    - **What it checks.** The tilt angle α against an axis-invariant design envelope over the 17×17 box, from exactly
      180°.
    - **Result.** 4.5e-4 outside the envelope against E = 9.6e-4, slack −4.77e-2.
    - **Why a full check from exactly 180° is not generic.** On the singular set, the plant's rounding noise picks the
      split branch.
    - **The law stays within C's bound.** The measured bend is 6.447° = −wψ/2, against wπ/2 = 12.893°.
  - **R2: inverted and tumbling** (`recover_tumble.yaml`). Rates at `rate_max` on all three axes, and rotors at hover.
    - **Status.** A strict xfail (`raises=AssertionError`, `strict=True`) on "`tests/regression/quad/L06/` does not
      exist". It extends the L4 known failing item: it blocks L6 and must pass before L8 (owner decision 15).
    - **Cause, measured.** Evidence is in `tests/regression/quad/L05/results/recovery_cause/`, from a bit-exact replay
      of 20747 executions.
      - No L3 flag is set; s = t = 1 throughout.
      - At execution 1, |ω×Jω| is (0.302, 0.245, 0.055) N·m. That is 41/44/33 % of τ_held, and above the largest
        roll and pitch torques requested (0.136, 0.213).
      - Counterfactual distance to Gazebo: the design model 7.96; with ω×Jω added, 1.46e-2; with DShot too, 7.9e-3.
    - **Behaviour.** The vehicle does recover.
      - α < 90° at 0.259 s, against 0.124 s for the design model; final α is 8.1e-4 rad.
      - The test leaves the envelope from execution 3, by up to 0.80 rad and 5.3 rad/s.
  - **Harness facts.**
    - **Rate channels at execution 0.** The plugin applies the initial rates after host step 0 (0003 item 11), so the
      rate channels skip execution 0 only. This is not a tolerance term, and the attitude channels stay checked.
    - **Separatrix field.** The runner fills the L2 separatrix-margin field with the state's own value, because the
      analytic separatrix check does not apply to a recovery scenario (lead decision, H).
    - **F's rounding term.** The T3 rounding term in the recovery envelope is borrowed from the step envelopes and
      tagged INFERRED. It is immaterial: about 1e-5, against a halving term of 4.1e-2.
  - **Controls.**
    - A planted trace held at the initial state fails R1, R2 and the side check.
    - `att_kp` = 0 fails: 23286 violations on R1, 68062 on R2.
    - `att_kp` × 1.1 still fails R1 (worst slack +0.217, on ω_x). So the fine control holds with no new term (owner
      decision 24).
- **T4 truth plumbing.** The TRUTH record equals the float cast of the step's body state. Without the element, the log
  is byte-identical to an L4 run.
- **"The L4 suite still green."** No new check is added.
  - The L04 frozen suites run unchanged in both images: ctest frozen, the L04 tools, and the L04 gz suite (40 passed,
    1 xfailed, 0 skipped).
  - The L5 report quotes their counts.
  - L5 must not create `tests/regression/quad/L06/`.

### G. Layout and CI

- **Firmware.**
  - `fw/attitude/` (`marv_attitude`): `attitude_loop.hpp` and `angle_mode.hpp`, with sources holding
    `from_params`/`load_config` and the float instantiation.
  - `fw/types/include/marv/types/attitude_state.hpp`.
  - `fw/rate`: `execute_bypass` only.
- **Composition.**
  - `fw/compositions/l5_attitude_scripted/`:
    - `include/marv/composition.hpp`, which declares `attitude_input`;
    - `include/marv/l5_script.hpp`;
    - `src/`;
    - `params/l5_attitude_scripted_register.yaml`, which holds the `l5_*` stick segments, the chirp, the yaw
      disturbance (`l5_dist_yaw_nm`, `l5_dist_t0_us`) and the thrust.
  - `fw/params/CMakeLists.txt`: `marv_params_l5_attitude_scripted` (the product set plus EXTRA_REGISTER). Attitude
    generation runs for every set with SCENARIO.
- **SIL.** `fw/sil/include/marv_truth.h`, `fw/sil/src/truth_state.cpp`, `fw/sil/marv_sil_truth.map`, and the
  TRUTH_STATE option in `fw/sil/CMakeLists.txt`. `sim/l5_attitude_scripted/CMakeLists.txt`:
  `marv_add_sil_library(marv_sil_l5_attitude_scripted COMPOSITION marv_composition_l5_attitude_scripted
  PARAMS marv_params_l5_attitude_scripted TRUTH_STATE)`.
- **Gazebo.** `sim/gz/adapter` gains `truth_attitude` (target `marv_gz_truth_attitude`); the plugin gains the element
  and record type 5. `tools/sim/lockstep_log.py` and `tools/card/gen_world.py` gain the record and the element.
- **Generators and registers.** `tools/card/attitude.py`, `tools/card/flatten.py` and `design/scenario_values.yaml`
  (`angle_tilt_max`, `yaw_deadband`). `tools/sim/gen_l5_chirp.py` generates the chirp scenarios.
- **Runner and scenarios.** `tools/sim/l5_scenario.py` and `tools/sim/run_l5.py`. `scenarios/quad/L05/`: `step_roll`,
  `step_pitch`, `yaw_release`, `yaw_fallback`, `chirp_roll`, `chirp_pitch`, `chirp_yaw`, `recover_inverted`,
  `recover_inverted_exact` and `recover_tumble`.
- **Preset.** `CMakePresets.json` gains `host-gz-l5` (MARV_GZ_SIL = `marv_sil_l5_attitude_scripted`, MARV_GZ_PARAMS =
  `marv_params_l5_attitude_scripted`, build dir `build/host-gz-l5`).
- **Frozen suite.** `tests/regression/quad/L05/` holds:
  - `CMakeLists.txt` and `param_ids` (`att_kp`, `att_yaw_weight`, `att_loop_ratio`, `angle_tilt_max`,
    `yaw_deadband`, `att_yaw_alpha_min`, `att_yaw_t_cross`);
  - `unit/{rate_bypass, truth_state, attitude, angle, composition}`;
  - `t3/` (with `reference/`);
  - `tools/`, `gz/` and `controls/`;
  - `results/step_cause/` and `results/step_controls/` (decision 19's evidence);
  - `results/recovery_cause/` (R2's cause, with its bit-exact replay).
- **G3.** `cmake/flight_targets.cmake` and `tools/ci/check_g3.py`, as in B.
- **`ci/run_ci.sh`.**
  - Add `tests/regression/quad/L05/tools` to the pytest list (`:458-459`).
  - Add the step "L5: T3 oracle reproduces attitude_t3_golden.txt" and its perturbed-input control.
  - Add the acro identity golden's reproduce step and its control.
  - Add the two G3 truth-export controls.
  - The frozen C++ suites join by glob (`tests/regression/CMakeLists.txt:7-12`).
- **`ci/run_ci_gz.sh`.** Add `gz_build_l5` and `gz_l5` (`pytest_no_skips tests/regression/quad/L05/gz`) after `gz_l4`.
- **Frozen files.** This design modifies none:
  - the L04 scenario tests accept the new register entries;
  - the L00 `check_g3` tests keep `non_sil_exports` and manifests without `truth_state`;
  - the L01 manifest rule takes `L05/param_ids`.
  The regression-change check will say so if this is wrong.

### H. Lead decisions made during the build

- **Scope of the core rule (`b194ea3`).** Core §7.5's flight-rate-group rule applies to groups "whose rate no product
  requirement sets by its own rule". QF-8 and 0005 owner decision 1 (the rate loop at 3.2 kHz) therefore stand.
- **Tilt-limit check.** The attitude configuration check adds 0 < `angle_tilt_max` ≤ π.
- **Truth quaternion sign.** The truth adapter canonicalises q to w ≥ 0 (core §3).
- **Chirp units.** The attitude chirp is in rad/s at the attitude output.
- **Composition controls.** Controls that need `att_loop_ratio` above 1 run only in the composition tests.
- **Q fixture.** Q is computed on the fixture `attitude_t3_q_inputs.txt`.
- **Step alignment.** The step test compares against the plan's exact stamps (312.5 µs period).
- **Recovery δ.** R1's δ = 0.01 rad, a labelled scenario test value (owner decision 22). The reason is in F.
- **Separatrix field.** The recovery runner fills the L2 separatrix-margin field with the state's own value. The
  analytic separatrix check does not apply to a recovery scenario.
- **Rate channels at execution 0.** The recovery's rate channels skip execution 0, where the plugin has not yet
  applied the initial rates (0003 item 11). This is not a tolerance term.
- **Recovery F.** The recovery envelope borrows the step envelopes' T3 rounding term, tagged INFERRED. It is about
  1e-5, against a halving term of 4.1e-2.

## Spec gaps logged

- Quad §4 L5's pass bar lists no T4 chirp. QF-3 requires verification by chirp. Owner decision 16 adds the T4 attitude
  chirp to the pass bar.
- This record's first chirp amplitude rule (the torque envelope τ_held at the attitude output) made the vehicle tumble.
  Owner decision 20's sweep supersedes it.
- The L5 opening gains a feed-forward input (a world angular velocity: the angle-mode yaw-rate command) and a fault
  output. The setpoint is still a quaternion.
- The L4 opening gains `execute_bypass`, which is additive.
- The L7 opening sends body rates to L5. L5 carries them in `AttitudeState`.
  - The attitude law does not use them: the rate loop takes its rates from L4, and from L6 once it exists.
  - Angle mode uses their world-down component for the heading lock (owner decision 17).
  - From L7, the lock's crossing depends on the estimator's rate. Guard (a) bounds the effect of noise.
- Core §7.5 applied SIM-7 minimisation to every rate group, even where the rate has no material cost. Owner decision
  21 adds the flight-rate-group rule, applied in `b194ea3`: a group runs at its parent's rate unless EMB-3 fails.
- In angle mode the yaw stick is a rate command (owner decision 14) that goes through the L4 bypass (owner
  decision 8). QF-2's first-order reference model therefore does not shape the angle-mode yaw response. QF-2 is
  verified in acro, at L4.
- Betaflight's `angle_limit` applies per Euler axis; MARV's `angle_tilt_max` bounds the total tilt.
- Betaflight's default yaw deadband is 0, so braking toward the heading lock starts only at exactly zero stick.
- Owner decision 17's fallback term ω_release/α_max,yaw is the fastest stop at the worst corner's authority, not the
  longest. Owner decision 18 adds t_cross, the design model's latest linear-regime crossing, and takes the later of
  the two.
- t_cross is defined from a unit-rate steady state. After a short yaw input the rate loop is not at steady state, and
  its crossing can come later; the fallback then locks first, and the envelope covers it.
- T4 angle steps no longer enforce the one-tick delay control. DShot quantisation at hover (Q) exceeds the delay's
  design shift, so the delay metric is enforced at T3 (owner decision 19).
- QF-3 in SIL checks the attitude loop at the nominal plant only. The box is checked at T3 only, as it was for QF-3 at
  L4.
- Quad §4 L5 names a large-angle recovery scenario but gives no pass rule. 0006 defines one: the envelope from the
  exact initial state (R1 from π − δ), plus an α-only side check at exactly π (owner decision 22).
- The L2 scenario had no initial rotor speed, so a recovery started from rotors at rest. Owner decision 23 adds the
  field, with 0007 as its record.

## Carried forward

- **L8.** The G3 guard that only test compositions may carry `TRUTH_STATE` (lead, Q6). Until then, a product SIL
  library is simply one built without the flag.
- **L8.** `yaw_deadband` (0, Betaflight's default) needs revisiting with real sticks.
- **L6.** Evaluate DShot error diffusion, carrying the rounding residual to the next tick, in the actuator-chain work
  (owner decision 19). It would remove the hover limit cycle without noise dither. It changes L3's output, so it needs
  its own decision record.
- **L9.** Re-check EMB-3 for the attitude loop at 3.2 kHz with measured WCET (owner decision 21).
- **L6 and L8.** The L4 T4 acro recovery and the L5 R2 recovery must both pass before L8, and each blocks L6.
- **L6.** Evaluate ω×Jω feed-forward together with the D term, then re-run both known failing items.
- **0007.** `run_l4` passes the rotor speed through, but `l4_scenario` does not accept it yet.

## Why

The L4 handoff (`docs/handoff.md`, 2026-09-30) listed these as the owner's calls:
- how the band box applies to the outer loop;
- SIM-7's quantity and uncertainty;
- the route for truth attitude;
- the prefilter's place in the attitude path;
- yaw in angle mode;
- the tilt limit.

Luis answered them in decisions 1–12. The drafts of this record and the build raised thirteen more, which he answered
in decisions 13–25:
- the yaw-gain compensation;
- the heading hold;
- the recovery scenarios;
- the attitude chirp;
- when the heading locks after release;
- the lock's fallback time;
- DShot quantisation in the T4 step tolerance;
- the chirp amplitude;
- the attitude loop's rate;
- R1's start near the singular set;
- the rotor spin-up in recovery scenarios;
- the T4 tolerance policy;
- the frozen L01 edit.

The lead decided the SIM-7 seed (Q4, now moot under decision 21), the SIL classification (Q6) and the build details
in H.

This record turns those answers into interfaces, a gain rule, a rate rule and tests that assert rules rather than
snapshots (decision 7), so that L6's rate-loop change regenerates L5 without editing a frozen test.

Four findings drove the extra questions:
- **A stick-integrated heading runs away and reverses the yaw.** Decision 14 replaces it with a lock on release.
  Decision 17 moves the lock to the end of the braking, so the heading is not pulled back through the stopping
  distance. Decision 18 makes its fallback the latest linear-regime stop.
- **SIM-7 minimisation had nothing material to save.** It also needed a rate-dependent uncertainty closed by a fixed
  point. Decision 21 runs the loop at its parent's rate unless EMB-3 fails.
- **DShot quantisation at hover sets the T4 step floor.** Decision 19 adds Q to the tolerance and moves the fine
  delay metric to T3.
- **The recovery exposed two harness limits and the known coupling.** The two limits were rotors starting at rest, and
  an exact-180° start whose branch the plant's rounding picks. Decisions 22 and 23 fix the harness rather than adding
  a term (decision 24). R2's remaining exit is gyroscopic coupling, so it joins the L4 item.

## Evidence

- **Design.** The generator report, `marv_params_attitude_report.txt` (generated at build), gives the live values in E.
- **T3.** `tests/regression/quad/L05/t3/` holds the golden, the envelope and the Q table with their fixtures. CI
  reproduces them. The results are in F.
- **T4 steps.** `tests/regression/quad/L05/results/step_cause/` (the cause: DShot quantisation at hover, by bit-exact
  replay and counterfactual) and `results/step_controls/` (the fine controls against the widened predicate).
- **T4 chirp, yaw release and fallback.** The results are in F; the scenarios are in `scenarios/quad/L05/`.
- **Spec amendment.** Commit `b194ea3` (owner decision 21).
- **Recovery.**
  - `tests/regression/quad/L05/results/recovery_cause/` holds R2's cause, from a bit-exact replay of 20747 executions.
  - The scenarios are `scenarios/quad/L05/recover_inverted.yaml`, `recover_inverted_exact.yaml` and
    `recover_tumble.yaml`.
  - The superseded rest-rotor measurements are in commit `7028cf8`.
- **L2 initial rotor speed.** 0007, including the bit-identity proof of owner decision 23.

PENDING:
- the acro bit-identity golden and its command;
- the truth-plumbing results;
- the independent review;
- Full CI both images, run by the lead on clean copies of `quad-l5` at f56c0fb (2026-09-30), with
  `MARV_CI_BASE_REF=master`:
  - `ci/run_ci.sh` in `marv-ci`: exit 0, 49 steps, ALL STEPS PASSED; ctest host-debug and host-release 454/454, frozen
    453/453; regression change check: 1 frozen file changed, covered by 2 decision records (0006, 0007).
  - `ci/run_ci_gz.sh` in `marv-ci-gz`: exit 0, 10 steps; L02 gz and tools green; `gz_l4` 40 passed, 1 xfailed;
    `gz_l5` 63 passed, 1 xfailed (R2); 0 skipped.

## Approval

PENDING (core §7.3).
