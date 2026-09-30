# 0006: quad L5 attitude-loop choices

**DRAFT (architect, 2026-09-30; revised after owner decisions 13–18 and lead decisions Q4 and Q6).** Not yet
approved by Luis. Numbers marked PENDING come from the generator, the T3 oracle or the T4 runs.

## What changed

**Frozen files changed.** None expected. Files L5 adds under `tests/regression/quad/L05/` need no record (core §7.3).
The design below was checked against the frozen files it could touch (G, last bullet). If implementation finds a
frozen edit is needed, it is recorded here with Luis's approval. PENDING.

**Interfaces changed (all additive).**
- L4 opening: `RateLoop::execute_bypass` (owner decision 8). There is no other L4 change (owner decision 14: no new L4
  accessor).
- SIL: a test-only truth-state entry, `marv_truth_state_set`, exported only by SIL libraries built with
  `TRUTH_STATE` (owner decision 11, which overrides decision 4). Product SIL libraries are unchanged.
- G3: the exports check distinguishes product and truth-state SIL libraries, with negative controls.
- Gazebo plugin: an optional `<attitude_source>truth</attitude_source>` element and a log record type 5 (TRUTH).
- A new firmware type, `AttitudeState<T>`, which is the L5 input now and L7's opening later.

**Known failing item.** If the large-angle recovery scenario R2 hits the gyroscopic coupling limit, it extends the L4
item, with the same gating: it blocks L6 and must pass before L8 (owner decision 15). PENDING.

This record holds the owner's choices for quad L5 (quad spec §4 L5) where the spec left a choice, and the lead's
choices that follow from them.

## Owner decisions (Luis, 2026-09-30, verbatim)

1. Band box: rate gains fixed at nominal; at each τ×J corner, close the rate loop on that corner's plant, then check
   attitude-loop margin around it. Worst corner ≥ PM_min.
2. Design model: exact sampled-data closed rate loop (tools/card/rate.py).
3. Loop rate: SIM-7.
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
10. SIM-7 quantity and uncertainty (the option Luis selected, "PM loss vs meas. U"): quantity = the worst-corner
    attitude PM of the gains designed at the rate-loop rate (3.2 kHz), evaluated at each lower candidate rate
    3.2 kHz/2^k; uncertainty = the chirp margin-measurement uncertainty E_H + U_A + U_d, derived as at L4 (0.056° there,
    0005 lines 253-268). No new number.
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
  hold).**
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

### E. Gain rule and design model (`tools/card/attitude.py`; `flatten.py --out-attitude --sim7-u`)

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
- **SIM-7 (owner decisions 3 and 10).**
  1. k_ref is the rule at N = 1 (3.2 kHz).
  2. q(N) = PM_worst(k_ref, N) for N = 2^i.
  3. Δ_i = |q(2^i) − q(2^{i−1})|. N* = 2^{i*}, where i* is the largest i with Δ_j < U for every j ≤ i (N* = 1 if
     Δ_1 ≥ U). The scan stops at the first failure, or where k_ref has no unique crossover or is unstable.
  4. The final gains are the rule at T_a = N*·T. Running k_ref at N* would sit below PM_min, since q(2) < q(1).
  - The report prints the halving table (core §7.5).
- **U and the circularity (lead, Q4, accepted).**
  - **Definition.** U = the minimum over axes of (E_H + U_A + U_d), the chirp rule of 0005 (lines 253-268). The
    minimum, because a smaller U gives the higher, safer rate.
  - **The circularity.** The attitude chirp that measures U runs at N*, and N* depends on U. It is closed as a fixed
    point:
    1. **Seed U⁰.** The L4 rate-loop chirp's U, re-measured with the frozen L4 chirp harness, which does not depend on
       N*. It is committed as a measurement (core §2 rule 3) under `design/measured/sim7_u/`: the command; the inputs
       (scenarios, card hash, commit); the raw per-axis PMs (m = 1 and 2, A and A/2) and U_d terms; and `u.yaml`
       (value in rad, method measured, source). 0005's 0.056° is prose, not a source.
    2. **Iteration.** N*⁰ = SIM-7(U⁰). Build. The L5 T4 attitude chirp at N*⁰ measures U_att by the same rule, and U_att
       replaces the seed as the committed measurement. Regenerate: N*¹ = SIM-7(U_att).
    3. **Termination.** Done when N*¹ = N*⁰. A two-cycle takes the higher rate and is reported.
  - **Frozen check.** The frozen T4 chirp test asserts that SIM-7(U_att measured on the run) equals the live
    `att_loop_ratio`. Its control is a planted U that moves N*.
  - **Location.** `design/measured/` is outside `tests/regression`. L6 can re-measure after its rate-loop change
    without a decision record (owner decision 7), and the frozen fixed-point check catches a stale value.
- **Parameters emitted** (the product set; `tests/regression/quad/L05/param_ids`):
  - `att_kp` (f32, 1/s, derived, σ UNKNOWN);
  - `att_yaw_weight` (f32, unit "1", derived by owner decision 12's rule, σ UNKNOWN);
  - `att_loop_ratio` (i32, unit "1", derived by SIM-7, σ exact);
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
      T_a = N*·T. It starts from the steady state of tracking a unit yaw rate (ω = 1, m = 0, I = 0, e_prev = 0), and the
      reference is zero from the release execution on. This is D's braking: yaw error zero, ω_ff = 0.
    - **Crossing time.** t_c = n·T_a, with n the first attitude execution n ≥ 1 at which ω ≤ 0. The model is linear
      from a unit rate, so t_c does not depend on ω_r.
    - **Value.** t_cross = the maximum of t_c over nominal and the four corners, stored as the smallest f32 at or above
      it (rounded up, so the fallback never fires before a linear-regime crossing).
    - **Refusals.** The generator refuses if some member does not cross within its scan bound (`MAX_STEP_SAMPLES`, the
      method constant `rate.py` uses for t63).
    - It regenerates whenever the rate loop or N* changes (owner decision 18).
    - The firmware validates t_cross > 0 and finite.
  - The firmware also reads `rate_max_*` (the clamp and the yaw-rate command), `rate_loop_divisor` and the tick
    period. The attitude divisor is `rate_loop_divisor` × `att_loop_ratio` ticks.
- **Report** (`marv_params_attitude_report.txt`). It contains:
  - the inputs and the f32 rate gains used;
  - the model statement;
  - per-corner PM, crossover and Jury radius, at N = 1 and at N*, per axis;
  - the SIM-7 table, with U, its source file and the verdicts;
  - w, with the three α;
  - k in f32 and double, and the yaw axis's effective linear gain;
  - `att_yaw_alpha_min`, with its τ_max,yaw and J inputs;
  - t_c per corner, and `att_yaw_t_cross`;
  - δ_num, the bracket, the bisections and the stepped-down flag.
- **Scratch values.** These were computed by the architect with numpy in a session scratch. They are not a source;
  the generator produces the real ones.

  | Quantity | Value |
  | --- | --- |
  | w | 0.14326 (α_yaw 83.33 / α_pitch 581.67 rad/s², from the L4 report) |
  | k_ref (N = 1) | 3.0873 s⁻¹ |
  | q(N) for N = 1, 2, 4, 8 | 45.000°, 44.959°, 44.876°, 44.712° |
  | Δ₁, Δ₂ with U = 0.056° | 0.041° (< U), 0.082° (≥ U), so N* = 2 (1.6 kHz) |
  | k at N = 2 | 3.0855 s⁻¹ |
  | PM at N = 2, every axis | 69.38° nominal; worst 45.000° at (J+, τ+) |
  | Crossover at N = 2, every axis | 3.47–4.61 rad/s |
  | α_min,yaw, and ω_r/α_min at 11.69 rad/s | 83.33 rad/s², 0.140 s |
  | t_c per corner | nominal 0.190 s; (J−, τ−) 0.111 s; (J−, τ+) 0.118 s; (J+, τ−) 0.256 s; (J+, τ+) 0.254 s |
  | t_cross | 0.25625 s at (J+, τ−); a 9×9 grid maximum equals it |
  | Yaw release: travel release→lock, then excursion past the lock | 0.66–1.51 rad, then 0.32–1.07 rad (D) |
  | Fallback case: d = τ_held,yaw ≈ 0.164 N·m from full-stick release | no member crosses before its fallback |

  - Under the compensated law the yaw loop is the tilt loop, up to f32 rounding. The attitude loop's worst corner is
    (J+, τ+); the rate loop's is (J−, τ+). A 5×5 grid's minimum is that corner.
- **Rejected.**
  - loopshape's e^{−sT} model (owner decision 2).
  - Designing at N = 1 and running at N*.
  - Comparing q(N) with q(1) instead of halving.
  - U from 0005's prose.
  - U under `tests/regression`: L6 would need a record to re-measure it.
  - A separate yaw gain w·k (the first draft; owner decision 13).

### F. Tests and controls (core §7.2; owner decisions 7, 14, 15 and 16)

- **Goldens.** There are only two, both on a fixed input the test owns: the T3 step golden and the acro bit-identity
  golden (A). Every other test asserts a rule on the live parameters.
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
  - The SIM-7 table, recomputed, gives the emitted `att_loop_ratio`.
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
  - **Controls.** Attitude gain × 1.1 and one tick of added delay must each leave the tolerance.
  - **Kinematics.** Integrated with sub-steps halved until the trajectory changes by less than the tolerance
    (core §7.5).
- **T3 envelopes** (recorded, used at T4).
  - For each step script, the yaw-release script and each recovery scenario: the box envelope (17×17 grid, halving)
    of the design model, driven by the exact script from the exact initial state and never re-seeded (Luis's L4 rule).
  - **Property.** Each envelope's end value (tilt, heading relative to release, yaw rate) is below the T4 tolerance,
    so a trace inside the envelope has settled. Otherwise the duration doubles (core §7.5).
- **Tolerance terms at T4.** As at L4:
  - E is from 0003 item 9 (m = 1 against m = 2);
  - F = the T3 tolerance + the envelope's last-halving change + the kinematics halving change.
- **T4 angle-mode steps** (gz).
  - **Script.** Full-stick roll and pitch segments, and their release.
  - **Predicate.** At every attitude execution, the tilt components and the heading relative to the lock lie inside
    the envelope ± (E + F).
  - **Other checks.** A clean run and every DShot within [idle, 2047].
  - **Altitude.** Drift is allowed (0005 decision 9). The generated world has no ground plane (`gen_world.py:32`), so
    there is no contact.
  - **Controls.** `att_kp` = 0 through a SIL override (the §7.2 metric control); the step moved one attitude execution
    off its stamp.
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
    - (i) During the sustained segment, the yaw rate lies inside its envelope ± (E + F). This is the angle-mode yaw
      step.
    - (ii) **The yaw rate reaches zero with no reversal beyond the band envelope.** After release, σ_r·ψ̇_m lies inside
      its envelope ± (E + F) at every attitude execution. This includes the envelope's most negative value, which is
      the reversal bound.
    - (iii) **The heading holds after the lock.** For n ≥ n_l, ψ_m(n) − ψ_m(n_l) lies inside its envelope ± (E + F).
      Also, n_l lies within the envelope's range of lock executions.
    - (iv) After release, the heading relative to the release lies inside its envelope ± (E + F).
    - (v) At the end, the yaw rate and the heading relative to the lock are within E + F of zero. By the T3 property,
      the envelope has closed there.
  - **Other checks.** A clean run and the DShot range.
  - **Controls.**
    - A planted trace whose heading goes back to the heading where the yaw input started (the catch-up behaviour
      decision 14 excludes) must fail.
    - `att_kp` = 0 through a SIL override (no pull-back to the lock) must fail.
    - The release moved one attitude execution off its stamp must fail.
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
      inside the envelope ± (E + F). The envelope is the design model with the same script, d and lock logic, never
      re-seeded. At the end both are within E + F of zero: the lock held, and the integrator rejected d.
    - (iii) A clean run and the DShot range.
  - **Controls.**
    - `att_yaw_t_cross` overridden through the SIL to the run duration (a value taken from the scenario, not a new
      number), so the fallback cannot fire within the run: the heading is never pulled back to ψ_m(n_fb), and (ii)
      must fail.
    - A planted trace that crosses zero before n_fb must fail (i).
- **T4 attitude chirp (owner decision 16; gz).** QF-3 and owner decision 10's U both need it.
  - **Injection.** At the attitude output, added to the rate setpoint before `execute_bypass`, one axis at a time
    (roll, pitch and yaw, all at k).
  - **Band.** [min corner ω_c/a, a·max corner ω_c] (0005).
  - **Amplitude.** The largest A for which the design model's peak torque request, over the band and the box, stays
    inside the collective-held envelope τ_held. This is 0005's rule, moved to this injection point.
  - **Duration.** Doubled until |ΔPM| < E_H + U_A.
  - **Estimator and reconstruction.** The DTFT ratio. L_a = C_a·G_m/(1 − C_a·G_m), with the implemented f32 gain.
  - **Terms.** E_H, U_A and U_d as in 0005.
  - **Pass.** PM − U ≥ PM_min, and PM inside the design range. The test also asserts the SIM-7 fixed point (E).
  - **Control.** Attitude gain × c must fail, with c the smallest power of two whose design PM_nom is below
    PM_min − U.
- **T4 large-angle recovery (owner decision 15; gz)**, with labelled scenario test values.
  - **R1: inverted at rest.** q0 = [0, 1, 0, 0], exactly on C's singular set, with zero rates. The setpoint is level,
    locked at D's initial heading. It must pass normally: a single axis, no coupling.
  - **R2: inverted and tumbling.** Body rates (`rate_max_roll`, `rate_max_pitch`, `rate_max_yaw`): the acro limit, and
    the regime of the L4 known failing item.
  - **Predicate.** A clean run, the DShot range, and tilt, heading error and body rates inside the design-model
    envelope from the exact initial state ± (E + F).
    - The design model omits ω×Jω, as at L4, so a coupling limit shows up as an envelope exit.
  - **If R2 fails.**
    1. Run an L4-style bit-exact replay (`tests/regression/quad/L05/replay/`) and write a cause file under `results/`.
    2. If the cause is gyroscopic coupling (no L3 flag, s = t = 1, gyroscopic torque comparable to the rate loop's),
       R2's check becomes a strict xfail (`raises=AssertionError`, an unexpected pass fails). Its condition is the L4
       item's: "`tests/regression/quad/L06/` does not exist". It therefore blocks L6 and must pass before L8. The
       known failing item is extended in this record and in the handoff, and Luis is told. There is no tuning (owner
       decision 5).
    3. Any other cause is an L5 defect.
  - **Controls.** A planted trace held at the initial attitude fails; `att_kp` = 0 fails.
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
- **Generators and registers.** `tools/card/attitude.py`, `tools/card/flatten.py`, `design/scenario_values.yaml`
  (`angle_tilt_max`, `yaw_deadband`) and `design/measured/sim7_u/`.
- **Runner and scenarios.** `tools/sim/l5_scenario.py` and `tools/sim/run_l5.py`. `scenarios/quad/L05/`: `step_roll`,
  `step_pitch`, `yaw_release`, `yaw_fallback`, `chirp_roll`, `chirp_pitch`, `chirp_yaw`, `recover_inverted` and
  `recover_tumble`.
- **Preset.** `CMakePresets.json` gains `host-gz-l5` (MARV_GZ_SIL = `marv_sil_l5_attitude_scripted`, MARV_GZ_PARAMS =
  `marv_params_l5_attitude_scripted`, build dir `build/host-gz-l5`).
- **Frozen suite.** `tests/regression/quad/L05/` holds:
  - `CMakeLists.txt` and `param_ids` (`att_kp`, `att_yaw_weight`, `att_loop_ratio`, `angle_tilt_max`,
    `yaw_deadband`, `att_yaw_alpha_min`, `att_yaw_t_cross`);
  - `unit/{rate_bypass, truth_state, attitude, angle, composition}`;
  - `t3/` (with `reference/`);
  - `tools/`, `gz/` and `controls/`;
  - `replay/` and `results/`, only if R2 needs them.
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

## Spec gaps logged

- Quad §4 L5's pass bar lists no T4 chirp. QF-3 requires verification by chirp, and owner decision 10's U is a chirp
  uncertainty. Owner decision 16 adds the T4 attitude chirp.
- The L5 opening gains a feed-forward input (a world angular velocity: the angle-mode yaw-rate command) and a fault
  output. The setpoint is still a quaternion.
- The L4 opening gains `execute_bypass`, which is additive.
- The L7 opening sends body rates to L5. L5 carries them in `AttitudeState`.
  - The attitude law does not use them: the rate loop takes its rates from L4, and from L6 once it exists.
  - Angle mode uses their world-down component for the heading lock (owner decision 17).
  - From L7, the lock's crossing depends on the estimator's rate. Guard (a) bounds the effect of noise.
- Core §7.5 ("halve until the change is below U") gives no direction for a rate chosen downward from a reference rate.
  Here SIM-7's U also depends on the rate chosen. 0006 defines the direction and closes the circularity by a fixed
  point.
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
- QF-3 in SIL checks the attitude loop at the nominal plant only. The box is checked at T3 only, as it was for QF-3 at
  L4.
- Quad §4 L5 names a large-angle recovery scenario but gives no pass rule. 0006 defines one: the envelope from the
  exact initial state.

## Carried forward

- **L8.** The G3 guard that only test compositions may carry `TRUTH_STATE` (lead, Q6). Until then, a product SIL
  library is simply one built without the flag.
- **L8.** `yaw_deadband` (0, Betaflight's default) needs revisiting with real sticks.
- **L6.** If R2 is an xfail, it joins the L4 known failing item: it blocks L6 and must pass before L8 (owner
  decision 15).

## Why

The L4 handoff (`docs/handoff.md`, 2026-09-30) listed these as the owner's calls:
- how the band box applies to the outer loop;
- SIM-7's quantity and uncertainty;
- the route for truth attitude;
- the prefilter's place in the attitude path;
- yaw in angle mode;
- the tilt limit.

Luis answered them in decisions 1–12. The drafts of this record raised six more, which he answered in
decisions 13–18:
- the yaw-gain compensation;
- the heading hold;
- the recovery scenarios;
- the attitude chirp;
- when the heading locks after release;
- the lock's fallback time.

The lead decided the SIM-7 seed (Q4) and the SIL classification (Q6).

This record turns those answers into interfaces, a gain rule, a rate rule and tests that assert rules rather than
snapshots (decision 7), so that L6's rate-loop change regenerates L5 without editing a frozen test.

Two findings drove the extra questions:
- **A stick-integrated heading runs away and reverses the yaw.** Decision 14 replaces it with a lock on release.
  Decision 17 moves the lock to the end of the braking, so the heading is not pulled back through the stopping
  distance. Decision 18 makes its fallback the latest linear-regime stop.
- **The SIM-7 uncertainty is circular.** It depends on the rate it chooses, so it is closed by a fixed point with a
  committed seed.

## Evidence

PENDING:
- the generator report (w, k, the effective yaw gain, the SIM-7 table, N*);
- the committed U measurements (the seed and U_att) and the fixed-point iterations;
- the acro bit-identity golden and its command;
- the T3 golden, tolerance, controls and envelopes;
- the T4 steps, yaw release, chirp, recovery and truth-plumbing results;
- R2's cause file, if R2 fails;
- the independent review;
- full CI in both images on a clean copy.

## Approval

PENDING (core §7.3).
