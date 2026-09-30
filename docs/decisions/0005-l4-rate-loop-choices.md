# 0005: quad L4 rate-loop choices

## What changed

**Frozen files changed.** Files L4 adds under `tests/regression/quad/L04/` need no record (core §7.3). L4
modifies two frozen L01 files. The two register entries of owner decision 2 broke two frozen L01 tests, which still
asserted the L1 state of the register. Luis approved generalising both to the manifest rule of 0004, and both stay
exact:

- **`tests/regression/quad/L01/tools/test_card_lint.py`**, `test_budget_holds_the_two_recorded_values`.
  - **Unchanged.** The `PM_min` and `chi2_gate_quantile` assertions.
  - **Replaced.** The final "every other entry is UNKNOWN" assertion now requires every other numeric budget entry to
    be listed in exactly one later-layer `tests/regression/quad/Lnn/param_ids`, and every remaining entry to be
    UNKNOWN.
- **`tests/regression/quad/L01/tools/test_flatten_report.py`**, `test_generated_ids_are_the_card_and_budget_ids`.
  The expected id set becomes the L01 manifest ∪ the numeric budget entries claimed by later manifests. It remains an
  exact set equality.
- **Negative controls, added to the same files.** Each of these fails the rule:
  - a numeric budget entry absent from every later manifest;
  - an entry listed in two later manifests;
  - a manifest id that is not produced.
- **Luis, 2026-09-30:** "Option 1, approved, recorded in 0005." He also asked for a list of the other frozen L00–L03
  tests that assert a snapshot of repo state rather than a rule, so that they can be fixed once. That list is
  reported separately and changes nothing in this record.

This record holds the owner's choices for quad L4 (quad spec §4 L4) where the spec left a choice, and the lead's
choices that follow from them.

## Owner decisions (Luis, 2026-09-30, verbatim)

1. **Loop rate.** "Rate loop 3.2 kHz (integer division of the tick) as a labelled scenario value. QF-8 stays open
   until motor τ σ exists. Compute and record the crossover-vs-loop-rate curve anyway. Log a spec gap: QF-8 ignores
   filtering/aliasing; revisit at L6."
2. **Robustness bands (items 2 and 3 of the L3 handoff).** "Card σ for τ and J stay UNKNOWN. Add design-budget
   register entries: tau_robustness_band = ±30% (rationale: Blaha 2024, same vehicle 20 ms bench vs 26 ms flight) and
   inertia_robustness_band = ±50% (rationale: published Crazyflie inertias differ up to ~47%). QF-3 margins must hold
   across both bands. QF-2's α_max uses J at the top of its band (conservative τ_ref). B1 replay narrows both later."
3. **Non-finite values (4a).** "NaN: L4 checks finiteness of gyro, setpoint and its own output. On non-finite: reset
   integrators, output zero torque, raise a fault flag. No L3 change; L3's idle behaviour remains the backstop.
   Planted-NaN test."
4. **Anti-windup (4b).** "Clamping anti-windup: freeze an axis integrator only if its flag is set AND (requested −
   achieved) exceeds the L3 tolerance bound AND the error would drive further into saturation. Test the rounding case
   (flag set, achieved == requested) integrates normally."
5. **Rate law.** The spec says "rate PID", but the toolbox rule (`loopshape.ts`) yields a PI (the symmetric optimum).
   Luis chose "PID structure, kd = 0": "Option 3 (PID structure, kd = 0 derived for now), plus T1 tests of the D path
   with nonzero kd against hand-computed values so it's not only tested at zero. D design (derivative filter rule,
   crossover) moves to L6 with the gyro chain, since its limit is noise. Record L4's ~70 ms τ_ref as the PI baseline
   in 0005, not the target freestyle response."
6. **ω_max.** QF-1's maximum rate is Betaflight's default maximum rate: "Option 1, cited from Betaflight at a pinned
   commit, scenario value."
7. **QF-2 metric.** Luis chose the band envelope. At T3, ω_ref(t) lies inside the envelope of design-model step
   responses over the τ×J band box. At T4, the Gazebo response lies inside that envelope, widened by the L2 T4 rule's
   integration error E. His words: "Option 1, but verify the negative controls (gains × 1.1, one tick of added
   delay) still fail it. If the band envelope is too wide to catch them, golden = nominal-plant response within the
   integration bound, and use the band envelope only for the robustness check."

8. **Gain rule.** Asked to choose between loopshape's fixed structure with a robust margin condition and a MIGO-type
   rule: "Gain rule: Option 1. Record 161 ms (worst corner) as the PI baseline in 0005. Faster response is expected
   from D at L6 and INDI at B3, not from changing this rule."
9. **τ_max envelope.** "τ_max: Option 2, air-mode envelope. The mixer does shift the collective (QF-6, 0004 step c),
   so the collective-held envelope designs yaw to a limit the system doesn't have. Transient thrust change during a
   full-stick step is expected freestyle behaviour. The T4 step tests should allow for the altitude drift, and
   saturation during the transient is covered by the L4 anti-windup tests."

10. **Frozen L01 tests.** "Option 1, approved, recorded in 0005" (see What changed).
11. **Acro combined segment.** The linear box bound fails on pitch by 0.084 rad/s inside the combined full-stick
    segment. Luis chose to apply the linear bound only on the single-axis segments and the reversal. In the combined
    segment the test checks a clean run, the DShot range, and recovery into the envelope after the segment. His words:
    "Option 1. Before recording it in 0005, determine the cause from the trace rather than "likely": check whether L3
    saturation flags were set at the time of the pitch excursion. If yes, record it as documented desaturation. If
    no, record gyroscopic coupling as a known limit of the per-axis PI, to revisit at L6 (D) and B3 (INDI). In the
    combined segment, also check that the saturation flags are consistent with achieved < requested whenever they're
    set, so the segment still verifies something."

12. **Gyroscopic coupling in the combined segment (known failing item, safety).** A bit-exact replay found no
    L3 flag set at any point in the acro run (s = t = 1 throughout), so the cause is gyroscopic coupling. In the
    combined full-stick segment:
    - roll falls from its 11.7 rad/s setpoint to 0.09 rad/s;
    - the integrator winds up without any saturation, so the anti-windup correctly does not freeze it;
    - after the stick centres, roll reaches 10.3 rad/s and is still 0.26 rad/s at 5.0 s.

    Luis rejected re-seeding the recovery envelope from the wound-up state ("that defines the uncommanded roll as
    correct"). His ruling:
    "1. Keep the strict acro recovery check and record it in 0005 as a known failing item (expected failure,
    tracked): cause = PI baseline bandwidth (ω_c 8.3 rad/s) can't reject gyroscopic coupling; integrator absorbs then
    releases it, giving uncommanded roll with the stick centred. Safety item. L4 may be tagged with this item open. L6
    cannot pass until it passes, and it must pass before L8 (pilot in the loop).
    2. Fix the recovery definition bug separately: a vehicle at rest must pass. Define recovery relative to the
    zero-setpoint response, with a negative control.
    3. No firmware change now. At L6, evaluate ω×Jω feed-forward together with the D-term design, and re-run this
    check."
    - **Implementation (lead).** The recovery check is a strict pytest xfail whose condition is "`tests/regression/
      quad/L06/` does not exist". Once L6 work starts it becomes a normal test that must pass. An unexpected pass
      fails CI. The recovery predicate is |ω_a(n)| ≤ Z_a(n) + E + F at every execution of the zero segment after the
      combined segment. Z_a(n) is the peak over the box of |ω| of the linear design model driven by the exact script
      from rest, never re-seeded. A vehicle at rest passes by construction.
    - **Measured (2026-09-30).** Under this rule the recovery fails on all three axes, not only roll:

      | Axis | Peak \|ω\| (rad/s) | Bound (rad/s) | Executions outside, of 3201 |
      | --- | --- | --- | --- |
      | Roll | 9.61 | 1.13 | 1377 |
      | Pitch | 4.75 | 1.92 | 1037 |
      | Yaw | 1.79 | 1.65 | 350 |

      The known failing item covers all three axes.
    - **Controls.** A planted rest trace passes on every axis; a trace stuck at rate_max fails.
    - **Evidence.** `tests/regression/quad/L04/results/acro_cause/` holds the script, the command and `cause.txt`: the
      flags, s, t, requested and achieved torque, and the gyroscopic torque over the excursion, plus the roll collapse.
    - **Other acro checks (pass normally).** (iii) on the linear executions; a clean run; DShot within [48, 1430]. The
      replay reproduces the logged DShot bit for bit at 16001 of 16001 executions (m = 1 and m = 2), with a
      one-ulp control. The fidelity gate is DShot-quantised: torque, flags, s and t come from the replay, whose
      inputs are the logged samples and whose outputs match the log. In the
      combined segment s = t = 1 at every execution. Flag consistency holds vacuously (no flags); a planted-row control
      shows the check is live.

## Lead decisions

- **ω_max source (owner decision 6).** The source is Betaflight tag 4.5.2, commit
  `024f8e13d4e642eb6a380308685b9ea3aa3ef1a2`; the lead checked every line cited here against the raw files at that
  commit on 2026-09-30.
  - `src/main/fc/controlrate_profile.c:48-57`: the default profile is `RATES_TYPE_ACTUAL`, with rcRates 7, rcExpo 0
    and rates 67 on roll, pitch and yaw.
  - `src/main/fc/rc.c:209-219` (`applyActualRates`, selected at `rc.c:803`): at full stick with zero expo, the rate
    is rates × 10 = 670 °/s.
  - The cap `rate_limit` defaults to `CONTROL_RATE_CONFIG_RATE_LIMIT_MAX` = 1998 °/s (`src/main/fc/rc_controls.h:80`,
    applied at `rc.c:663`), so it does not bind.
  - ω_max = 670 °/s = 11.694 rad/s on each axis, a labelled scenario value (pilot preference, QF-1).
  - Not checked: whether 4.5.2 is the newest 4.x tag, and board-specific overrides of the defaults.

- **Gain rule (confirmed by Luis).** The toolbox's symmetric optimum, designed at nominal τ and J, gives 45° at
  nominal. Its worst-case margin over τ ∈ [0.7, 1.3]·τ and J ∈ [0.5, 1.5]·J is about 35°, so it fails owner
  decision 2. The lead's first robust variant optimised kp and ki freely. That degenerates to a P controller
  (ki → 0) and was the source of the "~70 ms" figure; it is rejected. The rule L4 uses:
  - **Structure.** Loopshape's own (`loopshape.ts:37-40, 132-139`). The integral zero is ω_z = ω_c,nom/a, with
    a = spacing(`PM_min`).
  - **Crossover.** ω_c,nom = sup{ω : PM_worst(ω′) ≥ `PM_min` for every ω′ ∈ (0, ω]}. PM_worst is the minimum over
    the τ×J band box, found by a geometric scan and then bisection.
  - **Scaling.** The design is normalised to J = 1 and scaled by the card's nominal J per axis
    (kp_a = J_a·κ_p, ki_a = J_a·κ_i), so the crossover is the same on every axis.
  - **Design model.** The exact sampled-data loop: 1/(Js(1+τs)), ZOH-discretised at the loop period T, with an
    instantaneous truth-gyro sample and no computation, DShot or ESC delay (the plant has none, handoff QF-4 note).
    Loopshape's e^{−sT} is not used: its second half-sample models IMU averaging, which L6 adds as the gyro chain's
    group delay.
  - **Residual.** The omitted DShot frame costs about 0.024° of margin at crossover.
  - **Band box.** The corners suffice. For the continuous model this is proved monotonicity in τ, with a condition
    the generator checks and refuses the card if it fails, and unimodality in J. For the discrete model T3 checks it
    on a grid.
  - **Numerics.** The margin is evaluated on the f32-rounded gains with a numerical guard δ_num.
  - **Rejected.** A MIGO-type rule (maximise ki under the robust constraint), about 10 % faster, because it is a
    different rule from the one the spec names. Luis: "Faster response is expected from D at L6 and INDI at B3, not
    from changing this rule."
  - **Scratch values at 3.2 kHz** (architect; the committed generator produces the real ones): ω_c,nom ≈ 8.3 rad/s,
    nominal PM ≈ 52°, worst case 45.0° at (J low, τ high).
- **Generator numerics (`tools/card/rate.py`).** These are search-method constants, not vehicle numbers.
  - **Scan.** The crossover scan starts at (π/T)·2⁻¹⁷ and steps by 2^(1/16). A card with no feasible crossover at or
    above the start is refused.
  - **Crossover uniqueness.** Checked on a 1024-point log grid below π/T. A tangential double root would escape that
    grid.
  - **Bisection.** It stops when the f32 gains stop changing. The guard then walks back to the last feasible point
    with PM_worst ≥ `PM_min` + δ_num.
  - **τ_ref.** Stored as the smallest f32 at or above max(τ_cl, ω_max/α_max), so the stored value never falls below
    either term.
  - **Result at 3.2 kHz** (generator, 2026-09-30):
    - ω_c,nom = 8.320 rad/s, PM_nom = 52.06°, PM_worst = 45.000001° at (J low, τ high);
    - τ_cl = 0.1609375 s, which binds on every axis;
    - τ_max = 2.443 / 1.832 / 0.537 N·m (roll / pitch / yaw).
- **PI baseline (Luis).** The recorded baseline is τ_cl ≈ 161 ms at the worst band corner, not ~70 ms. It is the PI
  baseline, not the target freestyle response.
- **kd.** `rate_kd_{roll,pitch,yaw}` = 0, method derived (PI rule, no derivative term; D design at L6), σ exact.
- **Gains to firmware.** Continuous SI gains kp, ki, kd and τ_ref per axis, all f32, method derived(rule), σ UNKNOWN,
  written by a generator that flatten runs (the `--out-mixer` precedent). The firmware takes dt from sample stamps
  (core §3) and panics when |dt − T| > 1 µs (the hal_sim truncation bound; L9 replaces it with the ODR error).
- **Scenario values.** A new product-level register, `design/scenario_values.yaml` (method scenario, σ choice), read
  by `flatten.py --scenario`. Its entries: `tick_period_num_us` = 625 and `tick_period_den` = 4, `rate_loop_divisor`
  = 2, and `rate_max_{roll,pitch,yaw}` = 670 °/s.
- **QF-2.**
  - **Achievable time constant.** τ_cl is the maximum over the band box of t63, the first rate-loop sample at which
    the design-model closed loop, without the prefilter, reaches 1 − e⁻¹ of a step. Corners, checked on a grid by
    halving (core §7.5).
  - **τ_max (Luis: air-mode envelope).** The largest single-axis torque that `allocate()` delivers with s = 1 (t = 1
    for yaw), the collective free to shift (QF-6, 0004 step c). Luis: "Transient thrust change during a full-stick
    step is expected freestyle behaviour."
  - **α_max and τ_ref.** α_max,a = τ_max,a / (J_a·(1 + `inertia_robustness_band`)), and
    τ_ref,a = max(τ_cl, ω_max,a/α_max,a). Scratch: yaw's authority term is ≈ 0.14 s, so τ_cl binds on every axis.
  - **Firmware.** A first-order setpoint prefilter (model following), r_n = r_{n−1} + (1 − e^{−dt/τ_ref})(sp − r_{n−1}),
    seeded from the first valid gyro value.
- **QF-2 metric: the fallback applies, by proof.** L depends on the gains only through kp/J and ki/J. Gains × 1.1 is
  therefore exactly J × 1/1.1, which lies inside the ±50 % band, so its response is a member of the envelope and can
  never leave it. Per Luis's instruction, the golden is the nominal-plant response within a float rounding bound, and
  the band envelope is used only for robustness: it is recorded at T3 and is the tolerance band at T4.
- **T3.**
  - **Setup.** The float rate loop runs against the design plant simulated in double at tick resolution.
  - **Golden.** Per-axis step trajectories from an independent double oracle, with a first-order rounding tolerance
    derived from the float law.
  - **Margins.** Computed analytically from the exact discrete L(e^{jωT}) at nominal and at the corners, against
    `PM_min` − δ_num.
  - **Negative controls.** Gains × 1.1 and one added tick, as the spec requires.
  - **Fixed-input golden (lead).** The golden runs the firmware law on the test's own committed input fixture
    (`t3/reference/rate_t3_inputs.txt`, equal to the product values on 2026-09-30), not on the live product
    parameters. A legitimate card or register change therefore does not break it, following Luis's ruling against
    snapshot tests. The live design is covered by rule checks on the live parameters (`L04/tools/test_rate_design.py`),
    and the parameter wiring by `L04/unit/rate/params_test.cpp`. CI regenerates the golden from the fixture and
    compares it byte for byte, with a perturbed-input control.
  - **Result (2026-09-30).**
    - Tolerance: ℓ1 × max ρ = 6.33e-4 rad/s per axis. The observed float error is 9.8e-6 rad/s.
    - Negative controls: gains × 1.1 moves the trajectory 0.369 rad/s (582× the tolerance); one added tick moves it
      4.12e-3 rad/s (6.5×).
    - Band envelope: a 17×17 grid over the box. Its last halving changes it by at most 0.0402 rad/s. Corners alone
      under-cover it by 0.91 rad/s.
  - **T4 envelope widening (lead).** At T4 the envelope widens by E_i (0003's integration error, m = 1 against m = 2)
    plus F. F is the T3 tolerance plus the envelope's last-halving change.
- **PID law.**
  - **Form.** Parallel. The integrator is forward Euler, deferred one execution, so the anti-windup freeze uses the
    allocation of the very request the increment would enlarge. The derivative is taken on the measurement.
  - **Order.** Rate loop, then `allocate`, then `thrust_to_dshot`, then the allocation is recorded back.
    `mix()` discards the achieved torque, so the composition calls the two stages directly (no L3 change).
- **Anti-windup bound (4b).** freeze_a = flag_a ∧ |req_a − ach_a| > b_a ∧ e_a·(req_a − ach_a) > 0, with
  b_a = (γ₄ + ε)·(|B̂||M̂||v|)_a. Here v = [achieved thrust, achieved torque], the vector actually allocated, and γ and
  ε are exactly as in 0004 item 2.
- **Fault extension (4a).** A cleared GyroValid flag is a fault too: an invalid gyro reads exactly 0, not NaN
  (`imu_sample.hpp:30-32`). During a failed execution the output is zero torque. `fault_active` is per execution,
  `fault_latched` is sticky until init, and a fault counter counts failures. Execution resumes from the reset state
  (no failsafe manager exists before B2). The planted-NaN test is T1, because the SIL rejects non-finite samples
  before the composition sees them.
- **Truth gyro.**
  - **Source.** A sim-side `marv::truth` command source fills `marv_imu_meas.gyro_rad_s` with the plant's body rate
    and sets GyroValid, selected by an optional plugin element. Without the element, L2 behaviour and logs are
    unchanged.
  - **Log.** It records the IMU sample actually passed to the SIL.
  - **Frozen test.** The frozen L2 adapter test is untouched.
  - **Label.** L4 T4 runs are labelled truth-fed.
- **T4.**
  - **Composition.** An L4 test composition reads the step schedule, chirp and collective thrust from its own
    scenario-method parameters, so the SIL ABI is not widened.
  - **Chirp.** Injected at the plant input, before `allocate()`. Margins come from the log alone, by indirect
    closed-loop identification.
  - **Tolerances.** The pass tolerances come from 0003's integration error E (m = 1 against m = 2), plus the
    sensitivity to chirp amplitude.
  - **Altitude.** The step tests allow altitude drift (Luis).
  - **Acro predicate.** A clean run, every DShot within [idle, 2047], and body rates bounded by the envelope.
    Saturation during transients is expected (air mode) and is covered by the T1 anti-windup tests.
- **T4 per-axis steps (result, 2026-09-30).**
  - **Runner.** `tools/sim/l4_scenario.py` and `tools/sim/run_l4.py`, with scenarios `scenarios/quad/L04/step_*.yaml`.
  - **Scenario values.** A 1.0 s settle, about 30 motor time constants. A 1 m/s horizontal initial velocity, so every
    read is fresh: 0 stale reads in every run.
  - **Result.** Every axis lies inside the envelope ± (E + F), with F = 0.0409 rad/s and E ≈ 0. With a single-axis
    zero-order-hold torque, m = 1 and m = 2 integrate identically.
  - **Negative controls.** kp = ki = 0 on the stepped axis, the core §7.2 metric control, fails on every axis (worst
    11.96 rad/s outside). Moving the step one execution off its stamp also fails, which guards the alignment exactly.
  - **Time resolution.** The predicate cannot detect an envelope shift smaller than 48 executions (15 ms): F is 98 %
    grid-convergence H, and the response moves at most 0.013 rad/s per execution. The lead therefore dropped a
    one-execution envelope-shift control. It was the lead's own addition, not a spec control.
- **T4 chirp margins (result, 2026-09-30).**
  - **Band.** [2.465, 37.16] rad/s = [min corner ω_c / a, a · max corner ω_c].
  - **Amplitude.** A_a = τ_held,a / max_box |S|. τ_held is the collective-held hover torque envelope, and
    max_box |S| = 1.528 at (J low, τ high). This gives 0.486 / 0.365 / 0.107 N·m (roll / pitch / yaw).
  - **Duration.** By convergence (core §7.5): the start value D₀ = 4 s converged at D = 8 s, with
    |ΔPM| = 0.040° < E_H + U_A. The chirp is followed by a 5 s tail.
  - **Estimator.** A plain DTFT ratio Y/D over the record. That is adequate for a deterministic simulation: on the
    design loop it recovers the known margin to 1e-12 rad.
  - **Reconstruction.** L = C·G_m/(1 − C·G_m), with C the implemented law and its f32 gains.
  - **Terms.**
    - E_H = |PM(m = 1) − PM(m = 2)| ≈ 1e-12°.
    - U_A = |PM(A) − PM(A/2)| = 0.054°.
    - U_d = 0.0013°: the chirp is recomputed in double while the firmware uses float32, propagated to the margin by a
      stated rule.
  - **Result.** Measured PM = 52.154° on every axis (design nominal 52.061°), 7.10° of slack above `PM_min`.
  - **Negative control.** Gains × 4 (design PM 42.50°), measured at 41.98°, fails as required.
- **T4 acro.** The script is seven 0.5 s full-stick segments from 1.0 s: roll +R, roll −R (the reversal), pitch +R,
  yaw +R, zero, roll+pitch+yaw +R (combined), zero.
  - **Bound (iii).** The linear design-model response to the exact script, peaked over the band box, + E + F. It
    replaces "R·(1 + step overshoot)", which did not cover a reversal and passed only by timing.
  - **Scope.** Per owner decision 11, (iii) applies to the single-axis segments and the reversal.
  - **Other checks.** A clean run and every DShot within [idle, 2047].
  - **Negative control.** Segment 1 at 2R breaks (iii).
  - **Cause of the combined-segment excursion.** Determined from a bit-exact replay of the firmware on the logged
    gyro samples (see below).
- **L4 test composition (`fw/compositions/l4_rate_scripted`).**
  - **Parameter set.** `marv_params_l4_rate_scripted` is the product set with the composition's own register appended
    last, so the product ids are a prefix. A ctest checks every product id, type and unit (with a control).
  - **Link order.** `marv_rate` and `marv_mixer` link the product runtime, and the SIL library links the extended one
    first. A mis-order is a loud link error, not a silent table swap.
  - **L9 constraint.** A flight composition uses the product set only. This arrangement is for test compositions.
  - **Writes.** Only on rate-loop ticks. The hal_sim latch holds the command in between.
  - **Validation.** Segment stamps are ≥ 0 and strictly increasing. The chirp fields are checked only when a chirp
    axis is set. Thrust is finite and ≥ 0.
  - **Faults.** During a rate fault the torque request is zero plus the chirp, if one is running.
- **QF-8 curve.** Loop rates 6400/2ⁿ Hz down to the profile's 12.5 Hz minimum. The script, its inputs and the raw
  table are committed under `tests/regression/quad/L04/results/qf8/`. The verdict stays open until motor τ σ exists.

## Spec gaps logged

- QF-8 ignores filtering and aliasing. Revisit at L6 (owner decision 1).
- Quad §4 L4 says "rate PID", but the rule it names (`loopshape.ts`) yields a PI. L4 builds the PID structure with
  kd = 0 (owner decision 5).
- QF-2's "thrust margin × arm" is ambiguous, and "achievable closed-loop time constant" is undefined. Both are
  defined above.
- The L4 opening gains a fault output and an allocation-feedback input, as 0004 item 4 did for L3.
- The L3 handoff said the Gazebo plugin would not change for L4. It gains an optional truth-gyro element.
- QF-3 names no gain margin, so none is checked.
- QF-3 is checked across the register's robustness bands, not "at the card's ±σ" as §4 L4 says, because the card has
  no σ for τ or J (owner decision 2).

## Why

The L3 handoff (`docs/handoff.md`, 2026-09-30) listed several L4 choices as the owner's calls: the loop rate, the σ of
τ and J, and the anti-windup edge cases. While L4 was designed and built, more places turned up where the spec left a
choice or the toolbox rule did not meet the owner's requirement:

- "rate PID" against the toolbox's PI;
- the robust gain rule;
- the QF-2 metric;
- the τ_max envelope;
- the snapshot-style L01 tests;
- the acro combined segment.

Each was put to Luis or decided by the lead as recorded above. The two frozen L01 edits generalise tests that asserted
the L1 register state, so that a legitimate register entry no longer breaks them. Both stay exact, and neither is
loosened.

## Evidence

- **Numbers.** The gain generator's report (`marv_params_rate_report.txt`, generated at build) holds the design
  numbers. `tests/regression/quad/L04/results/qf8/qf8_curve.txt` holds the QF-8 curve (host and image
  byte-identical). `tests/regression/quad/L04/t3/` holds the T3 golden, tolerance and envelope, reproduced by CI.
  `tests/regression/quad/L04/results/acro_cause/cause.txt` holds the combined-segment cause.
- **Frozen L01 change.** Five new negative controls in the two edited files. Each plants a violation, and the
  generalised rule fails on it.
- **Independent review.** Every L4 unit was reviewed against its original acceptance criteria:
  - rate module, truth gyro, scenario register, gain generator, T3 and composition: PASS;
  - T4: see the layer review.
- **CI.** Full CI for both images (`ci/run_ci.sh` in `marv-ci`, `ci/run_ci_gz.sh` in `marv-ci-gz`), run by the lead
  on clean copies before the push. The results are in the push report.

## Approval

Luis's approval of the push (core §7.3): Luis, 2026-09-30, after the L4 report (both CI images green on a clean copy): "Commit, tag, push".
