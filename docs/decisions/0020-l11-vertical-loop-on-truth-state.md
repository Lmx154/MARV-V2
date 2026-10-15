# 0020: quad L11, the vertical loop on truth state (closed: passed on Luis's approval, 2026-10-15; tag `quad-L11-pass` by Luis)

The ACTIVE spine had no layer producing the thrust that L3 consumes (quad §4 L3 opening): L5's collective is the
per-run scenario value `l5_thrust_n` = m·g(h0) (fw/compositions/l5_attitude_scripted/params/l5_attitude_scripted_register.yaml:9-10),
so the hover holds attitude only, every disturbance costs height for good (no drag at v0, sim/plant/include/marv_plant.h:10),
and because the plant evaluates g at the current height (marv_plant.h:83-84) a fixed-thrust hover is an unstable
equilibrium in height. Luis watched the L5 level hold in the Gazebo GUI, asked for a hover that holds, chose to scope
an altitude loop, and ruled on the architect's proposal of 2026-10-13. This record opens the layer; its stages land in
their own commits and are listed under "What changed" as they do.

## Owner decisions (Luis, 2026-10-13, verbatim)

On the demonstration: "is it drifting upwards? should stay hovering no? poke it several times". Offered "Scope an
altitude loop" / "Leave L5 as specified" / "Keep the demo running", he chose "Scope an altitude loop" ("No code yet: I
bring you the spec line, the layer it belongs to, the tests and the number sources as a decision packet; you rule, then
it gets built under the usual CI gates.").

On the proposal, each question with the option he chose (its explanatory text in parentheses):

1. "Add the altitude loop to the ACTIVE spine as a truth-fed, SIL-only layer, per the proposal?" — **"Yes"** ("The spec
   text above enters quad §4 under a decision record; build starts under the usual gates.").
2. "Label for the new layer?" — **"L11"** ("L06–L10 stay fixed (tags, frozen trees, decisions 0009–0019); the §4 text
   sits after L5.").
3. "Build order relative to L6 stage (e)?" — **"In parallel"** ("Needs nothing from L6; one coupling: the FF switch-on
   commit must also keep L11 green.").
4. "The proposed spec text, as written above?" — **"Approve as written"** ("Goes in verbatim with the §5.1 row and the
   quad:365 change.").
5. "`thrust_gain_robustness_band` (loop-gain factor from the thrust coefficient's unknown σ): the generator is blocked on
   it." — **"0.5"** ("Labelled design choice, the class of `inertia_robustness_band` = 0.5; recorded as yours in the
   budget with the date.").
6. "Climb/descent rate clamp inside L11?" — **"None"** ("Only the mixer's thrust limits bound it; a pilot-mode clamp is a
   later §6.1 decision.").
7. "Roll-shove disturbance for T4: the demonstration's 0.02 N·m for 0.2 s as a labelled scenario value?" — **"Yes"**
   ("Rationale recorded: it tilted the card's vehicle 8–9° and was levelled in under a second on 2026-10-13.").
8. "Mark the spec's unsourced \"T/W ≈ 4.5\" (quad §6.3.4 row 1) UNVERIFIED against the card's 6.47?" — **"Yes"** ("Both
   values kept and marked, per the number rule; B1 replay settles it.").

## Why

A layer must exist in quad §4 before anything is built (core §0; CLAUDE.md, the ACTIVE-spec rule), and no ACTIVE text
assigned thrust production to a layer: the only thrust-producing outer loop in the spec was B2's position/velocity
control (quad §5.1 row "Position / velocity control, trajectory generation (B2)"), PARKED. The proposal places the loop
as a new layer, truth-fed first and estimator later, exactly as L4 and L5 were built, so the law is proven now and the
opening a vertical estimator must fill is fixed. Placing it inside L5 would reopen a tagged layer and grow L6 stage
(e)'s Monte Carlo mid-stage; renumbering L06–L10 is irreversible; L8's throttle is the pilot's; B3's "altitude hold on
seed gains" is PARKED and defines no hold. Nothing flies with L11: before it can, a barometer model in the plant, a
vertical estimator layer and a §6.1 pilot mode are each a separate decision.

## What changed

- `docs/spec/10-quad-flight-software.md`: §4 gains "L11 — Vertical loop on truth state" after L5, the approved text
  verbatim; §5.1 gains the row "Vertical loop (spine: truth-fed until a vertical estimator exists) | height reference,
  vertical state → thrust request | the rate loop's rate unless that fails EMB-3, then SIM-7 (core §7.5) | 1"; "When L10
  passes, the spine is done" becomes "When L10 and L11 pass, the spine is done"; §6.3.4 row 1 marks "T/W ≈ 4.5"
  UNVERIFIED with the card's T_max/(m·g) = 6.47 beside it (both kept).
- `docs/handoff.md`: the layer opened, its stages, and the Gazebo GUI findings of 2026-10-13.
- `design/budget.yaml`: unchanged by S0; `thrust_gain_robustness_band` = 0.5 (ruling 5) landed with stage S3 together
  with its `tests/regression/quad/L11/param_ids` claim and the regenerated product parameter table (decision 0004
  manifest rule; the frozen files are named under S3 below).
- No file under `tests/regression/` changes in this commit.
- Stages (one commit each, full `ci/local_ci.sh` per commit; appended as they land): S0 this record and the spec text.
  - S1, the truth vertical channel and its gate: `fw/types/include/marv/types/vertical_state.hpp` (`VerticalState<T>`);
    `fw/sil/include/marv_truth_vertical.h`, `fw/sil/src/truth_vertical.cpp` (`marv_truth_vertical`,
    `marv_truth_vertical_set`, the attitude channel's status codes and valid-bit rule), the export maps
    `fw/sil/marv_sil_truth_vertical.map` and `marv_sil_truth_state_vertical.map`, the `TRUTH_VERTICAL` option of
    `marv_add_sil_library` (fw/sil/CMakeLists.txt) and the manifest flag `truth_vertical` (cmake/flight_targets.cmake);
    `tools/ci/check_g3.py`'s exports rule: allowed exports are marv_sil_*, plus marv_truth_state_set iff truth_state,
    plus marv_truth_vertical_set iff truth_vertical, which a flagged library must export (G3-ERROR otherwise); three
    controls under `tests/regression/quad/L11/controls/` as ci/run_ci.sh steps (unflagged export, planted extra
    export, flagged without the export); T1 tests under `tests/regression/quad/L11/unit/truth_vertical/` (20, frozen,
    with a harness-side stub composition) and `tests/regression/quad/L11/tools/test_check_g3_truth_vertical.py` (38
    cases), the L11 tools directory joining the per-push tools step. L5's SIL library exports are unchanged (the same
    eight symbols). Reviewed PASS against I1 and I2. Noticed, deferred to the stage that first uses it on the target:
    `fw/types/src/instantiate.cpp` does not instantiate `VerticalState<float>` for m33 yet.
  - S2, the law: `fw/vertical` (`VerticalConfig<T>` {kp_z, kp_v, ki_v, period, hover_thrust} with `validate`;
    `VerticalLaw<T>` with `init(cfg, mixer_cfg)`, `execute(state, z_ref) -> {thrust_request, e_z, e_v, fault}`,
    `record_allocation`, `reset`, accessors), the law e_z = z_ref − z, v_sp = k_z e_z, e_v = v_sp − v_z,
    I_n = I_(n−1) + (freeze ? 0 : k_i e_v,(n−1) T), T_req = T_h − (k_v e_v + I_n) (below the reference ⇒ more thrust);
    the law never clamps (the mixer owns the limits); anti-windup = decision 0005 item 4 on the collective: freeze iff
    |T_req − T_ach| exceeds the thrust row of the rate loop's rounding bound (γ_n + ε)(|B̂||M̂||v|)_thrust — B̂'s thrust
    row is 1 per motor (rate::effectiveness), so it is the column sums of |M̂| from the mixer configuration — AND e_v
    drives further into the limit; invalid state (valid bit clear, non-finite z, v_z, z_ref, output or integrator) ⇒
    exactly T_h, integrator and freeze reset, a fault latch cleared by `init` only (the rate and attitude loops'
    pattern). 22 frozen T1 tests under `tests/regression/quad/L11/unit/vertical/` (hand-computed executions, sign,
    both clamp sides, the rounding case, the torque part of the bound, the fault cases incl. overflow, validate,
    reset/init, no-clamp); m33 builds the library. Reviewed PASS against I3–I5 after two corrections (the bound form,
    the overflow tests). Not yet: parameters, composition, generator (S3, S4).
  - S3, the gain generator, its wiring and the T3 design test: `tools/card/vertical.py` (the vertical-velocity PI by
    rate.py's rule with J → m and the height P by attitude.py's sup rule on the exact lifted closed velocity loop; the
    gravity gradient γ = −∂g/∂h in both loops' feasibility; feasibility = PM ≥ PM_min and Ms ≤ Ms_max over nominal and
    the four corners of τ(1 ± tau_robustness_band) × r ∈ {1 − b_k, (1 + b_k)·m_card/m_min}, b_k =
    thrust_gain_robustness_band and m_min the card's open mass conflict; Ms by rate_lead.py's golden-section rule;
    Jury at every corner; the admissible r-interval reported); `tools/card/flatten.py --out-vertical`;
    `design/scenario_values.yaml` gains the design site `site_latitude_rad` = 0.82 rad and `site_height_m` = 500 m as
    labelled scenario values (the rationale of scenarios/quad/L02/hover.yaml; γ varies 0.14 % over all latitudes);
    `design/budget.yaml` gains `thrust_gain_robustness_band` = 0.5 (ruling 5); `fw/params/CMakeLists.txt` runs the
    generator for every parameter set that takes a scenario; `tests/regression/quad/L11/param_ids` claims
    thrust_gain_robustness_band, site_latitude_rad, site_height_m, vert_kp_v, vert_ki_v, vert_kp_z, vert_loop_ratio.
    Frozen files changed by this stage: `tests/regression/quad/L06/product_params/marv_params_product_table.txt`
    (regenerated in the CI image: seven new rows — site_latitude_rad 0.819999993, site_height_m 500, vert_kp_v
    4.78701973, vert_ki_v 13.0025043, vert_kp_z 1.73426414, vert_loop_ratio 1, thrust_gain_robustness_band 0.5 — the
    later ids shifted, count 94 → 101, a new schema hash; no existing value changed) and
    `tests/regression/quad/L06/product_params/README.md` (the regeneration command now runs the image as the user with
    its venv and build under /tmp, so no root-owned file lands in the checkout; ci/run_ci.sh's regen message says the
    same). Nominal result at the committed card and site: k_v = 4.787019729614258 N·s/m, k_i = 13.002504348754883 N/m,
    k_z = 1.7342641353607178 1/s; inner crossover 6.557 rad/s, PM-limited at exactly 45.000° at the (r_lo, τ_hi)
    corner; outer crossover 2.069 rad/s, Ms-limited at 2.000 at the same corner; admissible r 0.5 to 3.221.
    Tests: `tests/regression/quad/L11/tools/test_vertical_generator.py` (15) and `test_vertical_design.py` (14, an
    independent model: PM and Ms over the corners and a halved 17×17 grid, tightness in L05's history-point form,
    Jury, rule 7, and the controls k_i × 1.1, k_z × 1.1, one added tick, loop disabled — unstable with γ at the pole
    e^{√γT}, marginal without — and the inner pair scaled along the design family (1.1 k_v, 1.21 k_i), because k_v × 1.1
    alone adds phase at the binding corner and cannot break the margin: §5's "k_v × 1.1" control is replaced by that
    pair, the spec's "gains × 1.1" unchanged). Both run per push (8.4 s and 15.4 s alone). Reviewed PASS against §4
    and §5.
    Two more frozen files changed by this stage, regenerated by their own generator in the CI image
    (`uv run python tools/sim/l6_ff_eval.py`, the command on each file's second line) because their headers list the
    SHA-256 of `design/budget.yaml` and `design/scenario_values.yaml`, which this stage changed:
    `tests/regression/quad/L06/results/ff_eval/card_worst.txt` and
    `tests/regression/quad/L06/results/ff_eval/sweep.txt`; in each only the two hash lines differ and no data line
    changed (the local CI core job on the first S3 commit failed exactly on the byte-for-byte check of the first).
  - S4, the composition: `fw/compositions/l11_vertical_scripted/` (L5's composition with the vertical loop in place of
    the constant thrust: per tick the attitude group, then the vertical group on every rate tick × `vert_loop_ratio`
    — z_ref from the stamped segment script, `VerticalLaw::execute`, plus the thrust disturbance from its stamp —
    then the rate group with the torque pulse as the added torque and the thrust request, `record_allocation` on the
    request that went to the mixer, `hal_actuators_write`); `VerticalConfig` from vert_kp_z, vert_kp_v, vert_ki_v,
    the period by attitude.cpp's rule and `vert_hover_thrust_n`, validated with hal_panic; the truth vertical state
    latched and checked for freshness as the attitude is (decision 0006 B); the law's fault flag is not acted on (L5
    does not act on the attitude law's either): the fallback T_h is the behaviour. Register (scenario, sigma choice,
    neutral defaults, sil_override): vert_hover_thrust_n (N; the neutral 0 makes init panic, so every run sets it),
    l11_seg_count and l11_seg1..8 {t_us, roll, pitch, yaw, z_ref_m} (z_ref is 0 before the first segment, L5's
    sticks-zero rule), l11_dist_thrust_n/t_us, l11_dist_torque_axis/nm/t_us/dur_us. The SIL library
    `marv_sil_l11_vertical_scripted` (sim/l11_vertical_scripted/, TRUTH_STATE TRUTH_VERTICAL, params
    marv_params_l11_vertical_scripted) exports marv_sil_* plus both setters; L5's library is byte-identical on fresh
    builds; the product table unchanged. Tests `tests/regression/quad/L11/unit/composition/` (37 frozen entries: 34
    gtest incl. parameter validation — non-finite values are refused by the SIL parameter interface before init, the
    truthful treatment — hover equality with the mixer-plus-diffuser reference of {T_h, 0}, sign, script stamps, the
    thrust disturbance, the torque pulse per axis, group order and subset, freshness panics, the fallback, and T11 a
    planted clamp proving the anti-windup wiring; the id-prefix script with two controls). Nine mutations each caught
    after T11. Reviewed PASS against §1, §4, §5.
  - S5, the Gazebo-side truth vertical feed: the plugin's `<vertical_source>truth</vertical_source>` (refused for any
    other value, without `<attitude_source>truth</attitude_source>`, or when the plugin is built without
    MARV_GZ_TRUTH_VERTICAL, derived from the SIL library's MARV_SIL_TRUTH_VERTICAL property as the truth_state define
    is); the adapter `marv::truth::TruthVertical` (sim/gz/adapter), the chain TruthVertical(TruthAttitude(gyro)),
    VALID iff both float casts of the plant's down position and velocity are finite; log record 8 VERTICAL
    {tick, z, vz, flags} directly after the tick's TICK and TRUTH records, only with the element, read by
    tools/sim/lockstep_log.py (must follow its tick's TRUTH record) and refused explicitly by sensor_replay; the run
    report's one conditional line; the preset `host-gz-l11`, the CI group `l11` (builds host-gz-l5 and host-gz-l11)
    and the Actions job `gz-l11`; `ci/local_ci.sh` and CLAUDE.md's job list gain gz-l11. Tests:
    tests/regression/quad/L11/gz (13: the accepted element with one record 8 per tick equal to the float casts of the
    logged body state and the hover DShot at the reference, two planted refusals with good-world controls, and the I1
    guard) and tests/regression/quad/L11/unit/truth_vertical_source (the adapter, 5). I1 evidence:
    tests/regression/quad/L11/i1/l5_step_roll_m2_records.txt records the SHA-256 of the L5 step_roll m = 2 seed-1 log's
    records (36,691,855 B) with the exact command; recorded on the host from the pre-S5 plugin and reproduced in the
    marv-ci-gz image; the L5 log is byte-identical with the plugin built from HEAD before S5 and after it. Reviewed PASS
    against §1 (Truth feed) and I1.
    Amended before its push: the run report's conditional line (tools/sim/run_scenario.py) iterated the L4 runner's
    tick summary, a `range`, and raised in every L04 gz test (the batch run's gz-l4 job caught it); the line now
    checks only dict ticks. Verified: core, gz-l2, gz-l5, gz-l11 on the batch head before the amend and gz-l4 on the
    amended commit; the only difference between the two is that guard.
  - Paused (Luis, 2026-10-14, verbatim: "STOP testing stop creating tests and start consolidating the suite"):
    S6 and S7. S6a-1 (the vertical T3 oracle, `tests/regression/quad/L11/t3/reference/`, its inputs and SHA256SUMS,
    the refdata registry entry) is built and parked uncommitted in the working tree; the C++ T3 test, the
    envelopes, Q and the T4 gz suite are not started. The layer therefore has no T3 golden or T4 test yet and
    cannot be declared passed; the loop itself (S1–S5) flies in SIL: the L11 composition holds height to the
    millimetre under the demonstration's shoves (handoff, "Recovery inspection").
  - S6a-2, the C++ T3 golden of the vertical loop (§5 "T3 golden (core)"): files under `tests/regression/`:
    `tests/regression/quad/L11/t3/t3_test.cpp` (new: the float `VerticalLaw::execute` path, configured from
    `reference/vertical_t3_inputs.txt`, closed with the design plant in double in the oracle's per-tick order, compared
    per execution on t_req, z and v with the golden's recorded tolerance for the height step and the thrust offset; the
    fixture's table keys, every key but `hover_thrust_n` and `thrust_offset_n`, must equal the built defaults of
    `marv_params_l11_vertical_scripted`, with an inline one-ulp control; the controls k_v, k_i, k_z × 1.1 each and one
    added tick must leave the tolerance), `tests/regression/quad/L11/t3/CMakeLists.txt` (new: the executable, the
    `quad/L11/t3` reference set fixture, label L11 through `marv_frozen_suite`) and
    `tests/regression/quad/L11/CMakeLists.txt` (modified: `add_subdirectory(t3)`). Outside `tests/regression/`:
    `ci/run_ci.sh` gains the steps `vert_t3_reference_reproduces` and `vert_t3_reference_control` (vert_kp_v one f32 ulp
    up must fail the SHA256SUMS check naming `vertical_t3_golden.txt`, cached as L5's), and `ci/blast_radius.yaml` maps the
    new files, the oracle and its fixture to L11. No oracle, fixture, SHA256SUMS, `fw/` or generator change.
  - S6b-1, the T3 box envelopes of the two vertical scripts (§5 "T3 envelopes"; the roll shove's coupled envelope and Q
    are later packets): `tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py` gains `Box` (the §4 box, read from
    `design/budget.yaml` and the card by the rule `tools/card/vertical.py` uses: τ(1 ± `tau_robustness_band`), r ∈ [(1 −
    b_k) min R, (1 + b_k) max R] = [0.5, 1.5398936]), the member plant (the golden's `Plant` with the lag τ and the thrust
    deviation about the hover trim scaled by r; at r = 1 the golden's bits; every member starts at the same exact trim, because §4 defines r as a
    loop-gain factor and a trim error is scenario (ii), the thrust offset; lead's ruling, 2026-10-15), the 17 × 17 grid and `Envelope` of L5's
    oracle, and writes `vertical_t3_envelope.txt` beside the golden (channel `z` per script, L5's file format, its
    docstring defines every line; the segment starts at the golden's length and doubles until the end value is below F =
    T3 tolerance + halving change + kinematics change); `tests/regression/quad/L11/t3/reference/SHA256SUMS` gains the
    envelope's line. `vertical_t3_golden.txt` is byte-identical (SHA-256 8b475411…2afdcd62f before and after; run length
    unchanged), `vertical_t3_inputs.txt` is not touched, so the refdata registry entry is unchanged.
  - S6b-2, the roll shove's coupled T3 envelope (§5 "T3 envelopes"; Q is a later packet):
    `tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py` gains the third script `roll_shove` in
    `vertical_t3_envelope.txt` (channel `z`, S6b-1's format, its docstring defines the added lines): `Cascade` (L5's fixtures
    `attitude_t3_inputs.txt` and `attitude_t3_q_inputs.txt`, read from L5's reference directory; every key they share with
    `vertical_t3_inputs.txt` is checked equal, and their rate, attitude and mixer values equal the L11 parameter table's), `CoupledPlant`
    (four rotor speeds with their own first-order lags, f_i = kω_i², the card's rotor geometry on J ω' with no ω×Jω, 0006's
    quaternion kinematics, the vertical channel with T·R33; at level and r = 1 it is the golden's Plant to 2e-17) and
    `shove_member_run` (the composition's per-tick order: L5's `attitude_law` and `AngleMode` and `Quantiser.allocate`
    imported, the chain and rate-law lines those of L5's `Member.attitude_step`, because L5's oracle has no function for them
    apart from that class and its own single-axis plant). Scenario: the roll pulse 0.02 N·m for 0.2 s (Owner decisions 7)
    added to the rate loop's torque request before the mixer from the stamp of vertical execution 1 (312 µs: the start
    convention of the other two scripts, no separate labelled source exists; the response is the same shifted in time because
    the vehicle starts at its trim), the §4 box and grid with J at the card's value and r scaling the vertical thrust deviation
    only. The roll shove has no golden, so the tolerance_z term of its F is 0 (it does not decide the run length: the end value
    3.6e-8 m is below the halving change alone, 1.5e-5 m); its segment started at the golden's length and did not double.
    `tests/regression/quad/L11/t3/reference/SHA256SUMS` carries the new envelope line; `vertical_t3_golden.txt` and the
    `height_step` and `thrust_offset` blocks are byte-identical to S6b-1's, `vertical_t3_inputs.txt` is not touched. The
    oracle now takes 205 s at `--procs 4` (S6b-1: 17 s), 3.4 times `per_push_check_time_max`: the lead's decision.
  - S6b-3, the DShot quantisation term Q of the three T4 scripts (§5 "T4": "Q = the box maximum of quantised − unquantised
    trajectory through the live diffuser"): `tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py` writes
    `vertical_t3_q.txt` beside the golden and the envelope (channel `z`, L5's `attitude_t3_q.txt` line format, so L5's
    `parse_q` in `tests/regression/quad/L05/gz/test_t4_steps.py` reads it; the docstring defines every line) and
    `tests/regression/quad/L11/t3/reference/SHA256SUMS` carries its line. Per script, per member of the 9 × 9 grid of S6b-1's
    `Box`: the maximum over the executions of |z quantised − z unquantised| (the unquantised member is the envelope's, over the
    envelope's own segment length), then L5's selection (`q_script`: the grid maximum where it exceeds the maximum over the
    corners and the box centre, else the corner maximum). The quantised member is L5's `Quantiser` (imported; `Actuator` adds
    the ESC-map speed command and the saturation counters): `mixer::allocate` at the law's collective T_cmd, the diffuser of
    decision 0017 in float32 once per rate execution with the carries 0 at the start, marv_plant's ESC map, then the design
    plant's lag and thrust; at zero torque (height step, thrust offset) every motor is T_cmd / 4, for the roll shove it is
    S6b-2's coupled cascade with the actuator in place of w_c,i = sqrt(f_i / k). One departure from L5: the plant's thrust is the
    fixture's k w², not the card's double k omega² (they differ by 3e-8 relative, a plant gain error that the box factor r
    covers), so Q is the quantisation alone. The oracle refuses when a quantised member leaves [T_min, T_max], the mixer scales
    or clamps an allocation, or a DShot value reaches a bound of [d_lo, 2047] = [48, 2047]; none does. Negative control
    (one member, run in the working tree, not committed): the same quantiser without the diffusion gives Q 6.6e-6 m against
    2.2e-8 m with it over the first 3000 executions. Q (m, used / corners / grid; rule; DShot range over the box):
    height_step 2.620e-7 / 8.161e-8 / 2.620e-7, grid, 321 to 1160; thrust_offset 1.240e-7 / 1.240e-7 / 1.240e-7, corners, 758 to
    776; roll_shove 4.849e-8 / 3.362e-8 / 4.849e-8, grid, 752 to 779. `q_allocation_only` is not written (L5 writes it only
    when the allocation saturates; here that is a refusal). `vertical_t3_golden.txt` and `vertical_t3_envelope.txt` are
    byte-identical (8b475411…2afdcd62f, d253d730…833ea64), `vertical_t3_inputs.txt` is not touched. `--golden-only` writes the
    golden alone; `ci/run_ci.sh` `vert_t3_oracle_run` (used only by `vert_t3_reference_control`) passes it, so the one-ulp
    control pays 4.5 s, not the full generation, and still requires the `vertical_t3_golden.txt: sha256` mismatch line (the
    envelope and Q files are then reported `missing`, a line that does not match that pattern). Full generation at `--procs 4`:
    5 min 38 s (S6b-2: 205 s); `--procs 2` and `--procs 4` give identical files (`--procs 1` would exceed 15 minutes).
  - S7a, the T4 scenario files and runner of the three vertical scripts (§5 "T4"; the T4 tests are S7b):
    `tools/sim/l11_scenario.py` (the L5 schema with the vertical script: `segments` {start_vertical_execution, z_ref_m},
    `end_vertical_execution`, optional `thrust_disturbance` {thrust_n number or derived "mass_conflict", start} and
    `torque_pulse` {axis, torque_nm, duration_s, start}; the fields shared with L5 are validated by L5's own validator),
    `tools/sim/run_l11.py` (the plan on vertical executions, stamp floor(2n·625/4) µs; the run starts at the hover trim
    with the rotors at hover speed and no settle phase, so the run's vertical execution n is the oracle's execution n;
    m = 2 and 1; the L5 runner's log and report shape, record 8 per execution),
    `scenarios/quad/L11/{height_step,thrust_offset,roll_shove}.yaml`: z_ref −1.0 m (the oracle's Z_STEP_M), the
    mass-conflict thrust offset (derived, equal to `vertical_t3_inputs.txt` thrust_offset_n), the roll pulse 0.02 N·m
    for 0.2 s, each starting at vertical execution 1 (312 µs), every run 45482 vertical executions (14.213 s, the
    envelopes' count − 1). One frozen file changed: `tests/regression/quad/L04/tools/test_scenario_schema.py` (L11 joins
    the shared schema test's base, missing-field and unknown-field families; no existing case changes; no L11-only case
    family, core §7.7: the committed L11 files are bound to the oracle's envelope by the S7b T4 tests). No oracle,
    fixture, `fw/` or generator change.
  - S7b, the T4 tests of the vertical loop, their controls and the CI wiring (§5 "T4"): file under `tests/regression/`:
    `tests/regression/quad/L11/gz/test_t4_vertical.py` (new, 9 tests; the form of L5's `test_t4_steps.py`). Per scenario one
    [m = 2, m = 1] sequence (`tools/sim/run_l11.py`), cached per module so the checks and controls share runs; the predicate is
    z at every vertical execution inside the recorded envelope (`vertical_t3_envelope.txt`, channel `z`) ± (E + F + Q), E per
    execution from m = 1 against m = 2, F the header's, Q from `vertical_t3_q.txt`, with a clean run (complete trailer, a TRUTH
    and a valid VERTICAL record per tick, no stale read), DShot in [idle, 2047] (L5's `dshot_idle_bound`) and the last
    execution within E + F + Q of the final z_ref; one binding test (the three scenarios' z_ref, thrust offset, pulse axis,
    N·m, start stamp and duration, step execution, end = count − 1 and hover thrust equal the envelope header and
    `vertical_t3_inputs.txt`, the initial state is the hover trim, with a moved-pulse control). All 9 pass, slack worst
    (negative = inside, the margin to the widened band) height_step −7.749680e-03 at n = 1, thrust_offset −1.563741e-04 at
    n = 45482, roll_shove −1.495459e-05 at n = 3. Controls, tried once each on every scenario (scratch, then committed
    only where they fail): loop disabled (`vert_kp_z`, `vert_kp_v`, `vert_ki_v` = 0 through `sil_override`; no composition
    parameter disables the loop and none was added) fails (ii) (z −25.5 m at the end, 44768 violations) and (iii) (+0.18 m,
    44283), committed; `vert_kp_z` × f32(1.1) fails all three (875, 855, 525 violations), committed; `vert_ki_v` × f32(1.1) fails
    (ii) (949) and (iii) (786) and passes the height step (slack −5.159e-03); not committed (lead, core §7.7: `vert_kp_z` ×
    1.1 already fails every script, so it is each script's fine control) and T3-enforced; `vert_kp_v` × f32(1.1) passes all three (slacks −7.750e-03, −1.445e-04, −1.476e-05), T3-enforced on every script
    (the same reason as S3's replacement of the k_v × 1.1 control: alone it adds phase at the binding corner); the one added
    tick is a firmware delay that the harness cannot inject, not tried at T4 and T3-enforced, as L5's (0006 decision 19/24).
    Outside `tests/regression/`: `ci/run_ci_gz.sh` (group `l11` gains `gz_reference_l11`, `refdata ensure quad/L11/t3`, before
    `gz_l11`; `gz_l11` runs through `pytest_gz_dir`; the file is on the held-back list: 123.6 s alone with 11 tests (before the lead dropped the two `vert_ki_v` controls), over
    `per_push_check_time_max`, so the development check skips it unless the diff touches it or its directory's helpers, and the
    full run runs it) and `ci/blast_radius.yaml` (the `tools/refdata/**` and `tests/regression/quad/L04/t3/reference/**` rules
    gain gz group `l11`, because the new file imports refdata; `tests/regression/quad/L11/gz/**` was already mapped;
    `ci/affected.py --check-manifest` OK). Times: the L11 gz directory 140.7 s serial, 125.5 s under `ci/run_ci_gz.sh l11`
    (4 workers, one file each), each sequence about 12 s; with 9 tests the directory took 100.6 s (22 passed) in the
    development check of 2026-10-15. No oracle, fixture, scenario, runner, `fw/` or generator change.

## Close (S8, 2026-10-15)

Luis approves the close (Approval, below, unchanged) and tags the commit. Counts: `ctest --preset host-debug -N -L L11` lists 89
entries (before and after the consolidation); pytest collection is 52 in `tests/regression/quad/L11/tools` (55 before the
consolidation) and 22 in `tests/regression/quad/L11/gz`. Result: `ci/dev_check.sh` of 2026-10-15 on the S6–S7 tree (`ade460b`),
exit 0, which selected everything (`ci/affected.py`: `ci/blast_radius.yaml`, `ci/run_ci.sh` and `ci/run_ci_gz.sh` differ from
`master`): ctest 749/749 in host-debug and host-release, tools tests 1107 passed, gz l11 22 passed in 100.6 s, gz l4 and gz l5
ALL STEPS PASSED. The merge gate (`ci/local_ci.sh`) has not run on this head.

### Pass bar to tests (quad §4 L11 "Pass bar" and §5 above)

| Pass-bar line | Tests (count) | CI step | Result |
| --- | --- | --- | --- |
| T1 the law, the anti-windup, the invalid-state fallback | `tests/regression/quad/L11/unit/vertical/vertical_test.cpp` (22 ctest: L11Vertical 15, L11VerticalConfig 2, L11VerticalInvalid 5); `tests/regression/quad/L11/unit/composition/composition_test.cpp` (34: fallback, T11 anti-windup, order, freshness, script, hover, sign, disturbance, pulse, parameters) | `host_preset` (debug, release); `ci/smoke.sh:26` runs the composition's 37 entries | pass |
| T1 contract tests on the truth channel | `tests/regression/quad/L11/unit/truth_vertical/truth_vertical_test.cpp` (20, with `tests/regression/quad/common/truth_channel_t1.hpp`); `tests/regression/quad/L11/unit/truth_vertical_source/truth_vertical_source_test.cpp` (5); gz: `test_vertical_feed.py` (7), `test_vertical_refusals.py` (4), `test_i1_l5_log.py` (2) | `host_preset`; `gz_l11` (`ci/run_ci_gz.sh:231`) | pass |
| T1 G3 controls for the export | `tests/regression/quad/L11/controls/g3_truth_vertical_{unflagged,planted,missing}.cpp`; `tests/regression/quad/L11/tools/test_check_g3_truth_vertical.py` (26) | `g3_truth_vertical_unflagged_control`, `_planted_control`, `_missing_control` (`ci/run_ci.sh:1026-1028`); `tools_tests` | pass |
| T1 composition order, stale-latch panic, id prefix | `composition_test.cpp` (L11CompositionGroups 2, L11CompositionFreshness 5); `l11_composition_param_ids` and its 2 controls (`tools/ci/check_param_ids.py`) | `host_preset` | pass |
| T3 QF-3 (`PM_min`, `Ms_max`) over the box with the gravity gradient | `tests/regression/quad/L11/tools/test_vertical_design.py` (14: D1 margins, D2 17×17 grid, D3 tightness, D4 Jury, D5 rule 7) and `test_vertical_generator.py` (12) | `tools_tests` (`ci/run_ci.sh:968`) | pass |
| T3 controls: gains × 1.1, one added tick, loop disabled | `test_vertical_design.py` C1 (the pair 1.1 k_v, 1.21 k_i), C2/C3 (k_i, k_z × 1.1), C4 (one tick), C5 (loop disabled, Jury and the pole e^{√γT}); `test_vertical_generator.py` (loop disabled, sign flip) | `tools_tests` | pass |
| T3 height step and thrust offset against the design model | `tests/regression/quad/L11/t3/t3_test.cpp` (L11T3Reference 1, L11T3Step 3 incl. the k_v, k_i, k_z × 1.1 and one-tick controls); the oracle and its inputs reproduced from `tests/regression/quad/L11/t3/reference/SHA256SUMS` | `host_preset`; `vert_t3_reference_reproduces` (`ci/run_ci.sh:967`) and `vert_t3_reference_control` (`:1021`) | pass |
| T4 height step, thrust offset, roll shove inside the envelope ± (E + F + Q), return to the reference | `tests/regression/quad/L11/gz/test_t4_vertical.py` (9: 3 predicates, 1 binding test, loop disabled on (ii) and (iii), `vert_kp_z` × 1.1 on all three) | `gz_l11` (held back from the development check by `ci/run_ci_gz.sh:102`; it ran in this check because the branch adds it) | pass (slack to the widened band, S7b: height_step −7.7e-3 m, thrust_offset −1.6e-4 m, roll_shove −1.5e-5 m) |
| The loop disabled fails | `test_t4_vertical.py` `test_control_loop_disabled_fails_the_predicate[thrust_offset, roll_shove]` | `gz_l11` | pass (fails as required) |
| The L4 and L5 suites still green | ctest L04, L05 (in the 749); gz l4 and gz l5 groups | `host_preset`; `gz_l4`, `gz_l5` | pass (ALL STEPS PASSED) |

### The band condition (§4 "thrust_gain_robustness_band")

`thrust_gain_robustness_band` = 0.5 (`design/budget.yaml:114-115`) lies inside the admissible r interval
[0.5, 3.2209806638985086] of `build/host-debug/generated/marv_params/flatten/marv_params_vertical_report.txt:56-60`: lowest r 0.5
(the next grid point 0.47880164034928685 fails), highest r 3.2209806638985086 (3.363585661014858 fails), r = 1 passes, "r box
inside True" for the box r in [0.5, 1.5398936]. PM_worst is 45.000003° (f32 gains; 45.000001° in double) against PM_min 45° at the
corner (r_lo, τ_hi): the gains are sized to the box by the rule, so the low edge of the interval is the design point, not
slack.

### Deviations from the literal pass-bar text

- T3 control "k_v × 1.1" is the pair (1.1 k_v, 1.21 k_i): k_v alone adds phase at the binding corner and cannot break the margin
  (S3; `test_vertical_design.py` C1 prints k_v alone, not asserted). k_i and k_z × 1.1, and the unchanged "gains × 1.1" of §4,
  are controls as written.
- T4: `vert_kp_v` × 1.1 passes all three scripts (same reason) and the one added tick is a firmware delay the harness cannot
  inject: both are T3-enforced (S7b; L5's precedent, decision 0006 items 19 and 24). `vert_ki_v` × 1.1 fails (ii) and (iii) but
  is not committed, because `vert_kp_z` × 1.1 already fails every script (S7b, §7.7).
- `test_t4_vertical.py` is held back from the development check (S7b; `ci/run_ci_gz.sh:97-103`, 123.6 s alone before two controls
  were dropped, over `per_push_check_time_max`); the full run always runs it.

### Open items carried past the close

- Card mass 0.772 kg against 0.752 kg, UNVERIFIED, two sources: owner B1 (log replay). The band's lower end `m_min` and the
  thrust-offset scenario (ii) both use it.
- `thrust_coeff` σ UNKNOWN and INFERRED, and the σ of the three derived gains UNKNOWN: owner B1.
- T/W ≈ 4.5 (quad §6.3.4 row 1, unsourced) against the card's T_max/(m·g) = 6.47, both kept, UNVERIFIED: owner B1.
- EMB-3 cost of the vertical group ("a few tens of flops", the lead's estimate): owner L9, which re-checks with measured WCET.
- Coupling with the ω×Jω feed-forward, INFERRED until L11 runs with FF on: owner L6 stage (e), whose switch-on commit must keep
  L11 green.
- `VerticalState<float>` m33 instantiation: `fw/types/src/instantiate.cpp:11-12` instantiates `ActuatorOutput` and
  `AttitudeState<float>` only, there is no `VerticalState` line. The m33 build does not need one: `fw/vertical` and
  `fw/compositions/l11_vertical_scripted` are in the m33 build (`CMakeLists.txt:37, 44`; `build/m33/fw/vertical/libmarv_vertical.a`
  and `build/m33/fw/compositions/l11_vertical_scripted/libmarv_composition_l11_vertical_scripted.a` exist), `VerticalState` is a
  plain template with no member function (`fw/types/include/marv/types/vertical_state.hpp:12-18`) and its `static_assert` (`:20`)
  instantiates it in every translation unit that includes the header, and `fw/vertical/src/vertical.cpp:6` instantiates
  `VerticalLaw<float>`. The explicit line (L5's pattern) stays deferred, as S1 noted, to the stage that first uses it on the target; no
  ACTIVE layer does yet (L11 is SIL-only and nothing flies with it, §6).

### Consolidation (core §7.7 item 4)

Nothing is loosened: every removed test is listed with a test that still fails on the same planted break, in every context the
removed test ran in. All five candidates were evaluated; the two files changed under `tests/regression/` are
`tests/regression/quad/L11/tools/test_vertical_generator.py` and `tests/regression/quad/L11/tools/test_vertical_design.py`, plus
the helper `tests/regression/quad/L11/gz/l11_gz.py`.

| Candidate | Verdict |
| --- | --- |
| (a) `tests/regression/quad/L11/gz/l11_gz.py` duplicated `build_parameters` and `add_vertical` of `tools/sim/run_l11.py` | Merged. `l11_gz.py` takes `add_vertical` and `VERTICAL_ELEMENT` from `run_l11` and builds its parameter table from `run_l11.build_parameters` (the L11 build) or `run_l5.build_parameters` (the L5 build) plus `read_param_defaults`; its two copies are deleted (−8 lines net). No test changes behaviour: the five worlds (`world()` for L11, hover-scaled, without the attitude source, L5 without and with the vertical element) and both parameter tables are byte-identical before and after. |
| (b) `gz/test_vertical_feed.py`, `gz/test_vertical_refusals.py` | Kept (owner/lead ruling): they run in the development check, `test_t4_vertical.py` (which subsumes part of the feed check) does not. |
| (c) `unit/truth_vertical_source/truth_vertical_source_test.cpp` against `gz/test_vertical_feed.py` | Kept. The unit test holds assertions the Gazebo run cannot produce: a non-finite body clears the valid bit and hands over an invalid state (2 tests), a set with the wrong tick is refused and the SIL does not run, the held state across 3 ticks with its stamp and hash, bitwise casts of −0.0 and 1e-30. The feed test covers what the unit test cannot: the plugin element, record 8 beside TRUTH, the real plant's states. Different levels, and the unit test runs in the development check for an adapter change without a Gazebo build. |
| (d) `test_vertical_generator.py` against `test_vertical_design.py` | Three tests removed, one trimmed, see below. |
| (e) `unit/composition/composition_test.cpp` against `ci/smoke.sh` and `unit/vertical/vertical_test.cpp` | Kept. The smoke is not a copy of the composition's entries: `ci/smoke.sh:26` runs them (`-R '^(L11Composition|l11_composition_param_ids)'`, 37 entries), so removing them removes the smoke (core §7.6 item 1). Against `vertical_test.cpp` they overlap only in sign and the freeze (T11); the law tests cover the rounding bound, the torque part, the fault cases and the configuration refusals, the composition tests the wiring (script stamps, group order, freshness, disturbance, pulse, parameter interface), and neither catches the other's breaks. |

Removed from `test_vertical_generator.py` (15 → 12 collected items; the directory 55 → 52):

| Removed | Still covers it | Planted break (tools/card/vertical.py in a scratch copy of the tree, never in the repository) |
| --- | --- | --- |
| `test_outer_loop_every_corner_meets_pm_min_minus_delta_num_ms_max_and_jury_on_the_f32_gain` | `test_vertical_design.py` D1 (independent model PM and Ms at the five corners, each equal to the generator's row within 2^-30) and D4 (independent Jury radius equal to the row within 2^-16) | outer row PM −1e-3 rad: D1 fails (the removed test did not); Ms × 1.01: D1; Jury radius 2.0: D4 |
| the PM and Ms half of `test_inner_loop_every_corner_meets_pm_min_minus_delta_num_and_ms_max_on_the_f32_gains`; the test is kept as `test_the_inner_loop_is_jury_stable_and_rule_7_holds_at_every_corner` with its Jury (`rad < 1`) and rule 7 assertions | D1 | inner row PM −1e-3 rad: D1 and D3 fail; Ms × 1.01: D1. Inner Jury radius 2.0 is caught only by the kept test (in HEAD, only by the old one) |
| `test_the_height_gain_is_tight_against_its_bracket_and_every_later_history_point_fails_the_guard` | `test_vertical_design.py` D3 height (independent feasibility and guard); it gained the assertions `bracket_width == hi - lo` and `stepped_down == (steps > 0)`, the only two the removed test held that D3 did not | emitted k_z 1% low: D3 and D1 fail; outer guard dropped: D1, D2, D3, C1–C4; `stepped_down` inverted: D3 only (absorbed assertion); recorded `bracket_width` doubled: D3 |
| `test_the_velocity_gain_pair_is_tight_against_its_bracket_and_every_later_history_point_fails_the_guard` | D3 velocity, with the same two absorbed assertions | emitted k_v 1% low: D1, D2, D3, C1–C4; inner guard dropped: D3; `stepped_down` inverted: D3 only |

Contexts: both files are in `tests/regression/quad/L11/tools`, run together by `tools_tests` (`ci/run_ci.sh:907`) in the development
check (rule `tests/regression/quad/L11/tools/**`, `ci/blast_radius.yaml:909-913`) and in the full run; neither is in
`nightly_tools_tests` (`ci/run_ci.sh:875-882`) and nothing else invokes either file alone, so the covering tests ran in every
context the removed ones did. Kept after the same probe, because only they catch the break: the loop-disabled test of the
generator (`VerticalLoop.stable` always true, the generator's own predicate: only that test fails; D/C5 use their own model), the
sign-flip and convergence tests, the three refusals, `flatten --out-vertical` and the ids test. Development check on the
consolidated tree (`ci/dev_check.sh`, 27 min 43 s): exit 0, 81 steps PASS and none FAIL, ctest 749/749 in host-debug and
host-release (L11: 89 before and after), tools tests 1104 (1107 before, minus the 3 removed), gz l11 22 passed in 101.4 s, every gz
group ALL STEPS PASSED. Time: the directory is 20.8 s
(55 tests) before and 21.2 s (52) after, since the cost is the two fixtures; the saving is lines (−105) and maintenance, not seconds.

### What remains for the layer to pass (core §7.2)

Luis's approval of this close (Approval, below); the full local merge gate `ci/local_ci.sh` on the head; the merge into `master`;
Actions green on it; the commit tagged `quad-L11-pass` by Luis.

## Evidence

- The demonstration of 2026-10-13 on Luis's host (the demo worlds, builder, shim and raw pose traces are in
  `~/marv-handover-2026-10-13/`, outside the repository; they are not design inputs): the L5 level hold at the balance
  height drifts −0.019 mm/s over 19 s with tilt 0.00°; a 0.02 N·m roll torque for 0.2 s (gz ApplyLinkWrench) tilts the
  vehicle 8.2–8.9° and it is level (< 0.5°) again after 0.7–0.95 s, leaving about 0.1 m/s of descent each time; after six
  shoves z = −3.5 m at −0.28 m/s. Started 1.5 m above the balance height and left for half an hour, the hold climbed
  at +0.63 mm/s² at z ≈ 205 m, the free-air gravity gradient (3.086 × 10⁻⁶ s⁻² × 205 m), time constant 569 s.
- The source of the gradient: `sim/plant/include/marv_plant.h:81-84` (g at h = site_height_m − pos_ned_m[2]); the
  constant thrust: `l5_attitude_scripted.cpp:178-181, 212-213`; the thrust slot: `rate_group.hpp:69-72`; the mixer's
  collective clamp: `mixer.hpp:136-145`; the truth channel and G3: `fw/sil/include/marv_truth.h:14-28`,
  `tools/ci/check_g3.py:19-22`.
- Derived values checked by the lead from the card (vehicles/uzh_neurobem_5in.yaml): T_max = 4·k·ω_max² = 49.00 N,
  T_min = 4·k·ω_min² = 0.141 N, T_max/(m·g) = 6.47, the mass-conflict offset (0.772 − 0.752) kg · g(φ, h0) = 0.19613 N.
- The evidence survey and the architect's full decision are in `~/marv-handover-2026-10-13/altitude/`; the parts Luis
  ruled on are reproduced below.

## Approval

Luis's rulings of 2026-10-13, quoted above in full, and his push of this commit (core §7.3).

The close (S8): Luis, 2026-10-15, verbatim, on the lead's list "1. Your approval to declare L11 passed. 2. The full
local merge gate (`ci/local_ci.sh`, all five jobs). 3. The merge into `master` and a green Actions run. 4. Your
`quad-L11-pass` tag.": "go ahead with 1,2,3". The approval holds on the merge gate passing on the closing commit; the
layer is passed (core §7.2) when that commit is green on `master` and tagged by Luis.

## The proposal as ruled (architect, 2026-10-13; label L11, band 0.5, no clamp, shove 0.02 N·m for 0.2 s)

### 1. Decision

**Placement.** A new spine layer, **"Vertical loop on truth state"**. It needs L3, L4 and L5 and nothing from L6, so it
is built after L5 in dependency order and **in parallel with L6 stage (e)**, outside (e)'s Monte Carlo.
- **Label: Luis decides; I recommend `L11`.** The directory must match `L[0-9][0-9]` (cmake/flight_targets.cmake:68 keeps
  controls out of the flight manifest that way; `L5b` would put its G3 controls in the flight set). Renumbering L06–L10
  is irreversible (tags quad-L0..L5-pass, the frozen L06 tree, decisions 0009–0019). The §4 text sits after L5.

**Law.** The vertical axis becomes a fourth axis of today's cascade: thrust↔torque, m↔J, v_z↔ω, z↔θ.
- Inner: a vertical-velocity PI (kd = 0, forward-Euler integrator), L4's law and rule (tools/card/rate.py:1-49). Outer: a
  height P on the exact closed velocity loop, L5's rule (0006 E, 0006:557-564).
- NED, z down:

      e_z = z_ref - z;  v_sp = k_z e_z;  e_v = v_sp - v_z;  I += k_i T e_v(prev);  T_req = T_h - (k_v e_v + I)

  k_v and k_i are in N (mass-normalised gains × card mass, as rate gains are J × κ). T_h is the hover feed-forward (§4).
- **Where T_req goes.** Into the existing `thrust` argument of `RateGroupStep::execute_bypass`
  (fw/rate_group/include/marv/rate_group/rate_group.hpp:69-72; rate_group.cpp:103-116), the slot `l5_thrust_n` fills
  with a constant today (fw/compositions/l5_attitude_scripted/src/l5_attitude_scripted.cpp:178-181, 212-213).
- **Mixer: unchanged.** The collective keeps the lowest priority and is clamped after torque (fw/mixer/include/marv/mixer/
  mixer.hpp:4-6, 214-221). **The mixer alone owns the thrust limits**; the law never clamps around it.
- **Anti-windup.** 0005 item 4 (0005:41) transposed: freeze I only if achieved ≠ requested collective beyond the L3
  bound AND e_v drives further into the limit. Signal: `alloc.achieved_thrust` vs the request (mixer.hpp:61, 234),
  recorded after each rate execution as `rate.record_allocation` is.
- **Invalid state** (valid bit clear, or z or v_z not finite): output T_h, reset I, raise a fault flag. That is exactly
  today's L5 behaviour (the 0005 item 3 pattern).
- **No tilt compensation (1/cos θ) and no climb/descent clamp** in this layer (§2 rejected; the clamp is a question to Luis).

**Rate group.**
- `vert_loop_ratio` = 1 (the rate loop's 3.2 kHz) by core §7.5's parent-rate rule (core:439-442), as 0006 E ruled for
  attitude (0006:570-577). Order per tick: attitude, vertical, rate; vertical ticks ⊆ rate ticks; zero computation delay,
  so the lifted model at N = 1 is exact. EMB-3: a few tens of flops on §5.4's estimates (lead's estimate; L9 re-checks).

**Truth feed (item c): a new struct and a new export.** `marv_truth_state` is not extended.
- **Struct and build.** `fw/sil/include/marv_truth_vertical.h`: `marv_truth_vertical {u32 struct_size; u32 flags
  (VERTICAL_VALID); u64 tick; f32 z_ned_m; f32 vz_ned_m_s}` and `marv_truth_vertical_set` (truth_state.cpp:57-84's status
  codes and valid-bit rule), in `fw/sil/src/truth_vertical.cpp` via a new `marv_add_sil_library(... TRUTH_VERTICAL)`
  (fw/sil/CMakeLists.txt:13-18, 48-50) setting `MARV_SIL_TRUTH_VERTICAL` and a manifest flag (flight_targets.cmake:102-109).
- **G3 rule** (tools/ci/check_g3.py:19-22, 50, 321-331), extended additively: a `truth_vertical` library exports within
  `marv_sil_*` ∪ {`marv_truth_state_set` if truth_state} ∪ {`marv_truth_vertical_set`}, and must export the last.
- **Hook and opening.** `marv::composition::vertical_input(const VerticalState<float>&)`, only in the new composition's
  composition.hpp (l5 composition.hpp:18 pattern); `fw/types/include/marv/types/vertical_state.hpp` `{t_us, z_ned_m,
  vz_ned_m_s, valid}` is the opening a vertical estimator fills later, as AttitudeState is L7's (0006:224-230).
- **Plugin.** `<vertical_source>truth</vertical_source>`, refused unless `<attitude_source>truth</attitude_source>` is
  present and the library has `MARV_SIL_TRUTH_VERTICAL` (pattern sim/gz/plugin/src/lockstep.cpp:424-449).
- **Adapter and log.** `marv::truth::TruthVertical` wraps the source chain (truth_attitude.hpp:22-43 pattern); record
  type 8 only with the element (reader tools/sim/lockstep_log.py:50; sensor_replay.cpp:365 refuses it explicitly).

### 2. Rationale, invariants, rejected alternatives

**Why a spine layer.** No ACTIVE text produces L3's thrust input (quad:209); the only thrust-producing outer loop is B2's
"Position / velocity control" (quad:383, PARKED). Building needs a §4 entry (core:31-33; CLAUDE.md). L4/L5's pattern,
truth-fed first and estimator later, proves the law now and fixes the opening the estimator must fill.

**Why the cascade.** Velocity is the plant 1/(m s (1+τ_m s)), the rate loop's 1/(J s (1+τ s)) form (rate.py:17-19). So
rate.py's ZOH-exact model, PI form, sup rule and corner machinery apply with J → m, and attitude.py's lifted P rule
applies to height. No new rule family enters the repo.

**Invariants, each with its guard**

| Id | Invariant | Guard |
| --- | --- | --- |
| I1 | L5 byte-identical: composition, register (incl. `l5_thrust_n`), SIL library and exports, plugin path without the element, logs; truth struct stays 48 B (lockstep_log.py:18-19, 50) | Frozen L05 suite green; G3 export check on the L5 lib unchanged; new per-push check: an L5 log has no type-8 record and its raw records equal the pre-change hash on one scenario (0006 B rule, lockstep_log.hpp:37-40) |
| I2 | G3: the vertical composition (a flight target under fw/) holds no truth symbol; truth enters only via `marv_truth_vertical_set` | check_g3 on every build + three controls in `L11/controls` (0006:262-266): unflagged lib exporting the setter → G3-EXPORT; flagged lib + planted extra export → G3-EXPORT; flagged manifest without the export → G3-ERROR |
| I3 | Number rule: gains derived by rule, f32, no literal in `fw/vertical` (G1, G2) | G1 lint; a T3 test recomputes each gain from live inputs; the generator refuses infeasible inputs (rate.py GenError pattern) |
| I4 | Thrust limits only in the mixer; no integrator windup against them | L03 suite untouched; T1: freeze iff (clamped AND driving further); rounding case (achieved = requested) integrates; planted saturation |
| I5 | Invalid state → exactly T_h, I reset, fault flag | T1: planted NaN, valid bit clear; the composition panics on a stale latch (0006 B freshness) |
| I6 | Closed loop strictly stable with the gravity gradient in the model, at every corner | T3 Jury; control: loop disabled (all three gains 0) must fail Jury (pole at e^{+√γ T}) |
| I7 | Runs labelled "truth-fed (gyro, attitude, vertical), perfect-model"; never validation runs (core:351-353; 0006:267-268) | Runner label + G3 |

**Rejected**
- **Inside L5 as a second opening.** Reopens a tagged layer and silently grows L6 (e)'s Monte Carlo mid-stage ((e)
  covers "each L4 and L5 scenario", quad:294). **Renumbering to sit between L5 and L6:** irreversible (above).
- **Part of L8 (B).** Throttle is the pilot's (quad:324-325); L8's flight composition cannot carry a truth feed under G3
  (0006:260-261). **B3 S3 forward (C):** PARKED; presupposes baro and identification (quad:649), defines no hold.
- **Leave as is (D).** Luis rejected it, and the drift has no test that could fail.
- **Extending `marv_truth_state`.** Changes its size, so the L5 log bytes (lockstep_log.py:50), sensor_replay's skip
  (sensor_replay.cpp:336) and the frozen fixtures (L05 truth_state_test.cpp:237, 252); it also forces `vertical_input`
  into the L5 composition (truth_state.cpp:82 links the hook). Breaks I1.
- **A single height PID.** No gain rule for it exists in the repo; the cascade reuses two committed rules.
- **Tilt compensation T/cos θ.** Needs an inverted-flight rule; ArduPilot's floor and inverted factor would be uncited
  constants, and the QA-1 θ_max floor (quad:519) is B2's. Feedback rejects the tilt loss as a disturbance, and the
  roll-shove T4 test measures it. Revisit with the pilot mode.
- **3-D position/velocity opening.** The spine has no horizontal estimator (quad:382); only z and v_z are honest.

### 4. Register entries and inputs

**Product parameters** (`tests/regression/quad/L11/param_ids`; f32 unless marked, method derived(<rule>), σ UNKNOWN):
`vert_kp_v` (N·s/m) and `vert_ki_v` (N/m) by rate.py's rule with J → card mass and the gain corners below; `vert_kp_z`
(1/s) by attitude.py's sup rule on the lifted closed velocity loop; `vert_loop_ratio` (i32, "1", σ exact, core §7.5).

**Composition parameters** (scenario, σ choice, neutral default, set per run by `sil_override`, the l5 register pattern):
- `vert_hover_thrust_n` (N): per-run value by run_l4.hover_thrust's rule (tools/sim/run_l4.py:204-208), card mass ×
  WGS 84 normal gravity at the scenario site; 7.5705781 N at the L2 site
  (tests/regression/quad/L05/t3/reference/attitude_t3_q_inputs.txt:9).
- The z_ref segment script (stamp, z_ref); the thrust disturbance (N, t0); the torque disturbance pulse (axis, N·m, t0,
  duration), generalising `l5_dist_yaw_nm` (0006 F).

**Budget** (design/budget.yaml).
- Reused, used_by extended, values unchanged, not re-listed (the L01 "exactly one later manifest" rule, 0005 What
  changed): `PM_min` (budget:6-19), `Ms_max` (budget:114), `tau_robustness_band` (budget:90; τ_m ∈ τ(1 ± b_τ)),
  `per_push_check_time_max`.
- **New: `thrust_gain_robustness_band` = UNKNOWN (Luis).** Loop-gain factor r = (k_true/k_card)·(m_card/m_true): k-ratio
  ∈ 1 ± b_k, normalised mass m_true/m_card ∈ {0.974093, 1} (the card's value and its conflict; rate.py's J_true role). Until it has a value the generator runs on the mass hull and τ band and must report
  the admissible gain-ratio interval at which PM_worst ≥ PM_min and Ms ≤ Ms_max. The layer cannot freeze until the
  entry has a value inside that interval.

**Card inputs** (vehicles/uzh_neurobem_5in.yaml): mass 0.772 kg, σ 0.010 (half the spread), conflict 0.752 kg in two
sources, UNVERIFIED ("B1 log replay settles it"); thrust_coeff 1.562522e-6 N/(rad/s)², σ UNKNOWN, INFERRED; speed_range
[150, 2800] rad/s, σ UNKNOWN; motor_lag τ 0.033 s, σ UNKNOWN (budget band instead). Plant: g at h = site_height −
pos_ned[2], wrench from the end-of-step motor state (sim/plant/include/marv_plant.h:81-84); no drag, ground effect or
battery at v0 (marv_plant.h:10).

**Derived values** (rule → result, double): T_max = 4kω_max² (zero torque, the mixer's c_hi) = 49.0007 N; T_min =
4k·idle_speed², idle_speed = speed_range.min (tools/card/mixer.py:37) = 0.14063 N; ω_hover = √(T_h/4k) = 1100.58 rad/s
(DShot 765, agrees with 0017:3); normalised-mass corner 0.752/0.772 = 0.974093; mass-conflict thrust offset (0.772 − 0.752)·g(φ,h0)
= 0.19613 N; γ = −∂g/∂h at (φ, h0) from the plant's own normal-gravity function (rule only; the packet's 3.086e-6 s⁻²
is a session figure, not a source, core §2 rule 3). **Conflict for Luis (flagged, not decided):** quad:676 says "T/W ≈
4.5" with no source; the card gives T_max/T_h = 6.47. If k alone explains it, the true k is 4.5/6.47 = 0.70 of the card's.

### 5. Pass bar (thresholds from rules; numbers come at build from the generator)

**T1 (core, ms).**
- The law against hand-computed values, including the sign (below reference ⇒ more thrust); I4; I5; T_h finite > 0 and
  within [c_lo, c_hi] at zero torque; truth_vertical status codes (E_STATE, E_NULL, E_ABI, E_TICK, E_INPUT; reserved
  bits; valid-bit rule); composition order (vertical before rate, ⊆ rate ticks, stale-latch panic, id prefix); G3 controls.

**T3 margins (core).**
- Model: exact ZOH lifted, s = [lagged thrust, v, z, I, e_prev], γ included. Corners (τ, r) at nominal + four, plus a
  17×17 box grid refined by halving, last-halving change recorded (0006:658).
- Checks: PM_worst ≥ PM_min − δ_num and Ms_worst ≤ Ms_max on f32 gains (δ_num per rate.py rule 6); tightness (the next
  f32 gain above fails); Jury at every member; rate.py rule 7's monotonicity condition at the (r_lo, τ_lo) corner.
- Controls (each a CI step that passes only if the check fails): k_v, k_i, k_z × 1.1 and one added tick break the
  margin check; loop disabled breaks Jury (I6). That is how "would have diverged without the loop" is verified: τ =
  569 s (packet) cannot show in a 60 s run, and core §7.1's placement rule (core:373-375) puts a linear property at T3.

**T3 golden (core).**
- The float firmware path (vertical law → T_req) against a double design plant at tick resolution. The plant carries
  the collective rotor speed with its first-order lag and T = k·Σω² (exact for marv_plant v0 at level); margins use
  its linearisation at hover.
- Scripts: height step, thrust offset. Golden from an independent double oracle; tolerance the first-order rounding
  bound (0006:675-679); stored per 0011 (generator + inputs + `tests/regression/quad/L11/t3/reference/SHA256SUMS`,
  regenerated in the image, one-ulp perturbed-input control). Controls: k_v, k_i, k_z × 1.1 and one added tick.

**T3 envelopes (recorded; used at T4).** One box envelope per T4 script, from the exact initial state, never re-seeded.
The roll shove uses a coupled plant: per-rotor speed lags, f_i = kω_i², the mixer in double, 0006's quaternion
kinematics without ω×Jω, and the vertical channel with T·R33. Duration doubles until the envelope's end value is below
the T4 tolerance (0006:692-693), which sets run length.

**T4 (gz, truth-fed).**
- Predicate: z at every vertical execution inside envelope ± (E + F + Q), with E = |y(m=1) − y(m=2)| (0003 item 9);
  F = the T3 tolerance + last-halving change; Q = the box maximum of quantised − unquantised trajectory through the live
  diffuser (0006:697-699; 0017). Also: a clean run, DShot in [idle, 2047], end within E + F + Q of z_ref.

| Scenario | Amplitude | Note |
| --- | --- | --- |
| (i) Height step | labelled scenario value, generator-proved linear (no collective clamp over the box) | |
| (ii) Thrust offset | 0.19613 N (mass conflict), sign as the conflict (vehicle lighter than the card) | integral action |
| (iii) Roll shove | 0.02 N·m for 0.2 s, the demonstration's values (packet; labelled scenario value, Luis to confirm) | actuator-side, like 0006 F's yaw disturbance; a true external wrench is B1/SIM-6 |

- Controls: loop disabled (SIL override) must fail (ii) and (iii). Gains × 1.1 and one tick are tried at T4; if they
  pass there they are recorded as T3-enforced (0006 decision 19/24, 0006:725-731) after any harness limit is fixed.
- No recovery scenario: R2's collective question stays Luis's (§6).

**Freezes:** all T1, T3 (margins, golden, envelopes via SHA256SUMS), the three T4 scenarios with their controls, and
the G3 controls.

**CI cost.**
- T1/T3 in `core`. T4 in a new group `l11` / Actions job `gz-l11` with its own `host-gz-l11` preset (groups may not share
  builds, ci/run_ci_gz.sh:148-150). Per-test wall time UNKNOWN until measured; the only committed datum is L2 free fall,
  Release: marginal RTF 9.183 per core, 2.084 s startup (tests/regression/quad/L02/results/sim3/image_release.txt). Any
  test over `per_push_check_time_max` (60 s) under local CI (S9) runs nightly, and its spec line says so.

### 6. What it gates and what it does not change

**Gates.** Nothing in L6–L8 (G3 keeps the truth-fed loop out of every flight composition). Before it can fly, each a
separate decision for Luis:
1. A barometer class model in marv_plant, profile-driven like L6 (a). The board profile already has a barometer class
   (BMP581 row, core:310); it joins per "other sensor classes join with the layer that first uses them" (quad:149-150).
2. A vertical estimator layer (baro + accel) that fills `VerticalState`. Its gain rule needs a noise line like L6 (c)'s
   `d_path_noise_budget`: the truth-fed sup rule ignores sensor noise and will set a crossover a noisy estimate cannot
   support.
3. A §6.1 pilot mode (altitude hold within FREESTYLE) at L8, with a climb-rate stick map and its limits.

**Does not change:** L5's composition, register, tests and `l5_thrust_n`; decision 0005 item 9 (0005:60-63) still
governs the L4/L5 step tests (L11 states its own altitude claim and supersedes nothing); the mixer and rate-group
interfaces; L6 (e)'s scope; the R2 package question 2 ("may the allocator raise the collective in a recovery?",
docs/handoff.md "Open items"), which stays Luis's.

**New coupling.** (e)'s FF switch-on commit must also keep L11 green. Low risk: ω×Jω is small at the shove's 8–9°
(packet). INFERRED until L11 runs with FF on.

### 7. Build stages (one commit each, full `ci/local_ci.sh` per commit)

| Stage | Content | Tier | Estimate |
| --- | --- | --- | --- |
| S0 | Spec text and decision record 00NN with Luis's answers; budget entry | lead | 0.5 d |
| S1 | VerticalState; truth_vertical (.h/.cpp/map); TRUTH_VERTICAL option; manifest flag; check_g3 rule; three controls; T1 | senior | 1 d |
| S2 | `fw/vertical` (law, config, validate, anti-windup, fallback); T1 | standard | 1 d |
| S3 | `tools/card/vertical.py` (rate.py/attitude.py transposed, γ, Ms, corners, admissible-interval report); `flatten --out-vertical`; T3 margins and controls | senior | 2 d |
| S4 | Composition `l11_vertical_scripted`; register; SIL library; composition T1 | standard | 1 d |
| S5 | Plugin element; TruthVertical adapter; log type 8 (plugin, lockstep_log.py, sensor_replay refusal); I1 byte-identity check | senior | 1.5 d |
| S6 | T3 oracle, golden, vertical and coupled envelopes, refdata per 0011 | senior | 2 d |
| S7 | Scenarios; runner; T4 gz tests and controls; CI group/job; S9 timing | senior | 2 d |
| S8 | Close: results record, handoff, Luis's approval; tag by Luis | lead | 0.5 d |

S3 is blocked on `thrust_gain_robustness_band`. Total about 11.5 working days (an estimate, not a commitment).
