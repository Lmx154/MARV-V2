# 0020: quad L11, the vertical loop on truth state (opened)

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
- `design/budget.yaml` is unchanged in this commit: `thrust_gain_robustness_band` = 0.5 (ruling 5) lands with stage S3,
  the gain generator, together with its `tests/regression/quad/L11/param_ids` claim and the product parameter table
  it changes (decision 0004 manifest rule; the L06 product_params table is frozen, so that commit names it here).
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
