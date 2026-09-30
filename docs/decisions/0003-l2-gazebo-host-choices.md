# 0003: quad L2 Gazebo host choices

## What changed

No frozen file under `tests/regression/` is changed or deleted. This record holds the owner's choices for quad L2 (quad
spec §4 L2) where the spec left a choice or names an `UNKNOWN`:

1. **CI Gazebo.** Luis chose a second pinned image for the T4 steps. The existing `marv-ci` image stays as it is, and
   `marv-ci-gz` (the same Ubuntu 24.04 base, with gz-sim 8 / Harmonic pinned) runs only the Gazebo steps.
2. **SIM-3.** Luis chose to measure the throughput now and compare it later. L2 measures the real-time factor per core
   and records it in the run report. `design/budget.yaml` `batch_plan` is `value: UNKNOWN`, so the required value and
   the comparison are reported as `UNKNOWN`, and the SIM-3 item stays open (not passed) until `batch_plan` is set
   (core C-6).
3. **Adapter test scope.** Luis chose to compare all v0 outputs. `marv_plant` v0 has no sensor models, since they
   arrive at L6. The L2 adapter test compares everything v0 returns (the wrench, the motor states and eRPM) bit-exact
   against a direct `marv_plant` call, plus exact ENU↔NED round-trips. Adding sensor bytes to the frozen adapter test
   at L6 needs its own decision record.
4. **Q-D4.** Luis chose generated world variants. One template, expanded by the SDF generator, writes a pilot world
   (real-time factor 1) and a test world (real-time factor 0, uncapped). The physics step is set in the same generated
   file. Nothing calls `set_physics` at start.
5. **Command source at L2.** Luis chose open-loop firmware. There is no controller before L3/L4, so a composition
   `l2_open_loop` outputs a per-motor DShot read from parameters, and the scenario sets those parameters through the
   SIL override (core §4). The plugin always steps the firmware in-process (quad §3.1 step 4), an integer number of
   ticks per host step. It passes one IMU sample per tick, zeroed and flagged invalid, because `marv_plant` v0 has no
   sensor models. L3/L4 replace the composition; the plugin does not change.
6. **What "ENU↔NED round-trips are exact" means** (Luis accepted this reading).
   - **Vectors.** World ENU→NED is (x,y,z)↦(y,x,−z) and body FLU→FRD is (x,y,z)↦(x,−y,−z). Both are signed
     permutations, their own inverses, and bit-exact both ways, ±0 included (unary negation only).
   - **Plant outputs.** These are compared bitwise (force, torque after the exact map, rotor speeds, eRPM and
     erpm_valid) against a direct `marv_plant` call. That call gets the same `marv_plant_body` the adapter built, the
     same per-tick commands and the same dt.
   - **Attitude.** The ENU→NED attitude change is a 180° rotation about (1,1,0)/√2, so no binary64 quaternion formula
     is exact. The quaternion is held to a derived rounding bound, u = 2⁻⁵³:
     - forward, ≤ 6u‖q‖ per component against an independent Hamilton-product route;
     - round trip, ≤ (3+3√2)u‖q‖ per component.
   - **Body rates.** ω_frd = R(q)ᵀ·perm(ω_enu) is a rotation and gets a bound derived the same way.
   - **Seeds.** The seed dimension is kept in the test, though `rng_seed` is unused at v0.
7. **Hover under DShot quantisation** (Luis accepted the bracket reading, as in L1's hover test). No integer DShot
   gives exact hover thrust. The hover scenario runs at D_lo = ⌊D*⌋ and at D_hi = D_lo + 1, where D* is the inverse ESC
   map at ω_h = √(m·g(φ,h0)/(4k)). It passes when all three hold:
   - a_D matches the analytic acceleration for that command (first-order lag closed form, g(φ, h0 − p)) under the rule
     of item 9;
   - a(D_lo) > 0 > a(D_hi);
   - |a| ≤ 4k·(ω(D_hi)² − ω(D_lo)²)/m.
   The check time T is at least τ·ln(2a_T/F_a).
   The reference samples the lag closed form the way `marv_plant` defines its output (frozen L1 ABI, `marv_plant.h`:
   the wrench is evaluated from the end-of-step motor state and held over the step). Over tick j the thrust is
   4k·ω(t_{j+1})². A continuous-thrust reference differs from the plant's defined input by ½·a_T·t_tick in velocity,
   independent of the host step. Measured on the first wiring run: 7.660228e-4 m/s against a_T·t_tick/2 =
   7.660183e-4 m/s. The halving rule cannot remove that difference, and it is not Gazebo's integration error, which
   is what this scenario checks.
8. **Ticks per host step (lead decision).**
   - The host step is H = m·t_tick with any integer m ≥ 1, and the state is read once per host step.
   - For each tick j: one `marv_sil_tick(j, 1, …)`, then one `marv_plant_step(body, dshot_j, t_tick)`. The motor
     sub-step is h = t_tick (exact zero-order hold).
   - The applied wrench is the mean of the m per-tick wrenches, summed in tick order starting from W_0 and divided
     once, so m = 1 is the identity bitwise.
   - This loop lives in the host-free adapter, so the T1 tests cover it.
9. **T4 pass rule under SIM-7 (lead decision).**
   - **Step sequence.** H_k = 2^(K−k)·t_tick for k = 0..K, with K = 2. One tick is the lockstep floor.
   - **Terms.** d_k = q_k − q_{k−1}, E = |d_K| and ρ = d_K/d_{K−1}.
   - **Pass rule.** A quantity passes if either (i) or (ii) holds:
     - (i) E ≤ F and |q_K − q_ref| ≤ F + F_ref;
     - (ii) ρ ∈ (0,1) and |q̂ − q_ref| ≤ E + F(1+ρ)/(1−ρ) + F_ref, where q̂ = q_K + d_K·ρ/(1−ρ) is the Aitken /
       Richardson extrapolation with the observed order.
   - **Roundoff floor.** F = N·u·Q, the first-order recursive-summation bound of Higham (2002) §4.2, plus the counted
     roundings of q's own formula.
   - **No assumed order.** No integrator order is assumed. That DART uses semi-implicit Euler is `INFERRED`. Every
     d_k, ρ, observed order, E and F goes in the run report.
   - **Negative controls.** These are injected through the harness (SIL overrides), never through code:
     - free fall and rotation: motor 1 at DShot 48;
     - hover: motor 1 at D ± 1.
10. **Determinism and physics engine (lead decision).**
    - **Determinism check.** SIM-2 compares the plugin's binary log, byte for byte, from two separate gz processes. The
      log holds no wall-clock, host or path bytes.
    - **Determinism control.** The generated SDF's ixx is set one ulp higher, and the first differing byte must lie
      in an integrated-state field.
    - **Physics engine.** The world names the dartsim engine explicitly. The DART library version is pinned in
      `marv-ci-gz`. The gz-sim, gz-physics, DART and sdformat versions go in every run report. Changing the engine or
      DART version needs a decision record.
11. **gz-sim 8 behaviour found while building L2, and the lead's response to it.** The source is gz-sim8
    `src/systems/physics/Physics.cc`. Lines are those of the gz-sim8 branch head, read through a code-search index; the
    velocity finding is confirmed by the plugin smoke tests.
    - **Velocity commands.** Link `LinearVelocityCmd` / `AngularVelocityCmd` are link-frame, reset to zero after use and
      kept (l.4154–4164), which pins the link's velocity to zero on every later step.
      - Fix: the plugin applies the scenario's initial velocity and body rates once, at step 0, and removes both
        command components at the next step.
      - Evidence: `tests/regression/quad/L02/gz/test_plugin_smoke.py` shows that rates and velocity persist, and that
        skipping the removal breaks both checks (the negative control, env `MARV_GZ_TEST_KEEP_VEL_CMD`, test-only).
    - **Stale reads.** A link's WorldPose and velocities are rewritten only when its pose has moved more than 1e-6
      since the last recorded pose (`ChangedLinks`, l.3450–3459). When they are written, the value is exact
      (`Component::SetData` always assigns). Reads can therefore be stale while a body barely moves.
      - Rule, amending item 9: each T4 quantity is evaluated at the last fresh host step, one whose raw gz read differs
        bitwise from the previous step's. The reference is evaluated at the time of that read, t_i = i·H: the state
        after i physics steps. The simTime that PreUpdate sees is already (i+1)·H.
      - A scenario fails if it has no fresh read after the settling time (hover) or in the last half of the run. The
        run report lists the number of stale steps.
      - The plant's own input may also be stale. At L2 that is harmless: 1e-6 m of height changes g by about 3e-12
        m/s², and the L2 scenarios have no torque that depends on attitude.
    - **Inertia is not checked by the plugin.** The plugin refuses a world whose SDF mass differs bitwise from the
      plant's mass, or whose products of inertia are nonzero. It does not compare the diagonal inertia: the plant does
      not use inertia, and item 10's determinism control perturbs ixx.
    - **G3 exports.** The plugin (`sim/gz/plugin`) is a shared library but not a SIL library, so
      `cmake/flight_targets.cmake` excludes it from the G3 exports check's SIL list. It is outside `fw/`, so G3's
      symbol and include checks don't cover it either.

## Why

The L2 handoff (`docs/handoff.md`, 2026-09-29) listed these as the owner's calls: item 1 is a toolchain change, item 2
consumes an `UNKNOWN` budget entry, item 3 is a pass-bar item that v0 cannot meet as written, and item 4 is spec
decision Q-D4, "decided at L2". Item 5 arose because the only existing composition (L0) is a synthetic test chain
that cannot fly, while quad §3.1 has the plugin step the firmware.

## Evidence

Luis's answers, 2026-09-29, given verbatim as the selected options: "Second pinned image for T4"; "Measure now, compare
later"; "All v0 outputs"; "Generated world variants"; "Open-loop firmware"; and, for items 6 and 7, "Accept this
reading" and "Accept bracket reading". Items 8–10 are the lead's decisions, taken after an architect review of the L2
test semantics, and are frozen with the L2 tests.

## Approval

Luis's approval of the pull request or push (core §7.3). Luis, 2026-09-30, after the L2 report listing items 1–11:
"yes go ahead, commit, push and replace the handoff".
