# 0004: quad L3 mixer / allocation choices

## What changed

**Frozen files changed.** L3 adds 17 parameters to the product set `marv_params`: `idle_speed` and
`mixer_m<i>_{thrust,roll,pitch,yaw}`. That broke one frozen L01 check. Luis chose to extend the check and to
restructure it so that this is the last such edit:

- **`tests/regression/quad/L01/tools/test_flatten_report.py`.**
  - The id list `EXPECTED_IDS` moves out of the file into a per-layer manifest. It is now read from
    `tests/regression/quad/L01/param_ids`, and its content is unchanged: the 27 card and budget ids. The two card-only
    checks that use it are unchanged.
  - `test_cmake_product_set_comes_from_the_card_and_writes_the_report` now asserts that the built `ParamId` set equals
    the union of every `tests/regression/quad/L*/param_ids`, and that the manifests are disjoint. It is still an
    exact-set check, so nothing is loosened.
- **New manifests.** `tests/regression/quad/L01/param_ids` (new, the former `EXPECTED_IDS`) and
  `tests/regression/quad/L03/param_ids` (new, the 17 L3 ids).
- **Future layers.** A later layer that adds product-set parameters commits its own `Lnn/param_ids` and does not edit
  L01.
- **Negative controls, run by the lead on 2026-09-30.** An id planted in the L03 manifest fails the check (set
  mismatch). An L01 id repeated in the L03 manifest fails it too ("listed in more than one layer manifest").

The rest of this record holds the owner's choices for quad L3 (quad spec §4 L3) where the spec left a choice, and the
lead's choices that follow from them.

1. **Idle floor (Luis).** A parameter `idle_speed` (rad/s) is derived from the card's `speed_range.min` (150 rad/s,
   method published, σ `UNKNOWN`). QF-6 asks for the *measured* minimum stable rotor speed. None exists yet, so this
   entry stands in, and a measured value replaces it later without mixer changes. `idle_speed` must equal the plant's
   minimum speed (`marv_plant` config `omega_min`), and a test checks this.
2. **"Within float tolerance" for the identity check (Luis).**
   - **Stored mixer.** The generator computes the mixer M = B⁻¹ in double from the card and stores it as parameters.
   - **Effectiveness matrix.** B is in per-motor thrust. Column i is [1, −y_i, x_i, s_i·c_q]ᵀ, with (x_i, y_i) the
     FRD rotor position, s_i = `rotor_yaw_sign_m<i>` and c_q = `rotor_torque_ratio`. This is the forward map of
     `marv_plant` v0.
   - **Bound.** The check is per element: |fl(M̂·B̂) − I| ≤ (γ_n + ε)·(|M|·|B|). The hats are the float values,
     γ_n = nε/(1−nε), n = 4 and ε = float32 epsilon (Higham, *Accuracy and Stability of Numerical Algorithms*, §3.5).
   - **Why the bound holds.** Higham's γ_n is written in the unit roundoff u = ε/2. Here γ_n uses ε, which leaves
     slack for the second-order terms and the double-precision error in M.
   - **Negative control.** Perturb one entry of M beyond the bound, and the check fails.
   - **Scope.** L10 needs its own rule and does not reuse this one.
3. **Desaturation (Luis; step c amended by Luis on 2026-09-30).**
   - **Where it works.** Allocation is in per-motor thrust, with f_min = k·ω_idle² and f_max = k·ω_max². Then
     ω = √(f/k), then DShot through the inverse ESC map.
   - **Priority.** Roll/pitch > yaw > collective thrust:
     - a. If roll/pitch alone exceeds the available spread, scale roll and pitch together (direction preserved) and
       set yaw to zero.
     - b. Otherwise shift the collective so that roll/pitch fits. This is air mode: full authority at zero and at
       full throttle (QF-6).
     - c. Add yaw, shifting the collective again as far as keeps roll/pitch feasible, and clip only yaw if it still
       does not fit.
   - **Step c amendment.** Luis's first wording of step c ("add yaw within the remaining headroom") left no yaw
     authority at zero throttle. He chose "shift the collective for yaw too".
   - **Tests.** One test per case a–c, plus a randomized property test that no motor leaves [f_min, f_max] for any
     input.
4. **Opening outputs (Luis).** Besides DShot per motor, L3 outputs three per-axis saturation flags: roll, pitch and
   yaw, each set when achieved < requested. L4 uses them for anti-windup. It also outputs the achieved collective thrust
   in N, for B2's vertical controllers. These outputs are part of the L3 opening now, so no later record is needed to
   add them. Quad §4 L3 lists only "DShot 48–2047 per motor" as the output.
5. **Algorithm and storage (lead decisions).**
   - **Parameters.** The mixer is stored as 16 f32 parameters, `mixer_m<i>_{thrust,roll,pitch,yaw}` (row i of M, so
     f_i = Σ_axis M[i,axis]·u_axis). Each is method `derived(...)` with the rule, and σ `UNKNOWN`, the σ of its inputs.
   - **Generator refusals.** The generator refuses a card for any of these:
     - B is singular;
     - any thrust-column entry M[i,thrust] ≤ 0;
     - zero torque is not achievable, i.e. max_i f_min/M[i,thrust] > min_i f_max/M[i,thrust].
   - **Collective direction.** The collective is the thrust column of M, a pure-thrust direction by construction, so
     shifting it moves no torque. It is uniform (¼) only for a symmetric card.
   - **Allocation form.** f = s·(M_roll·τx + M_pitch·τy) + t·M_yaw·τz + c·M_thrust.
     - s is the largest value in [0, 1] for which some c keeps every f_i in [f_min, f_max]. This is case a when s < 1,
       and then t = 0.
     - Otherwise t is the largest value in [0, 1] with the same property, given s = 1. This is case c.
     - c is the requested thrust clamped to its feasible interval. This is case b.
     - s and t come from a closed pairwise form over motor pairs (i, j), so there is no iteration.
   - **Reported values.**
     - Achieved roll and pitch are s·request. Achieved yaw is t·request. Achieved thrust is c, taken at allocation
       level before DShot quantisation.
     - A flag is set when its factor is below 1 and its request is nonzero.
   - **Output path.**
     - Each f_i is clamped into [f_min, f_max]. The clamp is written as comparisons, so a NaN goes to f_min.
     - ω_i = √(f_i/k).
     - DShot is the inverse of the linear-in-ω ESC map, with endpoints `rotor_speed_min` / `rotor_speed_max` at
       48 / 2047. The ESC map is a labelled scenario value.
     - DShot is rounded to nearest, then clamped to [⌈D(ω_idle)⌉, 2047], so rounding can never put a motor below
       idle.
   - **Non-finite requests.** Motors stay within [f_min, f_max]. Nothing more is specified at L3.

## Why

The L2 handoff (`docs/handoff.md`, 2026-09-30) listed items 1–3 as the owner's calls. The spec gives the idle floor as
"the card's minimum stable rotor speed" (L3) and "the measured minimum stable rotor speed" (QF-6), and the card has
neither. "Within float tolerance" had no rule. The desaturation priority and air-mode behaviour had to be documented
before the tests were written. Item 4 extends the spec's L3 opening, at the owner's request.

## Evidence

Luis's instructions, 2026-09-30:
- On items 1–3, his message: "Idle floor: add an idle_speed parameter derived from the card's speed_range.min …";
  "Tolerance rule: the generator computes the mixer in double and stores it as parameters …"; and "Desaturation:
  allocate in per-motor thrust …".
- On step c, the selected option "Shift collective for yaw too".
- On the frozen L01 change: "Option 1, and I'll approve the edit. In the same change, restructure so this is the last
  time: move EXPECTED_IDS into a per-layer manifest (tests/regression/quad/L01/param_ids), add
  tests/regression/quad/L03/param_ids for the 17 L3 IDs, and have the test assert built set == union of all per-layer
  manifests. The check stays exact, but future layers add their own manifest in their own folder instead of editing
  L01. Record both in 0004."
- On item 4: "Three torque flags (roll, pitch, yaw: achieved < requested), plus the achieved collective thrust in N as
  an output value. L4 uses the flags; B2's vertical controllers will use achieved thrust. Include it now so the L3
  opening doesn't need a decision record later."

## Approval

Luis's approval of the push (core §7.3). Luis, 2026-09-30, after the L3 report (independent review PASS, full CI
pending): "once ci is green, commit, push and replace handoff".
