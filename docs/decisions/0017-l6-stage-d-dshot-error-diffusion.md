# 0017: quad L6 stage (d), DShot error diffusion

Stage (d) of quad spec §4 "L6" ((d) DShot error diffusion). Hover sits at DShot 765, and one step is 4.56 mN per motor.
Stateless rounding holds the request up to ½ step away, which produces a ±0.006 rad attitude limit cycle
(`L05/results/step_cause/`; 0006 decisions 19 and 20). A per-motor error carry makes the mean applied command follow the
request. This record changes L3's thrust → DShot output once it is live, so it is its own record (quad spec §4 L6
Builds).

## Owner decisions (Luis, 2026-10-03, verbatim; 0014, fourth round, item 5)

"**5. Stage (d), record 0017 (0016 is R2's harness fix).**
1. **Scheme: approved.** A per-motor carry, with u clamped to the command range before rounding and the carry taken from the clamped value. The saturated remainder is never carried, so there is no windup at the limits. The existing L3 saturation flag still reports the clamp.
2. **Live in its own atomic commit in (d): yes.** It doesn't touch the xfail gate. If it makes a strict xfail pass, stop and report.
3. **Pass bar: yes, with two changes.**
   - The bound holds for any in-range request sequence, not only a constant one, because the sum of errors telescopes to the carry difference. So test random and adversarial sequences, and the saturation case separately.
   - The 1.2e-4 term must be derived (a float32 accumulation bound, like P6's rounding cap), not measured. Use the measured value only as evidence that the bound isn't loose by orders of magnitude.
   - Motor-speed line: yes, if it is a derived bound (hover rotor-speed ripple from ±½-step dither through the card's motor lag) with the diffuser-off case as its negative control. That limit cycle is why this stage exists. Send me the text to approve.
4. **Reset rule: approved.** Name the disarm and motor-stop paths explicitly as non-diffuser writes, and give each reset path its own T1 test.
5. **Q: same definition, recomputed with the diffuser in the loop.** It should shrink. Re-confirm that the fine controls still fail (decision 24).
6. **D-path budget unit: unchanged.** The DShot step's size doesn't change.
7. **AM32 per-frame behaviour: yes, on the pre-L8 bench list.** The diffuser's benefit assumes the ESC applies each frame. That stays INFERRED until the bench shows it."

**Motor-speed line, text sent to Luis for approval (2026-10-04), pending:**

> (d) T1: at hover, the rotor-speed error that the diffused command leaves after the card's first-order motor lag stays
> within (1 − e^(−T_r/τ−)) DShot steps + δ_max, converted to rad/s through the card's DShot-to-speed map. Here τ− is the
> low corner of the motor-lag band, T_r the rate-execution period and δ_max the derived float32 accumulation bound.
> Negative control: with the diffuser off (stateless rounding), the same check fails.

Derivation: q_n = d̃_n + e_(n−1) − e_n with |e| ≤ ½. Through the lag a = e^(−T_r/τ), summation by parts bounds the error
by (1 − a)(½ + ½) = 1 − a steps. A stateless constant error of up to ½ step passes the lag unattenuated.

## Why

The DShot step quantises the hover command. Stateless rounding holds a request up to ½ step away for as long as the
request stays inside one step, so the vehicle limit-cycles around hover (±0.006 rad, `L05/results/step_cause/`). That
cycle also costs the roll/pitch chirp about 2.5° of measurement uncertainty (0006 decision 20). With the carry, the
applied command's window mean follows the request within 1/N + δ_max. The step's size and the D-path budget unit are
unchanged (owner decision 6).

## The design (architect draft, 2026-10-01, adopted)

- **Where.** A new stateful `DshotDiffuser<T>` in `fw/mixer`. `thrust_to_dshot` and `mix` stay unchanged, so L03 is
  bit-identical.
- **The law, per motor, once per actuator write (one rate execution).**
  - d̃ = clamp(d*, d_lo, d_max), with negated comparisons: NaN, negative and −inf go to d_lo; +inf goes to d_max.
  - u = fl(d̃ + e), q = clamp(round(u), d_lo, d_max), e = u − q. The last step is exact by Sterbenz.
  - Clamping before rounding keeps |e| ≤ ½. Round-then-clamp lets the carry grow without bound at saturation.
- **Reset** e = 0 on init, on a mixer configuration reload, and on every non-diffuser write. Disarm and motor stop are
  named explicitly; at L8, ESC commands 1–47 also count. A rate-loop fault does not reset it.
- **Unchanged:** saturation and the integrator freeze (`allocate` works before quantisation), and the idle rule
  (0004 item 5).

## What changed

P1, the diffuser and its T1 tests (not live).

**Firmware** (`fw/mixer/include/marv/mixer/mixer.hpp`, `fw/mixer/src/mixer.cpp`).
- d* is factored out as `dshot_of_speed`, `dshot_floor` and `dshot_unrounded`, and `thrust_to_dshot` now calls them.
  Its output is bit-identical:
  - the comparison predicates are unchanged;
  - a 2^20-point sweep against a verbatim copy of the old function finds 0 mismatches;
  - on m33 the only object change is one compare with swapped operands, the same "less or unordered" predicate.
- `DshotDiffuser<T>` (`init`, `reset`, `apply`, `apply_unrounded`, `carry`) stores its own copy of the config and is
  instantiated for float. No composition calls it yet: that is P2, the live switch.

**δ_max, derived** (it differs from the draft's 1.22e-4):
- The only rounded operation is u = fl(d̃ + e). The clamps, the round and e = u − q are exact.
- u ∈ [d_lo − ½, d_max + ½] ⊂ [2^5, 2^11), where the largest float32 spacing is 2^-13 (in [2^10, 2^11)). So
  |ρ| ≤ 2^-14 = 6.10e-5 step per write.
- The window mean averages N such terms, so δ_max holds for every N.
- The draft's 2047.5·ε/2 is the standard u·|x| bound on the same addition, 2× looser.
- Measured over 12.06 M motor writes per configuration: max |ρ| = 6.1035e-5, so the bound is attained; max |e| = 0.5.

**Reset paths.** `fw/` has no disarm or motor-stop path today: there is no arming logic until L8.
- The stop writes that exist run before composition init (`hal_sim.cpp:69`, `marv_sil.cpp:204, 213`) or in
  compositions without the mixer (`l0.cpp:56`, `l2_open_loop.cpp:57`).
- The future callers are named in the header (`mixer.hpp:309-315`): init (`RateGroupStep::init`,
  `rate_group.cpp:75`), configuration reload (none at runtime today), disarm and motor stop.
- Each path has its own T1 test, which also checks that a copy that skips the reset fails.

**T1 tests** (`tests/regression/quad/L06/dshot_diffusion/`, 14 tests). They run on the product configuration
(d_lo 48) and on a raised-idle one (d_lo 648).
- |e| ≤ ½ on random (2^20 writes), near-½, thrust-driven, ramp and adversarial sequences; saturation, NaN and ±inf in
  their own test.
- |mean(q) − mean(d̃)| ≤ 1/N + δ_max for every window N without a reset (½/N from a reset).
- Bit-identity: `thrust_to_dshot` is unchanged, and with e = 0 the diffuser equals it for every integer command.
- Negative controls, each breaking its check:
  - round-then-clamp: the carry reaches inf at the first saturated write;
  - carry forced to 0 at fraction 0.3: windows fail from N* = 4 = ⌊1/(0.3 − δ)⌋ + 1;
  - sign-flipped carry: the mean fails by write 2–5;
  - stateless rounding: the mean fails at write 5 or 8.
  - Each mutation was also applied to `mixer.hpp` itself and broke its test. These mutations are not committed.

**Frozen file changed:** `tests/regression/quad/L06/CMakeLists.txt`, one added line, `add_subdirectory(dshot_diffusion)`.
Nothing else under `tests/regression/` changes.

P2, the atomic live switch.

**Firmware.**
- `fw/rate_group` holds a `DshotDiffuser` member: `RateGroupStep::init` resets it (`rate_group.cpp:81`), and `finish`
  applies it to the allocation (`:115`, which replaces `thrust_to_dshot`). Both live compositions (`l4_rate_scripted`,
  `l5_attitude_scripted`) and the L4 replay follow by construction.
- `l0`, `l2_open_loop`, `thrust_to_dshot` and `mix` are unchanged, and L03 stays green.
- `marv_sil_init` runs `composition::init` once per process (a second call returns E_STATE), so every run starts from
  a zero carry.
- Comment-only edits in the two compositions and in `l4_acro_replay.cpp` name the diffuser.

**Q, same definition, with the diffuser in the loop** (owner decision 5).
- The oracle's `Quantiser(diffusion=True)` holds the firmware's per-motor float32 carry and is reset per script run.
  Over 550 k writes × 4 motors it matches `DshotDiffuser` with 0 mismatches; a binary64 control mismatches 19998 of
  20000.
- The stateless path is kept for the dead-band facts (hover DShot 765, dead bands unchanged) and as the control.
- `recovery_model.py` (`Quant3`) and `tools/sim/run_l5.py` follow the same model.
- `attitude_t3_q.txt` was regenerated in marv-ci (hash `189d7e01…`); the golden and envelope are byte-identical.
- Q before → after (θ rad / ω rad/s; yaw as ω / heading_release / heading_lock):
  - roll 7.80e-3 → 2.98e-6 / 3.67e-2 → 6.01e-5;
  - pitch 7.03e-3 → 1.79e-6 / 3.31e-2 → 5.35e-5;
  - yaw_release 4.78e-3 → 1.60e-5 / 1.45e-3 → 1.14e-6 / 9.85e-4 → 5.91e-7;
  - yaw_fallback 3.48e-3 → 1.53e-5 / 1.09e-3 → 1.19e-6 / 6.23e-4 → 6.06e-7.

**Fine controls (decision 24) still fail, by more.**
- Steps att_kp ×1.1: roll slack +4.205e-2 → +4.989e-2; pitch +4.287e-2 → +4.990e-2.
- Recovery kp ×1.1: +0.385 → +0.424.
- Chirp att_kp ×2: measured PM roll 34.88° → 35.53°, pitch 34.86° → 35.52°, yaw 35.59° → 35.62°; all fail.

**The xfail gate.** No strict xfail passes: L4 gz 40 passed, 1 xfailed; L5 gz 63 passed, 2 xfailed. R2's worst slack
is +2.349 → +2.368; R1X α +5.05e-3 → +1.17e-2; the acro margins are unchanged.

**The limit cycle** (the reason for this stage): the gz step excursion outside the envelope falls from 5.73e-3 to
6.5e-10 rad (roll) and from 4.99e-3 to 7.7e-10 rad (pitch).

**The chirp plateau admission is unchanged.**
- Roll and pitch each admit k2–k5. Roll k1 has no crossover; pitch k1 stays excluded (shift 9.20° → 9.25°).
- The k5 shift moves from −2.60° to +0.03°.
- The slack is roll 7.600° → 13.256°, pitch 8.197° → 13.624°, yaw 15.188° → 15.170°.

**Committed records regenerated by their own generators** (they embed Q; their tests are unchanged). No conclusion
changes:
- `L06/results/r2_lower_bound/bound.txt`: the R2 lower bound holds with more margin. On w_y, F + Q falls from 0.031128
  to 0.01833 and X*/(F + Q) rises from 9.17 to 15.6 (X* 0.28537); on w_x it rises from 6.57 to 15.3, on w_z from 4.99
  to 5.02. The controls NC1 and NC2 still collapse the bound.
- `r2_lower_bound/e_measured.txt` (it checks against `bound.txt`): E(17) 6.62e-4 against 0.26704 needed (1/403).
- `r2_lower_bound/hover_tumble.txt`: the prop strike gives 19463 → 20204 violations (reported, no pass bar).
- `r2_envelope_a/envelope_a.txt`: only the input-hash line changes.
- `r1x_coupling/{gz_new,coupling}.txt` (`gz_new.txt` and `e_measured.txt` are gz re-captures with the live firmware,
  each run twice byte-identical): the conclusion holds. Q falls from 6.735e-3 to 2.665e-6. gz fails with 74 →
  136 violations; C stays closer to gz than D on every channel; the ideal feed-forward still removes the excess.
- The README and `.md` prose of these records, and the Q rule and table in `L05/t3/README.md`, are restated from the
  data.
- Left as records of their own runs, since no test re-runs them: `L06/results/ff_on/`, `ff_eval/`, `r2_ff_diagnosis/`,
  and `L05/results/{step_cause,step_controls,recovery_cause}`.

**Frozen files changed by P2** (complete list):
- `tests/regression/quad/L04/unit/composition/composition_test.cpp` and
  `tests/regression/quad/L05/unit/composition/composition_test.cpp`: the DShot reference is the diffuser over the step's
  thrusts, still bit-exact; stateless rounding is the control.
- `tests/regression/quad/L06/rate_group/fault_test.cpp`: the composition's DShot reference is a diffuser run over the
  step's thrusts at every execution; the head path at :148 keeps its stateless reference; stateless is the control.
  The direct composition-equals-head DShot equality is now checked only at the fault executions: away from them the
  head's own carry history differs from the step's ({764,764,764,763} vs {764,764,764,764}), which comes with the
  carry.
- `tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py` (`Quantiser(diffusion=…)`, `q_script`),
  `tests/regression/quad/L05/t3/reference/SHA256SUMS` (the Q line), `tests/regression/quad/L05/t3/README.md`.
- `tests/regression/quad/L05/tools/test_attitude_t3_q.py`: the dead-band facts call the stateless path explicitly;
  values unchanged.
- `tests/regression/quad/L05/gz/recovery_model.py`: `Quant3(q, diffusion=True)`.
- `tests/regression/quad/L04/replay/l4_acro_replay.cpp`: comment only.
- Regenerated or restated (above):
  - `tests/regression/quad/L06/results/r1x_coupling/README.md`
  - `tests/regression/quad/L06/results/r1x_coupling/coupling.txt`
  - `tests/regression/quad/L06/results/r1x_coupling/gz_new.txt`
  - `tests/regression/quad/L06/results/r2_envelope_a/envelope_a.txt`
  - `tests/regression/quad/L06/results/r2_lower_bound/README.md`
  - `tests/regression/quad/L06/results/r2_lower_bound/bound.txt`
  - `tests/regression/quad/L06/results/r2_lower_bound/e_measured.md`
  - `tests/regression/quad/L06/results/r2_lower_bound/e_measured.txt`
  - `tests/regression/quad/L06/results/r2_lower_bound/hover_tumble.txt`

No assertion, bound, tolerance or test name is loosened. Every reference stays bit-exact, and each moved reference keeps
the stateless law as a failing control.

## Evidence

P1:
- ctest 656/656 in host-debug and host-release.
- m33 builds with 0 warnings.
- G1 is clean (`lint_g1.py` in marv-ci); `check_constants.py` passes.
- Independent review: PASS.

P2 (working tree, before its commit):
- ctest 656/656 in host-debug and host-release.
- m33 builds with 0 warnings; G1 is clean (`lint_g1.py` in marv-ci); `check_constants.py` passes.
- Q regenerated in marv-ci (`refdata.py ensure quad/L05/t3 --force`); the host regeneration is byte-identical.
- The tools suite: 1080 passed.
- gz on the host: L04 40 passed, 1 xfailed; L05 63 passed, 2 xfailed.
- Mutation controls: firmware set back to stateless fails 9 of the moved tests; a carry reset on fault fails
  `L4CompositionFault` and `L6RateGroupFault`.
- Independent review: PASS. Its one prose finding is fixed.
- The local CI on the commit is in the handoff.

## Still to come in stage (d)

- The motor-speed T1 test, once Luis approves the text above.
- The evaluation and the stage close.

## Approval

Owner decisions: Luis, 2026-10-03, as quoted. The motor-speed line text, P1 and the stage: pending.
