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

**Motor-speed line: approved by Luis (2026-10-04) with four edits, verbatim:**

"**The motor-speed line: approved, with four edits.** Apply them to the text in 0017 and quote the final text in the report.
1. **Define the error.** It is the plant's rotor speed under the diffused command minus the same motor lag driven by the unquantised clamped request d̃, both starting from the same state. Say so in the line.
2. **Between samples.** The derivation bounds the error at write instants. The plant integrates between them. Either extend the derivation to every plant step (the same summation by parts, with the partial-period factor), or assert only at write instants and say so in the line. I prefer the first.
3. **The rad/s conversion.** Use the largest slope of the card's DShot-to-speed map over hover ± 1 step, as a derived value with that rule. Don't use the slope at hover alone.
4. **Run it on the plant's own motor model** (`marv_plant`), not a re-implementation, as with R2's ω̇ = 0 check. Keep τ− as the corner: it gives the largest 1 − a, so it's the conservative one.

Negative control: as written, diffuser off must fail. Report its ratio at the hover request. Hover's fractional part is about 0.06 step, against a bound near 0.02 step if τ− is half the card's 33 ms (INFERRED: use the real band). If the stateless control doesn't fail at hover, stop and report rather than choosing another request."

**The final line** (quad §4 L6 pass bar, after the (d) T1 line):

> (d) T1: at hover, the rotor-speed error stays within (1 − e^(−T_r/τ−)) DShot steps + δ_max at every plant step,
> converted to rad/s by the largest slope of the card's DShot-to-speed map over hover ± 1 step. The error is the
> rotor speed of the plant's own motor model (`marv_plant`) under the diffused command minus the same motor lag driven
> by the unquantised clamped request d̃, both from the same state. τ− is the low corner of the motor-lag band, T_r the
> rate-execution period and δ_max the derived float32 bound. Negative control: with the diffuser off (stateless
> rounding), the same check fails at the hover request.

Derivation, at every plant step. The applied command is q_n = d̃_n + e_(n−1) − e_n with |e| ≤ ½.
- Through the lag, the error at time s ∈ (0, T_r] after write n is Σ_j w_j (e_(j−1) − e_j), with a_s = e^(−s/τ),
  w_n = 1 − a_s and w_j = a_s a^(n−1−j) (1 − a) for j < n.
- Summation by parts bounds it by ½ (total variation of w) = max(1 − a_s, a_s (1 − a)) ≤ 1 − a.
- At the write instants (s = T_r) this is the earlier bound 1 − a.
- Luis's check (2026-10-04): the error at time s is ½[|1 − a_s(2 − a)| + 1 − a·a_s] = max(1 − a_s, a_s(1 − a)), and
  both terms are ≤ 1 − a.
- With the card's values (τ = 33 ms, band 0.3, so τ− = 23.1 ms; T_r = 312.5 µs), 1 − a = 0.01344 step. The test
  derives the value itself.
- A stateless constant error passes the lag unattenuated: at hover's fractional part (about 0.06 step) the control is
  expected to fail by about 4.5× (INFERRED; the test measures it).

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
  **Narrowed** (Luis, 2026-10-04: named here as narrowed). At each fault execution the composition's DShot is no longer
  compared with the head path's own DShot (69c62f2's stateless path, :148). With the carry, the head's DShot history
  differs from the step's ({764,764,764,763} vs {764,764,764,764}). In its place, the composition's DShot is compared at
  every execution with a diffuser run over the step's thrusts, using the head's thrusts at fault executions.
  - Kept: at every fault execution the step's torque, request and thrusts equal the head's bit for bit, before
    quantisation (`fault_test.cpp:308-310`); the composition's DShot equals the step's at every tick (`:276`).
  - The head is the path without the chain, so it equals the step only at fault executions. That was the test's scope
    before this change too.
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

Apart from the fault-test narrowing above, no assertion, bound, tolerance or test name is loosened. Every reference stays bit-exact, and each moved reference keeps
the stateless law as a failing control.

The motor-speed T1 test and the FF-on re-capture (after Luis's approvals of 2026-10-04).

**The motor-speed T1 test** (`tests/regression/quad/L06/dshot_diffusion/motor_speed_test.cpp`, target
`unit_l6_dshot_motor_speed`, 3 tests, 0.38 s).
- The terms:
  - τ− = (1 − 0.300000012)·0.033 = 23.0999996 ms (the band is the parameter set's float32);
  - T_r = 2 × 156.25 µs;
  - 1 − a = 0.0134370447 step, with a read from the plant itself (a = d̂², d̂ the plant's one-tick decay);
  - δ_max = 2^-14 step;
  - the bound is 0.0134980799 step = 0.0178939028 rad/s at slope 2650/1999 = 1.32566283 rad/s per step. The card's map
    is linear (asserted), so this is the largest slope over hover ± 1 step.
- The plant runs in double. Its rounding adds ρ = 1.2e-9 rad/s, derived on its line in the test.
- The hover request: 1.89264452 N per motor, so d* = 765.059814 (fraction 0.0598). Start: rotors at rest, zero carry,
  as a SIL run starts. No transient is excluded.
- Results, at every plant step:

  | Case | Max error | Share of the bound |
  |---|---|---|
  | Hover, diffused | 0.00925186 rad/s (0.00697904 step), 32768 steps × 4 motors | 0.517 |
  | Random in-range sequence (seed 601, extra) | 0.0105733 rad/s | 0.591 |
  | Control, stateless at hover | 0.0792938 rad/s (0.0598 step), first over at plant step 38 | 4.43, fails |

- **The d̃ reference (for Luis's acceptance).** `marv_plant` has no entry that takes a fractional command: the motor
  takes `uint16_t` DShot, and the substep is private (`sim/plant/src/plant_model.hpp:51, 74`).
  - The reference B is therefore built from two runs of `marv_plant`'s own motor model, one under ⌊d̃⌋ and one under
    ⌊d̃⌋ + 1, both started from B's speed at each write. Their outputs are combined as (1 − f)·ω_lo + f·ω_hi.
  - This is exact in real arithmetic, because the plant is affine in its command: the ESC map is linear (asserted by
    the test) and the lag step is a zero-order hold.
  - The lag is not re-implemented. A scratch check against a long-double lag at d̃ agrees to 1.7e-11 rad/s.
  - The affinity it relies on is asserted (`PlantMotorStepIsAffineInTheCommandAtEveryStep`, the reviewer's gap). From
    one state, marv_plant's motor advance under lo, lo + 1 and lo + 2 has a second difference of at most 4.55e-13 rad/s,
    against a derived double-rounding bound of 1.85e-9 rad/s. It is checked at every plant step for four motors, at
    hover and at rest, with lo at the hover floor, d_lo and d_max − 2.
  - Control: unequal spacing (lo, lo + 1, lo + 3) gives at least 8.94e-3 rad/s, 4.8e6× the bound.
  - The target now holds 5 tests.

**The FF-on records re-captured with the diffuser live** (owner, 2026-10-04: their own commands, the "before (d)"
numbers kept). Each case was run twice, byte-identical. No verdict changes.
- Acro, FF-on roll margin: +6.07 → +5.72 mrad/s (pass). FF-off roll excess: 5.2322 → 5.2318 rad/s (fail, as before).
- R1X α: FF off +5.05e-3 (74 violations) → +1.174e-2 rad (136 violations); FF-on slack 2.923e-2 → 2.295e-2 rad
  (pass).
- R2: FF off 14420 → 15206 violations (worst w_x +2.349 → +2.368 rad/s); FF on 2391 → 2431 violations (worst w_z
  +0.2373 → +0.2375 rad/s).

**Per-push file times (S9), measured in the CI image** (marv-ci, `--cpus 4 --memory 15740260352`, a fresh clone of
`c424cbe`, each file alone after `uv sync` and the T3 references).
- All 41 per-push tools files pass; together they take 311.3 s.
- Nearest the limit:

  | File | Alone time |
  |---|---|
  | `L06/tools/test_r1x_coupling.py` | 61.4, 60.6, 61.5, 61.0 s (58.9 s at stage (c)) |
  | `L06/tools/test_r2_lower_bound.py` | 58.2, 57.8, 58.0, 58.3 s (56.2 s at stage (c); margin 1.7–2.2 s) |
  | `L06/tools/test_rate_lead.py` | 44.1 s |
  | `L06/tools/test_l6_ff_eval.py` | 40.1 s |
  | `L06/tools/test_r2_envelope_a.py` | 22.8 s |

- **Applied under Luis's standing S9 rule** (2026-10-03: "Anything still over 60 s in the CI image ... goes nightly,
  under the S9 rule, recorded ... with its measured time"): `test_r1x_coupling.py` joins the nightly list in
  `ci/run_ci.sh`.
  - Per push it is `--ignore`d; `MARV_CI_MODE=full` runs it.
  - The gz R1X run stays per push (gz-l5, `test_t4_recovery.py`). The coupling counterfactual record
    (`r1x_coupling/`) is checked nightly.

**Frozen files changed by this commit:**
- `tests/regression/quad/L06/dshot_diffusion/CMakeLists.txt`: the `unit_l6_dshot_motor_speed` target (lines added).
- `tests/regression/quad/L06/results/ff_on/gz_acro.txt`, `tests/regression/quad/L06/results/ff_on/gz_r1x.txt` and
  `tests/regression/quad/L06/results/ff_on/gz_r2.txt`: re-captured with `capture_gz.py`.
- `tests/regression/quad/L06/results/ff_on/README.md`: the new numbers, with the "before (d)" ones kept.

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

- The evaluation and the stage close.

## Approval

Owner decisions: Luis, 2026-10-03, as quoted. The motor-speed line: Luis, 2026-10-04, with the four edits above.
P1 and P2 approved: Luis, 2026-10-04. The stage: pending.
