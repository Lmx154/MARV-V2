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

## Stage (d) close

**1. What landed.**
- `4bff7af`, P1: `DshotDiffuser<T>` in `fw/mixer`, with 14 T1 tests. Not live; flight output unchanged.
- `432daca`, P2: the diffuser goes live in `RateGroupStep` (reset at init, applied at every rate execution), so both
  scripted compositions and the L4 replay diffuse. Unchanged: `l0`, `l2_open_loop`, `thrust_to_dshot`, `mix`,
  allocation, saturation, the integrator freeze, the idle rule and the D-path budget unit.
- `43a23a2` (local): the motor-speed T1 test, the FF-on records re-captured with the diffuser, `test_r1x_coupling.py`
  moved to nightly (S9), and the P1/P2 approvals. No flight change.

**2. Product numbers, before (d) → after.**
- Q:
  - roll θ 7.80e-3 → 2.98e-6 rad, ω 3.67e-2 → 6.01e-5 rad/s;
  - pitch θ 7.03e-3 → 1.79e-6 rad, ω 3.31e-2 → 5.35e-5 rad/s;
  - yaw ω 4.78e-3 → 1.60e-5 rad/s (release), 3.48e-3 → 1.53e-5 rad/s (fallback).
- The gz hover limit cycle (step excursion outside the envelope): roll 5.73e-3 → 6.5e-10 rad, pitch
  4.99e-3 → 7.7e-10 rad.
- The fine controls (decision 24) still fail, by more:
  - steps att_kp ×1.1: roll +4.205e-2 → +4.989e-2, pitch +4.287e-2 → +4.990e-2;
  - recovery kp ×1.1: +0.385 → +0.424;
  - chirp gain ×2: measured PM roll 34.88° → 35.53°, pitch 34.86° → 35.52°, yaw 35.59° → 35.62°.
- The chirp slack: roll 7.600° → 13.256°, pitch 8.197° → 13.624°, yaw 15.188° → 15.170°. The admission set is
  unchanged.
- R2:
  - FF off: worst w_x +2.349 → +2.368 rad/s; violations 14420 → 15206.
  - FF on: violations 2391 → 2431; worst w_z +0.2373 → +0.2375 rad/s.
- R1X α:
  - FF off: +5.05e-3 → +1.174e-2 rad (fails);
  - FF on: slack 2.923e-2 → 2.295e-2 rad (passes).
- Acro:
  - FF-on roll margin: 6.07 → 5.72 mrad/s;
  - FF-off roll excess: 5.2322 → 5.2318 rad/s.

**3. The pass bar, line by line.**
- (d) "carried error within one DShot step": met. max |e| = 0.5 step over 12.06 M writes; margin ½ step.
- (d) "mean applied command over a window matches the request within the stated bound": met for every window N with
  1/N + δ_max. The bound is attained; the worst window comes within 2δ of it.
- (d) "the L03 suite stays green": met.
- (d) the motor-speed line: met.
  - Hover: 0.00925 rad/s against 0.0179 rad/s (0.517 of the bound).
  - Random sequence: 0.591 of the bound.
  - Stateless control: 4.43×, fails.
  - The affine step it relies on is asserted (second difference 4.55e-13 against 1.85e-9 rad/s; control 4.8e6×).
- Added by your rulings:
  - any sequence: random, adversarial and ramp sequences, with saturation separately, all within the bound;
  - δ_max = 2^-14 step, derived and attained (ratio 1.0000);
  - four reset paths, each with its own T1 test;
  - Q recomputed with the diffuser in the loop (oracle and firmware: 0 mismatches over 550 k writes);
  - the fine controls re-confirmed.

**4. Known failing items** (strict xfail while `L06/XFAIL_GATE_CLOSED` is absent). No strict xfail passes: L4 gz
40 passed, 1 xfailed; L5 gz 63 passed, 2 xfailed.
- L4 acro, FF off: fails (roll excess 5.23 rad/s). FF on: passes by 5.72 mrad/s, at the card plant without noise.
  (e): FF live.
- L5 R2, FF off: 15206 violations. FF on: 2431 violations, all on the rate channels (the attitude channels are inside).
  (e): FF live plus the yaw choice below.
- L5 R1X α, FF off: fails (+1.17e-2 rad). FF on: passes. (e): FF live.

**5. Frozen changes: 24 files.** None relaxed; one narrowed.
- P1: 1 (the `add_subdirectory` line).
- P2: 18 (0017, "Frozen files changed by P2"):
  - references moved to the diffuser, with stateless rounding as the failing control: 3;
  - Q recomputed: 5;
  - comment only: 1;
  - Q-embedding records regenerated, conclusions unchanged: 9. The R2 lower bound's margin grows, 9.17× → 15.6× on w_y.
- `43a23a2`: 5 (the motor-speed target's CMake lines; the 4 FF-on records, with "before (d)" kept).
- Narrowed: the fault test no longer compares the composition's DShot with the head path's own DShot at fault
  executions. In its place, a diffuser reference checks every execution. Thrust-level equality at every fault
  execution is kept (`fault_test.cpp:308-310`).

**6. Verification, by commit.**
- `4bff7af`:
  - local: core 1691 s, gz-l2 363 s, gz-l4 447 s, gz-l5 901 s, all pass;
  - Actions 37170708960: 3636 / 773 / 1548 / 2286 s, all pass.
- `432daca`:
  - local: 1946 / 394 / 498 / 1014 s, all pass;
  - Actions 37177370498: 3649 / 769 / 1516 / 2337 s, all pass.
- `43a23a2`: local 1546 / 342 / 418 / 882 s, all pass (ctest 661/661 ×2, frozen 660/660). Actions: after the push.
- The Actions core time:
  - before stage (c), 1200–1604 s;
  - `af25f5e` (the stage (c) head) 2530 s;
  - every push since, 3636–3886 s, including the docs-only `69f98f4` at 3886 s.
  - So the growth after `af25f5e` does not come from code. Same runner pool and image; the job logs need
    authentication, so the cause inside the script is not established.
- The per-push files nearest 60 s (marv-ci, alone):
  - `test_r1x_coupling.py` 60.6–61.5 s, now nightly;
  - `test_r2_lower_bound.py` 57.8–58.3 s, margin 1.7–2.2 s;
  - `test_rate_lead.py` 44.1 s.

**7. Limits and findings.**
- `fw/` has no disarm or motor-stop path until L8; those resets are modelled in tests.
- δ_max and the window bound are attained, so neither is loose.
- `marv_plant` has no fractional-command entry. The motor-speed reference is the affine combination of two plant runs,
  exact because the motor step is affine, and that affinity is asserted.
- Records left as records of their own runs: `ff_eval/`, `r2_ff_diagnosis/` and `L05/results/*`. The FF-on capture
  headers still say "stage (c)".
- The acro FF-on margin shrank by 0.35 mrad/s with the diffuser.
- AM32 applying each frame stays INFERRED until the bench.
- S9 is still unenforced for pytest.

**8. Next and carried.**
- The R2 yaw options for (e), from a scratch analysis on `4a76755`, before (d); the FF-on R2 run has since changed by
  +40 violations.
  - What saturates: only the yaw allocation, at executions 1–93 (0.3–29 ms; lowest t 0.761). The yaw integrator freeze
    is on over exactly those executions. Roll and pitch never saturate.
  - The w_z violations fall well after that window (executions 1564–3116 and 5768–5955).
  - The design model with allocation and freeze ("AF") reproduces the gz yaw excess in sign and timing, and about
    65 % of its size: +0.151 against +0.237 rad/s, with 1364 of gz's 1741 violations matched. The freeze alone
    explains it; the lost torque does not.
  - AF does not reproduce gz's roll/pitch rate violations (0 against 324/326). In the closed-loop tool, removing the
    freeze clears w_y, which suggests the yaw freeze leaks into roll and pitch (INFERRED).
  - The options:

    | Option | R2 FF on | Still proves | Stops proving |
    |---|---|---|---|
    | (a) allocation + freeze in the envelope | 2391 → 1182 | attitude; FF off is still caught; settle | that anti-windup stays inside the linear design response (the reference copies `record_allocation`) |
    | (b) R2 yaw-limited | w_z outside saturation: no change; dropping w_z: 650 (160 with (a)) | attitude and the roll/pitch rates | the yaw rate trajectory |
    | (c1) firmware anti-windup change (product, flight safety) | predicted 1253 | the check unchanged | — (windup risk under sustained saturation) |

  - None of them passes on its own. After (a), the rest is the FF form plus the rotor-speed lag (about the W4 tool's
    1178).
- Before (e): the acro margin under noise, from 5.72 mrad/s.
- The (e) sizing:
  - Seeds: tracking 22 (p = c = 0.90), safety 59 (p = c = 0.95).
  - Scenarios: 14 tracking and 4 safety (acro, R1, R2, R1X), so 52 nominal gz processes per push.
  - Measured gz wall per seed and corner (marv-ci-gz, 4 CPUs): tracking 194 s (L4 steps 3 × 4.8, L4 chirps 3 × 15.5,
    L5 steps 2 × 7.75, yaw 2 × 7.75, L5 chirps 39.9 / 40.0 / 22.3), safety 23 s (acro 5.5, recoveries 3 × 5.85).
  - Confirmation at nominal plus the worst corner: 2 × (22 × 194 + 59 × 23) ≈ 11,260 s ≈ 3.1 h serial gz time
    (1.6 h at nominal only), plus the T3 design time.
  - Turn-on corners (0012): the nominal plus each scenario's worst T3 corner. The metric that ranks "worst" is
    undefined, and no 64-corner T3 scan exists yet.
  - The CI split today: core, gz-l2, gz-l4 and gz-l5 run in parallel per push; nightly adds the 6 nightly tools files.
    There is no Monte Carlo hook yet.
- Pre-L8: J with σ, τ_m, AM32 per-frame behaviour, the prop-strike pass bar.
- Carried: S9 enforcement for pytest; the `step_cause` re-run at N = 1.

**9. For Luis.**
1. Accept the two-run superposition as the d̃ reference of the motor-speed test? **yes (recommended)** / no
2. Approve stage (d), and push `43a23a2` and this close? **yes (recommended)** / no
3. R2 for (e): **a (recommended)** / b / c1 / defer. With (a), the reference becomes the law Gazebo flies, your chirp
   principle. It still fails, at 1182 violations; the FF-form and rotor-lag residual then gets its own decision.
4. Give me read access to the Actions logs (`gh` installed and authenticated) to find the core-time cause?
   **yes (recommended)** / no
5. Rank the "worst T3 corner" by the smallest predicate slack per scenario? **yes (recommended)** / no

## Approval

Owner decisions: Luis, 2026-10-03, as quoted. The motor-speed line: Luis, 2026-10-04, with the four edits above.
P1 and P2 approved: Luis, 2026-10-04.

Stage (d) approved: Luis, 2026-10-04.

Luis, 2026-10-04, on the close's section 9 (verbatim, abridged to the rulings):

"**1. The motor-speed reference as "the plant's own motor model": yes.** The blend is exact because the motor step is
affine, and the test asserts that, with a control that fails by 4.8e6×.

**2. Stage (d): approved. Push 43a23a2 and the close.** ... The re-captured FF-on records' headers still say "stage (c)".
If that text is the generator's output, leave it and say so in the README.

**3. R2 for (e): defer, leaning toward c1. Not (a), not (b).** ... Not (a): as described, the reference copies
`record_allocation` from the run, so the envelope would come from the observed run, which we never do. ... The freeze is a
controller choice, not physics. ... A design model that computes its own allocation limits from its own state is
legitimate physics. ... Not (b): it turns off the yaw-rate trajectory on a safety check. ... Why defer: the model doesn't
reproduce Gazebo yet. ... The freeze is per axis (`rate_loop.hpp:217`), so a yaw saturation can't freeze the roll or
pitch integrators. Whatever drives those violations is missing from AF.

**4. Actions logs: not needed.** ... Every step slowed by about the same factor between af25f5e and 4a76755, including
steps 0016 doesn't touch ... So the jump is the runner, not the code: likely slower hosted-runner hardware (INFERRED).
... Actions times don't predict anything; keep measuring S9 in the local CI image, which is the rule anyway. Plan (e)'s CI
on the slower Actions figure.

**5. Worst T3 corner = smallest predicate slack per scenario: yes, normalised.** Per scenario, take the corner with the
smallest slack divided by that channel's own bound, minimised over channels so units compare. Keep both corners on a
tie. It is computed on the T3 design model, never picked from Gazebo results."

The R2 items (re-run on the post-(d) head, the reproduction gap, c1 as a design option, the FF-form and rotor-lag
residual), the acro margin under noise and the (e) decision round go to Luis together, with no repo writes until he
chooses.
