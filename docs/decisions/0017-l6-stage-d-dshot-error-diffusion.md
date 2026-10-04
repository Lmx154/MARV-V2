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

## Evidence

P1:
- ctest 656/656 in host-debug and host-release.
- m33 builds with 0 warnings.
- G1 is clean (`lint_g1.py` in marv-ci); `check_constants.py` passes.
- Independent review: PASS.

## Still to come in stage (d)

- **P2, the atomic live switch.** The diffuser goes into the compositions' rate group, with init reset. Then:
  - the frozen consumers change: the composition tests, the replays, the L05 T3 Q oracle, `recovery_model.py` and
    `step_cause.py`, each listed here;
  - Q is recomputed with the diffuser, and the fine controls are re-confirmed (decision 24);
  - if a strict xfail passes, stop and report.
- The motor-speed T1 test, once Luis approves the text above.
- The evaluation and the stage close.

## Approval

Owner decisions: Luis, 2026-10-03, as quoted. The motor-speed line text, P1 and the stage: pending.
