# 0009: quad L6 staging, the known-failing gate and the L6 choices

This record opens L6. Luis's messages below call it "0008"; the old L5 session used that number first (decision 0008,
rate-bypass regenerate safe.directory), so this record is 0009. The quotes are left verbatim.

This record opens L6. It holds Luis's L6 decisions verbatim, the lead's choices he approved, the spec and register
changes, and the one frozen-test change they need. Later L6 stages append their own decisions here (or in their own
records, where a decision says so).

## What changed

**Frozen files changed.** Two, the same one-line change in each: the strict-xfail condition of the known failing items.
- **`tests/regression/quad/L04/gz/test_t4_acro.py`** (the L4 acro combined-segment recovery, decision 0005 owner
  decision 12).
- **`tests/regression/quad/L05/gz/test_t4_recovery.py`** (the L5 R2 recovery, decision 0006 owner decision 15).

In each, the condition `not L06.exists()`, with `L06 = tests/regression/quad/L06`, becomes `not XFAIL_GATE.exists()`, with
`XFAIL_GATE = tests/regression/quad/L06/XFAIL_GATE_CLOSED`. The docstring and reason text are updated to name the gate
file. Nothing else changes: the predicates, bounds, envelopes, `strict=True` and `raises=AssertionError` are as they
were.

**Spec (quad, ACTIVE; Luis approved the text, 2026-09-30).**
- §4 L6 is rewritten. It adds to Builds the vibration model (B1 fallback), the eRPM path, the D term, the ω×Jω
  evaluation and DShot error diffusion. It adds stages (a)–(e), each with its lines of the pass bar, and the gate
  file.
- §6.2 QF-3 adds "sensitivity peak M_s ≤ `Ms_max`".
- §6.2 QF-7 says which harmonics the notches follow until a vibration spectrum is measured.

**Register (`design/budget.yaml`).** New entries: `Ms_max` 2.0, `d_path_noise_budget` 0.5,
`t4_pass_probability_tracking` 0.90, `t4_confidence_tracking` 0.90, `t4_pass_probability_safety` 0.95,
`t4_confidence_safety` 0.95 and `allan_check_confidence` 0.99. Luis named two entries, `t4_pass_probability` and
`t4_confidence`, with one value per class. A register entry holds one scalar, so each is split by class suffix.

**New manifest.** `tests/regression/quad/L06/param_ids` lists the seven entries. flatten.py emits every numeric
register entry, and decision 0004 requires each emitted id to be claimed by a manifest.

## Why

- **The gate.** L6 is built and closed in stages (Luis, decision 3 below). Stage (a) is the first to commit anything
  under `tests/regression/quad/L06/`, and the new manifest above does so now. Under the old condition that would turn
  both known failing items into normal tests from stage (a) onward, and CI would stay red until the D term and ω×Jω
  work in stage (c). The known failing items themselves are unchanged: they still block L6 (stage (e) creates the gate
  file and they must pass) and must pass before L8. Nothing is loosened: the bar, the envelopes and the strictness
  are the same, and only the trigger moves from "any L6 file exists" to "L6 declares it is closing".
- **The spec lines.** Core §7.2 requires the pass bar to be written before the layer is built. The D term, ω×Jω and
  DShot error diffusion were L6 work by Luis's instruction (handoff, 2026-09-30) and decision 0005, not by the spec
  text.

## Owner decisions (Luis, 2026-09-30, verbatim)

First message (on the lead's proposals for items 1–5 of the handoff):

> Decisions:
>
> 1. I'll confirm Actions on e9bfaae; tag when I say so.
> 2. Storage: generator + inputs + SHA-256, as you propose. No LFS. Removal of
>    the committed files gets its own decision record; no history rewrite.
> 3. Gate file: approved (decision 0008). The extra L6 pass-bar lines go into
>    the quad spec's L6 section. Draft them for my approval before stage (a)
>    builds.
> 4. IMU values: don't block on a recording. Datasheet values where they exist
>    (UNVERIFIED); labelled scenario values for the missing ones (bias
>    instability, random walk, quantisation, latency, ODR error), to be
>    replaced by my still-bench dataset later. Stage (a)'s Allan check tests
>    model-vs-profile consistency, so this is valid.
>    Vibration: yes, use the B1 fallback (amplitude sweep, flagged unsourced).
>    Notches at the harmonics the vibration model includes (1×/2×/3× rotor
>    frequency per the B1 table).
> 5. D1–D7 and F1–F3: all as you recommend. Register values:
>    - Ms_max = 2.0 (rationale: Åström & Murray, Feedback Systems: Ms ≤ 2
>      guarantees gain margin ≥ 6 dB and phase margin ≥ 29°; our PM_min is
>      stricter on phase, Ms covers gain).
>    - D-path noise budget = RMS torque noise per motor ≤ 0.5 DShot step at
>      hover (rationale: noise below actuator resolution; design choice, revisit
>      with bench motor-temperature data).
> 6. New register entries (not p): t4_pass_probability and t4_confidence.
>    Tracking scenarios 0.90/0.90 (22 seeds); safety scenarios (acro-coupling,
>    recoveries) 0.95/0.95 (59 seeds). Heavy runs at T3; propose the T4
>    confirmation seed count with its cost.
> 7. CI split as you propose: per push = run_ci.sh + frozen T4 at committed
>    seeds; nightly + before any tag = multi-seed Monte Carlo; tag only after
>    multi-seed green. Split gz_l4/gz_l5 into parallel jobs. Send the final
>    version with measured times.
>
> Start with the spec draft (item 3), then stage (a).

Second message (on the spec draft):

> Approved: apply the L6 spec draft and write 0008.
> 1. allan_check_confidence = 0.99.
> 2. Both interpretations confirmed: D-path RMS contribution per motor ≤ 0.5 ×
>    the hover DShot step thrust (≈ 2.28 mN); chirp margins are tracking
>    (0.90/0.90).
> 3. Fine, send the T4 seed count with the final CI split.
> 4. If the derived Allan record length makes the bias-instability T1 check
>    slow, run it nightly instead of per push and say so in the spec line.
>
> Coordination: the other session is the old L5 lead. Let it finish verifying
> d7a044c as root and push it; then I'm closing it. After that, you are the
> only session that writes to the repo. Tag quad-L5-pass on the first commit
> that's green in Actions.
>
> Also: make local CI run as root the way Actions does (or make Actions match
> local), so a local pass means the same thing as an Actions pass. Include it
> with the CI split.

## Lead decisions (approved by Luis as recommended, owner decision 5)

The rate loop's D term (stage (c)):

| # | Choice | Decision |
| --- | --- | --- |
| D1 | What D differentiates | The measurement, as today (`rate_loop.hpp:244`; decision 0005). No setpoint kick. |
| D2 | D's input | The gyro chain's output through a first-order D low-pass, discretised like the prefilter (1 − e^(−dt/T_f)). |
| D3 | Gain rule | `rate.py`'s sup rule on crossover kept, with the controller written as PI × lead and the lead's peak phase centred on crossover. The lead ratio N comes from the noise budget. Not a numerical optimisation (opaque, several optima). Not a copied D/P ratio (typed gains). |
| D4 | Cap on N and crossover | `d_path_noise_budget` (noise) and `Ms_max` (gain margin), plus the existing ω_c²·T·τ_lo < ½ proof condition and QF-8. |
| D5 | Yaw | The same rule on every axis; no hand-typed exception. |
| D6 | Dynamic D, setpoint feed-forward | Not at L6. The τ_ref prefilter tightens by itself as crossover rises. |
| D7 | Attitude gains | `attitude.py` designs on the closed rate loop including D and the chain; L5 regenerates by rule (0006 line 57). |

ω×Jω feed-forward (stage (c)):

| # | Choice | Decision |
| --- | --- | --- |
| F1 | Include it? | Decided by an evaluation on the design model with ω×Jω in the plant, comparing PID and PID + feed-forward on the L4 acro and L5 R2 envelopes. |
| F2 | Input | Filtered measured ω. In R2 the rate setpoint is about 0 while ω is at rate_max, so a setpoint-based term does nothing. Noise is small because the term is quadratic and ω ≈ 0 at hover. |
| F3 | J | The card's diagonal J (`inertia_xx/yy/zz`, new firmware parameter reads, claimed by a manifest). The error from J's ±50 % band is checked at the corners; J's σ remains UNKNOWN. |

## Citation corrected after approval

The approved draft cited IEEE Std 952-1997 Annex C for the Allan bound. That standard could not be read (paywalled), so
the spec line and the `allan_check_confidence` rationale cite NIST SP 1065 (Riley 2008) instead: §5.3.2 eq. 45
(chi-square interval) and §5.4.1 Table 5 (overlapping-AVAR equivalent degrees of freedom), which were read. The rule and
the value are unchanged.

## Spec gaps logged

- The quad spec's §4 L6 did not list the D term, ω×Jω or DShot error diffusion, which decision 0005 and Luis's handoff
  instruction assign to L6. Fixed by the rewrite above.
- Core §5 says profiles are fitted from Luis's measured datasets. Until his still-bench dataset exists, stage (a) uses
  datasheet values (UNVERIFIED) and labelled scenario values (owner decision 4). The Allan check then tests
  model-vs-profile consistency, not the profile's truth.
- QF-7 assumed a measured vibration spectrum. None exists, so the B1 amplitude-sweep fallback stands in, flagged
  unsourced (owner decision 4).

## Evidence

- Both gated tests still xfail strictly with the gate file absent: the full `ci/run_ci_gz.sh` run on this change (see
  the commit that adds this record).
- The gate-file negative control is stage (e)'s: creating the file must turn both into normal tests.

## Approval

Luis, 2026-09-30, in the conversation quoted above ("Approved: apply the L6 spec draft and write 0008").
