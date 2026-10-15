# 0025: the R2 package ruled (quad L5 R2, L6 stage (e))

## Owner decision (Luis, 2026-10-15, verbatim)

"keep your recommendations for R2 just so we can continue"

The recommendations he accepted are the lead's of 2026-10-15, as put to him (questions Q0–Q5 below). Questions 6–8
of the same message (noise weight W per point or constant, the gz re-measure, C6a) were not ruled and stay open.

## What changed

No frozen file changed, deleted or renamed, and no code changed. This record states the rulings that govern the next
L6 stage (e) work; each takes effect in the change that builds it, with its own record.

- **Q0, how R2 is judged: (a), a safety predicate.** R2 (the L5 large-angle recovery from a steady tumble at
  `rate_max` on all three axes, inverted; decision 0015) will assert the recovery itself: back to level with the body
  rates inside bounds by a deadline derived from the card, no motor below idle, no divergence. The comparison against
  the design-model envelope (E + F + Q, today's predicate, tests/regression/quad/L05/gz/test_t4_recovery.py:203-221)
  becomes run-and-report, as the prop-strike start already is (decision 0015). Before anything is built, the architect
  designs the predicate, its deadline rule and its negative control (test-harness semantics), and the replacement for
  the quad spec's L6 (e) line "Both known failing items pass" is quoted in full to Luis for approval. Until then R2
  stays a strict xfail and `tests/regression/quad/L06/XFAIL_GATE_CLOSED` is not created.
  **This loosens a pass-bar line** (core §7.3); it is Luis's ruling, made with that stated.
- **Q1, anti-windup: freeze.** The per-axis clamping freeze (decision 0005) stays the rate loop's law.
- **Q2, may the allocator raise the collective in a recovery: no.** The thrust request in a recovery stays the hover
  request (decision 0015); L3's collective keeps the lowest priority.
- **Q3, candidate (i), the rotor-lag-inverting allocation, as the study's base: no.** No spec change.
- **Q4, commit the R2 in-range witness: dropped.** The witness no longer exists (Evidence).
- **Q5, branch `l6-0018-c1`: delete** once this record carries c1's numbers (it does, under Evidence). The branch head is
  `7ea4d298837009db7446e5c0980e622d61d6e7f6`.

## Why

Every controller variant measured fails R2 by a wide margin, so the check, not the controller, was the question.
R2 compares a saturated, nonlinear manoeuvre against a linear design-model envelope at every execution; the question R2
exists for is "whether the vehicle's rotation recovers" (decision 0019, line 21). With Q0 (a), Q2 and Q3 lose their
purpose: both were ways of making the envelope comparison pass. Q1: c1 trades a recorded flight-safety regression (yaw
trim loss) for a fraction of the violations and still fails.

## Evidence

- R2 today, FF on: 2431 violations; FF off: 15206 (tests/regression/quad/L06/results/ff_on/README.md:79).
- c1, back-calculation anti-windup with T_t = T_i (branch `l6-0018-c1`, commit `7ea4d29`, its message): R2 2431 → 1617
  violations; it loses 91 % of a yaw trim within one T_i under sustained roll saturation (99.8 % by release), where the
  freeze loses none; the L5 rate-bypass golden (pinned to quad-L4-pass) fails on 502 of 512 rows under it.
- Removing the allocator limit: 1178 violations (tests/regression/quad/L06/results/r2_ff_diagnosis/README.md:29). Ideal
  FF and no saturation: 423 (one nominal run, INFERRED from h_more2.txt:1).
- Candidate (i) with FF on and no anti-windup: a floor of 1072 violations; the witness's ΣT peak 17.2 N. These two
  numbers survive only as lines of docs/handoff.md ("Open items"); see the next item.
- **Lost evidence.** The R2 study (candidates (i)–(iii)), the anti-windup variants V1–V3, the in-range witness, the c1
  raw logs and the stage (e) noise-suite memo were kept outside the repository in `~/marv-e-round-2026-10-04/` and
  `~/marv-r2yaw-2026-10-04/` (docs/handoff.md, "Saved outside the repo"). On 2026-10-15 neither directory exists and a
  search of the host's file system finds none of their archives. Nothing in the committed records depends on them;
  any later work that needs them regenerates them.

## Approval

Luis, 2026-10-15, in session (quoted above).
