# 0029: T4 behaviour predicates (S, B, T) and `t4_settle_fraction`

## What changed

- `design/budget.yaml`: new entry `t4_settle_fraction` = 0.02 (Ogata, *Modern Control Engineering*, §5-3). It is a
  test-harness value; as with `per_push_check_time_max` and the other `t4_*` entries it is claimed in a layer manifest.
- `tests/regression/quad/L06/param_ids`: one line added, `t4_settle_fraction` (the L6 manifest holds the `t4_*` entries).
- `tests/regression/quad/L06/product_params/marv_params_product_table.txt`: regenerated in the CI image (101 to 102
  records, row 101 added, new schema hash). `tests/regression/quad/L06/results/ff_eval/{card_worst,sweep}.txt`: only the
  `design/budget.yaml` sha256 input line changes.
- `tools/sim/t4_rule.py`: adds `settles` (S), `bounded` (B) and `shrinks` (T) beside the L2 rule; nothing existing changes.
- `tests/regression/quad/L02/tools/test_t4_rule.py`: six known-answer tests appended; no existing test is touched.
  (The module and its test already existed, so the new cases extend them, core §7.7.)
- `docs/spec/10-quad-flight-software.md` §4.1: the T4 lines of L4, L5 and L11 and the L6 known-failing lines now say S/B
  (T for acro) with s = `t4_settle_fraction`, the loop-disabled control, chirp PM − U ≥ `PM_min`, no design-model
  envelope. R2 is no longer a known failing item; the L4 acro recovery is the one, owned by the deferred feed-forward.
- `ci/blast_radius.yaml` is unchanged: `t4_rule.py` and `tests/regression/quad/L02/tools/**` already select the L02 tools
  tests and the l2, l4, l5 and l11 gz groups.

Predicates (m = 1 run, E = |m1 − m2| per quantity, Δ = the largest |q − q_ref| in the segment among quantities of the
same unit): S, |q − q_ref| ≤ s·Δ + E at the segment's last execution; B, |q(n) − q_ref| ≤ |q(n0) − q_ref| + s·Δ + E at
every n of a segment without a disturbance onset; T (acro only), the stepped axis's error at the end is smaller than at
the start. The L4/L5/L11 gz tests, the fine controls and the L6 (e) study tests change in packets P1 to P4 (0030 to 0033).

## Why

Core §7.2 item 3: a metric test needs a control that breaks it, and the fine controls (gains × 1.1, one tick) compare
a gz run with a design-model envelope far inside what Gazebo's own error allows. Decision 0026 puts the merge run on
behaviour predicates with the loop-disabled control. The T3 goldens and QF-3 margins keep the fine sensitivity.

## Evidence

- R2 recovers in gz: final α 8.1e-4 rad (`tests/regression/quad/L05/results/recovery_cause/cause.txt:35`), so its strict
  xfail goes.
- The L4 acro recovery fails B: roll 0.09 to 10.29 rad/s with the sticks centred
  (`tests/regression/quad/L04/results/acro_cause/cause.txt:31`), so it stays the one strict xfail.
- T3 coverage that stays: goldens L4T3Step, L5T3Step, L11T3Step; QF-3 margins (`test_rate_design.py`,
  `test_attitude_design.py`, `test_vertical_design.py`); lock logic L5Angle, L5T3Envelope; alignment L4CompositionScript,
  L5CompositionSchedule, RateGroups. Gap: large-angle recovery has no T3 check.
- The new known-answer tests pass (`uv run pytest tests/regression/quad/L02/tools/test_t4_rule.py`); they need no
  negative control (core §7.2 item 3).

## Approval

Luis, 2026-10-15, "1,2,3 all yes" to: (1) `t4_settle_fraction` = 0.02, the 2 % settling criterion (Ogata, §5-3, which
also gives 5 %); (2) the L4 acro recovery stays the only known failing item, owned by the deferred L6 feed-forward;
(3) the deletions and amending the L4, L5, L11 and L6 T4 pass-bar lines in quad §4.1.
