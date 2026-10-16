# 0030: L4 T4 gz tests on the behaviour predicates

## What changed

Packet P1 of decision 0029, three modified files:

- `tests/regression/quad/L04/gz/test_t4_steps.py`: `test_step_meets_qf2_envelope` becomes `test_step_settles_and_stays_bounded`
  (S and B on the three gyro axes of the m = 1 run, E = |m1 − m2|, deadline the scenario's horizon, 10 τ_ref of QF-2).
- `tests/regression/quad/L04/gz/test_t4_chirp.py`: PM − U ≥ PM_min (U = E_H + U_A + U_d) and the duration check stay.
- `tests/regression/quad/L04/gz/test_t4_acro.py`: (i) clean, (ii) DShot range, (iv) replay fidelity and (v) no saturation
  stay; (iii) is B on the commanded axis (nonzero stick) of every linear segment and T on every single-axis one. The
  strict xfail `test_acro_recovers_after_the_combined_segment` becomes `test_acro_rejects_cross_axis_coupling`: B on the
  uncommanded axes of every linear segment and B on the recovery window, one known failing item, reason text naming
  decisions 0029 and 0030 and the deferred L6 feed-forward.
- `docs/spec/10-quad-flight-software.md` §4.1 L4 T4 and L6 lines: the known failing item is the acro cross-axis
  coupling (linear segments and recovery); the steps control fails S (not B).
- `docs/decisions/0030-l4-t4-predicates.md`: this record.

Removed, and what still covers it (decision 0029, invariant 4):

| Removed | Still covered by |
| --- | --- |
| steps: design-model envelope comparison (`oracle_envelope`, `Envelope`, `evaluate` on lo/hi) | golden L4T3Step; QF-3 margins over the box in `tests/regression/quad/L04/tools/test_rate_design.py` |
| steps: `test_alignment_is_exact`, `test_control_alignment_off_by_one_execution_fails` (±1) | L4CompositionScript (a segment acts from its stamp, not before) and L4CompositionWrites (DShot only on due ticks); the n_exec assertion stays in the live-parameters test |
| chirp: "inside the design range ± U" half, `control_k`, `test_control_gains_times_k_fails_the_predicate` | `test_rate_design.py` PM margins over the box; golden L4T3Step |
| acro: (iii) `rate_bound_evaluation` envelope use, `test_control_double_rate_segment_fails_the_rate_bound` | golden L4T3Step; `test_rate_design.py`; the loop-disabled control below |
| acro: `test_control_replay_of_a_one_ulp_thrust_change_does_not_reproduce_the_run` | nothing at T4: (iv) is an exact-equality plumbing check, its fine control goes |
| acro: `test_control_recovery_check_passes_rest_and_fails_a_stuck_rate` (Z envelope) | B's known-answer cases in `tests/regression/quad/L02/tools/test_t4_rule.py` |
| acro: a passing bound on the uncommanded axes of the linear segments (the old envelope held them) | nothing passing: the strict xfail `test_acro_rejects_cross_axis_coupling` turns red when the feed-forward fixes it; the M4 flight test card puts gentle acro before full-stick acro |

One control per metric test, the loop disabled: kp = ki = 0 on the stepped axis (steps), on the chirp axis (chirp, the
measured loop has no unique crossover, |L| < 1 over the band, so the PM check fails), on all three axes (acro, T fails).
`design_bound`, `run_params`, `s_t_findings` and their constants stay in `test_t4_acro.py`: the evidence scripts
`tests/regression/quad/L04/results/acro_cause/acro_cause.py` and `tests/regression/quad/L06/results/ff_on/capture_gz.py` import them.

## Why

Core §7.2 item 3 and decision 0029: the fine controls compare a gz run with a design-model envelope far inside Gazebo's own
error, and the T3 goldens and QF-3 margins keep the fine sensitivity.

## Evidence

Steps (`uv run pytest tests/regression/quad/L04/gz/test_t4_steps.py`): S on the stepped axis 1.8e-2 against a bound of
2.3e-1 rad/s on all three axes; the loop-disabled run fails S (error 11.7 rad/s) and passes B (nothing moves). Chirp roll:
PM 81.45°, slack 36.4°; the loop-disabled run has |L| = 0.32 at the band edge and no crossover. Acro: T holds on the four
single-axis segments, the loop-disabled run fails T (error 11.69 rad/s at start and end); B holds on the commanded axis
of every linear segment. B fails on uncommanded axes, all with that axis's stick centred: segment 3 roll 0.815 against
0.265 and yaw 0.337 against 0.261, segment 4 roll 0.311 against 0.256, segment 5 roll 0.992 against 0.258 and pitch
0.687 against 0.258 (rad/s); the recovery window roll 3.55 against 0.29. The cause is gyroscopic coupling the per-axis PI
does not reject, as in the recovery; for segment 3 roll, the axis released from the reversal, its own lag is not
separated from the coupling (UNKNOWN). `tests/regression/quad/L06/results/ff_on/capture_gz.py` no longer runs (the
`fly` signature changed); it is evidence for the L6 (e) study that decision 0033 deletes.

## Approval

Luis, 2026-10-15, "1,2,3 all yes" (decision 0029). On folding the uncommanded-axis B of the linear segments into the
known failing item (option A: one strict xfail for the acro cross-axis coupling, linear segments and recovery, owned by
the deferred feed-forward), Luis, 2026-10-16: "A".
