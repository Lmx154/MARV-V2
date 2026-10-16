# 0031: L5 T4 gz tests on the behaviour predicates, and the quantity roles

## What changed

Packet P2 of decision 0029.

- `tests/regression/quad/L05/gz/test_t4_steps.py`: S and B on roll, pitch and heading, step and release segments.
- `tests/regression/quad/L05/gz/test_t4_yaw.py`: S and B on yaw rate, tilt and heading relative to the lock; the lock-by-crossing item (release) and the
  no-crossing-before-fallback and lock-is-the-fallback items stay.
- `tests/regression/quad/L05/gz/test_t4_recovery.py`: S and B on α, |ω| and heading (R1X: α and |ω|, its heading is chosen by rounding noise). R2's strict
  xfail is gone. R1X's run passes, so it is a normal test.
- `tests/regression/quad/L05/gz/test_t4_chirp.py`: PM − U ≥ `PM_min` and the duration check stay.
- One control per test, `att_kp = 0`: steps fail S (1.047 against 0.021 rad); yaw fails S on the heading (release 1.252 against
  0.026 rad, fallback 0.513 against 0.011 rad); R1, R2, R1X fail S on α (3.132, 2.686, 3.142 rad against 0.063); chirp has no
  crossover.
- `tools/sim/t4_rule.py` adds the roles and `scenario_checks`; seven known-answer tests are appended to
  `tests/regression/quad/L02/tools/test_t4_rule.py` (`settles`, `bounded` unchanged; L4 unchanged).
- The `end_attitude_execution` rule text of the four `scenarios/quad/L05/recover_*.yaml`; quad §4.1 L5 pass bar.

## Roles (the loosening)

B applies only to what a scenario steps or holds. A role is static, from the scenario file and the firmware logic, never run
data. STEPPED: error nonzero at n0. HELD: constant reference. DRIVEN: error 0 at n0 by construction, set moving by the scenario.
S applies to every quantity; B to STEPPED and HELD quantities of a segment without a disturbance onset. Where every quantity of
a unit in a segment is HELD, Δ is the largest Δ of that unit over the scenario's segments with a STEPPED or DRIVEN quantity of
it (ValueError if none).

Plainly, three B checks are no longer applied: the heading after the yaw lock (DRIVEN, 0.6157 against 0.0128 rad before), R1 and
R1X |ω| (DRIVEN, from rest; 6.59 and 6.67 against 0.14 rad/s), R2 heading (DRIVEN; 1.596 against 0.063 rad). And the yaw tilt in
the hold (HELD, 1.3e-14 rad, round-off) takes the release's Δ (≈ 0.6 rad, bound 0.012). Those quantities keep S.

## Removed, and what still covers it

| Removed | Still covered by |
| --- | --- |
| Recorded T3 envelope bands (all four files), `Q`, `F` terms | goldens L5T3Step, L5T3Envelope; QF-3 margins `test_attitude_design.py` |
| Alignment tests and their ±1 controls (steps, yaw) | L5CompositionSchedule; `test_l5_scenario.py` binds release and end to the envelope header |
| Fine controls: × 1.1 (steps, recovery), × c (chirp), `t_cross_run`, planted heading-back, planted held trace and planted α | the `att_kp = 0` controls; golden L5T3Step |
| Chirp "inside the design range ± U" half | `test_attitude_design.py` PM margins over the box |
| Yaw: recorded lock range, `heading_release` band | L5Angle (T1), L5T3Envelope |
| Recovery design-model envelope (`make_design`, `test_design_envelope_has_settled`) | none at T3: the stated gap, large-angle recovery |
| Recovery: the build's parameters equal the recorded T3 inputs (`make_design`, `recorded_inputs`) | not needed: it tied the run to the design envelope, which is gone; steps and yaw keep their live-parameter checks |
| Yaw tilt falsifiability by `att_kp = 0` (the vehicle stays level) | the known-answer cases and L3MixerIdentity (`mixer_test.cpp:147`) |

## Why

Core §7.2 items 3 and 6 and decision 0029. B as 0029 wrote it compares each error with its value at the segment's
start, which fits a step; it misfires where the error is zero at the start by construction and the scenario sets the
quantity moving, and where a quantity is round-off only (architect ruling, 2026-10-16).

## Open risks

The heading bounce after the yaw lock has no T4 bound (the T3 golden L5T3Envelope keeps the design bound). The R1X |ω| transient
is unbounded at T4.

## Evidence scripts that no longer run

Their imports from the L5 tests (`make_design`, `parse_envelope`, `alignment_findings`, the old `evaluate`, `series`, `fly`)
are gone: `tests/regression/quad/L05/results/{step_cause,step_controls,recovery_cause}`; the L06 results `ff_on/capture_gz.py`,
`r1x_coupling/{r1x_coupling,r1x_model,capture_gz}.py`, `r2_envelope_a`, `r2_lower_bound/{r2_lower_bound,measure_e,hover_tumble}.py`
and `r2_ff_diagnosis/{b_ladder,c_wz,h_more,h_more2}.py`. They are evidence only; the L06 study tests go with decision 0033.

## Evidence

Sequential runs on this change: `test_t4_steps.py` 6 passed, `test_t4_yaw.py` 7 passed, `test_t4_recovery.py` 12 passed,
`test_t4_chirp.py` 15 passed; `test_t4_rule.py` 35 passed; L4 `test_t4_acro.py` 6 passed, 1 xfailed, `test_t4_steps.py` 9
passed (L4 unchanged); `tests/regression/quad/L05/tools` 140 passed. Control values above.

## Approval

Luis, 2026-10-15, "1,2,3 all yes" (decision 0029). On the roles, Luis, 2026-10-16: "yes" (to: "B now applies only to what a
scenario steps or holds. Recovery rates from rest, R2's heading and the heading after the yaw lock must settle instead, tilt
during the yaw hold is checked against the scenario's angle motion, and L4 is unchanged.").
