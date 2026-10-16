# 0032: L11 T4 gz tests on the behaviour predicates

## What changed

Packet P3 of decision 0029, with the quantity roles of decision 0031.

- `tests/regression/quad/L11/gz/test_t4_vertical.py`: S on z in all three scenarios; B on the height step; the thrust offset and
  the roll shove are S only (a disturbance onset). Roles, static from the scenario file: z is STEPPED in `height_step` (a
  segment, a reference change) and DRIVEN in `thrust_offset` and `roll_shove` (error 0 at the onset, the disturbance sets it
  moving). Uses `t4_rule.scenario_checks`; the deadline is the end of the run, the T3 golden's `segment_executions` (45482).
- The binding is re-pointed from the envelope header to the golden: end execution of all three scenarios = the golden header's
  `segment_executions`; step, z_ref and thrust disturbance of `height_step` and `thrust_offset` = the golden's blocks; the roll
  shove (no golden block) starts at the golden's `step_execution`, its `t0_us` derived from the recorded inputs. Added control:
  a moved deadline is found. The torque pulse's axis, torque and duration lost their cross-check (they were tied to the envelope
  header, which copied the oracle's `SHOVE_*`); the scenario file is their only record now.
- One control per test, the loop disabled (`vert_kp_z`, `vert_kp_v`, `vert_ki_v` = 0), now on all three scenarios.
- `tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py` no longer writes `vertical_t3_envelope.txt` or
  `vertical_t3_q.txt` (`main` writes the golden; `--golden-only` is gone); the envelope, shove and Q code stays in the file,
  unreferenced. `tests/regression/quad/L11/t3/reference/SHA256SUMS` drops the two lines; the golden's hash is unchanged
  (`refdata ensure quad/L11/t3` regenerated and verified it).
- `ci/run_ci.sh` (`--golden-only`, comments, the reproduction step text), `ci/run_ci_gz.sh` (a comment), the `end_vertical_execution`
  rule texts and header comments of `scenarios/quad/L11/{height_step,thrust_offset,roll_shove}.yaml`, quad §4.1 L11 T4 (cites 0032).

## Removed, and what still covers it

| Removed | Still covered by |
| --- | --- |
| Recorded T3 envelope band (lo/hi per execution), `F` and `Q` terms | golden L11T3Step (`tests/regression/quad/L11/t3/t3_test.cpp`); QF-3 margins `tests/regression/quad/L11/tools/test_vertical_design.py` |
| Fine control: `vert_kp_z` × 1.1 on all three scripts | golden L11T3Step; `test_vertical_design.py`; the loop-disabled controls |
| Loop-disabled control on `thrust_offset`, `roll_shove` fails the band | kept as S, and added for `height_step` |
| Torque pulse axis, torque, duration tie to the envelope header | none; the scenario file is the record (stated gap) |

## Why

Core §7.2 item 3 and decision 0029: the band compares a gz run with a design model far inside Gazebo's own error; T3 keeps the
fine sensitivity.

## Evidence

`uv run pytest tests/regression/quad/L11/gz/test_t4_vertical.py -rA -q`: 7 passed in 76 s (eight gz sequences before, six now).
S on z, value against bound (m): height step 0 against 2.0e-2 (B 0 against 2.01e-2); thrust offset 9.5e-7 against 1.9e-4; roll
shove 5.5e-10 against 1.5e-5. Loop disabled: height step fails S, 1.000 against 2.0e-2 (B passes, 1.1e-4: nothing moves);
thrust offset 25.5 against 0.511; roll shove 0.180 against 3.7e-3.

## Approval

Luis, 2026-10-15, "1,2,3 all yes" (decision 0029). On the roles, Luis, 2026-10-16: "yes" (decision 0031). Files under
`tests/regression/` changed here: `tests/regression/quad/L11/gz/test_t4_vertical.py`,
`tests/regression/quad/L11/t3/reference/vertical_t3_oracle.py`, `tests/regression/quad/L11/t3/reference/SHA256SUMS`; none deleted.
