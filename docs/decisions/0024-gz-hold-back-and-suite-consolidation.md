# 0024: the long Gazebo T4 files held back from the development check; three duplicated test groups consolidated

The open trims (2) and (3) of the handoff (decision 0021 follow-up). Nothing is loosened: every check that ran before
still runs in the full run at the merge, and every removed test case is covered by an equivalent case on the same
input.

## Owner decisions (Luis, 2026-10-15, verbatim)

Choosing among (A) finish L11, (B) L6 stage (e), (C) the open test trims: "C. Continue the test trims".

Multiple-choice rulings on the lead's questions, same day, the labels he chose:
- Trim (2): "L5 + L4 chirp (Recommended)": hold back L5 chirp/yaw/recovery and L4 chirp from the development check,
  running them only at the merge or when the file itself changes; core §7.6 gets one sentence naming the exception.
- Truth-channel suites, first round: "One body, both channels (Recommended)". The lead then found that the two SIL
  libraries are shared libraries exporting the same `marv_sil_*` symbols, so one executable cannot link both; second
  round: "Shared header, 2 binaries (Recommended)": helpers and shared bodies in one header, 41 entries stay.
- Scenario schemas and param scripts: "Both (Recommended)".

## What changed

### Trim (2): the development check holds back four Gazebo files (core §7.6, one sentence added)

- `ci/run_ci_gz.sh`: one list of held-back files; outside `MARV_CI_MODE=full`, `gz_l5` and `gz_l4` pass them to pytest
  as `--ignore`, except a file the diff against the development check's base touches, or whose directory has a changed
  non-`test_*.py` file (conftest, helpers); with no base available it runs everything. Full mode (Actions,
  `ci/local_ci.sh`) is unchanged. `ci/dev_check.sh`: passes the base through.
- Held back: `tests/regression/quad/L05/gz/test_t4_chirp.py`, `tests/regression/quad/L05/gz/test_t4_yaw.py`,
  `tests/regression/quad/L05/gz/test_t4_recovery.py`, `tests/regression/quad/L04/gz/test_t4_chirp.py`. None of these
  files changed.
- `docs/spec/00-core-contracts.md` §7.6 item 2: the exception sentence.

### Trim (3a): truth-channel T1 suites share one header

- `tests/regression/quad/common/truth_channel_t1.hpp` (new): the shared helpers and the 19 shared tests as templates
  over a channel traits type, registered by a macro so the ctest names stay `TruthState.<Name>` and
  `TruthVertical.<Name>`.
- `tests/regression/quad/L05/unit/truth_state/truth_state_test.cpp`: the `Attitude` traits, the macro line and the two
  L05-only tests (zero quaternion, tiny quaternion). 418 → 110 lines.
- `tests/regression/quad/L11/unit/truth_vertical/truth_vertical_test.cpp`: the `Vertical` traits, the macro line and
  the L11-only test (valid bit set with zero fields). 393 → 84 lines.
- `tests/regression/quad/L05/unit/truth_state/CMakeLists.txt`,
  `tests/regression/quad/L11/unit/truth_vertical/CMakeLists.txt`: the include directory of the header.
- Where the two suites iterated different input values, the shared body uses the union, so L05 gained denorm_min,
  -Inf and -NaN cases (all pass). 41 ctest entries before and after, names identical.
- Decision 0020's citation "L05 truth_state_test.cpp:237, 252" (the reserved-flag-bit loop and the field loop) now
  lives in `EInputForAReservedFlagBit` and `EInputForAnyNonzeroFieldWithTheValidBitClear` of the header, with the L05
  counts in `truth_state_test.cpp`'s `Attitude` traits.

### Trim (3b): the shared scenario-schema refusal cases in one file

- `tests/regression/quad/L04/tools/test_scenario_schema.py` (new, 29 tests): one test per (case family, schema),
  looping over the cases and reporting every one that does not refuse as expected. It lives in L04/tools so it runs in
  the core tools step (L02/tools runs only in the gz l2 group).
- `tests/regression/quad/L02/tools/test_scenario.py` (62 → 35), `tests/regression/quad/L04/tools/test_l4_scenario.py`
  (68 → 47), `tests/regression/quad/L05/tools/test_l5_scenario.py` (52 → 43): the cases now in the shared file are
  removed. A (schema, case) pair is in the shared file only if that schema had it before; no case was widened to a
  schema that did not have it. Total 182 → 154.
- Two L04 tests lost their schema half and were renamed: `test_unknown_fields_and_stem_mismatch_refused` →
  `test_unknown_step_field_refused`, `test_tick_must_be_the_registers_and_derived` → `test_tick_must_be_derived`.
- Found defect, fixed (§7.7): the old L02 and L04 duplicate-key tests passed with the loader's duplicate-key guard
  removed (the duplicated mapping replaced the entry and the document failed for another reason). The shared test
  writes the committed file twice, with an "unmodified copy loads" control, and fails when the guard is broken.
- Kept untouched: the decision 0014 controls and literals in `test_l5_scenario.py`; the assertion decision 0015 cites
  ("line 315", already stale at 320) is now at lines 297–298.

### Trim (3c): one param-ids script

- `tools/ci/check_param_ids.py` (new): the L11 script with the prefix list optional; without prefixes it checks what
  the old `check_param_prefix.py` checked, with the same exit codes.
- Deleted: `tests/regression/quad/L04/unit/composition/check_param_prefix.py`,
  `tests/regression/quad/L05/unit/composition/check_param_prefix.py` (byte-identical),
  `tests/regression/quad/L11/unit/composition/check_param_ids.py`.
- `tests/regression/quad/L04/unit/composition/CMakeLists.txt`,
  `tests/regression/quad/L05/unit/composition/CMakeLists.txt`,
  `tests/regression/quad/L11/unit/composition/CMakeLists.txt`: point at the new script. The 7 entries (3 positive, 4
  WILL_FAIL controls) keep their names, arguments and labels; the smoke's `l11_composition_param_ids` is unchanged.

### Map

`ci/blast_radius.yaml`: rules for `tests/regression/quad/common/**` (L05, L11 ctest), `tools/ci/check_param_ids.py`
(L04, L05, L11 ctest) and `tools/sim/l5_scenario.py` → `tests/regression/quad/L04/tools`.

## Why

Luis's redirect of 2026-10-14 (decision 0021) and his rulings above. Trim (2): the L5 Gazebo test step of a
development check is set by its slowest file under `--dist loadfile`; measured serially on the host on 2026-10-15
(load < 1.5): L5 steps 69 s, chirp 250 s, yaw 70 s, recovery 104 s, truth_plumbing 13 s; L4 steps 44 s, chirp 101 s,
acro 29 s, truth_gyro 4 s. Holding back the four files takes the L5 test step from about 250 s to about 70 s and L4's
from about 100 s to about 45 s. Trim (3): duplicated code and cases (core §7.7 item 4, consolidate).

## Evidence

- Per-file times: the runner's serial runs on `test-trims` at `5e58f24` (157 passed, 3 xfailed, 0 skipped).
- Hold-back, measured on the host: `MARV_CI_MODE=full bash ci/run_ci_gz.sh l4` ran the whole directory, 40 passed,
  1 xfailed, 105 s; per-push against the merge base, 28 passed, 1 xfailed, 44 s (the 12 chirp items held back);
  per-push l5, 21 passed, 71 s (chirp, yaw and recovery held back). No base, or an unresolvable base, runs everything.
  The "touched" branches were exercised against historical bases with the selection function extracted and pytest
  stubbed: a changed `recovery_model.py` or a changed held-back file runs it; a change to another test file does not.
  A change outside the gz directory (tools/, fw/) that selects the group does not un-hold the files; the full run at
  the merge covers them.
- Truth suites: `ctest -N` lists identical before/after (41); 41/41 pass in host-debug and host-release; a planted
  break in the vertical channel fails only `TruthVertical.*`.
- Schemas: `pytest` 154 passed, 0 skipped; a runtime monkeypatch (no source edit) breaking the bare-number check,
  the quaternion norm tolerance and the duplicate-key loader fails the corresponding shared tests for each schema that
  has them.
- Param script: `ctest -N` lists identical (7); 7/7 pass; hand runs show the swapped and bad-prefix controls fail on
  their own messages.
- `ci/smoke.sh` PASSED; `ci/affected.py --check-manifest` OK.

## Approval

Luis's rulings above; the merge into `master` is his.
