# 0028: Regression-change check accepts the regeneration commit line

## What changed

Core §7.3 (decision 0026) lets a regeneration with unchanged thresholds be covered by a commit-message line
`golden regenerated: <path or glob>: <cause>`. This change makes `tools/ci/check_regression_changes.py` accept it:

- A modified file under `tests/regression/` is covered by a decision record that names it, or by such a line in any
  commit of the checked range whose glob matches the file. Deleted and renamed files still need a record.
- The failure message names each uncovered file and both accepted forms of cover.
- `tests/regression/quad/L00/tools/test_check_regression_changes.py` is extended with five cases (a line covers a
  modified file; a glob covers modified files; a non-matching glob or empty cause fails; a line does not cover a
  deletion; a line does not cover a rename). No existing case is changed, loosened or removed.

## Why

Decision 0026 changed the rule; the check still demanded a record for every regeneration (core §7.3: "Until the check
accepts that line (quad §4.0, M0), CI still asks for a record").

## Evidence

The extended test file, run with `uv run pytest tests/regression/quad/L00/tools/test_check_regression_changes.py`;
its negative controls are the non-matching-glob, deletion and rename cases. Nothing is loosened: the check only
accepts a new form of cover for modified files.

## Approval

Luis, 2026-10-15: "very well, let's accept the recommended changes" (decision 0026).
