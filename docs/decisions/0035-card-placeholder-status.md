# 0035: the card tools accept a PLACEHOLDER status

## What changed

M1 packet P8 (commit 2058498). `tools/card/schema.py` accepts the status tag `PLACEHOLDER` on card and profile entries;
it requires a `placeholder: {reason, replaced_by}` mapping exactly when the tag is present and rejects it on budget
entries. `tools/card/report.py` lists every PLACEHOLDER with its reason and replacement. Two frozen test files gain
cases; none is removed or loosened:

- `tests/regression/quad/L01/tools/test_card_lint.py`: a PLACEHOLDER with reason and replacement lints clean; one
  without a reason, without a replacement, or on a budget entry fails.
- `tests/regression/quad/L01/tools/test_flatten_report.py`: the run report lists the PLACEHOLDERs (known answer).

New data: `sensors/profiles/madflight_fc3v2_revc.yaml` (LSM6DSV, ST DS13476 Rev 5) and
`vehicles/marv_5in_fc3v2.yaml` (Luis's airframe; physics PLACEHOLDER until the M4 stand and pendulum).

## Why

Core §2 (decision 0026) made `PLACEHOLDER` a provenance class: "A `PLACEHOLDER` carries its reason and what will
replace it." The card linter knew only `UNVERIFIED` and `INFERRED`, so the airframe card could not say that its
physics are another airframe's values until M4 measures them.

## Evidence

`uv run pytest tests/regression/quad/L01/tools/test_card_lint.py tests/regression/quad/L01/tools/test_flatten_report.py`:
125 passed. The new cases against the previous `schema.py` and `report.py`: 9 failed, 4 passed (the existing lint-clean
cases and the budget case, which the old schema also rejects). `ci/dev_check.sh --base HEAD`: PASS in 2696 s. Card
lint over both cards: exit 0. Independent review: PASS.

## Approval

Luis, 2026-10-15, on the advisors' recommendations (decision 0026): "very well, let's accept the recommended changes".
The airframe card's spin directions: Luis, 2026-10-16, "5. yes".
