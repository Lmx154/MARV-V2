# 0002: The G1 tools test stops pinning the full literal list of constants.hpp

## What changed

`tests/regression/quad/L00/tools/test_lint_g1.py`, test
`test_real_constants_hpp_holds_its_literal_and_repo_sources_are_otherwise_clean`:

- Before: the scanner's literals for the real `fw/prim/include/marv/prim/constants.hpp` had to equal exactly
  `["3", "48", "2047"]`, the L0 contents of that file.
- After: the L0 literals `["3", "48", "2047"]` must still be found, in that order, among the scanner's literals for
  `constants.hpp`. The second half of the test is unchanged: every other source under `fw/` must scan clean.

## Why

`constants.hpp` is the one file G1 exempts, because every cited constant must live there (core §7.4 G1). Core §9 adds
cited constants to it layer by layer: WGS 84 gravity at quad L1, atmosphere in B1, χ² tables in B2. Quad L1 adds the
NIMA TR8350.2 gravity constants, so the exact-list pin fails. There is nowhere else to put those constants under `fw/`
without failing G1 itself.

The pin described the contents of the repository at L0, not how the lint behaves. Every layer that adds a cited
constant would need a new decision record, and none of them would be about the gate. What the test was protecting
still holds:

- The scanner sees real literals in the exempt file, which is its positive control, still asserted.
- Every non-exempt `fw/` source is clean, still asserted.

G1 itself (clang-tidy over `fw/`, `tools/ci/lint_g1.py`, and its CI negative control) is not changed.

The pin was the only thing that made a new literal in `constants.hpp` visible, and no check existed that every constant
in that file carries a citation (the file's header comment only asserted it). Luis's approval of this record is
conditional on such a check, so it is added in this same change:

- `tools/ci/check_constants.py` (default target `constants.hpp`) splits the file into paragraphs by blank lines. Every
  namespace-scope `constexpr` variable, and every non-`constexpr` variable defined with a numeric initialiser, must sit
  in a paragraph whose comment lines before its first declaration contain a `Citation:` (or
  `Citation for every entry below:`) with text, and exactly one kind tag: `Kind: math`, `Kind: physics` or
  `Kind: standard` (a published standard or specification: SI, WGS 84, a protocol such as DShot). No other kind is
  accepted.
- `constants.hpp` holds physics, mathematics and published-standard constants only. Vehicle numbers (pole count, mass,
  and so on) belong in the card, never there: a paragraph whose comment refers to `vehicles/`, `sensors/profiles/`,
  `design/budget`, or the words card, vehicle card or profile fails.
- The existing paragraphs gained a `Kind:` line in their comments; no declaration, value, name or order changed.

## Evidence

- On `quad-l1` after commit 5081e3a (WGS 84 gravity), before this change:
  `uv run pytest tests/regression/quad/L00/tools -q` gives 1 failed, 258 passed. The failure is this test, because
  `constants.hpp` has 7 literals beyond the pinned list, starting with `6378137.0`.
- After this change the same command gives 259 passed (see the commit that adds this record).
- Negative control: removing `48` from `constants.hpp` still fails the test, so the scanner's positive control keeps
  its resolving power.
- The citation check did not exist before this change. It is added by `tools/ci/check_constants.py`, the CI step
  "constants: every constant in constants.hpp carries a citation and a physics/standard/math kind" (right after the G1
  step of `ci/run_ci.sh`), and two CI negative controls that pass only if the tool exits 1 with the intended reason:
  `tests/regression/quad/L01/controls/constants_uncited.hpp` (a planted uncited constant) and
  `constants_vehicle_number.hpp` (a planted vehicle number citing the card). The pytest
  `tests/regression/quad/L01/tools/test_check_constants.py` also plants an uncited constant in an otherwise valid copy
  of the real file and removes the Citation line from a copy of the WGS 84 paragraph; both must fail.

## Approval

Luis's approval of the pull request or push (core §7.3). Luis authorised a frozen-test change under a decision record
on 2026-09-29 (item 5 of his L1 instructions); this record asks him to approve this specific change.
