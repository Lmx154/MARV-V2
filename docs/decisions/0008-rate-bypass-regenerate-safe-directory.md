# 0008: the rate-bypass golden regeneration trusts the checkout as a git safe.directory

## What changed

`tests/regression/quad/L05/unit/rate_bypass/regenerate.sh`, which is frozen since L5 reached `master` at `e9bfaae`.
Its `git archive` of `fw/` at the ref `quad-L4-pass` now runs as `git -c safe.directory=<repo root> -C <repo root>`,
with the root taken from the script's own location. Before, it ran as `git -C "$(git rev-parse --show-toplevel)"`.
Nothing else changed: not the test assertions, the fixture, the golden, the controls or the compile command.

## Why

GitHub Actions runs the CI container as root on a checkout owned by the runner user. git then refuses the repository
for "dubious ownership". So `git archive` printed nothing, `tar` failed, and the step "L5: acro identity golden
reproduces from fw/ at quad-L4-pass" failed. That is run 36785715345 on `e9bfaae`.

Local CI had been run with `-u $(id -u):$(id -g)`, and there the files are owned by the caller, which hides the
problem. `ci/run_ci.sh` already sets `safe.directory` for its own git call, the regression-change check
(`ci/run_ci.sh:37-40`). This script now does the same for its call.

The check itself is unchanged. The golden must still reproduce byte for byte from `fw/` at `quad-L4-pass`, and a kp
perturbed by one ulp must still fail to reproduce it.

## Evidence

- **Reproduced.** In `marv-ci` as root (no `-u`) on a clean copy, the old script printed "detected dubious
  ownership" and then "tar: This does not look like a tar archive".
- **Fixed.** The fixed script reproduces the committed golden byte for byte in `marv-ci` as root, and on the host.
- **Full CI as root.** Both parts ran as root on clean copies of `d7a044c` (2026-09-30), with
  `MARV_CI_BASE_REF=ca9ebcb`.
  - `ci/run_ci.sh`: ALL STEPS PASSED.
  - `ci/run_ci_gz.sh`: `gz_l4` 40 passed, 1 xfailed; `gz_l5` 63 passed, 1 xfailed.
- **The regression-change check on the push range.** `check_regression_changes.py --base e9bfaae` flagged this file
  on run 36788978797, because the change had no record. That is why this record exists.

## Approval

Luis, 2026-09-30, answering "Approve 0008, so I can commit it with the handoff, push, tag quad-L5-pass on the green
head and end the session?": "Approve 0008".
