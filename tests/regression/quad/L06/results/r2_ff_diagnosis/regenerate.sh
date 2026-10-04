#!/usr/bin/env bash
# Regenerates the large files of this record into <work dir> and verifies them (decision 0011 style; not in CI).
#   1. the gz input logs, with capture_gz.py's own path (L6 ff_on) and the test-only switch MARV_GZ_TEST_ZERO_FIRST_READ=1,
#      which restores the pre-decision-0016 zero gyro read at step 0 (the behaviour these diagnosis inputs were produced
#      with); the logs are kept in <work dir>/gz/recover_tumble_ff_{off,on}_run/
#   2. recovery_cause_tool (L05 recovery_cause/build_tool.sh) and tool2 (build2.sh) against build/host-gz-l5
#   3. every script that writes a hashed file or a committed output
#   4. SHA256SUMS (sha256sum -c, paths relative to the work dir), then the committed small files byte for byte.
#   Every build and every script comes from a pristine detached git worktree of the commit this record was produced and
#   verified at (PIN below), not from the current tree, so these pre-0016 outputs stay reproducible whatever the firmware
#   becomes. The scripts run are the record's own copies in that worktree (identical to the ones in the current tree).
# Exits nonzero on any mismatch. The work dir must not hold a gz/ directory unless MARV_R2FF_REUSE_GZ=1 (skips step 1).
# Usage (repository root, after `uv sync --frozen`; gz-sim 8): tests/regression/quad/L06/results/r2_ff_diagnosis/regenerate.sh <work dir>
set -euo pipefail
# Decision 0016's commit: the tree this record was produced and verified at (the first with MARV_GZ_TEST_ZERO_FIRST_READ).
PIN=4a76755bedb5dfb07a26aa65653c1a2b35b5008e
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../../../../../.." && pwd)"
mkdir -p "${1:?work dir}"
work="$(cd "$1" && pwd)"
export MARV_R2FF_WORK="$work" PYTHONDONTWRITEBYTECODE=1
cd "$root"
t0=$SECONDS
stamp() { echo "regenerate: $* ($((SECONDS - t0)) s)"; }

src="$work/src"
[[ ! -e "$src" ]] || { echo "regenerate: $src exists" >&2; exit 2; }
git worktree add --detach "$src" "$PIN" > /dev/null
trap 'cd "$root"; git worktree remove --force "$src"; git worktree prune' EXIT
rec="$src/${here#"$root"/}"
uv_() { uv run --frozen --project "$root" "$@"; }
cd "$src"
stamp "pinned tree $PIN ready"

cmake --preset host-gz-l5 -DMARV_PYTHON="$root/.venv/bin/python" > /dev/null
cmake --build --preset host-gz-l5
uv_ python tools/refdata/refdata.py ensure quad/L05/t3
stamp "build and reference ready"

if [[ "${MARV_R2FF_REUSE_GZ:-0}" != 1 ]]; then
  [[ ! -e "$work/gz" ]] || { echo "regenerate: $work/gz exists (set MARV_R2FF_REUSE_GZ=1 to reuse it)" >&2; exit 2; }
  MARV_GZ_TEST_ZERO_FIRST_READ=1 uv_ python tests/regression/quad/L06/results/ff_on/capture_gz.py \
    --case r2 --work "$work/gz" --out "$work/capture_gz_r2.txt" > /dev/null
  for v in off on; do
    d=("$work"/gz/recover_tumble_ff_"$v"_*)
    [[ ${#d[@]} == 1 ]] || { echo "regenerate: expected one recover_tumble_ff_${v}_* in $work/gz" >&2; exit 2; }
    mv "${d[0]}" "$work/gz/recover_tumble_ff_${v}_run"
  done
fi
stamp "gz logs ready"

tests/regression/quad/L05/results/recovery_cause/build_tool.sh "$work/recovery_cause_tool"
"$rec/build2.sh" "$work/tool2"
stamp "tools built"

py() { uv_ python "$rec/$1" "${@:2}"; }
py a_replay.py > /dev/null
py b_ladder.py A B > /dev/null
py b_ladder.py C D E F G H I J K M > /dev/null
py c_wz.py > /dev/null
py h_more.py > /dev/null
py h_more2.py > /dev/null
for v in ideal zoh fw fw0 ideal+zr fw+zr none; do py d_c8fw.py "$v" > /dev/null; done
py e_sens.py > /dev/null
stamp "scripts done"

fail=0
(cd "$work" && sha256sum -c "$here/SHA256SUMS") || fail=1
for f in "$work"/sim_[!i]*.txt "$work"/replay_*.txt; do
  grep -q "  ${f#"$work"/}\$" "$here/SHA256SUMS" || { echo "not in SHA256SUMS: ${f#"$work"/}"; fail=1; }
done
for f in "$here"/{a_replay,c_wz,e_sens,h_more,h_more2}.txt "$here"/b_ladder_*.txt "$here"/d_c8fw_*.txt "$here"/sim_in_*.txt; do
  if cmp -s "$f" "$work/$(basename "$f")"; then echo "$(basename "$f"): OK"; else echo "$(basename "$f"): DIFFERS"; fail=1; fi
done
stamp "verify finished, mismatches: $fail"
exit "$fail"
