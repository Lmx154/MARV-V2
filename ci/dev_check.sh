#!/usr/bin/env bash
# The development check (core 7.6, tier 2): the smoke test, then only the tests the change can reach, in this working
# tree's own incremental build trees (no fresh clone, no container). The full suite is the merge gate, ci/local_ci.sh.
#
# Usage: ci/dev_check.sh [--base <rev>]   (default base: git merge-base HEAD master; the diff includes uncommitted and
#                                          untracked files)
# The selection comes from ci/affected.py and the map ci/blast_radius.yaml. A path the map does not know selects
# everything, so the check never silently skips.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

base_args=()
if (($# > 0)); then
  [[ "$1" == "--base" && $# -eq 2 ]] || {
    echo "usage: ci/dev_check.sh [--base <rev>]" >&2
    exit 2
  }
  base_args=(--base "$2")
fi

# One heavy job at a time on this host (ci/heavy.sh): take the lock, or stop here when someone holds it.
if [[ "${MARV_HEAVY_HELD:-}" != "${MARV_HEAVY_LOCK:-/tmp/marv-heavy.lock}" ]]; then
  exec bash ci/heavy.sh -- bash "${BASH_SOURCE[0]}" "$@"
fi

started="${SECONDS}"

echo "##### smoke"
ci/smoke.sh

# The map must still match the CI steps, the build's include graph and the Python imports (core 7.6); smoke.sh has
# just configured host-debug, so compile_commands.json is current.
echo "##### blast-radius map check"
uv run python ci/affected.py --check-manifest

plan="$(uv run python ci/affected.py ${base_args[@]+"${base_args[@]}"} --json)"
field() {
  # Print one field of the plan: a list joined by the given separator, or a scalar.
  printf '%s' "${plan}" | uv run python -c '
import json, sys
v = json.load(sys.stdin)[sys.argv[1]]
print(sys.argv[2].join(v) if isinstance(v, list) else str(v).lower())' "$1" "${2:-,}"
}

full="$(field full)"
# The base the selection used (empty when it could not be resolved); ci/run_ci_gz.sh reads it to hold back the long gz files.
MARV_CI_BASE_REF="$(field base)"
export MARV_CI_BASE_REF
labels="$(field ctest_labels ,)"
steps="$(field run_ci_steps ,)"
pytest_paths="$(field pytest_paths ' ')"
gz_groups="$(field gz_groups ' ')"

echo "##### selection (ci/affected.py)"
uv run python ci/affected.py ${base_args[@]+"${base_args[@]}"}

# The ctest and pytest selections run inside run_ci.sh's host-debug and tools steps.
[[ -n "${labels}" ]] && steps="${steps:+${steps},}host_preset:host-debug"
[[ -n "${pytest_paths}" ]] && steps="${steps:+${steps},}tools_tests"
steps="$(printf '%s' "${steps}" | tr ',' '\n' | awk 'NF && !seen[$0]++' | paste -sd, -)"

if [[ "${full}" == "true" ]]; then
  echo "##### the change reaches everything: every core step, every gz group"
  bash ci/run_ci.sh
elif [[ -n "${steps}" ]]; then
  echo "##### core steps: ${steps}"
  MARV_CI_ONLY="${steps}" MARV_CTEST_LABELS="${labels}" MARV_PYTEST_PATHS="${pytest_paths}" bash ci/run_ci.sh
else
  echo "##### no core step selected"
fi

not_run=()
for group in ${gz_groups}; do
  if command -v gz >/dev/null 2>&1; then
    echo "##### gz group ${group}"
    bash ci/run_ci_gz.sh "${group}"
  else
    not_run+=("${group}")
  fi
done

echo "##### dev check: PASS in $((SECONDS - started)) s"
if ((${#not_run[@]} > 0)); then
  echo "NOT RUN (gz-sim not on PATH): gz groups ${not_run[*]}. Run them where Gazebo is installed before merging."
  exit 3
fi
