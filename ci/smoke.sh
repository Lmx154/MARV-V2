#!/usr/bin/env bash
# One-command smoke test, run in the developer's own tree (not a container): uv sync --frozen, configure host-debug, build the
# one target it needs (unit_l11_composition and what it links), then run the L11 composition entries through ctest: the
# L11Composition* gtest suites and the l11_composition_param_ids* script tests.
#
# Covers the whole flight chain through the SIL ABI (marv_sil.h, marv_truth.h, marv_truth_vertical.h): the card-derived
# parameters, the gyro chain, rate, attitude, vertical, mixer and DShot output, as the l11_vertical_scripted composition
# assembles them.
# Does not cover: any plant (no dynamics, no sensors from a model), Gazebo, the m33 target, release builds, the T3 reference
# oracles, the tools tests and the G1/G3/G8 lints.
# Exit status: 0 on pass, non-zero on a failed step or test. Prints the elapsed seconds at the end.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

start="$(date +%s.%N)"
report() {
  awk -v a="${start}" -v b="$(date +%s.%N)" -v s="$1" 'BEGIN { printf "smoke: %s after %.1f s\n", s, b - a }'
}
trap 'report FAILED' ERR

uv sync --frozen
cmake --preset host-debug
cmake --build --preset host-debug --target unit_l11_composition
ctest --preset host-debug -L L11 -R '^(L11Composition|l11_composition_param_ids)'

trap - ERR
report PASSED
