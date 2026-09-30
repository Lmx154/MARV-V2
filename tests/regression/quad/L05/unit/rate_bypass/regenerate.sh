#!/usr/bin/env bash
# Regenerates the I-A1 acro golden against fw/ at a git ref (default: tag quad-L4-pass), never modifying the ref.
#
#   tests/regression/quad/L05/unit/rate_bypass/regenerate.sh [--ref REF] [--inputs FILE] [--out FILE]
#
# Extracts fw/ of REF into a temporary directory, compiles golden_gen.cpp against it with the host flags of the
# build (-ffp-contract=off, -fno-exceptions -fno-rtti) and runs it on the fixture. Defaults: the committed inputs,
# and --out = reference/rate_bypass_acro_golden.txt. The golden is regenerated only inside the marv-ci image (CLAUDE.md).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ref=quad-L4-pass
inputs="${here}/reference/rate_bypass_inputs.txt"
out="${here}/reference/rate_bypass_acro_golden.txt"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ref) ref="$2"; shift 2 ;;
    --inputs) inputs="$2"; shift 2 ;;
    --out) out="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
# The repository root is six levels up (tests/regression/quad/L05/unit/rate_bypass). safe.directory on the command line
# lets git read a checkout owned by another user, as when the CI container runs as root (GitHub Actions).
root="$(cd "${here}/../../../../../.." && pwd)"
git -c safe.directory="${root}" -C "${root}" archive "${ref}" fw | tar -x -C "${tmp}"
"${CXX:-g++}" -std=c++20 -O2 -ffp-contract=off -fno-exceptions -fno-rtti -Wall -Wextra -Werror \
  -I"${tmp}/fw/prim/include" -I"${tmp}/fw/types/include" -I"${tmp}/fw/mixer/include" -I"${tmp}/fw/rate/include" \
  -I"${here}" "${here}/golden_gen.cpp" -o "${tmp}/golden_gen"
"${tmp}/golden_gen" "${inputs}" "${here}/reference/rate_bypass_fixture.txt" "${out}"
