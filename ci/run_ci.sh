#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

step() {
  local name="$1"
  shift
  echo "=== ${name}"
  if "$@"; then
    echo "PASS: ${name}"
  else
    echo "FAIL: ${name}"
    exit 1
  fi
}

host_preset() {
  cmake --preset "$1" \
    && cmake --build --preset "$1" \
    && ctest --preset "$1"
}

m33_build() {
  cmake --preset m33 && cmake --build --preset m33
}

frozen_suites() {
  local listing count
  listing="$(ctest --preset host-release -L frozen -N)"
  count="$(printf '%s\n' "${listing}" | sed -n 's/^Total Tests: *//p')"
  echo "frozen tests found: ${count}"
  ctest --preset host-release -L frozen
}

regression_change_check() {
  GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=safe.directory GIT_CONFIG_VALUE_0="${PWD}" \
    uv run python tools/ci/check_regression_changes.py --base "${MARV_CI_BASE_REF}" --head HEAD
}

pb2_negative_control() {
  local log status=0
  log="$(mktemp)"
  cmake --build --preset m33 --target pb2_double_promotion >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control compiled cleanly: -Wdouble-promotion is not enforced"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q 'error: .*\[-Werror=double-promotion\]' "${log}"; then
    echo "control failed, but not with a -Wdouble-promotion diagnostic"
    rm -f "${log}"
    return 1
  fi
  if grep 'error:' "${log}" | grep -vq '\[-Werror=double-promotion\]'; then
    echo "control failed with a diagnostic other than -Wdouble-promotion"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

step "uv sync --frozen" uv sync --frozen
step "tools tests (pytest tests/tools)" uv run pytest tests/tools -q
if [[ -n "${MARV_CI_BASE_REF:-}" ]]; then
  step "regression change check against ${MARV_CI_BASE_REF}" regression_change_check
fi
step "host-debug: configure, build, ctest" host_preset host-debug
step "host-release: configure, build, ctest" host_preset host-release
step "frozen suites (ctest -L frozen)" frozen_suites
step "m33: configure, build" m33_build
step "PB2 negative control (planted float to double promotion must fail m33)" pb2_negative_control

echo "ALL STEPS PASSED"
