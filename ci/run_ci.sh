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

g1_lint() {
  uv run python tools/ci/lint_g1.py --clang-tidy clang-tidy-18
}

# g1_control <file> <tag that must appear> <tags that must not appear...>
g1_control() {
  local file="$1" want="$2" log status=0
  shift 2
  log="$(mktemp)"
  uv run python tools/ci/lint_g1.py --clang-tidy clang-tidy-18 "${file}" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]]; then
    echo "control did not fail as a lint violation (exit ${status}): G1 is not enforced for ${file}"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^${want}" "${log}"; then
    echo "control failed, but not with a ${want} diagnostic"
    rm -f "${log}"
    return 1
  fi
  local other
  for other in "$@"; do
    if grep -q "^${other}" "${log}"; then
      echo "control failed with an unintended ${other} diagnostic"
      rm -f "${log}"
      return 1
    fi
  done
  rm -f "${log}"
}

g8_check() {
  uv run python tools/ci/check_claude_md.py CLAUDE.md
}

g8_control() {
  local log status=0
  log="$(mktemp)"
  uv run python tools/ci/check_claude_md.py tests/controls/g8_claude_md_missing_unknown.md >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]] || ! grep -q "^G8-MISSING: .*'## UNKNOWN rule' is missing" "${log}"; then
    echo "control did not fail for the missing UNKNOWN rule (exit ${status}): G8 is not enforced"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

g1_literal_control() {
  g1_control tests/controls/g1_planted_literal.cpp G1-TIDY G1-NOLINT
}

g1_constexpr_control() {
  g1_control tests/controls/g1_planted_constexpr.cpp G1-SCAN G1-TIDY G1-NOLINT
}

g1_nolint_control() {
  g1_control tests/controls/g1_planted_nolint.cpp G1-NOLINT G1-TIDY G1-SCAN
}

g3_check() {
  uv run python tools/ci/check_g3.py "$@"
}

# g3_control <preset> <target> <intended tag> <intended text> <check args...>
# Builds the planted target, runs the G3 check on the control manifest and requires exit status 1 (violation), the
# intended diagnostic, and no diagnostic of another G3 check.
g3_control() {
  local preset="$1" target="$2" want="$3" text="$4" log status=0 build_log
  shift 4
  build_log="$(mktemp)"
  if ! cmake --build --preset "${preset}" --target "${target}" >"${build_log}" 2>&1; then
    cat "${build_log}"
    echo "control target ${target} did not build"
    rm -f "${build_log}"
    return 1
  fi
  rm -f "${build_log}"
  log="$(mktemp)"
  g3_check "$@" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]]; then
    echo "control did not fail as a G3 violation (exit ${status}): G3 is not enforced for ${target}"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^${want} " "${log}" || ! grep -qF -- "${text}" "${log}"; then
    echo "control failed, but not with a ${want} diagnostic containing ${text}"
    rm -f "${log}"
    return 1
  fi
  local other
  for other in G3-SYMBOL G3-INCLUDE G3-EXPORT G3-VACUOUS G3-ERROR; do
    if [[ "${other}" != "${want}" ]] && grep -q "^${other}" "${log}"; then
      echo "control failed with an unintended ${other} diagnostic"
      rm -f "${log}"
      return 1
    fi
  done
  rm -f "${log}"
}

g3_host_symbols() {
  g3_check symbols --manifest build/host-debug/g3_manifest.json --nm nm
}

g3_host_includes() {
  g3_check includes --manifest build/host-debug/g3_manifest.json
}

g3_host_exports() {
  g3_check exports --manifest build/host-debug/g3_manifest.json --nm nm
}

g3_m33_symbols() {
  g3_check symbols --manifest build/m33/g3_manifest.json --nm arm-none-eabi-nm
}

g3_m33_includes() {
  g3_check includes --manifest build/m33/g3_manifest.json
}

g3_symbol_control() {
  g3_control host-debug g3_planted_truth G3-SYMBOL "contains 'marv::truth::'" \
    symbols --manifest build/host-debug/tests/controls/g3_control_symbol.json --nm nm
}

g3_m33_symbol_control() {
  g3_control m33 g3_planted_truth G3-SYMBOL "contains 'marv::truth::'" \
    symbols --manifest build/m33/tests/controls/g3_control_symbol.json --nm arm-none-eabi-nm
}

g3_include_control() {
  g3_control host-debug g3_planted_include G3-INCLUDE "inside fw/hal/sim/" \
    includes --manifest build/host-debug/tests/controls/g3_control_include.json \
    --compile-commands build/host-debug/compile_commands.json
}

g3_export_control() {
  g3_control host-debug g3_planted_export G3-EXPORT "exports 'planted_extra_export'" \
    exports --manifest build/host-debug/tests/controls/g3_control_export.json --nm nm
}

step "uv sync --frozen" uv sync --frozen
step "tools tests (pytest tests/tools)" uv run pytest tests/tools -q
step "G8: CLAUDE.md keeps the ACTIVE-spec, number, UNKNOWN and CI-gate sections" g8_check
if [[ -n "${MARV_CI_BASE_REF:-}" ]]; then
  step "regression change check against ${MARV_CI_BASE_REF}" regression_change_check
fi
step "host-debug: configure, build, ctest" host_preset host-debug
step "G1: no unexplained numeric literals under fw/ (clang-tidy, token scan, NOLINT check)" g1_lint
step "G3: no truth or harness symbol in any host flight library (nm -C)" g3_host_symbols
step "G3: no flight translation unit has a harness include directory (host)" g3_host_includes
step "G3: the SIL libraries export only marv_sil_* (nm -D)" g3_host_exports
step "host-release: configure, build, ctest" host_preset host-release
step "frozen suites (ctest -L frozen)" frozen_suites
step "m33: configure, build" m33_build
step "G3: no truth or harness symbol in any m33 flight library (arm-none-eabi-nm -C)" g3_m33_symbols
step "G3: no flight translation unit has a harness include directory (m33)" g3_m33_includes
step "PB2 negative control (planted float to double promotion must fail m33)" pb2_negative_control
step "G1 negative control (planted magic literal must fail clang-tidy)" g1_literal_control
step "G1 negative control (planted literal-initialised constexpr must fail the token scan)" g1_constexpr_control
step "G1 negative control (planted NOLINT of a magic-number check must fail)" g1_nolint_control
step "G3 negative control (planted marv::truth symbol in a flight library must fail the symbol check, host)" g3_symbol_control
step "G3 negative control (planted marv::truth symbol in a flight library must fail the symbol check, m33)" g3_m33_symbol_control
step "G3 negative control (planted fw/hal/sim include directory must fail the include check)" g3_include_control
step "G3 negative control (planted extra SIL export must fail the export check)" g3_export_control
step "G8 negative control (a CLAUDE.md without the UNKNOWN rule must fail)" g8_control

echo "ALL STEPS PASSED"
