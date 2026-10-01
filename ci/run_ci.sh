#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

# shellcheck source=ci/peakmem.sh
source ci/peakmem.sh

# MARV_CI_MODE: per-push (default) or full (nightly and workflow_dispatch). The multi-seed Monte Carlo of L6 stage e hooks
# in here when it exists; nothing reads the mode yet (owner decision: per push = frozen T4 at committed seeds).
echo "MARV_CI_MODE=${MARV_CI_MODE:-per-push}"

step() {
  local name="$1"
  shift
  echo "=== ${name}"
  peakmem_start
  if "$@"; then
    peakmem_report "${name}"
    echo "PASS: ${name}"
  else
    peakmem_report "${name}"
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

sigma_check_control() {
  local log status=0 want='error: static assertion failed: param_defaults: a record violates invariant I1'
  cmake --preset host-debug >/dev/null || return 1
  log="$(mktemp)"
  cmake --build --preset host-debug --target sigma_kind_mismatch >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control compiled cleanly: the sigma_kind / sigma static_assert is not enforced"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "${want}" "${log}"; then
    echo "control failed, but not with the I1 static_assert"
    rm -f "${log}"
    return 1
  fi
  if grep 'error:' "${log}" | grep -vq "${want}"; then
    echo "control failed with a diagnostic other than the I1 static_assert"
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

constants_check() {
  uv run python tools/ci/check_constants.py
}

# constants_control <file> <reason text that must appear>
constants_control() {
  local file="$1" want="$2" log status=0
  log="$(mktemp)"
  uv run python tools/ci/check_constants.py "${file}" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]]; then
    echo "control did not fail as a constants finding (exit ${status}): the constants check is not enforced for ${file}"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^constants: ${file}:[0-9]*: [A-Za-z0-9_]*: ${want}" "${log}"; then
    echo "control failed, but not with the reason: ${want}"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

constants_uncited_control() {
  constants_control tests/regression/quad/L01/controls/constants_uncited.hpp "no 'Citation:'"
}

constants_vehicle_number_control() {
  constants_control tests/regression/quad/L01/controls/constants_vehicle_number.hpp "comment refers to vehicle or part data"
}

g8_check() {
  uv run python tools/ci/check_claude_md.py CLAUDE.md
}

g8_control() {
  local log status=0
  log="$(mktemp)"
  uv run python tools/ci/check_claude_md.py tests/regression/quad/L00/controls/g8_claude_md_missing_unknown.md >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]] || ! grep -q "^G8-MISSING: .*'## UNKNOWN rule' is missing" "${log}"; then
    echo "control did not fail for the missing UNKNOWN rule (exit ${status}): G8 is not enforced"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

g1_literal_control() {
  g1_control tests/regression/quad/L00/controls/g1_planted_literal.cpp G1-TIDY G1-NOLINT
}

g1_constexpr_control() {
  g1_control tests/regression/quad/L00/controls/g1_planted_constexpr.cpp G1-SCAN G1-TIDY G1-NOLINT
}

g1_nolint_control() {
  g1_control tests/regression/quad/L00/controls/g1_planted_nolint.cpp G1-NOLINT G1-TIDY G1-SCAN
}

l1_card_lint() {
  uv run python tools/card/lint.py --card vehicles/uzh_neurobem_5in.yaml --budget design/budget.yaml
}

# The core 2.1 example card lacks sigma on every published entry: the linter must exit 1 with a sigma reason on it.
l1_card_lint_control() {
  local log status=0
  log="$(mktemp)"
  uv run python tools/card/lint.py --card tests/regression/quad/L01/fixtures/card/spec_2_1_example_card.yaml \
    >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]] || ! grep -q '^tests/regression/quad/L01/fixtures/card/spec_2_1_example_card.yaml: [^:]*: sigma: missing' "${log}"; then
    echo "control did not fail with a sigma finding (exit ${status}): the card linter does not enforce the sigma policy"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

# The product parameter set is built from the linted card (tools/card/flatten.py, run at build time). A card that fails
# the sigma policy (sigma = 0 on a published entry) must fail the build of the generated set, with the sigma finding.
l1_flatten_control() {
  local dir log status=0
  dir="$(mktemp -d)"
  log="$(mktemp)"
  if ! cmake -S . -B "${dir}" -G Ninja -DMARV_TARGET=host -DCMAKE_BUILD_TYPE=Debug \
       -DMARV_VEHICLE_CARD=tests/regression/quad/L01/controls/card_sigma_zero.yaml \
       -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="${PWD}/build/host-debug/_deps/googletest-src" >"${log}" 2>&1; then
    cat "${log}"
    echo "control configure failed: the sigma-zero card must be refused at build time, not at configure time"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  cmake --build "${dir}" --target marv_params_generated >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control built: the product parameter set accepts a card that fails the sigma policy"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  if ! grep -q 'card_sigma_zero.yaml: mass: sigma: 0 is rejected' "${log}"; then
    echo "control build failed, but not with the sigma finding on the card"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  rm -rf "${dir}" "${log}"
}

# Configuring with a Python that cannot import PyYAML must stop with the PyYAML message (fw/params/CMakeLists.txt).
l1_pyyaml_control() {
  local dir venv log status=0
  dir="$(mktemp -d)"
  venv="$(mktemp -d)"
  log="$(mktemp)"
  python3 -m venv --without-pip "${venv}"
  if "${venv}/bin/python" -c "import yaml" 2>/dev/null; then
    echo "control setup failed: ${venv}/bin/python imports PyYAML"
    rm -rf "${dir}" "${venv}" "${log}"
    return 1
  fi
  cmake -S . -B "${dir}" -G Ninja -DMARV_TARGET=host -DMARV_PYTHON="${venv}/bin/python" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]] || ! grep -q "cannot import PyYAML" "${log}"; then
    echo "control did not fail with the PyYAML message (exit ${status}): the PyYAML configure check is not enforced"
    rm -rf "${dir}" "${venv}" "${log}"
    return 1
  fi
  rm -rf "${dir}" "${venv}" "${log}"
}

l1_report() {
  cat build/host-debug/generated/marv_params/l1_report.txt
}

# plant_ref.py writes plant_ref_expected.txt beside itself, so it runs on a copy of the reference directory.
plant_ref_reference_dir=tests/regression/quad/L01/unit/plant/reference

plant_ref_run() {
  local dir="$1"
  cp "${plant_ref_reference_dir}/plant_ref.py" "${dir}/plant_ref.py"
  uv run python "${dir}/plant_ref.py"
}

plant_ref_reproduces() {
  local dir status=0
  dir="$(mktemp -d)"
  cp "${plant_ref_reference_dir}/plant_ref_inputs.txt" "${dir}/plant_ref_inputs.txt"
  plant_ref_run "${dir}"
  diff -u "${plant_ref_reference_dir}/plant_ref_expected.txt" "${dir}/plant_ref_expected.txt" || status=$?
  rm -rf "${dir}"
  return "${status}"
}

# The same run on inputs with the mass changed must differ from the committed expected file.
plant_ref_control() {
  local dir
  dir="$(mktemp -d)"
  sed 's/^mass_kg .*/mass_kg 0.76/' "${plant_ref_reference_dir}/plant_ref_inputs.txt" >"${dir}/plant_ref_inputs.txt"
  if cmp -s "${plant_ref_reference_dir}/plant_ref_inputs.txt" "${dir}/plant_ref_inputs.txt"; then
    echo "control setup failed: the perturbation did not change the inputs"
    rm -rf "${dir}"
    return 1
  fi
  plant_ref_run "${dir}"
  if diff -q "${plant_ref_reference_dir}/plant_ref_expected.txt" "${dir}/plant_ref_expected.txt" >/dev/null; then
    echo "control produced the committed expected file from perturbed inputs: the reproduction check cannot fail"
    rm -rf "${dir}"
    return 1
  fi
  rm -rf "${dir}"
}

# T3 reference data is not committed (decision 0011): tools/refdata/refdata.py regenerates it from the committed inputs into
# ${MARV_REFERENCE_DIR:-build/reference} and checks it against the committed SHA256SUMS. The reproduction steps below
# regenerate (--force) and the ctest fixtures of the T3 suites then read that same directory. The L5 generator runs on the
# runner's 4 CPUs (ci/local_ci.sh RUNNER_CPUS); the output does not depend on the number of processes.
refdata="tools/refdata/refdata.py"
t3_reference_dir=tests/regression/quad/L04/t3/reference

t3_oracle_run() {
  local dir="$1"
  uv run python "${t3_reference_dir}/rate_t3_oracle.py" --dir "${dir}"
}

t3_reference_reproduces() {
  uv run python "${refdata}" ensure quad/L04/t3 --force
}

# The same run with rate_kp_roll one float32 ulp up must fail the SHA256SUMS check, naming rate_t3_golden.txt.
t3_reference_control() {
  local dir log
  dir="$(mktemp -d)"
  log="$(mktemp)"
  uv run python - "${t3_reference_dir}/rate_t3_inputs.txt" "${dir}/rate_t3_inputs.txt" <<'PY'
import struct
import sys

out = []
for line in open(sys.argv[1]):
    if line.startswith("rate_kp_roll "):
        x = float.fromhex(line.split()[1])
        bits = struct.unpack("<I", struct.pack("<f", x))[0] + 1
        line = f"rate_kp_roll {struct.unpack('<f', struct.pack('<I', bits))[0].hex()}\n"
    out.append(line)
open(sys.argv[2], "w").write("".join(out))
PY
  if cmp -s "${t3_reference_dir}/rate_t3_inputs.txt" "${dir}/rate_t3_inputs.txt"; then
    echo "control setup failed: the perturbation did not change the inputs"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  t3_oracle_run "${dir}" || { rm -rf "${dir}" "${log}"; return 1; }
  if uv run python "${refdata}" verify quad/L04/t3 --dir "${dir}" 2>"${log}"; then
    echo "control passed the SHA256SUMS check from a perturbed input: the reproduction check cannot fail"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  cat "${log}"
  if ! grep -q "rate_t3_golden.txt: sha256" "${log}"; then
    echo "control failed, but not on rate_t3_golden.txt"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  rm -rf "${dir}" "${log}"
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
    symbols --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_symbol.json --nm nm
}

g3_m33_symbol_control() {
  g3_control m33 g3_planted_truth G3-SYMBOL "contains 'marv::truth::'" \
    symbols --manifest build/m33/tests/regression/quad/L00/controls/g3_control_symbol.json --nm arm-none-eabi-nm
}

g3_include_control() {
  g3_control host-debug g3_planted_include G3-INCLUDE "inside fw/hal/sim/" \
    includes --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_include.json \
    --compile-commands build/host-debug/compile_commands.json
}

g3_object_control() {
  g3_control host-debug g3_planted_object G3-SYMBOL "contains 'marv::truth::'" \
    symbols --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_object.json --nm nm
}

g3_m33_object_control() {
  g3_control m33 g3_planted_object G3-SYMBOL "contains 'marv::truth::'" \
    symbols --manifest build/m33/tests/regression/quad/L00/controls/g3_control_object.json --nm arm-none-eabi-nm
}

g3_interface_control() {
  g3_control host-debug g3_planted_interface_carrier G3-INCLUDE "INTERFACE include directory" \
    includes --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_interface.json \
    --compile-commands build/host-debug/compile_commands.json
}

g3_forced_control() {
  g3_control host-debug g3_planted_forced G3-INCLUDE "forced include" \
    includes --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_forced.json \
    --compile-commands build/host-debug/compile_commands.json
}

g3_export_control() {
  g3_control host-debug g3_planted_export G3-EXPORT "exports 'planted_extra_export'" \
    exports --manifest build/host-debug/tests/regression/quad/L00/controls/g3_control_export.json --nm nm
}

g3_plant_control() {
  g3_control host-debug g3_planted_plant G3-SYMBOL "contains 'marv_plant_'" \
    symbols --manifest build/host-debug/tests/regression/quad/L01/controls/g3_control_plant.json --nm nm \
    && g3_control host-debug g3_planted_plant G3-SYMBOL "contains 'marv::plant::'" \
    symbols --manifest build/host-debug/tests/regression/quad/L01/controls/g3_control_plant.json --nm nm
}


# L5 T3: attitude_t3_oracle.py reads attitude_t3_inputs.txt and attitude_t3_q_inputs.txt and writes attitude_t3_golden.txt,
# attitude_t3_envelope.txt and attitude_t3_q.txt into --dir (decision 0006 F); regenerated and checked as for L4 (decision 0011).
att_t3_reference_dir=tests/regression/quad/L05/t3/reference
att_t3_procs=4

att_t3_oracle_run() {
  local dir="$1"
  uv run python "${att_t3_reference_dir}/attitude_t3_oracle.py" --dir "${dir}" --procs "${att_t3_procs}"
}

att_t3_reference_reproduces() {
  uv run python "${refdata}" ensure quad/L05/t3 --force --procs "${att_t3_procs}"
}

# The same run with att_kp one float32 ulp up must fail the SHA256SUMS check, naming attitude_t3_golden.txt and
# attitude_t3_envelope.txt (the Q file is checked as well and is not required to differ).
att_t3_reference_control() {
  local dir log name
  dir="$(mktemp -d)"
  log="$(mktemp)"
  cp "${att_t3_reference_dir}/attitude_t3_q_inputs.txt" "${dir}/attitude_t3_q_inputs.txt"
  uv run python - "${att_t3_reference_dir}/attitude_t3_inputs.txt" "${dir}/attitude_t3_inputs.txt" <<'PY'
import struct
import sys

out = []
for line in open(sys.argv[1]):
    if line.startswith("att_kp "):
        x = float.fromhex(line.split()[1])
        bits = struct.unpack("<I", struct.pack("<f", x))[0] + 1
        line = f"att_kp {struct.unpack('<f', struct.pack('<I', bits))[0].hex()}\n"
    out.append(line)
open(sys.argv[2], "w").write("".join(out))
PY
  if cmp -s "${att_t3_reference_dir}/attitude_t3_inputs.txt" "${dir}/attitude_t3_inputs.txt"; then
    echo "control setup failed: the perturbation did not change the inputs"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  att_t3_oracle_run "${dir}" || { rm -rf "${dir}" "${log}"; return 1; }
  if uv run python "${refdata}" verify quad/L05/t3 --dir "${dir}" 2>"${log}"; then
    echo "control passed the SHA256SUMS check from a perturbed input: the reproduction check cannot fail"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  cat "${log}"
  for name in attitude_t3_golden.txt attitude_t3_envelope.txt; do
    if ! grep -q "${name}: sha256" "${log}"; then
      echo "control failed, but not on ${name}"
      rm -rf "${dir}" "${log}"
      return 1
    fi
  done
  rm -rf "${dir}" "${log}"
}

# L5 rate bypass (decision 0006 A, I-A1): the fixture and the acro golden regenerate from fw/ at quad-L4-pass.
rate_bypass_dir=tests/regression/quad/L05/unit/rate_bypass

rate_bypass_fixture_reproduces() {
  local dir status=0
  dir="$(mktemp -d)"
  uv run python "${rate_bypass_dir}/gen_fixture.py" --dir "${dir}" || status=$?
  if [[ ${status} -eq 0 ]]; then
    cmp "${rate_bypass_dir}/reference/rate_bypass_fixture.txt" "${dir}/rate_bypass_fixture.txt" || status=$?
  fi
  rm -rf "${dir}"
  return "${status}"
}

rate_bypass_golden_reproduces() {
  local dir status=0
  dir="$(mktemp -d)"
  "${rate_bypass_dir}/regenerate.sh" --out "${dir}/golden.txt" || status=$?
  if [[ ${status} -eq 0 ]]; then
    cmp "${rate_bypass_dir}/reference/rate_bypass_acro_golden.txt" "${dir}/golden.txt" || status=$?
  fi
  rm -rf "${dir}"
  return "${status}"
}

# The golden regenerated with rate_kp_roll one float32 ulp up must differ from the committed one.
rate_bypass_golden_control() {
  local dir
  dir="$(mktemp -d)"
  uv run python - "${rate_bypass_dir}/reference/rate_bypass_inputs.txt" "${dir}/inputs.txt" <<'PY'
import struct
import sys

out = []
for line in open(sys.argv[1]):
    if line.startswith("rate_kp_roll "):
        x = float.fromhex(line.split()[1])
        bits = struct.unpack("<I", struct.pack("<f", x))[0] + 1
        line = f"rate_kp_roll {struct.unpack('<f', struct.pack('<I', bits))[0].hex()}\n"
    out.append(line)
open(sys.argv[2], "w").write("".join(out))
PY
  if cmp -s "${rate_bypass_dir}/reference/rate_bypass_inputs.txt" "${dir}/inputs.txt"; then
    echo "control setup failed: the perturbation did not change the inputs"
    rm -rf "${dir}"
    return 1
  fi
  "${rate_bypass_dir}/regenerate.sh" --inputs "${dir}/inputs.txt" --out "${dir}/golden.txt" || { rm -rf "${dir}"; return 1; }
  if cmp -s "${rate_bypass_dir}/reference/rate_bypass_acro_golden.txt" "${dir}/golden.txt"; then
    echo "control produced the committed golden from a perturbed input: the reproduction check cannot fail"
    rm -rf "${dir}"
    return 1
  fi
  rm -rf "${dir}"
}

g3_truth_unflagged_control() {
  g3_control host-debug g3_truth_unflagged_export G3-EXPORT "exports 'marv_truth_state_set'" \
    exports --manifest build/host-debug/tests/regression/quad/L05/controls/g3_control_truth_unflagged.json --nm nm
}

g3_truth_planted_control() {
  g3_control host-debug g3_truth_planted_export G3-EXPORT "exports 'marv_truth_planted'" \
    exports --manifest build/host-debug/tests/regression/quad/L05/controls/g3_control_truth_planted.json --nm nm
}

step "uv sync --frozen" uv sync --frozen
step "L4: T3 oracle regenerates rate_t3_golden.txt and rate_t3_envelope.txt from rate_t3_inputs.txt, matching reference/SHA256SUMS" t3_reference_reproduces
step "L5: T3 oracle regenerates attitude_t3_golden.txt, attitude_t3_envelope.txt and attitude_t3_q.txt from their inputs, matching reference/SHA256SUMS" att_t3_reference_reproduces
step "tools tests (pytest tests/regression/quad/L00/tools tests/regression/quad/L01/tools tests/regression/quad/L03/tools tests/regression/quad/L04/tools tests/regression/quad/L05/tools)" \
  uv run pytest tests/regression/quad/L00/tools tests/regression/quad/L01/tools tests/regression/quad/L03/tools tests/regression/quad/L04/tools tests/regression/quad/L05/tools -q
step "G8: CLAUDE.md keeps the ACTIVE-spec, number, UNKNOWN and CI-gate sections" g8_check
step "L1: the committed vehicle card, its sensor profile and the design budget lint clean (sigma policy)" l1_card_lint
step "L1: plant known-answer reference reproduces plant_ref_expected.txt from plant_ref_inputs.txt" plant_ref_reproduces
step "L5: rate-bypass fixture reproduces from gen_fixture.py" rate_bypass_fixture_reproduces
step "L5: acro identity golden reproduces from fw/ at quad-L4-pass" rate_bypass_golden_reproduces
if [[ -n "${MARV_CI_BASE_REF:-}" ]]; then
  step "regression change check against ${MARV_CI_BASE_REF}" regression_change_check
fi
step "host-debug: configure, build, ctest" host_preset host-debug
step "L1: the run report of the product parameter set (card, budget, provenance)" l1_report
step "G1: no unexplained numeric literals under fw/ (clang-tidy, token scan, NOLINT check)" g1_lint
step "constants: every constant in constants.hpp carries a citation and a physics/standard/math kind" constants_check
step "G3: no truth or harness symbol in any host flight library (nm -C)" g3_host_symbols
step "G3: no flight translation unit has a harness include directory (host)" g3_host_includes
step "G3: the SIL libraries export only marv_sil_* (nm -D)" g3_host_exports
step "host-release: configure, build, ctest" host_preset host-release
step "frozen suites (ctest -L frozen)" frozen_suites
step "m33: configure, build" m33_build
step "G3: no truth or harness symbol in any m33 flight library (arm-none-eabi-nm -C)" g3_m33_symbols
step "G3: no flight translation unit has a harness include directory (m33)" g3_m33_includes
step "PB2 negative control (planted float to double promotion must fail m33)" pb2_negative_control
step "L1 negative control (a record table with a Known record of sigma 0 must fail the static_assert, host)" sigma_check_control
step "G1 negative control (planted magic literal must fail clang-tidy)" g1_literal_control
step "G1 negative control (planted literal-initialised constexpr must fail the token scan)" g1_constexpr_control
step "G1 negative control (planted NOLINT of a magic-number check must fail)" g1_nolint_control
step "constants negative control (a planted uncited constant must fail the constants check)" constants_uncited_control
step "constants negative control (a planted vehicle number citing the card must fail the constants check)" constants_vehicle_number_control
step "G3 negative control (planted marv::truth symbol in a flight library must fail the symbol check, host)" g3_symbol_control
step "G3 negative control (planted marv::truth symbol in a flight library must fail the symbol check, m33)" g3_m33_symbol_control
step "G3 negative control (planted fw/hal/sim include directory must fail the include check)" g3_include_control
step "G3 negative control (planted marv::truth symbol in an OBJECT library must fail the symbol check, host)" g3_object_control
step "G3 negative control (planted marv::truth symbol in an OBJECT library must fail the symbol check, m33)" g3_m33_object_control
step "G3 negative control (planted fw/hal/sim in an INTERFACE library's include directories must fail the include check)" g3_interface_control
step "G3 negative control (planted -include of a fw/hal/sim file must fail the include check)" g3_forced_control
step "G3 negative control (planted extra SIL export must fail the export check)" g3_export_control
step "G8 negative control (a CLAUDE.md without the UNKNOWN rule must fail)" g8_control
step "G3 negative control (planted marv_plant_ and marv::plant:: symbols in a flight library must fail the symbol check)" g3_plant_control
step "L1 negative control (the core 2.1 example card without sigma must fail the card linter)" l1_card_lint_control
step "L1 negative control (a card with sigma = 0 on a published entry must fail the parameter set build)" l1_flatten_control
step "L1 negative control (a Python without PyYAML must fail the configure)" l1_pyyaml_control
step "L1 negative control (perturbed plant reference inputs must not reproduce the committed expected file)" plant_ref_control
step "L4 negative control (a perturbed T3 input, kp one ulp up, must fail the SHA256SUMS check)" t3_reference_control
step "L5 negative control (a perturbed T3 input, att_kp one ulp up, must fail the SHA256SUMS check)" att_t3_reference_control
step "L5 negative control (a perturbed rate-bypass input, kp one ulp up, must not reproduce the acro identity golden)" rate_bypass_golden_control
step "G3 negative control (unflagged SIL library exporting marv_truth_state_set must fail the export check)" g3_truth_unflagged_control
step "G3 negative control (truth_state library also exporting marv_truth_planted must fail the export check)" g3_truth_planted_control

echo "ALL STEPS PASSED"
