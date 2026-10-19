#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

# shellcheck source=ci/peakmem.sh
source ci/peakmem.sh

# MARV_CI_MODE: per-push (default; the development check, ci/dev_check.sh) or full (the merge gate: every Actions run and
# ci/local_ci.sh, core 7.6). The multi-seed Monte Carlo of L6 stage e hooks
# in here when it exists (owner decision: per push = frozen T4 at committed seeds). Today the mode selects the tools tests
# only: per push skips nightly_tools_tests, full runs them.
echo "MARV_CI_MODE=${MARV_CI_MODE:-per-push}"

# Selection hooks (all unset or empty = today's behaviour; the selector in ci/dev_check.sh sets them):
#   MARV_CI_ONLY       comma-separated step ids; only those steps run, the others print SKIP. A step id is the function name
#                      given to step, except host_preset, whose id carries its preset (host_preset:host-debug,
#                      host_preset:host-release). The uv sync --frozen step always runs.
#   MARV_CTEST_LABELS  comma-separated ctest labels (L04,L11); host_preset runs only the ctest entries carrying one of them.
#   MARV_PYTEST_PATHS  space-separated pytest paths; tools_tests runs those instead of its literal list.
step() {
  local name="$1" id
  shift
  id="$1"
  if [[ "${id}" == host_preset ]]; then
    id="host_preset:$2"
  fi
  if [[ -n "${MARV_CI_ONLY:-}" && "$1" != uv && ",${MARV_CI_ONLY}," != *",${id},"* ]]; then
    echo "SKIP (not selected): ${name}"
    return 0
  fi
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
  local label_args=()
  if [[ -n "${MARV_CTEST_LABELS:-}" ]]; then
    label_args=(-L "^(${MARV_CTEST_LABELS//,/|})\$")
  fi
  cmake --preset "$1" \
    && cmake --build --preset "$1" \
    && { [[ "$1" != host-release ]] || { printf 'frozen tests found: '; ctest --preset "$1" -N -L frozen | tail -1; }; } \
    && ctest --preset "$1" --parallel "$(nproc)" ${label_args[@]+"${label_args[@]}"}
}

m33_build() {
  local elf=build/m33/fw/hal/target/marv_fc3v2.elf
  cmake --preset m33 && cmake --build --preset m33 \
    && arm-none-eabi-size -A "${elf}" && arm-none-eabi-readelf -A "${elf}"
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

gs3_check() {
  uv run python tools/ci/check_gs3.py
}

# The GCS unit tests and the type check (GCS-first unit GX-0): vitest (gcs/src and the TypeScript side of the L00 codec
# conformance suite) and svelte-check. The dependencies are gcs/node_modules (`cd gcs && npm ci`). The merge gate (MARV_CI_MODE=full:
# Actions, ci/local_ci.sh) installs them itself in its fresh tree; the development check never installs behind the developer's
# back, it fails loudly instead, so that a missing install is never read as a pass.
gcs_tests() {
  if [[ -n "${MARV_CACHE_DIR:-}" ]]; then
    export npm_config_cache="${MARV_CACHE_DIR}/npm"
  fi
  if [[ ! -x gcs/node_modules/.bin/vitest ]]; then
    if [[ "${MARV_CI_MODE:-per-push}" == full ]]; then
      (cd gcs && npm ci) || return 1
    else
      echo "gcs/node_modules is missing (no vitest): run 'cd gcs && npm ci', then ci/dev_check.sh again. The GCS tests did not run." >&2
      return 1
    fi
  fi
  (cd gcs && npm test && npm run check)
}

gs3_control() {
  local log status=0
  log="$(mktemp)"
  uv run python tools/ci/check_gs3.py tests/regression/quad/L00/controls/gs3_mavlink_include.cpp >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 1 ]] || ! grep -q "^GS3-VIOLATION: .*gs3_mavlink_include.cpp:3: names MAVLink" "${log}"; then
    echo "control did not fail for the planted MAVLink include (exit ${status}): GS-3 is not enforced"
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

# The per-push time limit (design/budget.yaml, per_push_check_time_max) is the CTest TIMEOUT of every per-push check
# (cmake/marv_test_timeout.cmake). Positive check: in the host-debug build every test carries TIMEOUT equal to the
# register value, read here independently with PyYAML, except the three reference-set generation steps (decision 0011),
# which carry none.
per_push_timeout_applied() {
  ctest --preset host-debug --show-only=json-v1 | uv run python -c '
import json, sys, yaml
limit = yaml.safe_load(open("design/budget.yaml"))["per_push_check_time_max"]["value"]
tests = json.load(sys.stdin)["tests"]
bad, without = [], []
for t in tests:
    got = [p["value"] for p in t["properties"] if p["name"] == "TIMEOUT"]
    if not got:
        without.append(t["name"])
    elif got != [limit]:
        bad.append((t["name"], got))
print(f"tests: {len(tests)}, with TIMEOUT = {limit}: {len(tests) - len(without) - len(bad)}, without TIMEOUT: {sorted(without)}")
if bad or sorted(without) != ["marv_reference_quad_L04_t3_ensure", "marv_reference_quad_L05_t3_ensure", "marv_reference_quad_L11_t3_ensure"]:
    print("TIMEOUT differs from the register value:", bad[:5], "unexpectedly without:", without)
    sys.exit(1)
'
}

# Negative control for the per-push timeout: a planted test (tests/regression/quad/L06/controls) sleeps three times the
# limit. The control build replaces the register value with 1 s through MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE (used only
# here) so it takes seconds; ctest must report the planted test as a timeout failure, and its TIMEOUT must be the override.
per_push_timeout_control() {
  local dir log status=0
  dir="$(mktemp -d)"
  log="$(mktemp)"
  if ! cmake -S . -B "${dir}" -G Ninja -DMARV_TARGET=host -DCMAKE_BUILD_TYPE=Debug -DMARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE=1 \
       -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="${PWD}/build/host-debug/_deps/googletest-src" >"${log}" 2>&1; then
    cat "${log}"
    echo "control configure failed"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  if ! ctest --test-dir "${dir}" -R '^per_push_timeout_planted$' --show-only=json-v1 \
       | uv run python -c 'import json,sys; t=json.load(sys.stdin)["tests"]; sys.exit(0 if len(t)==1 and [p["value"] for p in t[0]["properties"] if p["name"]=="TIMEOUT"]==[1.0] else 1)'; then
    echo "control setup failed: the planted test is missing or its TIMEOUT is not the override"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  ctest --test-dir "${dir}" -R '^per_push_timeout_planted$' >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control passed: a test that overruns the limit does not fail ctest"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  if ! grep -q 'per_push_timeout_planted.*Timeout' "${log}"; then
    echo "control failed, but not as a timeout"
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
# $MARV_REFERENCE_DIR, else $MARV_CACHE_DIR/reference, else build/reference, and checks it against the committed SHA256SUMS. The
# reproduction steps below regenerate (--force) and the ctest fixtures of the T3 suites then read that same directory. With
# MARV_CACHE_DIR set (the persistent CI cache) they do not force: refdata.py reuses an entry whose stamp (the generator, its
# inputs, its import closure, the repository files it reads, uv.lock, the interpreter) is current and whose files match
# SHA256SUMS, and regenerates otherwise, so the entry exists only because the generator produced the committed hashes from
# exactly the stamped files. The L5 generator runs on the runner's 4 CPUs (ci/local_ci.sh RUNNER_CPUS); the output does not
# depend on the number of processes.
refdata="tools/refdata/refdata.py"
refdata_force=(--force)
if [[ -n "${MARV_CACHE_DIR:-}" ]]; then
  refdata_force=()
fi
t3_reference_dir=tests/regression/quad/L04/t3/reference

# The verdict of a T3 negative control is a function of the set's stamp, the perturbed input, tools/refdata and the text of the
# control itself, so with MARV_CACHE_DIR set a verified failure is recorded and not re-run while that key is unchanged
# (tools/refdata/refdata.py control-key). Without MARV_CACHE_DIR a control always runs. t3_control_cached <set> <control>
# <perturbed inputs> <function>... returns 0 on a hit; t3_control_store <set> <control> <verify log> records a verified failure
# (only called once the control has behaved as required).
t3_control_key=""
t3_control_cached() {
  local set="$1" name="$2" inputs="$3" text record
  shift 3
  t3_control_key=""
  [[ -n "${MARV_CACHE_DIR:-}" ]] || return 1
  text="$(mktemp)"
  declare -f "$@" t3_control_cached t3_control_store >"${text}"
  t3_control_key="$(uv run python "${refdata}" control-key "${set}" --input "${inputs}" --text "${text}")" \
    || { rm -f "${text}"; t3_control_key=""; return 1; }
  rm -f "${text}"
  record="$(uv run python "${refdata}" control-lookup "${set}" "${name}" "${t3_control_key}")" || return 1
  echo "control verdict cached for key ${t3_control_key} (recorded failure: ${record})"
}

t3_control_store() {
  [[ -n "${MARV_CACHE_DIR:-}" && -n "${t3_control_key}" ]] || return 0
  uv run python "${refdata}" control-store "$1" "$2" "${t3_control_key}" --log "$3"
}

t3_oracle_run() {
  local dir="$1"
  uv run python "${t3_reference_dir}/rate_t3_oracle.py" --dir "${dir}"
}

t3_reference_reproduces() {
  uv run python "${refdata}" ensure quad/L04/t3 ${refdata_force[@]+"${refdata_force[@]}"}
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
  if t3_control_cached quad/L04/t3 "${FUNCNAME[0]}" "${dir}/rate_t3_inputs.txt" t3_reference_control t3_oracle_run; then
    rm -rf "${dir}" "${log}"
    return 0
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
  t3_control_store quad/L04/t3 "${FUNCNAME[0]}" "${log}"
  rm -rf "${dir}" "${log}"
}

# The product parameter table (decision 0014, owner ruling 2026-10-03): the table the build generates
# (fw/params/CMakeLists.txt, tools/gen/params_gen.py --out-table) must equal the committed one byte for byte.
product_table_name=marv_params_product_table.txt
product_table_committed=tests/regression/quad/L06/product_params/${product_table_name}
product_table_regen='the safe command of tests/regression/quad/L06/product_params/README.md (docker run marv-ci as your user with UV_PROJECT_ENVIRONMENT=/tmp/venv, a build in /tmp/build, --target marv_params_generated) and copy the generated '"${product_table_name}"' over '"${product_table_committed}"' (a change needs a decision record)'

# product_table_compare <generated table> <committed table>
product_table_compare() {
  local generated="$1" committed="$2"
  if [[ ! -f "${generated}" ]]; then
    echo "product table ${generated} was not generated"
    return 1
  fi
  if ! cmp -s "${generated}" "${committed}"; then
    echo "the generated product parameter table ${generated} differs from the committed ${committed}:"
    diff -u "${committed}" "${generated}" | head -n 40 || true
    echo "regenerate: ${product_table_regen}"
    return 1
  fi
}

product_table_host() {
  product_table_compare "build/host-debug/generated/marv_params/${product_table_name}" "${product_table_committed}"
}

product_table_m33() {
  product_table_compare "build/m33/generated/marv_params/${product_table_name}" "${product_table_committed}"
}

# A copy of the committed table with one value changed (mass, 0.772000015 to 0.772000030) must fail the comparison with the
# build's table, showing the changed row and the regeneration command.
product_table_control() {
  local dir log status=0
  dir="$(mktemp -d)"
  log="$(mktemp)"
  sed 's/^\(0\tmass\tf32\t\)0\.772000015\t/\10.772000030\t/' "${product_table_committed}" >"${dir}/${product_table_name}"
  if cmp -s "${product_table_committed}" "${dir}/${product_table_name}"; then
    echo "control setup failed: the perturbation did not change the table"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  product_table_compare "build/host-debug/generated/marv_params/${product_table_name}" "${dir}/${product_table_name}" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control passed: a changed value in the committed table is not detected"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  if ! grep -q '^+0.*mass.*0\.772000015' "${log}" || ! grep -q '^-0.*mass.*0\.772000030' "${log}" || ! grep -q '^regenerate: ' "${log}"; then
    echo "control failed, but not with the changed mass row and the regeneration command"
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

# param_table_control <preset>: the post-link parameter-table check (cmake/param_table.cmake, tools/ci/check_param_table.py)
# on its planted control (tests/regression/quad/L09/controls): the FC3v2 runtime linked with the product set's table ahead
# of FC3v2's must fail the build in the check, with the table-size finding and no compile or link error, and leave no image.
param_table_control() {
  local preset="$1" target=param_table_planted log status=0 left
  cmake --preset "${preset}" >/dev/null || return 1
  log="$(mktemp)"
  cmake --build --preset "${preset}" --target "${target}" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control built: an image holding another set's parameter table passes the post-link check (${preset})"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^PARAM-TABLE: .*${target}[^:]*: kParamDefaults is [0-9]* bytes; the set has [0-9]* records" "${log}"; then
    echo "control failed, but not with the parameter-table finding (${preset})"
    rm -f "${log}"
    return 1
  fi
  if grep -qE 'error:|undefined reference|multiple definition' "${log}"; then
    echo "control failed with a compile or link error besides the parameter-table finding (${preset})"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
  left="$(find "build/${preset}/tests/regression/quad/L09/controls" -maxdepth 1 -type f -name "${target}*")"
  if [[ -n "${left}" ]]; then
    echo "control failed, but the image that failed the check was left: ${left}"
    return 1
  fi
}

param_table_host_control() {
  param_table_control host-debug
}

param_table_m33_control() {
  param_table_control m33
}

# flash_layout_control: the post-link flash-layout check (cmake/flash_layout.cmake, tools/ci/check_flash_layout.py) on its
# planted control (tests/regression/quad/L09/flash_layout): an m33 image whose flash bytes reach the config slots must fail the
# build in the check, with the slot finding and no compile or link error, and leave no image.
flash_layout_control() {
  local target=flash_layout_planted log status=0 left
  cmake --preset m33 >/dev/null || return 1
  log="$(mktemp)"
  cmake --build --preset m33 --target "${target}" >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control built: an image that reaches the config slots passes the post-link check"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^FLASH-LAYOUT: ${target}[^:]*: the image ends at 0x[0-9a-f]*, [0-9]* bytes into the config slots" "${log}"; then
    echo "control failed, but not with the flash-layout finding"
    rm -f "${log}"
    return 1
  fi
  if grep -qE 'error:|undefined reference|multiple definition|will not fit|overflowed' "${log}"; then
    echo "control failed with a compile or link error besides the flash-layout finding"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
  left="$(find build/m33/tests/regression/quad/L09/flash_layout -maxdepth 1 -type f -name "${target}*")"
  if [[ -n "${left}" ]]; then
    echo "control failed, but the image that failed the check was left: ${left}"
    return 1
  fi
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
  uv run python "${refdata}" ensure quad/L05/t3 ${refdata_force[@]+"${refdata_force[@]}"} --procs "${att_t3_procs}"
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
  if t3_control_cached quad/L05/t3 "${FUNCNAME[0]}" "${dir}/attitude_t3_inputs.txt" att_t3_reference_control att_t3_oracle_run; then
    rm -rf "${dir}" "${log}"
    return 0
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
  t3_control_store quad/L05/t3 "${FUNCNAME[0]}" "${log}"
  rm -rf "${dir}" "${log}"
}

# L11 T3: vertical_t3_oracle.py reads vertical_t3_inputs.txt and writes vertical_t3_golden.txt into --dir (decision 0020 section
# 5; the envelope and Q files are no longer generated, decision 0032); regenerated and checked as for L4 and L5 (decision 0011).
vert_t3_reference_dir=tests/regression/quad/L11/t3/reference
vert_t3_procs=4

vert_t3_oracle_run() {
  local dir="$1"
  uv run python "${vert_t3_reference_dir}/vertical_t3_oracle.py" --dir "${dir}" --procs "${vert_t3_procs}"
}

vert_t3_reference_reproduces() {
  uv run python "${refdata}" ensure quad/L11/t3 ${refdata_force[@]+"${refdata_force[@]}"} --procs "${vert_t3_procs}"
}

# The same run with vert_kp_v one float32 ulp up must fail the SHA256SUMS check, naming vertical_t3_golden.txt.
vert_t3_reference_control() {
  local dir log
  dir="$(mktemp -d)"
  log="$(mktemp)"
  uv run python - "${vert_t3_reference_dir}/vertical_t3_inputs.txt" "${dir}/vertical_t3_inputs.txt" <<'PY'
import struct
import sys

out = []
for line in open(sys.argv[1]):
    if line.startswith("vert_kp_v "):
        x = float.fromhex(line.split()[1])
        bits = struct.unpack("<I", struct.pack("<f", x))[0] + 1
        line = f"vert_kp_v {struct.unpack('<f', struct.pack('<I', bits))[0].hex()}\n"
    out.append(line)
open(sys.argv[2], "w").write("".join(out))
PY
  if cmp -s "${vert_t3_reference_dir}/vertical_t3_inputs.txt" "${dir}/vertical_t3_inputs.txt"; then
    echo "control setup failed: the perturbation did not change the inputs"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  if t3_control_cached quad/L11/t3 "${FUNCNAME[0]}" "${dir}/vertical_t3_inputs.txt" vert_t3_reference_control vert_t3_oracle_run; then
    rm -rf "${dir}" "${log}"
    return 0
  fi
  vert_t3_oracle_run "${dir}" || { rm -rf "${dir}" "${log}"; return 1; }
  if uv run python "${refdata}" verify quad/L11/t3 --dir "${dir}" 2>"${log}"; then
    echo "control passed the SHA256SUMS check from a perturbed input: the reproduction check cannot fail"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  cat "${log}"
  if ! grep -q "vertical_t3_golden.txt: sha256" "${log}"; then
    echo "control failed, but not on vertical_t3_golden.txt"
    rm -rf "${dir}" "${log}"
    return 1
  fi
  t3_control_store quad/L11/t3 "${FUNCNAME[0]}" "${log}"
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

g3_truth_vertical_unflagged_control() {
  g3_control host-debug g3_truth_vertical_unflagged_export G3-EXPORT "exports 'marv_truth_vertical_set'" \
    exports --manifest build/host-debug/tests/regression/quad/L11/controls/g3_control_truth_vertical_unflagged.json --nm nm
}

g3_truth_vertical_planted_control() {
  g3_control host-debug g3_truth_vertical_planted_export G3-EXPORT "exports 'marv_truth_vertical_planted'" \
    exports --manifest build/host-debug/tests/regression/quad/L11/controls/g3_control_truth_vertical_planted.json --nm nm
}

# A flag without the export is a tool-level error (exit status 2, G3-ERROR), not a violation (exit status 1), so g3_control
# does not fit: builds the planted target, requires exit status 2, a first line G3-ERROR naming the missing export, and
# no other G3 diagnostic.
g3_truth_vertical_missing_control() {
  local target=g3_truth_vertical_missing_export log status=0 build_log
  build_log="$(mktemp)"
  if ! cmake --build --preset host-debug --target "${target}" >"${build_log}" 2>&1; then
    cat "${build_log}"
    echo "control target ${target} did not build"
    rm -f "${build_log}"
    return 1
  fi
  rm -f "${build_log}"
  log="$(mktemp)"
  g3_check exports --manifest build/host-debug/tests/regression/quad/L11/controls/g3_control_truth_vertical_missing.json \
    --nm nm >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 2 ]]; then
    echo "control did not fail as a G3 tool error (exit ${status}): the truth_vertical flag without the export is not rejected"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q "^G3-ERROR " "${log}" || ! grep -qF -- "does not export marv_truth_vertical_set" "${log}"; then
    echo "control failed, but not with a G3-ERROR diagnostic naming marv_truth_vertical_set"
    rm -f "${log}"
    return 1
  fi
  local other
  for other in G3-SYMBOL G3-INCLUDE G3-EXPORT G3-VACUOUS; do
    if grep -q "^${other}" "${log}"; then
      echo "control failed with an unintended ${other} diagnostic"
      rm -f "${log}"
      return 1
    fi
  done
  rm -f "${log}"
}

# Tools tests that run in the full mode only (nightly and workflow_dispatch), not per push: the files whose alone time exceeds the
# 60 s per-push limit (owner ruling of 2026-10-03). Each time is the file run alone in the CI image (docker run --cpus 4 --memory
# 15740260352 on a quiet host, load < 1.5) after the S9 speed-ups, the session fixtures included. The per-push files closest
# to the limit stay per push. Per push these files are --ignore'd; the full mode
# runs them with the rest. S9, decision 0014 fifth round item 6.
nightly_tools_tests=(
  tests/regression/quad/L06/tools/test_chirp_admission.py   # 111.7 s alone. S9, decision 0014 fifth round item 6
  tests/regression/quad/L06/tools/test_notch_mixes.py       # 92.4 s alone. S9, decision 0014 fifth round item 6
  tests/regression/quad/L06/tools/test_attitude_lead.py     # 81.4 s alone. S9, decision 0014 fifth round item 6
  tests/regression/quad/L01/tools/test_flatten_report.py    # 64.5 s alone. S9, decision 0014 fifth round item 6
  tests/regression/quad/L05/tools/test_attitude_t3.py       # 60.7 s alone (60.5 s to 61.0 s in four runs). S9, decision 0014 fifth round item 6
)

tools_tests() {
  local skip=() f
  if [[ "${MARV_CI_MODE:-per-push}" != full ]]; then
    for f in "${nightly_tools_tests[@]}"; do
      skip+=(--ignore "${f}")
    done
  fi
  local guard=()
  if [[ "${MARV_CI_MODE:-per-push}" != full ]]; then
    guard=(-p pytest_file_time_guard)
  else
    # Decision 0022 item 5: workers in the merge run, one file per worker (module fixtures run once). The count is nproc capped
    # at 4 (the runner's CPUs, RUNNER_CPUS in ci/local_ci.sh; docker --cpus does not lower nproc); MARV_PYTEST_WORKERS overrides
    # it, 0 = serial. Per push stays serial: the time guard measures alone time in-process and cannot run under workers.
    local workers="${MARV_PYTEST_WORKERS:-}" cpus
    if [[ -z "${workers}" ]]; then
      cpus="$(nproc)"
      workers=$((cpus < 4 ? cpus : 4))
    fi
    if ((workers > 0)); then
      guard=(-n "${workers}" --dist loadfile)
    fi
  fi
  local paths=(tests/regression/quad/L00/tools tests/regression/quad/L01/tools tests/regression/quad/L03/tools tests/regression/quad/L04/tools tests/regression/quad/L05/tools tests/regression/quad/L06/tools tests/regression/quad/L11/tools tests/tools)
  if [[ -n "${MARV_PYTEST_PATHS:-}" ]]; then
    read -r -a paths <<<"${MARV_PYTEST_PATHS}"
  fi
  PYTHONPATH=tools/ci uv run pytest "${paths[@]}" -q ${guard[@]+"${guard[@]}"} ${skip[@]+"${skip[@]}"}
}

# Core 7.6: the blast-radius map stays in step with this script, ci/run_ci_gz.sh and the repository. A step no rule
# selects and that is not merge-only fails here, so a new step cannot silently drop out of the development check. It
# runs after host-debug's configure: its C++ cross-check reads build/host-debug/compile_commands.json.
blast_radius_manifest() {
  uv run python ci/affected.py --check-manifest
}

# S9 for pytest (decision 0014 fifth round item 6; closed in quad L6 stage (e), decision 0019 ruling 12 and the owner ruling
# S9 (a)): per push, the tools step loads tools/ci/pytest_file_time_guard.py, which measures every tools-test file's alone
# wall time (shared session fixtures charged in full to every file that uses them) against per_push_check_time_max, read
# from design/budget.yaml. The guard FAILS the step only under ci/local_ci.sh, which passes MARV_LOCAL_CI=1 into every job
# container (S9 was always measured in the local CI image and local CI is the definition of verified); everywhere else, in
# particular on GitHub Actions, it prints the per-file table and any over-limit files and leaves the exit status alone. It
# does not key on GITHUB_ACTIONS.
# Positive check: the guard passes on a short real file with the register limit. Negative control: the planted test
# (tests/regression/quad/L06/controls/test_per_push_pytest_overrun.py, outside every normal collection) sleeps three times
# the limit; MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE=1 (used only here, as the CTest control does) makes that seconds. The
# control sets MARV_LOCAL_CI=1 itself, so it proves the guard's detection in both settings: the session must fail through
# the guard, naming the planted file.
per_push_pytest_guard_applied() {
  local log status=0
  log="$(mktemp)"
  PYTHONPATH=tools/ci uv run pytest tests/regression/quad/L00/tools/test_check_claude_md.py -q -p pytest_file_time_guard >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -ne 0 ]] || ! grep -q 'FILE-TIME-GUARD PASSED' "${log}"; then
    echo "positive check failed: the guard does not pass a short file under the register limit"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

per_push_pytest_control() {
  local log status=0 planted=tests/regression/quad/L06/controls/test_per_push_pytest_overrun.py
  log="$(mktemp)"
  MARV_LOCAL_CI=1 MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE=1 PYTHONPATH=tools/ci uv run pytest "${planted}" -q -p pytest_file_time_guard >"${log}" 2>&1 || status=$?
  cat "${log}"
  if [[ ${status} -eq 0 ]]; then
    echo "control passed: a test file that overruns the limit does not fail the guard"
    rm -f "${log}"
    return 1
  fi
  if ! grep -q 'FILE-TIME-GUARD FAILED' "${log}" || ! grep -q "> 1 s  ${planted}" "${log}"; then
    echo "control failed, but not through the guard naming the planted file"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

step "uv sync --frozen" uv sync --frozen
step "L4: T3 oracle regenerates rate_t3_golden.txt and rate_t3_envelope.txt from rate_t3_inputs.txt, matching reference/SHA256SUMS" t3_reference_reproduces
step "L5: T3 oracle regenerates attitude_t3_golden.txt, attitude_t3_envelope.txt and attitude_t3_q.txt from their inputs, matching reference/SHA256SUMS" att_t3_reference_reproduces
step "L11: T3 oracle regenerates vertical_t3_golden.txt from vertical_t3_inputs.txt, matching reference/SHA256SUMS" vert_t3_reference_reproduces
step "tools tests (pytest tests/regression/quad/L00/tools tests/regression/quad/L01/tools tests/regression/quad/L03/tools tests/regression/quad/L04/tools tests/regression/quad/L05/tools tests/regression/quad/L06/tools tests/regression/quad/L11/tools tests/tools; the nightly list only in MARV_CI_MODE=full)" \
  tools_tests
if [[ "${MARV_CI_MODE:-per-push}" != full ]]; then
  step "per-push pytest guard (positive check: a short tools-test file passes the per-file time guard)" per_push_pytest_guard_applied
fi
step "G8: CLAUDE.md keeps the ACTIVE-spec, number, UNKNOWN and CI-gate sections" g8_check
step "GS-3: no tracked file outside the codec includes a MAVLink header or names MAVLink or an in-scope message" gs3_check
step "GCS: vitest (npm test) and svelte-check (npm run check) in gcs/" gcs_tests
step "L1: the committed vehicle card, its sensor profile and the design budget lint clean (sigma policy)" l1_card_lint
step "L1: plant known-answer reference reproduces plant_ref_expected.txt from plant_ref_inputs.txt" plant_ref_reproduces
step "L5: rate-bypass fixture reproduces from gen_fixture.py" rate_bypass_fixture_reproduces
step "L5: acro identity golden reproduces from fw/ at quad-L4-pass" rate_bypass_golden_reproduces
if [[ -n "${MARV_CI_BASE_REF:-}" ]]; then
  step "regression change check against ${MARV_CI_BASE_REF}" regression_change_check
fi
step "host-debug: configure, build, ctest" host_preset host-debug
step "core 7.6: ci/blast_radius.yaml maps every CI step, pytest path and gz group, cross-checked against the host-debug build and the Python imports" blast_radius_manifest
step "product parameters: the host-debug generated table equals the committed tests/regression/quad/L06/product_params table byte for byte" product_table_host
step "L1: the run report of the product parameter set (card, budget, provenance)" l1_report
step "G1: no unexplained numeric literals under fw/ (clang-tidy, token scan, NOLINT check)" g1_lint
step "constants: every constant in constants.hpp carries a citation and a physics/standard/math kind" constants_check
step "G3: no truth or harness symbol in any host flight library (nm -C)" g3_host_symbols
step "G3: no flight translation unit has a harness include directory (host)" g3_host_includes
step "G3: the SIL libraries export only marv_sil_* (nm -D)" g3_host_exports
step "per-push timeout: every ctest test of host-debug carries TIMEOUT = per_push_check_time_max (reference-set steps exempt)" per_push_timeout_applied
step "host-release: configure, build, ctest" host_preset host-release
step "m33: configure, build" m33_build
step "product parameters: the m33 (flight) generated table equals the committed table byte for byte" product_table_m33
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
step "GS-3 negative control (a planted file outside the codec that includes a MAVLink header must fail)" gs3_control
step "G3 negative control (planted marv_plant_ and marv::plant:: symbols in a flight library must fail the symbol check)" g3_plant_control
step "parameter-table negative control (a planted image linking the product table under the FC3v2 runtime must fail the post-link check, host)" param_table_host_control
step "parameter-table negative control (a planted image linking the product table under the FC3v2 runtime must fail the post-link check, m33)" param_table_m33_control
step "flash-layout negative control (a planted m33 image reaching the config slots must fail the post-link check)" flash_layout_control
step "L1 negative control (the core 2.1 example card without sigma must fail the card linter)" l1_card_lint_control
step "L1 negative control (a card with sigma = 0 on a published entry must fail the parameter set build)" l1_flatten_control
step "L1 negative control (a Python without PyYAML must fail the configure)" l1_pyyaml_control
step "per-push timeout negative control (a planted test that overruns the limit must fail ctest as a timeout)" per_push_timeout_control
step "per-push pytest negative control (a planted test file that sleeps three times the limit must fail the per-file time guard)" per_push_pytest_control
step "L1 negative control (perturbed plant reference inputs must not reproduce the committed expected file)" plant_ref_control
step "L4 negative control (a perturbed T3 input, kp one ulp up, must fail the SHA256SUMS check)" t3_reference_control
step "L5 negative control (a perturbed T3 input, att_kp one ulp up, must fail the SHA256SUMS check)" att_t3_reference_control
step "L11 negative control (a perturbed T3 input, vert_kp_v one ulp up, must fail the SHA256SUMS check on vertical_t3_golden.txt)" vert_t3_reference_control
step "L5 negative control (a perturbed rate-bypass input, kp one ulp up, must not reproduce the acro identity golden)" rate_bypass_golden_control
step "product parameters negative control (a committed table with one value changed must fail the byte-for-byte comparison)" product_table_control
step "G3 negative control (unflagged SIL library exporting marv_truth_state_set must fail the export check)" g3_truth_unflagged_control
step "G3 negative control (truth_state library also exporting marv_truth_planted must fail the export check)" g3_truth_planted_control
step "G3 negative control (unflagged SIL library exporting marv_truth_vertical_set must fail the export check)" g3_truth_vertical_unflagged_control
step "G3 negative control (truth_vertical library also exporting marv_truth_vertical_planted must fail the export check)" g3_truth_vertical_planted_control
step "G3 negative control (truth_vertical flag without the marv_truth_vertical_set export must be a G3-ERROR)" g3_truth_vertical_missing_control

echo "ALL STEPS PASSED"
