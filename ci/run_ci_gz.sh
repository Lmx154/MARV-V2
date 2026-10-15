#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"

# shellcheck source=ci/peakmem.sh
source ci/peakmem.sh

# MARV_CI_MODE: per-push (default; the development check, ci/dev_check.sh) or full (the merge gate: every Actions run and
# ci/local_ci.sh, core 7.6). In full mode every gz step runs everything. In any other mode gz_l4 and gz_l5 hold back the
# long files of gz_held_back (below) unless the diff against the development check's base (MARV_CI_BASE_REF, exported by
# ci/dev_check.sh) touches the file or a non-test_*.py file of its directory; with no base they run everything. The
# multi-seed Monte Carlo of L6 stage e hooks in here when it exists (owner decision: per push = frozen T4 at committed seeds).
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

gz_toolchain() {
  local versions dir
  versions="$(gz sim --force-version 8 --versions)"
  echo "${versions}"
  printf '%s\n' "${versions}" | grep -Eq '(^|[^0-9.])8\.[0-9]+' || {
    echo "gz sim --force-version 8 does not report 8.x"
    return 1
  }
  dir="$(mktemp -d)"
  cat >"${dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.28)
project(gz_toolchain LANGUAGES CXX)
find_package(gz-sim8 REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(gz-physics7 REQUIRED)
# A found package is not a usable one: the gz-sim8 config names CPPZMQ::CPPZMQ, and CMake reports a missing target only at
# generate time, for a target that links it. So a target links gz-sim8.
file(WRITE "${CMAKE_BINARY_DIR}/probe.cpp" "int main() { return 0; }\n")
add_executable(probe "${CMAKE_BINARY_DIR}/probe.cpp")
target_link_libraries(probe PRIVATE gz-sim8::gz-sim8)
message(STATUS "gz-sim8 ${gz-sim8_VERSION} sdformat14 ${sdformat14_VERSION} gz-physics7 ${gz-physics7_VERSION}")
CMAKE
  if ! cmake -G Ninja -S "${dir}" -B "${dir}/build" >"${dir}/cmake.log" 2>&1; then
    cat "${dir}/cmake.log"
    echo "cmake config for gz-sim8, sdformat14 or gz-physics7 not found"
    rm -rf "${dir}"
    return 1
  fi
  grep -F 'gz-sim8 ' "${dir}/cmake.log" | tail -n 1
  rm -rf "${dir}"
}

gz_build() {
  uv sync --frozen || return 1
  cmake --preset host-gz && cmake --build --preset host-gz
}

# Decision 0022 item 5: workers in both modes, one file per worker (module fixtures run once; every gz run has its own transport
# partition and temporary directory). The count is nproc capped at 4 (the runner's CPUs, RUNNER_CPUS in ci/local_ci.sh);
# MARV_PYTEST_WORKERS overrides it, 0 = serial.
pytest_no_skips() {
  local log workers="${MARV_PYTEST_WORKERS:-}" cpus par=()
  if [[ -z "${workers}" ]]; then
    cpus="$(nproc)"
    workers=$((cpus < 4 ? cpus : 4))
  fi
  if ((workers > 0)); then
    par=(-n "${workers}" --dist loadfile)
  fi
  log="$(mktemp)"
  uv sync --frozen || return 1
  if ! uv run pytest "$@" -q -rs ${par[@]+"${par[@]}"} 2>&1 | tee "${log}"; then
    rm -f "${log}"
    return 1
  fi
  if grep -Eq '(^|[^0-9])[0-9]+ skipped|^SKIPPED' "${log}"; then
    echo "a test was skipped: in the gz image, skipped means broken"
    rm -f "${log}"
    return 1
  fi
  rm -f "${log}"
}

# The long T4 files that the development check holds back (owner decision 2026-10-15); the merge run (MARV_CI_MODE=full) runs
# them all. The list is defined here only.
gz_held_back=(
  tests/regression/quad/L05/gz/test_t4_chirp.py
  tests/regression/quad/L05/gz/test_t4_yaw.py
  tests/regression/quad/L05/gz/test_t4_recovery.py
  tests/regression/quad/L04/gz/test_t4_chirp.py
  tests/regression/quad/L11/gz/test_t4_vertical.py
)

# Run a gz test directory. Outside full mode the held-back files of that directory get --ignore, except a file the diff
# against MARV_CI_BASE_REF touches itself or whose directory has a touched non-test_*.py file (conftest.py, helpers).
# MARV_CI_BASE_REF unset or unusable: no ignores. One line names what was held back and why.
pytest_gz_dir() {
  local dir="$1" mode="${MARV_CI_MODE:-per-push}" base="${MARV_CI_BASE_REF:-}"
  local ignore=() held=() f c d changed=() touched listing
  if [[ "${mode}" == full ]]; then
    echo "held back in ${dir}: none (MARV_CI_MODE=full)"
  elif [[ -z "${base}" ]]; then
    echo "held back in ${dir}: none (MARV_CI_BASE_REF unset, no base to diff against: running everything)"
  elif ! listing="$(uv run python -c '
import sys
sys.path.insert(0, "ci")
import affected
print("\n".join(affected.changed_paths(affected.resolve_base(sys.argv[1]), None)))' "${base}")"; then
    echo "held back in ${dir}: none (cannot list the diff against ${base}: running everything)"
  else
    mapfile -t changed <<<"${listing}"
    for f in "${gz_held_back[@]}"; do
      [[ "${f}" == "${dir}"/* ]] || continue
      d="${f%/*}"
      touched=""
      for c in ${changed[@]+"${changed[@]}"}; do
        if [[ "${c}" == "${f}" ]]; then
          touched="${c}"
        elif [[ "${c}" == "${d}"/* && "${c##*/}" != test_*.py ]]; then
          touched="${c}"
        fi
      done
      if [[ -z "${touched}" ]]; then
        ignore+=("--ignore=${f}")
        held+=("${f}")
      fi
    done
    if ((${#held[@]} > 0)); then
      echo "held back in ${dir} (MARV_CI_MODE=${mode}, not touched by the diff against ${base}; the full run runs them): ${held[*]}"
    else
      echo "held back in ${dir}: none (every held-back file here is touched by the diff against ${base}, or none is held back here)"
    fi
  fi
  pytest_no_skips "${dir}" ${ignore[@]+"${ignore[@]}"}
}

gz_plugin_smoke() {
  pytest_no_skips tests/regression/quad/L02/gz/test_plugin_smoke.py
}

gz_first_read() {
  pytest_no_skips tests/regression/quad/L02/gz/test_first_read.py
}

gz_determinism() {
  pytest_no_skips tests/regression/quad/L02/gz/test_determinism.py
}

# The plugin's sensor elements on the host-gz build (decision 0019, W1): the clock corner, the IMU model's seed
# determinism and the planted refusals.
gz_sensors() {
  pytest_no_skips tests/regression/quad/L06/gz
}

gz_runner_tools() {
  pytest_no_skips tests/regression/quad/L02/tools
}

gz_analytic() {
  pytest_no_skips tests/regression/quad/L02/gz/test_analytic.py
}

# The L4 plugin: the same plugin source on the l4_rate_scripted composition (MARV_GZ_SIL = marv_sil_l4_rate_scripted).
gz_build_l4() {
  uv sync --frozen || return 1
  cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4
}

# test_truth_gyro.py runs on the host-gz build (the L2 rotation world); test_t4_steps.py on the host-gz-l4 build.
gz_l4() {
  pytest_gz_dir tests/regression/quad/L04/gz
}

# The plugin's sensor elements on the host-gz-l4 build (decision 0019, W1 C3): run_l4's sensors passthrough (the world
# diff over every L4 scenario) and the accelerometer tie of ruling 11 (test e) with its static guard.
gz_sensors_l4() {
  pytest_no_skips tests/regression/quad/L06/gz_compositions/test_l4_sensors.py
}

# The L5 plugin: the same plugin source on the l5_attitude_scripted composition (MARV_GZ_SIL =
# marv_sil_l5_attitude_scripted, a TRUTH_STATE library), decision 0006 G.
gz_build_l5() {
  uv sync --frozen || return 1
  cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5
}

# The L5 T4 suite reads the T3 envelope and Q files, which are generated, not committed (decision 0011): tools/refdata
# regenerates them into build/reference (with MARV_CACHE_DIR set: into $MARV_CACHE_DIR/reference, reused when the stamp is
# current and the files match) and checks them against reference/SHA256SUMS. 4 = the runner's CPUs (ci/local_ci.sh).
gz_reference_l5() {
  uv run python tools/refdata/refdata.py ensure quad/L05/t3 --procs 4
}

# The L5 T4 suite (angle steps, truth plumbing, chirp, yaw release and fallback, recovery) on the host-gz-l5 build.
gz_l5() {
  pytest_gz_dir tests/regression/quad/L05/gz
}

# The same on the host-gz-l5 build (decision 0019, W1 C3): run_l5's sensors passthrough and test e.
gz_sensors_l5() {
  pytest_no_skips tests/regression/quad/L06/gz_compositions/test_l5_sensors.py
}

# The L11 plugin: the same plugin source on the l11_vertical_scripted composition (MARV_GZ_SIL =
# marv_sil_l11_vertical_scripted, a TRUTH_STATE and TRUTH_VERTICAL library), decision 0020 section 5 "CI cost".
gz_build_l11() {
  uv sync --frozen || return 1
  cmake --preset host-gz-l11 && cmake --build --preset host-gz-l11
}

# The L11 T4 suite reads the T3 envelope and Q files of the vertical loop, which are generated, not committed (decision 0011), as
# gz_reference_l5 above. 4 = the runner's CPUs (ci/local_ci.sh).
gz_reference_l11() {
  uv run python tools/refdata/refdata.py ensure quad/L11/t3 --procs 4
}

# The L11 gz suite: the <vertical_source> element and record 8 on the host-gz-l11 build, its planted refusals (one on the
# host-gz-l5 build) and the I1 guard (an L5 log on the host-gz-l5 build has no record 8 and its recorded records hash), and
# the T4 tests of the vertical loop (test_t4_vertical.py, held back from the development check as the long L05 files are).
gz_l11() {
  pytest_gz_dir tests/regression/quad/L11/gz
}

# Usage: ci/run_ci_gz.sh [l2|l4|l5|l11]. No argument runs every step (l2, l4, l5, l11 in order). The Actions jobs gz-l2,
# gz-l4, gz-l5 and gz-l11 run one group each, in parallel on separate runners, so a group must not depend on another
# group's build: gz_build is in l2 and in l4 (test_truth_gyro.py runs on the host-gz build), gz_build_l5 in l5 and in l11
# (the L11 suite's refusal and I1 tests run on the host-gz-l5 build).
group_l2() {
  step gz_toolchain gz_toolchain
  step gz_build gz_build
  step gz_plugin_smoke gz_plugin_smoke
  step gz_first_read gz_first_read
  step gz_sensors gz_sensors
  step gz_determinism gz_determinism
  step gz_runner_tools gz_runner_tools
  step gz_analytic gz_analytic
}

group_l4() {
  step gz_build gz_build
  step gz_build_l4 gz_build_l4
  step gz_l4 gz_l4
  step gz_sensors_l4 gz_sensors_l4
}

group_l5() {
  step gz_build_l5 gz_build_l5
  step gz_reference_l5 gz_reference_l5
  step gz_l5 gz_l5
  step gz_sensors_l5 gz_sensors_l5
}

group_l11() {
  step gz_build_l5 gz_build_l5
  step gz_build_l11 gz_build_l11
  step gz_reference_l11 gz_reference_l11
  step gz_l11 gz_l11
}

case "${1:-all}" in
  l2) group_l2 ;;
  l4) group_l4 ;;
  l5) group_l5 ;;
  l11) group_l11 ;;
  all)
    group_l2
    group_l4
    group_l5
    group_l11
    ;;
  *)
    echo "usage: ci/run_ci_gz.sh [l2|l4|l5|l11]" >&2
    exit 2
    ;;
esac

echo "ALL STEPS PASSED"
