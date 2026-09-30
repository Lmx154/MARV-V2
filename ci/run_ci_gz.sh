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

pytest_no_skips() {
  local log
  log="$(mktemp)"
  uv sync --frozen || return 1
  if ! uv run pytest "$@" -q -rs 2>&1 | tee "${log}"; then
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

gz_plugin_smoke() {
  pytest_no_skips tests/regression/quad/L02/gz/test_plugin_smoke.py
}

gz_determinism() {
  pytest_no_skips tests/regression/quad/L02/gz/test_determinism.py
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
  pytest_no_skips tests/regression/quad/L04/gz
}

step gz_toolchain gz_toolchain
step gz_build gz_build
step gz_plugin_smoke gz_plugin_smoke
step gz_determinism gz_determinism
step gz_runner_tools gz_runner_tools
step gz_analytic gz_analytic
step gz_build_l4 gz_build_l4
step gz_l4 gz_l4
