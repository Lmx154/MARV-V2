#!/usr/bin/env bash
# Builds recovery_cause_tool (as ../step_cause/build_tool.sh builds step_cause_tool) against the host-gz-l5 build's firmware libraries (the libraries the Gazebo plugin links), with
# that build's flags (-std=c++20 -ffp-contract=off -g, compile_commands.json), and the SIL's override code compiled in as
# tests/regression/quad/L04/replay does. No CMake target is added (this directory is a results record, not a suite).
# Usage (repository root): tests/regression/quad/L06/results/r2_ff_diagnosis/build2.sh <output binary>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../../../../../.." && pwd)"
b="$root/build/host-gz-l5"
out="${1:?output binary}"
c++ -std=c++20 -g -ffp-contract=off -Wall -Wextra \
  -I"$root/fw/compositions/l5_attitude_scripted/include" -I"$root/fw/sil/include" -I"$root/fw/sil/src" \
  -I"$b/generated/marv_params_l5_attitude_scripted" -I"$root/fw/params/include" -I"$root/fw/hal/include" \
  -I"$root/fw/hal/sim/include" -I"$root/fw/types/include" -I"$root/fw/prim/include" -I"$root/fw/sched/include" \
  -I"$root/fw/attitude/include" -I"$root/fw/rate/include" -I"$root/fw/mixer/include" \
  -I"$root/fw/gyro_chain/include" -I"$root/fw/rate_group/include" \
  "$here/tool2.cpp" "$root/fw/sil/src/param_override.cpp" \
  "$b/fw/params/libmarv_params_l5_attitude_scripted_runtime.a" "$b/fw/params/libmarv_params_l5_attitude_scripted_generated.a" \
  "$b/fw/attitude/libmarv_attitude.a" "$b/fw/rate_group/libmarv_rate_group.a" "$b/fw/gyro_chain/libmarv_gyro_chain.a" \
  "$b/fw/rate/libmarv_rate.a" "$b/fw/mixer/libmarv_mixer.a" \
  "$b/fw/sched/libmarv_sched.a" "$b/fw/types/libmarv_types_instantiate.a" "$b/fw/hal/sim/libmarv_hal_sim.a" \
  "$b/fw/prim/libmarv_prim.a" "$b/fw/params/libmarv_params_l5_attitude_scripted_runtime.a" \
  -o "$out"
