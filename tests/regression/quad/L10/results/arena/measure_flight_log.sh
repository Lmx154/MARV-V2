#!/usr/bin/env bash
# The free RAM of the flight image with the onboard logger (MARV_FC3V2_FLIGHT_LOG, onboard logger unit U4-4). Its post-link check
# L-5 (tools/ci/check_log_ring.py) removes the image while the ring does not hold the fast stream for the stall budget, which is
# the case at the 0.809 s budget entry until unit U4-7, so this script links the image again with the link command of the build
# tree's own rule (ninja -t commands, the command up to its first post-link step: the same objects, libraries, flags and linker
# script, no check), then runs measure_ram.sh on it. Nothing in the source tree is changed.
# Usage: measure_flight_log.sh <m33 build tree configured with -DMARV_FC3V2_FLIGHT_LOG=ON, built> [<nm> <size>]
set -euo pipefail
build="$(readlink -f "$1")"
here="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
elf="fw/hal/target/marv_fc3v2.elf"
cmd="$(ninja -C "${build}" -t commands "${elf}" | tail -n 1)"
link="${cmd%% && cd *}"
[[ "${link}" != "${cmd}" && "${link}" == *" -o ${elf} "* ]] || {
  echo "measure_flight_log.sh: no link command for ${elf} in ${build}" >&2
  exit 2
}
(cd "${build}" && eval "${link}")
echo "relinked: ${build}/${elf} (the build tree's link command, without its post-link checks)"
grep -E '^(MARV_FC3V2_FLIGHT_LOG|MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN):' "${build}/CMakeCache.txt"
bash "${here}/measure_ram.sh" "${build}/${elf}" "${2:-arm-none-eabi-nm}" "${3:-arm-none-eabi-size}"
