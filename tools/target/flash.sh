#!/usr/bin/env bash
# Flash the FC3v2 image over SWD with the Raspberry Pi Debug Probe (decision 0027 item 7). Never run by CI.
#
#   MARV_PROBE_SERIAL=<probe serial> tools/target/flash.sh [--probe-only] [elf]
#
# --probe-only connects, halts and prints the target's flash bank (size included), then resumes it without
# programming: use it to confirm the probe is wired to the FC3v2 (2 MB flash) before the first write.
# OPENOCD overrides the OpenOCD binary (default: raspberrypi/openocd sdk-2.3.0 build in ~/pico/openocd-install).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
probe_only=0
if [[ "${1:-}" == "--probe-only" ]]; then probe_only=1; shift; fi
elf="${1:-$root/build/m33/fw/hal/target/marv_fc3v2.elf}"
openocd="${OPENOCD:-$HOME/pico/openocd-install/bin/openocd}"
: "${MARV_PROBE_SERIAL:?set MARV_PROBE_SERIAL to the debug probe serial (ls /dev/serial/by-id/ shows it)}"

[[ -x "$openocd" ]] || { echo "flash: OpenOCD not found at $openocd (set OPENOCD)" >&2; exit 1; }
echo "flash: $("$openocd" --version 2>&1 | head -n 1)"

common=(-f interface/cmsis-dap.cfg -c "adapter serial $MARV_PROBE_SERIAL" -f target/rp2350.cfg -c "adapter speed 5000")
if (( probe_only )); then
  exec "$openocd" "${common[@]}" -c "init" -c "reset halt" -c "flash probe 0" -c "flash info 0" -c "resume" -c "exit"
fi

[[ -f "$elf" ]] || { echo "flash: no image at $elf (cmake --build --preset m33)" >&2; exit 1; }
echo "flash: image $elf sha256 $(sha256sum "$elf" | cut -d' ' -f1)"
exec "$openocd" "${common[@]}" -c "program $elf verify reset exit"
