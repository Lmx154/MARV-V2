#!/usr/bin/env bash
# Flash the FC3v2 image over SWD with the Raspberry Pi Debug Probe (decision 0027 item 7). Never run by CI.
#
#   MARV_PROBE_SERIAL=<probe serial> tools/target/flash.sh [--probe-only] [elf]
#
# --probe-only connects, halts and prints the target's flash size (flash probe), then reboots it without
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
# Whole-chip reboot through the watchdog, as the SDK's watchdog_reboot does: PSM WDSEL selects every domain but the
# oscillators, then WATCHDOG CTRL TRIGGER. OpenOCD's reset restarts the cores only, so the USB controller keeps its
# session and the host never re-enumerates the console. Addresses and bits from the pinned SDK's
# hardware/regs/{addressmap,psm,watchdog}.h (PSM_BASE + PSM_WDSEL_OFFSET, PSM_WDSEL_BITS & ~(ROSC|XOSC),
# WATCHDOG_BASE + WATCHDOG_CTRL_OFFSET, WATCHDOG_CTRL_TRIGGER_BITS). The chip resets mid-command, so OpenOCD's
# shutdown reports a failed access; that is expected.
reboot=(-c "targets rp2350.cm0" -c "halt"
        -c "write_memory 0x40018008 32 [expr {0x01ffffff & ~0xc}]"
        -c "write_memory 0x400d8000 32 [expr {[read_memory 0x400d8000 32 1] | 0x80000000}]"
        -c "shutdown")
if (( probe_only )); then
  log="$("$openocd" "${common[@]}" -c "init" -c "reset halt" -c "flash probe 0" "${reboot[@]}" 2>&1)" || true
  echo "$log"
  grep -q 'size = [0-9]* KiB' <<<"$log" || { echo "flash: no flash bank found" >&2; exit 1; }
  exit 0
fi

[[ -f "$elf" ]] || { echo "flash: no image at $elf (cmake --build --preset m33)" >&2; exit 1; }
echo "flash: image $elf sha256 $(sha256sum "$elf" | cut -d' ' -f1)"
log="$("$openocd" "${common[@]}" -c "program $elf verify" "${reboot[@]}" 2>&1)" || true  # the reboot ends in an error
echo "$log"
grep -q '\*\* Verified OK \*\*' <<<"$log" || { echo "flash: verify did not pass" >&2; exit 1; }
