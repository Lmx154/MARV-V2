#!/usr/bin/env python3
"""FC3v2 arming-state trace over SWD, without halting the core (quad spec 4.0 M2 bench). Bench only; never run by CI.

  MARV_PROBE_SERIAL=<serial> python3 tools/target/arming_trace.py SECONDS RAW_OUT [--elf ELF] [--stall-imu-after S]

Attaches OpenOCD (no reset, no halt: a halted core pauses the RP2350 timer, DBGPAUSE) and polls the composition's
arming state byte as fast as the probe allows. On every change it records the timer (TIMERAWL, us) and the latest
receiver frame (stamp low word and frame count from the published ManualControl slot), so each failsafe stage is
timed against the firmware's own clock and frame stamp. The raw OpenOCD output goes to RAW_OUT; the decoded
transitions go to stdout.

--stall-imu-after S clears the IMU interrupt (IO_IRQ_BANK0) in core 0's NVIC after S seconds, so no sample is
delivered; the IMU read-deadline alarm stays enabled and must fire.

Addresses come from the ELF's symbols (arm-none-eabi-nm), so the ELF must be the flashed image.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

# marv::arming::ArmState, fw/arming/include/marv/arming/arming.hpp (enumerator order).
STATES = [
    "boot", "switch_off", "switch_not_cycled", "throttle_up", "imu_stale", "imu_invalid", "saturated", "no_link",
    "failsafe", "motor_test", "armed", "armed_hold", "armed_failsafe_centred", "estimator_invalid",
    "calibrating", "not_upright",
]
TIMER0_TIMERAWL = 0x400B0028  # RP2350 datasheet: TIMER0 base 0x400b0000, TIMERAWL offset 0x28
NVIC_ICER0 = 0xE000E180  # Armv8-M ARM: NVIC_ICER0, write 1 to clear an enable
IO_IRQ_BANK0 = 21  # pico-sdk src/rp2350/hardware_regs/include/hardware/regs/intctrl.h
MANUAL_CONTROL_SIZE = 104  # sizeof(marv::ManualControl): nm shows g_slot (2 slots) as 0xd0 bytes
FRAMES_OFFSET = 8  # ManualControl: TimeUs t_us (8 bytes), then std::uint32_t frames


def symbol(elf: str, pattern: str, size: int, at: int | None = None) -> int:
    out = subprocess.run(["arm-none-eabi-nm", "-SC", elf], check=True, capture_output=True, text=True).stdout
    hits = [ln.split() for ln in out.splitlines() if re.search(pattern, ln)]
    hits = [h for h in hits if len(h) >= 4 and int(h[1], 16) == size and (at is None or int(h[0], 16) == at)]
    if len(hits) != 1:
        sys.exit(f"arming_trace: symbol {pattern!r} of size {size} not unique in {elf}: {hits}")
    return int(hits[0][0], 16)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", type=float)
    ap.add_argument("raw_out", type=Path)
    ap.add_argument("--elf", default="build/m33/fw/hal/target/marv_fc3v2.elf")
    ap.add_argument("--stall-imu-after", type=float, default=None)
    a = ap.parse_args()

    state = symbol(a.elf, r"composition::.*g_arming_state$", 1)
    slot = symbol(a.elf, r"hal_target::.*g_slot$", 2 * MANUAL_CONTROL_SIZE)
    # Two files name an anonymous g_active; the receiver's is the one just before its g_slot (manual_control.cpp).
    active = slot - 8
    if symbol(a.elf, r"hal_target::.*g_active$", 4, at=active) != active:
        sys.exit(f"arming_trace: no g_active at 0x{active:x}, before g_slot 0x{slot:x}")
    stall_ms = -1 if a.stall_imu_after is None else int(a.stall_imu_after * 1000)
    tcl = f"""
proc rd {{addr}} {{ return [lindex [read_memory $addr 32 1] 0] }}
proc trace {{dur_ms stall_ms}} {{
  set t0 [ms]
  set last -1
  set n 0
  set stalled 0
  while {{1}} {{
    set now [ms]
    if {{$now - $t0 >= $dur_ms}} break
    if {{$stall_ms >= 0 && !$stalled && $now - $t0 >= $stall_ms}} {{
      mww {NVIC_ICER0:#x} {1 << IO_IRQ_BANK0:#x}
      echo "S [rd {TIMER0_TIMERAWL:#x}]"
      set stalled 1
    }}
    set s [expr {{[rd {state:#x}] & 0xff}}]
    set tr [rd {TIMER0_TIMERAWL:#x}]
    incr n
    if {{$s != $last}} {{
      set act [rd {active:#x}]
      set base [expr {{{slot:#x} + $act * {MANUAL_CONTROL_SIZE}}}]
      echo "T $s $tr [rd $base] [rd [expr {{$base + {FRAMES_OFFSET}}}]] $n [expr {{$now - $t0}}]"
      set last $s
    }}
  }}
  echo "N $n [expr {{[ms] - $t0}}]"
}}
"""
    serial = os.environ.get("MARV_PROBE_SERIAL")
    if not serial:
        sys.exit("arming_trace: set MARV_PROBE_SERIAL")
    openocd = os.environ.get("OPENOCD", str(Path.home() / "pico/openocd-install/bin/openocd"))
    with tempfile.NamedTemporaryFile("w", suffix=".tcl", delete=False) as f:
        f.write(tcl)
    cmd = [openocd, "-f", "interface/cmsis-dap.cfg", "-c", f"adapter serial {serial}", "-f", "target/rp2350.cfg",
           "-c", "adapter speed 5000", "-c", "init", "-f", f.name,
           "-c", f"trace {int(a.seconds * 1000)} {stall_ms}", "-c", "shutdown"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    a.raw_out.write_text(p.stdout + p.stderr)

    prev_t = None
    for ln in (p.stdout + p.stderr).splitlines():
        w = ln.split()
        if not w:
            continue
        if w[0] == "S":
            print(f"IMU interrupt cleared at timer {int(w[1], 0)} us")
        elif w[0] == "T":
            s, tr, stamp, frames = int(w[1], 0), int(w[2], 0), int(w[3], 0), int(w[4], 0)
            age = (tr - stamp) & 0xFFFFFFFF
            since = "" if prev_t is None else f"  +{(tr - prev_t) & 0xFFFFFFFF} us since previous"
            print(f"{STATES[s] if s < len(STATES) else s:>24}  timer {tr}  frame age {age} us  frames {frames}{since}")
            prev_t = tr
        elif w[0] == "N":
            n, ms = int(w[1]), int(w[2])
            print(f"polls {n} in {ms} ms: {1000 * ms / max(n, 1):.0f} us per poll (the timing resolution)")
    if p.returncode != 0:
        sys.exit(f"arming_trace: openocd exit {p.returncode}; see {a.raw_out}")


if __name__ == "__main__":
    main()
