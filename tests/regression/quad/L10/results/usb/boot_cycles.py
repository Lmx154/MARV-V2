#!/usr/bin/env python3
"""Bench only (ground-segment unit 12a): reboot the board N times through the probe and read the first console seconds of each.

  boot_cycles.py <label> <cycles> <out_dir>   (MARV_PROBE_SERIAL set; board running the image under test)

Each cycle: tools/target/flash.sh --probe-only (halt, whole-chip watchdog reboot, no programming), wait for the console port, wait
12 s (ModemManager), capture 5 s with capture.py, and print the boot-to-IMU-start time, the last rate timing_faults, the last
attitude timing_faults and est_valid. Raw captures go to <out_dir>/<label>_<n>.log. Bounded: every wait has a limit.
"""
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00"
CAPTURE = ROOT / "tests/regression/quad/L10/results/sd/capture.py"


def main() -> int:
    label, cycles, out = sys.argv[1], int(sys.argv[2]), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    for n in range(1, cycles + 1):
        subprocess.run([str(ROOT / "tools/target/flash.sh"), "--probe-only"], env=os.environ, capture_output=True, timeout=60)
        time.sleep(3)
        deadline = time.time() + 30
        while not os.path.exists(PORT) and time.time() < deadline:
            time.sleep(0.5)
        time.sleep(12)
        log = out / f"{label}_{n}.log"
        subprocess.run([sys.executable, str(CAPTURE), PORT, str(log), "5"], capture_output=True, timeout=30)
        text = log.read_text(errors="replace") if log.exists() else ""
        start = re.search(r"IMU start at (\d+) us", text)
        rate = re.findall(r"rate executions/s=[\d.]+ executions=\d+ timing_faults=(\d+)", text)
        att = re.findall(r"attitude faults=(\d+) timing_faults=(\d+)", text)
        est = re.findall(r"est_valid=(\d)", text)
        print(f"{label} {n}: imu_start_us={start.group(1) if start else '-'} rate_timing_faults={rate[-1] if rate else '-'} "
              f"attitude_faults={att[-1][0] if att else '-'} attitude_timing_faults={att[-1][1] if att else '-'} "
              f"est_valid={est[-1] if est else '-'}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
