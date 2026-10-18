#!/usr/bin/env python3
"""Post-link check of the m33 flash layout (decision 0053 Q3; ground-segment unit 14): the image ends below the config slots.

The FC3v2 board keeps its two config slots (A and B, marv/hal/storage.hpp ConfigSlot) of --slot-bytes each (the design-budget
entry config_slot_bytes, design/budget.yaml) at the top of its --flash-bytes flash (PICO_FLASH_SIZE_BYTES, the board header),
outside every image, so that flashing an image never touches a stored config. This check reads the linked ELF's flash bytes
the way the board hashes them (tools/config/image_hash.py: its loadable segments at XIP addresses, contiguous from
__flash_binary_start to __flash_binary_end) and fails unless the image ends at or below the first slot's start,
XIP_BASE + flash_bytes - slots * slot_bytes.

Usage: check_flash_layout.py --elf <file> --flash-bytes <n> --slot-bytes <n> --slots <n> [--delete-on-failure]
Exit status: 0 the image is below the slots; 1 it reaches them, or its flash bytes cannot be determined (lines tagged
FLASH-LAYOUT; with --delete-on-failure the ELF is removed so no image that failed is left to flash); 2 usage or a file that
cannot be read.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "config"))
import image_hash  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--elf", type=Path, required=True)
    ap.add_argument("--flash-bytes", type=int, required=True)
    ap.add_argument("--slot-bytes", type=int, required=True)
    ap.add_argument("--slots", type=int, required=True)
    ap.add_argument("--delete-on-failure", action="store_true")
    a = ap.parse_args(argv)
    try:
        raw = a.elf.read_bytes()
    except OSError as e:
        print(f"check_flash_layout: {a.elf}: cannot read: {e.strerror}", file=sys.stderr)
        return 2
    slots_start = image_hash.XIP_BASE + a.flash_bytes - a.slots * a.slot_bytes
    findings = []
    if a.slots * a.slot_bytes > a.flash_bytes:
        findings.append(f"{a.slots} slots of {a.slot_bytes} bytes do not fit the {a.flash_bytes} byte flash")
    try:
        img = image_hash.flash_image_from_elf(raw)
        if img.end > slots_start:
            findings.append(f"the image ends at {img.end:#010x}, {img.end - slots_start} bytes into the config slots, which "
                            f"start at {slots_start:#010x} ({a.slots} x {a.slot_bytes} bytes at the top of the "
                            f"{a.flash_bytes} byte flash)")
    except image_hash.ImageError as e:
        findings.append(f"the image's flash bytes cannot be determined: {e}")
    if findings:
        for f in findings:
            print(f"FLASH-LAYOUT: {a.elf.name}: {f}")
        if a.delete_on_failure:
            a.elf.unlink(missing_ok=True)
            print(f"FLASH-LAYOUT: {a.elf.name}: removed")
        return 1
    print(f"check_flash_layout: {a.elf.name}: image {img.start:#010x}..{img.end:#010x} ({len(img.data)} bytes) ends "
          f"{slots_start - img.end} bytes below the config slots at {slots_start:#010x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
