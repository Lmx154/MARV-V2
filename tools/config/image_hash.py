#!/usr/bin/env python3
"""The image hash of an FC3v2 firmware image (decision 0053; ground-segment unit 14; core IF-1): the SHA-256 of the image's
flash bytes, the same bytes the board hashes at boot (fw/hal/target/src/flash_image.cpp) and reports as its image identity.

The image's flash bytes run from the XIP window's start (the linker's __flash_binary_start) to the linker's image end
symbol __flash_binary_end (the Pico SDK's section_flash_begin.incl and section_flash_end.incl). From an ELF they are its
loadable segments whose load (physical) addresses lie in the flash window, in address order; they must tile that range
with no gap (a gap would hold whatever the flash held before, which the host cannot know), so the bytes equal
`arm-none-eabi-objcopy -O binary` of the ELF. A .bin is taken as those bytes already.

  image_hash.py <image.elf | image.bin> [--bin-out <file>]

Prints `sha256 <hex>` and the byte count and range, and with --bin-out writes the flash bytes (the .bin the validator's
--image takes, tools/validate/validate_config.py, whose report image hash is the file's SHA-256). Exit 0, or 1 when the ELF
has no flash image or a gap, 2 when the file cannot be read.
"""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

# ELF constants (System V gABI, "ELF Header", "Program Header", "Sections", "Symbol Table").
ELF_MAGIC = b"\x7fELF"
EI_CLASS = 4
EI_DATA = 5
ELFCLASS32 = 1
ELFDATA2LSB = 1
PT_LOAD = 1
SHT_SYMTAB = 2
EHDR = struct.Struct("<16sHHIIIIIHHHHHH")
PHDR = struct.Struct("<IIIIIIII")
SHDR = struct.Struct("<IIIIIIIIII")
SYM = struct.Struct("<IIIBBH")

# The RP2350's XIP window base, where flash offset 0 appears, and the SRAM base, where the XIP windows and their aliases end
# (RP2350 datasheet section 2.2, address map; the pinned Pico SDK's hardware/regs/addressmap.h XIP_BASE and SRAM_BASE).
XIP_BASE = 0x10000000
SRAM_BASE = 0x20000000
START_SYMBOL = "__flash_binary_start"
END_SYMBOL = "__flash_binary_end"


class ImageError(Exception):
    pass


@dataclass(frozen=True)
class FlashImage:
    data: bytes  # the flash bytes from `start`
    start: int  # load address of the first byte (XIP_BASE for an image at flash offset 0)
    end: int  # one past the last byte: __flash_binary_end
    segments: tuple[tuple[int, int], ...]  # (load address, file size) of each loadable segment in the flash window


def _symbols(elf: bytes, shoff: int, shentsize: int, shnum: int) -> dict[str, int]:
    sections = [SHDR.unpack_from(elf, shoff + i * shentsize) for i in range(shnum)]
    out: dict[str, int] = {}
    for sh in sections:
        if sh[1] != SHT_SYMTAB:
            continue
        strtab = sections[sh[6]]
        names = elf[strtab[4]:strtab[4] + strtab[5]]
        for k in range(sh[5] // SYM.size):
            st_name, st_value, _size, _info, _other, st_shndx = SYM.unpack_from(elf, sh[4] + k * SYM.size)
            name = names[st_name:names.index(b"\0", st_name)].decode("ascii", "replace")
            if st_shndx != 0 and name in (START_SYMBOL, END_SYMBOL):
                out[name] = st_value
    return out


def flash_image_from_elf(elf: bytes) -> FlashImage:
    """The flash bytes of a 32-bit little-endian ELF: every loadable segment loaded at an XIP address."""
    if elf[:4] != ELF_MAGIC or elf[EI_CLASS] != ELFCLASS32 or elf[EI_DATA] != ELFDATA2LSB:
        raise ImageError("not a 32-bit little-endian ELF")
    h = EHDR.unpack_from(elf, 0)
    phoff, shoff, phentsize, phnum, shentsize, shnum = h[5], h[6], h[9], h[10], h[11], h[12]
    syms = _symbols(elf, shoff, shentsize, shnum)
    if START_SYMBOL not in syms or END_SYMBOL not in syms:
        raise ImageError(f"no {START_SYMBOL} / {END_SYMBOL} symbol (not a Pico SDK flash image)")
    start, end = syms[START_SYMBOL], syms[END_SYMBOL]
    segments = []
    for i in range(phnum):
        p_type, p_offset, _vaddr, p_paddr, p_filesz, _memsz, _flags, _align = PHDR.unpack_from(elf, phoff + i * phentsize)
        if p_type == PT_LOAD and p_filesz > 0 and XIP_BASE <= p_paddr < SRAM_BASE:
            segments.append((p_paddr, p_offset, p_filesz))
    segments.sort()
    if not segments:
        raise ImageError("no loadable segment in the flash window")
    data = bytearray()
    at = start
    for paddr, offset, size in segments:
        if paddr != at:
            raise ImageError(f"the flash bytes are not contiguous: a segment starts at {paddr:#010x}, the previous one ends "
                             f"at {at:#010x}")
        data += elf[offset:offset + size]
        at = paddr + size
    if at != end:
        raise ImageError(f"the loadable segments end at {at:#010x}, {END_SYMBOL} is {end:#010x}")
    return FlashImage(bytes(data), start, end, tuple((p, s) for p, _o, s in segments))


def flash_image(path: Path) -> FlashImage:
    raw = Path(path).read_bytes()
    if raw[:4] == ELF_MAGIC:
        return flash_image_from_elf(raw)
    return FlashImage(raw, XIP_BASE, XIP_BASE + len(raw), ((XIP_BASE, len(raw)),))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image", type=Path, help="the image: an ELF, or the .bin of its flash bytes")
    ap.add_argument("--bin-out", type=Path, help="write the flash bytes here")
    a = ap.parse_args(argv)
    try:
        img = flash_image(a.image)
    except OSError as e:
        print(f"image_hash: {a.image}: cannot read: {e.strerror}", file=sys.stderr)
        return 2
    except ImageError as e:
        print(f"image_hash: {a.image}: {e}", file=sys.stderr)
        return 1
    if a.bin_out is not None:
        a.bin_out.parent.mkdir(parents=True, exist_ok=True)
        a.bin_out.write_bytes(img.data)
    print(f"sha256 {hashlib.sha256(img.data).hexdigest()}")
    print(f"bytes {len(img.data)} flash {img.start:#010x}..{img.end:#010x} ({a.image})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
