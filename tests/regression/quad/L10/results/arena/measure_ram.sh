#!/usr/bin/env bash
# The free RAM of an m33 image (onboard logger unit U4-2, design-budget entry disarmed_arena_bytes): its linker symbols __end__
# (the start of the SDK heap reservation, after .bss) and __HeapLimit (the end of main RAM), their difference in bytes, the
# section sizes, and the arena (the store's scratch before unit U4-2) and the FatFs volume objects.
# Usage: measure_ram.sh <image.elf> [<nm> <size>]   (default arm-none-eabi-nm, arm-none-eabi-size)
set -euo pipefail
elf="$1"
nm="${2:-arm-none-eabi-nm}"
size="${3:-arm-none-eabi-size}"
echo "image: ${elf}"
syms="$("${nm}" "${elf}")"
grep -E ' (__end__|__HeapLimit|__StackBottom|__StackTop)$' <<<"${syms}"
end="$(awk '$3 == "__end__" {print $1}' <<<"${syms}")"
limit="$(awk '$3 == "__HeapLimit" {print $1}' <<<"${syms}")"
echo "free (__HeapLimit - __end__): $((0x${limit} - 0x${end})) B"
"${size}" -A "${elf}" | grep -E '^\.(ram_vector_table|text|data|bss|heap|stack_dummy) '
"${nm}" -S -C "${elf}" | grep -E ' (\(anonymous namespace\)::g_arena|\(anonymous namespace\)::g_store_scratch|marv::hal_target::\(anonymous namespace\)::g_fs|marv::\(anonymous namespace\)::g_fs)$' || true
