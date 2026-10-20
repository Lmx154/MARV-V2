# The disarmed arena: free RAM of the flight image (onboard logger unit U4-2)

Evidence for `MARV_FC3V2_RAM_FREE_BYTES` (fw/hal/target/CMakeLists.txt), the measured input from which src/main.cpp derives
the flight image's arena. Read by no test.

- Script: `measure_ram.sh <elf>`. It prints the linker symbols `__end__` (the start of the SDK heap reservation, after `.bss`)
  and `__HeapLimit` (the end of main RAM), their difference, the section sizes, the arena (the store's scratch before U4-2) and
  the FatFs volume object.
- Inputs: `marv_fc3v2.elf` of the default m33 build (`cmake --preset m33 && cmake --build --preset m33`), copy_to_ram, so
  `.text` and `.data` sit in main RAM with `.bss`.

| File | Image | Arena | Free RAM |
| --- | --- | --- | --- |
| base_b7b41f1.txt | b7b41f1, before U4-2 (`g_store_scratch`) | 102400 B | 85076 B |
| arena_at_slot.txt | U4-2 with `MARV_FC3V2_RAM_FREE_BYTES` 0 (the working memory alone) | 102400 B | 84688 B |
| arena_180736.txt | U4-2 as committed | 180736 B | 6336 B |

The derivation (src/main.cpp): 102400 + 512 x floor((84688 - 2048 - 4096) / 512) = 102400 + 78336 = 180736 B. Here 2048 B is
`PICO_HEAP_SIZE` (Pico SDK 2.3.1 pico/platform.h, the `.heap` section the linker places at `__end__`) and 4096 B is the margin
`MARV_FC3V2_ARENA_MARGIN_BYTES`, a PLACEHOLDER (the reason and the replacement are in fw/hal/target/CMakeLists.txt). The
committed image keeps 6336 B free: the heap reservation, the margin and 192 B above them.
