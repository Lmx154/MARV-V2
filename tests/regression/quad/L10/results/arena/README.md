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
| flight_log_at_slot.txt | U4-4 flight-log image (`MARV_FC3V2_FLIGHT_LOG=ON`) with its `MARV_FC3V2_RAM_FREE_BYTES` 0 | 102400 B | 55388 B |
| flight_log_151552.txt | U4-4 flight-log image as committed (83eae44) | 151552 B | 6236 B |
| emb6_open_151552.txt | U4-4 its measurement build (`MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN=ON`) at the same value | 151552 B | 5020 B |

The derivation (src/main.cpp): 102400 + 512 x floor((84688 - 2048 - 4096) / 512) = 102400 + 78336 = 180736 B. Here 2048 B is
`PICO_HEAP_SIZE` (Pico SDK 2.3.1 pico/platform.h, the `.heap` section the linker places at `__end__`) and 4096 B is the margin
`MARV_FC3V2_ARENA_MARGIN_BYTES`, a PLACEHOLDER (the reason and the replacement are in fw/hal/target/CMakeLists.txt). The
committed image keeps 6336 B free: the heap reservation, the margin and 192 B above them.

The flight image with the onboard logger (unit U4-4): its post-link L-5 check removes the image while the ring does not hold the
fast stream for the stall budget (the case at the 0.809 s entry until unit U4-7), so `measure_flight_log.sh <build tree>` links it
again with the build tree's own link command (`ninja -t commands`, up to the first post-link step) and runs `measure_ram.sh` on
it; the measurement build keeps its image and is measured directly. The derivation as above: 102400 + 512 x floor((55388 - 2048 -
4096) / 512) = 102400 + 49152 = 151552 B; the ring is the arena less the 16384 B staging, 135168 B (8448 cells, 135152 B usable).
The image keeps 6236 B free (92 B above the heap reservation and the margin); the measurement build, whose console trigger and
banner add about 1.2 KB, keeps 5020 B (2972 B above the heap reservation: inside the margin). Both images were built from 83eae44;
flight_log_at_slot.txt from 5a1a2f3 with the image wiring uncommitted (its version string is 7 bytes longer).
