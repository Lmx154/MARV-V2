# 0027: Pico SDK pin and the target build (M1)

## What changed

No file under `tests/regression/` changes in this record. It fixes how M1 (quad spec §4.0) builds for the target:

1. **SDK:** Pico SDK 2.3.1, commit `079c6f39023649b154152db30f1d781e884879bc`
   ([release](https://github.com/raspberrypi/pico-sdk/releases/tag/2.3.1)), and TinyUSB at that release's submodule
   commit `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` (0.18.0). Fetched by `cmake/pico_sdk.cmake` with FetchContent from
   the GitHub archive URLs with SHA-256 hashes (the `cmake/googletest.cmake` pattern); configure fails unless the SDK
   reports version 2.3.1.
2. **No picotool, no UF2** (`PICO_NO_PICOTOOL=1`): the build output is the ELF, flashed over SWD.
3. **Board:** a custom header in a repository-root `boards/` directory (outside `fw/`; pins are hardware
   configuration, like `vehicles/`), selected with `PICO_BOARD` and `PICO_BOARD_HEADER_DIRS`. It declares the B package
   (`PICO_RP2350A 0`) and 2 MB of flash, defines no default UART, and cites https://madflight.com/Board-FC3/ for every
   pin. The chip is an RP2354B (marking 2354B0A4); its in-package 2 MB flash is the only flash (Luis).
4. **Platform and toolchain:** `PICO_PLATFORM=rp2350-arm-s`, `PICO_HARD_FLOAT_ABI=1`; the SDK's
   `pico_arm_cortex_m33_gcc` toolchain replaces `cmake/toolchain-m33.cmake`; the compiler stays
   `gcc-arm-none-eabi 15:13.2.rel1-2` (`ci/Dockerfile`), checked at configure.
5. **Containment:** only `fw/hal/target` (the `marv_hal_target` library and the image's `main.cpp`) links SDK targets;
   flight modules never see SDK include directories (CLAUDE.md, "no Pico SDK headers above the HAL").
6. **Console:** USB CDC (`pico_enable_stdio_usb 1`, `pico_enable_stdio_uart 0`). Console output never runs in the tick
   (USB stdio can block on output); the USB interrupt load counts in the EMB-3 timing measurement.
7. **Flashing:** `tools/target/flash.sh`, OpenOCD (raspberrypi/openocd `sdk-2.3.0`, the build on Luis's host) through
   the Raspberry Pi Debug Probe: `program <elf> verify reset exit`. Never part of CI.
8. **CI:** the existing `m33_build` step also links the ELF and prints its size and ABI tags. No new job.

Build order: P1 SDK pin and toolchain swap; P2 `hal_target` and the linked ELF; P4 the flash script and the first
flash on the bench; then P3, the CI containment checks (no SDK include directory and no console calls in flight
targets outside `fw/hal/target`).

## Why

M1 needs a reproducible target build. The stock `pico2` board cannot serve the FC3v2: it declares the A package
(GPIO 0–29, rejecting the IMU's SCK and MOSI on GPIO 30 and 31), sends its default UART to GPIO 0/1 (the receiver
port), and sets 4 MB of flash. A hashed archive pins content, works with the fresh clones of `ci/local_ci.sh` and
Actions without submodules, and matches the existing googletest pattern. 2.3.1 over 2.3.0: it fixes races in
`pico_sync` and the timer alarm pool, with the same TinyUSB.

Rejected: a git submodule (every clone path must recurse, TinyUSB nested); FetchContent from git at a SHA (full
histories); an environment path checked by version only (does not pin content); the SDK baked into the CI image
(host and CI differ); keeping our toolchain file (unsupported by the SDK); softfp (changes the m33 ABI).

## Evidence

The architect's proposal of 2026-10-15 (lead session); the archive hashes are re-computed by the implementer. Open
risks carried into M1: OpenOCD `rp2350.cfg` on this board is INFERRED until the first "Verified OK"; whether the image
pulls `malloc` (EMB-4) is UNKNOWN until P2 links; what `hal_panic` does to the motors is decided before the DShot work.

## Approval

Luis, 2026-10-15, verbatim: "1. yes use 2.3.1 2. yes 3. yes, skip them 4. 2354B0A4, no additional flash chip on the
board, so the only flash we have is what's on the chip itself."
