# Config slots in the FC3v2 board's internal flash, ground-segment unit 14 (decision 0053)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C (RP2354B, in-package W25Q16JV flash, 2 MB: OpenOCD reads
`w25q16jv id = 0x1540ef size = 2048 KiB in 512 sectors`), SD card in, Debug Probe on SWD, ESC unpowered, never armed, no
motor command (the console shows `arming=no_link` throughout: no receiver bound). Date 2026-10-18. Branch `gs-slots`.

Images (not committed; their hashes are):
- A: banner `quad-L2-pass-404-g1024e27`, ELF sha256 `21328734...6429`, image hash (flash bytes) `5d8a461f...ca9e`, 219164 B.
- B: banner `quad-L2-pass-408-gaa50aa4` (the same code built at a later commit: a harmless change, the git description),
  ELF sha256 `09dbce41...b664`, image hash `1a1462d4...1788`, 219164 B. Same parameter schema `0x1648260490893e6e`.

## Design as built

- Slots: `config_slot_bytes` (design/budget.yaml) = 98304 B = 24 x 4096 B sectors; rule: the smallest multiple of the erase
  sector (FLASH_SECTOR_SIZE) >= 2 x the largest generated config (`marv_params_fc3v2_eskf.mcfg`, 47224 B). Slot A at flash
  offset 0x1d0000 (XIP 0x101d0000), slot B at 0x1e8000, ending at 2 MB. `tools/config/mcfg.py encode --max-bytes` refuses any
  generated config above half a slot (49152 B). Post-link gate `tools/ci/check_flash_layout.py` (cmake/flash_layout.cmake):
  the image ends 1681380 B below slot A.
- Writes (fw/hal/target/src/storage_flash.cpp): only in `store::boot`, before the IMU, receiver and motor outputs start, then
  sealed. Erase sector by sector and program page by page through the SDK flash driver, interrupts disabled per call, the
  image running from SRAM; each erase and program read back; the store then checks the file's SHA-256, writes the commit
  record (sequence, length, SHA-256) and reads it back.
- Boot request: POWMAN SCRATCH0..7 (32 bytes, the config hash). The watchdog scratch registers were the design's choice, but
  only SCRATCH0..3 (16 bytes) are free of bootrom use on the RP2350 (datasheet 5.2.4: SCRATCH4..7 the boot vector,
  SCRATCH2..3 reboot parameters; the SDK's watchdog_reboot clears SCRATCH4). POWMAN SCRATCH survive every watchdog reset
  and are lost on power-on, brown-out and the RUN pin (datasheet table 528); not password protected (6.4). Deviation for the
  lead to accept.
- Reboot: `watchdog_reboot` after MARV_FC3V2_REBOOT_DELAY_MS (PLACEHOLDER 100 ms), so the command's acknowledgement already
  queued reaches the host (an immediate reset lost it: the first factory reset of the session rebooted without an
  acknowledgement).
- Image hash: SHA-256 of [__flash_binary_start, __flash_binary_end) on the SHA-256 accelerator (pico_sha256, CPU-fed);
  host side `tools/config/image_hash.py <elf|bin> [--bin-out <bin>]` (= `objcopy -O binary`); the validator's `--image` is
  that .bin, so its report's image hash is the board's.

- Store scratch: the flight image gives the store a full slot (98304 B) of working memory; flight image bss ends at
  0x2007c118, 16 KB below the heap limit 0x20080000 (no malloc in the image). The bench-statistics image (12.7 KB more bss)
  overflowed RAM by 1952 B with a full slot, so it gets half a slot (49152 B, every config the generator writes) and refuses
  a larger staged config at activation. The SD and USB bench images give it none and run no link service (the SD bench's
  buffers leave no room; it failed to link already at fd2f58b, by 25004 B alone and 41004 B with the statistics); the HIL image
  does not boot through the store.

## Results

| Done-when | Step (raw/) | Result |
| --- | --- | --- |
| (e) image hash | `e_hash` | Boot line, link identity, `image_hash.py` of the ELF and SHA-256 of `objcopy -O binary`: all `5d8a461f...ca9e`, 219164 B, 16157 us on the accelerator. PASS |
| (a) activation | `a3_activate` | att_kp staged one ulp up (0x4011145f) over the helper; `validate_config.py --image <A .bin>`: PASS, report `67dc4605...`, image hash A, config hash `7d7c605c...`; activate ACCEPTED; after the reboot: `source slot A sequence 1`, status ok, `boot request activate (done)`, bound to A; identity active, ok, config hash `7d7c605c...`, report id `67dc4605...`; att_kp active 0x4011145f; est_valid=1. PASS |
| (a) second activation | `a4_activate` | att_kp one ulp down (0x4011145d), report `3cae7988...` (PASS): `slot B sequence 2`, config `678d4788...`. PASS |
| (a) revert | `r2_revert` (after `a5_activate`: slot A sequence 4) | `slot A sequence 5`, config `7d7c605c...`, `boot request revert (done)`: the store copies the older intact slot over the newest one. PASS. `r1_revert` (slot B 2 -> B 3, same result) is marked FAIL only because the script then expected the other slot; its check was corrected. |
| (a) factory reset | `f1_factory`, `a2x_factory`, `c2_factory`, `c5_factory` | `source factory`, status ok, `boot request factory reset (done)`, identity config hash = factory .mcfg `5961b933...`. PASS |
| (b) flash.sh | `b1_flash_B` (slot A bound to A, flash image B over SWD) | OpenOCD erases to the image's last sector only (`Adding extra erase range, 0x10035900 .. 0x10035fff`); after the reboot the slot is kept: config hash `7d7c605c...`, `status refused: not validated (the config is not bound to this image's hash and a validation report)`, console `arming=config_refused`, identity state refused, refusal not validated. PASS |
| (b) picotool UF2 | `b2_uf2_A`, `b3_uf2_B` | Maintenance grant over the link (reboot command param1 3: IN_PROGRESS, ACCEPTED), then `picotool load -f -v -x <uf2>` (UF2 from `picotool uf2 convert`, binary 0x10000000..0x1003581c). A: the slot is bound again, status ok. B: refused, not validated. Config hash kept both times. PASS |
| (b) re-bound | `b4_rebind_B` | The same config (att_kp 0x4011145f, staged copy equal to the running one) validated for B's .bin (report `06d14f54...`) and activated on B: `slot B sequence 2`, bound to B, status ok. PASS |
| (c) corrupt slot | `c3_activate`, `c4_corrupt`, `c5_factory` | Injection: OpenOCD reads the newest slot's first sector, the script inverts the byte at file offset 200 (flash 0x1d00c8, 0xd9 -> 0x26), OpenOCD writes the sector back (`program ... verify`) and reboots. Boot: `source factory, status refused: corrupt (the slot's content does not match its commit record (SHA-256): the slot is corrupt)`, `arming=config_refused`, identity refused/corrupt; it stays so until the explicit factory reset (`c5_factory`: ok). PASS |
| (d) post-link gate | `d_flash_layout_control.log`; every m33 build log | The flight image passes (`check_flash_layout: marv_fc3v2.elf: image 0x10000000..0x1003581c (219164 bytes) ends 1681380 bytes below the config slots at 0x101d0000`); the planted control (1900544 B of read-only data) fails: `FLASH-LAYOUT: flash_layout_planted.elf: the image ends at 0x101d1588, 5512 bytes into the config slots`, ELF removed; `flash_layout_control` returns 0. PASS |

Boot time (`boot: IMU start at ... us after reset`, all after an SWD or USB-initiated reset, not a power-on):

| Boot | IMU start |
| --- | --- |
| m4 at fd2f58b (before this unit), factory | 10088, 10112 us (`boot_time_base_console*.log`) |
| this unit, factory, no slot | 26589, 26656 us (`boot_time_slots_console*.log`, image of bd4cf75 + the slot code), 26611 us (`e_hash`, image A): +16.5 ms, the image hash (16157 us) |
| this unit, a committed slot, no request | 86119, 86178, 90893 us: the store's software SHA-256 of the slot file and the decode (fw/store) |
| with a corrupt newest slot | 47462, 68281 us |
| applying an activation (SD read, verify, erase + program 24 sectors, read-back, commit) | 1113340 to 1223666 us |
| factory reset (erase of the committed slots) | 928712 to 1764326 us |

## Files

- `slots_common.py`, `bench_slots.py`: the PC side (helper on if02, console on if00, validator, flash.sh, picotool, OpenOCD).
  `bench_slots.py <step> <name>` writes `raw/<name>.json` (PASS/FAIL per check), console and ground logs.
- `raw/`: every step's JSON, console logs (`*.console*.log`), helper ground logs, flash/OpenOCD/picotool logs.
- `validation/<step>/`: the staged `.mcfg`, `report.json`, `validate.log` and the validator's work files of each activation.
- Not in the tree: the scripts' `images/` (the flash bytes as .bin given to the validator, the UF2 files given to picotool, the
  slot sectors read and written over SWD) and `raw/gs-runs/` (the helper's run directories; each step's ground log is in
  `raw/`). Their SHA-256:

```
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a1_activate.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a2_activate.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a3_activate.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a4_activate.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a5_activate.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  a6_activate.bin
5be2c44ec17752bd52593be44404ad3f5382bc59309bdae92593d5ebdf562e59  b2_uf2_A.uf2
70b40bb34ffec7c0fad7477bc92c23e82cfecdffd4b812b78d7d64076b2809b6  b3_uf2_B.uf2
1a1462d4e1dd8ec37a1a15499d45eae94c5fee46c7f12e8b85efc35f879b1788  b4_rebind_B.bin
08bdb5109eec82a8ee5e7da5619078b6d3e40f873743b206de64c54d84d53168  c1_corrupt.sector.bin
1a1462d4e1dd8ec37a1a15499d45eae94c5fee46c7f12e8b85efc35f879b1788  c3_activate.bin
659643d86bfbaf735c14d3cfb6f5f36b7b4f1047cb964ed468098f84a4e853be  c4_corrupt.sector.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  e_hash.bin
5d8a461fd3faa40c4e719f3264fb85ab40be286af8c79e92cef06ca7c89dca9e  e_hash.objcopy.bin
```

## Session notes

- `a1_activate` (PASS) and `a2_activate`: a2 staged att_kp two ulps up; the validator FAILED it (qf3_table_fc3v2: attitude
  phase margin 45 deg minus 3.4e-6 deg, below the 45 deg limit) and the script activated it anyway (the board does not see
  the verdict; the G7 gate is the ground station's). The script now refuses without a PASS (`a2x_factory` reset the board).
- `c1_corrupt`: the board refused the corrupt slot correctly; the script's parser missed the fault text's parentheses and
  stopped; `c1_corrupt_state` records that state, and `c3`-`c5` repeat (c) with the corrected parser.
- `A_flash.log`, `flash_e.log`, `boot_time_*`: flash.sh logs and console captures of the boot-time comparison.
- Final: the flight image of `0ff4131` (banner `quad-L2-pass-412-g0ff4131`, image hash `e9e48456...b105` = host) flashed with
  flash.sh after `c5_factory`: `source factory, status ok`, est_valid=1, rate and attitude timing_faults 0, IMU faults 0,
  usb_reset_refused 0 (`final_look.*`, `final_flash.log`).
