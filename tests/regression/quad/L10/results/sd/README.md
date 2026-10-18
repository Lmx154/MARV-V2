# SD card bench, ground-segment unit 13 (decision 0053)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, 32 GB (63.9 GB by CSD: see below) FAT32 card inserted, ESC unpowered,
no motor command. Date 2026-10-18. Code: branch `gs-sd`, commits `3dc9385` (FatFs) and `df50c16` (driver, bench).

## Files

- `capture.py`: time-bounded console capture (never an unbounded read). `analyze.py`: the numbers of a bench log.
- `sd_bench_run1.log`: raw console of the bench image (`MARV_FC3V2_SD_BENCH=ON`, `MARV_FC3V2_BENCH_STATS=ON`), flashed with
  `tools/target/flash.sh`, captured 12 s after the reboot with
  `capture.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00 sd_bench_run1.log 900 "sd_bench: op raw_read"`.
  `analysis_run1.txt` is `analyze.py sd_bench_run1.log`.
- `flight_boot1.log`: raw console of the normal flight image (card inserted) for 8 s after a reboot.

## Numbers

| Item | Result |
| --- | --- |
| Mount | SPI mode init 244.6 ms (CMD0 to CSD), type SDHC/SDXC, CSD 124735488 sectors of 512 B = 63864569856 B, FAT32, cluster 64 sectors, 1948483 clusters, 1948437 free |
| Write/verify | 1000 cycles of a 40256 B buffer through `hal_config_pending_write` + `_read` and raw FatFs write/sync/read: **0 errors** |
| `hal_config_pending_write` (mount, mkdir, temp write, sync, remove, rename, unmount) | max 809.1 ms, median 62.0 ms, min 55.9 ms |
| `hal_config_pending_read` (mount, open, read, unmount) | max 47.3 ms, median 40.3 ms |
| raw mount | max 15.2 ms, median 6.1 ms |
| raw `f_write` 40256 B | max 94.3 ms, median 37.7 ms |
| raw `f_sync` | max 73.3 ms, median 6.9 ms |
| raw `f_read` 40256 B | max 30.0 ms, median 29.7 ms |
| IMU interrupt-entry spacing, window max, idle before (10 windows) | 20104 cycles = 134.0 us |
| same, during the 1000 cycles (170 windows) | 20129 cycles = 134.2 us (+0.2 us) |
| same, idle after | 20073 cycles = 133.8 us |
| IMU counters over the whole run | 1682055 samples, ovr 0, spi 0, partial 0, clamp 0, order 0, empty 0 |
| Boot to IMU start, card inserted | 9940 us (bench image), 10007 us (flight image), `hal_time_us` just before `imu_start` |
| Flight image after the bench | `est_valid=1`, attitude `timing_faults=0`, rate `timing_faults=0` |

The worst single write stall is the 809 ms `pending_write` (one operation in 1000; the next-largest raw write is 94 ms): the
card's own busy time, which the driver waits for in the background loop only. It did not move the IMU spacing: the SD traffic
is polled SPI0 in thread mode below the IMU interrupt. The spacing floor at idle is 133 us (7500 Hz); the 0.2 us difference
is inside the idle windows' own spread (132.8 to 134.0 us).

Notes: the bench image (BENCH_STATS) shows rate `timing_faults=1` from its first console line, 10 s before the first SD
access (t = 18 s of the run); the flight image shows 0. Not investigated here.

## Pins and card detect

GPIO 34 SCK, 35 MOSI, 36 MISO, 39 CS (plain GPIO), SPI0: SPI0 for 34-36 CONFIRMED from the Pico SDK 2.3.1
`src/rp2350/hardware_regs/include/hardware/regs/io_bank0.h` (FUNCSEL 1: `GPIO34_..._SPI0_SCLK` line 4788,
`GPIO35_..._SPI0_TX` line 4924, `GPIO36_..._SPI0_RX` line 5059). The card answered in SPI mode with those pins; nothing else
was changed. Card detect GPIO 25 reads high with the pull-up and low with the pull-down (`pull_up=1 pull_down=0`, card
inserted), i.e. it is not driven: the jumper's effect is not visible, polarity unknown. Nothing gates on it.

## Runbook: boot with the card removed (Luis)

1. Flash the flight image (`cmake --build build/m33 --target marv_fc3v2`, `tools/target/flash.sh`), power off, remove the card, power on.
2. After 12 s run `python3 tests/regression/quad/L10/results/sd/capture.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00 flight_boot_nocard.log 8`
   and compare the `boot: IMU start at N us` line with 9940 / 10007 us above (the bench image with the card out also prints
   `sd_bench: card_detect ... pull_up= pull_down=` and `init FAILED`, which gives the card-detect level with no card).
