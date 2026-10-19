# Onboard logger on the board, unit U2a: smoke capture and the pendulum procedure

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, SD card inserted (63.9 GB, FAT32), ESC unpowered, no motor command, no props.
Date 2026-10-19. Code: branch `log-target` (head after this change; `fbdad56` is the image's source), bench log image
`MARV_FC3V2_LOG_BENCH` (no link service, store scratch 0, 256 KiB ring, 16 KiB staging, `log_file_bytes` 64 MiB, all log parameters
PLACEHOLDER, see `fw/log/params/log_register.yaml`).

## Files

- `log_bench.py`: drives the image's `log` console commands and pulls the file over the link port. `analyze.py`: the numbers of a pulled
  file and the gyro CSV. Both are bench scripts, run by no test.
- `smoke1.mlog.xz`: the pulled file, 20503552 B (`xz -dk smoke1.mlog.xz`, sha256 of the decompressed file
  `12faae4f7834933d9b47ad957d0baf71494d9c39f925c1c5748388e7b977c883`). `smoke1.console.log`: the raw console during the pull.
  `analysis_smoke1.txt`: `analyze.py smoke1.mlog`.

## Result of the smoke capture (board still for the whole 60 s; it was NOT rotated)

| Item | Result |
| --- | --- |
| Boot | `Absent` to `Ready` with the 64 GB card: the first status line after boot reads `log state=Ready ... file=1 start_sector=8422656 capacity=67108864` (the boot console, including the transition line, was not captured: the board reboots before the port opens) |
| Capture | `log start` (bench trigger, nothing armed), 60 s, `log stop`; the logger went Logging, PostRoll (5 s PLACEHOLDER post-roll), closed the file (`close_us=46588`, `written=20503552`), prepared file 2 for the next run |
| File | 40046 sectors, all valid, one session `0xc88fa4284ff958fb`, End record `disarm`, decoded clean (`mlog.py` strict mode) |
| IMU records | 488243 over 65.126 s (60 s plus 5 s post-roll and the pre-roll) = **7496.9 Hz** (nominal 7500.3 Hz; the board's own sample counter reads 7497.0 /s) |
| IMU spacing | median 133 us, min 132 us, max 134 us; 0 spacings above 1.5 x median |
| Drops | 0 IMU, 0 rate-tick, 0 ManualControl; 0 gap records; 0 write failures |
| Ring | high water 3398 cells of 16384, deepest backlog before a drain 48 cells (`backlog_max`), longest time between two logger passes 31955 us (`pass_max_us`) |
| Worst write time (`log hist`) | `n=1251 fail=0 bytes=20496384 max_us=29161 mean_us=12476.1`; 10240-12288 us: 5, 12288-14336 us: 1236, 14336-16384 us: 7, 24576-28672 us: 1, 28672-32768 us: 2 (one write is one 16 KiB chunk) |
| `prepare` (mount, f_expand 64 MiB) / `close` | 169297 us / 46588 us, both while disarmed |
| Gyro, board still | mean/sd/peak-to-peak rad/s: x -0.00211/0.00185/0.01833, y +0.00428/0.00180/0.01588, z -0.00255/0.00140/0.01222 (the sensor's noise and bias) |
| Gyro response to rotation | **NOT SHOWN**: nobody rotated the board during this capture. Run the procedure below once and look at the peak-to-peak of each axis |
| ManualControl records | 0: no receiver link was on, so no frame arrived (the ring counter `pushed` third entry is 0) |

The 0.809 s `sd_log_write_stall_max_s` (budget entry, PLACEHOLDER) came from the unit-13 `hal_config_pending_write` (mount, mkdir, rename);
this image's data path writes straight into the preallocated region and its worst write here is 29.2 ms. One 60 s run does not replace
the 30-minute soak (U2b); the ring margin against 0.809 s is +25.8 ms (`tools/ci/check_log_ring.py`).

## Pendulum run: exact commands (from the repository root, board on USB with the Debug Probe, ESC not powered)

1. Build and flash the log image (one time; this replaces the flight image):

   ```sh
   uv sync --frozen
   cmake --preset m33 -B build/m33-log -DMARV_FC3V2_LOG_BENCH=ON
   cmake --build build/m33-log --target marv_fc3v2 -j4
   MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh build/m33-log/fw/hal/target/marv_fc3v2.elf
   ```

   The console is `/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00` and shows `log state=Ready` once a second after about
   a second of boot. The board is not armed in this image and nothing it does moves a motor (the receiver is not needed).
2. Log one axis (repeat for each of the three; every run makes a new file; the card has room for hundreds):

   ```sh
   python3 tests/regression/quad/L10/results/log_bench/log_bench.py run --seconds 60 --out pendulum_roll.mlog
   ```

   The script sends `log start`, waits 60 s (start the swing right after it prints `run: logging for 60 s`; trim to the swings later), sends
   `log stop`, waits for the file to close, pulls it over the link port (about 55 s for 20 MB) and prints the write-time histogram.
   Use a longer `--seconds` for a longer run: the file holds 64 MiB, about 210 s at 315 kB/s, and logging stops there. Exit 0 means
   the dump completed with every byte. If a pull fails, the file is still on the card: `... log_bench.py pull --index N --out file.mlog`
   (N is the `file=` number in the transition line).
3. Decode to gyro CSV (time, gyro x y z in rad/s, FRD body axes, at 100 Hz; omit `--gyro-rate-hz` for every 7.5 kHz record):

   ```sh
   python3 tests/regression/quad/L10/results/log_bench/analyze.py pendulum_roll.mlog --gyro-csv pendulum_roll.csv --gyro-rate-hz 100
   ```

   The first lines it prints must show `end record: yes`, the IMU rate near 7497 Hz and `drops imu/rate/manual 0/0/0`. For every record kind
   (`imu.csv`, `rate_tick.csv`, `status.csv`, `header.json`): `python3 tools/log/mlog.py pendulum_roll.mlog --csv-dir pendulum_roll_csv`.
4. Put the flight image back when done (`cmake --build --preset m33` for the normal image, then
   `MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh`). The flight image does not log yet (unit U4).

Do not unplug USB or reset the board while `run` is between `log stop` and the dump: a file whose End record was not written is
decodable up to the last completed write (strict mode stops at the first bad sector; `--resync` skips it), but the pull needs the
logger back in `Ready`.
