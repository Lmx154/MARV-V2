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
   a second of boot. SAFETY: this image links the full flight composition and the motor outputs; it stays disarmed only because nothing arms it. During the pendulum keep the flight battery UNPLUGGED (the ESC unpowered; a battery strapped on for the mass is fine if its lead is disconnected) and the transmitter OFF, or remove the props. The receiver is not needed for logging.
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

# Unit U2b: the 30-minute soak, EMB-3 with the hooks, power cut, card off, card full

Bench evidence, not a frozen test. Board FC3v2 Rev-C, 64 GB FAT32 card, ESC unpowered, never armed (the bench trigger `log start` is the
only trigger; `arming=no_link` throughout), no motors. 2026-10-19. Branch `log-target-u2b` from `log-target` `cbf2eca`. Scripts:
`soak_analyze.py` (whole-file streaming decode through `tools/log/mlog.py`'s own decoder, strict), `faults.py` (power cut, card off, card full),
`emb3_capture.py`, `log_bench.py` (`--dump-timeout` added). The 568 MB soak files are NOT committed (too large): their sha256 and the
analysis are, the files are in `~/marv-bench-logs/u2b/`. Everything else raw is committed (xz).

Images (ELFs in `~/marv-bench-images/`, hashes there): soak 1 `u2b-soak-9adf3b7` (ELF sha256 `0dd433dd...cc14`, uninstrumented), soak 2
`u2b-soak2-727294a` (ELF `f8d81fad...d43e`, image hash `71366afd...3d49`; its banner reads `9adf3b7` because the version string is taken at
configure time), faults `u2b-faults-727294a` (16 MiB file), EMB-3 `u2b-emb3-727294a` (ELF `bc9b1f1f...bd27`, image hash `ede9ea97...1ff1`).
All: `cmake --preset m33 -B <dir> -DMARV_FC3V2_LOG_BENCH=ON -DMARV_FC3V2_LOG_BENCH_FILE_BYTES=<bytes>` (soak 1073741824, faults 16777216),
the EMB-3 one also `-DMARV_FC3V2_BENCH_STATS=ON -DMARV_FC3V2_LOG_RING_BYTES=249856`, Pico SDK sources from the m4 build tree, `-j4`, under the host lock.
Boot banner before the work: `marv_fc3v2 quad-L2-pass-532-g4c5470c` (flight image `907fd4b7...`, image hash `17bb03aa...`).

## 1. Soak (30 min of logging, 1 GiB file)

| Item | Soak 1 (`soak/soak1/`) | Soak 2 (`soak/soak2/`, adds `backlog_post_write_max`) |
| --- | --- | --- |
| File | 568007168 B, 1109389 sectors, all valid, one session, End record `disarm`, sha256 `a9fafbcf...e7e5` | 568105984 B, 1109582 sectors, all valid, one session, End `disarm`, sha256 in `soak2.mlog.sha256` |
| IMU records | 13530588 over 1805.115 s = 7495.69 Hz; spacing 132-135 us, 0 above 1.5 x median, 0 missing | 13532935 over 1805.455 s = 7495.58 Hz; same spacing, 0 missing |
| Gap records / drops while logging (imu, rate, manual) | 0 / 0, 0, 0 | 0 / 0, 0, 0 |
| Write failures, close failures | 0, 0 | 0, 0 |
| Writes (16 KiB chunks) | n 34668, max 162.46 ms, mean 12.62 ms | n 34674, max 161.58 ms, mean 12.61 ms |
| Write-time histogram, us:count | 10240-12288:422 12288-14336:33734 14336-16384:287 16384-20480:36 20480-24576:4 24576-28672:44 28672-32768:93 32768-40960:8 40960-49152:7 49152-57344:7 57344-65536:5 65536-81920:2 131072-163840:19 | 10240-12288:603 12288-14336:33609 14336-16384:240 16384-20480:20 20480-24576:7 24576-28672:45 28672-32768:97 32768-40960:9 40960-49152:13 49152-57344:6 57344-65536:5 65536-81920:3 131072-163840:17 |
| Longest time between two logger passes (`pass_max_us`) | 506945 us | 513688 us |
| Deepest backlog at the start of a pass (`backlog_max`) | 558 cells (status record) | 53 cells |
| Ring occupancy at the end of the worst write (`backlog_post_write_max`, cells of 16 B) | not measured | **5358 cells = 85728 B = 0.2730 s of the 314011.5 B/s stream** (of 16383 usable) |
| Ring high water (the ring's own, since boot) | 16383: saturated by the boot-time prepare, see below; says nothing about logging | same |

Host load average at the start 40.5 (soak 1, unlocked, other workers building) and 13.1 (soak 2), 1.7 at the end of soak 2; the measurements
are the board's own (its clock), the host only reads them. The soak ran unlocked: a lock would have waited hours behind other workers' checks.

`pass_max_us` is NOT the stall: a pass drains the ring continuously (one pass can chain several chunk writes), so a pass of 0.5 s did not fill the
ring. The ring's occupancy at the end of a write is the quantity EMB-6 sizes the buffer against. The ring's own `high_water` is since boot and
is 16383 (full) because the prepare after boot, and after each file closes, takes 1.0-1.1 s (`prepare_us`, mount plus f_expand of 1 GiB) while the
ring keeps filling: drops there are counted in Ready, before logging (soak 2 final line: 1547 imu records dropped in the next file's prepare).
That is outside logging; for the flight image it means a re-arm within about a second of a post-roll end finds the next file not yet prepared.

**Second instrumented run, the EMB-3 capture (600 s of logging, ring 249856 B, same logger code): `backlog_post_write_max` 9618 cells = 153888 B =
0.4901 s**, longest write 163.3 ms, `pass_max_us` 553273, 0 drops while logging. So the ring occupancy in the worst moment of two runs is 0.273 s
and 0.490 s: one run is not enough to call the worst case.

**Proposed replacement of the placeholder** `sd_log_write_stall_max_s` (0.809 s, unit 13): **0.491 s** (the maximum of the instrumented runs,
rounded up). Not applied to `design/budget.yaml`: see `budget_sd_log_write_stall_max_s.proposed.patch` (it applies with `git apply`) and the
report. The L-5 post-link check passes the bench image with it (ring 262144 B usable 262128 B against 314011.5 x 0.491 = 154180 B, margin
+344 ms), and fails the flight image's borrowed 98304 B scratch (98288 B against 154180 B: it covers 0.313 s), so EMB-6 is open for the flight image.

## 2. EMB-3 with the logger's tick hooks (`emb3/`)

Image: bench log image + `MARV_FC3V2_BENCH_STATS`, ring 249856 B (the log + stats combination does not link at the default 262144 B nor at
254080 B in this tree: `region RAM overflowed by 3744 bytes` at 254080; 249856 B links). Its L-5 check ran against a TEMPORARY budget value
0.274 s (the first soak-2 figure; ring usable 249840 B, margin +521.6 ms); the value was reverted in the tree. With the proposed 0.491 s the
same ring still passes (249840 B against 154180 B). Capture 660 s on the board: 30 s idle (Ready, hooks pushing), `log start` at +30 s, 600 s of
logging (190.6 MB written, 11633 writes, 0 write failures), `log stop` at +630 s, 30 s of tail. `tools/target/bench_stats.py report`:

| | With the logger hooks, logging (this run) | Telemetry at the cap (`../telemetry/`) | Pre-telemetry baseline (`../link_board/`) |
| --- | --- | --- | --- |
| T (mean interrupt-entry spacing) | 133.415 us | 133.402 us | 133.413 us |
| C = irq + account max, including start-up | **81.09 us** (after the first line 80.15 us) | 86.33 us | 80.55 us |
| U = C / T | **0.608** (margin +39.2 % of T) | 0.647 | 0.604 |
| R = C + B, B = 0.78 us, vs D = T | **81.86 us** (+51.55 us margin) | 87.09 us | 81.33 us |

EMB-3 PASS with the hooks active (+0.54 us C, +0.004 U over the baseline). Samples 4976750, `ovr spi partial clamp order empty` 0, rate-loop and
attitude `timing_faults` 0, `motor_drops` 0. The telemetry column is a different image (flight, no logger); the logger image runs no link service,
so the two costs have not been measured together (unit U4's gate).

## 3. Power cut mid-log (L-4): 12 resets, all PASS (`faults/run1/` cycles 1-3, `faults/extra/` cycles 1-9)

Method: `faults.py powercut`: `log start`, wait 7.3-15 s (a different phase each cycle), reset the board through the Debug Probe
(`tools/target/flash.sh --probe-only`: `reset halt` on the cores, then the chip's watchdog reboot), wait for the console, read the new boot's
`Ready` line, pull the cut file (16 MiB preallocated) whole and classify every sector. The SD card is NOT power-cycled (it stays powered by the
board's regulator); only the writer is cut. A real power cut needs the physical runbook below.

| Result over 12 cycles | |
| --- | --- |
| Valid contiguous prefix of the cut session vs the last console `cursor=` before the reset | prefix >= cursor in all 12 (the console lags by up to 1 s of writes) |
| Where the strict decoder stopped | at the first sector outside the prefix, named by sector number and reason: 9 times `magic` (never written), 3 times `crc` (cycles 2, 3 and 8 of the extra run: a torn sector with a good magic and a bad CRC, a write cut inside a chunk) |
| Sectors of the same session after the stop | 0 in all 12 (no write out of order) |
| Valid sectors of ANOTHER session after the stop (stale) | 0 in all 12: the region held no stale MLOG sectors, so the stale-session rejection was NOT exercised on the board (the U1 host tests cover it) |
| End record | absent in all 12, as expected; `ends_inside_record` False |
| Next boot | `Ready` with the next file index (file+1), no `Fault`, the cut file untouched and pullable |

## 4. Card absent (L-3) (`faults/run1/cardoff.txt`)

`log card off` simulates the removal at the storage (prepare answers NoCard, writes fail); it is not a removed card. The state machine, as the
logger names it (`Absent` is the "no card" state; the status line carries the counters):
- Case A, card off while Ready, 6 s: stays `Ready` (a Ready logger probes nothing). `log start`: `Ready -> Logging -> Fault` within 22 ms, `write_failures` 1, `sessions` 1. `log stop` (trigger off = disarmed): `Fault -> Absent` (close fails, `close_failures`), then a re-probe every 2 s (6 in 12 s, `absent` counter rising). `log card on`: `Absent -> Ready` with the next file.
- Case B, card off while Logging: `Logging -> Fault`, `write_failures` 2, `close_failures` 1 at the stop, ring drops 0, **0 re-probes while the trigger stayed on**, 5 after `log stop`; `log card on` brings `Ready`. The partial file (file 12) decodes up to its last completed write (3086 sectors, 37467 IMU records), no End record.
- Flight composition unaffected: over the whole run rate-loop `executions/s` 3747.2-3748.0, rate `timing_faults` 0, attitude `faults` 0 and `timing_faults` 0, no `motor_drops`; the logger never called `hal_panic` (the board kept printing).
- Not tested: a removed card (physical), a card that answers with errors, a full card (`prepare_full`), arming during the states (the bench trigger takes the place of `armed` in `service`; the board was never armed).

## 5. Card full (L-3) (`faults/run1/full.txt`)

Image with a 16 MiB file. `log start`: `Logging -> Full` when the file is used up at 16752128 B of 16777216 (two staging chunks kept in reserve for the End record). In `Full` the logger writes nothing more (cursor stays 16752128, 10 s later still `Full`), counts no ring drops (the ring is discarded to the pre-roll), the board keeps running (63 console lines, `timing_faults` 0). `log stop`: `Full -> Absent -> Ready` with the next file. The pulled file decodes clean: 32719 sectors valid, End record `full`. No panic. `prepare_full` (the card itself full) was not exercised.

## Physical runbook: a real card pull and a real power cut (needs Luis; not done)

Battery UNPLUGGED, ESC unpowered, transmitter off, no props, USB only. Flash the image of `u2b-faults-727294a` (16 MiB file) and open the console.
1. Wait for `log state=Ready`. Run `python3 tests/regression/quad/L10/results/log_bench/log_bench.py run --seconds 40 --out pull.mlog` in one shell but stop it after `run: logging for 40 s` appears (Ctrl-C), or send `log start` by hand on the console.
2. About 15 s into logging pull the SD card out of its socket. Expected on the console within one second: `log: Logging -> Fault`, `write_failures` >= 1, the imu status lines keep their cadence (`samples/s` about 7496, `timing_faults` unchanged), no reboot.
3. Send `log stop`: `Fault -> Absent`, then `probe found no card` every 2 s. Reinsert the card, wait for `Absent -> Ready` (a new file) and pull the cut file: `log_bench.py pull --index N --out cut.mlog`; decode with `python3 tools/log/mlog.py cut.mlog`: valid up to the last completed write, no End record.
4. Power cut: with the logger Logging, unplug the USB cable (board and card lose power together), replug, then pull the cut file the same way and decode it. Repeat three times at different moments. Expected: the decoder stops at a named sector, every sector before it is valid, other sessions' sectors (if any) are rejected.
Write the date, the card make, what the console showed and the decoder output into this directory.

## Noticed, not changed

- The prepare after boot and after each file closes takes 1.0-1.1 s for a 1 GiB file (169 ms for 64 MiB): longer than the ring holds (0.835 s), so the ring overflows in `Ready` (counted, not logging). U4: a re-arm just after a post-roll end.
- Every boot and every close reserves the next full-size file; an unused one is never truncated (the card fills with 1 GiB files in bench use: files 4 to 25 exist on the card now, the 16 MiB ones are small).
- `log_bench.py run` takes the first `log hist` line it sees; the full line is in the console log (the histogram in its printout may be cut).
