# Telemetry streams on the FC3v2 board, ground-segment unit 11a (EMB-3 with every stream at its cap)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, SD card in, ESC unpowered, never armed, no motor command (the console
shows `arming=no_link` throughout: no receiver bound). Date 2026-10-19. Branch `tele-bench` from `m4` at `4c5470c` (contains the
merged telemetry); firmware tree clean at the build (banner `quad-L2-pass-532-g4c5470c`, no `-dirty`).

## Images (not committed; their hashes are)

| Image | Build | ELF sha256 | Image hash (flash bytes, boot line) | Size |
| --- | --- | --- | --- | --- |
| bench statistics (EMB-3 capture) | `cmake --preset m33 -B build/m33-bs -DMARV_FC3V2_BENCH_STATS=ON` | `81e877ca...c0eb` | `46d0f375...56a9` | 208268 B |
| flight `marv_fc3v2` (left on the board) | `cmake --preset m33` | `907fd4b7...592c` | `17bb03aa...acfae` | 204964 B |

Full values: ELF `81e877caf57ed45d91ebf2f9e9c93ef8933793c5e416e2d182e2a7583768c0eb` and
`907fd4b7cc665a827d4686bfe8d05b848eea29a9f2c6a8963265f8760879592c`; image `46d0f3754a6d757918902f0cf385844731932dfeb464a71389752d2a6e7a56a9`
and `17bb03aa722ebdc49ce895465a57d1c2e995754914bb34b606d56300ca5acfae`. Both pass `check_param_table` (flight and bench-stats:
155 records, schema `0x0fd34ce78b31e33f`) and `check_flash_layout` (bench-stats image ends 0x10032d8c, 1684084 B below the
slots at 0x101ce000). Both built with the Pico SDK 2.3.1 and TinyUSB sources copied from the `m4` build tree (the `wt-tele` build
tree that held them had been removed), `-j4`, under the host lock.

## Procedure (what was run)

1. Boot console read before anything (`tools/target/bench_stats.py capture <if00> 4`): the board ran the bench log image
   (`marv_fc3v2 quad-L2-pass-439-g3478327-dirty`, image hash `0f78d468...80e0`, `tick quad_fc + onboard SD log (MARV_FC3V2_LOG_BENCH)`,
   `config: source factory, status ok, boot request none`). Config slots: no old or incompatible config was found at any flash, so
   no factory reset was needed or done (see below).
2. `MARV_PROBE_SERIAL=900678EF479C3A0E tools/target/flash.sh build/m33-bs/fw/hal/target/marv_fc3v2.elf` (`** Verified OK **`), 14 s
   wait, banner: `marv_fc3v2 quad-L2-pass-532-g4c5470c`, `image: sha256 46d0f375...`, `config: source factory, status ok, boot request
   none`, `bench stats: on (MARV_FC3V2_BENCH_STATS, DWT CYCCNT at clk_sys)`.
3. Under the host lock (`ci/heavy.sh --wait 10800 -- ...`; taken for the capture because the host-side read must not stall under
   other workers' builds: a stalled reader would show as dropped frames): `uv run python
   tests/regression/quad/L10/results/telemetry/bench_telemetry.py capture tests/regression/quad/L10/results/telemetry/raw/telemetry_load`
   (defaults: 20 s closed, 20 s open, 60 s load, 20 s after) and `... analyze .../raw/telemetry_load`.
   Host load average at the start of the capture 13.9 (other workers' processes outside the lock), 2.2 at its end.
4. Flight image flashed back (same command, `build/m33/...`), banner read (below).

## Results

Requested: every stream at 1 us; granted its cap `tlm_hz_* = 198 Hz` (interval 5051 us = 197.98 Hz), `MESSAGE_INTERVAL` read back for
each of the ten.

| Quantity | Result |
| --- | --- |
| Load phase | 60.000 s, 118841 frames, 6629138 B, 0 sequence gaps, 60 heartbeats |
| Frame rate | 1980.7 frames/s (ten streams x 197.98 Hz + 1 Hz heartbeat) |
| Byte rate | 110485 B/s = 73.7 % of `telemetry_byte_rate_max` 150000 B/s (PASS). The budget counts every frame at its longest (198 Hz x 755 B = 149490 B/s); the measured frames are shorter (MAVLink 2 drops trailing zero bytes) |
| Rate per stream, board stamps | ATTITUDE, ATTITUDE_QUATERNION, RC_CHANNELS, ATTITUDE_TARGET 197.980 Hz; HIGHRES_IMU 197.878 Hz; ACTUATOR_OUTPUT_STATUS 197.980 Hz; MARV_FC_STATUS 197.981 Hz; MARV_ESTIMATOR 197.980 Hz (requested cap 197.98 Hz); SYS_STATUS and MARV_IDENTITY carry no stamp: 197.95 and 197.98 Hz by host count |
| Frames per stream | 11879 each (SYS_STATUS 11877, HIGHRES_IMU 11872); 60 s x 197.98 Hz = 11879 |
| Stamp gaps | between 533 us and 10.14 ms at ms or us stamp resolution (`gap_min_us`/`gap_max_us` in `analysis.txt`) |
| IMU samples | 2414228 in the capture, `ovr`, `spi`, `partial`, `clamp`, `order`, `empty` all 0 |
| Rate-loop `timing_faults` | 1 at the first line and 1 at the last (the boot-time count of the m4 image, as in `../link_board/README.md`); no fault during the capture |
| Attitude `timing_faults` | 0; `est_valid` 1; `usb_reset_refused` 0 |

IMU interrupt timing, window maxima (1 s windows) per phase:

| Phase | Entry spacing max | IMU interrupt `irq` max | USBCTRL chain max |
| --- | --- | --- | --- |
| closed (link port not open) | 134.11 us | 81.63 us | 88.46 us |
| open (heartbeat only) | 134.15 us | 81.65 us | 53.91 us |
| load (ten streams at the cap) | 134.17 us | 81.66 us | 89.07 us |
| after | 134.09 us | 81.66 us | 89.63 us |

EMB-3 over the whole capture (`analysis.txt`, `tools/target/bench_stats.py report`), against the pre-telemetry baseline (the link
unit's armed run at `3184b1f`, `../link_board/README.md`: parameter listings at 430.7 kB/s):

| | Telemetry at the cap | Pre-telemetry baseline | Bound |
| --- | --- | --- | --- |
| T (measured mean interrupt-entry spacing) | 133.402 us | 133.413 us | |
| C = irq + account max, including start-up | 86.33 us | 80.55 us | |
| C after the first line | 85.53 us | | |
| U = C / T | 0.647 (margin +35.3 % of T) | 0.604 (+39.6 %) | n = 1 bound 1.000 (core EMB-3) |
| RTA R = C + B, B = 0.76 us | 87.09 us (margin +46.31 us to D = T) | 81.33 us (+52.09 us) | R <= D = T |
| R after the first line | 86.29 us (+47.11 us) | | |

Verdict against the budget: EMB-3 PASS with every telemetry stream at its cap, by utilisation (U 0.647 <= 1.000) and by exact
response-time analysis (R 87.09 us <= T 133.40 us). The byte budget holds (110.5 kB/s <= 150 kB/s). C grew by 5.78 us and U by
0.043 against the baseline. The growth is NOT attributed to telemetry: the closed phase, with no link activity at all, already
shows `irq` max 81.63 us, so it is in every phase; the baseline image is `3184b1f` and the `m4` image since then also carries
the angle mode, the rate feed-forward and the seqlock snapshot written once per rate tick. Separating the snapshot's share needs
a bench-statistics build of the `m4` parent of the telemetry merge: UNKNOWN, not built.

## Not established

- `u = 0.5` of `telemetry_byte_rate_max` stays the budget's PLACEHOLDER. This run shows the link carrying 110.5 kB/s (37 % of the
  300000 B/s reference throughput) with 0 sequence gaps; it does not replace the placeholder, and no budget value was changed.
- The board has no per-stream drop counter in the console; frame loss on the board side is judged only by the host's sequence
  numbers (0 gaps) and the received counts above.
- Armed timing under telemetry load: not measured (the board was never armed; no receiver bound).

## Files

- `bench_telemetry.py`: the capture and the analysis (unchanged by this run).
- `raw/telemetry_load.console.log`, `raw/telemetry_load.phases.json`: the capture (console with host arrival times; the phase
  windows and the load statistics). `raw/telemetry_load.analysis.txt`: the analysis (note: its profile path is the scratch
  worktree's).

## Flight image left on the board

After the EMB-3 run and the ground log of `../gcs/gx5/board/`, the flight image (`907fd4b7...`) was flashed (`** Verified OK **`)
and the banner read:

```
marv_fc3v2 quad-L2-pass-532-g4c5470c
image: sha256 17bb03aa722ebdc49ce895465a57d1c2e995754914bb34b606d56300ca5acfae (204964 bytes, SHA-256 accelerator, 15109 us)
config: source factory, status ok, boot request none
bench stats: off
```

with `est_valid=1`, `arming=no_link`, spacing [133,134] us, no overrun. The flash order was: bench-statistics image, capture, flight image,
ground log (the flight image ran for the ground log).
