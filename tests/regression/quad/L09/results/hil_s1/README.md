# HIL S1: bidirectional USB link proof (decision 0046), 2026-10-17

Image: the bench-only `MARV_FC3V2_HIL_LINK` variant, `quad-L2-pass-241-gf4905df`
(sha256 `38136ba04a0bbc3df3882c29c94bb3c07a7a9d1d2a3970b898c8e91fe9962a0f`), on the FC3v2 over USB; no composition, no
IMU, no receiver, no DShot program loaded (no motor pad driven); battery unplugged; Luis not present (no motor can run).

Command (host `tools/target/hil_link.py`, 60 s each at 7500 samples/s):

    python3 tools/target/hil_link.py run /dev/ttyACM0 60 --rate 7500 --batch N s1_bNN.bin

Results (each run's own summary, `s1_b*.summary.txt`): LINK OK for N = 1, 4, 8, 16; 450 000 samples each way, 0 CRC
failures, 0 sequence gaps, 0 bytes dropped on either side.

| N per frame | host→board | board→host | round trip p99 / max (µs) | board arrival gap max (µs) |
|---|---|---|---|---|
| 1 | 390 kB/s | 300 kB/s | 405 / 3322 | 732 |
| 4 | 300 kB/s | 300 kB/s | 814 / 1260 | 1463 |
| 8 | 285 kB/s | 300 kB/s | 1505 / 1701 | 1305 |
| 16 | 278 kB/s | 300 kB/s | 2311 / 2430 | 2344 |

Raw output kept for N = 4 only (the configuration the design uses; the others' raw files total ~14 MB compressed and
stayed in the session scratchpad): `s1_b04.bin.xz` (uplink bytes as received), `s1_b04.sends.csv.xz` (host send times),
`s1_b04.chunks.csv.xz` (host read times); `SHA256SUMS` holds the uncompressed hashes. Decode: `xz -dk` then
`python3 tools/target/hil_link.py` (see its `--help`).

For decision 0046 item 2: at N = 4 the round trip's maximum is 1.26 ms = 9.5 ticks of 133.3 µs, so the command delay
is N_delay ≥ 10 ticks; N_max (the rate loop's margin at latency 3 + N_delay) is computed in S2.
