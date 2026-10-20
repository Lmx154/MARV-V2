GX-11 (replay of a ground log, GS-8) evidence. Base `6aabd62` (origin/m4).

- `record_sil_session.py`, `record.json`: the recording of the fixture of `tests/tools/test_gs_replay.py`, from a real Gazebo SIL run
  (`ci/heavy.sh -- uv run python tests/regression/quad/L10/results/gcs/gx11/record_sil_session.py <out_dir>`; the helper's SIL launcher, the
  host-gz-quadfc build of this commit). The helper's ground log (`ground.log`, 477 received chunks, 23595 B, 19 sent chunks) and the frames the live
  client decoded from the bytes it received over the WebSocket (`live.decoded.jsonl`, 477 frames) are committed, compressed, as the test's fixture
  `tests/tools/data/gx11/sil_session/` (`ground.log`, `live.decoded.jsonl.xz`). The script checks that the WebSocket bytes equal the log's
  received stream before it writes them. The session token in `record.json` is redacted by hand for this run; the script redacts it now.
- `light/`, `live/`: the Playwright run of `gcs/e2e/vehicle.spec.ts`, last test (a replay of the session just recorded shows the values the
  live session showed): `tools/gs/e2e.sh -- vehicle.spec.ts` (light, fake vehicle: 2 passed, 1 skipped as in light mode) and
  `tools/gs/e2e.sh --live -- vehicle.spec.ts -g replay` (gz SIL: 1 passed). `report.json`, and the Vehicle page after the replay
  (`vehicle-6-replayed-vehicle.png`) beside the Logs page's replay status (`vehicle-5-replay-finished.png`).
- Control of the e2e (not committed output): the helper's replay mutated to drop the last byte of every seventh chunk fails the row comparison
  (`expect.poll(...).toEqual(recorded)`), 1 failed; restored.
