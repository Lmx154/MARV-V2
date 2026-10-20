GX-15 (the Logs page) evidence.

- light/: `ci/heavy.sh --wait 10800 -- tools/gs/e2e.sh --out-dir tests/regression/quad/L10/results/gcs/gx15/light` (light mode). Spec
  gcs/e2e/logs.spec.ts on the sample tests/tools/data/u1_sample.mlog (logger unit U1's encoder, tests/tools/test_mlog.py good_log(600)) put on a
  fake mounted card; the expected gyro_x min and max are the decoder's own (tools/log/mlog.py), computed by tools/gs/e2e.sh. report.json is the
  whole light run: 18 passed, 2 skipped (the live-only specs). logs-1-imported.png: the card file imported, the SHA-256 and the decoder report;
  logs-2-plot.png: gyro_x plotted (min 500, max 40333.5, 600 of 600 buckets); logs-3-narrow.png: the control, a narrower range; logs-4-refused.png: a
  path inside the repository refused with the helper's reason (W-1).
