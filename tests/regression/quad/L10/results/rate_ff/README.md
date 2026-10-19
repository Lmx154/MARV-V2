# Decision 0058: the acro roll step and the roll flick, before and after the reference feed-forward (gz SIL)

This is a simulation on INFERRED / PLACEHOLDER card values (`vehicles/marv_7in_fc3v2.yaml`), not a flight result. It is evidence,
not a test, and nothing was tuned. The runs were made at commit 94178a1 with a clean tree (`report.txt` line 4).

```sh
cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc -j4 && uv sync --frozen
uv run python tests/regression/quad/L10/results/rate_ff/rate_ff_sil.py --out-dir <dir>     # before/ and after/
```

Each run directory holds `world.sdf`, `report.txt`, `analysis.txt`, `traces.csv.xz` (the two event windows, about 940 Hz),
`scenario/hover.yaml` and `SHA256SUMS`. The logs are not committed; their sha256 values are in `analysis.txt`. Each run took
about 5 s of wall time (RTF 3).

## The runs

The world is `tools/sim/run_quad_fc.py`'s, with the same card, tick, site, model gyro, launch pad, 10-tick command delay and
125 Hz frames. Only the stick script and the analysis differ (`rate_ff_sil.py` docstring). The script hovers in angle mode,
then in acro flies:
- a roll step of 100 deg/s for 0.6 s;
- after re-levelling in angle mode, a roll flick of 200 deg/s for 175 ms.

These are the architect's two inputs (Serena memory `quad-m4/architect-rate-response-2026-10-18`). The stick values come
from the rates curve. The commanded rate is computed from the stick the firmware received. The response is the truth body
roll rate.

- `after`: the image as built: `marv_params_fc3v2` with `rate_ref_ff_enable` 1 and tau_ref roll/pitch 0.065964 s (rule step 14).
- `before`: the same image with `<sil_override>` setting `rate_ref_ff_enable` 0 and `rate_tau_ref_{roll,pitch}` 0.21945825 s.
  That tau_ref is decision 0005's rule on this card (`rate_lead.tau_ref_rule(design, False)`). With the feed-forward off the law
  is bit-identical (decision 0058 I2), so this is the loop before the change. The two worlds differ only in these three
  overrides and the log path.

## Result (roll; the architect's linear model, nominal design delays, in brackets)

| | before (prefilter only) | after (model following) |
| --- | --- | --- |
| step 100 deg/s: t63 | 387.3 ms [371] | 117.6 ms [116] |
| step: overshoot | none within the 0.6 s window (0.956 of the command at its end) [15 %, later] | 6.1 % [7.7 %] |
| flick 200 deg/s, 175 ms: reach | 37.7 % [39 %] | 81.3 % [81 %] |
| flick: peak after release | 172.8 ms [164] | 7.1 ms [6] |
| flick: to 10 % of peak, from the peak | 641.7 ms [610] | 354.3 ms [348] |
| flick: to 10 % of peak, from release | 814.5 ms | 361.3 ms |
| flick: undershoot after the stop | -1.1 % of command | -1.8 % of command |

The flick stick was held for 176.0 ms; the stick arrives in 8 ms frames. "To 10 %" follows the architect's `compare.py`, which
measures from the peak; the time from release is listed beside it. The SIL world adds a 10-tick command delay and the model
gyro, which the linear model does not have. The architect's numbers fall within about 6 % of these (t63 within 2 ms after the
change).
