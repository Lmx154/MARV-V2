# SIM-7 seed U^0: the L4 rate-loop chirp's U, re-measured

A measurement (core §2 rule 3) for decision 0006 section E, "U and the circularity", step 1. It is outside
`tests/regression`, so L6 can re-measure it after a rate-loop change without a decision record (owner decision 7).

## What it is

U = min over axes of (E_H + U_A + U_d), in rad (`u.yaml`, key `U`). Per axis, from the frozen L4 chirp harness:

- E_H = |PM(m = 1) - PM(m = 2)|, U_A = |PM(A) - PM(A/2)|;
- U_d = `run_l4.float_input_term` of the (m = 1, A) run.

`measure_u.py` imports the harness (`tools/sim/run_l4.py`) and the frozen test module
`tests/regression/quad/L04/gz/test_t4_chirp.py` (its `RUNS`, `AXES`, `HALF`, `CARD`, `SCEN`, `PLUGIN_DIR`) and calls
`run_l4.run_chirp`, `run_l4.chirp_margin` and `run_l4.float_input_term` as the test does. Nothing under
`tests/regression/`, `tools/sim/run_l4.py` or `scenarios/quad/L04/` is modified. The test's fourth run (half duration)
checks the duration's convergence only and does not enter U; it is not run.

## Files

| File | Content |
| --- | --- |
| `measure_u.py` | `measure`: the gz runs, writes `raw.json` and `inputs.json`. `post`: `raw.json` to `u.yaml`, no gz. |
| `run_in_container.sh` | `uv sync --frozen`, the `host-gz-l4` build, then `measure` and `post`. |
| `raw.json` | Per axis: PM (rad) and crossover for m = 1, m = 2 and A/2; the U_d terms; the design PMs and `PM_min`. |
| `inputs.json` | Git commit, hashes of the script, the test module, `run_l4.py`, the card, the plugin and each scenario. |
| `u.yaml` | U in rad (and degrees), method `measured`, the source, the rule text, and E_H, U_A, U_d, U per axis. |

## Command

From a clean copy of the repository (no `build/`, `.venv`, `__pycache__`, `.claude`), with `marv-ci` then `marv-ci-gz`
built (`docs/handoff.md`, "How to verify"). `$COMMIT` is the full hash of the commit the copy was made from: a
worktree's `.git` file does not resolve in the container, so the script reads it from `MARV_COMMIT`.

```
docker build -t marv-ci -f ci/Dockerfile . && docker build -t marv-ci-gz -f ci/Dockerfile.gz .
rsync -a --exclude build --exclude .venv --exclude __pycache__ --exclude .claude ./ "$COPY"/
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -e MARV_COMMIT="$COMMIT" -v "$COPY":/src -w /src marv-ci-gz \
  design/measured/sim7_u/run_in_container.sh
```

Then copy `raw.json`, `inputs.json` and `u.yaml` from `$COPY/design/measured/sim7_u/` back here.

Post-processing alone (no gz), which must reproduce `u.yaml` byte for byte from `raw.json`:

```
uv run python design/measured/sim7_u/measure_u.py post
```

`--out-dir DIR` reads `raw.json` from and writes `u.yaml` to `DIR`.
