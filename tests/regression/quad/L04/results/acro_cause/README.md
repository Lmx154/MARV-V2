Cause of the L4 acro pitch excursion and of the roll recovery failure (decision 0005; Luis's ruling of 2026-09-30). Raw
output of `acro_cause.py` (the script, in this directory). It flies `scenarios/quad/L04/acro.yaml` at m = 2 and m = 1 as
`tests/regression/quad/L04/gz/test_t4_acro.py` does, replays both runs with `tests/regression/quad/L04/replay/l4_acro_replay`
(refusing to report unless the replay reproduces both runs bit for bit), and uses the test's own rate bound. The file
header lists the inputs: the scenario, the card and the build's parameter table, each with its SHA-256.

Command (from the repository root, after `uv sync --frozen`, with gz-sim 8 and
`cmake --preset host-gz-l4 && cmake --build --preset host-gz-l4`):
`uv run python tests/regression/quad/L04/results/acro_cause/acro_cause.py --out tests/regression/quad/L04/results/acro_cause/cause.txt`

File: `cause.txt`. Verdict: gyroscopic coupling, not saturation. No saturation flag is set anywhere in the run and
s = t = 1 at every execution. The per-axis PI baseline cannot reject the coupling, and the roll integrator absorbs it and
then releases it, which gives an uncommanded roll with the stick centred. The strict recovery check of
`test_t4_acro.py` is therefore a known failing item (strict xfail until `tests/regression/quad/L06` exists). The gz logs
are not committed: a rerun regenerates them (the runs are deterministic, decision 0003).
