Cause of the L5 T4 angle-mode step excursions (decision 0006 F "T4 angle-mode steps"). Raw output of `step_cause.py`
(the script, in this directory). It flies `scenarios/quad/L05/step_roll.yaml` and `step_pitch.yaml` at m = 2 and m = 1 as
`tests/regression/quad/L05/gz/test_t4_steps.py` does and applies that test's own predicate. It then replays the m = 1 run
through the composition path with `step_cause_tool` (refusing to report unless the replay reproduces every DShot command
bit for bit). Finally it runs a counterfactual: the same float firmware path closed around a binary64 rotational plant in
four variants, from the T3 design plant to the design plant with the DShot quantisation and marv_plant's DShot -> thrust ->
torque map inserted. The file header lists the inputs (card, the build's parameter table, the envelope, the tool source),
each with its SHA-256.

Files:
- `step_cause.py`: the script.
- `step_cause_tool.cpp`: the replay and counterfactual tool (harness only, not firmware). It mirrors
  `l5_attitude_scripted.cpp` `tick()` on the host-gz-l5 build's firmware libraries.
- `build_tool.sh`: builds the tool against those libraries with that build's flags. No CMake target is added.
- `cause.txt`: the raw output.

Command (from the repository root, after `uv sync --frozen`, with gz-sim 8 and
`cmake --preset host-gz-l5 && cmake --build --preset host-gz-l5`):
`uv run python tests/regression/quad/L05/results/step_cause/step_cause.py --work <scratch dir> --out tests/regression/quad/L05/results/step_cause/cause.txt`

The gz logs, the replay rows and the simulated traces go to `--work`. They are not committed: a rerun regenerates them
(the runs are deterministic, decision 0003). The script takes about 30 s.

Verdict (measured, `cause.txt`): DShot quantisation at hover.
- **Replay.** The replay is bit-exact at all 44689 rate executions of each run. In the settled tails, about 97 % of rate
  executions command (765, 765, 765, 765). Their torque request reaches 8.03e-4 N m (roll) and 6.02e-4 N m (pitch),
  equal to the derived request dead band.
- **Counterfactual.** With the quantisation inserted into the design model, the step reproduces the gz trace within
  4.9e-4 rad (roll) and 3.0e-4 rad (pitch) over the whole window, which is below F. The worst excess is the same
  (+6.52e-3 and +5.94e-3 rad), and so are the limit-cycle amplitude and the end offsets.
- **Controls.** The unquantised variants (design, cont) pass with 0 violations. The design variant equals the T3 golden
  within 2e-7 rad.
- **Ruled out.** Gyroscopic coupling is zero: the run is single-axis, and |w x Jw| < 1e-16 N m in gz. Translation, drag
  and gravity with height are bounded by the residual 4.9e-4 rad, since the counterfactual has none of them.
