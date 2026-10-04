R2's E, measured on R2's own two runs (decision 0014, owner decisions second round, item 2: E first, before R2's setup
change). It answers the question `bound.txt` leaves open: can the run term E close the gap between the forced excess X* and
the design tolerance F + Q?

E is the R2 predicate's run term (`tests/regression/quad/L05/gz/test_t4_recovery.py`): E_c(n) = |y_c(m = 1, n) − y_c(m = 2, n)|,
over the two runs of the scenario's `m_sequence` [2, 1]. The runs are the test's own. The script calls the test's `fly` on
the test's `make_design`, applies the test's clean-run check to the runs, and evaluates E with `evaluate`'s expression. It
refuses unless its per-channel maximum equals `evaluate`'s `max_E`. Setup: R2 with hover rotors, the scenario the proof reads
(`scenarios/quad/L05/recover_tumble_prop_strike.yaml`, decision 0015), and the final stage (c) gains of the host-gz-l5 build (att_kp
3.1539721). The composition flies the rate loop through `execute_bypass`, which skips only the reference prefilter: the stage (c)
D term and the gyro chain's low-pass are in these runs; the feed-forward is not (it is inert until stage (e), decision
0014).

Files:
- `measure_e.py`: the script. Its docstring holds the method.
- `e_measured.txt`: its raw output.
- The pytest is `tests/regression/quad/L06/tools/test_r2_e_measured.py`, in the path of the CI tools step. It does not re-run
  gz. It parses `e_measured.txt` and checks three things, each with a control:
  - internal consistency: every printed E is |y1 − y2| exactly, and the summary and stop lines quote the rows;
  - the proof values: they are `bound.txt`'s as printed;
  - the comparison itself.

Command (repository root, after `uv sync --frozen`, gz-sim 8, `cmake --preset host-gz-l5 && cmake --build --preset
host-gz-l5` and `uv run python tools/refdata/refdata.py ensure quad/L05/t3`):
`uv run python tests/regression/quad/L06/results/r2_lower_bound/measure_e.py --work <scratch dir> --out tests/regression/quad/L06/results/r2_lower_bound/e_measured.txt`

The gz logs go to `--work`. A rerun regenerates them (decision 0003). The run takes about 23 s on 16 cores (the design 15 s,
the two gz runs 6 s) and about 57 s in the marv-ci-gz image under the runner's 4 CPUs (the design 49 s).

Inputs. The script reads all of them; none is retyped. `e_measured.txt` records their sha256 and the gz, gz-physics, DART
and sdformat versions.
- The card, the scenario, and `test_t4_recovery.py` with `recovery_model.py`: the runs, the TRUTH series, the design, F and Q.
- `run_l5.py`: the runner the test calls.
- The recorded T3 fixture and the generated T3 envelope file (decision 0011).
- The build's parameter table.
- `r2_lower_bound.py` and `bound.txt`. The script calls the proof's `compute()`, which reuses the same design object, for
  n*, X* and F + Q at full precision.

Findings (`e_measured.txt`):

1. **E does not close the gap. The stop condition is not met.** The predicate can hold at the proof's binding execution n*
   only if E(n*) ≥ X* − (F + Q).
   - **w_y, the binding axis:** E(17) = 6.7043e-4 rad/s, against 0.25424 needed (1/379 of it).
   - **w_x:** E(12) = 2.6798e-4 rad/s, against 0.18308 needed (1/683).
   - **w_z:** E(22) = 2.0981e-5 rad/s, against 0.031588 needed (1/1506). w_z is forced at these gains (X* 0.039504 > F + Q
     0.0079155).
2. **Where the forced excess alone exceeds the tolerance.** X(n) − (F + Q) > E(n) at n = 1–32 on w_y (all 32 of the proof's
   32 executions), at n = 1–23 on w_x (23) and at n = 3–32 on w_z (30). The test's own `evaluate` on the same runs first
   fails at n = 1 (w_x), n = 1 (w_y) and n = 3 (w_z).
3. **E over the whole predicate window** (n = 1 .. 20292). The largest E is 0.012977 (w_x, n 281), 0.0065413 (w_y, n 152)
   and 0.0022211 (w_z, n 348). Each is below its axis's needed value: by 14.1 times (w_x), 38.9 times (w_y) and 14.2 times
   (w_z). Over the proof's window (n = 1 .. 32) the largest E is 3.0518e-4 (w_x, n 19), 1.4935e-3 (w_y, n 32) and
   3.4332e-5 (w_z, n 32).
4. **Determinism.** Two host runs of the final-gain output, with decision 0016's plugin first-read fix, are byte-identical, sha256
   `fd0c16f81180767d264b1b8896696357745b35efdf96cf51963f9d87291c6586`. The marv-ci-gz image was not re-run for this output.
   For the earlier output (other gains) two image runs were byte-identical to each other and to the host's.

Not in the output: wall times, and the plugin binary's sha256. The script prints that sha256 on stderr only, because the
binary differed between the two builds of the same source, in the earlier measurement: `57e3d70d…` on the host, `8cca1fb3…` in the image. The cause is
INFERRED (the Debug build's embedded paths or the toolchain). The generated parameter table, which is in the output, is
identical in both builds.

Limits:
- E belongs to the run pair, and so to the controller that flew it. This measurement is for today's gains and hover rotors.
  A new gain set, or R2's steady-tumble setup, needs a re-run, as `README.md` says for X* and F + Q. On a scenario whose
  rotors are not `hover`, the proof's `compute()` refuses, so this script stops before the comparison.
- The runs are truth-fed and perfect-model (core section 6), not a validation run.
