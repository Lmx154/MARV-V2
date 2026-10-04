R1X's α excess in gz is the gyroscopic coupling ω×Jω, under the stage (c) gains and under the old ones (decision 0014,
owner decisions third round, item 3; Luis, 2026-10-02: "Evidence first, as with R2: commit the counterfactual showing that
Gazebo's excess is the coupling. That means the design model plus ω×Jω reproduces Gazebo, as for R2's 7.96 → 1.46e-2, and
that it is identical under the old and new gains.").

R1X (`scenarios/quad/L05/recover_inverted_exact.yaml`) starts at exactly [0, 1, 0, 0], at rest, rotors at hover. Its side
check (`tests/regression/quad/L05/gz/test_t4_recovery.py`) compares gz's tilt angle α with the design model's α envelope
from the exact start, by E + F + Q. At exactly 180° the tilt axis is chosen by rounding noise: gz's first plant step leaves
the body at q_gz(1), which picks the branch of the tilt-yaw split. On that branch the body rates are not about one axis, so
ω×Jω is not zero. On the design model's exact branch it is zero, because that branch is a pure roll.

Files:
- `capture_gz.py`: flies R1X in gz in a given tree with that tree's own test code, and writes the compact gz summary.
  Not run by the pytest.
- `gz_new.txt`, `gz_old.txt`: its raw output (inputs of the analysis).
  - new: this repository at the stage (c) gains.
  - old: a checkout of 69c62f2 with its own host-gz-l5 build and T3 reference.
- `r1x_model.py`: the model runs on gz's branch and their reductions, shared by both scripts. Its docstring holds the model.
- `r1x_coupling.py`: the analysis. Its docstring holds the method and the claims. It does not run gz.
- `coupling.txt`: the analysis's raw output.
- The pytest is `tests/regression/quad/L06/tools/test_r1x_coupling.py`, in the path of the CI tools step. It re-runs the
  analysis and compares the output with `coupling.txt` byte for byte. It also asserts the claims, each with a control.

Commands (repository root, after `uv sync --frozen`):
- The analysis, which needs the generated L5 T3 reference (`uv run python tools/refdata/refdata.py ensure quad/L05/t3`,
  decision 0011):
  `uv run python tests/regression/quad/L06/results/r1x_coupling/r1x_coupling.py --out tests/regression/quad/L06/results/r1x_coupling/coupling.txt`
  - About 17 s on 16 cores. In the marv-ci image under `--cpus 4` the pytest takes 60 s, and the output was byte-identical
    to the host's (for the earlier coupling.txt, sha256 `21357056ea2590f28e29f818d2272c49d8e935e1f732df5c1931a7fd1b3b4394`;
    not re-run in the image for the regenerated one).
- The captures, which need gz-sim 8, the tree's host-gz-l5 build and its generated T3 reference:
  - `uv run python tests/regression/quad/L06/results/r1x_coupling/capture_gz.py --tag new --work <dir> --out tests/regression/quad/L06/results/r1x_coupling/gz_new.txt`
  - `uv run python tests/regression/quad/L06/results/r1x_coupling/capture_gz.py --tag old --root <checkout of 69c62f2> --rev 69c62f2 --work <dir> --out tests/regression/quad/L06/results/r1x_coupling/gz_old.txt`
  - Each takes about 25 s (the design 13 s, the two gz runs 6 s, the model 3 s). A second capture of each was
    byte-identical on the host.
  - With `--rev`, the script first checks the checkout against `git ls-tree -r 69c62f2`, blob for blob (522 files).
  - The gz logs go to `--work`; a rerun regenerates them (decision 0003).
  - `gz_old.txt` was captured before the R1X predicate change in `test_t4_recovery.py`; its recorded `test_t4_recovery.py`
    sha256 is the file before that change. `gz_new.txt` was re-captured after it, with the DShot diffuser live (decision
    0017); a second capture was byte-identical.

Why two scripts. The distances need the full gz series: 20293 and 20745 executions, and one run's float32 q and ω alone are
7 × 4 B per execution, about 0.57 MB. That is too large to commit (decision 0011). So:
- The capture computes them, with the full series in memory, and records the sha256 of every series (gz TRUTH, gz channels,
  each model run).
- The analysis re-runs the new gains' design and model runs from the recorded fixture and gz's q(1). It refuses unless the
  rendered design and model sections equal `gz_new.txt`'s line for line, so the captured distances were computed against
  exactly these series.
- The old gains' model needs the 69c62f2 tree's code, so `gz_old.txt`'s model section is quoted. It was captured by the
  same `r1x_model.py`, whose sha256 the analysis checks.

Inputs. The scripts read all of them; none is retyped. The summaries record the sha256 of each, and `coupling.txt` records
those the analysis reads:
- The card and the R1X scenario.
- The tree's `test_t4_recovery.py` (make_design, fly, evaluate, and the clean-run check), `recovery_model.py`,
  `attitude_t3_oracle.py`, the recorded fixture `attitude_t3_inputs.txt` (the gains), `attitude_t3_q_inputs.txt`, the
  generated `attitude_t3_envelope.txt`, and `run_l5.py`.
- The capture also reads the build's parameter table, which the test's make_design checks against the fixture.

Method (`r1x_model.py`). Every run is the nominal member of the test's design model (`recovery_model.member_run`,
unchanged), with up to two swaps for that run only:
- **gz's branch.** The first kinematic update is pre-multiplied by conj(q0) ⊗ q_gz(1), so the model's attitude at
  execution 1 is gz's. Nothing else of gz enters.
- **Plant C** (`r2_envelope_a`'s). J ω' = m − ω×Jω, τ m' = u + f − m from m(0) = 0, RK4 with 4 steps per tick. R1X starts at
  rest at the hover trim, so the motor deviation and ω×Jω are both 0 at t = 0.

The runs:
- D: the design model.
- C: plant C.
- CF: plant C plus the ideal lag-compensated feed-forward f = c + τċ.
- I: RK4 on the design plant (R = max |I − D|).
- C/2 and CF/2: half the RK4 steps (K).
- C0: plant C from the exact start, the control.

Findings (`coupling.txt`). Distances are max over n = 0 .. N − 1 of |gz − run|, in rad or rad/s.

1. **New gains (stage (c)): the design model plus ω×Jω reproduces gz and its excess.**
   - gz fails the test: 136 violations, first at n 2218, worst at n 2338 with slack +1.17e-2; max(α − hi) is 3.59e-2.
   - D on gz's branch stays inside the envelope (0 violations at F + Q).
   - C on the same branch fails: 143 violations, first at n 2213, worst at n 2338 with slack +1.28e-2; max(α − hi) is 3.58e-2.

   | Channel | D (no ω×Jω) | C (+ ω×Jω) |
   |---|---|---|
   | α | 9.36e-2 | 9.42e-4 |
   | ω_x | 0.181 | 6.02e-3 |
   | ω_y | 0.551 | 3.93e-3 |
   | ω_z | 0.172 | 3.07e-3 |
   | err_z | 6.73e-2 | 7.21e-4 |

   The largest distance falls from 0.551 (ω_y) to 6.02e-3 (ω_x). C's α distance is within the test's F + Q (2.30e-2); D's is
   not.
2. **Old gains (69c62f2): the same reproduction, inside a wider envelope.**
   - gz passes: 0 violations, worst slack −4.77e-2.
   - D and C on gz's branch both stay inside.

   | Channel | D (no ω×Jω) | C (+ ω×Jω) |
   |---|---|---|
   | α | 9.97e-2 | 2.70e-3 |
   | ω_x | 0.380 | 1.18e-2 |
   | ω_y | 0.886 | 3.50e-2 |
   | ω_z | 0.188 | 2.45e-2 |
   | err_z | 8.55e-2 | 6.90e-3 |

   The largest distance falls from 0.886 to 3.50e-2, both on ω_y.
3. **Old against new.**
   - q_gz(1) = [4.33e-17, 1, 7.85e-17, 4.33e-17] and ω_gz(1) are bit-identical under both gain sets, in both runs. The seed
     execution's command is 0 whatever the gains, so the branch is the same.
   - ω_y/ω_x at n 300 is −0.4449 (old) and −0.4453 (new), a change of +0.07 %.
   - The coupling integral Σ|ω×Jω| T_a on gz's rates changes by −12.0 % (roll), +5.9 % (pitch) and −6.7 % (yaw).
   - The coupling's α effect, max |α_C − α_D|, is 0.1014 old and 0.0933 new (−8.0 %). The lead's diagnosis had 0.101 and
     0.099; its new figure was at the earlier product k 3.228.
   - What changed is the envelope. F + Q went from 4.81e-2 to 2.30e-2 (Q 6.74e-3 before the DShot diffuser of decision 0017,
     2.67e-6 with it), and max(α_C − hi) went from 0 (C never above hi) to 3.58e-2.
   - No threshold is set on these changes, because none is sourced. They are reported, not judged.
4. **The ideal feed-forward removes it.** Under both gain sets, CF equals D within K + R on every channel (the largest
   |CF − D| is 3.6e-11, on α) and stays inside the envelope. Control: C is beyond K + R from D on every channel.
5. **Control of the distance claim.** C0, the coupled model from the exact start, is a pure roll, so ω×Jω = 0 and it stays
   inside the envelope. Against gz it is not closer than D on any channel (for example ω_x 12.7 rad/s new, 14.0 old). The
   branch seed is what makes C match gz.

Assumptions and limits:
- The plant is the design model's first-order torque lag with ω×Jω (`r2_envelope_a`'s plant C). `marv_plant`'s rotor-speed
  lag with thrust ∝ W² and the DShot rounding are not modelled. That is the residual C − gz (largest 6.0e-3 new, on ω_x; 3.5e-2 old, on ω_y). The DShot diffuser is live in the new capture.
- Only the nominal member is run on gz's branch. The envelope's other members run only from the exact start, as the test
  computes them.
- The feed-forward of claim 4 is ideal (continuous, true ω, the plant's J and τ). The firmware's is evaluated by c5
  (`../ff_eval/`). Stage (e) is where R1X must pass with the firmware's feed-forward live (decision 0014, third round,
  item 3).
- A new gain set or scenario changes the design and the gz runs, so re-capture both summaries.
