# 0019: quad L6 stage (e), the noise Monte Carlo, the corner search and the gate

Stage (e) of quad spec §4 "L6": the L4 and L5 scenarios rerun with noise and filters, the T3 Monte Carlo over noise
seeds, and the gate file that turns the three known failing items into normal tests with the feed-forward live
(decisions 0009 and 0014). This record opens the stage. It holds Luis's rulings of 2026-10-04 and the stage's commits as
they land. The infrastructure lands first, in its own commits, while R2's path is being decided. The gate commit comes
last, once every item passes.

## Owner decisions (Luis, 2026-10-04, verbatim)

On the (e) decision round (the architect's proposal; the lead's report of 2026-10-04):

"**3. Noise-term rule: NT-2. F1 is a real contradiction in the ACTIVE spec, and this is the fix.**
- At quantile p, a build that is exactly correct passes all N seeds with probability p^N ≈ 1 − c. So the test would reject a correct build 90–95 % of the time.
- With the noise term at the per-seed quantile 1 − (1 − k)/N (Bonferroni over the N seeds), a correct build passes every seed with probability at least k. That is the same family-wise convention as the Allan, independence and moments checks.
- **Guard.** The decision-24 fine controls must still fail with the noise term in place. If one stops failing, stop and report; don't adjust anything.

**4. The register entry: yes, 0.99.** Use a new entry (one entry, one purpose), for example `noise_term_confidence`, claimed in L06/param_ids, with the same rationale as the other family-wise entries.

**5. Predicates judge the gz truth state: yes.**
- The reason is what R2 and acro are for: they ask whether the vehicle's rotation recovers.
- Judging the received sample would test the sensor's noise against an envelope, not the vehicle. The noise still reaches the truth trajectory through the loop, and the propagated noise term covers that.
- That acro fails on the received sample at T4c is not the reason for this choice.
- Truth is read by the test harness only; G3 still holds for flight builds.

**6. Bias corners from the nonlinear design model: yes, as a delta only.**
- The bias term is (nonlinear model with the bias) − (nonlinear model without it), added to the linear envelope.
- The coupling itself never enters the envelope. Otherwise acro and R2 would stop asking whether the controller rejects it.

**7. Per push: PP-A.** It is the spec's split: every scenario at its committed seed per push, the confirmation seeds nightly. Run the confirmation seeds also before every stage close and every tag. Re-measure the gz times after the plugin's IMU, rotor and clock wiring (F4) before you fix the shard count.

**8. Keep 22 and 59: yes.**

**9. Clock corners inside the T3 corner search: yes.** This is my stage (a) and (b) ruling: T3 searches the corners, and T4 flies the nominal plus each scenario's worst corner.

**10. Question E, the Gazebo step rounding: outward, not nearest.**
- ±65 ppm is a bound. Nearest rounding (±64.004) never reaches it. Outward (±70.4 at m = 1, ±67.2 at m = 2) covers it, at negligible cost (≤ 0.16 mrad/s at 65 ppm).
- One condition: measure E at the nominal clock, which is exact at both m. That way the m-dependent corner value never enters E.
- Record both realised values in 0012's question E entry. If outward breaks anything the 0.16 mrad/s figure doesn't explain, stop and report.

**11. Structural ties fly one canonical corner: yes, for exact ties only.**
- Each tie needs a per-push test proving it bit-identical. For example, two corners differing only in accel bias give identical logs in L4 and L5.
- At L7 the estimator reads the accelerometer, so that test will fail and the ties will break by themselves.
- The chirp tie under a constant bias collapses only if it is shown to be exact. If it is approximate, fly it.

**12. Close S9 for pytest in the (e) CI change: yes,** with the planted-overrun control.

**The gate commit:** agreed as you wrote it. FF on, the gate file and the per-push FF-iff-gate check go in one atomic commit. If any item still fails, there's no gate and no per-item gates; FF stays off and the record comes to me."

Also from that round: "Worst T3 corner = smallest predicate slack per scenario: yes, normalised. Per scenario, take the corner
with the smallest slack divided by that channel's own bound, minimised over channels so units compare. Keep both corners
on a tie. It is computed on the T3 design model, never picked from Gazebo results." (0017, Approval.)

**The spec line (F1), approved:** "Spec line: approved as you wrote it. ... Approved: Luis, 2026-10-04." It is applied in
the same change as the `noise_term_confidence` register entry:

> Envelopes carry a noise term, derived by propagating the noise model through the design model at the per-seed
> quantile 1 − (1 − `noise_term_confidence`)/N, so that a correct build passes every seed with probability at least
> `noise_term_confidence`. The fine negative controls of 0006 decision 24 still fail with the noise term in place.

On c1 (the anti-windup law of the R2 round), 2026-10-04: "**Hold 0018. Don't commit it to master yet.** ... Park it on a
local branch, `l6-0018-c1`. Commit the tree there, don't push, and return master's tree to 4bfd66c. ... The (e)
infrastructure commits then proceed on master as ruled." And: "**Now, independent of c1: pin
`r2_ff_diagnosis/regenerate.sh` to the commit it records.** It goes in its own small commit, or with the first (e)
infrastructure commit. It is a reproducibility fix, so the old-law outputs stay reproducible whatever the law becomes."
When an anti-windup change does land, the L5 rate-bypass golden follows ruling (a): its generator is re-pinned to the
commit that introduces the law, and the test is renamed to say which law it reproduces. The quad-L4-pass golden is kept
as its negative control and must fail on the saturating rows. `l6_ff_eval.py` follows the law in the same commit, and
its committed sweeps are confirmed byte-identical.

## Owner decisions, second round (Luis, 2026-10-04, verbatim)

On the noise suite design memo and the S9 guard:

"**1. Run statistic: Bonferroni inside the run.** The predicate is "every point inside the envelope", so the per-seed event is the worst point of the run, and Bonferroni holds under any dependence between points. Two conditions:
- The point count comes from the predicate's own definition (the points it actually checks), never chosen.
- The decision-24 guard in C7 is what keeps this honest. If the wider W lets a fine control pass, stop and report rather than adjust.

**2. Time-varying scenarios: frozen extremes for the per-push W, verified by a committed Monte Carlo check.** Not on INFERRED alone: an unproven claim can't set a pass threshold.
- The check, run nightly, gives each time-varying scenario's Monte Carlo σ̂ with a χ² upper confidence bound. That bound must be ≤ the frozen-extremes σ. Add it with a planted control: an understated σ must fail.
- The confidence is a new register entry (one entry, one purpose), for example `noise_sigma_bound_confidence` = 0.99, claimed in L06/param_ids, with the family-wise rationale.
- If any scenario fails the check, that scenario's W uses the Monte Carlo bound instead, and you tell me.

**3. Structure: yes.** Keep the frozen T4 tests as the T4c truth-gyro guard, and put the noise suite on regenerated T4e envelopes in new files. When the gate commit lands, the known items must pass in both: the frozen T4c tests (where the strict xfails live) and the T4e noise suite.

**4. S9 pytest guard: (a).** Enforce it in `ci/local_ci.sh`; on Actions, report per-file times without failing. S9 was always measured in the local CI image, and local CI is the definition of verified.
- Measure with load < 1.5, as the handoff says.
- `test_r2_lower_bound.py` at 57.8–58.3 s is the file to watch. If it crosses 60 s in the local image, it moves to nightly under the existing rule.
- Commit the guard now.

**5. The gz re-measure:** I'll tell you when the machine is quiet. Until then, build C4–C6, which are T3 work and not timing-sensitive. Fix the gz shard count only after the re-measure."

The memo's other points were decided by the lead and accepted by Luis ("Your self-decided memo points are accepted"):
- δ is computed at the card plant plus the four τ×J corners;
- the yaw sign items are widened by W;
- the T3 Monte Carlo streams are dumped from the C model;
- the seed rationale is corrected in C7;
- the noise term is named W;
- no fine controls run against the T3 Monte Carlo.

The noise suite design memo (2026-10-04) is saved at `~/marv-e-round-2026-10-04/e_noise/design.md`; its commit plan is C4–C7.

## Why

The spec's (e) pass bar (quad §4 L6): the T3 Monte Carlo over noise seeds, the T4 reruns with noise and filters, and
both known failing items passing. The decision round found that the ACTIVE spec's noise-term line contradicts its
every-seed rule (F1). The T4 corners are now searched at T3 (decisions 0012 and 0013; rulings 9 and 11). The
per-push / nightly split keeps per-push within S9.

## What changed

**1. `r2_ff_diagnosis/regenerate.sh` is pinned to the commit it records** (a reproducibility fix; Luis, 2026-10-04).
- The pinned commit is `4a76755bedb5dfb07a26aa65653c1a2b35b5008e`, decision 0016's commit:
  - it is the only commit that ever touched `r2_ff_diagnosis/`, and the directory is unchanged since;
  - it introduced the `MARV_GZ_TEST_ZERO_FIRST_READ` switch.
- `PIN` is one variable at the top. The script builds and runs everything from a detached `git worktree` of that commit,
  removed at exit:
  - the host-gz-l5 build, the refdata, `capture_gz.py` with the switch, `build_tool.sh`, `build2.sh` and the Python
    scripts;
  - Python runs through the main `.venv` (`uv.lock` and `pyproject.toml` are unchanged since `4a76755`).
- The hash and small-file checks compare against the current tree's record directory. So a run also shows that the
  committed record equals the pinned outputs.
- Result on `4bfd66c` (the diffuser live): 35 hashed files and every committed small file reproduce, with 0 mismatches,
  in 424 s at 2 build jobs (331 s of it the cold pinned build and the refdata).

Frozen files changed by item 1:
- `tests/regression/quad/L06/results/r2_ff_diagnosis/regenerate.sh`: the pin.
- `tests/regression/quad/L06/results/r2_ff_diagnosis/README.md`: the regenerate section says what is pinned and why.
No data file, `SHA256SUMS` or summary changes.

**2. W1, commit C1: the Gazebo plugin's IMU-model, rotor-speed and clock wiring** (F4 of the (e) round; the design memo
of 2026-10-04 with the lead's decisions on its questions).
- **Interface.** All new SDF elements are optional. With none of them present the plugin takes exactly today's code
  path.
  - `<gyro_source>model` feeds the SIL the plant IMU model's bytes (decision 0012), in place of the truth gyro.
  - `<imu_model>` carries the profile's figures as exact reprs, written by `gen_imu_config.imu_model_element` from
    `imu_config()`:
    - latency, noise density, bias instability, LSB and full scale for gyro and accel;
    - the six turn-on signs in {−1, 0, +1}, with the plugin computing bias = sign × bound;
    - the profile SHA-256 (format-checked only: the plugin holds no reference hash).
  - `<rotor_speed_model>`: latency 2 ticks (1 rate period); a 3-bit exponent / 9-bit mantissa grid with 1 µs period
    unit.
  - `<clock_corner>` (−1/0/+1) with `<odr_error>` 6.5e-05. `gen_world` writes the outward-rounded integer-ns host step;
    the plugin re-derives it in exact integer arithmetic and refuses any other step.
- **Data path.**
  - The IMU sample is the plant's own IMU model on the held body, with latency 1 and noise index = tick.
  - The rotor sample is the plant's speed, put on the grid and delayed 2 ticks. `marv_sil_tick_with_rotor_speed` stages
    it as `hal_rotor_speed()`, so the notches track.
  - The clock error enters only the plant's clock, through a new `AdapterConfig::t_tick_true_s` (0 keeps today's
    computation). `fw/` never sees e.
  - `TruthAttitude` gains the 3- and 4-argument forwarding overrides.
- **Logging.**
  - Record type 6, SENSORS (242 bytes): written once, first after the header.
  - Record type 7, ROTOR (the tick plus 20 bytes): after TICK and any TRUTH.
  - Both appear only when a new element is present. The header stays version 1, and truth stays in the harness-only
    STEP and TRUTH records (G3).
- **Lead decisions** (the memo's questions):
  - the sensor numbers live in the world (the card pattern);
  - the realised tick at a corner is the new adapter field;
  - the bias state is not logged (the replay rebuilds it);
  - vibration is off: its amplitude is UNKNOWN, a known gap;
  - a clock corner with the truth gyro is allowed only behind the test-only env var
    `MARV_GZ_TEST_CLOCK_CORNER_ANY_GYRO`. It is used by the L2 T1 clock test, and a per-push scan proves it appears
    nowhere else.
- **Tests**, each with negative controls:
  - (a), per push in core (`L06/tools/test_world_sensors.py`): every L2 world × m × 7 sensor sets differs from the
    default only by the inserted elements (plus `max_step_size` iff c ≠ 0). The emitted values round-trip bitwise.
  - (a), one-time: the sha256 of every gz-suite log is identical before and after (L02 32/32, L04 33/33, L05 47/47).
    The control: a model run and a c = +1 run differ.
  - (c) `L06/gz/test_sensor_seed.py`: seed determinism.
  - (d) `L06/gz/test_sensor_clock.py`: the exact clock at m ∈ {1, 2} × c ∈ {−1, 0, +1}.
  - (f) `L06/gz/test_sensor_refusals.py`: 13 planted malformed worlds refused.
  - They run in the gz-l2 job (`ci/run_ci_gz.sh`, step `gz_sensors`).
- No `fw/` change and no frozen file changed. The realised clock values are recorded in 0012's question E entry.

**3. W1, commit C2: the sensor replay and test b** (the (a) T1 line "the adapter's sensor bytes equal a direct marv_plant
call").
- **`sim/gz/sensor_replay`**, a host tool built in every host build and needing no Gazebo. It reads a model-gyro log
  (header, SENSORS, STEP bodies, TICK, ROTOR) and calls `marv_plant` directly in the adapter's order: IMU sample, rotor
  sample, step.
  - The true tick is derived from the header and SENSORS, not read from the log.
  - Per tick it compares the IMU bytes, the rotor bytes and the plant outputs bitwise.
  - Once per log, it checks SENSORS against `imu_corner_config(signs)`, the turn-on bounds and the ODR error.
  - It also checks the rotor-speed sensor's latency (ticks), exponent and mantissa bits and period unit against the
    profile's values (`gen_world.rotor_speed_values`, passed in by the test), not against the log. Controls: latency + 1
    and mantissa + 1 in SENSORS each report one config mismatch naming the field (the review's finding).
- **Test b** (`tests/regression/quad/L06/gz/test_sensor_replay.py`, 28 tests, about 10 s, in the gz-l2 `gz_sensors`
  step): 4 runs, every one with 0 mismatches over 512 or 1024 ticks, all six turn-on signs nonzero:
  - hover at m = 1, c = 0 and at m = 2, c = +1;
  - rotation at m = 1, c = −1 and at m = 2, c = 0.
- **Controls**, each made by editing a copy of the log:
  - one flipped IMU bit gives exactly one mismatch;
  - the seed + 1 makes the IMU mismatch on every tick;
  - samples one tick late make the IMU mismatch on every tick, and the rotor on 340–377 ticks (hover).
- No `fw/` or frozen-file change.

**4. W1, commit C3: the L4 and L5 drivers carry the sensors, and the accelerometer tie is proven (ruling 11)**.
- **Drivers.** `run_l4.py` and `run_l5.py` take `sensors=None`, passed through to the world. With the model gyro the
  world carries `<gyro_source>model`, and the truth-gyro element is left out (the plugin takes exactly one). L5's
  attitude source stays truth.
- **World diff extended** (`tests/regression/quad/L06/gz_compositions/test_l4_sensors.py` and `test_l5_sensors.py`,
  per push in gz-l4 and gz-l5, because they need the plugin build's parameter table):
  - every L4 and L5 scenario × m × the 8 sensor sets is checked;
  - `sensors=None` gives a byte-identical world;
  - with sensors, the only differences are the inserted elements, the truth-gyro line removed under the model gyro,
    and `max_step_size` when c ≠ 0;
  - controls are flagged: a truth-gyro line left in, a seed edit, a c = +1 world checked as c = 0.
  - The frozen `test_world_sensors.py` is unchanged; the new files import its sensor sets.
- **Bit-identity:** every gz-suite log is identical by sha256 before and after (L04 33/33, L05 47/47).
- **Test e, the exact accelerometer tie:**
  - The case: L4 step_roll and L5 recover_tumble, m = 1, seed 1, model gyro and rotor on.
  - The two corners differ only in the three accel signs. Their logs are equal byte for byte after zeroing the corner's
    own accel fields in SENSORS and the 12 accel bytes of each tick's IMU sample.
  - Every SIL-visible input and output agrees: the stamps, gyro, flags, DShot, plant outputs, TRUTH and ROTOR.
  - Non-vacuity: the accel bytes differ on every tick, and AccelValid is set on every tick.
  - Control: flipping the gyro x sign changes the DShot on 12122 of 16044 ticks (L4) and 29096 of 40590 ticks (L5).
  - Static guard: the files compiled into the L4 and L5 SIL libraries that name the accel fields are exactly the SIL
    boundary (`marv_sil.h`, `imu_sample.hpp`, `marv_sil.cpp`). Its control is the same scan on `marv_sil_l0`, which
    flags `l0.cpp:88`.
  - At L7 the estimator reads the accelerometer; this test is then expected to fail, and the tie breaks by itself.
- **CI:** steps `gz_sensors_l4` and `gz_sensors_l5` in `ci/run_ci_gz.sh`.
- No `fw/` change. One frozen file changed: `tests/regression/quad/L06/results/r2_envelope_a/envelope_a.txt`, regenerated
  by its own command (`r2_envelope_a.py --out …`). Only its input-hash line changes, because it records `run_l5.py`'s
  sha256, which this commit changes: `f4ef68ef…` becomes `5d611f4f…`. No data line or conclusion changes. The full
  tools suite, nightly files included, gives 1096 passed.
- **Known, not changed:** `run_scenario.locate_offset` has no names for record types 5–7.

**5. The `noise_term_confidence` register entry and the F1 spec line** (rulings 3 and 4; the spec line approved,
"Approved: Luis, 2026-10-04").
- **The entry:** `design/budget.yaml` gains `noise_term_confidence`: 0.99, unit 1, method design-budget, σ choice, in
  the form of `allan_check_confidence` and `stream_independence_confidence`.
  - Its rationale: the family-wise confidence that a correct build passes every one of the N seeds, at the Bonferroni
    per-seed quantile 1 − (1 − c)/N (Dunn 1961); one entry, one purpose; Luis's choice of 2026-10-04.
  - `used_by`: the stage (e) T3 Monte Carlo.
- **Product parameter table:** the entry enters it as record 93, so 94 records in all; schema hash
  0x6563bcd58bca2c34. It was regenerated by the README's command in marv-ci, and the regenerated table equals the
  committed one byte for byte.
- **The spec:** docs/spec/10-quad-flight-software.md §4 L6 pass bar (e) now carries Luis's approved line in place of "at
  quantile p".
- **Frozen files changed by item 5:**
  - `tests/regression/quad/L06/param_ids`: one line, `noise_term_confidence`, appended so that no existing id shifts;
  - `tests/regression/quad/L06/product_params/marv_params_product_table.txt`: regenerated;
  - `tests/regression/quad/L06/results/ff_eval/card_worst.txt` and `tests/regression/quad/L06/results/ff_eval/sweep.txt`:
    regenerated by their own commands (`l6_ff_eval.py --corners 11`, and `--corners all --sensitivity`). Only the
    `budget.yaml` sha256 line changes (`fe18e4c6…` to `673aae3e…`); no data line changes.
- The full tools suite, nightly files included, gives 1096 passed.

**6. The S9 guard for pytest** (ruling 12; second round, item 4: "(a)").
- `tools/ci/pytest_file_time_guard.py`, a stdlib pytest plugin with no new dependency, is loaded in the core job's
  per-push tools step.
  - It reads `per_push_check_time_max` from `design/budget.yaml`, as the CTest TIMEOUT of 0013 does.
  - A file's time is the setup, call and teardown of its tests plus its collection time.
  - A session- or package-scoped fixture is charged in full to every file that uses it, and its cost is removed from
    the file that happened to trigger it, so each file is charged what it pays alone.
  - Not counted: interpreter start-up, conftest import, and the teardown of shared fixtures.
- **Enforcement (ruling (a)):**
  - under `ci/local_ci.sh` it fails the step: `local_ci.sh` passes `-e MARV_LOCAL_CI=1` into every job container;
  - elsewhere, including Actions, it prints the per-file table and any files over the limit, with the exit status
    unaffected.
  - Nightly-only files are `--ignore`d per push, and the guard is off in full mode.
- **Controls** (`ci/run_ci.sh`):
  - the planted file `tests/regression/quad/L06/controls/test_per_push_pytest_overrun.py`, outside the normal collection,
    sleeps 3 × the limit (labelled factor 3) and must make the guard fail. The control sets `MARV_LOCAL_CI=1` itself, so
    it proves detection in both settings;
  - the positive check must pass.
- **The file to watch:** `test_r2_lower_bound.py` at 57.8–58.3 s alone in the CI image. If it crosses 60 s it moves to
  nightly under the existing rule. Timings are taken at load < 1.5 only.
- No frozen file changed.

**7. C4: the noise term W, `tools/sim/l6_noise_term.py`** (the noise suite memo §1; rulings: NT-2, Bonferroni inside the
run, frozen extremes).
- **The model:** the T4e linear design model, built from `l6_ff_eval.setup` and `linear_response` and equal to it bit for
  bit:
  - latency 1, the low-pass, the 12 notches at ω_hover;
  - the PID with its D low-pass, FF+lag linearised;
  - the motor lag, and the ZOH plant at t_nom/(1 + e).
  - The FF is linearised in the controller only; the plant has no ω×Jω (ruling 6).
- **σ per truth channel:** σ_c² = (σ_d² + LSB²/12)·Σh̄² + K_rw²·τ0·ΣH̄², with σ_d, LSB and K_rw from the profile's
  loaders.
  - It is computed exactly per dt phase: four phase-class impulse runs plus the tick-0 seed path, with the step sums by
    recursion.
  - It is the max over the J/τ grid and the frozen operating points (FF at |ω| = 0 and at the band peak P, every sign
    pattern).
  - `sigma_series` gives σ_c(n) per checked execution; `sigma` gives its max.
- **W:** W_c = z·σ_c, with z = Φ⁻¹(1 − (1 − `noise_term_confidence`)/(2·N·M_s)). N comes from the register's seed rule,
  and M_s (the predicate's checked points) is passed in by the caller, never chosen.
- **Tests** (`tests/regression/quad/L06/tools/test_l6_noise_term.py`, 6 tests; the file's own work about 3 s, plus the
  shared `rate_lead_design` fixture):
  - the T4e model equals `l6_ff_eval.linear_response` bit for bit, on 3 axes × 5 members; control: latency 0 differs;
  - σ from the formula equals the direct one-injection-per-tick computation within a first-order rounding bound (N·u·Q,
    Higham §4.2): at most 0.045 of the bound. Controls: swapped dt phases give 6.3e9× the bound; a dropped seed path
    gives 7.4e14×;
  - the coupled FF path is checked the same way, at 0.097 of its bound;
  - σ_d × 1.1 gives W × 1.092, and latency + 1 also raises W;
  - z reproduces 3.506 / 3.761 at M = 1 and 5.893 / 6.054 at M = 120000.
  - A scratch Monte Carlo (1000 runs, not committed) gives σ̂/σ = 1.035 (white) and 0.987 (random walk), inside the χ²
    band [0.938, 1.063].
- **Findings** (scratch runs, for C6/C7):
  - The prefilter's seed from the first noisy sample dominates σ early in a run.
  - step_roll over the full grid: σ = 3.823e-4 rad/s, W = 2.04e-3 rad/s (M_s 4822).
  - acro, roll, card member: σ(14750), at the T4e binding execution, is 3.22e-4 rad/s, so z·σ(n) = 1.9e-3 rad/s. The
    max over the run is 3.33e-3 at execution 539 (the seed path), so a constant W would be 1.96e-2. On a 3×3 grid the
    constant W reaches 34.4 mrad/s, above acro's T4e margin of 25.1. How W is applied, constant or per point, is put to
    Luis.
- No frozen file changed.

**8. C5: the nonlinear bias delta** (ruling 6: "(nonlinear model with the bias) − (nonlinear model without it), added
to the linear envelope. The coupling itself never enters the envelope.").
- **Hooks in `tools/sim/l6_ff_eval.py`**, all off by default (`simulate(..., bias=None, clock_error=0.0, torque=None)`):
  - a gyro bias added after the latency, with the chain seeded from the first sample;
  - the plant tick t_nom/(1 + e);
  - a torque before allocation;
  - `StepPlan` / `step_plan` for the L4 steps.
  - With the hooks off, `ff_eval/card_worst.txt` and `sweep.txt` reproduce byte-identical (`cmp`).
  - The chirp plan adapter is left to C6: it needs build parameters the fixture lacks.
- **`tools/sim/l6_bias_delta.py`** (new):
  - the corners, with the bias = sign × the profile's turn-on bound by `imu_corner_config`'s rule, and the clock error
    realised as in 0012's question E entry;
  - a 3-axis model with an L4 mode and an L5 mode (the oracle's AngleMode and `recovery_model.law` plus the yaw-rate
    term, over `simulate`'s inner loop);
  - `l4_delta` and `l5_delta` = NL(corner) − NL(nominal), signed.
  - **Lead decision:** δ is computed with DShot quantisation off. Q already covers quantisation, and the stateless rounding
    in `simulate` is not the firmware's, since the diffuser is live (0017).
- **Reproduction**, bit for bit:
  - the L4 mode equals `simulate` on acro (T4e, PID + FF + lag, coupling), with the hooks off and with all of them on;
  - the L5 design mode equals `recovery_model.member_run` (tumble and inverted starts, the card member and a box corner),
    and the T3 golden θ/ω and yaw lock (step_roll, yaw_release).
  - Nonlinear against the design plant on L5 step_roll: w 1.69e-3 against its rule 3.25e-3, and φ 1.04e-4 against
    2.66e-4. The J × 1.1 control misses by about 30×.
- **Checks** (`tests/regression/quad/L06/tools/test_l6_bias_delta.py`, 15 tests, each metric check with a negative
  control: J × 1.1, att_kp × 1.1, FF on, sticks one execution late):
  - L4, FF off: the bias acts exactly as a −b offset on the setpoint from the seed execution (0 exactly; FF on differs by
    4.57e-3 rad/s);
  - the L4 hold at hover against the linear −b·s(n): 1.0e-6 rad/s against a one-tick rule of 3.5e-6. The rule is
    INFERRED: the nonlinear plant holds each tick's torque and lags the rotor speed, so it cannot equal the linear
    model to rounding;
  - L5: the identity (bias b against a rate reference −b) holds exactly. The offset δ_err − b/att_kp is −4.5e-9 / −4.95e-9
    (roll / yaw), within the derived bound 5.41e-9;
  - accel-only corners give δ = 0 at L4 and L5 (full model, FF on), and flipping the gyro x sign changes δ.
- **Findings:**
  - the clock-only δ on L4 step_roll is 3.9e-4 rad/s on roll;
  - the realised e at m = 1 is −70.3950441888891 / +70.40495650893823 ppm;
  - t_nom/(1 + e) differs from the plugin's host step by 1 ulp at c = −1.
- No frozen file changed.

## Evidence

Each item's checks are listed with it as it lands.

## Approval

Owner decisions: Luis, 2026-10-04, as quoted. The stage: open.
