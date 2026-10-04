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

## Evidence

Each item's checks are listed with it as it lands.

## Approval

Owner decisions: Luis, 2026-10-04, as quoted. The stage: open.
