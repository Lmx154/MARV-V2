The FF-on report runs of L6 stage (c): one T4 run each of L4 acro, L5 R2 and L5 R1X with the rate loop's
lag-compensated ω×Jω feed-forward switched on through the test harness, set against the same run with it off. Reported,
not asserted (decision 0014, owner decision 2; Luis, 2026-10-01: "stage (c) records FF-on evidence: the T3 evaluation and
one T4 run of the acro and R2 scenarios with FF switched on through the test harness. These are reported, not asserted";
third round, item 3: R1X "must pass at (e) with FF live").

The switch. `rate_ff_enable` (`design/scenario_values.yaml`) is 0 in every build, so the firmware flies the PID law and
every suite runs FF-off. With it at 1, `rate::from_params` (`fw/rate/src/rate_loop.cpp`) reads J (`inertia_xx/yy/zz`),
τ_m (`motor_tau`) and T_ff (`rate_ff_filter_tau`, 12.311602 ms, `tools/card/rate_lead.py` rule step 9). The FF-on runs
set it by name as a harness `sil_override` (`rate_ff_enable` i32 1). The FF-off runs carry no harness override: they are
the suites' default runs. Stage (e) switches the default in the change that creates `XFAIL_GATE_CLOSED`.

Files:
- `capture_gz.py`: flies each case FF-off and FF-on with the tests' own code and writes the compact summary. Not run by
  the pytest. Its docstring holds the method and the output format.
- `gz_acro.txt`, `gz_r2.txt`, `gz_r1x.txt`: its raw output, one per case.
- The pytest is `tests/regression/quad/L06/tools/test_ff_on.py`, in the path of the CI tools step. It does not run gz. It
  checks each summary's internal consistency: the switch (overrides, build default 0, FF-on series differ from FF-off),
  every verdict against its recorded statistics by the test's own rule, each worst execution re-computed from its terms,
  and the compare section. Each check has a planted-inconsistency control.

Commands (repository root, after `uv sync --frozen`; gz-sim 8; acro needs the host-gz-l4 build, r2 and r1x the host-gz-l5
build and the generated L5 T3 reference, `uv run python tools/refdata/refdata.py ensure quad/L05/t3`):
- `uv run python tests/regression/quad/L06/results/ff_on/capture_gz.py --case acro --work <dir> --out tests/regression/quad/L06/results/ff_on/gz_acro.txt`
- the same with `--case r2 ... gz_r2.txt` and `--case r1x ... gz_r1x.txt`.
- On 16 cores acro takes about 28 s (the bound 12 s, four gz runs 11 s, the replays); r2 and r1x take a design of 16 s and
  14 s plus four gz runs of 12 s. A second capture of each was byte-identical on the host.
- The gz logs go to `--work`; a rerun regenerates them (decision 0003).

What each case runs (nothing of a predicate is restated in the script):
- **acro:** `test_t4_acro.design_bound` and `fly` (m = 2 then m = 1, the replays, `evaluate`). Both predicates are
  recorded: the pass bar (i)-(v) and the recovery predicate over the window after the combined full-stick segment (the
  strict xfail).
- **R2:** `test_t4_recovery.make_design(R2)` and `fly`: the six-channel envelope predicate (the strict xfail).
- **R1X:** `make_design(R1X, alpha)` and `fly(..., chs=("alpha",))`: the alpha envelope predicate (the strict xfail).
- R2 and R1X: the module's clean-run check is applied to each run; all four runs of each case are clean.

Excess: the worst excess over the tolerance (positive), or minus the slack (negative, a margin). rad/s for rates, rad for
attitude. Acro recovery: |ω| against Z + F + E. R2, R1X: out against E + F + Q.

**Acro, recovery predicate: FAIL → PASS.**

| Axis | FF-off excess (violations, first) | FF-on excess (violations) |
| --- | --- | --- |
| roll | +5.2322 (1518, 13813) | −0.006068 (0) |
| pitch | +2.1931 (1872, 12800) | −0.1433 (0) |
| yaw | +0.2448 (711, 12961) | −0.1149 (0) |

- The FF-on roll margin, 6.07 mrad/s, sits at execution 12897, 97 executions into the window, while ω_x decays from full
  stick (|ω| 12.7947 against Z + F + E 12.8008).
- The pass bar (i)-(v) passes in both. Smallest slack, off → on: roll 0.3277 → 0.3278, pitch 0.2245 → 0.2038, yaw
  0.0360 → 0.1830. The replay reproduces both FF-on runs bit for bit (iv), so `l4_acro_replay` carries the switch too.
  No saturation flag in the combined segment. DShot range 48..1307 → 48..1446.
- T3 prediction (c5, `../ff_eval/`, T4c: truth gyro, notches bypassed): PASS by 3.2 mrad/s, with E = 0 and TOL = 0, which
  is stricter than the test. The gz margin is 6.1 mrad/s. Both are knife edges. c5's flight configuration (T4e, 24.1 mrad/s)
  is not flown until stage (e).

**R2 (steady tumble), envelope predicate: FAIL → FAIL.** Violations 14420 → 2391. These are the numbers with decision
0016's first-read fix in the gz plugin (the scenario's initial body rates are fed at step 0); with
`MARV_GZ_TEST_ZERO_FIRST_READ=1` the plugin restores the zero read at step 0 and the pre-0016 numbers are reproduced
exactly (13923 → 10398).

| Channel | FF-off excess (violations, first) | FF-on excess (violations, first) |
| --- | --- | --- |
| err_x | +0.2365 (1326, 1737) | −0.005088 (0) |
| err_y | +0.3791 (3050, 120) | −0.003806 (0) |
| err_z | +0.2947 (3537, 41) | −0.001579 (0) |
| w_x | +2.3491 (2799, 27) | +0.1406 (324, 917) |
| w_y | +1.3264 (747, 13) | +0.1626 (326, 891) |
| w_z | +1.1477 (2961, 836) | +0.2373 (1741, 1564) |

- The worst channel moves from w_x (+2.349 at n 333) to w_z (+0.237 at n 2234). FF-on has no attitude violation and is
  better on every channel.
- DShot spans 48..2047 in both variants: motors reach both ends of the range. Not examined further here.
- Prediction (`../r2_envelope_a/`): the design model with the ideal feed-forward (continuous, true ω, exact J and τ, the
  first-order torque lag it inverts exactly) stays inside envelope A to rounding. The measured FF-on result agrees with
  the W4 diagnosis model's no-zero-read prediction (about 2416 violations, worst w_z +0.24; decision 0014, W4). What
  remains (0.14 to 0.24 rad/s on the rate channels) is attributed by that diagnosis to yaw saturation plus the yaw
  integrator freeze, INFERRED (not shown by this run).

**R1X (exactly 180°), alpha envelope predicate: FAIL → PASS.**
- FF-off: +5.048e-3 rad (74 violations, first 2271, worst n 2338). FF-on: −2.923e-2 (0 violations, worst n 14026; the
  largest distance outside the envelope 9.0e-4).
- Prediction (`../r1x_coupling/`, claim 4): the design model plus ω×Jω with the ideal feed-forward stays inside, under
  both gain sets. The gz run agrees.

FF-off identity. Each FF-off run is the suite's default run. R1X's FF-off TRUTH series (m = 1 and m = 2) has the same
sha256 as `../r1x_coupling/gz_new.txt`, captured before the feed-forward parameters existed; its verdict is the same.

Inputs. The script reads all of them and retypes none; each summary records the sha256 of each: the card, the scenario, the
test module, the T3 oracle or design model and fixtures, the runner, the build's parameter table and `capture_gz.py`.

Limits:
- One T4 run pair per case, at the card plant. No J or τ corner is flown (c5 carries the corners to the pre-L8 gate).
- The stage (c) T4 sensor configuration: truth gyro, notches bypassed, latency 0. The flight configuration is stage (e)'s.
- A new gain set, scenario or feed-forward parameter changes the runs: re-capture all three summaries.
