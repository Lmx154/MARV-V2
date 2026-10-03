Lower bound on the R2 rate excess forced by the motor lag (decision 0014, owner decision 5, condition 1: evidence first).

R2 as specified before its steady-tumble setup (decision 0015), now the hover-rotor tumble
`scenarios/quad/L05/recover_tumble_prop_strike.yaml`, starts inverted, at `rate_max` on all three axes, with every rotor at the
card's hover speed. The rotor torque at t = 0 is therefore zero, while the coupling ω×Jω is not. The script bounds from below how far
the body rate must leave the R2 design-model envelope for every command history. It then compares that bound with the
tolerance the R2 predicate applies (`tests/regression/quad/L05/gz/test_t4_recovery.py`: envelope ± (E + F + Q)).

Files:
- `r2_lower_bound.py`: the script. Its docstring holds the derivation, steps 1 to 6.
- `bound.txt`: its raw output.
- The pytest is `tests/regression/quad/L06/tools/test_r2_lower_bound.py`, in the path of the CI tools step. It re-runs the
  script, compares the output with `bound.txt` byte for byte, and asserts the claim, the controls and the cross-check.

Command (repository root, after `uv sync --frozen`):
`uv run python tests/regression/quad/L06/results/r2_lower_bound/r2_lower_bound.py --out tests/regression/quad/L06/results/r2_lower_bound/bound.txt`

The script imports `test_t4_recovery`, which needs the generated L5 T3 reference (`tools/refdata/refdata.py ensure
quad/L05/t3`, decision 0011). CI generates it in an earlier step. The run takes about 15 s on 16 cores and about 50 s under
the runner's 4 CPUs.

Inputs. The script reads all of them; none is retyped. `bound.txt` records their sha256.
- The card: J, k, τ, the speed range and B (`gen_plant_config`, `mixer.mixer_matrix`).
- The scenario: ω0, the m sequence and the tick. `run_l5.hover_rotor_speeds` gives the initial rotor speeds; they equal the
  world's speeds in `cause.txt`.
- `test_t4_recovery.make_design`, the test's own function. It gives the R2 envelope, F and Q from the recorded T3 fixture.
- `tests/regression/quad/L05/results/recovery_cause/cause.txt` section 2, the m = 1 gz trace of R2, for the cross-check.

Findings (`bound.txt`):

1. **Coupling at t = 0.** ω0×Jω0 is (0.30083, −0.24614, −0.054697) N m. The rotor torque is 0.
2. **Departure from ω0, for every command history (no gain enters).**
   - Roll: at least 0.2563 rad/s.
   - Pitch: at least 0.27245 rad/s.
   - Yaw: at least 0.020931 rad/s.
3. **Against the predicate.** X* is the forced excess over the envelope.
   - **w_y is the binding axis.** X* is 0.28537 rad/s at execution 17 (5.3125 ms), against F + Q = 0.031128 rad/s, a ratio of
     9.17. F is 0.017472, of which the envelope's halving term is 0.015583; Q is 0.013656.
   - w_x: 0.21596 against 0.032875 (6.57), at execution 12.
   - w_z is forced too: 0.039504 against 0.0079155 (4.99), at execution 22.
   - At execution 17 the predicate can hold only if the run's step-size term satisfies E ≥ 0.25424 rad/s (w_y). E is a
     measurement of each gz run; `e_measured.md` measures it.
4. **The figure in decision 0014.** The "tolerance of about 1.3e-3 rad/s" is R1's w_y F (`cause.txt:55`), which is the
   T3 rate term alone. R2's own F + Q on w_y is 0.031128 rad/s. The architect's form g²τ/(2J·2τ_max) gives 0.1299 rad/s
   (section 5), which is 4.17 times R2's tolerance. Its slew, 2τ_max/τ = 111.05 N m/s, does cover the largest reachable
   slew over the window (68.843 N m/s). This script's bound uses the reachable torque set itself instead of a slew bound.
5. **Cross-check against `cause.txt`.** At executions 1 and 16, every observed departure and excess is at least the bound,
   after allowing half a printed unit. At execution 16 on w_y, the observed excess is 0.56552 against 0.28514.
6. **Negative controls.**
   - **NC1, steady-tumble rotors** (T = M (F, ω0×Jω0)). At the hover collective, T₁ = −0.30146 N, so no rotor speeds give
     that state. The control therefore uses the smallest collective with every rotor at or above W_min, 8.917 N, which
     is a value for this control only. There the bound collapses: L* is 0 and X* is below 0 on every axis.
   - **NC2, equal moments** (ω×Jω = 0). L* is 0 on every axis.

Assumptions:
- A rigid body with the card's diagonal J. The card's σ for J is UNKNOWN.
- The motor model of `sim/plant`:
  - a first-order rotor-speed lag τ;
  - thrust k W² and torque B k W²;
  - commands {0} ∪ [W_min, W_max];
  - no aerodynamic or rotor-gyroscopic torque (marv_plant v0 has none).
- gz applies the wrench computed at the end of each host step (`marv_plant.cpp` `marv_plant_step`; `lockstep.cpp`
  `AddWorldWrench`).
- gz's rigid-body step follows Euler's equation. `cause.txt` section 1 supports this: a torque-free body matches the
  first step within 1.6e-5 rad/s.
- F carries the T3 rounding term, which the test docstring tags INFERRED.

Every step only enlarges the actuator or shrinks the coupling, so L under-estimates the departure:
- the command hull is [0, W_max];
- each rotor sits at its own extreme;
- the torque is taken one host step early;
- the a-priori box only shrinks the coupling;
- the sums are right-endpoint lower sums.

Limit: X and F + Q come from today's R2 design, the recorded fixture's gains. A new gain set changes the envelope and F + Q,
so re-run the script. On w_y the envelope never moves past ω0 in the push direction within the window, so there X ≥ L.
