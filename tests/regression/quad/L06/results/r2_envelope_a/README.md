R2 under envelope A: the design model with ω×Jω in the plant, the motors starting at the steady-tumble torque τ0, and
exact lag-compensated feed-forward in the controller (decision 0015; decision 0014, owner decision 5 and second round
item 1; Luis, 2026-10-01: "R2 envelope: A").

R2 (`scenarios/quad/L05/recover_tumble.yaml`) starts at the steady tumble. Its design model
(`tests/regression/quad/L05/gz/test_t4_recovery.py`, `recovery_model.member_run`, no ω×Jω) is linear in deviations from an
operating point:
- **Envelope A** starts the model's motor state at 0, a deviation from the steady-tumble trim. This is the R2 envelope.
- **Envelope B** starts it at τ0, the plant's torque at t = 0. This is the literal reading, withdrawn.

The owner's argument is that A is the envelope a perfect controller would track and B the envelope no controller can track.
This run turns that argument into a committed result. Both envelopes come from the design model, never from an observed
run, and R2 stays a strict xfail.

Files:
- `r2_envelope_a.py`: the script. Its docstring holds the model, the tolerance rule and the claims.
- `envelope_a.txt`: its raw output.
- The pytest is `tests/regression/quad/L06/tools/test_r2_envelope_a.py`, in the path of the CI tools step. It re-runs the
  script, compares the output with `envelope_a.txt` byte for byte, and asserts the claims, the guard and the control.

Command (repository root, after `uv sync --frozen`):
`uv run python tests/regression/quad/L06/results/r2_envelope_a/r2_envelope_a.py --out tests/regression/quad/L06/results/r2_envelope_a/envelope_a.txt`

The script imports `test_t4_recovery`, which needs the generated L5 T3 reference (`tools/refdata/refdata.py ensure
quad/L05/t3`, decision 0011). CI generates it in an earlier step. The run takes about 10 s on 16 cores, about 20 s in the
marv-ci image under `--cpus 4` (where its output is byte-identical to the host's), and about 33 s pinned to 4 logical CPUs
(2 cores) of the host.

Inputs. The script reads all of them; none is retyped. `envelope_a.txt` records their sha256.
- The card and the R2 scenario: ω0, q0 and the window, and τ0 (`run_l5.steady_tumble`, `run_l5.rotor_torque`).
- The recorded T3 fixture `attitude_t3_inputs.txt`, read by `test_t4_recovery.recorded_inputs`: today's L5 gains, J, τ, the
  bands and the tick.
- The design model's code: `test_t4_recovery.py`, `recovery_model.py`, `attitude_t3_oracle.py`; and `run_l5.py`.

Method:
- **Envelopes.** `member_run` over the 17×17 τ×J grid, reduced by pointwise min and max. A covers R2's whole window,
  executions 0 to 20292. B covers the first 32 executions, which is all (b) needs.
- **Run C.** `member_run` of the nominal member, unchanged, with its plant swapped for this run only:
  - the rigid body J ω' = m − ω×Jω;
  - the design model's first-order torque lag, τ m' = u + f − m, from m(0) = τ0;
  - the ideal feed-forward f = ω×Jω + τ·d/dt(ω×Jω), exact in J and τ, continuous in time, on the true ω;
  - RK4, 4 steps per tick.

  The controller, the gains, the kinematics and the channels are the test's own.
- **Why it should track A.** With v = m − ω×Jω, the plant is J ω' = v, τ v' = u − v: the design model exactly, from
  v(0) = τ0 − ω0×Jω0.
- **Tolerance δ = V + K + R per channel.**
  - V, the trim's rounding: the design model's response to v0 = τ0 − ω0×J_d ω0. J_d is the fixture's float32 J; τ0
    comes from the card's binary64 J.
  - K, integration: the change when the RK4 step is halved.
  - R, the method floor: the same integrator on the design model's own plant, against the design model.

  A guard checks that K + R is at binary64 rounding scale.

Findings (`envelope_a.txt`):

1. **The trim's rounding.** v0 = (−3.02e-8, 2.76e-8, 2.60e-9) N m. All of it is the float32 rounding of J; the steady-tumble
   rule's own binary64 residue is about 2e-16 N m. Its response through the design model is V = 2.46e-7, 2.70e-7 and
   1.02e-7 rad/s (ω_x, ω_y, ω_z).
2. **(a) Run C stays inside envelope A to rounding over the whole window.**
   - It equals the design model from v0 to 5.3e-14, 3.1e-14 and 4.3e-14 rad/s, within K + R (2.9e-13, 2.2e-13, 3.4e-13).
   - Against RK4 steps per tick, the largest residual over the rate channels falls from 4.7e-12 (1 step) to 3.0e-13 (2),
     then holds at about 5e-14 (4 and 8): the rounding floor.
   - Against A's nominal member it differs by at most V.
   - It is outside A only at execution 1, by 3.8e-9, 4.1e-9 and 1.9e-10 rad/s, all within δ. There every member of A still
     sits at ω0: the first rate period's command is the seed's 0, so A has zero width at executions 0 and 1.
   - From execution 2 on it is inside A. The least margin is 1.1e-5, 1.4e-5 and 7.2e-6 rad/s (ω_x, ω_y, ω_z), at executions
     20056, 19731 and 18441, where A has narrowed to a width of 2.9e-3, 2.7e-3 and 1.7e-3 rad/s.
3. **(b) Run C leaves envelope B at execution 1 (t = 0.3125 ms).** It is outside B by 0.0249, 0.0243 and 0.00263 rad/s, and
   the gap grows to 0.636, 0.605 and 0.0224 rad/s within the first 32 executions. B's members start with the full τ0 in the
   motor and none of it balanced, so they all leave ω0 at once.
4. **Control: without the feed-forward, run C leaves envelope A.** It leaves at execution 1 (1.8e-4, 1.7e-4 and 1.9e-5 rad/s
   beyond δ). The largest exits are 2.02 rad/s (ω_x, execution 336), 0.811 (ω_y, 189) and 1.06 (ω_z, 1764). That is the
   coupling.
5. **Envelope A is the R2 test's.** The pytest checks the script's reduction against `recovery_model.envelope` over the first
   32 executions. A one-off host check, not committed, found it equal (difference 0) to `test_t4_recovery.make_design`'s lo and hi at
   every execution and channel of the whole window, executions 0 to 20292, at the final gains.

Assumptions and limits:
- The motor is the design model's first-order torque lag, which the feed-forward inverts exactly. `marv_plant`'s rotor-speed
  lag with thrust ∝ W² is not modelled here, so this is not a claim about the gz plant.
- The feed-forward is ideal: continuous in time, on the true ω, with the plant's own J and τ. The firmware's feed-forward is
  sampled, filtered and uses the card's J; c5 (`tests/regression/quad/L06/results/ff_eval/`) evaluates that one.
- Only the nominal member is run. At any other grid member the identity above holds with that member's J and τ (not run).
- The gains are today's recorded fixture. A new gain set changes both envelopes, so re-run the script.
