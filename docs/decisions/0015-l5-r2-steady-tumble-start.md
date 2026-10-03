# 0015: L5 R2 starts from a steady tumble; the hover-rotor tumble becomes a run-and-report case

This record changes a known safety item, the L5 R2 recovery (0006 decision 15), as owner decision 5 of 0014 requires:
its own record, not a section of the stage (c) record. The strict xfail and its gating are unchanged (0009: the gate
file `tests/regression/quad/L06/XFAIL_GATE_CLOSED`).

## What changed

- **The setup.** R2's initial rotor speeds change from `hover` to `steady_tumble`, by one derived rule
  (`tools/sim/run_l5.py`, `steady_tumble`):
  - the rotor speeds are the mixer inverse of the steady-tumble torque τ0 = ω0×Jω0 at collective c*;
  - c* is the hover collective if that allocation is feasible; otherwise it is the smallest collective at which every
    rotor stays within the card's thrust range.
  - The floor is the card's own minimum rotor thrust, k·ω_min² = 0.035157 N.
  - Today hover is infeasible (motor 1 would need −0.301 N), so c* = 8.917025278630033 N (117.8 % of hover, 7.570578 N).
    The rotor speeds are 150.0, 1426.648, 1334.443 and 1366.840 rad/s.
- **The torque balanced** is every torque `marv_plant` applies at t = 0 (`plant_model.hpp`, `Model::wrench`): the rotor
  thrust moments r×(0, 0, −T) and the rotor yaw reaction s·c_q·T, i.e. exactly the mixer's B. `marv_plant` models no
  rotor inertia (so no gyroscopic torque), no drag and no position-dependent torque.
- **The extra collective does not touch the check.** The world has no ground or contact, and `marv_plant` v0 has no drag
  or position-dependent torque (`recover_tumble.yaml`'s rationale). The firmware's thrust request stays "hover" (m g).
- **The envelope.** It is still the design model, never the observed run. The design model starts at motor deviation 0
  from the steady-tumble trim (owner decision A, below), the existing convention.
- **The hover-rotor tumble** is kept as `scenarios/quad/L05/recover_tumble_prop_strike.yaml`, the crash or prop-strike
  case. It is run and reported (`tests/regression/quad/L06/results/r2_lower_bound/hover_tumble.{py,txt}`), not
  asserted, and has no pass bar until L8. The R2 lower-bound proof (0014, c3) and the E measurement (0014, c6) now read
  this scenario; every proven number is unchanged.
- **E.** R2's T4 predicate uses E measured on the new steady-tumble runs by the same rule, |y(m=1) − y(m=2)|. The test
  computes it; nothing changes there.

**Frozen files changed:**
- `tests/regression/quad/L05/gz/test_t4_recovery.py`, three hunks:
  - the docstring's Runs paragraph (R1 at hover, R2 at `steady_tumble`, this record);
  - one docstring sentence: the design model's motor state starts at 0, the deviation from the trim of the initial rotor
    speeds;
  - the assertion `st["rotor_speed_rad_s"] == "hover"` becomes `== ("steady_tumble" if name == R2 else "hover")`.

  The strict xfail, its gate, the predicate, the bounds, E and the test names are unchanged.
- `tests/regression/quad/L05/tools/test_l5_scenario.py`: the same keyword assertion, at line 315.
- `tests/regression/quad/L05/gz/recovery_model.py` is unchanged: it was edited during the change and then restored to
  HEAD exactly, because envelope A needs no seed.

Outside `tests/regression/`: `scenarios/quad/L05/recover_tumble.yaml` (initial rotor speeds `steady_tumble` and the rule
text), the new `scenarios/quad/L05/recover_tumble_prop_strike.yaml`, and `tools/sim/run_l5.py` / `l5_scenario.py` (the
rule `steady_tumble` and the keyword).

## Why

R2's own scenario calls it "the acro limit", a pilot at full stick on every axis. A body held at constant ω needs
τ = ω×Jω (Euler). Hover rotors describe a body already being kicked at t = 0, not a steady tumble. They came from 0006
decision 23 as a spin-up fix (0 → hover), not as a choice to unbalance the torque.

The evidence that R2 as specified could not be passed by any controller, gathered before the change (owner decision 5,
condition 1):
- **The lower bound** (0014, c3, independently reviewed): from the hover start, every command history forces ω_y past
  the envelope by at least 0.28283 rad/s at execution 16, against F + Q = 0.1151 (ω_x: 0.22176 against 0.0895).
- **The measured E** (0014, c6): E(16) on ω_y is 4.71e-4 rad/s, against the 0.16773 the predicate would need. With E
  measured, the infeasibility is proven.

## Owner decisions (Luis, 2026-10-01, verbatim)

From 0014, owner decision 5 (first round):

> **5. R2: (a), steady-tumble rotor speeds, under four conditions.** […] Conditions: 1. **Evidence first.** […]
> 2. **Derive the rotor speeds by rule.** Hover collective plus the mixer inverse of τ = ω×Jω at the scenario's ω, from
> the card. Add a T1 check that the plant's ω̇ at t = 0 is zero within its derived tolerance. Its negative control is
> hover rotors, which must fail it. 3. **Its own decision record.** […] The strict xfail and its gating stay exactly as
> they are. 4. **Keep the hover-rotor tumble.** Keep it as a run-and-report evaluation. […] Propose one at L8, from an
> absolute recovery requirement rather than the linear envelope.

(The full text is in 0014.)

From 0014, the second round, item 1:

> **1. R2 setup: (a).** Write it as one derived rule. The rotor speeds are the mixer inverse of the steady-tumble torque
> at collective c\*. c\* is the hover collective if that allocation is feasible; otherwise it is the smallest collective
> at which every rotor stays within the card's thrust range (today 8.917 N, 118 % of hover).
> - The floor is the card's own minimum rotor thrust, as in your 0014 note. State that in the record.
> - The torque is every torque the plant applies at t = 0 at those rotor speeds, not only ω×Jω (include the rotor yaw
>   reaction, and rotor gyroscopic torque if `marv_plant` models it). The T1 check, ω̇ = 0 at t = 0 within its derived
>   tolerance and run on the plant itself, catches anything left out. Hover rotors are its negative control and must
>   fail it.
> - Record why the extra collective doesn't touch the check: the world has no ground or contact, and `marv_plant` v0 has
>   no drag or position-dependent torque (`recover_tumble.yaml`'s rationale). The firmware's thrust request stays
>   "hover" (m g).
> - The envelope comes from the design model started from the same initial state, including the initial torque in its
>   motor-lag state, and never from the observed run.
> - The frozen edits (the `rotor_speed_rad_s == "hover"` assertion in `test_t4_recovery.py` and the scenario's rule
>   text) go under R2's own decision record. The strict xfail and its gating stay exactly as they are.
> - Keep the hover-rotor tumble as a run-and-report evaluation (the crash or prop-strike case). Record its excess, and
>   note "no pass bar until L8" in the handoff.

The envelope ruling (third message, after the lead showed that the literal reading kicks the coupling-free envelope):

> **R2 envelope: A.** The design model's motor state starts at 0, as a deviation from the steady-tumble trim. My
> earlier line, "including the initial torque in its motor-lag state", was ambiguous. A is what I meant, so B's literal
> reading is withdrawn.
>
> Why A, stated as the rule for record 0015:
>
> - **The design model is linear in deviations from an operating point.** At hover its motor state starts at 0, meaning
>   "at the hover trim", not "zero thrust". R2's operating point at t = 0 is now the steady tumble. The same convention
>   gives motor deviation 0 there, so A is the existing convention, not a new choice.
> - **A is the envelope a perfect controller would track; B is the envelope no controller can track.**
>   - In the plant, the motors start at τ0 and lag toward the command, while the coupling ω×Jω(t) changes as ω decays.
>   - Give the firmware ideal lag-compensated FF (command = design command + ω×Jω(t)). The τ0 transient and the coupling
>     then cancel, and the net torque equals the design command alone. That is exactly envelope A.
>   - Envelope B adds a τ0·e^(−t/τ_m) kick that the plant doesn't have at t = 0, because τ0 is balanced by ω×Jω there.
>     So even a perfect controller would fail B. B would put back the very mismatch the setup change removed.
> - **So A asks exactly R2's question:** does the controller reject the change in the coupling as the body recovers?
>
> **Evidence for 0015:** reuse c5's counterfactual tooling. Run the design model with ω×Jω in the plant, the plant
> motors starting at τ0 and exact lag-compensated FF in the controller, and show it stays inside envelope A to
> rounding. Also show it leaves B near t = 0. That's a T3 run, so it's cheap, and it turns the argument above into a
> committed result.
>
> Nothing else changes: the envelope is still generated by the design model, never from the observed run, and R2 stays
> a strict xfail. Continue with 3a–3d as planned.

## Evidence

- **T1, the steady start** (`tests/regression/quad/L06/tools/test_steady_tumble.py`, run on `marv_plant` through its C
  ABI). With the rule's rotor speeds, ω̇ at t = 0 = (9.0e-14, 3.0e-15, −3.8e-15) rad/s², against derived tolerances
  (7.9e-13, 7.5e-13, 1.4e-13).
  - Negative control, hover rotors: (−120.3, +117.2, +12.7) rad/s². It fails, as required.
- **Envelope A, the ideal-FF counterfactual** (Luis's evidence request; `tests/regression/quad/L06/results/r2_envelope_a/`,
  pytest `tests/regression/quad/L06/tools/test_r2_envelope_a.py`). The test's own `member_run` is used with the plant
  swapped for this one run: a rigid body with ω×Jω, R2's ω0, the motors starting at τ0, the design-model controller plus
  exact lag-compensated FF (ω×Jω(t) + τ_m·d/dt). "Motor model exact" is read as the design model's first-order torque
  lag, which the FF inverts exactly.
  - **(a) It stays inside envelope A to rounding.**
    - It equals the design model started from the trim rounding v0 = τ0 − ω0×J_f32·ω0 = (−3.0e-8, 2.8e-8, 2.6e-9) N·m
      to 6e-14 rad/s. All of v0 comes from the fixture's float32 J.
    - It is outside A only at n = 1, by at most 4.1e-9 rad/s, within δ = (2.9, 3.3, 1.4)e-7 rad/s. A has zero width
      there.
    - From n = 2 on it is strictly inside (smallest margin 3.9e-5 rad/s).
  - **(b) It leaves envelope B at n = 1** (t = 0.3125 ms), by 0.025 / 0.024 / 0.003 rad/s, growing to 0.64 / 0.61 / 0.027.
  - **Controls.** Without the FF it leaves A from n = 1, by up to 2.40 / 1.15 / 1.14 rad/s. Plain FF without the lag term
    fails (a) by 0.77 / 0.55 / 0.14.
  - It is byte-identical on the host and in marv-ci (`--cpus 4`); the pytest takes 20 s there.
- **The R2 gz run under A, today's gains:** 15 passed, 1 strict xfail (no XPASS).

  | Channel | outside the envelope | over E + F + Q | first violation |
  | --- | --- | --- | --- |
  | ω_x | 3.030 rad/s | 2.932 | n = 33 |
  | ω_y | 1.759 rad/s | 1.639 | n = 25 |
  | ω_z | 1.340 rad/s | 1.275 | n = 1054 |

  The attitude errors exceed by 0.27–0.56 rad. The old hover-rotor R2 exceeded from n = 4. Stage (e) (FF live) is where
  R2 must pass.
- **The evidence at the final stage (c) gains** (att_kp 3.1539721, window 0..20292; W3b, each artefact regenerated by its own
    script, the byte-for-byte pytests pass). The proof is stronger:
    - X* against F + Q: ω_y 0.28537 at n 17 against 0.031128 (9.17×); ω_x 0.21596 against 0.032875 (6.57×); ω_z now
      forced, 0.039504 against 0.0079155 (4.99×).
    - Measured E at n*: 5.33e-3 / 1.75e-3 / 2.54e-3 rad/s, against 0.254 / 0.183 / 0.0316 needed. The largest E over the
      whole window is below the needed value on every axis. The stop condition is not met.
    - Envelope A still holds to rounding (C = D(v0) to 5e-14). B is left at n = 1, and the no-FF control leaves A.
    - The hover tumble: 20461 violations (reported).
    - The numbers above are the earlier-gain record.
- **The hover-rotor tumble (run and report):** 14353 violations, recorded in `hover_tumble.txt`. No pass bar until L8.

## Approval

Owner decisions: Luis, 2026-10-01, as quoted above.

The change as built: approved. Luis, 2026-10-03 (verbatim): "0015: approved as built, on this checklist. Confirm each
point in one line in 0015; if any differs, show me." Each point, confirmed:
1. **The c\* rule:** hover if that allocation is feasible, else the smallest feasible collective, with the card's
   minimum rotor thrust k·ω_min² = 0.035157 N as the floor ("What changed", the setup).
2. **The torque includes everything the plant applies:** the rotor thrust moments and the yaw reaction, i.e. the
   mixer's B; `marv_plant` has no rotor inertia, drag or position-dependent torque ("What changed", the torque balanced).
3. **The T1 ω̇ = 0 check, with hover rotors failing it:** `tests/regression/quad/L06/tools/test_steady_tumble.py`.
   ω̇ is at most 9.0e-14 rad/s² against tolerances of at least 1.4e-13; hover rotors give (−120.3, +117.2, +12.7) and
   fail (Evidence).
4. **E measured on both setups:**
   - the hover-rotor setup, committed (`results/r2_lower_bound/e_measured.txt`, 0014 c6);
   - the steady tumble, measured by the T4 test itself on every run, |y(m=1) − y(m=2)|. Its values are in the run
     output (CI, at the final gains, R2 FF-off: E at most 2.9e-2 rad/s); no separate artefact is committed.
5. **Envelope A, with the perfect-FF counterfactual committed:** `tests/regression/quad/L06/results/r2_envelope_a/` and
   its pytest. Under ideal lag-compensated FF it stays inside A to rounding; without FF it leaves A (Evidence).
6. **The crash case kept as run-and-report, with "no pass bar until L8" in the handoff:**
   `scenarios/quad/L05/recover_tumble_prop_strike.yaml`, `results/r2_lower_bound/hover_tumble.txt`, and
   `docs/handoff.md` (known items).
7. **The strict xfail and its gating unchanged:** the R2 xfail lines of `test_t4_recovery.py` are untouched since
   1159d5c. The file's other hunks are this record's keyword and docstring and 0014's R1X section.

Note for 0016: R2's gz numbers above carry the step-0 zero-rate read (0014, W4). 0016 removes that read and records
R2's numbers again.
