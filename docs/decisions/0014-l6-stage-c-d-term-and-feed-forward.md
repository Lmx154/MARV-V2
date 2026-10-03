# 0014: quad L6 stage (c), the D term and ω×Jω feed-forward

This record opens L6 stage (c) (quad spec §4 L6). It holds Luis's stage (c) decisions verbatim, the accepted design,
and the build, evidence and close as they come. The change to R2's initial rotor speeds gets its own record (owner
decision 5, condition 3).

## What changed

Nothing built yet. The plan, in four commits, keeps CI green at each one:
1. **fw:** the D low-pass (T_f = 0 keeps today's unfiltered law), the lag-compensated ω×Jω feed-forward (inert while
   J = 0), and the gyro chain's NaN guard, each with T1 tests. No product change.
2. **tools:** the PI × lead gain rule, the Ms and noise checks, the T_ff rule and the attitude regeneration, with no
   product change.
3. **One atomic switch:** the new product gains, the chain wired into the compositions, every regeneration
   (0011 SHA256SUMS) and frozen edit, plus this record's list of them.
4. **Evaluation results:** FF-on runs of acro and R2 (reported, not asserted), and the `step_cause` re-run.

The known failing items stay strict xfails until stage (e) creates the gate file. FF goes live at (e) with it (owner
decision 2).

## Why

Quad spec §4 L6 stage (c) and decisions 0009 (D1–D7, F1–F3) and 0013 (the chain costs 9.6° at today's gains: the
worst-corner PM is 35.5° against the 45° floor).

## Stage (c): the design (architect consult, 2026-10-01; scratch numbers, the generator recomputes them)

- **Feasibility.** Model: the exact discrete loop of `tools/card/gyro_chain_design.py`, with the chain at ESC 2 %, all
  notches at ω_th (the worst operating point, confirmed on a 9×9 J×τ grid), the 1-sample latency, and the firmware law.
  PI × lead meets the floors at every band-box corner.
- **Gain rule (D3/D4).**
  - Prototype: C = K(1 + ω_i/s)(1 + s/ω_z)/(1 + s/ω_p), with ω_i = ω/2.414, ω_z = ω/√N, ω_p = ω√N, and K from |L_nom| = 1.
  - ω_c(N) by `rate.py`'s sup-PM rule.
  - N* = the largest N within `Ms_max` and the D-path noise budget.
  - At the flown ESC, Ms binds, not noise: N* = 3.878, nominal ω_c 12.13 rad/s.
    - Worst PM 45.00° at (J−, τ+); worst Ms 2.000 at (J−, τ−).
    - D-path RMS 0.494 mN against 2.281 mN.
    - ω_c²Tτ_lo = 0.0071.
  - For comparison, PI alone with the chain reaches 6.02 rad/s.
- **D low-pass (D2).** T_f = 1/ω_p = 1/(√N*·ω_c,nom), the lead's own pole (41.9 ms at N*). T_f = 0 keeps the unfiltered
  L4 law, so the frozen L4 hand tables and the L5 rate-bypass golden stay valid.
- **ω×Jω evaluation (F1–F3).** Variants: PID; plain FF; lag-compensated FF (adds τ_m·d/dt(ω×Jω), with ω̇ from the chain
  output through a first-order T_ff). The input is the chain output at the rate execution. Plants: the card plant and
  the J corners.
  - Scratch predictions on the card plant:
    - D alone fails acro;
    - plain FF still fails;
    - lag-compensated FF passes.
  - Over the J band every variant fails: FF cancels the coupling only where J is known.
- **R2 as specified is infeasible for any realisable controller.** It starts with zero differential thrust while the
  pitch coupling is 0.246 N·m. The motor lag forces an excess over the envelope.
  - **Correction (c3, 2026-10-01).** The "about 1.3e-3 rad/s" tolerance the architect quoted is R1's predicate line
    (`L05/results/recovery_cause/cause.txt:55`), not R2's. R2's tolerance on ω_y is F + Q = 0.1151 rad/s.
  - c3's bound: X* = 0.283 rad/s on ω_y (ratio 2.46) and 0.222 on ω_x (ratio 2.48); ω_z is not forced.
  - Unproven until the condition 1 evidence is committed and reviewed (see the build).
- **Wiring.**
  - One shared rate-group step: `chain.filter` every tick, `update_notches(hal_rotor_speed())` at the rate execution
    (before that tick's filter), and the rate loop on the chain output. It is used by both compositions and the replay
    tools.
  - The Gazebo plugin's rotor-speed and IMU-model wiring stays at stage (e). Stage (c) T4 flies the truth gyro with
    notches bypassed, and its envelopes model that configuration. T3 carries the ω_th worst case.
- **NaN guard (0013 known limit).** The chain resets an axis to the steady state of the next valid finite sample, with
  a flag and a counter, and seeds at the first valid sample, so D sees no start-up kick.
- **Findings for Luis.**
  - At ESC 2 % the closed-loop time constant at the J+ corner is about 0.161 s, barely changed from PI.
  - A crystal ESC would give N* 4.06, ω_c 23.6 rad/s, τ_cl about 0.086 s, with noise then binding.

## Owner decisions (Luis, 2026-10-01, verbatim)

> Good report. Feasibility at 45.0° and Ms 2.000 with the chain in the loop is the result we needed. The answers
> follow, in your order. Two differ from your recommendations: 3 and part of 5.
>
> **1. FF form: lag-compensated.** Plain FF leaves acro failing, so it doesn't meet the bar.
>
> **2. FF goes live at stage (e), with the gate file: yes.** That avoids the unexpected-pass trap. So that (e) is a
> confirmation rather than a first try, stage (c) records FF-on evidence: the T3 evaluation and one T4 run of the acro
> and R2 scenarios with FF switched on through the test harness. These are reported, not asserted.
>
> **3. J corners: physical 3-D corners, not common-scale.** ω×Jω depends on the differences between the principal
> moments, for example (J_yy − J_zz)·ω_y·ω_z. A common scale moves all of J together and never tests those differences,
> which is exactly where FF mis-cancels. Use the vertices of the ±50 % box intersected with the triangle inequalities:
> each moment at most the sum of the other two. That drops the corners that aren't real inertias. The common-scale
> corners stay as a subset, and rate.py's per-axis convention is unchanged for the loop margins. This is a T3
> design-model run, so the extra corners are cheap.
>
> **4. Acro closes at the card plant: yes, with the corner excess carried to a pre-L8 gate rather than to B1/B3.** The
> original ruling (0005 decision 12) was that it must pass before L8. Record the excess at the physical corners from
> item 3. Add to the handoff: before L8, measure J with its σ (S0, pendulum), then re-run the acro check over the
> measured band. The pilot doesn't fly until it passes there.
>
> **5. R2: (a), steady-tumble rotor speeds, under four conditions.**
>
> Why (a) is a setup fix and not a re-seed: R2's own scenario file calls it "the acro limit", a pilot at full stick on
> every axis. A body held at constant ω needs τ = ω×Jω (Euler's equation). So hover rotors don't describe a steady
> tumble: they describe a body that is already being kicked at t = 0. Hover rotors came from 0006 decision 23 as a
> spin-up fix (0 → hover), not as a choice to unbalance the torque. The envelope still comes from the design model,
> never from the observed run.
>
> Conditions:
> 1. **Evidence first.** Commit the lower-bound proof (the 0.13 rad/s motor-lag excess against the 1.3e-3 rad/s
>    tolerance) as a script with its inputs and raw output, with a negative control, and have it independently
>    reviewed. This happens before the predicate changes, not after.
> 2. **Derive the rotor speeds by rule.** Hover collective plus the mixer inverse of τ = ω×Jω at the scenario's ω, from
>    the card. Add a T1 check that the plant's ω̇ at t = 0 is zero within its derived tolerance. Its negative control is
>    hover rotors, which must fail it.
> 3. **Its own decision record.** This changes a known safety item: the frozen check that R2 starts at "hover"
>    (`test_t4_recovery.py:275`) and the scenario. So it gets its own record, not a section of the stage (c) record.
>    The strict xfail and its gating stay exactly as they are.
> 4. **Keep the hover-rotor tumble.** Keep it as a run-and-report evaluation. It is the collision or prop-strike case,
>    and it doesn't disappear. Record its excess, and log in the handoff that this case has no pass bar yet. Propose one
>    at L8, from an absolute recovery requirement rather than the linear envelope.
>
> Not (b): it puts the coupling inside the envelope, so the check would stop asking whether the controller rejects it.
> Not (c): the motor-lag excess persists in the attitude after any start window.
>
> **6. The N\* rule: yes.** The floors (`PM_min`, `Ms_max`) are the margin, so the design sitting exactly on them is
> intended. Don't add a second margin on top. Keep the numeric guard.
>
> **7. Crystal ESC as a requirement: no.** The flown ESC would fail it the day it's written. Record it in the handoff as
> a finding: the ESC clock error is the largest single limit on crossover. Also record a B3 candidate: estimate each
> ESC's clock scale in flight from the gyro's harmonic peaks, which tightens ε without new hardware (INFERRED, an idea
> only).
>
> **8. T_ff rule: reuse `d_path_noise_budget`, no new entry.** Apply it to the combined RMS of the D path plus the FF
> path, at rate_max, in units of the hover DShot step. Rate_max is where FF noise peaks; at hover it is about zero,
> because the term is quadratic in ω. The hover step is the conservative unit. Applying it to the combined total stops
> the budget being spent twice. That is a new line in the spec's (c) pass bar, so send me its text to approve.
>
> **One more finding to log.** A closed-loop time constant of 0.161 s at the J+ corner is the "sluggish" question
> again, and the ±50 % J band drives it as much as the ESC does. Measuring J (S0) and the motor τ_m on the bench shrinks
> the band box and raises the achievable crossover by rule. Put both on the pre-L8 hardware list.

## Owner decisions, second round (Luis, 2026-10-01, verbatim)

> Good work on commits 1 and 2. Here are my three answers. Continue stage (c) on them in the order at the bottom, and
> only stop to ask if something below turns out to be false.
>
> **1. R2 setup: (a).** Write it as one derived rule. The rotor speeds are the mixer inverse of the steady-tumble torque
> at collective c\*. c\* is the hover collective if that allocation is feasible; otherwise it is the smallest
> collective at which every rotor stays within the card's thrust range (today 8.917 N, 118 % of hover).
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
>
> **2. E: yes, and do it first, before R2's setup change.** E for R2 is already defined as \|y(m=1) − y(m=2)\| over
> its own two runs (the scenario's `m_sequence`), so this measures a defined term. A full-rate tumble is where a
> borrowed E is least representative. Commit the script, its inputs and its raw output alongside the lower-bound
> proof. If the measured E closes the 0.13 vs 1.3e-3 rad/s gap, stop and report. After the setup change, R2's T4
> predicate uses E measured on the new steady-tumble runs, by the same rule.
>
> **3. Spec lines: approved with this wording.** If your draft differs in substance, show me yours before applying it.
>
> Pass bar, replacing the existing (c) noise line:
>
> > (c) T3: with the profile's noise, the combined RMS contribution of the D path and the ω×Jω feed-forward path to each
> > motor's command is at most `d_path_noise_budget` × the hover DShot step's thrust. It is evaluated at hover and at
> > `rate_max` on all three axes, with the feed-forward at the card's J and τ_m. The budget is spent once, on the sum.
>
> Keep hover in the line, so no check the approved bar already makes is dropped. Record why the hover DShot step is the
> conservative unit: thrust per step grows with rotor speed.
>
> Builds, replacing the existing feed-forward bullet:
>
> > - ω×Jω feed-forward with motor-lag compensation, from the filtered measured rate and the card's J and motor time
> >   constant τ_m. Its derivative filter T_ff is set by the combined D + FF noise line of the pass bar. Built and tested
> >   in stage (c), and inert until stage (e) switches it on, in the same change that creates
> >   `tests/regression/quad/L06/XFAIL_GATE_CLOSED`.
>
> Cite c5's evaluation in 0014 as the evidence that the old bullet's condition ("built if the evaluation shows … need
> it") is met.
>
> **One finding to carry.** At the physical J corners, FF can make things worse than PID (pitch at corner 11, J = 1.5 /
> 1.5 / 0.5: 5.14 → 9.46 rad/s). Record it in 0014. In the handoff's pre-L8 gate, the measured-J re-check must run
> acro both with FF on and with FF off, so FF only flies if it helps over the measured band.
>
> **Order:**
> 1. Measure R2's E (item 2).
> 2. R2's setup change, under its own record, with the steady-tumble T1 check and the hover-rotor negative control.
> 3. Commit 3, the atomic switch: product gains, the chain wired into the compositions, every L4/L5 regeneration and the
>    frozen edits. Before the close, run one reported T4 run each of acro and the new R2 with FF switched on through the
>    harness, not asserted.
> 4. The stage (c) close: full `ci/local_ci.sh`, then my approval, then push. Watch the two test files at 54 s and 50 s
>    against the 60 s limit. If either crosses it in the CI image, bring me the S9 nightly option rather than trimming
>    the test.
>
> **Meanwhile, stage (d).** While commit 3's regenerations run, draft stage (d)'s decision round (DShot error
> diffusion: the design and the questions only, no repo writes), so it can start as soon as (c) closes. Bring me its
> questions with the stage (c) close.

Lead notes on the second round:
- **The E stop condition, with the corrected numbers.** "0.13 vs 1.3e-3 rad/s" are the architect's first figures.
  c3's reviewed proof gives a forced ω_y excess of 0.28283 rad/s at execution 16, against F + Q = 0.1151. So the measured
  E "closes the gap" iff E(16) on ω_y ≥ 0.1677 rad/s. That is the stop condition used.
- **The spec lines** are applied in Luis's wording (quad §4 L6, Builds and pass bar (c)).
  - Why the hover DShot step is the conservative unit: thrust per DShot step grows with rotor speed (thrust ∝ ω², so
    ΔT ≈ 2kω·Δω). The line is evaluated at hover and at `rate_max`, both at or above hover rotor speed, so the hover
    step is the smallest step at the evaluated points and the budget is the strictest there.
  - Correction (stage (d) architect, 2026-10-01). An earlier version of this note called the hover step "the smallest
    step in the flown range". That is false: the step at idle (150 rad/s) is about 0.62 mN, against 4.56 mN at hover.
    Luis's reason, that thrust per step grows with ω, is the correct one.
  - The old bullet's condition ("built if the evaluation shows the L4 acro and L5 R2 checks need it") is met by c5's
    evaluation: PID alone fails L4 acro at the card plant in every sensor configuration, and so does plain FF.
    Lag-compensated FF passes in the flight configuration.
- **Finding carried (Luis).** At the physical J corners FF can make things worse than PID: pitch at corner 11,
  J/J0 = (1.5, 1.5, 0.5), goes 5.14 → 9.46 rad/s. The pre-L8 gate's measured-J re-check runs acro with FF on and with
  FF off, so FF flies only if it helps over the measured band.

## Owner decisions, third round (Luis, 2026-10-02, verbatim)

> Good diagnosis, especially catching that ω_th is the best case for the attitude loop, not the worst. Answers, in your
> order.
>
> **1. Attitude design over every chain configuration flown or tested: yes (k = 3.154).** This is the same rule as
> t_cross. Two additions:
> - **Apply the rule to the rate loop too.** Assert the rate loop's PM and Ms over the same configuration set, rather
>   than relying on "ω_th is the worst case". That assumption just failed once for the attitude loop. If the rate loop
>   is fine, this costs one T3 sweep. If it isn't, stop and report.
> - **The bypassed, latency-0 configuration stays in the set** while Gazebo flies it. Dropping it at stage (e) would
>   need its own decision, so don't plan on it.
>
> **2. Chirp plateau: a derived rule, not a picked threshold.** Any threshold I pick after seeing this table would be
> tuned to the result. Use this instead:
> - **Admission.** A run k is in the plateau if its modelled nonlinear PM shift (|kinematic + quantisation|, from the
>   design model, as you computed) is smaller than the PM change the fine negative control (attitude gains × 1.1)
>   produces on the design model at the same corner. Rationale: a run whose nonlinearity is as large as the fine fault
>   can't tell the two apart, so it isn't measuring the linear margin. The threshold then comes from an existing
>   control, not a new number.
> - **Floor guard.** The plateau must hold at least three runs, so U_A is a spread and not a single difference. Fewer
>   than three fails the test; it does not shrink silently. If the derived rule leaves fewer than three on roll or
>   pitch, stop and report rather than adjusting anything.
> - **Unchanged:** pass = min − U_A ≥ PM_min over the admitted runs. The gains × 2 control must still fail. Excluded runs
>   are still flown and printed with their modelled shift.
> - **Record** in 0014 that this amends 0006 decision 20's plateau definition, and why: A_env rose 1.43× with the new
>   loop, so k1 now reaches 173° of tilt. Stage (d)'s error diffusion shrinks the quantisation shift, so k5 may re-enter
>   by rule, with no edit.
>
> **3. R1X: (a), narrowed.**
> - Xfail only the envelope predicate, with strict xfail on the same gate file. R1X's other assertions keep running as
>   normal tests: the clean run, the DShot range and the settled end value. That way the side check doesn't go dark
>   until (e).
> - Evidence first, as with R2: commit the counterfactual showing that Gazebo's excess is the coupling. That means the
>   design model plus ω×Jω reproduces Gazebo, as for R2's 7.96 → 1.46e-2, and that it is identical under the old and
>   new gains.
> - R1X joins R2's line in the handoff: it must pass at (e) with FF live.
> - It's a frozen-test change, so it goes in its own section of 0014 with my approval quoted.
>
> Not (b): coupling inside the envelope stops the check from asking whether the controller rejects it. Not (c): Gazebo
> flies identically under both gain sets, so the gains aren't the cause.
>
> **4. The frozen count edit in `test_l5_scenario.py`: authorized, after k is final.** Item 1 changes k again (3.228 →
> 3.154), so 19829 is already stale. Make the edit once, with the value from the final k. While you're there, make the
> positive assertions (lines ~139 and ~230) read the segment start from the scenario's derived value instead of a
> literal, so the next regeneration needs no frozen edit. Keep line ~77's "not the recorded one" control pinned to a
> literal on purpose, and say so in a comment. Record it in 0014 with my authorization quoted. If the permission prompt
> appears again, show it to me and I'll approve it there.
>
> Then continue: FF-on report runs, full local CI, and the close.

Lead notes on the third round:
- **Correction.** The design text "notches at ω_th (the worst operating point)" holds for the rate loop only. For the
  attitude loop, ω_th is the best case: the chain's lag sits in the rate feedback, so it appears as phase lead from rate
  command to true rate. At product k 3.228, (J+, τ+), latency 1, the attitude PM is 44.54° at 600 rad/s, 44.25° at
  hover, 44.02° at 2800 rad/s, and 43.87° bypassed with latency 0.
- **The configuration set** for every design rule from now on: notches at ω_th or above (up to ω_max), and notches
  bypassed, each with latency 1 (flight) and 0 (the truth-gyro T4 configuration).

## Owner decisions, fourth round: the stage (c) close (Luis, 2026-10-03, verbatim)

"Good close: full local CI on the whole tree, and every frozen file is covered by a record. Rulings follow in your
order. Three of your lead rulings come back with a question, and the commit waits on those answers.

**1. S9: B, then A for whatever is still over.**
- First, measure the L01 flattener's slowdown instead of inferring it. If sharing the configuration-set design brings it
  under 60 s, it stays per push and needs no record. Moving a frozen file to nightly is the last resort.
- Anything still over 60 s in the CI image after B goes nightly, under the S9 rule, recorded in 0014 with its measured
  time.
- To cover A's coverage gap, confirm that a per-push check still compares the generated product parameters against
  their generator inputs (hash). Then a design-tool change that moves a gain is still caught on push, even though the
  tool's own tests run nightly. If no such check exists, tell me before moving anything.
- The nightly job runs before every stage close and every tag, as already agreed.
- Record the files at 55–60 s with their margins, so the next slowdown is expected rather than a surprise.

**2. 0016, the plugin's first gyro read returns the starting rates: approved.** This is a harness fix of the kind owner
decision 24 prefers: real hardware never reads 0 rad/s from a tumbling body.
- Show every existing scenario bit-identical (the L4/L5 replays unchanged), or name each one that changes and why.
- Add a T1 test that the first read equals the initial state's rates, with the old behaviour (0) as its negative
  control.
- Report R2's excess with the fix. R2 stays a strict xfail, so if it now passes, stop and report.
- Make it its own commit after commit 3, not part of the atomic switch.

**3. 0015: approved as built, on this checklist.** Confirm each point in one line in 0015; if any differs, show me.
- The c\* rule (hover, else the smallest feasible collective, with the card's minimum thrust as the floor).
- The torque includes everything the plant applies.
- The T1 ω̇ = 0 check, with hover rotors failing it.
- E measured on both setups.
- Envelope A, with the perfect-FF counterfactual committed.
- The crash case kept as a run-and-report, with "no pass bar until L8" in the handoff.
- The strict xfail and its gating unchanged.

**4. Your lead rulings.**

Accepted:
- Refused samples never enter the chain.
- The chain restarts with the rate loop.
- Attitude timing at the worst case over all configurations.
- No-crossover runs excluded from the plateau. That is 0006 decision 20 already.
- R1X's −8 % recorded.
- R1X now asserts the DShot range.
- The recovery end moved to 20292 by its own rule.
- The L4 T3 reference generated with notches bypassed, accepted on one condition: QF-3 (PM and Ms) and the stability
  check are asserted separately over every chain configuration. Then the golden is only the T4 comparison reference,
  not the proof that the chain is stable.

Sent back. I need these before the commit:
- **Gyro-chain config refuses only divisor 0.** Loosening a product check because frozen tests broke is the pattern we
  don't accept. Send me the stricter rule, the configurations it refused and the frozen tests that build them. If
  those configurations can't occur in flight, the stricter rule moves to the generator/param lint with a negative
  control, and the firmware check stays at least as strict as today's. If they can occur, the stricter rule was wrong,
  and 0014 should say why.
- **"The chirp check now uses a different reference."** Which reference did it change from and to, and why? Two
  lines.
- **"A shared notch speed for all motors."** If the firmware drives all 12 notches from one speed, I don't accept it:
  per-rotor tracking is the design (QF-7), and in acro the rotors differ widely. If it's a design-model simplification,
  show that the per-motor spread is covered by the configuration set. Say which it is.

**5. Stage (d), record 0017 (0016 is R2's harness fix).**
1. **Scheme: approved.** A per-motor carry, with u clamped to the command range before rounding and the carry taken from the clamped value. The saturated remainder is never carried, so there is no windup at the limits. The existing L3 saturation flag still reports the clamp.
2. **Live in its own atomic commit in (d): yes.** It doesn't touch the xfail gate. If it makes a strict xfail pass, stop and report.
3. **Pass bar: yes, with two changes.**
   - The bound holds for any in-range request sequence, not only a constant one, because the sum of errors telescopes to the carry difference. So test random and adversarial sequences, and the saturation case separately.
   - The 1.2e-4 term must be derived (a float32 accumulation bound, like P6's rounding cap), not measured. Use the measured value only as evidence that the bound isn't loose by orders of magnitude.
   - Motor-speed line: yes, if it is a derived bound (hover rotor-speed ripple from ±½-step dither through the card's motor lag) with the diffuser-off case as its negative control. That limit cycle is why this stage exists. Send me the text to approve.
4. **Reset rule: approved.** Name the disarm and motor-stop paths explicitly as non-diffuser writes, and give each reset path its own T1 test.
5. **Q: same definition, recomputed with the diffuser in the loop.** It should shrink. Re-confirm that the fine controls still fail (decision 24).
6. **D-path budget unit: unchanged.** The DShot step's size doesn't change.
7. **AM32 per-frame behaviour: yes, on the pre-L8 bench list.** The diffuser's benefit assumes the ESC applies each frame. That stays INFERRED until the bench shows it.

**Order:**
1. Answer the three sent-back rulings.
2. S9 (B, then A).
3. Commit 3, with no tag.
4. Core CI on that exact commit.
5. Push on my go-ahead.
6. 0016 as its own commit.
7. Stage (d)."

## The fourth round's sent-back rulings: the facts (2026-10-03, for Luis)

- **The divisor rule.**
  - The stricter rule is 0013's: `GyroChainConfig::validate` refuses `rate_divisor < 2`. It was pushed at 1159d5c and
    still in place at 69c62f2. Its frozen T1 assertion is `tests/regression/quad/L06/gyro_chain/fault_test.cpp:204`
    (divisor 1 → `ConfigError::Divisor`).
  - Commit 3 relaxed it to `< 1` and edited that assertion to expect `None`. That edit loosens a frozen test.
  - The configuration it refuses is divisor 1.
  - The frozen tests that build divisor 1 are SIL schedule tests that fly a divisor-1 composition through the
    parameter interface:
    - `L4CompositionWrites.NegativeControlDivisorOneChangesTheCommandOnOddTicks`
      (`tests/regression/quad/L04/unit/composition/composition_test.cpp:276-289`);
    - `L5CompositionSchedule.DivisorOneRatioFourOnTheAttitudeTick` and
      `L5CompositionSchedule.DivisorOneRatioFourThreeTicksAfterTheAttitudeTick` (L5 composition test).
    With commit 3's wiring, the chain is validated at composition init, so divisor 1 panics.
  - **In flight:** no flight build has a runtime parameter write. The only writes are the SIL harness's overrides.
    The product's `rate_loop_divisor` is 2 (`design/scenario_values.yaml`). The generator refuses D < 2
    (`tools/card/gyro_chain_design.py:87-89`), tested by `test_cutoff_refuses_a_divisor_below_two`. So divisor 1
    cannot occur in flight today.
  - **The chain's real precondition** is the low-pass cutoff below the rate loop's Nyquist, f_c < f_s/(2D). Today's
    check is only f_c < f_s/2. The product cutoff, 625.42 Hz, against f_s = 6400 Hz: f_s/(2D) = 3200, 1600 and
    1067 Hz at D = 1, 2 and 3.
  - Status: the options are with Luis.
- **The chirp reference.**
  - It changed from `tools/card/rate.py`'s PI design model (`run_l5.chirp_design`) to `tools/card/rate_lead.py`'s
    model of the flown T4 (c) configuration (latency 0, notches bypassed, D and low-pass; `run_l5.chirp_design_lead`).
  - The reason: from stage (c) the flown law is PI × lead with the chain's low-pass, and its attitude gain comes from
    `attitude_lead` on that loop. The PI model no longer describes the law the chirp measures. The PI path is kept
    bit-identical as the reference for earlier records.
- **The shared notch speed.**
  - It is a design-model simplification. The firmware tracks each rotor (`fw/gyro_chain/include/marv/gyro_chain/
    gyro_chain.hpp:202-220`, fed by `hal_rotor_speed()`'s per-motor sample).
  - Evidence (scratch, to be committed as a test):
    - Every multiset of four per-motor speeds was evaluated: the set's level-3 grid plus bypassed, 715 multisets,
      at latency 1 and 0. Each ran on three axes, at nominal and the four corners.
    - Rate loop: worst PM 45.0000407° and Ms 1.9999432, the set's own worst (4 × ω_th, latency 1). No mix is worse.
    - Attitude loop: worst PM 45.0000114°, the set's own worst. No mix is worse.
    - Negative control: one motor's notches forced active at 0.9 ω_th breaks both floors (PM 44.560°, Ms 2.0267).
  - The prose premise above (phase monotone in each notch's speed) is now checked numerically for this card, for
    magnitude too. Below each notch's centre, phase and |H| are non-decreasing in its speed. Crossovers (7.0–31.6
    rad/s) and the Ms peaks (27.4–61.4 rad/s) lie below ω_th = 348 rad/s.
- **The condition on the L4 T3 golden** (QF-3 and stability asserted over every configuration). Two pieces were not
  asserted. Both pass (scratch evidence):
  - **Attitude Ms over the set:** worst 1.9150 (bypassed, latency 0, roll J+ τ+) against Ms_max 2.0. Control: Ms
    reaches 2.0 at k × 1.0742.
  - **Rate-loop closed-loop stability over the set:** `Model.margins` finds |L| = 1 crossings only (no encirclement
    or closed-loop poles). The Stein certificate on the lifted state space, design model and firmware 2-periodic
    monodromy, certifies all 120 loops (8 configurations × 15); the largest spectral radius is 0.99925. Gain margin
    is at least 2.594. Control: the radius is below 1 at gains × GM/1.1 and above it at × 1.1·GM.
- **The per-push pin of the generated gains:**
  - The gains are generated at build time and not committed (`fw/params/CMakeLists.txt`).
  - The one per-push check that ties the live design to a committed fixture is in
    `tests/regression/quad/L06/tools/test_attitude_lead.py:265-270`. It compares the L5 T3 fixture's gains, T_f,
    att_kp and t_cross with the live `rate_lead`/`attitude_lead`, and only for L5. It sits in one of the files over
    60 s.
  - No independent pin of the product parameter set exists. Per Luis, nothing moves to nightly before he rules.

## Owner decisions, fifth round: the sent-back rulings (Luis, 2026-10-03, verbatim)

"Thanks for owning the fault_test.cpp edit. The answers to the three questions I sent back are clear, and the
shared-notch evidence (715 mixes plus a control) is exactly what I wanted. Rulings:

**1. Divisor: (b), the cutoff below the rate loop's Nyquist, f_c < f_s/(2D), plus D ≥ 1.**

Why this isn't the loosening I sent back: 0013's "below 2" came from the generator's cutoff rule, which sets the
low-pass gain to a_min at the rate loop's Nyquist f_r/2 (0013, the design, "Low-pass"). It was the domain of that rule,
not a firmware requirement. Your check carries the physics of that same rule into the firmware. It is stricter at every
D ≥ 2, which is every schedule the generator can produce. The three frozen divisor-1 SIL tests show D = 1 is a schedule
the compositions legitimately fly.

Conditions:
- **Make the frozen edit to `L06/gyro_chain/fault_test.cpp` net-stricter**, not just flipped:
  - D = 1 with a valid cutoff is accepted; this is the flipped line.
  - D = 0 is still refused.
  - **New:** the smallest D at which the product cutoff reaches f_s/(2D) is refused. That is the case the old rule
    accepted and the new one catches, so it is the control showing (b) is stricter where it matters.
  - **New:** the same cutoff at D − 1 is accepted, to bracket the boundary.
- The generator keeps its D ≥ 2 rule and its test.
- Record it in 0014 with my approval quoted. Say plainly that one frozen case was relaxed, why (generator domain, not a
  firmware requirement), and what replaced it.

**2. The chirp reference change: accepted.** The reference must be the law Gazebo actually flies. One check: add (or
point me to) a test that rate_lead with the lead off (T_f = 0, N = 1) reproduces rate.py's chirp reference exactly. Then
the pre-stage-(c) references are provably the special case and nothing was lost in the switch.

**3. The shared notch speed: accepted.** It's a design-model simplification, and the firmware tracks each rotor. Record
the 715-mix evaluation and its control in 0014.

**4. The three new tests: yes, all three in commit 3.** That's the attitude Ms over the set, the rate-loop stability
certificate over the 120 loops, and the per-motor mixes in their own file (about 28 s). Each needs its control, as you
described.

**5. Parameter pin: yes. Commit the generated product parameter table itself, not only its hash.**
- It's small and human-readable, so a gain change shows up as a diff in review. 0011's hashes were for large reference
  data, which this isn't.
- Core compares the build's generated table byte for byte with the committed one on every push, with a planted-change
  control. The generation already runs in the build, so the check is nearly free.
- The file's header names its generator and inputs. It is a derived copy, checked against its source, never edited by
  hand.
- Put it under `tests/regression/quad/L06/`, so it's frozen. Any change to flight gains then comes with a decision
  record in the same change, which is the right bar for a flight product.
- This must exist before anything moves to nightly.

**6. S9: apply the two output-identical speed-ups, then move to nightly whatever is still over 60 s. No design cache.**
- A cache keyed on inputs and tool source can serve a stale design and let a test pass on yesterday's answer. With the
  pin in place, the nightly tool tests lose no product coverage: any change in output is caught on push. So the cache
  isn't worth the risk.
- If the frozen flattener test drops under 60 s after the speed-ups, it stays per push. If not, it moves to nightly with
  its own frozen-change entry in 0014.
- The files at 59–60 s stay per push. Record their margins.
- On your correction (nothing enforces a per-file limit on pytest): keep measuring per file, because a file is the unit
  a contributor reruns. Log in the handoff that S9 is unenforced for pytest, as an item to close at the next CI change.
  It isn't part of commit 3.

**Order:**
1. Divisor (b) and the fault_test edit.
2. The three tests.
3. The parameter pin.
4. The speed-ups, then the nightly moves.
5. Full local CI.
6. Commit 3, with no tag.
7. Core CI on that exact commit.
8. Push on my go-ahead.
9. Then 0016, then stage (d) as already ruled."

## Divisor (b), as built (fifth round, item 1; frozen-test change)

Approval, Luis, 2026-10-03 (verbatim, from the fifth round above): "Divisor: (b), the cutoff below the rate loop's
Nyquist, f_c < f_s/(2D), plus D ≥ 1. … Record it in 0014 with my approval quoted. Say plainly that one frozen case was
relaxed, why (generator domain, not a firmware requirement), and what replaced it."

- **The firmware rule** (`fw/gyro_chain/include/marv/gyro_chain/gyro_chain.hpp`, `validate`):
  - D < 1 is refused (Divisor);
  - otherwise the cutoff is refused unless 0 < f_c and 2·f_c·period·D < 1, i.e. f_c < f_s/(2D) (Cutoff). The test is
    multiplicative and negated, so NaN is refused and nothing divides by D.
  - `fw/rate_group/src/rate_group.cpp`'s refusal message states the new bound.
  - The generator keeps D ≥ 2 and its test (`tools/card/gyro_chain_design.py`, `test_cutoff_refuses_a_divisor_below_two`).
- **One frozen case was relaxed: divisor 1 is accepted** (`tests/regression/quad/L06/gyro_chain/fault_test.cpp`, the
  divisor-1 line, Divisor → None).
  - Why: 0013's D ≥ 2 was the domain of the generator's low-pass rule (a_min at the rate loop's Nyquist f_r/2), not a
    firmware requirement.
  - The three frozen divisor-1 SIL schedule tests show that D = 1 is a schedule the compositions fly
    (`L4CompositionWrites.NegativeControlDivisorOneChangesTheCommandOnOddTicks`,
    `L5CompositionSchedule.DivisorOneRatioFour{OnTheAttitudeTick, ThreeTicksAfterTheAttitudeTick}`).
- **What replaced it,** in the same frozen file:
  - D = 0 is still refused.
  - **D\*** is computed in the test from the fixture's product cutoff (625.4 Hz at f_s 6400 Hz): it is the smallest D at
    which the cutoff reaches f_s/(2D), D\* = 6, and it is refused. The old rule accepted it.
  - D\* − 1 = 5 is accepted, bracketing the boundary.
- **A consequential frozen line** (lead decision, not named in the ruling; for Luis's review). The old rule's
  upper-edge acceptance, cutoff 0.99·f_s/2 at the fixture's D = 2 → None, contradicts (b) by design.
  - It becomes the same edge under (b), 0.99·f_s/(2D) → None.
  - The old value, 0.99·f_s/2 at D = 2, is now asserted refused (Cutoff).
  - f_s/2 → Cutoff is kept.
- The rule is stricter than the old one at every D ≥ 2, and so at every schedule the generator produces.
- No composition, test or oracle uses a cutoff that (b) now refuses: they all use the product cutoff at D = 2.
- The frozen T3 oracles keep the old f_s/2 check (`rate_t3_oracle.py:340`, `attitude_t3_oracle.py:1699`). It is
  consistent with (b) at D = 2. Not changed.
- Checks: host-debug and host-release ctest 642/642; the m33 build is clean. G1 runs in full local CI.

## Fifth round, items 2–5, as built

- **The three new tests** (item 4; in `tests/regression/quad/L06/tools/`, all added by stage (c)):
  - **Attitude Ms over the set** (`test_attitude_lead.py`,
    `test_ms_is_within_ms_max_over_the_configuration_set_and_k_times_1_1_exceeds_it`):
    - Ms ≤ Ms_max on all 120 loops of the level L+1 set. The worst is 1.9150393 (bypassed, latency 0, roll J+ τ+)
      against 2.0.
    - The helper |1/(1 + k·F/(z−1))| is tied to `Loop.margin` through Ms ≥ 1/(2 sin(PM/2)).
    - Control: k × 1.1 gives 2.0301.
  - **Rate-loop stability over the set** (`test_rate_lead.py`,
    `test_every_rate_loop_of_the_set_is_certified_stable_in_the_design_and_the_firmware_model`):
    - The Stein certificate (attitude_lead step 5', its own certified bound, no added tolerance) holds and ρ < 1 on
      all 120 loops. This covers the design model and the firmware 2-periodic monodromy at expf −2, 0 and +2 ulp.
      The largest ρ is 0.9992518.
    - Control: at gains × GM/1.1 every loop is certified; at 1.1·GM none is. The minimum GM is 2.5937.
  - **Per-motor mixes** (`test_notch_mixes.py`, 7 tests): every multiset of {3 level-1 grid speeds, bypassed} at both
    latencies, 70 configurations.
    - The equal-speed mixes reproduce the set bit for bit, so the comparisons are exact.
    - Rate: PM ≥ and Ms ≤ the set's worst, which is the minimum over the mixes exactly (45.0000407°, 1.9999432).
    - Attitude: PM ≥ the design's PM_worst.
    - Rate control: one motor's notches forced active at 0.9 ω_th give PM 44.560° and Ms 2.0267, failing the set, PM_min
      and Ms_max.
    - Attitude control: k × 1.1 on the binding mix. The forced notch does not bind the attitude loop, which is bound by
      bypassed at latency 0.
    - Premise: monotonicity below each notch centre (control: it fails just above each centre). Every crossover and Ms
      peak lies below ω_th (control: gains × 1000).
- **The 715-mix evaluation** (item 3): recorded in "The fourth round's sent-back rulings: the facts", above. The level-1
  subset is the per-push test.
- **The chirp special case** (item 2): not exact. With the lead off (N = 1, no chain, latency 0):
  - rate_lead's f32 kp and ki are bit-identical to rate.py's, and kd = 0. Existing tests already pin this:
    `test_rate_lead.py::test_n1_without_chain_or_latency_reproduces_rate_py` and
    `test_attitude_lead.py::test_todays_pi_through_this_rule_reproduces_attitude_py`.
  - The chirp reference's numbers agree only to rounding:
    - per-loop PM to 1.4e-13 rad;
    - crossover angle to 1.1e-16 rad;
    - band edge at the 14th digit;
    - G_τ to 1.8e-13 relative.
  - The cause: `attitude.Loop.response` solves a 4×4 complex system, while `LeadLoop.resp` evaluates rule step 2''s
    closed form. It is the same function with different rounding.
  - Status: Luis chose (i) (sixth round); the test is built there ("(i), as built").
- **The parameter pin** (item 5):
  - The table is `tests/regression/quad/L06/product_params/marv_params_product_table.txt`, with a README. It holds 93
    records in id order. f32 values are printed with `%.9g` and as hexadecimal floats, exact to the bit.
  - The header names the generator (`tools/gen/params_gen.py --out-table`, run by `fw/params/CMakeLists.txt`) and its
    repository-relative inputs.
  - The marv-ci image, the host and the m33 build generate identical bytes.
  - The check is in `ci/run_ci.sh` (`product_table_*`). It compares byte for byte after the host-debug build and after
    the m33 build. The planted-change control alters the mass value in a copy, and the comparison must fail.
  - The generator gains `--out-table` / `--table-input`; `marv_add_param_set` gains `TABLE` / `TABLE_INPUTS` and
    `marv_add_card_param_set` gains `PRODUCT_TABLE`. Only `marv_params` uses them.

## Fifth round, item 6 (S9), as built

- **The two output-identical speed-ups:**
  - `tools/card/rate_lead.py` evaluates the f32 guard step-down candidates ahead on the process pool and decides in
    order on the first that passes (`first_passing_guard`; `procs <= 1` is the old serial loop).
  - `tools/card/attitude_lead.py` runs the step-1'-only design through the same pool path as the set design
    (`set_sup_rule`): 23.0 s → 6.7 s on the host.
  - Evidence: the generated product table equals the committed one and the pre-edit one. All 11 flatten outputs are
    byte-identical before and after. The full result dicts of `rate_lead.design` and of the step-1'-only design are
    identical with default procs and with procs = 1, except the loop class name.
  - Ten cached internal Stein margins of yaw loops differ, and the verdicts are the same. They are not outputs.
- **Per-file wall time alone** (marv-ci, 4 CPUs, host load < 1.5), with the nightly list in `ci/run_ci.sh`
  (`nightly_tools_tests`):
  - **Per push, closest to the limit:** `L06/tools/test_r1x_coupling.py` 58.9 s (margin 1.1 s) and
    `L06/tools/test_r2_lower_bound.py` 56.2 s (margin 3.8 s). Then `test_rate_lead.py` 43.8 s and
    `test_l6_ff_eval.py` 39.7 s.
  - **Nightly:** `L06/tools/test_chirp_admission.py` 111.7 s, `L06/tools/test_notch_mixes.py` 92.4 s and
    `L06/tools/test_attitude_lead.py` 81.4 s.
  - **Nightly, frozen (CI selection change only; each file is unedited):**
    - `tests/regression/quad/L01/tools/test_flatten_report.py`: 64.5 s and 65.1 s, over 60 s after the speed-ups.
    - `tests/regression/quad/L05/tools/test_attitude_t3.py`: 60.5–61.0 s in four runs. The fourth round recorded
      it at 59.6 s, so it now sits just over the limit (lead reading of the > 60 s rule, for Luis's review).
  - Per push, these files are `--ignore`d. `MARV_CI_MODE=full` (nightly and workflow_dispatch, `.github/workflows/ci.yml`)
    runs them with the rest. Collected: 1014 per push, plus 60 nightly-only, 1074 in all.
  - Product coverage on push comes from the parameter pin (item 5). The L5 T3 fixture's live-design check
    (`test_attitude_lead.py`) now runs nightly, before every stage close and every tag.

## Owner decisions, sixth round: the last open items of the close (Luis, 2026-10-03, verbatim)

"**Chirp special case: (i).** Assert the lead-off gains bit-identical, and the chirp reference against rate.py within
the existing equivalence convention, 2⁻³⁰. The 1e-13 observed is far inside it, and a second code path isn't worth it.

**test_attitude_t3.py to nightly: accepted,** by the "> 60 s" rule at 60.5–61.0 s.

**The extra divisor frozen line: accepted.** It is the stricter boundary case I asked for."

**(i), as built** (`tests/regression/quad/L06/tools/test_chirp_lead_off.py`, per push, 2.5 s alone).
- The law: rate_lead at N = 1, no chain, latency 0 gives f32 kp and ki bit-identical to rate.py, with kd = 0.
- The reference: `run_l5.t4_lead_reference`'s quantities on that law (T_f = 0) against `run_l5.chirp_design`'s, within
  2⁻³⁰. Measured: PM 1.41e-13 rad, crossover 8.2e-14 relative, band edge 6.8e-14 relative, G_τ 1.80e-13 relative.
- Control (lead decision): the lead at its design values (rate_lead's product gains, kd ≠ 0, T_f > 0; no chain,
  latency 0) breaks it: PM 0.174 rad, G_τ 0.32 relative. T_f alone cannot be the control. With kd = 0, LeadLoop never
  reads T_f, and the design T_f with kd = 0 gives the same deviations as T_f = 0, digit for digit.

## Stage (c): the build

**c1, the inert firmware pieces (commit 1).**
- **`fw/rate`.** `RateConfig` gains `d_filter_tau` (T_f), `inertia` (J), `motor_tau` (τ_m) and `ff_filter_tau` (T_ff),
  all defaulting to 0, and `ConfigError` gains `DFilter` and `Feedforward` (appended). `from_params()` leaves the new
  fields at 0.
  - **D low-pass:** Df_n = Df_(n−1) + (1 − e^(−dt/T_f))(D_n − Df_(n−1)), behind a branch, so T_f = 0 runs today's exact
    expression.
  - **Feed-forward:** u += g + τ_m·ġ, with g = ω×(Jω) on the loop's measured y and ġ the backward difference through a
    T_ff filter. It is skipped entirely when all J = 0. Sign: Euler, J ω̇ = τ − ω×(Jω), so holding ω takes
    τ = +ω×(Jω); the lag inversion u_req = u + τ_m·u̇ is derived inline.
- **`fw/gyro_chain`.**
  - **Input guard:** a non-finite input holds that axis's last output (+0 before any), keeps its states, and sets a
    per-axis flag and a saturating counter. The next finite sample reseeds the axis to its steady state.
  - **Seeding:** x = y = c is a fixed point because Σb = 1 + a1 + a2 for the notch and the low-pass. First-sample seeding
    sits behind `GyroChainConfig::seed_first_sample`, default false. The frozen `gyro_chain/fault_test.cpp` compares
    against a zero-state reference, so seeding by default would break it; commit 3 sets the flag true in the wiring (lead
    decision).
- **Tests** (`tests/regression/quad/L06/d_term/`): 21. They check:
  - bit-identity to today's law over 3000 ticks with the new parameters at their defaults;
  - the D-filter closed form;
  - the FF known answer (ω = (1, −2, 3), J = (½, ¼, ⅛) → (¾, 9⁄8, ½), exact) and the lag term against a double
    reference;
  - the chain's seeding and guard.

  Negative controls: T_f and τ_m ×1.1 and ÷1.1, permuted J, a flipped sign, T_ff ±10 %, an unseeded chain, and an
  unguarded chain.
- **Checks:** ctest 631/631 in debug and release; m33 with zero warnings; G1 (marv-ci image), the constants check and G3
  are clean. Reviewer: PASS, confirming bit-identity both from the diff and by test, and the FF sign. The two formula
  citations were tightened (Goldstein §5.5 without an unverified equation number; the lag identity derived inline).

**c2, the PI × lead gain rule as a tool** (`tools/card/rate_lead.py`; tests in
`tests/regression/quad/L06/tools/test_rate_lead.py`). Not yet wired into the product parameters (commit 3).
- **The design:**
  - N* 3.877 (Ms binds); nominal ω_c 12.129 rad/s.
  - Worst PM 45.000002° at (J−, τ+); worst Ms 1.99995 at (J−, τ−).
  - T_f 41.87 ms; T_ff 12.31 ms.
  - Combined D + FF RMS at rate_max equals the budget (2.281 mN); at hover the D path is 0.494 mN.
  - t63 at the J+ corner 0.151 s.
  - 12 physical J vertices.
- **Cross-check:** with N = 1, H = 1 and latency 0 it reproduces `rate.py`'s PI to 2.4e-14. A time-domain Monte Carlo
  matches the noise integral to 0.3 %.
- **Rate_max noise operating point (lead decision).** At ω = rate_max on all axes the steady-tumble rotor speeds are
  undefined (one motor would need −0.301 N). So the noise uses the notch-free chain, an upper bound because every notch
  has |H| ≤ 1. That gives T_ff 12.3 ms; the hover-notch convention would give 1.8 ms.
- A guard fix is in progress (α in f32, as the firmware computes it).

**c3, the R2 lower bound** (owner decision 5, condition 1; `tests/regression/quad/L06/results/r2_lower_bound/`,
pytest `tests/regression/quad/L06/tools/test_r2_lower_bound.py`).
- **Proven, given the plant model.** From R2's start (ω0 = rate_max on all axes, hover rotors, zero torque), every
  command history in [0, W_max] forces the truth rate past the design envelope:
  - ω_y by at least 0.28283 rad/s at execution 16;
  - ω_x by at least 0.22176 at execution 13.

  R2's tolerances there are F + Q = 0.1151 (ω_y) and 0.0895 (ω_x); ω_z is not forced.
- **Not proven:** that the predicate fails. The predicate's tolerance is E + F + Q, and E (the m = 1 vs m = 2 run
  difference) is not recorded at execution 16. The check could still pass only if E(16) ≥ 0.1677 rad/s on ω_y.
- **Negative controls:** equal moments, and steady-tumble rotors at the minimal feasible collective. Both collapse the
  bound to 0.
- **Cross-check:** the committed R2 trace's excess is at or above the bound (ω_y 0.563 at execution 16).
- **Independent review: PASS** (2026-10-01). No non-conservative step; the coupling was recomputed by hand; the raw
  output reproduces byte for byte.
- **Condition 2 as worded is infeasible.** Hover collective plus the mixer inverse of ω0×Jω0 needs motor 1 at
  −0.30146 N. The smallest collective that keeps every rotor at or above its minimum is 8.917 N (118 % of hover).
  Question to Luis, pending.

**c2, follow-ups.**
- **f32 guard on the firmware's exact run-time coefficients.** It computes α in float, with dt = float(dt_us)/1e6, and
  analyses the loop as 2-periodic (312/313 µs) by a lifted harmonic transfer, which reduces exactly to the LTI loop at
  equal dt. It checks `expf` at 0 and ±2 ulp (INFERRED bound).
  - Result after one step-down: N* 3.877369, ω_c 12.129191 rad/s, guard PM 45.000042°, Ms 1.999943, T_ff 12.3116 ms.
- **Cross-check tests.** Each has a control that fails by a wide margin.
  - The equal-dt reduction.
  - The tool's controller against the compiled firmware `RateLoop<float>`: within 0.10 of the f32 error bound, with α
    bit for bit.
  - The coherent D + FF noise formula against a 120k-execution seeded brute-force simulation: within 0.04–0.24 %
    against a 4σ bound of 1.1 %.
- **Parallel N search** (`--procs`, the `attitude_t3_oracle.py` pattern). Byte-identical to the serial run for every
  process count. The design takes about 30 s on 4 cores, down from 56 s serial.
- Reviewer: PASS. The loop definition (D on the measurement) and the periodic analysis were re-derived.

**c4, the attitude gains on the new rate loop (D7)** (`tools/card/attitude_lead.py`; `attitude.py` refactor; tests
`tests/regression/quad/L06/tools/{conftest.py,test_attitude_lead.py}`).
- **The `attitude.py` refactor** (lead-approved, minimal): `Loop._evaluate_grid`, `Loop.radius`, `Loop.release_crossing`
  and `yaw_terms`. The PI path is byte-identical: the attitude and rate yaml/report files compare equal against HEAD's
  `attitude.py`, and the L05 tools pass 149. Only the order of the `alpha_min` refusal changes, on the refusal path.
- **The design on the stage (c) inner loop** (rate_lead's N* gains, the chain at ω_th, 1-sample latency):
  - k = 3.2277713 (today 3.0872879);
  - PM nominal 61.268°, worst 45.000016° at yaw (J+, τ+);
  - crossover 3.837–4.376 rad/s; t_cross 0.274375 s;
  - w and α_min unchanged; att_loop_ratio 1.
- **Stability (0006 E step 5) for the 34-state loop.** The Jury table fails numerically at that order, and a forward
  error bound on repeated squaring stalls above 1. So stability is proved by a Lyapunov certificate:
  - P from Smith's doubling;
  - rigorous Gershgorin lower bounds with Higham rounding bounds on λ_min(P − AᵀPA) and on λ_min(TᵀPT), with T = L⁻ᵀ
    from P's Cholesky factor (an invertible congruence);
  - both positive ⇒ ρ(A) < 1 for the double matrix with f32 gains. That is the same scope as the Jury check had for the
    PI loop.

  All 15 loops are certified (margins 0.93–1.0). The lead added a non-finite guard (a NaN row fails the certificate).
- **Checks:**
  - Cross-check: today's PI reproduces `attitude.py` within 2⁻³⁰.
  - The squaring radius agrees with Jury on the PI loop (≤ 1.4e-6).
  - Controls: latency +1; gains 10 % above the stability limit are not certified, 10 % below are. In an independent
    stress test the certificate never passed an unstable matrix (300 random matrices).
  - Reviewer: PASS on soundness.

**c5, the ω×Jω evaluation, L4 acro** (`tools/sim/l6_ff_eval.py`; results and README in
`tests/regression/quad/L06/results/ff_eval/`; pytest `tests/regression/quad/L06/tools/test_l6_ff_eval.py`).
- **The model** reproduces the committed acro gz cause analysis for today's PI: 1378/1036/352 executions outside against
  1377/1037/350, and trace residuals ≤ 0.05 % of peak. Without ω×Jω it gives 0 outside.
- **Lag-compensated FF at the card plant**, by sensor configuration (reported, not asserted; owner decision 2):

  | Sensor configuration | Result |
  | --- | --- |
  | flight, as stage (e) flies: notches tracking the rotor-speed telemetry | PASS by 24.1 mrad/s (0 outside) |
  | stage (c) T4: truth gyro, notches bypassed | PASS by 3.2 mrad/s |
  | no chain, no latency | FAIL by 1.3 mrad/s |
  | T3 worst case: notches pinned at ω_th | FAIL by 58.1 mrad/s |

- PID alone and plain FF fail in every configuration.
- **Physical J corners (owner decisions 3 and 4):** no variant passes at any of the 12 corners. The worst is corner 11,
  J/J0 = (1.5, 1.5, 0.5): lag-compensated FF exceeds by 9.46 rad/s on pitch, and FF worsens some corners against PID.
  This is carried to the pre-L8 gate (measured J).
- **UNKNOWNs:** E is not modelled (E = 0, stricter); τ ±30 % is not swept at the corners. The acro test's
  `script_response` and oracle are PI-only and are extended in commit 3. tau_ref uses `rate.py` rule step 9 with
  rate_lead's τ_cl (lead decision).

- **c3/c6 at the final stage (c) gains** (att_kp 3.1539721, window 0..20292; W3b, each artefact regenerated by its own
    script, the byte-for-byte pytests pass). The proof is stronger:
    - X* against F + Q: ω_y 0.28537 at n 17 against 0.031128 (9.17×); ω_x 0.21596 against 0.032875 (6.57×); ω_z now
      forced, 0.039504 against 0.0079155 (4.99×).
    - Measured E at n*: 5.33e-3 / 1.75e-3 / 2.54e-3 rad/s, against 0.254 / 0.183 / 0.0316 needed. The largest E over the
      whole window is below the needed value on every axis. The stop condition is not met.
    - Envelope A still holds to rounding (C = D(v0) to 5e-14). B is left at n = 1, and the no-FF control leaves A.
    - The hover tumble: 20461 violations (reported).
    - The numbers above are the earlier-gain record.

**c6, R2's E measured** (Luis's second round, item 2, step 1 of the order). Files:
`tests/regression/quad/L06/results/r2_lower_bound/{measure_e.py, e_measured.txt, e_measured.md}`, pytest
`tests/regression/quad/L06/tools/test_r2_e_measured.py`.
- R2's own two runs (m = 1 and m = 2, today's setup and gains) go through the test's own code (`fly`, `make_design`,
  `series`). The script refuses unless its maximum E equals `evaluate`'s `max_E`.
- **Results:**
  - E(16) on ω_y = 4.7112e-4 rad/s, against the 0.16773 the predicate would need: about 1/356.
  - E(13) on ω_x = 3.8624e-4, against 0.13227 needed.
  - The forced excess beats the measured E at executions 4–29 (ω_y) and 3–22 (ω_x). The test's own `evaluate` first
    fails at 4 and 3.
  - Largest E over the whole window: 0.0117 rad/s.
- **The stop condition is not met.** With E measured, the infeasibility of R2 as specified is proven, not conditional.
- **Determinism:** two host runs are byte-identical, and two runs in marv-ci-gz (fresh clone, `--cpus 4`, the runner's
  memory limit) are identical to them.
- The README is a separate file (`e_measured.md`) because editing the proof's README would be a frozen edit once
  committed.

**Commit 3, the atomic switch (in progress; nothing committed until every part is green).**
- **3a, the product parameters.** `flatten.py --out-rate-lead/--out-attitude-lead` (`rate.py`/`attitude.py` stay
  available as the L4/L5 PI reference). `fw/params` switches the product and the L4/L5 composition sets together.
  `RateConfig::from_params` reads `rate_d_filter_tau_*`. The new ids are claimed in `tests/regression/quad/L06/param_ids`.
  - Values: kp/ki/kd roll 0.024631018/0.077088505/0.0014596869, pitch 0.020690056/0.064754345/0.0012261370, yaw
    0.042365350/0.132592216/0.0025106615; T_f 0.041869674 s; τ_ref 0.150625 s (rule step 9 with rate_lead's τ_cl);
    att_kp 3.227771282.
  - Byte-identical across process counts and in marv-ci.
  - No J/τ_m/T_ff parameters yet (stage (e)).
- **3b, the gyro chain wired in.** `fw/rate_group` (`RateGroupStep`) is used by both compositions, `l4_acro_replay` and
  the three L05 results tools.
  - Per tick it runs `chain.filter`; on a rate tick, `update_notches(hal_rotor_speed())` comes first. The rate loop runs
    on that tick's chain output, and `seed_first_sample = true`.
  - Lead rulings:
    1. **Fault semantics unchanged (safety).** A sample the rate loop would refuse (GyroValid clear or a non-finite
       axis) never enters the chain. The rate loop receives the raw sample, so its fault path fires exactly as at
       `69c62f2`. This is proved against a test-local HEAD reproduction at every execution (`L6RateGroupFault`), with
       controls.
    2. **Divisor** (superseded by the fifth round, item 1; see "Divisor (b), as built"). `GyroChainConfig::validate`
       refused only `rate_divisor == 0`. The chain's arithmetic does not use
       the divisor; the D ≥ 2 precondition belongs to the cutoff design rule, and the generator keeps refusing D < 2
       (`gyro_chain_design.py`). Frozen edit: `tests/regression/quad/L06/gyro_chain/fault_test.cpp:204` (divisor 1 now
       expects None; divisor 0 still refused). With it, the frozen divisor-1 schedule tests (#266, #379, #380) pass
       unedited. **For Luis's review:** this relaxes a validator assertion.
    3. **The chain restarts with the rate loop.** It reseeds only on the rate tick where the rate loop reseeds (keyed to
       `RateLoop::seeding()`). A refused sample on a non-rate tick is skipped without a reseed, so D never
       differentiates an unfiltered jump.
  - Replay parity: `l4_acro_replay` reproduces the SIL composition's DShot at every rate execution
    (`L6ReplayParity`), with a control.
  - Reviewer: PASS, safety first.
  - Frozen edits: `L04/replay/{l4_acro_replay.cpp, CMakeLists.txt}`; `L05/results/{step_cause, recovery_cause,
    step_controls}/{*_tool.cpp, build_tool.sh}`; `L06/gyro_chain/fault_test.cpp:204`; `L06/CMakeLists.txt`
    (`rate_group`).
- **3c-L4, the L4 T3 on the stage (c) path** (frozen edits: `L04/t3/{t3_test.cpp, reference/rate_t3_oracle.py,
  reference/rate_t3_inputs.txt, reference/SHA256SUMS, README.md, CMakeLists.txt}`).
  - **Harness:** the chain and rate loop are composed exactly as `RateGroupStep` does it. Notches are bypassed (T3 has
    no rotor speed), so the chain is the low-pass. The gyro feed convention is unchanged (no latency).
  - **Oracle:** extended with D (filtered), the low-pass and seven rounding nodes.
  - **Result:** float error 1.4 % of the derived tolerance. Controls: gains ×1.1 moves the golden 283×; one tick of
    delay moves it 3.8× (was 6.5×; the extra filtering softens a delay step).
  - The regenerated golden and envelope are byte-identical between the host and marv-ci.
  - **Lead note:** the T3 golden checks firmware against oracle. The ω_th notch worst case of the (b, c) QF-3 line is
    carried by the design-model margins (`rate_lead`), not by this golden.
- **3c-L5** (in progress), lead ruling: **`att_yaw_t_cross` is the latest zero crossing over the band box and over
  every chain configuration the firmware flies or is tested in.** That is notches at ω_th and all bypassed, each with
  latency 1 (flight) and 0 (the truth-gyro T4 configuration).
  - On one configuration only (0.274375 s), 47 of 289 yaw-release envelope members (notches bypassed, latency 0)
    locked by fallback before crossing (latest crossing 0.2925 s), and the frozen lock-before-fallback assertion
    failed.
  - The fallback is a backup; placing it after the latest real crossing in every configuration is the conservative
    side. No frozen assertion changes. **For Luis's review.**

- **3c-L5, the L5 T3 on the stage (c) path** (frozen edits: `L05/t3/{t3_test.cpp, CMakeLists.txt, README.md,
  reference/attitude_t3_oracle.py, reference/attitude_t3_inputs.txt, reference/SHA256SUMS}`).
  - Harness and oracle as for L4: attitude law → rate loop with filtered D → chain low-pass, notches bypassed.
  - The float coefficients' fixed offsets are bounded through the loop's signed response.
  - The `ref` node's phase under-count (2.52786e-7 against a brute-force 2.52822e-7) is fixed.
  - **t_cross by the lead's rule:** 0.292500019 s, set at (J+, τ−), notches bypassed, latency 1 and 0. The yaw-release
    envelope locks at executions 20446–20765, and the fallback is at 20766.
  - Float path: 0.37–0.76 % of the tolerance.
  - Controls (tilt / yaw): attitude gain ×1.1 ×2452 / ×70.0; rate gains ×1.1 ×1145 / ×202.6; one tick ×28.1 / ×5.1.
  - Golden, envelope and Q are byte-identical on the host and in marv-ci (220 s at `--cpus 4`).
- **3d-L4, the L4 consumers and the gz-l4 suite.**
  - **Frozen edits:**
    - `L04/tools/test_l4_acro_bound.py`: `law(p, su)` returns the stage (c) law tuple; `tick_map` replaces `plant_map`.
    - `L04/gz/test_t4_acro.py`: `design_bound` uses the stage (c) law tuple and `tick_map`; the docstring is updated.
    - `L04/gz/test_t4_steps.py:95`: `kd == 0` becomes `rate_d_filter_tau_<axis> > 0`.

    Predicates, bounds and names are unchanged.
  - **`tools/sim/run_l4.py`:**
    - `script_response`/`script_envelope` mirror the oracle's `closed_loop` operation for operation; the frozen acro-bound
      test passes, bit-identical.
    - **Lead-accepted:** the chirp's QF-3 design reference is now `rate_lead`'s model of the T4 (c) configuration
      (latency 0, notches bypassed) instead of `rate.py`'s PI. The design model is `rate_lead` from stage (c) on. The PI
      path is bit-identical, so `run_l5` is unaffected.
  - **gz-l4:** 40 passed, 1 xfailed (acro recovery, strict), 0 skipped.
    - Steps pass (worst slack −6.0e-3 rad/s against F 6.0e-3).
    - Chirp passes: PM 81.459° on each axis, PM_min 45°, inside the design range [61.09°, 81.59°]. The control (gains ×3)
      fails.
    - The acro coupled-axis check (iii) passes (smallest slack 0.036 rad/s). Acro recovery fails with roll excess
      5.23 rad/s, so the xfail holds.
  - **c5's FF evaluation** is regenerated on the new oracle; every stage (c) result is byte-identical, so the
    conclusions stand.

- **W1a, the design rules over the configuration set** (owner, third round, item 1).
  - **Rate-loop set assertion** (`rate_lead.py` step 13): PASS. The worst case over the set is the design point itself
    (notches at ω_th, latency 1): PM 45.00004°, Ms 1.99994. Every other configuration has more margin, up to bypassed
    with latency 0 at PM 61.18°, Ms 1.391.
  - **Notch grid rule:** geometric from ω_th to ω_max, with halving levels until the worst value no longer changes
    (core §7.5).
  - **Attitude k over the set** (`attitude_lead.py`): att_kp 3.1539721 (was 3.2277713). It is set by bypassed, latency
    0, roll (J+, τ+) at PM 45.00001°. Stability is certified over the set. t_cross, w, α_min and att_loop_ratio are
    unchanged.
  - **Controls:** the ω_th-only k (3.2278) puts 15 loops of the set below PM_min (worst 43.87°). Gains ×1.1 fail the rate
    assertion, and a planted sub-floor PM at bypassed/latency 0 fails it while the design point passes.
  - **Lead acceptance: one common notch speed for all four motors is sufficient.** Each notch's phase contribution at
    crossover is monotone in its own frequency, and the contributions add in a cascade. So any mix of motor speeds lies
    between the all-at-ω_th and all-bypassed extremes, and both are in the set.
- **W2, the chirp plateau admission** (owner, third round, item 2; **amends 0006 decision 20's plateau definition**).
  - Why it amends 0006: A_env rose 1.43× with the new loop (the peak torque per rad/s fell from 0.0648 to 0.0453 N·m),
    so k1 now reaches 173° of tilt. Stage (d)'s error diffusion shrinks the quantisation shift, so k5 may re-enter by
    rule with no edit.
  - **The rule:** run k is admitted iff its modelled nonlinear PM shift (full-quaternion design model with the DShot
    quantiser, minus the linear model, nominal plant, T4 (c) configuration) is smaller in magnitude than the PM change of
    the fine control (attitude gains ×1.1) on the design model.
    - Threshold 4.217°; at least 3 admitted runs (`PLATEAU_MIN_RUNS`, else the test fails); excluded runs are flown and
      printed.
    - Lead acceptance: a run whose model gives no unique crossover is excluded, because it has no shift and no margin
      to measure (roll k1).
  - **Result at the final k** (`tests/regression/quad/L05/gz/test_t4_chirp.py`, 15 passed):
    - roll k2–k5 admitted: min 57.518°, U_A 4.841, slack +7.600°;
    - pitch k2–k5 admitted: min 57.551°, U_A 4.339, slack +8.197° (k1 excluded, shift +9.20°);
    - yaw unchanged: slack +15.19°;
    - the ×2 control fails on every axis.
  - **Frozen edit:** `tests/regression/quad/L05/gz/test_t4_chirp.py`. The admission table, the floor assertion and the
    plateau restricted to admitted runs, plus the stage (c) `control_c` hunks from 3d-L5. Predicates, bounds and names
    are unchanged.

- **W1b, L5 at the final k.**
  - **L5 T3 regenerated at att_kp 3.1539721.** The fixture refresh changes one line of `attitude_t3_inputs.txt`. The
    regeneration ran in marv-ci (`--cpus 4`) and is byte-identical on the host.
    - New `SHA256SUMS`: golden `8e34a02f…`, envelope `6e7b4493…`, Q `a9b145bb…`.
    - Controls (tilt / yaw): attitude gain ×1.1 ×2464.6 / ×70.7; rate gains ×1.1 ×1137.8 / ×206.3; one tick ×28.1 /
      ×5.1. The CI control (att_kp +1 ulp) fails the hash check on all three files.
    - Float path 0.19–0.77 % of the tolerance.
    - yaw_release locks at 20910–21229 against the fallback at 21230, a margin of one execution by construction
      (t_cross is the latest crossing).
  - **Scenario counts by rule:** H 20292. step_roll, step_pitch, yaw_release and yaw_fallback start at 20293 and end at
    40584. The recovery scenarios' end goes 20744 → 20292 by their own rule (10/k ÷ T_a, ceiling, doubled once).
    20744 was never by-rule: at k 3.087 the rule gives 20732; 10372 was the N = 2 era's H. All four recovery envelopes
    end below F at 20292.
  - **The authorized frozen edit,** `tests/regression/quad/L05/tools/test_l5_scenario.py` (Luis, 2026-10-02: "The frozen
    count edit in `test_l5_scenario.py`: authorized, after k is final. … Make the edit once, with the value from the
    final k. While you're there, make the positive assertions (lines ~139 and ~230) read the segment start from the
    scenario's derived value instead of a literal, so the next regeneration needs no frozen edit. Keep line ~77's 'not
    the recorded one' control pinned to a literal on purpose, and say so in a comment. Record it in 0014 with my
    authorization quoted."):
    - lines 141–142, 173–174 and 234–235 read the segment start from the scenario file;
    - the control at 75–79 stays a literal (20294, `!= 20293`), with a comment citing this ruling.
  - **Other frozen edits:** `tests/regression/quad/L05/t3/{reference/attitude_t3_inputs.txt, reference/SHA256SUMS,
    README.md}`.
  - ctest 642/642 in debug and in release.

**Frozen file changed: `tests/regression/quad/L06/CMakeLists.txt`.** One `add_subdirectory(d_term)` line is appended.

## L5 R1X: the exact-180° α envelope predicate becomes a known failing item (frozen-test change)

Luis, 2026-10-02 (third round, item 3, verbatim): "R1X: (a), narrowed. Xfail only the envelope predicate, with strict
xfail on the same gate file. R1X's other assertions keep running as normal tests: the clean run, the DShot range and
the settled end value. That way the side check doesn't go dark until (e). Evidence first, as with R2: commit the
counterfactual showing that Gazebo's excess is the coupling. That means the design model plus ω×Jω reproduces Gazebo,
as for R2's 7.96 → 1.46e-2, and that it is identical under the old and new gains. R1X joins R2's line in the handoff:
it must pass at (e) with FF live. It's a frozen-test change, so it goes in its own section of 0014 with my approval
quoted. Not (b): coupling inside the envelope stops the check from asking whether the controller rejects it. Not (c):
Gazebo flies identically under both gain sets, so the gains aren't the cause."

**The evidence** (`tests/regression/quad/L06/results/r1x_coupling/`, raw output `coupling.txt`; the gz summaries
`gz_new.txt` and `gz_old.txt` come from `capture_gz.py`, run twice each and byte-identical; pytest
`tests/regression/quad/L06/tools/test_r1x_coupling.py`, 13 tests; byte-identical in marv-ci).
- **The models.** D is the design model with no ω×Jω; C adds it. Both are seeded with Gazebo's own first-step rounding
  q_gz(1), so they follow Gazebo's branch.
- **New gains** (att_kp 3.154): Gazebo fails the predicate (74 violations, slack +5.05e-3). C reproduces it (85
  violations, slack +6.08e-3); D stays inside. Distance to Gazebo, D → C: α 9.38e-2 → 2.28e-3, ω_y 0.551 → 7.25e-3,
  err_z 6.77e-2 → 1.06e-3.
- **Old gains** (`69c62f2`, 3.087): Gazebo passes (slack −4.77e-2). C and D both stay inside. Distance, D → C:
  α 9.97e-2 → 2.70e-3, ω_y 0.886 → 3.50e-2.
- **Control C0:** from the exact start the coupled model is a pure roll (ω×Jω = 0) and is never closer to Gazebo than
  D.
- **"Identical under the old and new gains", precisely:**
  - Gazebo's branch is identical: q_gz(1) and ω_gz(1) are bit-identical, and ω_y/ω_x at n 300 changes by +0.22 %.
  - The coupling's effect on α is close but not identical: 0.1014 rad old, 0.0933 new (−8 %). The coupling integral
    changes by −11.8 / +6.0 / −6.6 % per axis.
  - What changed is the envelope: F + Q went 4.81e-2 → 2.97e-2. The new loop's tighter band exposes a coupling lag that
    the old envelope contained.
- **Ideal feed-forward:** C with ideal lag-compensated FF equals D within K + R (largest gap 3.6e-11) and stays inside,
  under both gain sets.

**The frozen change,** `tests/regression/quad/L05/gz/test_t4_recovery.py`:
- `test_exact_180_tilt_angle_is_inside_the_alpha_envelope` keeps its name and becomes a strict xfail
  (`raises=AssertionError, strict=True`) while `tests/regression/quad/L06/XFAIL_GATE_CLOSED` is absent, the same gate
  as R2. Its reason (`EXACT_KNOWN_FAILING`) cites this section and the evidence.
- The new normal test `test_exact_180_run_is_clean_on_truth_and_in_the_dshot_range` carries R1X's clean-run check and,
  newly asserted for R1X, the DShot range. This tightens the test.
- The settled-end test is unchanged; the docstring gains three lines.

**gz-l5 at the final gains:** 63 passed, 2 xfailed (the R2 envelope and the R1X envelope), 0 failed, 0 skipped.

## W4: the feed-forward switch and the FF-on report runs

- **The switch.**
  - `rate_ff_enable` is i32 0, a scenario-register entry (`design/scenario_values.yaml`).
  - `rate_ff_filter_tau` (T_ff, 12.311602 ms) is derived by `tools/card/rate_lead.py` rule step 9 and emitted by
    `flatten.py --out-rate-lead`.
  - J and τ_m are the card's `inertia_xx/yy/zz` and `motor_tau`.
  - `rate::from_params` reads the FF parameters only when `rate_ff_enable == 1`.
  - Stage (e) switches the default in the change that creates `XFAIL_GATE_CLOSED`.
- **Inert by default.** With the switch at 0:
  - ctest passes 642/642 in debug and in release;
  - every gz FF-off result is unchanged;
  - R1X's truth sha matches.
- **Frozen file changed: `tests/regression/quad/L06/param_ids`.** Five ids are appended, with their provenance
  comments: `rate_d_filter_tau_{roll,pitch,yaw}`, `rate_ff_enable` and `rate_ff_filter_tau`.
- **Regenerated:** `tests/regression/quad/L06/results/ff_eval/{card_worst.txt, sweep.txt}`. Only the inputs-sha line
  changes.
- **The FF-on report runs** (owner decision 2, reported, not asserted).
  - Evidence: `tests/regression/quad/L06/results/ff_on/` (`capture_gz.py`, `gz_acro.txt`, `gz_r2.txt`,
    `gz_r1x.txt`, README). Its pytest is `tests/regression/quad/L06/tools/test_ff_on.py`.
  - **Acro recovery: FAIL → PASS.** Margin 6.07 mrad/s on roll, a knife edge. T3's prediction is 3.2 mrad/s.
  - **R1X α: FAIL → PASS** (+5.05e-3 → −2.92e-2).
  - **R2: FAIL → FAIL.** Violations 13923 → 10398; the worst channel moves from w_x +3.866 to w_y +1.975. With FF on,
    err_z and w_y are worse. DShot spans 48..2047 in both variants.
  - **Diagnosis** (scratch, not committed; it is committed as evidence once Luis rules):
    - **Main cause: the gz step-0 zero-rate read** (`sim/gz/plugin/src/lockstep.cpp`, 0003 item 11; recorded in
      `tests/regression/quad/L05/results/recovery_cause/README.md`). Execution 0 reads ω = 0 and execution 1 reads ω0,
      so the chain and the rate loop seed on 0 and then see a step. The PI loop was immune. The D term (impulse about
      −0.58·Jω0 per axis) and the FF lag term (τ_m·τ0) differentiate the step.
    - A closed-loop tool model with the zero read reproduces gz to 0.016 rad/s (w_y +1.995 against +1.988). Without it,
      the same model gives 2416 violations, worst w_z +0.24, and every attitude channel inside the envelope.
    - **Second cause: yaw saturation plus the yaw integrator freeze** (`record_allocation`). Neither is in the linear
      design model. Without the zero read, the allocator flags yaw at executions 1–93. With the allocator limit also
      removed: 1178 violations (w_x +0.031, w_z +0.066).
    - **What remains:** T_ff and the rotor-speed lag against the design's torque lag, w_z +0.0086 against F + Q
      0.0079 (INFERRED, one nominal run).
    - **Ruled out:** motor 1 at its floor (the allocator chooses that collective itself). A centred c* is worse
      (11562 violations).
    - It puts at risk the plan "R2 passes at (e) with FF live" (third round, item 3), so it goes to Luis.

## Frozen files changed by stage (c)

Every file under `tests/regression/` that stage (c) modifies, as a complete list. Files the stage adds are not
listed. The reason for each change is in the section named.

- `tests/regression/quad/L04/gz/test_t4_acro.py`: 3d-L4.
- `tests/regression/quad/L04/gz/test_t4_steps.py`: 3d-L4.
- `tests/regression/quad/L04/replay/CMakeLists.txt`: 3b (wiring).
- `tests/regression/quad/L04/replay/l4_acro_replay.cpp`: 3b (wiring).
- `tests/regression/quad/L04/t3/CMakeLists.txt`: 3c-L4.
- `tests/regression/quad/L04/t3/README.md`: 3c-L4.
- `tests/regression/quad/L04/t3/reference/SHA256SUMS`: 3c-L4.
- `tests/regression/quad/L04/t3/reference/rate_t3_inputs.txt`: 3c-L4.
- `tests/regression/quad/L04/t3/reference/rate_t3_oracle.py`: 3c-L4.
- `tests/regression/quad/L04/t3/t3_test.cpp`: 3c-L4.
- `tests/regression/quad/L04/tools/test_l4_acro_bound.py`: 3d-L4.
- `tests/regression/quad/L05/gz/recovery_model.py`: 3d-L5 (envelope A).
- `tests/regression/quad/L05/gz/test_t4_chirp.py`: third round, item 2 (the plateau admission).
- `tests/regression/quad/L05/gz/test_t4_recovery.py`: 0015 (R2) and the R1X section.
- `tests/regression/quad/L05/results/recovery_cause/build_tool.sh`: 3b (wiring).
- `tests/regression/quad/L05/results/recovery_cause/recovery_cause_tool.cpp`: 3b (wiring).
- `tests/regression/quad/L05/results/step_cause/build_tool.sh`: 3b (wiring).
- `tests/regression/quad/L05/results/step_cause/step_cause_tool.cpp`: 3b (wiring).
- `tests/regression/quad/L05/results/step_controls/build_tool.sh`: 3b (wiring).
- `tests/regression/quad/L05/results/step_controls/step_controls_tool.cpp`: 3b (wiring).
- `tests/regression/quad/L05/t3/CMakeLists.txt`: 3c-L5 and W1b.
- `tests/regression/quad/L05/t3/README.md`: 3c-L5 and W1b.
- `tests/regression/quad/L05/t3/reference/SHA256SUMS`: 3c-L5 and W1b.
- `tests/regression/quad/L05/t3/reference/attitude_t3_inputs.txt`: 3c-L5 and W1b.
- `tests/regression/quad/L05/t3/reference/attitude_t3_oracle.py`: 3c-L5 and W1b.
- `tests/regression/quad/L05/t3/t3_test.cpp`: 3c-L5 and W1b.
- `tests/regression/quad/L05/tools/test_l5_scenario.py`: third round, item 4 (the authorized edit) and 0015.
- `tests/regression/quad/L05/unit/composition/CMakeLists.txt`: 3b (wiring), 3d-L5.
- `tests/regression/quad/L05/unit/composition/composition_test.cpp`: 3b (wiring), 3d-L5.
- `tests/regression/quad/L06/CMakeLists.txt`: commit 1 and 3b.
- `tests/regression/quad/L06/d_term/rate_loop_test.cpp`: 3a (the inert law is built explicitly, since from_params now reads T_f).
- `tests/regression/quad/L06/gyro_chain/fault_test.cpp`: 3b (reseed rule) + divisor (b).
- `tests/regression/quad/L06/param_ids`: 3a and W4.
- `tests/regression/quad/L06/results/ff_eval/README.md`: 3d-L4 (the acro test's Z describes the stage (c) law at T4c) and W4.
- `tests/regression/quad/L06/results/ff_eval/card_worst.txt`: W4 (regenerated).
- `tests/regression/quad/L06/results/ff_eval/sweep.txt`: W4 (regenerated).
- `tests/regression/quad/L06/results/r2_lower_bound/README.md`: 0015 (the bound now names the hover-rotor scenario `recover_tumble_prop_strike`), and the inputs-sha line regenerated.
- `tests/regression/quad/L06/results/r2_lower_bound/bound.txt`: 0015 (the bound now names the hover-rotor scenario `recover_tumble_prop_strike`), and the inputs-sha line regenerated.
- `tests/regression/quad/L06/results/r2_lower_bound/r2_lower_bound.py`: 0015 (the bound now names the hover-rotor scenario `recover_tumble_prop_strike`), and the inputs-sha line regenerated.
- `tests/regression/quad/L06/tools/test_attitude_lead.py`: W1b (the configuration set).
- `tests/regression/quad/L06/tools/test_l6_ff_eval.py`: 3d-L4 (the identity check moves to the stage (c) law; the PI law keeps its cause.txt check).
- `tests/regression/quad/L06/tools/test_rate_lead.py`: W1b (the configuration set).

## Evidence

- **Full local CI** (`ci/local_ci.sh`, base 1159d5c) on snapshot `f6f4610` (the whole stage (c) tree, before this
  section): core, gz-l2, gz-l4 and gz-l5 all pass.
  - ctest 642/642 in debug and in release; the frozen suites 641/641.
  - Tools step: 1063 passed, 1 skipped (`test_gen_sdf_plant_config.py`, which needs gz; the skip predates stage (c)).
  - gz-l4: 40 passed, 1 xfailed. gz-l5: 63 passed, 2 xfailed.
  - The regression change check passes: 32 frozen files, 2 decision records.
- **Per-file wall time in the CI image** (4 CPUs, run one at a time on a quiet host), against S9's 60 s:
  - Over the limit: `L06/tools/test_chirp_admission.py` 111.8 s, `L06/tools/test_attitude_lead.py` 93.0 s,
    `L01/tools/test_flatten_report.py` 66.6 s.
  - At the limit: `L05/tools/test_attitude_t3.py` 59.6 s, `L06/tools/test_r1x_coupling.py` 59.1 s.
  - Under it: `L06/tools/test_r2_lower_bound.py` 55.9 s.
  - Per Luis (second round), this goes to him as the S9 nightly option; no test is trimmed.
- **Full local CI on the fifth-round tree** (snapshot `b4c9e16`, base 1159d5c): core, gz-l2, gz-l4 and gz-l5 all
  pass.
  - ctest 642/642 in debug and in release; the frozen suites 641/641; G1 and G3 pass.
  - The product table matches byte for byte for host-debug and for m33.
  - The per-push tools step: 1013 passed, 1 skipped (the gz-only test), in 274 s.
  - gz-l4: 40 passed, 1 xfailed. gz-l5: 63 passed, 2 xfailed.
  - The regression change check passes: 32 frozen files, 2 records.
- **S9 B, the shared design** (Luis, fourth round, item 1).
  - `tests/regression/quad/L06/tools/conftest.py` gains a session fixture, `attitude_lead_design`.
    `test_attitude_lead.py` and `test_chirp_admission.py` use it. The chirp test asserts that the call it replaces
    received identical inputs, a guard with a 4-of-4 planted-change control. No assertion, control or tolerance
    changes, and no frozen file is edited.
  - Times (marv-ci, 4 CPUs), alone → in the tools session:
    - `test_attitude_lead.py`: 93 s → 93.5 s. It pays for the shared designs; its third design is its own.
    - `test_chirp_admission.py`: 111.9 s → 50.8 s.
    - Unchanged, because sharing cannot reach them: `L01/tools/test_flatten_report.py` (frozen) 65.8 s, which runs
      the CMake generation as a subprocess; `L05/tools/test_attitude_t3.py` (frozen) 59.6 s.
    - `test_r1x_coupling.py` 59.0 s and `test_r2_lower_bound.py` 56.5 s: distinct envelopes.
  - The whole tools step: 543 s → 508 s; 1063 passed, 1 skipped.
  - The CMake generation (62.3 s): `rate_lead.design` 26.1 s, `attitude_lead.design` 35.1 s and `attitude.design`
    1.0 s, on `usable_cpus()` = 4 processes at about 73 % use. No work is duplicated.

## Stage (c) close

**1. What stage (c) adds.**
- Firmware: the D term on the measurement through a low-pass (T_f = 1/ω_p = 41.87 ms); the lag-compensated ω×Jω FF (T_ff 12.31 ms), built and inert (`rate_ff_enable` 0) until (e); the chain's input guard and first-sample seeding; `fw/rate_group`, one tick step (chain, notch update, rate loop) shared by both compositions and the replay tools.
- Tools: `rate_lead.py` (PI × lead, N* = the largest N within Ms_max and the D + FF noise budget, f32 guard on the 312/313 µs loop, physical 3-D J corners); `attitude_lead.py` (att_kp over the configuration set); Stein (rate, 120 loops) and Lyapunov (attitude) stability certificates.
- 0015: R2 starts from a steady tumble at c* 8.917 N (hover is infeasible: motor 1 would need −0.301 N); the old hover-rotor tumble is `recover_tumble_prop_strike`.

**2. Product numbers, before (stage (b), today's PI with the chain) → after.**
- Rate ω_c 8.32 rad/s (today's L4 gains) → 12.129 rad/s nominal; N 1 → N* 3.877.
- Worst PM 35.45° (ESC 2 %, J−, τ+) → 45.00004° over the set (J−, τ+); worst Ms not recorded before → 1.99994 (J−, τ−).
- att_kp 3.0872879 → 3.1539721 (bound by bypassed, latency 0, roll J+ τ+, PM 45.00001°); attitude worst Ms 1.9150; t_cross 0.274375 s → 0.2925 s. Attitude PM and crossover before: not recorded in these records.
- Closed-loop time constant at J+: about 0.161 s (design consult, "barely changed from PI"); the c2 tool's t63 is 0.151 s.
- Noise: budget 0.5 × 4.56 mN = 2.281 mN; D + FF at rate_max 2.281 mN (100 %, T_ff is set by it); D path at hover 0.494 mN (21.7 %).

**3. The pass bar.**
- (b, c) T3, QF-3 over the band box with the chain and the D low-pass in the loop: PM 45.00004° ≥ 45°, Ms 1.99994 ≤ 2.0; margin zero by design (Luis, first round).
- (c) T3, D + FF noise ≤ budget at hover and rate_max: met; zero margin at rate_max (T_ff rule).
  - At hover ω = 0, so the FF path's noise contribution is zero to first order and the combined value equals the D path's 0.494 mN (21.7 %). Today only the maximum over the operating points is asserted (`test_rate_lead.py:128`); the hover assertion comes with 0016's commit.
- (c) T3, goldens regenerate with D and the controls still break them: L4 float error 1.4 % of tolerance, gains ×1.1 283×, one tick 3.8×; L5 at final k gains ×1.1 ×2464.6 / ×70.7, rate ×1.1 ×1137.8 / ×206.3, one tick ×28.1 / ×5.1.
- (c) T3, the ω×Jω evaluation recorded (c5): lag-compensated FF passes at the card plant by 24.1 mrad/s; PID alone and plain FF fail everywhere; nothing passes at the 12 physical J corners.
- Added by rulings: configuration-set PM/Ms (above; controls fail); per-motor mixes, 70 per push (715 in the scratch evaluation), none worse than the set (control 44.560°, 2.0267); rate certificate ρ ≤ 0.9992518, min GM 2.5937 (control at 1.1·GM); attitude Ms over the set 1.9150 (control 2.0301); chirp plateau admission, slack ≥ +7.600° (control ×2 fails every axis); divisor f_c < f_s/(2D), D* 6 refused, 5 accepted; parameter pin, 93 records byte for byte; chirp lead-off reduction to rate.py within 2⁻³⁰ (control: the lead on, 0.174 rad).

**4. Known failing items** (strict xfail while `L06/XFAIL_GATE_CLOSED` is absent; an unexpected pass fails CI; no reseeding or bound tuning).
- L4 acro: FF off, roll excess 5.23 rad/s. FF on (W4): pass by 6.07 mrad/s (T3 predicted 3.2), at the card plant only. (e): FF live with the gate file.
- L5 R2: FF off 13923 violations (w_x +3.866 rad/s); FF on 10398 (w_y +1.975). Causes: the gz step-0 zero-rate read (0016), then yaw saturation and the integrator freeze. 0016 expects about 2416 violations (w_z +0.24 rad/s), still failing. (e) needs the yaw choice too (model the allocation and freeze, or rule R2 yaw-limited).
- L5 R1X (exact-180° α predicate only): FF off 74 violations (slack +5.05e-3); FF on pass (−2.92e-2). (e): FF live.

**5. Frozen changes.** 32 files that existed at `1159d5c` (the CI count), plus 10 added by commits 1–2 and changed by commit 3: 42 listed above.
- 3b wiring onto `RateGroupStep`, 12 (replay, results tools, L05 composition, L06 CMake, `fault_test.cpp`, which also carries divisor (b)); 3c-L4 L4 T3, 6; 3c-L5/W1b L5 T3 at the final k, 6; 3d-L4 L4 consumers, 5; W4 and 3a, 4; W1b set tests, 2; 3d-L5 envelope A, 1; third round item 2 chirp, 1; 0015/R1X, 5.
- Relaxed, each approved: divisor 1 now accepted (generator domain), replaced by D* 6 refused / 5 accepted and 0.99·f_s/2 at D = 2 now refused (net stricter); R1X α became a strict xfail, with a new normal test for the clean run and the DShot range; the chirp plateau admits only runs with |shift| < 4.217°, floor 3 (amends 0006 decision 20); R2 hover start → steady tumble, the hover case kept as a reported scenario.

**6. Verification.**
- Full local CI, all four jobs, on snapshot `b4c9e16`: pass; tools 1013 passed, 1 skipped; gz-l4 40 passed, 1 xfailed; gz-l5 63 passed, 2 xfailed.
- Core on `7ecfe8b`: pass (1578 s); ctest 642/642 debug and release, frozen 641/641; tools 1016 passed, 1 skipped; regression check 32 files, 2 records.
- `b4c9e16` → `7ecfe8b`: 0014 (+36/−2), `docs/handoff.md` (+48), the new `L06/tools/test_chirp_lead_off.py` (+140), and two docstring lines in `L06/tools/test_notch_mixes.py` (+2/−2). None can affect gz-l2/l4/l5: those jobs collect only `L02`, `L04/gz` and `L05/gz`, and nothing built changed.

**7. Recorded limits and findings.**
- Physical J corners: FF can be worse than PID (pitch, corner 11: 5.14 → 9.46 rad/s); FF flies only if the pre-L8 re-run over the measured band shows it helps.
- Response about 0.161 s at J+; the ESC clock error is the largest single limit on crossover (a crystal ESC: ω_c 23.6 rad/s, τ about 0.086 s).
- The hover-rotor tumble (prop strike, 14353 violations) has no pass bar until L8.
- S9 is unenforced for pytest. Nightly: `test_chirp_admission` 111.7 s, `test_notch_mixes` 92.4 s, `test_attitude_lead` 81.4 s, `test_flatten_report` 65 s, `test_attitude_t3` 61 s. Nearest per push: `test_r1x_coupling` 58.9 s (1.1 s margin), `test_r2_lower_bound` 56.2 s (3.8 s).
- Acro FF-on margin 6.07 mrad/s is a knife edge for (e); the T3 oracles keep the f_s/2 bound (consistent at D = 2); the rate_max noise point is an upper bound (the hover convention would give T_ff 1.8 ms).

**8. Carried forward.** 0016 (gz first read returns the starting rates; R2's FF-off numbers then change); stage (d) as 0017 (rulings: fourth round item 5; the motor-speed line text is owed to Luis); the pre-L8 gate and hardware list (J with σ, τ_m, AM32 per-frame, the prop-strike pass bar); owed from 0009 the final CI split with times and the T4 seed count with cost, from 0012 the turn-on-corner count with cost; the `step_cause` re-run at N = 1; S9 pytest enforcement at the next CI change.

**9. For Luis.**
- Accept the four lead decisions as recorded (rate_max noise on the notch-free chain, τ_ref from τ_cl, `seed_first_sample` on in the wiring, the geometric notch grid)? yes/no.
- Commit the W4 diagnosis scripts as evidence with 0016? yes/no.
- Approve stage (c) and push? yes/no.

## Approval

Owner decisions: Luis, 2026-10-01, as quoted above. The spec lines (FF Builds bullet, combined D + FF noise line):
Luis, 2026-10-01, approved in his wording (second round, item 3).
Stage (c) approved: Luis, 2026-10-03.

Luis, 2026-10-03, on the close summary's section 9 (verbatim):

"1. **The four lead decisions: yes.**
   - Rate_max noise on the notch-free chain is the conservative case.
   - The other three (τ_ref from τ_cl, first-sample seeding in the wiring, the geometric notch grid) are covered by the green suites and by the 70-mix per-push test.
2. **The W4 diagnosis scripts: yes, commit them with 0016.** Include their inputs and raw output. The 0016 prediction (about 2416 violations) is a measurement, and under the number rule it counts only with those committed.
3. **Stage (c): approved, and push.**"
