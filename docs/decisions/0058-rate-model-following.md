# 0058: The rate loop follows its reference model with a feed-forward; tau_ref is bounded by motor lag plus delay

## What changed

1. **Law** (`fw/rate`). In acro (`execute`) the torque gains the reference feed-forward
   `u_ff,n = J_a (r_n - r_(n-1)) / dt_n`, with r the QF-2 reference model's output and J_a the card's principal inertia. It
   is never applied in `execute_bypass` (angle mode). The switch is a new scenario parameter, `rate_ref_ff_enable`. It is 1
   in `fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml`, so `marv_params_fc3v2` and `marv_params_fc3v2_eskf` have
   it on. It is 0 in `design/scenario_values.yaml`, so every SIL and regression set has it off. The id sits right after
   `rate_ff_enable` in both registers, so product and FC3v2 ids stay equal.
2. **tau_ref rule** (`tools/card/rate_lead.py`, new rule step 14). With the switch on:
   `tau_ref,a = r32_up(max(omega_max,a / alpha_max,a, tau_m (1 + b_tau) + d))`. The loop delay d is the profile's latency,
   plus the low-pass group delay at w_c,nom with every notch bypassed, plus one rate-loop period. The generator refuses a
   set that breaks invariant I4: tau_ref must meet the bound, and `J rate_max / tau_ref <= tau_max` at J (1 - b_J, 1, 1 + b_J).
   With the switch off, decision 0005's `max(tau_cl, omega_max / alpha_max)` is unchanged. The gains, the margins and the
   configuration set do not read the switch.
   FC3v2 result: roll and pitch go from 0.219458 s to 0.065964 s; yaw stays at 0.310547 s, where authority binds.
3. **Amendment of decision 0005, QF-2.** 0005 read QF-2 as a prefilter in front of the closed loop, with
   tau_ref >= tau_cl (the closed-loop t63). That composite does not follow its own model: t63 is 371 ms against a tau_ref of
   211 ms (architect, 2026-10-18). Under model following, the achievable time constant is the motor lag at the slow end of
   its band plus the loop delay. The feed-forward depends on the setpoint path only, so the feedback loop, its gains and
   QF-3 are unchanged.
4. **Proposed spec sentence** (Luis applies it; `docs/spec/10-quad-flight-software.md` is not edited here). Add to the QF-2 rule:
   "With the reference feed-forward on (model following), the achievable time constant is τ_m(1 + b_τ) plus the loop
   delay; without it, it is the closed-loop t63."
5. **Frozen files edited** (tests added, no threshold or predicate loosened; the product table regenerated with one new
   row and later ids shifted by one, no value changed):
   tests/regression/quad/L04/param_ids
   tests/regression/quad/L05/unit/rate_bypass/rate_bypass_test.cpp
   tests/regression/quad/L06/d_term/rate_loop_test.cpp
   tests/regression/quad/L06/tools/test_rate_lead.py
   tests/regression/quad/L06/product_params/marv_params_product_table.txt
   tests/regression/quad/L06/results/ff_eval/card_worst.txt
   tests/regression/quad/L06/results/ff_eval/sweep.txt
   (the two ff_eval outputs re-run by their README commands: only the input sha256 lines changed, for the edited
   design/scenario_values.yaml; sweep.txt's design/budget.yaml line was already stale and is current again)

## Why

Luis flew HIL S4b in ACRO and found it sluggish. The architect's linear model (Serena memory
`quad-m4/architect-rate-response-2026-10-18`) shows the cause: the prefilter sits in series with a slower closed loop.
Here is what model following changes in that model (ACRO, nominal):

| Response | Today | With feed-forward |
| --- | --- | --- |
| 175 ms flick reach | 39 % of command | 81 % |
| Flick peak after release | 164 ms | 6 ms |
| Flick to 10 % | 610 ms | 348 ms |
| Step t63 / overshoot | 371 ms / 15 % | 116 ms / 7.7 % |

All band corners stay clean at tau_ref 0.065 s. The alternatives were rejected: 1-DOF with a short tau_ref (21 %
overshoot), lower margins, narrower bands without measurements, and a lag-compensated or raw-stick feed-forward.

## Evidence

- Invariants and their checks:
  - I1, the torque's gyro sensitivity is the same with the feed-forward on and off: `L6RefFeedforward.*`, controlled by
    adding the omega x J omega term, which reads the gyro. The switch moves only `rate_tau_ref_*`: `test_the_switch_moves_tau_ref_only`.
  - I2, the law without the feed-forward is bit-identical: `L6DTerm.DefaultsAreOff...` and its control. The frozen L04, L05
    and L06 suites pass on the off sets.
  - I3, no feed-forward in the bypass: `L5RateBypassNoReferenceFeedforward.*`.
  - I4: the `test_model_following_*` tests, with two controls, plus the generator's refusal.
- SIL comparison in acro, before against after, in gz SIL with quad_fc on the FC3v2 card
  (`tests/regression/quad/L10/results/rate_ff/`):

  | Response | Before | After |
  | --- | --- | --- |
  | Step t63 | 387 ms | 118 ms |
  | Step overshoot | none in 0.6 s | 6.1 % |
  | Flick reach | 37.7 % | 81.3 % |
  | Flick peak after release | 173 ms | 7 ms |
  | Flick, peak to 10 % | 642 ms | 354 ms |

## Approval

Luis, 2026-10-18 (AskUserQuestion): feed-forward "Approve (Recommended)", for model following with the tau_ref bound,
with this record amending 0005. Pull-request approval: pending.
