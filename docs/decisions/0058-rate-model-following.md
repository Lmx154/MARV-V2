# 0058: The rate loop follows its reference model with a feed-forward; tau_ref is bounded by motor lag plus delay

## What changed

1. **Law** (`fw/rate`). In acro (`execute`) the torque gains `u_ff,n = J_a (r_n - r_(n-1)) / dt_n`, with r the QF-2 reference
   model's output and J_a the card's principal inertia. It is never applied in `execute_bypass` (angle mode). The switch is a
   new scenario parameter, `rate_ref_ff_enable`. It is 1 in `fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml`
   (`marv_params_fc3v2` and `_eskf`) and 0 in `design/scenario_values.yaml` (every SIL and regression set). It sits right
   after `rate_ff_enable` in both, so the product and FC3v2 ids stay equal.
2. **tau_ref** (`tools/card/rate_lead.py` rule step 14). With the switch on,
   `tau_ref,a = r32_up(max(omega_max,a / alpha_max,a, tau_m (1 + b_tau) + d))`. The loop delay d is the latency, plus the
   low-pass group delay at w_c,nom with every notch bypassed, plus one rate-loop period. The generator refuses a breach of
   I4: tau_ref below its bound, or `J rate_max / tau_ref > tau_max` at J (1 - b_J, 1, 1 + b_J). With the switch off, 0005's
   `max(tau_cl, omega_max / alpha_max)` is unchanged. The gains and margins do not read the switch. On FC3v2, roll and pitch
   go from 0.219458 s to 0.065964 s; yaw stays at 0.310547 s, where authority binds. The rule for d is `INFERRED` until the
   bench motor tau (eRPM) and the HIL loop delay are measured.
3. **Amends decision 0005, QF-2.** 0005 read QF-2 as a prefilter in front of the closed loop, with tau_ref >= tau_cl. That
   composite does not follow its model: t63 is 371 ms against a tau_ref of 211 ms. Under model following, the achievable time
   constant is the band's slowest motor lag plus the loop delay. The feed-forward depends on the setpoint path only, so the
   feedback loop and QF-3 are unchanged.
4. **Spec sentence** for the QF-2 rule, applied to quad spec §QF-2 (Luis, 2026-10-19, AskUserQuestion: "Apply (Recommended)"): "With the reference
   feed-forward on (model following), the achievable time constant is τ_m(1 + b_τ) plus the loop delay; without it, it is the
   closed-loop t63."
5. **Frozen files edited.** Tests were added and nothing was loosened. The product table gained one row, with later ids
   shifted and no value changed. The two ff_eval outputs were re-run by their README commands, and only their input sha256
   lines changed: `tests/regression/quad/L04/param_ids`, `tests/regression/quad/L05/unit/rate_bypass/rate_bypass_test.cpp`,
   `tests/regression/quad/L06/d_term/rate_loop_test.cpp`, `tests/regression/quad/L06/tools/test_rate_lead.py`,
   `tests/regression/quad/L06/product_params/marv_params_product_table.txt`,
   `tests/regression/quad/L06/results/ff_eval/card_worst.txt`, `tests/regression/quad/L06/results/ff_eval/sweep.txt`.
   The clamp adds `tests/regression/quad/L06/param_ids` and appends clamp tests (the term's clamp set explicitly where a test
   enables J_ref; no check loosened) to `tests/regression/quad/L06/d_term/rate_loop_test.cpp` and
   `tests/regression/quad/L05/unit/rate_bypass/rate_bypass_test.cpp`, with `L04/param_ids`, `L06/tools/test_rate_lead.py`, the
   product table (three derived rows and `rate_ff_clamp_k`, later ids shifted, no value changed) and the two ff_eval outputs
   (input sha256 lines only) touched again.

## Why

Luis flew HIL S4b in ACRO: "i won't lie that was extremely hard to control. it seemed like everything was sluggish to respond
but also the delayed response was very aggressive and was sluggish to stop". The architect's linear model (Serena memory
`quad-m4/architect-rate-response-2026-10-18`) traces this to the prefilter sitting in series with a slower closed loop. Model
following removes that: all band corners stay clean at a tau_ref of 0.065 s. The alternatives were rejected: 1-DOF with a
short tau_ref (21 % overshoot), lower margins, narrower bands without measurements, and a lag-compensated or raw-stick
feed-forward.

## Open: gates the first props-on acro flight

I4 bounds the feed-forward against the collective-free tau_max, and nothing clamps u_ff. Review (2026-10-19): a full roll or
pitch slam asks for about 0.6 tau_max; a full yaw reversal asks for 1.33 to 2 tau_max for about 0.2 s. Before acro with props
on: a per-axis symmetric clamp on u_ff at k tau_max (at hover collective), k set by Luis (a safety number), with a T3 test and
a clamp-off bit-identity case. Angle mode, hover and props-off work are unaffected; the angle-to-acro switch is continuous.
Luis, 2026-10-19 (AskUserQuestion, k as a fraction of the authority at hover throttle): "0.5 (Recommended)".
Built (branch `rate-ff-clamp`): in `execute`, `u_ff,a` is limited to `+-k tau_max_hover,a` before it is added (`fw/rate`;
`validate` refuses k outside (0, 1], and a missing clamp, when some J_ref > 0). k is the scenario parameter `rate_ff_clamp_k`
(0.5 in both registers). `tau_max_hover,a` is derived (`rate_tau_max_hover_<axis>`, `tools/card/rate_lead.py` rule step 15):
the largest symmetric torque on axis a alone that keeps every motor within [k idle^2, k speed_max^2] with the collective at
m g, rounded down to f32. FC3v2: roll 0.9153, pitch 0.8040, yaw 0.0861 N m (INFERRED card inputs). Angle mode, the feedback
path and the switch-off law are unchanged.

## Evidence

- I1 `L6RefFeedforward.*` (gyro sensitivity equal with the feed-forward on and off; control adds ω×Jω) and
  `test_the_switch_moves_tau_ref_only`. I2 `L6DTerm.DefaultsAreOff...` (bit identity, with control; frozen L04-L06 pass).
  I3 `L5RateBypassNoReferenceFeedforward.*`. I4 `test_model_following_*` (two controls) and the generator's refusal.
- Roll in acro: model (architect's, nominal) and gz SIL of quad_fc on the FC3v2 card (`tests/regression/quad/L10/results/rate_ff/`).

  | Response | Model before | SIL before | Model after | SIL after |
  | --- | --- | --- | --- | --- |
  | Step 100 deg/s: t63 | 371 ms | 387 ms | 116 ms | 118 ms |
  | Step overshoot | 15 % | none in 0.6 s | 7.7 % | 6.1 % |
  | Flick 200 deg/s, 175 ms: reach | 39 % | 37.7 % | 81 % | 81.3 % |
  | Flick: peak after release | 164 ms | 173 ms | 6 ms | 7 ms |
  | Flick: peak to 10 % | 610 ms | 642 ms | 348 ms | 354 ms |

## Approval

Luis, 2026-10-18 (AskUserQuestion), on the feed-forward: "Approve (Recommended)". This covers model following with the tau_ref
bound, with this record amending 0005. Review 2026-10-19: PASS, merge allowed, with the open gate above.
