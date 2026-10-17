# Handoff: M3 merged; M4 software (real card, angle mode, I-hold) and HIL stages S1-S2 on `m4`; open questions for Luis

2026-10-17. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show 55120ad:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (M4 as amended at 21dc067: Luis's order for the work before the stand arrives).

## Where things are
| Item | State |
| --- | --- |
| M0-M3 | merged into `master` (M3 at `55120ad`, local_ci on `609d3e9`: six jobs PASS); not tagged (Luis's) |
| M4 | branch `m4`, pushed at `ab63781` (last dev check PASS at `f0bb97b`; since then only CI-ignored paths); flight image `gf0bb97b` on the board |
| HIL | S1, S2 (incl. the launch pad, 0047) and S3 merged into `m4` at `ab63781`: the board's DShot matches the host SIL's within 1 count, arming on identical ticks (`L09/results/hil_s3b`) |

## M4 so far (decisions 0043-0047)
- Real airframe card `vehicles/marv_5in_fc3v2.yaml` (Luis's measurements: 732 g with pack, motors at x ±0.112, y ±0.1275 m,
  battery on top; sourced k_T 2.8e-6, torque 0.012 m, tau 0.05 s, inertia from `vehicles/marv_5in_fc3v2_inertia_model.py`,
  all INFERRED/UNVERIFIED/PLACEHOLDER until stand and pendulum). `marv_params_fc3v2` builds from it (0044). Board profile
  completed; IMU latency 3 samples (INFERRED: batching + LPF2 group delay). QF-3 on the real card: rate worst PM 45.0045°,
  Ms 1.9994; nominal-row PM 73.1° (worst corner sits at PM_min by construction of the gain rule).
- Angle mode in quad_fc (0043): `rc_channel_angle` default 0 = acro (Betaflight's no-mode default); the attitude law's
  off-period executions are counted faults, never halts, fail closed; init refuses att_loop_ratio != 1.
- Rate integrator zeroed while armed below mincheck (0045, Betaflight's airmode-OFF rule; the default airmode-ON latch
  is pending Luis).
- HIL (0046): S1 link proof: 7500 samples/s each way, zero loss, round trip max 1.26 ms at 4 per frame
  (`L09/results/hil_s1`). Plugin rounds the fc3v2 tick to whole ns (−1.6 ppm). N-tick command delay line; N_max on the
  nominal rows = 249 (`L09/results/hil_s2`), N ≥ 10 needed. quad_fc is a SIL library (`marv_sil_quad_fc`, fc3v2 set)
  with ManualControl staging. Launch pad (0047): holds the quad until thrust ≥ weight; not ground contact, no landing.

## Waiting on Luis
1. Angle switch: which channel/switch on the RadioMaster Pocket and which position is angle (`rc_channel_angle`,
   `angle_switch_threshold`).
2. Airmode: keep "I zeroed whenever throttle < mincheck" or Betaflight's default latch (zero only until 25 % throttle).
3. HIL spec edits (0046 item 6): quad §1 line 42, L10 needs, the T6 comparison with the N-tick delay.
4. Plant `omega_min > 0` check vs his speed_range [0, 2991]: loosen frozen test `PlantAbiCreate.RefusesEveryInvalid
   ConfigField` (L01) to accept 0, or a small positive card minimum. Blocks Gazebo flying the real card.
5. A short bench: EMB-3 with angle mode flying (needs the switch), and the props-off radio check of angle mode.
6. Tags M1-M3.
7. The m33 build has no -ffp-contract=off (GCC default fast; host has it): quad L10's pass bar says "off on both builds",
   core allows a tolerance. Accept the measured 1-count tolerance (recommended) or add the flag to the flight build.

## Next (no hardware needed)
- S2e: the real-card Gazebo hover with quad_fc in angle mode on the launch pad, N = 10 (needs item 4); then HIL S4: the
  board flies Gazebo closed loop with Luis's radio (needs items 1, 4); then S5 (S4's sticks replayed in SIL).

## Noticed, not changed
- Boot rate timing fault when the receiver streams at boot (INFERRED: UART traffic). `t3_test.cpp` builds the rate
  loop's config by hand without its timing fields (rate check off there). sensor_replay refuses logs with records 9-11.
- ESC "Motor KV" 820 vs 1700. Speed range max is the no-load speed (optimistic thrust ceiling until the stand).

## Rules the next lead keeps
- Standing permission (Luis, 2026-10-15): commit and push each unit that passed review and `ci/dev_check.sh`; merge
  into `master` after `ci/local_ci.sh` passes. Loosening a test, spec changes and tags stay Luis's. Fix forward.
- Never run a dev check while coders build (a timing test times out under load). Read the check's result before
  pushing. Flashing during M4: permitted by Luis (2026-10-17). No ESC power or motor spin without Luis present.
