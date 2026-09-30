# 0007: L2 initial rotor speed (additive interface change)

## What changed

**Frozen files changed.** None. `tests/regression/quad/L0[0-4]/` is untouched; the new tests are under
`tests/regression/quad/L05/` (`unit/plant_initial_rate/`, `tools/test_initial_rotor_speed.py`).

**Interfaces changed (all additive, default 0 = behaviour as before).**
- `marv_plant_config` (`sim/plant/include/marv_plant.h`) gains `double initial_omega_rad_s[MARV_PLANT_N_MOTORS]`, the
  rotor speed at the first step, logical motor order.
  - Placement: appended last, after `rng_seed`. `marv_plant_create` checks `struct_size == sizeof(marv_plant_config)`
    exactly, so the check stays coherent: a caller built against the older header passes the older size and is refused
    with `MARV_PLANT_E_ABI`; it is never read past its struct. Every caller in the repository builds against this header
    and value-initialises the config (`marv_plant_config c{}`), so the new field is 0 for all of them.
  - Validation: each entry finite and in `[0, omega_max_rad_s]`, else `MARV_PLANT_E_CONFIG` (the existing code for an
    invalid config value; no new error code). The bounds are closed; -0.0 is accepted.
  - `Model` starts `omega_` from it. Zero gives the old zero-initialised state, bit for bit.
- Gazebo plugin: an optional `<initial_rotor_speed_rad_s>w1 w2 w3 w4</initial_rotor_speed_rad_s>` (rad/s, logical
  motor order). Absent: all zeros, and the plugin behaves as before.
- L2 scenario schema (`tools/sim/scenario.py`): an optional `initial_state.rotor_speed_rad_s` (4 numbers >= 0). The L5
  schema (`tools/sim/l5_scenario.py`) accepts the same, or the text `hover` with label `derived`. `gen_world.py` writes
  the element only when the field is present. The L4 runner passes the field through when a document has it (the L4
  schema does not admit it yet); the L5 runner resolves `hover` and passes it through.
- Hover rotor speed (`run_l5.hover_rotor_speeds`): `omega_i = sqrt(T_i / k)`, `T_i = M[i, thrust] m g(phi, h0)`, from
  `run_l4.hover_thrust`, the firmware mixer matrix `M = B^-1` (`tools/card/mixer.py`, pure collective = zero torque) and
  the card's `k`. It is refused if some `omega_i` lies outside the card's speed range.

## Why

Owner decision (Luis, verbatim): "Spin-up: Option 2. Add an initial rotor speed to marv_plant_config and the scenario,
defaulting to 0 so all existing scenarios and frozen goldens stay bit-identical (prove it: full L00–L04 suites green,
goldens unchanged). Recovery scenarios start at the card's hover rotor speed. Decision record for the additive L2
interface change." Recovery scenarios (L5) would otherwise spend the start in the motor spin-up from rest.

## Evidence

The no-change claim is proved by the L00–L04 suites (host-debug, host-release, L02 and L04 gz suites, the Python tool
tests), the golden reproduce steps of `ci/run_ci.sh`, a byte-compare of generated L2, L4 and L5 worlds before and after,
and an empty `git diff --stat master... -- tests/regression/quad/L0[0-4]`. Commands and results are in the hand-off of
the change that adds this record.

## Approval

Luis's owner decision above; approval of the pull request, linked.
