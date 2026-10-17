# 0044: M4 P1: the flight parameter set takes the real 7-inch card

## What changed

1. `marv_params_fc3v2` (the target image's parameter set) is built from `vehicles/marv_5in_fc3v2.yaml`, which now carries
   the real airframe (Luis's measured mass and motor positions; sourced thrust, torque, motor-lag and inertia estimates,
   INFERRED/UNVERIFIED/PLACEHOLDER until the M4 stand and pendulum), instead of the UZH 5-inch card. The SIL's
   `MARV_VEHICLE_CARD` stays the UZH card. The board profile `sensors/profiles/madflight_fc3v2_revc.yaml` gains the entries
   the design tools need (datasheet, INFERRED or PLACEHOLDER, each sourced in the file).
2. `tools/card/flatten.py`: a refusal of the report-only attitude comparator (`attitude.design`, the "today" column) is
   reported as "not computed: <reason>" instead of aborting the build. The stability test, every threshold and the
   flight rule `attitude_lead.design` are unchanged.
3. `tests/regression/quad/L09/unit/quad_fc_gate_test.cpp`, `ASwitchCycleArmsAndThenTheMixerOutputForTheSticksIsWrittenNotStop`:
   the scenario stick vector `fly` goes from (0.5, −0.25, 0.75, 0.5) to (0.5, 0, 0.75, 0.5). No assertion changes.

## Why

Quad spec §4.0 M4 (amended, Luis 2026-10-17: "1. OK"): the card carries the real airframe and the gains are regenerated
from it. On the real card's rectangle (motors at x ±0.112, y ±0.1275 m) the mixer's roll and pitch entries are ±1.9608
and ±2.2321, so for the old stick vector the roll and pitch terms cancel on motors 1 and 2 (both counter-clockwise,
diagonal) to within one DShot count, and the test's `motor[0] != motor[1]` was decided by rounding. With roll only the
two motors differ by twice the roll term on any geometry. The comparator refusal is numerical (the closed-loop
polynomial's p(1) is about 16 double epsilons at the scan start) and the column is informational.

## Evidence

QF-3 on the real card, from the build's reports: rate PM 45.0024° (min 45°), Ms 1.99947 (max 2.0); attitude PM 45.00007°;
vertical PM 45.000003°, Ms 1.341. Regenerated gains and the profile entries: M4 P1 commits b41aa56..f5712f7.

## Approval

Luis, 2026-10-17, approved the order ("1. OK"); the stick-vector change loosens nothing (the predicate is unchanged).
`idle_speed` is now 0 (from his speed_range ruling's minimum); the armed zero-throttle output stays DShot 48, the same
floor as the M2 image (mixer.hpp dshot_floor), so the motors idle as on the M2 bench (that the ESC spins at 48 is
INFERRED from the M2 bench, not a committed test; QF-6 measures the minimum stable speed).
