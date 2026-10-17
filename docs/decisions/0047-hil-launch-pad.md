# 0047: HIL launch pad: a held start until lift-off, not ground contact

## What changed

The gz plugin gains an opt-in `<launch_pad>` element. From tick 0 the body is held at its start pose (zero velocity,
zero applied wrench) and the plant's contact force (marv_plant_contact_set, decision 0042) is −(the plant's total
force including gravity) for the rotor state the tick's IMU sample sees, so the IMU reads R^T(0, 0, −g(h)) and the gyro
0. Release is the first tick on which that force's NED-down component is ≥ 0 (the pad would pull: vertical thrust
reaches the weight); afterwards the contact force is exactly 0 and the hold never re-engages. Worlds without the element
are byte-identical. The run report states "launch pad hold until tick k; not ground contact; no landing" and that the
pad resists torques and lateral forces. The L10 plumbing test drops its estimator_accel_healthy_min = 0 and accel-bias
workaround. No frozen test changes.

## Why

HIL S4 must rehearse boot gyro calibration, estimator validity, the upright check, arming and lift-off (decision 0046),
which need a still vehicle reading −g. Gazebo ground contact stays deferred (quad §4.0 deferred list, Luis 2026-10-16):
no ground plane, collision or landing is added.

## Evidence

sim/plant (marv_plant_contact_set, M3 P5); tests/regression/quad/L10/gz/test_quad_fc_plumbing.py (the workaround it
replaces); docs/decisions/0042 item 3; docs/decisions/0046.

## Approval

Within M4's HIL scope (Luis, 2026-10-17, "2. yes"); Luis informed that the launch pad is not the deferred ground contact.
