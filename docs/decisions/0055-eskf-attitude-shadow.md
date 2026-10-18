# 0055: An error-state Kalman filter for attitude and gyro bias, in shadow, before first flight

## What changed

Quad §4.0 M4 gains (Luis, 2026-10-18): an error-state Kalman filter (ESKF) estimating attitude and gyro bias from the IMU
alone, following the error-state structure of PX4's EKF2 (BSD-3; quad spec's navigation-filter section) reduced to these
states, designed so magnetometer, GNSS and barometer updates plug in later (their drivers do not exist yet). It runs in
**shadow**: Mahony (decision 0042) keeps flying; the ESKF is computed alongside, logged and compared (SIL, Gazebo, replay of
the committed bench logs, quad L7 T2 data). A parameter can select the ESKF as the controlling estimator in SIL only.
Switching the board to the ESKF, and whether the board runs it at all (its CPU cost against EMB-3, measured U = 0.699), are
Luis's decisions after the comparison. The B2 24-state navigation filter stays PARKED.

## Why

Luis, 2026-10-18: "use any time to polish the gui and implement other estimators like the ESKF". His answers to the lead's
two questions: "Shadow first (Recommended)" and "Attitude + gyro bias (Recommended)".

## Evidence

docs/spec/10-quad-flight-software.md (estimator rows, the EKF2 structure note); L09/results/m4-bench-card.md (EMB-3).

## Approval

Luis, 2026-10-18 (conversation), quoted above. Every number the filter needs (process and measurement noise) is sourced
from the sensor profile or labelled PLACEHOLDER (CLAUDE.md number rule); none is a safety number while it only shadows.
