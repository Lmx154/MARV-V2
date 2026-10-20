GX-9 (the SIL pilot bridge) evidence.

- light/: tools/gs/e2e.sh (light mode), spec gcs/e2e/pilot.spec.ts with the fake joystick of tools/gs/e2e_fake_joystick.py. report.json is the
  whole light run: 14 passed, 2 skipped (the live-only specs). pilot-1-run-with-pilot.png: a run with the Pilot option, "Open joystick" enabled;
  pilot-2-open.png: the joystick open, state, frames sent and the 16 channels (ch5 at 1811, ch1 sweeping); pilot-3-closed.png: after Close, no
  frames; pilot-4-run-stopped.png: stopping the run closed the joystick.
- BENCH.md: the runbook for Luis's RadioMaster check (not run by an agent). The verdict goes in VERDICT.md beside it.
