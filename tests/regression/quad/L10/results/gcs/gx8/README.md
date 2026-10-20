GX-8 (the sim channel) Playwright evidence: tools/gs/e2e.sh (light) and tools/gs/e2e.sh --live, specs gcs/e2e/sim.spec.ts.

- live/: the helper started the Gazebo SIL (GUI off), the plugin published the sim channel. sim-1-present.png: the SIM page with its
  panels while the run publishes; sim-2-stopped.png: the run's processes stopped (SIGSTOP), the panels removed, the page says no
  simulation is publishing; sim-3-continued.png: continued, back; sim-4-run-stopped.png: after Stop on the Simulation page the SIM entry is
  gone from the navigation. report.json is the whole live run: 10 passed, 1 failed, 1 skipped; the failure is
  connection.spec.ts "closing the device returns the status bar to no device and no heartbeat" in live mode (its "Start SIL run" button is
  disabled at the point it clicks it: the run of the spec before it is still going), a spec of GX-0 that this unit does not touch and
  that the GX-5 live evidence did not include (only vehicle.spec.ts was run there).
- light/: the fake simulator of tools/gs/e2e_fake_sim.py (a SIL record at 20 Hz). The run of report.json: connection, sim, simulation and
  vehicle specs, 10 passed, 2 skipped.
