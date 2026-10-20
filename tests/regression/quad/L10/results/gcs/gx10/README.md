GX-10 (HIL from the GCS) evidence. The live S4b run with Luis flying is not here: its steps are RUNBOOK.md.

- light/: Playwright (tools/gs/e2e.sh -- bench.spec.ts), specs gcs/e2e/bench.spec.ts, against the fakes of tools/gs/e2e_fake_bench.py (a Debug
  Probe and a board showing the HIL image's descriptors on a fake sysfs, a pty for the board's port, a HIL build tree, a flash.sh stand-in; the
  HIL run is a stand-in `sleep`). No port of the host was opened. report.json: 2 passed.
  - bench-1-refused.png: preflight with the ESC acknowledgement unticked and autoconnect not suspended: those two checks name their reasons,
    the other four are ok.
  - bench-2-allowed.png: after the hold and the acknowledgement all six checks are ok.
  - bench-3-running.png: the launched run (stand-in), Launch off, the verdict form off while it goes.
  - bench-4-verdict.png: after Stop (no process left) the verdict saved: mode ANGLE and text, in out/verdict.json beside the report.
  - bench-5-motor-refused.png / bench-6-flashed.png: bench-profile and bench-log refused naming the TB and presence condition (the script never
    ran); the hil kind flashed (control).
- Helper pytest: tests/tools/test_gs_hil.py. Plugin port lock: tests/regression/quad/L10/gz/test_hil_port_lock.py (the helper holds the port and the
  plugin is refused; the plugin holds it and the helper and a plain flock are refused).
