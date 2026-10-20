GX-16 (console) evidence. The board's console is not opened here: a real board is the bench check, not an e2e.

- light/: Playwright (tools/gs/e2e.sh -- console.spec.ts), spec gcs/e2e/console.spec.ts, against the fake console of tools/gs/e2e_fake_console.py (a pty
  linked as an -if00 entry of the by-id directory: it prints a banner and a line a second, and answers each line it gets with the bytes it
  received). No port of the host was opened. report.json: 2 passed.
  - console-1-closed.png: closed, the input is off and Open is on (control).
  - console-2-open.png: open, the banner and the board's lines, the typed lines as sent (spaces and case kept, an empty line) and the board's
    answer to each.
  - console-3-closed.png: after Close the output stops (the same wait that saw new lines while open); the helper's console.log holds the
    board's banner and the lines sent.
  - console-4-held.png: with the board held for a HIL run Open is off and the helper's reason is shown; released, Open is on (control).
- Helper pytest: tests/tools/test_gs_console.py. Page logic: gcs/src/lib/console/console_state.test.ts.
