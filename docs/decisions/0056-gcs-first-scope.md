# 0056: Ground segment ACTIVE scope widened to GCS-first

## What changed

The scope of decision 0051 (params-first) is widened so the whole suite is usable from the ground control software:

- (a) `docs/spec/20-ground-segment.md` status line: G2 without the mission planner (connection manager, telemetry,
  recording and replay, the GCS driving Gazebo SIL and HIL, a pilot input for SIL only), G5 without the S0–S2 wizard
  (bench tools, the console motor test of decision 0038), IF-7 log download over USB. Still PARKED: the mission planner
  (IF-4), G4, IF-8 and IF-6 to a real vehicle, the S0–S2 wizard, the rocket.
- (b) ground §3: controller passthrough is the SIL pilot bridge (the helper reads the USB joystick; the SIL decodes
  its channel frames with the board's own function and parameters). It replaces the quad spine's L8 joystick bridge and
  the browser Gamepad API wording.
- (c) ground §3: a lock changes only in its source entry with Luis's verbatim words and the date, never over the link;
  the commit carries `lock ruling: <name>: Luis <YYYY-MM-DD>: "<words>"`, which CI checks. Removing a lock or changing a
  locked safety value also needs a record.
- (d) ground §1: bench images (HIL, bench log, bench profile) never fly, are flashed over SWD outside the G7 gate, and a
  motor-capable one is flashed from the GCS only after its props-off bench test on that exact image and with Luis's
  presence acknowledgement.
- (e) ground §5: new row GX (GCS-first) with its exit test and control.
- (f) core §0 table, ground row; (j) core §4, the lock-ruling sentence.
- (g) `CLAUDE.md`: the ACTIVE-spec rule names the GCS-first scope; the MVP direction gains Luis's 2026-10-18 line.
- (h), (i) `docs/spec/10-quad-flight-software.md`: §4.0 deferred list and L8 Opening say the joystick bridge is the GCS's
  SIL pilot bridge; IF-8 to a real vehicle comes later behind the same struct.

## Why

Luis, 2026-10-18: "i don't need to fly right now. instead i need you to start another task and it is to make it so this
entire suite is accessible and usable via the ground control software ui. I want to be able to modify the sim
parameters, launch the sim, see the sim sensor vals, remap my radiomaster, and more. it seems like the ui is a second
class citizen right now."

"I would like to be able to use a button within the ui to open up the entire sim and also use the ui to see all the
realtime stats of the drone. like the virtual sensor values and stuff stream all of that."

"just like how it would work on a real flight. this ui is supposed to be our ground control software"

## Evidence

Architect ruling `ground-segment/architect-gcs-first-class-2026-10-18` (Serena memory): architecture (browser, helper,
sim channel), the GX units (GX-0 to GX-18) with done-when lines, 14 invariants with guards, rejected alternatives. Facts it
rests on, each with file:line in the ruling: the SIL has no live manual-control source; stick remaps have no effect in
SIL; the validator does not check stick channels against the arm channel; SIL overrides reach the staged table.

## Approval

Luis, 2026-10-18, AskUserQuestion answers: scope "Approve (Recommended)" (the widening, and priority: GCS-first after
logger U1–U3, which keep their build slots; hardware contact still wins build slots); SIL pilot bridge with the
RadioMaster as USB joystick "Approve (Recommended)"; lock procedure "Approve as proposed (Recommended)"; operating rules
"Approve all four (Recommended)": GCS-triggered builds under the heavy lock, HIL preflight and bench images by SWD (a
motor-capable bench image from the GCS only after its props-off TB), log retrieval by card pull now and IF-7 later, SIL
overrides may override locked values (labelled, never reaching a board).
