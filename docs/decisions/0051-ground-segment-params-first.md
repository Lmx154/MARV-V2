# 0051: Ground segment ACTIVE, params-first scope

## What changed

`20-ground-segment.md` becomes ACTIVE for a params-first scope (core §0 table, the ground spec's status line, CLAUDE.md's
ACTIVE-spec rule): G0 (functional interface), G1 (MAVLink v2 codec and the MARV dialect; transports USB CDC and the SIL
socket; the radio UART later), G3 (configurator rendered from the registry manifest, flash, config files on SD and
internal flash; ends the core §4 bootstrap exception), and from G5 the parameter and provenance pages with live plots. The
rest of G2 (mission planner), G4 and the rest of G5 stay PARKED. Parameters travel over MAVLink and are stored on the
board's microSD card (inserted, Luis 2026-10-17).

## Why

Luis, 2026-10-17: "Development in the software is far ahead of the actual hardware being delivered for the real safe test
flight so we should just go ahead and make the ground station gui with all of its parameters. Be sure to use mavlink for
the parameters and store them in the SD card, but you may work it out as you see fit." Scope chosen from the lead's
options: "Params-first (Recommended)".

## Evidence

docs/spec/20-ground-segment.md §5 (phases G0-G5), §6 (GD-1 to GD-3); core §4 (parameter interface, bootstrap exception),
§8 (functional interface).

## Approval

Luis, 2026-10-17 (conversation), quoted above. The design (MAVLink parameter mapping, storage, validation) follows from an
architect ruling recorded separately.
