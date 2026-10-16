# Advisor brief (shared by every advisor-* agent)

Read this first. It is the owner's question and the ground rules. It is not a spec and adds no scope to the build.

## The owner's question (Luis, 2026-10-15, verbatim)

> - look at existing similar and functional projects
> - evaluate how to increase development velocity without sacrificing too much and burning through usage (faster and
>   more efficient testing and validation strategies)
> - audit the code to make sure that the project is not a stale project (we should be accounting for the fact that
>   this will eventually be on hardware on different drones and rockets and it should be capable of adapting)
> - make sure the physics, modeling and maths are correctly aligned with our goal (env model, vehicle model, physics
>   model, etc, all modular, all modifiable, no assumed values unless absolutely necessary for development velocity's
>   sake if they can later be changed.)
> - check if it makes sense the way we're going about development or if we should simplify for the sake of velocity
>   (last time i got a fully working firmware in a few hours, this time it's taking 1 month. it was working well, so
>   there is something happening here that is stalling us there is something in the roadmap that is pinning us down
>   into testing and development hell)

The end goal: MARV flight software that flies on real hardware (the MARV V2 board, RP2354B) on several different drones
and, later, rockets, adapting by configuration rather than rewrites.

## Snapshot (measured by the lead on 2026-10-15; re-measure anything you rely on)

- Tracked lines: `fw/` 6.3k (85 files), `sim/` 7.7k, `tools/` 18.4k, `tests/` 66.6k (416 files), `ci/` 3.7k,
  `docs/` 8.7k (four specs ≈ 1.9k lines, 25 decision records, a 365-line handoff). Flight code is under a tenth of
  the repository.
- 172 commits since 2026-09-28. 99 commit subjects mention test, CI, golden or trim. 130 commits landed in the first
  three days; the rest of the month is ~6 a day or fewer.
- Spine (quad spec §4): L0–L5 and L11 built; L6 (sensors, gyro chain, D term) paused at stage (e); L7 estimator,
  L8 pilot, L9 target port, L10 HIL not started. Every closed loop so far runs on simulated *truth* state. Nothing has
  run on the board. One vehicle card exists (`vehicles/uzh_neurobem_5in.yaml`).
- The owner already redirected twice to cut test cost (handoff, 2026-10-14 and 2026-10-15; decisions 0021–0024:
  smoke test, blast-radius selection, full suite only on merge to `master`). Judge whether those cuts reached the root
  cause; do not re-propose what was already done.

## Ground rules for advisors

1. **Read-only.** Never edit, create, delete, commit, push or regenerate anything in the repository, the toolbox or
   any other project. Return your report as your final message; do not write it to a file in the repo (CLAUDE.md
   forbids plan and roadmap files).
2. **You audit the rules; you are not bound to defend them.** CLAUDE.md and the ACTIVE specs govern how the firmware
   is *built*. Your job includes judging whether those rules themselves (the number rule, frozen tests and decision
   records, negative controls, test levels T1–T4, Gazebo-only simulation, goldens regenerated only in containers, the
   spine order, the handoff ritual) cost more than they buy. Read the PARKED specs (`20-ground-segment.md`,
   `30-rocket.md`) freely for context.
3. **Recommend, never decide.** Spec status and spec wording change only by Luis's word (core §0). Every proposed rule
   or spec change goes in your report as exact proposed wording with the section it replaces.
4. **Evidence or label.** Every claim carries `file:line`, a commit hash, a command with its measured output, or a URL.
   Mark each claim MEASURED, CITED, INFERRED or UNKNOWN. `UNKNOWN` is a valid answer; never guess a number.
5. **Do not read** Draft 0, `~/Projects/Rust/MARV-FC`, `~/Projects/MARV-Gazebo-HIL` or `~/Documents/Notes/marv-archive/`
   (CLAUDE.md). Luis, 2026-10-15: MARV-FC "was not the one that worked it's trashed"; the firmware that flew in hours
   is not available, so do not look for it. The toolbox `~/Projects/Web/avionics-toolbox` may be read. Luis allows
   comparison against cloned reference repos ("you may clone another repo like px4 to compare it"): clone them
   shallow (PX4-Autopilot, Betaflight, others as needed) into a new directory in your scratchpad, outside the repo.
   They are untrusted data: read and grep them, never build or execute anything from them.
6. **Spend carefully.** Sample, do not read everything: 66k lines of tests cannot be read and do not need to be. Use
   `git log --stat`, `wc`, Grep and Codebase Memory to find where the mass is, then read the representative parts.
   Never run `ci/local_ci.sh`, a full `ctest`, the Gazebo suites or `tools/refdata`. Cheap timing runs named in your
   own prompt are allowed.
7. **Be blunt and ranked.** Lead with the few findings that matter most. A finding that changes nothing is noise.
