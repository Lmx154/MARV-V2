# Handoff: MVP realignment (decision 0026); next M0 beside M1

2026-10-15. A state file, not a history: at most 80 lines, rewritten (not appended) at every milestone boundary.
History is in git and `docs/decisions/`; the previous handoff is `git show 341e54d:docs/handoff.md`.

## Read first
- `CLAUDE.md`, then quad spec §4.0 (the MVP build order, M0–M6, and the deferred list), then the milestone you execute.
- Nothing else by default. Decisions 0001–0025 are background, read only when a task touches their subject.

## Where things are
| Item | State |
| --- | --- |
| L0–L5 | passed, tagged `quad-L0-pass` … `quad-L5-pass` |
| L11 vertical loop (truth state) | passed, tagged `quad-L11-pass` (`341e54d`) |
| L6 sensors, gyro chain, D term | stages (a)–(d) closed; the rest deferred until after M4 |
| Known failing items | L4 acro under noise, L5 R2, L5 R1X α: strict xfail, owned by the deferred L6 (e) |
| M0 fast loop and test rules | not started |
| M1 target bring-up | not started; nothing has run on the board; the `m33` preset links no image |
| M2–M6 | not started |

## The realignment (merged into `master` at `6308a8e`; local merge gate PASS: core 695 s, gz-l2 139, gz-l4 218, gz-l5 555, gz-l11 116)
- Decision 0026: the advisor findings, Luis's acceptance and realignment words, verbatim. M2 receiver: ELRS over CRSF
  with Betaflight 4.5's link-loss failsafe (Luis). M4 includes building or buying a thrust stand; pack 4S 1600 mAh.
- Core spec §2 (`PLACEHOLDER`, the UNKNOWN split), §7.1 (TB bench level, T4 at merge, T6 deferred), §7.2 items 3 and
  6 (controls only where a threshold decides; freeze requirements, not outputs), §7.3 (records only to loosen,
  delete or change a spec; the `golden regenerated:` commit line), §7.4.
- Quad spec §0, §1, §4.0 (new: MVP order), §4.1 (the layers, unchanged apart from MVP-subset notes), §6.1 (minimal
  arming), §6.3 (B1 after M4).
- `CLAUDE.md` rewritten for MVP first; this handoff; `ci/blast_radius.yaml` ignores `.claude/**`.
- `.claude/agents/advisor-*.md`, `project-advisor.md`, `.claude/advisors/brief.md`: the four advisors and the synthesizer
  (Luis: "commit the advisor files"); run them when the direction needs re-auditing.

## Next
1. **M0** (beside M1, one packet per bullet in §4.0 M0). First measure the baselines: an Actions full run and a
   one-module development check. Fold the Actions reference-cache fix (the L11 set regenerates cold every run; core
   45 min on `341e54d`, INFERRED cause) into M0.
2. **M1** on the madflight FC3v2 Rev-C (branch `m1-sdk`). Hardware verified 2026-10-16: RP2354B rev 3, 2048 KiB
   W25Q16JV in package, clk_sys 150 MHz, clk_usb 48 MHz; SWD flashing (`tools/target/flash.sh`, Pico 2 debugprobe
   900678EF479C3A0E, GP2 SWCLK / GP3 SWDIO) and the USB console work; a panic streams "PANIC: <msg>" over USB (P5).
   Next packets (architect plan, 2026-10-16): P6 `quad_fc` composition with an always-disarmed motor gate, replacing
   the L11 proxy; P7 timing-fault rule (counted fault, SIL tolerance unchanged; record + Luis); P8 LSM6DSV profile and
   the airframe card (physics PLACEHOLDER until M4); P9 LSM6DSV driver and IRQ tick loop; P10 PIO DShot600; P11
   console motor test; P12 M1 evidence (stage times, EMB-3, soak).
   Luis's rulings, 2026-10-16, verbatim: panic motors "take option A" (panic keeps sending DShot stop frames);
   motor test "1. 500 2. 3 seconds 3. yes 4. yes 5. yes 6. yes" = DShot cap 500, at most 3 s per command, unlock
   token `PROPS-OFF` on the command line (any byte, `stop` or disconnect cancels), board timing tolerance
   T_rate·odr_error + 2δ as a PLACEHOLDER until P12, PX4 quad-X spin (1, 2 CCW; 3, 4 CW), tick 7.68 kHz with
   divisor 2. AM32 version and settings UNKNOWN (no passthrough yet; M4).

## Open for Luis
- Branches merged into `master` and deletable: `ci-curl-pin`, `ci-heavy-serial`, `quad-l11`, `quad-l11-s6`,
  `recovery/hover-demo`, `test-blast-radius`, `test-trims`, `quad-l6-spec`. `l6-c6a-wip` (`14e3c3a`, unmerged, the
  half-built L6 (e) reference set) waits for the L6 resume or is dropped.

## Known defects the advisors found (fix inside the milestone named)
- M1: the rate loop and attitude law halt on a sample spacing off by more than 1 µs, and every composition halts
  unless `hal_time_us()` equals the sample stamp (`rate_loop.hpp:52,230-241`; `l5_attitude_scripted.cpp:194`).
- M3: `marv_plant` reports zero specific force at rest (`marv_plant.cpp:255-264`).
- M4: rotor inertia is absent from plant and design model (yaw zero near crossover, INFERRED); the rate loop crosses
  over at about 8–12 rad/s because of the ±50 % inertia and ±30 % motor-lag bands; measurement narrows them.
- M5: four motors are compile-time (`actuator.hpp:16`, `mixer.hpp:28`, both C ABIs, `tools/card`).
- Carried from L11: the card mass conflict (0.772 vs 0.752 kg) and `thrust_coeff` σ, settled by M4's measurements.

## Git and host
- `master` = `origin/master` = the realignment plus this handoff; `realign-mvp` is merged (deletable). Tags up to `quad-L11-pass`. One stash, `stash@{0}` (old quad-l0 WIP): not ours, leave it.
- `datasheets/` is present and gitignored. `~/marv-handover-2026-10-13/` holds the old demo kit, outside the repo.
- gz-sim 8.15 on PATH; `clang-tidy-18` 18.1.3; no `gh` CLI (the Actions API is readable with `curl`).
- Agent worktrees inside the checkout break `tests/regression/quad/L06/tools/test_world_sensors.py`; keep them out.

## Rules the next lead keeps
- Commit, push, tag and merge only on Luis's word. Fix forward; never rewrite pushed history.
- Verified: a commit by `ci/dev_check.sh`; a merge by `ci/local_ci.sh` on the exact commit (`--parallel 2` at most).
- Reports to Luis: results first, numbers with units; every question answerable in one word, with a recommendation.
- A tool a check needs is missing on the host: tell Luis at once; do not work around it.
- Luis's machine is shared: check `uptime` before any timing.
