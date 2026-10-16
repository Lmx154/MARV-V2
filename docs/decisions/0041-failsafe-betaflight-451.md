# 0041: the link-loss failsafe follows Betaflight 4.5.1 exactly; re-arming after a failsafe needs the switch cycled

## What changed

Quad spec §4.0 M2's failsafe sentence is corrected to Betaflight 4.5.1's behaviour, read from its source: the last
stick values are held until 300 ms after the last valid frame (was "until 400 ms"); a link that returns before the
1.5 s disarm returns control at once (was "only once the link has been valid for 0.5 s"); after the disarm, arming is
possible again only after 0.5 s of valid link and with the arm switch cycled off and on. Detection at 100 ms, centred
sticks and minimum throttle after the hold, and Drop at 1.5 s are unchanged. Decision 0040's Q5.1 and Q5.2 are closed
by this record. No code or test exists yet for these numbers (M2 P1 implements them).

## Why

Luis asked for Betaflight's failsafes (2026-10-15, "yes keep betaflight failsafes"); the spec's paraphrase differed from
Betaflight 4.5.1 in two places (decision 0040 Q5). Source, tag 4.5.1 (lines ±2, read by a fetch tool):
`src/main/rx/rx.c:136,506` (100 ms loss), `rx.c:131` `MAX_INVALID_PULSE_TIME_MS` 300 and `rx.c:732-741` (hold, then
AUTO: centre and `rx_min_usec`), `flight/failsafe.c:68` (`failsafe_recovery_delay` 5 = 0.5 s), `:74` (`failsafe_delay`
15 = 1.5 s), `:77` (Drop), `:214-216` and `:389-390` (link down at 1.5 s, disarm). Stage-1 recovery is immediate because
the link state only goes down at the 1.5 s check (2026.6.2 `failsafe.c:162-164,198-202`, same logic). The arm-switch
cycle is Betaflight's usual arming practice (INFERRED from its arming flow, not read) and is ruled here on its own.

## Approval

Luis, 2026-10-16, to "1. Match Betaflight 4.5.1 exactly (300 ms hold, immediate recovery before 1.5 s)? 2. Require the
arm switch to be flipped off and on again after a failsafe disarm?": "1. yes 2. yes".
