# GX-7 Radio page: evidence and what is still UNKNOWN

`light/` holds the light e2e of `gcs/e2e/radio.spec.ts` against the fake vehicle (`tools/gs/e2e.sh`): its Playwright JSON report and
screenshots. The fake vehicle is a harness, not the firmware: it streams the RC channels (channel 1 swings, channel 14 is a switch),
serves the register's channel map and refuses a colliding channel (rule R-1) as the composition validate does. It shows that the page
follows the stream, derives the mapping from the parameters, points out an unmapped switch, finds it, warns before staging and shows the
vehicle's refusal. It shows nothing about the real receiver decode or about a remap changing which channel moves a stick.

## UNKNOWN until GX-9 and Luis

1. SIL remap end to end (needs GX-9, the pilot bridge). UNKNOWN: that a remap staged on the Radio page, validated (G7) and activated, makes
   the SIL read the stick from the new channel. Steps: start the SIL with the pilot bridge and the RadioMaster as the joystick; on the
   Radio page note which channel moves for the angle switch; press Detect on "Angle switch", move the switch, Use channel, give a source
   text, Stage; Validate on the staged panel (PASS for the pair), Activate; after the relaunch the Mapping table's "Running" column shows
   the new channel and the angle flag of the Vehicle page follows the switch. Control: before the remap the switch does not change the
   angle flag.
2. Bench with Luis's RadioMaster on the board (TB). UNKNOWN: the channel bars, detection and thresholds against the real receiver, the
   stick-centre scatter and the switch travel that would replace the two PLACEHOLDERs (`CENTRE_TOLERANCE`, `MOVED_US` in
   `gcs/src/lib/state/radio_state.ts`). Steps: connect the board over the link, open the Radio page with the transmitter on; with the
   sticks released and the throttle down record the "Display checks" and the Position of channels 1 to 4 for 30 s (the scatter in us);
   move each switch through its positions and record the Position (us) of its channel; press Detect on "Angle switch" and move the
   switch Luis sends on channel 14 (the page should highlight channel 14 and name it in the hint while the angle channel is 8); log the
   readings as the raw output of the page's table. Luis fixes the radio's CH8 mix, or stages channel 14 here and activates it.
3. The sign of each stick on the bars is the contract's x (roll, yaw as sent; the firmware negates pitch): the page shows the channel,
   not the control's sign. Bench per stick, as `rc_input.hpp` says.
