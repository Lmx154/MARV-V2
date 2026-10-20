# GX-9 bench check: the RadioMaster as the SIL pilot (Luis flies; not run by an agent)

Not yet done. The verdict goes in `VERDICT.md` beside this file (below). Until then the EdgeTX channel to joystick-axis layout and the
joydev permissions are UNKNOWN: the helper assumes channel n is axis n-1 (tools/gs/pilot.py), a PLACEHOLDER.

## 1. Plug in
1. Radio: in EdgeTX set the USB mode to *Joystick (HID)* (SYS, Hardware / USB Mode, or choose it at the USB prompt), then connect the
   RadioMaster to this host. `ls -l /dev/input/js*` must list a `js<N>` node (`jstest` is not needed).
2. Permissions: the helper opens `/dev/input/js<N>` read-only. If `cat /dev/input/js0 >/dev/null` says "Permission denied", the user is not
   in the group that owns the node (`ls -l` shows it, usually `input`): add it (`sudo usermod -aG input $USER`) and log in again. The
   GCS shows the same refusal in the Pilot panel ("permission denied").

## 2. Log the js events per channel at plus and minus 100 %
Run, in a terminal, one line per event with the axis number and value (the joystick API's struct js_event is 8 bytes: time ms u32,
value s16, type u8, number u8; type 2 is an axis, +0x80 is the burst at open):

    python3 - <<'PY' | tee build/gx9-js-events.log
    import struct, sys
    f = open(sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0", "rb", buffering=0)
    while True:
        t, v, kind, n = struct.unpack("<IhBB", f.read(8))
        if kind & 0x7F == 2:
            print(f"t={t} ms axis={n} value={v}", flush=True)
    PY

Move one channel at a time through -100 %, 0, +100 % (EdgeTX Monitor shows the channel) and write down which axis moves and whether it
ends at -32767 / +32767:

| EdgeTX channel (RadioMaster) | js axis number | value at -100 % | value at +100 % | note |
| --- | --- | --- | --- | --- |
| CH1 (aileron) | | | | |
| CH2 (elevator) | | | | |
| CH3 (throttle) | | | | |
| CH4 (rudder) | | | | |
| CH5 (arm, ch5) | | | | |
| CH8 (angle) | | | | |
| any other channel that moves | | | | |

The helper maps axis value v (-32767 .. 32767) to channel ticks 992 + round(v / 32767 * 819.2) (the CRSF -100 % .. +100 % range, 173 .. 1811).
If a channel does not come out at the axis the table lists, give the helper the layout: `axis_map` in the Pilot panel is 16 numbers, one per
channel, the js axis that carries it (`-` for none), for example `0 1 2 3 4 5 6 7 - - - - - - - -`.

## 3. Fly the SIL
1. `uv run python tools/gs/helper.py`, open the printed address, Simulation page: tick *Pilot*, Launch (the build `host-gz-quadfc` must exist
   and no heavy job may hold the lock), choose the joystick in the Pilot panel, *Open joystick*. The Pilot panel shows the 16 channels.
2. With ch5 low the vehicle stays disarmed; ch5 high with the throttle down arms it; raise the throttle to lift off the pad. Check each stick
   against what the SIM page shows (the sign convention is the board's: rc_input.hpp, INFERRED, checked here per stick): roll right = right
   side down, pitch forward = nose down, yaw right = nose right.
3. *Close joystick* (or unplug the radio): the frames stop at once and the vehicle's link-loss failsafe acts (the plugin stages nothing more).
4. A remap test (R-2): in the Parameters page stage `rc_channel_roll` and `rc_channel_pitch` swapped (2 and 1), validate, activate in the
   SIL; the roll stick must now move the pitch (the same decode as on the board).

## 4. Where the verdict goes
Write `tests/regression/quad/L10/results/gcs/gx9/VERDICT.md`: the date, the radio and EdgeTX version, the table of section 2 filled in,
the `axis_map` used, the permission fix applied, and for each stick and the arm switch pass or fail with a sentence, and commit it with the
raw `build/gx9-js-events.log` copied beside it. A layout that differs from the default is then written into the helper as its default
(tools/gs/pilot.py) in a change of its own; nothing here edits a number in code.
