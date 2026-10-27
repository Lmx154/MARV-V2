# Stick rest offsets, RadioMaster Pocket over ELRS (2026-10-27)

Bench evidence for the `yaw_deadband` choice. Board FC3v2 on the flight image built from 3531c3f (banner
`quad-L2-pass-664-g3531c3f`), ESC unpowered, props off, ch5 off (`arming=switch_off` throughout). Capture:
`tools/target/readcon.py <board if00> 120` (one status line per second; a fast stick flick between two lines is not seen).

Result (`python3 analyze.py console.log`, 120 samples, all at rest): roll -0.006..0.000, pitch 0.000, throttle
+0.001..+0.006, yaw -0.059..-0.040. Luis: "yaw stick has some play so it stays at that. all the sticks have a little bit
of play". Proposed `yaw_deadband` 0.07 (stick fraction): above the largest yaw rest reading, 0.059.
Not covered: the play after a deflection in each direction (the 1 Hz lines caught no deflection); no roll or pitch
deadband exists in the firmware (angle mode has the yaw one only, `fw/attitude/include/marv/attitude/angle_mode.hpp:86`).
