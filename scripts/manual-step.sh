#!/usr/bin/env bash
# Replays a pilot into `marv_ground manual --js FIFO`: js_event records on a FIFO, timed in sim seconds from the moment
# marv_ground opens it.
#
#   scripts/manual-step.sh [--scale S] SEQUENCE FIFO
#
#   step PROFILE   RadioMaster, CH6 on PROFILE (hold|freestyle|stabilized|agile) from the start: CH5 on at 14 s with the
#                  throttle at the bottom (arm), throttle full up 17-20 s, full forward 25-31 s, release, end at 43 s.
#   ch6            RadioMaster, CH6 on hold: arm at 14 s, climb 17-20 s, CH6 to freestyle 24 s, stabilized 28 s,
#                  agile 32 s, hold 36 s, end at 40 s.
#   xbox           Xbox: A (arm) at 14 s, left stick up 15-18 s, Y (stabilized) 20 s, LB (freestyle) 23 s,
#                  RB (agile) 26 s, X (hold) 29 s, right stick forward 31-35 s, release, end at 42 s.
#
# marv_ground picks its mapping by the device name, which for a FIFO is its path: the RadioMaster sequences need a path
# containing "radiomaster", xbox one without. The FIFO is created if missing. Closing it at the end is marv_ground's
# joystick loss: it sends centred sticks (hold) and exits.
#
# --scale S: sim seconds per wall second (default 1); the flight controller in the loop runs at about 0.67.
set -euo pipefail

scale=1
if [[ ${1:-} == --scale ]]; then
    scale=$2
    shift 2
fi
[[ $# -ge 2 ]] || { sed -n '2,19p' "$0" >&2; exit 2; }
seq=$1
fifo=${!#}
case $seq in
    step) [[ $# -eq 3 ]] || { echo "manual-step.sh: step needs a profile" >&2; exit 2; }; profile=$2 ;;
    ch6|xbox) [[ $# -eq 2 ]] || { echo "manual-step.sh: $seq takes no profile" >&2; exit 2; }; profile=hold ;;
    *) echo "manual-step.sh: unknown sequence $seq" >&2; exit 2 ;;
esac
radio=0
[[ ${fifo,,} == *radiomaster* ]] && radio=1
if [[ $seq == xbox && $radio == 1 ]] || [[ $seq != xbox && $radio == 0 ]]; then
    echo "manual-step.sh: $seq needs a FIFO path $([[ $seq == xbox ]] && echo without || echo with) \"radiomaster\"" >&2
    exit 2
fi
[[ -p $fifo ]] || mkfifo "$fifo"

exec python3 - "$seq" "$profile" "$fifo" "$scale" <<'EOF'
import os, struct, sys, time
seq, profile, path, scale = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
BUTTON, AXIS, INIT = 1, 2, 0x80
# CH6 mid-band values (bands edged at -16384, 0, +16384, ground/src/pilot.hpp ch6_profile).
CH6 = {'hold': -32767, 'freestyle': -10923, 'stabilized': 10923, 'agile': 32767}
if profile not in CH6: sys.exit('manual-step.sh: unknown profile ' + profile)
f = os.open(path, os.O_WRONLY)  # blocks until marv_ground opens it
t0 = time.time()
def ev(v, typ, n): os.write(f, struct.pack('<IhBB', int((time.time() - t0) * 1000) & 0xFFFFFFFF, v, typ, n))
def at(t): time.sleep(max(0.0, t0 + t / scale - time.time()))
def press(n): ev(1, BUTTON, n); ev(0, BUTTON, n)
if seq in ('step', 'ch6'):
    # The user's radio at rest with the throttle at the bottom; CH6 on the profile.
    init = [0, 0, -32767, 0, -32767, CH6[profile], -32767, -32767]
    for n in range(8): ev(init[n], AXIS | INIT, n)
    for n in range(24): ev(0, BUTTON | INIT, n)
    at(14); ev(32767, AXIS, 4)   # CH5 on, throttle bottom: arm
    at(17); ev(32767, AXIS, 2)   # throttle full up
    at(20); ev(0, AXIS, 2)       # centre: hold height
    if seq == 'step':
        at(25); ev(32767, AXIS, 1)   # elevator full forward
        at(31); ev(0, AXIS, 1)       # release
        at(43)
    else:
        for t, p in ((24, 'freestyle'), (28, 'stabilized'), (32, 'agile'), (36, 'hold')):
            at(t); ev(CH6[p], AXIS, 5)
        at(40)
else:
    init = [0, 0, -32767, 0, 0, -32767, 0, 0]  # xpad: triggers (a2, a5) rest at -32767
    for n in range(8): ev(init[n], AXIS | INIT, n)
    for n in range(11): ev(0, BUTTON | INIT, n)
    at(14); press(0)                 # A: arm
    at(15); ev(-32767, AXIS, 1)      # left stick up: climb
    at(18); ev(0, AXIS, 1)
    for t, b in ((20, 3), (23, 4), (26, 5), (29, 2)):  # Y stabilized, LB freestyle, RB agile, X hold
        at(t); press(b)
    at(31); ev(-32767, AXIS, 4)      # right stick up: forward
    at(35); ev(0, AXIS, 4)
    at(42)
os.close(f)
EOF
