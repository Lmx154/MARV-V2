#!/usr/bin/env bash
# Start a paused gz-sim server, fly the graded profile against it, stop it.
#
# Arguments are passed through to gz_fly, except --gui which is handled here:
#
#   --mode rate|angle     which loop is graded. Rate grades the measured body
#                         rate against a stick-commanded rate; Angle grades the
#                         ESTIMATED bank against a stick-commanded lean, and
#                         additionally grades the estimate against a gyro
#                         dead-reckoned truth.
#   --profile step|hold|hover|tune  a step to grade, level flight only, the
#                         height-hold hover profile that also LEARNS a hover
#                         collective in flight (core's HoverLearnConfig), or
#                         the phase 3 autotune identification run.
#   --seconds N           whole run, INCLUDING the one-second takeoff. The
#                         aircraft starts on the ground now -- that is the only
#                         place its accelerometer can see gravity, and Angle
#                         mode cannot fly until the estimator has primed.
#   --trace FILE          write marv-trace v11, then read it back and replay it.
#   --hover-out FILE      after the run, if a hover throttle was actually
#                         learned, write it to FILE. Writes nothing (and says
#                         so on stderr) if nothing was learned this run.
#   --hover-in FILE       read a previously learned hover throttle (from a
#                         prior --hover-out) and fly THIS run's hover
#                         collective from it, in place of gz_fly's compiled-in
#                         first-flight seed. Absent, empty, unparseable or
#                         out-of-bounds REFUSES the run (exit 2) rather than
#                         silently falling back to the seed. An explicit
#                         --throttle always wins over --hover-in.
#                         REQUIRED with --profile tune: the tune places its
#                         gains from an AUTHORITY BUDGET whose headroom is
#                         1 - this number (PLAN.md Q11), and 1 - the
#                         compiled-in seed would be a default that flies.
#                         Note --throttle still wins for what is FLOWN; the
#                         budget always reads --hover-in.
#   --tune-out FILE       with --profile tune: on a PASS whose tune reached
#                         Done with all three axes placed, write the
#                         identified gains as `# marv-gains v1`. Refuses
#                         (stderr, exit 3) rather than writing a file a later
#                         --gains-in could not tell from a good one. The file
#                         carries nine `#` provenance lines a --gains-in run
#                         skips: `# authority` (the budget each axis was
#                         placed from), `# placed` (which characteristic
#                         equation was solved -- cubic or pi -- and at what
#                         wn) and `# plant` (what was identified). It used to
#                         carry `# target ... wn=`; there is no wn to ask for
#                         since PLAN.md Q11, and --tune-wn/--tune-yaw-wn went
#                         with it.
#   --gains-in FILE       fly the per-axis rate gains a previous --profile
#                         tune wrote, on ANY profile. Absent, empty,
#                         malformed or out-of-bounds REFUSES the run (exit 2)
#                         rather than silently flying the swept gains. An
#                         explicit --kp/--ki/--kd always wins over the file.
#   --expect-k R,P,Y      REQUIRED with --profile tune: the hand computation
#                         the identified plant gain is graded against
#                         (ADR-0004 S1 gives 128.9,40.34,3.533 for the X3).
#                         Also REQUIRED with --profile step on EVERY axis in
#                         either mode (D24; it was --axis yaw alone before),
#                         where the COMMANDED axis's entry is the plant gain
#                         the model-predicted tracking grade predicts from.
#                         --k-tol sets the fraction it must land within.
#   --expect-tau S        REQUIRED with --profile step, any axis, either
#                         mode: the ROTOR time constant in seconds, one
#                         scalar because it belongs to the motors and not to
#                         an axis. The predicted plant is K/(s(tau*s+1)), not
#                         K/s -- on roll the lag is a third of the closed
#                         loop. sitl/gazebo/marv_quad.sdf:481-482 bounds the
#                         X3's to [0.0125, 0.025]; the tune identifies 0.0175
#                         on every axis of every airframe it has flown.
#                         REFUSED on any other profile: the tune IDENTIFIES
#                         the lag and must not be handed it.
#   --gps-delay-ms N      DELIVERY DELAY on the simulated GPS, whole
#                         milliseconds, default 0 (= off). The sample keeps
#                         its own stamp and is bound to the frame N ms later,
#                         so Sample::age_us reads N on the frame it arrives --
#                         which is the quantity ADR-0007's fusion horizon is
#                         sized from, and since unit 2b the estimator's
#                         horizon IS sized from it (1.5x, setup.cpp) and its
#                         GPS staleness bound carries it as a term.
#                         The graded profiles fly 100 since unit 2b -- the
#                         one copy of that number is
#                         scripts/lib/rig.sh's GPS_DELAY_MS and
#                         sitl/CMakeLists.txt's MARV_GPS_DELAY_MS, and its
#                         basis is written in the first of those. The tool's
#                         own default stays 0, the off switch: a delay is a
#                         caller's statement about its rig, not something
#                         compiled in. The run is expected to PASS either
#                         way, and check-reproducibility.sh takes the flag
#                         like any other (byte-identical at 0 and at 100,
#                         and both are registered as ctest entries).
#   --gui --pace          watch it happen, at roughly wall-clock speed.
#   --wait-viewer         after the server is up, wait (up to 30 s) for a
#                         subscriber to appear on the viewer's data topic
#                         before flying, instead of flying to an empty room.
#                         Implied by --gui. Refuses loudly (exit 2) on
#                         timeout rather than reporting a run nobody watched.
#
# Gains and limits: --kp --ki --kd (rate), --att-kp --att-ki --trust-band
# (estimator), --angle-p --accel-max --yaw-accel (outer loop), --lean
# (envelope). A sweep is a shell loop over these, not a rebuild.
#
# There is NO flag whitelist here: everything except --gui and --wait-viewer
# is passed through to gz_fly, which owns every usage refusal. Nothing needed
# removing when --tune-wn/--tune-yaw-wn went.
#
#   ./scripts/gz-fly.sh --mode rate  --profile hold --seconds 3
#   ./scripts/gz-fly.sh --mode rate  --profile step --seconds 4
#   ./scripts/gz-fly.sh --mode angle --profile hold --seconds 3
#   ./scripts/gz-fly.sh --mode angle --profile step --seconds 4 \
#       --trace /tmp/marv-angle-v5.txt
#
# The server is started WITHOUT -r so the world begins paused: gz_fly steps it
# (ADR-0002, lockstep). A free-running world would make the result depend on
# how busy this machine happened to be.
set -uo pipefail
cd "$(dirname "$0")/.."

# gz_server_pids, gz_refuse_if_served, gz_sweep_leaked -- shared with
# gz-show.sh and baro-loss.sh so the leaked-server guard is one piece of
# code instead of three.
. scripts/lib/gz-server.sh

# The world, overridable so a sweep can point at a generated variant instead
# of editing the tracked SDF in place. scripts/mass-sweep.sh uses this: a
# sweep that sed-ed the real model would leave it modified if it were ever
# interrupted, and the file it corrupts is the one every other run depends on.
WORLD="${MARV_WORLD:-sitl/gazebo/marv_quad.sdf}"
FLY=build/native/sitl/gz_fly

if [ ! -x "$FLY" ]; then
    echo "gz_fly not built. Configure with -DMARV_GAZEBO=ON:" >&2
    echo "    cmake -S . -B build/native -DCMAKE_BUILD_TYPE=Debug -DMARV_GAZEBO=ON" >&2
    echo "    cmake --build build/native" >&2
    exit 2
fi

# --gui runs the viewer alongside the server. The GUI is a separate process
# that attaches to the running world; it does NOT drive it, so lockstep is
# unaffected and a watched run produces the same trace as an unwatched one.
GUI=0
WAIT_VIEWER=0
ARGS=()
for a in "$@"; do
    case "$a" in
        --gui) GUI=1 ;;
        --wait-viewer) WAIT_VIEWER=1 ;;
        *) ARGS+=("$a") ;;
    esac
done
# --gui implies --wait-viewer: flying before a launched-but-not-yet-attached
# viewer subscribes is the empty-room case --wait-viewer exists to refuse.
[ "$GUI" = "1" ] && WAIT_VIEWER=1

# gz_server_pids, gz_refuse_if_served: scripts/lib/gz-server.sh (sourced
# above). A LEAKED SERVER FROM AN EARLIER RUN POISONS THIS ONE, and
# gz_refuse_if_served is the guard: see its comment in that file for why.
gz_refuse_if_served

gz sim -s -v 1 "$WORLD" >/tmp/marv-gz-server.log 2>&1 &
SERVER=$!
GUIPID=""
cleanup() {
    [ -n "$GUIPID" ] && kill "$GUIPID" 2>/dev/null
    # Kill the PROCESS GROUP, not just $SERVER.
    #
    # `gz` is a Ruby script that loads the server in-process, and the job bash
    # backgrounds is not always the ruby process itself: observed directly,
    # $SERVER was 331165 while the surviving server was ruby at 331166 in the
    # same process group. `kill $SERVER` killed the leader, ruby was never
    # signalled, and it was reparented to init still advertising the world --
    # forever, and invisibly, until the next run measured a frozen world.
    kill "$SERVER" 2>/dev/null
    kill -- -"$SERVER" 2>/dev/null

    # gz_sweep_leaked: scripts/lib/gz-server.sh -- polls for the process to
    # actually go away, then escalates. See its comment there for why a
    # cleanup that does not check is how the leak survived in the first
    # place.
    gz_sweep_leaked
    wait "$SERVER" 2>/dev/null
}
trap cleanup EXIT

# Wait for the world to advertise its control service rather than sleeping a
# guessed amount: a fixed sleep is a race that passes on this machine.
for _ in $(seq 1 100); do
    if gz service -l 2>/dev/null | grep -q '/world/marv/control'; then break; fi
    sleep 0.1
done

if [ "$GUI" = "1" ]; then
    gz sim -g -v 1 >/tmp/marv-gz-gui.log 2>&1 &
    GUIPID=$!
fi

# --wait-viewer: poll for an actual subscriber rather than sleeping a guessed
# amount, same reasoning as the control-service wait just above -- the
# `sleep 6` this replaces was a race by its own comment's admission, timing
# how long a viewer took to attach on one machine, once.
#
# The viewer's Scene3D plugin does not subscribe to a pose topic; it
# subscribes to /world/marv/state (a SerializedStepMap), which is how it gets
# poses without a second wire format. Found by launching `gz sim -g` against
# a live server and reading `gz topic -i`: /world/marv/pose/info and
# /world/marv/dynamic_pose/info both showed "No subscribers" with the viewer
# attached and rendering, while /world/marv/state showed the viewer's address.
if [ "$WAIT_VIEWER" = "1" ]; then
    VIEWER_TOPIC=/world/marv/state
    VIEWER_OK=0
    for _ in $(seq 1 300); do
        if gz topic -i -t "$VIEWER_TOPIC" 2>/dev/null | grep -q '^Subscribers'; then
            VIEWER_OK=1
            break
        fi
        sleep 0.1
    done
    if [ "$VIEWER_OK" != "1" ]; then
        echo "FAIL: no viewer subscribed to $VIEWER_TOPIC within 30 s." >&2
        echo "      Launch a viewer (gz sim -g) against this world before" >&2
        echo "      passing --wait-viewer, or drop the flag to fly headless." >&2
        exit 2
    fi
fi

if [ "$GUI" = "1" ]; then
    # Aim the camera at the aircraft and keep it there. Without this the
    # default view looks at the origin from a few metres away and the quad is
    # simply not in frame -- which looks exactly like a bridge that is not
    # working.
    gz service -s /gui/follow/offset \
        --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'x: -2.5, y: -2.5, z: 1.5' >/dev/null 2>&1
    gz service -s /gui/follow \
        --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'data: "marv_quad"' >/dev/null 2>&1
fi

"$FLY" "${ARGS[@]}"
RC=$?
echo "--- server log (tail) ---"
tail -5 /tmp/marv-gz-server.log
exit "$RC"
