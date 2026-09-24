#!/usr/bin/env bash
# Start the marv world paused, run marv_bridge against it, stop the world. Arguments go to the bridge:
#
#   scripts/sim.sh --sitl --seconds S [--mission FILE | --ground] [--log out.csv]
#   scripts/sim.sh --port /dev/ttyACM0 --seconds S [--mission FILE | --ground] [--log out.csv]
#
# Exit code is the bridge's (2 if a gz server was already running, 1 if the world never came up).
#   --gui (anywhere in the arguments) also opens the Gazebo window on the same world.
#   --world x3|x500 (anywhere; default x3) picks the aircraft: x3 sitl/gazebo/marv_quad.sdf, x500 sitl/gazebo/x500.sdf
#   (PX4's x500; the bridge gets its --max-rot-velocity 1000, and ~/sim/PX4-gazebo-models/models, when present, is
#   added to GZ_SIM_RESOURCE_PATH for its meshes; without it the x500 flies with its mesh visuals removed).
set -uo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
source "$root/scripts/lib/gz-server.sh"

bridge="$root/build/native/bridge/marv_bridge"
[ -x "$bridge" ] || { echo "sim.sh: $bridge not built" >&2; exit 1; }

gui=0
world=x3
args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --gui) gui=1 ;;
        --world) world="${2:-}"; shift ;;
        *) args+=("$1") ;;
    esac
    shift
done
case "$world" in
    x3) sdf="$root/sitl/gazebo/marv_quad.sdf" ;;
    x500)
        sdf="$root/sitl/gazebo/x500.sdf"
        args=(--max-rot-velocity 1000 "${args[@]}")
        px4_models="$HOME/sim/PX4-gazebo-models/models"
        if [ -d "$px4_models" ]; then
            export GZ_SIM_RESOURCE_PATH="$px4_models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
        else
            # gz sim refuses to load a world whose model:// URIs do not resolve: drop the visuals that use them.
            echo "sim.sh: no $px4_models: the x500 flies without its meshes (the viewer shows no aircraft)" >&2
            nomesh="$(mktemp --suffix=.sdf)"
            awk '/<visual /{buf = ""; v = 1} v {buf = buf $0 "\n"; if (/<\/visual>/) {v = 0; if (buf !~ /model:\/\//) printf "%s", buf}; next} {print}' \
                "$sdf" >"$nomesh"
            sdf="$nomesh"
        fi
        ;;
    *) echo "sim.sh: --world x3|x500, not '$world'" >&2; exit 2 ;;
esac

gz_refuse_if_served

server=""
viewer=""
cleanup() {
    [ -n "$viewer" ] && kill "$viewer" 2>/dev/null
    [ -n "$server" ] && kill "$server" 2>/dev/null
    wait "$server" 2>/dev/null
    gz_sweep_leaked
    [ -n "${nomesh:-}" ] && rm -f "$nomesh"
}
trap cleanup EXIT

gz sim -s -v 1 "$sdf" &
server=$!

for _ in $(seq 1 100); do
    gz service -l 2>/dev/null | grep -q '^/world/marv/control$' && break
    kill -0 "$server" 2>/dev/null || { echo "sim.sh: gz sim exited during startup" >&2; exit 1; }
    sleep 0.1
done
gz service -l 2>/dev/null | grep -q '^/world/marv/control$' \
    || { echo "sim.sh: /world/marv/control did not appear within 10 s" >&2; exit 1; }
# The control service is advertised while the world still loads; the clock means its loop is running.
timeout 10 gz topic -e -t /world/marv/clock -n 1 >/dev/null \
    || { echo "sim.sh: /world/marv/clock did not start within 10 s" >&2; exit 1; }

if [ "$gui" = 1 ]; then
    # The window is a client of the running server. Default renderer first (ogre2); if it dies at once (no GPU
    # support), fall back to ogre 1. Its log: /tmp/marv-gz-gui.log.
    gz sim -g -v 1 >/tmp/marv-gz-gui.log 2>&1 &
    viewer=$!
    sleep 5
    if ! kill -0 "$viewer" 2>/dev/null; then
        echo "sim.sh: the ogre2 viewer exited (see /tmp/marv-gz-gui.log); retrying with ogre" >&2
        gz sim -g -v 1 --render-engine ogre >>/tmp/marv-gz-gui.log 2>&1 &
        viewer=$!
    fi
fi

"$bridge" "${args[@]}"
exit $?
