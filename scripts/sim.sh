#!/usr/bin/env bash
# Start the marv world paused, run marv_bridge against it, stop the world. Arguments go to the bridge:
#
#   scripts/sim.sh --sitl --seconds S [--mission FILE | --ground] [--log out.csv]
#   scripts/sim.sh --port /dev/ttyACM0 --seconds S [--mission FILE | --ground] [--log out.csv]
#
# Exit code is the bridge's (2 if a gz server was already running, 1 if the world never came up).
#   --gui (anywhere in the arguments) also opens the Gazebo window on the same world.
#   --world ID (anywhere; default x3) picks the aircraft from the catalog sitl/airframes/ID (x3, x500, ...):
#   build/native/gcs/marv_worldgen writes its world in calm air at ETH Zurich to build/native/sim/generated/ID.sdf and
#   names its max rotor speed (the bridge's --max-rot-velocity) and mesh directory (added to GZ_SIM_RESOURCE_PATH;
#   without it the model's model:// visuals are dropped and the viewer shows no aircraft).
#   --world-file PATH instead runs a world generated beforehand (the GCS launcher's, with wind and a location); the
#   caller passes the airframe's --max-rot-velocity and sets GZ_SIM_RESOURCE_PATH. The sensors are on link base_link.
set -uo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
source "$root/scripts/lib/gz-server.sh"

bridge="$root/build/native/bridge/marv_bridge"
worldgen="$root/build/native/gcs/marv_worldgen"
[ -x "$bridge" ] || { echo "sim.sh: $bridge not built" >&2; exit 1; }

gui=0
world=x3
world_file=""
args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --gui) gui=1 ;;
        --world) world="${2:-}"; shift ;;
        --world-file) world_file="${2:-}"; shift ;;
        *) args+=("$1") ;;
    esac
    shift
done
if [ -n "$world_file" ]; then
    [ -f "$world_file" ] || { echo "sim.sh: --world-file $world_file: no such file" >&2; exit 2; }
    sdf="$world_file"
else
    [ -x "$worldgen" ] || { echo "sim.sh: $worldgen not built" >&2; exit 1; }
    sdf="$root/build/native/sim/generated/$world.sdf"
    gen="$("$worldgen" "$world" "$sdf")" || { echo "sim.sh: --world $world: see above" >&2; exit 2; }
    args=(--max-rot-velocity "$(sed -n 's/^max_rot_velocity=//p' <<<"$gen")" "${args[@]}")
    resources="$(sed -n 's/^resource_path=//p' <<<"$gen")"
    [ -n "$resources" ] && export GZ_SIM_RESOURCE_PATH="$resources${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
fi
args=(--link base_link "${args[@]}")
# gz-transport discovery is UDP multicast, sent on every host interface (docker bridges included) and answered on
# each one it arrived on. The bridge's subscribe burst overflowed the server's 208 KiB receive buffer (measured: up
# to 13 datagrams dropped at bridge start); a subscriber registration lost there is never resent, and that topic
# (clock, gnss, ...) never reaches the bridge. On loopback alone the burst is a few datagrams per topic.
export GZ_IP="${GZ_IP:-127.0.0.1}"

gz_refuse_if_served

server=""
viewer=""
cleanup() {
    [ -n "$viewer" ] && kill "$viewer" 2>/dev/null
    [ -n "$server" ] && kill "$server" 2>/dev/null
    wait "$server" 2>/dev/null
    gz_sweep_leaked
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
