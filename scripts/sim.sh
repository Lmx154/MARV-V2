#!/usr/bin/env bash
# Start the marv world paused, run marv_bridge against it, stop the world. Arguments go to the bridge:
#
#   scripts/sim.sh --sitl --seconds S [--mission FILE] [--log out.csv]
#   scripts/sim.sh --port /dev/ttyACM0 --seconds S [--mission FILE] [--log out.csv]
#
# Exit code is the bridge's (2 if a gz server was already running, 1 if the world never came up).
set -uo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
source "$root/scripts/lib/gz-server.sh"

bridge="$root/build/native/bridge/marv_bridge"
[ -x "$bridge" ] || { echo "sim.sh: $bridge not built" >&2; exit 1; }

gz_refuse_if_served

server=""
cleanup() {
    [ -n "$server" ] && kill "$server" 2>/dev/null
    wait "$server" 2>/dev/null
    gz_sweep_leaked
}
trap cleanup EXIT

gz sim -s -v 1 "$root/sitl/gazebo/marv_quad.sdf" &
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

"$bridge" "$@"
exit $?
