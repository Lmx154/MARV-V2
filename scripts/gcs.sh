#!/usr/bin/env bash
# Build the web GCS if its sources changed, then run marv_gcs. Arguments go to marv_gcs:
#
#   scripts/gcs.sh [--udp HOST:PORT | --serial DEV] [--http PORT]
#
# Default link: automatic, the flight controller on USB while no sim runs, the sim's bridge (127.0.0.1:14650) while
# one does. --udp or --serial fixes it. Open the printed URL in a browser.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
web="$root/gcs/web"
gcs="$root/build/native/gcs/marv_gcs"
[ -x "$gcs" ] || { echo "gcs.sh: $gcs not built (cmake --build build/native)" >&2; exit 1; }

# One marv_gcs per user: it holds this lock, with its pid and URL in it (MARV_GCS_LOCK is for tests only). When one
# runs, point at it instead of starting another.
lock="${MARV_GCS_LOCK:-${XDG_RUNTIME_DIR:+$XDG_RUNTIME_DIR/marv-gcs.lock}}"
lock="${lock:-/tmp/marv-gcs-$(id -u).lock}"
if [ -e "$lock" ] && ! flock -n "$lock" true 2>/dev/null; then
    read -r pid url <"$lock" || true
    echo "gcs.sh: marv_gcs is already running (pid ${pid:-?}): open ${url:-its URL} (stop it first to start another)"
    exit 0
fi

if [ ! -d "$web/node_modules" ] || [ "$web/package-lock.json" -nt "$web/node_modules" ]; then
    (cd "$web" && npm ci)
fi
if [ ! -f "$web/build/index.html" ] ||
    [ -n "$(find "$web/src" "$web/package.json" "$web/vite.config.ts" -newer "$web/build/index.html" -print -quit)" ]; then
    (cd "$web" && npm run build)
fi

port=8765
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    [ "${args[$i]}" = "--http" ] && port="${args[$((i + 1))]:-$port}"
done
echo "gcs.sh: open http://127.0.0.1:$port/"
exec "$gcs" --web "$web/build" "$@"
