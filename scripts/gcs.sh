#!/usr/bin/env bash
# Build the web GCS if its sources changed, then run marv_gcs. Arguments go to marv_gcs:
#
#   scripts/gcs.sh [--udp HOST:PORT | --serial DEV] [--http PORT]
#
# Default link: UDP to marv_bridge --ground on 127.0.0.1:14650. Open the printed URL in a browser.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
web="$root/gcs/web"
gcs="$root/build/native/gcs/marv_gcs"
[ -x "$gcs" ] || { echo "gcs.sh: $gcs not built (cmake --build build/native)" >&2; exit 1; }

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
