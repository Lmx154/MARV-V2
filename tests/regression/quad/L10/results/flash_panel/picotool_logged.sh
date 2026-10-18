#!/bin/sh
# Bench only: records every picotool invocation (time, arguments) in raw/picotool_invocations.log, then runs the real picotool.
# The helper is started with --picotool pointing here, so a flash refused before picotool leaves no line.
LOG="$(dirname "$0")/raw/picotool_invocations.log"
echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) picotool $*" >> "$LOG"
exec "${MARV_PICOTOOL_REAL:-$HOME/pico/picotool-install/picotool/picotool}" "$@"
