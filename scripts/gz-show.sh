#!/usr/bin/env bash
# WATCH marv fly, in one Gazebo window, at roughly wall-clock speed.
#
# What this is: gz-fly.sh --gui already flies one profile with a viewer
# attached, but there was no single command that strings several of the
# graded profiles together behind ONE persistent viewer for a person to
# watch. This is that command.
#
# What this is NOT: a check. Every run here passes --pace, so its wall-clock
# timing is not the graded timing, and nothing here replaces
# ./scripts/gz-fly.sh run WITHOUT --pace against the five profiles CLAUDE.md
# lists, or ./scripts/negative-controls.sh. This script exists so a person
# can watch those same profiles fly; the numbers it prints are gz-fly.sh's
# own graded output, unfiltered, but a --pace run is not the graded run.
#
# Usage:
#   ./scripts/gz-show.sh [profile ...] [--seconds N] [--software]
#
# profile is one or more of:
#   hover        (default) engage the height hold and climb to ~15 m; the
#                only profile that also learns a hover collective in flight.
#   rate-hold    rate mode, level flight only.
#   rate-step    rate mode, a commanded step and recovery.
#   angle-hold   angle mode, level flight only.
#   angle-step   angle mode, a commanded lean and release.
#   all          hover, then rate-step, angle-step, rate-hold, angle-hold.
#
# --seconds N   overrides every requested profile's run length (default:
#               hover 45 s; step 4 s; hold 3 s -- the step/hold defaults
#               match the graded set's own).
# --software    force LIBGL_ALWAYS_SOFTWARE=1 and the ogre render engine,
#               for a machine (or sandbox) whose viewer cannot get a
#               hardware GL context.
#
# THE NO-GL DIAGNOSIS. A viewer that cannot get a GL context does not
# necessarily crash or exit -- it can sit there logging EGL/dri2 warnings
# while genuinely rendering through a working vendor path underneath them.
# Measured directly against this project's own sandbox 2026-09-02: Mesa's
# dri2 path failed and logged exactly the warnings below, while the NVIDIA
# EGL vendor picked the context up anyway and the viewer stayed alive,
# burning CPU on its render loop, GLCache populated under
# ~/.cache/nvidia/GLCache. So the log signature ALONE is not treated as
# failure here. Failure is gated on LIVENESS: the viewer process died, or it
# never showed up as a subscriber on the world's state topic (see
# gz-fly.sh's --wait-viewer for why /world/marv/state and not a pose topic)
# within the wait window. Only then is the signature grep used, to enrich
# the diagnostic with a likely cause; a live, subscribed viewer that merely
# logged the warning gets one advisory line and the show continues.
#
# Known no-GL log signatures (from /tmp/marv-gz-gui.log):
#   failed to create dri2 screen
#   Unable to create
#   EGL_BAD
#   Failed to create OpenGL context
set -uo pipefail
cd "$(dirname "$0")/.."

# X3_EXPECT_K / X3_EXPECT_TAU: the two step profiles below are refused
# without them since D24, and this script watches the tracked X3 fly.
. scripts/lib/airframes.sh
# GPS_DELAY_MS: the rig's delivery delay, one copy (scripts/lib/rig.sh), so
# what is WATCHED here is the same binding the graded profiles fly.
. scripts/lib/rig.sh

# gz_server_pids, gz_sweep_leaked: shared with gz-fly.sh and baro-loss.sh.
. scripts/lib/gz-server.sh

FLY=build/native/sitl/gz_fly
if [ ! -x "$FLY" ]; then
    echo "gz_fly not built. Configure with -DMARV_GAZEBO=ON:" >&2
    echo "    cmake -S . -B build/native -DCMAKE_BUILD_TYPE=Debug -DMARV_GAZEBO=ON" >&2
    echo "    cmake --build build/native" >&2
    exit 2
fi

# (a) No display, no show. Refuse before touching gz at all.
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "FAIL: neither DISPLAY nor WAYLAND_DISPLAY is set. gz-show.sh opens" >&2
    echo "      a Gazebo window; run it from a desktop terminal, not a" >&2
    echo "      headless shell or a bare ssh session." >&2
    exit 2
fi

# --- argument parsing -------------------------------------------------------
RAW_PROFILES=()
SECONDS_OVERRIDE=""
SOFTWARE=0
NEED_SECONDS=0
for a in "$@"; do
    if [ "$NEED_SECONDS" = "1" ]; then
        SECONDS_OVERRIDE="$a"
        NEED_SECONDS=0
        continue
    fi
    case "$a" in
        --seconds) NEED_SECONDS=1 ;;
        --software) SOFTWARE=1 ;;
        hover|rate-hold|rate-step|angle-hold|angle-step|all) RAW_PROFILES+=("$a") ;;
        *)
            echo "gz-show.sh: unknown argument '$a'" >&2
            echo "  usage: gz-show.sh [profile ...] [--seconds N] [--software]" >&2
            echo "  profile: hover rate-hold rate-step angle-hold angle-step all" >&2
            exit 2
            ;;
    esac
done
if [ "$NEED_SECONDS" = "1" ]; then
    echo "gz-show.sh: --seconds needs a value" >&2
    exit 2
fi

PROFILES=()
for p in "${RAW_PROFILES[@]:-}"; do
    [ -z "$p" ] && continue
    if [ "$p" = "all" ]; then
        PROFILES+=(hover rate-step angle-step rate-hold angle-hold)
    else
        PROFILES+=("$p")
    fi
done
[ ${#PROFILES[@]} -eq 0 ] && PROFILES=(hover)

# --- per-profile mapping -----------------------------------------------------
profile_default_seconds() {
    case "$1" in
        hover) echo 45 ;;
        rate-hold|angle-hold) echo 3 ;;
        rate-step|angle-step) echo 4 ;;
    esac
}
profile_watch() {
    case "$1" in
        hover) echo "rest ~1 s, climb, hold near 15 m" ;;
        rate-step) echo "a sharp roll and recovery" ;;
        angle-step) echo "a lean held then released" ;;
        rate-hold|angle-hold) echo "nothing should move" ;;
    esac
}
FLY_ARGS=()
build_fly_args() {
    local name="$1" secs="$2"
    case "$name" in
        hover)      FLY_ARGS=(--mode angle --profile hover --seconds "$secs") ;;
        rate-hold)  FLY_ARGS=(--mode rate  --profile hold  --seconds "$secs") ;;
        rate-step)  FLY_ARGS=(--mode rate  --profile step  --seconds "$secs"
                              --expect-k "$X3_EXPECT_K"
                              --expect-tau "$X3_EXPECT_TAU") ;;
        angle-hold) FLY_ARGS=(--mode angle --profile hold  --seconds "$secs") ;;
        angle-step) FLY_ARGS=(--mode angle --profile step  --seconds "$secs"
                              --expect-k "$X3_EXPECT_K"
                              --expect-tau "$X3_EXPECT_TAU") ;;
        *)
            echo "gz-show.sh: internal: unknown profile '$name'" >&2
            exit 2
            ;;
    esac
}

# --- the leaked-server guard is sourced from scripts/lib/gz-server.sh -----
# (gz_server_pids, gz_sweep_leaked). Each gz-fly.sh child this script runs
# already verifies ITS OWN server is gone via that same logic before it
# exits; the sweep below is gz-show.sh's own, independent check for the
# case gz-show.sh itself is interrupted mid-run.

# --- viewer + server lifecycle ----------------------------------------------
VIEWER_PID=""
FLY_PID=""
cleanup() {
    [ -n "$VIEWER_PID" ] && kill "$VIEWER_PID" 2>/dev/null
    [ -n "$FLY_PID" ] && kill "$FLY_PID" 2>/dev/null
    [ -n "$VIEWER_PID" ] && wait "$VIEWER_PID" 2>/dev/null
    [ -n "$FLY_PID" ] && wait "$FLY_PID" 2>/dev/null

    gz_sweep_leaked
}
trap cleanup EXIT INT

# Poll for the control service rather than sleeping a guessed amount --
# copied from gz-fly.sh's own wait, same reasoning: a fixed sleep is a race
# that passes on this machine.
wait_for_control() {
    for _ in $(seq 1 100); do
        if gz service -l 2>/dev/null | grep -q '/world/marv/control'; then return 0; fi
        sleep 0.1
    done
    echo "FAIL: /world/marv/control never came up." >&2
    exit 2
}

# Launch the viewer, then gate success on LIVENESS -- see the header comment
# for why the EGL/dri2 log signature alone is not the test. Up to 30 s, the
# same bound gz-fly.sh's own --wait-viewer uses, so this and the flight it is
# watching time out together rather than gz-show.sh giving up early or
# hanging past the point gz-fly.sh itself would refuse.
launch_viewer() {
    if [ "$SOFTWARE" = "1" ]; then
        LIBGL_ALWAYS_SOFTWARE=1 gz sim -g -v 1 --render-engine ogre \
            >/tmp/marv-gz-gui.log 2>&1 &
    else
        gz sim -g -v 1 >/tmp/marv-gz-gui.log 2>&1 &
    fi
    VIEWER_PID=$!

    local ok=0
    for _ in $(seq 1 150); do
        if ! kill -0 "$VIEWER_PID" 2>/dev/null; then break; fi
        if gz topic -i -t /world/marv/state 2>/dev/null | grep -q '^Subscribers'; then
            ok=1
            break
        fi
        sleep 0.2
    done

    if [ "$ok" != "1" ]; then
        echo "FAIL: the Gazebo viewer did not attach." >&2
        if kill -0 "$VIEWER_PID" 2>/dev/null; then
            echo "      pid $VIEWER_PID is alive but never subscribed to" >&2
            echo "      /world/marv/state within 30 s." >&2
        else
            echo "      pid $VIEWER_PID exited." >&2
        fi
        echo "      The viewer has no GPU context: run this from a desktop" >&2
        echo "      terminal, not over ssh or from a sandbox; or retry with" >&2
        echo "      --software." >&2
        echo "      Last 5 lines of /tmp/marv-gz-gui.log:" >&2
        tail -5 /tmp/marv-gz-gui.log >&2
        VIEWER_PID=""
        exit 2
    fi

    # Advisory only -- the viewer is confirmed alive and subscribed above,
    # so a logged EGL/dri2 warning here is not a failure by itself.
    if grep -qE 'failed to create dri2 screen|Unable to create|EGL_BAD|Failed to create OpenGL context' \
        /tmp/marv-gz-gui.log; then
        echo "note: viewer log shows EGL/dri2 warnings; if the window is black, retry with --software" >&2
    fi

    # Aim the camera at the aircraft -- copied from gz-fly.sh's --gui path.
    # Without this the default view looks at the origin from a few metres
    # away and the quad is simply not in frame.
    gz service -s /gui/follow/offset \
        --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'x: -2.5, y: -2.5, z: 1.5' >/dev/null 2>&1
    gz service -s /gui/follow \
        --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'data: "marv_quad"' >/dev/null 2>&1
}

# Run one profile. The viewer, once up, persists across gz-fly.sh's own
# server restarts between profiles -- gz-transport subscriptions are by
# topic name, not by server process, so a freshly started server advertising
# the same /world/marv/* names should be picked back up without relaunching
# the viewer. Only relaunch it if it actually died.
run_profile() {
    local name="$1"
    local secs="${SECONDS_OVERRIDE:-$(profile_default_seconds "$name")}"
    build_fly_args "$name" "$secs"
    local watch
    watch="$(profile_watch "$name")"
    local log="/tmp/marv-show-${name}.log"

    echo
    echo "=== $name (${secs}s) -- watch for: $watch"

    if [ -n "$VIEWER_PID" ] && ! kill -0 "$VIEWER_PID" 2>/dev/null; then
        echo "viewer (pid $VIEWER_PID) is gone; relaunching before this run" >&2
        VIEWER_PID=""
    fi

    local rc
    if [ -z "$VIEWER_PID" ]; then
        "./scripts/gz-fly.sh" --pace --wait-viewer \
            --gps-delay-ms "$GPS_DELAY_MS" "${FLY_ARGS[@]}" \
            > >(tee "$log") 2>&1 &
        FLY_PID=$!
        wait_for_control
        launch_viewer
        wait "$FLY_PID"
        rc=$?
        FLY_PID=""
    else
        "./scripts/gz-fly.sh" --pace --wait-viewer \
            --gps-delay-ms "$GPS_DELAY_MS" "${FLY_ARGS[@]}" \
            > >(tee "$log") 2>&1
        rc=$?
    fi

    echo "gz-show: $name exited $rc -- full output: $log"
    return "$rc"
}

# --- run the show ------------------------------------------------------------
OVERALL_RC=0
for name in "${PROFILES[@]}"; do
    run_profile "$name" || OVERALL_RC=$?
done

echo
echo "close the Gazebo window to finish (or Ctrl-C)"
if [ -n "$VIEWER_PID" ]; then
    wait "$VIEWER_PID" 2>/dev/null
fi
exit "$OVERALL_RC"
