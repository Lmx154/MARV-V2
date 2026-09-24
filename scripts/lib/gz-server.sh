# Shared gz-sim server-lifecycle helpers -- sourced, not run.
#
# This carries the code that used to be copied three ways (gz-fly.sh,
# gz-show.sh, baro-loss.sh) to guard against the two rig defects CLAUDE.md's
# "A single gz run is not evidence" section describes: a leaked server from
# an earlier run silently poisoning this one (every graded number reads
# 0.000 off a world nobody is stepping), and this script's own cleanup
# failing to notice a server it started did not actually die.
#
# Sourceable: no top-level side effects, no `set`, no `exit` outside a
# function body, no trap installed here. The caller's own `set -uo
# pipefail` and `trap cleanup EXIT` govern as before; this file only adds
# functions to that shell.

# Processes that are ACTUALLY a gz server.
#
# The first version of this guard was `pgrep -f "gz sim -s"`, and it refused
# 12 runs out of 12 -- because it matched the calling shell, whose command
# line happened to contain those words. A guard that fires on the words rather
# than on the thing is worse than no guard, so the command line is only a
# candidate filter here; comm is what decides. `gz` is a Ruby script that
# loads the server in-process, so a real server's comm is ruby.
gz_server_pids() {
    local pid
    for pid in $(pgrep -f 'gz sim -s' 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        case "$(ps -o comm= -p "$pid" 2>/dev/null)" in
            ruby|gz|gz-sim*) echo "$pid" ;;
        esac
    done
}

# A LEAKED SERVER FROM AN EARLIER RUN POISONS THIS ONE. Refuse to start.
#
# Two servers both advertise /world/marv/control AND the IMU topic under the
# same name. The bridge then steps one world while its subscription may be
# bound to the other -- which nothing is stepping. The run does not fail: it
# completes all 3000 frames, replay compares identical, and every graded
# number is exactly 0.000, because the IMU dutifully republished the initial
# state of a world that never moved. A run that measures a frozen world while
# reporting a full frame count is worse than one that crashes.
#
# Measured 2026-09-01: two leaked servers were found alive from earlier runs,
# and they account for much of the variance CLAUDE.md recorded as a startup
# race. Hence a hard stop -- prefer a loud failure to a plausible number.
#
# The authoritative question is "is someone already serving this world?", so
# ask the transport, not the process table. The pid check is the backstop.
#
# Exits the calling process directly (rather than returning non-zero) so
# every caller gets the same refusal without having to remember to check.
gz_refuse_if_served() {
    if gz service -l 2>/dev/null | grep -q '/world/marv/control' \
       || [ -n "$(gz_server_pids)" ]; then
        echo "FAIL: a gz-sim server is already serving /world/marv/control." >&2
        echo "      This run would measure the wrong world. Kill it and retry:" >&2
        echo "          pkill -9 -f 'gz sim'" >&2
        for p in $(gz_server_pids); do ps -o pid=,args= -p "$p" >&2; done
        exit 2
    fi
}

# Then VERIFY, and escalate. A cleanup that does not check is how the
# leak survived in the first place.
gz_sweep_leaked() {
    for _ in $(seq 1 30); do
        [ -z "$(gz_server_pids)" ] && break
        sleep 0.1
    done
    # Loud on purpose: a server that outlived its own cleanup is the
    # leaked-server signature CLAUDE.md wants seen, not swept quietly.
    for p in $(gz_server_pids); do
        echo "gz-server: a gz-sim server survived cleanup (pid $p) -- killing it" >&2
        kill -9 "$p" 2>/dev/null
    done
}
