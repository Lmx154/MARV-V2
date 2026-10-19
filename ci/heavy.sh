#!/usr/bin/env bash
# The host-wide heavy lock: one build, test run or simulation at a time on this machine (the oomd risk is mechanical, not prose).
# ci/dev_check.sh and ci/local_ci.sh take it themselves; an agent runs any other heavy command under it, and the GCS helper
# (tools/gs, ground-segment unit GX-1) refuses to launch a run while it is held.
#
# Usage: ci/heavy.sh [--wait SECONDS] [--] command [args...]   run the command holding the lock
#        ci/heavy.sh --held                                     exit 0 iff MARV_HEAVY_HELD names this lock and it is really held
#        ci/heavy.sh --status                                   exit 0 and print "free" if nobody holds it, else exit 75 and print
#                                                               the holder line
# The lock is one flock(2) lock on $MARV_HEAVY_LOCK (default /tmp/marv-heavy.lock; tools/gs/hoststate.py has the same default).
# The default is a fixed path in the world-writable /tmp on purpose: the lock is host-wide and every checkout and shell must agree
# on it. That assumes ONE user on this machine; on a shared host set MARV_HEAVY_LOCK to a private path (in this script, the
# helper and every shell that uses either).
# It is held by this script's own shell, not by the command's descendants: a daemon the command leaves behind does not keep it.
# --wait SECONDS: wait that long for a holder to finish (default HEAVY_PROBE_WAIT_S: long enough to ride out the helper's probe).
# While held, the lock file holds one line: "pid=<pid> user=<user> since=<UTC> cmd=<command>". It is stale text when the lock is
# free; only the lock says whether it is held. Exit 75 (EX_TEMPFAIL) when the lock is not obtained; the command's own status
# otherwise. Re-entrant: when MARV_HEAVY_HELD is this lock's path AND the lock is really held (a probe with flock -n -s fails), the
# command runs without taking it again, so dev_check.sh may be run under heavy.sh; a stale variable in a shell that has outlived
# its lock counts for nothing.
set -euo pipefail

lock="${MARV_HEAVY_LOCK:-/tmp/marv-heavy.lock}"
HEAVY_PROBE_WAIT_S=1 # labelled harness value: the helper's non-blocking probe holds a shared lock for microseconds
wait_s="${HEAVY_PROBE_WAIT_S}"

usage() {
  echo "usage: ci/heavy.sh [--wait SECONDS] [--] command [args...] | ci/heavy.sh --status" >&2
  exit 2
}

holder_line() {
  local text
  text="$(head -c 512 "${lock}" 2>/dev/null | head -n 1 || true)"
  echo "${text:-unknown holder}"
}

held_by_ancestor() {
  [[ "${MARV_HEAVY_HELD:-}" == "${lock}" && -e "${lock}" ]] && ! (exec 9<"${lock}" && flock -n -s 9)
}

while (($# > 0)); do
  case "$1" in
    --held)
      (($# == 1)) || usage
      held_by_ancestor
      exit $?
      ;;
    --status)
      (($# == 1)) || usage
      if [[ -e "${lock}" ]] && ! (exec 9<"${lock}" && flock -n -s 9); then
        echo "held: $(holder_line)"
        exit 75
      fi
      echo free
      exit 0
      ;;
    --wait)
      (($# >= 2)) || usage
      [[ "$2" =~ ^[0-9]+$ ]] || usage
      wait_s="$2"
      shift 2
      ;;
    --)
      shift
      break
      ;;
    -*) usage ;;
    *) break ;;
  esac
done
(($# > 0)) || usage

if held_by_ancestor; then
  exec "$@"
fi

exec 9>>"${lock}"
if ! flock -w "${wait_s}" 9; then
  echo "ci/heavy.sh: the heavy lock ${lock} is held: $(holder_line)" >&2
  echo "ci/heavy.sh: wait for it (--wait SECONDS) or run this later; one heavy job at a time on this host" >&2
  exit 75
fi
printf 'pid=%s user=%s since=%s cmd=%s\n' "$$" "${USER:-$(id -un)}" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${*//$'\n'/ }" >"${lock}"
export MARV_HEAVY_HELD="${lock}"
status=0
"$@" 9>&- || status=$?
exit "${status}"
