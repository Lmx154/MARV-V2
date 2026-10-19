#!/usr/bin/env bash
# The GCS end-to-end run (GCS-first unit GX-0): the built GCS in a browser (Playwright, gcs/e2e/) against the helper
# (tools/gs/helper.py) and a vehicle.
#
#   tools/gs/e2e.sh          light: the vehicle is the fake of tools/gs/e2e_fake_vehicle.py on a free UDP port. No build of
#                            the firmware, no simulator, no board.
#   tools/gs/e2e.sh --live   live: the helper's own SIL launcher starts the simulator (tools/sim/run_quad_fc.py: needs gz-sim 8
#                            and the host-gz-quadfc build, so it is heavy work under the host's heavy-work rules); the spec
#                            starts it from the page.
#
# Needs `uv sync --frozen`, `cd gcs && npm ci` and `cd gcs && npx playwright install chromium` (the browser lives in the
# Playwright cache, not in the repository). It builds the GCS (gcs/build, ignored by git) and removes every process it started
# on exit. The staleness control of the specs stops the fake vehicle (SIGSTOP of its process group, MARV_E2E_VEHICLE_PID) and
# continues it; in live mode the vehicle page's control stops the processes of the helper's run (those with MARV_E2E_RUNS_DIR, the
# helper's run directory, in their command line) and continues them. The Playwright JSON report and the failure screenshots land in the output directory (default build/gs-e2e/out).
#
#   tools/gs/e2e.sh [--live] [--out-dir DIR] [-- <playwright test arguments>]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
root="$(pwd)"

mode=light
# Light mode: the SIL run is `sleep` of this many seconds (a labelled harness value: it only has to outlast a spec; stop ends it). The
# number is unusual on purpose, so that a spec can look for the stand-in process by it.
STAND_IN_SECONDS=7654.321
out_dir="${root}/build/gs-e2e/out"
while (($# > 0)); do
  case "$1" in
    --live) mode=live ;;
    --out-dir)
      (($# >= 2)) || { echo "--out-dir needs a directory" >&2; exit 2; }
      out_dir="$(realpath -m "$2")"
      shift
      ;;
    --) shift; break ;;
    *) echo "usage: tools/gs/e2e.sh [--live] [--out-dir DIR] [-- <playwright test arguments>]" >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -d gcs/node_modules ]]; then
  echo "gcs/node_modules is missing: run 'cd gcs && npm ci' first" >&2
  exit 1
fi
if ! (cd gcs && npx --no-install playwright --version >/dev/null 2>&1); then
  echo "Playwright is not installed in gcs/node_modules: run 'cd gcs && npm ci' first" >&2
  exit 1
fi

work="$(mktemp -d "${TMPDIR:-/tmp}/marv-gs-e2e.XXXXXX")"
pids=()
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${pids[@]-}"; do
    # A group a spec left stopped (the staleness control) only handles SIGTERM once it is continued.
    [[ -n "${pid}" ]] && kill -CONT -- "-${pid}" 2>/dev/null || true
    [[ -n "${pid}" ]] && kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]-}"; do
    [[ -n "${pid}" ]] && kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  rm -rf "${work}"
  exit "${status}"
}
trap cleanup EXIT INT TERM

echo "##### build the GCS (gcs/build)"
(cd gcs && npm run build >"${work}/build.log" 2>&1) || { cat "${work}/build.log"; exit 1; }

udp_port="$(uv run python -c '
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])')"

# The helper refuses a run while the host's heavy lock is held (H-3). This script is often run under that lock (ci/heavy.sh), so the
# helper watches a lock file of its own: the run is this script's own heavy work, which the outer lock already covers.
helper_args=(--port 0 --runs-dir "${work}/runs" --sil-udp-port "${udp_port}" --heavy-lock "${work}/heavy.lock")
if [[ "${mode}" == light ]]; then
  echo "##### fake vehicle on udp ${udp_port}"
  mkdir -p build/gs-e2e/defs
  setsid uv run python tools/gs/e2e_fake_vehicle.py --udp-port "${udp_port}" --defs-dir build/gs-e2e/defs \
    >"${work}/vehicle.log" 2>&1 &
  pids+=("$!")
  for _ in $(seq 1 300); do
    grep -q '^fake vehicle on' "${work}/vehicle.log" 2>/dev/null && break
    kill -0 "${pids[0]}" 2>/dev/null || { cat "${work}/vehicle.log"; echo "the fake vehicle did not start" >&2; exit 1; }
    sleep 0.1
  done
  grep -q '^fake vehicle on' "${work}/vehicle.log" || { cat "${work}/vehicle.log"; echo "the fake vehicle did not start" >&2; exit 1; }
  # A library of one stand-in SIL image (an empty file) with a small parameter manifest in the build tree the helper checks SIL
  # overrides against (tools/sim/sil_run.py), so that the Simulation page lists parameters and the helper validates its overrides; the
  # run itself is the stand-in command `sleep` (no simulator), and the fake vehicle already holds the run's UDP port, which is what the
  # helper's autoconnect waits for. The run_start / run_stop answers are the helper's own; only the simulator is replaced.
  gz_tree="${work}/build-root/host-gz-quadfc"
  mkdir -p "${gz_tree}/sim/quad_fc" "${gz_tree}/generated/marv_params_fc3v2/marv/params"
  : >"${gz_tree}/sim/quad_fc/libmarv_sil_quad_fc.so"
  uv run python - "${gz_tree}/generated/marv_params_fc3v2/marv/params/params_manifest.json" <<'PY'
import json, sys
names = (("att_kp", "f32", False), ("rate_kp_roll", "f32", False), ("rc_channel_roll", "i32", False), ("rc_channel_arm", "i32", True))
params = {
    name: {"id": i, "type": kind, "unit": "1", "link_id": name, "lock": locked, "default_bits": "0x00000001", "method": "scenario",
           "sigma_kind": "Choice", "sigma": 0.0, "sigma_bits": "0x00000000", "origin": "DefaultFromCard", "source": "stand-in of the GCS e2e",
           "source_file": "e2e.sh"}
    for i, (name, kind, locked) in enumerate(names)
}
json.dump({"schema_hash": "0x00000000000000e2", "count": len(params), "params": params}, open(sys.argv[1], "w"))
PY
  helper_args+=(--build-root "${work}/build-root" --sil-command "sleep ${STAND_IN_SECONDS}")
fi

echo "##### helper"
setsid uv run python tools/gs/helper.py "${helper_args[@]}" >"${work}/helper.log" 2>&1 &
pids+=("$!")
url=""
for _ in $(seq 1 300); do
  url="$(sed -n 's/^GCS: *//p' "${work}/helper.log" | head -n 1)"
  [[ -n "${url}" ]] && break
  kill -0 "${pids[-1]}" 2>/dev/null || { cat "${work}/helper.log"; echo "the helper did not start" >&2; exit 1; }
  sleep 0.1
done
[[ -n "${url}" ]] || { cat "${work}/helper.log"; echo "the helper did not print its address" >&2; exit 1; }
echo "GCS served at ${url%%\?*}"

echo "##### playwright (${mode})"
mkdir -p "${out_dir}"
status=0
(
  cd gcs
  # The staleness control stops and continues the light mode's fake vehicle (its process group); the live mode has no such process.
  MARV_E2E_URL="${url}" MARV_E2E_MODE="$([[ "${mode}" == live ]] && echo live || true)" MARV_E2E_OUT="${out_dir}" \
    MARV_E2E_VEHICLE_PID="$([[ "${mode}" == light ]] && echo "${pids[0]}" || true)" \
    MARV_E2E_STAND_IN="$([[ "${mode}" == light ]] && echo "${STAND_IN_SECONDS}" || true)" \
    MARV_E2E_RUNS_DIR="${work}/runs" \
    npx --no-install playwright test "$@"
) || status=$?
echo "report: ${out_dir}/report.json"
if ((status != 0)); then
  echo "##### helper log"
  cat "${work}/helper.log"
fi
exit "${status}"
