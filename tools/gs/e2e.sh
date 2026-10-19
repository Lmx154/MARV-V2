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
# continues it. The Playwright JSON report and the failure screenshots land in the output directory (default build/gs-e2e/out).
#
#   tools/gs/e2e.sh [--live] [--out-dir DIR] [-- <playwright test arguments>]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
root="$(pwd)"

mode=light
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

helper_args=(--port 0 --runs-dir "${work}/runs" --sil-udp-port "${udp_port}")
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
  # An empty library: the run is not about the images in build/.
  mkdir -p "${work}/build-root"
  helper_args+=(--build-root "${work}/build-root")
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
    npx --no-install playwright test "$@"
) || status=$?
echo "report: ${out_dir}/report.json"
if ((status != 0)); then
  echo "##### helper log"
  cat "${work}/helper.log"
fi
exit "${status}"
