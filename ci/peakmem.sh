# shellcheck shell=bash
# Sourced by ci/run_ci.sh and ci/run_ci_gz.sh. Not executable on its own.
#
# Per-step peak memory of the container. The cgroup v2 file memory.peak is cumulative since the container started and is
# read-only inside an unprivileged container (writing 0 to reset it fails with "Read-only file system", checked in a
# docker run on this host), so a per-step peak is taken by sampling memory.current in a background loop and keeping the
# maximum. The figure is the working set, memory.current minus inactive_file (the docker stats convention): page cache
# the kernel can drop under pressure is not counted, because it does not cause an OOM kill.
# MARV_CGROUP_DIR overrides the cgroup directory (tests of this file only).

PEAKMEM_DIR="${MARV_CGROUP_DIR:-/sys/fs/cgroup}"
PEAKMEM_INTERVAL_S="${MARV_PEAKMEM_INTERVAL_S:-0.2}"
PEAKMEM_PID=""
PEAKMEM_FILE=""

peakmem_working_set() {
  local cur inactive
  read -r cur <"${PEAKMEM_DIR}/memory.current" || return 1
  inactive="$(awk '$1 == "inactive_file" { print $2 }' "${PEAKMEM_DIR}/memory.stat" 2>/dev/null)" || inactive=0
  echo $((cur - ${inactive:-0}))
}

peakmem_sampler() {
  local max=0 v
  while :; do
    if v="$(peakmem_working_set)" && ((v > max)); then
      max="${v}"
      echo "${max}" >"${PEAKMEM_FILE}"
    fi
    sleep "${PEAKMEM_INTERVAL_S}"
  done
}

peakmem_start() {
  PEAKMEM_PID=""
  PEAKMEM_FILE=""
  if [[ ! -r "${PEAKMEM_DIR}/memory.current" || ! -r "${PEAKMEM_DIR}/memory.stat" ]]; then
    return 0
  fi
  PEAKMEM_FILE="$(mktemp)"
  echo 0 >"${PEAKMEM_FILE}"
  peakmem_sampler &
  PEAKMEM_PID=$!
}

# peakmem_report <step name>
peakmem_report() {
  local name="$1" bytes saved
  if [[ -z "${PEAKMEM_FILE}" ]]; then
    echo "PEAK-MEM: ${name} UNKNOWN (${PEAKMEM_DIR}/memory.current or memory.stat not readable: no cgroup v2 memory controller visible)"
    return 0
  fi
  kill "${PEAKMEM_PID}" 2>/dev/null || true
  wait "${PEAKMEM_PID}" 2>/dev/null || true
  # One last sample: a step shorter than the interval would otherwise report 0.
  bytes="$(peakmem_working_set 2>/dev/null)" || bytes=0
  saved="$(<"${PEAKMEM_FILE}")"
  if ((saved > bytes)); then
    bytes="${saved}"
  fi
  rm -f "${PEAKMEM_FILE}"
  PEAKMEM_FILE=""
  echo "PEAK-MEM: ${name} ${bytes} ($(awk -v b="${bytes}" 'BEGIN { printf "%.2f GiB", b / 1073741824 }'))"
}
