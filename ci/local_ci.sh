#!/usr/bin/env bash
# The definition of verified: runs the jobs of .github/workflows/ci.yml locally, one after the other, in the same
# containers, as root, under the GitHub runner's CPU and memory limits, so that a local pass predicts an Actions pass.
#
# Usage: ci/local_ci.sh [--commit <rev>] [--no-cache] [--parallel N] [job ...]
#   jobs: core, gz-l2, gz-l4, gz-l5, gz-l7, gz-l11 (default: all six, in that order, sequentially)
#   --commit: the commit to check, default HEAD. The commit is cloned fresh (git clone, tags included), so uncommitted
#             and untracked files are not tested, exactly as on Actions.
#   --no-cache: run with an empty throwaway cache volume instead of marv-ci-cache (a from-scratch proof).
#   --parallel N: run up to N jobs at once, each in its own container with the limits below and its own clone, its log
#             in a file (the directory is printed; MARV_LOCAL_CI_LOGDIR sets it). N containers at the runner's memory
#             limit must fit the host RAM (free -b total), else the script refuses. Default 1: sequential, output on
#             stdout as well as in the log. The ccache statistics at the end of a job are those of the shared cache
#             since the job started, so under --parallel they include the other running jobs.
# The cache: every container mounts the docker volume marv-ci-cache at /cache (MARV_CACHE_DIR=/cache; the compiler cache is
# /cache/ccache, ci/Dockerfile). A named volume keeps the root-owned cache files out of the home directory; it survives
# runs, and `docker volume rm marv-ci-cache` empties it. Every output line of a job is prefixed with the seconds since
# the container started, so the step times are in the log.
# Environment: MARV_CI_BASE_REF (passed to the core job, as ci.yml does), MARV_CI_MODE (default full: this is the merge
# gate, core 7.6, and ci.yml runs every job in full mode).
#
# Runner limits, measured. Source: ci/runner_resources/run-36803301740-core.txt, the raw output of the "Runner
# resources" step of ci.yml on Actions run 36803301740 (job core, ubuntu-24.04 image 20260920.314.1), logged before the
# job built or ran anything: nproc 4; free -b total 16765378560 B, available 15740260352 B.
# Runner RAM = free -b total. Headroom = total minus available at job start (1025118208 B): what the runner's OS, agent
# and docker daemon hold before the container starts. One reading from one job; a later reading that differs replaces it.
# The runner also has 3221221376 B of swap; the local container gets none (--memory-swap = --memory), the strict side,
# so a local pass predicts an Actions pass but a local OOM is not proof of an Actions OOM.
RUNNER_CPUS="${MARV_LOCAL_CI_CPUS:-4}"
RUNNER_RAM_BYTES="${MARV_LOCAL_CI_RUNNER_RAM_BYTES:-16765378560}"
HEADROOM_BYTES="${MARV_LOCAL_CI_HEADROOM_BYTES:-1025118208}"

set -euo pipefail
export MARV_CI_MODE="${MARV_CI_MODE:-full}"

usage() {
  echo "usage: ci/local_ci.sh [--commit <rev>] [--no-cache] [--parallel N] [core|gz-l2|gz-l4|gz-l5|gz-l7|gz-l11 ...]" >&2
  exit 2
}

commit=HEAD
no_cache=0
parallel=1
jobs=()
add_job() {
  local j
  for j in ${jobs[@]+"${jobs[@]}"}; do
    [[ "${j}" == "$1" ]] && return 0
  done
  jobs+=("$1")
}
while (($# > 0)); do
  case "$1" in
    --commit)
      (($# >= 2)) || usage
      commit="$2"
      shift 2
      ;;
    --no-cache)
      no_cache=1
      shift
      ;;
    --parallel)
      (($# >= 2)) || usage
      [[ "$2" =~ ^[1-9][0-9]*$ ]] || usage
      parallel="$2"
      shift 2
      ;;
    core | gz-l2 | gz-l4 | gz-l5 | gz-l7 | gz-l11)
      add_job "$1"
      shift
      ;;
    *) usage ;;
  esac
done
((${#jobs[@]} > 0)) || jobs=(core gz-l2 gz-l4 gz-l5 gz-l7 gz-l11)
((parallel <= ${#jobs[@]})) || parallel=${#jobs[@]}

mem_bytes=$((RUNNER_RAM_BYTES - HEADROOM_BYTES))
mem_gib="$(awk -v b="${mem_bytes}" 'BEGIN { printf "%.2f", b / 1073741824 }')"

if ((parallel > 1)); then
  host_ram="$(free -b | awk '/^Mem:/ { print $2 }')"
  if ((parallel * mem_bytes > host_ram)); then
    echo "local CI: --parallel ${parallel} needs ${parallel} x ${mem_bytes} B = $((parallel * mem_bytes)) B of container memory limits, the host has ${host_ram} B (free -b total): refusing." >&2
    echo "local CI: lower --parallel, or lower the per-job limit with MARV_LOCAL_CI_RUNNER_RAM_BYTES (then a pass no longer predicts an Actions pass for memory)." >&2
    exit 2
  fi
  host_cpus="$(nproc)"
  if ((parallel * RUNNER_CPUS > host_cpus)); then
    echo "local CI: warning: --parallel ${parallel} x --cpus ${RUNNER_CPUS} exceeds the host's ${host_cpus} CPUs; job times will not match a runner's." >&2
  fi
fi

src_repo="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
sha="$(git -C "${src_repo}" rev-parse --verify "${commit}^{commit}")"

work="$(mktemp -d "${TMPDIR:-/tmp}/marv-local-ci.XXXXXX")"
logdir="${MARV_LOCAL_CI_LOGDIR:-${TMPDIR:-/tmp}/marv-local-ci-logs-${sha:0:8}-$$}"
mkdir -p "${logdir}"
if ((no_cache)); then
  cache_volume="marv-ci-cache-tmp-$$"
else
  cache_volume="marv-ci-cache"
fi

cleanup() {
  local j
  # shellcheck disable=SC2046
  kill $(jobs -p) 2>/dev/null || true
  for j in "${jobs[@]}"; do
    docker rm -f "marv-local-ci-${j}-$$" >/dev/null 2>&1 || true
  done
  # The containers run as root, so the clones hold root-owned files: remove them from inside a container.
  if docker image inspect marv-ci >/dev/null 2>&1; then
    docker run --rm -v "${work}":/work --entrypoint sh marv-ci -c 'rm -rf /work/*' >/dev/null 2>&1 || true
  fi
  rm -rf "${work}"
  if ((no_cache)); then
    docker volume rm -f "${cache_volume}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

docker volume create "${cache_volume}" >/dev/null

# One clone per job: no job shares a mutable /src with another.
for job in "${jobs[@]}"; do
  mkdir -p "${work}/${job}"
  git clone --quiet --no-hardlinks "${src_repo}" "${work}/${job}/src"
  git -C "${work}/${job}/src" fetch --quiet origin "${sha}" 2>/dev/null || true
  git -C "${work}/${job}/src" checkout --quiet --detach "${sha}"
done
echo "local CI: commit ${sha} cloned to ${work}/<job>/src"
echo "local CI: runner limits --cpus ${RUNNER_CPUS} --memory ${mem_bytes} (${mem_gib} GiB; runner RAM ${RUNNER_RAM_BYTES} B minus headroom ${HEADROOM_BYTES} B, measured on Actions run 36803301740), --memory-swap equal"
echo "local CI: cache volume ${cache_volume} at /cache$( ((no_cache)) && echo ' (empty, throwaway: --no-cache)'), parallel ${parallel}, logs in ${logdir}"

# The images are built once, from the first job's clone (the Dockerfiles are those of the commit).
need_gz=0
for job in "${jobs[@]}"; do
  [[ "${job}" == gz-* ]] && need_gz=1
done
(
  cd "${work}/${jobs[0]}/src"
  docker build -q -t marv-ci -f ci/Dockerfile . >/dev/null
  if ((need_gz)); then
    docker build -q -t marv-ci-gz -f ci/Dockerfile.gz . >/dev/null
  fi
)

# Runs inside the container around the job command: trust /src for git (the clone belongs to the host user, the container
# runs as root; tests/tools/test_affected.py runs git there), zero the ccache statistics, prefix every line with the seconds since
# the start, print the statistics at the end, keep the job's exit status.
# shellcheck disable=SC2016
container_wrapper='set -o pipefail; SECONDS=0; git config --global --add safe.directory /src; ccache -z >/dev/null 2>&1 || true
"$@" 2>&1 | while IFS= read -r l || [[ -n "${l}" ]]; do printf "[%5d s] %s\n" "${SECONDS}" "${l}"; done
s=${PIPESTATUS[0]}
echo "##### ccache -s"
ccache -s || true
exit "${s}"'

# run_job <name> <image> <command...>: one job in its own container and clone. Writes the log to ${logdir}/<name>.log
# (and to stdout when the jobs run one after the other) and the summary line to ${work}/result.<name>.
run_job() {
  local name="$1" image="$2" status oom exit_code started container log
  shift 2
  container="marv-local-ci-${name}-$$"
  log="${logdir}/${name}.log"
  started="${SECONDS}"
  echo "##### job ${name}: docker run ${image} $*"
  if ((parallel > 1)); then
    docker run --name "${container}" --cpus "${RUNNER_CPUS}" --memory "${mem_bytes}" --memory-swap "${mem_bytes}" \
      -e MARV_CI_BASE_REF -e MARV_CI_MODE -e MARV_LOCAL_CI=1 -e MARV_CACHE_DIR=/cache \
      -v "${work}/${name}/src":/src -v "${cache_volume}":/cache -w /src "${image}" \
      bash -c "${container_wrapper}" bash "$@" >"${log}" 2>&1 || true
  else
    docker run --name "${container}" --cpus "${RUNNER_CPUS}" --memory "${mem_bytes}" --memory-swap "${mem_bytes}" \
      -e MARV_CI_BASE_REF -e MARV_CI_MODE -e MARV_LOCAL_CI=1 -e MARV_CACHE_DIR=/cache \
      -v "${work}/${name}/src":/src -v "${cache_volume}":/cache -w /src "${image}" \
      bash -c "${container_wrapper}" bash "$@" 2>&1 | tee "${log}" || true
  fi
  # docker run exits 125 without creating the container when it cannot start it (bad option, missing image); then
  # there is nothing to inspect and the job is a failure, not an abort of this script.
  if oom="$(docker inspect -f '{{.State.OOMKilled}}' "${container}" 2>/dev/null)"; then
    exit_code="$(docker inspect -f '{{.State.ExitCode}}' "${container}")"
    docker rm -f "${container}" >/dev/null
  else
    oom="false"
    exit_code="125"
  fi
  if [[ "${oom}" == "true" || "${exit_code}" == "137" ]]; then
    status="OOM-KILLED (OOMKilled=${oom}, exit ${exit_code}) under --memory ${mem_bytes}: this job would be killed on Actions"
  elif [[ "${exit_code}" == "0" ]]; then
    status="PASS"
  else
    status="FAIL (exit ${exit_code})"
  fi
  echo "##### job ${name}: ${status} in $((SECONDS - started)) s" | tee -a "${log}"
  echo "${name}: ${status} in $((SECONDS - started)) s" >"${work}/result.${name}"
  [[ "${status}" == "PASS" ]]
}

job_image() {
  if [[ "$1" == core ]]; then echo marv-ci; else echo marv-ci-gz; fi
}

job_command() {
  if [[ "$1" == core ]]; then echo ci/run_ci.sh; else echo ci/run_ci_gz.sh "${1#gz-}"; fi
}

failed=0
if ((parallel == 1)); then
  for job in "${jobs[@]}"; do
    # shellcheck disable=SC2046
    run_job "${job}" "$(job_image "${job}")" $(job_command "${job}") || failed=1
  done
else
  # A sliding window of at most ${parallel} jobs.
  running=()
  for job in "${jobs[@]}"; do
    while ((${#running[@]} >= parallel)); do
      wait -n || true
      still=()
      for pid in "${running[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then still+=("${pid}"); fi
      done
      running=(${still[@]+"${still[@]}"})
    done
    echo "##### job ${job}: started, log ${logdir}/${job}.log"
    # shellcheck disable=SC2046
    run_job "${job}" "$(job_image "${job}")" $(job_command "${job}") >/dev/null 2>&1 &
    running+=("$!")
  done
  wait || true
fi

results=()
for job in "${jobs[@]}"; do
  if [[ -f "${work}/result.${job}" ]]; then
    line="$(<"${work}/result.${job}")"
  else
    line="${job}: FAIL (no result)"
  fi
  results+=("${line}")
  if [[ "${line}" != *": PASS in "* ]]; then
    failed=1
    if ((parallel > 1)); then
      echo "##### job ${job}: last 40 lines of ${logdir}/${job}.log"
      tail -n 40 "${logdir}/${job}.log" 2>/dev/null || true
    fi
  fi
done

echo "##### local CI summary (commit ${sha})"
printf '%s\n' "${results[@]}"
echo "local CI: logs in ${logdir}"
if ((failed != 0)); then
  echo "LOCAL CI FAILED"
  exit 1
fi
echo "LOCAL CI PASSED"
