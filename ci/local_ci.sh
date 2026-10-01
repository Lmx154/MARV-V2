#!/usr/bin/env bash
# The definition of verified: runs the jobs of .github/workflows/ci.yml locally, one after the other, in the same
# containers, as root, under the GitHub runner's CPU and memory limits, so that a local pass predicts an Actions pass.
#
# Usage: ci/local_ci.sh [--commit <rev>] [job ...]
#   jobs: core, gz-l2, gz-l4, gz-l5 (default: all four, in that order, sequentially)
#   --commit: the commit to check, default HEAD. The commit is cloned fresh (git clone, tags included), so uncommitted
#             and untracked files are not tested, exactly as on Actions.
# Environment: MARV_CI_BASE_REF (passed to the core job, as ci.yml does), MARV_CI_MODE (per-push or full).
#
# Runner limits. Source: GitHub-hosted standard runner for public repositories, ubuntu-24.04: 4 vCPU, 16 GB RAM,
# https://docs.github.com/en/actions/reference/runners/github-hosted-runners (section "Standard GitHub-hosted runners for
# public repositories"). UNVERIFIED until a job's logged `free -b` (the "Runner resources" step of ci.yml) confirms it.
# 16 GB is read as 16e9 bytes, the lower of the two readings (16 GiB is larger), so the limit errs on the strict side.
# Headroom: UNKNOWN. The rule is "what the runner's OS, the runner agent and the docker daemon use before the container
# starts"; no source for that number has been found, so the value below is a SCENARIO value (1 GiB), not a measurement.
# Replace it with the first idle-runner reading of `free -b` (total minus available) once a job has logged one.
RUNNER_CPUS="${MARV_LOCAL_CI_CPUS:-4}"
RUNNER_RAM_BYTES="${MARV_LOCAL_CI_RUNNER_RAM_BYTES:-16000000000}"
HEADROOM_BYTES="${MARV_LOCAL_CI_HEADROOM_BYTES:-1073741824}"

set -euo pipefail

usage() {
  echo "usage: ci/local_ci.sh [--commit <rev>] [core|gz-l2|gz-l4|gz-l5 ...]" >&2
  exit 2
}

commit=HEAD
jobs=()
while (($# > 0)); do
  case "$1" in
    --commit)
      (($# >= 2)) || usage
      commit="$2"
      shift 2
      ;;
    core | gz-l2 | gz-l4 | gz-l5)
      jobs+=("$1")
      shift
      ;;
    *) usage ;;
  esac
done
((${#jobs[@]} > 0)) || jobs=(core gz-l2 gz-l4 gz-l5)

mem_bytes=$((RUNNER_RAM_BYTES - HEADROOM_BYTES))
mem_gib="$(awk -v b="${mem_bytes}" 'BEGIN { printf "%.2f", b / 1073741824 }')"

src_repo="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
sha="$(git -C "${src_repo}" rev-parse --verify "${commit}^{commit}")"

work="$(mktemp -d "${TMPDIR:-/tmp}/marv-local-ci.XXXXXX")"
clone="${work}/src"
container=""

cleanup() {
  if [[ -n "${container}" ]]; then
    docker rm -f "${container}" >/dev/null 2>&1 || true
  fi
  # The containers run as root, so the clone holds root-owned files: remove them from inside a container.
  if docker image inspect marv-ci >/dev/null 2>&1; then
    docker run --rm -v "${work}":/work --entrypoint rm marv-ci -rf /work/src >/dev/null 2>&1 || true
  fi
  rm -rf "${work}"
}
trap cleanup EXIT

git clone --quiet --no-hardlinks "${src_repo}" "${clone}"
git -C "${clone}" fetch --quiet origin "${sha}" 2>/dev/null || true
git -C "${clone}" checkout --quiet --detach "${sha}"
echo "local CI: commit ${sha} cloned to ${clone}"
echo "local CI: runner limits --cpus ${RUNNER_CPUS} --memory ${mem_bytes} (${mem_gib} GiB; runner RAM ${RUNNER_RAM_BYTES} B UNVERIFIED minus headroom ${HEADROOM_BYTES} B UNKNOWN scenario value), --memory-swap equal"

build_ci() {
  docker build -q -t marv-ci -f ci/Dockerfile . >/dev/null
}

build_ci_gz() {
  build_ci
  docker build -q -t marv-ci-gz -f ci/Dockerfile.gz . >/dev/null
}

# run_job <name> <image> <command...>
run_job() {
  local name="$1" image="$2" status oom exit_code started
  shift 2
  container="marv-local-ci-${name}-$$"
  started="${SECONDS}"
  echo "##### job ${name}: docker run ${image} $*"
  docker run --name "${container}" --cpus "${RUNNER_CPUS}" --memory "${mem_bytes}" --memory-swap "${mem_bytes}" \
    -e MARV_CI_BASE_REF -e MARV_CI_MODE -v "${clone}":/src -w /src "${image}" "$@" || true
  oom="$(docker inspect -f '{{.State.OOMKilled}}' "${container}")"
  exit_code="$(docker inspect -f '{{.State.ExitCode}}' "${container}")"
  docker rm -f "${container}" >/dev/null
  container=""
  if [[ "${oom}" == "true" || "${exit_code}" == "137" ]]; then
    status="OOM-KILLED (OOMKilled=${oom}, exit ${exit_code}) under --memory ${mem_bytes}: this job would be killed on Actions"
  elif [[ "${exit_code}" == "0" ]]; then
    status="PASS"
  else
    status="FAIL (exit ${exit_code})"
  fi
  echo "##### job ${name}: ${status} in $((SECONDS - started)) s"
  results+=("${name}: ${status} in $((SECONDS - started)) s")
  [[ "${status}" == "PASS" ]]
}

results=()
failed=0
cd "${clone}"
for job in "${jobs[@]}"; do
  case "${job}" in
    core)
      build_ci
      run_job core marv-ci ci/run_ci.sh || failed=1
      ;;
    gz-l2 | gz-l4 | gz-l5)
      build_ci_gz
      run_job "${job}" marv-ci-gz ci/run_ci_gz.sh "${job#gz-}" || failed=1
      ;;
  esac
done

echo "##### local CI summary (commit ${sha})"
printf '%s\n' "${results[@]}"
if ((failed != 0)); then
  echo "LOCAL CI FAILED"
  exit 1
fi
echo "LOCAL CI PASSED"
