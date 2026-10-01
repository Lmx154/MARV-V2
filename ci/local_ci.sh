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
echo "local CI: runner limits --cpus ${RUNNER_CPUS} --memory ${mem_bytes} (${mem_gib} GiB; runner RAM ${RUNNER_RAM_BYTES} B minus headroom ${HEADROOM_BYTES} B, measured on Actions run 36803301740), --memory-swap equal"

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
  # docker run exits 125 without creating the container when it cannot start it (bad option, missing image); then
  # there is nothing to inspect and the job is a failure, not an abort of this script.
  if oom="$(docker inspect -f '{{.State.OOMKilled}}' "${container}" 2>/dev/null)"; then
    exit_code="$(docker inspect -f '{{.State.ExitCode}}' "${container}")"
    docker rm -f "${container}" >/dev/null
  else
    oom="false"
    exit_code="125"
  fi
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
