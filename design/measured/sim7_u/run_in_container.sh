#!/usr/bin/env bash
# Builds the host-gz-l4 plugin and runs the measurement and its post-processing; meant for marv-ci-gz (see README.md).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"
uv sync --frozen
cmake --preset host-gz-l4
cmake --build --preset host-gz-l4
uv run python design/measured/sim7_u/measure_u.py measure
uv run python design/measured/sim7_u/measure_u.py post
