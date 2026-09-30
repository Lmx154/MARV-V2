#!/usr/bin/env bash
# Builds the host-gz-l5 plugin and runs the L5 attitude-chirp measurement at the live att_loop_ratio (N = 2), then at the N that
# its U_att gives, without regenerating the product (measure_u_att.py --design-u); meant for marv-ci-gz (see README.md).
# Writes raw_att.json, inputs_att.json, u_att.yaml here and the same under n1/ for the re-measurement.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"
uv sync --frozen
cmake --preset host-gz-l5
cmake --build --preset host-gz-l5
uv run python design/measured/sim7_u/measure_u_att.py measure
uv run python design/measured/sim7_u/measure_u_att.py post
uv run python design/measured/sim7_u/measure_u_att.py measure --out-dir design/measured/sim7_u/n1 \
  --design-u design/measured/sim7_u/u_att.yaml
uv run python design/measured/sim7_u/measure_u_att.py post --out-dir design/measured/sim7_u/n1
