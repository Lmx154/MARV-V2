SIM-3 throughput of the L2 plugin, Release build (docs/decisions/0003 item 2). Raw output of `tools/sim/measure_sim3.py`
(the script; inputs: `vehicles/uzh_neurobem_5in.yaml`, `scenarios/quad/L02/free_fall.yaml`, m = 1, 6400 host steps = the scenario's T).

Command, host: `cmake --preset host-gz-release && cmake --build --preset host-gz-release && uv run python tools/sim/measure_sim3.py --output tests/regression/quad/L02/results/sim3/host_release.txt`

Command, image (`marv-ci-gz`, clean copy of the tree): `docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/src -w /src marv-ci-gz bash -c 'uv sync --frozen && cmake --preset host-gz-release && cmake --build --preset host-gz-release && uv run python tools/sim/measure_sim3.py --output tests/regression/quad/L02/results/sim3/image_release.txt'`

Files: `host_release.txt`, `image_release.txt`. One measurement each; the required value and the comparison are UNKNOWN, SIM-3 stays open.
