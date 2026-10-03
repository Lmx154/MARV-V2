# Product parameter table (frozen)

`marv_params_product_table.txt` is the table of every record of the product parameter set (`marv_params`: the vehicle card,
the design budget and the scenario register, flattened and extended by `tools/card/`), in id order: id, name, type, value
(nine significant digits and the exact hexadecimal float), sigma and its kind, unit, method, origin, lock and source. It is
what the host and the flight (m33) builds compile in. A change to any flight value changes this file, so it shows as a diff in
review (owner ruling 2026-10-03, quad L6 stage (c)).

It is a derived copy, never edited by hand. `ci/run_ci.sh` compares it byte for byte (`cmp`) with the table each build
generates (host-debug and m33), and runs a planted-change control against that comparison. Its header names the generator
(`tools/gen/params_gen.py --out-table`, run by `fw/params/CMakeLists.txt`) and the inputs.

Regenerate it inside the CI image (goldens are regenerated only there) and copy the build's table over this one:

    docker run --rm --cpus 4 --memory 15740260352 --memory-swap 15740260352 -v "$PWD":/src -w /src marv-ci bash -lc \
      'uv sync --frozen -q && cmake --preset host-debug && cmake --build --preset host-debug --target marv_params_generated \
       && cp build/host-debug/generated/marv_params/marv_params_product_table.txt tests/regression/quad/L06/product_params/'

Changing this file (or a card, budget, scenario or `tools/card/` input that changes it) needs a decision record,
`docs/decisions/NNNN-<slug>.md`, in the same change (core 7.3).
