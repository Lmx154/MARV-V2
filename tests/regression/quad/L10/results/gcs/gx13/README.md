# GX-13 DERIVE: evidence

The helper's `derive` command (tools/gs/derive.py) builds a `.mcfg` from a copy of a vehicle card with edits: the repository's
`flatten_cached.py`, `params_gen.py` and `mcfg.py encode`, with the arguments of `marv_add_card_param_set(marv_params_fc3v2 ...)`
(fw/params/CMakeLists.txt), on copies in `<helper run dir>/derive/<id>/`. The repository is never written (W-1).

| File | What it is |
| --- | --- |
| `derive_real.py`, `derive_real.log` | one real derivation of `vehicles/marv_7in_fc3v2.yaml` (no edit) under `ci/heavy.sh`: 41.4 s on 16 CPUs (the flatten is nearly all of it); the `.mcfg`, the 13 flatten files (derivation reports included), the product table and the four generated files are byte-identical to the build tree `build/host-debug` of this worktree; `git status` and the diff against HEAD are the same before and after |
| `derive_vs_cmake_pytest.log` | `tests/tools/test_gs_derive_cmake.py` under `ci/heavy.sh`: 3 passed in 164 s. A CMake configure and build of `marv_params_fc3v2_generated` in a temporary copy of the tracked files, then the same card with `mass` edited (0.732 to 0.8, with a source text) written into the COPY only; the helper's derivation of each is compared file by file with the CMake build's. The unedited and the edited `.mcfg` differ, and the set of parameters whose default changed (the mass and the values derived from it) is the same in both |
| `light/` | the light end-to-end run of the Derive panel (`gcs/e2e/derive.spec.ts`, `tools/gs/e2e.sh`): `report.json` and screenshots. The flatten and parameter generator are the stand-ins of `tools/gs/e2e_fake_derive.py` (a second, not a minute); the lint and the `.mcfg` encoder are the real ones |

Reproduce: `ci/heavy.sh --wait 10800 -- .venv/bin/python tests/regression/quad/L10/results/gcs/gx13/derive_real.py --work-dir <dir outside the repository> --cmake-tree build/host-debug`
(after `cmake --preset host-debug && cmake --build --preset host-debug --target marv_params_fc3v2_generated`).

Controls and refusals are in `tests/tools/test_gs_derive.py` (stand-in steps, the real lint): an edit with an empty or unchanged source text is refused with the
lint's finding and nothing runs (the same edit with a source text derives); card names outside `vehicles/*.yaml`, traversal, unknown or locked edit keys, wrong
value kinds are refused; the heavy lock, free memory and one-at-a-time refusals each have their control; cancel stops the process group and frees the lock.

PLACEHOLDERs (tools/gs/derive.py): DERIVE_MIN_FREE_BYTES (2 GiB, unmeasured), DERIVE_STEP_TIMEOUT_S, DERIVE_STOP_S, LINT_TIMEOUT_S, DERIVE_TAIL_LINES, MAX_EDITS,
MAX_SOURCE_CHARS. None decides a result.
