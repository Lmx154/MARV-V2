# 0001: parameter record sigma kind, lock and vector entries

## What changed

- L0 interface `fw/params/include/marv/params/param_types.hpp` gains `enum class SigmaKind : std::uint8_t { Known = 0, Exact, Unknown, Choice }` and `ParamRecord::sigma_kind`, placed after `locked`. `sizeof(ParamRecord)` is unchanged.
- `params_init` (`fw/params/src/param.cpp`) enforces invariant I1: a record is accepted iff its kind is Known with a finite sigma > 0, or its kind is Exact, Unknown or Choice with sigma exactly +0.0f. An out-of-range kind is rejected.
- Invariant I1 has one C++ implementation, `constexpr bool sigma_consistent(SigmaKind, float)` in `param_types.hpp` (comparisons and `std::bit_cast`, no `std::isfinite` or `std::signbit`), which `params_init` calls; the old private `sigma_valid` is removed.
- Build-time check (Luis, 2026-09-29: "the generator rejects a sigma_kind / σ mismatch at build time, not just firmware startup"): `param_defaults.cpp` now defines `constexpr ParamRecord kParamDefaults[kParamCount]` (the `extern const` declaration in `param_ids.hpp` is unchanged and gives the definition external linkage) and ends the table with `static_assert(sigma_table_consistent(kParamDefaults), ...)`, so a record that violates I1 fails to compile. `params_gen.py` also validates every final (kind, sigma) record against a Python mirror of I1 after the token to kind mapping and before it writes anything; on a mismatch it exits 1, names the entry and writes no file.
- The SIL override (`fw/sil/src/param_override.cpp`) sets Known when the override sigma is > 0, else Exact with sigma +0.0f. The C ABI (`marv_sil.h`) does not change.
- `tools/gen/params_gen.py` gains the sigma tokens `UNKNOWN`, `choice` and `exact` (numeric 0 is Exact), the optional entry fields `lock` and `shape` (frd3, diag3, range, motors), and a fourth output `params_provenance.json`. `param_ids.hpp`, `params_manifest.json` and the schema-hash algorithm are unchanged; `param_defaults.cpp` gains `SigmaKind::<kind>` in each record and `true` for locked entries.
- `fw/params/CMakeLists.txt` lists the new output and adds the fixture set `marv_params_l1_fixture` (sources under `tests/regression/quad/L01/fixtures/`).
- No file under `tests/regression/` was modified. New files only: `tests/regression/quad/L01/{fixtures,tools,unit/params}`.

## Why

Luis 2026-09-29, items 1 and 5 (sigma policy; generator gaps): an `UNKNOWN` sigma must never read as 0. A record needs to say whether sigma 0 means "exact", "not established" or "a design choice", and the generator needs locks and vector entries to express the L1 card.

## Evidence

- `cmake --preset host-debug && cmake --build --preset host-debug && ctest --preset host-debug`: all frozen tests (L00 and the L1 param-record tests) and `sil_exports` pass.
- `uv run pytest tests/regression/quad/L00/tools -q` passes unchanged; `tests/regression/quad/L01/tools/test_params_gen_l1.py` covers every new generator rule with a refusal test.
- For the L0 fixtures, `param_ids.hpp` and `params_manifest.json` generated before and after the change are byte-identical (`cmp`); only `param_defaults.cpp` differs (the new field; for the build-time check, `constexpr` and the `static_assert`).
- `sizeof(ParamRecord)`: 40 bytes on host and 28 bytes on the m33 build, before and after (`sigma_kind` sits in the padding after `locked`).
- Build-time check controls: `tests/regression/quad/L01/tools/test_params_gen_sigma_check.py` injects a fault into the generator's token to kind mapping (a Known kind with sigma 0, an Unknown kind with sigma > 0) and requires exit 1 and no output, and shows the unpatched generator accepts the same input (with `check_records` disabled, three of its tests fail). `tests/regression/quad/L01/controls/sigma_kind_mismatch.cpp` (target `sigma_kind_mismatch`, EXCLUDE_FROM_ALL) is a table in the generated form with one Known record of sigma 0 and the same `static_assert`; the CI step `sigma_check_control` passes only if its compile fails with `error: static assertion failed: param_defaults: a record violates invariant I1` and no other error.
- `cmake --preset m33 && cmake --build --preset m33` builds.

## Approval

Luis's approval of the pull request or push (core §7.3).
