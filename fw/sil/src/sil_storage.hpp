// The SIL's config boot (decision 0053, ground-segment unit 4b). Defined by storage.cpp in a library built with STORAGE
// (MARV_SIL_STORAGE; the flight composition: the store's boot step on hal_sim's run-directory storage) and by marv_sil.cpp
// otherwise (the compiled table).
#pragma once

#include <span>

#include <marv/params/param_ids.hpp>
#include <marv/params/param_types.hpp>

namespace marv::sil {

// The table marv_sil_init starts from, before the overrides: `defaults` unless a run directory is open and holds a stored
// config the boot policy chooses. Called once, by marv_sil_init, before params_init and composition::init.
[[nodiscard]] std::span<const ParamRecord, kParamCount> boot_table(std::span<const ParamRecord, kParamCount> defaults) noexcept;

}  // namespace marv::sil
