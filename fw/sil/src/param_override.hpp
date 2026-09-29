#pragma once

#include <cstdint>
#include <span>

#include <marv/params/param_ids.hpp>
#include <marv/params/param_types.hpp>

#include "marv_sil.h"

namespace marv::sil {

struct OverrideStatus {
  bool ok;
  std::uint32_t index;  // index of the first rejected override; 0 when ok
};

// Checks every override against the defaults; changes nothing. Rejects an unknown id, a type mismatch, a non-finite
// f32 value, a non-finite or negative sigma and a repeated id.
[[nodiscard]] OverrideStatus validate_overrides(std::span<const ParamRecord, kParamCount> defaults,
                                                std::span<const marv_sil_param_override> overrides) noexcept;

// Validates first (merged is untouched on failure), then copies the defaults into merged and applies each override:
// value and sigma replaced (sigma_kind Known if sigma > 0, else Exact with sigma +0), origin Manual, method Scenario, source "sil-override"; lock and unit unchanged.
[[nodiscard]] OverrideStatus apply_overrides(std::span<const ParamRecord, kParamCount> defaults,
                                             std::span<const marv_sil_param_override> overrides,
                                             std::span<ParamRecord, kParamCount> merged) noexcept;

}  // namespace marv::sil
