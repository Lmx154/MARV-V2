#include "param_override.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace marv::sil {

OverrideStatus validate_overrides(std::span<const ParamRecord, kParamCount> defaults,
                                  std::span<const marv_sil_param_override> overrides) noexcept {
  std::array<bool, kParamCount> seen{};
  for (std::size_t i = 0; i < overrides.size(); ++i) {
    const marv_sil_param_override& o = overrides[i];
    const OverrideStatus bad{false, static_cast<std::uint32_t>(i)};
    if (o.id >= kParamCount) {
      return bad;
    }
    const ParamType type = defaults[o.id].value.type;
    if (o.type != static_cast<std::uint32_t>(type)) {
      return bad;
    }
    if (type == ParamType::F32 && !std::isfinite(o.f32)) {
      return bad;
    }
    if (!std::isfinite(o.sigma) || o.sigma < 0.0f) {
      return bad;
    }
    if (seen[o.id]) {
      return bad;
    }
    seen[o.id] = true;
  }
  return {true, 0};
}

OverrideStatus apply_overrides(std::span<const ParamRecord, kParamCount> defaults,
                               std::span<const marv_sil_param_override> overrides,
                               std::span<ParamRecord, kParamCount> merged) noexcept {
  const OverrideStatus status = validate_overrides(defaults, overrides);
  if (!status.ok) {
    return status;
  }
  std::copy(defaults.begin(), defaults.end(), merged.begin());
  for (const marv_sil_param_override& o : overrides) {
    ParamRecord& r = merged[o.id];
    if (r.value.type == ParamType::F32) {
      r.value = ParamValue{ParamType::F32, o.f32, 0};
    } else {
      r.value = ParamValue{ParamType::I32, 0.0f, o.i32};
    }
    r.sigma = o.sigma;
    r.origin = ParamOrigin::Manual;
    r.method = ParamMethod::Scenario;
    r.source = "sil-override";
  }
  return status;
}

}  // namespace marv::sil
