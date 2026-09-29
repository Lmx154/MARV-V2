#include <marv/params/param.hpp>

#include <cmath>
#include <cstddef>

#include <marv/hal/hal.hpp>

namespace marv {
namespace {

ParamRecord g_table[kParamCount]{};
bool g_loaded = false;

[[nodiscard]] bool id_valid(ParamId id) noexcept { return static_cast<std::size_t>(id) < kParamCount; }

[[nodiscard]] bool record_valid(const ParamRecord& r, const ParamRecord& generated) noexcept {
  if (r.value.type != generated.value.type || r.unit != generated.unit || r.source == nullptr) {
    return false;
  }
  if (!sigma_consistent(r.sigma_kind, r.sigma)) {
    return false;
  }
  if (r.value.type == ParamType::F32) {
    return std::isfinite(r.value.f32) && r.value.i32 == 0;
  }
  return r.value.f32 == 0.0f;
}

}  // namespace

ParamRecord param_get(ParamId id) noexcept {
  if (!id_valid(id)) {
    hal_panic("param_get with an invalid parameter id");
  }
  if (!g_loaded) {
    hal_panic("param_get before params_init");
  }
  return g_table[static_cast<std::size_t>(id)];
}

const char* param_name(ParamId id) noexcept {
  if (!id_valid(id)) {
    hal_panic("param_name with an invalid parameter id");
  }
  return generated::kParamNames[static_cast<std::size_t>(id)];
}

std::span<const ParamRecord, kParamCount> param_defaults() noexcept {
  return std::span<const ParamRecord, kParamCount>{generated::kParamDefaults};
}

bool params_init(std::span<const ParamRecord, kParamCount> table) noexcept {
  if (g_loaded) {
    return false;
  }
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (!record_valid(table[i], generated::kParamDefaults[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < kParamCount; ++i) {
    g_table[i] = table[i];
  }
  g_loaded = true;
  return true;
}

}  // namespace marv
