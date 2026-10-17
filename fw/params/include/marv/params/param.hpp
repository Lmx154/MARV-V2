#pragma once

#include <cstddef>
#include <span>

#include <marv/params/param_ids.hpp>
#include <marv/params/param_types.hpp>

namespace marv {

// Copy of the runtime record; O(1). An invalid id or a call before a successful params_init is a hal_panic.
[[nodiscard]] ParamRecord param_get(ParamId id) noexcept;

// The typed value of the runtime record. The type is fixed at compile time by ParamTraits; nothing is checked at run
// time besides what param_get checks.
template <ParamId Id>
[[nodiscard]] typename ParamTraits<Id>::value_type param_value() noexcept {
  const ParamValue v = param_get(Id).value;
  if constexpr (ParamTraits<Id>::type == ParamType::F32) {
    return v.f32;
  } else {
    return v.i32;
  }
}

// The typed value of `table`'s record: param_value's read on a given table. No range check: the caller passes a table of
// the full kParamCount records.
template <ParamId Id>
[[nodiscard]] typename ParamTraits<Id>::value_type param_value(const ParamTable& table) noexcept {
  const ParamValue v = table[static_cast<std::size_t>(Id)].value;
  if constexpr (ParamTraits<Id>::type == ParamType::F32) {
    return v.f32;
  } else {
    return v.i32;
  }
}

// The runtime table that params_init loaded. A call before a successful params_init is a hal_panic.
[[nodiscard]] ParamTable param_active_table() noexcept;

// The generated name of a parameter. An invalid id is a hal_panic.
[[nodiscard]] const char* param_name(ParamId id) noexcept;

// The generated default table (static storage, const).
[[nodiscard]] std::span<const ParamRecord, kParamCount> param_defaults() noexcept;

// Validates the whole table, then loads it. Returns false and changes nothing if any record is invalid or if a
// previous call succeeded. Call once, before composition::init.
[[nodiscard]] bool params_init(std::span<const ParamRecord, kParamCount> table) noexcept;

}  // namespace marv
