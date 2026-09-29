#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace marv {

enum class ParamType : std::uint8_t { F32 = 0, I32 };
enum class ParamOrigin : std::uint8_t { DefaultFromCard = 0, DefaultFromRegister, Manual, Identified };
enum class ParamMethod : std::uint8_t {
  Published = 0,
  Measured,
  Identified,
  Datasheet,
  Derived,
  DesignBudget,
  Scenario
};

// How to read ParamRecord::sigma. Known: a 1-sigma > 0. Exact: no uncertainty (a definition or a count), sigma is +0.
// Unknown: the uncertainty has not been established, sigma is +0 and must not be read as Exact. Choice: a design
// choice or scenario value, not a measurement, sigma is +0.
enum class SigmaKind : std::uint8_t { Known = 0, Exact, Unknown, Choice };

struct ParamValue {
  ParamType type;
  float f32;
  std::int32_t i32;  // the unused member is exactly 0
};

struct ParamRecord {
  ParamValue value;
  float sigma;  // 1-sigma, same unit; > 0 for SigmaKind::Known, +0.0f for every other kind
  ParamOrigin origin;
  ParamMethod method;
  bool locked;
  SigmaKind sigma_kind;
  const char* unit;    // SI symbol, static storage; "1" if dimensionless
  const char* source;  // source text, or the rule for Derived; static storage
};

// Invariant I1 (docs/decisions/0001), the one implementation: a record is consistent iff (kind Known and sigma finite
// and > 0) or (kind Exact, Unknown or Choice and sigma exactly +0.0f); any other kind value is inconsistent. Usable in
// constant expressions: no std::isfinite or std::signbit, only comparisons (NaN and inf fail the range test) and a bit
// test for +0.0f.
[[nodiscard]] constexpr bool sigma_consistent(SigmaKind kind, float sigma) noexcept {
  switch (kind) {
    case SigmaKind::Known:
      return sigma > 0.0f && sigma <= std::numeric_limits<float>::max();
    case SigmaKind::Exact:
    case SigmaKind::Unknown:
    case SigmaKind::Choice:
      return std::bit_cast<std::uint32_t>(sigma) == 0;
  }
  return false;
}

// I1 over every record of a table; the generated param_defaults.cpp static_asserts it.
template <std::size_t N>
[[nodiscard]] constexpr bool sigma_table_consistent(const ParamRecord (&table)[N]) noexcept {
  for (const ParamRecord& r : table) {
    if (!sigma_consistent(r.sigma_kind, r.sigma)) {
      return false;
    }
  }
  return true;
}

}  // namespace marv
