#pragma once

#include <cstdint>

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

}  // namespace marv
