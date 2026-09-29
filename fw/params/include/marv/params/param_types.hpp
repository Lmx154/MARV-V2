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

struct ParamValue {
  ParamType type;
  float f32;
  std::int32_t i32;  // the unused member is exactly 0
};

struct ParamRecord {
  ParamValue value;
  float sigma;  // 1-sigma, same unit; finite, >= 0
  ParamOrigin origin;
  ParamMethod method;
  bool locked;
  const char* unit;    // SI symbol, static storage; "1" if dimensionless
  const char* source;  // source text, or the rule for Derived; static storage
};

}  // namespace marv
