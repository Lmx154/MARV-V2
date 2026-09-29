// Test-only composition. Motor outputs:
//   normal mode  [tick number, sample code, f32 parameter, i32 parameter]
//   probe mode   [tick number, record metadata code, sigma code, value code]   (parameter probe, see sil_test_codes.hpp)
// Tick 0 and every tick with n % 7 == 6 leave the latch untouched.
#include <cstdint>
#include <cstring>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/params/param.hpp>

#include "sil_test_codes.hpp"

namespace marv::composition {
namespace {

constexpr Tick kHoldPeriod = 7;

Tick g_n = 0;

DshotValue dshot(std::uint16_t raw) noexcept { return DshotValue::from_raw(raw).value_or(DshotValue::stop()); }

std::uint32_t bits(float f) noexcept {
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

std::uint32_t scaled(float v) noexcept { return v > 0.0f ? static_cast<std::uint32_t>(v * 1000.0f) : 0u; }

std::uint32_t nonneg(std::int32_t v) noexcept { return v > 0 ? static_cast<std::uint32_t>(v) : 0u; }

bool same_string(const char* a, const char* b) noexcept { return std::strcmp(a, b) == 0; }

std::uint16_t probe_code(const ParamRecord& r, const ParamRecord& def) noexcept {
  std::uint32_t code = static_cast<std::uint32_t>(r.origin);
  code |= static_cast<std::uint32_t>(r.method) << siltest::kProbeMethodShift;
  code |= r.locked == def.locked ? siltest::kProbeLockBit : 0u;
  code |= r.unit == def.unit ? siltest::kProbeUnitBit : 0u;
  code |= same_string(r.source, "sil-override") ? siltest::kProbeSourceBit : 0u;
  return siltest::clamp_code(code);
}

}  // namespace

void init() noexcept {
  const float mass = param_value<ParamId::fixture_mass_kg>();
  if (mass < 0.0f) {
    hal_panic("test composition: negative fixture mass");
  }
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("test composition: hal_time_us differs from the sample stamp");
  }
  const Tick n = g_n++;
  const float f32 = param_value<ParamId::fixture_mass_kg>();
  const std::int32_t i32 = param_value<ParamId::fixture_motor_count>();
  const std::int32_t probe = param_value<ParamId::fixture_gate_count>();
  if (n == 0 || n % kHoldPeriod == kHoldPeriod - 1) {
    return;
  }

  ActuatorOutput<kMotors, kServos> o{};
  o.motor[0] = dshot(siltest::clamp_code(static_cast<std::uint32_t>(n % 1000)));
  if (probe >= 0 && static_cast<std::size_t>(probe) < kParamCount) {
    const auto id = static_cast<ParamId>(probe);
    const ParamRecord r = param_get(id);
    const ParamRecord& def = param_defaults()[static_cast<std::size_t>(probe)];
    o.motor[1] = dshot(probe_code(r, def));
    o.motor[2] = dshot(siltest::clamp_code(scaled(r.sigma)));
    o.motor[3] = dshot(siltest::clamp_code(r.value.type == ParamType::F32 ? scaled(r.value.f32) : nonneg(r.value.i32)));
  } else {
    const siltest::SampleWords w{bits(s.gyro_rad_s[0]), bits(s.gyro_rad_s[1]), bits(s.gyro_rad_s[2]),
                                 bits(s.accel_m_s2[0]), bits(s.accel_m_s2[1]), bits(s.accel_m_s2[2]),
                                 bits(s.temp_k),        s.flags};
    o.motor[1] = dshot(siltest::sample_code(w));
    o.motor[2] = dshot(siltest::clamp_code(scaled(f32)));
    o.motor[3] = dshot(siltest::clamp_code(nonneg(i32)));
  }
  hal_actuators_write(o);
}

}  // namespace marv::composition
