// Test-only composition: latches what attitude_input handed it and reports it on the motor outputs every tick as
// [calls since the previous tick, flags, hash low, hash high] (probe_codes.hpp).
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>

#include "probe_codes.hpp"

namespace marv::composition {
namespace {

AttitudeState<float> g_latch{};
bool g_have = false;
std::uint32_t g_calls = 0;

DshotValue dshot(std::uint16_t raw) noexcept { return DshotValue::from_raw(raw).value_or(DshotValue::stop()); }

}  // namespace

void init() noexcept {}

void attitude_input(const AttitudeState<float>& a) noexcept {
  g_latch = a;
  g_have = true;
  ++g_calls;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("truth-state test composition: hal_time_us differs from the sample stamp");
  }
  std::uint32_t flags = 0;
  std::uint32_t hash = 0;
  if (g_have) {
    flags |= truthtest::kFlagHave;
    flags |= g_latch.valid ? truthtest::kFlagValid : 0u;
    flags |= g_latch.t_us == s.t_us ? truthtest::kFlagStampMatches : 0u;
    hash = truthtest::hash_of(g_latch);
  }
  const truthtest::Row r = truthtest::row_of(g_calls, flags, hash);
  g_calls = 0;

  ActuatorOutput<kMotors, kServos> o{};
  o.motor[0] = dshot(r.calls);
  o.motor[1] = dshot(r.flags);
  o.motor[2] = dshot(r.hash_lo);
  o.motor[3] = dshot(r.hash_hi);
  hal_actuators_write(o);
}

}  // namespace marv::composition
