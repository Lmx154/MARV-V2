// The fixed L0 test composition. Three rate groups (divisors are i32 parameters) update held state from the IMU
// sample and float parameters; every tick writes one quad-X output whose DShot values depend on that held state.
// The mixing below is a test signal path, not a vehicle mixer: it makes the sample, the parameters and the group
// firings each visible in the output. Time comes from the sample stamp; state is static.
#include <array>
#include <cstddef>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::composition {
namespace {

using Vec3f = prim::Vec3<float>;

enum Group : std::size_t { kFast, kMid, kSlow, kGroupCount };

struct HeldState {
  Vec3f rate_lp;      // rad/s, low-passed gyro, updated by the fast group
  Vec3f accel_lp;     // m/s^2, low-passed accelerometer, updated by the mid group
  float bias = 0.0f;  // throttle fraction, updated by the slow group
};

sched::RateGroups<kGroupCount> g_groups;
HeldState g_state;
Tick g_n = 0;

[[nodiscard]] std::uint32_t divisor(std::int32_t d) noexcept { return d < 1 ? 0U : static_cast<std::uint32_t>(d); }

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

// One-pole low pass: x moves a fraction alpha of the way to the input.
[[nodiscard]] Vec3f low_pass(const Vec3f& x, const Vec3f& in, float alpha) noexcept { return x + (in - x) * alpha; }

[[nodiscard]] bool is_motor(std::size_t m, Motor a) noexcept { return m == motor_index(a); }

[[nodiscard]] Vec3f mix_row(std::size_t m) noexcept {
  const float roll = (is_motor(m, Motor::M2RearLeft) || is_motor(m, Motor::M3FrontLeft)) ? 1.0f : -1.0f;
  const float pitch = (is_motor(m, Motor::M1FrontRight) || is_motor(m, Motor::M3FrontLeft)) ? 1.0f : -1.0f;
  const float yaw = (is_motor(m, Motor::M1FrontRight) || is_motor(m, Motor::M2RearLeft)) ? 1.0f : -1.0f;
  return Vec3f{roll, pitch, yaw};
}

// Throttle fraction u -> DShot: not above 0 (or NaN) is stop, else clamped to [0, 1] and mapped linearly onto the
// throttle range, truncating.
[[nodiscard]] DshotValue throttle(float u) noexcept {
  if (!(u > 0.0f)) {
    return DshotValue::stop();
  }
  constexpr float span = static_cast<float>(prim::kDshotThrottleMax - prim::kDshotThrottleMin);
  const float c = u < 1.0f ? u : 1.0f;
  const auto raw = static_cast<std::uint16_t>(static_cast<float>(prim::kDshotThrottleMin) + (c * span));
  return DshotValue::from_raw(raw).value_or(DshotValue::stop());
}

}  // namespace

void init() noexcept {
  const std::array<std::uint32_t, kGroupCount> divisors{divisor(param_value<ParamId::l0_div_fast>()),
                                                        divisor(param_value<ParamId::l0_div_mid>()),
                                                        divisor(param_value<ParamId::l0_div_slow>())};
  if (!g_groups.init(divisors)) {
    hal_panic("l0 composition: a rate-group divisor is below 1");
  }
  g_state = HeldState{};
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l0 composition: hal_time_us differs from the sample stamp");
  }
  const Tick n = g_n++;
  const std::uint32_t due = g_groups.due(n);

  if (fired(due, kFast)) {
    g_state.rate_lp = low_pass(g_state.rate_lp, s.gyro_rad_s, param_value<ParamId::l0_alpha_fast>());
  }
  if (fired(due, kMid)) {
    g_state.accel_lp = low_pass(g_state.accel_lp, s.accel_m_s2, param_value<ParamId::l0_alpha_mid>());
  }
  if (fired(due, kSlow)) {
    const float drive = (param_value<ParamId::l0_gain_bias>() * g_state.rate_lp.norm()) +
                        (param_value<ParamId::l0_gain_temp>() * (s.temp_k - param_value<ParamId::l0_temp_ref_k>()));
    g_state.bias += param_value<ParamId::l0_alpha_slow>() * (drive - g_state.bias);
  }

  const Vec3f drive = (g_state.rate_lp * param_value<ParamId::l0_gain_rate>()) +
                      (g_state.accel_lp * param_value<ParamId::l0_gain_accel>());
  const float base = param_value<ParamId::l0_base_throttle>() + g_state.bias;
  ActuatorOutput<kMotors, kServos> o{};
  for (std::size_t m = 0; m < kMotors; ++m) {
    o.motor[m] = throttle(base + mix_row(m).dot(drive));
  }
  hal_actuators_write(o);
}

}  // namespace marv::composition
