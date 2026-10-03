#include "marv/rate_group/rate_group.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>

namespace marv::rate_group {
namespace {

const char* message(gyro_chain::ConfigError e) noexcept {
  switch (e) {
    case gyro_chain::ConfigError::None:
      break;
    case gyro_chain::ConfigError::NonFinite:
      return "gyro chain: the tick period, gyro_lpf_cutoff_hz, gyro_notch_q_h<k> or gyro_notch_omega_min is not finite";
    case gyro_chain::ConfigError::Period:
      return "gyro chain: the tick period is not positive";
    case gyro_chain::ConfigError::Divisor:
      return "gyro chain: rate_loop_divisor is below 1";
    case gyro_chain::ConfigError::Cutoff:
      return "gyro chain: gyro_lpf_cutoff_hz is not in (0, f_s / (2 D))";
    case gyro_chain::ConfigError::NotchQ:
      return "gyro chain: a notch quality gyro_notch_q_h<k> is not positive";
    case gyro_chain::ConfigError::Threshold:
      return "gyro chain: gyro_notch_omega_min is not positive";
  }
  return "gyro chain: unknown configuration error";
}

// The rate loop's own gyro check (rate_loop.hpp, execute): the valid flag set and every axis finite.
bool gyro_usable(const ImuSample& s) noexcept {
  using std::isfinite;
  bool ok = (s.flags & imu_flag(ImuFlag::GyroValid)) != 0;
  for (std::size_t a = 0; a < gyro_chain::kAxes; ++a) {
    ok = ok && isfinite(s.gyro_rad_s[a]);
  }
  return ok;
}

}  // namespace

gyro_chain::GyroChainConfig<float> chain_from_params() noexcept {
  gyro_chain::GyroChainConfig<float> c;
  // The tick period in microseconds is one float quotient of two integers, and the conversion to seconds one division.
  const float period_us = static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  c.rate_divisor = divisor < 1 ? 0 : static_cast<std::uint32_t>(divisor);
  c.cutoff_hz = param_value<ParamId::gyro_lpf_cutoff_hz>();
  c.notch_q = {param_value<ParamId::gyro_notch_q_h1>(), param_value<ParamId::gyro_notch_q_h2>(),
               param_value<ParamId::gyro_notch_q_h3>()};
  c.omega_threshold_rad_s = param_value<ParamId::gyro_notch_omega_min>();
  c.seed_first_sample = true;
  return c;
}

void require_valid(const gyro_chain::GyroChainConfig<float>& c) noexcept {
  const gyro_chain::ConfigError e = gyro_chain::validate(c);
  if (e != gyro_chain::ConfigError::None) {
    hal_panic(message(e));
  }
}

gyro_chain::GyroChainConfig<float> load_chain_config() noexcept {
  const gyro_chain::GyroChainConfig<float> c = chain_from_params();
  require_valid(c);
  return c;
}

void RateGroupStep::init(const rate::RateConfig<float>& rate_cfg, const mixer::MixerConfig<float>& mixer_cfg,
                         const gyro_chain::GyroChainConfig<float>& chain_cfg) noexcept {
  mixer_ = mixer_cfg;
  rate_.init(rate_cfg, mixer_cfg);
  chain_.init(chain_cfg);
  filtered_ = ImuSample{};
}

void RateGroupStep::filter(const ImuSample& s, bool rate_due) noexcept {
  if (rate_due) {
    chain_.update_notches(hal_rotor_speed());
  }
  filtered_ = s;
  if (!gyro_usable(s)) {
    return;
  }
  if (rate_due && rate_.seeding()) {
    chain_.reseed();
  }
  filtered_.gyro_rad_s = chain_.filter(s.gyro_rad_s);
}

Execution RateGroupStep::execute(const prim::Vec3<float>& setpoint, const prim::Vec3<float>& added,
                                 float thrust) noexcept {
  return finish(rate_.execute(filtered_, setpoint), added, thrust);
}

Execution RateGroupStep::execute_bypass(const prim::Vec3<float>& reference, const prim::Vec3<float>& added,
                                        float thrust) noexcept {
  return finish(rate_.execute_bypass(filtered_, reference), added, thrust);
}

Execution RateGroupStep::finish(const rate::RateOutput<float>& out, const prim::Vec3<float>& added,
                                float thrust) noexcept {
  Execution e;
  e.rate = out;
  e.request = out.torque + added;
  e.alloc = mixer::allocate(mixer_, mixer::Request<float>{thrust, e.request});
  rate_.record_allocation(e.request, e.alloc);
  e.dshot = mixer::thrust_to_dshot(mixer_, e.alloc.f);
  return e;
}

}  // namespace marv::rate_group
