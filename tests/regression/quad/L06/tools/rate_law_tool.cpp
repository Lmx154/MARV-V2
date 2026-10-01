// Drives the firmware rate law (fw/rate/include/marv/rate/rate_loop.hpp, RateLoop<float>) for test_rate_lead_checks.py:
// the tool's controller C(z) (tools/card/rate_lead.py) against the law it models. Compiled by the test with the host
// compiler against the fw headers (as gyro_chain_coeff_tool.cpp is).
//
// stdin: line 1 "kp ki kd T_f tau_ref period" (C hexadecimal floats, every axis gets the same values), then one
// execution per line "t_us y" (y a hexadecimal float, the gyro rate on every axis). Each execution is execute_bypass with
// reference 0: the output feedback on y the margins model (no prefilter, no setpoint). inertia, motor_tau and
// ff_filter_tau stay 0, so the law is the PID with the D low-pass and no feed-forward.
// stdout: per execution "u alpha" in hexadecimal: u the roll torque, alpha the D low-pass coefficient the law computes
// at that execution's dt (rate_loop.hpp: dt = float(dt_us) / float(1e6), alpha = 1 - exp(-dt / T_f) in float); alpha
// is 0x0p+0 at the first execution, which seeds and has no dt.
// Exit 2 on a malformed input, 3 when validate refuses the configuration.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "marv/rate/rate_loop.hpp"

// rate_loop.cpp makes these instantiations; its other definitions read the product parameter set, which this tool has
// not got, so the instantiation is made here from the same header.
template class marv::rate::RateLoop<float>;
template marv::rate::ConfigError marv::rate::validate<float>(const marv::rate::RateConfig<float>&) noexcept;

namespace marv::rate::detail {
[[noreturn]] void panic_period() noexcept { std::abort(); }
}  // namespace marv::rate::detail

int main() {
  using namespace marv;
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double tf = 0;
  double tau_ref = 0;
  double period = 0;
  if (std::scanf("%la %la %la %la %la %la", &kp, &ki, &kd, &tf, &tau_ref, &period) != 6) {
    return 2;
  }
  rate::RateConfig<float> c{};
  for (std::size_t a = 0; a < rate::kTorqueAxes; ++a) {
    c.kp[a] = static_cast<float>(kp);
    c.ki[a] = static_cast<float>(ki);
    c.kd[a] = static_cast<float>(kd);
    c.tau_ref[a] = static_cast<float>(tau_ref);
    c.d_filter_tau[a] = static_cast<float>(tf);
  }
  c.period = static_cast<float>(period);
  if (rate::validate(c) != rate::ConfigError::None) {
    return 3;
  }
  rate::RateLoop<float> loop;
  loop.init(c, mixer::MixerConfig<float>{});
  unsigned long long t = 0;
  unsigned long long t_prev = 0;
  double y = 0;
  bool first = true;
  while (std::scanf("%llu %la", &t, &y) == 2) {
    ImuSample s{};
    s.t_us = t;
    s.gyro_rad_s = prim::Vec3<float>(static_cast<float>(y), static_cast<float>(y), static_cast<float>(y));
    s.flags = imu_flag(ImuFlag::GyroValid);
    const rate::RateOutput<float> out = loop.execute_bypass(s, prim::Vec3<float>());
    float alpha = 0.0F;
    if (!first) {
      const float dt =
          static_cast<float>(t - t_prev) / static_cast<float>(prim::kMicrosecondsPerSecond);
      alpha = 1.0F - std::exp(-dt / c.d_filter_tau[0]);
    }
    std::printf("%a %a\n", static_cast<double>(out.torque[0]), static_cast<double>(alpha));
    first = false;
    t_prev = t;
  }
  return 0;
}
