// The T2 bench-log replay of the ESKF in shadow (M4, decision 0055) beside Mahony, on the committed logs: the same parse, still
// segments, trim, checks (a) and (c') and controls as replay.hpp (decision 0042 item 2), with the estimator under test as a
// parameter. replay.hpp's Replayer and evaluate() hold a Mahony<float>; Replayer2 and evaluate_with below are the same code
// with the estimator and its configuration as template parameters, and the test first requires the Mahony run through them to
// equal replay.hpp's evaluate() exactly, so the ESKF is held to the same bar by the same code. The log's validity (two still
// segments, theta_rot >= pi/2, the 10 s window, each segment at least T_s = ln(theta_rot / share) / Kp) is the protocol's, at
// Mahony's flown Kp, whatever the estimator. One difference, the ESKF's contract (eskf.hpp initialisation): it is stepped only
// from the first sample the boot calibration has corrected (the completing one), since it starts with b = 0 and the
// calibration's residual as its bias prior; Mahony is stepped from the log's first sample, as replay.hpp does. Both must pass
// (a) and (c'), and each control must fail its check, for the ESKF as for Mahony.
//
// Reported per estimator besides the checks: the settling time in still 2, from the first sample of the untrimmed still 2 to the
// first sample from which the estimate's down direction stays within the share of still 2's -mean(f) direction up to the end of
// the trimmed still 2 (0 if it never leaves it); the bias estimate at the end of each still segment (after the boot
// calibration, so a residual); for the ESKF the accelerometer counts (fused, gated, tilt resets).
//
// Parameters: the FC3v2 set with the ESKF register (marv_params_fc3v2_eskf, fw/params/CMakeLists.txt), which holds every id of
// marv_params_fc3v2 with its value and the eskf_* entries.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <marv/params/param.hpp>

#include "marv/estimator/eskf.hpp"
#include "replay.hpp"

namespace marv::l07 {
namespace {

const std::vector<std::string> kRealLogs = {"t2_hand_rotation_2026-10-16.bin"};

// The frozen replay's still-segment trim (replay_test.cpp kTrimS, a test-harness scenario value; rationale in replay.hpp).
constexpr double kTrimS = 0.5;

bool params_ready() {
  static const bool ok = params_init(param_defaults());
  return ok;
}

Params read_params() {
  EXPECT_TRUE(params_ready());
  Params p;
  p.mahony = estimator::MahonyConfig<float>{param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
                                            param_value<ParamId::estimator_accel_healthy_min>(),
                                            param_value<ParamId::estimator_accel_healthy_max>()};
  p.gyro_cal = estimator::GyroCalConfig<float>{static_cast<std::uint64_t>(param_value<ParamId::gyro_cal_duration_us>()),
                                               param_value<ParamId::gyro_cal_movement_max>()};
  p.share = static_cast<double>(param_value<ParamId::attitude_tilt_error_max>());
  p.tick_num = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  p.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  p.tol_us = static_cast<std::uint32_t>(param_value<ParamId::rate_period_tolerance_us>());
  p.trim_s = kTrimS;
  return p;
}

estimator::EskfConfig<float> read_eskf() {
  EXPECT_TRUE(params_ready());
  estimator::EskfConfig<float> c;
  c.gyro_noise_density = param_value<ParamId::eskf_gyro_noise_density>();
  c.gyro_bias_random_walk = param_value<ParamId::eskf_gyro_bias_random_walk>();
  c.accel_noise_density = param_value<ParamId::eskf_accel_noise_density>();
  c.angle_sigma_init = param_value<ParamId::eskf_angle_sigma_init>();
  c.gyro_bias_sigma_init = param_value<ParamId::eskf_gyro_bias_sigma_init>();
  c.accel_gate = param_value<ParamId::eskf_accel_gate>();
  c.accel_reset_us = static_cast<std::uint64_t>(param_value<ParamId::eskf_accel_reset_us>());
  c.angle_var_min = param_value<ParamId::eskf_angle_var_min>();
  c.angle_var_max = param_value<ParamId::eskf_angle_var_max>();
  c.gyro_bias_var_min = param_value<ParamId::eskf_gyro_bias_var_min>();
  c.gyro_bias_var_max = param_value<ParamId::eskf_gyro_bias_var_max>();
  c.accel_healthy_min = param_value<ParamId::estimator_accel_healthy_min>();
  c.accel_healthy_max = param_value<ParamId::estimator_accel_healthy_max>();
  c.accel_fusion_divisor = static_cast<std::uint32_t>(param_value<ParamId::eskf_accel_fusion_divisor>());
  return c;
}

// Whether the estimator is stepped only on the calibrated gyro: the ESKF starts with b = 0 and the calibration's residual as its
// bias prior (eskf.hpp initialisation), so it takes no sample before the calibration completes; Mahony takes every sample.
template <class Est>
inline constexpr bool kCalibratedOnly = false;
template <>
inline constexpr bool kCalibratedOnly<estimator::Eskf<float>> = true;

// replay.hpp's Replayer with the estimator as a parameter: the boot gyro calibration, then the estimator step, quad_fc's flag rule.
template <class Est, class Cfg>
class Replayer2 {
 public:
  Replayer2(const Cfg& cfg, const estimator::GyroCalConfig<float>& cal, bool calibrate) : calibrate_(calibrate) {
    est_.init(cfg);
    cal_.init(cal);
  }
  void negate_gyro_axis(std::size_t axis) { negate_axis_ = static_cast<int>(axis); }
  void step(const ImuSample& raw, bool accel_on = true) {
    ImuSample s = raw;
    if (calibrate_) {
      cal_.update(raw.gyro_rad_s, (raw.flags & imu_flag(ImuFlag::GyroValid)) != 0, raw.t_us);
      if (cal_.complete()) {
        done_ = true;
        s.gyro_rad_s = raw.gyro_rad_s - cal_.bias();
      }
    }
    if (negate_axis_ >= 0) {
      s.gyro_rad_s[static_cast<std::size_t>(negate_axis_)] = -s.gyro_rad_s[static_cast<std::size_t>(negate_axis_)];
    }
    if ((s.flags & imu_flag(ImuFlag::GyroValid)) == 0 || (kCalibratedOnly<Est> && calibrate_ && !done_)) {
      return;
    }
    const std::uint64_t since = started_ && s.t_us > prev_us_ ? s.t_us - prev_us_ : 0;
    prev_us_ = s.t_us;
    started_ = true;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const prim::Vec3<float> no_accel{nan, nan, nan};
    const prim::Vec3<float> accel = accel_on && (s.flags & imu_flag(ImuFlag::AccelValid)) != 0 ? s.accel_m_s2 : no_accel;
    est_.step(s.gyro_rad_s, accel,
              since > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                : static_cast<std::uint32_t>(since));
  }
  [[nodiscard]] const Est& estimator() const { return est_; }
  [[nodiscard]] bool calibration_complete() const { return done_; }

 private:
  Est est_;
  estimator::GyroCal<float> cal_;
  bool calibrate_ = true;
  bool done_ = false;
  int negate_axis_ = -1;
  std::uint64_t prev_us_ = 0;
  bool started_ = false;
};

template <class Est>
[[nodiscard]] Dir down_of(const Est& e) {
  const auto& q = e.quat();
  const double w = q.w;
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  return {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
}

struct Extra {
  double settle2_s = 0.0;
  Dir bias_end1{};  // rad/s
  Dir bias_end2{};
  estimator::EskfCounts counts{};
};

template <class Est>
Dir bias_of(const Est& e) {
  return {static_cast<double>(e.bias()[0]), static_cast<double>(e.bias()[1]), static_cast<double>(e.bias()[2])};
}

// replay.hpp's evaluate() with the estimator as a parameter (the calibration report fields are not filled), plus Extra.
template <class Est, class Cfg>
Report evaluate_with(std::span<const ImuSample> samples, const Params& p, const Cfg& cfg, Control control, Extra* extra) {
  Report r;
  const std::vector<Segment> segs = two_longest(merge_runs(samples, still_runs(samples, p), p.share));
  if (segs.size() < 2) {
    r.invalid = "fewer than two still segments";
    return r;
  }
  r.raw1 = segs[0];
  r.raw2 = segs[1];
  if (!trimmed(samples, r.raw1, p.trim_s, r.still1) || !trimmed(samples, r.raw2, p.trim_s, r.still2)) {
    r.invalid = "a still segment is empty after the trim";
    return r;
  }
  const Dir f1 = mean_accel(samples, r.still1);
  const Dir f2 = mean_accel(samples, r.still2);
  r.theta_rot = angle_between(f1, f2);
  r.rotation_s = seconds(samples[r.raw1.last], samples[r.raw2.first]);
  r.still1_s = seconds(samples[r.still1.first], samples[r.still1.last]);
  r.still2_s = seconds(samples[r.still2.first], samples[r.still2.last]);
  if (!(p.mahony.kp > 0.0F) || !(p.share > 0.0)) {
    r.invalid = "Kp or the tilt share is not positive";
    return r;
  }
  r.ts_required_s = std::log(r.theta_rot / p.share) / static_cast<double>(p.mahony.kp);
  if (r.theta_rot < kRotationMinRad) {
    r.invalid = "theta_rot is below pi/2";
    return r;
  }
  if (r.rotation_s > kRotationWindowS) {
    r.invalid = "the rotation between the still segments takes longer than the protocol window";
    return r;
  }
  if (r.still1_s < r.ts_required_s || r.still2_s < r.ts_required_s) {
    r.invalid = "a still segment is shorter than T_s = ln(theta_rot / share) / Kp";
    return r;
  }
  std::size_t axis_a = 0;
  for (std::size_t a = 1; a < prim::kSpatialDim; ++a) {
    if (std::fabs(f2[a]) > std::fabs(f2[axis_a])) {
      axis_a = a;
    }
  }
  std::array<double, prim::kSpatialDim> rotated{};
  for (std::size_t k = r.still1.last + 1; k < r.still2.first; ++k) {
    const double dt = seconds(samples[k - 1], samples[k]);
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      rotated[a] += static_cast<double>(samples[k].gyro_rad_s[a]) * dt;
    }
  }
  std::size_t axis_c = 0;
  for (std::size_t a = 1; a < prim::kSpatialDim; ++a) {
    if (std::fabs(rotated[a]) > std::fabs(rotated[axis_c])) {
      axis_c = a;
    }
  }

  const bool calibrate = control != Control::NoCalibration;
  Replayer2<Est, Cfg> main(cfg, p.gyro_cal, calibrate);
  std::optional<Replayer2<Est, Cfg>> at_end1;
  bool ok_a = true;
  const Dir ref2 = unit({-f2[0], -f2[1], -f2[2]});
  std::size_t last_out2 = r.raw2.first;
  bool out2 = false;
  for (std::size_t k = 0; k <= r.still2.last; ++k) {
    ImuSample s = samples[k];
    if (control == Control::NegateStill2Accel && k >= r.still2.first) {
      s.accel_m_s2[axis_a] = -s.accel_m_s2[axis_a];
    }
    main.step(s);
    if (k >= r.raw2.first && angle_between(down_of(main.estimator()), ref2) > p.share) {
      last_out2 = k;
      out2 = true;
    }
    if (k == r.still1.last) {
      if (calibrate) {
        if (!main.calibration_complete()) {
          r.invalid = "the boot gyro calibration did not complete within still segment 1: the log does not start with the board still";
          return r;
        }
        r.calibrated = true;
      }
      at_end1 = main;
      r.err_a1 = angle_between(down_of(main.estimator()), unit({-f1[0], -f1[1], -f1[2]}));
      ok_a = ok_a && main.estimator().valid();
      if (extra != nullptr) {
        extra->bias_end1 = bias_of(main.estimator());
      }
    }
  }
  r.err_a2 = angle_between(down_of(main.estimator()), ref2);
  ok_a = ok_a && main.estimator().valid();
  r.pass_a = ok_a && r.err_a1 <= p.share && r.err_a2 <= p.share;
  if (extra != nullptr) {
    extra->bias_end2 = bias_of(main.estimator());
    extra->settle2_s = out2 && last_out2 < r.still2.last ? seconds(samples[r.raw2.first], samples[last_out2 + 1]) : 0.0;
    if (out2 && last_out2 >= r.still2.last) {
      extra->settle2_s = std::numeric_limits<double>::infinity();
    }
    if constexpr (requires { main.estimator().counts(); }) {
      extra->counts = main.estimator().counts();
    }
  }

  Replayer2<Est, Cfg> gyro_only = *at_end1;
  if (control == Control::NegateRotationGyro) {
    gyro_only.negate_gyro_axis(axis_c);
  }
  for (std::size_t k = r.still1.last + 1; k <= r.still2.last; ++k) {
    gyro_only.step(samples[k], false);
  }
  r.bound_c = r.theta_rot / 2.0;
  r.err_c = angle_between(down_of(gyro_only.estimator()), ref2);
  r.pass_c = gyro_only.estimator().valid() && r.err_c <= r.bound_c;
  return r;
}

std::vector<std::uint8_t> read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

constexpr double kRadToDeg = prim::kDegreesPerHalfTurn / prim::kPi;

void print_row(const char* who, const Report& r, const Extra& x, const Params& p) {
  std::printf("  %-6s (a) err1 %.3e err2 %.3e rad (<= %.5f): %s;  (c') err %.3e rad (<= %.5f): %s;  settle in still 2 %.3f s\n", who,
              r.err_a1, r.err_a2, p.share, r.pass_a ? "pass" : "FAIL", r.err_c, r.bound_c, r.pass_c ? "pass" : "FAIL",
              x.settle2_s);
  std::printf("         bias end of still 1 (%.4f, %.4f, %.4f) deg/s, end of still 2 (%.4f, %.4f, %.4f) deg/s\n",
              x.bias_end1[0] * kRadToDeg, x.bias_end1[1] * kRadToDeg, x.bias_end1[2] * kRadToDeg, x.bias_end2[0] * kRadToDeg,
              x.bias_end2[1] * kRadToDeg, x.bias_end2[2] * kRadToDeg);
}

TEST(L07EskfReplay, CommittedBenchLogsMahonyAndTheEskfSideBySide) {
  const Params p = read_params();
  const estimator::EskfConfig<float> eskf = read_eskf();
  using Mahony = estimator::Mahony<float>;
  using Eskf = estimator::Eskf<float>;
  for (const std::string& name : kRealLogs) {
    SCOPED_TRACE(name);
    const std::vector<std::uint8_t> bytes = read_file(std::string(MARV_L07_RESULTS_DIR) + "/" + name);
    ASSERT_FALSE(bytes.empty());
    const ParsedLog log = parse_log(bytes, p);
    ASSERT_TRUE(log.ok()) << log.error;

    // The generic path reproduces replay.hpp's evaluate() for Mahony exactly.
    const Report frozen = evaluate(log.samples, p);
    Extra xm;
    const Report m = evaluate_with<Mahony>(log.samples, p, p.mahony, Control::None, &xm);
    ASSERT_TRUE(frozen.valid()) << frozen.invalid;
    ASSERT_TRUE(m.valid()) << m.invalid;
    EXPECT_EQ(m.err_a1, frozen.err_a1);
    EXPECT_EQ(m.err_a2, frozen.err_a2);
    EXPECT_EQ(m.err_c, frozen.err_c);
    EXPECT_EQ(m.pass_a, frozen.pass_a);
    EXPECT_EQ(m.pass_c, frozen.pass_c);

    Extra xe;
    const Report e = evaluate_with<Eskf>(log.samples, p, eskf, Control::None, &xe);
    ASSERT_TRUE(e.valid()) << e.invalid;
    std::printf("%s: theta_rot %.6f rad, T_s required %.3f s (kp %.4f, share %.5f rad), still1 %.3f s, still2 %.3f s, rotation %.3f s\n",
                name.c_str(), e.theta_rot, e.ts_required_s, static_cast<double>(p.mahony.kp), p.share, e.still1_s, e.still2_s,
                e.rotation_s);
    print_row("Mahony", m, xm, p);
    print_row("ESKF", e, xe, p);
    std::printf("         ESKF accelerometer: fused %u, gated %u, tilt resets %u\n", xe.counts.fused, xe.counts.gated,
                xe.counts.resets);
    EXPECT_TRUE(e.pass_a);
    EXPECT_TRUE(e.pass_c);

    Extra xca;
    const Report ca = evaluate_with<Eskf>(log.samples, p, eskf, Control::NegateStill2Accel, &xca);
    std::printf("  control (a), ESKF: err2 %.3e rad: %s; fused %u, gated %u, tilt resets %u\n", ca.err_a2,
                ca.pass_a ? "pass" : "FAIL", xca.counts.fused, xca.counts.gated, xca.counts.resets);
    EXPECT_FALSE(ca.pass_a);
    const Report cc = evaluate_with<Eskf>(log.samples, p, eskf, Control::NegateRotationGyro, nullptr);
    std::printf("  control (c'), ESKF: err %.3e rad: %s\n", cc.err_c, cc.pass_c ? "pass" : "FAIL");
    EXPECT_FALSE(cc.pass_c);
  }
}

}  // namespace
}  // namespace marv::l07
