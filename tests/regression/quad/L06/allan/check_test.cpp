// L6 stage (a), T1 (quad spec section 4, L6 pass bar (a)): the simulated IMU's Allan deviation matches the profile's noise
// density and bias instability, and the two negative controls (noise density x 1.1, bias instability x 1.1) do not.
//
// The model alone, through the C ABI: a plant with its rotors at rest and a body at rest, so the truth is constant (gyro 0,
// specific force exactly 0 at zero thrust) and the measurement is the bias walk, the white noise and the quantisation.
// The IMU configuration is the generated profile header's (marv::sim::imu_profile_config()), the sample interval tau0 the
// tick period of design/scenario_values.yaml, the confidence allan_check_confidence of design/budget.yaml.
//
// Checks. Per channel (gyro x y z, accel x y z), the overlapping Allan variance (NIST SP 1065 section 5.2.4 eq. 11), streamed
// on the integer counts, at two averaging factors:
//  - the noise-density check at tau_N = tau0 (m = 1);
//  - the bias-instability check at tau_B = m* tau0, m* = round((N / B)^2 / tau0) (tau* = (N / B)^2).
// The expected value is the closed form of allan.hpp. The bound is the two-sided chi-square interval of the variance ratio
// with the overlapping Allan variance's equivalent degrees of freedom (SP 1065 section 5.3.2 eq. 44-45, section 5.4.1
// Table 5; the smaller of the W FM and RW FM edf at each factor, N = n + 1 phase points) at the per-check level
// alpha = (1 - allan_check_confidence) / 12 (Bonferroni over the 6 x 2 checks: the checks on one record are not
// independent, and Bonferroni holds under any dependence; owner decision B, decision 0012).
//
// Record length. The shortest record, in samples, at which the perturbed model's own interval lies wholly outside the nominal
// interval: expected_perturbed / expected_nominal >= chi2_upper / chi2_lower at the record's edf, for each check
// (allan.hpp, shortest_record). The twelve checks share one record, the longest of the four classes (gyro and accel, noise
// density and bias instability; the three axes of a class are identical). It is derived here, not stored.
//
// Seed. The plant's noise streams are keyed by rng_seed (stream 0, the primary IMU). Every run here, nominal and control,
// uses kSeed. It is a labelled test value, written before the first run; if a check fails at it the seed is not changed.
// The controls reuse the seed, so each control's noise is the nominal run's noise scaled by the perturbation: a control
// does not draw an independent sample, it is the nominal realisation of the perturbed model.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "allan.hpp"
#include "allan_register.hpp"
#include "marv/sim/imu_profile_config.hpp"
#include "support.hpp"

namespace {

using namespace marv::l06;
using namespace marv::l06::allan;
namespace imu = marv::plant::imu_test;

constexpr std::uint64_t kSeed = 1;  // labelled test value (owner, 2026-10-01); never changed after a result
constexpr std::size_t kChannels = 6;
constexpr std::size_t kChecksPerChannel = 2;
constexpr std::size_t kChecks = kChannels * kChecksPerChannel;  // 12, derived
constexpr double kPerturbation = 1.1;                           // the spec's x 1.1
constexpr double kMicrosecondsPerSecond = 1.0e6;                // unit conversion
// The counts are recovered from the float outputs as round(out / lsb); the float rounding of count * lsb is far below
// half a count, and this is the labelled bound on the residual (counts) that proves it.
constexpr double kCountResidualMax = 0.01;

constexpr double tick_period_s() {
  return static_cast<double>(kTickPeriodNumUs) / (static_cast<double>(kTickPeriodDen) * kMicrosecondsPerSecond);
}
constexpr double check_alpha() { return (1.0 - kAllanCheckConfidence) / static_cast<double>(kChecks); }

Axis axis_of(const marv_plant_imu_axis_config& a) { return {a.noise_density, a.bias_instability, a.lsb}; }

const char* const kChannelNames[kChannels] = {"gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"};
constexpr bool is_gyro(std::size_t ch) { return ch < kChannels / 2; }

struct Plan {
  std::array<std::size_t, 2> m_star{};          // gyro, accel: the bias-instability factor
  std::array<std::size_t, 2> noise_record{};    // gyro, accel: the shortest record separating N x 1.1 at m = 1
  std::array<std::size_t, 2> bias_record{};     // gyro, accel: the shortest record separating B x 1.1 at m*
  std::size_t record = 0;                       // their maximum, the shared record
};

Plan make_plan(const marv_plant_imu_config& profile) {
  Plan p;
  const std::array<Axis, 2> axes = {axis_of(profile.gyro), axis_of(profile.accel)};
  for (std::size_t c = 0; c < 2; ++c) {
    const Axis& nominal = axes[c];
    Axis noise_up = nominal;
    noise_up.noise_density *= kPerturbation;
    Axis bias_up = nominal;
    bias_up.bias_instability *= kPerturbation;
    p.m_star[c] = bias_averaging_factor(nominal, tick_period_s());
    p.noise_record[c] = shortest_record(nominal, noise_up, tick_period_s(), 1, check_alpha());
    p.bias_record[c] = shortest_record(nominal, bias_up, tick_period_s(), p.m_star[c], check_alpha());
    p.record = std::max({p.record, p.noise_record[c], p.bias_record[c]});
  }
  return p;
}

struct Measured {
  std::array<double, kChannels> var_noise{};  // overlapping Allan variance at m = 1, (rad/s)^2 or (m/s^2)^2
  std::array<double, kChannels> var_bias{};   // at m*
  std::size_t flagged = 0;                    // samples whose flags differ from gyro and accel valid, nothing else
  double max_residual = 0.0;                  // max |out / lsb - count| over every sample and channel
  double seconds = 0.0;                       // wall time of the run
};

Measured run(const marv_plant_imu_config& cfg, const Plan& plan) {
  const auto t0 = std::chrono::steady_clock::now();
  imu::PlantHandle plant(kSeed, 0.0);  // rotors at rest: the specific force is exactly 0
  EXPECT_EQ(marv_plant_imu_attach(plant.get(), &cfg), MARV_PLANT_OK);
  const marv_plant_body body = imu::identity_body();  // at rest: the truth rate is 0
  std::array<OverlappingAllan, kChannels> noise = {OverlappingAllan(1), OverlappingAllan(1), OverlappingAllan(1),
                                                   OverlappingAllan(1), OverlappingAllan(1), OverlappingAllan(1)};
  std::array<OverlappingAllan, kChannels> bias = {
      OverlappingAllan(plan.m_star[0]), OverlappingAllan(plan.m_star[0]), OverlappingAllan(plan.m_star[0]),
      OverlappingAllan(plan.m_star[1]), OverlappingAllan(plan.m_star[1]), OverlappingAllan(plan.m_star[1])};
  Measured out;
  const std::array<double, kChannels> lsb = {cfg.gyro.lsb, cfg.gyro.lsb, cfg.gyro.lsb,
                                             cfg.accel.lsb, cfg.accel.lsb, cfg.accel.lsb};
  for (std::size_t k = 0; k < plan.record; ++k) {
    marv_plant_imu_out o{};
    if (marv_plant_imu_sample(plant.get(), &body, tick_period_s(), &o) != MARV_PLANT_OK) {
      ADD_FAILURE() << "marv_plant_imu_sample refused at sample " << k;
      return out;
    }
    if (o.flags != imu::kValidBits) {
      ++out.flagged;
    }
    const float v[kChannels] = {o.gyro_rad_s.x, o.gyro_rad_s.y, o.gyro_rad_s.z,
                                o.accel_m_s2.x, o.accel_m_s2.y, o.accel_m_s2.z};
    for (std::size_t ch = 0; ch < kChannels; ++ch) {
      const double scaled = static_cast<double>(v[ch]) / lsb[ch];
      const auto count = static_cast<std::int64_t>(std::llround(scaled));
      out.max_residual = std::max(out.max_residual, std::fabs(scaled - static_cast<double>(count)));
      noise[ch].push(count);
      bias[ch].push(count);
    }
  }
  for (std::size_t ch = 0; ch < kChannels; ++ch) {
    out.var_noise[ch] = noise[ch].variance(lsb[ch]);
    out.var_bias[ch] = bias[ch].variance(lsb[ch]);
  }
  out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

struct Outcome {
  std::size_t channel;
  bool bias_check;
  double nu;
  double sigma_hat;
  double sigma_expected;
  double lower;   // ADEV interval edges around the expected value
  double upper;
  double margin;  // min(hat / lower, upper / hat): >= 1 inside the bound, < 1 outside
  bool inside;
};

std::array<Outcome, kChecks> judge(const Measured& m, const marv_plant_imu_config& profile, const Plan& plan) {
  std::array<Outcome, kChecks> r{};
  const std::array<Axis, 2> axes = {axis_of(profile.gyro), axis_of(profile.accel)};
  for (std::size_t ch = 0; ch < kChannels; ++ch) {
    const std::size_t cls = is_gyro(ch) ? 0 : 1;
    for (std::size_t kind = 0; kind < kChecksPerChannel; ++kind) {
      const bool bias = kind == 1;
      const std::size_t factor = bias ? plan.m_star[cls] : 1;
      const double expected_var = expected_variance(axes[cls], tick_period_s(), factor);
      const Interval iv = ratio_interval(edf(plan.record, factor), check_alpha());
      Outcome o{};
      o.channel = ch;
      o.bias_check = bias;
      o.nu = iv.nu;
      o.sigma_hat = std::sqrt(bias ? m.var_bias[ch] : m.var_noise[ch]);
      o.sigma_expected = std::sqrt(expected_var);
      o.lower = o.sigma_expected * std::sqrt(iv.lower);
      o.upper = o.sigma_expected * std::sqrt(iv.upper);
      o.margin = std::min(o.sigma_hat / o.lower, o.upper / o.sigma_hat);
      o.inside = o.sigma_hat >= o.lower && o.sigma_hat <= o.upper;
      r[ch * kChecksPerChannel + kind] = o;
    }
  }
  return r;
}

void print_plan(const marv_plant_imu_config& profile, const Plan& plan) {
  std::printf("[allan] tau0 = %.9g s, alpha per check = %.6g (confidence %.6g over %zu checks), seed %llu\n", tick_period_s(),
              check_alpha(), kAllanCheckConfidence, kChecks, static_cast<unsigned long long>(kSeed));
  const char* const cls_name[2] = {"gyro", "accel"};
  const std::array<Axis, 2> axes = {axis_of(profile.gyro), axis_of(profile.accel)};
  for (std::size_t c = 0; c < 2; ++c) {
    const double sigma_d2 = axes[c].noise_density * axes[c].noise_density / (2.0 * tick_period_s());
    std::printf("[allan] %s: N = %.6g, B = %.6g, lsb = %.6g, m* = %zu (tau* = %.6g s); record for N x 1.1 at m = 1: %zu, "
                "for B x 1.1 at m*: %zu samples; quantisation lsb^2/12 over sigma_d^2 = %.3g\n",
                cls_name[c], axes[c].noise_density, axes[c].bias_instability, axes[c].lsb, plan.m_star[c],
                static_cast<double>(plan.m_star[c]) * tick_period_s(), plan.noise_record[c], plan.bias_record[c],
                axes[c].lsb * axes[c].lsb / 12.0 / sigma_d2);
  }
  std::printf("[allan] record = %zu samples = %.6g s\n", plan.record, static_cast<double>(plan.record) * tick_period_s());
}

void print_outcomes(const char* title, const std::array<Outcome, kChecks>& r, const Measured& m) {
  std::printf("[allan] %s (wall %.3f s, flagged samples %zu, max count residual %.3g)\n", title, m.seconds, m.flagged,
              m.max_residual);
  for (const Outcome& o : r) {
    std::printf("[allan]   %-8s %-5s nu=%-12.6g sigma_hat=%-12.6g expected=%-12.6g bound=[%.6g, %.6g] hat/exp=%.5f margin=%.4f %s\n",
                kChannelNames[o.channel], o.bias_check ? "bias" : "noise", o.nu, o.sigma_hat, o.sigma_expected, o.lower,
                o.upper, o.sigma_hat / o.sigma_expected, o.margin, o.inside ? "inside" : "OUTSIDE");
  }
}

TEST(AllanT1, RecordLengthIsDerivedByTheRule) {
  const marv_plant_imu_config profile = marv::sim::imu_profile_config();
  const Plan plan = make_plan(profile);
  print_plan(profile, plan);
  const std::array<Axis, 2> axes = {axis_of(profile.gyro), axis_of(profile.accel)};
  for (std::size_t c = 0; c < 2; ++c) {
    Axis noise_up = axes[c];
    noise_up.noise_density *= kPerturbation;
    Axis bias_up = axes[c];
    bias_up.bias_instability *= kPerturbation;
    const double ratio = axes[c].noise_density / axes[c].bias_instability;
    EXPECT_NEAR(static_cast<double>(plan.m_star[c]) * tick_period_s(), ratio * ratio, tick_period_s());  // tau_B = tau*
    ASSERT_GT(plan.noise_record[c], 0U);
    ASSERT_GT(plan.bias_record[c], 0U);
    EXPECT_TRUE(separates(axes[c], noise_up, tick_period_s(), 1, plan.noise_record[c], check_alpha()));
    EXPECT_FALSE(separates(axes[c], noise_up, tick_period_s(), 1, plan.noise_record[c] - 1, check_alpha()));
    EXPECT_TRUE(separates(axes[c], bias_up, tick_period_s(), plan.m_star[c], plan.bias_record[c], check_alpha()));
    EXPECT_FALSE(separates(axes[c], bias_up, tick_period_s(), plan.m_star[c], plan.bias_record[c] - 1, check_alpha()));
    EXPECT_GE(plan.record, plan.noise_record[c]);
    EXPECT_GE(plan.record, plan.bias_record[c]);
  }
}

TEST(AllanT1, NominalMatchesTheProfile) {
  const marv_plant_imu_config profile = marv::sim::imu_profile_config();
  const Plan plan = make_plan(profile);
  const Measured m = run(profile, plan);
  const std::array<Outcome, kChecks> r = judge(m, profile, plan);
  print_plan(profile, plan);
  print_outcomes("nominal", r, m);
  EXPECT_EQ(m.flagged, 0U);
  EXPECT_LT(m.max_residual, kCountResidualMax);
  for (const Outcome& o : r) {
    EXPECT_TRUE(o.inside) << kChannelNames[o.channel] << (o.bias_check ? " bias" : " noise") << " hat " << o.sigma_hat
                          << " bound [" << o.lower << ", " << o.upper << "]";
  }
}

// A control run: the plant is the profile with one figure x 1.1 on both sensors, judged against the NOMINAL profile.
void expect_control_rejected(const char* title, bool perturb_noise_density) {
  const marv_plant_imu_config profile = marv::sim::imu_profile_config();
  marv_plant_imu_config control = profile;
  (perturb_noise_density ? control.gyro.noise_density : control.gyro.bias_instability) *= kPerturbation;
  (perturb_noise_density ? control.accel.noise_density : control.accel.bias_instability) *= kPerturbation;
  const Plan plan = make_plan(profile);
  const Measured m = run(control, plan);
  const std::array<Outcome, kChecks> r = judge(m, profile, plan);
  print_plan(profile, plan);
  print_outcomes(title, r, m);
  EXPECT_EQ(m.flagged, 0U);
  for (const Outcome& o : r) {
    if (o.bias_check == !perturb_noise_density) {  // the check the perturbation targets
      EXPECT_FALSE(o.inside) << kChannelNames[o.channel] << (o.bias_check ? " bias" : " noise") << " hat " << o.sigma_hat
                             << " bound [" << o.lower << ", " << o.upper << "]";
    }
  }
}

TEST(AllanT1, NoiseDensityPlus10PercentIsRejected) { expect_control_rejected("control: noise density x 1.1", true); }

TEST(AllanT1, BiasInstabilityPlus10PercentIsRejected) { expect_control_rejected("control: bias instability x 1.1", false); }

}  // namespace
