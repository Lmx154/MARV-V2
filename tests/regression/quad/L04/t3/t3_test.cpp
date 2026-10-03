// The L4 T3 suite (quad spec 4 L4, decision 0005 "T3"; quad spec 4 L6 stage (c), decision 0014): the stage (c) rate path,
// the firmware GyroChain<float> in front of the firmware RateLoop<float> (D term with its low-pass), configured from the
// committed input fixture reference/rate_t3_inputs.txt (not from the live product parameters), is closed with the design
// plant J w' = u_m, tau u_m' = u - u_m in double, simulated at tick resolution with the exact per-tick zero-order hold,
// against the committed double oracle reference/rate_t3_oracle.py (see README.md for the rounding tolerance derivation).
//
// Harness: every tick the gyro sample is the plant omega at that tick as a float with GyroValid set (no sensor latency,
// this suite's convention) and goes through the chain as the rate-group step (fw/rate_group) composes it: on a rate tick
// update_notches first, then filter. T3 has no rotor-speed sample: update_notches gets every motor invalid, so every
// notch is bypassed and the chain is its low-pass. The controller runs every rate_loop_divisor ticks on the chain output
// with the stamp floor(tick num / den) microseconds, and its torque is held until the next execution (zero computation
// delay). The mixer is out of the loop and record_allocation is not called; the feed-forward is off (J = 0), as in the
// product parameters.
//
// Numbers in this file are one of: read from the product parameters, derived (the rule is stated), or a "scenario test
// value" named with its reason.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <marv/gyro_chain/gyro_chain.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/rotor_speed_sample.hpp>

namespace {

using namespace marv;
using marv::gyro_chain::GyroChain;
using marv::gyro_chain::GyroChainConfig;
using marv::rate::RateConfig;
using marv::rate::RateLoop;
using V3 = prim::Vec3<float>;

constexpr std::size_t kA = rate::kTorqueAxes;
constexpr std::array<const char*, kA> kAxisName{"roll", "pitch", "yaw"};
constexpr std::array<const char*, kA> kInertia{"inertia_xx", "inertia_yy", "inertia_zz"};

// Scenario test value: negative control (a), every gain multiplied by 1.1 (quad spec 4 L4 pass bar).
constexpr float kGainScale = 1.1F;
// Scenario test value: negative control (b), one tick of delay between the controller output and the plant (quad spec 4
// L4 pass bar).
constexpr unsigned kControlDelayTicks = 1;
// Derived: the horizon rule of the oracle, N = 1 + ceil(kHorizonTaus tau_ref / T); the value is the oracle's
// HORIZON_TAUS (a scenario test value: ten reference time constants, the step is settled to e^-10 of its size).
constexpr double kHorizonTaus = 10;

// ---- the committed reference -------------------------------------------------------------------------------------

struct Golden {
  std::size_t executions = 0;
  double setpoint = 0;
  std::map<std::string, double> numbers;  // l1_*, rho_*, tolerance
  std::vector<double> y;
};

struct Envelope {
  std::size_t executions = 0;
  double halving_max_change = 0;
  std::vector<double> lo;
  std::vector<double> hi;
};

std::string reference_path(const char* name) { return std::string(MARV_L4_T3_REFERENCE_DIR) + "/" + name; }

double parse(const std::string& text) { return std::stod(text); }  // decimal and hex floats

std::string read_all(const char* name) {
  std::ifstream f(reference_path(name));
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

// Removes '#' comment lines and trailing comments.
std::vector<std::string> tokens_of(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    line = line.substr(0, line.find('#'));
    std::istringstream words(line);
    std::string w;
    while (words >> w) {
      out.push_back(w);
    }
  }
  return out;
}

std::map<std::string, Golden> read_golden() {
  std::map<std::string, Golden> out;
  const std::vector<std::string> t = tokens_of(read_all("rate_t3_golden.txt"));
  std::size_t i = 0;
  while (i < t.size()) {
    if (t[i] != "axis") {
      return {};
    }
    Golden g;
    const std::string axis = t[i + 1];
    i += 2;
    while (i < t.size() && t[i] != "trajectory") {
      const std::string& key = t[i];
      if (key == "executions") {
        g.executions = static_cast<std::size_t>(parse(t[i + 1]));
      } else if (key == "setpoint") {
        g.setpoint = parse(t[i + 1]);
      } else {
        g.numbers[key] = parse(t[i + 1]);
      }
      i += 2;
    }
    ++i;
    for (std::size_t n = 0; n < g.executions && i < t.size(); ++n, ++i) {
      g.y.push_back(parse(t[i]));
    }
    out[axis] = g;
  }
  return out;
}

std::map<std::string, Envelope> read_envelope() {
  std::map<std::string, Envelope> out;
  const std::vector<std::string> t = tokens_of(read_all("rate_t3_envelope.txt"));
  std::size_t i = 0;
  while (i < t.size()) {
    if (t[i] != "axis") {
      return {};
    }
    Envelope e;
    const std::string axis = t[i + 1];
    i += 2;
    while (i < t.size() && t[i] != "envelope_min_max") {
      if (t[i] == "executions") {
        e.executions = static_cast<std::size_t>(parse(t[i + 1]));
      } else if (t[i] == "halving_max_change") {
        e.halving_max_change = parse(t[i + 1]);
      }
      i += 2;
    }
    ++i;
    for (std::size_t n = 0; n < e.executions && i + 1 < t.size(); ++n, i += 2) {
      e.lo.push_back(parse(t[i]));
      e.hi.push_back(parse(t[i + 1]));
    }
    out[axis] = e;
  }
  return out;
}

const std::map<std::string, Golden>& golden() {
  static const std::map<std::string, Golden> g = read_golden();
  return g;
}

const std::map<std::string, Envelope>& envelope() {
  static const std::map<std::string, Envelope> e = read_envelope();
  return e;
}

// ---- the fixture -------------------------------------------------------------------------------------------------
// reference/rate_t3_inputs.txt: the f32 values the oracle used (hex floats, exact). It is the test's own fixture; it
// equals the product parameters of 2026-10-01 (stage (c), decision 0014) and does not follow later changes to them. The
// rotor geometry, the mixer and the anti-windup are out of the loop here, so the geometry stays at its zero default.

struct Product {
  RateConfig<float> cfg;
  GyroChainConfig<float> chain;
  std::array<double, kA> inertia{};
  std::array<double, kA> rate_max{};
  double motor_tau = 0;
  std::uint64_t num_us = 0;
  std::uint64_t den = 0;
  std::uint64_t divisor = 0;
  double tick_s = 0;
  double period_s = 0;
};

const Product* product() {
  static const Product* built = []() -> const Product* {
    std::map<std::string, float> v;
    const std::vector<std::string> t = tokens_of(read_all("rate_t3_inputs.txt"));
    for (std::size_t i = 0; i + 1 < t.size(); i += 2) {
      v[t[i]] = static_cast<float>(parse(t[i + 1]));
    }
    static Product p;
    for (std::size_t a = 0; a < kA; ++a) {
      const std::string axis = kAxisName[a];
      p.cfg.kp[a] = v.at("rate_kp_" + axis);
      p.cfg.ki[a] = v.at("rate_ki_" + axis);
      p.cfg.kd[a] = v.at("rate_kd_" + axis);
      p.cfg.d_filter_tau[a] = v.at("rate_d_filter_tau_" + axis);
      p.cfg.tau_ref[a] = v.at("rate_tau_ref_" + axis);
      p.inertia[a] = static_cast<double>(v.at(kInertia[a]));
      p.rate_max[a] = static_cast<double>(v.at("rate_max_" + axis));
    }
    p.motor_tau = static_cast<double>(v.at("motor_tau"));
    p.num_us = static_cast<std::uint64_t>(v.at("tick_period_num_us"));
    p.den = static_cast<std::uint64_t>(v.at("tick_period_den"));
    p.divisor = static_cast<std::uint64_t>(v.at("rate_loop_divisor"));
    // The period as rate::from_params forms it: an exact float quotient in microseconds, then one division.
    p.cfg.period = v.at("rate_loop_divisor") * v.at("tick_period_num_us") / v.at("tick_period_den") /
                   static_cast<float>(prim::kMicrosecondsPerSecond);
    p.tick_s = static_cast<double>(p.num_us) / static_cast<double>(p.den) / prim::kMicrosecondsPerSecond;
    p.period_s = static_cast<double>(p.divisor) * p.tick_s;
    // The chain as rate_group::chain_from_params forms it: the tick period as one float quotient in microseconds, then
    // one division; the first sample seeds the states.
    p.chain.period =
        v.at("tick_period_num_us") / v.at("tick_period_den") / static_cast<float>(prim::kMicrosecondsPerSecond);
    p.chain.rate_divisor = static_cast<std::uint32_t>(p.divisor);
    p.chain.cutoff_hz = v.at("gyro_lpf_cutoff_hz");
    p.chain.notch_q = {v.at("gyro_notch_q_h1"), v.at("gyro_notch_q_h2"), v.at("gyro_notch_q_h3")};
    p.chain.omega_threshold_rad_s = v.at("gyro_notch_omega_min");
    p.chain.seed_first_sample = true;
    const bool valid = rate::validate(p.cfg) == rate::ConfigError::None &&
                       gyro_chain::validate(p.chain) == gyro_chain::ConfigError::None;
    return valid ? &p : nullptr;
  }();
  return built;
}

#define REQUIRE_PRODUCT()                       \
  const Product* const pp = product();          \
  ASSERT_NE(pp, nullptr) << "reading rate_t3_inputs.txt"; \
  [[maybe_unused]] const Product& prod = *pp; \
  ASSERT_FALSE(golden().empty()) << "reading rate_t3_golden.txt"; \
  ASSERT_FALSE(envelope().empty()) << "reading rate_t3_envelope.txt"

// ---- the closed loop -----------------------------------------------------------------------------------------------

// The exact per-tick zero-order hold of J w' = u_m, tau u_m' = u - u_m for u held over a tick h:
//   u_m' = u_m e + u (1 - e),  w' = w + (u h + (u_m - u) tau (1 - e)) / J,  e = exp(-h / tau).
struct Plant {
  double inertia = 1;
  double tau = 1;
  double h = 0;
  double w = 0;
  double um = 0;

  void tick(double u) {
    const double x = h / tau;
    const double e = std::exp(-x);
    const double em = -std::expm1(-x);
    w += (u * (h - tau * em) + um * tau * em) / inertia;
    um = e * um + em * u;
  }
};

// Runs a step of the setpoint on `axis` (0 -> rate_max at execution 1, the others 0) for n_exec executions and returns
// the plant omega of `axis` at each execution (before that execution's output acts). `delay_ticks` ticks after an
// execution still apply the previous torque. `clean` is false if the rate loop raised a fault or a notch was not
// bypassed.
std::vector<double> run_step(const Product& prod, const RateConfig<float>& cfg, std::size_t axis, std::size_t n_exec,
                             unsigned delay_ticks, bool* clean = nullptr, double* other_axes_max = nullptr) {
  RateLoop<float> loop;
  loop.init(cfg, mixer::MixerConfig<float>());
  GyroChain<float> chain;
  chain.init(prod.chain);
  const RotorSpeedSample no_rotor_speed{};  // every motor invalid: every notch bypassed
  std::array<Plant, kA> plant;
  for (std::size_t a = 0; a < kA; ++a) {
    plant[a].inertia = prod.inertia[a];
    plant[a].tau = prod.motor_tau;
    plant[a].h = prod.tick_s;
  }
  V3 sp;
  sp[axis] = static_cast<float>(prod.rate_max[axis]);
  std::array<double, kA> u_old{};
  std::vector<double> y;
  bool ok = true;
  double other = 0;
  for (std::size_t n = 0; n < n_exec; ++n) {
    y.push_back(plant[axis].w);
    std::array<double, kA> u_new{};
    for (std::uint64_t k = 0; k < prod.divisor; ++k) {
      const std::uint64_t tick = prod.divisor * n + k;
      ImuSample s{};
      s.t_us = tick * prod.num_us / prod.den;
      s.flags = imu_flag(ImuFlag::GyroValid);
      for (std::size_t a = 0; a < kA; ++a) {
        s.gyro_rad_s[a] = static_cast<float>(plant[a].w);
      }
      if (k == 0) {
        chain.update_notches(no_rotor_speed);
        ok = ok && chain.bypass_flags() == kRotorSpeedFlagsDefined;
      }
      s.gyro_rad_s = chain.filter(s.gyro_rad_s);
      if (k == 0) {
        const rate::RateOutput<float> out = loop.execute(s, sp);
        ok = ok && !out.fault_active;
        for (std::size_t a = 0; a < kA; ++a) {
          u_new[a] = static_cast<double>(out.torque[a]);
        }
      }
      for (std::size_t a = 0; a < kA; ++a) {
        plant[a].tick(k < delay_ticks ? u_old[a] : u_new[a]);
      }
    }
    u_old = u_new;
    for (std::size_t a = 0; a < kA; ++a) {
      if (a != axis) {
        other = std::max(other, std::abs(plant[a].w));
      }
    }
  }
  if (clean != nullptr) {
    *clean = ok;
  }
  if (other_axes_max != nullptr) {
    *other_axes_max = other;
  }
  return y;
}

RateConfig<float> scaled(RateConfig<float> cfg, float k) {
  for (std::size_t a = 0; a < kA; ++a) {
    cfg.kp[a] *= k;
    cfg.ki[a] *= k;
    cfg.kd[a] *= k;
  }
  return cfg;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0;
  for (std::size_t n = 0; n < std::min(a.size(), b.size()); ++n) {
    m = std::max(m, std::abs(a[n] - b[n]));
  }
  return m;
}

// ---- 1. the reference is self-consistent ----------------------------------------------------------------------------

TEST(L4T3Reference, HorizonFollowsTheRuleAndTheSetpointIsRateMax) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    const double q = kHorizonTaus * static_cast<double>(prod.cfg.tau_ref[a]) / prod.period_s;
    // N - 1 = ceil(q), computed here without a ceil so a rounding of q at an integer cannot flip it.
    EXPECT_GE(static_cast<double>(g.executions - 1), q * (1 - 1e-12)) << kAxisName[a];
    EXPECT_LT(static_cast<double>(g.executions - 1), q + 1) << kAxisName[a];
    EXPECT_EQ(g.setpoint, prod.rate_max[a]) << kAxisName[a];
    EXPECT_EQ(g.y.size(), g.executions) << kAxisName[a];
    EXPECT_EQ(envelope().at(kAxisName[a]).executions, g.executions) << kAxisName[a];
  }
}

TEST(L4T3Reference, ToleranceIsTheSumOfL1TimesRho) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    double sum = 0;
    for (const char* node : {"x", "f", "r", "e", "I", "D", "u"}) {
      sum += g.numbers.at(std::string("l1_") + node) * g.numbers.at(std::string("rho_") + node);
    }
    EXPECT_NEAR(sum, g.numbers.at("tolerance"), 1e-12 * sum) << kAxisName[a];
  }
}

// ---- 2. the golden and its negative controls -------------------------------------------------------------------------

TEST(L4T3Step, FloatLoopMatchesTheDoubleOracleWithinTheDerivedTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    bool clean = false;
    double other = 1;
    const std::vector<double> y = run_step(prod, prod.cfg, a, g.executions, 0, &clean, &other);
    const double tol = g.numbers.at("tolerance");
    const double diff = max_abs_diff(y, g.y);
    std::printf("[T3] %-5s max|dy| = %.6e  tolerance = %.6e  (%.1f %% of the bound)\n", kAxisName[a], diff, tol,
                100 * diff / tol);
    EXPECT_TRUE(clean) << kAxisName[a] << ": a fault was raised or a notch was not bypassed";
    EXPECT_EQ(other, 0.0) << kAxisName[a] << ": the other axes must stay at rest";
    EXPECT_EQ(y.size(), g.y.size());
    EXPECT_LE(diff, tol) << kAxisName[a];
    // The step is a step: the response starts at rest and ends near the setpoint.
    EXPECT_EQ(y.front(), 0.0);
    EXPECT_LT(std::abs(y.back() - prod.rate_max[a]), 0.5 * prod.rate_max[a]) << kAxisName[a];
  }
}

TEST(L4T3Step, NegativeControlGainsTimesOnePointOneLeaveTheTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    const std::vector<double> y = run_step(prod, scaled(prod.cfg, kGainScale), a, g.executions, 0);
    const double tol = g.numbers.at("tolerance");
    const double diff = max_abs_diff(y, g.y);
    std::printf("[T3] %-5s control gains x %.1f: max|dy| = %.6e  tolerance = %.6e  (x%.1f)\n", kAxisName[a],
                static_cast<double>(kGainScale), diff, tol, diff / tol);
    EXPECT_GT(diff, tol) << kAxisName[a] << ": the golden cannot detect a gain error";
  }
}

TEST(L4T3Step, NegativeControlOneTickOfDelayLeavesTheTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    const std::vector<double> y = run_step(prod, prod.cfg, a, g.executions, kControlDelayTicks);
    const double tol = g.numbers.at("tolerance");
    const double diff = max_abs_diff(y, g.y);
    std::printf("[T3] %-5s control delay %u tick: max|dy| = %.6e  tolerance = %.6e  (x%.1f)\n", kAxisName[a],
                kControlDelayTicks, diff, tol, diff / tol);
    EXPECT_GT(diff, tol) << kAxisName[a] << ": the golden cannot detect a one-tick delay";
  }
}

// ---- 3. the band envelope (recorded, robustness only) ------------------------------------------------------------------

TEST(L4T3Envelope, NominalFloatTrajectoryLiesInsideTheBandEnvelope) {
  REQUIRE_PRODUCT();
  for (std::size_t a = 0; a < kA; ++a) {
    const Golden& g = golden().at(kAxisName[a]);
    const Envelope& env = envelope().at(kAxisName[a]);
    const std::vector<double> y = run_step(prod, prod.cfg, a, g.executions, 0);
    ASSERT_EQ(env.lo.size(), y.size());
    std::size_t outside = 0;
    for (std::size_t n = 0; n < y.size(); ++n) {
      if (y[n] < env.lo[n] || y[n] > env.hi[n]) {
        ++outside;
      }
    }
    EXPECT_EQ(outside, 0U) << kAxisName[a] << ": executions outside the envelope";
    EXPECT_GT(env.halving_max_change, 0.0);
  }
}

}  // namespace
