// The L5 T3 suite (quad spec 4 L5, decision 0006 F "T3 step" and "T3 envelopes"): the firmware path of angle mode, the attitude
// law and the rate loop in bypass (all float32), configured from the committed fixture reference/attitude_t3_inputs.txt (not
// from the live product parameters), is closed with the design plant in double at tick resolution and compared with the
// committed double oracle reference/attitude_t3_oracle.py (README.md has the rounding tolerance derivation).
//
// Harness. Tick j has the stamp floor(j num / den) us. The rate loop executes at j = 0 mod D (the seed execution at j = 0),
// the attitude group (angle mode, then the law) at j = 0 mod D N and before the rate group of the same tick; its rate setpoint
// is held until the next attitude execution, and the torque computed at tick j acts from tick j (zero computation delay). The
// plant per axis is J w' = u_m, tau u_m' = u - u_m with u held over a tick, exact per tick (and per kinematic sub-step), no
// w x Jw; the body quaternion is integrated in double from the exact per-axis angle increments of each sub-step. The
// firmware sees the plant's q and w as float32 with valid set.
//
// Numbers in this file are one of: read from the committed reference, derived (the rule is stated), or a "scenario test
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

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>

namespace {

using namespace marv;
using attitude::AngleMode;
using attitude::AnglePhase;
using attitude::AngleSticks;
using attitude::AttitudeConfig;
using attitude::AttitudeLaw;
using rate::RateConfig;
using rate::RateLoop;
using V3 = prim::Vec3<float>;

constexpr std::size_t kA = rate::kTorqueAxes;
constexpr std::array<const char*, kA> kAxisName{"roll", "pitch", "yaw"};
constexpr std::array<const char*, kA> kInertia{"inertia_xx", "inertia_yy", "inertia_zz"};
constexpr std::array<const char*, 3> kScenario{"step_roll", "step_pitch", "yaw_release"};
constexpr std::array<const char*, 4> kNode{"ref", "e", "I", "u"};

// Scenario test value: negative control (a), the attitude gain k multiplied by 1.1 (quad spec 4 L5 pass bar).
constexpr float kGainScale = 1.1F;
// Scenario test value: negative control (b), one tick of delay between the controller output and the plant (quad spec 4 L5
// pass bar).
constexpr unsigned kControlDelayTicks = 1;
// Derived: the segment length rule of the oracle, H = ceil(kHorizonTaus 2^i / (k T_a)) attitude executions for some whole
// number i of doublings; the value is the oracle's HORIZON_TAUS (a scenario test value: ten attitude time constants 1 / k).
constexpr double kHorizonTaus = 10;
// Scenario test value: the most doublings the oracle may have applied (its MAX_DOUBLINGS).
constexpr int kMaxDoublings = 4;
// Scenario test value: kinematic sub-steps per tick of the golden (the oracle's KIN_SUBSTEPS); the halving check runs twice
// this.
constexpr unsigned kKinSubsteps = 1;

// ---- the committed reference -------------------------------------------------------------------------------------------

struct Golden {
  std::string axis;
  std::size_t executions = 0;
  std::map<std::string, double> numbers;  // l1_*, rho_*, tolerance_*, kinematics_*, lock_*, release_execution
  std::vector<double> theta;
  std::vector<double> omega;
};

struct Channel {
  std::size_t first = 0;
  std::size_t count = 0;
  double halving = 0;
  std::vector<double> lo;
  std::vector<double> hi;
};

struct EnvelopeScenario {
  std::string axis;
  std::map<std::string, double> numbers;
  std::map<std::string, Channel> channel;
};

std::string reference_path(const char* name) { return std::string(MARV_L5_T3_REFERENCE_DIR) + "/" + name; }

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
  const std::vector<std::string> t = tokens_of(read_all("attitude_t3_golden.txt"));
  std::size_t i = 0;
  Golden* cur = nullptr;
  while (i < t.size()) {
    if (t[i] == "scenario") {
      cur = &out[t[i + 1]];
      i += 2;
    } else if (cur == nullptr) {
      return {};
    } else if (t[i] == "axis") {
      cur->axis = t[i + 1];
      i += 2;
    } else if (t[i] == "executions") {
      cur->executions = static_cast<std::size_t>(parse(t[i + 1]));
      i += 2;
    } else if (t[i] == "theta" || t[i] == "omega") {
      std::vector<double>& v = t[i] == "theta" ? cur->theta : cur->omega;
      ++i;
      for (std::size_t n = 0; n < cur->executions && i < t.size(); ++n, ++i) {
        v.push_back(parse(t[i]));
      }
    } else {
      cur->numbers[t[i]] = parse(t[i + 1]);
      i += 2;
    }
  }
  return out;
}

std::map<std::string, EnvelopeScenario> read_envelope() {
  std::map<std::string, EnvelopeScenario> out;
  const std::vector<std::string> t = tokens_of(read_all("attitude_t3_envelope.txt"));
  std::size_t i = 0;
  EnvelopeScenario* cur = nullptr;
  while (i < t.size()) {
    if (t[i] == "scenario") {
      cur = &out[t[i + 1]];
      i += 2;
    } else if (cur == nullptr) {
      return {};
    } else if (t[i] == "axis") {
      cur->axis = t[i + 1];
      i += 2;
    } else if (t[i] == "channel") {
      Channel& c = cur->channel[t[i + 1]];
      c.first = static_cast<std::size_t>(parse(t[i + 3]));
      c.count = static_cast<std::size_t>(parse(t[i + 5]));
      c.halving = parse(t[i + 7]);
      i += 8;  // channel NAME first F count C halving_max_change X
      ++i;     // min_max
      for (std::size_t n = 0; n < c.count && i + 1 < t.size(); ++n, i += 2) {
        c.lo.push_back(parse(t[i]));
        c.hi.push_back(parse(t[i + 1]));
      }
    } else {
      cur->numbers[t[i]] = parse(t[i + 1]);
      i += 2;
    }
  }
  return out;
}

const std::map<std::string, Golden>& golden() {
  static const std::map<std::string, Golden> g = read_golden();
  return g;
}

const std::map<std::string, EnvelopeScenario>& envelope() {
  static const std::map<std::string, EnvelopeScenario> e = read_envelope();
  return e;
}

// ---- the fixture -------------------------------------------------------------------------------------------------------
// reference/attitude_t3_inputs.txt: the f32 values the oracle used (hex floats, exact). It is the test's own fixture; it equals
// the product parameters of 2026-09-30 and does not follow later changes to them. The rotor geometry, the mixer and the
// anti-windup are out of the loop here, so the geometry stays at its zero default.

struct Product {
  RateConfig<float> rate;
  AttitudeConfig<float> att;
  std::array<double, kA> inertia{};
  double motor_tau = 0;
  std::uint64_t num_us = 0;
  std::uint64_t den = 0;
  std::uint64_t divisor = 0;
  std::uint64_t ratio = 0;
  double tick_s = 0;
  double period_s = 0;
  double att_period_s = 0;
};

const Product* product() {
  static const Product* built = []() -> const Product* {
    std::map<std::string, float> v;
    const std::vector<std::string> t = tokens_of(read_all("attitude_t3_inputs.txt"));
    for (std::size_t i = 0; i + 1 < t.size(); i += 2) {
      v[t[i]] = static_cast<float>(parse(t[i + 1]));
    }
    static Product p;
    for (std::size_t a = 0; a < kA; ++a) {
      const std::string axis = kAxisName[a];
      p.rate.kp[a] = v.at("rate_kp_" + axis);
      p.rate.ki[a] = v.at("rate_ki_" + axis);
      p.rate.kd[a] = v.at("rate_kd_" + axis);
      p.rate.tau_ref[a] = v.at("rate_tau_ref_" + axis);
      p.att.rate_max[a] = v.at("rate_max_" + axis);
      p.inertia[a] = static_cast<double>(v.at(kInertia[a]));
    }
    p.motor_tau = static_cast<double>(v.at("motor_tau"));
    p.num_us = static_cast<std::uint64_t>(v.at("tick_period_num_us"));
    p.den = static_cast<std::uint64_t>(v.at("tick_period_den"));
    p.divisor = static_cast<std::uint64_t>(v.at("rate_loop_divisor"));
    p.ratio = static_cast<std::uint64_t>(v.at("att_loop_ratio"));
    // The periods as the firmware forms them: an exact float quotient in microseconds, then one division.
    p.rate.period = v.at("rate_loop_divisor") * v.at("tick_period_num_us") / v.at("tick_period_den") /
                    static_cast<float>(prim::kMicrosecondsPerSecond);
    p.att.period = v.at("att_loop_ratio") * v.at("rate_loop_divisor") * v.at("tick_period_num_us") /
                   v.at("tick_period_den") / static_cast<float>(prim::kMicrosecondsPerSecond);
    p.att.kp = v.at("att_kp");
    p.att.yaw_weight = v.at("att_yaw_weight");
    p.att.tilt_max = v.at("angle_tilt_max");
    p.att.yaw_deadband = v.at("yaw_deadband");
    p.att.yaw_alpha_min = v.at("att_yaw_alpha_min");
    p.att.yaw_t_cross = v.at("att_yaw_t_cross");
    p.tick_s = static_cast<double>(p.num_us) / static_cast<double>(p.den) / prim::kMicrosecondsPerSecond;
    p.period_s = static_cast<double>(p.divisor) * p.tick_s;
    p.att_period_s = static_cast<double>(p.ratio) * p.period_s;
    return rate::validate(p.rate) == rate::ConfigError::None && attitude::validate(p.att) == attitude::ConfigError::None
               ? &p
               : nullptr;
  }();
  return built;
}

#define REQUIRE_PRODUCT()                                               \
  const Product* const pp = product();                                  \
  ASSERT_NE(pp, nullptr) << "reading attitude_t3_inputs.txt";           \
  [[maybe_unused]] const Product& prod = *pp;                           \
  ASSERT_EQ(golden().size(), kScenario.size()) << "reading attitude_t3_golden.txt"; \
  ASSERT_EQ(envelope().size(), kScenario.size() + 1) << "reading attitude_t3_envelope.txt"

// ---- the closed loop -----------------------------------------------------------------------------------------------------

struct Qd {
  double w = 1;
  double x = 0;
  double y = 0;
  double z = 0;
};

Qd mul(const Qd& a, const Qd& b) {
  return Qd{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// q = exp(phi / 2) of a rotation vector phi (rad).
Qd expq(const std::array<double, 3>& phi) {
  const double angle = std::sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
  if (angle == 0) {
    return Qd{};
  }
  const double s = std::sin(angle / 2) / angle;
  return Qd{std::cos(angle / 2), phi[0] * s, phi[1] * s, phi[2] * s};
}

// The exact zero-order-hold step of J w' = u_m, tau u_m' = u - u_m, theta' = w for u held over h, e = exp(-h / tau):
//   u_m' = u_m e + u (1 - e),  w' = w + (tau (1 - e) u_m + g u) / J,
//   theta' = theta + h w + (tau g u_m + (h^2 / 2 - tau g) u) / J,  g = h - tau (1 - e).
// It returns the angle increment of the step.
struct Plant {
  double inertia = 1;
  double tau = 1;
  double h = 0;
  double w = 0;
  double um = 0;
  double th = 0;

  double step(double u) {
    const double x = h / tau;
    const double e = std::exp(-x);
    const double em = -std::expm1(-x);
    const double g = h - tau * em;
    const double th_new = th + h * w + (tau * g * um + (h * h / 2 - tau * g) * u) / inertia;
    w += (tau * em * um + g * u) / inertia;
    um = e * um + em * u;
    const double d = th_new - th;
    th = th_new;
    return d;
  }
};

struct Trace {
  std::vector<double> theta;  // unwrapped angle of the scripted axis at each attitude execution
  std::vector<double> omega;
  std::size_t lock_execution = 0;  // the first attitude execution at or after the release with the lock (0: none)
  bool clean = true;               // no fault flag, every rate setpoint inside the bound
  double other_axes_max = 0;       // the largest |w| of the other two axes
};

std::size_t axis_of(std::size_t scenario) { return scenario; }

AngleSticks<float> sticks_of(std::size_t scenario, std::size_t a, std::size_t a_s, std::size_t a_r) {
  AngleSticks<float> s;
  const float v = (a >= a_s && a < a_r) ? 1.0F : 0.0F;
  (scenario == 0 ? s.roll : (scenario == 1 ? s.pitch : s.yaw)) = v;
  return s;
}

// One run of a script: `delay_ticks` ticks after a rate execution still apply the previous torque; `substeps` kinematic
// sub-steps per tick.
Trace run_script(const Product& prod, const AttitudeConfig<float>& acfg, const RateConfig<float>& rcfg, std::size_t scenario,
               std::size_t n_att, std::size_t a_r, unsigned delay_ticks, unsigned substeps) {
  AngleMode<float> angle;
  angle.init(acfg);
  AttitudeLaw<float> law;
  law.init(acfg);
  RateLoop<float> loop;
  loop.init(rcfg, mixer::MixerConfig<float>());
  std::array<Plant, kA> plant;
  for (std::size_t a = 0; a < kA; ++a) {
    plant[a].inertia = prod.inertia[a];
    plant[a].tau = prod.motor_tau;
    plant[a].h = prod.tick_s / static_cast<double>(substeps);
  }
  Qd q;
  V3 r_hold;
  std::array<double, kA> u_old{};
  std::array<double, kA> u_new{};
  Trace run;
  const std::size_t axis = axis_of(scenario);
  const std::uint64_t per_att = prod.divisor * prod.ratio;
  const std::size_t a_s = 1;
  for (std::size_t a = 0; a < n_att; ++a) {
    for (std::uint64_t n = 0; n < prod.ratio; ++n) {
      const std::uint64_t tick0 = per_att * a + prod.divisor * n;
      const std::uint64_t stamp = tick0 * prod.num_us / prod.den;
      if (n == 0) {
        AttitudeState<float> s;
        s.t_us = stamp;
        s.q = prim::Quat<float>(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                                static_cast<float>(q.z));
        for (std::size_t i = 0; i < kA; ++i) {
          s.omega_frd[i] = static_cast<float>(plant[i].w);
        }
        s.valid = true;
        const attitude::AngleOutput<float> out = angle.execute(sticks_of(scenario, a, a_s, a_r), s);
        const attitude::AttitudeOutput<float> cmd = law.execute(s, out.q_sp, out.yaw_rate_cmd);
        run.clean = run.clean && out.valid && !cmd.fault_active;
        r_hold = cmd.rate_setpoint;
        run.theta.push_back(plant[axis].th);
        run.omega.push_back(plant[axis].w);
        if (scenario == 2 && a >= a_r && run.lock_execution == 0 && angle.phase() == AnglePhase::Locked) {
          run.lock_execution = a;
        }
      }
      ImuSample imu{};
      imu.t_us = stamp;
      imu.flags = imu_flag(ImuFlag::GyroValid);
      for (std::size_t i = 0; i < kA; ++i) {
        imu.gyro_rad_s[i] = static_cast<float>(plant[i].w);
      }
      const rate::RateOutput<float> out = loop.execute_bypass(imu, r_hold);
      run.clean = run.clean && !out.fault_active;
      for (std::size_t i = 0; i < kA; ++i) {
        u_new[i] = static_cast<double>(out.torque[i]);
      }
      for (std::uint64_t k = 0; k < prod.divisor; ++k) {
        for (unsigned sub = 0; sub < substeps; ++sub) {
          std::array<double, 3> d{};
          for (std::size_t i = 0; i < kA; ++i) {
            d[i] = plant[i].step(k < delay_ticks ? u_old[i] : u_new[i]);
          }
          q = mul(q, expq(d));
        }
        const double nq = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        q = Qd{q.w / nq, q.x / nq, q.y / nq, q.z / nq};
      }
      u_old = u_new;
      for (std::size_t i = 0; i < kA; ++i) {
        if (i != axis) {
          run.other_axes_max = std::max(run.other_axes_max, std::abs(plant[i].w));
        }
      }
    }
  }
  return run;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0;
  for (std::size_t n = 0; n < std::min(a.size(), b.size()); ++n) {
    m = std::max(m, std::abs(a[n] - b[n]));
  }
  return m;
}

// The release execution of a script's golden (an integer stored as a double).
std::size_t release_of(const Golden& g) { return static_cast<std::size_t>(g.numbers.at("release_execution")); }

Trace golden_run(const Product& prod, std::size_t scenario, unsigned delay_ticks = 0,
               const AttitudeConfig<float>* acfg = nullptr, const RateConfig<float>* rcfg = nullptr,
               unsigned substeps = kKinSubsteps) {
  const Golden& g = golden().at(kScenario[scenario]);
  return run_script(prod, acfg != nullptr ? *acfg : prod.att, rcfg != nullptr ? *rcfg : prod.rate, scenario, g.executions,
                    release_of(g), delay_ticks, substeps);
}

RateConfig<float> scaled(RateConfig<float> cfg, float k) {
  for (std::size_t a = 0; a < kA; ++a) {
    cfg.kp[a] *= k;
    cfg.ki[a] *= k;
    cfg.kd[a] *= k;
  }
  return cfg;
}

// The largest ratio of a channel difference to its tolerance.
double factor(const Golden& g, const Trace& run) {
  return std::max(max_abs_diff(run.theta, g.theta) / g.numbers.at("tolerance_theta"),
                  max_abs_diff(run.omega, g.omega) / g.numbers.at("tolerance_omega"));
}

// ---- 1. the reference is self-consistent -------------------------------------------------------------------------------------

TEST(L5T3Reference, SegmentLengthFollowsTheRuleAndTheScriptsShareIt) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    EXPECT_EQ(g.axis, kAxisName[axis_of(s)]);
    EXPECT_EQ(g.theta.size(), g.executions) << kScenario[s];
    EXPECT_EQ(g.omega.size(), g.executions) << kScenario[s];
    EXPECT_EQ(g.executions, 1 + 2 * (release_of(g) - 1)) << kScenario[s];
    const std::size_t h = release_of(g) - 1;
    bool found = false;
    for (int i = 0; i <= kMaxDoublings; ++i) {
      const double q = kHorizonTaus * std::pow(2.0, i) / (static_cast<double>(prod.att.kp) * prod.att_period_s);
      found = found || (static_cast<double>(h) >= q * (1 - 1e-12) && static_cast<double>(h) < q + 1);
    }
    EXPECT_TRUE(found) << kScenario[s] << ": H = " << h << " is not the rule's for any doubling";
    EXPECT_EQ(release_of(golden().at(kScenario[0])), release_of(g));
  }
}

TEST(L5T3Reference, ToleranceIsTheSumOfL1TimesRho) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    for (const char* ch : {"theta", "omega"}) {
      double sum = 0;
      for (const char* node : kNode) {
        sum += g.numbers.at(std::string("l1_") + ch + "_" + node) * g.numbers.at(std::string("rho_") + node);
      }
      EXPECT_NEAR(sum, g.numbers.at(std::string("tolerance_") + ch), 1e-12 * sum) << kScenario[s] << " " << ch;
    }
  }
}

// ---- 2. the golden and its negative controls -------------------------------------------------------------------------------------

TEST(L5T3Step, FloatPathMatchesTheDoubleOracleWithinTheDerivedTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Trace run = golden_run(prod, s);
    const double tol_t = g.numbers.at("tolerance_theta");
    const double tol_w = g.numbers.at("tolerance_omega");
    const double dt = max_abs_diff(run.theta, g.theta);
    const double dw = max_abs_diff(run.omega, g.omega);
    std::printf("[T3] %-11s max|dtheta| = %.6e  tolerance = %.6e  (%.2f %%)   max|domega| = %.6e  tolerance = %.6e  (%.2f %%)\n",
                kScenario[s], dt, tol_t, 100 * dt / tol_t, dw, tol_w, 100 * dw / tol_w);
    EXPECT_TRUE(run.clean) << kScenario[s] << ": a fault was raised";
    EXPECT_EQ(run.other_axes_max, 0.0) << kScenario[s] << ": the other axes must stay at rest";
    EXPECT_EQ(run.theta.size(), g.theta.size());
    EXPECT_LE(dt, tol_t) << kScenario[s];
    EXPECT_LE(dw, tol_w) << kScenario[s];
    // A step is a step: the response starts at rest and ends near rest (the release).
    EXPECT_EQ(run.theta.front(), 0.0);
    EXPECT_EQ(run.omega.front(), 0.0);
  }
}

TEST(L5T3Step, YawLockIsAtTheGoldenExecutionAndItsDecisionMarginExceedsTheRateTolerance) {
  REQUIRE_PRODUCT();
  const Golden& g = golden().at("yaw_release");
  const Trace run = golden_run(prod, 2);
  EXPECT_EQ(run.lock_execution, static_cast<std::size_t>(g.numbers.at("lock_execution")));
  // The yaw rate at the execution before the lock has the release sign and at the lock it has not: both by more than the
  // rate tolerance plus the cast bound, so a rounding difference cannot move the lock.
  const double margin = std::min(g.numbers.at("lock_margin_before"), -g.numbers.at("lock_margin_at"));
  EXPECT_GT(margin, g.numbers.at("tolerance_omega") + g.numbers.at("lock_rate_bound"));
  std::printf("[T3] yaw lock at execution %zu, decision margin %.6e rad/s (rate tolerance %.6e)\n", run.lock_execution, margin,
              g.numbers.at("tolerance_omega"));
  const std::size_t l = run.lock_execution;
  ASSERT_GT(l, release_of(g));
  // The recorded margins are the oracle's own samples on both sides of the crossing.
  EXPECT_GT(g.omega[l - 1], 0.0);
  EXPECT_LE(g.omega[l], 0.0);
}

TEST(L5T3Step, KinematicsHalvingChangesTheTrajectoryByLessThanTheTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Trace coarse = golden_run(prod, s, 0, nullptr, nullptr, kKinSubsteps);
    const Trace fine = golden_run(prod, s, 0, nullptr, nullptr, 2 * kKinSubsteps);
    const double dt = max_abs_diff(coarse.theta, fine.theta);
    const double dw = max_abs_diff(coarse.omega, fine.omega);
    std::printf("[T3] %-11s kinematics halving: max|dtheta| = %.3e  max|domega| = %.3e\n", kScenario[s], dt, dw);
    EXPECT_LT(dt, g.numbers.at("tolerance_theta")) << kScenario[s];
    EXPECT_LT(dw, g.numbers.at("tolerance_omega")) << kScenario[s];
    // The oracle's recorded change is below the tolerance too (it refuses to write the reference otherwise).
    EXPECT_LT(g.numbers.at("kinematics_change_theta"), g.numbers.at("tolerance_theta")) << kScenario[s];
    EXPECT_LT(g.numbers.at("kinematics_change_omega"), g.numbers.at("tolerance_omega")) << kScenario[s];
  }
}

TEST(L5T3Step, NegativeControlAttitudeGainTimesOnePointOneLeavesTheTolerance) {
  REQUIRE_PRODUCT();
  AttitudeConfig<float> cfg = prod.att;
  cfg.kp *= kGainScale;
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Trace run = golden_run(prod, s, 0, &cfg);
    const double f = factor(g, run);
    std::printf("[T3] %-11s control attitude gain x %.1f: max(diff / tolerance) = x%.1f\n", kScenario[s],
                static_cast<double>(kGainScale), f);
    EXPECT_GT(f, 1.0) << kScenario[s] << ": the golden cannot detect an attitude gain error";
  }
}

TEST(L5T3Step, NegativeControlRateGainsTimesOnePointOneLeaveTheTolerance) {
  REQUIRE_PRODUCT();
  const RateConfig<float> cfg = scaled(prod.rate, kGainScale);
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Trace run = golden_run(prod, s, 0, nullptr, &cfg);
    const double f = factor(g, run);
    std::printf("[T3] %-11s control rate gains x %.1f: max(diff / tolerance) = x%.1f\n", kScenario[s],
                static_cast<double>(kGainScale), f);
    EXPECT_GT(f, 1.0) << kScenario[s] << ": the golden cannot detect a rate gain error";
  }
}

TEST(L5T3Step, NegativeControlOneTickOfDelayLeavesTheTolerance) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < kScenario.size(); ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Trace run = golden_run(prod, s, kControlDelayTicks);
    const double f = factor(g, run);
    std::printf("[T3] %-11s control delay %u tick: max(diff / tolerance) = x%.1f\n", kScenario[s], kControlDelayTicks, f);
    EXPECT_GT(f, 1.0) << kScenario[s] << ": the golden cannot detect a one-tick delay";
  }
}

// ---- 3. the band envelopes (recorded, used at T4) ----------------------------------------------------------------------------

bool inside(const Channel& c, std::size_t execution, double value) {
  if (execution < c.first || execution >= c.first + c.count) {
    return false;
  }
  return value >= c.lo[execution - c.first] && value <= c.hi[execution - c.first];
}

TEST(L5T3Envelope, NominalFloatTrajectoryLiesInsideTheBandEnvelope) {
  REQUIRE_PRODUCT();
  for (std::size_t s = 0; s < 2; ++s) {
    const Trace run = golden_run(prod, s);
    const EnvelopeScenario& env = envelope().at(kScenario[s]);
    const Channel& c = env.channel.at("theta");
    ASSERT_EQ(c.count, run.theta.size());
    std::size_t outside = 0;
    for (std::size_t n = 0; n < run.theta.size(); ++n) {
      outside += inside(c, n, run.theta[n]) ? 0 : 1;
    }
    EXPECT_EQ(outside, 0U) << kScenario[s] << ": executions outside the envelope";
    EXPECT_GT(c.halving, 0.0);
  }
}

TEST(L5T3Envelope, YawReleaseNominalLiesInsideTheEnvelopeAndItsLockIsWithinTheRecordedRange) {
  REQUIRE_PRODUCT();
  const Trace run = golden_run(prod, 2);
  const EnvelopeScenario& env = envelope().at("yaw_release");
  const std::size_t a_r = release_of(golden().at("yaw_release"));
  ASSERT_GT(run.lock_execution, 0U);
  EXPECT_GE(static_cast<double>(run.lock_execution), env.numbers.at("lock_execution_min"));
  EXPECT_LE(static_cast<double>(run.lock_execution), env.numbers.at("lock_execution_max"));
  std::size_t outside = 0;
  for (std::size_t n = 0; n < run.omega.size(); ++n) {
    outside += inside(env.channel.at("omega"), n, run.omega[n]) ? 0 : 1;
  }
  for (std::size_t n = a_r; n < run.theta.size(); ++n) {
    outside += inside(env.channel.at("heading_release"), n, run.theta[n] - run.theta[a_r]) ? 0 : 1;
  }
  for (std::size_t n = run.lock_execution; n < run.theta.size(); ++n) {
    outside += inside(env.channel.at("heading_lock"), n, run.theta[n] - run.theta[run.lock_execution]) ? 0 : 1;
  }
  EXPECT_EQ(outside, 0U) << "executions outside the envelope";
}

// The envelope's end value is below F = the T3 tolerance + the envelope's last-halving change + the kinematics halving change
// of its script: below F, so below E + F (E >= 0) of T4, so a trace inside the envelope has settled (decision 0006 F).
TEST(L5T3Envelope, EveryEnvelopeEndsBelowTheToleranceTermF) {
  REQUIRE_PRODUCT();
  auto end_value = [](const Channel& c) { return std::max(std::abs(c.lo.back()), std::abs(c.hi.back())); };
  for (std::size_t s = 0; s < 2; ++s) {
    const Golden& g = golden().at(kScenario[s]);
    const Channel& c = envelope().at(kScenario[s]).channel.at("theta");
    const double f = g.numbers.at("tolerance_theta") + c.halving + g.numbers.at("kinematics_change_theta");
    EXPECT_LT(end_value(c), f) << kScenario[s];
  }
  const Golden& g = golden().at("yaw_release");
  for (const char* name : {"yaw_release", "yaw_fallback"}) {
    const EnvelopeScenario& env = envelope().at(name);
    const Channel& w = env.channel.at("omega");
    const Channel& h = env.channel.at("heading_lock");
    EXPECT_LT(end_value(w), g.numbers.at("tolerance_omega") + w.halving + g.numbers.at("kinematics_change_omega")) << name;
    EXPECT_LT(end_value(h), g.numbers.at("tolerance_theta") + h.halving + g.numbers.at("kinematics_change_theta")) << name;
  }
}

// Control of the settle property: an envelope cut to half its length (the segment not doubled) does not end below F.
TEST(L5T3Envelope, ControlAnEnvelopeCutAtTheEndOfTheHoldIsNotSettled) {
  REQUIRE_PRODUCT();
  const Golden& g = golden().at("step_roll");
  const Channel& c = envelope().at("step_roll").channel.at("theta");
  const std::size_t release = release_of(g);
  const double f = g.numbers.at("tolerance_theta") + c.halving + g.numbers.at("kinematics_change_theta");
  // At the release the tilt is at the full-stick value, far from zero: the same test fails there.
  EXPECT_GT(std::max(std::abs(c.lo[release - 1]), std::abs(c.hi[release - 1])), f);
}

// The T3 property of the fallback script (decision 0006 F): every box member keeps sigma_r w > 0 up to its fallback execution,
// and it locks by the fallback, at one execution: nothing crosses earlier.
TEST(L5T3Envelope, FallbackScriptKeepsTheYawRateSignUpToTheFallbackForEveryBoxMember) {
  REQUIRE_PRODUCT();
  const EnvelopeScenario& fb = envelope().at("yaw_fallback");
  EXPECT_GT(fb.numbers.at("min_sigma_omega_to_fallback"), 0.0);
  EXPECT_EQ(fb.numbers.at("lock_execution_min"), fb.numbers.at("fallback_execution_min"));
  EXPECT_EQ(fb.numbers.at("lock_execution_max"), fb.numbers.at("fallback_execution_max"));
  EXPECT_GT(fb.numbers.at("stick_scale"), 0.0);
  EXPECT_EQ(fb.numbers.at("disturbance_nm"), static_cast<double>(static_cast<float>(fb.numbers.at("disturbance_nm"))));
  // Control: the release script (no disturbance) crosses before its fallback, so its smallest sigma_r w is not positive.
  const EnvelopeScenario& rel = envelope().at("yaw_release");
  EXPECT_LE(rel.numbers.at("min_sigma_omega_to_fallback"), 0.0);
  EXPECT_LT(rel.numbers.at("lock_execution_max"), rel.numbers.at("fallback_execution_min"));
}

}  // namespace
