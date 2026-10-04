// L6 stage (d), T1, the motor-speed line (decision 0017, "The final line", Luis 2026-10-04):
//   at hover, the rotor-speed error stays within (1 - e^(-T_r/tau-)) DShot steps + delta_max at every plant step,
//   converted to rad/s by the largest slope of the card's DShot-to-speed map over hover +- 1 step. The error is the
//   rotor speed of the plant's own motor model (marv_plant) under the diffused command minus the same motor lag driven
//   by the unquantised clamped request d~, both from the same state. Negative control: with the diffuser off
//   (stateless rounding), the same check fails at the hover request.
//
// The run. marv_plant through its C ABI with the card's configuration (the generated card header) and motor_tau_s =
// tau-. A plant step is one tick h and the motor sub-step is h (as the gz plugin sets it), so every step is exactly one
// sub-step of the plant's motor model, omega <- c + (omega - c) d^, with d^ the plant's exp(-h / tau-) and c the plant's
// ESC map of the held command (sim/plant/src/plant_model.hpp). One actuator write per rate execution, every
// R = rate_loop_divisor ticks (T_r = R h); the command is held over its R steps, as the firmware holds it.
//   A  one plant driven by DshotDiffuser<float>::apply of the hover allocation at every write (the rate group's path:
//      allocate, then apply).
//   B  the same motor lag driven by d~. The plant takes integer DShot only (marv_plant_cmd; Model::advance and
//      omega_cmd take uint16), so B is the superposition of two marv_plant runs from B's state at the write, one under
//      lo = floor(d~) and one under hi = lo + 1 (lo when d~ is an integer), combined after every step as
//      omega_B = (1 - f) omega_lo + f omega_hi, f = d~ - lo. In real arithmetic this is the plant under the command
//      (1 - f) c(lo) + f c(hi) = c(d~): the map is linear in DShot (esc_map LINEAR_IN_OMEGA, asserted) and the sub-step
//      is affine in (omega, c), with weights that sum to 1. The next write's two plants start from the combined speed
//      (initial_omega_rad_s; the rotor speed is the only motor state the speed update reads).
//   Start-up: both at write 0 from the plant's default state (rotors at rest), the diffuser just initialised (zero
//   carry), as a SIL run starts. The error is 0 there and no step is excluded: the bound holds from any common state.
//
// The threshold, at every plant step and motor: s (1 - a + delta_max) + rho.
//   tau-   (1 - tau_robustness_band) tau: the band from the product parameter set (design/budget.yaml), tau from the
//          card header (rotors.motor_lag.tau). The low corner gives the largest 1 - a.
//   a      the plant's decay over T_r, d^^R. d^ is read from the plant itself: one step from 1 rad/s under DShot 0 returns
//          fl(0 + fl(fl(1 - 0) d^)) = d^ exactly. So e^(-T_r / tau-) is the plant's own evaluation.
//   bound  0017's summation by parts at every plant step: after write n and m <= R steps of it the error is
//          s sum_j w_j (q_j - d~_j), with w_n = 1 - d^^m, w_j = d^^m a^(n-1-j) (1 - a) for j < n, q_j - d~_j =
//          e_(j-1) - e_j + rho_j (P1) and e_(-1) = 0. So |error| <= s (max(1 - d^^m, d^^m (1 - a)) + delta_max)
//          <= s (1 - a + delta_max).
//   s      the largest slope of the card's map over hover +- 1 step. The card's map is linear in rotor speed from
//          speed_range[0] at DShot 48 to speed_range[1] at 2047 (esc_map), so every step has the same slope,
//          (omega_max - omega_min) / (2047 - 48), and that is the largest over hover +- 1 step.
//   delta_max  2^-14 step, P1's rule (diffusion_test.cpp): half the float spacing at d_max + 1/2.
//   rho    the plant's double rounding, derived (u = 2^-53, gamma_n = n u / (1 - n u), Higham 2nd ed. section 3.1).
//          Every speed is in [0, W], W = 2 omega_max (asserted at every step); every map value is in [0, W] (below).
//            map    c^(D) = fl(omega_min + fl((omega_max - omega_min) fl((D - 48) / 1999))), the subtraction exact, is
//                   within gamma_3 omega_max of the linear map. A's c^(q) and B's (1 - f) c^(lo) + f c^(hi) each differ
//                   from s (q - d~) + the map's value at d~ by at most 2 gamma_3 omega_max in total, and the lag's
//                   weights sum to at most 1.
//            step   fl(c + fl(fl(omega - c) d^)): three roundings, at most gamma_4 W per step against the exact step
//                   from the same speed (|omega - c| <= W, |c| <= W). The lag contracts an earlier error by d^ per step,
//                   so A carries at most gamma_4 W / (1 - d^).
//            B      gamma_4 W per step for (1 - f) lo + f hi, plus the combination (two products and a sum, with exact
//                   weights: 1 - f is exact because f is a multiple of the float spacing of d~), gamma_2 W. That is fed
//                   to the next write's plants at most once per step, and once more in the current output:
//                   (gamma_4 + gamma_2) W / (1 - d^) + gamma_2 W.
//          rho = (2 gamma_4 + gamma_2) W / (1 - d^) + gamma_2 W + 2 gamma_3 omega_max, evaluated with each gamma_n
//          replaced by (n + 1) u. The excess, at least u W / (1 - d^) (about 1e-10 rad/s), covers every rounding in
//          evaluating the threshold and the difference omega_A - omega_B (each below 1e-15 rad/s).
//
// Affinity of the plant's step (what B's superposition needs, checked numerically): from one state, one write period of the
// plant's own advance under the DShot commands lo, lo + 1, lo + 2 gives a second difference omega(lo) - 2 omega(lo + 1) +
// omega(lo + 2) that is zero in real arithmetic (the map and the sub-step are affine in the command), so at every plant step
// of the period it is at most the three runs' rounding: the three runs start from the same speed, each carries at most
// gamma_4 W / (1 - d^) of step rounding and gamma_3 omega_max of map rounding (rho's terms), and the second difference
// weights them 1, 2, 1, so 4 (gamma_4 W / (1 - d^) + gamma_3 omega_max). The states are the hover speed and rest; the
// commands are the hover floor, d_lo and d_max - 2.
// Negative control: lo, lo + 1, lo + 3 (unequal spacing) gives s (1 - d^^m) at step m, far above that bound.
//
// Negative control: stateless rounding (thrust_to_dshot, the firmware before P2) in place of the diffuser, the same check
// at the hover request, must fail.
// Extra (owner decision 3: any sequence): the same check on a random in-range request sequence with a committed seed.
//
// Numbers here are cited constants (the DShot range, the float formats), derived values (the rule is on the line), or
// "scenario test values" named with their reason. The card's values come from the generated card header, the mixer and
// the run's timing from the product parameter set.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>

#include MARV_PLANT_CARD_HEADER
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/gravity.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/actuator.hpp>

#include "marv_plant.h"

namespace {

using namespace marv;
using namespace marv::mixer;

constexpr std::size_t kN = kMotors;
static_assert(kN == MARV_PLANT_N_MOTORS, "the mixer and the plant index the same four motors");

using Speeds = std::array<double, kN>;
using Commands = std::array<std::uint16_t, kN>;
using Thrusts = std::array<float, kN>;
using Requests = std::array<float, kN>;

constexpr float kDMax = static_cast<float>(prim::kDshotThrottleMax);
constexpr double kHalf = 0.5;
// binary64 unit roundoff u = eps / 2.
constexpr double kU = std::numeric_limits<double>::epsilon() / 2.0;

// ---- scenario test values ------------------------------------------------------------------------------------
// The hover site: scenarios/quad/L02/hover.yaml (0.82 rad, 500 m), the site of every L4 and L5 scenario, whose thrust
// rule is m g(phi, h0) rounded to float (tools/sim/run_l4.py hover_thrust). Rotor speeds do not depend on it.
constexpr double kSiteLat = 0.82;      // rad
constexpr double kSiteHeight = 500.0;  // m
// The plant's noise seed: unused (no IMU is attached, and marv_plant_step draws nothing).
constexpr std::uint64_t kPlantSeed = 0;
// Run lengths in writes: 2^14 at hover (5.1 s, about 220 tau-; the carry pattern at a 0.06 fraction repeats every 17
// writes or so), 2^16 for the random sequence. Short enough for well under a second in a debug build.
constexpr std::size_t kHoverWrites = std::size_t{1} << 14;
constexpr std::size_t kRandomWrites = std::size_t{1} << 16;
// Seed of the random sequence's per-motor mt19937 streams (motor i uses seed + i): committed, arbitrary.
constexpr std::uint32_t kSeedRandom = 601;
// rho's rounding counts (file comment): a plant step (gamma_4), the combination (gamma_2), the map (gamma_3).
constexpr int kStepRoundings = 4;
constexpr int kCombineRoundings = 2;
constexpr int kMapRoundings = 3;
// The second difference's weights 1, 2, 1 (file comment): the sum of their magnitudes.
constexpr double kSecondDiffWeight = 1.0 + 2.0 + 1.0;
// The affinity control's third command offset: lo + 3 in place of lo + 2, scenario test value (any unequal spacing).
constexpr std::uint16_t kControlOffset = 3;
constexpr std::uint16_t kAffineOffset = 2;
// The first four affine cases (hover floor and d_lo, at both states): lo + 3 stays in the DShot range there.
constexpr std::size_t kControlCases = 4;

// gamma_n bounded above by (n + 1) u (valid while n (n + 1) u <= 1).
double gamma_up(int n) { return static_cast<double>(n + 1) * kU; }

// d~: the request clamped to [d_lo, d_max] as the law defines it (NaN and -inf to d_lo, +inf to d_max).
float clamp_request(float d, float d_lo) {
  if (!(d >= d_lo)) {
    return d_lo;
  }
  return d > kDMax ? kDMax : d;
}

// ---- the setup -------------------------------------------------------------------------------------------------

struct Rig {
  MixerConfig<float> mix;
  float d_lo = 0.0F;
  marv_plant_config plant{};  // the card's, with the scenario fields and motor_tau_s = tau-
  bool esc_linear = false;
  double h = 0.0;             // tick, s
  std::uint32_t r = 0;        // rate_loop_divisor
  double tau = 0.0;           // the card's, s
  double band = 0.0;          // tau_robustness_band
  double tau_minus = 0.0;     // s
  double d_hat = 0.0;         // the plant's decay per step
  double one_minus_a = 0.0;   // 1 - d^^R
  double one_minus_a_libm = 0.0;  // 1 - exp(-T_r / tau-) evaluated here, for the report only
  double s = 0.0;             // rad/s per DShot step
  double delta_max = 0.0;     // DShot steps
  double w = 0.0;             // W = 2 omega_max, rad/s
  double rho = 0.0;           // rad/s
  double threshold = 0.0;     // rad/s
  Thrusts f_hover{};          // N per motor
  float thrust = 0.0F;        // N
};

const marv_plant_body& body() {
  static const marv_plant_body b = [] {
    marv_plant_body x{};
    x.struct_size = sizeof x;
    x.q_wxyz[0] = 1.0;
    return x;
  }();
  return b;
}

// A marv_plant from cfg with the given rotor speeds; destroyed with the object.
class Plant {
 public:
  Plant(const marv_plant_config& cfg, const Speeds& omega0) {
    marv_plant_config c = cfg;
    for (std::size_t i = 0; i < kN; ++i) {
      c.initial_omega_rad_s[i] = omega0[i];
    }
    status_ = marv_plant_create(&c, &p_);
  }
  ~Plant() { marv_plant_destroy(p_); }
  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;

  bool ok() const { return status_ == MARV_PLANT_OK; }

  // One plant step of dt under the commands; false on any status but OK.
  bool step(const Commands& d, double dt, Speeds& omega) {
    marv_plant_cmd cmd{};
    cmd.struct_size = sizeof cmd;
    for (std::size_t i = 0; i < kN; ++i) {
      cmd.dshot[i] = d[i];
    }
    marv_plant_out out{};
    out.struct_size = sizeof out;
    if (marv_plant_step(p_, &body(), &cmd, dt, &out) != MARV_PLANT_OK) {
      return false;
    }
    for (std::size_t i = 0; i < kN; ++i) {
      omega[i] = out.rotor_speed_rad_s[i];
    }
    return true;
  }

 private:
  marv_plant* p_ = nullptr;
  marv_plant_status status_ = MARV_PLANT_E_NULL;
};

const Rig* rig() {
  static const Rig* built = []() -> const Rig* {
    if (!params_init(param_defaults())) {
      return nullptr;
    }
    static Rig su;
    su.mix = from_params();
    if (validate(su.mix) != ConfigError::None) {
      return nullptr;
    }
    su.d_lo = dshot_floor(su.mix);

    marv_plant_config c{};
    MARV_PLANT_CARD_FILL(&c);
    su.esc_linear = c.esc_map == MARV_PLANT_ESC_LINEAR_IN_OMEGA;
    const auto num_us = static_cast<double>(param_value<ParamId::tick_period_num_us>());
    const auto den = static_cast<double>(param_value<ParamId::tick_period_den>());
    su.h = num_us / (den * prim::kMicrosecondsPerSecond);
    su.r = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
    su.tau = c.motor_tau_s;
    su.band = static_cast<double>(param_value<ParamId::tau_robustness_band>());
    su.tau_minus = (1.0 - su.band) * su.tau;
    c.motor_tau_s = su.tau_minus;
    c.motor_substep_s = su.h;
    c.site_lat_rad = kSiteLat;
    c.site_height_m = kSiteHeight;
    c.rng_seed = kPlantSeed;
    su.plant = c;

    // d^ from the plant: one step from 1 rad/s under DShot 0 (file comment).
    {
      Speeds unit{};
      unit.fill(1.0);
      Plant p(su.plant, unit);
      Speeds out{};
      if (!p.ok() || !p.step(Commands{}, su.h, out)) {
        return nullptr;
      }
      su.d_hat = out[0];
    }
    double a = 1.0;
    for (std::uint32_t m = 0; m < su.r; ++m) {
      a *= su.d_hat;
    }
    su.one_minus_a = 1.0 - a;
    su.one_minus_a_libm = 1.0 - std::exp(-(static_cast<double>(su.r) * su.h) / su.tau_minus);

    su.s = (c.omega_max_rad_s - c.omega_min_rad_s) /
           static_cast<double>(prim::kDshotThrottleMax - prim::kDshotThrottleMin);
    su.delta_max = std::ldexp(1.0, std::ilogb(static_cast<double>(kDMax) + kHalf) - std::numeric_limits<float>::digits);
    su.w = 2.0 * c.omega_max_rad_s;
    su.rho = (2.0 * gamma_up(kStepRoundings) + gamma_up(kCombineRoundings)) * su.w / (1.0 - su.d_hat) +
             gamma_up(kCombineRoundings) * su.w + 2.0 * gamma_up(kMapRoundings) * c.omega_max_rad_s;
    su.threshold = su.s * (su.one_minus_a + su.delta_max) + su.rho;

    // The hover allocation: m g(phi, h0) in double, rounded to float, through the product mixer at zero torque.
    su.thrust = static_cast<float>(c.mass_kg * prim::normal_gravity<double>(kSiteLat, kSiteHeight));
    su.f_hover = allocate(su.mix, Request<float>{su.thrust, prim::Vec3<float>{}}).f;
    return &su;
  }();
  return built;
}

#define REQUIRE_RIG()                                               \
  const Rig* const sp = rig();                                      \
  ASSERT_NE(sp, nullptr) << "params_init, validate or the plant";   \
  const Rig& su = *sp;                                              \
  ASSERT_TRUE(su.esc_linear) << "the slope rule and B's superposition need the linear map"

// ---- the run ---------------------------------------------------------------------------------------------------

struct Write {
  Commands q{};    // A's command
  Requests d{};    // d~, B's request
};

struct Result {
  bool ok = true;            // every plant created and stepped
  bool in_range = true;      // every speed in [0, W]
  std::size_t steps = 0;
  double max_err = 0.0;      // rad/s, over every step and motor
  std::size_t first_over = 0;  // 1-based step of the first error above the threshold, 0 if none
};

template <class NextWrite>
Result run(const Rig& su, std::size_t writes, NextWrite&& next) {
  Result res;
  Plant a(su.plant, Speeds{});
  res.ok = a.ok();
  Speeds omega_b{};
  for (std::size_t n = 0; n < writes && res.ok; ++n) {
    const Write w = next();
    Commands lo{};
    Commands hi{};
    Speeds f{};
    for (std::size_t i = 0; i < kN; ++i) {
      const auto d = static_cast<double>(w.d[i]);
      const double whole = std::floor(d);
      lo[i] = static_cast<std::uint16_t>(whole);
      f[i] = d - whole;
      hi[i] = f[i] > 0.0 ? static_cast<std::uint16_t>(lo[i] + 1) : lo[i];
    }
    Plant pl(su.plant, omega_b);
    Plant ph(su.plant, omega_b);
    if (!pl.ok() || !ph.ok()) {
      res.ok = false;
      break;
    }
    for (std::uint32_t m = 0; m < su.r; ++m) {
      Speeds wa{};
      Speeds wl{};
      Speeds wh{};
      if (!a.step(w.q, su.h, wa) || !pl.step(lo, su.h, wl) || !ph.step(hi, su.h, wh)) {
        res.ok = false;
        break;
      }
      ++res.steps;
      for (std::size_t i = 0; i < kN; ++i) {
        omega_b[i] = (1.0 - f[i]) * wl[i] + f[i] * wh[i];
        for (const double v : {wa[i], wl[i], wh[i], omega_b[i]}) {
          res.in_range = res.in_range && v >= 0.0 && v <= su.w;
        }
        const double err = std::fabs(wa[i] - omega_b[i]);
        res.max_err = std::max(res.max_err, err);
        if (err > su.threshold && res.first_over == 0) {
          res.first_over = res.steps;
        }
      }
    }
  }
  return res;
}

enum class Law { Diffused, Stateless };

// The hover request at every write: A's command from the diffuser or from stateless rounding, B's d~ from the same d*.
Result run_hover(const Rig& su, Law law) {
  DshotDiffuser<float> diffuser;
  diffuser.init(su.mix);
  return run(su, kHoverWrites, [&]() {
    Write w;
    const std::array<DshotValue, kN> q =
        law == Law::Diffused ? diffuser.apply(su.f_hover) : thrust_to_dshot(su.mix, su.f_hover);
    for (std::size_t i = 0; i < kN; ++i) {
      w.q[i] = q[i].raw();
      w.d[i] = clamp_request(dshot_unrounded(su.mix, su.f_hover[i]), su.d_lo);
    }
    return w;
  });
}

void print_rig(const Rig& su) {
  std::printf("[motor speed] tau = %.9g s, band = %.9g, tau- = %.9g s; h = %.9g s, R = %u, T_r = %.9g s\n", su.tau,
              su.band, su.tau_minus, su.h, su.r, static_cast<double>(su.r) * su.h);
  std::printf("[motor speed] d^ = %.17g; 1 - a = %.9g step (1 - exp(-T_r/tau-) here: %.9g); delta_max = %.9g step\n",
              su.d_hat, su.one_minus_a, su.one_minus_a_libm, su.delta_max);
  std::printf("[motor speed] s = %.9g rad/s per step; bound = %.9g step = %.9g rad/s; rho = %.3g rad/s; threshold = "
              "%.9g rad/s\n",
              su.s, su.one_minus_a + su.delta_max, su.s * (su.one_minus_a + su.delta_max), su.rho, su.threshold);
  for (std::size_t i = 0; i < kN; ++i) {
    std::printf("[motor speed] hover: thrust %.9g N, motor %zu f = %.9g N, d* = %.9g (fraction %.6g)\n",
                static_cast<double>(su.thrust), i + 1, static_cast<double>(su.f_hover[i]),
                static_cast<double>(dshot_unrounded(su.mix, su.f_hover[i])),
                static_cast<double>(dshot_unrounded(su.mix, su.f_hover[i])) -
                    std::floor(static_cast<double>(dshot_unrounded(su.mix, su.f_hover[i]))));
  }
}

void print_result(const char* name, const Rig& su, const Result& r) {
  std::printf("[motor speed] %s: %zu plant steps x %zu motors, max |omega_A - omega_B| = %.9g rad/s = %.9g step; "
              "ratio to the threshold %.4f; first step over it: %zu\n",
              name, r.steps, kN, r.max_err, r.max_err / su.s, r.max_err / su.threshold, r.first_over);
}

TEST(L6DshotMotorSpeed, HoverErrorStaysWithinTheBoundAtEveryPlantStep) {
  REQUIRE_RIG();
  print_rig(su);
  for (std::size_t i = 0; i < kN; ++i) {
    const float d = dshot_unrounded(su.mix, su.f_hover[i]);
    ASSERT_TRUE(d >= su.d_lo && d <= kDMax) << "motor " << i + 1 << ": the hover request is clamped";
  }
  const Result r = run_hover(su, Law::Diffused);
  print_result("hover, diffused", su, r);
  ASSERT_TRUE(r.ok) << "a plant refused a create or a step";
  EXPECT_TRUE(r.in_range) << "a speed left [0, W]: rho's derivation does not hold";
  EXPECT_EQ(r.steps, kHoverWrites * su.r);
  EXPECT_LE(r.max_err, su.threshold);
  EXPECT_EQ(r.first_over, 0U);
}

struct SecondDiff {
  bool ok = true;
  bool in_range = true;
  std::size_t steps = 0;
  double max_dd = 0.0;                                  // rad/s, over every step and motor
  double min_dd = std::numeric_limits<double>::max();  // rad/s
};

// One write period of the plant's own advance from omega0 under lo, lo + 1 and lo + third (all four motors), the second
// difference omega(lo) - 2 omega(lo + 1) + omega(lo + third) after every plant step.
SecondDiff second_difference(const Rig& su, const Speeds& omega0, const Commands& lo, std::uint16_t third) {
  SecondDiff res;
  Commands d1 = lo;
  Commands d2 = lo;
  for (std::size_t i = 0; i < kN; ++i) {
    d1[i] = static_cast<std::uint16_t>(lo[i] + 1);
    d2[i] = static_cast<std::uint16_t>(lo[i] + third);
  }
  Plant p0(su.plant, omega0);
  Plant p1(su.plant, omega0);
  Plant p2(su.plant, omega0);
  if (!p0.ok() || !p1.ok() || !p2.ok()) {
    res.ok = false;
    return res;
  }
  for (std::uint32_t m = 0; m < su.r; ++m) {
    Speeds w0{};
    Speeds w1{};
    Speeds w2{};
    if (!p0.step(lo, su.h, w0) || !p1.step(d1, su.h, w1) || !p2.step(d2, su.h, w2)) {
      res.ok = false;
      return res;
    }
    ++res.steps;
    for (std::size_t i = 0; i < kN; ++i) {
      for (const double v : {w0[i], w1[i], w2[i]}) {
        res.in_range = res.in_range && v >= 0.0 && v <= su.w;
      }
      const double dd = std::fabs(w0[i] - 2.0 * w1[i] + w2[i]);
      res.max_dd = std::max(res.max_dd, dd);
      res.min_dd = std::min(res.min_dd, dd);
    }
  }
  return res;
}

// The second-difference bound (file comment).
double second_difference_bound(const Rig& su) {
  return kSecondDiffWeight * (gamma_up(kStepRoundings) * su.w / (1.0 - su.d_hat) +
                              gamma_up(kMapRoundings) * su.plant.omega_max_rad_s);
}

// The (state, lo) cases: states rest and the hover speed (the card's linear map at d*), lo = the hover floor per motor,
// d_lo and d_max - 2 for every motor (the first four cases keep lo + 3 in range).
struct AffineCase {
  Speeds omega0;
  Commands lo;
};

std::array<AffineCase, 6> affine_cases(const Rig& su) {
  Speeds hover{};
  Commands lo_hover{};
  for (std::size_t i = 0; i < kN; ++i) {
    const float d = dshot_unrounded(su.mix, su.f_hover[i]);
    hover[i] = su.plant.omega_min_rad_s + su.s * (static_cast<double>(d) - static_cast<double>(prim::kDshotThrottleMin));
    lo_hover[i] = static_cast<std::uint16_t>(std::floor(static_cast<double>(d)));
  }
  Commands lo_low{};
  Commands lo_top{};
  lo_low.fill(static_cast<std::uint16_t>(std::floor(static_cast<double>(su.d_lo))));
  lo_top.fill(static_cast<std::uint16_t>(prim::kDshotThrottleMax - kAffineOffset));
  const Speeds rest{};
  return {AffineCase{hover, lo_hover}, AffineCase{rest, lo_hover}, AffineCase{hover, lo_low},
          AffineCase{rest, lo_low},    AffineCase{hover, lo_top},  AffineCase{rest, lo_top}};
}

TEST(L6DshotMotorSpeed, PlantMotorStepIsAffineInTheCommandAtEveryStep) {
  REQUIRE_RIG();
  const double bound = second_difference_bound(su);
  double worst = 0.0;
  for (const AffineCase& c : affine_cases(su)) {
    const SecondDiff r = second_difference(su, c.omega0, c.lo, kAffineOffset);
    ASSERT_TRUE(r.ok) << "a plant refused a create or a step";
    EXPECT_TRUE(r.in_range) << "a speed left [0, W]: the bound's derivation does not hold";
    EXPECT_EQ(r.steps, su.r);
    EXPECT_LE(r.max_dd, bound);
    worst = std::max(worst, r.max_dd);
  }
  std::printf("[motor speed] affine step: max second difference %.9g rad/s, bound %.9g rad/s (ratio %.4g)\n", worst, bound,
              worst / bound);
}

TEST(L6DshotMotorSpeedControl, UnequalSpacingFailsTheAffinityCheck) {
  REQUIRE_RIG();
  const double bound = second_difference_bound(su);
  const auto cases = affine_cases(su);
  double least = std::numeric_limits<double>::max();
  for (std::size_t k = 0; k < kControlCases; ++k) {
    const SecondDiff r = second_difference(su, cases[k].omega0, cases[k].lo, kControlOffset);
    ASSERT_TRUE(r.ok) << "a plant refused a create or a step";
    EXPECT_GT(r.min_dd, bound) << "the unequal-spacing control does not fail at every step (owner stop rule: report)";
    least = std::min(least, r.min_dd);
  }
  std::printf("[motor speed] affine control (lo, lo + 1, lo + 3): least second difference %.9g rad/s, bound %.9g rad/s "
              "(ratio %.4g)\n", least, bound, least / bound);
}

TEST(L6DshotMotorSpeedControl, StatelessRoundingFailsTheCheckAtHover) {
  REQUIRE_RIG();
  const Result r = run_hover(su, Law::Stateless);
  print_result("control, stateless at hover", su, r);
  ASSERT_TRUE(r.ok) << "a plant refused a create or a step";
  EXPECT_TRUE(r.in_range);
  EXPECT_GT(r.max_err, su.threshold) << "the stateless control does not fail at hover (owner stop rule: report)";
  EXPECT_NE(r.first_over, 0U);
}

TEST(L6DshotMotorSpeedExtra, RandomInRangeSequenceStaysWithinTheBoundAtEveryPlantStep) {
  REQUIRE_RIG();
  DshotDiffuser<float> diffuser;
  diffuser.init(su.mix);
  std::array<std::mt19937, kN> rng;
  for (std::size_t i = 0; i < kN; ++i) {
    rng[i].seed(kSeedRandom + static_cast<std::uint32_t>(i));
  }
  std::uniform_real_distribution<float> request(su.d_lo, kDMax);
  const Result r = run(su, kRandomWrites, [&]() {
    Write w;
    Requests d_star{};
    for (std::size_t i = 0; i < kN; ++i) {
      d_star[i] = request(rng[i]);
    }
    const std::array<DshotValue, kN> q = diffuser.apply_unrounded(d_star);
    for (std::size_t i = 0; i < kN; ++i) {
      w.q[i] = q[i].raw();
      w.d[i] = clamp_request(d_star[i], su.d_lo);
    }
    return w;
  });
  print_result("extra, random in-range sequence, diffused", su, r);
  ASSERT_TRUE(r.ok) << "a plant refused a create or a step";
  EXPECT_TRUE(r.in_range) << "a speed left [0, W]: rho's derivation does not hold";
  EXPECT_EQ(r.steps, kRandomWrites * su.r);
  EXPECT_LE(r.max_err, su.threshold);
  EXPECT_EQ(r.first_over, 0U);
}

}  // namespace
