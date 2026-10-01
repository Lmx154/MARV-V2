// L6 stage (b) T1 (decision 0013, packet P6): the notches track the rotor speeds across the vibration sweep.
//
// The loop under test: the plant's gyro vibration and rotor-speed sensor model (sim/plant) feed the firmware gyro chain
// (fw/gyro_chain), IMU noise off, and the chain removes the vibration as designed (|H| <= a_min at every component).
//
// Setup (every value is a labelled test value, a cited value, or derived on its line):
//  - A plant of the frozen L1 fixture, the body held at rest (the same marv_plant_body, zero rate, passed to every sample;
//    no rigid-body integration), the four rotors driven by DShot commands from rest to steady speeds. Each tick, in the
//    host's order (marv_plant.h): IMU sample, rotor-speed sample, marv_plant_step.
//  - IMU: the profile's configuration (marv::sim::imu_profile_config(): LSB, full scale, latency 1) with N = B = 0 (turn-on
//    bias is already 0). Gyro truth is 0, so the gyro output is quantise(v), v the vibration.
//  - Vibration: A_1 = A_2 = A_3 = A (a labelled test choice: the model has one amplitude per harmonic and nothing sources a
//    ratio), omega_hover from decision 0013 P5, exponent p = 2 (owner decision 2).
//  - Rotor-speed model: 14 poles, grid 3 / 9 / 1 us (rotor_speed/support.hpp grid_config, whose constants cite Betaflight
//    dshot.c), latency = the rate divisor = 2 ticks.
//  - Chain: GyroChainConfig from the generated product parameters (gyro_lpf_cutoff_hz, gyro_notch_q_h1..h3,
//    gyro_notch_omega_min, rate_loop_divisor, tick period), cross-checked against decision 0013 P5's design report. Per tick chain.filter(gyro); at every rate-loop execution (every
//    divisor ticks) chain.update_notches(the tick's rotor sample).
//
// Measurement. The residual of each vibration component is measured by a joint least-squares fit of cos and sin at every
// distinct component frequency to the chain output over a window after settling (the frequencies are the steady rotor
// speeds, known). Rationale against the max |y| over a whole-period window: the four rotors' 12 frequencies are not
// commensurate, so no window holds whole periods of all of them, and a max |y| is blind to which component leaked; the
// joint fit has no leakage between components (their separation is asserted through the fit's operator norm) and gives the
// amplitude of each component, so a per-component bound with a per-component allowance is checked. Components of equal
// frequency (all four rotors at one speed) cannot be separated: they are one group, whose input amplitude is bounded by
// the sum of the component amplitudes.
//
// Bound per group g and axis (the packet's check 1):
//     amplitude_out(g) <= a_min * sum of the component amplitudes c + allowance(g)
//   c = A (omega_i / omega_hover)^2 (the plant's formula, exponent 2, at the plant's steady rotor speed).
//   allowance = (D_q + D_f + D_tr) R_g + phi_tol sum(c), where
//     D_q  = (LSB / 2 + u X) l1(chain)         the IMU's rounding (half an LSB, plus the float storage of its output, u = 2^-24,
//                                              X the peak gyro value) through the chain's impulse response l1 norm;
//     D_f  = max |float chain - double reference chain| over the window (measured, run in step with the chain on the same
//                                              input and the same notch updates: the K-form design of the test support, direct form I
//                                              in double); asserted at most the derived worst-case bound
//                                              X sum_k gamma_5 (|b|_1 l1(cum_(k-1)) + |a|_1 l1(cum_k)) l1(g_k)  (decision 0013 P2's
//                                              analysis, first order: local error gamma_5 of the direct form I sum, carried to the
//                                              output by g_k = H_(k+1..K) / A_k, signals bounded by the cumulative l1 norms). The
//                                              bound alone is 100 to 1000 times the measurement (2e-6 to 1e-5 rad/s against 7e-4 to 1.5e-2) and would leave
//                                              the check without power. The measurement includes the float coefficients' error;
//     D_tr = X tol                             the remaining transient: with constant coefficients, the input before the
//                                              settling time contributes at most X times the impulse-response tail, tol;
//     R_g  = sqrt(R_cos^2 + R_sin^2)           the l1 norm of the rows of the least-squares operator (G^-1 Phi^T): a disturbance
//                                              of amplitude at most D changes the fitted amplitude by at most D R_g;
//     phi_tol sum(c)                           the phase drift of the rotors' last settling (below).
//
// Settling. (1) The motor lag is exactly first order (plant_model.hpp: omega - cmd decays as exp(-t/tau)). The drift of the
// fitted component's phase after t is at most h omega_max tau exp(-t/tau) (h = 3, omega_max the card's top speed); t is the
// smallest with that at most phi_tol = a_min / 64 (the drift then contributes 1.6 % of the bound). (2) After it the chain's
// own settling: the smallest n whose impulse-response tail of the design chain is at most tol = 1e-6 (support.hpp).
// (3) plus the rotor-speed latency, 2 ticks.
//
// Speed sets: equal (all four rotors at one speed), distinct (four speeds), single (rotor 1 only; rotors 2 to 4 stopped,
// their notches bypassed and flagged). With 12 notches of Q about 2 in a cascade, a component that has another rotor's
// notch within ~10 % is attenuated by that notch too (measured: down to 0.004 of c with a_min = 0.1), so on the equal and
// distinct sets a tracking error cannot reach the bound; on the single sets the only other active notches are rotor 1's
// own harmonics, 2x and 3x away, and a tracking error shows undiluted. The controls are therefore judged on the single sets.
// Controls (each must break the check on every single set; the nominal runs are the positive twins):
//  (a) the chain is given the rotor speeds x (1 + 1.5 eps), beyond the designed tracking error; its positive twin is
//      x (1 + 0.9 eps - 2^-8) (the telemetry truncation always reads high by up to 2^-8, so the twin's worst total offset
//      is 0.9 eps);
//  (b) slot 1 of the chain is given slot 4's speed and valid bit (a mis-routed rotor). A permutation of the motors between the
//      plant and the chain, the packet's wording, is NOT a fault of this chain: Q is the same for every motor of a harmonic, so
//      the 12 notch frequencies are the same set in any order. The test asserts that equivalence (the permuted run passes) and
//      uses the mis-route as the control;
//  (c) harmonic mismatch: speeds x 2/3;
//  (d) all notches bypassed (every motor reported invalid: the fault path, flags 0): fails on every set, equal and distinct too.
// Check 3 (linearity) has its own control: the same predicate between a nominal and a control run must fail.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

#include "../gyro_chain/support.hpp"
#include "../rotor_speed/support.hpp"
#include "marv/sim/imu_profile_config.hpp"
#include "marv_params_gyro_chain_design.hpp"
#include "marv_plant.h"
#include "plant_model.hpp"
#include "vibration_model.hpp"

namespace {

namespace gct = marv::gyro_chain::test;
namespace pt = marv::plant::test;
namespace rt = marv::plant::rotor_test;
using marv::RotorSpeedSample;
using marv::gyro_chain::GyroChain;
using marv::gyro_chain::GyroChainConfig;
using marv::gyro_chain::kHarmonics;
using marv::gyro_chain::kMotors;

// ---- The chain's configuration: the generated product parameters (P1: tools/card/gyro_chain_params.py through flatten.py) ----
// f_c, Q_h1..h3 and omega_th are read through param_get like the tick period (support.hpp tick_period_s) and the rate-loop
// divisor. The relative tracking error eps and its three terms are the generator's own (marv_params_gyro_chain_design.hpp, written
// by tools/card/gyro_chain_params.py through flatten.py from the profile's telemetry_mantissa_bits (2^-(m - 1) = 2^-8), imu
// odr_error (65 ppm) and rotor_speed esc_clock_error (the flown part's 2 %, AT32F421 HICK, UNVERIFIED)), so the test checks the
// Q that ships.
double omega_threshold() {
  EXPECT_TRUE(gct::init_params());
  return static_cast<double>(marv::param_value<marv::ParamId::gyro_notch_omega_min>());
}
std::uint32_t rate_divisor() {
  EXPECT_TRUE(gct::init_params());
  return static_cast<std::uint32_t>(marv::param_value<marv::ParamId::rate_loop_divisor>());
}
double telemetry_step() { return marv::gyro_chain_design::kGyroChainTelemetryStep; }
double epsilon() { return marv::gyro_chain_design::kGyroChainEpsilon; }

// ---- Scenario / cited values of the test ------------------------------------------------------------------------------
constexpr double kAMin = gct::kAMin;                // gyro_chain_attenuation_min, 0.1 (decision 0013 owner decision 3)
constexpr double kOmegaHover = 1100.591;            // rad/s: decision 0013 P5 (standard gravity), labelled
constexpr double kOmegaMaxCard = 2800.0;            // rad/s: the card's speed range top (P5's omega_max), labelled
constexpr double kRateMax = 11.694;                 // rad/s: scenario register rate_max_roll (11.693705988362009), P5
constexpr double kTwoPi = 2.0 * marv::prim::kPi;
constexpr std::uint64_t kSeed = 11;                 // labelled: any seed; the phases are drawn from it
constexpr std::size_t kWindow = 4096;               // fit window, ticks (0.64 s): bin width 1.6 Hz against component
                                                    // separations of at least 5 Hz at the lowest speed (R_g asserted)
constexpr double kPhiTol = kAMin / 64.0;            // rad: the settling criterion above
constexpr double kInverseTol = 1e-9;                // labelled: |G G^-1 - I| of the fit's 24 x 24 normal matrix (well inside, 1e-12 seen)
constexpr double kDecayTol = 1e-12;                 // labelled: the design impulse response's last value (the true one is below 1e-26)
constexpr double kP5AMax = 0.748;                   // rad/s, decision 0013 P5's A_max (three digits as reported)
constexpr double kP5AMaxTol = 5e-4;                 // half a unit of the last reported digit
constexpr double kRowNormMax = 4.0;                 // labelled: R_g of well separated bins is about 2 (2 sqrt(2) 2/pi)
constexpr std::size_t kDesignLen = 6000;            // design impulse response length: the slowest pole radius is below
                                                    // 0.99 (notch at omega_th, Q 2), 0.99^6000 < 1e-26; checked
constexpr std::size_t kSweepSteps = 4;              // k = 0..3: A_max 2^-k; labelled; the sweep is UNSOURCED (decision 0013)
constexpr std::array<double, kMotors> kDistinctFactors{1.0, 0.93, 0.85, 0.78};
                                                    // labelled: the four motors' speeds as fractions of the motor-1 speed;
                                                    // all 12 harmonic ratios are at least 7 % apart

// The tick period of the product parameter set, in s as the float of the generator, widened.
double tick_period() {
  EXPECT_TRUE(gct::init_params());
  return static_cast<double>(gct::tick_period_s());
}

// The rotor-speed delay: one rate-loop period in ticks (decision 0013 owner decision 7; the scenario value rate_loop_divisor).
std::size_t rate_latency_ticks() { return rate_divisor(); }

double gyro_fsr() { return marv::sim::imu_profile_config().gyro.full_scale; }
double gyro_lsb() { return marv::sim::imu_profile_config().gyro.lsb; }

// A_max, P5's rule: 12 A (omega_max / omega_hover)^2 + rate_max <= FSR.
double a_max() {
  const double r = kOmegaMaxCard / kOmegaHover;
  return (gyro_fsr() - kRateMax) / (12.0 * r * r);
}

// The plant's ESC map (marv_plant.h: linear in the DShot value from omega_min at 48 to omega_max at 2047), restated.
double omega_of_dshot(std::uint16_t d) {
  if (d == 0) {
    return 0.0;  // DShot 0 stops the motor
  }
  return pt::kOmegaMin + (pt::kOmegaMax - pt::kOmegaMin) * static_cast<double>(d - pt::kDshotMin) /
                             static_cast<double>(pt::kDshotMax - pt::kDshotMin);
}
std::uint16_t dshot_near(double omega) {
  const double f = (omega - pt::kOmegaMin) / (pt::kOmegaMax - pt::kOmegaMin) * static_cast<double>(pt::kDshotMax - pt::kDshotMin);
  return static_cast<std::uint16_t>(std::lround(f) + pt::kDshotMin);
}
// The smallest command whose speed is at least omega; the largest whose speed is at most omega.
std::uint16_t dshot_at_least(double omega) {
  std::uint16_t d = dshot_near(omega);
  while (omega_of_dshot(d) < omega) { ++d; }
  while (d > pt::kDshotMin && omega_of_dshot(static_cast<std::uint16_t>(d - 1)) >= omega) { --d; }
  return d;
}
std::uint16_t dshot_at_most(double omega) {
  std::uint16_t d = dshot_near(omega);
  while (omega_of_dshot(d) > omega) { --d; }
  while (omega_of_dshot(static_cast<std::uint16_t>(d + 1)) <= omega) { ++d; }
  return d;
}

enum class Kind : std::uint8_t { Equal, Distinct, Single };

struct SpeedSet {
  std::string name;
  std::array<std::uint16_t, kMotors> dshot{};
  Kind kind = Kind::Equal;
  // Bit i set iff motor i + 1's notches are bypassed in steady state: its speed is below omega_th (stopped included).
  std::uint32_t expect_bypass = 0;
};

// Speed sets. The grid runs from omega_th (the lowest set: every motor at the first command with at least
// omega_th (1 + eps), so that the telemetry's reading, up to 2^-8 high and never low, is above omega_th) to omega_max = 2800
// rad/s (the top set: motor 1 at the largest command with a speed of at most 2800). Between: 700 rad/s, the hover speed
// (1100.591), 1800 rad/s: labelled, spread over the range. The distinct sets put the four motors at the fractions
// kDistinctFactors of their base; their base starts at 460 so that the slowest motor (0.78) is still above omega_th (1 + eps).
std::vector<SpeedSet> speed_sets() {
  std::vector<SpeedSet> sets;
  const std::array<double, 5> equal_targets{omega_threshold() * (1.0 + epsilon()), 700.0,
                                            kOmegaHover, 1800.0, kOmegaMaxCard};
  for (std::size_t i = 0; i < equal_targets.size(); ++i) {
    SpeedSet s;
    s.name = "equal@" + std::to_string(static_cast<int>(equal_targets[i]));
    const std::uint16_t d = i == 0 ? dshot_at_least(equal_targets[i]) : (i + 1 == equal_targets.size() ? dshot_at_most(equal_targets[i]) : dshot_near(equal_targets[i]));
    s.dshot = {d, d, d, d};
    sets.push_back(s);
  }
  const std::array<double, 5> distinct_bases{460.0, 700.0, kOmegaHover, 1800.0, kOmegaMaxCard};
  for (std::size_t i = 0; i < distinct_bases.size(); ++i) {
    SpeedSet s;
    s.name = "distinct@" + std::to_string(static_cast<int>(distinct_bases[i]));
    s.kind = Kind::Distinct;
    for (std::size_t m = 0; m < kMotors; ++m) {
      const double target = distinct_bases[i] * kDistinctFactors[m];
      s.dshot[m] = (i + 1 == distinct_bases.size() && m == 0) ? dshot_at_most(target) : dshot_near(target);
    }
    sets.push_back(s);
  }
  // The single-rotor sets: motor 1 at the speed of the equal set, motors 2 to 4 stopped (DShot 0: speed exactly 0, a valid
  // reading below omega_th, so their notches are bypassed and flagged, and they add no vibration). The chain then has only
  // motor 1's three notches active, so no other notch attenuates a component of motor 1: this is where a tracking error
  // shows undiluted. (With 12 notches of Q about 2 in a cascade, a component 7 % from another rotor's notch is already
  // attenuated by that notch, which is why the controls (a) and (c) are judged on these sets.)
  for (std::size_t i = 0; i < equal_targets.size(); ++i) {
    SpeedSet s;
    s.name = "single@" + std::to_string(static_cast<int>(equal_targets[i]));
    s.kind = Kind::Single;
    s.dshot = {sets[i].dshot[0], 0, 0, 0};
    sets.push_back(s);
  }
  for (SpeedSet& s : sets) {
    for (std::size_t m = 0; m < kMotors; ++m) {
      if (omega_of_dshot(s.dshot[m]) < omega_threshold()) {
        s.expect_bypass |= marv::rotor_speed_valid_bit(m);
      }
    }
  }
  return sets;
}

// ---- The simulation ---------------------------------------------------------------------------------------------------
struct Trace {
  std::vector<std::array<float, 3>> gyro;    // the IMU's output (quantised)
  std::vector<std::array<float, 3>> gyro_u;  // the same signal without the quantiser (the unquantised companion)
  double gyro_u_peak = 0.0;
  std::vector<RotorSpeedSample> rotor;
  double gyro_peak = 0.0;
};

struct PlantDeleter {
  void operator()(marv_plant* p) const { marv_plant_destroy(p); }
};

class Sim {
 public:
  Sim(const std::array<std::uint16_t, kMotors>& dshot, double amp, double period) : dshot_(dshot), dt_(period) {
    marv_plant_config c = pt::fixture_config();
    c.rng_seed = kSeed;
    marv_plant* p = nullptr;
    EXPECT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
    plant_.reset(p);
    marv_plant_imu_config imu = marv::sim::imu_profile_config();
    imu.gyro.noise_density = 0.0;
    imu.gyro.bias_instability = 0.0;
    imu.accel.noise_density = 0.0;
    imu.accel.bias_instability = 0.0;
    EXPECT_EQ(marv_plant_imu_attach(plant_.get(), &imu), MARV_PLANT_OK);
    marv_plant_vibration_config vib{};
    vib.struct_size = sizeof(vib);
    for (double& a : vib.amplitude_rad_s) {
      a = amp;
    }
    vib.omega_hover_rad_s = kOmegaHover;
    vib.speed_exponent = 2.0;
    EXPECT_EQ(marv_plant_vibration_attach(plant_.get(), &vib), MARV_PLANT_OK);
    EXPECT_EQ(marv_plant_rotor_speed_attach(plant_.get(), &rotor_cfg()), MARV_PLANT_OK);
    body_ = pt::identity_body();
    cmd_ = pt::make_cmd(dshot[0], dshot[1], dshot[2], dshot[3]);

    // The companion's signal: the plant's own vibration code (VibrationModel: the same phases from the same seed and stream,
    // the same formula) on a twin of the plant's motor model (Model: the same omega and theta, bitwise: the L06/vibration
    // tests pin it), without the IMU: no quantiser, no clamp. The latency is the IMU's: sample k carries the truth of
    // sample k - latency, sample 0 its own (marv_plant.h). The twin is advanced exactly as the plant is (after the sample).
    marv::plant::Params<double> mp;
    mp.tau = c.motor_tau_s;
    mp.substep = c.motor_substep_s;
    mp.omega_min = c.omega_min_rad_s;
    mp.omega_max = c.omega_max_rad_s;
    twin_ = std::make_unique<marv::plant::Model<double>>(mp);
    marv::plant::VibrationParams vp;
    vp.amplitude = {amp, amp, amp};
    vp.omega_hover = kOmegaHover;
    vp.exponent = 2;
    vib_.attach(vp, kSeed);
    imu_latency_ = marv::sim::imu_profile_config().latency_samples;
    for (std::size_t m = 0; m < kMotors; ++m) {
      dshot_u16_[m] = dshot[m];
    }
  }

  void advance(std::size_t n, Trace& tr) {
    const std::uint32_t sat_mask = (1U << MARV_PLANT_IMU_GYRO_SAT_X) | (1U << MARV_PLANT_IMU_GYRO_SAT_Y) | (1U << MARV_PLANT_IMU_GYRO_SAT_Z);
    for (std::size_t i = 0; i < n; ++i) {
      marv_plant_imu_out io{};
      EXPECT_EQ(marv_plant_imu_sample(plant_.get(), &body_, dt_, &io), MARV_PLANT_OK);
      EXPECT_TRUE((io.flags & (1U << MARV_PLANT_IMU_GYRO_VALID)) != 0U);
      EXPECT_EQ(io.flags & sat_mask, 0U) << "the gyro saturated: the sweep's no-saturation rule is broken";
      marv_plant_rotor_speed_out ro{};
      EXPECT_EQ(marv_plant_rotor_speed_sample(plant_.get(), &ro), MARV_PLANT_OK);
      marv_plant_out po = pt::make_out();
      EXPECT_EQ(marv_plant_step(plant_.get(), &body_, &cmd_, dt_, &po), MARV_PLANT_OK);
      tr.gyro.push_back({io.gyro_rad_s.x, io.gyro_rad_s.y, io.gyro_rad_s.z});
      for (const float g : tr.gyro.back()) {
        tr.gyro_peak = std::max(tr.gyro_peak, static_cast<double>(std::fabs(g)));
      }
      // The twin's truth of this tick, delayed by the IMU's latency, as the float the chain receives.
      truth_.push_back(vib_.value(twin_->omega(), twin_->theta()));
      const std::array<double, 3>& delayed = truth_[truth_.size() > imu_latency_ ? truth_.size() - 1 - imu_latency_ : 0];
      tr.gyro_u.push_back({static_cast<float>(delayed[0]), static_cast<float>(delayed[1]), static_cast<float>(delayed[2])});
      for (std::size_t a = 0; a < 3; ++a) {
        tr.gyro_u_peak = std::max(tr.gyro_u_peak, std::fabs(static_cast<double>(tr.gyro_u.back()[a])));
        // The companion signal is the plant's IMU output without its rounding: they differ by at most half an LSB plus the
        // float storage of the value (no clamp: nothing saturates).
        EXPECT_LE(std::fabs(static_cast<double>(tr.gyro.back()[a]) - static_cast<double>(tr.gyro_u.back()[a])),
                  gyro_lsb() / 2.0 + 2.0 * gct::kU * std::fabs(static_cast<double>(tr.gyro.back()[a])) )
            << "the unquantised signal is not the plant's IMU output before its quantiser";
      }
      twin_->advance(dshot_u16_, dt_);
      RotorSpeedSample s{};
      for (std::size_t m = 0; m < kMotors; ++m) {
        s.omega_rad_s[m] = ro.omega_rad_s[m];
      }
      s.flags = ro.flags;
      tr.rotor.push_back(s);
      omega_true_ = {po.rotor_speed_rad_s[0], po.rotor_speed_rad_s[1], po.rotor_speed_rad_s[2], po.rotor_speed_rad_s[3]};
    }
  }
  const std::array<double, kMotors>& omega_true() const { return omega_true_; }

 private:
  static const marv_plant_rotor_speed_config& rotor_cfg() {
    static const marv_plant_rotor_speed_config c = rt::grid_config(static_cast<std::uint32_t>(rate_latency_ticks()));
    return c;
  }
  std::array<std::uint16_t, kMotors> dshot_;
  double dt_;
  std::unique_ptr<marv_plant, PlantDeleter> plant_;
  marv_plant_body body_{};
  marv_plant_cmd cmd_{};
  std::array<double, kMotors> omega_true_{};
  std::unique_ptr<marv::plant::Model<double>> twin_;
  marv::plant::VibrationModel vib_;
  std::size_t imu_latency_ = 0;
  std::array<std::uint16_t, kMotors> dshot_u16_{};
  std::vector<std::array<double, 3>> truth_;
};

// ---- Chain variants ---------------------------------------------------------------------------------------------------
struct Variant {
  std::string name;
  std::function<RotorSpeedSample(const RotorSpeedSample&)> map;
};

RotorSpeedSample scaled(const RotorSpeedSample& s, double k) {
  RotorSpeedSample o = s;
  for (float& w : o.omega_rad_s) {
    w = static_cast<float>(static_cast<double>(w) * k);
  }
  return o;
}

double twin_scale() { return 0.9 * epsilon() - telemetry_step(); }
double beyond_scale() { return 1.5 * epsilon(); }
constexpr double kHarmonicScale = 2.0 / 3.0;  // labelled: h f_chain = (2/3) h f misses every harmonic of the rotor

std::vector<Variant> variants() {
  std::vector<Variant> v;
  v.push_back({"nominal", [](const RotorSpeedSample& s) { return s; }});
  v.push_back({"twin x(1+0.9eps-2^-8)", [](const RotorSpeedSample& s) { return scaled(s, 1.0 + twin_scale()); }});
  v.push_back({"(a) x(1+1.5eps)", [](const RotorSpeedSample& s) { return scaled(s, 1.0 + beyond_scale()); }});
  v.push_back({"permuted (equivalence)", [](const RotorSpeedSample& s) {
                 RotorSpeedSample o = s;
                 for (std::size_t m = 0; m < kMotors; ++m) {
                   o.omega_rad_s[m] = s.omega_rad_s[(m + 1) % kMotors];
                 }
                 return o;
               }});
  v.push_back({"(b) slot 1 reads slot 4", [](const RotorSpeedSample& s) {
                 RotorSpeedSample o = s;
                 o.omega_rad_s[0] = s.omega_rad_s[kMotors - 1];
                 o.flags = (s.flags & ~marv::rotor_speed_valid_bit(0)) | ((s.flags >> (kMotors - 1)) & 1U);
                 return o;
               }});
  v.push_back({"(c) harmonic x2/3", [](const RotorSpeedSample& s) { return scaled(s, kHarmonicScale); }});
  v.push_back({"(d) bypassed", [](const RotorSpeedSample& s) {
                 RotorSpeedSample o = s;
                 o.flags = 0;
                 o.omega_rad_s = {};
                 return o;
               }});
  return v;
}

GyroChainConfig<float> chain_config() {
  EXPECT_TRUE(gct::init_params());
  GyroChainConfig<float> c;
  const std::array<float, kHarmonics> q{marv::param_value<marv::ParamId::gyro_notch_q_h1>(),
                                        marv::param_value<marv::ParamId::gyro_notch_q_h2>(),
                                        marv::param_value<marv::ParamId::gyro_notch_q_h3>()};
  c.period = gct::tick_period_s();
  c.rate_divisor = rate_divisor();
  c.cutoff_hz = marv::param_value<marv::ParamId::gyro_lpf_cutoff_hz>();
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    c.notch_q[h] = q[h];
  }
  c.omega_threshold_rad_s = marv::param_value<marv::ParamId::gyro_notch_omega_min>();
  return c;
}

struct Replay {
  std::vector<std::array<double, 3>> y;  // from `from` to the end
  std::vector<std::array<double, 3>> y_ref;  // the double reference chain's output, same range
  std::uint32_t bypass_flags = 0;
  std::array<std::uint32_t, kMotors> bypass_count{};
  std::size_t updates = 0;
};

std::vector<gct::Stage> design_stages(const RotorSpeedSample& s, const GyroChainConfig<float>& cfg);

// Runs the firmware chain (float) and, in step, the double reference: the same notch rule, the K-form design of the test
// support (independent of the firmware's RBJ form), direct form I in double. The reference separates the float arithmetic of
// the chain from the design: the fit of the difference of the two outputs is the float arithmetic's and the float
// coefficients' effect on each component's amplitude, which the derived cap (round_cap) must bound.
Replay replay(const Trace& tr, const Variant& v, std::size_t from, bool unquantised) {
  const std::vector<std::array<float, 3>>& gyro = unquantised ? tr.gyro_u : tr.gyro;
  const GyroChainConfig<float> cfg = chain_config();
  GyroChain<float> chain;
  chain.init(cfg);
  const std::size_t divisor = rate_divisor();
  Replay r;
  std::vector<gct::Stage> ref_stages(marv::gyro_chain::kNotches + 1);
  struct State {
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  };
  std::array<std::array<State, marv::gyro_chain::kNotches + 1>, 3> ref_state{};
  for (std::size_t j = 0; j < gyro.size(); ++j) {
    if (j % divisor == 0) {
      const RotorSpeedSample sample = v.map(tr.rotor[j]);
      chain.update_notches(sample);
      ref_stages = design_stages(sample, cfg);
      ++r.updates;
    }
    const gct::Vec3f out = chain.filter(gct::Vec3f{gyro[j][0], gyro[j][1], gyro[j][2]});
    std::array<double, 3> ref{};
    for (std::size_t a = 0; a < 3; ++a) {
      double val = static_cast<double>(gyro[j][a]);
      for (std::size_t k = 0; k < ref_stages.size(); ++k) {
        const gct::Stage& c = ref_stages[k];
        State& st = ref_state[a][k];
        const double yv = c.b0 * val + c.b1 * st.x1 + c.b2 * st.x2 - c.a1 * st.y1 - c.a2 * st.y2;
        st.x2 = st.x1;
        st.x1 = val;
        st.y2 = st.y1;
        st.y1 = yv;
        val = yv;
      }
      ref[a] = val;
    }
    if (j >= from) {
      r.y.push_back({out[0], out[1], out[2]});
      r.y_ref.push_back(ref);
    }
  }
  r.bypass_flags = chain.bypass_flags();
  for (std::size_t m = 0; m < kMotors; ++m) {
    r.bypass_count[m] = chain.bypass_count(m);
  }
  return r;
}

// ---- The double design of a variant's chain at its steady speeds and the allowances ----------------------------------------
struct Bounds {
  double l1 = 0.0;
  double cum_l1_max = 0.0;  // largest l1 norm of the impulse response up to a stage: bounds a stage's signal from the noise
  std::size_t n_settle = 0;
  gct::Plan plan;                    // P2's rounding plan (support.hpp make_plan) of the active notches and the low-pass
  std::vector<gct::StageErr> se;     // the same stages with their float-coefficient error bounds
  std::vector<double> coef_effect;   // per group: first-order bound on |H_float - H_design| at the group's frequency
  std::array<double, marv::gyro_chain::kNotches> gain{};  // |H| of the whole chain at the true component frequencies
};

std::vector<gct::Stage> design_stages(const RotorSpeedSample& s, const GyroChainConfig<float>& cfg) {
  const double period = static_cast<double>(cfg.period);
  std::vector<gct::Stage> st;
  for (std::size_t m = 0; m < kMotors; ++m) {
    const double omega = static_cast<double>(s.omega_rad_s[m]);
    const bool usable = (s.flags & marv::rotor_speed_valid_bit(m)) != 0 && std::isfinite(omega) &&
                        omega >= static_cast<double>(cfg.omega_threshold_rad_s);
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      const double f0 = static_cast<double>(h + 1) * omega / kTwoPi;
      if (usable && f0 < 0.5 / period) {
        st.push_back(gct::notch_design(f0, static_cast<double>(cfg.notch_q[h]), period));
      } else {
        st.push_back(gct::Stage{});
      }
    }
  }
  st.push_back(gct::lowpass_design(static_cast<double>(cfg.cutoff_hz), period));
  return st;
}

std::vector<double> impulse(std::size_t n) {
  std::vector<double> d(n, 0.0);
  d[0] = 1.0;
  return d;
}

// P2's plan inputs for the same chain: the active notches (identity stages are exact in float and left out, as plan_for
// does) and the low-pass, each with its coefficient error bound at the float inputs (f0 from the float speed, kF0RelErr).
std::vector<gct::StageErr> design_errs(const RotorSpeedSample& s, const GyroChainConfig<float>& cfg) {
  std::vector<gct::StageErr> se;
  const double period = static_cast<double>(cfg.period);
  for (std::size_t m = 0; m < kMotors; ++m) {
    const double omega = static_cast<double>(s.omega_rad_s[m]);
    const bool usable = (s.flags & marv::rotor_speed_valid_bit(m)) != 0 && std::isfinite(omega) &&
                        omega >= static_cast<double>(cfg.omega_threshold_rad_s);
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      const double f0 = static_cast<double>(h + 1) * omega / kTwoPi;
      if (usable && f0 < 0.5 / period) {
        se.push_back(gct::notch_err(f0, cfg.notch_q[h], cfg.period, gct::kF0RelErr));
      }
    }
  }
  se.push_back(gct::lowpass_err(cfg.cutoff_hz, cfg.period));
  return se;
}

// Flush-to-zero while P2's plan runs its 40000-sample impulse responses: a response that decays past 1e-308 would otherwise
// run in denormals (about 100 times slower, 2 s per speed set instead of 0.02 s). Values below 1e-308 are irrelevant to the
// plan (its sums are of order 1). A no-op where the SSE control word does not exist.
class FlushDenormals {
 public:
#if defined(__SSE2__)
  FlushDenormals() : saved_(_mm_getcsr()) { _mm_setcsr(saved_ | kFlushZero | kDenormalsZero); }
  ~FlushDenormals() { _mm_setcsr(saved_); }

 private:
  static constexpr unsigned kFlushZero = 0x8000U;     // MXCSR FZ, Intel SDM vol. 1 section 10.2.3
  static constexpr unsigned kDenormalsZero = 0x0040U;  // MXCSR DAZ
  unsigned saved_;
#endif
};

Bounds make_bounds(const std::vector<gct::Stage>& st, std::vector<gct::StageErr> se,
                   const std::array<double, kMotors>& omega_true, double period) {
  const FlushDenormals flush;
  Bounds b;
  b.se = std::move(se);
  b.plan = gct::make_plan(b.se);
  const std::size_t k_n = st.size();
  std::vector<std::vector<double>> cum(k_n + 1);
  cum[0] = impulse(kDesignLen);
  for (std::size_t k = 0; k < k_n; ++k) {
    cum[k + 1] = gct::run_stage(st[k], cum[k]);
  }
  EXPECT_LT(std::fabs(cum[k_n].back()), kDecayTol) << "the design impulse response has not decayed";
  for (const std::vector<double>& c : cum) {
    b.cum_l1_max = std::max(b.cum_l1_max, gct::l1(c));
  }
  b.l1 = gct::l1(cum[k_n]);
  b.n_settle = gct::settle_samples(cum[k_n], gct::kTransientTol);
  const double fs = 1.0 / period;
  for (std::size_t m = 0; m < kMotors; ++m) {
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      const double f = static_cast<double>(h + 1) * omega_true[m] / kTwoPi;
      b.gain[marv::gyro_chain::notch_index(m, h)] = std::abs(gct::chain_response(st, gct::digital_w(f, fs)));
    }
  }
  return b;
}

// P2's rounding bound (support.hpp rounding_bound, DTFT-aware) at the digital frequency w, for this test's window N and for
// signals of peak s_late after the settling time (P2: 1) and x_early before it (P2: 1): the bound is linear in the signal
// amplitudes, so P2's per-stage local errors (delta_late, delta_early, for unit signals) scale with them. The input's own
// rounding (the IMU's float storage) is the u |x| term. Formula: the comment of support.hpp Plan.
double round_dtft(const Bounds& b, double w, double s_late, double x_early) {
  const double n = static_cast<double>(kWindow);
  const gct::Cd z1 = std::polar(1.0, -w);
  const gct::Cd z2 = z1 * z1;
  const std::vector<gct::Stage>& d = b.plan.design;
  gct::Cd total = 1.0;
  for (const gct::Stage& st : d) {
    total *= gct::stage_response(st, w);
  }
  double r = 2.0 * gct::kU * std::abs(total) * s_late + (2.0 / n) * (2.0 * gct::kU) * b.plan.m1.back() * x_early;
  for (std::size_t k = 0; k < d.size(); ++k) {
    gct::Cd g = 1.0 / (1.0 + d[k].a1 * z1 + d[k].a2 * z2);
    for (std::size_t j = k + 1; j < d.size(); ++j) {
      g *= gct::stage_response(d[j], w);
    }
    r += 2.0 * b.plan.delta_late[k] * s_late * std::abs(g) +
         (2.0 / n) * (b.plan.delta_late[k] * s_late + b.plan.delta_early[k] * x_early) * b.plan.m1[k];
  }
  return gct::kBoundSlack * r;
}

// ---- The joint least-squares fit --------------------------------------------------------------------------------------
struct Group {
  std::size_t harmonic = 0;     // 1..3
  double omega_rad_s = 0.0;     // the component frequency h omega_i
  double c_unit = 0.0;          // sum of (omega_i / omega_hover)^2 over the members: c = A c_unit
  std::vector<std::size_t> members;
};

std::vector<Group> make_groups(const SpeedSet& s, const std::array<double, kMotors>& omega) {
  std::vector<Group> groups;
  for (std::size_t m = 0; m < kMotors; ++m) {
    if (s.dshot[m] == 0) {
      continue;  // a stopped rotor has no vibration (c = 0)
    }
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      Group* found = nullptr;
      for (Group& g : groups) {
        if (g.harmonic == h + 1 && s.dshot[g.members.front()] == s.dshot[m]) {
          found = &g;
        }
      }
      if (found == nullptr) {
        groups.emplace_back();
        found = &groups.back();
        found->harmonic = h + 1;
        found->omega_rad_s = static_cast<double>(h + 1) * omega_of_dshot(s.dshot[m]);
      }
      found->members.push_back(m);
      const double r = omega[m] / kOmegaHover;
      found->c_unit += r * r;
    }
  }
  return groups;
}

struct Fit {
  std::size_t g = 0;
  std::vector<double> phi;   // kWindow x 2g, row-major: cos, sin per group
  std::vector<double> ginv;  // (2g)^2
  std::vector<double> row_norm;  // R_g, one per group
};

Fit make_fit(const std::vector<Group>& groups, double period) {
  Fit f;
  f.g = groups.size();
  const std::size_t q = 2 * f.g;
  f.phi.assign(kWindow * q, 0.0);
  for (std::size_t n = 0; n < kWindow; ++n) {
    for (std::size_t k = 0; k < f.g; ++k) {
      const double a = groups[k].omega_rad_s * period * static_cast<double>(n);
      f.phi[n * q + 2 * k] = std::cos(a);
      f.phi[n * q + 2 * k + 1] = std::sin(a);
    }
  }
  std::vector<double> gram(q * q, 0.0);
  for (std::size_t n = 0; n < kWindow; ++n) {
    for (std::size_t a = 0; a < q; ++a) {
      for (std::size_t b = 0; b < q; ++b) {
        gram[a * q + b] += f.phi[n * q + a] * f.phi[n * q + b];
      }
    }
  }
  // Gauss-Jordan with partial pivoting.
  std::vector<double> inv(q * q, 0.0);
  std::vector<double> m = gram;
  for (std::size_t i = 0; i < q; ++i) {
    inv[i * q + i] = 1.0;
  }
  for (std::size_t c = 0; c < q; ++c) {
    std::size_t piv = c;
    for (std::size_t r = c + 1; r < q; ++r) {
      if (std::fabs(m[r * q + c]) > std::fabs(m[piv * q + c])) {
        piv = r;
      }
    }
    for (std::size_t k = 0; k < q; ++k) {
      std::swap(m[c * q + k], m[piv * q + k]);
      std::swap(inv[c * q + k], inv[piv * q + k]);
    }
    const double d = m[c * q + c];
    for (std::size_t k = 0; k < q; ++k) {
      m[c * q + k] /= d;
      inv[c * q + k] /= d;
    }
    for (std::size_t r = 0; r < q; ++r) {
      if (r != c) {
        const double e = m[r * q + c];
        for (std::size_t k = 0; k < q; ++k) {
          m[r * q + k] -= e * m[c * q + k];
          inv[r * q + k] -= e * inv[c * q + k];
        }
      }
    }
  }
  double inv_err = 0.0;
  for (std::size_t a = 0; a < q; ++a) {
    for (std::size_t b = 0; b < q; ++b) {
      double s = 0.0;
      for (std::size_t k = 0; k < q; ++k) {
        s += gram[a * q + k] * inv[k * q + b];
      }
      inv_err = std::max(inv_err, std::fabs(s - (a == b ? 1.0 : 0.0)));
    }
  }
  EXPECT_LT(inv_err, kInverseTol) << "the fit's normal matrix is ill-conditioned: components too close for the window";
  f.ginv = inv;
  // R_k = l1 norm of the row k of (G^-1 Phi^T).
  std::vector<double> rk(q, 0.0);
  for (std::size_t n = 0; n < kWindow; ++n) {
    for (std::size_t a = 0; a < q; ++a) {
      double s = 0.0;
      for (std::size_t b = 0; b < q; ++b) {
        s += f.ginv[a * q + b] * f.phi[n * q + b];
      }
      rk[a] += std::fabs(s);
    }
  }
  for (std::size_t k = 0; k < f.g; ++k) {
    f.row_norm.push_back(std::hypot(rk[2 * k], rk[2 * k + 1]));
  }
  return f;
}

std::vector<double> amplitudes(const Fit& f, const std::vector<double>& y) {
  const std::size_t q = 2 * f.g;
  std::vector<double> v(q, 0.0);
  for (std::size_t n = 0; n < kWindow; ++n) {
    for (std::size_t a = 0; a < q; ++a) {
      v[a] += f.phi[n * q + a] * y[n];
    }
  }
  std::vector<double> amp;
  for (std::size_t k = 0; k < f.g; ++k) {
    double c = 0.0;
    double s = 0.0;
    for (std::size_t b = 0; b < q; ++b) {
      c += f.ginv[(2 * k) * q + b] * v[b];
      s += f.ginv[(2 * k + 1) * q + b] * v[b];
    }
    amp.push_back(std::hypot(c, s));
  }
  return amp;
}

// ---- A set's context, a run's result ----------------------------------------------------------------------------------
struct RunResult {
  // per group x axis (index g * 3 + axis)
  std::vector<double> amp;
  std::vector<double> c;      // sum of the component amplitudes at the IMU
  std::vector<double> bound;  // a_min c + allowance
  std::vector<double> allow;
  double worst_excess = -std::numeric_limits<double>::infinity();  // max over (g, axis) of amp - bound
  double worst_ratio = 0.0;                                          // max over (g, axis) of amp / bound
  double worst_amp = 0.0;                                            // the amp and bound of the worst ratio
  double worst_bound = 0.0;
  std::uint32_t bypass_flags = 0;
  std::array<std::uint32_t, kMotors> bypass_count{};
  std::size_t updates = 0;
  bool gains_ok = true;
  double allow_over_ac = 0.0;  // largest allowance / (a_min c) over the groups (reported)
  // The power rule (control (a) variant only): max over the groups of (G_a - a_min) c - allowance, G_a the design gain of the
  // variant's chain at the group's component frequency (min over the group's members). Positive iff the bound can tell the
  // control's chain from the design's: G_a c - allowance > a_min c.
  double power_margin = -std::numeric_limits<double>::infinity();
  double power_gain = 0.0;     // G_a of the group that gives the margin
  double power_allow_over_ac = 0.0;  // that group's allowance / (a_min c)
  double cap_over_ac = 0.0;    // largest float-arithmetic cap / (a_min c)
  double diff_over_cap = 0.0;  // largest measured float-vs-reference amplitude difference / its cap
};

struct SetContext {
  SpeedSet set;
  std::vector<Group> groups;
  Fit fit;
  std::vector<Variant> var;
  std::vector<Bounds> bounds;
  std::size_t n_motor = 0;
  std::size_t window_start = 0;
  bool ready = false;
};

std::size_t motor_settle_ticks(double period) {
  const double h_max = static_cast<double>(kHarmonics);
  const double t = pt::kTau * std::log(h_max * kOmegaMaxCard * pt::kTau / kPhiTol);
  return static_cast<std::size_t>(std::ceil(t / period));
}

std::vector<RunResult> judge(SetContext& ctx, const Trace& tr, double amp, bool unquantised);

// Both checks of one simulation: the quantised one (the IMU's output) and the unquantised companion (the same signal before
// the IMU's quantiser).
struct RunPair {
  std::vector<RunResult> q;
  std::vector<RunResult> u;
};

// Runs a set at amplitude `amp` and judges every variant, quantised and unquantised. The first run of a set builds its
// context (the designs come from the telemetry after the motor settling, which does not depend on the amplitude).
RunPair run_set(SetContext& ctx, double amp) {
  const double period = tick_period();
  const GyroChainConfig<float> cfg = chain_config();
  Sim sim(ctx.set.dshot, amp, period);
  Trace tr;
  if (!ctx.ready) {
    ctx.n_motor = motor_settle_ticks(period);
    sim.advance(ctx.n_motor, tr);
    ctx.var = variants();
    ctx.groups = make_groups(ctx.set, sim.omega_true());
    ctx.fit = make_fit(ctx.groups, period);
    std::size_t n_chain = 0;
    for (const Variant& v : ctx.var) {
      const std::vector<gct::Stage> st = design_stages(v.map(tr.rotor.back()), cfg);
      ctx.bounds.push_back(make_bounds(st, design_errs(v.map(tr.rotor.back()), cfg), sim.omega_true(), period));
      for (const Group& g : ctx.groups) {
        ctx.bounds.back().coef_effect.push_back(
            gct::coefficient_effect(ctx.bounds.back().se, gct::digital_w(g.omega_rad_s / kTwoPi, 1.0 / period)));
      }
      n_chain = std::max(n_chain, ctx.bounds.back().n_settle);
    }
    ctx.window_start = ctx.n_motor + n_chain + rate_latency_ticks();
    ctx.ready = true;
    for (const double rk : ctx.fit.row_norm) {
      EXPECT_LT(rk, kRowNormMax) << ctx.set.name;
    }
  } else {
    sim.advance(ctx.n_motor, tr);
  }
  sim.advance(ctx.window_start + kWindow - ctx.n_motor, tr);
  // The settling criterion of the motors: the rotors are within the drift allowance of their commanded speeds.
  const double omega_tol = kPhiTol / (static_cast<double>(kHarmonics) * pt::kTau);
  for (std::size_t m = 0; m < kMotors; ++m) {
    EXPECT_LE(std::fabs(sim.omega_true()[m] - omega_of_dshot(ctx.set.dshot[m])), omega_tol) << ctx.set.name;
  }

  RunPair pair;
  pair.q = judge(ctx, tr, amp, false);
  pair.u = judge(ctx, tr, amp, true);
  return pair;
}

// The unquantised check's rounding term is the float storage of the input alone (no half LSB): see the header.
std::vector<RunResult> judge(SetContext& ctx, const Trace& tr, double amp, bool unquantised) {
  const double period = tick_period();
  std::vector<RunResult> out;
  const double half_lsb = unquantised ? 0.0 : gyro_lsb() / 2.0;
  const double x = unquantised ? tr.gyro_u_peak : tr.gyro_peak;
  for (std::size_t vi = 0; vi < ctx.var.size(); ++vi) {
    const Replay rp = replay(tr, ctx.var[vi], ctx.window_start, unquantised);
    const Bounds& b = ctx.bounds[vi];
    RunResult r;
    r.bypass_flags = rp.bypass_flags;
    r.bypass_count = rp.bypass_count;
    r.updates = rp.updates;
    const double d_q = (half_lsb + gct::kU * x) * b.l1;
    const double d_tr = x * gct::kTransientTol;
    const double dist = d_q + d_tr;
    // The float arithmetic and float coefficients: the derived cap (P2's DTFT rounding bound at each group's frequency, made
    // to the least-squares fit's operator, plus the coefficient effect on each tone's own gain). Signals: tones sum to at
    // most c_total after the settling time (every |H_k| <= 1), plus the IMU's rounding through the cumulative l1 norm; the
    // input peak x before it.
    double c_total = 0.0;
    for (const Group& g : ctx.groups) {
      c_total += amp * g.c_unit;
    }
    const double s_late = c_total + (half_lsb + gct::kU * x) * b.cum_l1_max;
    const std::size_t q = 2 * ctx.groups.size();
    std::vector<double> bw(ctx.groups.size());
    for (std::size_t g = 0; g < ctx.groups.size(); ++g) {
      bw[g] = round_dtft(b, gct::digital_w(ctx.groups[g].omega_rad_s / kTwoPi, 1.0 / period), s_late, x);
    }
    std::vector<double> cap(ctx.groups.size());
    for (std::size_t g = 0; g < ctx.groups.size(); ++g) {
      double e_c = 0.0;
      double e_s = 0.0;
      for (std::size_t j = 0; j < q; ++j) {
        e_c += std::fabs(ctx.fit.ginv[(2 * g) * q + j]) * bw[j / 2];
        e_s += std::fabs(ctx.fit.ginv[(2 * g + 1) * q + j]) * bw[j / 2];
      }
      const double scale = static_cast<double>(kWindow) / 2.0;  // (G^-1 Phi^T) = (G^-1 N/2) (2/N Phi^T): the DTFT bound's weights
      cap[g] = scale * std::hypot(e_c, e_s) +
               amp * ctx.groups[g].c_unit * b.coef_effect[g];
    }
    for (std::size_t m = 0; m < kMotors; ++m) {
      for (std::size_t h = 0; h < kHarmonics; ++h) {
        if (vi == 0 && ctx.set.dshot[m] != 0 && b.gain[marv::gyro_chain::notch_index(m, h)] > kAMin) {
          r.gains_ok = false;
        }
      }
    }
    std::vector<std::vector<double>> amps(3);
    for (std::size_t a = 0; a < 3; ++a) {
      std::vector<double> y(kWindow);
      for (std::size_t n = 0; n < kWindow; ++n) {
        y[n] = rp.y[n][a];
      }
      amps[a] = amplitudes(ctx.fit, y);
      std::vector<double> diff(kWindow);
      for (std::size_t n = 0; n < kWindow; ++n) {
        diff[n] = rp.y[n][a] - rp.y_ref[n][a];
      }
      const std::vector<double> d_amp = amplitudes(ctx.fit, diff);
      for (std::size_t g = 0; g < ctx.groups.size(); ++g) {
        EXPECT_LE(d_amp[g], cap[g]) << ctx.set.name << " " << ctx.var[vi].name << " group " << g << " axis " << a
                                    << ": the firmware chain differs from the design reference beyond the derived cap";
        r.cap_over_ac = std::max(r.cap_over_ac, cap[g] / (kAMin * amp * ctx.groups[g].c_unit));
        r.diff_over_cap = std::max(r.diff_over_cap, d_amp[g] / cap[g]);
      }
    }
    for (std::size_t g = 0; g < ctx.groups.size(); ++g) {
      const double c = amp * ctx.groups[g].c_unit;
      const double allow = dist * ctx.fit.row_norm[g] + cap[g] + kPhiTol * c;
      for (std::size_t a = 0; a < 3; ++a) {
        const double bound = kAMin * c + allow;
        r.amp.push_back(amps[a][g]);
        r.c.push_back(c);
        r.bound.push_back(bound);
        r.allow.push_back(allow);
        r.worst_excess = std::max(r.worst_excess, amps[a][g] - bound);
        if (amps[a][g] / bound > r.worst_ratio) {
          r.worst_ratio = amps[a][g] / bound;
          r.worst_amp = amps[a][g];
          r.worst_bound = bound;
        }
        r.allow_over_ac = std::max(r.allow_over_ac, allow / (kAMin * c));
      }
      double gain = std::numeric_limits<double>::infinity();
      for (const std::size_t m : ctx.groups[g].members) {
        gain = std::min(gain, b.gain[marv::gyro_chain::notch_index(m, ctx.groups[g].harmonic - 1)]);
      }
      const double margin = (gain - kAMin) * c - allow;
      if (margin > r.power_margin) {
        r.power_margin = margin;
        r.power_gain = gain;
        r.power_allow_over_ac = allow / (kAMin * c);
      }
    }
    out.push_back(std::move(r));
  }
  return out;
}

// The linearity predicate of check 3: the ratio amplitude / c of two runs of one set agrees within the allowances.
bool linear(const RunResult& a, const RunResult& b) {
  for (std::size_t i = 0; i < a.amp.size(); ++i) {
    if (std::fabs(a.amp[i] / a.c[i] - b.amp[i] / b.c[i]) > a.allow[i] / a.c[i] + b.allow[i] / b.c[i]) {
      return false;
    }
  }
  return true;
}

constexpr std::size_t kNominal = 0, kTwin = 1, kBeyond = 2, kPermuted = 3, kMisrouted = 4, kHarmonic = 5, kBypassed = 6;

double amplitude_k(std::size_t k) { return a_max() * std::pow(0.5, static_cast<double>(k)); }

// ---- Tests -------------------------------------------------------------------------------------------------------------
TEST(Tracking, SweepAmplitudeMatchesP5) {
  // A_max by P5's rule at this test's constants is the 0.748 rad/s of decision 0013 P5.
  EXPECT_NEAR(a_max(), kP5AMax, kP5AMaxTol);
}

// ---- The grid: every speed set at every sweep amplitude, quantised and unquantised -------------------------------------------
struct Grid {
  std::vector<SpeedSet> sets;
  std::vector<std::vector<RunPair>> runs;  // [set][k]
};

Grid run_grid() {
  Grid g;
  g.sets = speed_sets();
  for (const SpeedSet& s : g.sets) {
    SetContext ctx;
    ctx.set = s;
    g.runs.emplace_back();
    for (std::size_t k = 0; k < kSweepSteps; ++k) {
      g.runs.back().push_back(run_set(ctx, amplitude_k(k)));
    }
  }
  return g;
}

// The slowest rotor that spins (a stopped rotor has no vibration).
double slowest_active_speed(const SpeedSet& s) {
  double w = std::numeric_limits<double>::infinity();
  for (const std::uint16_t d : s.dshot) {
    if (d != 0) {
      w = std::min(w, omega_of_dshot(d));
    }
  }
  return w;
}

// The power rule's decision (quantised or unquantised): the point (set, k) is asserted iff the control (a) chain's design gain
// G_a at some component satisfies G_a c - allowance > a_min c, i.e. the bound has the power to break that control. Derived:
// no labelled threshold.
bool powered(const RunResult& a_control) { return a_control.power_margin > 0.0; }
bool asserted(const RunPair& p) { return powered(p.q[kBeyond]); }

// The weakest (smallest) worst-ratio of a control over the single-rotor sets at k, for the report (the quantised check:
// over the asserted points only).
double weakest_control(const Grid& g, std::size_t k, std::size_t control, bool unquantised) {
  double w = std::numeric_limits<double>::infinity();
  for (std::size_t s = 0; s < g.sets.size(); ++s) {
    if (g.sets[s].kind == Kind::Single && (unquantised || asserted(g.runs[s][k]))) {
      const RunPair& p = g.runs[s][k];
      w = std::min(w, (unquantised ? p.u : p.q)[control].worst_ratio);
    }
  }
  return w;
}

const std::array<std::size_t, 4> kControls{kBeyond, kMisrouted, kHarmonic, kBypassed};
const std::array<const char*, 4> kControlNames{"(a) x(1+1.5eps)", "(b) slot 1 reads slot 4", "(c) harmonic x2/3", "(d) all bypassed"};

TEST(Tracking, ParametersMatchTheDesignReport) {
  // The chain's parameters are the generated product parameters; they equal decision 0013 P5's design report for the 2 %
  // ESC (AT32F421, -40..105 C): f_c 625.4161 Hz, Q 2.0070 / 1.8224 / 1.5373, omega_th 348.037 rad/s. Tolerance of each: half a
  // unit of the last digit the report gives, plus one float ulp of the value (the parameters are float).
  const GyroChainConfig<float> c = chain_config();
  auto tol = [](double decimals, double v) { return 0.5 * std::pow(10.0, -decimals) + v * std::ldexp(1.0, -23); };
  EXPECT_NEAR(c.cutoff_hz, 625.4161, tol(4, 625.4161));
  EXPECT_NEAR(c.notch_q[0], 2.0070, tol(4, 2.0070));
  EXPECT_NEAR(c.notch_q[1], 1.8224, tol(4, 1.8224));
  EXPECT_NEAR(c.notch_q[2], 1.5373, tol(4, 1.5373));
  EXPECT_NEAR(c.omega_threshold_rad_s, 348.037, tol(3, 348.037));
  EXPECT_EQ(c.rate_divisor, 2U);
  // epsilon (P5: 0.02397125 at 2 %, 65 ppm, 2^-8) and omega_hover = omega_th / sqrt(a_min) (P5: 1100.591).
  EXPECT_NEAR(epsilon(), 0.02397125, 1e-8);
  EXPECT_NEAR(static_cast<double>(c.omega_threshold_rad_s) / std::sqrt(static_cast<double>(
                  marv::param_value<marv::ParamId::gyro_chain_attenuation_min>())),
              kOmegaHover, 1e-6 * kOmegaHover);
  EXPECT_NEAR(static_cast<double>(marv::param_value<marv::ParamId::gyro_chain_attenuation_min>()), kAMin, 1e-7);
}

TEST(Tracking, QuantisedCheckResolutionRule) {
  // The quantised check (the IMU's output). On the single-rotor sets (where a tracking error shows undiluted, see the header) a
  // point (set x sweep amplitude k) is ASSERTED iff the check has power there: G_a c - allowance > a_min c for some component,
  // G_a the design gain of the control (a) chain (the double K-form design of the chain given x(1 + 1.5 eps)) at the
  // component's frequency. At an asserted point (a) (b) (c) must break the bound. Points failing the rule are run and printed
  // with their allowance, G_a and the verdict, not asserted. On every set and at every point, asserted or not, nominal, the
  // positive twin and the permutation equivalence pass, and (d) breaks the bound (its gain is about 1, power everywhere).
  // The equal and distinct sets have no (a) power by design (cross-notch dilution), so the rule does not apply there.
  const Grid g = run_grid();
  std::printf("A_max %.6f rad/s, a_min %.2f, eps %.6f; tick %.3f us, window %zu ticks\n", a_max(), kAMin,
              epsilon(), tick_period() * 1e6, kWindow);
  std::size_t asserted_points = 0;
  for (std::size_t s = 0; s < g.sets.size(); ++s) {
    std::string floor_line;
    for (std::size_t k = 0; k < kSweepSteps; ++k) {
      const RunPair& p = g.runs[s][k];
      const std::vector<RunResult>& r = p.q;
      const bool single = g.sets[s].kind == Kind::Single;
      const bool on = single && asserted(p);
      EXPECT_LE(r[kNominal].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " nominal";
      EXPECT_LE(r[kTwin].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " positive twin";
      EXPECT_LE(r[kPermuted].worst_excess, 0.0)
          << g.sets[s].name << " k=" << k << ": a permutation of the motors is invisible to the chain (equal Q per harmonic)";
      EXPECT_EQ(r[kNominal].bypass_flags, g.sets[s].expect_bypass) << g.sets[s].name << ": bypassed iff the rotor is below omega_th";
      EXPECT_TRUE(r[kNominal].gains_ok) << g.sets[s].name << ": the design gain at a component exceeds a_min";
      EXPECT_EQ(r[kBypassed].bypass_flags, marv::kRotorSpeedFlagsDefined) << g.sets[s].name << " (d): every motor flagged";
      EXPECT_GT(r[kBypassed].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " (d) does not break the bound";
      if (on) {
        ++asserted_points;
        floor_line += " " + std::to_string(k);
        for (const std::size_t c : {kBeyond, kMisrouted, kHarmonic}) {
          EXPECT_GT(r[c].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " " << r[c].worst_ratio << " control does not break";
        }
      }
      std::printf("%-14s k=%zu %s nominal %.3f twin %.3f | (a) %.3f (b) %.3f (c) %.3f (d) %.3f | allow/(a_min c) %.3f cap/(a_min c) %.4f "
                  "diff/cap %.3f | G_a %.4f allow(a)/(a_min c) %.3f power %.3g\n",
                  g.sets[s].name.c_str(), k, single ? (on ? "ASSERTED" : "below floor") : "n/a(no (a) power)", r[kNominal].worst_ratio, r[kTwin].worst_ratio,
                  r[kBeyond].worst_ratio, r[kMisrouted].worst_ratio, r[kHarmonic].worst_ratio, r[kBypassed].worst_ratio,
                  r[kNominal].allow_over_ac, r[kNominal].cap_over_ac, r[kNominal].diff_over_cap, r[kBeyond].power_gain,
                  r[kBeyond].power_allow_over_ac, r[kBeyond].power_margin);
    }
    std::printf("FLOOR %-14s asserted k:%s\n", g.sets[s].name.c_str(), floor_line.empty() ? " none" : floor_line.c_str());
    // The floor guard (owner addition 2): at the card's hover rotor speed and above, every k is asserted (on the single-rotor
    // sets, where the rule applies).
    if (g.sets[s].kind == Kind::Single && slowest_active_speed(g.sets[s]) >= kOmegaHover) {
      for (std::size_t k = 0; k < kSweepSteps; ++k) {
        EXPECT_TRUE(asserted(g.runs[s][k])) << "floor guard: " << g.sets[s].name << " k=" << k
                                            << " is below the derived floor although every rotor is at or above omega_hover";
      }
    }
  }
  std::printf("asserted points: %zu of %zu single-rotor points\n", asserted_points, g.sets.size() / 3 * kSweepSteps);
  EXPECT_GT(asserted_points, 0U);
  for (std::size_t k = 0; k < kSweepSteps; ++k) {
    double worst_ratio = 0.0;
    double w_amp = 0.0;
    double w_bound = 0.0;
    std::string w_name;
    for (std::size_t s = 0; s < g.sets.size(); ++s) {
      const RunResult& nom = g.runs[s][k].q[kNominal];
      if (nom.worst_ratio > worst_ratio) {
        worst_ratio = nom.worst_ratio;
        w_amp = nom.worst_amp;
        w_bound = nom.worst_bound;
        w_name = g.sets[s].name;
      }
    }
    std::printf("k=%zu A=%.5f: worst nominal residual %.6g vs bound %.6g (ratio %.3f) at %s; weakest control over single sets: "
                "(a) %.3f (b) %.3f (c) %.3f (d) %.3f\n",
                k, amplitude_k(k), w_amp, w_bound, worst_ratio, w_name.c_str(), weakest_control(g, k, kBeyond, false),
                weakest_control(g, k, kMisrouted, false), weakest_control(g, k, kHarmonic, false),
                weakest_control(g, k, kBypassed, false));
  }
  // Check 3, linearity: the ratio amplitude / c agrees across the sweep amplitudes (the vibration is linear in A, the chain
  // is linear, only the quantisation is not, and its allowance is included).
  for (std::size_t s = 0; s < g.sets.size(); ++s) {
    for (std::size_t k = 1; k < kSweepSteps; ++k) {
      EXPECT_TRUE(linear(g.runs[s][0].q[kNominal], g.runs[s][k].q[kNominal])) << g.sets[s].name << " k=0 vs " << k;
    }
  }
  // Its control: the predicate between a nominal run and a run of the (a) control chain fails, on every single-rotor set.
  std::size_t failing = 0;
  std::size_t singles = 0;
  for (std::size_t s = 0; s < g.sets.size(); ++s) {
    if (g.sets[s].kind == Kind::Single) {
      ++singles;
      failing += linear(g.runs[s][1].q[kNominal], g.runs[s][0].q[kBeyond]) ? 0U : 1U;
    }
  }
  std::printf("linearity control (nominal k=1 vs control (a) k=0): fails on %zu of %zu single-rotor sets\n", failing, singles);
  EXPECT_EQ(failing, singles);
}

TEST(Tracking, UnquantisedCompanionDownToTheThreshold) {
  // The same grid with the IMU's quantiser off (the signal is the plant's own vibration code on a twin of its motor model,
  // delayed by the IMU's latency, before the IMU's rounding; Sim checks it against the plant's IMU output to half an LSB).
  // The allowance is only the float arithmetic (the derived cap), the float storage of the input, the transient and the phase
  // drift. Asserted at every point, down to omega_th (the lowest sets are at omega_th (1 + eps)) and every k: nominal, the
  // twin and the permutation equivalence pass; (d) breaks the bound on every set; (a) (b) (c) on every single-rotor set.
  const Grid g = run_grid();
  for (std::size_t s = 0; s < g.sets.size(); ++s) {
    for (std::size_t k = 0; k < kSweepSteps; ++k) {
      const std::vector<RunResult>& r = g.runs[s][k].u;
      EXPECT_LE(r[kNominal].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " nominal";
      EXPECT_LE(r[kTwin].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " positive twin";
      EXPECT_LE(r[kPermuted].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " permutation equivalence";
      EXPECT_TRUE(r[kNominal].gains_ok) << g.sets[s].name;
      EXPECT_GT(r[kBypassed].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " (d) does not break the bound";
      if (g.sets[s].kind == Kind::Single) {
        EXPECT_TRUE(powered(r[kBeyond])) << g.sets[s].name << " k=" << k << ": the companion has no power against control (a)";
        for (const std::size_t c : {kBeyond, kMisrouted, kHarmonic}) {
          EXPECT_GT(r[c].worst_excess, 0.0) << g.sets[s].name << " k=" << k << " " << r[c].worst_ratio << " control does not break";
        }
      }
      std::printf("U %-14s k=%zu nominal %.3f twin %.3f | (a) %.3f (b) %.3f (c) %.3f (d) %.3f | allow/(a_min c) %.4f\n",
                  g.sets[s].name.c_str(), k, r[kNominal].worst_ratio, r[kTwin].worst_ratio, r[kBeyond].worst_ratio,
                  r[kMisrouted].worst_ratio, r[kHarmonic].worst_ratio, r[kBypassed].worst_ratio, r[kNominal].allow_over_ac);
    }
  }
  for (std::size_t k = 0; k < kSweepSteps; ++k) {
    double worst_nominal = 0.0;
    double worst_twin = 0.0;
    for (std::size_t s = 0; s < g.sets.size(); ++s) {
      worst_nominal = std::max(worst_nominal, g.runs[s][k].u[kNominal].worst_ratio);
      worst_twin = std::max(worst_twin, g.runs[s][k].u[kTwin].worst_ratio);
    }
    std::printf("U k=%zu: worst nominal ratio %.3f, worst twin %.3f; weakest control over single sets: (a) %.3f (b) %.3f (c) %.3f (d) %.3f\n",
                k, worst_nominal, worst_twin, weakest_control(g, k, kBeyond, true), weakest_control(g, k, kMisrouted, true),
                weakest_control(g, k, kHarmonic, true), weakest_control(g, k, kBypassed, true));
  }
}

TEST(Tracking, BelowThresholdTheNotchesAreBypassedAndFlagged) {
  // Check 2: below omega_th the notches are bypassed and the chain says so. The assertion is on the flags and counters, not
  // on a residual. All four rotors at the lowest command (DShot 48, 250 rad/s) and at four commands below omega_th: every
  // update of the run bypasses every motor (the rotors never reach omega_th), so every counter equals the update count. A
  // mixed set (one rotor below, three above) flags exactly the slow motor.
  const double period = tick_period();
  const std::uint16_t d_th = dshot_near(omega_threshold());
  struct Case {
    std::array<std::uint16_t, kMotors> dshot;
    std::uint32_t expect_flags;
    bool all_below;
  };
  const std::uint16_t d_min = static_cast<std::uint16_t>(pt::kDshotMin);
  const std::vector<Case> cases{
      {{d_min, d_min, d_min, d_min}, marv::kRotorSpeedFlagsDefined, true},
      {{d_min, static_cast<std::uint16_t>(d_th - 20), static_cast<std::uint16_t>(d_th - 10), static_cast<std::uint16_t>(d_th - 2)},
       marv::kRotorSpeedFlagsDefined, true},
      {{d_min, dshot_near(700.0), dshot_near(1100.0), dshot_near(1800.0)}, marv::rotor_speed_valid_bit(0), false},
  };
  for (const Case& c : cases) {
    for (const std::uint16_t d : c.dshot) {
      EXPECT_TRUE(c.all_below == false || omega_of_dshot(d) < omega_threshold());
    }
    Sim sim(c.dshot, amplitude_k(0), period);
    Trace tr;
    sim.advance(motor_settle_ticks(period) + kWindow, tr);
    const Replay rp = replay(tr, variants()[kNominal], 0, false);
    EXPECT_EQ(rp.bypass_flags, c.expect_flags);
    if (c.all_below) {
      for (std::size_t m = 0; m < kMotors; ++m) {
        EXPECT_EQ(rp.bypass_count[m], rp.updates) << "motor " << m << ": bypassed at every update";
      }
    } else {
      EXPECT_EQ(rp.bypass_count[0], rp.updates);
      for (std::size_t m = 1; m < kMotors; ++m) {
        EXPECT_LT(rp.bypass_count[m], rp.updates) << "motor " << m << ": active once above omega_th";
      }
    }
  }
}

}  // namespace
