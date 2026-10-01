// L6 stage (a), the IMU's noise: seeded determinism, independence from marv_plant_step, the sample formulas against an
// independent oracle (with controls that must fail), and the SIL validity rules over a fuzz of states.
//
// The oracle restates the model from the normals (noise.hpp, sample_normals) and the formulas of imu_model.hpp. It is
// compared exactly (every output bit): the operations are mirrored one for one in IEEE double, the build forbids
// contraction, and the LSBs are powers of two, so the same operations give the same bits. A mutated oracle (a wrong
// random-walk gain, a wrong white sigma, the RW and white normals swapped) must not match.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "noise/noise.hpp"
#include "support.hpp"

namespace {

using namespace marv::plant::imu_test;
namespace noise = marv::plant::noise;

constexpr std::uint64_t kSeed = 20261001;  // labelled: any fixed seed
constexpr std::uint64_t kOtherSeed = kSeed + 1;

// Noise parameters, labelled test values (not sensor figures), chosen so that over kOracleSamples samples every value
// stays inside the 20-bit word at the LSBs below (no saturation, so the oracle needs no clamp):
//  - gyro: N = 1e-4 rad/s/sqrt(Hz), B = 2e-4 rad/s, LSB 2^-20: word range +-0.5 rad/s; truth 0.3; sigma_d at dt = 0.01
//    is 1e-4 / sqrt(0.02) = 7.1e-4; K = (sqrt(6)/2) 4e-8 / 1e-4 = 4.9e-4, per-sample drift K sqrt(dt) = 4.9e-5.
//  - accel: N = 2e-3, B = 4e-3 m/s^2, LSB 2^-16: word range +-8 m/s^2; truth z = -4.27; sigma_d 0.014; K = 9.8e-3.
constexpr double kGyroN = 1.0e-4;
constexpr double kGyroB = 2.0e-4;
constexpr double kAccelN = 2.0e-3;
constexpr double kAccelB = 4.0e-3;
constexpr double kNoisyGyroLsb = 1.0 / 1048576.0;  // 2^-20
constexpr double kNoisyAccelLsb = 1.0 / 65536.0;   // 2^-16
constexpr double kOmega0 = 2000.0;                 // rad/s: thrust 3.2 N, f_z = -4.2667 m/s^2
constexpr int kOracleSamples = 200;                // labelled: enough that the bias walks tens of thousands of counts

marv_plant_imu_config noisy(std::uint32_t latency, double gyro_b, double accel_b, double gyro_n, double accel_n) {
  marv_plant_imu_config c = zero_noise_config(latency);
  c.gyro.noise_density = gyro_n;
  c.gyro.bias_instability = gyro_b;
  c.gyro.lsb = kNoisyGyroLsb;
  c.accel.noise_density = accel_n;
  c.accel.bias_instability = accel_b;
  c.accel.lsb = kNoisyAccelLsb;
  c.gyro.turn_on_bias[0] = 0.01;  // labelled: a turn-on bias the walk starts from
  c.accel.turn_on_bias[2] = -0.02;
  return c;
}

marv_plant_imu_config default_noisy(std::uint32_t latency = 0) { return noisy(latency, kGyroB, kAccelB, kGyroN, kAccelN); }

std::vector<marv_plant_imu_out> run_plant(std::uint64_t seed, const marv_plant_imu_config& c, int n, double dt = kDt) {
  PlantHandle plant(seed, kOmega0);
  EXPECT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  std::vector<marv_plant_imu_out> v;
  for (int k = 0; k < n; ++k) {
    v.push_back(sample_ok(plant.get(), body_with_omega(0.3, 0.3, 0.3), dt));
  }
  return v;
}

struct Mutation {
  double k_scale = 1.0;
  double white_scale = 1.0;
  bool swap_rw_and_white = false;
};

float quantise(double y, double lsb) { return static_cast<float>(std::round(y / lsb) * lsb); }

std::vector<marv_plant_imu_out> oracle(std::uint64_t seed, const marv_plant_imu_config& c, int n, double dt,
                                       Mutation m = {}) {
  const double thrust = 4.0 * (kThrustCoeff * kOmega0 * kOmega0);
  const double fz = -(thrust / kMass);
  const double truth_g[3] = {0.3, 0.3, 0.3};
  const double truth_a[3] = {0.0, 0.0, fz};
  auto gain = [&](const marv_plant_imu_axis_config& a) {
    return (a.noise_density > 0.0 && a.bias_instability > 0.0)
               ? std::sqrt(6.0) * 0.5 * a.bias_instability * a.bias_instability / a.noise_density * m.k_scale
               : 0.0;
  };
  auto white = [&](const marv_plant_imu_axis_config& a) {
    return a.noise_density > 0.0 ? a.noise_density / std::sqrt(2.0 * dt) * m.white_scale : 0.0;
  };
  double bg[3] = {c.gyro.turn_on_bias[0], c.gyro.turn_on_bias[1], c.gyro.turn_on_bias[2]};
  double ba[3] = {c.accel.turn_on_bias[0], c.accel.turn_on_bias[1], c.accel.turn_on_bias[2]};
  std::vector<marv_plant_imu_out> v;
  for (int k = 0; k < n; ++k) {
    // Latency 0 only: the truth is constant, so the delay does not enter here (it is tested in kat_test.cpp).
    noise::SampleNormals z = noise::sample_normals(seed, noise::kStreamPrimaryImu, static_cast<std::uint64_t>(k));
    if (m.swap_rw_and_white) {
      for (std::size_t i = 0; i < 3; ++i) {
        std::swap(z[i], z[3 + i]);
        std::swap(z[6 + i], z[9 + i]);
      }
    }
    marv_plant_imu_out o{};
    float g[3];
    float a[3];
    for (std::size_t i = 0; i < 3; ++i) {
      bg[i] += gain(c.gyro) * std::sqrt(dt) * z[i];
      ba[i] += gain(c.accel) * std::sqrt(dt) * z[6 + i];
      g[i] = quantise(truth_g[i] + bg[i] + white(c.gyro) * z[3 + i], c.gyro.lsb);
      a[i] = quantise(truth_a[i] + ba[i] + white(c.accel) * z[9 + i], c.accel.lsb);
    }
    o.gyro_rad_s = {g[0], g[1], g[2]};
    o.accel_m_s2 = {a[0], a[1], a[2]};
    o.flags = kValidBits;
    v.push_back(o);
  }
  return v;
}

bool same_run(const std::vector<marv_plant_imu_out>& a, const std::vector<marv_plant_imu_out>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!same_bits(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

TEST(ImuNoise, MatchesTheOracleExactly) {
  const marv_plant_imu_config c = default_noisy();
  EXPECT_TRUE(same_run(run_plant(kSeed, c, kOracleSamples, kDt), oracle(kSeed, c, kOracleSamples, kDt)));
  // The seed is the plant's rng_seed: another seed matches the oracle of that seed.
  EXPECT_TRUE(same_run(run_plant(kOtherSeed, c, kOracleSamples, kDt), oracle(kOtherSeed, c, kOracleSamples, kDt)));
}

TEST(ImuNoise, OracleControlsMustFail) {
  const marv_plant_imu_config c = default_noisy();
  const std::vector<marv_plant_imu_out> real = run_plant(kSeed, c, kOracleSamples, kDt);
  constexpr double kOnePercent = 1.01;  // labelled: a 1 % error in a gain must already be seen
  Mutation k_wrong;
  k_wrong.k_scale = kOnePercent;
  Mutation w_wrong;
  w_wrong.white_scale = kOnePercent;
  Mutation swapped;
  swapped.swap_rw_and_white = true;
  EXPECT_FALSE(same_run(real, oracle(kSeed, c, kOracleSamples, kDt, k_wrong)));
  EXPECT_FALSE(same_run(real, oracle(kSeed, c, kOracleSamples, kDt, w_wrong)));
  EXPECT_FALSE(same_run(real, oracle(kSeed, c, kOracleSamples, kDt, swapped)));
  EXPECT_FALSE(same_run(real, oracle(kOtherSeed, c, kOracleSamples, kDt)));
}

TEST(ImuNoise, DegenerateParametersFollowTheirRules) {
  // B = 0: K = 0 (white noise only; the bias stays at the turn-on value). N = 0 with B > 0: K = 0 and no white noise
  // (the turn-on bias alone, constant). Both against the oracle, which has the same rule written independently.
  const marv_plant_imu_config b_zero = noisy(0, 0.0, 0.0, kGyroN, kAccelN);
  EXPECT_TRUE(same_run(run_plant(kSeed, b_zero, kOracleSamples, kDt), oracle(kSeed, b_zero, kOracleSamples, kDt)));
  const marv_plant_imu_config n_zero = noisy(0, kGyroB, kAccelB, 0.0, 0.0);
  const std::vector<marv_plant_imu_out> run = run_plant(kSeed, n_zero, kOracleSamples, kDt);
  EXPECT_TRUE(same_run(run, oracle(kSeed, n_zero, kOracleSamples, kDt)));
  for (const marv_plant_imu_out& o : run) {  // constant: the first sample repeated
    EXPECT_TRUE(same_bits(o, run.front()));
  }
  // Controls: B = 0 differs from B > 0 (the bias walks), and N > 0 differs from N = 0.
  EXPECT_FALSE(same_run(run_plant(kSeed, b_zero, kOracleSamples, kDt), run_plant(kSeed, default_noisy(), kOracleSamples, kDt)));
  EXPECT_FALSE(same_run(run, run_plant(kSeed, default_noisy(), kOracleSamples, kDt)));
}

TEST(ImuNoise, SameSeedBitIdenticalDifferentSeedDiffers) {
  const marv_plant_imu_config c = default_noisy(3);
  const std::vector<marv_plant_imu_out> a = run_plant(kSeed, c, kOracleSamples);
  EXPECT_TRUE(same_run(a, run_plant(kSeed, c, kOracleSamples)));
  const std::vector<marv_plant_imu_out> d = run_plant(kOtherSeed, c, kOracleSamples);
  EXPECT_FALSE(same_run(a, d));
  // Differs at most samples, not by one stray value: count the samples that differ (labelled: more than half).
  std::size_t differing = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    differing += same_bits(a[i], d[i]) ? 0U : 1U;
  }
  EXPECT_GT(differing, a.size() / 2);
}

TEST(ImuNoise, TheImuDoesNotChangeStepOutputs) {
  // Two plants of one config; one takes IMU samples (noise on, latency 2) between steps. Their marv_plant_step
  // outputs must be bit-identical over a varying command sequence. Control: the sequence moves the outputs.
  PlantHandle with(kSeed);
  PlantHandle without(kSeed);
  const marv_plant_imu_config c = default_noisy(2);
  ASSERT_EQ(marv_plant_imu_attach(with.get(), &c), MARV_PLANT_OK);
  const marv_plant_body b = body_with_omega(0.3, -0.2, 0.1);
  marv_plant_out first{};
  for (int k = 0; k < 60; ++k) {  // labelled
    const std::uint16_t d = static_cast<std::uint16_t>(300 + 20 * k);
    const marv_plant_cmd cmd = make_cmd(d, d, d, d);
    marv_plant_out o1 = make_out();
    marv_plant_out o2 = make_out();
    (void)sample_ok(with.get(), b);
    ASSERT_EQ(marv_plant_step(with.get(), &b, &cmd, kDt, &o1), MARV_PLANT_OK);
    ASSERT_EQ(marv_plant_step(without.get(), &b, &cmd, kDt, &o2), MARV_PLANT_OK);
    EXPECT_TRUE(same_bits(o1, o2)) << k;
    if (k == 0) {
      first = o1;
    } else if (k == 59) {
      EXPECT_FALSE(same_bits(first, o1));
    }
  }
}

TEST(ImuNoise, TheSeedDoesNotReachMarvPlantStep) {
  // Two plants that differ only in rng_seed give bit-identical step outputs: the seed feeds the IMU only.
  PlantHandle s1(1);
  PlantHandle s2(2);
  const marv_plant_body b = identity_body();
  const marv_plant_cmd cmd = make_cmd(900, 900, 900, 900);
  marv_plant_out o1 = make_out();
  marv_plant_out o2 = make_out();
  ASSERT_EQ(marv_plant_step(s1.get(), &b, &cmd, kDt, &o1), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_step(s2.get(), &b, &cmd, kDt, &o2), MARV_PLANT_OK);
  EXPECT_TRUE(same_bits(o1, o2));
}

// SIL validity rules (fw/sil/src/marv_sil.cpp:84-100, sample_valid), restated: all fields finite; no flag bit outside
// the nine defined; a clear valid bit means the field is exactly 0 and its saturation bits are clear; TempValid means
// temp_k > 0, otherwise temp_k == 0.
bool sil_valid(const marv_plant_imu_out& m) {
  const float all[7] = {m.gyro_rad_s.x, m.gyro_rad_s.y, m.gyro_rad_s.z, m.accel_m_s2.x,
                        m.accel_m_s2.y, m.accel_m_s2.z, m.temp_k};
  for (float f : all) {
    if (!std::isfinite(f)) {
      return false;
    }
  }
  const std::uint32_t defined = (std::uint32_t{1} << (MARV_PLANT_IMU_TEMP_VALID + 1)) - 1U;
  if ((m.flags & ~defined) != 0U) {
    return false;
  }
  const std::uint32_t gyro_sat = bit(MARV_PLANT_IMU_GYRO_SAT_X) | bit(MARV_PLANT_IMU_GYRO_SAT_Y) | bit(MARV_PLANT_IMU_GYRO_SAT_Z);
  const std::uint32_t accel_sat = bit(MARV_PLANT_IMU_ACCEL_SAT_X) | bit(MARV_PLANT_IMU_ACCEL_SAT_Y) | bit(MARV_PLANT_IMU_ACCEL_SAT_Z);
  if ((m.flags & bit(MARV_PLANT_IMU_GYRO_VALID)) == 0U &&
      (m.gyro_rad_s.x != 0.0F || m.gyro_rad_s.y != 0.0F || m.gyro_rad_s.z != 0.0F || (m.flags & gyro_sat) != 0U)) {
    return false;
  }
  if ((m.flags & bit(MARV_PLANT_IMU_ACCEL_VALID)) == 0U &&
      (m.accel_m_s2.x != 0.0F || m.accel_m_s2.y != 0.0F || m.accel_m_s2.z != 0.0F || (m.flags & accel_sat) != 0U)) {
    return false;
  }
  if ((m.flags & bit(MARV_PLANT_IMU_TEMP_VALID)) != 0U) {
    return m.temp_k > 0.0F;
  }
  return m.temp_k == 0.0F;
}

TEST(ImuNoise, SilValidityRuleControl) {
  // The restated rule rejects what it should (it is the control of the fuzz below).
  marv_plant_imu_out m{};
  m.flags = kValidBits;
  EXPECT_TRUE(sil_valid(m));
  m.temp_k = 1.0F;  // temp without TempValid
  EXPECT_FALSE(sil_valid(m));
  m.temp_k = 0.0F;
  m.flags = bit(MARV_PLANT_IMU_GYRO_SAT_X) | bit(MARV_PLANT_IMU_ACCEL_VALID);  // a saturation bit with gyro invalid
  EXPECT_FALSE(sil_valid(m));
  m.flags = kValidBits | bit(MARV_PLANT_IMU_TEMP_VALID + 1);  // a reserved bit
  EXPECT_FALSE(sil_valid(m));
  m.flags = kValidBits;
  m.gyro_rad_s.x = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(sil_valid(m));
}

TEST(ImuNoise, EveryOutputSatisfiesTheSilValidityRulesOverAFuzz) {
  // 2000 random (state, command, dt, config) draws from a fixed-seed Mersenne twister (labelled count and seed). Body
  // rates up to 20 rad/s (labelled: beyond a quad's, to exercise saturation at the small full scales below); unit
  // quaternions; DShot 0 or 48..2047; dt in [1e-4, 0.1] s; full scales that sometimes clamp. Each output must be valid
  // and, per axis, within its full scale.
  constexpr int kDraws = 2000;
  std::mt19937_64 rng(20261001);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::size_t saturated = 0;
  for (int d = 0; d < kDraws; ++d) {
    marv_plant_imu_config c = zero_noise_config(static_cast<std::uint32_t>(rng() % (kMaxLatency + 1)));
    c.gyro.noise_density = 1.0e-3 * std::fabs(gauss(rng));
    c.gyro.bias_instability = 1.0e-3 * std::fabs(gauss(rng));
    c.accel.noise_density = 1.0e-2 * std::fabs(gauss(rng));
    c.accel.bias_instability = 1.0e-2 * std::fabs(gauss(rng));
    c.gyro.full_scale = 10.0 + 10.0 * std::fabs(unit(rng));   // 10..20 rad/s: the 20 rad/s rates clamp sometimes
    c.accel.full_scale = 1.0 + 20.0 * std::fabs(unit(rng));   // 1..21 m/s^2: the 4 k w^2 / m up to ~30 clamps sometimes
    c.gyro.lsb = std::exp2(-std::floor(1.0 + 10.0 * std::fabs(unit(rng))));
    c.accel.lsb = std::exp2(-std::floor(1.0 + 10.0 * std::fabs(unit(rng))));
    for (int i = 0; i < 3; ++i) {
      c.gyro.turn_on_bias[i] = 0.1 * unit(rng);
      c.accel.turn_on_bias[i] = 0.5 * unit(rng);
    }
    PlantHandle plant(rng());
    ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
    for (int k = 0; k < 5; ++k) {  // labelled: a short run per draw so the delay line and walk are exercised
      marv_plant_body b = identity_body();
      double qn = 0.0;
      for (double& q : b.q_wxyz) {
        q = gauss(rng);
        qn += q * q;
      }
      for (double& q : b.q_wxyz) {
        q /= std::sqrt(qn);
      }
      for (double& w : b.omega_frd_rad_s) {
        w = 20.0 * unit(rng);
      }
      std::uint16_t dshot[MARV_PLANT_N_MOTORS];
      for (std::uint16_t& s : dshot) {
        s = (rng() % 8 == 0) ? std::uint16_t{0} : static_cast<std::uint16_t>(kDshotMin + rng() % (kDshotMax - kDshotMin + 1));
      }
      const marv_plant_cmd cmd = make_cmd(dshot[0], dshot[1], dshot[2], dshot[3]);
      const double dt = 1.0e-4 + 0.0999 * (0.5 + 0.5 * unit(rng));
      marv_plant_out po = make_out();
      ASSERT_EQ(marv_plant_step(plant.get(), &b, &cmd, dt, &po), MARV_PLANT_OK);
      const marv_plant_imu_out o = sample_ok(plant.get(), b, dt);
      ASSERT_TRUE(sil_valid(o)) << "draw " << d << " k " << k;
      ASSERT_EQ(o.flags & kValidBits, kValidBits);
      EXPECT_LE(std::fabs(static_cast<double>(o.gyro_rad_s.x)), c.gyro.full_scale);
      EXPECT_LE(std::fabs(static_cast<double>(o.gyro_rad_s.y)), c.gyro.full_scale);
      EXPECT_LE(std::fabs(static_cast<double>(o.gyro_rad_s.z)), c.gyro.full_scale);
      EXPECT_LE(std::fabs(static_cast<double>(o.accel_m_s2.x)), c.accel.full_scale);
      EXPECT_LE(std::fabs(static_cast<double>(o.accel_m_s2.y)), c.accel.full_scale);
      EXPECT_LE(std::fabs(static_cast<double>(o.accel_m_s2.z)), c.accel.full_scale);
      saturated += (o.flags & ~kValidBits) != 0U ? 1U : 0U;
    }
  }
  EXPECT_GT(saturated, 0U);  // control: the fuzz does reach the saturation flags
}

}  // namespace
