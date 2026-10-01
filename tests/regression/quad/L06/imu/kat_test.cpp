// L6 stage (a), zero-noise known-answer tests of the plant's IMU (decision 0012). With N = B = 0 every output is an
// exact function of the truth, the turn-on bias, the LSB and the clamps, so every check below is exact (EXPECT_EQ on
// floats): the LSBs are powers of two and the truths are multiples of them (support.hpp), so no rounding enters.
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "support.hpp"

namespace {

using namespace marv::plant::imu_test;

TEST(ImuKat, OutputOrderAndValue) {
  // Rotors at 2000 rad/s: thrust = 4 k w^2 = 4 * 2e-7 * 4e6 = 3.2 N, f_z = -3.2 / 0.75 = -4.26667 m/s^2 (FRD, up is
  // -z). Accel turn-on bias (0.5, -0.25, 0.125); z: y = -4.26667 + 0.125 = -4.14167 -> /0.0625 = -66.27 -> -66 counts
  // = -4.125 m/s^2. Gyro: the body rate (0.5, -1.25, 2.0), distinct per axis so a permutation shows.
  marv_plant_imu_config c = zero_noise_config();
  c.accel.turn_on_bias[0] = 0.5;
  c.accel.turn_on_bias[1] = -0.25;
  c.accel.turn_on_bias[2] = 0.125;
  PlantHandle plant(0, 2000.0);
  ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  const marv_plant_imu_out o = sample_ok(plant.get(), body_with_omega(0.5, -1.25, 2.0));
  EXPECT_EQ(o.gyro_rad_s.x, 0.5F);
  EXPECT_EQ(o.gyro_rad_s.y, -1.25F);
  EXPECT_EQ(o.gyro_rad_s.z, 2.0F);
  EXPECT_EQ(o.accel_m_s2.x, 0.5F);
  EXPECT_EQ(o.accel_m_s2.y, -0.25F);
  EXPECT_EQ(o.accel_m_s2.z, -4.125F);
  EXPECT_EQ(o.temp_k, 0.0F);
  EXPECT_EQ(o.flags, kValidBits);
}

TEST(ImuKat, SpecificForceIsThrustOverMassAlongMinusZ) {
  // After a step the rotor speeds are the step's; f_z = -(sum k w_i^2) / m, restated here from the step output. The
  // four rotors at different speeds, so a wrong sum or a wrong rotor shows. Accel LSB 2^-4: expected count is the
  // rounded quotient, restated independently.
  PlantHandle plant;
  const marv_plant_imu_config c = zero_noise_config();
  ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  const marv_plant_body b = identity_body();
  const marv_plant_cmd cmd = make_cmd(1000, 1200, 1400, 1600);
  marv_plant_out po = make_out();
  ASSERT_EQ(marv_plant_step(plant.get(), &b, &cmd, kDt, &po), MARV_PLANT_OK);
  double thrust = 0.0;
  for (double w : po.rotor_speed_rad_s) {
    thrust += kThrustCoeff * w * w;
  }
  const double expected_counts = std::round((-thrust / kMass) / kAccelLsb);
  const marv_plant_imu_out o = sample_ok(plant.get(), b);
  EXPECT_EQ(o.accel_m_s2.x, 0.0F);
  EXPECT_EQ(o.accel_m_s2.y, 0.0F);
  EXPECT_EQ(o.accel_m_s2.z, static_cast<float>(expected_counts * kAccelLsb));
  EXPECT_LT(o.accel_m_s2.z, -0.5F);  // control: thrust is non-trivial (at least 8 counts of 2^-4, not just 0)
}

TEST(ImuKat, FreeFallAccelIsExactlyZero) {
  // Zero thrust (rotors at rest, DShot 0 commands): the specific force is 0, with a positive sign bit clear, whatever
  // the body rate; gravity is never subtracted.
  PlantHandle plant;
  const marv_plant_imu_config c = zero_noise_config();
  ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  const marv_plant_body b = body_with_omega(1.0, -2.0, 3.0);
  const marv_plant_cmd stop = make_cmd(0, 0, 0, 0);
  marv_plant_out po = make_out();
  for (int k = 0; k < 4; ++k) {  // labelled: a few steps, each followed by a sample
    ASSERT_EQ(marv_plant_step(plant.get(), &b, &stop, kDt, &po), MARV_PLANT_OK);
    const marv_plant_imu_out o = sample_ok(plant.get(), b);
    EXPECT_EQ(o.accel_m_s2.x, 0.0F);
    EXPECT_EQ(o.accel_m_s2.y, 0.0F);
    EXPECT_EQ(o.accel_m_s2.z, 0.0F);
    EXPECT_FALSE(std::signbit(o.accel_m_s2.z));
    EXPECT_EQ(o.gyro_rad_s.y, -2.0F);
  }
}

struct TieCase {
  double counts_in;   // the bias, in LSB
  double counts_out;  // the expected integer count
};

TEST(ImuKat, RoundingTiesGoAwayFromZero) {
  // y = bias (truth 0). Half-way values must go away from zero (not to even): 0.5 -> 1 (even rule gives 0), 2.5 -> 3
  // (even rule gives 2); the doubles just inside the half-way point go to the nearer count.
  const double below = std::nextafter(0.5, 0.0);  // 0.49999999999999994, the largest double below 1/2
  const double above = std::nextafter(0.5, 1.0);  // the smallest double above 1/2
  const TieCase cases[] = {{0.5, 1.0},  {-0.5, -1.0}, {1.5, 2.0},  {-1.5, -2.0}, {2.5, 3.0},
                           {-2.5, -3.0}, {below, 0.0}, {-below, 0.0}, {above, 1.0}, {-above, -1.0}};
  for (const TieCase& t : cases) {
    marv_plant_imu_config c = zero_noise_config();
    for (int i = 0; i < 3; ++i) {
      c.gyro.turn_on_bias[i] = t.counts_in * kGyroLsb;
      c.accel.turn_on_bias[i] = t.counts_in * kAccelLsb;
    }
    const marv_plant_imu_out o = one_sample(c, identity_body());
    const float g = static_cast<float>(t.counts_out * kGyroLsb);
    const float a = static_cast<float>(t.counts_out * kAccelLsb);
    EXPECT_EQ(o.gyro_rad_s.x, g) << t.counts_in;
    EXPECT_EQ(o.gyro_rad_s.y, g) << t.counts_in;
    EXPECT_EQ(o.gyro_rad_s.z, g) << t.counts_in;
    EXPECT_EQ(o.accel_m_s2.x, a) << t.counts_in;
    EXPECT_EQ(o.accel_m_s2.y, a) << t.counts_in;
    EXPECT_EQ(o.accel_m_s2.z, a) << t.counts_in;
    if (t.counts_out == 0.0) {
      EXPECT_FALSE(std::signbit(o.gyro_rad_s.x));  // a zero count is +0, never -0
      EXPECT_FALSE(std::signbit(o.accel_m_s2.x));
    }
    EXPECT_EQ(o.flags, kValidBits);
  }
}

TEST(ImuKat, FullScaleClampAndPerComponentFlags) {
  // Full scale 100 at LSB 0.125 is 800 counts. x +150 and y -150 clamp (flags X, Y only); z = +100 is exactly the
  // limit: count 800 is not clamped, so no flag. Accel mirrors it with y exactly -100 (count -1600, the low limit,
  // not clamped) and x, z beyond.
  marv_plant_imu_config c = zero_noise_config();
  c.accel.turn_on_bias[0] = 150.0;
  c.accel.turn_on_bias[1] = -100.0;
  c.accel.turn_on_bias[2] = -150.0;
  const marv_plant_imu_out o = one_sample(c, body_with_omega(150.0, -150.0, 100.0));
  EXPECT_EQ(o.gyro_rad_s.x, 100.0F);
  EXPECT_EQ(o.gyro_rad_s.y, -100.0F);
  EXPECT_EQ(o.gyro_rad_s.z, 100.0F);
  EXPECT_EQ(o.accel_m_s2.x, 100.0F);
  EXPECT_EQ(o.accel_m_s2.y, -100.0F);
  EXPECT_EQ(o.accel_m_s2.z, -100.0F);
  EXPECT_EQ(o.flags, kValidBits | bit(MARV_PLANT_IMU_GYRO_SAT_X) | bit(MARV_PLANT_IMU_GYRO_SAT_Y) |
                         bit(MARV_PLANT_IMU_ACCEL_SAT_X) | bit(MARV_PLANT_IMU_ACCEL_SAT_Z));
}

TEST(ImuKat, FullScaleBetweenCountsClampsToTheCountBelow) {
  // Full scale 100.1 at LSB 0.125 is 800.8 counts: the limit is 800 counts (100 rad/s), so the output never exceeds
  // the full scale.
  marv_plant_imu_config c = zero_noise_config();
  c.gyro.full_scale = 100.1;
  const marv_plant_imu_out o = one_sample(c, body_with_omega(100.0625, -100.0625, 100.0));
  // 100.0625 / 0.125 = 800.5 -> count 801 > 800: clamped. 100.0 -> 800: not.
  EXPECT_EQ(o.gyro_rad_s.x, 100.0F);
  EXPECT_EQ(o.gyro_rad_s.y, -100.0F);
  EXPECT_EQ(o.gyro_rad_s.z, 100.0F);
  EXPECT_EQ(o.flags, kValidBits | bit(MARV_PLANT_IMU_GYRO_SAT_X) | bit(MARV_PLANT_IMU_GYRO_SAT_Y));
}

TEST(ImuKat, TwentyBitWordClampWhenTighterThanFullScale) {
  // Full scale 1e6 is far beyond 2^19 counts, so the word binds: counts in [-524288, 524287]. At LSB 0.125 that is
  // [-65536, 65535.875] rad/s. x = 1e5 and y = -1e5 clamp (flags); z = 65535.875 is the top count, not clamped.
  // The accel side (LSB 0.0625: [-32768, 32767.9375]): x = 32768 is count 524288, clamped; y = -32768 is the lowest
  // count, not clamped; z = 32767.9375 is the top count, not clamped.
  marv_plant_imu_config c = zero_noise_config();
  c.gyro.full_scale = kHugeFullScale;
  c.accel.full_scale = kHugeFullScale;
  c.accel.turn_on_bias[0] = 32768.0;
  c.accel.turn_on_bias[1] = -32768.0;
  c.accel.turn_on_bias[2] = 32767.9375;
  const marv_plant_imu_out o = one_sample(c, body_with_omega(1.0e5, -1.0e5, 65535.875));
  EXPECT_EQ(o.gyro_rad_s.x, 65535.875F);
  EXPECT_EQ(o.gyro_rad_s.y, -65536.0F);
  EXPECT_EQ(o.gyro_rad_s.z, 65535.875F);
  EXPECT_EQ(o.accel_m_s2.x, 32767.9375F);
  EXPECT_EQ(o.accel_m_s2.y, -32768.0F);
  EXPECT_EQ(o.accel_m_s2.z, 32767.9375F);
  EXPECT_EQ(o.flags, kValidBits | bit(MARV_PLANT_IMU_GYRO_SAT_X) | bit(MARV_PLANT_IMU_GYRO_SAT_Y) |
                         bit(MARV_PLANT_IMU_ACCEL_SAT_X));
}

TEST(ImuKat, WordLowEndClamps) {
  // y = -65536.125 -> -524289 counts, clamps to -524288 (-65536 rad/s) with the flag.
  marv_plant_imu_config c = zero_noise_config();
  c.gyro.full_scale = kHugeFullScale;
  const marv_plant_imu_out o = one_sample(c, body_with_omega(-65536.125, 0.0, 0.0));
  EXPECT_EQ(o.gyro_rad_s.x, -65536.0F);
  EXPECT_EQ(o.flags, kValidBits | bit(MARV_PLANT_IMU_GYRO_SAT_X));
}

// Delay. Truth of sample j (gyro): x = j + 1, y = (j + 1) / 2, z = -(j + 1); all multiples of the LSB, within 100.
std::array<double, 3> ramp_truth(std::uint32_t j) {
  const double s = static_cast<double>(j) + 1.0;
  return {s, 0.5 * s, -s};
}

std::vector<marv_plant_imu_out> run_ramp(std::uint32_t latency, std::uint32_t samples) {
  PlantHandle plant;
  const marv_plant_imu_config c = zero_noise_config(latency);
  EXPECT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  std::vector<marv_plant_imu_out> out;
  for (std::uint32_t k = 0; k < samples; ++k) {
    const std::array<double, 3> t = ramp_truth(k);
    out.push_back(sample_ok(plant.get(), body_with_omega(t[0], t[1], t[2])));
  }
  return out;
}

TEST(ImuKat, DelayOfLSamplesAndThePreStartRule) {
  // Sample k measures the truth of sample k - L; samples before the start measure the truth of sample 0. L = 0, 1, 3
  // (architect's set) and the maximum, 64 (the delay line's full length).
  const std::uint32_t samples = kMaxLatency + 6;  // labelled: run past the first wrap of the delay line
  for (std::uint32_t latency : {0U, 1U, 3U, kMaxLatency}) {
    const std::vector<marv_plant_imu_out> out = run_ramp(latency, samples);
    for (std::uint32_t k = 0; k < samples; ++k) {
      const std::array<double, 3> t = ramp_truth(k >= latency ? k - latency : 0U);
      EXPECT_EQ(out[k].gyro_rad_s.x, static_cast<float>(t[0])) << "L " << latency << " k " << k;
      EXPECT_EQ(out[k].gyro_rad_s.y, static_cast<float>(t[1])) << "L " << latency << " k " << k;
      EXPECT_EQ(out[k].gyro_rad_s.z, static_cast<float>(t[2])) << "L " << latency << " k " << k;
    }
    // Control: the delay is observable, so a run with L + 1 differs from this one at sample L + 1.
    if (latency < kMaxLatency) {
      const std::vector<marv_plant_imu_out> other = run_ramp(latency + 1, samples);
      EXPECT_FALSE(same_bits(out[latency + 1], other[latency + 1])) << "L " << latency;
    }
  }
}

TEST(ImuKat, ImpulseAppearsLSamplesLate) {
  // Impulse of 8 rad/s on x at sample 2, L = 3: only sample 5 sees it. An impulse at sample 0 shows the pre-start
  // rule: samples 0..L all see it (sample k < L uses the truth of sample 0, sample L uses sample 0 itself).
  const std::uint32_t latency = 3;
  const double impulse = 8.0;
  const std::uint32_t samples = 10;  // labelled
  for (std::uint32_t at : {2U, 0U}) {
    PlantHandle plant;
    const marv_plant_imu_config c = zero_noise_config(latency);
    ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
    for (std::uint32_t k = 0; k < samples; ++k) {
      const marv_plant_imu_out o = sample_ok(plant.get(), body_with_omega(k == at ? impulse : 0.0, 0.0, 0.0));
      const bool sees = at == 0 ? k <= latency : k == at + latency;
      EXPECT_EQ(o.gyro_rad_s.x, sees ? static_cast<float>(impulse) : 0.0F) << "at " << at << " k " << k;
    }
  }
}

TEST(ImuKat, AccelIsDelayedLikeTheGyro) {
  // The accel truth of a sample is the thrust after the step before it; with L = 3 sample k reports the thrust after
  // step max(k - 3, 0). DShot alternates between two values so successive thrusts differ by many counts.
  const std::uint32_t latency = 3;
  const std::uint32_t samples = 12;  // labelled
  PlantHandle plant;
  const marv_plant_imu_config c = zero_noise_config(latency);
  ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  const marv_plant_body b = identity_body();
  std::vector<double> thrust;
  std::vector<marv_plant_imu_out> out;
  for (std::uint32_t k = 0; k < samples; ++k) {
    const std::uint16_t d = (k % 2 == 0) ? 1800 : 600;
    const marv_plant_cmd cmd = make_cmd(d, d, d, d);
    marv_plant_out po = make_out();
    ASSERT_EQ(marv_plant_step(plant.get(), &b, &cmd, kDt, &po), MARV_PLANT_OK);
    double t = 0.0;
    for (double w : po.rotor_speed_rad_s) {
      t += kThrustCoeff * w * w;
    }
    thrust.push_back(t);
    out.push_back(sample_ok(plant.get(), b));
  }
  for (std::uint32_t k = 0; k < samples; ++k) {
    const double f = -thrust[k >= latency ? k - latency : 0U] / kMass;
    EXPECT_EQ(out[k].accel_m_s2.z, static_cast<float>(std::round(f / kAccelLsb) * kAccelLsb)) << k;
  }
  EXPECT_NE(out[4].accel_m_s2.z, out[5].accel_m_s2.z);  // control: the sequence is not constant
}

TEST(ImuKat, TurnOnBiasAppearsExactlyAndStaysWithoutNoise) {
  // N = 0: K = 0 and sigma_d = 0 whatever B is (the rule for the undefined B^2 / N), so the bias is the turn-on bias
  // at every sample, whatever dt. Turn-on gyro (0.25, -0.5, 0.75), accel (-0.125, 0.375, 1.0).
  for (double instability : {0.0, 0.01}) {  // labelled: B = 0 and a B > 0 that the N = 0 rule must ignore
    marv_plant_imu_config c = zero_noise_config();
    c.gyro.bias_instability = instability;
    c.accel.bias_instability = instability;
    const double g[3] = {0.25, -0.5, 0.75};
    const double a[3] = {-0.125, 0.375, 1.0};
    for (int i = 0; i < 3; ++i) {
      c.gyro.turn_on_bias[i] = g[i];
      c.accel.turn_on_bias[i] = a[i];
    }
    PlantHandle plant;
    ASSERT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
    for (double dt : {0.001, 0.0123, 0.5, 0.01, 0.01}) {  // labelled: dt varies, the bias does not drift
      const marv_plant_imu_out o = sample_ok(plant.get(), identity_body(), dt);
      EXPECT_EQ(o.gyro_rad_s.x, 0.25F);
      EXPECT_EQ(o.gyro_rad_s.y, -0.5F);
      EXPECT_EQ(o.gyro_rad_s.z, 0.75F);
      EXPECT_EQ(o.accel_m_s2.x, -0.125F);
      EXPECT_EQ(o.accel_m_s2.y, 0.375F);
      EXPECT_EQ(o.accel_m_s2.z, 1.0F);
    }
  }
}

}  // namespace
