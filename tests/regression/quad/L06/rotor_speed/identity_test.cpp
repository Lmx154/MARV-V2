// L6 stage (b), the rotor-speed model does not change the plant: marv_plant_step outputs and the IMU bytes are bit-identical
// with and without it (attached and sampled every tick, at several delays); and the delay is respected: sample k with a
// delay of L ticks is sample k - L of the same run without a delay (sample 0 before the start). Controls: a delay off by
// one tick does not match; a one-ulp change of the body changes the IMU bytes the identity compares.
#include <cmath>

#include "support.hpp"

namespace {

using namespace marv::plant::rotor_test;

constexpr std::size_t kTicks = 24;  // labelled: longer than the largest delay under test (4) by a wide margin
constexpr std::uint64_t kSeed = 11;  // labelled: any fixed seed; the IMU draws from it

marv_plant_cmd ramp_cmd(std::size_t k) {
  // Legal DShot (48..2047), different per motor and tick: 48 + (97 k + 211 i) mod 1900 (97 and 211 are primes coprime to
  // 1900 = 2^2 5^2 19), so every rotor speed changes every tick.
  marv_plant_cmd c{};
  c.struct_size = sizeof(c);
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    c.dshot[i] = static_cast<std::uint16_t>(kDshotMin + ((97 * k + 211 * i) % 1900));
  }
  return c;
}

// Noise on in every axis (labelled test values, not sensor figures), LSB 2^-10 and full scale 100 so the word does not bind.
marv_plant_imu_config imu_config() {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.latency_samples = 1;
  c.gyro.noise_density = 1.0e-3;
  c.gyro.bias_instability = 2.0e-3;
  c.gyro.lsb = 0x1p-10;
  c.gyro.full_scale = 100.0;
  c.accel.noise_density = 1.0e-2;
  c.accel.bias_instability = 2.0e-2;
  c.accel.lsb = 0x1p-10;
  c.accel.full_scale = 100.0;
  return c;
}

// A slow ramp on the roll rate, 0.1 + 0.01 k rad/s (labelled test values), and constant pitch and yaw rates.
double ramp_roll(std::size_t k) { return 0.1 + 0.01 * static_cast<double>(k); }

marv_plant_body body_of(double roll) {
  marv_plant_body b = identity_body();
  b.omega_frd_rad_s[0] = roll;
  b.omega_frd_rad_s[1] = -0.2;
  b.omega_frd_rad_s[2] = 0.05;
  return b;
}

// A zero-noise IMU with a gyro step of 2^-3 rad/s: a roll rate of exactly half a step, 2^-4, is a rounding tie (count 1,
// half away from zero), and one ulp below it is count 0 (the rule of imu_model.hpp step 5).
marv_plant_imu_config tie_imu_config() {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.latency_samples = 1;
  c.gyro.lsb = 0x1p-3;
  c.gyro.full_scale = 100.0;
  c.accel.lsb = 0x1p-10;
  c.accel.full_scale = 100.0;
  return c;
}

struct Trace {
  std::vector<marv_plant_out> out;
  std::vector<marv_plant_imu_out> imu;
  std::vector<marv_plant_rotor_speed_out> rotor;
};

// IMU sample, rotor sample (when rotor_cfg is set), step: the adapter's order. roll(k) is the body's roll rate at tick k.
Trace trace(const marv_plant_rotor_speed_config* rotor_cfg, const marv_plant_imu_config& imu = imu_config(),
        double (*roll)(std::size_t) = ramp_roll) {
  PlantHandle plant({}, kSeed);
  marv_plant* p = plant.get();
  EXPECT_EQ(marv_plant_imu_attach(p, &imu), MARV_PLANT_OK);
  if (rotor_cfg != nullptr) {
    EXPECT_EQ(marv_plant_rotor_speed_attach(p, rotor_cfg), MARV_PLANT_OK);
  }
  Trace r;
  for (std::size_t k = 0; k < kTicks; ++k) {
    const marv_plant_body body = body_of(roll(k));
    marv_plant_imu_out i{};
    EXPECT_EQ(marv_plant_imu_sample(p, &body, kSubstep, &i), MARV_PLANT_OK);
    r.imu.push_back(i);
    if (rotor_cfg != nullptr) {
      r.rotor.push_back(sample_ok(p));
    }
    const marv_plant_cmd cmd = ramp_cmd(k);
    marv_plant_out out{};
    out.struct_size = sizeof(out);
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, kSubstep, &out), MARV_PLANT_OK);
    r.out.push_back(out);
  }
  return r;
}

std::size_t differing_steps(const Trace& a, const Trace& b) {
  std::size_t n = 0;
  for (std::size_t k = 0; k < kTicks; ++k) {
    n += same_bits(a.out[k], b.out[k]) ? 0U : 1U;
  }
  return n;
}

std::size_t differing_imu(const Trace& a, const Trace& b) {
  std::size_t n = 0;
  for (std::size_t k = 0; k < kTicks; ++k) {
    n += std::memcmp(&a.imu[k], &b.imu[k], sizeof(a.imu[k])) == 0 ? 0U : 1U;
  }
  return n;
}

TEST(RotorSpeedIdentity, StepOutputsAndImuBytesAreBitIdenticalWithAndWithoutTheModel) {
  const Trace without = trace(nullptr);
  for (const std::uint32_t latency : {0U, 1U, 2U, 4U}) {
    const marv_plant_rotor_speed_config cfg = grid_config(latency);
    const Trace with = trace(&cfg);
    EXPECT_EQ(differing_steps(without, with), 0U) << "latency " << latency;
    EXPECT_EQ(differing_imu(without, with), 0U) << "latency " << latency;
  }
}

TEST(RotorSpeedIdentity, ControlAOneUlpBodyChangeChangesTheImuBytesTheIdentityCompares) {
  const marv_plant_imu_config tie = tie_imu_config();
  const Trace at_tie = trace(nullptr, tie, [](std::size_t) { return 0x1p-4; });
  const Trace below = trace(nullptr, tie, [](std::size_t) { return std::nextafter(0x1p-4, 0.0); });
  EXPECT_EQ(differing_imu(at_tie, below), kTicks);
  // The step outputs do not read the body rates (v0): they are unchanged, which is why the IMU bytes carry the check.
  EXPECT_EQ(differing_steps(at_tie, below), 0U);
  // And the identity holds on that config too, with the rotor model on.
  const marv_plant_rotor_speed_config cfg = grid_config(1);
  const Trace with = trace(&cfg, tie, [](std::size_t) { return 0x1p-4; });
  EXPECT_EQ(differing_imu(at_tie, with), 0U);
}

TEST(RotorSpeedLatency, SampleKWithDelayLIsSampleKMinusLWithoutDelay) {
  const marv_plant_rotor_speed_config none = grid_config(0);
  const Trace base = trace(&none);
  // Non-vacuous: consecutive undelayed samples differ (the rotors move every tick), so a shift is seen.
  for (std::size_t k = 1; k < kTicks; ++k) {
    EXPECT_FALSE(same_bits(base.rotor[k], base.rotor[k - 1])) << "tick " << k;
  }
  for (const std::uint32_t latency : {1U, 2U, 4U, 64U}) {
    const marv_plant_rotor_speed_config cfg = grid_config(latency);
    const Trace delayed = trace(&cfg);
    std::size_t exact = 0;
    std::size_t one_early = 0;  // control: a delay one tick shorter
    std::size_t one_late = 0;   // control: a delay one tick longer
    for (std::size_t k = 0; k < kTicks; ++k) {
      const auto src = [&](std::size_t back) { return k >= back ? k - back : std::size_t{0}; };
      exact += same_bits(delayed.rotor[k], base.rotor[src(latency)]) ? 0U : 1U;
      one_early += same_bits(delayed.rotor[k], base.rotor[src(latency - 1)]) ? 0U : 1U;
      one_late += same_bits(delayed.rotor[k], base.rotor[src(latency + 1)]) ? 0U : 1U;
    }
    EXPECT_EQ(exact, 0U) << "latency " << latency;
    // The controls differ at every tick that has data beyond the first sample, so they must not match everywhere.
    if (latency < kTicks) {
      EXPECT_GT(one_early, 0U) << "latency " << latency;
      EXPECT_GT(one_late, 0U) << "latency " << latency;
    }
  }
}

TEST(RotorSpeedFreshness, SampleOfTickJIsTheRotorStateAfterStepJMinusOne) {
  const marv_plant_rotor_speed_config cfg = grid_config(0);
  PlantHandle plant({400.0, 600.0, 800.0, 1000.0});
  ASSERT_EQ(marv_plant_rotor_speed_attach(plant.get(), &cfg), MARV_PLANT_OK);
  std::array<double, MARV_PLANT_N_MOTORS> speed = {400.0, 600.0, 800.0, 1000.0};  // the initial rotor speeds
  const marv_plant_body body = identity_body();
  for (std::size_t k = 0; k < 8; ++k) {  // labelled: a few ticks are enough to see a one-tick offset
    const marv_plant_rotor_speed_out o = sample_ok(plant.get());
    std::array<Reading, MARV_PLANT_N_MOTORS> want{};
    for (std::size_t i = 0; i < want.size(); ++i) {
      want[i] = oracle(speed[i], kPoleCount);
    }
    EXPECT_TRUE(matches(o, want)) << "tick " << k;
    const marv_plant_cmd cmd = ramp_cmd(k);
    marv_plant_out out{};
    out.struct_size = sizeof(out);
    ASSERT_EQ(marv_plant_step(plant.get(), &body, &cmd, kSubstep, &out), MARV_PLANT_OK);
    for (std::size_t i = 0; i < speed.size(); ++i) {
      speed[i] = out.rotor_speed_rad_s[i];  // what the next sample must have measured
    }
  }
}

}  // namespace
