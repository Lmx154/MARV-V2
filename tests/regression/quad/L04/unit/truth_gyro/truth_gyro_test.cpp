// The truth-gyro command source (quad L4): TruthGyroSilCommandSource hands the SIL a marv_imu_meas whose gyro is the
// float cast of the truth body rate per axis, flags exactly GyroValid, accel and temperature zero with their valid
// bits clear, and that sample passes the SIL's boundary check (marv_sil_tick returns MARV_SIL_E_INPUT otherwise).
//
// Negative controls: a body rate one float ulp away on one axis fails the bitwise comparison (each axis, both
// directions); a sample with a nonzero gyro and GyroValid clear is rejected by the SIL, so the acceptance check is live.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "marv/gz/adapter.hpp"
#include "marv/gz/truth_gyro.hpp"
#include "marv/params/param_ids.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv_plant.h"

namespace {

// The SIL period: 625 / 4 us (scenario value, as the L2 adapter test).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;

using Omega = std::array<double, 3>;

// Scenario values: body rates in rad/s chosen so that the float cast rounds (not exactly representable in float), with
// mixed signs, a signed zero and magnitudes over several decades.
const std::array<Omega, 4> kRates = {{{0.1, -1.0 / 3.0, 2.718281828459045},
                                      {-123.456789012345, 0.7071067811865476, -1.0e-3},
                                      {0.0, -0.0, 1.0e-30},
                                      {17.000000001, -17.000000001, 3.141592653589793}}};

marv_plant_body body_with(const Omega& w) {
  marv_plant_body b{};
  b.struct_size = sizeof(b);
  for (std::size_t i = 0; i < 3; ++i) {
    b.omega_frd_rad_s[i] = w[i];
  }
  return b;
}

std::uint32_t bits(float f) { return std::bit_cast<std::uint32_t>(f); }

std::array<float, 3> gyro_of(const marv_imu_meas& m) { return {m.gyro_rad_s.x, m.gyro_rad_s.y, m.gyro_rad_s.z}; }

// True iff the sample's gyro is, bitwise per axis, the float cast of `w`.
bool gyro_is_cast_of(const marv_imu_meas& m, const Omega& w) {
  const std::array<float, 3> g = gyro_of(m);
  for (std::size_t i = 0; i < 3; ++i) {
    if (bits(g[i]) != bits(static_cast<float>(w[i]))) {
      return false;
    }
  }
  return true;
}

TEST(L4TruthGyro, GyroIsTheFloatCastOfTheBodyRateBitwisePerAxis) {
  for (const Omega& w : kRates) {
    marv::truth::TruthGyroSilCommandSource src;
    src.set_body(body_with(w));
    EXPECT_TRUE(gyro_is_cast_of(src.imu(), w));
  }
}

TEST(L4TruthGyro, FlagsAreExactlyGyroValidAndAccelAndTemperatureAreZeroWithValidBitsClear) {
  for (const Omega& w : kRates) {
    marv::truth::TruthGyroSilCommandSource src;
    src.set_body(body_with(w));
    const marv_imu_meas& m = src.imu();
    EXPECT_EQ(m.flags, marv::imu_flag(marv::ImuFlag::GyroValid));
    EXPECT_EQ(m.flags & marv::imu_flag(marv::ImuFlag::AccelValid), 0u);
    EXPECT_EQ(m.flags & marv::imu_flag(marv::ImuFlag::TempValid), 0u);
    EXPECT_EQ(bits(m.accel_m_s2.x), bits(0.0F));
    EXPECT_EQ(bits(m.accel_m_s2.y), bits(0.0F));
    EXPECT_EQ(bits(m.accel_m_s2.z), bits(0.0F));
    EXPECT_EQ(bits(m.temp_k), bits(0.0F));
  }
}

TEST(L4TruthGyro, TheSampleFollowsEachSetBody) {
  marv::truth::TruthGyroSilCommandSource src;
  for (const Omega& w : kRates) {
    src.set_body(body_with(w));
    EXPECT_TRUE(gyro_is_cast_of(src.imu(), w));
  }
  EXPECT_TRUE(gyro_is_cast_of(src.imu(), kRates.back()));
  EXPECT_FALSE(gyro_is_cast_of(src.imu(), kRates.front()));
}

TEST(L4TruthGyro, NegativeControlOneFloatUlpOnAnyAxisFailsTheBitwiseCheck) {
  for (const Omega& w : kRates) {
    marv::truth::TruthGyroSilCommandSource src;
    src.set_body(body_with(w));
    ASSERT_TRUE(gyro_is_cast_of(src.imu(), w));
    for (std::size_t axis = 0; axis < 3; ++axis) {
      for (const float toward : {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        Omega planted = w;
        planted[axis] = static_cast<double>(std::nextafter(static_cast<float>(w[axis]), toward));
        EXPECT_FALSE(gyro_is_cast_of(src.imu(), planted)) << "axis " << axis;
      }
    }
  }
}

bool report_failures_on_stderr() {
  const ::testing::TestResult* r = ::testing::UnitTest::GetInstance()->current_test_info()->result();
  bool any = false;
  for (int i = 0; i < r->total_part_count(); ++i) {
    const ::testing::TestPartResult& part = r->GetTestPartResult(i);
    if (part.failed()) {
      any = true;
      std::fprintf(stderr, "%s:%d: %s\n", part.file_name(), part.line_number(), part.message());
    }
  }
  return any;
}

// marv_sil_init succeeds once per process, so the case runs in a forked child (as the L2 adapter test).
template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(L4TruthGyro, TheSilAcceptsTheSampleAndRejectsAGyroWithoutItsValidBit) {
  in_child([] {
    marv_sil_config c{};
    c.struct_size = sizeof(marv_sil_config);
    c.imu_meas_size = sizeof(marv_imu_meas);
    c.override_size = sizeof(marv_sil_param_override);
    c.tick_period_num_us = kPeriodNumUs;
    c.tick_period_den = kPeriodDen;
    c.param_schema_hash = marv::kParamSchemaHash;
    ASSERT_EQ(marv_sil_init(&c), MARV_SIL_OK);

    marv::truth::TruthGyroSilCommandSource src;
    std::uint64_t tick = 0;
    for (const Omega& w : kRates) {
      src.set_body(body_with(w));
      for (std::uint32_t j = 0; j < 2; ++j) {
        marv::gz::Dshot d{};
        ASSERT_TRUE(src.dshot(tick, d));
        ASSERT_EQ(src.last_status(), MARV_SIL_OK);
        ++tick;
      }
    }

    marv_imu_meas bad = src.imu();
    bad.flags = 0;  // nonzero gyro with GyroValid clear
    std::uint64_t t_us = 0;
    std::array<std::uint16_t, MARV_PLANT_N_MOTORS> d{};
    marv_sil_out o{};
    o.struct_size = sizeof(o);
    o.capacity_ticks = 1;
    o.t_us = &t_us;
    o.dshot = d.data();
    EXPECT_EQ(marv_sil_tick(tick, 1, &bad, &o), MARV_SIL_E_INPUT);
  });
}

}  // namespace
