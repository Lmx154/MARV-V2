// L6 stage (b), the SIL entry marv_sil_tick_with_rotor_speed and the HAL pull read (decision 0013, owner decision 7), against
// the real SIL sources and a probe composition that records what hal_rotor_speed() returned in every tick (CMakeLists.txt).
// The entry accepts valid samples and refuses rule-breaking ones before any tick runs; after marv_sil_tick the HAL reports
// every motor invalid; after the new entry it reports exactly the bytes passed, stamped with the tick's time.
// marv_sil_init succeeds once per process, so every case runs in a forked child.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include <marv/composition.hpp>
#include <marv/params/param_ids.hpp>

namespace {

// design/scenario_values.yaml: tick_period_num_us 625, tick_period_den 4 (156.25 us).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
constexpr std::size_t kMotors = marv::composition::kMotors;
constexpr std::uint32_t kAllValid = (1U << kMotors) - 1;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

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

template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

marv_sil_config make_config() {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.param_schema_hash = marv::kParamSchemaHash;
  return c;
}

void reach_ready() {
  const marv_sil_config cfg = make_config();
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

std::uint64_t stamp_of(std::uint64_t tick) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(tick) * kPeriodNumUs / kPeriodDen);
}

// A valid IMU sample (the IMU rules are not under test): gyro and accel valid, temperature absent.
marv_imu_meas imu_sample() {
  marv_imu_meas m{};
  m.gyro_rad_s = {0.1F, -0.2F, 0.3F};
  m.accel_m_s2 = {0.0F, 0.0F, -9.0F};
  m.flags = (1U << MARV_IMU_GYRO_VALID) | (1U << MARV_IMU_ACCEL_VALID);
  return m;
}

marv_rotor_speed_meas rotor_sample(std::array<float, kMotors> omega, std::uint32_t flags) {
  marv_rotor_speed_meas m{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    m.omega_rad_s[i] = omega[i];
  }
  m.flags = flags;
  return m;
}

// Outputs for k ticks of one call.
struct Out {
  explicit Out(std::uint32_t k) : t_us(k), dshot(k * kMotors) {
    o.struct_size = sizeof(o);
    o.capacity_ticks = k;
    o.t_us = t_us.data();
    o.dshot = dshot.data();
    o.servo_us = nullptr;
  }
  std::vector<std::uint64_t> t_us;
  std::vector<std::uint16_t> dshot;
  marv_sil_out o{};
};

marv_sil_status tick_with(std::uint64_t first, const std::vector<marv_rotor_speed_meas>& rotor) {
  const auto k = static_cast<std::uint32_t>(rotor.size());
  const std::vector<marv_imu_meas> imu(k, imu_sample());
  Out out(k);
  return marv_sil_tick_with_rotor_speed(first, k, imu.data(), rotor.data(), &out.o);
}

marv_sil_status tick_plain(std::uint64_t first, std::uint32_t k) {
  const std::vector<marv_imu_meas> imu(k, imu_sample());
  Out out(k);
  return marv_sil_tick(first, k, imu.data(), &out.o);
}

bool same_bytes(const float* a, const float* b) { return std::memcmp(a, b, kMotors * sizeof(float)) == 0; }

// Whether recorded tick i carries exactly `want` and the stamp of tick number `tick`.
bool recorded_exactly(std::size_t i, const marv_rotor_speed_meas& want, std::uint64_t tick) {
  const marv::rotor_probe::Seen& s = marv::rotor_probe::at(i);
  return same_bytes(s.rotor.omega_rad_s.data(), want.omega_rad_s) && s.rotor.flags == want.flags &&
         s.rotor.t_us == stamp_of(tick) && s.imu_t_us == stamp_of(tick) && s.hal_time_us == stamp_of(tick);
}

TEST(RotorSpeedSil, TheHalReadReportsExactlyTheBytesPassedWithTheStampOfTheTick) {
  in_child([] {
    reach_ready();
    // Per tick distinct speeds and valid bits (all, motors 1 and 3, none), with the awkward floats: the smallest subnormal,
    // the largest finite, an ordinary fraction. A clear bit carries exactly 0.
    const std::vector<marv_rotor_speed_meas> first = {
        rotor_sample({1234.5F, 987.25F, std::numeric_limits<float>::max(), std::numeric_limits<float>::denorm_min()},
                     kAllValid),
        rotor_sample({2200.125F, 0.0F, 1.0F / 3.0F, 0.0F}, 0b0101U),
        rotor_sample({0.0F, 0.0F, 0.0F, 0.0F}, 0U)};
    ASSERT_EQ(tick_with(0, first), MARV_SIL_OK);
    ASSERT_EQ(marv::rotor_probe::count(), first.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
      EXPECT_TRUE(recorded_exactly(i, first[i], i)) << "tick " << i;
    }
    // A second call continues the numbering: its sample is the latest, with its own stamp (625 / 4 * 3 = 468.75 floors to 468).
    const std::vector<marv_rotor_speed_meas> second = {rotor_sample({10.0F, 20.0F, 30.0F, 40.0F}, kAllValid)};
    ASSERT_EQ(tick_with(3, second), MARV_SIL_OK);
    ASSERT_EQ(marv::rotor_probe::count(), first.size() + 1);
    EXPECT_TRUE(recorded_exactly(3, second[0], 3));

    // Control: the comparison is exact to one ulp and sensitive to the motor index and the tick.
    marv_rotor_speed_meas off = first[0];
    off.omega_rad_s[2] = std::nextafter(off.omega_rad_s[2], 0.0F);
    EXPECT_FALSE(recorded_exactly(0, off, 0));
    marv_rotor_speed_meas swapped = first[0];
    std::swap(swapped.omega_rad_s[0], swapped.omega_rad_s[1]);
    EXPECT_FALSE(recorded_exactly(0, swapped, 0));
    EXPECT_FALSE(recorded_exactly(1, first[1], 2));
  });
}

TEST(RotorSpeedSil, AfterThePlainTickEveryMotorIsInvalidAndNoByteSurvives) {
  in_child([] {
    reach_ready();
    ASSERT_EQ(tick_plain(0, 2), MARV_SIL_OK);  // the plain entry first
    const marv_rotor_speed_meas none = rotor_sample({0.0F, 0.0F, 0.0F, 0.0F}, 0U);
    ASSERT_EQ(tick_with(2, {rotor_sample({500.0F, 600.0F, 700.0F, 800.0F}, kAllValid)}), MARV_SIL_OK);  // then valid bytes
    ASSERT_EQ(tick_plain(3, 2), MARV_SIL_OK);  // then the plain entry again: the earlier bytes must not remain
    ASSERT_EQ(marv::rotor_probe::count(), 5U);
    for (const std::size_t i : {0U, 1U, 3U, 4U}) {
      EXPECT_TRUE(recorded_exactly(i, none, i)) << "tick " << i;
    }
    EXPECT_FALSE(recorded_exactly(2, none, 2));  // the one tick with bytes is not the invalid sample
    EXPECT_TRUE(recorded_exactly(2, rotor_sample({500.0F, 600.0F, 700.0F, 800.0F}, kAllValid), 2));
  });
}

TEST(RotorSpeedSil, RuleBreakingSamplesAreRefusedBeforeAnyTickRuns) {
  in_child([] {
    reach_ready();
    const marv_rotor_speed_meas good = rotor_sample({100.0F, 200.0F, 300.0F, 400.0F}, kAllValid);
    struct Case {
      const char* what;
      marv_rotor_speed_meas sample;
    };
    const std::vector<Case> bad = {
        {"NaN speed", rotor_sample({100.0F, kNan, 300.0F, 400.0F}, kAllValid)},
        {"+inf speed", rotor_sample({100.0F, 200.0F, kInf, 400.0F}, kAllValid)},
        {"-inf speed", rotor_sample({100.0F, 200.0F, 300.0F, -kInf}, kAllValid)},
        {"NaN under a clear bit", rotor_sample({kNan, 0.0F, 0.0F, 0.0F}, 0U)},
        {"negative valid speed", rotor_sample({100.0F, -1.0F, 300.0F, 400.0F}, kAllValid)},
        {"speed under a clear valid bit", rotor_sample({100.0F, 200.0F, 5.0F, 400.0F}, kAllValid & ~(1U << 2))},
        {"reserved bit 4", rotor_sample({100.0F, 200.0F, 300.0F, 400.0F}, kAllValid | (1U << kMotors))},
        {"reserved bit 31", rotor_sample({100.0F, 200.0F, 300.0F, 400.0F}, kAllValid | (1U << 31))},
    };
    for (const Case& c : bad) {
      // The bad sample is at index 1 of 3; the others are good. Nothing may run, and the error index is 1.
      EXPECT_EQ(tick_with(0, {good, c.sample, good}), MARV_SIL_E_INPUT) << c.what;
      EXPECT_EQ(marv_sil_error_index(), 1U) << c.what;
      EXPECT_EQ(marv::rotor_probe::count(), 0U) << c.what;
    }
    // The session is as it was: the same first_tick succeeds.
    EXPECT_EQ(tick_with(0, {good, good, good}), MARV_SIL_OK);
    EXPECT_EQ(marv::rotor_probe::count(), 3U);
  });
}

TEST(RotorSpeedSil, ArgumentAndStateErrors) {
  in_child([] {
    const marv_rotor_speed_meas good = rotor_sample({100.0F, 200.0F, 300.0F, 400.0F}, kAllValid);
    EXPECT_EQ(tick_with(0, {good}), MARV_SIL_E_STATE);  // before init
    reach_ready();
    const std::vector<marv_imu_meas> imu(1, imu_sample());
    Out out(1);
    EXPECT_EQ(marv_sil_tick_with_rotor_speed(0, 1, imu.data(), nullptr, &out.o), MARV_SIL_E_NULL);
    EXPECT_EQ(marv_sil_tick_with_rotor_speed(0, 1, nullptr, &good, &out.o), MARV_SIL_E_NULL);
    EXPECT_EQ(marv_sil_tick_with_rotor_speed(0, 1, imu.data(), &good, nullptr), MARV_SIL_E_NULL);
    EXPECT_EQ(tick_with(1, {good}), MARV_SIL_E_TICK);  // first_tick is not the number of ticks run
    EXPECT_EQ(marv_sil_tick_with_rotor_speed(0, 0, imu.data(), &good, &out.o), MARV_SIL_E_COUNT);
    EXPECT_EQ(marv_sil_tick_with_rotor_speed(0, 2, imu.data(), &good, &out.o), MARV_SIL_E_COUNT);  // capacity is 1
    EXPECT_EQ(marv::rotor_probe::count(), 0U);
    ASSERT_EQ(tick_with(0, {good}), MARV_SIL_OK);
    ASSERT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
    EXPECT_EQ(tick_with(1, {good}), MARV_SIL_E_STATE);  // after shutdown
  });
}

}  // namespace
