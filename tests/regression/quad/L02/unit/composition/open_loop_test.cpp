// The l2_open_loop composition through the SIL C ABI (marv_sil.h): the per-motor DShot comes from ol_dshot_m1..m4 set
// by override, in the logical order; the default is stop; an illegal value panics at init, naming the motor. In the
// simulator hal_panic prints "hal_panic: <reason>" on stderr and calls std::abort, so a rejected value is a SIGABRT
// death with that message.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>

#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/actuator.hpp>

namespace {

constexpr std::size_t kMotors = marv::kQuadXMotors;
constexpr std::uint16_t kSentinel16 = 0xAAAA;
constexpr std::uint64_t kSentinel64 = 0xAAAAAAAAAAAAAAAAull;
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
constexpr std::uint32_t kAllValid =
    (1u << MARV_IMU_GYRO_VALID) | (1u << MARV_IMU_ACCEL_VALID) | (1u << MARV_IMU_TEMP_VALID);

using Command = std::array<std::int32_t, kMotors>;

const std::array<marv::ParamId, kMotors> kIds{marv::ParamId::ol_dshot_m1, marv::ParamId::ol_dshot_m2,
                                              marv::ParamId::ol_dshot_m3, marv::ParamId::ol_dshot_m4};

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

std::array<marv_sil_param_override, kMotors> overrides_for(const Command& c) {
  std::array<marv_sil_param_override, kMotors> ov{};
  for (std::size_t m = 0; m < kMotors; ++m) {
    ov[m].id = static_cast<std::uint32_t>(kIds[m]);
    ov[m].type = MARV_PARAM_I32;
    ov[m].i32 = c[m];
  }
  return ov;
}

marv_sil_status init_with(const marv_sil_param_override* ov, std::uint32_t n) {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.n_overrides = n;
  c.overrides = ov;
  c.param_schema_hash = marv::kParamSchemaHash;
  return marv_sil_init(&c);
}

marv_sil_status init_command(const Command& cmd) {
  const auto ov = overrides_for(cmd);
  return init_with(ov.data(), static_cast<std::uint32_t>(ov.size()));
}

// Even i: a valid sample that differs for every i. Odd i: zeroed and flagged invalid, as the Gazebo plugin passes it.
// The composition must ignore the content of both.
marv_imu_meas sample(std::uint32_t i) {
  marv_imu_meas m{};
  if ((i % 2) != 0) {
    return m;
  }
  const auto f = static_cast<float>(i);
  m.gyro_rad_s = {0.01f * f, -0.02f * f, 0.5f + f};
  m.accel_m_s2 = {-9.81f + f, 0.25f * f, 1.5f};
  m.temp_k = 300.0f + f;
  m.flags = kAllValid;
  return m;
}

std::uint64_t stamp(std::uint64_t n) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(n) * kPeriodNumUs / kPeriodDen);
}

// Ticks [first, first + k) and checks every row: stamp, and per motor the raw DShot equal to `want`.
void run_and_expect(std::uint64_t first, std::uint32_t k, const Command& want) {
  std::vector<marv_imu_meas> s;
  for (std::uint32_t i = 0; i < k; ++i) {
    s.push_back(sample(static_cast<std::uint32_t>(first) + i));
  }
  std::vector<std::uint64_t> t(k, kSentinel64);
  std::vector<std::uint16_t> dshot(static_cast<std::size_t>(k) * kMotors, kSentinel16);
  marv_sil_out out{};
  out.struct_size = sizeof(marv_sil_out);
  out.capacity_ticks = k;
  out.t_us = t.data();
  out.dshot = dshot.data();
  out.servo_us = nullptr;
  ASSERT_EQ(marv_sil_tick(first, k, s.data(), &out), MARV_SIL_OK);
  for (std::uint32_t i = 0; i < k; ++i) {
    EXPECT_EQ(t[i], stamp(first + i)) << "tick " << (first + i);
    for (std::size_t m = 0; m < kMotors; ++m) {
      EXPECT_EQ(dshot[(static_cast<std::size_t>(i) * kMotors) + m], static_cast<std::uint16_t>(want[m]))
          << "tick " << (first + i) << " logical motor " << (m + 1);
    }
  }
}

TEST(L2OpenLoopInfo, ReportsTheCompositionAndFourMotorsNoServos) {
  marv_sil_info info{};
  info.struct_size = sizeof(info);
  ASSERT_EQ(marv_sil_info_get(&info), MARV_SIL_OK);
  EXPECT_EQ(info.n_motors, kMotors);
  EXPECT_EQ(info.n_servos, 0u);
  EXPECT_STREQ(info.composition, "l2_open_loop");
  EXPECT_EQ(info.n_params, kMotors);
}

TEST(L2OpenLoop, OverriddenValuesReachTheirLogicalMotorsWithExactStamps) {
  in_child([] {
    // Distinct per motor, so a swapped mapping shows; the range ends are included.
    const Command want{marv::prim::kDshotThrottleMin, marv::prim::kDshotThrottleMax, 1000, 0};
    ASSERT_EQ(init_command(want), MARV_SIL_OK);
    constexpr std::uint32_t kFirst = 5;
    run_and_expect(0, kFirst, want);
    run_and_expect(kFirst, 2 * kFirst, want);
  });
}

TEST(L2OpenLoop, TheMappingIsPerMotorNotPositional) {
  in_child([] {
    const Command want{0, 300, 0, 0};
    ASSERT_EQ(init_command(want), MARV_SIL_OK);
    run_and_expect(0, 3, want);
  });
}

TEST(L2OpenLoop, WithoutOverridesEveryMotorIsStop) {
  in_child([] {
    ASSERT_EQ(init_with(nullptr, 0), MARV_SIL_OK);
    run_and_expect(0, 4, Command{});
  });
}

TEST(L2OpenLoop, AnOverrideOfOneMotorLeavesTheOthersAtStop) {
  in_child([] {
    marv_sil_param_override ov{};
    ov.id = static_cast<std::uint32_t>(marv::ParamId::ol_dshot_m3);
    ov.type = MARV_PARAM_I32;
    ov.i32 = 777;
    ASSERT_EQ(init_with(&ov, 1), MARV_SIL_OK);
    run_and_expect(0, 3, Command{0, 0, 777, 0});
  });
}

// ---- illegal values -----------------------------------------------------------------------------------------------

void expect_init_panics(std::size_t motor_index, std::int32_t value) {
  Command cmd{};
  cmd[motor_index] = value;
  const std::string want = "hal_panic: l2_open_loop composition: ol_dshot_m" + std::to_string(motor_index + 1) +
                           " \\(motor " + std::to_string(motor_index + 1) + "\\)";
  EXPECT_EXIT({ (void)init_command(cmd); std::exit(0); }, ::testing::KilledBySignal(SIGABRT), want)
      << "value " << value << " on motor " << (motor_index + 1);
}

TEST(L2OpenLoopIllegal, TheFirstReservedDshotValueIsRejectedNamingTheMotor) {
  for (std::size_t m = 0; m < kMotors; ++m) {
    expect_init_panics(m, 1);
  }
}

TEST(L2OpenLoopIllegal, TheReservedRangeEdgesAndAboveTheThrottleRangeAreRejected) {
  expect_init_panics(2, marv::prim::kDshotThrottleMin - 1);
  expect_init_panics(0, 47);
  expect_init_panics(1, marv::prim::kDshotThrottleMax + 1);
  expect_init_panics(3, 2048);
}

TEST(L2OpenLoopIllegal, ANegativeValueAndOneThatWouldWrapToAValidCodeAreRejected) {
  expect_init_panics(0, -1);
  expect_init_panics(1, 65536);
  expect_init_panics(2, INT32_MIN);
  expect_init_panics(3, INT32_MAX);
}

}  // namespace
