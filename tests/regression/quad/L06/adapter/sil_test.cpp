// L6 stage (a), the adapter's IMU path against the real SIL (open-loop composition): every IMU sample the plant gives
// passes the SIL's boundary validity rules (marv_sil_tick returns MARV_SIL_E_INPUT otherwise), and the SIL's own t_us
// stamps stay the nominal rational floor(n * 625 / 4) us at e = -65, 0 and +65 ppm, because e is applied to the plant's
// clock only. Control: e put into the SIL period changes the stamps.
// marv_sil_init succeeds once per process, so each case runs in a forked child (as the L2 adapter test).
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>

#include "marv/params/param_ids.hpp"
#include "support.hpp"

namespace {

using namespace marv::gz;
using namespace marv::gz::l6test;

// n * 625 / 4 in integers, floored: the nominal stamp of tick n (design/scenario_values.yaml).
std::uint64_t nominal_stamp(std::uint64_t n) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(n) * kPeriodNumUs / kPeriodDen);
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

template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

// Open-loop DShot by override (labelled test values, legal: 48..2047 or 0).
void init_sil(std::uint32_t num_us, std::uint32_t den) {
  const std::array<std::int32_t, 4> want = {700, 0, 1500, 2047};
  const marv::ParamId ids[4] = {marv::ParamId::ol_dshot_m1, marv::ParamId::ol_dshot_m2, marv::ParamId::ol_dshot_m3,
                                marv::ParamId::ol_dshot_m4};
  marv_sil_param_override ov[4] = {};
  for (std::size_t k = 0; k < 4; ++k) {
    ov[k].id = static_cast<std::uint32_t>(ids[k]);
    ov[k].type = MARV_PARAM_I32;
    ov[k].i32 = want[k];
  }
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = num_us;
  c.tick_period_den = den;
  c.n_overrides = 4;
  c.overrides = ov;
  c.param_schema_hash = marv::kParamSchemaHash;
  ASSERT_EQ(marv_sil_init(&c), MARV_SIL_OK);
}

constexpr std::size_t kSilTicks = 64;  // 16 whole 625/4 us periods: every fractional part .00, .25, .50, .75 occurs

// kSilTicks single-tick host steps through the real SIL. Returns the SIL's stamp of every tick.
std::vector<std::uint64_t> run_ticks(int corner, std::vector<marv_plant_imu_out>* samples) {
  const marv_plant_imu_config cfg = noisy_imu_config();
  ImuSilCommandSource src;
  AdapterConfig ac;
  ac.t_tick_nominal_s = t_nom();
  ac.clock_corner = corner;
  ac.odr_error = kOdrError;
  ac.imu = &cfg;
  Adapter adapter(make_plant(3), src, ac);  // labelled seed
  std::vector<std::uint64_t> stamps;
  for (std::uint64_t n = 0; n < kSilTicks; ++n) {
    const StepResult r = adapter.step(states()[n % states().size()], n, 1);
    EXPECT_EQ(r.status, Status::kOk) << "tick " << n << " sil status " << src.last_status();
    if (r.status != Status::kOk) {
      break;
    }
    EXPECT_EQ(src.last_status(), MARV_SIL_OK);
    stamps.push_back(src.last_stamp_us());
    if (samples != nullptr) {
      samples->push_back(r.ticks[0].imu);
    }
  }
  return stamps;
}

TEST(L6AdapterSil, EverySampleIsValidAndTheStampsAreNominalAtMinusZeroAndPlus65Ppm) {
  for (const int corner : {-1, 0, 1}) {
    in_child([corner] {
      init_sil(kPeriodNumUs, kPeriodDen);
      std::vector<marv_plant_imu_out> samples;
      const std::vector<std::uint64_t> stamps = run_ticks(corner, &samples);
      ASSERT_EQ(stamps.size(), kSilTicks);
      ASSERT_EQ(samples.size(), kSilTicks);
      for (std::uint64_t n = 0; n < kSilTicks; ++n) {
        EXPECT_EQ(stamps[n], nominal_stamp(n)) << "corner " << corner << " tick " << n;
        // The SIL accepted the sample (E_INPUT otherwise); the flags are the generic IMU's: gyro and accel valid, no
        // temperature.
        const std::uint32_t valid = (1u << MARV_IMU_GYRO_VALID) | (1u << MARV_IMU_ACCEL_VALID);
        EXPECT_EQ(samples[n].flags & valid, valid);
        EXPECT_EQ(samples[n].flags & (1u << MARV_IMU_TEMP_VALID), 0u);
      }
    });
  }
}

// Control: the SIL's validity check is live (a sample with a nonzero gyro and GyroValid clear is rejected).
TEST(L6AdapterSil, ARuleBreakingSampleIsRejectedByTheSil) {
  in_child([] {
    init_sil(kPeriodNumUs, kPeriodDen);
    ImuSilCommandSource src;
    marv_imu_meas bad{};
    bad.gyro_rad_s.x = 1.0F;  // flags 0: GyroValid clear, so the field must be zero
    Dshot d{};
    EXPECT_FALSE(src.dshot(0, &bad, d));
    EXPECT_EQ(src.last_status(), MARV_SIL_E_INPUT);
    const marv_imu_meas good{};
    EXPECT_TRUE(src.dshot(0, &good, d));
  });
}

// Control: e put into the SIL period (the wrong place) changes the stamps. e = +65 ppm: period 625 / 4 / (1 + e) us =
// 625e6 / (4 (1e6 + 65)) us, exactly the rational 625 * 1e6 / (4 * (1e6 + 65)); e = -65 ppm the same with 1e6 - 65.
TEST(L6AdapterSil, PuttingEIntoTheSilPeriodChangesTheStamps) {
  for (const int sign : {-1, 1}) {
    in_child([sign] {
      constexpr std::uint32_t kMega = 1000000;  // 1e6 us/s: ppm are parts per million
      constexpr std::uint32_t kPpm = 65;        // kOdrError = 65 ppm
      static_assert(static_cast<double>(kPpm) / static_cast<double>(kMega) == kOdrError);
      const std::uint32_t num = kPeriodNumUs * kMega;
      const std::uint32_t den = kPeriodDen * (sign > 0 ? kMega + kPpm : kMega - kPpm);
      init_sil(num, den);
      const std::vector<std::uint64_t> stamps = run_ticks(0, nullptr);
      ASSERT_EQ(stamps.size(), kSilTicks);
      std::size_t differing = 0;
      for (std::uint64_t n = 0; n < kSilTicks; ++n) {
        differing += stamps[n] != nominal_stamp(n) ? 1u : 0u;
      }
      EXPECT_GT(differing, 0u) << "sign " << sign;
    });
  }
}

}  // namespace
