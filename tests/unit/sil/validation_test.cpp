// Argument validation: E_NULL, E_ABI, E_PERIOD, E_SCHEMA, E_PARAM (with marv_sil_error_index), E_TICK, E_COUNT and
// E_INPUT (one test per IMU rule, with the sample index). A rejected call has no effect, so each test ends by showing
// the library still accepts a valid call.
#include <cmath>
#include <limits>

#include "sil_test_util.hpp"

namespace siltest {
namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr std::uint32_t kBadIndex = 3;
constexpr std::uint32_t kSamples = 5;

void init_ok() {
  const marv_sil_config cfg = make_config();
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

// ---- E_NULL -----------------------------------------------------------------------------------------------------

TEST(SilNull, InitNullPointers) {
  in_child([] {
    EXPECT_EQ(marv_sil_init(nullptr), MARV_SIL_E_NULL);

    const marv_sil_param_override ov = f32_override(0, 1.0f, 0.0f);
    marv_sil_config cfg = make_config(625, 4, nullptr, 1);  // one override, but no array
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_NULL);
    cfg = make_config(625, 4, &ov, 0);  // an array, but zero overrides: "NULL iff 0"
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_NULL);

    init_ok();  // nothing above took effect
  });
}

TEST(SilNull, TickNullPointers) {
  in_child([] {
    init_ok();
    const std::vector<marv_imu_meas> s = make_samples(2);
    OutBuf buf(2);
    marv_sil_out out = buf.out();

    EXPECT_EQ(marv_sil_tick(0, 2, nullptr, &out), MARV_SIL_E_NULL);
    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), nullptr), MARV_SIL_E_NULL);
    marv_sil_out bad = out;
    bad.t_us = nullptr;
    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &bad), MARV_SIL_E_NULL);
    bad = out;
    bad.dshot = nullptr;
    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &bad), MARV_SIL_E_NULL);
    EXPECT_TRUE(buf.untouched());

    // servo_us may be NULL because this composition has no servos.
    bad = out;
    bad.servo_us = nullptr;
    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &bad), MARV_SIL_OK);
  });
}

// ---- E_ABI ------------------------------------------------------------------------------------------------------

TEST(SilAbi, EachInitSizeField) {
  in_child([] {
    for (int which = 0; which < 3; ++which) {
      for (const std::int32_t delta : {-1, 1}) {
        marv_sil_config cfg = make_config();
        std::uint32_t* fields[] = {&cfg.struct_size, &cfg.imu_meas_size, &cfg.override_size};
        *fields[which] = static_cast<std::uint32_t>(static_cast<std::int32_t>(*fields[which]) + delta);
        EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_ABI) << "field " << which << " delta " << delta;
      }
      marv_sil_config zero = make_config();
      std::uint32_t* fields[] = {&zero.struct_size, &zero.imu_meas_size, &zero.override_size};
      *fields[which] = 0;
      EXPECT_EQ(marv_sil_init(&zero), MARV_SIL_E_ABI) << "field " << which << " zero";
    }
    init_ok();
  });
}

TEST(SilAbi, InfoStructSize) {
  marv_sil_info info{};
  info.struct_size = sizeof(marv_sil_info) - 1;
  EXPECT_EQ(marv_sil_info_get(&info), MARV_SIL_E_ABI);
  info.struct_size = 0;
  EXPECT_EQ(marv_sil_info_get(&info), MARV_SIL_E_ABI);
}

TEST(SilAbi, OutStructSize) {
  in_child([] {
    init_ok();
    const std::vector<marv_imu_meas> s = make_samples(2);
    OutBuf buf(2);
    for (const std::uint32_t size : {0u, static_cast<std::uint32_t>(sizeof(marv_sil_out)) - 1u,
                                     static_cast<std::uint32_t>(sizeof(marv_sil_out)) + 1u}) {
      marv_sil_out out = buf.out();
      out.struct_size = size;
      EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &out), MARV_SIL_E_ABI) << size;
    }
    EXPECT_TRUE(buf.untouched());
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &out), MARV_SIL_OK);
  });
}

// ---- E_PERIOD and E_SCHEMA ---------------------------------------------------------------------------------------

TEST(SilPeriod, InvalidPeriodsAreRefused) {
  in_child([] {
    struct P {
      std::uint32_t num, den;
    };
    for (const P p : {P{625, 0}, P{0, 0}, P{3, 4}, P{0, 1}, P{3, 4000000000u}}) {
      const marv_sil_config cfg = make_config(p.num, p.den);
      EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_PERIOD) << p.num << "/" << p.den;
    }
    init_ok();
  });
}

TEST(SilPeriod, BoundaryPeriodsAreAccepted) {
  in_child([] {
    const marv_sil_config cfg = make_config(4, 4);  // num == den: one microsecond per tick
    ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
    const std::vector<marv_imu_meas> s = make_samples(3);
    OutBuf buf(3);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, 3, s.data(), &out), MARV_SIL_OK);
    EXPECT_EQ(buf.t[0], 0u);
    EXPECT_EQ(buf.t[1], 1u);
    EXPECT_EQ(buf.t[2], 2u);
  });
}

TEST(SilSchema, WrongHashIsRefused) {
  in_child([] {
    const std::uint64_t wrong[] = {marv::kParamSchemaHash + 1, marv::kParamSchemaHash ^ 0x8000000000000000ull, 0};
    for (const std::uint64_t h : wrong) {
      marv_sil_config cfg = make_config();
      cfg.param_schema_hash = h;
      EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_SCHEMA) << h;
    }
    init_ok();
  });
}

// ---- E_PARAM ----------------------------------------------------------------------------------------------------

// Overrides [good, good, bad, good]: the rejected index is 2. Returns the init status and the error index.
struct InitResult {
  marv_sil_status status;
  std::uint32_t index;
};

InitResult init_with(marv_sil_param_override bad) {
  const marv_sil_param_override list[] = {f32_override(0, 1.5f, 0.1f), i32_override(1, 5, 0.0f), bad,
                                          f32_override(2, 0.5f, 0.0f)};
  const marv_sil_config cfg = make_config(625, 4, list, 4);
  const marv_sil_status s = marv_sil_init(&cfg);
  return {s, marv_sil_error_index()};
}

void expect_param_rejected(marv_sil_param_override bad, const char* what) {
  const InitResult r = init_with(bad);
  EXPECT_EQ(r.status, MARV_SIL_E_PARAM) << what;
  EXPECT_EQ(r.index, 2u) << what;
}

TEST(SilParam, UnknownId) {
  in_child([] {
    expect_param_rejected(f32_override(static_cast<std::uint32_t>(marv::kParamCount), 1.0f, 0.0f), "id == count");
    expect_param_rejected(f32_override(0xFFFFFFFFu, 1.0f, 0.0f), "id == UINT32_MAX");
    init_ok();
  });
}

TEST(SilParam, TypeMismatch) {
  in_child([] {
    marv_sil_param_override as_i32 = i32_override(0, 1, 0.0f);  // parameter 0 is f32
    expect_param_rejected(as_i32, "i32 override of an f32 parameter");
    marv_sil_param_override as_f32 = f32_override(1, 1.0f, 0.0f);  // parameter 1 is i32
    expect_param_rejected(as_f32, "f32 override of an i32 parameter");
    marv_sil_param_override unknown_type = f32_override(2, 1.0f, 0.0f);
    unknown_type.type = 2;
    expect_param_rejected(unknown_type, "type 2");
    unknown_type.type = 0xFFFFFFFFu;
    expect_param_rejected(unknown_type, "type UINT32_MAX");
    init_ok();
  });
}

TEST(SilParam, NonFiniteF32Value) {
  in_child([] {
    expect_param_rejected(f32_override(0, kNaN, 0.0f), "NaN");
    expect_param_rejected(f32_override(0, kInf, 0.0f), "+Inf");
    expect_param_rejected(f32_override(0, -kInf, 0.0f), "-Inf");
    init_ok();
  });
}

TEST(SilParam, BadSigma) {
  in_child([] {
    expect_param_rejected(f32_override(0, 1.0f, kNaN), "sigma NaN");
    expect_param_rejected(f32_override(0, 1.0f, kInf), "sigma +Inf");
    expect_param_rejected(f32_override(0, 1.0f, -kInf), "sigma -Inf");
    expect_param_rejected(f32_override(0, 1.0f, -0.5f), "sigma negative");
    expect_param_rejected(i32_override(1, 1, -1.0f), "i32 sigma negative");
    expect_param_rejected(i32_override(1, 1, kNaN), "i32 sigma NaN");
    init_ok();
  });
}

TEST(SilParam, DuplicateIdReportsTheSecondOccurrence) {
  in_child([] {
    expect_param_rejected(f32_override(0, 2.0f, 0.0f), "repeat of override 0");
    expect_param_rejected(i32_override(1, 9, 0.0f), "repeat of override 1");
    // A later override that repeats an earlier one is the rejected one, even when the earlier one is fine.
    const InitResult later = init_with(f32_override(2, 4.0f, 0.0f));  // list is [id 0, id 1, id 2, id 2]
    EXPECT_EQ(later.status, MARV_SIL_E_PARAM);
    EXPECT_EQ(later.index, 3u);
    init_ok();
  });
}

TEST(SilParam, FirstBadOverrideWinsAndFieldsThatDoNotApplyAreIgnored) {
  in_child([] {
    const marv_sil_param_override two_bad[] = {f32_override(0, 1.0f, 0.0f), f32_override(0, 1.0f, 0.0f),
                                               f32_override(100, 1.0f, 0.0f)};
    marv_sil_config cfg = make_config(625, 4, two_bad, 3);
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_PARAM);
    EXPECT_EQ(marv_sil_error_index(), 1u);

    // The unused member of an override is ignored: a NaN f32 next to an i32 value, garbage i32 next to an f32 value.
    marv_sil_param_override i32_with_nan = i32_override(1, 3, 0.0f);
    i32_with_nan.f32 = kNaN;
    marv_sil_param_override f32_with_garbage = f32_override(0, 1.0f, -0.0f);
    f32_with_garbage.i32 = 12345;
    const marv_sil_param_override ok[] = {i32_with_nan, f32_with_garbage};
    cfg = make_config(625, 4, ok, 2);
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
  });
}

// ---- E_TICK and E_COUNT --------------------------------------------------------------------------------------------

TEST(SilTick, RepeatAndSkipAreRefusedAndChangeNothing) {
  in_child([] {
    init_ok();
    const std::vector<marv_imu_meas> s = make_samples(4);
    OutBuf buf(4);
    marv_sil_out out = buf.out();

    EXPECT_EQ(marv_sil_tick(1, 4, s.data(), &out), MARV_SIL_E_TICK) << "skip at the start";
    EXPECT_EQ(marv_sil_tick(0xFFFFFFFFFFFFFFFFull, 4, s.data(), &out), MARV_SIL_E_TICK);
    EXPECT_TRUE(buf.untouched());
    EXPECT_EQ(marv_sil_tick(0, 4, s.data(), &out), MARV_SIL_OK);
    EXPECT_EQ(marv_sil_tick(0, 4, s.data(), &out), MARV_SIL_E_TICK) << "repeat";
    EXPECT_EQ(marv_sil_tick(3, 4, s.data(), &out), MARV_SIL_E_TICK) << "overlap";
    EXPECT_EQ(marv_sil_tick(5, 4, s.data(), &out), MARV_SIL_E_TICK) << "skip";
    EXPECT_EQ(marv_sil_tick(4, 4, s.data(), &out), MARV_SIL_OK) << "the next contiguous block still works";
    EXPECT_EQ(buf.t[0], reference_stamp(625, 4, 4));
  });
}

TEST(SilTick, ETickIsCheckedBeforeECount) {
  in_child([] {
    init_ok();
    const std::vector<marv_imu_meas> s = make_samples(2);
    OutBuf buf(2);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(7, 0, s.data(), &out), MARV_SIL_E_TICK);
    EXPECT_EQ(marv_sil_tick(7, 3, s.data(), &out), MARV_SIL_E_TICK);
  });
}

TEST(SilTick, ECountForZeroAndAboveCapacity) {
  in_child([] {
    init_ok();
    const std::vector<marv_imu_meas> s = make_samples(4);
    OutBuf buf(2);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(0, 0, s.data(), &out), MARV_SIL_E_COUNT);
    EXPECT_EQ(marv_sil_tick(0, 3, s.data(), &out), MARV_SIL_E_COUNT);
    EXPECT_EQ(marv_sil_tick(0, 0xFFFFFFFFu, s.data(), &out), MARV_SIL_E_COUNT);
    EXPECT_TRUE(buf.untouched());

    std::uint64_t dummy_t = kSentinel64;
    std::uint16_t dummy_d[kMotors] = {};
    marv_sil_out zero_cap = out;
    zero_cap.capacity_ticks = 0;
    zero_cap.t_us = &dummy_t;
    zero_cap.dshot = dummy_d;
    EXPECT_EQ(marv_sil_tick(0, 1, s.data(), &zero_cap), MARV_SIL_E_COUNT) << "capacity 0";

    EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &out), MARV_SIL_OK) << "k == capacity is fine";
  });
}

// ---- E_INPUT ------------------------------------------------------------------------------------------------------

// Ticks kSamples samples where sample kBadIndex is `bad` and expects E_INPUT at that index and nothing written.
// finish_with_clean_run then shows that no tick was consumed and the composition did not run.
void expect_input_rejected(const marv_imu_meas& bad, const char* what) {
  std::vector<marv_imu_meas> s = make_samples(kSamples);
  s[kBadIndex] = bad;
  OutBuf buf(kSamples);
  marv_sil_out out = buf.out();
  EXPECT_EQ(marv_sil_tick(0, kSamples, s.data(), &out), MARV_SIL_E_INPUT) << what;
  EXPECT_EQ(marv_sil_error_index(), kBadIndex) << what;
  EXPECT_TRUE(buf.untouched()) << what;
}

void finish_with_clean_run() {
  const std::vector<marv_imu_meas> clean = make_samples(kSamples);
  OutBuf buf(kSamples);
  marv_sil_out out = buf.out();
  EXPECT_EQ(marv_sil_tick(0, kSamples, clean.data(), &out), MARV_SIL_OK);
  Model model;
  expect_rows(buf, 0, clean, 625, 4, model);
}

float* float_field(marv_imu_meas& m, int which) {
  float* fields[] = {&m.gyro_rad_s.x, &m.gyro_rad_s.y, &m.gyro_rad_s.z, &m.accel_m_s2.x,
                     &m.accel_m_s2.y, &m.accel_m_s2.z, &m.temp_k};
  return fields[which];
}

constexpr int kFloatFields = 7;

TEST(SilInput, NaNInAnyField) {
  in_child([] {
    init_ok();
    for (int f = 0; f < kFloatFields; ++f) {
      marv_imu_meas m = make_sample(kBadIndex);
      *float_field(m, f) = kNaN;
      expect_input_rejected(m, "NaN");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, InfInAnyField) {
  in_child([] {
    init_ok();
    for (int f = 0; f < kFloatFields; ++f) {
      for (const float v : {kInf, -kInf}) {
        marv_imu_meas m = make_sample(kBadIndex);
        *float_field(m, f) = v;
        expect_input_rejected(m, "Inf");
      }
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, ReservedFlagBits) {
  in_child([] {
    init_ok();
    for (std::uint32_t bit = MARV_IMU_FLAG_COUNT; bit < 32; ++bit) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags |= 1u << bit;
      expect_input_rejected(m, "reserved flag bit");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, GyroFieldMustBeZeroWhenGyroValidIsClear) {
  in_child([] {
    init_ok();
    for (int f = 0; f < 3; ++f) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags &= ~(1u << MARV_IMU_GYRO_VALID);
      m.flags &= ~((1u << MARV_IMU_GYRO_SAT_X) | (1u << MARV_IMU_GYRO_SAT_Y) | (1u << MARV_IMU_GYRO_SAT_Z));
      m.gyro_rad_s = {0.0f, 0.0f, 0.0f};
      *float_field(m, f) = 1.0e-30f;  // any nonzero value, however small
      expect_input_rejected(m, "gyro nonzero with GyroValid clear");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, GyroSaturationMustBeClearWhenGyroValidIsClear) {
  in_child([] {
    init_ok();
    for (const int sat : {MARV_IMU_GYRO_SAT_X, MARV_IMU_GYRO_SAT_Y, MARV_IMU_GYRO_SAT_Z}) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags = (kAllValid & ~(1u << MARV_IMU_GYRO_VALID)) | (1u << sat);
      m.gyro_rad_s = {0.0f, 0.0f, 0.0f};
      expect_input_rejected(m, "gyro saturation with GyroValid clear");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, AccelFieldMustBeZeroWhenAccelValidIsClear) {
  in_child([] {
    init_ok();
    for (int f = 3; f < 6; ++f) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags = kAllValid & ~(1u << MARV_IMU_ACCEL_VALID);
      m.accel_m_s2 = {0.0f, 0.0f, 0.0f};
      *float_field(m, f) = -1.0e-30f;
      expect_input_rejected(m, "accel nonzero with AccelValid clear");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, AccelSaturationMustBeClearWhenAccelValidIsClear) {
  in_child([] {
    init_ok();
    for (const int sat : {MARV_IMU_ACCEL_SAT_X, MARV_IMU_ACCEL_SAT_Y, MARV_IMU_ACCEL_SAT_Z}) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags = (kAllValid & ~(1u << MARV_IMU_ACCEL_VALID)) | (1u << sat);
      m.accel_m_s2 = {0.0f, 0.0f, 0.0f};
      expect_input_rejected(m, "accel saturation with AccelValid clear");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, TempFieldMustBeZeroWhenTempValidIsClear) {
  in_child([] {
    init_ok();
    for (const float t : {1.0f, -1.0f, 1.0e-30f, 300.0f}) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags = kAllValid & ~(1u << MARV_IMU_TEMP_VALID);
      m.temp_k = t;
      expect_input_rejected(m, "temp nonzero with TempValid clear");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, TempValidRequiresPositiveTemperature) {
  in_child([] {
    init_ok();
    for (const float t : {0.0f, -0.0f, -1.0f, -300.0f, -std::numeric_limits<float>::denorm_min()}) {
      marv_imu_meas m = make_sample(kBadIndex);
      m.flags = kAllValid;
      m.temp_k = t;
      expect_input_rejected(m, "TempValid with temp_k <= 0");
    }
    finish_with_clean_run();
  });
}

TEST(SilInput, FirstBadSampleIsReported) {
  in_child([] {
    init_ok();
    std::vector<marv_imu_meas> s = make_samples(kSamples);
    s[1].temp_k = kNaN;
    s[kBadIndex].flags |= 1u << MARV_IMU_FLAG_COUNT;
    OutBuf buf(kSamples);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(0, kSamples, s.data(), &out), MARV_SIL_E_INPUT);
    EXPECT_EQ(marv_sil_error_index(), 1u);
    s[0].gyro_rad_s.x = kInf;
    EXPECT_EQ(marv_sil_tick(0, kSamples, s.data(), &out), MARV_SIL_E_INPUT);
    EXPECT_EQ(marv_sil_error_index(), 0u);
    EXPECT_TRUE(buf.untouched());
  });
}

TEST(SilInput, ErrorIndexRestsOnTheLastInputOrParamError) {
  in_child([] {
    init_ok();
    std::vector<marv_imu_meas> s = make_samples(kSamples);
    s[4].temp_k = kNaN;
    OutBuf buf(kSamples);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(0, kSamples, s.data(), &out), MARV_SIL_E_INPUT);
    EXPECT_EQ(marv_sil_error_index(), 4u);
    EXPECT_EQ(marv_sil_tick(1, kSamples, s.data(), &out), MARV_SIL_E_TICK);
    EXPECT_EQ(marv_sil_error_index(), 4u) << "a rejection that has no index leaves the last one in place";
  });
}

}  // namespace
}  // namespace siltest
