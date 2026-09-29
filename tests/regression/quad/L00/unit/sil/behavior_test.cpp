// Behaviour of a running session: output rows and stamps, the parameter override, bit-exact sample conversion, and
// "a rejected call has no effect" (byte-identical outputs of a clean run and a run with bad calls interleaved).
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#include <marv/params/param_types.hpp>

#include "sil_test_util.hpp"

namespace siltest {
namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

void init_with_overrides(const marv_sil_param_override* ov, std::uint32_t n, std::uint32_t num = 625,
                         std::uint32_t den = 4) {
  const marv_sil_config cfg = make_config(num, den, ov, n);
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

// ---- output rows --------------------------------------------------------------------------------------------------

void rows_for_period(std::uint32_t num, std::uint32_t den) {
  in_child([num, den] {
    init_with_overrides(nullptr, 0, num, den);
    constexpr std::uint32_t kCapacity = 8;
    Model model;
    std::uint64_t first = 0;
    for (const std::uint32_t k : {7u, 7u, 6u, 1u, 8u}) {
      const std::vector<marv_imu_meas> s = make_samples(k, static_cast<std::uint32_t>(first));
      OutBuf buf(kCapacity);
      std::vector<std::uint16_t> servo(kCapacity, kSentinel16);
      marv_sil_out out = buf.out();
      out.servo_us = servo.data();
      ASSERT_EQ(marv_sil_tick(first, k, s.data(), &out), MARV_SIL_OK);
      expect_rows(buf, first, s, num, den, model);
      for (std::size_t i = k; i < kCapacity; ++i) {
        EXPECT_EQ(buf.t[i], kSentinel64) << "row " << i << " is beyond k and must stay untouched";
        for (std::size_t m = 0; m < kMotors; ++m) {
          EXPECT_EQ(buf.dshot[(i * kMotors) + m], kSentinel16);
        }
      }
      for (const std::uint16_t v : servo) {
        EXPECT_EQ(v, kSentinel16) << "no servos: the servo buffer is never written";
      }
      first += k;
    }
  });
}

TEST(SilRows, StampsAndLatchFor625Over4) { rows_for_period(625, 4); }
TEST(SilRows, StampsAndLatchFor12500Over3) { rows_for_period(12500, 3); }

TEST(SilRows, TheRowsBeforeTheFirstWriteAreZeroAndALaterHoldRepeatsTheLatch) {
  in_child([] {
    init_with_overrides(nullptr, 0);
    const std::vector<marv_imu_meas> s = make_samples(8);
    OutBuf buf(8);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, 8, s.data(), &out), MARV_SIL_OK);
    for (std::size_t m = 0; m < kMotors; ++m) {
      EXPECT_EQ(buf.dshot[m], 0u) << "tick 0: the composition wrote nothing, the latch is at stop";
      EXPECT_NE(buf.dshot[kMotors + m], 0u) << "tick 1 wrote";
      EXPECT_EQ(buf.dshot[(6 * kMotors) + m], buf.dshot[(5 * kMotors) + m]) << "tick 6 wrote nothing: the latch holds";
    }
    // Motors 0 (tick number) and 1 (sample code) change with every write.
    for (std::size_t m = 0; m < 2; ++m) {
      EXPECT_NE(buf.dshot[(7 * kMotors) + m], buf.dshot[(6 * kMotors) + m]) << "tick 7 wrote again";
    }
  });
}

TEST(SilRows, ALongBlockKeepsExactStamps) {
  in_child([] {
    constexpr std::uint32_t kLong = 5000;
    init_with_overrides(nullptr, 0, 12500, 3);
    const std::vector<marv_imu_meas> s = make_samples(kLong);
    OutBuf buf(kLong);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, kLong, s.data(), &out), MARV_SIL_OK);
    Model model;
    expect_rows(buf, 0, s, 12500, 3, model);
  });
}

TEST(SilRows, NoTimeAdvancesInsideATickAndTheCompositionSeesTheStamp) {
  // The test composition panics (abort) when hal_time_us() differs from the sample's stamp; surviving the run with
  // exact t_us rows shows both.
  in_child([] {
    init_with_overrides(nullptr, 0, 625, 4);
    const std::vector<marv_imu_meas> s = make_samples(16);
    OutBuf buf(16);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, 16, s.data(), &out), MARV_SIL_OK);
    EXPECT_EQ(buf.t[2], 312u);
    EXPECT_EQ(buf.t[3], 468u);
    EXPECT_EQ(buf.t[4], 625u);
  });
}

// ---- override -----------------------------------------------------------------------------------------------------

TEST(SilOverride, TheCompositionSeesTheOverriddenValues) {
  in_child([] {
    const marv_sil_param_override ov[] = {f32_override(0, 1.25f, 0.25f), i32_override(1, 7, 0.0f)};
    init_with_overrides(ov, 2);
    const std::vector<marv_imu_meas> s = make_samples(6);
    OutBuf buf(6);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, 6, s.data(), &out), MARV_SIL_OK);
    Model model;
    model.f32 = 1.25f;
    model.i32 = 7;
    expect_rows(buf, 0, s, 625, 4, model);
    EXPECT_EQ(buf.dshot[kMotors + 2], kCodeBase + 1250u);
    EXPECT_EQ(buf.dshot[kMotors + 3], kCodeBase + 7u);
  });
}

TEST(SilOverride, WithoutOverridesTheCompositionSeesTheDefaults) {
  in_child([] {
    init_with_overrides(nullptr, 0);
    const std::vector<marv_imu_meas> s = make_samples(3);
    OutBuf buf(3);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, 3, s.data(), &out), MARV_SIL_OK);
    EXPECT_EQ(buf.dshot[kMotors + 2], kCodeBase + 750u);
    EXPECT_EQ(buf.dshot[kMotors + 3], kCodeBase + 4u);
  });
}

// Probe of parameter `id` through param_get inside the composition (parameter fixture_gate_count selects it).
// Returns the three probe motors of tick 1: metadata, sigma and value codes.
struct Probe {
  std::uint16_t meta, sigma, value;
};

Probe probe_after_init(std::uint32_t id, std::vector<marv_sil_param_override> extra) {
  extra.push_back(i32_override(static_cast<std::uint32_t>(marv::ParamId::fixture_gate_count),
                               static_cast<std::int32_t>(id), 0.0f));
  init_with_overrides(extra.data(), static_cast<std::uint32_t>(extra.size()));
  const std::vector<marv_imu_meas> s = make_samples(2);
  OutBuf buf(2);
  marv_sil_out out = buf.out();
  EXPECT_EQ(marv_sil_tick(0, 2, s.data(), &out), MARV_SIL_OK);
  EXPECT_EQ(buf.dshot[kMotors + 0], kCodeBase + 1u) << "tick number";
  return {buf.dshot[kMotors + 1], buf.dshot[kMotors + 2], buf.dshot[kMotors + 3]};
}

constexpr std::uint32_t kOriginManual = static_cast<std::uint32_t>(marv::ParamOrigin::Manual);
constexpr std::uint32_t kMethodScenario = static_cast<std::uint32_t>(marv::ParamMethod::Scenario);
constexpr std::uint32_t kUnchangedBits = kProbeLockBit | kProbeUnitBit;

std::uint32_t default_meta(std::size_t id) {
  const marv::ParamRecord& d = marv::generated::kParamDefaults[id];
  return static_cast<std::uint32_t>(d.origin) | (static_cast<std::uint32_t>(d.method) << kProbeMethodShift) |
         kUnchangedBits;
}

TEST(SilOverride, ParamGetShowsManualScenarioSilOverrideWithLockAndUnitUnchanged_F32) {
  in_child([] {
    const Probe p = probe_after_init(0, {f32_override(0, 1.25f, 0.25f)});
    const std::uint32_t meta = kOriginManual | (kMethodScenario << kProbeMethodShift) | kUnchangedBits | kProbeSourceBit;
    EXPECT_EQ(p.meta, clamp_code(meta));
    EXPECT_EQ(p.sigma, clamp_code(250));
    EXPECT_EQ(p.value, clamp_code(1250));
  });
}

TEST(SilOverride, ParamGetShowsManualScenarioSilOverrideWithLockAndUnitUnchanged_I32) {
  in_child([] {
    const Probe p = probe_after_init(1, {i32_override(1, 7, 0.5f)});
    const std::uint32_t meta = kOriginManual | (kMethodScenario << kProbeMethodShift) | kUnchangedBits | kProbeSourceBit;
    EXPECT_EQ(p.meta, clamp_code(meta));
    EXPECT_EQ(p.sigma, clamp_code(500));
    EXPECT_EQ(p.value, clamp_code(7));
  });
}

TEST(SilOverride, TheSelectorParameterCanBeProbedAndIsItselfManualOverridden) {
  in_child([] {
    const Probe p = probe_after_init(static_cast<std::uint32_t>(marv::ParamId::fixture_gate_count), {});
    const std::uint32_t meta = kOriginManual | (kMethodScenario << kProbeMethodShift) | kUnchangedBits | kProbeSourceBit;
    EXPECT_EQ(p.meta, clamp_code(meta));
    EXPECT_EQ(p.value, clamp_code(static_cast<std::uint32_t>(marv::ParamId::fixture_gate_count)));
  });
}

// Negative control for the probe: parameters that are not overridden keep the generated provenance and no
// "sil-override" source. Every id except the selector (id 6) can be probed without being overridden.
TEST(SilOverride, ParametersWithoutAnOverrideKeepTheirGeneratedRecord) {
  for (std::size_t id = 0; id < marv::kParamCount; ++id) {
    if (id == static_cast<std::size_t>(marv::ParamId::fixture_gate_count)) {
      continue;
    }
    in_child([id] {
      const Probe p = probe_after_init(static_cast<std::uint32_t>(id), {});
      const marv::ParamRecord& d = marv::generated::kParamDefaults[id];
      EXPECT_EQ(p.meta, clamp_code(default_meta(id))) << "id " << id;
      EXPECT_EQ(p.sigma, clamp_code(scaled(d.sigma))) << "id " << id;
      const std::uint32_t value = d.value.type == marv::ParamType::F32 ? scaled(d.value.f32) : nonneg(d.value.i32);
      EXPECT_EQ(p.value, clamp_code(value)) << "id " << id;
    });
  }
}

TEST(SilOverride, AnOverrideOfOneParameterDoesNotTouchItsNeighbours) {
  in_child([] {
    const Probe p = probe_after_init(2, {f32_override(3, 0.5f, 0.0f), i32_override(1, 9, 0.0f)});
    const marv::ParamRecord& d = marv::generated::kParamDefaults[2];
    EXPECT_EQ(p.meta, clamp_code(default_meta(2)));
    EXPECT_EQ(p.sigma, clamp_code(scaled(d.sigma)));
    EXPECT_EQ(p.value, clamp_code(scaled(d.value.f32)));
  });
}

// ---- bit-exact conversion ------------------------------------------------------------------------------------------

TEST(SilConversion, EverySampleFieldReachesTheCompositionBitForBit) {
  in_child([] {
    init_with_overrides(nullptr, 0);
    constexpr std::uint32_t kN = 200;
    std::vector<marv_imu_meas> s = make_samples(kN);
    // Hard cases on written ticks 1..5: negative zero, the smallest denormal, the largest finite, tiny normals.
    s[1].gyro_rad_s = {-0.0f, -0.0f, -0.0f};
    s[2].gyro_rad_s = {std::numeric_limits<float>::denorm_min(), -std::numeric_limits<float>::denorm_min(),
                       std::numeric_limits<float>::min()};
    s[3].accel_m_s2 = {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -0.0f};
    s[4].temp_k = std::numeric_limits<float>::denorm_min();
    s[5].accel_m_s2 = {std::nextafter(1.0f, 2.0f), std::nextafter(1.0f, 0.0f), std::nextafter(0.0f, 1.0f)};
    // Arbitrary finite bit patterns elsewhere.
    std::uint32_t state = 0x12345678u;
    for (std::uint32_t i = 8; i < kN; ++i) {
      float* fields[] = {&s[i].gyro_rad_s.x,  &s[i].gyro_rad_s.y,  &s[i].gyro_rad_s.z,
                         &s[i].accel_m_s2.x, &s[i].accel_m_s2.y, &s[i].accel_m_s2.z};
      for (float* field : fields) {
        float v = kNaN;
        while (!std::isfinite(v)) {
          state = state * 1664525u + 1013904223u;
          std::memcpy(&v, &state, sizeof v);
        }
        *field = v;
      }
    }
    // The control is sensitive: flipping -0.0 to +0.0 changes the code the composition would write.
    marv_imu_meas plus = s[1];
    plus.gyro_rad_s = {0.0f, 0.0f, 0.0f};
    ASSERT_NE(sample_code(words_of(s[1])), sample_code(words_of(plus)));

    OutBuf buf(kN);
    marv_sil_out out = buf.out();
    ASSERT_EQ(marv_sil_tick(0, kN, s.data(), &out), MARV_SIL_OK);
    Model model;
    expect_rows(buf, 0, s, 625, 4, model);
  });
}

// ---- a rejected call has no effect -----------------------------------------------------------------------------------

constexpr std::uint32_t kBlock = 5;
constexpr std::uint32_t kBlocks = 6;

void reject_inits_before_init() {
  EXPECT_EQ(marv_sil_init(nullptr), MARV_SIL_E_NULL);
  marv_sil_config cfg = make_config();
  cfg.struct_size += 1;
  EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_ABI);
  cfg = make_config(625, 0);
  EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_PERIOD);
  cfg = make_config();
  cfg.param_schema_hash ^= 1;
  EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_SCHEMA);
  const marv_sil_param_override bad[] = {f32_override(0, 9.0f, 0.0f), f32_override(0, 8.0f, 0.0f)};
  cfg = make_config(625, 4, bad, 2);
  EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_PARAM);
  const marv_sil_param_override bad_value[] = {f32_override(0, 9.0f, 0.0f), i32_override(1, 100, kNaN)};
  cfg = make_config(625, 4, bad_value, 2);
  EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_PARAM);
}

void reject_ticks(std::uint64_t next, const std::vector<marv_imu_meas>& clean) {
  OutBuf scratch(kBlock);
  marv_sil_out out = scratch.out();
  EXPECT_EQ(marv_sil_tick(next + 1, kBlock, clean.data(), &out), MARV_SIL_E_TICK);
  if (next > 0) {
    EXPECT_EQ(marv_sil_tick(next - 1, kBlock, clean.data(), &out), MARV_SIL_E_TICK);
  }
  EXPECT_EQ(marv_sil_tick(next, 0, clean.data(), &out), MARV_SIL_E_COUNT);
  EXPECT_EQ(marv_sil_tick(next, kBlock + 1, clean.data(), &out), MARV_SIL_E_COUNT);
  EXPECT_EQ(marv_sil_tick(next, kBlock, nullptr, &out), MARV_SIL_E_NULL);
  EXPECT_EQ(marv_sil_tick(next, kBlock, clean.data(), nullptr), MARV_SIL_E_NULL);
  marv_sil_out bad_abi = out;
  bad_abi.struct_size += 1;
  EXPECT_EQ(marv_sil_tick(next, kBlock, clean.data(), &bad_abi), MARV_SIL_E_ABI);
  for (std::uint32_t i = 0; i < kBlock; ++i) {
    std::vector<marv_imu_meas> s = clean;
    s[i].temp_k = kNaN;
    EXPECT_EQ(marv_sil_tick(next, kBlock, s.data(), &out), MARV_SIL_E_INPUT);
    s[i] = clean[i];
    s[i].flags |= 1u << MARV_IMU_FLAG_COUNT;
    EXPECT_EQ(marv_sil_tick(next, kBlock, s.data(), &out), MARV_SIL_E_INPUT);
  }
  EXPECT_TRUE(scratch.untouched()) << "a rejected tick wrote output";
}

std::vector<std::uint8_t> run_scenario(bool noisy) {
  const marv_sil_param_override ov[] = {f32_override(0, 1.25f, 0.25f), i32_override(1, 7, 0.0f)};
  if (noisy) {
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_E_STATE);
    const std::vector<marv_imu_meas> s = make_samples(kBlock);
    OutBuf buf(kBlock);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(0, kBlock, s.data(), &out), MARV_SIL_E_STATE);
    EXPECT_TRUE(buf.untouched());
    reject_inits_before_init();
  }
  init_with_overrides(ov, 2);
  std::vector<std::uint8_t> bytes;
  for (std::uint32_t b = 0; b < kBlocks; ++b) {
    const std::uint64_t first = static_cast<std::uint64_t>(b) * kBlock;
    const std::vector<marv_imu_meas> s = make_samples(kBlock, static_cast<std::uint32_t>(first));
    if (noisy) {
      const marv_sil_config cfg = make_config();
      EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_STATE);
      reject_ticks(first, s);
    }
    OutBuf buf(kBlock);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(first, kBlock, s.data(), &out), MARV_SIL_OK);
    if (noisy) {
      reject_ticks(first + kBlock, s);
    }
    const auto* t = reinterpret_cast<const std::uint8_t*>(buf.t.data());
    const auto* d = reinterpret_cast<const std::uint8_t*>(buf.dshot.data());
    bytes.insert(bytes.end(), t, t + (buf.t.size() * sizeof(std::uint64_t)));
    bytes.insert(bytes.end(), d, d + (buf.dshot.size() * sizeof(std::uint16_t)));
  }
  EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
  if (noisy) {
    const std::vector<marv_imu_meas> s = make_samples(kBlock);
    OutBuf buf(kBlock);
    marv_sil_out out = buf.out();
    EXPECT_EQ(marv_sil_tick(kBlocks * kBlock, kBlock, s.data(), &out), MARV_SIL_E_STATE);
    EXPECT_TRUE(buf.untouched());
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_E_STATE);
  }
  return bytes;
}

std::string output_path(const char* tag) {
  const std::string name = std::string{"marv_sil_noeffect_"} + tag + "_" + std::to_string(::getpid());
  return (std::filesystem::temp_directory_path() / name).string();
}

void write_bytes(const std::string& path, const std::vector<std::uint8_t>& bytes) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

std::vector<std::uint8_t> read_bytes(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

TEST(SilNoEffect, BadCallsInterleavedProduceByteIdenticalOutputs) {
  const std::string clean_path = output_path("clean");
  const std::string noisy_path = output_path("noisy");
  in_child([&] { write_bytes(clean_path, run_scenario(false)); });
  in_child([&] { write_bytes(noisy_path, run_scenario(true)); });
  const std::vector<std::uint8_t> clean = read_bytes(clean_path);
  const std::vector<std::uint8_t> noisy = read_bytes(noisy_path);
  std::remove(clean_path.c_str());
  std::remove(noisy_path.c_str());

  const std::size_t expected =
      kBlocks * kBlock * (sizeof(std::uint64_t) + (kMotors * sizeof(std::uint16_t)));
  ASSERT_EQ(clean.size(), expected);
  ASSERT_EQ(noisy.size(), expected);
  EXPECT_EQ(clean, noisy);
  // The comparison is not vacuous: the run has rows after the first write, with the overridden values.
  bool any_nonzero_dshot = false;
  for (std::size_t i = kBlock * sizeof(std::uint64_t); i < clean.size(); ++i) {
    any_nonzero_dshot = any_nonzero_dshot || clean[i] != 0;
  }
  EXPECT_TRUE(any_nonzero_dshot);
}

TEST(SilNoEffect, TheComparisonSeesADifferenceWhenOneIsPlanted) {
  // Negative control: a scenario whose noise actually consumes a tick differs from the clean one.
  const std::string clean_path = output_path("ctl_clean");
  const std::string bent_path = output_path("ctl_bent");
  in_child([&] { write_bytes(clean_path, run_scenario(false)); });
  in_child([&] {
    const marv_sil_param_override ov[] = {f32_override(0, 1.25f, 0.25f), i32_override(1, 8, 0.0f)};  // 8, not 7
    init_with_overrides(ov, 2);
    std::vector<std::uint8_t> bytes;
    for (std::uint32_t b = 0; b < kBlocks; ++b) {
      const std::uint64_t first = static_cast<std::uint64_t>(b) * kBlock;
      const std::vector<marv_imu_meas> s = make_samples(kBlock, static_cast<std::uint32_t>(first));
      OutBuf buf(kBlock);
      marv_sil_out out = buf.out();
      EXPECT_EQ(marv_sil_tick(first, kBlock, s.data(), &out), MARV_SIL_OK);
      const auto* t = reinterpret_cast<const std::uint8_t*>(buf.t.data());
      const auto* d = reinterpret_cast<const std::uint8_t*>(buf.dshot.data());
      bytes.insert(bytes.end(), t, t + (buf.t.size() * sizeof(std::uint64_t)));
      bytes.insert(bytes.end(), d, d + (buf.dshot.size() * sizeof(std::uint16_t)));
    }
    write_bytes(bent_path, bytes);
  });
  const std::vector<std::uint8_t> clean = read_bytes(clean_path);
  const std::vector<std::uint8_t> bent = read_bytes(bent_path);
  std::remove(clean_path.c_str());
  std::remove(bent_path.c_str());
  ASSERT_EQ(clean.size(), bent.size());
  EXPECT_NE(clean, bent);
}

}  // namespace
}  // namespace siltest
