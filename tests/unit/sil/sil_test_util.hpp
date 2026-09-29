// Shared helpers for the SIL library tests.
//
// marv_sil_init succeeds once per process, so every test that calls init, tick or shutdown runs its body in a forked
// child (in_child). The parent process only ever calls marv_sil_info_get, marv_sil_status_str and
// marv_sil_error_index, which do not change library state, so it stays UNINIT for the whole run (also under
// --gtest_shuffle --gtest_repeat=N).
#pragma once

#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <marv/composition.hpp>
#include <marv/params/param_ids.hpp>

#include "sil_test_codes.hpp"

namespace siltest {

inline constexpr std::size_t kMotors = marv::composition::kMotors;
inline constexpr std::uint16_t kSentinel16 = 0xAAAA;
inline constexpr std::uint64_t kSentinel64 = 0xAAAAAAAAAAAAAAAAull;

// Prints the failed assertions recorded so far in the current test on stderr, which the parent's death-test report
// shows. Returns true if there were any.
inline bool report_failures_on_stderr() {
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

// Runs body in a child process; the child exits 0 iff no gtest assertion failed in it.
template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

inline marv_sil_config make_config(std::uint32_t num_us = 625, std::uint32_t den = 4,
                                   const marv_sil_param_override* overrides = nullptr, std::uint32_t n_overrides = 0) {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = num_us;
  c.tick_period_den = den;
  c.n_overrides = n_overrides;
  c.overrides = overrides;
  c.param_schema_hash = marv::kParamSchemaHash;
  return c;
}

inline marv_sil_param_override f32_override(std::uint32_t id, float value, float sigma) {
  marv_sil_param_override o{};
  o.id = id;
  o.type = MARV_PARAM_F32;
  o.f32 = value;
  o.sigma = sigma;
  return o;
}

inline marv_sil_param_override i32_override(std::uint32_t id, std::int32_t value, float sigma) {
  marv_sil_param_override o{};
  o.id = id;
  o.type = MARV_PARAM_I32;
  o.i32 = value;
  o.sigma = sigma;
  return o;
}

constexpr std::uint32_t kAllValid = (1u << MARV_IMU_GYRO_VALID) | (1u << MARV_IMU_ACCEL_VALID) | (1u << MARV_IMU_TEMP_VALID);

// A valid sample that differs for every i.
inline marv_imu_meas make_sample(std::uint32_t i) {
  const auto f = static_cast<float>(i);
  marv_imu_meas m{};
  m.gyro_rad_s = {0.01f * f, -0.02f * f, 0.5f + f};
  m.accel_m_s2 = {-9.81f + f, 0.25f * f, 1.5f};
  m.temp_k = 300.0f + static_cast<float>(i % 5);
  m.flags = kAllValid | ((i % 3) == 0 ? (1u << MARV_IMU_GYRO_SAT_Z) : 0u) | ((i % 4) == 0 ? (1u << MARV_IMU_ACCEL_SAT_X) : 0u);
  return m;
}

inline std::vector<marv_imu_meas> make_samples(std::uint32_t k, std::uint32_t first = 0) {
  std::vector<marv_imu_meas> v;
  v.reserve(k);
  for (std::uint32_t i = 0; i < k; ++i) {
    v.push_back(make_sample(first + i));
  }
  return v;
}

inline std::uint32_t bits_of(float f) {
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

inline SampleWords words_of(const marv_imu_meas& m) {
  return {bits_of(m.gyro_rad_s.x),  bits_of(m.gyro_rad_s.y),  bits_of(m.gyro_rad_s.z), bits_of(m.accel_m_s2.x),
          bits_of(m.accel_m_s2.y), bits_of(m.accel_m_s2.z), bits_of(m.temp_k),        m.flags};
}

// Output buffers filled with a sentinel so untouched entries are visible.
struct OutBuf {
  explicit OutBuf(std::uint32_t capacity)
      : cap(capacity), t(capacity, kSentinel64), dshot(static_cast<std::size_t>(capacity) * kMotors, kSentinel16) {}

  marv_sil_out out() {
    marv_sil_out o{};
    o.struct_size = sizeof(marv_sil_out);
    o.capacity_ticks = cap;
    o.t_us = t.data();
    o.dshot = dshot.data();
    o.servo_us = nullptr;
    return o;
  }

  bool untouched() const {
    for (const std::uint64_t v : t) {
      if (v != kSentinel64) {
        return false;
      }
    }
    for (const std::uint16_t v : dshot) {
      if (v != kSentinel16) {
        return false;
      }
    }
    return true;
  }

  std::uint32_t cap;
  std::vector<std::uint64_t> t;
  std::vector<std::uint16_t> dshot;
};

inline std::uint64_t reference_stamp(std::uint32_t num_us, std::uint32_t den, std::uint64_t n) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(n) * num_us / den);
}

inline std::uint32_t scaled(float v) { return v > 0.0f ? static_cast<std::uint32_t>(v * 1000.0f) : 0u; }
inline std::uint32_t nonneg(std::int32_t v) { return v > 0 ? static_cast<std::uint32_t>(v) : 0u; }

// Expected DShot rows of the test composition in normal mode (see composition.cpp).
struct Model {
  std::array<std::uint16_t, kMotors> latch{};
  float f32 = 0.75f;
  std::int32_t i32 = 4;

  std::array<std::uint16_t, kMotors> step(std::uint64_t n, const marv_imu_meas& m) {
    constexpr std::uint64_t hold = 7;
    if (n != 0 && n % hold != hold - 1) {
      latch = {clamp_code(static_cast<std::uint32_t>(n % 1000)), sample_code(words_of(m)), clamp_code(scaled(f32)),
               clamp_code(nonneg(i32))};
    }
    return latch;
  }
};

// Checks rows [0, k) of `buf` for ticks first..first+k-1 against the period and the model.
inline void expect_rows(const OutBuf& buf, std::uint64_t first, const std::vector<marv_imu_meas>& samples,
                        std::uint32_t num_us, std::uint32_t den, Model& model) {
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const std::uint64_t n = first + i;
    EXPECT_EQ(buf.t[i], reference_stamp(num_us, den, n)) << "tick " << n;
    const auto want = model.step(n, samples[i]);
    for (std::size_t m = 0; m < kMotors; ++m) {
      EXPECT_EQ(buf.dshot[(i * kMotors) + m], want[m]) << "tick " << n << " motor " << m;
    }
  }
}

}  // namespace siltest
