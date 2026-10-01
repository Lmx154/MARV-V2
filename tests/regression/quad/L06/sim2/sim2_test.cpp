// L6 stage (a), T1 SIM-2 with noise: the same seed gives a bit-identical run (quad spec section 4 L6 pass bar (a); SIM-2).
// Every run is a separate process of sim/l6_noise_run (adapter + marv_plant + generic IMU + the l4_rate_scripted SIL, the
// firmware seeing the noisy IMU bytes), so no state is shared between runs.
//
//  1. Two processes, same seed and corner, at corners -1, 0, +1: byte-identical logs, equal to the printed SHA-256.
//  2. Control that must fail: seed + 1 gives a different log, and the first difference after the header's seed field is
//     in the IMU bytes of tick 0.
//  3. Control: corner -1 against +1 at the same seed: the plant state differs while every firmware tick number and stamp
//     is identical (the clock error is applied to the plant only).
// Distinct noise streams being independent is the noise suite's (L06/noise).
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "log_format.hpp"
#include "marv/sim/sha256.hpp"

namespace {

namespace run = marv::l6run;
using Bytes = std::vector<std::uint8_t>;

// Labelled test values. Seed: any value. Ticks: 4096 x 156.25 us = 0.64 s of flight, long enough that the scripted roll
// step reaches the rate loop through the noisy gyro and the body rate builds (about 50 ms of wall time per run).
constexpr const char* kSeed = "20261001";
constexpr const char* kSeedPlusOne = "20261002";
constexpr const char* kTicks = "4096";
constexpr std::size_t kTickCount = 4096;

struct Dir {
  std::filesystem::path path;
  Dir() {
    std::string tmpl = (std::filesystem::temp_directory_path() / "l6_sim2_XXXXXX").string();
    EXPECT_NE(mkdtemp(tmpl.data()), nullptr);
    path = tmpl;
  }
  ~Dir() { std::filesystem::remove_all(path); }
};

struct DriverRun {
  int exit_code = -1;
  Bytes log;
  std::string printed_sha;
};

// fork + exec of the driver with stdout to a file; the log is the driver's output file.
DriverRun run_driver(const Dir& dir, const std::string& name, const char* seed, int corner) {
  DriverRun r;
  const std::string log_path = (dir.path / (name + ".bin")).string();
  const std::string out_path = (dir.path / (name + ".txt")).string();
  const std::string corner_text = std::to_string(corner);
  const pid_t pid = fork();
  if (pid == 0) {
    if (std::freopen(out_path.c_str(), "w", stdout) == nullptr) {
      _exit(126);
    }
    execl(MARV_L6_NOISE_RUN, MARV_L6_NOISE_RUN, seed, corner_text.c_str(), kTicks, log_path.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  EXPECT_EQ(waitpid(pid, &status, 0), pid);
  r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  std::ifstream log(log_path, std::ios::binary);
  r.log.assign(std::istreambuf_iterator<char>(log), std::istreambuf_iterator<char>());
  std::ifstream out(out_path);
  std::string word;
  out >> word >> r.printed_sha;
  EXPECT_EQ(word, "sha256");
  return r;
}

std::string sha_of(const Bytes& b) {
  marv::sim::Sha256 s;
  s.update(b.data(), b.size());
  return s.hex();
}

std::size_t record(std::size_t n) { return run::kHeaderBytes + n * run::kRecordBytes; }

std::uint64_t u64_at(const Bytes& b, std::size_t off) {
  std::uint64_t v = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    v |= static_cast<std::uint64_t>(b[off + i]) << (8U * i);
  }
  return v;
}

double f64_at(const Bytes& b, std::size_t off) {
  const std::uint64_t u = u64_at(b, off);
  double d = 0.0;
  std::memcpy(&d, &u, sizeof(d));
  return d;
}

// The log is well formed and the firmware is in the loop: right size and header, ticks in order, and the DShot of some
// tick differs from tick 0's (the rate loop answered the roll step) while the body rate about x ends positive.
void expect_real_run(const DriverRun& r, const char* seed, int corner) {
  ASSERT_EQ(r.exit_code, 0);
  ASSERT_EQ(r.log.size(), run::kHeaderBytes + kTickCount * run::kRecordBytes);
  EXPECT_EQ(std::memcmp(r.log.data(), run::kMagic, sizeof(run::kMagic)), 0);
  EXPECT_EQ(u64_at(r.log, run::kHeaderSeedOffset), std::strtoull(seed, nullptr, 10));
  EXPECT_EQ(static_cast<std::int64_t>(u64_at(r.log, run::kHeaderCornerOffset)), corner);
  EXPECT_EQ(u64_at(r.log, run::kHeaderTicksOffset), kTickCount);
  bool dshot_moved = false;
  for (std::size_t n = 0; n < kTickCount; ++n) {
    ASSERT_EQ(u64_at(r.log, record(n) + run::kRecTickOffset), n);
    dshot_moved = dshot_moved || std::memcmp(&r.log[record(n) + run::kRecDshotOffset],
                                             &r.log[record(0) + run::kRecDshotOffset], run::kRecDshotBytes) != 0;
  }
  EXPECT_TRUE(dshot_moved);
  EXPECT_GT(f64_at(r.log, record(kTickCount - 1) + run::kRecBodyOffset + 4 * sizeof(double)), 0.0);
}

TEST(L6Sim2, SameSeedGivesByteIdenticalLogsInSeparateProcessesAtEveryClockCorner) {
  Dir dir;
  for (const int corner : {-1, 0, 1}) {
    const DriverRun a = run_driver(dir, "a" + std::to_string(corner), kSeed, corner);
    const DriverRun b = run_driver(dir, "b" + std::to_string(corner), kSeed, corner);
    expect_real_run(a, kSeed, corner);
    expect_real_run(b, kSeed, corner);
    EXPECT_EQ(a.log, b.log) << "corner " << corner;
    EXPECT_EQ(a.printed_sha, b.printed_sha) << "corner " << corner;
    EXPECT_EQ(a.printed_sha, sha_of(a.log)) << "corner " << corner;
    EXPECT_EQ(b.printed_sha, sha_of(b.log)) << "corner " << corner;
  }
}

// Control for the test above: a different seed must break it, first in the IMU bytes of tick 0.
TEST(L6Sim2, ControlADifferentSeedGivesADifferentLogWhoseFirstDifferenceIsTick0ImuBytes) {
  Dir dir;
  const DriverRun a = run_driver(dir, "a", kSeed, 0);
  const DriverRun b = run_driver(dir, "b", kSeedPlusOne, 0);
  expect_real_run(a, kSeed, 0);
  expect_real_run(b, kSeedPlusOne, 0);
  ASSERT_EQ(a.log.size(), b.log.size());
  EXPECT_NE(a.printed_sha, b.printed_sha);
  // The header differs in the seed field and nowhere else.
  for (std::size_t i = 0; i < run::kHeaderBytes; ++i) {
    const bool in_seed = i >= run::kHeaderSeedOffset && i < run::kHeaderSeedOffset + sizeof(std::uint64_t);
    if (!in_seed) {
      EXPECT_EQ(a.log[i], b.log[i]) << "header byte " << i;
    }
  }
  EXPECT_NE(u64_at(a.log, run::kHeaderSeedOffset), u64_at(b.log, run::kHeaderSeedOffset));
  const auto mismatch = std::mismatch(a.log.begin() + static_cast<std::ptrdiff_t>(run::kHeaderBytes), a.log.end(),
                                      b.log.begin() + static_cast<std::ptrdiff_t>(run::kHeaderBytes));
  ASSERT_NE(mismatch.first, a.log.end());
  const std::size_t first = static_cast<std::size_t>(mismatch.first - a.log.begin());
  EXPECT_GE(first, record(0) + run::kRecImuOffset);
  EXPECT_LT(first, record(0) + run::kRecImuOffset + run::kRecImuBytes);
}

// Control: the clock error reaches the plant and not the firmware.
TEST(L6Sim2, ControlClockCornersMinusAndPlusDifferInThePlantStateAndNotInTheFirmwareStamps) {
  Dir dir;
  const DriverRun lo = run_driver(dir, "lo", kSeed, -1);
  const DriverRun hi = run_driver(dir, "hi", kSeed, 1);
  expect_real_run(lo, kSeed, -1);
  expect_real_run(hi, kSeed, 1);
  ASSERT_EQ(lo.log.size(), hi.log.size());
  EXPECT_NE(lo.printed_sha, hi.printed_sha);
  std::size_t body_differs = 0;
  for (std::size_t n = 0; n < kTickCount; ++n) {
    EXPECT_EQ(u64_at(lo.log, record(n) + run::kRecTickOffset), u64_at(hi.log, record(n) + run::kRecTickOffset));
    EXPECT_EQ(u64_at(lo.log, record(n) + run::kRecStampOffset), u64_at(hi.log, record(n) + run::kRecStampOffset))
        << "tick " << n;
    body_differs += std::memcmp(&lo.log[record(n) + run::kRecBodyOffset], &hi.log[record(n) + run::kRecBodyOffset],
                                run::kRecBodyBytes) != 0
                        ? 1U
                        : 0U;
  }
  EXPECT_GT(body_differs, 0U);
}

}  // namespace
