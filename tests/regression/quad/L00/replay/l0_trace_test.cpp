// L0 T2 pass bar (quad spec section 4): the fixed L0 composition runs N ticks through the SIL entry point (in the
// null-plant runner, one process per run); the hash of its per-tick trace equals a committed golden. Negative
// controls: a perturbed sample, a perturbed parameter and a perturbed tick period each change the hash.
#include <gtest/gtest.h>

#include <sys/wait.h>

#include <cstdio>
#include <fstream>
#include <regex>
#include <string>

#include "marv/null_plant/trace.hpp"

#ifndef MARV_L0_RUNNER
#error "MARV_L0_RUNNER must name the marv_null_plant_run executable"
#endif
#ifndef MARV_L0_GOLDEN
#error "MARV_L0_GOLDEN must name golden_l0_trace.txt"
#endif

namespace {

struct RunResult {
  int exit_code;
  std::string out;
};

RunResult run(const std::string& args) {
  const std::string cmd = std::string{"\""} + MARV_L0_RUNNER + "\" " + args;
  std::FILE* p = popen(cmd.c_str(), "r");
  EXPECT_NE(p, nullptr) << cmd;
  RunResult r{-1, {}};
  if (p == nullptr) {
    return r;
  }
  char buf[256];
  while (std::fgets(buf, sizeof buf, p) != nullptr) {
    r.out += buf;
  }
  const int status = pclose(p);
  r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return r;
}

// The hash a run printed: exactly 16 lower-case hex digits and a newline; the run must have exited 0.
std::string hash_of(const std::string& args) {
  const RunResult r = run(args);
  EXPECT_EQ(r.exit_code, 0) << args;
  EXPECT_TRUE(std::regex_match(r.out, std::regex{"[0-9a-f]{16}\n"})) << "output: " << r.out;
  return r.out.substr(0, r.out.find('\n'));
}

// The committed hash: the first line that is neither empty nor a comment.
std::string golden() {
  std::ifstream f{MARV_L0_GOLDEN};
  EXPECT_TRUE(f.is_open()) << MARV_L0_GOLDEN;
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') {
      return line;
    }
  }
  ADD_FAILURE() << "no hash line in " << MARV_L0_GOLDEN;
  return {};
}

TEST(L0Trace, MatchesGolden) {
  const std::string g = golden();
  EXPECT_TRUE(std::regex_match(g, std::regex{"[0-9a-f]{16}"})) << g;
  EXPECT_EQ(hash_of(""), g);
}

TEST(L0Trace, RunsAreDeterministic) {
  EXPECT_EQ(hash_of(""), hash_of(""));
}

TEST(L0Trace, BatchingDoesNotChangeTheHash) {
  EXPECT_EQ(hash_of("--batch 1"), golden());
}

// Negative controls (core 7.2). Each perturbation must change the hash of an otherwise identical run.
TEST(L0TraceNegativeControl, PerturbedSampleChangesTheHash) {
  const std::string g = golden();
  EXPECT_NE(hash_of("--flip-bit 1000:gx:22"), g);
  EXPECT_NE(hash_of("--flip-bit 1000:ax:22"), g);
  EXPECT_NE(hash_of("--flip-bit 1008:temp:22"), g);
}

TEST(L0TraceNegativeControl, PerturbedFloatParameterChangesTheHash) {
  EXPECT_NE(hash_of("--set l0_gain_rate=0.09"), golden());
}

TEST(L0TraceNegativeControl, PerturbedRateGroupDivisorChangesTheHash) {
  EXPECT_NE(hash_of("--set l0_div_mid=5"), golden());
  EXPECT_NE(hash_of("--set l0_div_slow=17"), golden());
}

TEST(L0TraceNegativeControl, PerturbedTimingChangesTheHash) {
  EXPECT_NE(hash_of("--period 12500/3"), golden());
}

// The runner's own checks, tested on both sides of every boundary (they gate every run above).
TEST(NullPlantChecks, DshotValueBoundaries) {
  using marv::null_plant::dshot_value_ok;
  EXPECT_TRUE(dshot_value_ok(0));
  EXPECT_FALSE(dshot_value_ok(1));
  EXPECT_FALSE(dshot_value_ok(47));
  EXPECT_TRUE(dshot_value_ok(48));
  EXPECT_TRUE(dshot_value_ok(2047));
  EXPECT_FALSE(dshot_value_ok(2048));
  EXPECT_FALSE(dshot_value_ok(65535));
}

TEST(NullPlantChecks, ReferenceStampIsTheExactFloor) {
  using marv::null_plant::reference_stamp_us;
  const std::uint64_t expected[] = {0, 156, 312, 468, 625, 781, 937, 1093, 1250};  // floor(n * 625 / 4)
  for (std::uint64_t n = 0; n < 9; ++n) {
    EXPECT_EQ(reference_stamp_us(625, 4, n), expected[n]) << n;
  }
  EXPECT_EQ(reference_stamp_us(12500, 3, 1), 4166U);  // floor(12500 / 3)
  EXPECT_EQ(reference_stamp_us(12500, 3, 2), 8333U);
  EXPECT_EQ(reference_stamp_us(1, 1, UINT64_C(1) << 40), UINT64_C(1) << 40);
}

TEST(NullPlantRunner, RejectsAnUnknownParameterName) {
  const RunResult r = run("--set no_such_parameter=1 2>/dev/null");
  EXPECT_NE(r.exit_code, 0);
  EXPECT_TRUE(r.out.empty());
}

}  // namespace
