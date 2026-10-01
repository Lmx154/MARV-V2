// L6 stage (a), T1 (decision 0012; quad spec L6 pass bar "distinct streams are independent"): the sample
// cross-correlation of every pair of streams is within z / sqrt(n) at every lag 0..24, with z from the family-wise
// (Bonferroni) confidence over all pairs and lags at the budget's stream_independence_confidence (read from design/budget.yaml
// at configure time).
//
// Pairs: seed 1 against seed 2 (id 0); id 0 against id 1 (seed 1); the 12 per-sample channels against each other
// (66 pairs, seed 1, id 0, normal 12k + j of channel j). Draw-level pairs use the uniform of each draw, channel-level
// pairs the normals. For each pair rho_L = corr(a_i, b_{i+L}) for L = 0..24 and corr(b_i, a_{i+L}) for L = 1..24, so
// a shift of either stream against the other by up to 24 is seen (49 statistics per pair).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "budget_register.hpp"
#include "noise/noise.hpp"
#include "support.hpp"

namespace {

namespace noise = marv::plant::noise;

// Labelled test values. kSamples: series length n; the bound z / sqrt(n) is then about 0.018 at the pair/lag count
// below, so a correlation of that size would be seen, while a duplicate or shifted stream (rho about 1) is seen at any
// n; 2^16 keeps the 68 pairs cheap in debug. kMaxLag: the longest shift tested, 24, the packet's value (twice the 12
// channels' counter period). kSeedA, kSeedB: the two seeds. kIdA, kIdB: two stream ids (the primary IMU and the next
// number; any second id would do).
constexpr std::size_t kSamples = 1U << 16U;
constexpr std::size_t kMaxLag = 24;
constexpr std::uint64_t kSeedA = 1;
constexpr std::uint64_t kSeedB = 2;
constexpr std::uint16_t kIdA = noise::kStreamPrimaryImu;
constexpr std::uint16_t kIdB = 1;
// The control's shift: 12 draws, the packet's value (the naive "seed + id" derivation puts one stream a fixed number
// of draws from another).
constexpr std::uint64_t kControlShift = 12;

constexpr std::size_t kChannels = noise::kNormalsPerSample;
constexpr std::size_t kChannelPairs = kChannels * (kChannels - 1) / 2;
constexpr std::size_t kDrawPairs = 2;
constexpr std::size_t kLagsPerPair = 2 * kMaxLag + 1;
constexpr std::size_t kFamily = (kDrawPairs + kChannelPairs) * kLagsPerPair;

std::vector<double> draw_series(std::uint64_t seed, std::uint16_t id, std::uint64_t offset) {
  std::vector<double> u(kSamples);
  for (std::uint64_t n = 0; n < kSamples; ++n) {
    u[n] = noise::uniform_closed_open(noise::draw(seed, id, offset + n));
  }
  return u;
}

// The largest |rho| of a pair over its 49 lag statistics.
double max_abs_correlation(const std::vector<double>& a, const std::vector<double>& b) {
  double worst = 0.0;
  for (std::size_t lag = 0; lag <= kMaxLag; ++lag) {
    worst = std::fmax(worst, std::fabs(marv::l06::cross_correlation(a, b, lag)));
  }
  for (std::size_t lag = 1; lag <= kMaxLag; ++lag) {
    worst = std::fmax(worst, std::fabs(marv::l06::cross_correlation(b, a, lag)));
  }
  return worst;
}

double bound() {
  const double alpha = marv::l06::bonferroni_alpha(marv::l06::kStreamIndependenceConfidence, kFamily);
  return marv::l06::z_two_sided(alpha) / std::sqrt(static_cast<double>(kSamples));
}

TEST(L06NoiseIndependence, DistinctStreamsAndChannelsAreUncorrelated) {
  const double limit = bound();
  double worst = 0.0;
  const auto check = [&](const char* what, const std::vector<double>& a, const std::vector<double>& b) {
    const double r = max_abs_correlation(a, b);
    worst = std::fmax(worst, r);
    EXPECT_LE(r, limit) << what;
  };
  check("seed 1 vs seed 2, id 0", draw_series(kSeedA, kIdA, 0), draw_series(kSeedB, kIdA, 0));
  check("id 0 vs id 1, seed 1", draw_series(kSeedA, kIdA, 0), draw_series(kSeedA, kIdB, 0));

  std::vector<std::vector<double>> channel(kChannels, std::vector<double>(kSamples));
  for (std::uint64_t k = 0; k < kSamples; ++k) {
    const noise::SampleNormals s = noise::sample_normals(kSeedA, kIdA, k);
    for (std::size_t j = 0; j < kChannels; ++j) {
      channel[j][k] = s[j];
    }
  }
  for (std::size_t i = 0; i < kChannels; ++i) {
    for (std::size_t j = i + 1; j < kChannels; ++j) {
      check("channels", channel[i], channel[j]);
    }
  }
  std::cout << "[INDEPENDENCE] family " << kFamily << " tests, n " << kSamples << ", bound " << limit
            << ", worst |rho| " << worst << "\n";
}

// Control (a): stream B is stream A advanced by 12 draws, the naive derivation. The same statistic must exceed the
// same bound (it sees rho about 1 at lag 12 of the reverse-direction set).
TEST(L06NoiseIndependenceControl, StreamAdvancedByTwelveDrawsIsRejected) {
  const double r = max_abs_correlation(draw_series(kSeedA, kIdA, 0), draw_series(kSeedA, kIdA, kControlShift));
  std::cout << "[INDEPENDENCE] control shifted by " << kControlShift << ": max |rho| " << r << " (bound " << bound()
            << ")\n";
  EXPECT_GT(r, bound());
}

// Control (b): a duplicated key, the same stream twice.
TEST(L06NoiseIndependenceControl, DuplicatedKeyIsRejected) {
  const double r = max_abs_correlation(draw_series(kSeedA, kIdA, 0), draw_series(kSeedA, kIdA, 0));
  std::cout << "[INDEPENDENCE] control duplicated key: max |rho| " << r << " (bound " << bound() << ")\n";
  EXPECT_GT(r, bound());
}

}  // namespace
