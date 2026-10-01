// L6 stage (a), T1 (decision 0012): the vendored musl log, sin and cos agree with the platform's (glibc) to at most
// 1 ulp over the ranges Box-Muller uses: log on (0, 1] (the argument -2 ln u1 takes u1 in (0, 1]) and sin/cos on
// [0, 2 pi) (the argument 2 pi u2, u2 in [0, 1)). The error against a long double reference is also bounded, so a
// failure is attributable. Only cos is on the stream path; sin is vendored because cos shares its reduction and kernel
// files and is checked with it.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "noise/musl_math.h"
#include "noise/noise.hpp"
#include "support.hpp"

namespace {

namespace noise = marv::plant::noise;

// The double nearest 2 pi, as in noise.cpp.
constexpr double kTwoPi = 0x1.921fb54442d18p+2;
// Labelled test values. kDraws: random-looking arguments, one per draw, the exact arguments the stream produces
// (2^20 covers every binade of u1 that occurs in practice). kPerBinade: points per binade of the log sweep.
// kGrid: grid points of the sin/cos sweep. kNeighbours: consecutive doubles either side of each multiple of pi/2,
// where the results cross zero or peak and a relative error is most sensitive.
constexpr std::uint64_t kDraws = 1U << 20U;
constexpr std::uint64_t kPerBinade = 4096;
constexpr std::uint64_t kGrid = 1U << 20U;
constexpr int kNeighbours = 1024;
constexpr std::uint64_t kSeed = 1;
// The bound under test: the packet's "<= 1 ulp of glibc".
constexpr std::uint64_t kMaxUlp = 1;

using Fn = double (*)(double);

struct Worst {
  std::uint64_t vs_glibc = 0;
  double true_err_ulp = 0.0;
  double at_glibc = 0.0;
};

void measure(Fn mine, Fn theirs, long double (*ref)(long double), double x, Worst& w) {
  const double a = mine(x);
  const double b = theirs(x);
  const std::uint64_t d = marv::l06::ulp_distance(a, b);
  if (d > w.vs_glibc) {
    w.vs_glibc = d;
    w.at_glibc = x;
  }
  const long double exact = ref(static_cast<long double>(x));
  const double rounded = static_cast<double>(exact);
  const double ulp = std::nextafter(std::fabs(rounded), INFINITY) - std::fabs(rounded);
  if (ulp > 0.0) {
    const double err = static_cast<double>(std::fabs(static_cast<long double>(a) - exact) / static_cast<long double>(ulp));
    w.true_err_ulp = std::max(w.true_err_ulp, err);
  }
}

double glibc_log(double x) { return std::log(x); }
double glibc_sin(double x) { return std::sin(x); }
double glibc_cos(double x) { return std::cos(x); }
long double ref_log(long double x) { return std::log(x); }
long double ref_sin(long double x) { return std::sin(x); }
long double ref_cos(long double x) { return std::cos(x); }

std::vector<double> log_arguments() {
  std::vector<double> xs;
  for (std::uint64_t n = 0; n < kDraws; ++n) {
    xs.push_back(noise::uniform_open_closed(noise::draw(kSeed, noise::kStreamPrimaryImu, 2U * n)));
  }
  // Every binade of (0, 1] from 2^-53 (the smallest u1) up, kPerBinade points each, evenly spaced in the significand.
  for (int e = -53; e < 0; ++e) {
    for (std::uint64_t i = 0; i < kPerBinade; ++i) {
      const double frac = 1.0 + static_cast<double>(i) / static_cast<double>(kPerBinade);
      xs.push_back(std::ldexp(frac, e));
    }
  }
  // Around 1, where musl's log switches branches (|x - 1| < about 2^-4): a fine grid below 1 and the doubles at 1. The
  // grid spans [1 - 1/8, 1], twice the 2^-4 half-width of the near-1 branch, so both sides of its edge are covered
  // (the stream's u1 is at most 1, so the side above 1 is not on the stream path).
  for (std::uint64_t i = 0; i <= kGrid; ++i) {
    xs.push_back(1.0 - static_cast<double>(i) / static_cast<double>(kGrid) / 8.0);
  }
  for (int i = 0; i < kNeighbours; ++i) {
    xs.push_back(1.0 - static_cast<double>(i) * 0x1p-53);
  }
  xs.push_back(1.0);
  return xs;
}

std::vector<double> angle_arguments() {
  std::vector<double> xs;
  for (std::uint64_t n = 0; n < kDraws; ++n) {
    xs.push_back(kTwoPi * noise::uniform_closed_open(noise::draw(kSeed, noise::kStreamPrimaryImu, 2U * n + 1U)));
  }
  for (std::uint64_t i = 0; i < kGrid; ++i) {
    xs.push_back(kTwoPi * static_cast<double>(i) / static_cast<double>(kGrid));
  }
  for (int quarter = 0; quarter <= 4; ++quarter) {
    // The multiples of pi/2 in [0, 2 pi]: 2 pi * quarter / 4 for quarter = 0..4 (four quarter turns).
    const double centre = kTwoPi * static_cast<double>(quarter) / 4.0;
    double up = centre;
    double down = centre;
    for (int i = 0; i < kNeighbours; ++i) {
      up = std::nextafter(up, INFINITY);
      down = std::nextafter(down, 0.0);
      xs.push_back(up);
      xs.push_back(down);
    }
    xs.push_back(centre);
  }
  xs.erase(std::remove_if(xs.begin(), xs.end(), [](double x) { return !(x >= 0.0 && x <= kTwoPi); }),
           xs.end());
  return xs;
}

Worst sweep(Fn mine, Fn theirs, long double (*ref)(long double), const std::vector<double>& xs) {
  Worst w;
  for (const double x : xs) {
    measure(mine, theirs, ref, x, w);
  }
  return w;
}

void report(const char* name, const Worst& w, std::size_t n) {
  std::cout << "[ULP] " << name << ": " << n << " arguments, max ulp vs glibc " << w.vs_glibc << " (at " << w.at_glibc
            << "), max error vs long double " << w.true_err_ulp << " ulp\n";
}

TEST(L06NoiseLibm, LogWithinOneUlpOfGlibcOnZeroToOne) {
  const std::vector<double> xs = log_arguments();
  for (const double x : xs) {
    ASSERT_GT(x, 0.0);
    ASSERT_LE(x, 1.0);
  }
  const Worst w = sweep(marv_musl_log, glibc_log, ref_log, xs);
  report("log (0,1]", w, xs.size());
  EXPECT_LE(w.vs_glibc, kMaxUlp);
  EXPECT_LE(w.true_err_ulp, 1.0);
}

TEST(L06NoiseLibm, SinWithinOneUlpOfGlibcOnZeroToTwoPi) {
  const std::vector<double> xs = angle_arguments();
  const Worst w = sweep(marv_musl_sin, glibc_sin, ref_sin, xs);
  report("sin [0,2pi)", w, xs.size());
  EXPECT_LE(w.vs_glibc, kMaxUlp);
  EXPECT_LE(w.true_err_ulp, 1.0);
}

TEST(L06NoiseLibm, CosWithinOneUlpOfGlibcOnZeroToTwoPi) {
  const std::vector<double> xs = angle_arguments();
  const Worst w = sweep(marv_musl_cos, glibc_cos, ref_cos, xs);
  report("cos [0,2pi)", w, xs.size());
  EXPECT_LE(w.vs_glibc, kMaxUlp);
  EXPECT_LE(w.true_err_ulp, 1.0);
}

// Control: the ulp metric is not vacuous. A one-ulp perturbation of musl's result is seen as distance 1, a
// two-ulp one as 2, so the bounds above can fail.
TEST(L06NoiseLibmControl, UlpDistanceSeesAPerturbedResult) {
  // Labelled test value: 0.7, a fixed argument in (0, 1) clear of the near-1 branch (|x - 1| = 0.3 > 2^-4) with a
  // nonzero log (about -0.357), so a one-ulp step of the result is a defined distance.
  const double x = 0.7;
  const double y = marv_musl_log(x);
  const double up1 = std::nextafter(y, INFINITY);
  const double up2 = std::nextafter(up1, INFINITY);
  EXPECT_EQ(marv::l06::ulp_distance(y, y), 0U);
  EXPECT_EQ(marv::l06::ulp_distance(y, up1), 1U);
  EXPECT_EQ(marv::l06::ulp_distance(y, up2), 2U);
  EXPECT_GT(marv::l06::ulp_distance(y, up2), kMaxUlp);
}

}  // namespace
