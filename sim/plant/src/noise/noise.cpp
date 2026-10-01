#include "noise/noise.hpp"

#include <cmath>

#include "noise/musl_math.h"

namespace marv::plant::noise {

namespace {

// 2 pi, the double nearest to it (0x1.921fb54442d18p+2 = 6.283185307179586...): a rounding of a mathematical constant,
// not a measured number.
constexpr double kTwoPi = 0x1.921fb54442d18p+2;

}  // namespace

double normal(std::uint64_t seed, std::uint16_t id, std::uint64_t k) noexcept {
  const double u1 = uniform_open_closed(draw(seed, id, 2U * k));
  const double u2 = uniform_closed_open(draw(seed, id, 2U * k + 1U));
  // sqrt is correctly rounded by IEEE 754; -2 ln u1 >= 0 because u1 is in (0, 1].
  const double radius = std::sqrt(-2.0 * marv_musl_log(u1));
  return radius * marv_musl_cos(kTwoPi * u2);
}

SampleNormals sample_normals(std::uint64_t seed, std::uint16_t id, std::uint64_t sample) noexcept {
  SampleNormals out{};
  for (std::uint64_t j = 0; j < kNormalsPerSample; ++j) {
    out[j] = normal(seed, id, kNormalsPerSample * sample + j);
  }
  return out;
}

}  // namespace marv::plant::noise
