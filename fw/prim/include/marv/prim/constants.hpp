#pragma once

#include <cstddef>
#include <cstdint>

// The only file under fw/ exempt from gate G1 (no numeric literals other than 0, 1, 2 and 0.5).
// Every constant here carries its citation.
// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
namespace marv::prim {

// Dimension of physical space. Citation: the definition of physical 3-D Euclidean space, which has three
// dimensions.
inline constexpr std::size_t kSpatialDim = 3;

// Smallest and largest DShot throttle value. Citation: Betaflight DShot notes
// (https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/Dshot.mdx): 0 is reserved for
// disarmed, 1-47 are special commands, 48-2047 is throttle, the field is 11 bits wide.
inline constexpr std::uint16_t kDshotThrottleMin = 48;
inline constexpr std::uint16_t kDshotThrottleMax = 2047;

// Pi. Citation: the mathematical constant, the ratio of a circle's circumference to its diameter.
inline constexpr double kPi = 3.14159265358979323846;

// Seconds per minute. Citation: SI definition (the minute is exactly 60 s, an accepted non-SI unit of time).
inline constexpr double kSecondsPerMinute = 60.0;

// WGS 84 normal-gravity constants. Citation for every entry below: NIMA TR8350.2, "Department of Defense World
// Geodetic System 1984", 3rd ed., 4 July 1997, incl. Amendment 1 (3 January 2000) ("NIMA TR8350.2 3rd ed. Amdt 1
// (2000)").
// Semi-major axis a, m. Table 3.1.
inline constexpr double kWgs84A = 6378137.0;
// Inverse flattening 1/f, defining parameter. Table 3.1.
inline constexpr double kWgs84InvF = 298.257223563;
// Flattening f. Derived: f = 1 / (1/f) from the defining parameter above.
inline constexpr double kWgs84F = 1.0 / kWgs84InvF;
// First eccentricity squared e^2. Table 3.3.
inline constexpr double kWgs84E2 = 6.69437999014e-3;
// Normal gravity at the equator gamma_e, m/s^2. Table 3.4.
inline constexpr double kWgs84GammaE = 9.7803253359;
// Somigliana's constant k = b gamma_p / (a gamma_e) - 1. Table 3.4.
inline constexpr double kWgs84K = 0.00193185265241;
// m = omega^2 a^2 b / GM. Table 3.4.
inline constexpr double kWgs84M = 0.00344978650684;
// Coefficient of the h^2 term of the height series, gamma_h = gamma [1 - (2/a)(1 + f + m - 2 f sin^2 phi) h +
// (3/a^2) h^2]. Eq. (4-3).
inline constexpr double kWgs84HeightQuadCoeff = 3.0;

}  // namespace marv::prim
// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
