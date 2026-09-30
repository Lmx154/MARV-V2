#pragma once

// Cited constants of the Gazebo adapter (sim/gz). sim/ is outside gate G1, but the number rule holds: every number
// here carries its citation and a kind.
namespace marv::gz {

// Square root of two. Citation: the mathematical constant, the positive root of x^2 = 2, 1.41421356237309504880...
// (OEIS A002193). The literal below is rounded to nearest by the compiler, so kSqrt2 = fl(sqrt 2) with relative
// error <= kUnitRoundoff.
// Kind: math.
inline constexpr double kSqrt2 = 1.41421356237309504880;

// Unit roundoff u of IEEE 754 binary64 with round to nearest, u = 2^-53. Citation: IEEE 754-2019 section 3.3
// (binary64 has p = 53 significand digits) and Higham, "Accuracy and Stability of Numerical Algorithms", 2nd ed.
// (2002), section 2.1 (u = (1/2) beta^(1-p)).
// Kind: standard.
inline constexpr double kUnitRoundoff = 0x1p-53;

// Microseconds per second. Citation: SI prefix micro = 10^-6 (BIPM, "The International System of Units (SI)", 9th
// ed. (2019), Table 7).
// Kind: standard.
inline constexpr double kMicrosecondsPerSecond = 1000000.0;

}  // namespace marv::gz
