#pragma once

// Cited constants of the Gazebo lockstep plugin. sim/ is outside gate G1, but the number rule holds: every number here
// carries its citation and a kind.
#include <cstdint>

namespace marv::gz {

// Nanoseconds per microsecond. Citation: SI prefixes nano = 10^-9 and micro = 10^-6 (BIPM, "The International System
// of Units (SI)", 9th ed. (2019), Table 7), so 1 us = 1000 ns.
// Kind: standard.
inline constexpr std::uint64_t kNanosecondsPerMicrosecond = 1000;

// Nanoseconds per second. Citation: SI prefix nano = 10^-9 (BIPM, "The International System of Units (SI)", 9th ed.
// (2019), Table 7). Exactly representable in binary64.
// Kind: standard.
inline constexpr double kNanosecondsPerSecond = 1000000000.0;

}  // namespace marv::gz
