#pragma once

// Cited constants of the Gazebo lockstep plugin. sim/ is outside gate G1, but the number rule holds: every number here
// carries its citation and a kind.
#include <cstddef>
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

// Bytes of a SHA-256 message digest. Citation: NIST FIPS 180-4, "Secure Hash Standard (SHS)" (2015), section 1 (Figure 1:
// SHA-256 message digest size 256 bits), 256 / 8 = 32.
// Kind: standard.
inline constexpr std::size_t kSha256DigestBytes = 32;

// The plant's noise stream of the primary IMU, logged in record 6 (decision 0019). Citation: sim/plant/include/
// marv_plant.h, the generic-IMU block ("draws from the plant's seeded noise stream 0 (the primary IMU)").
// Kind: interface.
inline constexpr std::uint32_t kImuNoiseStreamId = 0;

// The IMU sample counter at the first tick, logged in record 6 (decision 0019). Rule: the adapter takes one IMU sample per
// tick from tick 0 (sim/gz/adapter/include/marv/gz/adapter.hpp), so sample k is tick k.
// Kind: derived.
inline constexpr std::uint64_t kImuCounterBase = 0;

}  // namespace marv::gz
