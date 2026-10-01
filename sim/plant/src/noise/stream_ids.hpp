// Noise stream ids of the plant (quad spec L6 stage (a); decision 0012). These numbers are ABI: they select a stream of
// the seeded generator (noise.hpp), so renumbering one changes every recorded run. Never renumber, never reuse; add new
// sensors at the next free number.
#pragma once

#include <cstdint>

namespace marv::plant::noise {

inline constexpr std::uint16_t kStreamPrimaryImu = 0;

}  // namespace marv::plant::noise
