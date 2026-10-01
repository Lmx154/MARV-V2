// Noise stream ids of the plant (quad spec L6 stage (a); decision 0012). These numbers are ABI: they select a stream of
// the seeded generator (noise.hpp), so renumbering one changes every recorded run. Never renumber, never reuse; add new
// sensors at the next free number.
#pragma once

#include <cstdint>

namespace marv::plant::noise {

inline constexpr std::uint16_t kStreamPrimaryImu = 0;
// L6 stage (b), decision 0013: the gyro vibration phases (vibration_model.hpp). Ids are ABI and are never renumbered.
inline constexpr std::uint16_t kVibration = 1;

}  // namespace marv::plant::noise
