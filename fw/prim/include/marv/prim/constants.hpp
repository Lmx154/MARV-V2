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

}  // namespace marv::prim
// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
