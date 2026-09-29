#pragma once

#include <cstddef>

// The only file under fw/ exempt from gate G1 (no numeric literals other than 0, 1, 2 and 0.5).
// Every constant here carries its citation.
// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
namespace marv::prim {

// Dimension of physical space. Citation: the definition of physical 3-D Euclidean space, which has three
// dimensions.
inline constexpr std::size_t kSpatialDim = 3;

}  // namespace marv::prim
// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
