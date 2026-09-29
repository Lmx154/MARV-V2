#pragma once

// Negative control for tools/ci/check_constants.py: a constant with no citation and no kind. Never compiled.
namespace marv::prim {

// Standard gravity as a bare number.
inline constexpr double kPlantedUncited = 9.80665;

}  // namespace marv::prim
