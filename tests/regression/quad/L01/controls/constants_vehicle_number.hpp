#pragma once

// Negative control for tools/ci/check_constants.py: a vehicle number citing the card. Never compiled.
namespace marv::prim {

// Motor pole count. Citation: vehicles/uzh_neurobem_5in.yaml, the vehicle card.
// Kind: physics.
inline constexpr int kMotorPoleCount = 14;

}  // namespace marv::prim
