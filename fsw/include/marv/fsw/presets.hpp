// Module presets: a named combination of flight-software modules, chosen at runtime (the avionics toolbox lab's
// swappable blocks). The id is what link::SetPreset carries and what Telemetry::preset reports; an unknown id runs
// preset 0.
#pragma once

#include <cstdint>

namespace marv {

enum class EstimatorKind : std::uint8_t {
    kEskf,
    kEkf,
    kUkf,
    kMahony,
    kComplementary,
};

enum class ControllerKind : std::uint8_t {
    kCascadedPid,
};

struct Preset {
    std::uint8_t id;
    const char* name;
    EstimatorKind estimator;
    ControllerKind controller;
};

inline constexpr Preset kPresets[] = {
    {0, "eskf", EstimatorKind::kEskf, ControllerKind::kCascadedPid},
    {1, "ekf", EstimatorKind::kEkf, ControllerKind::kCascadedPid},
    {2, "ukf", EstimatorKind::kUkf, ControllerKind::kCascadedPid},
    {3, "mahony", EstimatorKind::kMahony, ControllerKind::kCascadedPid},
    {4, "complementary", EstimatorKind::kComplementary, ControllerKind::kCascadedPid},
};

inline constexpr std::uint8_t kPresetCount = sizeof(kPresets) / sizeof(kPresets[0]);

// The preset with this id, or preset 0.
constexpr const Preset& preset_or_default(std::uint8_t id) { return id < kPresetCount ? kPresets[id] : kPresets[0]; }

}  // namespace marv
