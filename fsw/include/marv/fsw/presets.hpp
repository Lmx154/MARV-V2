// Factory setups: a named estimator choice with every parameter at its params.def default. kFactory[id] is the
// Setup that link::LoadFactory and link::SetPreset stage (factory 0 for an unknown id); Telemetry::preset reports the
// factory id the running Setup equals, else 0xFF.
#pragma once

#include <cstdint>

#include <marv/fsw/params.hpp>

namespace marv {

enum class EstimatorKind : std::uint8_t {
    kEskf,
    kEkf,
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
    {2, "mahony", EstimatorKind::kMahony, ControllerKind::kCascadedPid},
    {3, "complementary", EstimatorKind::kComplementary, ControllerKind::kCascadedPid},
};

inline constexpr std::uint8_t kPresetCount = sizeof(kPresets) / sizeof(kPresets[0]);

// The preset with this id, or preset 0.
constexpr const Preset& preset_or_default(std::uint8_t id) { return id < kPresetCount ? kPresets[id] : kPresets[0]; }

static_assert(static_cast<std::uint8_t>(EstimatorKind::kEskf) == param::k_estimator_eskf &&
                  static_cast<std::uint8_t>(EstimatorKind::kEkf) == param::k_estimator_ekf &&
                  static_cast<std::uint8_t>(EstimatorKind::kMahony) == param::k_estimator_mahony &&
                  static_cast<std::uint8_t>(EstimatorKind::kComplementary) == param::k_estimator_complementary,
              "EstimatorKind is the estimator kind index of params.def");

// Every family at kind 0 but the estimator, every parameter at its default.
constexpr param::Setup factory_setup(std::uint8_t estimator) {
    param::Setup s{};
    s.kind[param::k_estimator] = estimator;
    for (std::uint16_t i = 0; i < param::kParamCount; ++i) s.values[i] = param::kParamMeta[i].dflt;
    return s;
}

inline constexpr param::Setup kFactory[] = {
    factory_setup(param::k_estimator_eskf),
    factory_setup(param::k_estimator_ekf),
    factory_setup(param::k_estimator_mahony),
    factory_setup(param::k_estimator_complementary),
};
static_assert(sizeof(kFactory) / sizeof(kFactory[0]) == kPresetCount, "one factory setup per preset");

}  // namespace marv
