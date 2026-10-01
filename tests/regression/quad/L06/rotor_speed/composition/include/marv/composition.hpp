// Test-only composition for the rotor-speed SIL tests (L6 stage (b), decision 0013): what a composition tells the SIL entry
// about itself, and the probe that records what the HAL pull read gave on every tick. The probe lives in the test's own
// process: the SIL entry is compiled into the test executable (see CMakeLists.txt), so the test reads the recording
// directly.
#pragma once

#include <array>
#include <cstddef>

#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/rotor_speed_sample.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "rotor-speed-probe-test-composition";

void init() noexcept;
void tick(const ImuSample& s) noexcept;

}  // namespace marv::composition

namespace marv::rotor_probe {

// Capacity of the recording: a labelled test bound, above the ticks any test here runs.
inline constexpr std::size_t kCapacity = 16;

struct Seen {
  RotorSpeedSample rotor;  // what hal_rotor_speed() returned inside the tick
  TimeUs imu_t_us;         // the stamp of the ImuSample the tick was given
  TimeUs hal_time_us;      // hal_time_us() inside the tick
};

[[nodiscard]] std::size_t count() noexcept;
[[nodiscard]] const Seen& at(std::size_t i) noexcept;
void reset() noexcept;

}  // namespace marv::rotor_probe
