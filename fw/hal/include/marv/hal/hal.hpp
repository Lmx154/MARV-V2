#pragma once

#include <cstddef>
#include <span>

#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv {

// Monotonic. hal_sim: 0 before tick 0; stamp(n) from the start of tick n until tick n+1 starts.
[[nodiscard]] TimeUs hal_time_us() noexcept;

// Only inside composition::tick, at most once per tick; span sizes must equal the bound latch sizes. Else hal_panic.
void hal_actuators_write(std::span<const DshotValue> motor, std::span<const ServoUs> servo) noexcept;

template <std::size_t M, std::size_t S>
inline void hal_actuators_write(const ActuatorOutput<M, S>& o) noexcept {
  hal_actuators_write(std::span<const DshotValue>{o.motor}, std::span<const ServoUs>{o.servo});
}

[[noreturn]] void hal_panic(const char* reason) noexcept;  // hal_sim: message to stderr + std::abort()

}  // namespace marv
