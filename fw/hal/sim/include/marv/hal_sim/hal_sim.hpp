#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "marv/types/actuator.hpp"
#include "marv/types/rotor_speed_sample.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_sim {

struct TickPeriod {
  std::uint32_t num_us;  // period = num_us / den microseconds
  std::uint32_t den;     // den >= 1, num_us >= den
};

[[nodiscard]] constexpr bool period_valid(TickPeriod p) noexcept { return p.den >= 1 && p.num_us >= p.den; }

// Exact floor(n * num / den) without 128-bit arithmetic.
[[nodiscard]] constexpr TimeUs stamp_us(TickPeriod p, Tick n) noexcept {
  return (n / p.den) * p.num_us + ((n % p.den) * p.num_us) / p.den;
}

// Time 0; latches are set to stop / no pulse and bound for hal_actuators_write.
void setup(TickPeriod p, std::span<DshotValue> motor_latch, std::span<ServoUs> servo_latch) noexcept;

// pre: 0 first, then previous + 1 (else hal_panic). time := stamp_us(p, n); clears the per-tick write flag.
void begin_tick(Tick n) noexcept;

// Stages the rotor-speed sample that hal_rotor_speed() returns from the next begin_tick on (its t_us is the tick's); it
// replaces any earlier one. Never staged: every valid bit clear and every speed 0. pre: outside a tick (else hal_panic).
void stage_rotor_speed(const std::array<float, kQuadXMotors>& omega_rad_s, std::uint32_t flags) noexcept;

// Marks "outside a tick" so a later write panics.
void end_tick() noexcept;

}  // namespace marv::hal_sim
