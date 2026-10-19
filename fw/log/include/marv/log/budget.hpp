// The ring's sizing rule (core EMB-6: "The RAM log buffer is at least the log rate x the card's worst measured SD write
// stall"; invariant L-5) and the pre-roll derived from it. Pure arithmetic, usable in constant expressions.
//
// The fast stream is what the tick context pushes: one Imu record per tick, one RateTick record per rate-loop tick and one
// ManualControl record per RC frame. The ring holds capacity - 1 cells. The stall is the design-budget entry
// sd_log_write_stall_max_s (design/budget.yaml): while the card is busy for that long the consumer cannot drain, so the
// ring must hold that much of the stream. What the ring holds beyond it is the history the logger may keep while idle: the
// pre-roll, the stream kept before the arm edge.
#pragma once

#include <cstddef>
#include <cstdint>

#include "marv/log/format.hpp"
#include "marv/prim/constants.hpp"

namespace marv::log {

struct FastRates {
  double imu_hz = 0.0;        // Imu records per second (the tick rate)
  double rate_tick_hz = 0.0;  // RateTick records per second
  double manual_hz = 0.0;     // ManualControl records per second
};

// The tick rate from the tick period tick_num_us / tick_den (us), the rate-loop divisor and the RC frame rate.
[[nodiscard]] constexpr FastRates fast_rates(std::uint32_t tick_num_us, std::uint32_t tick_den, std::uint32_t rate_divisor,
                                             double manual_hz) noexcept {
  const double imu = prim::kMicrosecondsPerSecond * static_cast<double>(tick_den) / static_cast<double>(tick_num_us);
  return FastRates{imu, imu / static_cast<double>(rate_divisor), manual_hz};
}

[[nodiscard]] constexpr double fast_bytes_per_s(const FastRates& r) noexcept {
  return r.imu_hz * static_cast<double>(imu::kBytes) + r.rate_tick_hz * static_cast<double>(kCellBytes) +
         r.manual_hz * static_cast<double>(manual::kBytes);
}

[[nodiscard]] constexpr double ring_usable_bytes(std::size_t ring_cells) noexcept {
  return ring_cells == 0 ? 0.0 : static_cast<double>((ring_cells - 1) * kCellBytes);
}

// L-5: ring >= rate x stall.
[[nodiscard]] constexpr bool ring_covers_stall(std::size_t ring_cells, const FastRates& r, double stall_s) noexcept {
  return ring_usable_bytes(ring_cells) >= fast_bytes_per_s(r) * stall_s;
}

// The most the ring may hold while idle, in cells: what is left after a stall's worth (0 if the ring does not cover it).
[[nodiscard]] constexpr std::size_t pre_roll_cells(std::size_t ring_cells, const FastRates& r, double stall_s) noexcept {
  const double spare = ring_usable_bytes(ring_cells) - fast_bytes_per_s(r) * stall_s;
  return spare > 0.0 ? static_cast<std::size_t>(spare / static_cast<double>(kCellBytes)) : 0;
}

}  // namespace marv::log
