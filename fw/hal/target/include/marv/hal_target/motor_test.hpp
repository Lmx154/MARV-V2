#pragma once

// The console motor test's hand-off on the FC3v2 (M1 packet P11, decision 0038; fw/hal/target/src/motor_test.cpp).
// Console to tick: the background loop (main) is the only writer of the request and the cancel signal, the IMU interrupt
// (the tick, hal_motor_test) the only reader. The writer fills the slot the reader is not using and then flips an atomic
// index, so the reader is never blocked, never sees a half-written snapshot and runs in the same time whether or not the
// console is active; a cancel or a request is seen by the first rate tick after the flip. Tick to console: the
// composition's events (hal_motor_test_report) go to the background loop through a seqlock.

#include <cstdint>

#include "marv/hal/motor_test.hpp"

namespace marv::hal_target {

// Console side (background loop only). Every received byte, the command stop and a closed USB port are cancels.
void motor_test_post_cancel(MotorTestCancel reason) noexcept;
void motor_test_post_request(const MotorTestRequest& request) noexcept;

// Console side: the latest event the composition reported and how many it reported in all (0: none yet). Events that came
// while the loop was busy are not queued: the count tells how many there were.
struct MotorTestStatus {
  std::uint32_t events = 0;
  MotorTestEvent last{};
};
[[nodiscard]] MotorTestStatus motor_test_status() noexcept;

}  // namespace marv::hal_target
