#pragma once

// The RP2350 hardware watchdog of the FC3v2 image (quad spec §4.0 M1). The motors must not depend on the ESC's own
// signal-loss behaviour (bench, m1-bench-card.md step 3.3: with core 0 halted the AM32 ESC held its last throttle for
// 5 s or more), so a stalled rate tick resets the chip; the restarted firmware sends DShot stop frames at once
// (motor_output.hpp). Fed from the rate-execution path (main.cpp), not by the IMU interrupt alone and not by the idle loop.

#include <cstdint>

namespace marv::hal_target {

struct WatchdogBoot {
  std::int32_t timeout_ms = 0;
  bool last_reset_by_watchdog = false;  // the previous reset was a watchdog timeout (SDK watchdog_enable_caused_reboot)
};

// Reads the reset cause, then enables the watchdog with the debugger pause off: an SWD halt does not pause it. hal_panic
// when timeout_ms is below 1 or above the SDK's RP2350 maximum. Call once, after the motor output and the IMU tick run.
WatchdogBoot watchdog_start(std::int32_t timeout_ms) noexcept;

// Reloads the countdown; does nothing before watchdog_start. Safe from the interrupt and from hal_panic.
void watchdog_feed() noexcept;

}  // namespace marv::hal_target
