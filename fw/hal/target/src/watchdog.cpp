#include "marv/hal_target/watchdog.hpp"

#include <atomic>

#include <hardware/regs/watchdog.h>
#include <hardware/watchdog.h>

#include "marv/hal/hal.hpp"
#include "marv/prim/constants.hpp"

namespace marv::hal_target {

namespace {
std::atomic<bool> g_started{false};
}  // namespace

WatchdogBoot watchdog_start(std::int32_t timeout_ms) noexcept {
  // The SDK's limit: the countdown register is WATCHDOG_LOAD_BITS microseconds wide (hardware_watchdog watchdog.h: "On
  // RP2350 the maximum delay is 16777 milliseconds").
  constexpr std::uint32_t kMaxMs = WATCHDOG_LOAD_BITS / prim::kMicrosecondsPerMillisecond;
  if (timeout_ms < 1 || static_cast<std::uint32_t>(timeout_ms) > kMaxMs) {
    hal_panic("watchdog_timeout_ms is outside 1 to the RP2350 watchdog maximum");
  }
  WatchdogBoot boot;
  boot.timeout_ms = timeout_ms;
  boot.last_reset_by_watchdog = watchdog_enable_caused_reboot();
  watchdog_enable(static_cast<std::uint32_t>(timeout_ms), false);
  g_started.store(true, std::memory_order_release);
  return boot;
}

void watchdog_feed() noexcept {
  if (g_started.load(std::memory_order_acquire)) {
    watchdog_update();
  }
}

}  // namespace marv::hal_target
