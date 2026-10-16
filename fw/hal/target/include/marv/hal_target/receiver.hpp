#pragma once

// The ExpressLRS receiver link on the FC3v2 (M2 packet P5b, decision 0040 Q3; fw/hal/target/src/receiver.cpp). CRSF on SER0
// (UART0, GPIO 0 and 1, boards/madflight_fc3v2_revc.h) at kCrsfBaudElrs, 8N1. The UART's RX interrupt, at an IRQ priority
// strictly lower than the IMU interrupt's, feeds each byte to the CrsfDecoder; when its RC-frame counter changes the
// handler converts the channels (rc_input.hpp), stamps the frame with hal_time_us() and publishes the ManualControl
// through manual_control_publish. The handler is the one writer of the slot. Nothing is printed in the interrupt; the
// background loop reads the counters through a seqlock.

#include <cstdint>

#include "marv/hal_target/crsf.hpp"
#include "marv/hal_target/rc_input.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::hal_target {

struct ReceiverBoot {
  std::uint32_t baud_requested = 0;
  std::uint32_t baud_actual = 0;  // what the UART's divider gives (uart_init)
  std::uint32_t irq_priority = 0;  // the NVIC priority value set on the UART interrupt (larger is lower)
};

struct ReceiverStats {
  CrsfCounters counters{};
  std::uint32_t uart_errors = 0;  // received words with the overrun, break, parity or framing error flag set
  ManualControl last{};           // the newest published ManualControl (frames == 0: none yet)
};

// Configures the UART and enables its RX interrupt. hal_panic if `map` is not valid (rc_channel_map_valid) or the UART
// cannot be set to the baud rate. Call once, from main, before the IMU interrupt is enabled.
ReceiverBoot receiver_start(const RcChannelMap& map) noexcept;

// A consistent snapshot of the counters (a seqlock read; never blocks the interrupt).
[[nodiscard]] ReceiverStats receiver_stats() noexcept;

}  // namespace marv::hal_target
