// The ExpressLRS receiver link (marv/hal_target/receiver.hpp). Pins and the UART instance: boards/madflight_fc3v2_revc.h.
// The PL011 raises its RX interrupt at 4 bytes in the FIFO (RXIFLSEL 0, uart_set_irqs_enabled) and its receive-timeout
// interrupt 32 bit periods after the last byte, so a short frame is not held in the FIFO; the 32-entry FIFO holds about
// 760 us of line at the baud rate.
#include "marv/hal_target/receiver.hpp"

#include <atomic>
#include <cstdint>

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/crsf.hpp"
#include "marv/hal_target/manual_control.hpp"
#include "marv/hal_target/rc_input.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::hal_target {
namespace {

constexpr unsigned kTxPin = MADFLIGHT_FC3V2_SER0_TX_PIN;
constexpr unsigned kRxPin = MADFLIGHT_FC3V2_SER0_RX_PIN;
// The writer runs strictly below the IMU interrupt (PICO_HIGHEST_IRQ_PRIORITY, lsm6dsv.cpp), so the IMU interrupt preempts
// it and it never preempts the IMU interrupt or the reader, and strictly above USB (PICO_DEFAULT_IRQ_PRIORITY), whose
// interrupt the M1 bench measured at up to 1871.63 us (tests/regression/quad/L09/results/p12_bench/report.txt, run b),
// longer than the 32-byte RX FIFO lasts at the baud rate. The value is halfway between the two in the SDK's priority
// encoding (a larger value is a lower priority, hardware/irq.h), so it is a level the part implements whenever the
// default is.
constexpr std::uint32_t kIrqPriority = (PICO_HIGHEST_IRQ_PRIORITY + PICO_DEFAULT_IRQ_PRIORITY) / 2;
static_assert(kIrqPriority > PICO_HIGHEST_IRQ_PRIORITY, "the receiver interrupt must be below the IMU interrupt");
static_assert(kIrqPriority < PICO_DEFAULT_IRQ_PRIORITY, "the receiver interrupt must be above the USB interrupt");
constexpr std::uint32_t kErrorBits =
    UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS;

// Written by receiver_start, then read by the handler only.
uart_inst_t* g_uart = nullptr;
RcChannelMap g_map{};

// Handler only.
CrsfDecoder g_decoder;
ManualControl g_current{};
std::uint32_t g_seen_rc = 0;
std::uint32_t g_seen_link = 0;
std::uint32_t g_errors = 0;

// The seqlock: g_seq is odd while the handler writes g_published.
std::atomic<std::uint32_t> g_seq{0};
ReceiverStats g_published{};

void publish_stats() noexcept {
  const std::uint32_t q = g_seq.load(std::memory_order_relaxed);
  g_seq.store(q + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  g_published.counters = g_decoder.counters();
  g_published.uart_errors = g_errors;
  g_published.last = g_current;
  g_seq.store(q + 2, std::memory_order_release);
}

// The valid-frame count the pull reports: it wraps and skips 0 (manual_control.hpp, ManualControl::frames).
void next_frame_count() noexcept {
  ++g_current.frames;
  if (g_current.frames == 0) {
    g_current.frames = 1;
  }
}

void on_byte(std::uint8_t b) noexcept {
  static_cast<void>(g_decoder.push(b));
  const CrsfCounters& c = g_decoder.counters();
  bool changed = false;
  if (c.rc_frames != g_seen_rc) {
    g_seen_rc = c.rc_frames;
    const TimeUs now = hal_time_us();
    if (crsf_to_controls(g_decoder.rc(), g_map, g_current)) {
      g_current.t_us = now;
      next_frame_count();
      changed = true;
    }
  }
  if (c.link_stats_frames != g_seen_link) {
    g_seen_link = c.link_stats_frames;
    g_current.link_quality = rc_link_quality(g_decoder.link().up_link_quality);
    g_current.link_t_us = hal_time_us();
    changed = true;
  }
  if (changed) {
    manual_control_publish(g_current);
  }
}

void on_uart() noexcept {
  uart_hw_t* const hw = uart_get_hw(g_uart);
  bool any = false;
  while (uart_is_readable(g_uart)) {
    const std::uint32_t word = hw->dr;
    if ((word & kErrorBits) != 0) {
      ++g_errors;
    }
    on_byte(static_cast<std::uint8_t>(word & UART_UARTDR_DATA_BITS));
    any = true;
  }
  if (any) {
    publish_stats();
  }
}

}  // namespace

ReceiverBoot receiver_start(const RcChannelMap& map) noexcept {
  if (!rc_channel_map_valid(map)) {
    hal_panic("the rc_channel_* parameters are not channel numbers 1 .. 16");
  }
  g_map = map;
  g_uart = uart_get_instance(MADFLIGHT_FC3V2_SER0_UART);
  ReceiverBoot boot;
  boot.baud_requested = prim::kCrsfBaudElrs;
  boot.baud_actual = uart_init(g_uart, boot.baud_requested);
  if (boot.baud_actual == 0) {
    hal_panic("the receiver UART has no clock");
  }
  gpio_set_function(kTxPin, UART_FUNCSEL_NUM(g_uart, kTxPin));
  gpio_set_function(kRxPin, UART_FUNCSEL_NUM(g_uart, kRxPin));
  uart_set_format(g_uart, prim::kCrsfUartDataBits, prim::kCrsfUartStopBits, UART_PARITY_NONE);
  uart_set_fifo_enabled(g_uart, true);
  const unsigned irq = UART_IRQ_NUM(g_uart);
  irq_set_exclusive_handler(irq, &on_uart);
  irq_set_priority(irq, kIrqPriority);
  irq_set_enabled(irq, true);
  uart_set_irqs_enabled(g_uart, true, false);
  boot.irq_priority = kIrqPriority;
  return boot;
}

ReceiverStats receiver_stats() noexcept {
  ReceiverStats s{};
  for (;;) {
    const std::uint32_t q = g_seq.load(std::memory_order_acquire);
    if ((q & 1U) != 0) {
      continue;
    }
    s = g_published;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (g_seq.load(std::memory_order_relaxed) == q) {
      break;
    }
  }
  return s;
}

}  // namespace marv::hal_target
