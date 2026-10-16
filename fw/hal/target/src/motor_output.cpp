#include "marv/hal_target/motor_output.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <pico.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/dshot.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {
namespace {

// The motor pads are consecutive GPIOs, so one state machine drives them as a 4-bit output group.
constexpr unsigned kPadBase = MADFLIGHT_FC3V2_OUT0_PIN;
static_assert(MADFLIGHT_FC3V2_OUT1_PIN == kPadBase + 1 && MADFLIGHT_FC3V2_OUT2_PIN == kPadBase + 2 &&
                  MADFLIGHT_FC3V2_OUT3_PIN == kPadBase + kMotorPads - 1,
              "the motor pads are not consecutive GPIOs");

// Written by motor_output_start before the first tick; after that the tick (hal_actuators_write) is the only writer,
// until hal_panic disables interrupts and takes over.
PIO g_pio = nullptr;
unsigned g_sm = 0;
bool g_started = false;
PadMap g_pad{};
TimeUs g_stop_interval_us = 0;
TimeUs g_next_stop_us = 0;
TimeUs g_busy_until_us = 0;
std::uint64_t g_dropped = 0;

// Pushes the words of one frame if the state machine is idle: the last frame has had its duration plus one frame time of
// idle line (so the ESC sees the gap that ends a frame, also between a tick's frame and the panic loop's first) and the
// FIFO is empty.
[[nodiscard]] bool try_send(const FrameWords& words) noexcept {
  const TimeUs now = hal_time_us();
  if (now < g_busy_until_us || pio_sm_get_tx_fifo_level(g_pio, g_sm) != 0) {
    return false;
  }
  for (const std::uint32_t w : words) {
    pio_sm_put(g_pio, g_sm, w);
  }
  g_busy_until_us = now + 2 * frame_time_us();
  return true;
}

}  // namespace

MotorOutputBoot motor_output_start(const std::array<std::int32_t, kMotorPads>& pad_of_motor,
                                   TimeUs stop_interval_us) noexcept {
  PadMap pad{};
  if (!make_pad_map(pad_of_motor, pad)) {
    hal_panic("motor_pad is not a permutation of the motor outputs");
  }
  MotorOutputBoot boot;
  boot.clk_sys_hz = clock_get_hz(clk_sys);
  const PioClockDivider div = pio_clock_divider(boot.clk_sys_hz);
  if (div.integer == 0) {
    hal_panic("clk_sys cannot give the DShot600 PIO clock");
  }
  boot.div_int = div.integer;
  boot.div_frac8 = div.frac8;

  // The program, at offset 0 (nothing else uses PIO 0): the shift stalls with every pad low until a frame arrives.
  const PioTiming t = pio_timing();
  const unsigned all_pads = (1U << kMotorPads) - 1U;
  const std::array program{
      static_cast<std::uint16_t>(pio_encode_out(pio_x, kNibbleBits)),
      static_cast<std::uint16_t>(pio_encode_set(pio_pins, all_pads) | pio_encode_delay(t.high_delay)),
      static_cast<std::uint16_t>(pio_encode_mov(pio_pins, pio_x) | pio_encode_delay(t.data_delay)),
      static_cast<std::uint16_t>(pio_encode_set(pio_pins, 0) | pio_encode_delay(t.low_delay))};
  g_pio = pio0;
  const int sm = pio_claim_unused_sm(g_pio, false);
  if (sm < 0) {
    hal_panic("no free PIO state machine for the motor output");
  }
  g_sm = static_cast<unsigned>(sm);
  for (std::size_t i = 0; i < program.size(); ++i) {
    g_pio->instr_mem[i] = program[i];
  }

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, 0, program.size() - 1);
  sm_config_set_out_pins(&c, kPadBase, kMotorPads);
  sm_config_set_set_pins(&c, kPadBase, kMotorPads);
  sm_config_set_out_shift(&c, true, true, kWordBits);
  sm_config_set_clkdiv_int_frac8(&c, div.integer, div.frac8);
  pio_sm_init(g_pio, g_sm, 0, &c);
  pio_sm_set_pins_with_mask(g_pio, g_sm, 0, all_pads << kPadBase);
  pio_sm_set_consecutive_pindirs(g_pio, g_sm, kPadBase, kMotorPads, true);
  for (unsigned p = 0; p < kMotorPads; ++p) {
    pio_gpio_init(g_pio, kPadBase + p);
  }
  pio_sm_set_enabled(g_pio, g_sm, true);

  g_pad = pad;
  g_stop_interval_us = stop_interval_us;
  g_started = true;
  return boot;
}

void motor_output_write(std::span<const DshotValue> motor) noexcept {
  if (!g_started) {
    hal_panic("hal_actuators_write before motor_output_start");
  }
  std::array<DshotValue, kMotorPads> m{};
  for (std::size_t i = 0; i < kMotorPads; ++i) {
    m[i] = motor[i];
  }
  if (!try_send(pack_frames(g_pad, m))) {
    ++g_dropped;
  }
}

void motor_output_panic_service() noexcept {
  if (!g_started || hal_time_us() < g_next_stop_us) {
    return;
  }
  if (try_send(stop_words())) {
    g_next_stop_us = hal_time_us() + g_stop_interval_us;
  }
}

std::uint64_t motor_frames_dropped() noexcept { return g_dropped; }

}  // namespace marv::hal_target
