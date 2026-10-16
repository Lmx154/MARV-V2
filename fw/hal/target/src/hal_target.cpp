#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <pico.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <tusb.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/bench_stats.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/motor_output.hpp"
#include "marv/hal_target/watchdog.hpp"
#include "marv/types/actuator.hpp"

namespace marv {
namespace {

// Board package check (decision 0027 invariant I2): every pin the FC3v2 Rev-C header or the SDK defaults use exists on
// the board's package. With a board header of the A package (GPIO 0-29, for example PICO_BOARD=pico2) this fails.
constexpr bool pin_exists(unsigned pin) noexcept { return pin < NUM_BANK0_GPIOS; }

static_assert(pin_exists(MADFLIGHT_FC3V2_IMU_MISO_PIN) && pin_exists(MADFLIGHT_FC3V2_IMU_SCK_PIN) &&
                  pin_exists(MADFLIGHT_FC3V2_IMU_MOSI_PIN) && pin_exists(MADFLIGHT_FC3V2_IMU_CS_PIN) &&
                  pin_exists(MADFLIGHT_FC3V2_IMU_INT_PIN) && pin_exists(MADFLIGHT_FC3V2_IMU_CLKIN_PIN),
              "an IMU pin is not a GPIO of this package");
static_assert(pin_exists(MADFLIGHT_FC3V2_SER0_TX_PIN) && pin_exists(MADFLIGHT_FC3V2_SER0_RX_PIN) &&
                  pin_exists(MADFLIGHT_FC3V2_SER1_TX_PIN) && pin_exists(MADFLIGHT_FC3V2_SER1_RX_PIN),
              "a serial pin is not a GPIO of this package");
static_assert(pin_exists(MADFLIGHT_FC3V2_OUT0_PIN) && pin_exists(MADFLIGHT_FC3V2_OUT1_PIN) &&
                  pin_exists(MADFLIGHT_FC3V2_OUT2_PIN) && pin_exists(MADFLIGHT_FC3V2_OUT3_PIN),
              "a motor output pin is not a GPIO of this package");
static_assert(pin_exists(PICO_DEFAULT_I2C_SDA_PIN) && pin_exists(PICO_DEFAULT_I2C_SCL_PIN) &&
                  pin_exists(PICO_DEFAULT_LED_PIN) && pin_exists(PICO_DEFAULT_WS2812_PIN),
              "an I2C or LED pin is not a GPIO of this package");

}  // namespace

TimeUs hal_time_us() noexcept { return time_us_64(); }

// No rotor-speed source exists yet: every valid bit is clear and every speed 0 (the HAL contract for no sample).
RotorSpeedSample hal_rotor_speed() noexcept {
  RotorSpeedSample s{};
  s.t_us = hal_time_us();
  return s;
}

namespace hal_target {
namespace {

// Written only inside the tick (the IMU interrupt), read by its counter publication (lsm6dsv.cpp).
std::uint64_t g_actuator_writes = 0;

}  // namespace

std::uint64_t actuator_writes() noexcept { return g_actuator_writes; }

}  // namespace hal_target

// The bench image times the stage (bench_stats.hpp); every other image does nothing here.
void hal_tick_stage(TickStage stage) noexcept {
  using hal_target::bench::Timed;
  switch (stage) {
    case TickStage::Filter:
      hal_target::bench::stage(Timed::Filter);
      break;
    case TickStage::Rate:
      hal_target::bench::stage(Timed::Rate);
      break;
    case TickStage::Mixer:
      hal_target::bench::stage(Timed::Mixer);
      break;
    case TickStage::Gate:
      hal_target::bench::stage(Timed::Gate);
      break;
  }
}

// A write is counted and sends one DShot frame per motor (motor_output.hpp); quad_fc writes DshotValue::stop() on every
// rate tick (M1 P9 ruling). The span sizes must equal the board's outputs, the four motor pads OUT0-OUT3 and no servo, as
// hal_sim checks them against its bound latch.
void hal_actuators_write(std::span<const DshotValue> motor, std::span<const ServoUs> servo) noexcept {
  hal_target::bench::stage(hal_target::bench::Timed::Dshot);
  if (motor.size() != kQuadXMotors || !servo.empty()) {
    hal_panic("hal_actuators_write span size differs from the board's outputs");
  }
  ++hal_target::g_actuator_writes;
  hal_target::motor_output_write(motor);
}

// What a panic does to the motors (Luis, 2026-10-16, "take option A"): the loop keeps sending DShot motor-stop frames
// (DshotValue::stop() on every motor, nothing else) at a steady interval, motor_output_panic_service; if that fails the
// ESC's signal-loss stop applies (the AM32 timeout is INFERRED). Before the motor output is started it sends nothing.
// Interrupts are disabled and stay disabled, so no interrupt handler runs again: the USB device is serviced by polling
// instead. dcd_int_handler (tud_int_handler) is the body of
// the USBCTRL interrupt service routine; it only reads the controller's status registers, so calling it from the loop
// does what the interrupt would have done. tud_task then runs the queued device events, including the control requests
// that carry the 1200-baud line coding and the vendor reset request, so the SDK's reset to BOOTSEL keeps working.
// TinyUSB is not reentrant: a panic raised from inside a USB call can find its state mid-update; panics raised by the
// application code (not by TinyUSB or the stdio driver) do not.
// The message goes out as "PANIC: <reason>" on the CDC port whenever a host has it open, and starts again each time the
// transmit buffer has drained, so the output is paced by the host reading it. The board's LED pin is driven high; the
// madflight page publishes no active level for it (boards/madflight_fc3v2_revc.h).
void hal_panic(const char* reason) noexcept {
  static_cast<void>(save_and_disable_interrupts());
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, true);

  const char* const text = reason != nullptr ? reason : "";
  const std::array parts{std::span<const char>{"PANIC: ", std::strlen("PANIC: ")},
                         std::span<const char>{text, std::strlen(text)},
                         std::span<const char>{"\r\n", std::strlen("\r\n")}};
  std::size_t part = 0;
  std::size_t sent = 0;
  for (;;) {
    // The loop keeps the watchdog fed: the stop frames are already being sent, and a reset every timeout would restart
    // the firmware and drop the USB session, so the PANIC message could not be read.
    hal_target::watchdog_feed();
    hal_target::motor_output_panic_service();
    tud_int_handler(0);
    tud_task();
    if (!tud_cdc_connected()) {
      part = 0;
      sent = 0;
    } else if (part < parts.size()) {
      const std::span<const char> p = parts[part];
      sent += tud_cdc_write(p.data() + sent, static_cast<std::uint32_t>(p.size() - sent));
      if (sent == p.size()) {
        ++part;
        sent = 0;
      }
      tud_cdc_write_flush();
    } else if (tud_cdc_write_available() == CFG_TUD_CDC_TX_BUFSIZE) {
      part = 0;
    }
  }
}

}  // namespace marv
