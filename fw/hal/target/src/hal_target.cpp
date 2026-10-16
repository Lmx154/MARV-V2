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

// Stub until the DShot packet: no motor output exists, so a write is a fault.
void hal_actuators_write(std::span<const DshotValue>, std::span<const ServoUs>) noexcept {
  hal_panic("hal_actuators_write: no motor output on the target yet");
}

// With no motor output there is nothing for a panic to do to the motors. What panic does to them (cut, hold, idle) is
// decided before the DShot packet (decision 0027, Evidence). Interrupts are disabled and stay disabled, so no interrupt
// handler runs again: the USB device is serviced by polling instead. dcd_int_handler (tud_int_handler) is the body of
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
