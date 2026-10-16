#include <cstddef>
#include <span>

#include <pico.h>
#include <hardware/sync.h>
#include <hardware/timer.h>

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
// decided before the DShot packet (decision 0027, Evidence). Interrupts off, then spin.
void hal_panic(const char*) noexcept {
  static_cast<void>(save_and_disable_interrupts());
  for (;;) {
    tight_loop_contents();
  }
}

}  // namespace marv
