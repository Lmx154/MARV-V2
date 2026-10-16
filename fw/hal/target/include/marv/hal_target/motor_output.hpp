#pragma once

// The FC3v2 motor output (quad spec §4.0 M1, packet P10): plain DShot600 on the four motor pads OUT0 .. OUT3 from one PIO
// state machine (the frame logic and its timing are in dshot.hpp). The state machine sends exactly the frames the firmware
// pushes, one per hal_actuators_write, and never repeats one: with the firmware stalled no frames leave and the ESC's
// signal-loss stop applies (invariant I4; the AM32 timeout is INFERRED). Direction is never inverted in firmware (I6).

#include <array>
#include <cstdint>
#include <span>

#include "marv/hal_target/dshot.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

struct MotorOutputBoot {
  std::uint32_t clk_sys_hz = 0;
  std::uint32_t div_int = 0;     // PIO clock divider, 16.8 fixed point
  std::uint8_t div_frac8 = 0;
};

// Claims a state machine of PIO 0, loads the program, derives the PIO clock divider from clk_sys, drives the four pads low
// and starts the machine. `pad_of_motor` is the motor_pad parameter (logical motor 1 first); hal_panic if it is not a
// permutation of 0 .. 3 or the divider is out of range. `stop_interval_us`: the panic loop's stop-frame interval.
// Call once, from main, before imu_start (the first tick writes).
MotorOutputBoot motor_output_start(const std::array<std::int32_t, kMotorPads>& pad_of_motor,
                                   TimeUs stop_interval_us) noexcept;

// One frame per motor (hal_actuators_write). A frame is dropped, and counted, if the previous one has not left yet.
void motor_output_write(std::span<const DshotValue> motor) noexcept;

// The panic loop's call (hal_panic): sends DshotValue::stop() on every motor when stop_interval_us has passed since the
// last stop frame and the previous frame has left. Sends nothing before motor_output_start.
void motor_output_panic_service() noexcept;

// Frames dropped because the previous one was still being sent.
[[nodiscard]] std::uint64_t motor_frames_dropped() noexcept;

}  // namespace marv::hal_target
