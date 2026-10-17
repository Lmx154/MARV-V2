#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/hal/motor_test.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/types/rotor_speed_sample.hpp"
#include "marv/types/time.hpp"

namespace marv {

// Monotonic. hal_sim: 0 before tick 0; stamp(n) from the start of tick n until tick n+1 starts.
[[nodiscard]] TimeUs hal_time_us() noexcept;

// The latest rotor-speed sample, a pull read (SI rad/s; the eRPM to rad/s conversion is the driver's job). t_us is
// hal_time_us(); with no sample supplied for the tick every valid bit is clear and every speed 0. hal_sim: the sample
// the host staged before the tick began; before any tick, t_us 0.
[[nodiscard]] RotorSpeedSample hal_rotor_speed() noexcept;

// The console motor test's input, a pull read (decision 0038): the request and cancel counters and the latest request
// (motor_test.hpp). Wait-free and consistent (one snapshot of both counters and the request), callable from the tick;
// a cancel is visible to the next call after the console received it. hal_sim: what the host staged before the tick
// began; all counts 0 before any.
[[nodiscard]] MotorTestPoll hal_motor_test() noexcept;

// The pilot's input, a pull read (decision 0040 Q3; manual_control.hpp): the newest valid RC frame's stamp and sticks.
// Wait-free and one consistent snapshot, callable from the tick. Never decides validity: frames == 0 is "none yet" and
// the consumer ages t_us itself. hal_sim: what the host staged before the tick began (a value-initialised struct,
// frames 0, before any).
[[nodiscard]] ManualControl hal_manual_control() noexcept;

// The IMU read-deadline misses since init, a pull read (decision 0040 Q2): a count of the IMU interrupt entries that found
// the sensor FIFO overrun (samples lost because a read came later than FIFO depth / ODR, core EMB-6). The driver increments
// it before it delivers that batch, so the tick of the batch's first sample already sees it. Cumulative and never reset;
// the consumer latches it. Wait-free, callable from the tick. A sensor that delivers nothing for that long does not reach
// the tick: the target halts through hal_panic. hal_sim: the count the host staged before the tick began (0 before any).
[[nodiscard]] std::uint32_t hal_imu_deadline_misses() noexcept;

// The composition's reply to the console: one event per accepted request, refused request or end of a test. May be
// called in a tick; never blocks; the console prints it from the background loop.
void hal_motor_test_report(const MotorTestEvent& event) noexcept;

// The stages of a tick, for the target's per-stage execution times (core §4.1 EMB-1 and EMB-3; quad spec §4.1 L9).
enum class TickStage : std::uint8_t { Filter, Rate, Mixer, Gate };

// Inside composition::tick: `stage` starts here and runs until the next call, the actuator write or the end of the
// tick. Changes nothing the composition computes. hal_sim: no effect.
void hal_tick_stage(TickStage stage) noexcept;

// Only inside composition::tick, at most once per tick; span sizes must equal the bound latch sizes. Else hal_panic.
void hal_actuators_write(std::span<const DshotValue> motor, std::span<const ServoUs> servo) noexcept;

template <std::size_t M, std::size_t S>
inline void hal_actuators_write(const ActuatorOutput<M, S>& o) noexcept {
  hal_actuators_write(std::span<const DshotValue>{o.motor}, std::span<const ServoUs>{o.servo});
}

[[noreturn]] void hal_panic(const char* reason) noexcept;  // hal_sim: message to stderr + std::abort()

}  // namespace marv
