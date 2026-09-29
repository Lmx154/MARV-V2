#include "marv/hal_sim/hal_sim.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <span>

#include "marv/hal/hal.hpp"

namespace marv {
namespace {

struct State {
  hal_sim::TickPeriod period{};
  std::span<DshotValue> motor_latch{};
  std::span<ServoUs> servo_latch{};
  TimeUs now_us = 0;
  Tick next_tick = 0;
  bool configured = false;
  bool in_tick = false;
  bool written = false;
};

State g_state;

}  // namespace

TimeUs hal_time_us() noexcept { return g_state.now_us; }

void hal_actuators_write(std::span<const DshotValue> motor, std::span<const ServoUs> servo) noexcept {
  if (!g_state.in_tick) {
    hal_panic("hal_actuators_write outside a tick");
  }
  if (g_state.written) {
    hal_panic("hal_actuators_write called twice in one tick");
  }
  if (motor.size() != g_state.motor_latch.size() || servo.size() != g_state.servo_latch.size()) {
    hal_panic("hal_actuators_write span size differs from the bound latch");
  }
  std::copy(motor.begin(), motor.end(), g_state.motor_latch.begin());
  std::copy(servo.begin(), servo.end(), g_state.servo_latch.begin());
  g_state.written = true;
}

void hal_panic(const char* reason) noexcept {
  std::fputs("hal_panic: ", stderr);
  std::fputs(reason, stderr);
  std::fputc('\n', stderr);
  std::abort();
}

namespace hal_sim {

void setup(TickPeriod p, std::span<DshotValue> motor_latch, std::span<ServoUs> servo_latch) noexcept {
  if (!period_valid(p)) {
    hal_panic("hal_sim::setup with an invalid tick period");
  }
  g_state = State{};
  g_state.period = p;
  g_state.motor_latch = motor_latch;
  g_state.servo_latch = servo_latch;
  std::fill(motor_latch.begin(), motor_latch.end(), DshotValue::stop());
  std::fill(servo_latch.begin(), servo_latch.end(), ServoUs{});
  g_state.configured = true;
}

void begin_tick(Tick n) noexcept {
  if (!g_state.configured) {
    hal_panic("hal_sim::begin_tick before setup");
  }
  if (n != g_state.next_tick) {
    hal_panic("hal_sim::begin_tick out of order");
  }
  g_state.now_us = stamp_us(g_state.period, n);
  g_state.next_tick = n + 1;
  g_state.in_tick = true;
  g_state.written = false;
}

void end_tick() noexcept { g_state.in_tick = false; }

}  // namespace hal_sim
}  // namespace marv
