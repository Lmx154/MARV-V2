#include <cstdint>
#include <cstdio>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/hal/hal.hpp"
#include "marv/hal/motor_test.hpp"
#include "marv/hal/tick.hpp"
#include "marv/hal_target/console.hpp"
#include "marv/hal_target/dshot.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/motor_output.hpp"
#include "marv/hal_target/motor_test.hpp"
#include "marv/params/param.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace {

#if defined(MARV_FC3V2_COUNT_ONLY)
// The bench mode (fw/hal/target/CMakeLists.txt): the driver counts, stamps and checks every sample; nothing else runs.
void count_only(const marv::ImuSample&) noexcept {}
constexpr marv::hal_target::ImuSink kSink = &count_only;
constexpr const char* kTickMode = "count-only (MARV_FC3V2_COUNT_ONLY)";
#else
constexpr marv::hal_target::ImuSink kSink = &marv::composition::tick;
constexpr const char* kTickMode = "quad_fc";
#endif

// The console period: one second, so each line's counter differences are per-second figures.
constexpr auto kPrintPeriodUs = static_cast<marv::TimeUs>(marv::prim::kMicrosecondsPerSecond);

// How far boot got, printed at boot and again whenever a host opens the CDC port, so a host that attaches late still
// reads it (the lines printed at boot are lost when no host has the port open).
enum class Stage { kStart, kParamsOk, kInitOk, kMotorsOk, kImuOk };
Stage stage = Stage::kStart;
marv::hal_target::MotorOutputBoot motor_boot;
marv::hal_target::ImuBoot imu_boot;

void print_progress() {
  std::printf("marv_fc3v2 %s\n", MARV_FC3V2_VERSION);
  if (stage >= Stage::kParamsOk) {
    std::printf("params ok\n");
  }
  if (stage >= Stage::kInitOk) {
    std::printf("init ok\n");
  }
  if (stage >= Stage::kMotorsOk) {
    std::printf("motors ok: DShot600 on PIO 0, clk_sys %lu Hz, PIO divider %lu + %u/256 (derived, dshot.hpp)\n",
                static_cast<unsigned long>(motor_boot.clk_sys_hz), static_cast<unsigned long>(motor_boot.div_int),
                static_cast<unsigned>(motor_boot.div_frac8));
  }
  if (stage >= Stage::kImuOk) {
    std::printf("imu ok: LSM6DSV, SPI %lu Hz, FREQ_FINE %d (ODR %.1f Hz by DS13476 9.49), tick %s\n",
                static_cast<unsigned long>(imu_boot.spi_hz), static_cast<int>(imu_boot.freq_fine),
                imu_boot.odr_hz_by_trim, kTickMode);
  }
}

// Console counters, background loop only: lines refused by the parser, motor-test requests forwarded.
std::uint32_t console_refused = 0;
std::uint32_t console_forwarded = 0;
marv::hal_target::LineBuffer line_buffer;
std::uint32_t events_printed = 0;

// One line per second: the per-second differences of the counters over the measured interval, the running totals of
// the fault counters, the stamp spacing since the previous line and the newest sample (FRD, SI).
void print_stats(const marv::hal_target::ImuStats& now, const marv::hal_target::ImuStats& before, marv::TimeUs dt_us) {
  const auto per_s = [dt_us](std::uint64_t a, std::uint64_t b) {
    return static_cast<double>(a - b) * marv::prim::kMicrosecondsPerSecond / static_cast<double>(dt_us);
  };
  const marv::ImuSample& s = now.last;
  const marv::hal_target::MotorTestStatus test = marv::hal_target::motor_test_status();
  std::printf(
      "imu dt=%lluus irq/s=%.1f words/s=%.1f samples/s=%.1f ticks/s=%.1f writes/s=%.1f motor_drops=%llu | total samples=%llu ovr=%lu "
      "spi=%lu partial=%lu clamp=%lu order=%lu empty=%lu | spacing=[%llu,%llu]us | t=%llu gyro=(%+.4f,%+.4f,%+.4f) "
      "accel=(%+.3f,%+.3f,%+.3f) temp=%.2fK flags=0x%03lx | motor_test console_refused=%lu forwarded=%lu accepted=%lu "
      "refused=%lu stopped=%lu\n",
      static_cast<unsigned long long>(dt_us), per_s(now.irqs, before.irqs), per_s(now.words, before.words),
      per_s(now.samples, before.samples), per_s(now.ticks, before.ticks),
      per_s(now.actuator_writes, before.actuator_writes), static_cast<unsigned long long>(now.actuator_drops),
      static_cast<unsigned long long>(now.samples), static_cast<unsigned long>(now.fifo_overflows),
      static_cast<unsigned long>(now.spi_errors), static_cast<unsigned long>(now.partial),
      static_cast<unsigned long>(now.stamp_clamps), static_cast<unsigned long>(now.order_faults),
      static_cast<unsigned long>(now.empty_irqs), static_cast<unsigned long long>(now.spacing_min_us),
      static_cast<unsigned long long>(now.spacing_max_us), static_cast<unsigned long long>(s.t_us),
      static_cast<double>(s.gyro_rad_s[0]), static_cast<double>(s.gyro_rad_s[1]), static_cast<double>(s.gyro_rad_s[2]),
      static_cast<double>(s.accel_m_s2[0]), static_cast<double>(s.accel_m_s2[1]), static_cast<double>(s.accel_m_s2[2]),
      static_cast<double>(s.temp_k), static_cast<unsigned long>(s.flags), static_cast<unsigned long>(console_refused),
      static_cast<unsigned long>(console_forwarded), static_cast<unsigned long>(test.last.accepted),
      static_cast<unsigned long>(test.last.refused), static_cast<unsigned long>(test.last.stopped));
}

const char* refusal_text(marv::hal_target::LineRefusal r) {
  using marv::hal_target::LineRefusal;
  switch (r) {
    case LineRefusal::None:
      return "none";
    case LineRefusal::UnknownCommand:
      return "unknown command";
    case LineRefusal::Arguments:
      return "wrong number of arguments";
    case LineRefusal::NotANumber:
      return "a number is not decimal digits or does not fit 32 bits";
    case LineRefusal::MissingUnlock:
      return "the unlock token PROPS-OFF is missing";
    case LineRefusal::TooLong:
      return "line too long";
  }
  return "";
}

const char* reason_text(marv::MotorTestReason r) {
  using marv::MotorTestReason;
  switch (r) {
    case MotorTestReason::None:
      return "none";
    case MotorTestReason::Motor:
      return "motor is not 1-4";
    case MotorTestReason::Value:
      return "dshot value is below the throttle minimum or above the cap";
    case MotorTestReason::Duration:
      return "duration is outside the allowed range (1 ms to the maximum)";
    case MotorTestReason::Deadline:
      return "deadline";
    case MotorTestReason::Cancel:
      return "cancel";
    case MotorTestReason::Disconnect:
      return "disconnect";
  }
  return "";
}

// Every received byte cancels a running test (Luis, 2026-10-16); the line feed completes a line, which is parsed and, if
// a request, forwarded. The composition validates and replies through print_motor_test_events.
void console_byte(char c) {
  marv::hal_target::motor_test_post_cancel(marv::MotorTestCancel::Cancel);
  marv::hal_target::ParsedLine line;
  if (!line_buffer.push(c, line)) {
    return;
  }
  switch (line.kind) {
    case marv::hal_target::LineKind::Empty:
      break;
    case marv::hal_target::LineKind::Stop:
      std::printf("motor_test: stop received\n");
      break;
    case marv::hal_target::LineKind::Refused:
      ++console_refused;
      std::printf("motor_test refused: %s (console)\n", refusal_text(line.refusal));
      break;
    case marv::hal_target::LineKind::Request:
      ++console_forwarded;
      marv::hal_target::motor_test_post_request(line.request);
      break;
  }
}

void print_motor_test_events() {
  const marv::hal_target::MotorTestStatus st = marv::hal_target::motor_test_status();
  if (st.events == events_printed) {
    return;
  }
  const marv::MotorTestEvent& e = st.last;
  if (st.events - events_printed > 1) {
    std::printf("motor_test: %lu earlier events not shown\n", static_cast<unsigned long>(st.events - events_printed - 1));
  }
  events_printed = st.events;
  switch (e.kind) {
    case marv::MotorTestEventKind::Accepted:
      std::printf("motor_test accepted: motor %lu dshot %lu for %lu ms\n", static_cast<unsigned long>(e.request.motor),
                  static_cast<unsigned long>(e.request.dshot), static_cast<unsigned long>(e.request.duration_ms));
      break;
    case marv::MotorTestEventKind::Refused:
      std::printf("motor_test refused: %s (motor %lu dshot %lu for %lu ms)\n", reason_text(e.reason),
                  static_cast<unsigned long>(e.request.motor), static_cast<unsigned long>(e.request.dshot),
                  static_cast<unsigned long>(e.request.duration_ms));
      break;
    case marv::MotorTestEventKind::Stopped:
      std::printf("motor_test stopped: %s (motor %lu)\n", reason_text(e.reason),
                  static_cast<unsigned long>(e.request.motor));
      break;
  }
}

}  // namespace

// The M1 image: stdio over USB CDC, a banner, the default parameters, the composition's init, then the IMU: from then
// on the IMU interrupt ticks the composition once per sample (marv/hal_target/imu.hpp). The idle loop prints the IMU
// counters once per second; console output never runs in the tick (decision 0027 item 6).
int main() {
  stdio_init_all();
  print_progress();
  if (!marv::params_init(marv::param_defaults())) {
    marv::hal_panic("params_init rejected the default table");
  }
  stage = Stage::kParamsOk;
  std::printf("params ok\n");
  marv::composition::init();
  stage = Stage::kInitOk;
  std::printf("init ok\n");
  using marv::ParamId;
  motor_boot = marv::hal_target::motor_output_start(
      {marv::param_value<ParamId::motor_pad_m1>(), marv::param_value<ParamId::motor_pad_m2>(),
       marv::param_value<ParamId::motor_pad_m3>(), marv::param_value<ParamId::motor_pad_m4>()},
      marv::hal_target::stop_interval_us(marv::param_value<ParamId::rate_loop_divisor>(),
                                         marv::param_value<ParamId::tick_period_num_us>(),
                                         marv::param_value<ParamId::tick_period_den>()));
  stage = Stage::kMotorsOk;
  std::printf("motors ok\n");
  imu_boot = marv::hal_target::imu_start(kSink);
  stage = Stage::kImuOk;
  print_progress();

  bool was_connected = false;
  marv::hal_target::ImuStats before = marv::hal_target::imu_stats();
  marv::TimeUs t_before = marv::hal_time_us();
  for (;;) {
    const bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
      print_progress();
    }
    if (was_connected && !connected) {
      marv::hal_target::motor_test_post_cancel(marv::MotorTestCancel::Disconnect);
    }
    was_connected = connected;
    if (connected) {
      print_motor_test_events();
      for (int c = getchar_timeout_us(0); c != PICO_ERROR_TIMEOUT; c = getchar_timeout_us(0)) {
        console_byte(static_cast<char>(c));
      }
    }
    const marv::TimeUs t = marv::hal_time_us();
    if (t - t_before >= kPrintPeriodUs) {
      const marv::hal_target::ImuStats now = marv::hal_target::imu_stats();
      print_stats(now, before, t - t_before);
      before = now;
      t_before = t;
    }
    tight_loop_contents();
  }
}
