#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/composition.hpp"
#include "marv/hal/hal.hpp"
#include "marv/hal/motor_test.hpp"
#include "marv/hal/tick.hpp"
#include "marv/hal_target/bench_stats.hpp"
#include "marv/hal_target/console.hpp"
#include "marv/hal_target/dshot.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/motor_output.hpp"
#include "marv/hal_target/motor_test.hpp"
#include "marv/hal_target/watchdog.hpp"
#include "marv/params/param.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace {

#if defined(MARV_FC3V2_COUNT_ONLY)
// The bench mode (fw/hal/target/CMakeLists.txt): the driver counts, stamps and checks every sample; nothing else runs.
void count_only(const marv::ImuSample&) noexcept { marv::hal_target::watchdog_feed(); }
constexpr marv::hal_target::ImuSink kSink = &count_only;
constexpr const char* kTickMode = "count-only (MARV_FC3V2_COUNT_ONLY)";
#else
constexpr marv::hal_target::ImuSink kSink = &marv::composition::tick;
constexpr const char* kTickMode = "quad_fc";
#endif

// Read by the IMU interrupt after each batch (imu.hpp). The watchdog is fed here, and only when the rate executions have
// advanced since the last feed: a stalled rate tick (or a stopped IMU interrupt) stops the executions and so trips the
// watchdog (watchdog.hpp). The count-only image has no rate executions; its sink feeds instead.
std::uint64_t g_fed_executions = 0;

marv::hal_target::RateCounts rate_counts() noexcept {
  const marv::composition::RateCounts c = marv::composition::rate_counts();
  if (c.executions != g_fed_executions) {
    g_fed_executions = c.executions;
    marv::hal_target::watchdog_feed();
  }
  return marv::hal_target::RateCounts{c.executions, c.timing_faults};
}

#if defined(MARV_FC3V2_BENCH_ACCEPT_SPACING)
// Bench only (fw/hal/target/CMakeLists.txt): rate_period_tolerance_us is set, through the parameter table, to the largest
// value the rate loop accepts, (num - 1) / (2 den) for the tick period num / den us (rate_loop.hpp, validate_timing:
// 2 den tol < num), so on spacings off the PLACEHOLDER tick period the rate law runs its accepted path and its execution
// time is measured. Every other image loads the generated table unchanged.
std::array<marv::ParamRecord, marv::kParamCount> g_bench_table{};

std::span<const marv::ParamRecord, marv::kParamCount> param_table() {
  using marv::ParamId;
  const std::span<const marv::ParamRecord, marv::kParamCount> d = marv::param_defaults();
  for (std::size_t i = 0; i < marv::kParamCount; ++i) {
    g_bench_table[i] = d[i];
  }
  const std::int32_t num = d[static_cast<std::size_t>(ParamId::tick_period_num_us)].value.i32;
  const std::int32_t den = d[static_cast<std::size_t>(ParamId::tick_period_den)].value.i32;
  marv::ParamRecord& tol = g_bench_table[static_cast<std::size_t>(ParamId::rate_period_tolerance_us)];
  tol.value.i32 = den > 0 ? (num - 1) / (2 * den) : tol.value.i32;
  tol.origin = marv::ParamOrigin::Manual;
  tol.source = "bench: MARV_FC3V2_BENCH_ACCEPT_SPACING, the largest tolerance the rate loop accepts, (num - 1) / (2 den)";
  return std::span<const marv::ParamRecord, marv::kParamCount>{g_bench_table};
}
constexpr const char* kParamMode = "bench: rate_period_tolerance_us raised (MARV_FC3V2_BENCH_ACCEPT_SPACING)";
#else
std::span<const marv::ParamRecord, marv::kParamCount> param_table() { return marv::param_defaults(); }
constexpr const char* kParamMode = "generated defaults";
#endif

// The console period: one second, so each line's counter differences are per-second figures.
constexpr auto kPrintPeriodUs = static_cast<marv::TimeUs>(marv::prim::kMicrosecondsPerSecond);

// How far boot got, printed at boot and again whenever a host opens the CDC port, so a host that attaches late still
// reads it (the lines printed at boot are lost when no host has the port open).
enum class Stage { kStart, kParamsOk, kInitOk, kMotorsOk, kImuOk, kWatchdogOk };
Stage stage = Stage::kStart;
marv::hal_target::MotorOutputBoot motor_boot;
marv::hal_target::ImuBoot imu_boot;
marv::hal_target::WatchdogBoot watchdog_boot;

void print_progress() {
  std::printf("marv_fc3v2 %s\n", MARV_FC3V2_VERSION);
  if (stage >= Stage::kParamsOk) {
    std::printf("params ok: %s, rate_period_tolerance_us %ld\n", kParamMode,
                static_cast<long>(marv::param_value<marv::ParamId::rate_period_tolerance_us>()));
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
    std::printf("bench stats: %s\n", marv::hal_target::bench::kEnabled
                                         ? "on (MARV_FC3V2_BENCH_STATS, DWT CYCCNT at clk_sys)"
                                         : "off");
  }
  if (stage >= Stage::kWatchdogOk) {
    std::printf("watchdog ok: %ld ms, last reset: %s\n", static_cast<long>(watchdog_boot.timeout_ms),
                watchdog_boot.last_reset_by_watchdog ? "watchdog" : "other");
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
      "refused=%lu stopped=%lu | rate executions/s=%.1f executions=%llu timing_faults=%lu\n",
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
      static_cast<unsigned long>(test.last.refused), static_cast<unsigned long>(test.last.stopped),
      per_s(now.rate.executions, before.rate.executions), static_cast<unsigned long long>(now.rate.executions),
      static_cast<unsigned long>(now.rate.timing_faults));
}

#if defined(MARV_FC3V2_BENCH_STATS)
constexpr std::array<const char*, marv::hal_target::bench::kTimed> kTimedNames{
    "read", "decode", "filter", "rate", "mixer", "gate", "dshot", "publish", "bench", "irq", "account", "usb_irq"};

void print_spacing(const char* name, const marv::hal_target::bench::Spacing& s) {
  std::printf(" %s n=%llu sum=%llu min=%lu max=%lu", name, static_cast<unsigned long long>(s.count),
              static_cast<unsigned long long>(s.sum), static_cast<unsigned long>(s.min),
              static_cast<unsigned long>(s.max));
}

void print_axes(const char* name, const std::array<std::int64_t, marv::prim::kSpatialDim>& v) {
  std::printf(" %s=(%lld,%lld,%lld)", name, static_cast<long long>(v[0]), static_cast<long long>(v[1]),
              static_cast<long long>(v[2]));
}

// Two lines per second, cumulative since start except the window figures (bench_stats.hpp): the cycle counts per timed
// quantity with the nonzero histogram buckets as bucket:count, the interrupt-entry spacing, and the still-bench sums.
void print_bench(marv::TimeUs t) {
  namespace bench = marv::hal_target::bench;
  const bench::Snapshot& b = bench::snapshot();
  std::printf("bench_stages t=%llu", static_cast<unsigned long long>(t));
  for (std::size_t i = 0; i < bench::kTimed; ++i) {
    const bench::Timing& x = b.timing[i];
    std::printf(" | %s n=%llu max=%lu wmax=%lu h=", kTimedNames[i], static_cast<unsigned long long>(x.count),
                static_cast<unsigned long>(x.max), static_cast<unsigned long>(x.window_max));
    const char* sep = "";
    for (std::size_t k = 0; k < bench::kBuckets; ++k) {
      if (x.buckets[k] != 0) {
        std::printf("%s%u:%lu", sep, static_cast<unsigned>(k), static_cast<unsigned long>(x.buckets[k]));
        sep = ",";
      }
    }
  }
  std::printf(" |");
  print_spacing("spacing", b.spacing);
  print_spacing("| wspacing", b.window_spacing);
  std::printf(" | max_batch=%lu usb_preemptions=%lu\n", static_cast<unsigned long>(b.max_batch),
              static_cast<unsigned long>(b.usb_preemptions));
  std::printf("bench_imu t=%llu n=%llu", static_cast<unsigned long long>(t),
              static_cast<unsigned long long>(b.still_samples));
  print_axes("gs", b.gyro_sum);
  print_axes("gq", b.gyro_sq);
  print_axes("as", b.accel_sum);
  print_axes("aq", b.accel_sq);
  std::printf("\n");
}
#else
void print_bench(marv::TimeUs) {}
#endif

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
  marv::hal_target::bench::start();
  print_progress();
  if (!marv::params_init(param_table())) {
    marv::hal_panic("params_init rejected the default table");
  }
  stage = Stage::kParamsOk;
  std::printf("params ok: %s\n", kParamMode);
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
  imu_boot = marv::hal_target::imu_start(kSink, &rate_counts);
  stage = Stage::kImuOk;
  // Enabled last: the motor output sends stop frames and the IMU tick runs. The first feed is the rate execution that
  // follows; until then the countdown started here covers it.
  watchdog_boot = marv::hal_target::watchdog_start(marv::param_value<ParamId::watchdog_timeout_ms>());
  stage = Stage::kWatchdogOk;
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
      print_bench(t);
      before = now;
      t_before = t;
    }
    tight_loop_contents();
  }
}
