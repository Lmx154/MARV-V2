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
#include "marv/hal_target/flash.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/imu_log.hpp"
#include "marv/hal_target/link_port.hpp"
#include "marv/hal_target/motor_output.hpp"
#include "marv/hal_target/motor_test.hpp"
#include "marv/hal_target/receiver.hpp"
#include "marv/hal_target/usb.hpp"
#include "marv/link/service.hpp"
#if defined(MARV_FC3V2_SD_BENCH)
#include "marv/hal_target/sd_bench.hpp"
#endif
#include "marv/params/param.hpp"
#include "marv/prim/constants.hpp"
#include "marv/store/store.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace {

#if defined(MARV_FC3V2_COUNT_ONLY)
// The bench mode (fw/hal/target/CMakeLists.txt): the driver counts, stamps and checks every sample; nothing else runs.
void count_only(const marv::ImuSample&) noexcept {}
constexpr marv::hal_target::ImuSink kSink = &count_only;
constexpr const char* kTickMode = "count-only (MARV_FC3V2_COUNT_ONLY)";
#elif defined(MARV_FC3V2_IMU_LOG)
// The bench log (fw/hal/target/CMakeLists.txt): the one call that hands the composition a sample first copies it to the
// log ring, so the log holds exactly the stream the composition's tick receives, flags included.
void log_then_tick(const marv::ImuSample& s) noexcept {
  marv::hal_target::imu_log_push(s);
  marv::composition::tick(s);
}
constexpr marv::hal_target::ImuSink kSink = &log_then_tick;
constexpr const char* kTickMode = "quad_fc + IMU log (MARV_FC3V2_IMU_LOG)";
#else
constexpr marv::hal_target::ImuSink kSink = &marv::composition::tick;
constexpr const char* kTickMode = "quad_fc";
#endif

// Read by the IMU interrupt after each batch (imu.hpp).
marv::hal_target::RateCounts rate_counts() noexcept {
  const marv::composition::RateCounts c = marv::composition::rate_counts();
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

// The config store's hooks into the composition (marv/store/store.hpp; decision 0053, ground-segment unit 4b).
const char* validate_table(marv::ParamTable table) noexcept { return marv::composition::validate(table).reason; }
bool armed_now() noexcept { return marv::arming::is_armed(marv::composition::arming_status()); }
// The store's working memory (a pending file, a slot copy, the staged config's encoding; ground-segment unit 14), the image
// hash it binds stored configs to, and how long the hash took at boot. A slot's worth, so any file a slot holds can be
// applied; the bench-statistics image, whose statistics take the RAM a full slot would need, has half a slot, which still
// holds every config the generator writes (tools/config/mcfg.py --max-bytes, half a slot), and refuses a larger staged config
// (store::activate: too large).
#if defined(MARV_FC3V2_BENCH_STATS)
constexpr std::size_t kStoreScratchBytes = marv::kConfigSlotBytes / 2;
#else
constexpr std::size_t kStoreScratchBytes = marv::kConfigSlotBytes;
#endif
std::array<std::uint8_t, kStoreScratchBytes> g_store_scratch{};
marv::hal_target::ImageHash image_hash;
marv::TimeUs image_hash_us = 0;

// The console period: one second, so each line's counter differences are per-second figures.
constexpr auto kPrintPeriodUs = static_cast<marv::TimeUs>(marv::prim::kMicrosecondsPerSecond);

// How far boot got, printed at boot and again whenever a host opens the CDC port, so a host that attaches late still
// reads it (the lines printed at boot are lost when no host has the port open).
enum class Stage { kStart, kParamsOk, kInitOk, kMotorsOk, kReceiverOk, kImuOk };
Stage stage = Stage::kStart;
marv::hal_target::MotorOutputBoot motor_boot;
marv::hal_target::ReceiverBoot receiver_boot;
marv::hal_target::ImuBoot imu_boot;
// hal_time_us just before the IMU is started: the boot-to-IMU-start time (the timer counts from the chip reset).
marv::TimeUs imu_start_us = 0;

const char* config_status_text(marv::arming::ConfigStatus s) {
  using marv::arming::ConfigStatus;
  switch (s) {
    case ConfigStatus::Ok:
      return "ok";
    case ConfigStatus::Corrupt:
      return "refused: corrupt";
    case ConfigStatus::SchemaMismatch:
      return "refused: schema mismatch";
    case ConfigStatus::Invalid:
      return "refused: invalid";
    case ConfigStatus::NotValidated:
      return "refused: not validated";
  }
  return "";
}

const char* boot_request_text(marv::store::BootRequest r) {
  using marv::store::BootRequest;
  switch (r) {
    case BootRequest::None:
      return "none";
    case BootRequest::Activate:
      return "activate";
    case BootRequest::Revert:
      return "revert";
    case BootRequest::FactoryReset:
      return "factory reset";
    case BootRequest::Unmatched:
      return "unmatched";
  }
  return "";
}

void print_digest(const char* name, const marv::hal_target::ImageDigest& d) {
  std::printf("%s ", name);
  for (const std::uint8_t b : d) {
    std::printf("%02x", static_cast<unsigned>(b));
  }
}

// The image hash and what the config store's boot step did (ground-segment unit 14).
void print_config() {
  std::printf("image: ");
  print_digest("sha256", image_hash.sha256);
  std::printf(" (%lu bytes, %s, %llu us)\n", static_cast<unsigned long>(image_hash.bytes),
              image_hash.ok ? "SHA-256 accelerator" : "accelerator busy: unknown",
              static_cast<unsigned long long>(image_hash_us));
  const marv::store::BootReport& b = marv::store::boot_report();
  const bool slot = b.source == marv::store::ConfigSource::Slot;
  std::printf("config: source %s", slot ? (b.slot == marv::ConfigSlot::A ? "slot A" : "slot B") : "factory");
  if (slot) {
    std::printf(" sequence %lu ", static_cast<unsigned long>(b.sequence));
    print_digest("config_sha256", b.config_hash);
    print_digest(" bound_image", b.binding.image_sha256);
  }
  std::printf(", status %s", config_status_text(b.status));
  if (b.fault != marv::store::Fault::None) {
    std::printf(" (%s%s%s)", marv::store::fault_text(b.fault), b.validator_reason != nullptr ? ": " : "",
                b.validator_reason != nullptr ? b.validator_reason : "");
  }
  std::printf(", boot request %s", boot_request_text(b.request));
  if (b.request != marv::store::BootRequest::None) {
    std::printf(" (%s)", b.request_fault == marv::store::Fault::None ? "done" : marv::store::fault_text(b.request_fault));
  }
  std::printf("\n");
}

void print_progress() {
  std::printf("marv_fc3v2 %s\n", MARV_FC3V2_VERSION);
  if (stage >= Stage::kParamsOk) {
    print_config();
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
  if (stage >= Stage::kReceiverOk) {
    std::printf("receiver ok: CRSF on UART0 (GPIO 0/1), 8N1, %lu Hz requested (ExpressLRS default), %lu Hz set, RX IRQ priority %lu\n",
                static_cast<unsigned long>(receiver_boot.baud_requested),
                static_cast<unsigned long>(receiver_boot.baud_actual),
                static_cast<unsigned long>(receiver_boot.irq_priority));
  }
  if (stage >= Stage::kImuOk) {
    std::printf("imu ok: LSM6DSV, SPI %lu Hz, FREQ_FINE %d (ODR %.1f Hz by DS13476 9.49), tick %s\n",
                static_cast<unsigned long>(imu_boot.spi_hz), static_cast<int>(imu_boot.freq_fine),
                imu_boot.odr_hz_by_trim, kTickMode);
    std::printf("boot: IMU start at %llu us after reset (hal_time_us just before imu_start)\n",
                static_cast<unsigned long long>(imu_start_us));
    std::printf("bench stats: %s\n", marv::hal_target::bench::kEnabled
                                         ? "on (MARV_FC3V2_BENCH_STATS, DWT CYCCNT at clk_sys)"
                                         : "off");
  }
}

[[maybe_unused]] const char* arming_text(marv::arming::ArmState s) {
  using marv::arming::ArmState;
  switch (s) {
    case ArmState::Boot:
      return "boot";
    case ArmState::SwitchOff:
      return "switch_off";
    case ArmState::SwitchNotCycled:
      return "switch_not_cycled";
    case ArmState::ThrottleUp:
      return "throttle_up";
    case ArmState::ImuStale:
      return "imu_stale";
    case ArmState::ImuInvalid:
      return "imu_invalid";
    case ArmState::Saturated:
      return "saturated";
    case ArmState::NoLink:
      return "no_link";
    case ArmState::Failsafe:
      return "failsafe";
    case ArmState::MotorTest:
      return "motor_test";
    case ArmState::Armed:
      return "armed";
    case ArmState::ArmedHold:
      return "armed_hold";
    case ArmState::ArmedFailsafeCentred:
      return "armed_failsafe_centred";
    case ArmState::EstimatorInvalid:
      return "estimator_invalid";
    case ArmState::Calibrating:
      return "calibrating";
    case ArmState::NotUpright:
      return "not_upright";
    case ArmState::ConfigRefused:
      return "config_refused";
    case ArmState::Maintenance:
      return "maintenance";
  }
  return "";
}

// Console counters, background loop only: lines refused by the parser, motor-test requests forwarded.
std::uint32_t console_refused = 0;
std::uint32_t console_forwarded = 0;
marv::hal_target::LineBuffer line_buffer;
std::uint32_t events_printed = 0;

// One line per second: the per-second differences of the counters over the measured interval, the running totals of
// the fault counters, the stamp spacing since the previous line and the newest sample (FRD, SI).
[[maybe_unused]] void print_stats(const marv::hal_target::ImuStats& now, const marv::hal_target::ImuStats& before,
                 const marv::hal_target::ReceiverStats& rc_now, const marv::hal_target::ReceiverStats& rc_before,
                 marv::TimeUs dt_us) {
  const auto per_s = [dt_us](std::uint64_t a, std::uint64_t b) {
    return static_cast<double>(a - b) * marv::prim::kMicrosecondsPerSecond / static_cast<double>(dt_us);
  };
  const marv::ImuSample& s = now.last;
  const marv::hal_target::MotorTestStatus test = marv::hal_target::motor_test_status();
  const marv::ManualControl& rc = rc_now.last;
  const marv::TimeUs t_now = marv::hal_time_us();
  const marv::composition::AttitudeStatus attitude = marv::composition::attitude_status();
  const marv::composition::AttitudeCounts attitude_counts = marv::composition::attitude_counts();
  const long long rc_age_us = rc.frames == 0 ? -1 : static_cast<long long>(t_now > rc.t_us ? t_now - rc.t_us : 0);
  std::printf(
      "imu dt=%lluus irq/s=%.1f words/s=%.1f samples/s=%.1f ticks/s=%.1f writes/s=%.1f motor_drops=%llu | total samples=%llu ovr=%lu "
      "spi=%lu partial=%lu clamp=%lu order=%lu empty=%lu | spacing=[%llu,%llu]us | t=%llu gyro=(%+.4f,%+.4f,%+.4f) "
      "accel=(%+.3f,%+.3f,%+.3f) temp=%.2fK flags=0x%03lx | motor_test console_refused=%lu forwarded=%lu accepted=%lu "
      "refused=%lu stopped=%lu | rate executions/s=%.1f executions=%llu timing_faults=%lu | rc frames/s=%.1f total=%lu "
      "link_frames=%lu bad_crc=%lu bad_length=%lu dropped_bytes=%lu uart_errors=%lu age_us=%lld roll=%+.3f pitch=%+.3f "
      "throttle=%.3f yaw=%+.3f tilt=%.4f est_valid=%d | arming=%s | attitude faults=%lu timing_faults=%lu | usb_reset_refused=%lu\n",
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
      static_cast<unsigned long>(now.rate.timing_faults),
      per_s(rc_now.counters.rc_frames, rc_before.counters.rc_frames),
      static_cast<unsigned long>(rc_now.counters.rc_frames),
      static_cast<unsigned long>(rc_now.counters.link_stats_frames), static_cast<unsigned long>(rc_now.counters.bad_crc),
      static_cast<unsigned long>(rc_now.counters.bad_length), static_cast<unsigned long>(rc_now.counters.dropped_bytes),
      static_cast<unsigned long>(rc_now.uart_errors), rc_age_us, static_cast<double>(rc.roll),
      static_cast<double>(rc.pitch), static_cast<double>(rc.throttle), static_cast<double>(rc.yaw),
      static_cast<double>(attitude.tilt_rad), attitude.valid ? 1 : 0, arming_text(marv::composition::arming_status()),
      static_cast<unsigned long>(attitude_counts.faults), static_cast<unsigned long>(attitude_counts.timing_faults),
      static_cast<unsigned long>(marv::hal_target::usb_reset_refused()));
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
[[maybe_unused]] void print_bench(marv::TimeUs t) {
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
[[maybe_unused]] void print_bench(marv::TimeUs) {}
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
    case MotorTestReason::Armed:
      return "the vehicle is armed";
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

// The M1 image: stdio over USB CDC, a banner, the default parameters, the composition's init, the motor output, the receiver UART (CRSF, M2 P5b), then the IMU: from then
// on the IMU interrupt ticks the composition once per sample (marv/hal_target/imu.hpp). The idle loop prints the IMU
// and receiver counters once per second; console output never runs in the tick (decision 0027 item 6).
int main() {
  // USB-initiated resets are honoured only with the maintenance grant (usb.hpp); wired before the host can enumerate.
  marv::hal_target::usb_set_reset_gate(&marv::composition::maintenance_granted);
  stdio_init_all();
  marv::hal_target::bench::start();
  print_progress();
  // The image hash, then the config store's boot step: it applies a requested activation (the only flash writes this image
  // makes) and chooses the table by the boot policy, before the IMU, receiver and motor outputs start (store.hpp); the slot
  // writes are then sealed. params_init runs exactly once on what it returned (invariant P-1).
  const marv::TimeUs hash_start_us = marv::hal_time_us();
  image_hash = marv::hal_target::image_sha256();
  image_hash_us = marv::hal_time_us() - hash_start_us;
  const marv::ParamTable table = marv::store::boot(marv::store::BootInputs{
      param_table(), image_hash.sha256,
      marv::store::Hooks{&validate_table, &armed_now, &marv::composition::request_maintenance,
                         &marv::composition::maintenance_granted},
      g_store_scratch});
  marv::hal_target::config_slots_seal();
  if (table.size() != marv::kParamCount) {
    marv::hal_panic("the config store returned a table of another parameter set");
  }
  if (!marv::params_init(std::span<const marv::ParamRecord, marv::kParamCount>{table.data(), marv::kParamCount})) {
    marv::hal_panic("params_init rejected the default table");
  }
  stage = Stage::kParamsOk;
  print_config();
  std::printf("params ok: %s\n", kParamMode);
  marv::composition::set_config_status(marv::store::boot_report().status);
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
  receiver_boot = marv::hal_target::receiver_start({marv::param_value<ParamId::rc_channel_roll>(),
                                                    marv::param_value<ParamId::rc_channel_pitch>(),
                                                    marv::param_value<ParamId::rc_channel_throttle>(),
                                                    marv::param_value<ParamId::rc_channel_yaw>()});
  stage = Stage::kReceiverOk;
  std::printf("receiver ok\n");
  imu_start_us = marv::hal_time_us();
  imu_boot = marv::hal_target::imu_start(
      kSink, &rate_counts,
      {marv::param_value<ParamId::tick_period_num_us>(), marv::param_value<ParamId::tick_period_den>()});
  stage = Stage::kImuOk;
  print_progress();
#if !defined(MARV_FC3V2_USB_BENCH)
  // The ground link (ground-segment unit 12b) on the link CDC, served from this loop only; its identity carries the image hash
  // the store's boot step above was given.
  marv::link::service_init(marv::link::ServiceConfig{
      marv::link::Transport{&marv::hal_target::link_port_receive, &marv::hal_target::link_port_send,
                            &marv::hal_target::link_port_writable, &marv::hal_target::link_port_connected},
      image_hash.sha256, &armed_now, &marv::composition::request_maintenance, &marv::composition::maintenance_granted});
#endif

  bool was_connected = false;
#if !defined(MARV_FC3V2_IMU_LOG)
  marv::hal_target::ImuStats before = marv::hal_target::imu_stats();
  marv::hal_target::ReceiverStats rc_before = marv::hal_target::receiver_stats();
  marv::TimeUs t_before = marv::hal_time_us();
#endif
  for (;;) {
    const bool connected = stdio_usb_connected();
#if !defined(MARV_FC3V2_IMU_LOG)
    if (connected && !was_connected) {
      print_progress();
    }
#endif
    if (was_connected && !connected) {
      marv::hal_target::motor_test_post_cancel(marv::MotorTestCancel::Disconnect);
    }
    was_connected = connected;
#if defined(MARV_FC3V2_SD_BENCH)
    marv::hal_target::sd_bench_step();
#endif
#if defined(MARV_FC3V2_USB_BENCH)
    marv::hal_target::usb_bench_echo_step();
#endif
    if (connected) {
      print_motor_test_events();
      for (int c = getchar_timeout_us(0); c != PICO_ERROR_TIMEOUT; c = getchar_timeout_us(0)) {
        console_byte(static_cast<char>(c));
      }
    }
#if !defined(MARV_FC3V2_USB_BENCH)
    // Once the board's hal_reboot has armed its delayed watchdog reboot, step returns true; the loop keeps serving (the link's
    // queued answers go out) until the watchdog resets the chip.
    static_cast<void>(marv::link::service_step());
    marv::hal_target::link_port_pump();
#endif
#if defined(MARV_FC3V2_IMU_LOG)
    // The log variant prints no text status line or banner: the port carries binary records (imu_log_format.hpp).
    marv::hal_target::imu_log_service();
#else
    const marv::TimeUs t = marv::hal_time_us();
    if (t - t_before >= kPrintPeriodUs) {
      const marv::hal_target::ImuStats now = marv::hal_target::imu_stats();
      const marv::hal_target::ReceiverStats rc_now = marv::hal_target::receiver_stats();
      print_stats(now, before, rc_now, rc_before, t - t_before);
      print_bench(t);
      before = now;
      rc_before = rc_now;
      t_before = t;
    }
#endif
    tight_loop_contents();
  }
}
