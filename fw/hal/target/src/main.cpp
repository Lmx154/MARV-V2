#include <cstdint>
#include <cstdio>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/hal/hal.hpp"
#include "marv/hal/tick.hpp"
#include "marv/hal_target/dshot.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/motor_output.hpp"
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

// One line per second: the per-second differences of the counters over the measured interval, the running totals of
// the fault counters, the stamp spacing since the previous line and the newest sample (FRD, SI).
void print_stats(const marv::hal_target::ImuStats& now, const marv::hal_target::ImuStats& before, marv::TimeUs dt_us) {
  const auto per_s = [dt_us](std::uint64_t a, std::uint64_t b) {
    return static_cast<double>(a - b) * marv::prim::kMicrosecondsPerSecond / static_cast<double>(dt_us);
  };
  const marv::ImuSample& s = now.last;
  std::printf(
      "imu dt=%lluus irq/s=%.1f words/s=%.1f samples/s=%.1f ticks/s=%.1f writes/s=%.1f motor_drops=%llu | total samples=%llu ovr=%lu "
      "spi=%lu partial=%lu clamp=%lu order=%lu empty=%lu | spacing=[%llu,%llu]us | t=%llu gyro=(%+.4f,%+.4f,%+.4f) "
      "accel=(%+.3f,%+.3f,%+.3f) temp=%.2fK flags=0x%03lx\n",
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
      static_cast<double>(s.temp_k), static_cast<unsigned long>(s.flags));
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
    was_connected = connected;
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
