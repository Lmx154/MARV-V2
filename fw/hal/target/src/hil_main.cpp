#include <array>
#include <bit>
#include <climits>
#include <cstddef>
#include <cstdint>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/composition.hpp"
#include "marv/hal/hal.hpp"
#include "marv/hal/tick.hpp"
#include "marv/hal_target/hil.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/imu_log_format.hpp"
#include "marv/hal_target/lsm6dsv_decode.hpp"
#include "marv/hal_target/receiver.hpp"
#include "marv/params/param.hpp"

namespace {

// Read in the tick interrupt after each record's ticks, as the flight image's IMU interrupt reads it after each batch.
marv::hal_target::RateCounts rate_counts() noexcept {
  const marv::composition::RateCounts c = marv::composition::rate_counts();
  return marv::hal_target::RateCounts{c.executions, c.timing_faults};
}

// CRC-32 over the loaded table's values in ParamId order (hil_format.hpp HelloField::ParamValuesCrc): each record's value,
// the f32 bit pattern or the i32, 4 bytes little-endian.
std::array<std::uint8_t, marv::kParamCount * sizeof(std::uint32_t)> g_values{};

std::uint32_t param_values_crc() noexcept {
  for (std::size_t i = 0; i < marv::kParamCount; ++i) {
    const marv::ParamValue v = marv::param_get(static_cast<marv::ParamId>(i)).value;
    const std::uint32_t bits =
        v.type == marv::ParamType::F32 ? std::bit_cast<std::uint32_t>(v.f32) : static_cast<std::uint32_t>(v.i32);
    for (std::size_t b = 0; b < sizeof(bits); ++b) {
      g_values[i * sizeof(bits) + b] = static_cast<std::uint8_t>(bits >> (CHAR_BIT * b));
    }
  }
  return marv::hal_target::imu_log::crc32(g_values);
}

marv::hal_target::HilCompositionStatus composition_status() noexcept {
  const marv::composition::AttitudeCounts a = marv::composition::attitude_counts();
  return marv::hal_target::HilCompositionStatus{static_cast<std::uint32_t>(marv::composition::arming_status()), a.faults,
                                                a.timing_faults};
}

}  // namespace

// The HIL S3a image (MARV_FC3V2_HIL, fw/hal/target/CMakeLists.txt; decision 0046 items 3-5): the flight image's parameter
// table and composition init, unchanged; then, in place of the motor output, the receiver and the IMU, the USB link
// (marv/hal_target/hil.hpp): the host's samples tick the composition and its DShot values go back to the host. No DShot
// program is loaded and no motor pad is driven, not even with stop frames. The port carries binary records only
// (hil_format.hpp); a panic prints its text line (hal_panic). The receiver image (MARV_FC3V2_HIL_RECEIVER, HIL stage S4,
// decision 0046 item 4) starts the flight image's receiver as main.cpp does, before the tick interrupt, so the pilot's radio
// on the board's own receiver arms and flies the composition on the board clock; the host sends samples only.
int main() {
  stdio_init_all();
  if (!marv::params_init(marv::param_defaults())) {
    marv::hal_panic("params_init rejected the default table");
  }
  marv::composition::init();
  using marv::ParamId;
  marv::hal_target::HilConfig cfg;
#if defined(MARV_FC3V2_HIL_RECEIVER)
  cfg.receiver_boot = marv::hal_target::receiver_start({marv::param_value<ParamId::rc_channel_roll>(),
                                                       marv::param_value<ParamId::rc_channel_pitch>(),
                                                       marv::param_value<ParamId::rc_channel_throttle>(),
                                                       marv::param_value<ParamId::rc_channel_yaw>()});
  cfg.receiver = true;
#endif
  cfg.sink = &marv::composition::tick;
  cfg.rate_counts = &rate_counts;
  cfg.composition_status = &composition_status;
  cfg.param_schema_hash = marv::kParamSchemaHash;
  cfg.param_count = static_cast<std::uint32_t>(marv::kParamCount);
  cfg.param_values_crc = param_values_crc();
  cfg.tick_period_num_us = marv::param_value<ParamId::tick_period_num_us>();
  cfg.tick_period_den = marv::param_value<ParamId::tick_period_den>();
  cfg.rate_loop_divisor = marv::param_value<ParamId::rate_loop_divisor>();
  // D_HIL (decision 0046 item 3): the flight image's IMU read deadline D = FIFO depth / ODR (core EMB-6) at the table's
  // tick period, the same rule imu_start applies. fc3v2: floor(109 x 1953125 / 14649) us = 14532 us, 9.9 times the longest
  // arrival gap the HIL S1 link measured at 4 samples per record, 1463 us
  // (tests/regression/quad/L09/results/hil_s1/s1_b04.summary.txt).
  cfg.deadline_us = marv::hal_target::imu_read_deadline_us(cfg.tick_period_num_us, cfg.tick_period_den);
  cfg.version = MARV_FC3V2_VERSION;
  marv::hal_target::hil_start(cfg);

  bool was_connected = false;
  for (;;) {
    const bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
      marv::hal_target::hil_connect();
    }
    was_connected = connected;
    marv::hal_target::hil_service(connected);
    tight_loop_contents();
  }
}
