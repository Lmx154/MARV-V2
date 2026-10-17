#include <cstdint>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/composition.hpp"
#include "marv/hal/hal.hpp"
#include "marv/hal/tick.hpp"
#include "marv/hal_target/hil.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/lsm6dsv_decode.hpp"
#include "marv/params/param.hpp"

namespace {

// Read in the tick interrupt after each record's ticks, as the flight image's IMU interrupt reads it after each batch.
marv::hal_target::RateCounts rate_counts() noexcept {
  const marv::composition::RateCounts c = marv::composition::rate_counts();
  return marv::hal_target::RateCounts{c.executions, c.timing_faults};
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
// (hil_format.hpp); a panic prints its text line (hal_panic).
int main() {
  stdio_init_all();
  if (!marv::params_init(marv::param_defaults())) {
    marv::hal_panic("params_init rejected the default table");
  }
  marv::composition::init();
  using marv::ParamId;
  marv::hal_target::HilConfig cfg;
  cfg.sink = &marv::composition::tick;
  cfg.rate_counts = &rate_counts;
  cfg.composition_status = &composition_status;
  cfg.param_schema_hash = marv::kParamSchemaHash;
  cfg.param_count = static_cast<std::uint32_t>(marv::kParamCount);
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
