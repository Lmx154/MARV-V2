#pragma once

// The bench IMU log (quad spec §4.0 M3, packet P3a, decision 0042 item 2; fw/hal/target/src/imu_log.cpp). Built only into
// the image with MARV_FC3V2_IMU_LOG (fw/hal/target/CMakeLists.txt); the default image does not compile it.
//
// Every sample quad_fc's tick receives is copied, from the IMU interrupt, into a single-producer single-consumer ring in
// SRAM; the background loop packs the ring into framed records (imu_log_format.hpp) and writes them to the USB CDC port,
// with a status record once per second. The interrupt only copies and counts; it never touches USB.

#include "marv/types/imu_sample.hpp"

namespace marv::hal_target {

// The IMU interrupt only (the producer). A sample the ring has no room for is counted in the status record and dropped.
void imu_log_push(const ImuSample& s) noexcept;

// The background loop only (the consumer), called as often as it can run. While no host holds the port open it discards
// the ring, so a capture starts with the newest samples.
void imu_log_service() noexcept;

}  // namespace marv::hal_target
