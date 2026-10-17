#pragma once

// The bench HIL link measurement (HIL stage S1; fw/hal/target/src/hil_link.cpp, format in hil_link_format.hpp). Built only
// into the image with MARV_FC3V2_HIL_LINK (fw/hal/target/CMakeLists.txt). Background loop only; no interrupt touches it.

namespace marv::hal_target {

// Zeroes every counter and the sequence state: called when a host opens the port, so a run counts from its start.
void hil_link_reset() noexcept;

// Reads what the USB CDC port holds, validates the downlink records, writes a report per received sample and, once per
// second, the status record. Called as often as it can run while a host holds the port open.
void hil_link_service() noexcept;

}  // namespace marv::hal_target
