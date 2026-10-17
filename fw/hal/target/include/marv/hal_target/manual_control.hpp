#pragma once

// The pilot-input hand-off on the FC3v2 (M2 packet P3, decision 0040 Q3; fw/hal/target/src/manual_control.cpp). The
// receive path (the receiver UART's decoder, a single writer that runs at a lower priority than the IMU interrupt) fills
// the slot the reader is not using and then flips an atomic index; the IMU interrupt (the tick, hal_manual_control) is the
// only reader. The reader preempts the writer and is never preempted by it, so it is never blocked and never sees a
// half-written snapshot; the first rate tick after the flip sees the new struct.

#include "marv/types/manual_control.hpp"

namespace marv::hal_target {

// Receive path only (one writer). Stores `m` as given: the producer sets t_us from hal_time_us() and frames (never 0 once
// a frame has arrived); the slot decides nothing.
void manual_control_publish(const ManualControl& m) noexcept;

}  // namespace marv::hal_target
