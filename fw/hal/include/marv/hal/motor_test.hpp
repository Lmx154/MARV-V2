#pragma once

// The console motor test of the FC3v2 (M1 packet P11, decision 0038): the types of the request that reaches the
// quad_fc motor output gate through hal_motor_test() and of the events the gate reports back through
// hal_motor_test_report(). The console parses and forwards; the composition validates and decides every motor value.

#include <cstdint>

namespace marv {

// A request as parsed: unchecked numbers. The composition refuses a motor outside 1..4, a value outside
// [kDshotThrottleMin, motor_test_dshot_max] and a duration outside [1 ms, motor_test_duration_max_ms].
struct MotorTestRequest {
  std::uint32_t motor = 0;  // logical quad-X motor number
  std::uint32_t dshot = 0;
  std::uint32_t duration_ms = 0;
};

enum class MotorTestCancel : std::uint8_t { Cancel, Disconnect };

// What the HAL holds for the gate, one consistent snapshot. cancel_count increases on every cancel signal and
// request_count on every request (counters wrap); the gate acts on a count that differs from the one it saw last,
// cancel first, then the request. request_cancel_count is cancel_count as it was when the request was published: a
// request whose request_cancel_count differs from cancel_count was cancelled after it was published, and the gate
// refuses it (Cancel) instead of starting it.
struct MotorTestPoll {
  std::uint32_t cancel_count = 0;
  MotorTestCancel cancel_reason = MotorTestCancel::Cancel;
  std::uint32_t request_count = 0;
  std::uint32_t request_cancel_count = 0;
  MotorTestRequest request{};
};

enum class MotorTestEventKind : std::uint8_t { Accepted, Refused, Stopped };

enum class MotorTestReason : std::uint8_t {
  None,        // Accepted
  Motor,       // Refused: motor number outside 1..4
  Value,       // Refused: DShot value outside [kDshotThrottleMin, motor_test_dshot_max]
  Duration,    // Refused: duration outside [1 ms, motor_test_duration_max_ms]
  Deadline,    // Stopped: the duration has passed (or the stamps could not be trusted to count it)
  Cancel,      // Stopped: a received byte or the command stop; Refused: cancelled before it started
  Disconnect   // Stopped: the USB port was closed
};

struct MotorTestEvent {
  MotorTestEventKind kind = MotorTestEventKind::Accepted;
  MotorTestReason reason = MotorTestReason::None;
  MotorTestRequest request{};       // the request, or the running test's motor, value and duration
  std::uint32_t accepted = 0;       // counters since init, including this event
  std::uint32_t refused = 0;
  std::uint32_t stopped = 0;
};

}  // namespace marv
