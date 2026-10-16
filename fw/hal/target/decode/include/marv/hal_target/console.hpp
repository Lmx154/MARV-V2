#pragma once

// The FC3v2 console's line parser (M1 packet P11, decision 0038), without the Pico SDK so the L09 T1 tests build it on
// the host. The console only parses and forwards: the numbers of a request are checked by the quad_fc gate, not here.
//
// Commands, one per line:
//   motor_test <motor> <dshot> <ms> PROPS-OFF   a request; the literal token PROPS-OFF is the explicit unlock
//                                               (stateless: it is on the line or the line is refused)
//   stop                                        cancels a running test
// Numbers are decimal digits only (no sign), at most what a uint32_t holds. Tokens are separated by spaces; a carriage
// return before the line feed is ignored. An empty line is nothing. Any other line is refused with the reason.

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include "marv/hal/motor_test.hpp"

namespace marv::hal_target {

inline constexpr std::string_view kCmdMotorTest = "motor_test";
inline constexpr std::string_view kCmdStop = "stop";
inline constexpr std::string_view kUnlockToken = "PROPS-OFF";

enum class LineKind : std::uint8_t { Empty, Request, Stop, Refused };

enum class LineRefusal : std::uint8_t {
  None,
  UnknownCommand,  // the first token is not a command
  Arguments,       // wrong number of tokens
  NotANumber,      // a number is empty, has a non-digit, or does not fit a uint32_t
  MissingUnlock,   // the last token is not PROPS-OFF
  TooLong          // the line exceeds kLineCapacity
};

struct ParsedLine {
  LineKind kind = LineKind::Empty;
  LineRefusal refusal = LineRefusal::None;  // Refused only
  MotorTestRequest request{};               // Request only
};

// Parses one line without its terminator (a trailing carriage return is dropped).
[[nodiscard]] ParsedLine parse_line(std::string_view line) noexcept;

// The longest line that can be a request: the command, the unlock token and the numbers of the widest uint32_t, with a
// separator before each (a request has one more token than numbers: the count of separators), and a carriage return.
inline constexpr std::size_t kMaxNumberDigits = static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::digits10) + 1;
// The numbers of a request are its fields (motor, dshot, duration).
inline constexpr std::size_t kRequestNumbers = sizeof(MotorTestRequest) / sizeof(std::uint32_t);
inline constexpr std::size_t kLineCapacity =
    kCmdMotorTest.size() + kUnlockToken.size() + kRequestNumbers * (kMaxNumberDigits + 1) + 2;

// Accumulates received bytes into lines. A line longer than kLineCapacity is dropped to its end and refused (TooLong).
class LineBuffer {
 public:
  // Feeds one byte; returns true when a line is complete (a line feed arrived), with its parse in `out`.
  [[nodiscard]] bool push(char c, ParsedLine& out) noexcept;

 private:
  std::array<char, kLineCapacity> buf_{};
  std::size_t len_ = 0;
  bool overflow_ = false;
};

}  // namespace marv::hal_target
