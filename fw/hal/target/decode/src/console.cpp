#include "marv/hal_target/console.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include "marv/hal/motor_test.hpp"
#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

constexpr char kSeparator = ' ';
constexpr char kCarriageReturn = '\r';
constexpr char kLineFeed = '\n';
constexpr char kDigit0 = '0';
constexpr char kDigit9 = '9';

// The command, the numbers and the unlock token.
constexpr std::size_t kRequestTokens = kRequestNumbers + 2;

[[nodiscard]] bool parse_u32(std::string_view token, std::uint32_t& out) noexcept {
  if (token.empty()) {
    return false;
  }
  std::uint64_t v = 0;
  for (const char c : token) {
    if (c < kDigit0 || c > kDigit9) {
      return false;
    }
    v = v * prim::kDecimalRadix + static_cast<std::uint64_t>(c - kDigit0);
    if (v > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
  }
  out = static_cast<std::uint32_t>(v);
  return true;
}

[[nodiscard]] ParsedLine refused(LineRefusal why) noexcept {
  ParsedLine p;
  p.kind = LineKind::Refused;
  p.refusal = why;
  return p;
}

}  // namespace

ParsedLine parse_line(std::string_view line) noexcept {
  if (line.size() > kLineCapacity) {
    return refused(LineRefusal::TooLong);
  }
  if (!line.empty() && line.back() == kCarriageReturn) {
    line.remove_suffix(1);
  }
  std::array<std::string_view, kRequestTokens> token{};
  std::size_t count = 0;
  std::size_t i = 0;
  while (i < line.size()) {
    if (line[i] == kSeparator) {
      ++i;
      continue;
    }
    std::size_t end = i;
    while (end < line.size() && line[end] != kSeparator) {
      ++end;
    }
    if (count < token.size()) {
      token[count] = line.substr(i, end - i);
    }
    ++count;
    i = end;
  }
  if (count == 0) {
    return ParsedLine{};
  }
  if (token[0] == kCmdStop) {
    if (count != 1) {
      return refused(LineRefusal::Arguments);
    }
    ParsedLine p;
    p.kind = LineKind::Stop;
    return p;
  }
  if (token[0] != kCmdMotorTest) {
    return refused(LineRefusal::UnknownCommand);
  }
  if (count != kRequestTokens) {
    return refused(count > kRequestTokens || token[count - 1] == kUnlockToken ? LineRefusal::Arguments
                                                                              : LineRefusal::MissingUnlock);
  }
  if (token[kRequestTokens - 1] != kUnlockToken) {
    return refused(LineRefusal::MissingUnlock);
  }
  ParsedLine p;
  p.kind = LineKind::Request;
  const std::array<std::uint32_t*, kRequestNumbers> field{&p.request.motor, &p.request.dshot, &p.request.duration_ms};
  for (std::size_t k = 0; k < field.size(); ++k) {
    if (!parse_u32(token[k + 1], *field[k])) {
      return refused(LineRefusal::NotANumber);
    }
  }
  return p;
}

bool LineBuffer::push(char c, ParsedLine& out) noexcept {
  if (c != kLineFeed) {
    if (len_ < buf_.size()) {
      buf_[len_++] = c;
    } else {
      overflow_ = true;
    }
    return false;
  }
  out = overflow_ ? refused(LineRefusal::TooLong) : parse_line(std::string_view{buf_.data(), len_});
  len_ = 0;
  overflow_ = false;
  return true;
}

}  // namespace marv::hal_target
