// T1 tests of the FC3v2 console line parser (fw/hal/target/decode/console.hpp, M1 P11, decision 0038): the motor test
// command `motor_test <1-4> <dshot> <ms> PROPS-OFF` with its literal unlock token, the command stop, and the refusals.
// The parser checks the form only; the numbers' limits are the quad_fc gate's (quad_fc_gate_test.cpp). Expected values are
// stated by hand.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

#include <marv/hal_target/console.hpp>

namespace marv::hal_target {
namespace {

void expect_request(std::string_view line, std::uint32_t motor, std::uint32_t dshot, std::uint32_t ms) {
  const ParsedLine p = parse_line(line);
  ASSERT_EQ(p.kind, LineKind::Request) << line;
  EXPECT_EQ(p.request.motor, motor) << line;
  EXPECT_EQ(p.request.dshot, dshot) << line;
  EXPECT_EQ(p.request.duration_ms, ms) << line;
}

void expect_refused(std::string_view line, LineRefusal why) {
  const ParsedLine p = parse_line(line);
  EXPECT_EQ(p.kind, LineKind::Refused) << line;
  EXPECT_EQ(p.refusal, why) << line;
}

TEST(Console, TheUnlockedFormIsARequest) {
  expect_request("motor_test 1 100 500 PROPS-OFF", 1, 100, 500);
  expect_request("motor_test 4 500 3000 PROPS-OFF", 4, 500, 3000);
  expect_request("  motor_test   2   48   1   PROPS-OFF  ", 2, 48, 1);
  expect_request("motor_test 1 100 500 PROPS-OFF\r", 1, 100, 500);
  // The parser does not check limits: those lines are requests, for the gate to refuse.
  expect_request("motor_test 0 47 0 PROPS-OFF", 0, 47, 0);
  expect_request("motor_test 5 600 3001 PROPS-OFF", 5, 600, 3001);
  expect_request("motor_test 007 0100 0500 PROPS-OFF", 7, 100, 500);
  expect_request("motor_test 4294967295 4294967295 4294967295 PROPS-OFF", 4294967295U, 4294967295U, 4294967295U);
}

TEST(Console, WithoutTheUnlockTokenTheLineIsRefused) {
  expect_refused("motor_test 1 100 500", LineRefusal::MissingUnlock);
  expect_refused("motor_test 1 100 500 props-off", LineRefusal::MissingUnlock);
  expect_refused("motor_test 1 100 500 PROPS_OFF", LineRefusal::MissingUnlock);
  expect_refused("motor_test 1 100 500 PROPS-OFF1", LineRefusal::MissingUnlock);
  expect_refused("motor_test 1 100 500 yes", LineRefusal::MissingUnlock);
  expect_refused("motor_test 1 100 PROPS-OFF", LineRefusal::Arguments);
  expect_refused("motor_test PROPS-OFF", LineRefusal::Arguments);
  expect_refused("motor_test", LineRefusal::MissingUnlock);
}

TEST(Console, MalformedLinesAreRefusedWithTheirReason) {
  expect_refused("motor_test 1 100 500 PROPS-OFF extra", LineRefusal::Arguments);
  expect_refused("PROPS-OFF motor_test 1 100 500", LineRefusal::UnknownCommand);
  expect_refused("motor_test 1 100 500 PROPS-OFF PROPS-OFF", LineRefusal::Arguments);
  expect_refused("motor_test -1 100 500 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test +1 100 500 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 1 1e2 500 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 1 0x64 500 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 1 100 5.5 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 1 100 4294967296 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 99999999999999999999 100 500 PROPS-OFF", LineRefusal::NotANumber);
  expect_refused("motor_test 1\t100 500 PROPS-OFF", LineRefusal::Arguments);
  expect_refused("spin 1 100 500 PROPS-OFF", LineRefusal::UnknownCommand);
  expect_refused("Motor_Test 1 100 500 PROPS-OFF", LineRefusal::UnknownCommand);
  expect_refused("stop now", LineRefusal::Arguments);
  expect_refused("stopp", LineRefusal::UnknownCommand);
}

TEST(Console, StopAndEmptyLines) {
  EXPECT_EQ(parse_line("stop").kind, LineKind::Stop);
  EXPECT_EQ(parse_line(" stop\r").kind, LineKind::Stop);
  EXPECT_EQ(parse_line("").kind, LineKind::Empty);
  EXPECT_EQ(parse_line("   ").kind, LineKind::Empty);
  EXPECT_EQ(parse_line("\r").kind, LineKind::Empty);
}

TEST(Console, TheLongestRequestFitsTheLineCapacityAndALongerLineIsRefused) {
  const std::string longest = "motor_test 4294967295 4294967295 4294967295 PROPS-OFF\r";
  EXPECT_LE(longest.size(), kLineCapacity);
  EXPECT_EQ(parse_line(longest).kind, LineKind::Request);
  EXPECT_EQ(parse_line(std::string(kLineCapacity + 1, ' ')).refusal, LineRefusal::TooLong);

  LineBuffer b;
  ParsedLine p;
  for (const char c : longest) {
    EXPECT_FALSE(b.push(c, p));
  }
  ASSERT_TRUE(b.push('\n', p));
  EXPECT_EQ(p.kind, LineKind::Request);
  for (std::size_t i = 0; i < 2 * kLineCapacity; ++i) {
    EXPECT_FALSE(b.push('x', p));
  }
  ASSERT_TRUE(b.push('\n', p));
  EXPECT_EQ(p.kind, LineKind::Refused);
  EXPECT_EQ(p.refusal, LineRefusal::TooLong);
  // The next line is parsed afresh.
  for (const char c : std::string_view{"stop"}) {
    EXPECT_FALSE(b.push(c, p));
  }
  ASSERT_TRUE(b.push('\n', p));
  EXPECT_EQ(p.kind, LineKind::Stop);
}

TEST(Console, ALineIsOnlyCompleteAtItsLineFeedAndTheBufferParsesEachLine) {
  LineBuffer b;
  ParsedLine p;
  const std::string_view text = "motor_test 3 77 20 PROPS-OFF\nmotor_test 3 77 20\n\nstop\n";
  std::size_t lines = 0;
  ParsedLine got[4];
  for (const char c : text) {
    if (b.push(c, p)) {
      ASSERT_LT(lines, 4U);
      got[lines++] = p;
    }
  }
  ASSERT_EQ(lines, 4U);
  EXPECT_EQ(got[0].kind, LineKind::Request);
  EXPECT_EQ(got[0].request.motor, 3U);
  EXPECT_EQ(got[1].kind, LineKind::Refused);
  EXPECT_EQ(got[1].refusal, LineRefusal::MissingUnlock);
  EXPECT_EQ(got[2].kind, LineKind::Empty);
  EXPECT_EQ(got[3].kind, LineKind::Stop);
}

}  // namespace
}  // namespace marv::hal_target
