// The SIL's raw RC frame entry (marv_sil_stage_rc_frame, ground-segment unit GX-9, the SIL pilot bridge; invariant R-2): the
// ManualControl it stages is, bit for bit, the one rc_input::crsf_to_controls (the function the board's receiver calls) yields for
// the same 16 raw channels and the table's rc_channel_* map, with the frame count and the stamp of the frame, the link fields
// untouched.
//
// The entry's source (fw/sil/src/rc_frame.cpp) is compiled into this test, over hal_sim and the FC3v2 parameter set, and the
// staged value is read back through hal_manual_control(): the SIL library does not export it. params_init succeeds once per
// process, so every test runs in a forked child.
//
//  - BitIdenticalToCrsfToControls: edge frames (all 0, all 2047, the centre, the CRSF -100 % and +100 % ticks) and a deterministic
//    pseudo-random sequence, with the factory map (AETR).
//  - RemappedTableIsFollowed: the same with a table whose rc_channel_roll / _pitch / _throttle / _yaw are 3, 1, 4, 2; the control
//    is that the factory map decodes the same frames to other sticks, so a map that was not followed would fail.
//  - Refusals: not READY, a NULL frame, a struct_size or reserved word that is not the library's, a raw value above 2^11 - 1, and a
//    table whose map is not channel numbers each leave the staged control untouched.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>
#include <vector>

#include <marv/hal/hal.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/hal_target/crsf.hpp>
#include <marv/hal_target/rc_input.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/manual_control.hpp>

#include "sil_session.hpp"

namespace marv::sil {
namespace {
bool g_ready = false;
}
void set_ready(bool ready) noexcept { g_ready = ready; }
SessionView session_view() noexcept { return SessionView{g_ready, 0, hal_sim::TickPeriod{1, 1}}; }
}  // namespace marv::sil

namespace {

using namespace marv;
using hal_target::CrsfRcChannels;
using hal_target::RcChannelMap;

constexpr std::uint16_t kRawMax = 2047;  // 2^11 - 1: the CRSF channel field is 11 bits
constexpr std::uint16_t kRawCentre = 992;
constexpr std::uint16_t kRawMinus100 = 172;
constexpr std::uint16_t kRawPlus100 = 1811;
constexpr std::size_t kRandomFrames = 200;
constexpr std::uint32_t kLcgMul = 1664525;  // Numerical Recipes' linear congruential generator: a deterministic sequence, no tuning
constexpr std::uint32_t kLcgAdd = 1013904223;
constexpr std::uint64_t kFrameStepUs = 8000;  // the stamp advance per frame; any increasing sequence serves

static_assert(sizeof(ManualControl) == 104, "ManualControl has no padding: memcmp compares its fields");

bool report_failures_on_stderr() {
  const ::testing::TestResult* r = ::testing::UnitTest::GetInstance()->current_test_info()->result();
  bool any = false;
  for (int i = 0; i < r->total_part_count(); ++i) {
    const ::testing::TestPartResult& part = r->GetTestPartResult(i);
    if (part.failed()) {
      any = true;
      std::fprintf(stderr, "%s:%d: %s\n", part.file_name(), part.line_number(), part.message());
    }
  }
  return any;
}

template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

using Map = std::array<std::int32_t, 4>;  // roll, pitch, throttle, yaw

Map factory_map() {
  const auto d = param_defaults();
  const auto at = [&](ParamId id) { return d[static_cast<std::size_t>(id)].value.i32; };
  return {at(ParamId::rc_channel_roll), at(ParamId::rc_channel_pitch), at(ParamId::rc_channel_throttle),
          at(ParamId::rc_channel_yaw)};
}

// Boots hal_sim and the parameter table (the factory table with `map` as the four stick channels) and marks the SIL READY.
void boot(const Map& map) {
  static std::array<DshotValue, kQuadXMotors> motors;
  hal_sim::setup(hal_sim::TickPeriod{1, 1}, std::span<DshotValue>{motors}, std::span<ServoUs>{});
  std::array<ParamRecord, kParamCount> table{};
  const auto d = param_defaults();
  for (std::size_t i = 0; i < kParamCount; ++i) {
    table[i] = d[i];
  }
  const auto set = [&](ParamId id, std::int32_t v) { table[static_cast<std::size_t>(id)].value.i32 = v; };
  set(ParamId::rc_channel_roll, map[0]);
  set(ParamId::rc_channel_pitch, map[1]);
  set(ParamId::rc_channel_throttle, map[2]);
  set(ParamId::rc_channel_yaw, map[3]);
  ASSERT_TRUE(params_init(std::span<const ParamRecord, kParamCount>{table}));
  sil::set_ready(true);
}

marv_rc_frame frame_of(const std::array<std::uint16_t, MARV_RC_CHANNELS>& raw, std::uint64_t t_us) {
  marv_rc_frame f{};
  f.struct_size = sizeof(marv_rc_frame);
  f.t_us = t_us;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    f.raw[i] = raw[i];
  }
  return f;
}

std::vector<std::array<std::uint16_t, MARV_RC_CHANNELS>> frames() {
  std::vector<std::array<std::uint16_t, MARV_RC_CHANNELS>> out;
  for (const std::uint16_t v : {std::uint16_t{0}, kRawMax, kRawCentre, kRawMinus100, kRawPlus100}) {
    std::array<std::uint16_t, MARV_RC_CHANNELS> a{};
    a.fill(v);
    out.push_back(a);
  }
  std::uint32_t x = 1;
  for (std::size_t n = 0; n < kRandomFrames; ++n) {
    std::array<std::uint16_t, MARV_RC_CHANNELS> a{};
    for (std::uint16_t& v : a) {
      x = (x * kLcgMul) + kLcgAdd;
      v = static_cast<std::uint16_t>((x >> 16U) & kRawMax);
    }
    out.push_back(a);
  }
  return out;
}

// Stages every frame and compares the staged control with crsf_to_controls over `map`; returns how many frames differed in sticks
// from the same frames decoded with `other` (the control that the map matters).
std::size_t compare_all(const Map& map, const Map& other) {
  ManualControl expect{};
  ManualControl wrong{};
  std::size_t differing = 0;
  std::uint64_t t = 0;
  std::uint32_t n = 0;
  for (const auto& raw : frames()) {
    t += kFrameStepUs;
    const marv_rc_frame f = frame_of(raw, t);
    EXPECT_EQ(marv_sil_stage_rc_frame(&f), MARV_SIL_OK);
    CrsfRcChannels rc;
    for (std::size_t i = 0; i < raw.size(); ++i) {
      rc.raw[i] = raw[i];
    }
    EXPECT_TRUE(hal_target::crsf_to_controls(rc, RcChannelMap{map[0], map[1], map[2], map[3]}, expect));
    expect.t_us = t;
    expect.frames = ++n;
    EXPECT_TRUE(hal_target::crsf_to_controls(rc, RcChannelMap{other[0], other[1], other[2], other[3]}, wrong));
    if (wrong.roll != expect.roll || wrong.pitch != expect.pitch || wrong.yaw != expect.yaw || wrong.throttle != expect.throttle) {
      ++differing;
    }
    const ManualControl got = hal_manual_control();
    EXPECT_EQ(std::memcmp(&got, &expect, sizeof got), 0) << "frame " << n;
    EXPECT_EQ(got.link_quality, 0.0F);
    EXPECT_EQ(got.link_t_us, 0U);
  }
  return differing;
}

TEST(SilRcFrame, BitIdenticalToCrsfToControls) {
  in_child([] {
    const Map m = factory_map();
    boot(m);
    compare_all(m, m);
  });
}

TEST(SilRcFrame, RemappedTableIsFollowed) {
  in_child([] {
    const Map remap{3, 1, 4, 2};
    const Map factory = factory_map();
    ASSERT_NE(remap, factory);
    boot(remap);
    EXPECT_GT(compare_all(remap, factory), frames().size() / 2) << "the factory map must decode the frames to other sticks";
  });
}

TEST(SilRcFrame, Refusals) {
  in_child([] {
    const Map m = factory_map();
    ASSERT_NO_FATAL_FAILURE(boot(m));
    sil::set_ready(false);
    std::array<std::uint16_t, MARV_RC_CHANNELS> raw{};
    raw.fill(kRawPlus100);
    const marv_rc_frame good = frame_of(raw, kFrameStepUs);
    EXPECT_EQ(marv_sil_stage_rc_frame(&good), MARV_SIL_E_STATE);
    sil::set_ready(true);
    EXPECT_EQ(marv_sil_stage_rc_frame(nullptr), MARV_SIL_E_NULL);
    marv_rc_frame bad = good;
    bad.struct_size += 1;
    EXPECT_EQ(marv_sil_stage_rc_frame(&bad), MARV_SIL_E_ABI);
    bad = good;
    bad.reserved = 1;
    EXPECT_EQ(marv_sil_stage_rc_frame(&bad), MARV_SIL_E_ABI);
    bad = good;
    bad.raw[MARV_RC_CH_16] = kRawMax + 1;
    EXPECT_EQ(marv_sil_stage_rc_frame(&bad), MARV_SIL_E_INPUT);
    const ManualControl none{};
    const ManualControl got = hal_manual_control();
    EXPECT_EQ(std::memcmp(&got, &none, sizeof got), 0) << "an error stages nothing";
    EXPECT_EQ(marv_sil_stage_rc_frame(&good), MARV_SIL_OK);  // the control: the same frame is accepted
    EXPECT_EQ(hal_manual_control().frames, 1U);
  });
}

TEST(SilRcFrame, ATableWhoseMapIsNotChannelNumbersIsRefused) {
  in_child([] {
    ASSERT_NO_FATAL_FAILURE(boot(Map{0, 2, 3, 4}));
    std::array<std::uint16_t, MARV_RC_CHANNELS> raw{};
    raw.fill(kRawCentre);
    const marv_rc_frame f = frame_of(raw, kFrameStepUs);
    EXPECT_EQ(marv_sil_stage_rc_frame(&f), MARV_SIL_E_PARAM);
    EXPECT_EQ(hal_manual_control().frames, 0U);
  });
}

}  // namespace
