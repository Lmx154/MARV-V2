// T1 tests of the CRSF frame decoder (fw/hal/target/decode/crsf.hpp, M2 P5a). Expected values are restated from the TBS CRSF
// spec (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md: frame construction, CRC8 polynomial 0xD5 over type and
// payload, the 0x14 and 0x16 layouts) and from two published byte vectors, or computed by hand, not read from the code
// under test:
//   - the spec's CRC table, first row (the CRC8 of the single byte 0 .. 15);
//   - the spec's two 0x32.0x13 RC-over-WiFi example frames "0xC8 0x07 0x32 0xEE 0x13 0x13 0x01 0x66 0x06" and
//     "... 0x02 0x12 0x8E" (their CRC bytes are CRC8 0xD5 of type and payload);
//   - the CRSF working group's 0x16 example, all 16 channels at 992
//     (https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_RC_CHANNELS_PACKED):
//     "c8 18 16 e0 03 1f f8 c0 07 3e f0 81 0f 7c e0 03 1f f8 c0 07 3e f0 81 0f 7c ad".
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <marv/hal_target/crsf.hpp>

namespace marv::hal_target {
namespace {

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint8_t kSync = 0xC8;
constexpr std::uint8_t kRc = 0x16;
constexpr std::uint8_t kLs = 0x14;

// sync, length, type, payload, CRC. The CRC is the code under test's (checked against published values in CrcKnownAnswers).
Bytes frame(std::uint8_t type, const Bytes& payload, std::uint8_t sync = kSync) {
  Bytes f{sync, static_cast<std::uint8_t>(payload.size() + 2), type};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(crsf_crc8(f.data() + 2, f.size() - 2));
  return f;
}

// The packed payload of channel values, one bit at a time (a different method from the decoder's accumulator).
Bytes pack(const std::array<std::uint16_t, 16>& ch) {
  Bytes p(22, 0);
  for (std::size_t i = 0; i < 16; ++i) {
    for (std::size_t b = 0; b < 11; ++b) {
      if (((ch[i] >> b) & 1U) != 0) {
        const std::size_t bit = i * 11 + b;
        p[bit / 8] = static_cast<std::uint8_t>(p[bit / 8] | (1U << (bit % 8)));
      }
    }
  }
  return p;
}

std::vector<CrsfEvent> feed(CrsfDecoder& d, const Bytes& in) {
  std::vector<CrsfEvent> ev;
  for (const std::uint8_t b : in) {
    const CrsfEvent e = d.push(b);
    if (e != CrsfEvent::None) {
      ev.push_back(e);
    }
  }
  return ev;
}

std::array<std::uint16_t, 16> pattern() {
  std::array<std::uint16_t, 16> ch{};
  for (std::size_t i = 0; i < ch.size(); ++i) {
    ch[i] = static_cast<std::uint16_t>((i * 0x1A7U + 0x23U) & 0x7FFU);
  }
  return ch;
}

const Bytes kLsPayload{0x28, 0x2B, 0x64, 0xFB, 0x01, 0x02, 0x03, 0x2D, 0x5F, 0x07};  // up_snr = -5

TEST(Crsf, CrcKnownAnswers) {
  const std::array<std::uint8_t, 16> row{0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
                                         0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D};
  for (std::uint8_t i = 0; i < row.size(); ++i) {
    EXPECT_EQ(crsf_crc8(&i, 1), row[i]) << "byte " << int{i};
  }
  const std::array<std::uint8_t, 6> on{0x32, 0xEE, 0x13, 0x13, 0x01, 0x66};
  const std::array<std::uint8_t, 6> off{0x32, 0xEE, 0x13, 0x13, 0x02, 0x12};
  EXPECT_EQ(crsf_crc8(on.data(), on.size()), 0x06);
  EXPECT_EQ(crsf_crc8(off.data(), off.size()), 0x8E);
  EXPECT_EQ(crsf_crc8(nullptr, 0), 0x00);
}

TEST(Crsf, PublishedRcFrameAllChannels992) {
  const Bytes f{0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xe0,
                0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xad};
  CrsfDecoder d;
  EXPECT_EQ(feed(d, f), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  for (const std::uint16_t v : d.rc().raw) {
    EXPECT_EQ(v, 992);
  }
  EXPECT_EQ(d.counters().rc_frames, 1U);
  EXPECT_EQ(d.counters().bad_crc, 0U);
  EXPECT_EQ(d.counters().dropped_bytes, 0U);
}

TEST(Crsf, UnpackExtremesAndHandComputedPatterns) {
  Bytes zero(22, 0x00);
  Bytes full(22, 0xFF);
  for (const std::uint16_t v : crsf_unpack_channels(zero.data()).raw) {
    EXPECT_EQ(v, 0);
  }
  for (const std::uint16_t v : crsf_unpack_channels(full.data()).raw) {
    EXPECT_EQ(v, 2047);
  }
  // channel_01 = 1 is bit 0; channel_02 = 1 is bit 11 (byte 1, bit 3); channel_16 = 0x7FF is bits 165 .. 175
  // (byte 20 bits 5 .. 7 and byte 21).
  Bytes p(22, 0x00);
  p[0] = 0x01;
  p[1] = 0x08;
  p[20] = 0xE0;
  p[21] = 0xFF;
  const CrsfRcChannels c = crsf_unpack_channels(p.data());
  for (std::size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(c.raw[i], i == 0 || i == 1 ? 1 : (i == 15 ? 2047 : 0)) << "channel " << i + 1;
  }
}

TEST(Crsf, UnpackPatternMatchesBitwisePack) {
  const auto ch = pattern();
  const Bytes p = pack(ch);
  EXPECT_EQ(crsf_unpack_channels(p.data()).raw, ch);
  CrsfDecoder d;
  EXPECT_EQ(feed(d, frame(kRc, p)), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  EXPECT_EQ(d.rc().raw, ch);
}

TEST(Crsf, LinkStatsFields) {
  CrsfDecoder d;
  EXPECT_EQ(feed(d, frame(kLs, kLsPayload)), (std::vector<CrsfEvent>{CrsfEvent::LinkStats}));
  const CrsfLinkStats& l = d.link();
  EXPECT_EQ(l.up_rssi_ant1, 0x28);
  EXPECT_EQ(l.up_rssi_ant2, 0x2B);
  EXPECT_EQ(l.up_link_quality, 0x64);
  EXPECT_EQ(l.up_snr, -5);
  EXPECT_EQ(l.active_antenna, 0x01);
  EXPECT_EQ(l.rf_profile, 0x02);
  EXPECT_EQ(l.up_rf_power, 0x03);
  EXPECT_EQ(l.down_rssi, 0x2D);
  EXPECT_EQ(l.down_link_quality, 0x5F);
  EXPECT_EQ(l.down_snr, 7);
  EXPECT_EQ(d.counters().link_stats_frames, 1U);
}

TEST(Crsf, FrameSplitAcrossArbitraryByteBoundaries) {
  const auto ch = pattern();
  Bytes stream = frame(kRc, pack(ch));
  const Bytes ls = frame(kLs, kLsPayload);
  stream.insert(stream.end(), ls.begin(), ls.end());
  // Every chunking of the stream into pieces of 1 .. 9 bytes, the pieces fed byte by byte as a UART read would.
  for (std::size_t chunk = 1; chunk <= 9; ++chunk) {
    CrsfDecoder d;
    std::vector<CrsfEvent> ev;
    for (std::size_t at = 0; at < stream.size(); at += chunk) {
      const Bytes piece(stream.begin() + static_cast<std::ptrdiff_t>(at),
                        stream.begin() + static_cast<std::ptrdiff_t>(std::min(at + chunk, stream.size())));
      const auto e = feed(d, piece);
      ev.insert(ev.end(), e.begin(), e.end());
    }
    EXPECT_EQ(ev, (std::vector<CrsfEvent>{CrsfEvent::RcChannels, CrsfEvent::LinkStats})) << "chunk " << chunk;
    EXPECT_EQ(d.rc().raw, ch) << "chunk " << chunk;
    EXPECT_EQ(d.link().down_snr, 7) << "chunk " << chunk;
    EXPECT_EQ(d.counters().bad_crc + d.counters().bad_length + d.counters().dropped_bytes, 0U) << "chunk " << chunk;
  }
}

TEST(Crsf, ResyncAfterGarbage) {
  const auto ch = pattern();
  // Garbage with sync-looking bytes, bogus lengths and a 0x00 address, then a good frame.
  Bytes in{0x12, 0x34, kSync, 0xFF, 0x00, 0x01, kSync, 0x00, 0x00, 0x99};
  const Bytes good = frame(kRc, pack(ch));
  in.insert(in.end(), good.begin(), good.end());
  CrsfDecoder d;
  EXPECT_EQ(feed(d, in), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  EXPECT_EQ(d.rc().raw, ch);
  EXPECT_EQ(d.counters().rc_frames, 1U);
  EXPECT_GT(d.counters().dropped_bytes, 0U);
}

TEST(Crsf, FrameInsideAFailedCandidateIsFound) {
  // A sync byte and a plausible length (0x18) just before a real 0x16 frame: the candidate swallows the real frame's
  // first 24 bytes, fails its CRC, and the real frame must still be found on the rescan.
  const auto ch = pattern();
  Bytes in{kSync, 0x18};
  const Bytes good = frame(kRc, pack(ch));
  in.insert(in.end(), good.begin(), good.end());
  CrsfDecoder d;
  EXPECT_EQ(feed(d, in), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  EXPECT_EQ(d.rc().raw, ch);
  EXPECT_EQ(d.counters().bad_crc, 1U);
}

TEST(Crsf, BadCrcCountedAndDropped) {
  const auto ch = pattern();
  CrsfDecoder d;
  ASSERT_EQ(feed(d, frame(kRc, pack(ch))).size(), 1U);
  auto other = ch;
  other[0] ^= 0x7FFU;
  const Bytes good = frame(kRc, pack(other));
  std::uint32_t expected = 0;
  // A flipped bit in the type, any payload byte or the CRC: no event, the last good channels stay, one more bad CRC.
  for (std::size_t at = 2; at < good.size(); ++at) {
    Bytes bad = good;
    bad[at] = static_cast<std::uint8_t>(bad[at] ^ 0x10U);
    const std::uint32_t before = d.counters().rc_frames + d.counters().skipped_frames + d.counters().link_stats_frames;
    const auto ev = feed(d, bad);
    const std::uint32_t after = d.counters().rc_frames + d.counters().skipped_frames + d.counters().link_stats_frames;
    EXPECT_EQ(after, before) << "byte " << at;
    EXPECT_TRUE(ev.empty()) << "byte " << at;
    EXPECT_GT(d.counters().bad_crc, expected) << "byte " << at;
    expected = d.counters().bad_crc;
    EXPECT_EQ(d.rc().raw, ch) << "byte " << at;
  }
  EXPECT_EQ(feed(d, good), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  EXPECT_EQ(d.rc().raw, other);
}

TEST(Crsf, LengthOutOfRangeCountedAndDropped) {
  const auto ch = pattern();
  const Bytes good = frame(kRc, pack(ch));
  // Valid range is 2 .. 62 (spec "Frame Details"): 0, 1, 63 and 255 are discarded, then a good frame still decodes.
  for (const std::uint8_t len : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{63}, std::uint8_t{255}}) {
    CrsfDecoder d;
    Bytes in{kSync, len, kRc, 0x00, 0x00};
    in.insert(in.end(), good.begin(), good.end());
    const auto ev = feed(d, in);
    EXPECT_EQ(ev, (std::vector<CrsfEvent>{CrsfEvent::RcChannels})) << "length " << int{len};
    EXPECT_GE(d.counters().bad_length, 1U) << "length " << int{len};
    EXPECT_EQ(d.rc().raw, ch) << "length " << int{len};
  }
}

TEST(Crsf, LongestAndShortestValidFrames) {
  CrsfDecoder d;
  // Length 62: a 64-byte frame of an unknown type; length 2: type and CRC only.
  EXPECT_EQ(feed(d, frame(0x7A, Bytes(60, 0xA5))), (std::vector<CrsfEvent>{CrsfEvent::Skipped}));
  EXPECT_EQ(feed(d, frame(0x7A, Bytes{})), (std::vector<CrsfEvent>{CrsfEvent::Skipped}));
  EXPECT_EQ(d.counters().skipped_frames, 2U);
  EXPECT_EQ(d.counters().bad_length + d.counters().bad_crc + d.counters().dropped_bytes, 0U);
}

TEST(Crsf, OtherTypesCountedAndSkipped) {
  CrsfDecoder d;
  const auto ch = pattern();
  ASSERT_EQ(feed(d, frame(kRc, pack(ch))).size(), 1U);
  // 0x08 battery sensor (4 payload bytes in the spec), 0x21 flight mode text.
  EXPECT_EQ(feed(d, frame(0x08, Bytes{0, 1, 0, 0})), (std::vector<CrsfEvent>{CrsfEvent::Skipped}));
  EXPECT_EQ(feed(d, frame(0x21, Bytes{'A', 'C', 'R', 'O', 0})), (std::vector<CrsfEvent>{CrsfEvent::Skipped}));
  EXPECT_EQ(d.counters().skipped_frames, 2U);
  EXPECT_EQ(d.rc().raw, ch);
}

TEST(Crsf, SyncBytes) {
  const auto ch = pattern();
  CrsfDecoder d;
  // The broadcast address 0x00 is a sync byte; an address that is neither sync byte (0xEE, the transmitter module) is not.
  EXPECT_EQ(feed(d, frame(kRc, pack(ch), 0x00)), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  CrsfDecoder e;
  EXPECT_TRUE(feed(e, frame(kRc, pack(ch), 0xEE)).empty());
  EXPECT_EQ(e.counters().rc_frames, 0U);
}

TEST(Crsf, ExtraPayloadIgnoredShortPayloadRejected) {
  const auto ch = pattern();
  CrsfDecoder d;
  Bytes longer = pack(ch);
  longer.push_back(0xEE);  // a newer protocol version's extra field
  EXPECT_EQ(feed(d, frame(kRc, longer)), (std::vector<CrsfEvent>{CrsfEvent::RcChannels}));
  EXPECT_EQ(d.rc().raw, ch);
  Bytes ls_longer = kLsPayload;
  ls_longer.push_back(0xEE);
  EXPECT_EQ(feed(d, frame(kLs, ls_longer)), (std::vector<CrsfEvent>{CrsfEvent::LinkStats}));

  CrsfDecoder s;
  EXPECT_TRUE(feed(s, frame(kRc, Bytes(21, 0xFF))).empty());
  EXPECT_TRUE(feed(s, frame(kLs, Bytes(9, 0xFF))).empty());
  EXPECT_EQ(s.counters().bad_length, 2U);
  EXPECT_EQ(s.counters().rc_frames + s.counters().link_stats_frames, 0U);
}

}  // namespace
}  // namespace marv::hal_target
