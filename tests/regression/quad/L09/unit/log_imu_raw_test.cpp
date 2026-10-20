// The raw-count IMU record of log format version 2 (fw/log, unit U4-3; invariant V2-1 of the architect ruling 2026-10-20).
//
//  LogImuRaw   the layout (one cell, known bytes); the lossless claim: for every int16 count on every axis of both sensors, at the
//              full scales the firmware runs the part at and each mounting sign, the sample the firmware's own decode makes
//              (Lsm6dsvAssembler) comes back from its 16-byte record with the same flags and the same binary32 bit patterns,
//              and so the 32-byte Imu record written from it is byte-equal; a sample that is not whole counts of the scales is
//              refused by the encoder and the ring then writes the Imu record; the ring counts both kinds as the Imu stream.
//  LogImuRawRoundTrip   with MARV_LOG_IMU_RAW_DIR set, writes imu_raw.mlog (header, then for every count of every mounting the
//              32-byte record and the 16-byte record of the same sample, through the ring and the sector framer) for
//              tools/log/roundtrip_check.py --imu-raw, which decodes it with the Python decoder and requires the two to be equal.
#include <gtest/gtest.h>

#include <array>
#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <marv/hal_target/lsm6dsv_decode.hpp>

#include "marv/log/format.hpp"
#include "marv/log/framer.hpp"
#include "marv/log/header.hpp"
#include "marv/log/records.hpp"
#include "marv/log/ring.hpp"

namespace {

using namespace marv;
using namespace marv::log;
using hal_target::FifoWord;
using hal_target::ImuMounting;
using hal_target::Lsm6dsvAssembler;
using hal_target::Lsm6dsvScales;
using hal_target::WordResult;
using Bytes = std::vector<std::uint8_t>;

constexpr std::uint8_t kGyroTag = 0x01;
constexpr std::uint8_t kAccelTag = 0x02;
constexpr std::size_t kCounts = std::size_t{1} << 16;
constexpr std::int32_t kCountMin = -(1 << 15);
constexpr std::size_t kThird = kCounts / 3;

// The mountings the decode supports: the board's map, all plus, all minus, and a permuted one.
const std::array<ImuMounting, 4> kMountings{{ImuMounting{{0, 1, 2}, {-1, 1, -1}}, ImuMounting{{0, 1, 2}, {1, 1, 1}},
                                             ImuMounting{{0, 1, 2}, {-1, -1, -1}}, ImuMounting{{2, 0, 1}, {1, -1, 1}}}};

FifoWord word(std::uint8_t sensor, std::uint8_t slot, const std::array<std::int16_t, 3>& v) {
  FifoWord w{};
  w[0] = static_cast<std::uint8_t>((sensor << 3) | (slot << 1));
  for (std::size_t a = 0; a < 3; ++a) {
    const auto u = static_cast<std::uint16_t>(v[a]);
    w[1 + 2 * a] = static_cast<std::uint8_t>(u & 0xFF);
    w[2 + 2 * a] = static_cast<std::uint8_t>(u >> 8);
  }
  return w;
}

ImuRawScales scales_of(const ImuMounting& m, const Lsm6dsvScales& s) {
  ImuRawScales r;
  r.gyro_rad_s_per_lsb = s.gyro_rad_s_per_lsb;
  r.accel_m_s2_per_lsb = s.accel_m_s2_per_lsb;
  for (std::size_t i = 0; i < 3; ++i) {
    r.sign[i] = static_cast<std::int8_t>(m.sign[i]);
  }
  return r;
}

// The sample the firmware's decode makes from counts i + k * third (k = 0..2 gyro, 3..5 accel) as chip values.
ImuSample sample_of(Lsm6dsvAssembler& a, std::size_t i, std::uint64_t t_us) {
  std::array<std::int16_t, 3> g{};
  std::array<std::int16_t, 3> x{};
  for (std::size_t k = 0; k < 3; ++k) {
    g[k] = static_cast<std::int16_t>(static_cast<std::int32_t>((i + k * kThird) % kCounts) + kCountMin);
    x[k] = static_cast<std::int16_t>(static_cast<std::int32_t>((i + (k + 3) * kThird) % kCounts) + kCountMin);
  }
  ImuSample s{};
  EXPECT_EQ(a.push(word(kGyroTag, 1, g), s), WordResult::Held);
  EXPECT_EQ(a.push(word(kAccelTag, 1, x), s), WordResult::Sample);
  s.t_us = t_us;
  return s;
}

bool same_bits(const prim::Vec3<float>& a, const prim::Vec3<float>& b) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::bit_cast<std::uint32_t>(a[i]) != std::bit_cast<std::uint32_t>(b[i])) {
      return false;
    }
  }
  return true;
}

TEST(LogImuRaw, TheRecordIsOneCellWithTheDescribedLayout) {
  EXPECT_EQ(imu_raw::kBytes, kCellBytes);
  EXPECT_EQ(imu_raw::kBytes, 16U);
  EXPECT_EQ(imu_raw::kOffFlags, 1U);
  EXPECT_EQ(imu_raw::kOffTime, 2U);
  EXPECT_EQ(imu_raw::kOffGyro, 4U);
  EXPECT_EQ(imu_raw::kOffAccel, 10U);
  EXPECT_EQ(ring_record_bytes(Kind::ImuRaw), kCellBytes);
  EXPECT_EQ(kFormatVersion, 2U);
  EXPECT_EQ(imu::kBytes, 32U) << "version 1's Imu record is unchanged";

  ImuRawScales sc;
  sc.gyro_rad_s_per_lsb = 0.5F;
  sc.accel_m_s2_per_lsb = 0.25F;
  sc.sign = {-1, 1, -1};
  ImuSample s{};
  s.t_us = 0x1122334455667788ULL;
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid) | imu_flag(ImuFlag::TempValid) |
            imu_flag(ImuFlag::GyroSatX) | imu_flag(ImuFlag::AccelSatZ);
  s.gyro_rad_s = {0.5F * 32768.0F, 0.5F * -2.0F, -0.5F * 3.0F};  // counts: chip -32768 (FRD +32768), -2, 3
  s.accel_m_s2 = {-0.25F * 1.0F, 0.25F * 258.0F, -0.25F * 32767.0F};
  const auto r = encode_imu_raw(s, sc);
  ASSERT_TRUE(r.has_value());
  const std::array<std::uint8_t, 16> expect{
      0x0D, 0x07, 0x88, 0x77,                          // kind ImuRaw, flags (gyro, accel, temp valid), t_lo16
      0x00, 0x80, 0xFE, 0xFF, 0x03, 0x00,              // gyro chip counts -32768, -2, 3
      0x01, 0x00, 0x02, 0x01, 0xFF, 0x7F};             // accel chip counts 1, 258, 32767
  EXPECT_EQ(*r, expect);
  const ImuRawValue v = decode_imu_raw(*r, sc);
  EXPECT_EQ(v.flags, s.flags);
  EXPECT_EQ(v.t_lo16, 0x7788U);
}

TEST(LogImuRaw, EveryInt16CountComesBackBitForBitAtTheFirmwaresFullScalesAndEveryMounting) {
  const Lsm6dsvScales fs = hal_target::lsm6dsv_scales();  // gyro +-2000 dps, accel +-16 g (lsm6dsv.cpp:73-74)
  std::size_t checked = 0;
  for (const ImuMounting& m : kMountings) {
    ASSERT_TRUE(hal_target::valid(m));
    const ImuRawScales sc = scales_of(m, fs);
    std::array<std::bitset<kCounts>, 6> seen;  // chip counts seen on each sensor axis
    for (const bool with_temp : {false, true}) {
      Lsm6dsvAssembler a(m, fs);
      if (with_temp) {
        a.set_temperature(-1234);
      }
      for (std::size_t i = 0; i < kCounts; ++i) {
        const ImuSample s = sample_of(a, i, 1000 + i * 133);
        for (std::size_t k = 0; k < 6; ++k) {
          seen[k].set((i + k * kThird) % kCounts);
        }
        const auto rec = encode_imu_raw(s, sc);
        ASSERT_TRUE(rec.has_value()) << "count " << i << " mounting sign " << m.sign[0];
        const ImuRawValue v = decode_imu_raw(*rec, sc);
        ASSERT_EQ(v.flags, s.flags) << i;
        ASSERT_TRUE(same_bits(v.gyro_rad_s, s.gyro_rad_s)) << i;
        ASSERT_TRUE(same_bits(v.accel_m_s2, s.accel_m_s2)) << i;
        ASSERT_EQ(v.t_lo16, static_cast<std::uint16_t>(s.t_us));
        // So the 32-byte record the sample would have been written as is recoverable byte for byte.
        ImuSample back{};
        back.t_us = s.t_us;
        back.flags = v.flags;
        back.gyro_rad_s = v.gyro_rad_s;
        back.accel_m_s2 = v.accel_m_s2;
        ASSERT_EQ(encode_imu(back), encode_imu(s)) << i;
        ++checked;
      }
    }
    for (std::size_t k = 0; k < 6; ++k) {
      EXPECT_EQ(seen[k].count(), kCounts) << "every int16 count passes sensor axis " << k;
    }
  }
  EXPECT_EQ(checked, kMountings.size() * 2 * kCounts);
}

TEST(LogImuRaw, ASensorThatIsNotValidReadsPlusZeroAndTheFlagsAreKept) {
  const Lsm6dsvScales fs = hal_target::lsm6dsv_scales();
  const ImuRawScales sc = scales_of(kMountings[0], fs);
  ImuSample s{};
  s.flags = imu_flag(ImuFlag::AccelValid);
  s.accel_m_s2 = {fs.accel_m_s2_per_lsb * -1.0F, 0.0F, fs.accel_m_s2_per_lsb * 5.0F};
  const auto r = encode_imu_raw(s, sc);
  ASSERT_TRUE(r.has_value());
  const ImuRawValue v = decode_imu_raw(*r, sc);
  EXPECT_EQ(v.flags, s.flags);
  EXPECT_TRUE(same_bits(v.gyro_rad_s, prim::Vec3<float>{}));
  EXPECT_TRUE(same_bits(v.accel_m_s2, s.accel_m_s2));
}

TEST(LogImuRaw, ASampleThatIsNotWholeCountsIsRefusedAndTheRingWritesTheImuRecord) {
  const Lsm6dsvScales fs = hal_target::lsm6dsv_scales();
  const ImuRawScales sc = scales_of(kMountings[0], fs);
  ImuSample s{};
  s.t_us = 77;
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  s.gyro_rad_s = {fs.gyro_rad_s_per_lsb * 10.5F, 0.0F, 0.0F};  // half a count
  EXPECT_FALSE(encode_imu_raw(s, sc).has_value());
  ImuSample big = s;
  big.gyro_rad_s = {fs.gyro_rad_s_per_lsb * 40000.0F, 0.0F, 0.0F};  // beyond an int16
  EXPECT_FALSE(encode_imu_raw(big, sc).has_value());
  ImuSample flags = s;
  flags.gyro_rad_s = {};
  flags.flags |= imu_flag(ImuFlag::GyroSatX);  // saturation claimed with a count that is not at a limit
  EXPECT_FALSE(encode_imu_raw(flags, sc).has_value());
  ImuSample invalid = s;
  invalid.gyro_rad_s = {};
  invalid.flags = imu_flag(ImuFlag::AccelValid);
  invalid.gyro_rad_s = {1.0F, 0.0F, 0.0F};  // a field of an invalid sensor is exactly 0 (imu_sample.hpp)
  EXPECT_FALSE(encode_imu_raw(invalid, sc).has_value());
  ImuRawScales bad = sc;
  bad.sign[1] = 0;
  EXPECT_FALSE(encode_imu_raw(ImuSample{}, bad).has_value());
  ImuRawScales zero = sc;
  zero.accel_m_s2_per_lsb = 0.0F;
  EXPECT_FALSE(encode_imu_raw(ImuSample{}, zero).has_value());

  std::vector<Cell> storage(64);
  Ring ring(storage);
  EXPECT_TRUE(ring.push_imu_raw(s, sc));
  Lsm6dsvAssembler assembler(kMountings[0], fs);
  const ImuSample ok = sample_of(assembler, 5, 99);
  EXPECT_TRUE(ring.push_imu_raw(ok, sc));
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  ASSERT_EQ(ring.pop(out), imu::kBytes);
  EXPECT_EQ(out[0], static_cast<std::uint8_t>(Kind::Imu));
  ASSERT_EQ(ring.pop(out), imu_raw::kBytes);
  EXPECT_EQ(out[0], static_cast<std::uint8_t>(Kind::ImuRaw));
  const RingStats st = ring.stats();
  EXPECT_EQ(st.pushed[0], 2U) << "both kinds are the Imu stream";
  EXPECT_EQ(st.pushed[1], 0U);
}

TEST(LogImuRaw, ADropOfAnImuRawRecordIsCountedAsAnImuDropAndAnnouncedByAGap) {
  const Lsm6dsvScales fs = hal_target::lsm6dsv_scales();
  const ImuRawScales sc = scales_of(kMountings[0], fs);
  Lsm6dsvAssembler a(kMountings[0], fs);
  std::vector<Cell> storage(4);  // three usable cells
  Ring ring(storage);
  int pushed = 0;
  int dropped = 0;
  for (std::uint64_t t = 1; t <= 5; ++t) {
    (ring.push_imu_raw(sample_of(a, t, t), sc) ? pushed : dropped)++;
  }
  EXPECT_EQ(pushed, 3) << "a raw record is one cell";
  EXPECT_EQ(dropped, 2);
  EXPECT_EQ(ring.stats().dropped[0], 2U);
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  while (ring.pop(out) != 0) {
  }
  EXPECT_TRUE(ring.push_imu_raw(sample_of(a, 6, 6), sc));
  ASSERT_EQ(ring.pop(out), gap::kBytes);
  EXPECT_EQ(out[0], static_cast<std::uint8_t>(Kind::Gap));
  EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(out), gap::kOffDropped), 2U);
}

// ---- the file for the Python decoder -------------------------------------------------------------------------------------
class MemSink final : public ChunkSink {
 public:
  Bytes data;
  bool write_chunk(std::span<const std::uint8_t> chunk) noexcept override {
    data.insert(data.end(), chunk.begin(), chunk.end());
    return true;
  }
};

TEST(LogImuRawRoundTrip, TheWriterAndThePythonDecoderAgree) {
  const char* dir = std::getenv("MARV_LOG_IMU_RAW_DIR");
  const Lsm6dsvScales fs = hal_target::lsm6dsv_scales();
  const ImuRawScales header_scales = scales_of(kMountings[0], fs);
  Bytes staging(64 * kSectorBytes);
  SectorFramer framer(staging);
  MemSink sink;
  constexpr std::uint64_t kSession = 0x0F1E2D3C4B5A6978ULL;
  constexpr std::uint64_t kStart = (std::uint64_t{1} << 32) - 5'000'000;  // the run crosses the 32-bit wrap
  framer.begin(kSession);
  HeaderInfo h;
  h.session_start_us = kStart;
  h.tick_num_us = 1953125;
  h.tick_den = 14649;
  h.odr_hz = 7500.3F;
  h.imu_raw = &header_scales;
  ASSERT_TRUE(write_header(h, framer, sink));

  std::vector<Cell> storage(256);
  Ring ring(storage);
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  const auto drain = [&] {
    for (std::size_t n = ring.pop(out); n != 0; n = ring.pop(out)) {
      ASSERT_TRUE(framer.append(std::span<const std::uint8_t>(out).first(n), sink));
    }
  };
  // The mounting is one per file (the header carries one set of signs): the board's map, every count of both sensors.
  const ImuMounting& m = kMountings[0];
  Lsm6dsvAssembler a(m, fs);
  std::uint64_t t = kStart;
  for (std::size_t i = 0; i < kCounts; ++i) {
    t += 133;
    const ImuSample s1 = sample_of(a, i, t);
    ImuSample s2 = s1;
    s2.t_us = t + 1;
    ASSERT_TRUE(ring.push_imu(s1));
    ASSERT_TRUE(ring.push_imu_raw(s2, header_scales));
    RateTickRecord rt;
    rt.now_us = t + 1;
    ASSERT_TRUE(ring.push_rate_tick(rt));
    drain();
  }
  std::array<std::uint8_t, end::kBytes> end{};
  end[kLeadOffKind] = static_cast<std::uint8_t>(Kind::End);
  put<std::uint16_t>(end, kLeadOffLength, static_cast<std::uint16_t>(end.size()));
  put<std::uint64_t>(end, end::kOffTime, t);
  ASSERT_TRUE(framer.append(end, sink));
  ASSERT_TRUE(framer.flush(sink));
  EXPECT_EQ(get<std::uint16_t>(std::span<const std::uint8_t>(sink.data), kSecOffVersion), 2U);
  if (dir != nullptr) {
    std::ofstream(std::string(dir) + "/imu_raw.mlog", std::ios::binary)
        .write(reinterpret_cast<const char*>(sink.data.data()), static_cast<std::streamsize>(sink.data.size()));
  }
}

}  // namespace
