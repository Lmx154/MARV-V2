// The onboard logger's formats, ring and ring budget (fw/log, unit U1 LOG-CORE; invariants L-1 and L-5 of the architect ruling
// 2026-10-18).
//
//  LogFormat   the layouts (sizes, offsets, the CRC-32 check value, the known bytes of one record of each kind).
//  LogRing     push and pop of every kind, wrap-around, full ring (the newest whole record is dropped and counted per kind),
//              the Gap record at the place of the loss, the high-water mark, discard_oldest; L-1: the writer never waits
//              (consumer absent, consumer blocked until the writer is done, concurrent stress with stalls).
//  LogBudget   L-5: the bench ring covers rate x the worst stall (the design-budget entry sd_log_write_stall_max_s), the
//              derived pre-roll, and the negative controls (a ring too small, a stall too long) that the check must refuse.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <random>
#include <thread>
#include <vector>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "log_test_util.hpp"
#include "marv/log/params.hpp"

namespace {

using namespace logtest;

constexpr std::size_t kBenchRingBytes = std::size_t{1} << 18;  // 256 KiB, the bench log image's ring (architect ruling item 3)

std::vector<Cell> make_ring_storage(std::size_t cells) { return std::vector<Cell>(cells); }

TEST(LogFormat, LayoutsAreDerivedAndAddUp) {
  EXPECT_EQ(kSectorBytes, 512U);
  EXPECT_EQ(kSectorHeaderBytes, 20U);
  EXPECT_EQ(kSectorPayloadBytes, 488U);
  EXPECT_EQ(kSectorHeaderBytes + kSectorPayloadBytes + kSecCrcBytes, kSectorBytes);
  EXPECT_EQ(kCellBytes, 16U);
  EXPECT_EQ(kSlotBytes, 32U);
  EXPECT_EQ(imu::kBytes, 32U) << "the IMU record fills one slot";
  EXPECT_EQ(sizeof(std::array<std::uint8_t, kCellBytes>), kCellBytes);
  EXPECT_EQ(rate::kOffDshot + rate::kMotors * sizeof(std::uint16_t), kCellBytes) << "the rate-tick record is one cell";
  EXPECT_EQ(manual::kBytes, 112U);
  EXPECT_LE(manual::kUsedBytes, manual::kBytes);
  EXPECT_EQ(gap::kBytes, 32U);
  EXPECT_EQ(kMaxRingRecordBytes, manual::kBytes);
  for (std::size_t k = 0; k < kKinds; ++k) {
    const std::size_t n = ring_record_bytes(static_cast<Kind>(k));
    EXPECT_EQ(n % kCellBytes, 0U) << "kind " << k;
  }
}

TEST(LogFormat, Crc32CheckValue) {
  const std::array<std::uint8_t, 9> text{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc32(text), 0xCBF43926U);
}

TEST(LogFormat, ImuRecordKnownBytes) {
  ImuSample s{};
  s.t_us = 0x1122334455667788ULL;
  s.gyro_rad_s = {1.0F, 2.0F, -2.0F};
  s.accel_m_s2 = {0.5F, 0.0F, -1.0F};
  s.flags = 0x1C3;
  const auto r = encode_imu(s);
  const std::array<std::uint8_t, 32> expect{
      0x01, 0x00, 0xC3, 0x01,                          // kind, reserved, flags
      0x88, 0x77, 0x66, 0x55,                          // t_lo
      0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x40,  // gyro x = 1.0, y = 2.0
      0x00, 0x00, 0x00, 0xC0,                          // gyro z = -2.0
      0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00,  // accel x = 0.5, y = 0
      0x00, 0x00, 0x80, 0xBF};                         // accel z = -1.0
  EXPECT_EQ(r, expect);
}

TEST(LogFormat, RateTickAndGapKnownBytes) {
  RateTickRecord t;
  t.now_us = 0xAABBCCDD11ULL;
  t.arm_state = 3;
  t.gate_state = 5;
  t.dshot = {*DshotValue::from_raw(48), DshotValue::stop(), *DshotValue::from_raw(2047), *DshotValue::from_raw(1000)};
  const auto r = encode_rate_tick(t);
  const std::array<std::uint8_t, 16> expect{0x02, 0x03, 0x05, 0x00, 0xDD, 0xCC, 0xBB, 0xAA,
                                            0x30, 0x00, 0x00, 0x00, 0xFF, 0x07, 0xE8, 0x03};
  EXPECT_EQ(r, expect);
  const auto g = encode_gap(0x01020304U, {5, 6, 7});
  EXPECT_EQ(g[0], static_cast<std::uint8_t>(Kind::Gap));
  EXPECT_EQ(get<std::uint32_t>(g, gap::kOffTime), 0x01020304U);
  EXPECT_EQ(get<std::uint32_t>(g, gap::kOffDropped), 5U);
  EXPECT_EQ(get<std::uint32_t>(g, gap::kOffDropped + 8), 7U);
}

TEST(LogRing, PushPopEveryKindInOrder) {
  auto store = make_ring_storage(64);
  Ring ring(store);
  ASSERT_TRUE(ring.push_imu(make_imu(1)));
  ASSERT_TRUE(ring.push_rate_tick(make_rate(2)));
  ASSERT_TRUE(ring.push_manual(make_manual(3, 9)));
  ASSERT_TRUE(ring.push_imu(make_imu(4)));
  EXPECT_EQ(ring.backlog_cells(), (imu::kBytes + kCellBytes + manual::kBytes + imu::kBytes) / kCellBytes);
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  std::size_t n = ring.pop(out);
  ASSERT_EQ(n, imu::kBytes);
  EXPECT_TRUE(std::equal(out.begin(), out.begin() + n, encode_imu(make_imu(1)).begin()));
  n = ring.pop(out);
  ASSERT_EQ(n, kCellBytes);
  EXPECT_TRUE(std::equal(out.begin(), out.begin() + n, encode_rate_tick(make_rate(2)).begin()));
  n = ring.pop(out);
  ASSERT_EQ(n, manual::kBytes);
  EXPECT_TRUE(std::equal(out.begin(), out.begin() + n, encode_manual(make_manual(3, 9)).begin()));
  n = ring.pop(out);
  ASSERT_EQ(n, imu::kBytes);
  EXPECT_EQ(ring.pop(out), 0U);
  EXPECT_EQ(ring.backlog_cells(), 0U);
  const RingStats s = ring.stats();
  EXPECT_EQ(s.pushed[ring_kind_index(Kind::Imu)], 2U);
  EXPECT_EQ(s.pushed[ring_kind_index(Kind::RateTick)], 1U);
  EXPECT_EQ(s.pushed[ring_kind_index(Kind::ManualControl)], 1U);
  EXPECT_EQ(s.dropped[0] + s.dropped[1] + s.dropped[2], 0U);
}

TEST(LogRing, PopWithTooSmallBufferLeavesTheRecord) {
  auto store = make_ring_storage(16);
  Ring ring(store);
  ASSERT_TRUE(ring.push_imu(make_imu(1)));
  std::array<std::uint8_t, imu::kBytes - 1> small{};
  EXPECT_EQ(ring.pop(small), 0U);
  EXPECT_EQ(ring.backlog_cells(), imu::kBytes / kCellBytes);
}

TEST(LogRing, WrapsAroundTheEndWithMixedRecordSizes) {
  auto store = make_ring_storage(13);  // odd, so 1-, 2- and 7-cell records straddle the end
  Ring ring(store);
  std::uint64_t t = 0;
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  for (int round = 0; round < 400; ++round) {
    const int which = round % 3;
    Bytes want;
    if (which == 0) {
      const auto r = encode_imu(make_imu(++t));
      want.assign(r.begin(), r.end());
      ASSERT_TRUE(ring.push_imu(make_imu(t)));
    } else if (which == 1) {
      const auto r = encode_rate_tick(make_rate(++t));
      want.assign(r.begin(), r.end());
      ASSERT_TRUE(ring.push_rate_tick(make_rate(t)));
    } else {
      const auto r = encode_manual(make_manual(++t, static_cast<std::uint32_t>(t)));
      want.assign(r.begin(), r.end());
      ASSERT_TRUE(ring.push_manual(make_manual(t, static_cast<std::uint32_t>(t))));
    }
    const std::size_t n = ring.pop(out);
    ASSERT_EQ(n, want.size()) << "round " << round;
    EXPECT_TRUE(std::equal(want.begin(), want.end(), out.begin())) << "round " << round;
  }
  EXPECT_EQ(ring.stats().dropped[0] + ring.stats().dropped[1] + ring.stats().dropped[2], 0U);
}

TEST(LogRing, FullRingDropsTheNewestWholeRecordAndCountsPerKind) {
  auto store = make_ring_storage(8);  // 7 usable cells
  Ring ring(store);
  ASSERT_TRUE(ring.push_imu(make_imu(1)));   // 2 cells
  ASSERT_TRUE(ring.push_imu(make_imu(2)));   // 4
  ASSERT_TRUE(ring.push_imu(make_imu(3)));   // 6
  EXPECT_FALSE(ring.push_imu(make_imu(4)));  // needs 2, 1 left: dropped
  EXPECT_FALSE(ring.push_manual(make_manual(5, 1)));
  ASSERT_TRUE(ring.push_rate_tick(make_rate(6)));  // 7: the one cell that was left
  EXPECT_FALSE(ring.push_rate_tick(make_rate(7)));
  const RingStats s = ring.stats();
  EXPECT_EQ(s.pushed[ring_kind_index(Kind::Imu)], 3U);
  EXPECT_EQ(s.dropped[ring_kind_index(Kind::Imu)], 1U);
  EXPECT_EQ(s.dropped[ring_kind_index(Kind::ManualControl)], 1U);
  EXPECT_EQ(s.dropped[ring_kind_index(Kind::RateTick)], 1U);
  EXPECT_EQ(s.high_water_cells, 7U);
  // What was kept is whole and in order: nothing of a dropped record is in the ring.
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  for (std::uint64_t t = 1; t <= 3; ++t) {
    ASSERT_EQ(ring.pop(out), imu::kBytes);
    EXPECT_EQ(get<std::uint32_t>(out, imu::kOffTime), t);
  }
  ASSERT_EQ(ring.pop(out), kCellBytes);
  EXPECT_EQ(ring.pop(out), 0U);
}

TEST(LogRing, AGapRecordMarksTheLossAtItsPlaceInTheStream) {
  auto store = make_ring_storage(8);
  Ring ring(store);
  ASSERT_TRUE(ring.push_imu(make_imu(1)));
  ASSERT_TRUE(ring.push_imu(make_imu(2)));
  ASSERT_TRUE(ring.push_imu(make_imu(3)));
  EXPECT_FALSE(ring.push_imu(make_imu(4)));
  EXPECT_FALSE(ring.push_imu(make_imu(5)));
  EXPECT_FALSE(ring.push_manual(make_manual(6, 1)));
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  ASSERT_EQ(ring.pop(out), imu::kBytes);  // frees two cells; the gap needs two more than the record
  EXPECT_FALSE(ring.push_imu(make_imu(7))) << "room for the record but not for the gap record the drops owe";
  ASSERT_EQ(ring.pop(out), imu::kBytes);
  ASSERT_TRUE(ring.push_imu(make_imu(8)));
  // The stream now reads: imu 3, gap{imu 3, rate 0, manual 1 + the dropped 7}, imu 8.
  ASSERT_EQ(ring.pop(out), imu::kBytes);
  EXPECT_EQ(get<std::uint32_t>(out, imu::kOffTime), 3U);
  ASSERT_EQ(ring.pop(out), gap::kBytes);
  EXPECT_EQ(out[0], static_cast<std::uint8_t>(Kind::Gap));
  EXPECT_EQ(get<std::uint32_t>(out, gap::kOffTime), 8U) << "the time of the record that follows";
  EXPECT_EQ(get<std::uint32_t>(out, gap::kOffDropped), 3U);
  EXPECT_EQ(get<std::uint32_t>(out, gap::kOffDropped + 4), 0U);
  EXPECT_EQ(get<std::uint32_t>(out, gap::kOffDropped + 8), 1U);
  ASSERT_EQ(ring.pop(out), imu::kBytes);
  EXPECT_EQ(get<std::uint32_t>(out, imu::kOffTime), 8U);
  EXPECT_EQ(ring.stats().gaps, 1U);
  // The counts restart: the next gap reports only the new loss.
  ASSERT_TRUE(ring.push_imu(make_imu(9)));
  ASSERT_TRUE(ring.push_imu(make_imu(10)));
  ASSERT_TRUE(ring.push_imu(make_imu(11)));
  EXPECT_FALSE(ring.push_imu(make_imu(12)));
  EXPECT_EQ(ring.stats().dropped[ring_kind_index(Kind::Imu)], 4U) << "the running total keeps counting";
}

TEST(LogRing, DiscardOldestKeepsWholeRecords) {
  auto store = make_ring_storage(64);
  Ring ring(store);
  for (std::uint64_t t = 1; t <= 5; ++t) {
    ASSERT_TRUE(ring.push_imu(make_imu(t)));
    ASSERT_TRUE(ring.push_rate_tick(make_rate(t)));
  }
  const std::size_t before = ring.backlog_cells();
  const std::size_t removed = ring.discard_oldest(7);
  EXPECT_EQ(before - removed, ring.backlog_cells());
  EXPECT_LE(ring.backlog_cells(), 7U);
  EXPECT_GT(ring.backlog_cells(), 7U - 2U) << "stops at the first record boundary at or below the limit";
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  const std::size_t n = ring.pop(out);
  ASSERT_NE(n, 0U);
  EXPECT_TRUE(out[0] == static_cast<std::uint8_t>(Kind::Imu) || out[0] == static_cast<std::uint8_t>(Kind::RateTick));
  EXPECT_EQ(ring.stats().corrupt, 0U);
}

TEST(LogRing, HighWaterMarkIsTheMostCellsHeld) {
  auto store = make_ring_storage(64);
  Ring ring(store);
  std::array<std::uint8_t, kMaxRingRecordBytes> out{};
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(ring.push_imu(make_imu(static_cast<std::uint64_t>(i))));
  }
  EXPECT_EQ(ring.stats().high_water_cells, 5U * imu::kBytes / kCellBytes);
  while (ring.pop(out) != 0) {
  }
  ASSERT_TRUE(ring.push_imu(make_imu(99)));
  EXPECT_EQ(ring.stats().high_water_cells, 5U * imu::kBytes / kCellBytes) << "a mark, not a level";
}

// ---- L-1 --------------------------------------------------------------------------------------------------------------
TEST(LogRing, L1WriterNeverWaitsWithNoConsumer) {
  auto store = make_ring_storage(256);
  Ring ring(store);
  constexpr int kPushes = 200000;
  auto done = std::async(std::launch::async, [&] {
    int ok = 0;
    for (int i = 0; i < kPushes; ++i) {
      switch (i % 3) {
        case 0:
          ok += ring.push_imu(make_imu(static_cast<std::uint64_t>(i))) ? 1 : 0;
          break;
        case 1:
          ok += ring.push_rate_tick(make_rate(static_cast<std::uint64_t>(i))) ? 1 : 0;
          break;
        default:
          ok += ring.push_manual(make_manual(static_cast<std::uint64_t>(i), static_cast<std::uint32_t>(i))) ? 1 : 0;
      }
    }
    return ok;
  });
  ASSERT_EQ(done.wait_for(std::chrono::seconds(30)), std::future_status::ready) << "the writer waited";
  const RingStats s = ring.stats();
  std::uint64_t pushed = 0;
  std::uint64_t dropped = 0;
  for (std::size_t k = 0; k < kRingKinds; ++k) {
    pushed += s.pushed[k];
    dropped += s.dropped[k];
  }
  EXPECT_EQ(pushed + dropped, static_cast<std::uint64_t>(kPushes)) << "every record is pushed or counted dropped";
  EXPECT_EQ(static_cast<std::uint64_t>(done.get()), pushed);
  EXPECT_GT(dropped, 0U);
  EXPECT_LE(s.high_water_cells, ring.capacity_cells() - 1);
}

TEST(LogRing, L1WriterNeverWaitsForAConsumerStoppedMidRead) {
  // The consumer has looked at the ring (backlog_cells) and then stops until the test releases it, which it does only after
  // the writer has finished many times the ring's capacity. A writer that waited for the consumer would never finish.
  auto store = make_ring_storage(128);
  Ring ring(store);
  std::atomic<bool> release{false};
  std::atomic<bool> consumer_saw{false};
  std::promise<void> consumer_done;
  std::thread consumer([&] {
    while (ring.backlog_cells() == 0) {
      std::this_thread::yield();
    }
    consumer_saw = true;
    while (!release) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::array<std::uint8_t, kMaxRingRecordBytes> out{};
    while (ring.pop(out) != 0) {
    }
    consumer_done.set_value();
  });
  auto writer = std::async(std::launch::async, [&] {
    std::uint64_t t = 0;
    while (!consumer_saw) {
      (void)ring.push_imu(make_imu(++t));
    }
    for (int i = 0; i < 100000; ++i) {
      (void)ring.push_imu(make_imu(++t));
    }
    return t;
  });
  const bool finished = writer.wait_for(std::chrono::seconds(30)) == std::future_status::ready;
  release = true;
  consumer.join();
  ASSERT_TRUE(finished) << "the writer waited for a stopped consumer";
  const std::uint64_t attempts = writer.get();
  const RingStats s = ring.stats();
  EXPECT_EQ(s.pushed[0] + s.dropped[0], attempts);
  EXPECT_GT(s.dropped[0], 0U);
}

TEST(LogRing, L1ConcurrentStressLosesNothingSilently) {
  // A writer pushes IMU records whose payload is a function of the stamp; a consumer pops with random stalls. Afterwards:
  // every record popped is whole (payload matches its stamp), stamps strictly increase, and the stamps missing from the
  // popped stream are exactly the dropped ones, each loss announced by a Gap record whose counts add up.
  auto store = make_ring_storage(96);
  Ring ring(store);
  constexpr std::uint64_t kAttempts = 300000;
  std::atomic<bool> writer_done{false};
  std::vector<std::uint32_t> seen;
  std::uint64_t gap_imu = 0;
  std::thread consumer([&] {
    std::mt19937 rng(1234);
    std::array<std::uint8_t, kMaxRingRecordBytes> out{};
    for (;;) {
      const bool done = writer_done.load();
      const std::size_t n = ring.pop(out);
      if (n == 0) {
        if (done) {
          break;
        }
        continue;
      }
      if (rng() % 64 == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(rng() % 300));
      }
      if (out[0] == static_cast<std::uint8_t>(Kind::Imu)) {
        const auto t = get<std::uint32_t>(std::span<const std::uint8_t>(out), imu::kOffTime);
        EXPECT_EQ(get_f32(out, imu::kOffGyro), static_cast<float>(t) * 0.25F) << "torn record";
        EXPECT_EQ(get_f32(out, imu::kOffGyro + 8), -static_cast<float>(t)) << "torn record";
        seen.push_back(t);
      } else if (out[0] == static_cast<std::uint8_t>(Kind::Gap)) {
        gap_imu += get<std::uint32_t>(std::span<const std::uint8_t>(out), gap::kOffDropped);
      }
    }
  });
  std::thread writer([&] {
    for (std::uint64_t t = 1; t <= kAttempts; ++t) {
      (void)ring.push_imu(make_imu(t));
    }
    writer_done = true;
  });
  writer.join();
  consumer.join();
  ASSERT_FALSE(seen.empty());
  for (std::size_t i = 1; i < seen.size(); ++i) {
    ASSERT_LT(seen[i - 1], seen[i]) << "order or duplicate at " << i;
  }
  const RingStats s = ring.stats();
  EXPECT_EQ(s.pushed[0] + s.dropped[0], kAttempts);
  EXPECT_EQ(seen.size(), s.pushed[0]);
  EXPECT_EQ(kAttempts - seen.size(), s.dropped[0]);
  // Drops after the last published record have no gap record yet (it is owed to the next push); the rest are all announced.
  EXPECT_LE(gap_imu, s.dropped[0]);
  const std::uint64_t unannounced = s.dropped[0] - gap_imu;
  EXPECT_EQ(unannounced, kAttempts - seen.back()) << "only the tail after the last record is unannounced";
}

// ---- L-5 ----------------------------------------------------------------------------------------------------------------
struct BudgetFixture : ::testing::Test {
  ParamTable table = param_defaults();
  FastRates rates = fast_rates_from_params(table);
  double stall = write_stall_max_s(table);
  std::size_t bench_cells = kBenchRingBytes / kCellBytes;
};

TEST_F(BudgetFixture, TheBudgetEntryIsTheMeasuredWorstStall) {
  EXPECT_DOUBLE_EQ(stall, 0.809) << "tests/regression/quad/L10/results/sd/README.md: hal_config_pending_write max 809.1 ms";
  const ParamRecord r = table[static_cast<std::size_t>(ParamId::sd_log_write_stall_max_s)];
  EXPECT_EQ(r.method, ParamMethod::DesignBudget);
}

TEST_F(BudgetFixture, TheFastStreamRatesFollowTheTickAndTheDivisor) {
  EXPECT_NEAR(rates.imu_hz, 7500.3, 0.1);
  EXPECT_NEAR(rates.rate_tick_hz, rates.imu_hz / 2.0, 1e-9);
  EXPECT_NEAR(rates.manual_hz, 125.0, 1e-9);
  const double bytes = fast_bytes_per_s(rates);
  EXPECT_NEAR(bytes, 7500.3 * 32 + 3750.15 * 16 + 125.0 * 112, 10.0);
}

TEST_F(BudgetFixture, L5TheBenchRingCoversTheFastStreamOverTheWorstStall) {
  const double need = fast_bytes_per_s(rates) * stall;
  const double have = ring_usable_bytes(bench_cells);
  RecordProperty("need_bytes", std::to_string(need));
  RecordProperty("have_bytes", std::to_string(have));
  EXPECT_TRUE(ring_covers_stall(bench_cells, rates, stall)) << "need " << need << " B, the ring holds " << have << " B";
  EXPECT_GE(have, need);
}

TEST_F(BudgetFixture, L5NegativeControlsARingTooSmallOrAStallTooLongIsRefused) {
  EXPECT_FALSE(ring_covers_stall(bench_cells / 2, rates, stall)) << "half the ring must not pass";
  EXPECT_FALSE(ring_covers_stall(bench_cells, rates, stall * 1.1)) << "a stall 10 % longer than measured must not pass";
  EXPECT_FALSE(ring_covers_stall(0, rates, stall));
  // The boundary is sharp: one cell short of the exact requirement fails, the requirement itself passes.
  const auto exact = static_cast<std::size_t>(std::ceil(fast_bytes_per_s(rates) * stall / kCellBytes)) + 1;
  EXPECT_TRUE(ring_covers_stall(exact, rates, stall));
  EXPECT_FALSE(ring_covers_stall(exact - 1, rates, stall));
  // The flight image's borrowed scratch (98304 B, ruling item 5) does NOT cover the measured stall: EMB-6 is open there.
  EXPECT_FALSE(ring_covers_stall(98304 / kCellBytes, rates, stall));
}

TEST_F(BudgetFixture, PreRollIsWhatTheRingHoldsBeyondTheStall) {
  const std::size_t pre = pre_roll_cells(bench_cells, rates, stall);
  const double spare = ring_usable_bytes(bench_cells) - fast_bytes_per_s(rates) * stall;
  ASSERT_GT(spare, 0.0);
  EXPECT_EQ(pre, static_cast<std::size_t>(spare / kCellBytes));
  const double pre_s = static_cast<double>(pre * kCellBytes) / fast_bytes_per_s(rates);
  RecordProperty("pre_roll_s", std::to_string(pre_s));
  EXPECT_GT(pre_s, 0.0);
  EXPECT_LT(pre_s, 1.0);
  EXPECT_EQ(pre_roll_cells(bench_cells / 2, rates, stall), 0U) << "a ring that does not cover the stall keeps no history";
  EXPECT_GT(pre_roll_cells(bench_cells, rates, stall / 2), pre);
}

TEST_F(BudgetFixture, TheConfigFromParametersCarriesTheDerivedPreRoll) {
  const LoggerConfig c = logger_config_from_params(table, bench_cells, nullptr, nullptr);
  EXPECT_EQ(c.pre_roll_cells, pre_roll_cells(bench_cells, rates, stall));
  EXPECT_EQ(c.post_roll_us, 5000000U);
  EXPECT_EQ(c.file_bytes, 67108864U);
  EXPECT_EQ(c.slow_period_us, 4000U);
}

}  // namespace
