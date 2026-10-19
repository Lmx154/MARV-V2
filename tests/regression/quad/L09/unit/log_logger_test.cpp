// The onboard logger's framer, header and state machine (fw/log, unit U1 LOG-CORE; invariants L-1, L-3, L-4 and L-9 of the
// architect ruling 2026-10-18), against a scriptable in-memory card.
//
//  LogFramer      sectors are self-validating; a record spans sectors; flush pads; a refused chunk fails the framer; discard.
//  LogHeader      the header carries the identity, the boot report, the tick, the record kinds and fields, the telemetry
//                 hook and the whole parameter table, and ends on a sector boundary.
//  LogLogger      the states and their transitions: probe, pre-roll, arm, post-roll, re-arm, close; card absent, card full,
//                 file full, write failure, header failure, close failure; the Status and Telemetry records.
//  LogFaults      L-3: randomised fault injection (card absent, full, failing; arm flicker) ends in a state and counters,
//                 never a panic (hal_panic would abort the process; the static check is tools/ci/check_log_no_panic.py).
//  LogNoMetadata  L-9: a fake card that fails the test on any prepare or close while armed or Logging, over the same
//                 randomised runs, and a control that the fake does fail.
//  LogRoundTrip   a synthetic tick stream through the ring, the logger and the card, read back: equal to the input except the
//                 records counted dropped, each loss announced by a Gap record. With MARV_LOG_ROUNDTRIP_DIR set the file and
//                 the expected stream are written there for tools/log/roundtrip_check.py (the Python decoder's check).
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <random>
#include <string>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "log_test_util.hpp"
#include "marv/log/framer.hpp"
#include "marv/log/params.hpp"

namespace {

using namespace logtest;

// ---- a sink that keeps what it is given -------------------------------------------------------------------------------
class MemSink final : public ChunkSink {
 public:
  Bytes data;
  int chunks = 0;
  bool refuse = false;
  bool write_chunk(std::span<const std::uint8_t> chunk) noexcept override {
    if (refuse) {
      return false;
    }
    ++chunks;
    data.insert(data.end(), chunk.begin(), chunk.end());
    return true;
  }
};

constexpr std::uint64_t kSession = 0x0123456789ABCDEFULL;

std::uint64_t next_session(void* ctx) noexcept {
  auto* n = static_cast<std::uint64_t*>(ctx);
  return kSession + (*n)++;
}

const char* name_of(std::size_t id) noexcept { return param_name(static_cast<ParamId>(id)); }

Bytes record_of_size(std::size_t n, std::uint8_t fill) {
  // A header-style record (kind HeaderParam, lead length n) of n bytes, filled with `fill`.
  Bytes r(n, fill);
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::HeaderParam);
  r[kLeadOffSub] = 0;
  put<std::uint16_t>(r, kLeadOffLength, static_cast<std::uint16_t>(n));
  return r;
}

TEST(LogFramer, SectorsAreSelfValidating) {
  Bytes staging(2 * kSectorBytes);
  SectorFramer f(staging);
  MemSink sink;
  f.begin(kSession);
  ASSERT_TRUE(f.append(record_of_size(100, 0x11), sink));
  ASSERT_TRUE(f.flush(sink));
  ASSERT_EQ(sink.data.size(), kSectorBytes);
  const std::span<const std::uint8_t> s(sink.data);
  EXPECT_TRUE(std::equal(kSectorMagic.begin(), kSectorMagic.end(), s.begin()));
  EXPECT_EQ(get<std::uint64_t>(s, kSecOffSession), kSession);
  EXPECT_EQ(get<std::uint32_t>(s, kSecOffSeq), 0U);
  EXPECT_EQ(get<std::uint16_t>(s, kSecOffFirst), 0U);
  EXPECT_EQ(get<std::uint16_t>(s, kSecOffVersion), kFormatVersion);
  EXPECT_EQ(get<std::uint32_t>(s, kSecOffCrc), crc32(s.first(kSecOffCrc)));
  EXPECT_EQ(s[kSectorHeaderBytes + 100], 0) << "the rest of the payload is padding";
  // Flipping any one bit anywhere in the sector breaks the CRC.
  for (std::size_t byte = 0; byte < kSectorBytes; byte += 37) {
    Bytes bad = sink.data;
    bad[byte] ^= 0x10;
    EXPECT_NE(get<std::uint32_t>(std::span<const std::uint8_t>(bad), kSecOffCrc),
              crc32(std::span<const std::uint8_t>(bad).first(kSecOffCrc)))
        << "byte " << byte;
  }
}

TEST(LogFramer, ARecordSpansSectorsAndTheSectorNamesWhereTheNextOneBegins) {
  Bytes staging(4 * kSectorBytes);
  SectorFramer f(staging);
  MemSink sink;
  f.begin(kSession);
  ASSERT_TRUE(f.append(record_of_size(300, 1), sink));  // sector 0: 0..300
  ASSERT_TRUE(f.append(record_of_size(300, 2), sink));  // begins at 300, 188 bytes in sector 0, 112 in sector 1
  ASSERT_TRUE(f.append(record_of_size(1000, 3), sink)); // begins in sector 1 at 112, runs through sector 2 into 3
  ASSERT_TRUE(f.flush(sink));
  const std::span<const std::uint8_t> file(sink.data);
  ASSERT_EQ(file.size(), 4 * kSectorBytes);
  EXPECT_EQ(get<std::uint16_t>(file, 0 * kSectorBytes + kSecOffFirst), 0U);
  EXPECT_EQ(get<std::uint16_t>(file, 1 * kSectorBytes + kSecOffFirst), 112U);
  EXPECT_EQ(get<std::uint16_t>(file, 2 * kSectorBytes + kSecOffFirst), kNoRecord) << "only the middle of a record";
  const Parsed p = parse(file);
  EXPECT_TRUE(p.problems.empty());
  ASSERT_EQ(p.records.size(), 3U);
  EXPECT_EQ(p.records[0].bytes.size(), 300U);
  EXPECT_EQ(p.records[1].bytes.size(), 300U);
  EXPECT_EQ(p.records[2].bytes.size(), 1000U);
  EXPECT_EQ(p.records[2].bytes.back(), 3);
  EXPECT_EQ(p.records[1].bytes[200], 2);
}

TEST(LogFramer, AChunkIsHandedOverWhenTheStagingFillsAndTheSequenceRuns) {
  Bytes staging(2 * kSectorBytes);
  SectorFramer f(staging);
  MemSink sink;
  f.begin(kSession);
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(f.append(record_of_size(kSectorPayloadBytes, static_cast<std::uint8_t>(i + 1)), sink));
  }
  EXPECT_EQ(sink.chunks, 5);
  EXPECT_EQ(sink.data.size(), 10 * kSectorBytes);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(sink.data), i * kSectorBytes + kSecOffSeq), i);
  }
  EXPECT_EQ(f.next_sequence(), 10U);
  ASSERT_TRUE(f.flush(sink)) << "nothing staged: nothing handed over";
  EXPECT_EQ(sink.chunks, 5);
}

TEST(LogFramer, ARefusedChunkFailsTheFramerUntilBegin) {
  Bytes staging(kSectorBytes);
  SectorFramer f(staging);
  MemSink sink;
  sink.refuse = true;
  f.begin(kSession);
  EXPECT_FALSE(f.append(record_of_size(kSectorPayloadBytes, 1), sink));
  EXPECT_TRUE(f.failed());
  sink.refuse = false;
  EXPECT_FALSE(f.append(record_of_size(16, 1), sink));
  EXPECT_FALSE(f.flush(sink));
  EXPECT_TRUE(sink.data.empty());
  f.begin(kSession + 1);
  EXPECT_FALSE(f.failed());
  EXPECT_TRUE(f.append(record_of_size(16, 1), sink));
  EXPECT_TRUE(f.flush(sink));
  EXPECT_EQ(sink.data.size(), kSectorBytes);
  EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(sink.data), kSecOffSeq), 0U);
}

TEST(LogFramer, DiscardStagedTakesTheSequenceBackSoTheFileHasNoHole) {
  Bytes staging(4 * kSectorBytes);
  SectorFramer f(staging);
  MemSink sink;
  f.begin(kSession);
  ASSERT_TRUE(f.append(record_of_size(kSectorPayloadBytes, 1), sink));
  ASSERT_TRUE(f.append(record_of_size(kSectorPayloadBytes, 2), sink));
  EXPECT_EQ(f.next_sequence(), 2U);
  EXPECT_EQ(f.staged_sectors(), 2U);
  f.discard_staged();
  EXPECT_EQ(f.next_sequence(), 0U);
  ASSERT_TRUE(f.append(record_of_size(40, 9), sink));
  ASSERT_TRUE(f.flush(sink));
  ASSERT_EQ(sink.data.size(), kSectorBytes);
  EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(sink.data), kSecOffSeq), 0U);
}

// ---- the rig ------------------------------------------------------------------------------------------------------------
constexpr TimeUs kPostRoll = 1'000'000;
constexpr TimeUs kReprobe = 500'000;
constexpr TimeUs kStatusPeriod = 100'000;
constexpr std::uint64_t kTickUs = 133;

struct Rig {
  explicit Rig(std::size_t ring_cells = 512, std::size_t staging_sectors = 4, std::size_t pre_roll = 32,
               std::uint64_t file_bytes = 1 << 20, TimeUs slow_period = 0, const TelemetryHook* hook = nullptr)
      : cells(ring_cells),
        ring(cells),
        staging(staging_sectors * kSectorBytes),
        logger(ring, fake, staging, make_config(pre_roll, file_bytes, slow_period), make_header(hook)) {
    fake.capacity = file_bytes;
    fake.logger = &logger;
    fake.armed = &armed;
  }

  LoggerConfig make_config(std::size_t pre_roll, std::uint64_t file_bytes, TimeUs slow_period) {
    LoggerConfig c;
    c.file_bytes = file_bytes;
    c.post_roll_us = kPostRoll;
    c.status_period_us = kStatusPeriod;
    c.slow_period_us = slow_period;
    c.reprobe_period_us = kReprobe;
    c.pre_roll_cells = pre_roll;
    c.session_id = next_session;
    c.session_ctx = &sessions;
    return c;
  }

  HeaderInfo make_header(const TelemetryHook* hook) {
    HeaderInfo h;
    for (std::size_t i = 0; i < h.image_hash.size(); ++i) {
      h.image_hash[i] = static_cast<std::uint8_t>(i + 1);
      h.config_hash[i] = static_cast<std::uint8_t>(0xF0 - i);
    }
    h.schema_hash = 0xDEADBEEF12345678ULL;
    h.boot = BootFields{1, 2, 3, 4, 5, 6, 77};
    h.tick_num_us = 1953125;
    h.tick_den = 14649;
    h.odr_hz = 7500.3F;
    h.params = param_defaults();
    h.param_name = name_of;
    h.telemetry = hook;
    return h;
  }

  // One tick of the vehicle: the sample, every other tick a rate tick, an RC frame every 60th.
  void tick() {
    ++n;
    const TimeUs t = n * kTickUs;
    if (ring.push_imu(make_imu(t))) {
      accepted_imu.push_back(t);
    }
    if (n % 2 == 0 && ring.push_rate_tick(make_rate(t))) {
      accepted_rate.push_back(t);
    }
    if (n % 60 == 0) {
      ++frames;
      if (ring.push_manual(make_manual(t, frames))) {
        accepted_manual.push_back(t);
      }
    }
    now = t;
  }

  void service() { logger.service(now, armed); }

  void run_ticks(int count, int service_every = 8) {
    for (int i = 0; i < count; ++i) {
      tick();
      if (i % service_every == 0) {
        service();
      }
    }
  }

  // Advances the clock without ticks (background loop spinning) for `us`, in steps of `step`.
  void idle(TimeUs us, TimeUs step = 10'000) {
    for (TimeUs t = 0; t < us; t += step) {
      now += step;
      service();
    }
  }

  std::vector<Cell> cells;
  Ring ring;
  FakeStorage fake;
  Bytes staging;
  std::uint64_t sessions = 0;
  Logger logger;
  bool armed = false;
  TimeUs now = 0;
  std::uint64_t n = 0;
  std::uint32_t frames = 0;
  std::vector<std::uint64_t> accepted_imu, accepted_rate, accepted_manual;
};

std::uint32_t counter(const Rig& r, StatusField f) { return r.logger.counters()[static_cast<std::size_t>(f)]; }

// Arms, logs for `ticks`, disarms and lets the post-roll run out (the file is then closed).
void one_flight(Rig& r, int ticks) {
  r.armed = true;
  r.run_ticks(ticks);
  r.armed = false;
  r.idle(kPostRoll + 100'000);
}

// ---- header -------------------------------------------------------------------------------------------------------------
TEST(LogHeader, TheHeaderCarriesWhatTheDecoderNeedsAndEndsOnASectorBoundary) {
  const TelemetryHook hook = [] {
    static const std::array<FieldDesc, 2> fields{FieldDesc{"t_ref", "s", 0, 4, FieldType::F32},
                                                 FieldDesc{"mode", "1", 4, 1, FieldType::U8}};
    TelemetryHook h;
    h.version = 7;
    h.fields = fields;
    return h;
  }();
  Rig r(512, 4, 32, 1 << 20, 0, &hook);
  r.service();
  ASSERT_EQ(r.logger.state(), LoggerState::Ready);
  const Bytes file = r.fake.written();
  ASSERT_EQ(file.size() % kSectorBytes, 0U);
  const Parsed p = parse(file);
  EXPECT_TRUE(p.problems.empty());
  ASSERT_FALSE(p.records.empty());

  const Rec& info = p.records.front();
  ASSERT_EQ(info.kind, Kind::HeaderInfo);
  EXPECT_EQ(info.bytes.size(), info::kBytes);
  const std::span<const std::uint8_t> i(info.bytes);
  EXPECT_EQ(get<std::uint16_t>(i, info::kOffVersion), kFormatVersion);
  EXPECT_EQ(get<std::uint16_t>(i, info::kOffSector), kSectorBytes);
  EXPECT_EQ(get<std::uint8_t>(i, info::kOffImage), 1);
  EXPECT_EQ(get<std::uint8_t>(i, info::kOffImage + prim::kSha256DigestBytes - 1), 32);
  EXPECT_EQ(get<std::uint8_t>(i, info::kOffConfig), 0xF0);
  EXPECT_EQ(get<std::uint64_t>(i, info::kOffSchema), 0xDEADBEEF12345678ULL);
  EXPECT_EQ(get<std::uint32_t>(i, info::kOffTickNum), 1953125U);
  EXPECT_EQ(get<std::uint32_t>(i, info::kOffTickDen), 14649U);
  EXPECT_EQ(get_f32(i, info::kOffOdr), 7500.3F);
  EXPECT_EQ(get<std::uint32_t>(i, info::kOffBootSequence), 77U);
  for (std::size_t b = 0; b < 6; ++b) {
    EXPECT_EQ(i[info::kOffBoot + b], b + 1) << "boot byte " << b;
  }

  // Every schema kind and every field is described, with units.
  std::size_t kinds = 0;
  std::size_t fields = 0;
  std::size_t params = 0;
  std::size_t telemetry_fields = 0;
  for (const Rec& rec : p.records) {
    if (rec.kind == Kind::HeaderKind) {
      ++kinds;
    } else if (rec.kind == Kind::HeaderField) {
      ++fields;
      telemetry_fields += rec.bytes[kLeadOffSub] == static_cast<std::uint8_t>(Kind::Telemetry) ? 1 : 0;
    } else if (rec.kind == Kind::HeaderParam) {
      ++params;
    } else if (rec.kind == Kind::HeaderTelemetry) {
      EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(rec.bytes), header_telemetry::kOffVersion), 7U);
      EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(rec.bytes), header_telemetry::kOffFieldCount), 2U);
    }
  }
  std::size_t schema_fields = 0;
  for (const KindSchema& k : record_schema()) {
    schema_fields += k.fields.size();
  }
  EXPECT_EQ(kinds, record_schema().size());
  EXPECT_EQ(fields, schema_fields + 2);
  EXPECT_EQ(telemetry_fields, 1U + 2U) << "the telemetry lead field and the hook's two";
  EXPECT_EQ(params, kParamCount) << "the whole parameter table";
  // A parameter's name, value and unit round-trip.
  const ParamRecord pr = param_defaults()[static_cast<std::size_t>(ParamId::sd_log_write_stall_max_s)];
  bool found = false;
  for (const Rec& rec : p.records) {
    if (rec.kind != Kind::HeaderParam) {
      continue;
    }
    const std::span<const std::uint8_t> b(rec.bytes);
    if (get<std::uint16_t>(b, header_param::kOffId) == static_cast<std::uint16_t>(ParamId::sd_log_write_stall_max_s)) {
      found = true;
      EXPECT_EQ(rec.bytes[kLeadOffSub], static_cast<std::uint8_t>(ParamType::F32));
      EXPECT_EQ(get_f32(b, header_param::kOffValue), pr.value.f32);
      EXPECT_STREQ(reinterpret_cast<const char*>(&rec.bytes[header_param::kOffNames]), "sd_log_write_stall_max_s");
    }
  }
  EXPECT_TRUE(found);
}

// ---- state machine ------------------------------------------------------------------------------------------------------
TEST(LogLogger, ProbesOnTheFirstDisarmedPassAndIsReady) {
  Rig r;
  EXPECT_EQ(r.logger.state(), LoggerState::Absent);
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
  EXPECT_EQ(r.fake.prepare_calls, 1);
  EXPECT_GT(r.fake.write_calls, 0) << "the header is written while disarmed";
  EXPECT_EQ(r.logger.session_id(), kSession);
  r.run_ticks(100);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready) << "disarmed, ticking: no data written";
  EXPECT_EQ(r.fake.prepare_calls, 1);
}

TEST(LogLogger, ArmedLogsEveryRecordAndDisarmEndsTheFileAfterThePostRoll) {
  Rig r(2048, 4, 0);
  r.service();
  r.armed = true;
  r.run_ticks(3000);
  EXPECT_EQ(r.logger.state(), LoggerState::Logging);
  r.armed = false;
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::PostRoll);
  r.idle(kPostRoll / 2);
  EXPECT_EQ(r.logger.state(), LoggerState::PostRoll);
  EXPECT_EQ(r.fake.close_calls, 0);
  r.idle(kPostRoll);
  EXPECT_EQ(r.fake.close_calls, 1);
  ASSERT_EQ(r.fake.closed.size(), 1U);
  EXPECT_EQ(r.fake.last_close_used, r.fake.closed[0].size());
  EXPECT_EQ(r.fake.closed[0].size() % kSectorBytes, 0U);

  const Parsed p = parse(r.fake.closed[0]);
  EXPECT_TRUE(p.problems.empty());
  EXPECT_TRUE(p.ended);
  const auto ends = of_kind(p, Kind::End);
  ASSERT_EQ(ends.size(), 1U);
  EXPECT_EQ(ends[0].bytes[kLeadOffSub], static_cast<std::uint8_t>(EndReason::Disarm));
  // Every sample the ring accepted is in the file, in order, once.
  const auto imus = of_kind(p, Kind::Imu);
  ASSERT_EQ(imus.size(), r.accepted_imu.size());
  for (std::size_t i = 0; i < imus.size(); ++i) {
    ASSERT_EQ(imu_t_lo(imus[i]), static_cast<std::uint32_t>(r.accepted_imu[i])) << i;
    EXPECT_TRUE(std::equal(imus[i].bytes.begin(), imus[i].bytes.end(), encode_imu(make_imu(r.accepted_imu[i])).begin()));
  }
  EXPECT_EQ(of_kind(p, Kind::RateTick).size(), r.accepted_rate.size());
  EXPECT_EQ(of_kind(p, Kind::ManualControl).size(), r.accepted_manual.size());
  EXPECT_EQ(counter(r, StatusField::Sessions), 1U);
  EXPECT_EQ(r.ring.stats().dropped[0], 0U);
  // The first status record precedes the first data record, and carries the state.
  const auto status = of_kind(p, Kind::Status);
  ASSERT_GE(status.size(), 3U);
  EXPECT_EQ(status[0].bytes[kLeadOffSub], static_cast<std::uint8_t>(LoggerState::Logging));
  std::size_t first_status = 0;
  std::size_t first_imu = 0;
  for (std::size_t i = 0; i < p.records.size(); ++i) {
    if (p.records[i].kind == Kind::Status && first_status == 0) {
      first_status = i + 1;
    }
    if (p.records[i].kind == Kind::Imu && first_imu == 0) {
      first_imu = i + 1;
    }
  }
  EXPECT_LT(first_status, first_imu);
  // After the close the logger probes again and is Ready for the next flight with a new session.
  r.idle(kReprobe);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
  EXPECT_EQ(r.fake.prepare_calls, 2);
  EXPECT_EQ(r.logger.session_id(), kSession + 1);
}

TEST(LogLogger, AReArmInsideThePostRollContinuesTheSameFile) {
  Rig r(2048, 4, 0);
  r.service();
  r.armed = true;
  r.run_ticks(500);
  r.armed = false;
  r.idle(kPostRoll / 4);
  ASSERT_EQ(r.logger.state(), LoggerState::PostRoll);
  r.armed = true;
  r.run_ticks(500);
  EXPECT_EQ(r.logger.state(), LoggerState::Logging);
  r.armed = false;
  r.idle(kPostRoll + 100'000);
  EXPECT_EQ(r.fake.prepare_calls, 1 + 0) << "one file for both arms (the next probe is a reprobe period away)";
  ASSERT_EQ(r.fake.closed.size(), 1U);
  const Parsed p = parse(r.fake.closed[0]);
  EXPECT_TRUE(p.ended);
  EXPECT_EQ(of_kind(p, Kind::Imu).size(), r.accepted_imu.size());
  EXPECT_EQ(counter(r, StatusField::Sessions), 1U);
}

TEST(LogLogger, PreRollKeepsOnlyTheDerivedHistoryBeforeTheArmEdge) {
  constexpr std::size_t kPre = 40;  // cells
  Rig r(2048, 4, kPre);
  r.service();
  r.run_ticks(1000, 1);  // disarmed, serviced every tick: the ring is trimmed to the pre-roll each time
  EXPECT_GT(counter(r, StatusField::DiscardedCells), 0U);
  EXPECT_LE(r.ring.backlog_cells(), kPre);
  const std::size_t kept_imu = r.ring.backlog_cells();
  EXPECT_GT(kept_imu, 0U);
  r.armed = true;
  r.service();
  r.run_ticks(10);
  r.armed = false;
  r.idle(kPostRoll + 100'000);
  const Parsed p = parse(r.fake.closed.at(0));
  const auto imus = of_kind(p, Kind::Imu);
  ASSERT_FALSE(imus.empty());
  // The file starts with the newest records before the edge: its first sample is among the last (kPre / 2) accepted before it.
  const std::uint64_t first_t = imu_t_lo(imus.front());
  const std::uint64_t edge_t = 1000 * kTickUs;
  EXPECT_LE(first_t, edge_t);
  EXPECT_GT(first_t + 24 * kTickUs, edge_t) << "the pre-roll is at most the derived few cells, not the whole history";
}

TEST(LogLogger, NoCardIsAStateAndCountersAndArmingIsUnaffected) {
  Rig r(256, 4, 16);
  r.fake.prepare_status = StorageStatus::NoCard;
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::Absent);
  EXPECT_EQ(counter(r, StatusField::PrepareAbsent), 1U);
  EXPECT_EQ(r.fake.prepare_calls, 1);
  r.idle(kReprobe / 2);
  EXPECT_EQ(r.fake.prepare_calls, 1) << "no re-probe before the period";
  r.idle(kReprobe);
  EXPECT_EQ(r.fake.prepare_calls, 2);
  EXPECT_EQ(counter(r, StatusField::PrepareAbsent), 2U);
  // Armed with no card: nothing is probed, nothing is written, and the writer never drops (the ring is drained).
  const int prepares = r.fake.prepare_calls;
  r.armed = true;
  r.run_ticks(20000, 4);
  r.idle(2 * kReprobe);
  EXPECT_EQ(r.fake.prepare_calls, prepares);
  EXPECT_EQ(r.fake.write_calls, 0);
  EXPECT_EQ(r.logger.state(), LoggerState::Absent);
  EXPECT_EQ(r.ring.stats().dropped[0] + r.ring.stats().dropped[1] + r.ring.stats().dropped[2], 0U);
  // The card appears; after landing the logger finds it.
  r.fake.prepare_status = StorageStatus::Ok;
  r.armed = false;
  r.idle(kReprobe + 10'000);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
}

TEST(LogLogger, ACardWithNoRoomIsFullAndIsRetriedWhileDisarmed) {
  Rig r;
  r.fake.prepare_status = StorageStatus::Full;
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::Full);
  EXPECT_EQ(counter(r, StatusField::PrepareFull), 1U);
  r.armed = true;
  r.run_ticks(2000);
  EXPECT_EQ(r.logger.state(), LoggerState::Full);
  EXPECT_EQ(r.fake.write_calls, 0);
  r.fake.prepare_status = StorageStatus::Ok;
  r.armed = false;
  r.idle(kReprobe + 10'000);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
}

TEST(LogLogger, ACardThatFailsToPrepareIsAFault) {
  Rig r;
  r.fake.prepare_status = StorageStatus::Error;
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::Fault);
  EXPECT_EQ(counter(r, StatusField::PrepareError), 1U);
  r.fake.prepare_status = StorageStatus::Ok;
  r.idle(kReprobe + 10'000);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
}

TEST(LogLogger, AFailedHeaderWriteIsAFaultAndTheFileIsNotLogged) {
  Rig r;
  r.fake.fail_write_after = 0;
  r.fake.write_status = StorageStatus::Error;
  r.service();
  EXPECT_EQ(r.logger.state(), LoggerState::Fault);
  EXPECT_EQ(counter(r, StatusField::PrepareError), 1U);
  EXPECT_EQ(counter(r, StatusField::WriteFailures), 1U);
  r.armed = true;
  r.run_ticks(500);
  EXPECT_EQ(r.logger.state(), LoggerState::Fault);
  EXPECT_EQ(r.fake.write_calls, 1) << "no write after the failure";
}

TEST(LogLogger, TheFileFillingUpStopsTheSessionWithAnEndRecordAndTheCloseTruncates) {
  // Room for the header and a few chunks only: header ~ several KiB, chunk 2 KiB.
  Rig r(2048, 4, 0, 24 * 1024);
  r.service();
  ASSERT_EQ(r.logger.state(), LoggerState::Ready);
  r.armed = true;
  r.run_ticks(20000);
  EXPECT_EQ(r.logger.state(), LoggerState::Full);
  EXPECT_LE(r.logger.bytes_written(), r.logger.capacity_bytes());
  const std::uint64_t at_full = r.logger.bytes_written();
  r.run_ticks(2000);
  EXPECT_EQ(r.logger.bytes_written(), at_full) << "nothing more is written for this session";
  EXPECT_EQ(r.ring.stats().dropped[0], 0U) << "a full file does not back up the ring";
  EXPECT_EQ(r.fake.close_calls, 0) << "the close is metadata: not while armed";
  r.armed = false;
  r.service();
  EXPECT_EQ(r.fake.close_calls, 1);
  ASSERT_EQ(r.fake.closed.size(), 1U);
  const Parsed p = parse(r.fake.closed[0]);
  EXPECT_TRUE(p.problems.empty());
  ASSERT_TRUE(p.ended);
  EXPECT_EQ(of_kind(p, Kind::End)[0].bytes[kLeadOffSub], static_cast<std::uint8_t>(EndReason::Full));
  EXPECT_EQ(r.fake.closed[0].size(), at_full);
  EXPECT_EQ(r.logger.state(), LoggerState::Absent);
}

TEST(LogLogger, AWriteFailureMidFlightIsAFaultWithNoMoreWritesAndRecoversAfterTheDisarm) {
  Rig r(2048, 4, 0);
  r.service();
  r.armed = true;
  r.run_ticks(1000);
  const int ok_writes = r.fake.write_calls;
  r.fake.fail_write_after = ok_writes + 1;  // the next write succeeds, the one after fails
  r.fake.write_status = StorageStatus::Error;
  r.run_ticks(4000);
  EXPECT_EQ(r.logger.state(), LoggerState::Fault);
  EXPECT_GE(counter(r, StatusField::WriteFailures), 1U);
  const int writes = r.fake.write_calls;
  r.run_ticks(2000);
  EXPECT_EQ(r.fake.write_calls, writes) << "a faulted session writes nothing more";
  EXPECT_EQ(r.ring.stats().dropped[0], 0U);
  EXPECT_EQ(r.fake.close_calls, 0);
  r.fake.fail_write_after = -1;
  r.armed = false;
  r.service();
  EXPECT_EQ(r.fake.close_calls, 1) << "the file is closed at the disarm so what was written is kept";
  EXPECT_EQ(r.logger.state(), LoggerState::Absent);
  r.idle(kReprobe + 10'000);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
}

TEST(LogLogger, AFailedCloseIsCountedAndTheLoggerGoesOn) {
  Rig r(2048, 4, 0);
  r.fake.close_status = StorageStatus::Error;
  r.service();
  one_flight(r, 400);
  EXPECT_EQ(counter(r, StatusField::CloseFailures), 1U);
  EXPECT_EQ(r.fake.close_calls, 1);
  EXPECT_NE(r.logger.state(), LoggerState::Logging);
  r.idle(kReprobe + 10'000);
  EXPECT_EQ(r.logger.state(), LoggerState::Ready);
}

TEST(LogLogger, TheSlowStreamAndTheStatusRecordsAreWritten) {
  struct Source {
    std::uint32_t calls = 0;
  } src;
  TelemetryHook hook;
  hook.version = 3;
  hook.ctx = &src;
  hook.fill = [](void* ctx, std::span<std::uint8_t> out) noexcept -> std::size_t {
    auto* s = static_cast<Source*>(ctx);
    ++s->calls;
    put<std::uint32_t>(out, 0, s->calls);
    return sizeof(std::uint32_t);
  };
  Rig r(2048, 4, 0, 1 << 20, 4000, &hook);
  r.service();
  one_flight(r, 3000);
  const Parsed p = parse(r.fake.closed.at(0));
  const auto tel = of_kind(p, Kind::Telemetry);
  ASSERT_GT(tel.size(), 10U);
  EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(tel[0].bytes), telemetry::kOffBody), 1U);
  EXPECT_EQ(get<std::uint32_t>(std::span<const std::uint8_t>(tel[1].bytes), telemetry::kOffBody), 2U);
  EXPECT_EQ(counter(r, StatusField::TelemetryRecords), tel.size());
  const auto status = of_kind(p, Kind::Status);
  ASSERT_GE(status.size(), 2U);
  const std::span<const std::uint8_t> last(status.back().bytes);
  EXPECT_GT(get<std::uint64_t>(last, status::kOffTime), 0U);
  EXPECT_EQ(get<std::uint32_t>(last, status::kOffCounters + static_cast<std::size_t>(StatusField::PushedImu) * 4),
            r.ring.stats().pushed[0]);
  EXPECT_EQ(status.back().bytes.size(), status::kBytes);
}

TEST(LogLogger, AStaleTailOfAnEarlierSessionIsNotPartOfTheFile) {
  // The region still holds the sectors of an earlier file (same layout, another session id). The new file's sectors replace
  // the front; its End record stops the reader before the stale tail, and the stale tail alone is rejected by session.
  Rig first(2048, 4, 0, 64 * 1024);
  first.service();
  one_flight(first, 4000);
  ASSERT_FALSE(first.fake.closed.empty());
  Bytes old = first.fake.region;  // the whole region, including what the first file never reached
  Rig second(2048, 4, 0, 64 * 1024);
  second.fake.stale = old;
  second.sessions = 100;  // another session id
  second.service();
  one_flight(second, 300);
  const Bytes& whole = second.fake.region;
  const Parsed p = parse(second.fake.closed.at(0));
  EXPECT_TRUE(p.ended);
  EXPECT_TRUE(p.problems.empty());
  // Reading the untruncated region past the end record meets the first file's sectors: another session.
  const std::size_t used = second.fake.closed[0].size();
  ASSERT_GT(whole.size(), used + kSectorBytes);
  Bytes tail(whole.begin() + static_cast<std::ptrdiff_t>(used), whole.end());
  EXPECT_NE(get<std::uint64_t>(std::span<const std::uint8_t>(tail), kSecOffSession), p.session);
}

// ---- L-3 and L-9 ------------------------------------------------------------------------------------------------------
// Random flights: arm flicker, a card that comes and goes, fills, fails writes and closes. The runs complete (a hal_panic would
// abort the process), the counters only grow, and the fake card's L-9 detector (an ADD_FAILURE on prepare/close while armed or
// Logging) stays silent.
void random_run(unsigned seed) {
  std::mt19937 rng(seed);
  Rig r(256 + rng() % 512, 2 + rng() % 4, rng() % 64, 8 * 1024 + (rng() % 64) * 1024);
  std::array<std::uint32_t, kStatusFields> last{};
  for (int step = 0; step < 4000; ++step) {
    if (rng() % 50 == 0) {
      r.armed = !r.armed;
    }
    if (rng() % 80 == 0) {
      const StorageStatus pick[] = {StorageStatus::Ok, StorageStatus::Ok, StorageStatus::NoCard, StorageStatus::Full,
                                    StorageStatus::Error};
      r.fake.prepare_status = pick[rng() % 5];
    }
    if (rng() % 80 == 0) {
      r.fake.fail_write_after = rng() % 2 == 0 ? -1 : r.fake.write_calls + static_cast<int>(rng() % 4);
      r.fake.write_status = rng() % 2 == 0 ? StorageStatus::Error : StorageStatus::NoCard;
    }
    if (rng() % 80 == 0) {
      r.fake.close_status = rng() % 2 == 0 ? StorageStatus::Ok : StorageStatus::Error;
    }
    for (unsigned k = rng() % 40; k > 0; --k) {
      r.tick();
    }
    if (rng() % 3 != 0) {
      r.now += rng() % 50'000;
      r.service();
    }
    const auto& c = r.logger.counters();
    for (std::size_t i = 0; i < kStatusFields; ++i) {
      const auto f = static_cast<StatusField>(i);
      if (f == StatusField::RingBacklogCells || f == StatusField::RingHighWaterCells) {
        continue;  // levels, refreshed with the Status record
      }
      ASSERT_GE(c[i], last[i]) << "counter " << i << " fell, seed " << seed;
      last[i] = c[i];
    }
  }
  EXPECT_EQ(r.fake.metadata_while_armed, 0) << "seed " << seed;
}

TEST(LogFaults, L3RandomFaultInjectionEndsInStatesAndCountersNeverAPanic) {
  for (unsigned seed = 1; seed <= 60; ++seed) {
    random_run(seed);
    if (HasFatalFailure()) {
      return;
    }
  }
}

TEST(LogNoMetadata, L9NoPrepareOrCloseWhileArmedOrLoggingAcrossRandomRuns) {
  // random_run's Rig carries the fake with its detector; the assertion is metadata_while_armed == 0 inside, and any ADD_FAILURE
  // the fake raised is a failure of this test.
  for (unsigned seed = 1000; seed < 1040; ++seed) {
    random_run(seed);
  }
}

TEST(LogNoMetadata, L9TheDetectorFiresOnAMetadataCallWhileArmedOrLogging) {
  Rig r;
  r.service();
  r.armed = true;
  EXPECT_NONFATAL_FAILURE((void)r.fake.prepare(1024), "L-9: prepare called while armed");
  EXPECT_NONFATAL_FAILURE((void)r.fake.close(0), "L-9: close called while armed");
  r.run_ticks(50);
  ASSERT_EQ(r.logger.state(), LoggerState::Logging);
  r.armed = false;  // the arming flag is clear but the logger still reports Logging: the state alone trips it
  EXPECT_NONFATAL_FAILURE((void)r.fake.prepare(1024), "L-9: prepare called while Logging");
  EXPECT_EQ(r.fake.metadata_while_armed, 3);
}

// ---- round trip ---------------------------------------------------------------------------------------------------------
void write_expected(const std::string& dir, const Rig& r, const std::vector<std::uint64_t>& imu, const std::vector<std::uint64_t>& rate,
                    const std::vector<std::uint64_t>& man, const RingStats& s, const Bytes& file) {
  std::ofstream f(dir + "/roundtrip.mlog", std::ios::binary);
  f.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
  std::ofstream e(dir + "/roundtrip_expected.json");
  e << "{\n";
  auto bits = [](float v) { return std::bit_cast<std::uint32_t>(v); };
  e << "\"imu\": [";
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const ImuSample m = make_imu(imu[i]);
    e << (i ? "," : "") << "\n[" << m.t_us << "," << m.flags << "," << bits(m.gyro_rad_s[0]) << "," << bits(m.gyro_rad_s[1]) << ","
      << bits(m.gyro_rad_s[2]) << "," << bits(m.accel_m_s2[0]) << "," << bits(m.accel_m_s2[1]) << "," << bits(m.accel_m_s2[2]) << "]";
  }
  e << "],\n\"rate_tick\": [";
  for (std::size_t i = 0; i < rate.size(); ++i) {
    const RateTickRecord m = make_rate(rate[i]);
    e << (i ? "," : "") << "\n[" << m.now_us << "," << int{m.arm_state} << "," << int{m.gate_state};
    for (const DshotValue d : m.dshot) {
      e << "," << d.raw();
    }
    e << "]";
  }
  e << "],\n\"manual_control\": [";
  for (std::size_t i = 0; i < man.size(); ++i) {
    const ManualControl m = make_manual(man[i], static_cast<std::uint32_t>(man[i] / (60 * kTickUs)));
    e << (i ? "," : "") << "\n[" << m.frames << "," << m.t_us << "," << bits(m.roll) << "," << bits(m.throttle) << "," << bits(m.channels[0])
      << "," << bits(m.channels[m.channels.size() - 1]) << "," << bits(m.link_quality) << "]";
  }
  e << "],\n\"dropped\": [" << s.dropped[0] << "," << s.dropped[1] << "," << s.dropped[2] << "],\n\"session\": " << r.logger.session_id()
    << "\n}\n";
}

TEST(LogRoundTrip, ASyntheticTickStreamComesBackEqualExceptTheCountedDrops) {
  // A small ring and a background loop that stalls for long stretches, so the ring overflows several times.
  Rig r(512, 4, 0, 4 << 20);
  r.service();
  r.armed = true;
  std::mt19937 rng(7);
  for (int block = 0; block < 60; ++block) {
    const bool stall = block % 7 == 3;
    for (int i = 0; i < 400; ++i) {
      r.tick();
      if (!stall && i % 8 == 0) {
        r.service();
      }
    }
    if (rng() % 5 == 0) {
      r.service();
    }
  }
  r.armed = false;
  r.idle(kPostRoll + 200'000);
  ASSERT_EQ(r.fake.closed.size(), 1U);
  const Bytes& file = r.fake.closed[0];
  const Parsed p = parse(file);
  EXPECT_TRUE(p.problems.empty());
  ASSERT_TRUE(p.ended);

  const RingStats s = r.ring.stats();
  ASSERT_GT(s.dropped[0], 0U) << "the stalls must have overflowed the ring or the test shows nothing";
  EXPECT_EQ(s.pushed[0], r.accepted_imu.size());
  EXPECT_EQ(s.pushed[0] + s.dropped[0], r.n);

  // Equal to the input except the drops: the decoded stamps are exactly the accepted ones, in order.
  const auto imus = of_kind(p, Kind::Imu);
  ASSERT_EQ(imus.size(), r.accepted_imu.size());
  for (std::size_t i = 0; i < imus.size(); ++i) {
    ASSERT_EQ(imu_t_lo(imus[i]), static_cast<std::uint32_t>(r.accepted_imu[i])) << i;
    ASSERT_TRUE(std::equal(imus[i].bytes.begin(), imus[i].bytes.end(), encode_imu(make_imu(r.accepted_imu[i])).begin())) << i;
  }
  const auto rates = of_kind(p, Kind::RateTick);
  ASSERT_EQ(rates.size(), r.accepted_rate.size());
  for (std::size_t i = 0; i < rates.size(); ++i) {
    ASSERT_TRUE(std::equal(rates[i].bytes.begin(), rates[i].bytes.end(), encode_rate_tick(make_rate(r.accepted_rate[i])).begin())) << i;
  }
  const auto mans = of_kind(p, Kind::ManualControl);
  ASSERT_EQ(mans.size(), r.accepted_manual.size());

  // Each loss is announced: the gap records' counts add up to the drops (a final loss with nothing after it is not, which the
  // Status/End counters cover; with the disarm idle there are no further ticks, so any tail loss is checked separately).
  std::array<std::uint64_t, kRingKinds> announced{};
  for (const Rec& g : of_kind(p, Kind::Gap)) {
    for (std::size_t k = 0; k < kRingKinds; ++k) {
      announced[k] += get<std::uint32_t>(std::span<const std::uint8_t>(g.bytes), gap::kOffDropped + k * 4);
    }
  }
  for (std::size_t k = 0; k < kRingKinds; ++k) {
    EXPECT_LE(announced[k], s.dropped[k]) << "kind " << k;
  }
  EXPECT_GT(announced[0], 0U);
  // A gap sits where the loss was: the stamps jump across it, and by about the announced number of ticks.
  std::size_t gaps_checked = 0;
  for (std::size_t i = 1; i + 1 < p.records.size(); ++i) {
    if (p.records[i].kind != Kind::Gap) {
      continue;
    }
    // The nearest IMU record before and after the gap in stream order.
    std::int64_t before = -1;
    std::int64_t after = -1;
    for (std::size_t j = i; j-- > 0;) {
      if (p.records[j].kind == Kind::Imu) {
        before = static_cast<std::int64_t>(imu_t_lo(p.records[j]));
        break;
      }
    }
    for (std::size_t j = i + 1; j < p.records.size(); ++j) {
      if (p.records[j].kind == Kind::Imu) {
        after = static_cast<std::int64_t>(imu_t_lo(p.records[j]));
        break;
      }
    }
    if (before < 0 || after < 0) {
      continue;
    }
    const auto dropped = static_cast<std::int64_t>(get<std::uint32_t>(std::span<const std::uint8_t>(p.records[i].bytes), gap::kOffDropped));
    const std::int64_t ticks_between = (after - before) / static_cast<std::int64_t>(kTickUs) - 1;
    EXPECT_GE(ticks_between, dropped) << "the stamps jump across a gap by at least what it counts";
    ++gaps_checked;
  }
  EXPECT_GT(gaps_checked, 0U);

  if (const char* dir = std::getenv("MARV_LOG_ROUNDTRIP_DIR")) {
    write_expected(dir, r, r.accepted_imu, r.accepted_rate, r.accepted_manual, s, file);
  }
}

}  // namespace
