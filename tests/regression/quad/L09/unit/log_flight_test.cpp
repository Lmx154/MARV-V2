// The onboard logger of the flight image (MARV_FC3V2_FLIGHT_LOG; onboard logger unit U4-4, serena memories
// quad-m4/architect-onboard-logger-2026-10-18 items 1, 4, 5, 7 and quad-m4/architect-logger-u4-2026-10-20) on the host: the wiring,
// fw/hal/target/src/flight_log.cpp, compiled into this test over hal_sim with a fake card in place of the SD storage, driven as
// main.cpp and the HAL hooks drive it on the board: the store opens the arena, flight_log_start runs after the IMU start, the tick
// hooks of log_tick.hpp run in tick order (the sample sink's sample, then hal_manual_control's frame and hal_actuators_write's
// rate tick on a rate tick), and flight_log_service runs from the background loop.
//
//  FlightLogHooks.TheHooksReachTheArenaOnlyThroughTheLease
//      before flight_log_start, before the store opens the arena, and after the logger yields it to the store, a hook writes
//      nothing into the arena; once the logger leases it again its ring is back.
//  FlightLogSession.AnArmToDisarmSessionDecodesClean
//      disarmed (Ready, the pre-roll kept), armed, disarmed through the post-roll: the file is closed with an End record. With
//      MARV_LOG_FLIGHT_DIR set the test writes flight.mlog and flight_expected.json (every record the hooks pushed and the session's
//      times); tools/log/roundtrip_check.py --flight decodes the file with mlog.py and compares (ctest log_flight_decode): no
//      problem, an unbroken slice of the pushed stream from before the arm edge to the end of the post-roll, the samples that are
//      whole counts as raw-count records and the others as 32-byte records, and the Telemetry stream with the header's fields.
//  FlightLogEmb6Open.TheConsoleTriggerLogsWithoutArmingAndTheHeaderCarriesTheFlag
//      the measurement build (MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN, invariant L-5a; the target unit_log_flight_emb6_open only).
//
// flight_log.cpp and the lease keep their state in the process (flight_log_start is once per boot), so each test runs in a forked
// child, as unit_arena_lease does.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#include <marv/hal/hal.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/hal_target/flight_log.hpp>
#include <marv/hal_target/log_tick.hpp>
#include <marv/hal_target/lsm6dsv_decode.hpp>
#include <marv/log/format.hpp>
#include <marv/log/params.hpp>
#include <marv/params/param.hpp>
#include <marv/store/arena.hpp>
#include <marv/types/telemetry.hpp>

namespace {

using namespace marv;
using log::LoggerState;
using log::StorageStatus;
using Bytes = std::vector<std::uint8_t>;

// Scenario values of the test, not flight values: an arena large enough that the ring keeps a pre-roll at the set's stall budget,
// the session's times, and how often the background loop runs.
constexpr std::size_t kArenaBytes = 512 * 1024;
constexpr TimeUs kArmUs = 3'000'000;
constexpr TimeUs kDisarmUs = 4'500'000;
constexpr TimeUs kEndUs = 10'000'000;
constexpr std::uint64_t kServiceTicks = 4;
constexpr std::uint64_t kTicksPerFrame = 60;
constexpr std::uint64_t kInexactEvery = 97;  // every 97th sample is not a whole number of counts
constexpr std::uint8_t kArmedCode = 10;
constexpr std::uint8_t kDisarmedCode = 1;
constexpr std::array<std::int8_t, 3> kBoardSigns{-1, 1, -1};  // the FC3v2 Rev-C mounting map (the board header)

std::array<std::uint8_t, kArenaBytes> g_arena{};

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

// A card that keeps every file it is asked for in memory.
class FakeCard final : public log::LogStorage {
 public:
  std::vector<Bytes> files;
  std::vector<bool> closed;
  int releases = 0;

  log::Prepared prepare(std::uint64_t bytes) noexcept override {
    files.emplace_back();
    closed.push_back(false);
    open_ = true;
    return log::Prepared{StorageStatus::Ok, bytes - bytes % log::kSectorBytes};
  }
  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    if (!open_) {
      return StorageStatus::Error;
    }
    files.back().insert(files.back().end(), chunk.begin(), chunk.end());
    return StorageStatus::Ok;
  }
  StorageStatus close(std::uint64_t used_bytes) noexcept override {
    if (!open_) {
      return StorageStatus::Error;
    }
    open_ = false;
    files.back().resize(std::min<std::size_t>(files.back().size(), used_bytes));
    closed.back() = true;
    return StorageStatus::Ok;
  }
  void release() noexcept override {
    ++releases;
    open_ = false;
  }

 private:
  bool open_ = false;
};

FakeCard g_card;
bool g_armed = false;
std::uint64_t g_telemetry_sequence = 0;

std::uint64_t session_id(void*) noexcept { return 0x0123456789ABCDEFULL; }
const char* param_name_of(std::size_t id) noexcept { return param_name(static_cast<ParamId>(id)); }
std::uint8_t arm_state() noexcept { return g_armed ? kArmedCode : kDisarmedCode; }
bool armed_at(TimeUs t) { return t >= kArmUs && t < kDisarmUs; }
log::LoggerConfig config_for(std::size_t ring_cells, void*) noexcept {
  return log::logger_config_from_params(param_defaults(), ring_cells, &session_id, nullptr);
}

// A snapshot whose fields follow from its sequence number, so the decoded stream can be checked field by field.
bool telemetry_of(Telemetry& out) noexcept {
  ++g_telemetry_sequence;
  const std::uint64_t k = g_telemetry_sequence;
  out = Telemetry{};
  out.sequence = k;
  out.time_us = hal_time_us();
  out.rc_frames = static_cast<std::uint32_t>(k * 3);
  out.gyro_rad_s[0] = 0.5F * static_cast<float>(k);
  out.q_setpoint[3] = -0.125F * static_cast<float>(k);
  out.rc_channels[prim::kCrsfRcChannels - 1] = -0.25F * static_cast<float>(k);
  out.motor_dshot[kQuadXMotors - 1] = static_cast<std::uint16_t>(k % 2048);
  out.arm_state = kArmedCode;
  out.motor_gate = 1;
  return true;
}

log::ImuRawScales board_scales() {
  const hal_target::Lsm6dsvScales fs = hal_target::lsm6dsv_scales();
  log::ImuRawScales s;
  s.gyro_rad_s_per_lsb = fs.gyro_rad_s_per_lsb;
  s.accel_m_s2_per_lsb = fs.accel_m_s2_per_lsb;
  s.sign = kBoardSigns;
  return s;
}

// The sample of tick n as the driver's decode makes it (sign x count x scale in binary32), or, every kInexactEvery-th tick, one
// that is not a whole number of counts.
ImuSample sample_of(std::uint64_t n, const log::ImuRawScales& sc) {
  ImuSample s{};
  s.t_us = hal_time_us();
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid) | imu_flag(ImuFlag::TempValid);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto g = static_cast<std::int16_t>(static_cast<std::int64_t>((n * 7 + i * 1000) % 60000) - 30000);
    const auto a = static_cast<std::int16_t>(static_cast<std::int64_t>((n * 13 + i * 3000) % 60000) - 30000);
    s.gyro_rad_s[i] = static_cast<float>(sc.sign[i]) * static_cast<float>(g) * sc.gyro_rad_s_per_lsb;
    s.accel_m_s2[i] = static_cast<float>(sc.sign[i]) * static_cast<float>(a) * sc.accel_m_s2_per_lsb;
  }
  if (n % kInexactEvery == 0) {
    s.gyro_rad_s[0] += sc.gyro_rad_s_per_lsb * 0.25F;
  }
  s.temp_k = 300.0F;
  return s;
}

ManualControl manual_of(std::uint32_t frames) {
  ManualControl m{};
  m.frames = frames;
  m.t_us = hal_time_us();
  m.roll = 0.001F * static_cast<float>(frames);
  m.throttle = 0.5F;
  m.channels[0] = -1.0F;
  m.channels[prim::kCrsfRcChannels - 1] = 0.25F * static_cast<float>(frames % 4);
  m.link_quality = 1.0F;
  m.link_t_us = m.t_us;
  return m;
}

std::array<DshotValue, kQuadXMotors> dshot_of(std::uint64_t n) {
  std::array<DshotValue, kQuadXMotors> d{};
  for (std::size_t i = 0; i < d.size(); ++i) {
    d[i] = DshotValue::from_raw(static_cast<std::uint16_t>(48 + (n + i * 100) % 2000)).value_or(DshotValue::stop());
  }
  return d;
}

hal_target::FlightLogStart start_inputs(const log::ImuRawScales& sc) {
  hal_target::FlightLogStart s;
  s.storage = &g_card;
  s.config = &config_for;
  s.header.image_hash.fill(0xA5);
  s.header.schema_hash = kParamSchemaHash;
  s.header.tick_num_us = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>(param_defaults()));
  s.header.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>(param_defaults()));
  s.header.odr_hz = 7500.3F;
  s.header.params = param_defaults();
  s.header.param_name = &param_name_of;
  s.imu_raw = sc;
  s.telemetry = &telemetry_of;
  return s;
}

std::array<DshotValue, kQuadXMotors> g_latch{};

hal_sim::TickPeriod tick_period() {
  return hal_sim::TickPeriod{static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>(param_defaults())),
                             static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>(param_defaults()))};
}

void setup_sim() { hal_sim::setup(tick_period(), g_latch, {}); }

// One tick of the hooks, as the board's tick runs them.
void hook_tick(std::uint64_t n, const log::ImuRawScales& sc) {
  hal_sim::begin_tick(n);
  hal_target::log_tick_sample(sample_of(n, sc));
  if (n % 2 == 1) {
    hal_target::log_tick_manual(manual_of(static_cast<std::uint32_t>(n / kTicksPerFrame + 1)));
    const std::array<DshotValue, kQuadXMotors> d = dshot_of(n);
    hal_target::log_tick_rate(d);
  }
  hal_sim::end_tick();
}

bool arena_is(std::uint8_t v) {
  return std::all_of(g_arena.begin(), g_arena.end(), [v](std::uint8_t b) { return b == v; });
}

TEST(FlightLogHooks, TheHooksReachTheArenaOnlyThroughTheLease) {
  in_child([] {
    setup_sim();
    const log::ImuRawScales sc = board_scales();
    constexpr std::uint8_t kStoreBytes = 0x5A;  // what the store writes into the arena it owns
    g_arena.fill(kStoreBytes);
    std::uint64_t n = 0;
    hook_tick(n++, sc);  // before flight_log_start
    EXPECT_EQ(hal_target::flight_log(), nullptr);
    hal_target::flight_log_start(start_inputs(sc));
    ASSERT_NE(hal_target::flight_log(), nullptr);
    hal_target::flight_log_service(hal_time_us(), false);  // the store has not opened the arena: no lease
    EXPECT_FALSE(hal_target::flight_log()->holds_lease());
    hook_tick(n++, sc);
    EXPECT_TRUE(arena_is(kStoreBytes)) << "a hook wrote into the arena before the lease";

    store::arena_open(g_arena);
    hal_target::flight_log_service(hal_time_us(), false);
    ASSERT_TRUE(hal_target::flight_log()->holds_lease());
    ASSERT_NE(hal_target::flight_log()->logger(), nullptr);
    EXPECT_EQ(hal_target::flight_log()->logger()->state(), LoggerState::Ready);
    EXPECT_EQ(g_card.files.size(), 1U);

    // An activation takes the arena back from the Ready logger: the file is closed, the card given back, the ring withdrawn.
    ASSERT_TRUE(store::arena_reclaim());
    EXPECT_FALSE(hal_target::flight_log()->holds_lease());
    EXPECT_EQ(g_card.releases, 1);
    EXPECT_TRUE(g_card.closed.back());
    g_arena.fill(kStoreBytes);
    for (int i = 0; i < 10; ++i) {
      hook_tick(n++, sc);
    }
    EXPECT_TRUE(arena_is(kStoreBytes)) << "a hook wrote into the arena the store had taken back";

    // The next background pass leases it again and prepares a new file; the hooks reach the new ring.
    hal_target::flight_log_service(hal_time_us(), false);
    ASSERT_TRUE(hal_target::flight_log()->holds_lease());
    EXPECT_EQ(g_card.files.size(), 2U);
    g_arena.fill(kStoreBytes);
    hook_tick(n++, sc);
    EXPECT_FALSE(arena_is(kStoreBytes)) << "the hooks did not reach the leased ring";
  });
}

std::uint32_t bits(float v) { return std::bit_cast<std::uint32_t>(v); }

void write_expected(const std::string& dir, const std::vector<std::uint64_t>& ticks, const log::ImuRawScales& sc) {
  const Bytes& file = g_card.files.front();
  std::ofstream f(dir + "/flight.mlog", std::ios::binary);
  f.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
  // The pushed stream, regenerated tick by tick exactly as the hooks made it (the generators depend on the tick alone).
  std::vector<std::string> imu;
  std::vector<std::string> rate;
  std::vector<std::string> man;
  setup_sim();
  std::uint32_t last_frames = 0;
  for (const std::uint64_t n : ticks) {
    hal_sim::begin_tick(n);
    const ImuSample s = sample_of(n, sc);
    imu.push_back("[" + std::to_string(s.t_us) + "," + std::to_string(s.flags) + "," + std::to_string(bits(s.gyro_rad_s[0])) +
                  "," + std::to_string(bits(s.gyro_rad_s[1])) + "," + std::to_string(bits(s.gyro_rad_s[2])) + "," +
                  std::to_string(bits(s.accel_m_s2[0])) + "," + std::to_string(bits(s.accel_m_s2[1])) + "," +
                  std::to_string(bits(s.accel_m_s2[2])) + "," + (n % kInexactEvery == 0 ? "0" : "1") + "]");
    if (n % 2 == 1) {
      const auto frames = static_cast<std::uint32_t>(n / kTicksPerFrame + 1);
      if (frames != last_frames) {
        last_frames = frames;
        const ManualControl m = manual_of(frames);
        man.push_back("[" + std::to_string(m.frames) + "," + std::to_string(m.t_us) + "," + std::to_string(bits(m.roll)) + "," +
                      std::to_string(bits(m.channels[prim::kCrsfRcChannels - 1])) + "]");
      }
      const std::array<DshotValue, kQuadXMotors> d = dshot_of(n);
      std::string r = "[" + std::to_string(hal_time_us()) + "," + std::to_string(armed_at(hal_time_us()) ? kArmedCode : kDisarmedCode);
      for (const DshotValue v : d) {
        r += "," + std::to_string(v.raw());
      }
      rate.push_back(r + "]");
    }
    hal_sim::end_tick();
  }
  std::ofstream e(dir + "/flight_expected.json");
  e << "{\n";
  auto list = [&e](const char* name, const std::vector<std::string>& v) {
    e << "\"" << name << "\": [";
    for (std::size_t i = 0; i < v.size(); ++i) {
      e << (i ? ",\n" : "\n") << v[i];
    }
    e << "],\n";
  };
  list("imu", imu);
  list("rate_tick", rate);
  list("manual_control", man);
  const log::LoggerConfig c = config_for(0, nullptr);
  e << "\"arm_us\": " << kArmUs << ",\n\"disarm_us\": " << kDisarmUs << ",\n\"post_roll_us\": " << c.post_roll_us
    << ",\n\"slow_period_us\": " << c.slow_period_us << ",\n\"session\": " << session_id(nullptr) << ",\n\"build_flags\": 0"
    << ",\n\"telemetry_fields\": " << hal_target::flight_log_telemetry_fields().size() << ",\n\"armed_code\": " << int{kArmedCode}
    << ",\n\"imu_raw\": {\"gyro_rad_s_per_lsb\": " << bits(sc.gyro_rad_s_per_lsb) << ", \"accel_m_s2_per_lsb\": "
    << bits(sc.accel_m_s2_per_lsb) << ", \"sign\": [" << int{sc.sign[0]} << "," << int{sc.sign[1]} << "," << int{sc.sign[2]}
    << "]}\n}\n";
}

TEST(FlightLogSession, AnArmToDisarmSessionDecodesClean) {
  in_child([] {
    setup_sim();
    const log::ImuRawScales sc = board_scales();
    store::arena_open(g_arena);
    hal_target::log_tick_set_arm_source(&arm_state);
    hal_target::flight_log_start(start_inputs(sc));
    std::vector<std::uint64_t> ticks;
    bool was_logging = false;
    TimeUs closed_at = 0;
    for (std::uint64_t n = 0;; ++n) {
      const TimeUs now = hal_sim::stamp_us(tick_period(), n);
      if (now >= kEndUs) {
        break;
      }
      g_armed = armed_at(now);
      hook_tick(n, sc);
      ticks.push_back(n);
      if (n % kServiceTicks == 0) {
        hal_target::flight_log_service(hal_time_us(), g_armed);
        const log::Logger* l = hal_target::flight_log()->logger();
        ASSERT_NE(l, nullptr);
        const bool logging = l->state() == LoggerState::Logging || l->state() == LoggerState::PostRoll;
        if (was_logging && !logging && closed_at == 0) {
          closed_at = hal_time_us();
          EXPECT_TRUE(g_card.closed.front()) << "the session ended without closing its file";
        }
        was_logging = logging;
        EXPECT_FALSE(l->state() == LoggerState::Fault || l->state() == LoggerState::Full);
      }
    }
    const log::LoggerConfig c = config_for(0, nullptr);
    ASSERT_NE(closed_at, 0U) << "the logger did not close the session's file";
    EXPECT_GE(closed_at, kDisarmUs + c.post_roll_us);
    ASSERT_GE(g_card.files.size(), 1U);
    EXPECT_TRUE(g_card.closed.front());
    EXPECT_GT(g_telemetry_sequence, 0U);
    if (const char* dir = std::getenv("MARV_LOG_FLIGHT_DIR")) {
      write_expected(dir, ticks, sc);
    }
  });
}

#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
// The measurement build (invariant L-5a; unit_log_flight_emb6_open only): the console trigger logs without arming, a refused or
// foreign line changes nothing, the logger keeps the arena while the trigger is on, and the file's header carries the flag.
bool console_line(const char* s, bool armed) {
  bool answered = false;
  for (const char* c = s; *c != '\0'; ++c) {
    answered = hal_target::flight_log_emb6_open_console_byte(*c, armed);
  }
  return answered;
}

TEST(FlightLogEmb6Open, TheConsoleTriggerLogsWithoutArmingAndTheHeaderCarriesTheFlag) {
  in_child([] {
    setup_sim();
    const log::ImuRawScales sc = board_scales();
    store::arena_open(g_arena);
    hal_target::flight_log_start(start_inputs(sc));
    std::uint64_t n = 0;
    auto pass = [&n, &sc] {
      hook_tick(n++, sc);
      hal_target::flight_log_service(hal_time_us(), false);
    };
    pass();
    const log::Logger* l = hal_target::flight_log()->logger();
    ASSERT_NE(l, nullptr);
    ASSERT_EQ(l->state(), LoggerState::Ready);
    EXPECT_TRUE(console_line("log start\n", true)) << "answered, and refused while armed";
    pass();
    EXPECT_EQ(l->state(), LoggerState::Ready);
    EXPECT_FALSE(console_line("motor 1 100 10\n", false)) << "not a log command: it goes on to the motor-test parser";
    EXPECT_TRUE(console_line("log start\n", false));
    pass();
    EXPECT_EQ(l->state(), LoggerState::Logging) << "the trigger logs without arming";
    EXPECT_FALSE(store::arena_reclaim()) << "while the trigger is on the logger keeps the arena";
    EXPECT_TRUE(console_line("log stop\n", false));
    pass();
    EXPECT_EQ(l->state(), LoggerState::PostRoll);
    ASSERT_FALSE(g_card.files.empty());
    const Bytes& f = g_card.files.front();
    ASSERT_GE(f.size(), log::kSectorBytes);
    EXPECT_EQ(f[log::kSectorHeaderBytes + log::info::kOffBuildFlags], log::kBuildFlagEmb6Open)
        << "the header of the file does not carry the measurement build's flag";
    EXPECT_NE(hal_target::flight_log_emb6_open_banner[0], '\0');
  });
}
#endif

}  // namespace
