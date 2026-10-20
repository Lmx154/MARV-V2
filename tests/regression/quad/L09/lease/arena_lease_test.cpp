// Invariant L-8 of the disarmed arena (onboard logger unit U4-2; serena memory quad-m4/architect-logger-u4-2026-10-20): one
// owner at a time. The config store (fw/store) boots on hal_sim's file storage with an arena whose first kConfigSlotBytes are
// its working memory; the onboard logger on the arena (fw/log ArenaLogger) leases the whole arena from the background loop and
// writes to a scripted card whose volume the test watches.
//
//  OneLeaseAtATime         nothing is leased before the boot step; one holder at a time; a refusing holder keeps the arena.
//  LoggerYield             Logger::yield never gives the ring or the card up while armed, Logging or PostRoll; Ready yields.
//  ReadyLoggerYields       an activation takes the arena and the volume back from a Ready logger: the logger has closed its
//                          file and unmounted, and the tick no longer reaches the arena, before the store writes the arena or
//                          the volume.
//  PostRollLoggerKeepsBoth an activation is refused while the logger is in PostRoll: nothing of the arena, the volume or the boot
//                          request is touched; once post-roll is over the same activation goes through.
//
// The store and the lease keep their state in the process (boot is call-once), so every test that boots runs in a forked child,
// as store_test.cpp does, on its own temporary run directory.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <marv/hal/storage.hpp>
#include <marv/hal_sim/storage.hpp>
#include <marv/log/arena_logger.hpp>
#include <marv/log/format.hpp>
#include <marv/log/header.hpp>
#include <marv/log/logger.hpp>
#include <marv/log/ring.hpp>
#include <marv/log/storage.hpp>
#include <marv/params/param.hpp>
#include <marv/store/arena.hpp>
#include <marv/store/sha256.hpp>
#include <marv/store/store.hpp>

namespace {

using namespace marv;
using log::ArenaLogger;
using log::LoggerState;
using log::StorageStatus;
using Bytes = std::vector<std::uint8_t>;

// Scenario values of the test, not flight values: the arena beyond the store's working memory, the logger's staging, its file
// and its timing.
constexpr std::size_t kArenaExtraBytes = 16 * 1024;
constexpr std::size_t kArenaBytes = kConfigSlotBytes + kArenaExtraBytes;
constexpr std::size_t kStagingBytes = 4 * log::kSectorBytes;
constexpr std::uint64_t kFileBytes = 1 << 20;
constexpr TimeUs kPostRoll = 1'000'000;
constexpr TimeUs kReprobe = 500'000;
constexpr std::size_t kPreRollCells = 32;

std::array<std::uint8_t, kArenaBytes> g_arena{};

std::span<const std::uint8_t> as_bytes(std::string_view s) {
  return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}
const store::Digest kImage = store::sha256(as_bytes("unit_arena_lease image"));
const store::Digest kReport = store::sha256(as_bytes("unit_arena_lease report"));

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

Bytes read_file(const std::filesystem::path& p) {
  std::ifstream f(p, std::ios::binary);
  return Bytes(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

// ---------------------------------------------------------------------------------------------------- the store's side

bool g_armed = false;
int g_maintenance_requests = 0;
bool armed_hook() noexcept { return g_armed; }
void request_hook() noexcept { ++g_maintenance_requests; }
bool granted_hook() noexcept { return false; }

// In a child: opens the run directory, boots the store with the arena and loads its table.
void boot_store(const std::string& dir) {
  ASSERT_TRUE(hal_sim::storage_open(dir.c_str()));
  store::BootInputs in;
  in.factory = param_defaults();
  in.image_sha256 = kImage;
  in.hooks = store::Hooks{nullptr, &armed_hook, &request_hook, &granted_hook};
  in.scratch = std::span<std::uint8_t>{g_arena}.first(kConfigSlotBytes);
  in.arena = g_arena;
  const ParamTable t = store::boot(in);
  ASSERT_EQ(t.size(), kParamCount);
  ASSERT_TRUE(params_init(std::span<const ParamRecord, kParamCount>(t.data(), kParamCount)));
}

// ---------------------------------------------------------------------------------------------------- the card

// The logger's card with a volume: prepare mounts it, release unmounts it. At release (the logger's last act before its
// lease ends) it records what the store had done by then: whether the pending file existed and whether the arena still held
// what it held before the store's command.
class Card final : public log::LogStorage {
 public:
  bool mounted = false;
  int prepare_calls = 0;
  int close_calls = 0;
  int release_calls = 0;
  Bytes region;
  std::size_t cursor = 0;

  std::filesystem::path pending;  // the run directory's pending file; empty: not watched
  Bytes arena_before;             // empty: not watched
  bool pending_at_release = false;
  bool arena_changed_at_release = false;

  log::Prepared prepare(std::uint64_t bytes) noexcept override {
    ++prepare_calls;
    mounted = true;
    region.assign(static_cast<std::size_t>(bytes), 0);
    cursor = 0;
    return log::Prepared{StorageStatus::Ok, bytes};
  }

  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    EXPECT_TRUE(mounted) << "a write without the volume";
    if (cursor + chunk.size() > region.size()) {
      return StorageStatus::Full;
    }
    std::copy(chunk.begin(), chunk.end(), region.begin() + static_cast<std::ptrdiff_t>(cursor));
    cursor += chunk.size();
    return StorageStatus::Ok;
  }

  StorageStatus close(std::uint64_t) noexcept override {
    ++close_calls;
    EXPECT_TRUE(mounted) << "a close without the volume";
    return StorageStatus::Ok;
  }

  void release() noexcept override {
    ++release_calls;
    mounted = false;
    if (!pending.empty()) {
      pending_at_release = std::filesystem::exists(pending);
    }
    if (!arena_before.empty()) {
      arena_changed_at_release = !std::equal(arena_before.begin(), arena_before.end(), g_arena.begin());
    }
  }
};

std::uint64_t g_sessions = 0;
std::uint64_t next_session(void*) noexcept { return ++g_sessions; }

log::LoggerConfig config_for(std::size_t ring_cells, void*) noexcept {
  log::LoggerConfig c;
  c.file_bytes = kFileBytes;
  c.post_roll_us = kPostRoll;
  c.reprobe_period_us = kReprobe;
  c.pre_roll_cells = std::min(kPreRollCells, ring_cells);
  c.session_id = &next_session;
  return c;
}

ImuSample sample(TimeUs t) {
  ImuSample s{};
  s.t_us = t;
  s.gyro_rad_s = {0.25F, -0.5F, 1.0F};
  s.accel_m_s2 = {0.0F, 0.0F, -9.75F};
  s.temp_k = 300.0F;
  return s;
}

class ArenaLease : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string tmpl = (std::filesystem::temp_directory_path() / "marv_unit_arena_lease_XXXXXX").string();
    ASSERT_NE(mkdtemp(tmpl.data()), nullptr);
    dir_ = tmpl;
  }
  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }
  [[nodiscard]] std::filesystem::path pending() const { return std::filesystem::path(dir_) / "pending.mcfg"; }
  [[nodiscard]] std::filesystem::path boot_request() const { return std::filesystem::path(dir_) / "boot_request.bin"; }
  std::string dir_;
};

struct Holder {
  bool give_back = true;
  int asked = 0;
  static bool yield(void* ctx) noexcept {
    auto* h = static_cast<Holder*>(ctx);
    ++h->asked;
    return h->give_back;
  }
  [[nodiscard]] store::ArenaHolder holder() { return store::ArenaHolder{&Holder::yield, this}; }
};

// ====================================================================================================== the lease

TEST_F(ArenaLease, OneLeaseAtATime) {
  in_child([&] {
    Holder a;
    Holder b;
    EXPECT_TRUE(store::arena_lease(a.holder()).empty()) << "nothing is leased before the boot step";
    boot_store(dir_);

    const std::span<std::uint8_t> got = store::arena_lease(a.holder());
    EXPECT_EQ(got.data(), g_arena.data());
    EXPECT_EQ(got.size(), g_arena.size());
    EXPECT_TRUE(store::arena_leased());
    EXPECT_TRUE(store::arena_lease(b.holder()).empty()) << "a second lease while the first is out";
    EXPECT_TRUE(store::arena_lease(a.holder()).empty()) << "the holder leases twice";

    a.give_back = false;
    EXPECT_FALSE(store::arena_reclaim());
    EXPECT_EQ(a.asked, 1);
    EXPECT_TRUE(store::arena_leased()) << "a refusing holder keeps the arena";
    EXPECT_TRUE(store::arena_lease(b.holder()).empty());

    a.give_back = true;
    EXPECT_TRUE(store::arena_reclaim());
    EXPECT_EQ(a.asked, 2);
    EXPECT_FALSE(store::arena_leased());
    EXPECT_TRUE(store::arena_reclaim()) << "an idle arena is the store's at once";
    EXPECT_EQ(a.asked, 2);

    EXPECT_TRUE(store::arena_lease(store::ArenaHolder{nullptr, &b}).empty()) << "a holder that cannot be asked back";
    EXPECT_EQ(store::arena_lease(b.holder()).data(), g_arena.data());
    EXPECT_EQ(b.asked, 0);
  });
}

// ====================================================================================================== the logger's rule

TEST(ArenaLeaseLogger, LoggerYield) {
  std::vector<log::Cell> cells(512);
  log::Ring ring(cells);
  Bytes staging(kStagingBytes);
  Card card;
  log::Logger logger(ring, card, staging, config_for(cells.size(), nullptr), log::HeaderInfo{});
  TimeUs t = 0;

  logger.service(t, false);
  ASSERT_EQ(logger.state(), LoggerState::Ready);
  EXPECT_FALSE(logger.yield(t, true)) << "never while armed";
  EXPECT_EQ(logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.close_calls, 0);
  EXPECT_TRUE(card.mounted);

  logger.service(t, true);
  ASSERT_EQ(logger.state(), LoggerState::Logging);
  EXPECT_FALSE(logger.yield(t, false)) << "never while Logging, even with a disarmed reading";
  EXPECT_FALSE(logger.yield(t, true));
  EXPECT_EQ(logger.state(), LoggerState::Logging);

  t += kReprobe;
  logger.service(t, false);
  ASSERT_EQ(logger.state(), LoggerState::PostRoll);
  EXPECT_FALSE(logger.yield(t, false)) << "never while PostRoll";
  EXPECT_EQ(logger.state(), LoggerState::PostRoll);
  EXPECT_EQ(card.close_calls, 0);
  EXPECT_EQ(card.release_calls, 0);
  EXPECT_TRUE(card.mounted);

  t += kPostRoll;
  logger.service(t, false);
  ASSERT_EQ(logger.state(), LoggerState::Absent);
  EXPECT_EQ(card.close_calls, 1);
  t += kReprobe;
  logger.service(t, false);
  ASSERT_EQ(logger.state(), LoggerState::Ready);
  EXPECT_TRUE(logger.yield(t, false)) << "a Ready logger yields";
  EXPECT_EQ(logger.state(), LoggerState::Absent);
  EXPECT_EQ(card.close_calls, 2) << "Ready's file is closed";
  EXPECT_EQ(card.release_calls, 1);
  EXPECT_FALSE(card.mounted) << "the volume is given back";
}

// ====================================================================================================== the store and the logger

TEST_F(ArenaLease, ReadyLoggerYields) {
  in_child([&] {
    boot_store(dir_);
    Card card;
    ArenaLogger lg(card, kStagingBytes, &config_for, nullptr, log::HeaderInfo{});
    const TimeUs t = kReprobe;
    lg.service(t, false);
    ASSERT_TRUE(lg.holds_lease());
    ASSERT_NE(lg.logger(), nullptr);
    ASSERT_EQ(lg.logger()->state(), LoggerState::Ready);
    ASSERT_TRUE(card.mounted);
    EXPECT_TRUE(lg.push_imu(sample(t))) << "the tick reaches the ring";
    EXPECT_TRUE(store::arena_leased());

    card.pending = pending();
    card.arena_before.assign(g_arena.begin(), g_arena.end());
    const store::CommandResult r = store::activate(store::Binding{kImage, kReport});
    EXPECT_EQ(r.step, store::CommandStep::WaitingForGrant) << (r.reason != nullptr ? r.reason : "");

    EXPECT_EQ(card.release_calls, 1);
    EXPECT_FALSE(card.pending_at_release) << "the store wrote the volume before the logger gave it back";
    EXPECT_FALSE(card.arena_changed_at_release) << "the store wrote the arena before the logger gave it back";
    EXPECT_EQ(card.close_calls, 1) << "the Ready file is closed";
    EXPECT_FALSE(card.mounted);
    EXPECT_FALSE(lg.holds_lease());
    EXPECT_EQ(lg.logger(), nullptr);
    EXPECT_FALSE(store::arena_leased());

    const Bytes file = read_file(pending());
    ASSERT_FALSE(file.empty()) << "the activation wrote the pending file";
    EXPECT_TRUE(std::equal(file.begin(), file.end(), g_arena.begin())) << "the store encoded it at the arena's start";
    EXPECT_TRUE(std::filesystem::exists(boot_request()));

    const Bytes after(g_arena.begin(), g_arena.end());
    EXPECT_FALSE(lg.push_imu(sample(t))) << "a tick after the yield reaches no ring";
    EXPECT_FALSE(lg.push_rate_tick(log::RateTickRecord{}));
    EXPECT_FALSE(lg.push_manual(ManualControl{}));
    EXPECT_TRUE(std::equal(after.begin(), after.end(), g_arena.begin())) << "a tick after the yield wrote the arena";
  });
}

TEST_F(ArenaLease, PostRollLoggerKeepsBoth) {
  in_child([&] {
    boot_store(dir_);
    Card card;
    ArenaLogger lg(card, kStagingBytes, &config_for, nullptr, log::HeaderInfo{});
    TimeUs t = kReprobe;
    lg.service(t, false);
    ASSERT_EQ(lg.logger()->state(), LoggerState::Ready);
    lg.service(t, true);
    ASSERT_EQ(lg.logger()->state(), LoggerState::Logging);
    for (int i = 0; i < 100; ++i) {
      t += 133;
      EXPECT_TRUE(lg.push_imu(sample(t)));
    }
    lg.service(t, false);
    ASSERT_EQ(lg.logger()->state(), LoggerState::PostRoll);

    const Bytes before(g_arena.begin(), g_arena.end());
    const store::CommandResult r = store::activate(store::Binding{kImage, kReport});
    EXPECT_EQ(r.step, store::CommandStep::Refused);
    EXPECT_NE(r.reason, nullptr);
    EXPECT_FALSE(std::filesystem::exists(pending())) << "the store took the volume while the logger held it";
    EXPECT_FALSE(std::filesystem::exists(boot_request())) << "the activation went on while the logger held the arena";
    EXPECT_EQ(g_maintenance_requests, 0);
    EXPECT_TRUE(std::equal(before.begin(), before.end(), g_arena.begin())) << "the store wrote the leased arena";
    EXPECT_TRUE(store::arena_leased());
    EXPECT_TRUE(lg.holds_lease());
    EXPECT_EQ(lg.logger()->state(), LoggerState::PostRoll);
    EXPECT_TRUE(card.mounted);
    EXPECT_EQ(card.close_calls, 0);
    EXPECT_EQ(card.release_calls, 0);
    EXPECT_TRUE(lg.push_imu(sample(t + 1))) << "the tick still reaches the ring";

    t += kPostRoll;
    lg.service(t, false);
    ASSERT_EQ(lg.logger()->state(), LoggerState::Absent) << "post-roll is over: the file is closed";
    EXPECT_EQ(card.close_calls, 1);
    const store::CommandResult again = store::activate(store::Binding{kImage, kReport});
    EXPECT_EQ(again.step, store::CommandStep::WaitingForGrant) << (again.reason != nullptr ? again.reason : "");
    EXPECT_TRUE(std::filesystem::exists(pending()));
    EXPECT_FALSE(card.mounted);
    EXPECT_FALSE(lg.holds_lease());
  });
}

}  // namespace
