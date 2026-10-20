// Invariant L-10 of the log file's life (onboard logger unit U4-5; serena memory quad-m4/architect-logger-u4-2026-10-20, item 5):
// prepare reuses the newest file in place when it holds no data sector of its own session, never one that holds one; nothing is
// ever deleted or shortened to make room; a card with no room is a state. The logger runs on a card with a directory (FakeCard:
// the hooks of LifecycleStorage over an in-memory volume), so the rule, the selection of the newest file and the logger's close
// are the code that runs on the board; only FatFs is replaced.
//
//  FileLifeBoots     10 boots without arming leave one file (power off, and a Ready logger that yields); an armed boot whose
//                    first chunk never reached the card leaves a reusable file; every reuse writes a new session's header.
//  FileLifeData      a file with one data sector of a previous session is never reused, shortened or deleted (by a hand-made
//                    sector, a real flight closed, and a real flight cut by power); stale sectors of other sessions do not count;
//                    a card that fails to read the file does not get it reused.
//  FileLifeNewest    the newest file is the highest index with gaps and a shuffled directory; names that are not log files are
//                    ignored; an older header-only file is not reused while the newest holds data.
//  FileLifeFull      no room for the next file is the state Full and deletes nothing; a reusable newest file needs no room.
//  FileLifeClose     a file that logged is truncated at close to what it holds; a header-only file is closed whole.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "log_test_util.hpp"
#include "marv/log/file_life.hpp"

namespace {

using namespace logtest;

constexpr std::uint64_t kFileBytes = 1024 * kSectorBytes;
constexpr TimeUs kPostRoll = 100'000;
constexpr TimeUs kReprobe = 50'000;
constexpr std::uint64_t kTickUs = 133;
constexpr std::size_t kStagingSectors = 4;

struct Event {
  std::uint32_t index;
  std::uint64_t from;
  std::uint64_t to;
};

// A card with a log directory: the files by index, their bytes (a file's size is its vector's size), and a record of everything
// that shortens or deletes one. The hooks are the volume operations prepare needs; write and close are the SD storage's.
class FakeCard final : public LifecycleStorage {
 public:
  std::map<std::uint32_t, Bytes> files;
  std::vector<std::string> extra_names;  // directory entries that are not log files
  bool shuffle = false;                  // the directory lists the highest index first, then the rest in descending order
  std::size_t max_files = ~std::size_t{0};
  StorageStatus mount_status = StorageStatus::Ok;
  bool fail_reads = false;
  std::vector<Event> shortened;
  int created = 0;
  int reused = 0;
  int close_calls = 0;
  std::uint64_t last_close_used = 0;
  int open_index = 0;  // 0: none
  std::size_t cursor = 0;

  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    if (open_index == 0 || cursor + chunk.size() > files[static_cast<std::uint32_t>(open_index)].size()) {
      return StorageStatus::Error;
    }
    std::copy(chunk.begin(), chunk.end(), files[static_cast<std::uint32_t>(open_index)].begin() + static_cast<std::ptrdiff_t>(cursor));
    cursor += chunk.size();
    return StorageStatus::Ok;
  }

  StorageStatus close(std::uint64_t used_bytes) noexcept override {
    ++close_calls;
    last_close_used = used_bytes;
    if (open_index == 0) {
      return StorageStatus::Error;
    }
    Bytes& f = files[static_cast<std::uint32_t>(open_index)];
    if (used_bytes < f.size()) {
      shortened.push_back({static_cast<std::uint32_t>(open_index), f.size(), used_bytes});
      f.resize(static_cast<std::size_t>(used_bytes));
    }
    open_index = 0;
    return StorageStatus::Ok;
  }

 protected:
  void abandon() noexcept override { open_index = 0; }
  std::uint64_t largest_file_bytes() const noexcept override { return ~std::uint64_t{0}; }
  StorageStatus mount() noexcept override { return mount_status; }

  StorageStatus visit_names(NameVisitor visit, void* ctx) noexcept override {
    std::vector<std::uint32_t> order;
    for (const auto& [index, bytes] : files) {
      order.push_back(index);
    }
    if (shuffle) {
      std::reverse(order.begin(), order.end());
      if (order.size() > 2) {
        std::swap(order[0], order[1]);
      }
    }
    for (const std::uint32_t index : order) {
      const auto name = name_of_index(index);
      visit(ctx, std::string_view{name.data(), kNameBytes});
    }
    for (const std::string& n : extra_names) {
      visit(ctx, n);
    }
    return StorageStatus::Ok;
  }

  StorageStatus open_existing(std::uint32_t index, std::uint64_t bytes, bool& usable) noexcept override {
    const auto it = files.find(index);
    usable = it != files.end() && it->second.size() >= bytes;
    if (usable) {
      open_index = static_cast<int>(index);
      cursor = 0;
      ++reused;
    }
    return StorageStatus::Ok;
  }

  bool read_sector(std::uint64_t index, std::span<std::uint8_t> out) noexcept override {
    const Bytes& f = files[static_cast<std::uint32_t>(open_index)];
    if (fail_reads || (index + 1) * kSectorBytes > f.size()) {
      return false;
    }
    std::copy_n(f.begin() + static_cast<std::ptrdiff_t>(index * kSectorBytes), kSectorBytes, out.begin());
    return true;
  }

  void drop_existing() noexcept override {
    open_index = 0;
    --reused;
  }

  StorageStatus create(std::uint32_t index, std::uint64_t bytes) noexcept override {
    if (files.size() >= max_files) {
      return StorageStatus::Full;
    }
    files[index] = Bytes(static_cast<std::size_t>(bytes), 0);
    open_index = static_cast<int>(index);
    cursor = 0;
    ++created;
    return StorageStatus::Ok;
  }
};

std::uint64_t g_sessions = 0;
std::uint64_t next_session(void*) noexcept { return 0x1000 + ++g_sessions; }
const char* name_of(std::size_t id) noexcept { return param_name(static_cast<ParamId>(id)); }

HeaderInfo make_header() {
  HeaderInfo h;
  h.schema_hash = 0xDEADBEEF12345678ULL;
  h.tick_num_us = 1953125;
  h.tick_den = 14649;
  h.odr_hz = 7500.3F;
  h.params = param_defaults();
  h.param_name = name_of;
  return h;
}

// One boot of the logger on `card`.
struct Boot {
  explicit Boot(FakeCard& c, std::uint64_t file_bytes = kFileBytes)
      : card(c), cells(512), ring(cells), staging(kStagingSectors * kSectorBytes), logger(ring, card, staging, config(file_bytes), make_header()) {}

  static LoggerConfig config(std::uint64_t file_bytes) {
    LoggerConfig c;
    c.file_bytes = file_bytes;
    c.post_roll_us = kPostRoll;
    c.reprobe_period_us = kReprobe;
    c.pre_roll_cells = 32;
    c.session_id = next_session;
    return c;
  }

  void tick() {
    ++n;
    now = n * kTickUs;
    static_cast<void>(ring.push_imu(make_imu(now)));
    if (n % 2 == 0) {
      static_cast<void>(ring.push_rate_tick(make_rate(now)));
    }
  }
  void service() { logger.service(now, armed); }
  void run_ticks(int count) {
    for (int i = 0; i < count; ++i) {
      tick();
      if (i % 8 == 0) {
        service();
      }
    }
  }
  void idle(TimeUs us) {
    for (TimeUs t = 0; t < us; t += 10'000) {
      now += 10'000;
      service();
    }
  }
  // Arms, logs `ticks`, disarms and lets the post-roll run out: the file is closed.
  void flight(int ticks) {
    armed = true;
    run_ticks(ticks);
    armed = false;
    idle(kPostRoll + 100'000);
  }

  FakeCard& card;
  std::vector<Cell> cells;
  Ring ring;
  Bytes staging;
  Logger logger;
  bool armed = false;
  TimeUs now = 0;
  std::uint64_t n = 0;
};

// Sectors of the header of a header-only file.
std::size_t header_sectors(const Bytes& file) { return parse(file).sectors; }

// A valid sector of `session` at `index` whose first record is a Status record: what the first data sector of a flight is.
void put_data_sector(Bytes& file, std::size_t index, std::uint64_t session) {
  const std::span<std::uint8_t> s = std::span<std::uint8_t>{file}.subspan(index * kSectorBytes, kSectorBytes);
  std::fill(s.begin(), s.end(), std::uint8_t{0});
  std::copy(kSectorMagic.begin(), kSectorMagic.end(), s.begin());
  put<std::uint64_t>(s, kSecOffSession, session);
  put<std::uint32_t>(s, kSecOffSeq, static_cast<std::uint32_t>(index));
  put<std::uint16_t>(s, kSecOffFirst, 0);
  put<std::uint16_t>(s, kSecOffVersion, kFormatVersion);
  s[kSectorHeaderBytes + kLeadOffKind] = static_cast<std::uint8_t>(Kind::Status);
  put<std::uint16_t>(s, kSectorHeaderBytes + kLeadOffLength, static_cast<std::uint16_t>(status::kBytes));
  put<std::uint32_t>(s, kSecOffCrc, crc32(s.first(kSecOffCrc)));
}

Bytes header_only_file(std::uint64_t bytes = kFileBytes) {
  FakeCard scratch;
  scratch.files.clear();
  Boot b(scratch, bytes);
  b.service();
  Bytes f = scratch.files.begin()->second;
  return f;
}

std::uint64_t session_of(const Bytes& file) { return get<std::uint64_t>(file, kSecOffSession); }

// ---- boots without arming -----------------------------------------------------------------------------------------------
TEST(FileLifeBoots, TenBootsWithoutArmingLeaveOneFile) {
  FakeCard card;
  std::uint64_t last_session = 0;
  for (int boot = 0; boot < 10; ++boot) {
    Boot b(card);
    b.service();
    ASSERT_EQ(b.logger.state(), LoggerState::Ready) << "boot " << boot;
    if (boot % 2 == 1) {
      ASSERT_TRUE(b.logger.yield(b.now, false));  // a Ready logger gives the card up: its file is closed
    }
    ASSERT_EQ(card.files.size(), 1U) << "boot " << boot;
    const Bytes& f = card.files.begin()->second;
    ASSERT_EQ(f.size(), kFileBytes) << "the file is whole after boot " << boot;
    const Parsed p = parse(f);
    EXPECT_NE(p.session, last_session) << "boot " << boot << " wrote its own header";
    EXPECT_EQ(p.session, b.logger.session_id());
    EXPECT_EQ(p.sectors, header_sectors(header_only_file())) << "the whole header is this session's";
    last_session = p.session;
  }
  EXPECT_EQ(card.created, 1);
  EXPECT_EQ(card.reused, 9);
  EXPECT_TRUE(card.shortened.empty());
}

TEST(FileLifeBoots, AYieldedHeaderOnlyFileIsReused) {
  FakeCard card;
  Boot first(card);
  first.service();
  ASSERT_EQ(first.logger.state(), LoggerState::Ready);
  ASSERT_TRUE(first.logger.yield(first.now, false));
  EXPECT_EQ(card.close_calls, 1);
  EXPECT_EQ(card.last_close_used, kFileBytes) << "a header-only file is closed whole";
  EXPECT_TRUE(card.shortened.empty());
  const std::uint64_t first_session = first.logger.session_id();

  Boot second(card);
  second.service();
  ASSERT_EQ(second.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 1U);
  EXPECT_EQ(card.created, 1);
  EXPECT_EQ(session_of(card.files.begin()->second), second.logger.session_id());
  EXPECT_NE(second.logger.session_id(), first_session);
}

TEST(FileLifeBoots, AnArmedBootWhoseFirstChunkNeverReachedTheCardLeavesAReusableFile) {
  FakeCard card;
  {
    Boot b(card);
    b.service();
    b.armed = true;
    b.run_ticks(10);  // the Status record and a few samples: still in the staging buffer
    ASSERT_EQ(b.logger.state(), LoggerState::Logging);
    ASSERT_EQ(b.logger.bytes_written(), header_sectors(card.files.begin()->second) * kSectorBytes) << "no data chunk written";
  }  // power off
  Boot next(card);
  next.service();
  EXPECT_EQ(next.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 1U) << "the card holds no data sector of that session";
}

// ---- L-10: a file with data ---------------------------------------------------------------------------------------------
TEST(FileLifeData, AFileWithOneDataSectorOfAPreviousSessionIsNeverReusedShortenedOrDeleted) {
  FakeCard card;
  {
    Boot b(card);
    b.service();
    ASSERT_TRUE(b.logger.yield(b.now, false));
  }
  Bytes& f = card.files.begin()->second;
  const std::size_t h = header_sectors(f);
  ASSERT_GT(h, 0U);
  put_data_sector(f, h, session_of(f));
  const Bytes before = f;

  Boot next(card);
  next.service();
  ASSERT_EQ(next.logger.state(), LoggerState::Ready);
  ASSERT_EQ(card.files.size(), 2U) << "the next index was created";
  EXPECT_EQ(card.files.at(1), before) << "byte for byte untouched, same size";
  EXPECT_EQ(card.created, 2);
  EXPECT_TRUE(card.shortened.empty());
  ASSERT_TRUE(next.logger.yield(next.now, false));
  EXPECT_EQ(card.files.at(1), before);
  EXPECT_EQ(card.files.size(), 2U);
}

TEST(FileLifeData, ARealFlightIsClosedTruncatedAndNeverReused) {
  FakeCard card;
  Bytes flown;
  {
    Boot b(card);
    b.service();
    b.flight(2000);  // the close, then the logger prepares the next file at once: that one is header-only
    ASSERT_EQ(b.logger.state(), LoggerState::Ready);
    ASSERT_EQ(card.files.size(), 2U);
    flown = card.files.at(1);
    ASSERT_LT(flown.size(), kFileBytes) << "f_truncate on close stays";
    ASSERT_TRUE(parse(flown).ended);
    ASSERT_EQ(card.shortened.size(), 1U);
    ASSERT_EQ(card.files.at(2).size(), kFileBytes) << "the next file is header-only and whole";
  }  // power off

  for (int boot = 0; boot < 3; ++boot) {
    Boot next(card);
    next.service();
    ASSERT_EQ(next.logger.state(), LoggerState::Ready);
    EXPECT_EQ(card.files.size(), 2U) << "the flown file is not reused; the header-only one after it is";
    EXPECT_EQ(card.files.at(1), flown);
    EXPECT_EQ(session_of(card.files.at(2)), next.logger.session_id());
  }
  EXPECT_EQ(card.shortened.size(), 1U) << "only the flight's own close shortened anything";
}

TEST(FileLifeData, AFlightCutByPowerKeepsItsFileWholeAndItIsNotReused) {
  FakeCard card;
  {
    Boot b(card);
    b.service();
    b.armed = true;
    b.run_ticks(2000);  // many chunks on the card, never closed
    ASSERT_EQ(b.logger.state(), LoggerState::Logging);
  }
  const Bytes cut = card.files.at(1);
  ASSERT_EQ(cut.size(), kFileBytes);
  Boot next(card);
  next.service();
  EXPECT_EQ(card.files.size(), 2U);
  EXPECT_EQ(card.files.at(1), cut);
  EXPECT_TRUE(card.shortened.empty());
}

TEST(FileLifeData, SectorsOfOtherSessionsPastTheHeaderAreNotData) {
  FakeCard card;
  {
    Boot b(card);
    b.service();
    ASSERT_TRUE(b.logger.yield(b.now, false));
  }
  Bytes& f = card.files.begin()->second;
  const std::size_t h = header_sectors(f);
  put_data_sector(f, h, session_of(f) + 1);      // a valid sector, another session's: a leftover of an earlier file
  put_data_sector(f, h + 1, session_of(f) + 1);
  Boot next(card);
  next.service();
  EXPECT_EQ(next.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 1U) << "stale sectors of another session are not this file's data";
}

TEST(FileLifeData, AFileTheCardCannotReadBackIsNotReused) {
  FakeCard card;
  {
    Boot b(card);
    b.service();
    ASSERT_TRUE(b.logger.yield(b.now, false));
  }
  const Bytes before = card.files.at(1);
  card.fail_reads = true;
  Boot next(card);
  next.service();
  EXPECT_EQ(card.files.size(), 2U);
  EXPECT_EQ(card.files.at(1), before);
}

TEST(FileLifeData, ATooShortHeaderOnlyFileIsLeftAloneAndANewOneIsCreated) {
  FakeCard card;
  {
    Boot b(card, kFileBytes / 2);
    b.service();
    ASSERT_TRUE(b.logger.yield(b.now, false));
  }
  const Bytes before = card.files.at(1);
  Boot next(card);  // asks for more than the file holds
  next.service();
  EXPECT_EQ(card.files.size(), 2U);
  EXPECT_EQ(card.files.at(1), before);
}

// ---- the newest file ----------------------------------------------------------------------------------------------------
TEST(FileLifeNewest, TheNewestIsTheHighestIndexWithGapsAndAShuffledDirectory) {
  for (const bool shuffle : {false, true}) {
    FakeCard card;
    card.shuffle = shuffle;
    card.files[3] = header_only_file();
    card.files[7] = header_only_file();
    card.files[12] = header_only_file();
    const Bytes older3 = card.files[3];
    const Bytes older7 = card.files[7];
    Boot b(card);
    b.service();
    ASSERT_EQ(b.logger.state(), LoggerState::Ready);
    EXPECT_EQ(card.files.size(), 3U) << "no file 13 and no file 8";
    EXPECT_EQ(session_of(card.files.at(12)), b.logger.session_id()) << "file 12, the highest index, was reused";
    EXPECT_EQ(card.files.at(3), older3);
    EXPECT_EQ(card.files.at(7), older7);
  }
}

TEST(FileLifeNewest, ANewestFileWithDataGetsTheNextIndexAboveTheGapNotTheOldGap) {
  FakeCard card;
  card.files[3] = header_only_file();
  card.files[12] = header_only_file();
  put_data_sector(card.files[12], header_sectors(card.files[12]), session_of(card.files[12]));
  const Bytes older = card.files[3];
  const Bytes newest = card.files[12];
  Boot b(card);
  b.service();
  ASSERT_EQ(card.files.size(), 3U);
  ASSERT_TRUE(card.files.contains(13));
  EXPECT_EQ(card.files.at(3), older) << "an older header-only file is not reused while the newest holds data";
  EXPECT_EQ(card.files.at(12), newest);
}

TEST(FileLifeNewest, NamesThatAreNotLogFilesAreIgnored) {
  FakeCard card;
  card.files[5] = header_only_file();
  card.extra_names = {"00000099.txt", "0000099.mlog", "000000100.mlog", "0000009x.mlog", "NNNNNNNN.mlog", "readme", ".mlog"};
  Boot b(card);
  b.service();
  ASSERT_EQ(b.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 1U);
  EXPECT_EQ(session_of(card.files.at(5)), b.logger.session_id());
}

TEST(FileLifeNewest, TheNameCodecRoundTripsAndRejectsOtherNames) {
  for (const std::uint32_t i : {1U, 9U, 10U, 12345678U, largest_index()}) {
    const auto n = name_of_index(i);
    EXPECT_EQ(index_of_name(std::string_view{n.data(), kNameBytes}), i);
    EXPECT_EQ(n[kNameBytes], '\0');
  }
  EXPECT_EQ(index_of_name("00000000.mlog"), 0U);
  EXPECT_EQ(index_of_name("00000001.MLOG"), 0U);
  EXPECT_EQ(index_of_name("1.mlog"), 0U);
}

// ---- the card is full ---------------------------------------------------------------------------------------------------
TEST(FileLifeFull, NoRoomForTheNextFileIsAStateAndDeletesNothing) {
  FakeCard card;
  card.files[1] = header_only_file();
  card.files[2] = header_only_file();
  put_data_sector(card.files[2], header_sectors(card.files[2]), session_of(card.files[2]));
  card.max_files = 2;
  const auto snapshot = card.files;

  Boot b(card);
  b.service();
  EXPECT_EQ(b.logger.state(), LoggerState::Full);
  EXPECT_EQ(b.logger.counters()[static_cast<std::size_t>(StatusField::PrepareFull)], 1U);
  EXPECT_EQ(card.files, snapshot) << "no automatic deletion, nothing shortened";

  // Arming is not gated on it, and the file stays whole while the vehicle flies.
  b.armed = true;
  b.run_ticks(500);
  b.armed = false;
  b.idle(kPostRoll + 100'000);
  EXPECT_EQ(card.files, snapshot);
  EXPECT_TRUE(card.shortened.empty());
  EXPECT_NE(b.logger.state(), LoggerState::Logging);

  // Room appears (the user cleared the card): the next probe logs again.
  card.max_files = 3;
  b.idle(kReprobe * 4);
  EXPECT_EQ(b.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 3U);
}

TEST(FileLifeFull, AReusableNewestFileNeedsNoRoom) {
  FakeCard card;
  card.files[1] = header_only_file();
  card.files[2] = header_only_file();
  card.max_files = 2;
  Boot b(card);
  b.service();
  EXPECT_EQ(b.logger.state(), LoggerState::Ready);
  EXPECT_EQ(card.files.size(), 2U);
}

TEST(FileLifeFull, NoCardIsAbsentNotFull) {
  FakeCard card;
  card.mount_status = StorageStatus::NoCard;
  Boot b(card);
  b.service();
  EXPECT_EQ(b.logger.state(), LoggerState::Absent);
  EXPECT_TRUE(card.files.empty());
}

// ---- close --------------------------------------------------------------------------------------------------------------
TEST(FileLifeClose, AFileThatLoggedIsTruncatedToWhatItHolds) {
  FakeCard card;
  Boot b(card);
  b.service();
  b.flight(2000);
  ASSERT_EQ(card.shortened.size(), 1U);
  const Event& e = card.shortened[0];
  EXPECT_EQ(e.index, 1U);
  EXPECT_EQ(e.from, kFileBytes);
  EXPECT_EQ(card.files.at(1).size(), e.to);
  EXPECT_EQ(e.to % kSectorBytes, 0U);
  const Parsed p = parse(card.files.at(1));
  EXPECT_TRUE(p.ended);
  EXPECT_EQ(p.sectors * kSectorBytes, e.to) << "to the last sector the session wrote, the End record's";
  EXPECT_GT(e.to, header_sectors(header_only_file()) * kSectorBytes) << "it holds data";
}

}  // namespace
