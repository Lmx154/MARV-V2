// Shared by the onboard-logger tests (log_ring_test.cpp, log_logger_test.cpp): a scriptable in-memory LogStorage, a reader of
// the sectors it receives (independent of the framer: it follows format.hpp's description), and record builders.
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "marv/log/budget.hpp"
#include "marv/log/format.hpp"
#include "marv/log/header.hpp"
#include "marv/log/logger.hpp"
#include "marv/log/ring.hpp"
#include "marv/log/storage.hpp"

namespace logtest {

using namespace marv;
using namespace marv::log;
using Bytes = std::vector<std::uint8_t>;

// ---- records -----------------------------------------------------------------------------------------------------------
inline ImuSample make_imu(std::uint64_t t_us) {
  ImuSample s{};
  s.t_us = t_us;
  s.gyro_rad_s = {static_cast<float>(t_us) * 0.25F, static_cast<float>(t_us) * 0.5F, -static_cast<float>(t_us)};
  s.accel_m_s2 = {1.0F + static_cast<float>(t_us), 2.0F, -9.5F};
  s.temp_k = 300.0F;
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  return s;
}

inline RateTickRecord make_rate(std::uint64_t now_us) {
  RateTickRecord r;
  r.now_us = now_us;
  r.arm_state = static_cast<std::uint8_t>(now_us);
  r.gate_state = static_cast<std::uint8_t>(now_us >> 8);
  for (std::size_t i = 0; i < r.dshot.size(); ++i) {
    r.dshot[i] = *DshotValue::from_raw(static_cast<std::uint16_t>(prim::kDshotThrottleMin + i + (now_us & 0x3F)));
  }
  return r;
}

inline ManualControl make_manual(std::uint64_t t_us, std::uint32_t frames) {
  ManualControl m;
  m.t_us = t_us;
  m.frames = frames;
  m.roll = 0.1F;
  m.pitch = -0.2F;
  m.yaw = 0.3F;
  m.throttle = 0.4F;
  for (std::size_t i = 0; i < m.channels.size(); ++i) {
    m.channels[i] = static_cast<float>(i) * 0.0625F - 0.5F;
  }
  m.link_quality = 0.99F;
  m.link_t_us = t_us - 7;
  return m;
}

// ---- storage -------------------------------------------------------------------------------------------------------------
// The card as a byte vector with a scripted life. Every metadata call (prepare, close) while `armed` is read true, or while
// the logger is Logging, is a test failure (invariant L-9).
class FakeStorage final : public LogStorage {
 public:
  // The region prepare hands out; a nonzero fill pattern in `stale` is laid over it first (leftovers of earlier files).
  std::uint64_t capacity = 0;
  std::optional<Bytes> stale;
  StorageStatus prepare_status = StorageStatus::Ok;
  StorageStatus write_status = StorageStatus::Ok;
  int fail_write_after = -1;  // successful writes before write_status applies; -1: never
  StorageStatus close_status = StorageStatus::Ok;

  const Logger* logger = nullptr;
  const bool* armed = nullptr;

  int prepare_calls = 0;
  int write_calls = 0;
  int close_calls = 0;
  int metadata_while_armed = 0;
  std::uint64_t last_close_used = 0;
  Bytes region;       // the current file's region
  std::size_t cursor = 0;
  std::vector<Bytes> closed;  // each file as truncated at close

  Prepared prepare(std::uint64_t bytes) noexcept override {
    note_metadata("prepare");
    ++prepare_calls;
    if (prepare_status != StorageStatus::Ok) {
      return Prepared{prepare_status, 0};
    }
    const std::uint64_t cap = capacity != 0 ? capacity : bytes;
    region.assign(static_cast<std::size_t>(cap), 0);
    if (stale) {
      for (std::size_t i = 0; i < region.size(); ++i) {
        region[i] = (*stale)[i % stale->size()];
      }
    }
    cursor = 0;
    return Prepared{StorageStatus::Ok, cap};
  }

  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    ++write_calls;
    EXPECT_EQ(chunk.size() % kSectorBytes, 0U) << "a chunk is whole sectors";
    EXPECT_EQ(cursor % kSectorBytes, 0U);
    if (fail_write_after >= 0 && write_calls > fail_write_after) {
      return write_status;
    }
    EXPECT_LE(cursor + chunk.size(), region.size()) << "write beyond the prepared region";
    if (cursor + chunk.size() > region.size()) {
      return StorageStatus::Full;
    }
    std::copy(chunk.begin(), chunk.end(), region.begin() + static_cast<std::ptrdiff_t>(cursor));
    cursor += chunk.size();
    return StorageStatus::Ok;
  }

  StorageStatus close(std::uint64_t used_bytes) noexcept override {
    note_metadata("close");
    ++close_calls;
    last_close_used = used_bytes;
    if (close_status != StorageStatus::Ok) {
      return close_status;
    }
    closed.emplace_back(region.begin(), region.begin() + static_cast<std::ptrdiff_t>(std::min<std::uint64_t>(used_bytes, region.size())));
    return StorageStatus::Ok;
  }

  // The file as it is on the card now (whatever was written to the region so far, to the cursor).
  [[nodiscard]] Bytes written() const { return Bytes(region.begin(), region.begin() + static_cast<std::ptrdiff_t>(cursor)); }

 private:
  void note_metadata(const char* what) {
    const bool is_armed = armed != nullptr && *armed;
    const bool is_logging = logger != nullptr && logger->state() == LoggerState::Logging;
    if (is_armed || is_logging) {
      ++metadata_while_armed;
      ADD_FAILURE() << "L-9: " << what << " called while " << (is_armed ? "armed" : "Logging");
    }
  }
};

// ---- reader --------------------------------------------------------------------------------------------------------------
struct Rec {
  Kind kind;
  Bytes bytes;
  std::size_t sector;
};

struct Parsed {
  std::vector<Rec> records;
  std::vector<std::string> problems;
  std::uint64_t session = 0;
  std::size_t sectors = 0;
  bool ended = false;
};

[[nodiscard]] inline std::optional<std::size_t> record_length(std::span<const std::uint8_t> b) {
  const auto kind = static_cast<Kind>(b[0]);
  if (const std::size_t n = ring_record_bytes(kind); n != 0) {
    return n;
  }
  if (kind == Kind::Pad || kind_index(kind) >= kKinds) {
    return std::nullopt;
  }
  if (b.size() < kLeadBytes) {
    return 0;  // need more bytes to say
  }
  return get<std::uint16_t>(b, kLeadOffLength);
}

// Reads sectors until the first invalid one (a problem is listed) or the End record.
inline Parsed parse(std::span<const std::uint8_t> file) {
  Parsed p;
  Bytes pending;
  std::uint32_t expected = 0;
  for (std::size_t i = 0; (i + 1) * kSectorBytes <= file.size() && !p.ended; ++i) {
    const auto s = file.subspan(i * kSectorBytes, kSectorBytes);
    if (!std::equal(kSectorMagic.begin(), kSectorMagic.end(), s.begin())) {
      p.problems.push_back("magic@" + std::to_string(i));
      break;
    }
    if (get<std::uint32_t>(s, kSecOffCrc) != crc32(s.first(kSecOffCrc))) {
      p.problems.push_back("crc@" + std::to_string(i));
      break;
    }
    const auto session = get<std::uint64_t>(s, kSecOffSession);
    if (i == 0) {
      p.session = session;
    } else if (session != p.session) {
      p.problems.push_back("session@" + std::to_string(i));
      break;
    }
    if (get<std::uint32_t>(s, kSecOffSeq) != expected) {
      p.problems.push_back("sequence@" + std::to_string(i));
      break;
    }
    ++expected;
    ++p.sectors;
    const auto payload = s.subspan(kSectorHeaderBytes, kSectorPayloadBytes);
    const std::size_t first = get<std::uint16_t>(s, kSecOffFirst);
    std::size_t pos = 0;
    const std::size_t limit = first == kNoRecord ? payload.size() : first;
    while (!pending.empty() && pos < limit) {
      const auto n = record_length(pending);
      const std::size_t want = (n && *n != 0) ? *n - pending.size() : kLeadBytes - pending.size();
      const std::size_t take = std::min(want, limit - pos);
      pending.insert(pending.end(), payload.begin() + static_cast<std::ptrdiff_t>(pos),
                     payload.begin() + static_cast<std::ptrdiff_t>(pos + take));
      pos += take;
      const auto m = record_length(pending);
      if (m && *m != 0 && pending.size() == *m) {
        p.records.push_back({static_cast<Kind>(pending[0]), pending, i});
        if (static_cast<Kind>(pending[0]) == Kind::End) {
          p.ended = true;
        }
        pending.clear();
      }
    }
    if (!pending.empty()) {
      if (first != kNoRecord) {
        p.problems.push_back("cut@" + std::to_string(i));
        pending.clear();
      } else {
        continue;
      }
    }
    pos = first == kNoRecord ? payload.size() : first;
    while (pos < payload.size() && payload[pos] != 0) {
      const auto rest = payload.subspan(pos);
      const auto n = record_length(rest);
      if (!n) {
        p.problems.push_back("kind@" + std::to_string(i));
        break;
      }
      if (*n == 0 || *n > rest.size()) {
        pending.assign(rest.begin(), rest.end());
        break;
      }
      const auto kind = static_cast<Kind>(rest[0]);
      p.records.push_back({kind, Bytes(rest.begin(), rest.begin() + static_cast<std::ptrdiff_t>(*n)), i});
      if (kind == Kind::End) {
        p.ended = true;
      }
      pos += *n;
    }
  }
  return p;
}

[[nodiscard]] inline std::vector<Rec> of_kind(const Parsed& p, Kind k) {
  std::vector<Rec> out;
  for (const Rec& r : p.records) {
    if (r.kind == k) {
      out.push_back(r);
    }
  }
  return out;
}

[[nodiscard]] inline std::uint64_t imu_t_lo(const Rec& r) { return get<std::uint32_t>(r.bytes, imu::kOffTime); }

}  // namespace logtest
