// The header of a log file: the records written while the vehicle is disarmed, before the first data record, so the file
// explains itself (the decoder needs nothing else): the format version, the image, config and schema hashes, the boot report,
// the tick period and ODR, the list of record kinds with their fields and units, the Telemetry stream's version and fields
// (a placeholder hook until the unit-11a Telemetry exists), and the whole parameter table (name, type, value, unit).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/log/format.hpp"
#include "marv/log/framer.hpp"
#include "marv/params/param_types.hpp"
#include "marv/prim/constants.hpp"

namespace marv::log {

struct FieldDesc {
  const char* name;
  const char* unit;  // SI symbol, "1" if dimensionless
  std::uint16_t offset;
  std::uint8_t size;
  FieldType type;
};

// The slow stream. version 0 and no fields: no Telemetry stream yet. `fill` writes the snapshot's bytes into `out` and
// returns their count (0: nothing to log now); it is called from the background loop.
struct TelemetryHook {
  std::uint32_t version = 0;
  std::span<const FieldDesc> fields{};
  std::size_t (*fill)(void* ctx, std::span<std::uint8_t> out) noexcept = nullptr;
  void* ctx = nullptr;
};

// The boot report as plain bytes (fw/store's BootReport, which fw/log does not depend on).
struct BootFields {
  std::uint8_t config_source = 0;
  std::uint8_t slot = 0;
  std::uint8_t config_status = 0;
  std::uint8_t fault = 0;
  std::uint8_t request = 0;
  std::uint8_t request_fault = 0;
  std::uint32_t sequence = 0;
};

// The generated name of a parameter id (param_name in marv/params/param.hpp, which belongs to one parameter set).
using ParamNameFn = const char* (*)(std::size_t id) noexcept;

struct HeaderInfo {
  std::uint64_t session_start_us = 0;
  std::array<std::uint8_t, prim::kSha256DigestBytes> image_hash{};
  std::array<std::uint8_t, prim::kSha256DigestBytes> config_hash{};
  std::uint64_t schema_hash = 0;
  BootFields boot{};
  std::uint32_t tick_num_us = 0;  // the tick period in us as tick_num_us / tick_den
  std::uint32_t tick_den = 1;
  float odr_hz = 0.0F;
  std::span<const ParamRecord> params{};
  ParamNameFn param_name = nullptr;  // null: parameters are not written
  const TelemetryHook* telemetry = nullptr;
};

// One kind's description for the header (also read by tests and the decoder's documentation).
struct KindSchema {
  Kind kind;
  const char* name;
  std::size_t record_bytes;  // 0: variable
  std::span<const FieldDesc> fields;
};
[[nodiscard]] std::span<const KindSchema> record_schema() noexcept;

// Appends the header records through `framer` and flushes (so the data starts on a fresh sector). False: a chunk was refused.
[[nodiscard]] bool write_header(const HeaderInfo& info, SectorFramer& framer, ChunkSink& sink) noexcept;

}  // namespace marv::log
