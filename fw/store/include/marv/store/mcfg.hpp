// The MARV binary config file, `.mcfg`, on the firmware side (decision 0053 item 3; ground segment GD-3): a verifying
// reader and the canonical writer of the one section this image reads. The format's single source is the docstring of
// tools/config/mcfg.py; every size and offset below is derived from it field by field. No text is parsed: a record's
// source is an opaque zero-terminated byte string; names, types and units come from the image's compiled schema.
#pragma once

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/arming/arming.hpp"
#include "marv/params/param_types.hpp"
#include "marv/store/sha256.hpp"

namespace marv::store {

inline constexpr std::array<std::uint8_t, sizeof(std::uint64_t)> kMagic{'M', 'A', 'R', 'V', 'M', 'C', 'F', 'G'};
inline constexpr std::uint16_t kFormatVersion = 1;

// Header (all integers little-endian).
inline constexpr std::size_t kOffVersion = kMagic.size();
inline constexpr std::size_t kOffHeaderSize = kOffVersion + sizeof(std::uint16_t);
inline constexpr std::size_t kOffSequence = kOffHeaderSize + sizeof(std::uint16_t);
inline constexpr std::size_t kOffSectionCount = kOffSequence + sizeof(std::uint32_t);
inline constexpr std::size_t kOffHeaderReserved = kOffSectionCount + sizeof(std::uint16_t);
inline constexpr std::size_t kOffPayloadSize = kOffHeaderReserved + sizeof(std::uint16_t);
inline constexpr std::size_t kOffImage = kOffPayloadSize + sizeof(std::uint32_t);
inline constexpr std::size_t kOffReport = kOffImage + kDigestBytes;
inline constexpr std::size_t kOffPayloadHash = kOffReport + kDigestBytes;
inline constexpr std::size_t kOffHeaderTail = kOffPayloadHash + kDigestBytes;
inline constexpr std::size_t kHeaderBytes = kOffHeaderTail + sizeof(std::uint64_t);

// Section directory entry: kind u8 and three zero bytes, schema_hash u64, record_count u32, offset u32, length u32. The
// config hash's per-section entry is the same without the offset.
inline constexpr std::size_t kDirKind = 0;
inline constexpr std::size_t kDirSchema = kDirKind + sizeof(std::uint32_t);
inline constexpr std::size_t kDirCount = kDirSchema + sizeof(std::uint64_t);
inline constexpr std::size_t kDirOffset = kDirCount + sizeof(std::uint32_t);
inline constexpr std::size_t kDirLength = kDirOffset + sizeof(std::uint32_t);
inline constexpr std::size_t kDirEntryBytes = kDirLength + sizeof(std::uint32_t);
inline constexpr std::size_t kHashEntryBytes = kDirEntryBytes - sizeof(std::uint32_t);

// Record: value_bits u32, sigma_bits u32, source_off u32, type u8, sigma_kind u8, method u8, origin u8, flags u8, three
// zero bytes.
inline constexpr std::size_t kRecValue = 0;
inline constexpr std::size_t kRecSigma = kRecValue + sizeof(std::uint32_t);
inline constexpr std::size_t kRecSource = kRecSigma + sizeof(std::uint32_t);
inline constexpr std::size_t kRecType = kRecSource + sizeof(std::uint32_t);
inline constexpr std::size_t kRecSigmaKind = kRecType + 1;
inline constexpr std::size_t kRecMethod = kRecSigmaKind + 1;
inline constexpr std::size_t kRecOrigin = kRecMethod + 1;
inline constexpr std::size_t kRecFlags = kRecOrigin + 1;
inline constexpr std::size_t kRecordBytes = kRecFlags + sizeof(std::uint32_t);
inline constexpr std::uint8_t kFlagLocked = 1;

// Section bodies are padded with zero bytes to a multiple of this.
inline constexpr std::size_t kBodyAlign = sizeof(std::uint32_t);

enum class SectionKind : std::uint8_t { Common, Quad, Rocket, Count };
inline constexpr std::size_t kSectionKinds = static_cast<std::size_t>(SectionKind::Count);
// The section this image reads and writes.
inline constexpr SectionKind kImageSection = SectionKind::Quad;

// Why a config cannot be used. The groups map onto arming::ConfigStatus (fault_status).
enum class Fault : std::uint8_t {
  None,
  // The bytes are not a well-formed file (Corrupt).
  Truncated,    // shorter than a header, or not header_size + payload_size bytes
  Magic,        // not "MARVMCFG"
  Header,       // header_size or a reserved header field
  PayloadHash,  // the payload's SHA-256 differs from the header's
  Directory,    // the section directory breaks a rule (count, kind order, reserved bytes, contiguity, alignment)
  SlotHash,     // a committed slot's content differs from its commit record (length or SHA-256)
  // Not this image's schema (SchemaMismatch).
  Version,      // a format version this reader does not know
  NoSection,    // no section of this image's kind
  Schema,       // the section's schema hash is not kParamSchemaHash
  RecordCount,  // the section's record count is not kParamCount
  // The records are not a config this image accepts (Invalid).
  Record,     // a record the format does not allow (reserved bytes, enumeration range, flags, source offset or empty text)
  Type,       // a record's type differs from the compiled one
  NotFinite,  // an f32 value is NaN or infinite
  Sigma,      // sigma kind and sigma break invariant I1
  Lock,       // a locked parameter differs from its compiled value or provenance, or the lock flag differs
  Validator,  // the composition's validator rejects the table
  // Not validated for this image (NotValidated).
  NotBound,  // no binding, or bound to another image, or no report id
  // The boot step and the commands (not a status of a stored config).
  TooLarge,     // the file does not fit the working memory or a slot
  NoMemory,     // no working memory was given
  PendingRead,  // the pending file could not be read
  WriteFailed,  // a slot or the pending file could not be written
  ReadBack,     // a written slot does not read back as written
  Unmatched,    // the boot request matches no pending file, slot or the factory config
  NoLastGood,   // there is no last good stored config to return to
  Count
};

[[nodiscard]] const char* fault_text(Fault f) noexcept;

// The arming refusal of a stored config with fault `f`: Corrupt, SchemaMismatch, Invalid or NotValidated (Ok for None).
[[nodiscard]] arming::ConfigStatus fault_status(Fault f) noexcept;

// The binding of a config to an image and its validation report (G7): all zero = none.
struct Binding {
  Digest image_sha256{};
  Digest report_id{};
};

// True iff `b` names `image` (not all zero) and a report (not all zero).
[[nodiscard]] bool bound_to(const Binding& b, const Digest& image) noexcept;

// The composition's validator: null if the table is accepted, else the reason (static text). quad_fc: composition::validate.
using Validator = const char* (*)(ParamTable table) noexcept;

struct Decoded {
  Fault fault = Fault::None;
  const char* validator_reason = nullptr;  // the validator's text when fault is Validator
  std::uint32_t sequence = 0;
  Binding binding{};
  Digest config_hash{};  // of this image's section (core IF-1), set once the section is found
};

// Verifies `file` against the format and this image's compiled schema `compiled` (kParamCount records: the types, the
// units, the locks with their values and provenance) and writes the records into `out` (kParamCount records): sources
// point into `file`, units are the compiled table's. Then runs `validate` on `out` (none if null). The binding is
// returned, not checked. On a fault `out` holds whatever was written before it.
[[nodiscard]] Decoded decode(std::span<const std::uint8_t> file, ParamTable compiled, std::span<ParamRecord> out,
                             Validator validate) noexcept;

// The structural checks of decode (header, payload hash, directory, this image's section and its schema) and the config
// hash, without reading the records.
[[nodiscard]] Decoded section_hash(std::span<const std::uint8_t> file) noexcept;

// The canonical file of `table` (kParamCount records) as this image's one section, with `sequence` and `binding`
// (= tools/config/mcfg.py encode). Returns its size, or 0 if it does not fit `out` or a record cannot be encoded (a null
// or empty source).
[[nodiscard]] std::size_t encoded_size(ParamTable table) noexcept;
[[nodiscard]] std::size_t encode(ParamTable table, std::uint32_t sequence, const Binding& binding,
                                 std::span<std::uint8_t> out) noexcept;

// The config hash (core IF-1) of `table` as this image's section; streamed, no buffer.
[[nodiscard]] Digest config_hash(ParamTable table) noexcept;

// Reads and writes of little-endian fields.
template <class T>
[[nodiscard]] constexpr T load_le(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept {
  T v = 0;
  for (std::size_t i = sizeof(T); i > 0; --i) {
    v = static_cast<T>((v << CHAR_BIT) | bytes[offset + i - 1]);
  }
  return v;
}

template <class T>
constexpr void store_le(std::span<std::uint8_t> bytes, std::size_t offset, T v) noexcept {
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    bytes[offset + i] = static_cast<std::uint8_t>(v >> (CHAR_BIT * i));
  }
}

}  // namespace marv::store
