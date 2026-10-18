#include "marv/store/mcfg.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>

#include <marv/params/param_ids.hpp>

#include "internal.hpp"

namespace marv::store {
namespace detail {

[[nodiscard]] std::uint32_t value_bits(const ParamValue& v) noexcept {
  return v.type == ParamType::F32 ? std::bit_cast<std::uint32_t>(v.f32) : std::bit_cast<std::uint32_t>(v.i32);
}

[[nodiscard]] bool same_source(const char* a, const char* b) noexcept {
  return a == b || (a != nullptr && b != nullptr && std::strcmp(a, b) == 0);
}

// Value and provenance, bit for bit (the unit is the compiled one by construction).
bool same_record(const ParamRecord& a, const ParamRecord& b) noexcept {
  return a.value.type == b.value.type && value_bits(a.value) == value_bits(b.value) &&
         std::bit_cast<std::uint32_t>(a.sigma) == std::bit_cast<std::uint32_t>(b.sigma) && a.origin == b.origin &&
         a.method == b.method && a.locked == b.locked && a.sigma_kind == b.sigma_kind && same_source(a.source, b.source);
}

}  // namespace detail

namespace {

using detail::same_record;
using detail::same_source;
using detail::value_bits;

using Bytes = std::span<const std::uint8_t>;

constexpr auto kImageKind = static_cast<std::uint8_t>(kImageSection);
// In the config hash's entry the length takes the place the offset has in the directory entry.
constexpr std::size_t kHashLength = kDirOffset;

struct Section {
  Bytes body;
  std::uint64_t schema = 0;
  std::uint32_t count = 0;
};

[[nodiscard]] Digest digest_at(Bytes b, std::size_t offset) noexcept {
  Digest d{};
  std::copy_n(b.begin() + static_cast<std::ptrdiff_t>(offset), d.size(), d.begin());
  return d;
}

[[nodiscard]] bool all_zero(const Digest& d) noexcept {
  return std::all_of(d.begin(), d.end(), [](std::uint8_t x) { return x == 0; });
}

// The config hash's entry of a section: kind u8, three zero bytes, schema_hash u64, record_count u32, length u32.
[[nodiscard]] std::array<std::uint8_t, kHashEntryBytes> hash_entry(std::uint64_t schema, std::uint32_t count,
                                                                   std::size_t length) noexcept {
  std::array<std::uint8_t, kHashEntryBytes> e{};
  e[kDirKind] = kImageKind;
  store_le<std::uint64_t>(e, kDirSchema, schema);
  store_le<std::uint32_t>(e, kDirCount, count);
  store_le<std::uint32_t>(e, kHashLength, static_cast<std::uint32_t>(length));
  return e;
}

// The header, the payload hash, the directory and this image's section with its schema; fills d.sequence, d.binding and
// d.config_hash (once the section is found).
[[nodiscard]] Fault structure(Bytes file, Decoded& d, Section& s) noexcept {
  if (file.size() < kHeaderBytes) {
    return Fault::Truncated;
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), file.begin())) {
    return Fault::Magic;
  }
  if (load_le<std::uint16_t>(file, kOffVersion) != kFormatVersion) {
    return Fault::Version;
  }
  if (load_le<std::uint16_t>(file, kOffHeaderSize) != kHeaderBytes ||
      load_le<std::uint16_t>(file, kOffHeaderReserved) != 0 || load_le<std::uint64_t>(file, kOffHeaderTail) != 0) {
    return Fault::Header;
  }
  const std::size_t psize = load_le<std::uint32_t>(file, kOffPayloadSize);
  if (file.size() - kHeaderBytes != psize) {
    return Fault::Truncated;
  }
  const Bytes payload = file.subspan(kHeaderBytes);
  if (sha256(payload) != digest_at(file, kOffPayloadHash)) {
    return Fault::PayloadHash;
  }
  d.sequence = load_le<std::uint32_t>(file, kOffSequence);
  d.binding.image_sha256 = digest_at(file, kOffImage);
  d.binding.report_id = digest_at(file, kOffReport);

  const std::size_t count = load_le<std::uint16_t>(file, kOffSectionCount);
  if (count == 0 || count > kSectionKinds || count * kDirEntryBytes > psize) {
    return Fault::Directory;
  }
  std::size_t expected = count * kDirEntryBytes;
  std::size_t previous_kind = 0;
  bool found = false;
  for (std::size_t i = 0; i < count; ++i) {
    const Bytes entry = payload.subspan(i * kDirEntryBytes, kDirEntryBytes);
    const std::uint32_t kind_word = load_le<std::uint32_t>(entry, kDirKind);
    const std::size_t kind = kind_word & std::numeric_limits<std::uint8_t>::max();
    if ((kind_word >> CHAR_BIT) != 0 || kind >= kSectionKinds || (i > 0 && kind <= previous_kind)) {
      return Fault::Directory;
    }
    previous_kind = kind;
    const std::size_t offset = load_le<std::uint32_t>(entry, kDirOffset);
    const std::size_t length = load_le<std::uint32_t>(entry, kDirLength);
    if (offset != expected || length % kBodyAlign != 0 || length > psize - offset) {
      return Fault::Directory;
    }
    expected = offset + length;
    if (kind == kImageKind) {
      s.body = payload.subspan(offset, length);
      s.schema = load_le<std::uint64_t>(entry, kDirSchema);
      s.count = load_le<std::uint32_t>(entry, kDirCount);
      found = true;
    }
  }
  if (expected != psize) {
    return Fault::Directory;
  }
  if (!found) {
    return Fault::NoSection;
  }
  Sha256 h;
  h.update(hash_entry(s.schema, s.count, s.body.size()));
  h.update(s.body);
  d.config_hash = h.finish();
  if (s.schema != kParamSchemaHash) {
    return Fault::Schema;
  }
  if (s.count != kParamCount) {
    return Fault::RecordCount;
  }
  return Fault::None;
}

[[nodiscard]] Fault records(const Section& s, ParamTable compiled, std::span<ParamRecord> out) noexcept {
  const std::size_t table = s.count * kRecordBytes;
  if (s.body.size() < table) {
    return Fault::Record;
  }
  const Bytes strings = s.body.subspan(table);
  if (!strings.empty() && strings.back() != 0) {
    return Fault::Record;
  }
  for (std::size_t i = 0; i < s.count; ++i) {
    const Bytes r = s.body.subspan(i * kRecordBytes, kRecordBytes);
    const std::uint8_t type = r[kRecType];
    const std::uint8_t kind = r[kRecSigmaKind];
    const std::uint8_t method = r[kRecMethod];
    const std::uint8_t origin = r[kRecOrigin];
    const std::uint8_t flags = r[kRecFlags];
    if ((load_le<std::uint32_t>(r, kRecFlags) >> CHAR_BIT) != 0 || type > static_cast<std::uint8_t>(ParamType::I32) ||
        kind > static_cast<std::uint8_t>(SigmaKind::Choice) || method > static_cast<std::uint8_t>(ParamMethod::Scenario) ||
        origin > static_cast<std::uint8_t>(ParamOrigin::Identified) || (flags & ~kFlagLocked) != 0) {
      return Fault::Record;
    }
    const std::size_t off = load_le<std::uint32_t>(r, kRecSource);
    if (off >= strings.size() || strings[off] == 0) {
      return Fault::Record;
    }
    const auto sigma_kind = static_cast<SigmaKind>(kind);
    const auto sigma = std::bit_cast<float>(load_le<std::uint32_t>(r, kRecSigma));
    if (!sigma_consistent(sigma_kind, sigma)) {
      return Fault::Sigma;
    }
    const ParamRecord& c = compiled[i];
    if (static_cast<ParamType>(type) != c.value.type) {
      return Fault::Type;
    }
    const std::uint32_t bits = load_le<std::uint32_t>(r, kRecValue);
    ParamValue value{c.value.type, 0.0F, 0};
    if (value.type == ParamType::F32) {
      value.f32 = std::bit_cast<float>(bits);
      if (!std::isfinite(value.f32)) {
        return Fault::NotFinite;
      }
    } else {
      value.i32 = std::bit_cast<std::int32_t>(bits);
    }
    const ParamRecord rec{value,     sigma,      static_cast<ParamOrigin>(origin),
                          static_cast<ParamMethod>(method),
                          (flags & kFlagLocked) != 0,
                          sigma_kind, c.unit,    reinterpret_cast<const char*>(strings.data() + off)};
    if (rec.locked != c.locked || (c.locked && !same_record(rec, c))) {
      return Fault::Lock;
    }
    out[i] = rec;
  }
  return Fault::None;
}

// The canonical string table of a table: each record's offset, distinct texts in first-use order.
struct Layout {
  std::array<std::uint32_t, kParamCount> source_off{};
  std::size_t body_bytes = 0;
  bool ok = false;
};

[[nodiscard]] Layout layout(ParamTable t) noexcept {
  Layout l;
  if (t.size() != kParamCount) {
    return l;
  }
  std::size_t strings = 0;
  for (std::size_t i = 0; i < kParamCount; ++i) {
    const char* src = t[i].source;
    if (src == nullptr || src[0] == '\0') {
      return l;
    }
    bool seen = false;
    for (std::size_t j = 0; j < i && !seen; ++j) {
      if (same_source(t[j].source, src)) {
        l.source_off[i] = l.source_off[j];
        seen = true;
      }
    }
    if (!seen) {
      l.source_off[i] = static_cast<std::uint32_t>(strings);
      strings += std::strlen(src) + 1;
    }
  }
  const std::size_t unpadded = (kParamCount * kRecordBytes) + strings;
  l.body_bytes = unpadded + ((kBodyAlign - (unpadded % kBodyAlign)) % kBodyAlign);
  l.ok = true;
  return l;
}

// The section body of `t` with layout `l`, in order, to `sink`.
template <class Sink>
void emit_body(ParamTable t, const Layout& l, Sink&& sink) noexcept {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    const ParamRecord& r = t[i];
    std::array<std::uint8_t, kRecordBytes> rb{};
    store_le<std::uint32_t>(rb, kRecValue, value_bits(r.value));
    store_le<std::uint32_t>(rb, kRecSigma, std::bit_cast<std::uint32_t>(r.sigma));
    store_le<std::uint32_t>(rb, kRecSource, l.source_off[i]);
    rb[kRecType] = static_cast<std::uint8_t>(r.value.type);
    rb[kRecSigmaKind] = static_cast<std::uint8_t>(r.sigma_kind);
    rb[kRecMethod] = static_cast<std::uint8_t>(r.method);
    rb[kRecOrigin] = static_cast<std::uint8_t>(r.origin);
    rb[kRecFlags] = r.locked ? kFlagLocked : 0;
    sink(Bytes{rb});
  }
  std::size_t next = 0;
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (l.source_off[i] == next) {
      const std::size_t n = std::strlen(t[i].source) + 1;
      sink(Bytes{reinterpret_cast<const std::uint8_t*>(t[i].source), n});
      next += n;
    }
  }
  const std::array<std::uint8_t, kBodyAlign> zeros{};
  const std::size_t written = (kParamCount * kRecordBytes) + next;
  sink(Bytes{zeros}.first(l.body_bytes - written));
}

}  // namespace

const char* fault_text(Fault f) noexcept {
  switch (f) {
    case Fault::None:
      return "none";
    case Fault::Truncated:
      return "the file is truncated or longer than its header says";
    case Fault::Magic:
      return "the file does not start with MARVMCFG";
    case Fault::Header:
      return "a header field is not as the format says";
    case Fault::PayloadHash:
      return "payload SHA-256 mismatch: the file is corrupt";
    case Fault::Directory:
      return "the section directory is not as the format says";
    case Fault::SlotHash:
      return "the slot's content does not match its commit record (SHA-256): the slot is corrupt";
    case Fault::Version:
      return "a format version this image does not know";
    case Fault::NoSection:
      return "the file has no section of this image's kind";
    case Fault::Schema:
      return "the section's schema hash is not this image's: the config needs a migration";
    case Fault::RecordCount:
      return "the section's record count is not this image's parameter count";
    case Fault::Record:
      return "a record is not as the format says";
    case Fault::Type:
      return "a record's type differs from the image's";
    case Fault::NotFinite:
      return "an f32 value is not finite";
    case Fault::Sigma:
      return "a record's sigma does not match its sigma kind (I1)";
    case Fault::Lock:
      return "a locked parameter differs from its compiled value or provenance";
    case Fault::Validator:
      return "the composition's validator rejects the config";
    case Fault::NotBound:
      return "the config is not bound to this image's hash and a validation report";
    case Fault::TooLarge:
      return "the file is larger than the working memory or a slot";
    case Fault::NoMemory:
      return "no working memory for a config file on this image";
    case Fault::PendingRead:
      return "the pending activation file could not be read";
    case Fault::WriteFailed:
      return "a write to the config storage failed";
    case Fault::ReadBack:
      return "the written slot does not read back as written (SHA-256)";
    case Fault::Unmatched:
      return "the boot request matches no pending file, stored config or the factory config";
    case Fault::NoLastGood:
      return "there is no last good stored config";
    case Fault::Count:
      break;
  }
  return "unknown fault";
}

arming::ConfigStatus fault_status(Fault f) noexcept {
  switch (f) {
    case Fault::None:
      return arming::ConfigStatus::Ok;
    case Fault::Version:
    case Fault::NoSection:
    case Fault::Schema:
    case Fault::RecordCount:
      return arming::ConfigStatus::SchemaMismatch;
    case Fault::Record:
    case Fault::Type:
    case Fault::NotFinite:
    case Fault::Sigma:
    case Fault::Lock:
    case Fault::Validator:
      return arming::ConfigStatus::Invalid;
    case Fault::NotBound:
      return arming::ConfigStatus::NotValidated;
    default:
      return arming::ConfigStatus::Corrupt;
  }
}

bool bound_to(const Binding& b, const Digest& image) noexcept {
  return !all_zero(image) && b.image_sha256 == image && !all_zero(b.report_id);
}

Decoded decode(std::span<const std::uint8_t> file, ParamTable compiled, std::span<ParamRecord> out,
               Validator validate) noexcept {
  Decoded d;
  Section s;
  d.fault = structure(file, d, s);
  if (d.fault != Fault::None) {
    return d;
  }
  if (compiled.size() != kParamCount || out.size() != kParamCount) {
    d.fault = Fault::RecordCount;
    return d;
  }
  d.fault = records(s, compiled, out);
  if (d.fault != Fault::None) {
    return d;
  }
  if (validate != nullptr) {
    d.validator_reason = validate(ParamTable{out});
    if (d.validator_reason != nullptr) {
      d.fault = Fault::Validator;
    }
  }
  return d;
}

Decoded section_hash(std::span<const std::uint8_t> file) noexcept {
  Decoded d;
  Section s;
  d.fault = structure(file, d, s);
  return d;
}

std::size_t encoded_size(ParamTable table) noexcept {
  const Layout l = layout(table);
  return l.ok ? kHeaderBytes + kDirEntryBytes + l.body_bytes : 0;
}

std::size_t encode(ParamTable table, std::uint32_t sequence, const Binding& binding,
                   std::span<std::uint8_t> out) noexcept {
  const Layout l = layout(table);
  if (!l.ok) {
    return 0;
  }
  const std::size_t payload = kDirEntryBytes + l.body_bytes;
  const std::size_t size = kHeaderBytes + payload;
  if (out.size() < size) {
    return 0;
  }
  const std::span<std::uint8_t> file = out.first(size);
  std::fill(file.begin(), file.end(), std::uint8_t{0});
  std::copy(kMagic.begin(), kMagic.end(), file.begin());
  store_le<std::uint16_t>(file, kOffVersion, kFormatVersion);
  store_le<std::uint16_t>(file, kOffHeaderSize, static_cast<std::uint16_t>(kHeaderBytes));
  store_le<std::uint32_t>(file, kOffSequence, sequence);
  store_le<std::uint16_t>(file, kOffSectionCount, 1);
  store_le<std::uint32_t>(file, kOffPayloadSize, static_cast<std::uint32_t>(payload));
  std::copy(binding.image_sha256.begin(), binding.image_sha256.end(), file.begin() + kOffImage);
  std::copy(binding.report_id.begin(), binding.report_id.end(), file.begin() + kOffReport);
  const std::span<std::uint8_t> entry = file.subspan(kHeaderBytes, kDirEntryBytes);
  entry[kDirKind] = kImageKind;
  store_le<std::uint64_t>(entry, kDirSchema, kParamSchemaHash);
  store_le<std::uint32_t>(entry, kDirCount, static_cast<std::uint32_t>(kParamCount));
  store_le<std::uint32_t>(entry, kDirOffset, static_cast<std::uint32_t>(kDirEntryBytes));
  store_le<std::uint32_t>(entry, kDirLength, static_cast<std::uint32_t>(l.body_bytes));
  std::size_t pos = kHeaderBytes + kDirEntryBytes;
  emit_body(table, l, [&](Bytes b) {
    std::copy(b.begin(), b.end(), file.begin() + static_cast<std::ptrdiff_t>(pos));
    pos += b.size();
  });
  const Digest h = sha256(file.subspan(kHeaderBytes));
  std::copy(h.begin(), h.end(), file.begin() + kOffPayloadHash);
  return size;
}

Digest config_hash(ParamTable table) noexcept {
  const Layout l = layout(table);
  if (!l.ok) {
    return Digest{};
  }
  Sha256 h;
  h.update(hash_entry(kParamSchemaHash, static_cast<std::uint32_t>(kParamCount), l.body_bytes));
  emit_body(table, l, [&](Bytes b) { h.update(b); });
  return h.finish();
}

}  // namespace marv::store
