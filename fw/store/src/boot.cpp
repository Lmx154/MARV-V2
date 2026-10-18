#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <marv/hal/hal.hpp>
#include <marv/hal/storage.hpp>
#include <marv/params/param_ids.hpp>

#include "internal.hpp"
#include "marv/store/store.hpp"

namespace marv::store {
namespace {

using Bytes = std::span<const std::uint8_t>;

bool g_booted = false;
BootInputs g_in{};
BootReport g_report{};
// The decoded table of a pending file being verified, then of the slot this boot uses (its sources point into the slot).
std::array<ParamRecord, kParamCount> g_table{};

[[nodiscard]] ConfigSlot slot_at(std::size_t i) noexcept { return static_cast<ConfigSlot>(i); }

[[nodiscard]] std::size_t other(std::size_t i) noexcept { return kConfigSlots - 1 - i; }

[[nodiscard]] Digest digest_at(Bytes b, std::size_t offset) noexcept {
  Digest d{};
  std::copy_n(b.begin() + static_cast<std::ptrdiff_t>(offset), d.size(), d.begin());
  return d;
}

[[nodiscard]] detail::SlotView view_slot(ConfigSlot s) noexcept {
  detail::SlotView v;
  const Bytes region = hal_config_slot(s);
  if (region.size() < kCommitBytes + kHeaderBytes) {
    return v;
  }
  const Bytes rec = region.last(kCommitBytes);
  if (!std::equal(kCommitMagic.begin(), kCommitMagic.end(), rec.begin())) {
    return v;
  }
  v.committed = true;
  v.sequence = load_le<std::uint32_t>(rec, kCommitSequence);
  const std::size_t n = load_le<std::uint32_t>(rec, kCommitLength);
  if (n < kHeaderBytes || n > region.size() - kCommitBytes) {
    return v;
  }
  const Bytes file = region.first(n);
  if (sha256(file) != digest_at(rec, kCommitHash) || load_le<std::uint32_t>(file, kOffSequence) != v.sequence) {
    return v;
  }
  v.intact = true;
  v.file = file;
  return v;
}

// The slot a new config goes into: an uncommitted one, else one that is not intact, else the older one.
[[nodiscard]] std::size_t write_target(const detail::SlotViews& v) noexcept {
  const auto rank = [](const detail::SlotView& s) { return s.intact ? 2 : (s.committed ? 1 : 0); };
  std::size_t best = 0;
  for (std::size_t i = 1; i < kConfigSlots; ++i) {
    const bool lower = rank(v[i]) < rank(v[best]);
    const bool older = rank(v[i]) == rank(v[best]) && v[i].intact && v[i].sequence < v[best].sequence;
    if (lower || older) {
      best = i;
    }
  }
  return best;
}

[[nodiscard]] std::uint32_t next_sequence(const detail::SlotViews& v) noexcept {
  std::uint32_t top = 0;
  for (const detail::SlotView& s : v) {
    if (s.committed) {
      top = std::max(top, s.sequence);
    }
  }
  return top + 1;
}

// Writes `file` (verified) into `slot` with `sequence`: the file, a read-back of its SHA-256, then the commit record and
// its read-back. On a failure after the erase the slot is erased again, so it is never left committed to unverified bytes.
[[nodiscard]] Fault write_slot(ConfigSlot slot, std::span<std::uint8_t> file, std::uint32_t sequence) noexcept {
  if (file.size() + kCommitBytes > kConfigSlotBytes) {
    return Fault::TooLarge;
  }
  store_le<std::uint32_t>(file, kOffSequence, sequence);
  const CommitRecord rec = commit_record(sequence, file);
  const auto fail = [slot](Fault f) {
    static_cast<void>(hal_config_slot_write(slot, {}));
    return f;
  };
  if (!hal_config_slot_write(slot, file)) {
    return fail(Fault::WriteFailed);
  }
  Bytes region = hal_config_slot(slot);
  if (region.size() < file.size() + kCommitBytes || sha256(region.first(file.size())) != digest_at(rec, kCommitHash)) {
    return fail(Fault::ReadBack);
  }
  if (!hal_config_slot_commit(slot, rec)) {
    return fail(Fault::WriteFailed);
  }
  region = hal_config_slot(slot);
  if (region.size() < kCommitBytes || !std::equal(rec.begin(), rec.end(), region.last(kCommitBytes).begin())) {
    return fail(Fault::ReadBack);
  }
  return Fault::None;
}

// The full verification of an activation: the file, then the binding to this image.
[[nodiscard]] Fault verify_for_activation(Bytes file) noexcept {
  const Decoded d = decode(file, g_in.factory, g_table, g_in.hooks.validate);
  if (d.fault != Fault::None) {
    return d.fault;
  }
  return bound_to(d.binding, g_in.image_sha256) ? Fault::None : Fault::NotBound;
}

// Writes the verified pending file into the slot that holds the least.
[[nodiscard]] Fault apply_pending(std::span<std::uint8_t> file) noexcept {
  const Fault f = verify_for_activation(file);
  if (f != Fault::None) {
    return f;
  }
  const detail::SlotViews v = detail::view_slots();
  return write_slot(slot_at(write_target(v)), file, next_sequence(v));
}

// Copies the intact slot `source` into the other slot as the newest config (GS-11: returning to the last good file).
[[nodiscard]] Fault reactivate(std::size_t source, const detail::SlotViews& v) noexcept {
  const Bytes file = v[source].file;
  const Fault f = verify_for_activation(file);
  if (f != Fault::None) {
    return f;
  }
  if (g_in.scratch.empty()) {
    return Fault::NoMemory;
  }
  if (g_in.scratch.size() < file.size()) {
    return Fault::TooLarge;
  }
  std::copy(file.begin(), file.end(), g_in.scratch.begin());
  return write_slot(slot_at(other(source)), g_in.scratch.first(file.size()), next_sequence(v));
}

// Step 1 of the boot: the boot request `want` matched against the pending file, the slots and the factory config.
void process_request(const Digest& want) noexcept {
  Fault pending_fault = Fault::None;
  if (!g_in.scratch.empty()) {
    const PendingFile p = hal_config_pending_read(g_in.scratch);
    if (p.status == PendingRead::Ok) {
      const std::span<std::uint8_t> file = g_in.scratch.first(p.bytes);
      const Decoded h = section_hash(file);
      if (h.fault == Fault::None && h.config_hash == want) {
        g_report.request = BootRequest::Activate;
        g_report.request_fault = apply_pending(file);
        static_cast<void>(hal_config_pending_remove());
        return;
      }
    } else if (p.status == PendingRead::TooLarge) {
      pending_fault = Fault::TooLarge;
    } else if (p.status == PendingRead::Error) {
      pending_fault = Fault::PendingRead;
    }
  }
  const detail::SlotViews v = detail::view_slots();
  const std::size_t top = detail::newest(v);
  for (std::size_t i = 0; i < kConfigSlots; ++i) {
    if (!v[i].intact) {
      continue;
    }
    const Decoded h = section_hash(v[i].file);
    if (h.fault == Fault::None && h.config_hash == want) {
      g_report.request = BootRequest::Revert;
      g_report.request_fault = i == top ? Fault::None : reactivate(i, v);
      return;
    }
  }
  if (config_hash(g_in.factory) == want) {
    g_report.request = BootRequest::FactoryReset;
    for (const std::size_t i : {top, other(top)}) {
      if (i < kConfigSlots && v[i].committed && !hal_config_slot_write(slot_at(i), {})) {
        g_report.request_fault = Fault::WriteFailed;
      }
    }
    return;
  }
  g_report.request = BootRequest::Unmatched;
  g_report.request_fault = pending_fault != Fault::None ? pending_fault : Fault::Unmatched;
}

// Step 2 of the boot: the boot policy on the newest committed slot.
[[nodiscard]] ParamTable choose() noexcept {
  const detail::SlotViews v = detail::view_slots();
  const std::size_t top = detail::newest(v);
  if (top >= kConfigSlots) {
    return g_in.factory;
  }
  g_report.slot = slot_at(top);
  if (!v[top].intact) {
    g_report.fault = Fault::SlotHash;
    g_report.status = fault_status(g_report.fault);
    return g_in.factory;
  }
  const Decoded d = decode(v[top].file, g_in.factory, g_table, g_in.hooks.validate);
  if (d.fault != Fault::None) {
    g_report.fault = d.fault;
    g_report.validator_reason = d.validator_reason;
    g_report.status = fault_status(d.fault);
    return g_in.factory;
  }
  g_report.source = ConfigSource::Slot;
  g_report.sequence = d.sequence;
  g_report.binding = d.binding;
  g_report.config_hash = d.config_hash;
  if (!bound_to(d.binding, g_in.image_sha256)) {
    g_report.fault = Fault::NotBound;
    g_report.status = fault_status(g_report.fault);
  }
  return ParamTable{g_table};
}

}  // namespace

namespace detail {

SlotViews view_slots() noexcept {
  SlotViews v{};
  for (std::size_t i = 0; i < kConfigSlots; ++i) {
    v[i] = view_slot(slot_at(i));
  }
  return v;
}

std::size_t newest(const SlotViews& v) noexcept {
  std::size_t top = kConfigSlots;
  for (std::size_t i = 0; i < kConfigSlots; ++i) {
    if (v[i].committed && (top == kConfigSlots || v[i].sequence > v[top].sequence)) {
      top = i;
    }
  }
  return top;
}

const BootInputs* booted_inputs() noexcept { return g_booted ? &g_in : nullptr; }

}  // namespace detail

CommitRecord commit_record(std::uint32_t sequence, std::span<const std::uint8_t> file) noexcept {
  CommitRecord rec{};
  std::copy(kCommitMagic.begin(), kCommitMagic.end(), rec.begin());
  store_le<std::uint32_t>(rec, kCommitSequence, sequence);
  store_le<std::uint32_t>(rec, kCommitLength, static_cast<std::uint32_t>(file.size()));
  const Digest h = sha256(file);
  std::copy(h.begin(), h.end(), rec.begin() + kCommitHash);
  return rec;
}

ParamTable boot(const BootInputs& in) noexcept {
  if (g_booted) {
    hal_panic("store::boot called twice");
  }
  if (in.factory.size() != kParamCount) {
    hal_panic("store::boot: the factory table is not this image's parameter set");
  }
  g_booted = true;
  g_in = in;
  g_report = BootReport{};
  ConfigHashBytes want{};
  if (hal_config_request_take(want)) {
    process_request(want);
  }
  return choose();
}

const BootReport& boot_report() noexcept { return g_report; }

}  // namespace marv::store
