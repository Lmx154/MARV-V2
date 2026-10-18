#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <marv/hal/storage.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "internal.hpp"
#include "marv/store/store.hpp"

namespace marv::store {
namespace {

// The staged copy and the texts of the sources stated by sets since boot (each zero-terminated). Background loop only.
std::array<ParamRecord, kParamCount> g_staged{};
bool g_staged_ready = false;
std::array<char, kConfigSlotBytes> g_sources{};
std::size_t g_sources_used = 0;

[[nodiscard]] const BootInputs* staging_inputs() noexcept {
  const BootInputs* in = detail::booted_inputs();
  if (in != nullptr && !g_staged_ready) {
    const ParamTable active = param_active_table();
    std::copy(active.begin(), active.end(), g_staged.begin());
    g_staged_ready = true;
  }
  return in;
}

// With no way to ask, the vehicle counts as armed.
[[nodiscard]] bool armed(const BootInputs& in) noexcept { return in.hooks.armed == nullptr || in.hooks.armed(); }

[[nodiscard]] StageResult refuse(StageRefusal r) noexcept { return StageResult{r, stage_refusal_text(r)}; }

[[nodiscard]] CommandResult refused(const char* reason) noexcept { return CommandResult{CommandStep::Refused, reason}; }

constexpr const char* kCommandArmed = "refused while armed";
constexpr const char* kCommandNotBooted = "the config store has not booted";
constexpr const char* kNoRequest = "this board cannot keep a boot request";
constexpr const char* kNoPending = "the pending activation file could not be written";
constexpr const char* kNoReboot = "this board cannot reboot itself";

// Records the boot request and asks for the maintenance grant.
[[nodiscard]] CommandResult request_boot(const BootInputs& in, const Digest& want) noexcept {
  if (!hal_config_request_set(want)) {
    return refused(kNoRequest);
  }
  if (in.hooks.request_maintenance != nullptr) {
    in.hooks.request_maintenance();
  }
  return reboot_when_granted();
}

}  // namespace

const char* stage_refusal_text(StageRefusal r) noexcept {
  switch (r) {
    case StageRefusal::None:
      return "none";
    case StageRefusal::NotBooted:
      return "the config store has not booted";
    case StageRefusal::Armed:
      return "parameters can be set only while disarmed";
    case StageRefusal::Index:
      return "no parameter has this index";
    case StageRefusal::Locked:
      return "the parameter is locked: it keeps its compiled value";
    case StageRefusal::Type:
      return "the value's type is not the parameter's";
    case StageRefusal::NotFinite:
      return "an f32 value must be finite";
    case StageRefusal::Provenance:
      return "the origin or method is not one the record format knows";
    case StageRefusal::Sigma:
      return "sigma kind and sigma are inconsistent (I1: Known needs a finite sigma > 0, every other kind +0)";
    case StageRefusal::Source:
      return "the source must be non-empty text without a zero byte";
    case StageRefusal::SourceFull:
      return "no room for another source text until the next boot";
    case StageRefusal::Invalid:
      return "the composition's validator rejects the staged config";
  }
  return "unknown refusal";
}

StageResult stage_set(std::size_t index, ParamType type, std::uint32_t value_bits,
                      const StageProvenance* provenance) noexcept {
  const BootInputs* in = staging_inputs();
  if (in == nullptr) {
    return refuse(StageRefusal::NotBooted);
  }
  if (armed(*in)) {
    return refuse(StageRefusal::Armed);
  }
  if (index >= kParamCount) {
    return refuse(StageRefusal::Index);
  }
  const ParamRecord& compiled = in->factory[index];
  if (compiled.locked) {
    return refuse(StageRefusal::Locked);
  }
  if (type != compiled.value.type) {
    return refuse(StageRefusal::Type);
  }
  ParamValue value{type, 0.0F, 0};
  if (type == ParamType::F32) {
    value.f32 = std::bit_cast<float>(value_bits);
    if (!std::isfinite(value.f32)) {
      return refuse(StageRefusal::NotFinite);
    }
  } else {
    value.i32 = std::bit_cast<std::int32_t>(value_bits);
  }
  const StageProvenance p = provenance != nullptr ? *provenance : StageProvenance{};
  if (p.origin > ParamOrigin::Identified || p.method > ParamMethod::Scenario) {
    return refuse(StageRefusal::Provenance);
  }
  if (!sigma_consistent(p.sigma_kind, p.sigma)) {
    return refuse(StageRefusal::Sigma);
  }
  const bool stated = provenance != nullptr;
  if (stated && (p.source == nullptr || p.source_bytes == 0 || std::memchr(p.source, 0, p.source_bytes) != nullptr)) {
    return refuse(StageRefusal::Source);
  }
  if (stated && p.source_bytes + 1 > g_sources.size() - g_sources_used) {
    return refuse(StageRefusal::SourceFull);
  }
  const ParamRecord before = g_staged[index];
  g_staged[index] = ParamRecord{value,       p.sigma,       p.origin, p.method, false,
                                p.sigma_kind, compiled.unit, stated ? before.source : kNoSourceStated};
  const char* reason = in->hooks.validate != nullptr ? in->hooks.validate(ParamTable{g_staged}) : nullptr;
  if (reason != nullptr) {
    g_staged[index] = before;
    return StageResult{StageRefusal::Invalid, reason};
  }
  if (stated) {
    char* text = g_sources.data() + g_sources_used;
    std::copy_n(p.source, p.source_bytes, text);
    text[p.source_bytes] = '\0';
    g_sources_used += p.source_bytes + 1;
    g_staged[index].source = text;
  }
  return StageResult{};
}

ParamTable staged_table() noexcept {
  return staging_inputs() != nullptr ? ParamTable{g_staged} : ParamTable{};
}

bool staged_differs() noexcept {
  if (staging_inputs() == nullptr) {
    return false;
  }
  const ParamTable active = param_active_table();
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (!detail::same_record(g_staged[i], active[i])) {
      return true;
    }
  }
  return false;
}

CommandResult activate(const Binding& binding) noexcept {
  const BootInputs* in = staging_inputs();
  if (in == nullptr) {
    return refused(kCommandNotBooted);
  }
  if (armed(*in)) {
    return refused(kCommandArmed);
  }
  if (!bound_to(binding, in->image_sha256)) {
    return refused(fault_text(Fault::NotBound));
  }
  if (in->scratch.empty()) {
    return refused(fault_text(Fault::NoMemory));
  }
  const std::size_t n = encode(ParamTable{g_staged}, 0, binding, in->scratch);
  if (n == 0 || n + kCommitBytes > kConfigSlotBytes) {
    return refused(fault_text(Fault::TooLarge));
  }
  if (!hal_config_pending_write(in->scratch.first(n))) {
    return refused(kNoPending);
  }
  return request_boot(*in, config_hash(ParamTable{g_staged}));
}

CommandResult revert_to_last_good() noexcept {
  const BootInputs* in = staging_inputs();
  if (in == nullptr) {
    return refused(kCommandNotBooted);
  }
  if (armed(*in)) {
    return refused(kCommandArmed);
  }
  const detail::SlotViews v = detail::view_slots();
  const std::size_t top = detail::newest(v);
  if (top >= kConfigSlots || !v[kConfigSlots - 1 - top].intact) {
    return refused(fault_text(Fault::NoLastGood));
  }
  const Decoded h = section_hash(v[kConfigSlots - 1 - top].file);
  if (h.fault != Fault::None) {
    return refused(fault_text(h.fault));
  }
  return request_boot(*in, h.config_hash);
}

CommandResult reset_to_factory() noexcept {
  const BootInputs* in = staging_inputs();
  if (in == nullptr) {
    return refused(kCommandNotBooted);
  }
  if (armed(*in)) {
    return refused(kCommandArmed);
  }
  return request_boot(*in, config_hash(in->factory));
}

CommandResult reboot_when_granted() noexcept {
  const BootInputs* in = detail::booted_inputs();
  if (in == nullptr) {
    return refused(kCommandNotBooted);
  }
  if (in->hooks.maintenance_granted == nullptr || !in->hooks.maintenance_granted()) {
    if (in->hooks.request_maintenance != nullptr) {
      in->hooks.request_maintenance();
    }
    return CommandResult{CommandStep::WaitingForGrant, nullptr};
  }
  return hal_reboot() ? CommandResult{CommandStep::RebootRequested, nullptr}
                      : CommandResult{CommandStep::RebootFailed, kNoReboot};
}

}  // namespace marv::store
