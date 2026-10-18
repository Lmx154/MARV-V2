// The config store (decision 0053, ground-segment unit 4b; ground spec GS-4, GS-11): the boot step that applies a requested
// activation and chooses the table by the boot policy, the staged copy that the link edits while disarmed, and the
// commands that ask the next boot for a config (activate, revert to the last good stored config, reset to factory).
//
// Boot (once, before params_init, before the IMU, receiver and motor outputs start; invariant P-1: the caller then calls
// params_init exactly once with the returned table):
//   1. The boot request (hal_config_request_take, the config hash the previous run asked for), if any, is matched in
//      this order: the pending file (SD card), a stored slot, the factory config. A pending file is verified (SHA-256,
//      schema, records, locks, validator, bound to this image) and written into the slot that does not hold the newest
//      intact config, with sequence = newest + 1, read back (SHA-256) and only then committed; a slot is re-activated
//      the same way (revert); the factory config erases both slots (reset). Any failure writes nothing it has not
//      verified and is reported (BootReport::request_fault). Without a request the SD card is not touched.
//   2. The boot policy (ground segment, decision 0053 Q2), on the committed slot with the larger sequence:
//        no committed slot                                   -> factory table, Ok
//        verifies, schema, records, locks, validator, bound  -> slot table, Ok
//        the same but unbound or bound to another image      -> slot table, NotValidated
//        fails its SHA-256 (or the format)                   -> factory table, Corrupt
//        another schema                                      -> factory table, SchemaMismatch
//        records, locks or validator fail                    -> factory table, Invalid
//      A slot without its commit record (a torn write) is not a candidate. The status goes to
//      composition::set_config_status before composition::init; anything but Ok refuses arming (ConfigRefused).
//
// Slot layout (kConfigSlotBytes, marv/hal/storage.hpp): the .mcfg file from offset 0; the commit record in the last
// kCommitBytes: "MARVSLOT", sequence u32, file length u32, SHA-256 of the file (32 bytes), little-endian. A slot is
// committed iff its commit record starts with the magic; it is intact iff the length fits, the file's SHA-256 equals the
// record's and the file's header sequence equals the record's.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/arming/arming.hpp"
#include "marv/hal/storage.hpp"
#include "marv/params/param_types.hpp"
#include "marv/store/mcfg.hpp"

namespace marv::store {

inline constexpr std::array<std::uint8_t, sizeof(std::uint64_t)> kCommitMagic{'M', 'A', 'R', 'V', 'S', 'L', 'O', 'T'};
inline constexpr std::size_t kCommitSequence = kCommitMagic.size();
inline constexpr std::size_t kCommitLength = kCommitSequence + sizeof(std::uint32_t);
inline constexpr std::size_t kCommitHash = kCommitLength + sizeof(std::uint32_t);
inline constexpr std::size_t kCommitBytes = kCommitHash + kDigestBytes;
using CommitRecord = std::array<std::uint8_t, kCommitBytes>;

// The commit record of `file` written with `sequence`.
[[nodiscard]] CommitRecord commit_record(std::uint32_t sequence, std::span<const std::uint8_t> file) noexcept;

// What the store needs from the composition (quad_fc: composition::validate, arming_status, request_maintenance,
// maintenance_granted).
struct Hooks {
  Validator validate = nullptr;
  bool (*armed)() noexcept = nullptr;
  void (*request_maintenance)() noexcept = nullptr;
  bool (*maintenance_granted)() noexcept = nullptr;
};

struct BootInputs {
  ParamTable factory;   // the compiled table (kParamCount records, static storage): the types, units and locks too
  Digest image_sha256{};  // the running image's SHA-256; all zero = unknown, so no stored config is bound to it
  Hooks hooks;
  std::span<std::uint8_t> scratch;  // static working memory for a pending file or a slot copy, kept for the commands;
                                    // empty = none (nothing can be applied or activated)
};

enum class ConfigSource : std::uint8_t { Factory, Slot };
enum class BootRequest : std::uint8_t { None, Activate, Revert, FactoryReset, Unmatched };

struct BootReport {
  ConfigSource source = ConfigSource::Factory;
  ConfigSlot slot = ConfigSlot::A;  // with source Slot
  arming::ConfigStatus status = arming::ConfigStatus::Ok;
  Fault fault = Fault::None;                // why the newest committed slot is refused (status not Ok)
  const char* validator_reason = nullptr;   // with fault Validator
  std::uint32_t sequence = 0;               // with source Slot
  Binding binding{};                        // with source Slot
  Digest config_hash{};                     // with source Slot
  BootRequest request = BootRequest::None;  // the boot request this boot processed
  Fault request_fault = Fault::None;        // None: done; else why nothing (or nothing more) was written
};

// The boot step (above). Call once, before params_init; a second call is a hal_panic. Returns the table to load: the
// factory span or the store's decoded slot table (static storage).
[[nodiscard]] ParamTable boot(const BootInputs& in) noexcept;

// What boot found and did; default-constructed before boot.
[[nodiscard]] const BootReport& boot_report() noexcept;

// Staging (disarmed only; the staged copy is RAM, nothing is live, everything takes effect at the next boot). Call after
// params_init: the staged copy starts as the active table (param_active_table). A set is refused, changing nothing, unless
// (checked in this order) the vehicle is disarmed, the index is a parameter, the parameter is not locked, the type is the
// parameter's, an f32 value is finite, origin and method are values of their enumerations, sigma kind and sigma satisfy
// I1, a stated source is non-empty text without a zero byte and fits the source storage, and the composition's validator
// accepts the staged table with the new value.
enum class StageRefusal : std::uint8_t {
  None,
  NotBooted,
  Armed,
  Index,
  Locked,
  Type,
  NotFinite,
  Provenance,
  Sigma,
  Source,
  SourceFull,
  Invalid,
};

struct StageResult {
  StageRefusal refusal = StageRefusal::None;
  const char* reason = nullptr;  // null when accepted; the validator's text for Invalid
  [[nodiscard]] bool ok() const noexcept { return refusal == StageRefusal::None; }
};

// The provenance a set states. A set that states none (a plain parameter write on the link) is staged as origin Manual, method Scenario,
// sigma kind Choice, sigma 0 and source kNoSourceStated.
struct StageProvenance {
  ParamOrigin origin = ParamOrigin::Manual;
  ParamMethod method = ParamMethod::Scenario;
  SigmaKind sigma_kind = SigmaKind::Choice;
  float sigma = 0.0F;
  const char* source = nullptr;  // source_bytes bytes, not zero-terminated; copied
  std::size_t source_bytes = 0;
};
inline constexpr const char* kNoSourceStated = "set over the link; no source stated";

[[nodiscard]] StageResult stage_set(std::size_t index, ParamType type, std::uint32_t value_bits,
                                    const StageProvenance* provenance) noexcept;
[[nodiscard]] const char* stage_refusal_text(StageRefusal r) noexcept;

// The staged copy (the active table before any set); empty before boot.
[[nodiscard]] ParamTable staged_table() noexcept;

// True iff a staged record differs from the active one (value, sigma, provenance, source text).
[[nodiscard]] bool staged_differs() noexcept;

// The commands. Each is refused while armed. Accepted, it records the boot request (the config hash the next boot is to
// make active) and asks for the maintenance grant; the reboot happens only once the grant is given (reboot_when_granted;
// the link calls it again while it returns WaitingForGrant).
//   activate: writes the staged copy, bound to `binding` (this image and a report), as the pending file.
//   revert_to_last_good: asks for the intact committed slot that is not the newest (GS-11: an explicit activation).
//   reset_to_factory: asks for the factory config (the next boot erases both slots).
enum class CommandStep : std::uint8_t { Refused, WaitingForGrant, RebootRequested, RebootFailed };
struct CommandResult {
  CommandStep step = CommandStep::Refused;
  const char* reason = nullptr;  // with Refused or RebootFailed
};

[[nodiscard]] CommandResult activate(const Binding& binding) noexcept;
[[nodiscard]] CommandResult revert_to_last_good() noexcept;
[[nodiscard]] CommandResult reset_to_factory() noexcept;
[[nodiscard]] CommandResult reboot_when_granted() noexcept;

}  // namespace marv::store
