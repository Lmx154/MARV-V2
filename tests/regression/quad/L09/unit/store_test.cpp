// The config store (fw/store, decision 0053, ground-segment unit 4b) on the host: the SHA-256 known answers, the .mcfg
// reader and writer against the factory file the build writes with tools/config/mcfg.py, the boot policy (invariant B-1: one
// test per row), the boot's apply step, revert and reset to factory, the staging refusals and the commands, and invariant
// P-1 (the active table never changes after params_init).
//
// The store keeps its state in the process (boot is call-once, params_init is call-once), so every test that boots runs in a
// forked child, as in sil_quad_fc_test.cpp: the parent prepares the run directory (hal_sim's file storage: the two slots, the
// pending file, the boot request) through the HAL, the child opens it, boots, and checks. A reboot is the next child on the
// same directory. The parent never boots, so every child starts clean.
//
// Crafted files: a valid file is built with the store's own encode (marv::store::encode) from a copy of the compiled defaults
// with one or two records edited; a file that violates a format rule is that file with bytes edited, the payload hash
// recomputed with the store's own sha256 where the edit must get past it. The commit record of a slot is the store's own
// commit_record, over the original file where a test wants a slot whose content no longer matches it.
//
//  Sha256            FIPS 180-4 examples ("", "abc", the 448-bit and 896-bit messages, one million 'a'), chunked updates, block
//                    boundaries (hashlib), constant evaluation.
//  McfgFormat        the factory file decodes to param_defaults() bit for bit and encode(param_defaults()) is the file's bytes
//                    (tools/config/mcfg.py); the config hash three ways; every format rule refuses with its fault; fault groups.
//  StoreBootPolicy   B-1, one test per row of the boot policy table, plus the torn slot and the no-fallback rule.
//  StoreApply        the boot's apply step: success, every failure leaves nothing written and reports the reason, the torn
//                    write, A/B alternation.
//  StoreRevertReset  revert to the last good config and reset to factory through the boot request.
//  StoreStaging      every refusal of stage_set in order, a failed set changes nothing, the default provenance, staged_differs.
//  StoreCommands     activate, revert_to_last_good, reset_to_factory, reboot_when_granted; the end-to-end activation.
//  StoreInvariants   P-1.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <marv/composition.hpp>
#include <marv/hal/storage.hpp>
#include <marv/hal_sim/storage.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/store/mcfg.hpp>
#include <marv/store/sha256.hpp>
#include <marv/store/store.hpp>

#ifndef MARV_FACTORY_MCFG
#error "MARV_FACTORY_MCFG (the factory .mcfg of marv_params_fc3v2 in the build tree) is set by this directory's CMakeLists.txt"
#endif

namespace marv::store {
void PrintTo(Fault f, std::ostream* os) { *os << static_cast<int>(f) << " (" << fault_text(f) << ")"; }
void PrintTo(StageRefusal r, std::ostream* os) { *os << static_cast<int>(r) << " (" << stage_refusal_text(r) << ")"; }
void PrintTo(BootRequest r, std::ostream* os) { *os << "BootRequest " << static_cast<int>(r); }
void PrintTo(ConfigSource s, std::ostream* os) { *os << "ConfigSource " << static_cast<int>(s); }
void PrintTo(CommandStep s, std::ostream* os) { *os << "CommandStep " << static_cast<int>(s); }
}  // namespace marv::store

namespace marv::arming {
void PrintTo(ConfigStatus s, std::ostream* os) { *os << "ConfigStatus " << static_cast<int>(s); }
}  // namespace marv::arming

namespace {

using namespace marv;
using store::Binding;
using store::Digest;
using store::Fault;
using Bytes = std::vector<std::uint8_t>;
using Table = std::array<ParamRecord, kParamCount>;

constexpr const char* kFactoryMcfg = MARV_FACTORY_MCFG;
constexpr std::uint8_t kErasedByte = 0xFF;

// ---------------------------------------------------------------------------------------------------- the child harness

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

// ---------------------------------------------------------------------------------------------------- small helpers

std::span<const std::uint8_t> as_bytes(std::string_view s) {
  return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

Digest sha_of(std::string_view s) { return store::sha256(as_bytes(s)); }

std::string hex(const Digest& d) {
  std::string out;
  for (const std::uint8_t b : d) {
    char buf[3];
    std::snprintf(buf, sizeof buf, "%02x", b);
    out += buf;
  }
  return out;
}

std::size_t idx(ParamId id) { return static_cast<std::size_t>(id); }

std::uint32_t f32_bits(float f) { return std::bit_cast<std::uint32_t>(f); }
std::uint32_t i32_bits(std::int32_t i) { return std::bit_cast<std::uint32_t>(i); }

Table defaults_table() {
  Table t{};
  std::copy(param_defaults().begin(), param_defaults().end(), t.begin());
  return t;
}

std::uint32_t value_bits(const ParamValue& v) { return v.type == ParamType::F32 ? f32_bits(v.f32) : i32_bits(v.i32); }

bool same_text(const char* a, const char* b) { return a == b || (a != nullptr && b != nullptr && std::strcmp(a, b) == 0); }

// Value, sigma and provenance bit for bit, the source compared as text (strict: also by address).
bool record_same(const ParamRecord& a, const ParamRecord& b, bool strict = false) {
  return a.value.type == b.value.type && value_bits(a.value) == value_bits(b.value) && f32_bits(a.sigma) == f32_bits(b.sigma) &&
         a.origin == b.origin && a.method == b.method && a.locked == b.locked && a.sigma_kind == b.sigma_kind &&
         a.unit == b.unit && same_text(a.source, b.source) && (!strict || a.source == b.source);
}

::testing::AssertionResult tables_equal(ParamTable a, ParamTable b, bool strict = false) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "sizes " << a.size() << " and " << b.size();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!record_same(a[i], b[i], strict)) {
      return ::testing::AssertionFailure() << "record " << i << " (" << param_name(static_cast<ParamId>(i)) << ") differs";
    }
  }
  return ::testing::AssertionSuccess();
}

std::vector<ParamRecord> snapshot(ParamTable t) { return {t.begin(), t.end()}; }

Bytes read_file(const char* path) {
  std::ifstream f(path, std::ios::binary);
  return Bytes{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

bool bytes_equal(std::span<const std::uint8_t> a, std::span<const std::uint8_t> b) {
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

bool all_erased(std::span<const std::uint8_t> s) {
  return std::all_of(s.begin(), s.end(), [](std::uint8_t b) { return b == kErasedByte; });
}

// ---------------------------------------------------------------------------------------------------- crafted configs

const char* const kEditSource = "unit_store test edit";

// The compiled defaults with site_height_m (unlocked f32, read by no check) set to 500 + 100 k m as a Manual edit: a distinct
// valid config for every k.
Table cfg(int k) {
  Table t = defaults_table();
  ParamRecord& r = t[idx(ParamId::site_height_m)];
  r.value.f32 = 500.0F + (100.0F * static_cast<float>(k));
  r.origin = ParamOrigin::Manual;
  r.method = ParamMethod::Scenario;
  r.sigma_kind = SigmaKind::Choice;
  r.sigma = 0.0F;
  r.source = kEditSource;
  return t;
}

// A config the composition's validator rejects (rate_loop_divisor 0) that every format rule accepts.
Table invalid_cfg() {
  Table t = cfg(1);
  ParamRecord& r = t[idx(ParamId::rate_loop_divisor)];
  r.value.i32 = 0;
  r.origin = ParamOrigin::Manual;
  r.source = kEditSource;
  return t;
}

const Digest kImage = sha_of("unit_store image A");
const Digest kOtherImage = sha_of("unit_store image B");
const Digest kReport = sha_of("unit_store validation report");

Binding bound() { return Binding{kImage, kReport}; }
Binding unbound() { return Binding{}; }
Binding bound_to_other_image() { return Binding{kOtherImage, kReport}; }
Binding bound_without_report() { return Binding{kImage, Digest{}}; }

Bytes encode_file(const Table& t, std::uint32_t sequence, const Binding& b) {
  Bytes out(store::encoded_size(t));
  EXPECT_NE(out.size(), 0U);
  EXPECT_EQ(store::encode(t, sequence, b, out), out.size());
  return out;
}

void rehash(Bytes& f) {
  const Digest d = store::sha256(std::span<const std::uint8_t>(f).subspan(store::kHeaderBytes));
  std::copy(d.begin(), d.end(), f.begin() + store::kOffPayloadHash);
}

std::size_t rec_off(ParamId id) { return store::kHeaderBytes + store::kDirEntryBytes + (idx(id) * store::kRecordBytes); }
std::size_t strings_off() { return store::kHeaderBytes + store::kDirEntryBytes + (kParamCount * store::kRecordBytes); }

template <class T>
void put(Bytes& f, std::size_t offset, T v) {
  store::store_le<T>(f, offset, v);
}
template <class T>
T get(const Bytes& f, std::size_t offset) {
  return store::load_le<T>(f, offset);
}

// ---------------------------------------------------------------------------------------------------- the boot's inputs

bool g_armed = false;
bool g_granted = false;
int g_maintenance_requests = 0;
std::array<std::uint8_t, kConfigSlotBytes> g_scratch{};

const char* validate_hook(ParamTable t) noexcept { return composition::validate(t).reason; }
bool armed_hook() noexcept { return g_armed; }
void request_hook() noexcept { ++g_maintenance_requests; }
bool granted_hook() noexcept { return g_granted; }

struct BootOpts {
  bool scratch = true;
  bool known_image = true;  // false: the image hash is all zero (the SIL until unit 6)
  bool armed_hook = true;   // false: Hooks::armed is null
  bool maintenance_hooks = true;  // false: Hooks::request_maintenance and maintenance_granted are null
  hal_sim::StorageFault inject = hal_sim::StorageFault::None;
};

// In a child: opens the run directory and boots the store.
ParamTable boot_store(const std::string& dir, const BootOpts& o = {}) {
  EXPECT_TRUE(hal_sim::storage_open(dir.c_str()));
  hal_sim::storage_inject(o.inject);
  g_armed = false;
  g_granted = false;
  g_maintenance_requests = 0;
  store::BootInputs in;
  in.factory = param_defaults();
  in.image_sha256 = o.known_image ? kImage : Digest{};
  in.hooks = store::Hooks{&validate_hook, o.armed_hook ? &armed_hook : nullptr, o.maintenance_hooks ? &request_hook : nullptr,
                          o.maintenance_hooks ? &granted_hook : nullptr};
  if (o.scratch) {
    in.scratch = g_scratch;
  }
  return store::boot(in);
}

// The one params_init of the process, with the table boot returned.
void init_params(ParamTable t) {
  ASSERT_EQ(t.size(), kParamCount);
  EXPECT_TRUE(params_init(std::span<const ParamRecord, kParamCount>(t.data(), kParamCount)));
}

Bytes slot_copy(ConfigSlot s) {
  const std::span<const std::uint8_t> r = hal_config_slot(s);
  return Bytes(r.begin(), r.end());
}

// A slot holds `t` as written by the boot's apply step: the file with `sequence`, its commit record, the rest erased.
void expect_slot(ConfigSlot s, std::uint32_t sequence, const Table& t, const Binding& b) {
  const Bytes want = encode_file(t, sequence, b);
  const std::span<const std::uint8_t> region = hal_config_slot(s);
  ASSERT_EQ(region.size(), kConfigSlotBytes);
  EXPECT_TRUE(bytes_equal(want, region.first(want.size())));
  const store::CommitRecord rec = store::commit_record(sequence, want);
  EXPECT_TRUE(bytes_equal(rec, region.last(store::kCommitBytes)));
  EXPECT_TRUE(all_erased(region.subspan(want.size(), region.size() - want.size() - store::kCommitBytes)));
}

// ---------------------------------------------------------------------------------------------------- the parent's side

// The run directory as the HAL sees it, to prepare what a child boots on (the parent never boots the store).
class Prep {
 public:
  explicit Prep(const std::string& dir) { EXPECT_TRUE(hal_sim::storage_open(dir.c_str())); }
  ~Prep() { hal_sim::storage_close(); }
  Prep(const Prep&) = delete;
  Prep& operator=(const Prep&) = delete;

  // A committed slot: `file` written, the commit record of `record_of` (the file itself unless a test wants a mismatch).
  void slot(ConfigSlot s, const Bytes& file, std::uint32_t sequence) { slot_with_record(s, file, file, sequence); }
  void slot_with_record(ConfigSlot s, const Bytes& file, const Bytes& record_of, std::uint32_t sequence) {
    EXPECT_TRUE(hal_config_slot_write(s, file));
    EXPECT_TRUE(hal_config_slot_commit(s, store::commit_record(sequence, record_of)));
  }
  // A torn write: the file is there, the commit record was never programmed.
  void torn(ConfigSlot s, const Bytes& file) { EXPECT_TRUE(hal_config_slot_write(s, file)); }
  void erase(ConfigSlot s) { EXPECT_TRUE(hal_config_slot_write(s, {})); }
  void pending(const Bytes& file) { EXPECT_TRUE(hal_config_pending_write(file)); }
  void request(const Digest& hash) { EXPECT_TRUE(hal_config_request_set(hash)); }
};

class StoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string tmpl = (std::filesystem::temp_directory_path() / "marv_unit_store_XXXXXX").string();
    ASSERT_NE(mkdtemp(tmpl.data()), nullptr);
    dir_ = tmpl;
  }
  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }
  std::string dir_;
};

// ====================================================================================================== Sha256

std::string million_a() { return std::string(1000000, 'a'); }

TEST(Sha256, EmptyMessage) {
  EXPECT_EQ(hex(sha_of("")), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256, Abc) {
  EXPECT_EQ(hex(sha_of("abc")), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256, The448BitMessage) {
  EXPECT_EQ(hex(sha_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST(Sha256, The896BitMessage) {
  EXPECT_EQ(hex(sha_of("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu")),
            "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
}

TEST(Sha256, OneMillionA) {
  EXPECT_EQ(hex(sha_of(million_a())), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, ChunkedUpdatesEqualOneShot) {
  const std::string m = million_a();
  for (const std::size_t chunk : {std::size_t{1}, std::size_t{7}, std::size_t{63}, std::size_t{64}, std::size_t{65}, std::size_t{1000}}) {
    store::Sha256 h;
    for (std::size_t at = 0; at < 100000; at += chunk) {
      h.update(as_bytes(m).subspan(at, std::min(chunk, std::size_t{100000} - at)));
    }
    EXPECT_EQ(hex(h.finish()), hex(sha_of(std::string_view(m).substr(0, 100000)))) << "chunk " << chunk;
  }
}

// python3 -c "import hashlib; print(hashlib.sha256(b'a' * N).hexdigest())" for N = 55, 56, 63, 64, 65, 119, 120: the padding
// falls in the same block (55), spills into a second block (56 .. 63), fills a block exactly (64) and so on.
TEST(Sha256, PaddingAtTheBlockBoundaries) {
  struct Vector {
    std::size_t length;
    const char* digest;
  };
  const std::array<Vector, 7> vectors{{
      {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
      {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
      {63, "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34"},
      {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
      {65, "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f17eb0ae0"},
      {119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
      {120, "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c"},
  }};
  for (const Vector& v : vectors) {
    EXPECT_EQ(hex(sha_of(std::string(v.length, 'a'))), v.digest) << "length " << v.length;
  }
}

TEST(Sha256, IsEvaluatedAtCompileTime) {
  constexpr std::array<std::uint8_t, 3> abc{'a', 'b', 'c'};
  constexpr Digest d = store::sha256(abc);
  static_assert(d[0] == 0xba && d[1] == 0x78 && d[31] == 0xad);
  EXPECT_EQ(hex(d), hex(sha_of("abc")));
}

// ====================================================================================================== McfgFormat

TEST(McfgFormat, FactoryFileDecodesToDefaultsBitForBit) {
  const Bytes file = read_file(kFactoryMcfg);
  ASSERT_FALSE(file.empty()) << kFactoryMcfg;
  Table out{};
  const store::Decoded d = store::decode(file, param_defaults(), out, &validate_hook);
  EXPECT_EQ(d.fault, Fault::None);
  EXPECT_EQ(d.validator_reason, nullptr);
  EXPECT_TRUE(tables_equal(out, param_defaults()));
}

TEST(McfgFormat, EncodeOfDefaultsIsByteIdenticalToTheFactoryFile) {
  const Bytes file = read_file(kFactoryMcfg);
  ASSERT_FALSE(file.empty()) << kFactoryMcfg;
  const Bytes mine = encode_file(defaults_table(), 0, Binding{});
  EXPECT_EQ(mine.size(), file.size());
  EXPECT_TRUE(bytes_equal(mine, file));
  EXPECT_EQ(store::encoded_size(param_defaults()), file.size());
}

TEST(McfgFormat, TheConfigHashIsTheSameFromTheStreamTheReaderAndTheSectionCheck) {
  const Bytes file = read_file(kFactoryMcfg);
  Table out{};
  const store::Decoded d = store::decode(file, param_defaults(), out, nullptr);
  ASSERT_EQ(d.fault, Fault::None);
  EXPECT_EQ(store::config_hash(param_defaults()), d.config_hash);
  EXPECT_EQ(store::section_hash(file).fault, Fault::None);
  EXPECT_EQ(store::section_hash(file).config_hash, d.config_hash);
  EXPECT_NE(store::config_hash(cfg(1)), d.config_hash);
  // The hash is of the section, not of the header: sequence and binding do not change it.
  EXPECT_EQ(store::section_hash(encode_file(defaults_table(), 9, bound())).config_hash, d.config_hash);
}

TEST(McfgFormat, AnEditedTableRoundTripsAndTheWriterIsCanonical) {
  const Table t = cfg(3);
  const Bytes file = encode_file(t, 12, bound());
  Table out{};
  const store::Decoded d = store::decode(file, param_defaults(), out, &validate_hook);
  ASSERT_EQ(d.fault, Fault::None);
  EXPECT_TRUE(tables_equal(out, t));
  EXPECT_EQ(d.sequence, 12U);
  EXPECT_EQ(d.binding.image_sha256, kImage);
  EXPECT_EQ(d.binding.report_id, kReport);
  EXPECT_EQ(get<std::uint32_t>(file, store::kOffSequence), 12U);
  EXPECT_TRUE(bytes_equal(encode_file(out, 12, bound()), file));
}

TEST(McfgFormat, TheWriterRefusesWhatItCannotEncode) {
  const Table t = defaults_table();
  Bytes small(store::encoded_size(t) - 1);
  EXPECT_EQ(store::encode(t, 0, Binding{}, small), 0U);
  Table no_source = t;
  no_source[3].source = nullptr;
  EXPECT_EQ(store::encoded_size(no_source), 0U);
  Bytes out(store::encoded_size(t));
  EXPECT_EQ(store::encode(no_source, 0, Binding{}, out), 0U);
  Table empty_source = t;
  empty_source[3].source = "";
  EXPECT_EQ(store::encoded_size(empty_source), 0U);
  EXPECT_EQ(store::encoded_size(ParamTable{t}.first(kParamCount - 1)), 0U);
  EXPECT_EQ(store::config_hash(no_source), Digest{});
}

TEST(McfgFormat, TheDecoderCallsTheValidatorAndReportsItsText) {
  const Bytes file = encode_file(defaults_table(), 0, Binding{});
  Table out{};
  EXPECT_EQ(store::decode(file, param_defaults(), out, nullptr).fault, Fault::None);
  static const char* const kReason = "unit_store validator reason";
  const store::Decoded d = store::decode(file, param_defaults(), out, [](ParamTable) noexcept -> const char* { return kReason; });
  EXPECT_EQ(d.fault, Fault::Validator);
  EXPECT_EQ(d.validator_reason, kReason);
}

TEST(McfgFormat, BoundToNeedsTheImageAndAReport) {
  EXPECT_TRUE(store::bound_to(bound(), kImage));
  EXPECT_FALSE(store::bound_to(bound(), kOtherImage));
  EXPECT_FALSE(store::bound_to(bound_to_other_image(), kImage));
  EXPECT_FALSE(store::bound_to(bound_without_report(), kImage));
  EXPECT_FALSE(store::bound_to(unbound(), kImage));
  // An unknown image (all zero) is never matched, not even by an unbound config.
  EXPECT_FALSE(store::bound_to(unbound(), Digest{}));
  EXPECT_FALSE(store::bound_to(Binding{Digest{}, kReport}, Digest{}));
}

TEST(McfgFormat, TheCommitRecordLayout) {
  const Bytes file = encode_file(cfg(1), 5, bound());
  const store::CommitRecord rec = store::commit_record(5, file);
  EXPECT_TRUE(std::equal(store::kCommitMagic.begin(), store::kCommitMagic.end(), rec.begin()));
  EXPECT_EQ(store::load_le<std::uint32_t>(rec, store::kCommitSequence), 5U);
  EXPECT_EQ(store::load_le<std::uint32_t>(rec, store::kCommitLength), file.size());
  const Digest h = store::sha256(file);
  EXPECT_TRUE(std::equal(h.begin(), h.end(), rec.begin() + store::kCommitHash));
}

struct Refusal {
  const char* name;
  Fault fault;
  void (*edit)(Bytes&);
  bool rehash;  // the edit is in the payload and must get past the payload hash
};

std::size_t first_text_end(const Bytes& f, ParamId id) {
  return strings_off() + get<std::uint32_t>(f, rec_off(id) + store::kRecSource) +
         std::strlen(reinterpret_cast<const char*>(f.data() + strings_off() + get<std::uint32_t>(f, rec_off(id) + store::kRecSource)));
}

// Every rule of the format and the compiled schema, each refused with its own fault. The file is the defaults, sequence 7,
// bound; site_height_m is an unlocked f32, rate_loop_divisor an unlocked i32, rotor_speed_min a locked f32, inertia_xx a Known one.
const std::vector<Refusal>& refusals() {
  static const std::vector<Refusal> r{
      {"shorter than a header", Fault::Truncated, [](Bytes& f) { f.resize(store::kHeaderBytes - 1); }, false},
      {"one byte short", Fault::Truncated, [](Bytes& f) { f.pop_back(); }, false},
      {"one byte long", Fault::Truncated, [](Bytes& f) { f.push_back(0); }, false},
      {"magic", Fault::Magic, [](Bytes& f) { f[0] ^= 1; }, false},
      {"format version 2", Fault::Version, [](Bytes& f) { put<std::uint16_t>(f, store::kOffVersion, 2); }, false},
      {"format version 0", Fault::Version, [](Bytes& f) { put<std::uint16_t>(f, store::kOffVersion, 0); }, false},
      {"header size", Fault::Header, [](Bytes& f) { put<std::uint16_t>(f, store::kOffHeaderSize, store::kHeaderBytes + 1); }, false},
      {"reserved header field", Fault::Header, [](Bytes& f) { put<std::uint16_t>(f, store::kOffHeaderReserved, 1); }, false},
      {"reserved header tail", Fault::Header, [](Bytes& f) { f[store::kOffHeaderTail] = 1; }, false},
      {"payload byte flipped", Fault::PayloadHash, [](Bytes& f) { f[store::kHeaderBytes + store::kDirEntryBytes + 3] ^= 1; }, false},
      {"payload hash flipped", Fault::PayloadHash, [](Bytes& f) { f[store::kOffPayloadHash] ^= 1; }, false},
      {"no section", Fault::Directory, [](Bytes& f) { put<std::uint16_t>(f, store::kOffSectionCount, 0); }, false},
      {"four sections", Fault::Directory, [](Bytes& f) { put<std::uint16_t>(f, store::kOffSectionCount, 4); }, false},
      {"kind out of range", Fault::Directory, [](Bytes& f) { f[store::kHeaderBytes + store::kDirKind] = 3; }, true},
      {"reserved directory byte", Fault::Directory, [](Bytes& f) { f[store::kHeaderBytes + store::kDirKind + 1] = 1; }, true},
      {"section offset", Fault::Directory,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirOffset, store::kDirEntryBytes + 4); }, true},
      {"section length not a multiple of 4", Fault::Directory,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength, get<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength) + 1); },
       true},
      {"section length past the payload", Fault::Directory,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength, get<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength) + 4); },
       true},
      {"payload longer than the sections", Fault::Directory,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength, get<std::uint32_t>(f, store::kHeaderBytes + store::kDirLength) - 4); },
       true},
      {"no quad section", Fault::NoSection, [](Bytes& f) { f[store::kHeaderBytes + store::kDirKind] = 0; }, true},
      {"schema hash", Fault::Schema,
       [](Bytes& f) { put<std::uint64_t>(f, store::kHeaderBytes + store::kDirSchema, get<std::uint64_t>(f, store::kHeaderBytes + store::kDirSchema) ^ 1); },
       true},
      {"record count", Fault::RecordCount,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirCount, static_cast<std::uint32_t>(kParamCount) - 1); }, true},
      {"reserved record byte", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecFlags + 1] = 1; }, true},
      {"record type 2", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecType] = 2; }, true},
      {"sigma kind 4", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecSigmaKind] = 4; }, true},
      {"method 7", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecMethod] = 7; }, true},
      {"origin 4", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecOrigin] = 4; }, true},
      {"flag bit 1", Fault::Record, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecFlags] = 2; }, true},
      {"source offset past the table", Fault::Record,
       [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecSource, 0x00FFFFFF); }, true},
      {"source offset at a terminator", Fault::Record,
       [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecSource, static_cast<std::uint32_t>(first_text_end(f, ParamId::site_height_m) - strings_off())); },
       true},
      {"string table not terminated", Fault::Record, [](Bytes& f) { f.back() = 'x'; }, true},
      {"type differs from the compiled one", Fault::Type, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecType] = 1; }, true},
      {"NaN value", Fault::NotFinite, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecValue, 0x7FC00000U); }, true},
      {"infinite value", Fault::NotFinite, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecValue, 0x7F800000U); }, true},
      {"sigma on a Choice", Fault::Sigma, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecSigma, f32_bits(1.0F)); }, true},
      {"Known with sigma 0", Fault::Sigma, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::inertia_xx) + store::kRecSigma, 0); }, true},
      {"Known with NaN sigma", Fault::Sigma, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::inertia_xx) + store::kRecSigma, 0x7FC00000U); }, true},
      {"locked value", Fault::Lock,
       [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::rotor_speed_min) + store::kRecValue, get<std::uint32_t>(f, rec_off(ParamId::rotor_speed_min) + store::kRecValue) + 1); },
       true},
      {"lock flag cleared", Fault::Lock, [](Bytes& f) { f[rec_off(ParamId::rotor_speed_min) + store::kRecFlags] = 0; }, true},
      {"lock flag set on an unlocked parameter", Fault::Lock, [](Bytes& f) { f[rec_off(ParamId::site_height_m) + store::kRecFlags] = store::kFlagLocked; }, true},
      {"locked provenance", Fault::Lock, [](Bytes& f) { f[rec_off(ParamId::rotor_speed_min) + store::kRecOrigin] = 2; }, true},
      {"locked source text", Fault::Lock,
       [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::rotor_speed_min) + store::kRecSource, get<std::uint32_t>(f, rec_off(ParamId::site_height_m) + store::kRecSource)); },
       true},
      {"validator", Fault::Validator, [](Bytes& f) { put<std::uint32_t>(f, rec_off(ParamId::rate_loop_divisor) + store::kRecValue, 0); }, true},
  };
  return r;
}

TEST(McfgFormat, EveryRuleOfTheFormatRefusesWithItsOwnFault) {
  // The control: the unedited file is accepted by the same call.
  const Bytes good = encode_file(defaults_table(), 7, bound());
  Table out{};
  ASSERT_EQ(store::decode(good, param_defaults(), out, &validate_hook).fault, Fault::None);
  for (const Refusal& r : refusals()) {
    Bytes f = good;
    r.edit(f);
    if (r.rehash) {
      rehash(f);
    }
    const store::Decoded d = store::decode(f, param_defaults(), out, &validate_hook);
    EXPECT_EQ(d.fault, r.fault) << r.name;
    if (r.fault == Fault::Validator) {
      EXPECT_NE(d.validator_reason, nullptr) << r.name;
    }
  }
}

TEST(McfgFormat, TheStructuralCheckAloneAcceptsWhatOnlyTheRecordsRefuse) {
  const Bytes good = encode_file(defaults_table(), 7, bound());
  for (const Refusal& r : refusals()) {
    Bytes f = good;
    r.edit(f);
    if (r.rehash) {
      rehash(f);
    }
    const Fault structural = store::section_hash(f).fault;
    const bool structure_fault = r.fault <= Fault::RecordCount;
    if (structure_fault) {
      EXPECT_EQ(structural, r.fault) << r.name;
    } else {
      EXPECT_EQ(structural, Fault::None) << r.name;
    }
  }
}

// B-1's groups: Corrupt (header, directory, commit SHA), SchemaMismatch (version, no quad section, schema hash, record
// count), Invalid (record, type, finite, I1, lock, validator), NotValidated (unbound).
TEST(McfgFormat, FaultsMapToTheirArmingRefusalGroups) {
  using arming::ConfigStatus;
  for (const Fault f : {Fault::Truncated, Fault::Magic, Fault::Header, Fault::PayloadHash, Fault::Directory, Fault::SlotHash}) {
    EXPECT_EQ(store::fault_status(f), ConfigStatus::Corrupt) << store::fault_text(f);
  }
  for (const Fault f : {Fault::Version, Fault::NoSection, Fault::Schema, Fault::RecordCount}) {
    EXPECT_EQ(store::fault_status(f), ConfigStatus::SchemaMismatch) << store::fault_text(f);
  }
  for (const Fault f : {Fault::Record, Fault::Type, Fault::NotFinite, Fault::Sigma, Fault::Lock, Fault::Validator}) {
    EXPECT_EQ(store::fault_status(f), ConfigStatus::Invalid) << store::fault_text(f);
  }
  EXPECT_EQ(store::fault_status(Fault::NotBound), ConfigStatus::NotValidated);
  EXPECT_EQ(store::fault_status(Fault::None), ConfigStatus::Ok);
}

TEST(McfgFormat, EveryFaultHasItsOwnText) {
  std::vector<std::string> seen;
  for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(Fault::Count); ++i) {
    const char* text = store::fault_text(static_cast<Fault>(i));
    ASSERT_NE(text, nullptr);
    EXPECT_STRNE(text, "unknown fault") << int{i};
    EXPECT_EQ(std::count(seen.begin(), seen.end(), text), 0) << text;
    seen.emplace_back(text);
  }
}

// ====================================================================================================== StoreBootPolicy (B-1)

class StoreBootPolicy : public StoreTest {};

// Row 1: no MARV slot -> factory, allowed.
TEST_F(StoreBootPolicy, NoSlotBootsTheFactoryTableAndArmingIsAllowed) {
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.source, store::ConfigSource::Factory);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.fault, Fault::None);
    EXPECT_EQ(r.request, store::BootRequest::None);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(t.data(), param_defaults().data());
    EXPECT_TRUE(tables_equal(t, param_defaults(), true));
    init_params(t);
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
  });
}

// Row 2: the newest slot verifies, schema, records, locks and validator pass, bound to this image with a report -> slot, allowed.
TEST_F(StoreBootPolicy, NewestSlotVerifiedAndBoundBootsTheSlotAndArmingIsAllowed) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.source, store::ConfigSource::Slot);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.fault, Fault::None);
    EXPECT_EQ(r.sequence, 2U);
    EXPECT_EQ(r.binding.image_sha256, kImage);
    EXPECT_EQ(r.binding.report_id, kReport);
    EXPECT_EQ(r.config_hash, store::config_hash(cfg(2)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    init_params(t);
    EXPECT_TRUE(tables_equal(param_active_table(), cfg(2)));
  });
}

TEST_F(StoreBootPolicy, TheSlotWithTheLargerSequenceIsTheNewestWhicheverSlotItIsIn) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(3), 3, bound()), 3);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::A);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, cfg(3)));
  });
}

// Row 3: the same but unbound, or bound to another image -> slot, ConfigRefused(NotValidated).
TEST_F(StoreBootPolicy, UnboundOrForeignBoundSlotKeepsTheSlotTableAndRefusesNotValidated) {
  struct Case {
    const char* name;
    Binding binding;
    bool known_image;
  };
  const std::vector<Case> cases{
      {"no binding", unbound(), true},
      {"bound to another image", bound_to_other_image(), true},
      {"image without a report", bound_without_report(), true},
      {"report without an image", Binding{Digest{}, kReport}, true},
      {"bound, but this image's hash is unknown (all zero)", bound(), false},
  };
  for (const Case& c : cases) {
    {
      Prep p(dir_);
      p.slot(ConfigSlot::A, encode_file(cfg(1), 1, c.binding), 1);
    }
    in_child([&] {
      BootOpts o;
      o.known_image = c.known_image;
      const ParamTable t = boot_store(dir_, o);
      const store::BootReport& r = store::boot_report();
      EXPECT_EQ(r.source, store::ConfigSource::Slot) << c.name;
      EXPECT_EQ(r.status, arming::ConfigStatus::NotValidated) << c.name;
      EXPECT_EQ(r.fault, Fault::NotBound) << c.name;
      EXPECT_TRUE(tables_equal(t, cfg(1))) << c.name;
    });
  }
}

// Row 4: the newest slot fails its SHA-256 -> factory, ConfigRefused(Corrupt), and no fallback to the older good slot.
TEST_F(StoreBootPolicy, NewestSlotFailingItsShaBootsTheFactoryCorruptAndDoesNotFallBackToTheOlderSlot) {
  const Bytes good = encode_file(cfg(2), 2, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot_with_record(ConfigSlot::B, flipped, good, 2);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.source, store::ConfigSource::Factory);
    EXPECT_EQ(r.status, arming::ConfigStatus::Corrupt);
    EXPECT_EQ(r.fault, Fault::SlotHash);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
    EXPECT_FALSE(tables_equal(t, cfg(1)));
    init_params(t);
    EXPECT_TRUE(tables_equal(param_active_table(), param_defaults()));
  });
}

TEST_F(StoreBootPolicy, AnyBreakOfTheNewestSlotIsCorruptAndTheOlderGoodSlotStaysUnused) {
  const Bytes good = encode_file(cfg(2), 2, bound());
  struct Case {
    const char* name;
    Fault fault;
    Bytes file;
    Bytes record_of;
  };
  Bytes bad_payload = good;
  bad_payload[store::kOffPayloadHash] ^= 1;
  Bytes bad_header_sequence = good;
  put<std::uint32_t>(bad_header_sequence, store::kOffSequence, 9);
  Bytes bad_magic = good;
  bad_magic[0] ^= 1;
  Bytes truncated(good.begin(), good.begin() + store::kHeaderBytes - 1);
  const std::vector<Case> cases{
      {"payload hash wrong (commit record matches the broken file)", Fault::PayloadHash, bad_payload, bad_payload},
      {"header sequence differs from the commit record", Fault::SlotHash, bad_header_sequence, bad_header_sequence},
      {"magic wrong (commit record matches)", Fault::Magic, bad_magic, bad_magic},
      {"committed length shorter than a header", Fault::SlotHash, truncated, truncated},
  };
  for (const Case& c : cases) {
    {
      Prep p(dir_);
      p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
      p.slot_with_record(ConfigSlot::B, c.file, c.record_of, 2);
    }
    in_child([&] {
      const ParamTable t = boot_store(dir_);
      const store::BootReport& r = store::boot_report();
      EXPECT_EQ(r.source, store::ConfigSource::Factory) << c.name;
      EXPECT_EQ(r.status, arming::ConfigStatus::Corrupt) << c.name;
      EXPECT_EQ(r.fault, c.fault) << c.name;
      EXPECT_TRUE(tables_equal(t, param_defaults())) << c.name;
    });
  }
}

TEST_F(StoreBootPolicy, TheOlderSlotCorruptWhileTheNewestIsGoodDoesNotMatter) {
  const Bytes good = encode_file(cfg(1), 1, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot_with_record(ConfigSlot::A, flipped, good, 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

// Row 5: the schema differs from the image -> factory, ConfigRefused(schema mismatch).
TEST_F(StoreBootPolicy, AnotherSchemaBootsTheFactoryAndRefusesSchemaMismatch) {
  const Bytes base = encode_file(cfg(2), 2, bound());
  struct Case {
    const char* name;
    Fault fault;
    void (*edit)(Bytes&);
    bool rehash;
  };
  const std::vector<Case> cases{
      {"format version", Fault::Version, [](Bytes& f) { put<std::uint16_t>(f, store::kOffVersion, 2); }, false},
      {"no quad section", Fault::NoSection, [](Bytes& f) { f[store::kHeaderBytes + store::kDirKind] = 0; }, true},
      {"schema hash", Fault::Schema,
       [](Bytes& f) { put<std::uint64_t>(f, store::kHeaderBytes + store::kDirSchema, get<std::uint64_t>(f, store::kHeaderBytes + store::kDirSchema) ^ 1); },
       true},
      {"record count", Fault::RecordCount,
       [](Bytes& f) { put<std::uint32_t>(f, store::kHeaderBytes + store::kDirCount, static_cast<std::uint32_t>(kParamCount) + 1); }, true},
  };
  for (const Case& c : cases) {
    Bytes f = base;
    c.edit(f);
    if (c.rehash) {
      rehash(f);
    }
    {
      Prep p(dir_);
      p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
      p.slot(ConfigSlot::B, f, 2);
    }
    in_child([&] {
      const ParamTable t = boot_store(dir_);
      const store::BootReport& r = store::boot_report();
      EXPECT_EQ(r.source, store::ConfigSource::Factory) << c.name;
      EXPECT_EQ(r.status, arming::ConfigStatus::SchemaMismatch) << c.name;
      EXPECT_EQ(r.fault, c.fault) << c.name;
      EXPECT_TRUE(tables_equal(t, param_defaults())) << c.name;
    });
  }
}

// Row 6: the validator fails -> factory, ConfigRefused(invalid).
TEST_F(StoreBootPolicy, AConfigTheValidatorRejectsBootsTheFactoryAndRefusesInvalid) {
  const char* const expected = composition::validate(invalid_cfg()).reason;
  ASSERT_NE(expected, nullptr);
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(invalid_cfg(), 1, bound()), 1);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.source, store::ConfigSource::Factory);
    EXPECT_EQ(r.status, arming::ConfigStatus::Invalid);
    EXPECT_EQ(r.fault, Fault::Validator);
    EXPECT_STREQ(r.validator_reason, expected);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
  });
}

TEST_F(StoreBootPolicy, ARecordTheFormatRefusesBootsTheFactoryAndRefusesInvalid) {
  const Bytes base = encode_file(cfg(2), 2, bound());
  for (const Refusal& r : refusals()) {
    if (store::fault_status(r.fault) != arming::ConfigStatus::Invalid || r.fault == Fault::Validator) {
      continue;
    }
    Bytes f = base;
    r.edit(f);
    if (r.rehash) {
      rehash(f);
    }
    {
      Prep p(dir_);
      p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
      p.slot(ConfigSlot::B, f, 2);
    }
    in_child([&] {
      const ParamTable t = boot_store(dir_);
      EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Invalid) << r.name;
      EXPECT_EQ(store::boot_report().source, store::ConfigSource::Factory) << r.name;
      EXPECT_TRUE(tables_equal(t, param_defaults())) << r.name;
    });
  }
}

// A slot without its commit record (a torn write) is not a candidate.
TEST_F(StoreBootPolicy, ATornSlotIsNotACandidateEvenWithTheLargerSequence) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.torn(ConfigSlot::B, encode_file(cfg(2), 5, bound()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.source, store::ConfigSource::Slot);
    EXPECT_EQ(r.slot, ConfigSlot::A);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreBootPolicy, AloneATornSlotLeavesTheFactoryAllowed) {
  {
    Prep p(dir_);
    p.torn(ConfigSlot::A, encode_file(cfg(1), 1, bound()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Factory);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
  });
}

// The SD card is not in the boot path: without a boot request the pending file is neither read nor removed.
TEST_F(StoreBootPolicy, WithoutABootRequestThePendingFileIsNotTouchedAndNothingIsWritten) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.pending(encode_file(cfg(2), 0, bound()));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::None);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
    EXPECT_TRUE(bytes_equal(a, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b, slot_copy(ConfigSlot::B)));
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    EXPECT_EQ(hal_config_pending_read(buf).status, PendingRead::Ok);
  });
}

// ====================================================================================================== StoreApply

class StoreApply : public StoreTest {
 protected:
  // The usual start: slot A holds cfg(1), sequence 1, bound; a pending cfg(2) with the request for it.
  void prepare_pending_over_slot_a(const Table& pending, const Binding& binding, const Digest& want) {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.pending(encode_file(pending, 0, binding));
    p.request(want);
  }

  // The apply step refused: nothing was written, slot A still holds cfg(1) and is the config in use.
  void expect_nothing_written(const Bytes& a_before, const Bytes& b_before, ParamTable table) {
    EXPECT_TRUE(bytes_equal(a_before, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b_before, slot_copy(ConfigSlot::B)));
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Slot);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::A);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(table, cfg(1)));
  }
};

TEST_F(StoreApply, ASuccessfulActivationWritesTheInactiveSlotCommitsItAndTheSameBootUsesIt) {
  prepare_pending_over_slot_a(cfg(2), bound(), store::config_hash(cfg(2)));
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a_before = slot_copy(ConfigSlot::A);
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::Activate);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.source, store::ConfigSource::Slot);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_EQ(r.sequence, 2U);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.config_hash, store::config_hash(cfg(2)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    expect_slot(ConfigSlot::B, 2, cfg(2), bound());
    EXPECT_TRUE(bytes_equal(a_before, slot_copy(ConfigSlot::A)));
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    EXPECT_EQ(hal_config_pending_read(buf).status, PendingRead::None);
    ConfigHashBytes again{};
    EXPECT_FALSE(hal_config_request_take(again));
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::None);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_EQ(store::boot_report().sequence, 2U);
    EXPECT_EQ(store::boot_report().config_hash, store::config_hash(cfg(2)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

TEST_F(StoreApply, FromEmptySlotsTheFirstActivationIsSlotAWithSequenceOne) {
  {
    Prep p(dir_);
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::Activate);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.slot, ConfigSlot::A);
    EXPECT_EQ(r.sequence, 1U);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    expect_slot(ConfigSlot::A, 1, cfg(2), bound());
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
  });
}

TEST_F(StoreApply, ATornSlotIsTheTargetBeforeAnIntactOne) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.torn(ConfigSlot::B, encode_file(cfg(9), 4, bound()));
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    const Bytes a_before = [&] {
      EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
      return slot_copy(ConfigSlot::A);
    }();
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request_fault, Fault::None);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_EQ(store::boot_report().sequence, 2U);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    expect_slot(ConfigSlot::B, 2, cfg(2), bound());
    EXPECT_TRUE(bytes_equal(a_before, slot_copy(ConfigSlot::A)));
  });
}

// A newest slot that fails its SHA-256 is the target of a new activation (GS-11: a new activation leaves the Corrupt refusal).
TEST_F(StoreApply, ANewActivationReplacesACorruptNewestSlotAndLeavesTheCorruptRefusal) {
  const Bytes good = encode_file(cfg(9), 2, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot_with_record(ConfigSlot::B, flipped, good, 2);
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_EQ(r.sequence, 3U);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    expect_slot(ConfigSlot::B, 3, cfg(2), bound());
  });
}

TEST_F(StoreApply, WhenBothSlotsAreIntactTheOlderOneIsOverwritten) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 3, bound()), 3);
    p.slot(ConfigSlot::B, encode_file(cfg(3), 4, bound()), 4);
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::A);
    EXPECT_EQ(store::boot_report().sequence, 5U);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    expect_slot(ConfigSlot::A, 5, cfg(2), bound());
    expect_slot(ConfigSlot::B, 4, cfg(3), bound());
  });
}

// A/B alternation: five activations in a row, a reboot (a new child) between them.
TEST_F(StoreApply, ActivationsAlternateBetweenTheSlotsAndKeepThePreviousConfigIntact) {
  for (std::uint32_t k = 1; k <= 5; ++k) {
    {
      Prep p(dir_);
      p.pending(encode_file(cfg(static_cast<int>(k) + 1), 0, bound()));
      p.request(store::config_hash(cfg(static_cast<int>(k) + 1)));
    }
    in_child([&] {
      const ParamTable t = boot_store(dir_);
      const store::BootReport& r = store::boot_report();
      const ConfigSlot target = (k % 2 == 1) ? ConfigSlot::A : ConfigSlot::B;
      const ConfigSlot other = (k % 2 == 1) ? ConfigSlot::B : ConfigSlot::A;
      EXPECT_EQ(r.request_fault, Fault::None) << k;
      EXPECT_EQ(r.slot, target) << k;
      EXPECT_EQ(r.sequence, k);
      EXPECT_TRUE(tables_equal(t, cfg(static_cast<int>(k) + 1))) << k;
      expect_slot(target, k, cfg(static_cast<int>(k) + 1), bound());
      if (k > 1) {
        expect_slot(other, k - 1, cfg(static_cast<int>(k)), bound());
      } else {
        EXPECT_TRUE(all_erased(hal_config_slot(other)));
      }
    });
  }
}

// Every failure of the apply step: nothing written, the config in use is the one before, the reason reported.
TEST_F(StoreApply, ACorruptPendingFileWritesNothingAndIsReportedUnmatched) {
  const Table staged = cfg(2);
  Bytes pending = encode_file(staged, 0, bound());
  pending[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.pending(pending);
    p.request(store::config_hash(staged));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Unmatched);
    EXPECT_EQ(store::boot_report().request_fault, Fault::Unmatched);
    expect_nothing_written(a, b, t);
  });
}

TEST_F(StoreApply, APendingFileOfAnotherSchemaWritesNothingAndIsReportedUnmatched) {
  Bytes pending = encode_file(cfg(2), 0, bound());
  put<std::uint64_t>(pending, store::kHeaderBytes + store::kDirSchema, get<std::uint64_t>(pending, store::kHeaderBytes + store::kDirSchema) ^ 1);
  rehash(pending);
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.pending(pending);
    p.request(store::section_hash(pending).config_hash);
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request_fault, Fault::Unmatched);
    expect_nothing_written(a, b, t);
  });
}

TEST_F(StoreApply, APendingFileTheValidatorRejectsWritesNothingAndReportsValidator) {
  prepare_pending_over_slot_a(invalid_cfg(), bound(), store::config_hash(invalid_cfg()));
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
    EXPECT_EQ(store::boot_report().request_fault, Fault::Validator);
    expect_nothing_written(a, b, t);
  });
}

TEST_F(StoreApply, APendingFileThatChangesALockedParameterWritesNothingAndReportsLock) {
  Table changed_value = cfg(2);
  changed_value[idx(ParamId::rotor_speed_min)].value.f32 += 1.0F;
  Table cleared_flag = cfg(2);
  cleared_flag[idx(ParamId::rotor_speed_min)].locked = false;
  for (const Table* t : {&changed_value, &cleared_flag}) {
    prepare_pending_over_slot_a(*t, bound(), store::config_hash(*t));
    in_child([&] {
      EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
      const Bytes a = slot_copy(ConfigSlot::A);
      const Bytes b = slot_copy(ConfigSlot::B);
      const ParamTable table = boot_store(dir_);
      EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
      EXPECT_EQ(store::boot_report().request_fault, Fault::Lock);
      expect_nothing_written(a, b, table);
    });
  }
}

TEST_F(StoreApply, APendingFileNotBoundToThisImageAndAReportWritesNothingAndReportsNotBound) {
  for (const Binding& b : {unbound(), bound_to_other_image(), bound_without_report()}) {
    prepare_pending_over_slot_a(cfg(2), b, store::config_hash(cfg(2)));
    in_child([&] {
      EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
      const Bytes a = slot_copy(ConfigSlot::A);
      const Bytes bb = slot_copy(ConfigSlot::B);
      const ParamTable t = boot_store(dir_);
      EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
      EXPECT_EQ(store::boot_report().request_fault, Fault::NotBound);
      expect_nothing_written(a, bb, t);
    });
  }
}

TEST_F(StoreApply, AnUnknownImageHashBindsNothingSoNothingIsApplied) {
  prepare_pending_over_slot_a(cfg(2), bound(), store::config_hash(cfg(2)));
  in_child([&] {
    BootOpts o;
    o.known_image = false;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request_fault, Fault::NotBound);
    EXPECT_FALSE(hal_config_slot(ConfigSlot::B).empty());
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Slot);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::NotValidated);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreApply, AFailingSlotWriteLeavesTheTargetErasedAndReportsWriteFailed) {
  prepare_pending_over_slot_a(cfg(2), bound(), store::config_hash(cfg(2)));
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    BootOpts o;
    o.inject = hal_sim::StorageFault::Fails;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
    EXPECT_EQ(store::boot_report().request_fault, Fault::WriteFailed);
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
    expect_nothing_written(a, slot_copy(ConfigSlot::B), t);
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::A);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreApply, ASlotThatDoesNotReadBackAsWrittenLeavesTheTargetErasedAndReportsReadBack) {
  prepare_pending_over_slot_a(cfg(2), bound(), store::config_hash(cfg(2)));
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    BootOpts o;
    o.inject = hal_sim::StorageFault::ReadBackDiffers;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
    EXPECT_EQ(store::boot_report().request_fault, Fault::ReadBack);
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
    expect_nothing_written(a, slot_copy(ConfigSlot::B), t);
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreApply, AFailedFirstActivationLeavesTheFactoryAllowed) {
  {
    Prep p(dir_);
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    BootOpts o;
    o.inject = hal_sim::StorageFault::Fails;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request_fault, Fault::WriteFailed);
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Factory);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
  });
}

// A write that stopped before the commit record: the next boot ignores the slot and uses what it had.
TEST_F(StoreApply, ATornWriteWithoutItsCommitMarkerIsIgnoredOnTheNextBoot) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    // What an interrupted apply leaves: the new file in B, the commit record never programmed.
    p.torn(ConfigSlot::B, encode_file(cfg(2), 2, bound()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::A);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreApply, ATooLargePendingFileIsReported) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.pending(Bytes(kConfigSlotBytes + 1, 0));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Unmatched);
    EXPECT_EQ(store::boot_report().request_fault, Fault::TooLarge);
    expect_nothing_written(a, b, t);
  });
}

TEST_F(StoreApply, WithoutWorkingMemoryTheRequestCannotBeApplied) {
  prepare_pending_over_slot_a(cfg(2), bound(), store::config_hash(cfg(2)));
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    BootOpts o;
    o.scratch = false;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Unmatched);
    EXPECT_NE(store::boot_report().request_fault, Fault::None);
    expect_nothing_written(a, b, t);
  });
}

// ====================================================================================================== StoreRevertReset

class StoreRevertReset : public StoreTest {
 protected:
  // A holds cfg(1) (sequence 1), B holds cfg(2) (sequence 2), both bound and intact.
  void prepare_two_good_slots() {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
};

TEST_F(StoreRevertReset, RevertCopiesTheLastGoodConfigIntoTheOtherSlotAsTheNewest) {
  prepare_two_good_slots();
  {
    Prep p(dir_);
    p.request(store::config_hash(cfg(1)));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a_before = slot_copy(ConfigSlot::A);
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::Revert);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.source, store::ConfigSource::Slot);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_EQ(r.sequence, 3U);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.config_hash, store::config_hash(cfg(1)));
    EXPECT_TRUE(tables_equal(t, cfg(1)));
    expect_slot(ConfigSlot::B, 3, cfg(1), bound());
    EXPECT_TRUE(bytes_equal(a_before, slot_copy(ConfigSlot::A)));
    ConfigHashBytes again{};
    EXPECT_FALSE(hal_config_request_take(again));
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::None);
    EXPECT_EQ(store::boot_report().sequence, 3U);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreRevertReset, RevertToTheConfigAlreadyNewestWritesNothing) {
  prepare_two_good_slots();
  {
    Prep p(dir_);
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Revert);
    EXPECT_EQ(store::boot_report().request_fault, Fault::None);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_EQ(store::boot_report().sequence, 2U);
    EXPECT_TRUE(bytes_equal(a, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b, slot_copy(ConfigSlot::B)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

// GS-11: a Corrupt refusal ends with an explicit revert, factory reset or new activation.
TEST_F(StoreRevertReset, RevertLeavesTheCorruptRefusalByReturningToTheOlderGoodSlot) {
  const Bytes good = encode_file(cfg(2), 2, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot_with_record(ConfigSlot::B, flipped, good, 2);
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Corrupt);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
  });
  {
    Prep p(dir_);
    p.request(store::config_hash(cfg(1)));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::Revert);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.slot, ConfigSlot::B);
    EXPECT_EQ(r.sequence, 3U);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
    expect_slot(ConfigSlot::B, 3, cfg(1), bound());
  });
}

TEST_F(StoreRevertReset, RevertToAConfigThatIsNotBoundToThisImageWritesNothing) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound_to_other_image()), 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
    p.request(store::config_hash(cfg(1)));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Revert);
    EXPECT_EQ(store::boot_report().request_fault, Fault::NotBound);
    EXPECT_TRUE(bytes_equal(a, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b, slot_copy(ConfigSlot::B)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

TEST_F(StoreRevertReset, RevertWithoutWorkingMemoryWritesNothing) {
  prepare_two_good_slots();
  {
    Prep p(dir_);
    p.request(store::config_hash(cfg(1)));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    BootOpts o;
    o.scratch = false;
    const ParamTable t = boot_store(dir_, o);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Revert);
    EXPECT_EQ(store::boot_report().request_fault, Fault::NoMemory);
    EXPECT_TRUE(bytes_equal(a, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b, slot_copy(ConfigSlot::B)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

TEST_F(StoreRevertReset, ResetToFactoryErasesBothSlotsAndBootsTheFactoryAllowed) {
  prepare_two_good_slots();
  {
    Prep p(dir_);
    p.request(store::config_hash(param_defaults()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::FactoryReset);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.source, store::ConfigSource::Factory);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
    ConfigHashBytes again{};
    EXPECT_FALSE(hal_config_request_take(again));
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::None);
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Factory);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
  });
}

TEST_F(StoreRevertReset, ResetToFactoryLeavesTheCorruptRefusal) {
  const Bytes good = encode_file(cfg(2), 2, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot_with_record(ConfigSlot::B, flipped, good, 2);
    p.request(store::config_hash(param_defaults()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::FactoryReset);
    EXPECT_EQ(store::boot_report().request_fault, Fault::None);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
  });
}

TEST_F(StoreRevertReset, ResetToFactoryWithOneSlotErasesIt) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::B, encode_file(cfg(1), 1, bound()), 1);
    p.request(store::config_hash(param_defaults()));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::FactoryReset);
    EXPECT_EQ(store::boot_report().status, arming::ConfigStatus::Ok);
    EXPECT_TRUE(tables_equal(t, param_defaults()));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
  });
}

TEST_F(StoreRevertReset, ARequestMatchingNothingChangesNothingAndIsReported) {
  prepare_two_good_slots();
  {
    Prep p(dir_);
    p.request(sha_of("no such config"));
  }
  in_child([&] {
    EXPECT_TRUE(hal_sim::storage_open(dir_.c_str()));
    const Bytes a = slot_copy(ConfigSlot::A);
    const Bytes b = slot_copy(ConfigSlot::B);
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Unmatched);
    EXPECT_EQ(store::boot_report().request_fault, Fault::Unmatched);
    EXPECT_TRUE(bytes_equal(a, slot_copy(ConfigSlot::A)));
    EXPECT_TRUE(bytes_equal(b, slot_copy(ConfigSlot::B)));
    EXPECT_TRUE(tables_equal(t, cfg(2)));
    ConfigHashBytes again{};
    EXPECT_FALSE(hal_config_request_take(again));
  });
}

// The request is matched against the pending file first, then the slots, then the factory config.
TEST_F(StoreRevertReset, ThePendingFileIsMatchedBeforeASlotThatHoldsTheSameConfig) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(2), 1, bound()), 1);
    p.pending(encode_file(cfg(2), 0, bound()));
    p.request(store::config_hash(cfg(2)));
  }
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Activate);
    EXPECT_EQ(store::boot_report().request_fault, Fault::None);
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_EQ(store::boot_report().sequence, 2U);
    EXPECT_TRUE(tables_equal(t, cfg(2)));
  });
}

// ====================================================================================================== StoreStaging

class StoreStaging : public StoreTest {
 protected:
  // In a child: boots the store on the (empty) run directory and loads the factory table.
  ParamTable start(const BootOpts& o = {}) {
    const ParamTable t = boot_store(dir_, o);
    init_params(t);
    return t;
  }

  static store::StageProvenance provenance(const char* source, std::size_t bytes) {
    store::StageProvenance p;
    p.origin = ParamOrigin::Identified;
    p.method = ParamMethod::Measured;
    p.sigma_kind = SigmaKind::Known;
    p.sigma = 2.0F;
    p.source = source;
    p.source_bytes = bytes;
    return p;
  }
  static store::StageProvenance provenance(const char* source) { return provenance(source, std::strlen(source)); }

  // The refused set changed nothing: the staged table is as it was, byte for byte and by address, and differs from nothing.
  static void expect_unchanged(const std::vector<ParamRecord>& before) {
    EXPECT_TRUE(tables_equal(store::staged_table(), before, true));
    EXPECT_FALSE(store::staged_differs());
  }
};

TEST_F(StoreStaging, BeforeBootEverythingIsRefusedAsNotBooted) {
  in_child([&] {
    const store::StageResult r = store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1.0F), nullptr);
    EXPECT_EQ(r.refusal, store::StageRefusal::NotBooted);
    EXPECT_STREQ(r.reason, store::stage_refusal_text(store::StageRefusal::NotBooted));
    EXPECT_FALSE(r.ok());
    EXPECT_TRUE(store::staged_table().empty());
    EXPECT_FALSE(store::staged_differs());
    for (const store::CommandResult& c : {store::activate(bound()), store::revert_to_last_good(), store::reset_to_factory(), store::reboot_when_granted()}) {
      EXPECT_EQ(c.step, store::CommandStep::Refused);
      EXPECT_NE(c.reason, nullptr);
    }
    EXPECT_FALSE(hal_sim::reboot_requested());
  });
}

TEST_F(StoreStaging, ArmedRefusesEverySetAndEveryCommandAndWritesNothing) {
  in_child([&] {
    start();
    const std::vector<ParamRecord> before = snapshot(store::staged_table());
    g_armed = true;
    const store::StageResult r = store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1.0F), nullptr);
    EXPECT_EQ(r.refusal, store::StageRefusal::Armed);
    expect_unchanged(before);
    for (const store::CommandResult& c : {store::activate(bound()), store::revert_to_last_good(), store::reset_to_factory()}) {
      EXPECT_EQ(c.step, store::CommandStep::Refused);
      EXPECT_STREQ(c.reason, "refused while armed");
    }
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    EXPECT_EQ(hal_config_pending_read(buf).status, PendingRead::None);
    ConfigHashBytes request{};
    EXPECT_FALSE(hal_config_request_take(request));
    EXPECT_EQ(g_maintenance_requests, 0);
    EXPECT_FALSE(hal_sim::reboot_requested());
    g_armed = false;
    EXPECT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1.0F), nullptr).ok());
  });
}

TEST_F(StoreStaging, WithNoWayToAskTheVehicleCountsAsArmed) {
  in_child([&] {
    BootOpts o;
    o.armed_hook = false;
    start(o);
    EXPECT_EQ(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1.0F), nullptr).refusal, store::StageRefusal::Armed);
    EXPECT_EQ(store::activate(bound()).step, store::CommandStep::Refused);
    EXPECT_EQ(store::reset_to_factory().step, store::CommandStep::Refused);
  });
}

// The documented order: disarmed, index, locked, type, finite, origin and method, I1, source, source storage, validator. Each
// call breaks the named rule and every rule after it, so it must be the named one that answers.
TEST_F(StoreStaging, EveryRefusalAnswersInTheDocumentedOrderAndChangesNothing) {
  in_child([&] {
    start();
    const std::vector<ParamRecord> before = snapshot(store::staged_table());
    const std::size_t site = idx(ParamId::site_height_m);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    // A provenance that breaks origin, sigma kind and source at once.
    store::StageProvenance all_bad = provenance("a");
    all_bad.origin = static_cast<ParamOrigin>(4);
    all_bad.sigma_kind = static_cast<SigmaKind>(4);
    all_bad.source = nullptr;
    store::StageProvenance bad_method = provenance("a");
    bad_method.method = static_cast<ParamMethod>(7);
    store::StageProvenance known_sigma_zero = provenance("");  // also an empty source
    known_sigma_zero.sigma = 0.0F;
    store::StageProvenance choice_with_sigma = provenance("a");
    choice_with_sigma.sigma_kind = SigmaKind::Choice;
    store::StageProvenance bad_kind = provenance("");
    bad_kind.sigma_kind = static_cast<SigmaKind>(4);
    store::StageProvenance null_source = provenance("a");
    null_source.source = nullptr;
    store::StageProvenance empty_source = provenance("a");
    empty_source.source_bytes = 0;
    const std::array<char, 3> zero_inside{'a', '\0', 'b'};
    const store::StageProvenance zero_in_text = provenance(zero_inside.data(), zero_inside.size());
    std::string zero_and_huge(kConfigSlotBytes + 1, 'x');  // a zero byte in a text too long for the source storage
    zero_and_huge[10] = '\0';
    const store::StageProvenance zero_in_huge_text = provenance(zero_and_huge.data(), zero_and_huge.size());

    struct Case {
      const char* name;
      std::size_t index;
      ParamType type;
      std::uint32_t bits;
      store::StageProvenance prov;
      store::StageRefusal expect;
      bool armed;
    };
    const std::vector<Case> cases{
        {"armed beats a bad index", kParamCount, ParamType::F32, f32_bits(nan), all_bad, store::StageRefusal::Armed, true},
        {"index beats locked and type", kParamCount, ParamType::I32, f32_bits(nan), all_bad, store::StageRefusal::Index, false},
        {"locked beats type", idx(ParamId::rotor_speed_min), ParamType::I32, f32_bits(nan), all_bad, store::StageRefusal::Locked, false},
        {"locked i32", idx(ParamId::rc_channel_arm), ParamType::I32, i32_bits(5), provenance("a"), store::StageRefusal::Locked, false},
        {"type beats finite", site, ParamType::I32, f32_bits(nan), all_bad, store::StageRefusal::Type, false},
        {"NaN beats provenance", site, ParamType::F32, f32_bits(nan), all_bad, store::StageRefusal::NotFinite, false},
        {"infinity", site, ParamType::F32, f32_bits(inf), provenance("a"), store::StageRefusal::NotFinite, false},
        {"origin beats sigma and source", site, ParamType::F32, f32_bits(1.0F), all_bad, store::StageRefusal::Provenance, false},
        {"method", site, ParamType::F32, f32_bits(1.0F), bad_method, store::StageRefusal::Provenance, false},
        {"Known with sigma 0 beats the source", site, ParamType::F32, f32_bits(1.0F), known_sigma_zero, store::StageRefusal::Sigma, false},
        {"sigma on a Choice", site, ParamType::F32, f32_bits(1.0F), choice_with_sigma, store::StageRefusal::Sigma, false},
        {"unknown sigma kind", site, ParamType::F32, f32_bits(1.0F), bad_kind, store::StageRefusal::Sigma, false},
        {"null source", site, ParamType::F32, f32_bits(1.0F), null_source, store::StageRefusal::Source, false},
        {"empty source", site, ParamType::F32, f32_bits(1.0F), empty_source, store::StageRefusal::Source, false},
        {"a zero byte in the source", site, ParamType::F32, f32_bits(1.0F), zero_in_text, store::StageRefusal::Source, false},
        {"the source is checked before it is sized", site, ParamType::F32, f32_bits(1.0F), zero_in_huge_text, store::StageRefusal::Source, false},
    };
    for (const Case& c : cases) {
      g_armed = c.armed;
      const store::StageResult r = store::stage_set(c.index, c.type, c.bits, &c.prov);
      EXPECT_EQ(r.refusal, c.expect) << c.name;
      EXPECT_STREQ(r.reason, store::stage_refusal_text(c.expect)) << c.name;
      EXPECT_FALSE(r.ok()) << c.name;
      expect_unchanged(before);
    }
    g_armed = false;
    // The control: with nothing broken the same call is accepted.
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(1.0F), nullptr).ok());
  });
}

TEST_F(StoreStaging, TheValidatorRefusalCarriesItsReasonAndChangesNothing) {
  in_child([&] {
    start();
    const std::vector<ParamRecord> before = snapshot(store::staged_table());
    const store::StageProvenance p = provenance("a source");
    const store::StageResult r = store::stage_set(idx(ParamId::rate_loop_divisor), ParamType::I32, i32_bits(0), &p);
    EXPECT_EQ(r.refusal, store::StageRefusal::Invalid);
    Table candidate = defaults_table();
    candidate[idx(ParamId::rate_loop_divisor)].value.i32 = 0;
    ASSERT_NE(composition::validate(candidate).reason, nullptr);
    EXPECT_STREQ(r.reason, composition::validate(candidate).reason);
    expect_unchanged(before);
    // A stateless refusal: the same set without the failing value is accepted afterwards.
    EXPECT_TRUE(store::stage_set(idx(ParamId::rate_loop_divisor), ParamType::I32, i32_bits(2), &p).ok());
  });
}

TEST_F(StoreStaging, EveryRefusalHasItsOwnText) {
  std::vector<std::string> seen;
  for (std::uint8_t i = 0; i <= static_cast<std::uint8_t>(store::StageRefusal::Invalid); ++i) {
    const char* text = store::stage_refusal_text(static_cast<store::StageRefusal>(i));
    ASSERT_NE(text, nullptr);
    EXPECT_STRNE(text, "unknown refusal") << int{i};
    EXPECT_EQ(std::count(seen.begin(), seen.end(), text), 0) << text;
    seen.emplace_back(text);
  }
}

TEST_F(StoreStaging, AnAcceptedSetChangesOnlyTheStagedRecordAndCopiesTheSourceText) {
  in_child([&] {
    const ParamTable boot_table = start();
    const std::vector<ParamRecord> active = snapshot(param_active_table());
    EXPECT_FALSE(store::staged_differs());
    EXPECT_TRUE(tables_equal(store::staged_table(), active));
    std::string text = "measured on the bench, 2026-10-17";
    const store::StageProvenance p = provenance(text.c_str(), text.size());
    const store::StageResult r = store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(321.0F), &p);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.reason, nullptr);
    text.assign(text.size(), 'x');  // the store copied the text, it does not point into the caller's buffer
    const ParamTable staged = store::staged_table();
    ASSERT_EQ(staged.size(), kParamCount);
    for (std::size_t i = 0; i < kParamCount; ++i) {
      if (i == idx(ParamId::site_height_m)) {
        continue;
      }
      EXPECT_TRUE(record_same(staged[i], active[i], true)) << i;
    }
    const ParamRecord& s = staged[idx(ParamId::site_height_m)];
    EXPECT_EQ(s.value.type, ParamType::F32);
    EXPECT_EQ(s.value.f32, 321.0F);
    EXPECT_EQ(s.value.i32, 0);
    EXPECT_EQ(s.origin, ParamOrigin::Identified);
    EXPECT_EQ(s.method, ParamMethod::Measured);
    EXPECT_EQ(s.sigma_kind, SigmaKind::Known);
    EXPECT_EQ(s.sigma, 2.0F);
    EXPECT_FALSE(s.locked);
    EXPECT_EQ(s.unit, active[idx(ParamId::site_height_m)].unit);
    EXPECT_STREQ(s.source, "measured on the bench, 2026-10-17");
    EXPECT_TRUE(store::staged_differs());
    // P-1: nothing live changed.
    EXPECT_TRUE(tables_equal(param_active_table(), active, true));
    EXPECT_TRUE(tables_equal(boot_table, active, true));
    // An i32 parameter takes its value from the two's-complement bits.
    const store::StageProvenance q = provenance("a tolerance");
    EXPECT_TRUE(store::stage_set(idx(ParamId::rate_period_tolerance_us), ParamType::I32, i32_bits(4), &q).ok());
    EXPECT_EQ(store::staged_table()[idx(ParamId::rate_period_tolerance_us)].value.i32, 4);
    EXPECT_EQ(store::staged_table()[idx(ParamId::rate_period_tolerance_us)].value.f32, 0.0F);
    EXPECT_EQ(param_value<ParamId::rate_period_tolerance_us>(), param_defaults()[idx(ParamId::rate_period_tolerance_us)].value.i32);
  });
}

TEST_F(StoreStaging, ASetWithoutAStatedSourceIsStagedAsManualScenarioChoiceWithTheNoSourceText) {
  in_child([&] {
    start();
    const ParamRecord active = param_active_table()[idx(ParamId::inertia_xx)];
    ASSERT_EQ(active.sigma_kind, SigmaKind::Known);
    EXPECT_TRUE(store::stage_set(idx(ParamId::inertia_xx), ParamType::F32, f32_bits(0.005F), nullptr).ok());
    const ParamRecord s = store::staged_table()[idx(ParamId::inertia_xx)];
    EXPECT_EQ(s.origin, ParamOrigin::Manual);
    EXPECT_EQ(s.method, ParamMethod::Scenario);
    EXPECT_EQ(s.sigma_kind, SigmaKind::Choice);
    EXPECT_EQ(f32_bits(s.sigma), 0U);
    EXPECT_STREQ(s.source, "set over the link; no source stated");
    EXPECT_STREQ(s.source, store::kNoSourceStated);
    EXPECT_EQ(s.value.f32, 0.005F);
    EXPECT_EQ(s.unit, active.unit);
    EXPECT_FALSE(s.locked);
    // A set that states a source replaces it ...
    const store::StageProvenance p = provenance("the inertia from the bench pendulum");
    EXPECT_TRUE(store::stage_set(idx(ParamId::inertia_xx), ParamType::F32, f32_bits(0.005F), &p).ok());
    EXPECT_STREQ(store::staged_table()[idx(ParamId::inertia_xx)].source, "the inertia from the bench pendulum");
    EXPECT_EQ(store::staged_table()[idx(ParamId::inertia_xx)].sigma_kind, SigmaKind::Known);
    EXPECT_EQ(store::staged_table()[idx(ParamId::inertia_xx)].origin, ParamOrigin::Identified);
    // ... and a later plain set goes back to the default provenance.
    EXPECT_TRUE(store::stage_set(idx(ParamId::inertia_xx), ParamType::F32, f32_bits(0.006F), nullptr).ok());
    EXPECT_STREQ(store::staged_table()[idx(ParamId::inertia_xx)].source, store::kNoSourceStated);
    EXPECT_EQ(store::staged_table()[idx(ParamId::inertia_xx)].origin, ParamOrigin::Manual);
    EXPECT_EQ(store::staged_table()[idx(ParamId::inertia_xx)].sigma_kind, SigmaKind::Choice);
  });
}

TEST_F(StoreStaging, ARefusedSetAfterAnAcceptedOneLeavesTheAcceptedValue) {
  in_child([&] {
    start();
    EXPECT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(700.0F), nullptr).ok());
    const std::vector<ParamRecord> staged = snapshot(store::staged_table());
    EXPECT_EQ(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(std::numeric_limits<float>::infinity()), nullptr).refusal,
              store::StageRefusal::NotFinite);
    EXPECT_EQ(store::stage_set(idx(ParamId::rotor_speed_max), ParamType::F32, f32_bits(1.0F), nullptr).refusal, store::StageRefusal::Locked);
    EXPECT_TRUE(tables_equal(store::staged_table(), staged, true));
    EXPECT_EQ(store::staged_table()[idx(ParamId::site_height_m)].value.f32, 700.0F);
    EXPECT_TRUE(store::staged_differs());
  });
}

TEST_F(StoreStaging, TheSourceStorageIsBoundedAndAFailedSetDoesNotSpendIt) {
  in_child([&] {
    start();
    const std::size_t chunk = 1024;
    const std::string text(chunk - 1, 'x');  // each accepted set spends text + terminator = 1024 bytes
    const std::size_t sets = kConfigSlotBytes / chunk;
    const store::StageProvenance p = provenance(text.c_str(), text.size());
    for (std::size_t i = 0; i + 1 < sets; ++i) {
      ASSERT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1.0F), &p).ok()) << i;
    }
    // One chunk of room is left. A refused set (the validator) spends nothing ...
    const store::StageProvenance q = provenance(text.c_str(), text.size());
    EXPECT_EQ(store::stage_set(idx(ParamId::rate_loop_divisor), ParamType::I32, i32_bits(0), &q).refusal, store::StageRefusal::Invalid);
    // ... a text one byte too long for it is refused ...
    const std::string longer(chunk, 'y');
    const store::StageProvenance too_long = provenance(longer.c_str(), longer.size());
    const std::vector<ParamRecord> before = snapshot(store::staged_table());
    EXPECT_EQ(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(2.0F), &too_long).refusal, store::StageRefusal::SourceFull);
    EXPECT_TRUE(tables_equal(store::staged_table(), before, true));
    // ... the text that fits exactly is accepted ...
    EXPECT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(3.0F), &q).ok());
    // ... and then nothing more fits, however short, but a set without a stated source never needs room.
    const store::StageProvenance one = provenance("z");
    EXPECT_EQ(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(4.0F), &one).refusal, store::StageRefusal::SourceFull);
    EXPECT_EQ(store::staged_table()[idx(ParamId::site_height_m)].value.f32, 3.0F);
    EXPECT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(5.0F), nullptr).ok());
    EXPECT_EQ(store::staged_table()[idx(ParamId::site_height_m)].value.f32, 5.0F);
  });
}

TEST_F(StoreStaging, StagedDiffersSeesValueSigmaProvenanceAndSourceText) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
  }
  in_child([&] {
    start();  // the active table is cfg(1): site_height_m 600, Manual, Scenario, Choice, kEditSource
    EXPECT_EQ(store::boot_report().source, store::ConfigSource::Slot);
    EXPECT_FALSE(store::staged_differs());
    const std::size_t site = idx(ParamId::site_height_m);
    store::StageProvenance same;
    same.origin = ParamOrigin::Manual;
    same.method = ParamMethod::Scenario;
    same.sigma_kind = SigmaKind::Choice;
    same.sigma = 0.0F;
    same.source = kEditSource;
    same.source_bytes = std::strlen(kEditSource);
    // The same value, provenance and source text staged again (a copy of the text, not the same address): no difference.
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &same).ok());
    EXPECT_FALSE(store::staged_differs());
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(601.0F), &same).ok());
    EXPECT_TRUE(store::staged_differs());
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &same).ok());
    EXPECT_FALSE(store::staged_differs());
    store::StageProvenance other = same;
    other.method = ParamMethod::Measured;
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &other).ok());
    EXPECT_TRUE(store::staged_differs()) << "method";
    other = same;
    other.origin = ParamOrigin::Identified;
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &other).ok());
    EXPECT_TRUE(store::staged_differs()) << "origin";
    other = same;
    other.sigma_kind = SigmaKind::Known;
    other.sigma = 1.0F;
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &other).ok());
    EXPECT_TRUE(store::staged_differs()) << "sigma";
    other = same;
    other.source = "another text";
    other.source_bytes = std::strlen(other.source);
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &other).ok());
    EXPECT_TRUE(store::staged_differs()) << "source text";
    EXPECT_TRUE(store::stage_set(site, ParamType::F32, f32_bits(600.0F), &same).ok());
    EXPECT_FALSE(store::staged_differs());
  });
}

// ====================================================================================================== StoreCommands

class StoreCommands : public StoreStaging {
 protected:
  static std::vector<std::uint8_t> read_pending() {
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    const PendingFile f = hal_config_pending_read(buf);
    EXPECT_EQ(f.status, PendingRead::Ok);
    buf.resize(f.bytes);
    return buf;
  }
};

TEST_F(StoreCommands, ActivateWritesThePendingFileAndTheRequestAndAsksForTheGrant) {
  in_child([&] {
    start();
    const store::StageProvenance p = provenance("a source for the staged edit");
    ASSERT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(900.0F), &p).ok());
    Table staged{};
    std::copy(store::staged_table().begin(), store::staged_table().end(), staged.begin());
    const store::CommandResult c = store::activate(bound());
    EXPECT_EQ(c.step, store::CommandStep::WaitingForGrant);
    EXPECT_EQ(c.reason, nullptr);
    EXPECT_GE(g_maintenance_requests, 1);
    EXPECT_FALSE(hal_sim::reboot_requested());
    const Bytes pending = read_pending();
    EXPECT_TRUE(bytes_equal(pending, encode_file(staged, 0, bound())));
    ConfigHashBytes request{};
    ASSERT_TRUE(hal_config_request_take(request));
    EXPECT_EQ(request, store::config_hash(staged));
    EXPECT_EQ(request, store::section_hash(pending).config_hash);
    // The slots are the boot's to write, not the command's.
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::A)));
    EXPECT_TRUE(all_erased(hal_config_slot(ConfigSlot::B)));
  });
}

TEST_F(StoreCommands, ActivateRefusalsWriteNothing) {
  in_child([&] {
    start();
    ASSERT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(900.0F), nullptr).ok());
    for (const Binding& b : {unbound(), bound_to_other_image(), bound_without_report()}) {
      const store::CommandResult c = store::activate(b);
      EXPECT_EQ(c.step, store::CommandStep::Refused);
      EXPECT_STREQ(c.reason, store::fault_text(Fault::NotBound));
    }
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    EXPECT_EQ(hal_config_pending_read(buf).status, PendingRead::None);
    ConfigHashBytes request{};
    EXPECT_FALSE(hal_config_request_take(request));
    EXPECT_EQ(g_maintenance_requests, 0);
  });
  in_child([&] {
    BootOpts o;
    o.scratch = false;
    start(o);
    const store::CommandResult c = store::activate(bound());
    EXPECT_EQ(c.step, store::CommandStep::Refused);
    EXPECT_STREQ(c.reason, store::fault_text(Fault::NoMemory));
    EXPECT_EQ(g_maintenance_requests, 0);
  });
  in_child([&] {
    BootOpts o;
    o.known_image = false;  // the image hash is unknown: no binding can name it
    start(o);
    const store::CommandResult c = store::activate(bound());
    EXPECT_EQ(c.step, store::CommandStep::Refused);
    EXPECT_STREQ(c.reason, store::fault_text(Fault::NotBound));
  });
  in_child([&] {
    start();
    hal_sim::storage_close();  // the pending file cannot be written
    const store::CommandResult c = store::activate(bound());
    EXPECT_EQ(c.step, store::CommandStep::Refused);
    EXPECT_STREQ(c.reason, "the pending activation file could not be written");
    EXPECT_EQ(g_maintenance_requests, 0);
  });
}

TEST_F(StoreCommands, TheRebootComesOnlyOnceTheGrantIsGiven) {
  in_child([&] {
    start();
    ASSERT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(900.0F), nullptr).ok());
    EXPECT_EQ(store::activate(bound()).step, store::CommandStep::WaitingForGrant);
    const int asked = g_maintenance_requests;
    EXPECT_GE(asked, 1);
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(store::reboot_when_granted().step, store::CommandStep::WaitingForGrant);
    }
    EXPECT_EQ(g_maintenance_requests, asked + 3) << "each wait asks again";
    EXPECT_FALSE(hal_sim::reboot_requested());
    g_granted = true;
    const store::CommandResult c = store::reboot_when_granted();
    EXPECT_EQ(c.step, store::CommandStep::RebootRequested);
    EXPECT_EQ(c.reason, nullptr);
    EXPECT_TRUE(hal_sim::reboot_requested());
  });
}

TEST_F(StoreCommands, ACommandIssuedWithTheGrantAlreadyGivenRebootsAtOnce) {
  in_child([&] {
    start();
    g_granted = true;
    EXPECT_EQ(store::reset_to_factory().step, store::CommandStep::RebootRequested);
    EXPECT_TRUE(hal_sim::reboot_requested());
  });
}

TEST_F(StoreCommands, WithoutMaintenanceHooksTheRebootNeverHappens) {
  in_child([&] {
    BootOpts o;
    o.maintenance_hooks = false;
    start(o);
    g_granted = true;  // nothing asks the hook, so the grant is never seen
    EXPECT_EQ(store::reset_to_factory().step, store::CommandStep::WaitingForGrant);
    EXPECT_EQ(store::reboot_when_granted().step, store::CommandStep::WaitingForGrant);
    EXPECT_FALSE(hal_sim::reboot_requested());
  });
}

TEST_F(StoreCommands, ResetToFactoryRecordsTheFactoryConfigHash) {
  in_child([&] {
    start();
    const store::CommandResult c = store::reset_to_factory();
    EXPECT_EQ(c.step, store::CommandStep::WaitingForGrant);
    EXPECT_GE(g_maintenance_requests, 1);
    ConfigHashBytes request{};
    ASSERT_TRUE(hal_config_request_take(request));
    EXPECT_EQ(request, store::config_hash(param_defaults()));
    std::vector<std::uint8_t> buf(kConfigSlotBytes);
    EXPECT_EQ(hal_config_pending_read(buf).status, PendingRead::None);
  });
}

TEST_F(StoreCommands, RevertToLastGoodNeedsAnIntactSlotThatIsNotTheNewest) {
  in_child([&] {
    start();
    const store::CommandResult c = store::revert_to_last_good();
    EXPECT_EQ(c.step, store::CommandStep::Refused);
    EXPECT_STREQ(c.reason, store::fault_text(Fault::NoLastGood));
    ConfigHashBytes request{};
    EXPECT_FALSE(hal_config_request_take(request));
  });
  {
    Prep p(dir_);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    start();
    EXPECT_EQ(store::boot_report().slot, ConfigSlot::B);
    EXPECT_EQ(store::revert_to_last_good().step, store::CommandStep::Refused);
  });
}

TEST_F(StoreCommands, RevertToLastGoodRecordsTheOlderSlotsConfigHash) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    start();
    EXPECT_TRUE(tables_equal(param_active_table(), cfg(2)));
    const store::CommandResult c = store::revert_to_last_good();
    EXPECT_EQ(c.step, store::CommandStep::WaitingForGrant);
    EXPECT_GE(g_maintenance_requests, 1);
    EXPECT_FALSE(hal_sim::reboot_requested());
    // The request stays in storage for the next boot.
    ConfigHashBytes request{};
    ASSERT_TRUE(hal_config_request_take(request));
    EXPECT_EQ(request, store::config_hash(cfg(1)));
    EXPECT_TRUE(hal_config_request_set(request));
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::Revert);
    EXPECT_EQ(store::boot_report().request_fault, Fault::None);
    EXPECT_EQ(store::boot_report().sequence, 3U);
    EXPECT_TRUE(tables_equal(t, cfg(1)));
  });
}

TEST_F(StoreCommands, RevertToLastGoodIsRefusedWhenTheOlderSlotIsNotIntact) {
  const Bytes good = encode_file(cfg(1), 1, bound());
  Bytes flipped = good;
  flipped[store::kHeaderBytes + store::kDirEntryBytes + 40] ^= 1;
  {
    Prep p(dir_);
    p.slot_with_record(ConfigSlot::A, flipped, good, 1);
    p.slot(ConfigSlot::B, encode_file(cfg(2), 2, bound()), 2);
  }
  in_child([&] {
    start();
    EXPECT_EQ(store::revert_to_last_good().step, store::CommandStep::Refused);
  });
}

// The whole path of an activation: stage over the link, activate, reboot, and the next boot runs the staged config.
TEST_F(StoreCommands, AnActivationEndToEndStagedThenActiveAfterTheReboot) {
  const auto expected = [] {
    Table t = defaults_table();
    ParamRecord& r = t[idx(ParamId::site_height_m)];
    r.value.f32 = 900.0F;
    r.origin = ParamOrigin::Identified;
    r.method = ParamMethod::Measured;
    r.sigma_kind = SigmaKind::Known;
    r.sigma = 2.0F;
    r.source = "end to end source";
    ParamRecord& d = t[idx(ParamId::inertia_xx)];
    d.value.f32 = 0.005F;
    d.origin = ParamOrigin::Manual;
    d.method = ParamMethod::Scenario;
    d.sigma_kind = SigmaKind::Choice;
    d.sigma = 0.0F;
    d.source = store::kNoSourceStated;
    return t;
  };
  in_child([&] {
    start();
    const store::StageProvenance p = provenance("end to end source");
    ASSERT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(900.0F), &p).ok());
    ASSERT_TRUE(store::stage_set(idx(ParamId::inertia_xx), ParamType::F32, f32_bits(0.005F), nullptr).ok());
    EXPECT_TRUE(store::staged_differs());
    EXPECT_EQ(store::activate(bound()).step, store::CommandStep::WaitingForGrant);
    g_granted = true;
    EXPECT_EQ(store::reboot_when_granted().step, store::CommandStep::RebootRequested);
    EXPECT_TRUE(hal_sim::reboot_requested());
    // Nothing live changed before the reboot.
    EXPECT_TRUE(tables_equal(param_active_table(), param_defaults()));
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    const store::BootReport& r = store::boot_report();
    EXPECT_EQ(r.request, store::BootRequest::Activate);
    EXPECT_EQ(r.request_fault, Fault::None);
    EXPECT_EQ(r.source, store::ConfigSource::Slot);
    EXPECT_EQ(r.status, arming::ConfigStatus::Ok);
    EXPECT_EQ(r.sequence, 1U);
    EXPECT_EQ(r.config_hash, store::config_hash(expected()));
    EXPECT_TRUE(tables_equal(t, expected()));
    init_params(t);
    EXPECT_FALSE(store::staged_differs());
  });
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    EXPECT_EQ(store::boot_report().request, store::BootRequest::None);
    EXPECT_EQ(store::boot_report().config_hash, store::config_hash(expected()));
    EXPECT_TRUE(tables_equal(t, expected()));
  });
}

// ====================================================================================================== StoreInvariants

class StoreInvariants : public StoreStaging {};

// P-1: the active table never changes after params_init: not by staging, not by activation, not by a second params_init.
TEST_F(StoreInvariants, TheActiveTableIsUnchangedByStagingByActivationAndBySecondInit) {
  {
    Prep p(dir_);
    p.slot(ConfigSlot::A, encode_file(cfg(1), 1, bound()), 1);
  }
  in_child([&] {
    const ParamTable boot_table = start();
    const std::vector<ParamRecord> active = snapshot(param_active_table());
    const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
    EXPECT_TRUE(tables_equal(boot_table, active, true));

    const store::StageProvenance p = provenance("staged after init");
    EXPECT_TRUE(store::stage_set(idx(ParamId::site_height_m), ParamType::F32, f32_bits(1234.0F), &p).ok());
    EXPECT_TRUE(store::stage_set(idx(ParamId::inertia_xx), ParamType::F32, f32_bits(0.005F), nullptr).ok());
    EXPECT_EQ(store::stage_set(idx(ParamId::rotor_speed_min), ParamType::F32, f32_bits(1.0F), nullptr).refusal, store::StageRefusal::Locked);
    EXPECT_TRUE(store::staged_differs());
    EXPECT_TRUE(tables_equal(param_active_table(), active, true));
    EXPECT_EQ(param_value<ParamId::site_height_m>(), 600.0F);

    EXPECT_EQ(store::activate(bound()).step, store::CommandStep::WaitingForGrant);
    g_granted = true;
    EXPECT_EQ(store::reboot_when_granted().step, store::CommandStep::RebootRequested);
    EXPECT_EQ(store::revert_to_last_good().step, store::CommandStep::Refused);
    EXPECT_EQ(store::reset_to_factory().step, store::CommandStep::RebootRequested);
    EXPECT_TRUE(tables_equal(param_active_table(), active, true));
    EXPECT_EQ(param_value<ParamId::site_height_m>(), 600.0F);
    EXPECT_EQ(param_value<ParamId::rate_loop_divisor>(), divisor);

    // A second params_init, with the staged table or the very same one, is refused and changes nothing.
    EXPECT_FALSE(params_init(std::span<const ParamRecord, kParamCount>(store::staged_table().data(), kParamCount)));
    EXPECT_FALSE(params_init(param_defaults()));
    EXPECT_FALSE(params_init(std::span<const ParamRecord, kParamCount>(boot_table.data(), kParamCount)));
    EXPECT_TRUE(tables_equal(param_active_table(), active, true));
  });
}

TEST_F(StoreInvariants, SecondParamsInitReturnsFalseOnTheFactoryTableToo) {
  in_child([&] {
    const ParamTable t = boot_store(dir_);
    init_params(t);
    EXPECT_FALSE(params_init(param_defaults()));
    EXPECT_TRUE(tables_equal(param_active_table(), param_defaults()));
  });
}

}  // namespace
