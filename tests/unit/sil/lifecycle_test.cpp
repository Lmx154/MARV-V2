// Lifecycle UNINIT -> READY -> DONE: every function in every state, info_get and status_str.
#include <cstring>
#include <set>
#include <string>

#include "sil_test_util.hpp"

namespace siltest {
namespace {

marv_sil_info fresh_info() {
  marv_sil_info i{};
  i.struct_size = sizeof(marv_sil_info);
  return i;
}

void reach_ready() {
  const marv_sil_config cfg = make_config();
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

void reach_done() {
  reach_ready();
  ASSERT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
}

marv_sil_status tick_valid(std::uint64_t first, std::uint32_t k) {
  const std::vector<marv_imu_meas> s = make_samples(k, static_cast<std::uint32_t>(first));
  OutBuf buf(k);
  marv_sil_out out = buf.out();
  return marv_sil_tick(first, k, s.data(), &out);
}

void expect_info_ok() {
  marv_sil_info info = fresh_info();
  EXPECT_EQ(marv_sil_info_get(&info), MARV_SIL_OK);
  EXPECT_EQ(info.n_motors, marv::composition::kMotors);
  EXPECT_EQ(info.n_servos, marv::composition::kServos);
  EXPECT_EQ(info.n_params, marv::kParamCount);
  EXPECT_EQ(info.param_schema_hash, marv::kParamSchemaHash);
  ASSERT_NE(info.composition, nullptr);
  EXPECT_STREQ(info.composition, marv::composition::kName);
}

TEST(SilLifecycle, InfoGetBeforeInitReturnsTheCompositionCountsAndSchema) {
  // Runs in the parent: info_get does not change library state.
  expect_info_ok();
}

TEST(SilLifecycle, InfoGetWritesNothingWhenItRejects) {
  marv_sil_info info = fresh_info();
  info.struct_size = sizeof(marv_sil_info) + 1;
  info.n_motors = 77;
  info.n_servos = 78;
  info.n_params = 79;
  info.param_schema_hash = 80;
  info.composition = nullptr;
  EXPECT_EQ(marv_sil_info_get(&info), MARV_SIL_E_ABI);
  EXPECT_EQ(info.n_motors, 77u);
  EXPECT_EQ(info.n_servos, 78u);
  EXPECT_EQ(info.n_params, 79u);
  EXPECT_EQ(info.param_schema_hash, 80u);
  EXPECT_EQ(info.composition, nullptr);
  EXPECT_EQ(marv_sil_info_get(nullptr), MARV_SIL_E_NULL);
}

TEST(SilLifecycle, UninitMatrix) {
  in_child([] {
    expect_info_ok();
    EXPECT_EQ(tick_valid(0, 1), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_sil_tick(0, 0, nullptr, nullptr), MARV_SIL_E_STATE);  // the state is checked before the arguments
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_E_STATE);
    expect_info_ok();
    const marv_sil_config cfg = make_config();
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);  // UNINIT -> READY
    EXPECT_EQ(tick_valid(0, 1), MARV_SIL_OK);     // and the rejected calls above left it usable
  });
}

TEST(SilLifecycle, ReadyMatrix) {
  in_child([] {
    reach_ready();
    expect_info_ok();
    const marv_sil_config cfg = make_config();
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_STATE);  // second init
    EXPECT_EQ(marv_sil_init(nullptr), MARV_SIL_E_STATE);
    EXPECT_EQ(tick_valid(0, 3), MARV_SIL_OK);
    EXPECT_EQ(tick_valid(3, 3), MARV_SIL_OK);
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_STATE);
    expect_info_ok();
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_OK);  // READY -> DONE
  });
}

TEST(SilLifecycle, DoneMatrixAndDoneIsTerminal) {
  in_child([] {
    reach_done();
    expect_info_ok();
    const marv_sil_config cfg = make_config();
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_STATE);
    EXPECT_EQ(tick_valid(0, 1), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_sil_init(&cfg), MARV_SIL_E_STATE);
    EXPECT_EQ(tick_valid(0, 1), MARV_SIL_E_STATE);
    expect_info_ok();
  });
}

TEST(SilLifecycle, ShutdownWithoutTicksIsAllowed) {
  in_child([] {
    reach_ready();
    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
  });
}

TEST(SilLifecycle, StatusStrIsStaticNonNullAndDistinctPerStatus) {
  std::set<std::string> seen;
  for (marv_sil_status s = MARV_SIL_OK; s <= MARV_SIL_E_INPUT; ++s) {
    const char* text = marv_sil_status_str(s);
    ASSERT_NE(text, nullptr) << s;
    EXPECT_GT(std::strlen(text), 0u) << s;
    EXPECT_EQ(text, marv_sil_status_str(s)) << "static text: same pointer on every call, status " << s;
    EXPECT_TRUE(seen.insert(text).second) << "duplicate text for status " << s;
  }
  for (const marv_sil_status s : {static_cast<marv_sil_status>(-1), static_cast<marv_sil_status>(MARV_SIL_E_INPUT + 1),
                                  static_cast<marv_sil_status>(1000), INT32_MIN, INT32_MAX}) {
    EXPECT_NE(marv_sil_status_str(s), nullptr) << s;
    EXPECT_GT(std::strlen(marv_sil_status_str(s)), 0u) << s;
  }
}

TEST(SilLifecycle, ErrorIndexIsZeroBeforeAnyError) { EXPECT_EQ(marv_sil_error_index(), 0u); }
}  // namespace
}  // namespace siltest
