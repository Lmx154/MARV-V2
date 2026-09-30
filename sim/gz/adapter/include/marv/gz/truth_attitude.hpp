#pragma once

// Sim-side truth-attitude source for quad L5 (decision 0006 section B, 'Plugin'): before each marv_sil_tick the SIL is
// handed the plant's truth attitude and body rate through marv_truth_state_set (fw/sil/include/marv_truth.h, exported
// only by SIL libraries built with TRUTH_STATE). Lives under sim/ only (core 7.4 G3): the marv::truth namespace never
// reaches fw/. A separate static target, marv_gz_truth_attitude: the plugin links it only when the linked SIL library
// has the MARV_SIL_TRUTH_STATE property, so the plugin still builds against libraries without the entry.
//
// Freshness: within one host step the state is the body state at the step's start, held over the step's m ticks, as the
// truth gyro (truth_gyro.hpp); only `tick` changes from tick to tick.

#include <cstdint>

#include "marv/gz/adapter.hpp"
#include "marv_plant.h"
#include "marv_truth.h"

namespace marv::truth {

// A command source that wraps another: dshot(tick) calls marv_truth_state_set(state with tick = `tick`), then the inner
// source's dshot(tick) (the marv_sil_tick). set_body is called before Adapter::step. The caller has run marv_sil_init.
class TruthAttitude final : public marv::gz::CommandSource {
 public:
  explicit TruthAttitude(marv::gz::CommandSource& inner) : inner_(inner) {}
  // q_wxyz is the float cast of the body's, negated as a whole if w < 0 (canonical sign, core section 3); omega_frd_rad_s
  // is the float cast of the body's; flags = the valid bit only.
  void set_body(const marv_plant_body& body);
  // False if marv_truth_state_set was not OK (the inner source is then not called; truth_status() is the status) or if
  // the inner source failed (truth_status() is OK).
  bool dshot(std::uint64_t tick, marv::gz::Dshot& out) override;
  std::int32_t truth_status() const { return status_; }  // marv_sil_status of the last marv_truth_state_set
  const marv_truth_state& state() const { return state_; }  // the state last passed (or, before a call, the next one)

 private:
  marv::gz::CommandSource& inner_;
  marv_truth_state state_{};
  std::int32_t status_ = MARV_SIL_OK;
};

}  // namespace marv::truth
