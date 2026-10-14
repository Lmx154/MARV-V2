#pragma once

// Sim-side truth-vertical source for quad L11 (decision 0020 section 1, item c, 'Adapter and log'): before each
// marv_sil_tick the SIL is handed the plant's truth down-position and down-velocity through marv_truth_vertical_set
// (fw/sil/include/marv_truth_vertical.h, exported only by SIL libraries built with TRUTH_VERTICAL). Lives under sim/ only
// (core 7.4 G3): the marv::truth namespace never reaches fw/. A separate static target, marv_gz_truth_vertical: the plugin
// links it only when the linked SIL library has the MARV_SIL_TRUTH_VERTICAL property, so the plugin still builds against
// libraries without the entry. In the plugin it wraps the truth-attitude source (truth_attitude.hpp), which wraps the
// gyro source.
//
// Freshness: within one host step the state is the body state at the step's start, held over the step's m ticks, as the
// truth attitude; only `tick` changes from tick to tick.

#include <cstdint>

#include "marv/gz/adapter.hpp"
#include "marv_plant.h"
#include "marv_truth_vertical.h"

namespace marv::truth {

// A command source that wraps another: dshot(tick) calls marv_truth_vertical_set(state with tick = `tick`), then the inner
// source's dshot(tick). set_body is called before Adapter::step. The caller has run marv_sil_init.
class TruthVertical final : public marv::gz::CommandSource {
 public:
  explicit TruthVertical(marv::gz::CommandSource& inner) : inner_(inner) {}
  // z_ned_m and vz_ned_m_s are the float casts of the body's pos_ned_m[2] and vel_ned_m_s[2]; flags = the valid bit iff
  // both casts are finite, else flags 0 and both fields zero (the entry's valid-bit rule).
  void set_body(const marv_plant_body& body);
  // False if marv_truth_vertical_set was not OK (the inner source is then not called; truth_status() is the status) or
  // if the inner source failed (truth_status() is OK).
  bool dshot(std::uint64_t tick, marv::gz::Dshot& out) override;
  // The same, forwarding the tick's IMU sample (and rotor-speed sample) to the inner source's form of that arity.
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, marv::gz::Dshot& out) override;
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor,
             marv::gz::Dshot& out) override;
  std::int32_t truth_status() const { return status_; }  // marv_sil_status of the last marv_truth_vertical_set
  const marv_truth_vertical& state() const { return state_; }  // the state last passed (or, before a call, the next one)

 private:
  marv::gz::CommandSource& inner_;
  marv_truth_vertical state_{};
  std::int32_t status_ = MARV_SIL_OK;
};

}  // namespace marv::truth
