#pragma once

// The rate group's step (quad spec 4 L6 stage (c); decision 0014, "Wiring"): the gyro chain (fw/gyro_chain) in front of
// the rate loop (fw/rate), then the mixer (fw/mixer). One implementation, used by the L4 and L5 compositions and by the
// harness replays that re-execute the rate group, so the wiring exists once.
//
// Per tick, in tick order:
//   filter(s, rate_due)   on a rate tick first chain.update_notches(hal_rotor_speed()), so the notches are set from the
//                         latest rotor speeds before that tick's sample is filtered; then, on every tick,
//                         g = chain.filter(s.gyro_rad_s). The sample s with its gyro replaced by g (stamp, accel,
//                         temperature and flags unchanged) is held for the tick's rate execution. A sample the rate
//                         loop refuses (GyroValid clear or a gyro axis not finite) is not filtered: the chain keeps its
//                         state and the sample is held as given, so the rate loop's fault path runs as without the
//                         chain. On a rate tick whose execution will seed the rate loop (rate.seeding(): after init or
//                         a fault) a usable sample first reseeds the chain (GyroChain::reseed: every axis restarts at
//                         that sample's steady state), so the two restart together; a refused sample on a non-rate tick
//                         is skipped and the chain continues (lead decisions, stage (c) commit 3).
//   execute(setpoint, added, thrust), on a rate tick after filter:  out = rate.execute(held, setpoint);
//   execute_bypass(reference, added, thrust), the same with         out = rate.execute_bypass(held, reference);
//                         then request = out.torque + added; alloc = allocate({thrust, request});
//                         rate.record_allocation(request, alloc); dshot = diffuser.apply(alloc.f), the DShot error
//                         diffusion of decision 0017 (mixer::DshotDiffuser): one write per rate execution, its carry
//                         cleared by init only (a rate-loop fault does not clear it).
// Through marv_sil_tick no rotor speed is staged: hal_rotor_speed() reports every motor invalid, every notch is
// bypassed and the chain is its low-pass alone (the stage (c) T4 configuration, decision 0014).

#include <array>

#include "marv/gyro_chain/gyro_chain.hpp"
#include "marv/mixer/mixer.hpp"
#include "marv/prim/vec.hpp"
#include "marv/rate/rate_loop.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::rate_group {

// The chain configuration from the product parameters: gyro_lpf_cutoff_hz, gyro_notch_q_h1..h3, gyro_notch_omega_min,
// the tick period tick_period_num_us / tick_period_den microseconds in seconds, the rate divisor rate_loop_divisor (a
// value below 1 is read as 0, which validate refuses) and seed_first_sample set (decision 0014, c1: the D term sees no
// start-up transient). Not validated. pre: params_init succeeded.
[[nodiscard]] gyro_chain::GyroChainConfig<float> chain_from_params() noexcept;

// hal_panic naming the violated rule unless gyro_chain::validate(c) == None.
void require_valid(const gyro_chain::GyroChainConfig<float>& c) noexcept;

// chain_from_params, then require_valid: the firmware-facing init.
[[nodiscard]] gyro_chain::GyroChainConfig<float> load_chain_config() noexcept;

// One rate execution.
struct Execution {
  rate::RateOutput<float> rate{};                  // the rate loop's output
  prim::Vec3<float> request{};                     // rate.torque + added: the torque request passed to the mixer
  mixer::Allocation<float> alloc{};                // allocate({thrust, request})
  std::array<DshotValue, mixer::kMotors> dshot{};  // diffuser.apply(alloc.f)
};

class RateGroupStep {
 public:
  // pre: rate::validate(rate_cfg), mixer::validate(mixer_cfg) and gyro_chain::validate(chain_cfg) are None. Clears the
  // rate loop, the chain and the diffuser's carry (DshotDiffuser::init on mixer_cfg).
  void init(const rate::RateConfig<float>& rate_cfg, const mixer::MixerConfig<float>& mixer_cfg,
            const gyro_chain::GyroChainConfig<float>& chain_cfg) noexcept;

  // Every tick, in order (file comment).
  void filter(const ImuSample& s, bool rate_due) noexcept;

  // A rate execution, after filter(s, true) of the same tick (file comment).
  [[nodiscard]] Execution execute(const prim::Vec3<float>& setpoint, const prim::Vec3<float>& added,
                                  float thrust) noexcept;
  [[nodiscard]] Execution execute_bypass(const prim::Vec3<float>& reference, const prim::Vec3<float>& added,
                                         float thrust) noexcept;

  // The sample held by the latest filter: what the tick's rate execution gets.
  [[nodiscard]] const ImuSample& rate_input() const noexcept { return filtered_; }

 private:
  [[nodiscard]] Execution finish(const rate::RateOutput<float>& out, const prim::Vec3<float>& added,
                                 float thrust) noexcept;

  rate::RateLoop<float> rate_;
  mixer::MixerConfig<float> mixer_{};
  gyro_chain::GyroChain<float> chain_;
  ImuSample filtered_{};
  mixer::DshotDiffuser<float> diffuser_;
};

}  // namespace marv::rate_group
