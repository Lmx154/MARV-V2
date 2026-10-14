#pragma once

// The L11 vertical loop (decision 0020 section 1): a height P outer loop on a vertical-velocity PI inner loop, the
// collective-thrust axis of today's cascade (thrust/torque, m/J, v_z/omega, z/theta). Opening: in a VerticalState
// (z and v_z, NED, z down) and a height reference z_ref (m); out a collective thrust request (N). The mixer alone owns
// the thrust limits: the law never clamps (decision 0020 I4). Configuration is VerticalConfig (config.hpp).
//
// Execution n, period T (a fixed design period; the sample stamp is not used), NED with z down:
//   e_z   = z_ref - z                         height error, m
//   v_sp  = k_z e_z                           climb-rate setpoint, m/s (positive = down)
//   e_v,n = v_sp - v_z                        m/s
//   I_n   = I_(n-1) + (freeze_(n-1) ? 0 : k_i e_v,(n-1) T)     forward Euler, deferred one execution as the rate loop
//   u_n   = k_v e_v,n + I_n                   N
//   T_req = T_h - u_n                         N
// Sign convention: z is down-positive, so a vehicle below the reference has z > z_ref, e_z < 0, v_sp < 0 (a climb is
// asked for), e_v < 0 at rest, u < 0, and T_req = T_h - u > T_h: below the reference gives more thrust. u enters T_req
// with a minus, so a positive e_v (the demand to descend faster) lowers the thrust.
// The first execution after init or reset has e_v,(n-1) = 0, so it adds nothing to I.
//
// Anti-windup (decision 0005 item 4 transposed to the collective). freeze_n is recorded by record_allocation from
// execution n's allocation and gates the increment applied at execution n + 1. With diff = T_req - T_ach, T_ach =
// the mixer's achieved_thrust,
//   freeze = |diff| > b  and  e_v diff < 0
// i.e. the request is clamped by more than the rounding bound and e_v drives further into the limit. Derivation of the
// sign: u enters T_req with a minus, so a larger e_v (more u) gives less thrust. A low-side clamp (T_ach > T_req, diff
// < 0) is driven further by asking for even less thrust, which is e_v > 0; a high-side clamp (T_ach < T_req, diff > 0)
// is driven further by e_v < 0. With e_v = 0 or |diff| <= b nothing freezes.
// Rounding bound b: the thrust row of the rate loop's bound (rate_loop.hpp record_allocation, decision 0004 item 2),
//   b = (gamma_n + eps) (|B-hat| |M-hat| |v|)_thrust,  v = [achieved_thrust, achieved_torque],  n = kMotors,
//   gamma_n = n eps / (1 - n eps),  eps = the scalar epsilon.
// The thrust row of B-hat is [1, 1, 1, 1] for every configuration (rate::effectiveness), so the thrust row of
// |B-hat| |M-hat| is the column sums of |M-hat|, formed once in init from the mixer configuration as the rate loop does.
// The torque part of v is in the bound: a clamped collective c_lo / c_hi is formed from divisions whose rounding
// scales with the torque part of the motor thrusts.
// So an allocation that differs from the request by rounding only (achieved == requested) integrates.
//
// Invalid state (VerticalState::valid clear, or z or v_z not finite) or a non-finite z_ref or a non-finite T_req: the
// output is exactly T_h, I, the previous e_v and the freeze are reset, and the fault flag is set. The flag is a latch:
// it stays set through later valid executions (which resume from I = 0) and through reset(); only init() clears it,
// as the rate and attitude loops' fault latch.

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "marv/mixer/mixer.hpp"
#include "marv/types/vertical_state.hpp"
#include "marv/vertical/config.hpp"

namespace marv::vertical {

template <class T>
struct VerticalOutput {
  T thrust_request{};  // N: T_req, or exactly T_h on a fault
  T e_z{};             // m, 0 on a fault
  T e_v{};             // m/s, 0 on a fault
  bool fault = false;  // the fault latch (see the header comment)
};

template <class T>
class VerticalLaw {
 public:
  // pre: validate(cfg) == None and mixer::validate(mixer_cfg) == None. Clears all state and the fault latch.
  void init(const VerticalConfig<T>& cfg, const mixer::MixerConfig<T>& mixer_cfg) noexcept {
    using std::abs;
    cfg_ = cfg;
    for (std::size_t c = 0; c < mixer::kAxes; ++c) {
      T sum = T(0);
      for (std::size_t k = 0; k < mixer::kMotors; ++k) {
        sum += abs(mixer_cfg.m(k, c));  // |B-hat(thrust, k)| = 1
      }
      abs_bm_thrust_[c] = sum;
    }
    reset();
    latched_ = false;
  }

  // Clears the integrator, the previous e_v and the freeze. The fault latch is kept.
  void reset() noexcept { reset_state(); }

  [[nodiscard]] VerticalOutput<T> execute(const VerticalState<T>& s, T z_ref) noexcept {
    using std::isfinite;
    if (!s.valid || !isfinite(s.z_ned_m) || !isfinite(s.vz_ned_m_s) || !isfinite(z_ref)) {
      return enter_fault();
    }
    const T e_z = z_ref - s.z_ned_m;
    const T e_v = cfg_.kp_z * e_z - s.vz_ned_m_s;
    if (!freeze_) {
      integral_ = integral_ + cfg_.ki_v * e_v_prev_ * cfg_.period;
    }
    const T u = cfg_.kp_v * e_v + integral_;
    const T t_req = cfg_.hover_thrust - u;
    if (!isfinite(t_req) || !isfinite(integral_)) {
      return enter_fault();
    }
    e_v_prev_ = e_v;
    freeze_ = false;
    return VerticalOutput<T>{t_req, e_z, e_v, latched_};
  }

  // Records the freeze that gates the next execution's integrator increment; see the header comment for the rule.
  // `requested_thrust` is the T_req passed to the mixer, `al` the mixer's allocation of it.
  void record_allocation(const mixer::Allocation<T>& al, T requested_thrust) noexcept {
    using std::abs;
    const T eps = std::numeric_limits<T>::epsilon();
    const T n_eps = static_cast<T>(mixer::kMotors) * eps;
    const T gamma = n_eps / (T(1) - n_eps);
    const std::array<T, mixer::kAxes> v{al.achieved_thrust, al.achieved_torque[0], al.achieved_torque[1],
                                        al.achieved_torque[2]};
    T sum = T(0);
    for (std::size_t k = 0; k < mixer::kAxes; ++k) {
      sum += abs_bm_thrust_[k] * abs(v[k]);
    }
    const T bound = (gamma + eps) * sum;
    const T diff = requested_thrust - al.achieved_thrust;
    const bool further = (e_v_prev_ > T(0) && diff < T(0)) || (e_v_prev_ < T(0) && diff > T(0));
    freeze_ = abs(diff) > bound && further;
  }

  [[nodiscard]] T integrator() const noexcept { return integral_; }
  [[nodiscard]] bool frozen() const noexcept { return freeze_; }
  [[nodiscard]] bool fault() const noexcept { return latched_; }

 private:
  void reset_state() noexcept {
    integral_ = T(0);
    e_v_prev_ = T(0);
    freeze_ = false;
  }

  [[nodiscard]] VerticalOutput<T> enter_fault() noexcept {
    reset_state();
    latched_ = true;
    return VerticalOutput<T>{cfg_.hover_thrust, T(0), T(0), true};
  }

  VerticalConfig<T> cfg_{};
  std::array<T, mixer::kAxes> abs_bm_thrust_{};  // the thrust row of |B-hat| |M-hat|
  T integral_{};
  T e_v_prev_{};
  bool freeze_ = false;
  bool latched_ = false;
};

extern template class VerticalLaw<float>;

}  // namespace marv::vertical
