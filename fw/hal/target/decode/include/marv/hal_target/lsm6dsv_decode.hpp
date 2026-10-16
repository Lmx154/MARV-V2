#pragma once

// The LSM6DSV sample decode of the FC3v2 driver (fw/hal/target/src/lsm6dsv.cpp): FIFO words to ImuSample in body FRD,
// and the stamping rule. No Pico SDK, so the L09 T1 tests build it on the host.
//
// Assembly. The FIFO batches the gyroscope and the accelerometer at the same rate; a sample is the gyroscope word and
// the accelerometer word of one time slot (TAG_CNT, DS13476 §9.81 Table 209). That both words of one slot carry the same
// TAG_CNT is INFERRED (the datasheet calls it "a 2-bit counter which identifies sensor time slot"). A word whose slot
// differs from the held word's, or which repeats a sensor already held, first completes the held word as a partial
// sample: the missing sensor's valid bit clear and its field exactly 0 (imu_sample.hpp). A word with any other tag is
// refused (BadTag) and changes nothing.
//
// Conversion. Board FRD axis i = sign_i x chip axis chip_axis_i (the board file's mounting map), times the datasheet
// sensitivity in SI (lsm6dsv_scales). A saturation bit is set per FRD axis when its chip value is at the int16 limit;
// that the part clips there at the configured full scales is INFERRED. Temperature: the latest OUT_TEMP reading set
// with set_temperature, carried with TempValid by every sample assembled after it.
//
// Stamping rule. The INT1 handler reads t_irq = hal_time_us() on entry, then the FIFO. The k samples it assembles are
// stamped oldest first, the j-th (j = 0 .. k-1) at t_irq - round((k - 1 - j) T) with T = 1/ODR (nominal, 1e6/7680 us);
// a stamp not above the previous one becomes the previous one + 1 us (a clamp, counted), so t_us strictly increases.
// The newest sample's stamp is late by the time from its FIFO write to the handler's entry; an older sample's is off
// by that plus its back-step times the ODR error. That bound, delta, is a PLACEHOLDER: not yet measured; the M1 P12
// bench measurement (the INT1 edge against the stamp on a logic analyzer) replaces it.

#include <array>
#include <cstddef>
#include <cstdint>

#include <parts/lsm6dsv.hpp>

#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

inline constexpr std::size_t kImuAxes = prim::kSpatialDim;

// Board FRD axis i = sign[i] x chip axis chip_axis[i]. Valid iff chip_axis is a permutation of 0, 1, 2 and every sign
// is 1 or -1.
struct ImuMounting {
  std::array<std::size_t, kImuAxes> chip_axis{};
  std::array<int, kImuAxes> sign{};
};

[[nodiscard]] bool valid(const ImuMounting& m) noexcept;

// SI per LSB at the configured full scales, and the temperature at 0 LSB.
struct Lsm6dsvScales {
  float gyro_rad_s_per_lsb{};
  float accel_m_s2_per_lsb{};
  float temp_k_per_lsb{};
  float temp_k_at_zero_lsb{};
};

// From boards/parts/lsm6dsv.hpp (DS13476 Tables 3 and 5) and the SI constants of prim/constants.hpp; the accelerometer's
// g is read as the standard acceleration of gravity (INFERRED).
[[nodiscard]] Lsm6dsvScales lsm6dsv_scales() noexcept;

// One FIFO word as read from FIFO_DATA_OUT_TAG (78h) on: the tag byte, then X, Y, Z each low byte first.
using FifoWord = std::array<std::uint8_t, lsm6dsv::kFifoWordBytes>;

enum class WordResult : std::uint8_t { Held, Sample, BadTag };

class Lsm6dsvAssembler {
 public:
  // pre: valid(m).
  Lsm6dsvAssembler(const ImuMounting& m, const Lsm6dsvScales& s) noexcept : mount_(m), scales_(s) {}

  // The latest OUT_TEMP (20h-21h) value, two's complement.
  void set_temperature(std::int16_t raw) noexcept;

  // One FIFO word (file comment). Sample: *out holds the completed sample, t_us 0 (the stamper sets it). At most one
  // sample per word.
  [[nodiscard]] WordResult push(const FifoWord& w, ImuSample& out) noexcept;

 private:
  using Raw = std::array<std::int16_t, kImuAxes>;
  [[nodiscard]] ImuSample complete() noexcept;

  ImuMounting mount_;
  Lsm6dsvScales scales_;
  Raw gyro_{};
  Raw accel_{};
  std::uint8_t slot_ = 0;
  bool have_gyro_ = false;
  bool have_accel_ = false;
  bool have_temp_ = false;
  float temp_k_ = 0.0F;
};

// The stamping rule (file comment).
class ImuStamper {
 public:
  // pre: odr_hz > 0.
  explicit ImuStamper(std::uint32_t odr_hz) noexcept : odr_hz_(odr_hz) {}

  // The stamp of the sample `back` places before the newest of a batch whose handler began at t_irq (newest: 0).
  [[nodiscard]] TimeUs stamp(TimeUs t_irq, std::size_t back) noexcept;

  [[nodiscard]] std::uint32_t clamps() const noexcept { return clamps_; }

 private:
  std::uint64_t odr_hz_;
  TimeUs last_ = 0;
  bool have_last_ = false;
  std::uint32_t clamps_ = 0;
};

}  // namespace marv::hal_target
