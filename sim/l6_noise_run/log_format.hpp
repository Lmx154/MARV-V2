#pragma once

// The run log of l6_noise_run (L6 stage (a), quad spec section 4 L6 pass bar (a)): deterministic binary, every integer
// and every float/double stored as its bits, little endian, no padding. A header then one record per tick.
//
// Header, kHeaderBytes = 32:   magic "MARVL6N1" (8 bytes) | seed u64 | clock corner i64 | number of ticks u64
// Record, kRecordBytes = 112:  tick u64                      (the number handed to marv_sil_tick)
//                              firmware stamp u64            (the SIL's t_us of the tick, microseconds)
//                              IMU bytes, 32:                gyro x y z f32 | accel x y z f32 | temp_k f32 | flags u32
//                              DShot 4 x u16                 (logical motors 1..4)
//                              body quaternion 4 x f64       ([w, x, y, z], body to NED) held over the tick
//                              body rate 3 x f64             (rad/s, FRD) held over the tick

#include <cstddef>
#include <cstdint>

namespace marv::l6run {

inline constexpr char kMagic[8] = {'M', 'A', 'R', 'V', 'L', '6', 'N', '1'};
inline constexpr std::size_t kHeaderBytes = 32;
inline constexpr std::size_t kHeaderSeedOffset = 8;
inline constexpr std::size_t kHeaderCornerOffset = 16;
inline constexpr std::size_t kHeaderTicksOffset = 24;

inline constexpr std::size_t kRecordBytes = 112;
inline constexpr std::size_t kRecTickOffset = 0;
inline constexpr std::size_t kRecStampOffset = 8;
inline constexpr std::size_t kRecImuOffset = 16;
inline constexpr std::size_t kRecImuBytes = 32;
inline constexpr std::size_t kRecDshotOffset = 48;
inline constexpr std::size_t kRecDshotBytes = 8;
inline constexpr std::size_t kRecBodyOffset = 56;  // quaternion then rate: 7 x f64
inline constexpr std::size_t kRecBodyBytes = 56;

static_assert(kRecBodyOffset + kRecBodyBytes == kRecordBytes);

}  // namespace marv::l6run
