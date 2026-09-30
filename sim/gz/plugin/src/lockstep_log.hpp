#pragma once

// The binary log of the lockstep plugin (decision 0003 item 10). Little-endian, no padding, no wall-clock, host or path
// bytes. The reader is tools/sim/lockstep_log.py; the two must agree.
//
// File = header, then records. Every record is one type byte and a fixed-size payload of that type.
//
//   header   magic 8 bytes "MARVLOCK"; u32 version (1); u32 header_size (bytes, this header); u32 m (ticks per host
//            step); u32 tick_period_num_us; u32 tick_period_den; u64 seed; u32 n then n bytes of the gz-sim version
//            string (GZ_SIM_VERSION_FULL, no NUL). The scenario/world file's SHA-256 is not recorded (the plugin
//            never sees the file path): the runner reports it.
//
//   Per host step, in this order:
//   1 STEP     u64 iteration (gz UpdateInfo::iterations); u64 sim_time_ns; then doubles: the raw gz reads, pose position
//              ENU [3], quaternion FLU->ENU [w x y z], world linear velocity ENU [3], world angular velocity ENU [3];
//              the marv_plant_body passed, position NED [3], velocity NED [3], q [w x y z], omega FRD [3].
//   2 TICK     (m of them) u64 tick; u64 sil_t_us (the SIL's stamp of that tick); the 32 bytes of the marv_imu_meas
//              passed (zeroed, no valid bit set, unless the plugin has <gyro_source>truth</gyro_source>: then the gyro is
//              the float cast of the step's body omega FRD and flags is GyroValid only); u16 dshot[4] (motor 1..4); u32 erpm_valid; u32 0; then doubles: plant
//              force NED [3], torque NED [3], rotor speed [4], erpm [4], and the per-tick wrench in ENU, force [3],
//              torque [3].
//   5 TRUTH    only with the plugin's <attitude_source>truth</attitude_source>, directly after that tick's TICK record:
//              u64 tick; then the sizeof(marv_truth_state) bytes (fw/sil/include/marv_truth.h; 48 on the host, the last
//              4 being padding, zero) of the state passed to marv_truth_state_set before that tick's marv_sil_tick. The
//              trailer's counts do not include it. A log of a run without the element has no type 5 record and is the
//              log it was before the element existed.
//   3 APPLIED  doubles: the wrench applied to gz, W_bar in ENU, force [3], torque [3].
//
//   4 TRAILER  written when the plugin is destroyed: u64 step records, u64 tick records, u64 applied records. A file
//              without a trailer ended abnormally (a refusal at run time, an abort, a crash).
//
// Each host step is flushed to the operating system as one write.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace marv::gz {

enum class LogRecord : std::uint8_t { kStep = 1, kTick, kApplied, kTrailer, kTruth };

class LogBuffer {
 public:
  void u8(std::uint8_t v);
  void u16(std::uint16_t v);
  void u32(std::uint32_t v);
  void u64(std::uint64_t v);
  void f64(double v);
  void bytes(const void* p, std::size_t n);
  const std::vector<std::uint8_t>& data() const { return data_; }
  void clear() { data_.clear(); }

 private:
  std::vector<std::uint8_t> data_;
};

class LogFile {
 public:
  LogFile() = default;
  ~LogFile();
  LogFile(const LogFile&) = delete;
  LogFile& operator=(const LogFile&) = delete;

  // False (with `error` set) if the file cannot be created or the header cannot be written.
  bool open(const std::string& path, std::uint32_t m, std::uint32_t num_us, std::uint32_t den, std::uint64_t seed,
            const std::string& gz_version, std::string& error);
  bool is_open() const { return file_ != nullptr; }
  // Appends the buffer and flushes. False on a write failure.
  bool write(const LogBuffer& b);
  // Writes the trailer with the record counts and closes. A no-op if not open.
  void close_with_trailer(std::uint64_t steps, std::uint64_t ticks, std::uint64_t applied);

 private:
  std::FILE* file_ = nullptr;
};

}  // namespace marv::gz
