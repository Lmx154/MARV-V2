#pragma once

// The I-A1 acro scenario (quad spec 4 L5, decision 0006 "A"): the L04 T3 inputs plus the scripted fixture, run through
// RateLoop<float>::execute. The test compares the output with the committed golden; golden_gen.cpp compiles the same
// header against fw/rate at tag quad-L4-pass to produce the golden. Only rate_loop.hpp's execute / record_allocation /
// integrator are used, so the header builds against both versions.
//
// Numbers: read from the fixture files, or derived as stated.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::l5_rate_bypass {

constexpr std::size_t kAxes3 = rate::kTorqueAxes;
constexpr std::array<const char*, kAxes3> kAxisName{"roll", "pitch", "yaw"};

inline std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

// Whitespace tokens of a file, '#' comments removed.
inline std::vector<std::string> tokens_of(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    line = line.substr(0, line.find('#'));
    std::istringstream words(line);
    std::string w;
    while (words >> w) {
      out.push_back(w);
    }
  }
  return out;
}

inline float from_bits(std::uint32_t bits) {
  float f;
  std::memcpy(&f, &bits, sizeof f);
  return f;
}

inline std::uint32_t to_bits(float f) {
  std::uint32_t bits;
  std::memcpy(&bits, &f, sizeof bits);
  return bits;
}

// The T3 inputs as the rate configuration: gains and tau_ref per axis, and the period formed as rate::from_params forms
// it (an exact float quotient in microseconds, then one division). The rotor geometry stays at its zero default.
inline bool read_config(const std::string& inputs_path, rate::RateConfig<float>& cfg) {
  std::map<std::string, float> v;
  const std::vector<std::string> t = tokens_of(read_file(inputs_path));
  for (std::size_t i = 0; i + 1 < t.size(); i += 2) {
    v[t[i]] = static_cast<float>(std::stod(t[i + 1]));
  }
  cfg = rate::RateConfig<float>();
  for (std::size_t a = 0; a < kAxes3; ++a) {
    const std::string axis = kAxisName[a];
    if (!v.count("rate_kp_" + axis)) {
      return false;
    }
    cfg.kp[a] = v.at("rate_kp_" + axis);
    cfg.ki[a] = v.at("rate_ki_" + axis);
    cfg.kd[a] = v.at("rate_kd_" + axis);
    cfg.tau_ref[a] = v.at("rate_tau_ref_" + axis);
  }
  cfg.period = v.at("rate_loop_divisor") * v.at("tick_period_num_us") / v.at("tick_period_den") /
               static_cast<float>(prim::kMicrosecondsPerSecond);
  return rate::validate(cfg) == rate::ConfigError::None;
}

struct Step {
  std::uint64_t t_us = 0;
  std::array<std::uint32_t, kAxes3> gyro{};
  bool gyro_valid = false;
  std::array<std::uint32_t, kAxes3> setpoint{};
  std::array<bool, kAxes3> flag{};
  std::array<std::uint32_t, kAxes3> requested{};
  std::array<std::uint32_t, kAxes3> achieved{};
};

inline bool read_fixture(const std::string& path, std::vector<Step>& out) {
  const std::vector<std::string> t = tokens_of(read_file(path));
  constexpr std::size_t kColumns = 18;  // n t_us g0..2 valid s0..2 f0..2 q0..2 a0..2
  if (t.size() < 2 || t[0] != "executions") {
    return false;
  }
  const std::size_t n = static_cast<std::size_t>(std::stoul(t[1]));
  if (t.size() != 2 + n * kColumns) {
    return false;
  }
  out.clear();
  for (std::size_t i = 0; i < n; ++i) {
    const std::string* c = &t[2 + i * kColumns];
    Step s;
    s.t_us = std::stoull(c[1]);
    for (std::size_t a = 0; a < kAxes3; ++a) {
      s.gyro[a] = static_cast<std::uint32_t>(std::stoul(c[2 + a], nullptr, 16));
      s.setpoint[a] = static_cast<std::uint32_t>(std::stoul(c[6 + a], nullptr, 16));
      s.flag[a] = c[9 + a] == "1";
      s.requested[a] = static_cast<std::uint32_t>(std::stoul(c[12 + a], nullptr, 16));
      s.achieved[a] = static_cast<std::uint32_t>(std::stoul(c[15 + a], nullptr, 16));
    }
    s.gyro_valid = c[5] == "1";
    out.push_back(s);
  }
  return true;
}

// One line per execution: n, torque bits, fault_active, fault_latched, fault_count, integrator bits. The allocation of
// execution n is recorded after its output (so its freeze gates the increment of execution n + 1).
inline std::string run_acro(const rate::RateConfig<float>& cfg, const std::vector<Step>& steps) {
  rate::RateLoop<float> loop;
  loop.init(cfg, mixer::MixerConfig<float>());
  std::string out;
  char buf[256];
  for (std::size_t n = 0; n < steps.size(); ++n) {
    const Step& s = steps[n];
    ImuSample imu{};
    imu.t_us = s.t_us;
    imu.flags = s.gyro_valid ? imu_flag(ImuFlag::GyroValid) : 0U;
    prim::Vec3<float> sp;
    for (std::size_t a = 0; a < kAxes3; ++a) {
      imu.gyro_rad_s[a] = from_bits(s.gyro[a]);
      sp[a] = from_bits(s.setpoint[a]);
    }
    const rate::RateOutput<float> o = loop.execute(imu, sp);
    mixer::Allocation<float> al;
    prim::Vec3<float> req;
    for (std::size_t a = 0; a < kAxes3; ++a) {
      req[a] = from_bits(s.requested[a]);
      al.achieved_torque[a] = from_bits(s.achieved[a]);
    }
    al.flags.roll = s.flag[0];
    al.flags.pitch = s.flag[1];
    al.flags.yaw = s.flag[2];
    loop.record_allocation(req, al);
    const prim::Vec3<float>& in = loop.integrator();
    std::snprintf(buf, sizeof buf, "%zu 0x%08x 0x%08x 0x%08x %d %d %u 0x%08x 0x%08x 0x%08x\n", n, to_bits(o.torque[0]),
                  to_bits(o.torque[1]), to_bits(o.torque[2]), o.fault_active ? 1 : 0, o.fault_latched ? 1 : 0,
                  static_cast<unsigned>(o.fault_count), to_bits(in[0]), to_bits(in[1]), to_bits(in[2]));
    out += buf;
  }
  return out;
}

// The golden body: the file without its '#' comment lines.
inline std::string golden_body(const std::string& text) {
  std::istringstream lines(text);
  std::string line;
  std::string out;
  while (std::getline(lines, line)) {
    if (!line.empty() && line[0] != '#') {
      out += line + "\n";
    }
  }
  return out;
}

}  // namespace marv::l5_rate_bypass
