// marv_null_plant_run: drives the SIL entry (marv_sil.h) with a deterministic synthetic IMU stream and a null plant
// (no physics; the actuator output is not fed back), checks every returned row and prints the hash of the per-tick
// trace. One SIL run per process (the SIL init rule).
//
// Usage: marv_null_plant_run [options]
//   --seed N                  PRNG seed (u64, decimal or 0x hex)
//   --ticks N                 number of ticks (>= 1)
//   --period NUM/DEN          tick period NUM/DEN microseconds
//   --batch K1,K2,...         ticks per marv_sil_tick call, cycled; the last call is shortened to fit
//   --set NAME=VALUE          parameter override by name (repeatable); type and id from the generated manifest;
//                             sigma 0
//   --flip-bit TICK:FIELD:BIT flip bit BIT (0 = least significant) of a sample float at tick TICK after generation;
//                             FIELD is gx gy gz ax ay az or temp
//   --manifest PATH           params_manifest.json (default: the one generated for this build)
//   --trace-out PATH          also write the per-tick trace as text (n t_us motors... servos...)
//   --write-golden PATH       write the golden file (comment line + hash) instead of printing; needs --note; refuses
//                             --set and --flip-bit. ONLY run inside the CI image (see tests/regression/quad/L00/replay/CMakeLists.txt)
//   --note TEXT               text appended to the golden file's comment line
// Output: the 16-hex-digit trace hash and a newline on stdout. Exit 0 ok; 1 usage; 2 SIL status error; 3 check failed.
#include <marv_sil.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "marv/null_plant/imu_synth.hpp"
#include "marv/null_plant/manifest.hpp"
#include "marv/null_plant/trace.hpp"

#ifndef MARV_NULL_PLANT_DEFAULT_MANIFEST
#error "MARV_NULL_PLANT_DEFAULT_MANIFEST must name the generated params_manifest.json"
#endif

namespace {

using namespace marv::null_plant;

// The default scenario. Every number is a labelled SCENARIO VALUE (core 2), not a vehicle number.
constexpr std::uint64_t kScenarioSeed = 0x4D41525630303031ULL;  // "MARV0001"
constexpr std::uint64_t kScenarioTicks = 4000;
constexpr std::uint32_t kScenarioPeriodNumUs = 625;  // 6.4 kHz IMU: 156.25 us, not a whole number of microseconds
constexpr std::uint32_t kScenarioPeriodDen = 4;
constexpr const char* kScenarioBatch = "1,3,7,16,5,2,64,9";

struct FlipBit {
  std::uint64_t tick;
  int field;  // 0..6: gx gy gz ax ay az temp
  unsigned bit;
};

struct Options {
  std::uint64_t seed = kScenarioSeed;
  std::uint64_t ticks = kScenarioTicks;
  std::uint32_t num_us = kScenarioPeriodNumUs;
  std::uint32_t den = kScenarioPeriodDen;
  std::string batch_text = kScenarioBatch;
  std::vector<std::string> sets;
  std::optional<FlipBit> flip;
  std::string manifest = MARV_NULL_PLANT_DEFAULT_MANIFEST;
  std::string trace_out;
  std::string write_golden;
  std::string note;
};

[[nodiscard]] bool parse_u64(const std::string& s, std::uint64_t& out) {
  if (s.empty() || s[0] == '-') {
    return false;
  }
  char* end = nullptr;
  out = std::strtoull(s.c_str(), &end, 0);
  return *end == '\0';
}

[[nodiscard]] bool parse_batch(const std::string& text, std::vector<std::uint32_t>& out) {
  out.clear();
  std::size_t at = 0;
  while (at <= text.size()) {
    const std::size_t comma = text.find(',', at);
    const std::string item = text.substr(at, comma == std::string::npos ? std::string::npos : comma - at);
    std::uint64_t k = 0;
    if (!parse_u64(item, k) || k == 0 || k > UINT32_MAX) {
      return false;
    }
    out.push_back(static_cast<std::uint32_t>(k));
    if (comma == std::string::npos) {
      break;
    }
    at = comma + 1;
  }
  return !out.empty();
}

[[nodiscard]] bool parse_flip(const std::string& s, FlipBit& out) {
  const std::size_t a = s.find(':');
  const std::size_t b = a == std::string::npos ? a : s.find(':', a + 1);
  if (b == std::string::npos) {
    return false;
  }
  std::uint64_t bit = 0;
  if (!parse_u64(s.substr(0, a), out.tick) || !parse_u64(s.substr(b + 1), bit) || bit > 31) {
    return false;
  }
  out.bit = static_cast<unsigned>(bit);
  const std::string field = s.substr(a + 1, b - a - 1);
  static constexpr const char* kFields[] = {"gx", "gy", "gz", "ax", "ay", "az", "temp"};
  for (int i = 0; i < 7; ++i) {
    if (field == kFields[i]) {
      out.field = i;
      return true;
    }
  }
  return false;
}

[[nodiscard]] bool parse_args(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    const std::string value = has_value ? argv[i + 1] : "";
    std::uint64_t u = 0;
    if (arg == "--seed" && has_value && parse_u64(value, u)) {
      o.seed = u;
    } else if (arg == "--ticks" && has_value && parse_u64(value, u) && u >= 1) {
      o.ticks = u;
    } else if (arg == "--period" && has_value) {
      const std::size_t slash = value.find('/');
      std::uint64_t num = 0;
      std::uint64_t den = 0;
      if (slash == std::string::npos || !parse_u64(value.substr(0, slash), num) ||
          !parse_u64(value.substr(slash + 1), den) || num > UINT32_MAX || den > UINT32_MAX) {
        return false;
      }
      o.num_us = static_cast<std::uint32_t>(num);
      o.den = static_cast<std::uint32_t>(den);
    } else if (arg == "--batch" && has_value) {
      o.batch_text = value;
    } else if (arg == "--set" && has_value) {
      o.sets.push_back(value);
    } else if (arg == "--flip-bit" && has_value) {
      FlipBit f{};
      if (!parse_flip(value, f)) {
        return false;
      }
      o.flip = f;
    } else if (arg == "--manifest" && has_value) {
      o.manifest = value;
    } else if (arg == "--trace-out" && has_value) {
      o.trace_out = value;
    } else if (arg == "--write-golden" && has_value) {
      o.write_golden = value;
    } else if (arg == "--note" && has_value) {
      o.note = value;
    } else {
      return false;
    }
    ++i;
  }
  return true;
}

[[nodiscard]] bool build_overrides(const Options& o, std::vector<marv_sil_param_override>& out) {
  if (o.sets.empty()) {
    return true;
  }
  const std::optional<std::string> manifest = read_file(o.manifest.c_str());
  if (!manifest) {
    std::fprintf(stderr, "cannot read manifest %s\n", o.manifest.c_str());
    return false;
  }
  for (const std::string& set : o.sets) {
    const std::size_t eq = set.find('=');
    if (eq == std::string::npos) {
      std::fprintf(stderr, "--set expects NAME=VALUE, got '%s'\n", set.c_str());
      return false;
    }
    const std::string name = set.substr(0, eq);
    const std::string value = set.substr(eq + 1);
    const std::optional<ManifestEntry> entry = find_param(*manifest, name);
    if (!entry) {
      std::fprintf(stderr, "unknown parameter '%s' (not in the manifest)\n", name.c_str());
      return false;
    }
    marv_sil_param_override ov{};
    ov.id = entry->id;
    char* end = nullptr;
    if (entry->is_f32) {
      ov.type = MARV_PARAM_F32;
      ov.f32 = std::strtof(value.c_str(), &end);
    } else {
      ov.type = MARV_PARAM_I32;
      ov.i32 = static_cast<std::int32_t>(std::strtol(value.c_str(), &end, 10));
    }
    if (value.empty() || *end != '\0') {
      std::fprintf(stderr, "bad value in --set %s\n", set.c_str());
      return false;
    }
    out.push_back(ov);
  }
  return true;
}

void apply_flip(const FlipBit& f, marv_imu_meas& m) {
  float* const fields[] = {&m.gyro_rad_s.x, &m.gyro_rad_s.y, &m.gyro_rad_s.z, &m.accel_m_s2.x,
                           &m.accel_m_s2.y, &m.accel_m_s2.z, &m.temp_k};
  float* const p = fields[f.field];
  std::uint32_t bits = 0;
  std::memcpy(&bits, p, sizeof bits);
  bits ^= std::uint32_t{1} << f.bit;
  std::memcpy(p, &bits, sizeof bits);
}

[[nodiscard]] int fail_status(const char* what, marv_sil_status s) {
  std::fprintf(stderr, "%s: %s (error index %u)\n", what, marv_sil_status_str(s), marv_sil_error_index());
  return 2;
}

[[nodiscard]] int fail_check(const char* what, std::uint64_t n) {
  std::fprintf(stderr, "check failed at tick %" PRIu64 ": %s\n", n, what);
  return 3;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  std::vector<std::uint32_t> pattern;
  if (!parse_args(argc, argv, opt) || !parse_batch(opt.batch_text, pattern) ||
      (!opt.write_golden.empty() && (opt.note.empty() || !opt.sets.empty() || opt.flip))) {
    std::fprintf(stderr, "usage error; see the header comment of sim/null_plant/src/main.cpp\n");
    return 1;
  }
  std::vector<marv_sil_param_override> overrides;
  if (!build_overrides(opt, overrides)) {
    return 1;
  }

  marv_sil_info info{};
  info.struct_size = sizeof info;
  if (const marv_sil_status s = marv_sil_info_get(&info); s != MARV_SIL_OK) {
    return fail_status("marv_sil_info_get", s);
  }

  marv_sil_config cfg{};
  cfg.struct_size = sizeof cfg;
  cfg.imu_meas_size = sizeof(marv_imu_meas);
  cfg.override_size = sizeof(marv_sil_param_override);
  cfg.tick_period_num_us = opt.num_us;
  cfg.tick_period_den = opt.den;
  cfg.n_overrides = static_cast<std::uint32_t>(overrides.size());
  cfg.overrides = overrides.empty() ? nullptr : overrides.data();
  cfg.param_schema_hash = info.param_schema_hash;
  if (const marv_sil_status s = marv_sil_init(&cfg); s != MARV_SIL_OK) {
    return fail_status("marv_sil_init", s);
  }

  std::uint32_t capacity = 0;
  for (const std::uint32_t k : pattern) {
    capacity = k > capacity ? k : capacity;
  }
  std::vector<std::uint64_t> t_us(capacity);
  std::vector<std::uint16_t> dshot(static_cast<std::size_t>(capacity) * info.n_motors);
  std::vector<std::uint16_t> servo(static_cast<std::size_t>(capacity) * info.n_servos);
  marv_sil_out out{};
  out.struct_size = sizeof out;
  out.capacity_ticks = capacity;
  out.t_us = t_us.data();
  out.dshot = dshot.data();
  out.servo_us = info.n_servos == 0 ? nullptr : servo.data();

  std::FILE* trace = nullptr;
  if (!opt.trace_out.empty()) {
    trace = std::fopen(opt.trace_out.c_str(), "w");
    if (trace == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", opt.trace_out.c_str());
      return 1;
    }
  }

  ImuSynth synth{opt.seed};
  Fnv1a64 hash;
  std::vector<marv_imu_meas> imu(capacity);
  std::uint64_t n = 0;
  std::size_t call = 0;
  int rc = 0;
  while (n < opt.ticks && rc == 0) {
    std::uint64_t k = pattern[call % pattern.size()];
    ++call;
    if (k > opt.ticks - n) {
      k = opt.ticks - n;
    }
    for (std::uint64_t i = 0; i < k; ++i) {
      imu[i] = synth.next();
      if (opt.flip && opt.flip->tick == n + i) {
        apply_flip(*opt.flip, imu[i]);
      }
    }
    if (const marv_sil_status s = marv_sil_tick(n, static_cast<std::uint32_t>(k), imu.data(), &out);
        s != MARV_SIL_OK) {
      rc = fail_status("marv_sil_tick", s);
      break;
    }
    for (std::uint64_t i = 0; i < k && rc == 0; ++i) {
      const std::uint64_t tick = n + i;
      if (t_us[i] != reference_stamp_us(opt.num_us, opt.den, tick)) {
        rc = fail_check("stamp differs from floor(n*num/den)", tick);
        break;
      }
      hash.u64(tick);
      hash.u64(t_us[i]);
      if (trace != nullptr) {
        std::fprintf(trace, "%" PRIu64 " %" PRIu64, tick, t_us[i]);
      }
      for (std::uint32_t m = 0; m < info.n_motors; ++m) {
        const std::uint16_t v = dshot[(i * info.n_motors) + m];
        if (!dshot_value_ok(v)) {
          rc = fail_check("DShot value is neither 0 nor in 48..2047", tick);
          break;
        }
        hash.u16(v);
        if (trace != nullptr) {
          std::fprintf(trace, " %u", static_cast<unsigned>(v));
        }
      }
      for (std::uint32_t s = 0; s < info.n_servos; ++s) {
        const std::uint16_t v = servo[(i * info.n_servos) + s];
        hash.u16(v);
        if (trace != nullptr) {
          std::fprintf(trace, " %u", static_cast<unsigned>(v));
        }
      }
      if (trace != nullptr) {
        std::fputc('\n', trace);
      }
    }
    n += k;
  }
  if (trace != nullptr) {
    std::fclose(trace);
  }
  if (rc != 0) {
    return rc;
  }
  if (const marv_sil_status s = marv_sil_shutdown(); s != MARV_SIL_OK) {
    return fail_status("marv_sil_shutdown", s);
  }

  char hex[17];
  std::snprintf(hex, sizeof hex, "%016" PRIx64, hash.value());
  if (opt.write_golden.empty()) {
    std::printf("%s\n", hex);
    return 0;
  }
  std::FILE* g = std::fopen(opt.write_golden.c_str(), "w");
  if (g == nullptr) {
    std::fprintf(stderr, "cannot write %s\n", opt.write_golden.c_str());
    return 1;
  }
  std::fprintf(g,
               "# L0 T2 golden trace hash (FNV-1a 64 over the per-tick trace). Scenario values (labelled, core 2): "
               "seed=0x%" PRIx64 " ticks=%" PRIu64 " period=%u/%u us batch=%s composition=%s. %s\n%s\n",
               opt.seed, opt.ticks, opt.num_us, opt.den, opt.batch_text.c_str(), info.composition, opt.note.c_str(),
               hex);
  std::fclose(g);
  return 0;
}
