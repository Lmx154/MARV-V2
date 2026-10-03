// L6 stage (c), commit 3 (decision 0014, "Wiring"): the acro replay (tests/regression/quad/L04/replay/l4_acro_replay.cpp)
// and the l4_rate_scripted composition run the one shared rate-group step (fw/rate_group): the gyro chain on every tick,
// the notches updated at each rate execution, the rate loop on the chain output. The replay must reproduce the
// composition bit for bit on the composition's own run.
//
// Run. The composition through the SIL C ABI: marv_sil_init with the overrides below, then marv_sil_tick over kTicks
// ticks of the input below. Its overrides, the stamps it gave the ticks and its samples are written in the replay's input
// format (as tools/sim/run_l4.py replay_input writes them from a logged run), and the replay binary runs on them.
// Check. One replay row per rate execution (ticks 0, D, 2D, ..., D = rate_loop_divisor), no fault, and dshot1..dshot4 of
// each row equal to the composition's DShot at that tick.
// Control. The same replay on the input with every non-rate tick's gyro replaced by the preceding rate tick's. A replay
// that skipped the non-rate ticks (the replay before decision 0014) could not see this change; this one must, and must
// then disagree with the composition.
//
// Input (scenario test values, none a vehicle number): the hover collective, mass x the WGS 84 equatorial normal gravity
// of constants.hpp (NIMA TR8350.2 Table 3.4), as unit_l4_composition takes it; one setpoint segment from stamp 0 at
// kSetpointFraction of each axis's rate_max; per tick n and axis a the gyro g_a(n) = A_a sin(w_a n) + B (-1)^n: a slow
// tone the rate loop answers, plus a tone at the tick Nyquist frequency that the chain's low-pass attenuates and that the
// rate ticks alone would see as a constant.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/actuator.hpp>

namespace {

using namespace marv;

constexpr std::size_t kMotors = kQuadXMotors;
using Dshots = std::array<std::uint16_t, kMotors>;
constexpr std::uint32_t kGyroValid = 1u << MARV_IMU_GYRO_VALID;
constexpr std::size_t kAxes = 3;

// Scenario test values. 4096 ticks x 156.25 us = 0.64 s.
constexpr std::uint32_t kTicks = 4096;
constexpr std::array<double, kAxes> kToneAmpRadS{0.5, -0.3, 0.2};
constexpr std::array<double, kAxes> kToneRadPerTick{0.003, 0.005, 0.007};
constexpr double kNyquistAmpRadS = 0.25;
constexpr float kSetpointFraction = 0.25F;

// The replay's output columns (l4_acro_replay.cpp, Output): dshot1..dshot4 then fault after 18 leading columns.
constexpr std::size_t kTickColumn = 1;
constexpr std::size_t kDshotColumn = 18;
constexpr std::size_t kFaultColumn = kDshotColumn + kMotors;
constexpr std::size_t kColumns = kFaultColumn + 1;

struct Override {
  ParamId id;
  bool is_f32;
  float f32;
  std::int32_t i32;
};

struct SilRun {
  std::vector<Override> overrides;
  std::uint32_t divisor = 0;
  std::vector<marv_imu_meas> imu;
  std::vector<std::uint64_t> t_us;
  std::vector<Dshots> dshot;  // per tick
};

marv_imu_meas sample_at(std::uint32_t n) {
  const double nd = static_cast<double>(n);
  const double alt = (n % 2 == 0) ? kNyquistAmpRadS : -kNyquistAmpRadS;
  std::array<float, kAxes> g{};
  for (std::size_t a = 0; a < kAxes; ++a) {
    g[a] = static_cast<float>(kToneAmpRadS[a] * std::sin(kToneRadPerTick[a] * nd) + alt);
  }
  marv_imu_meas m{};
  m.gyro_rad_s = {g[0], g[1], g[2]};
  m.flags = kGyroValid;
  return m;
}

// The composition's run, once per process (marv_sil_init succeeds once per process).
const SilRun& composition_run() {
  static const SilRun run = [] {
    SilRun r;
    if (!params_init(param_defaults())) {
      ADD_FAILURE() << "params_init";
      return r;
    }
    const float hover = param_value<ParamId::mass>() * static_cast<float>(prim::kWgs84GammaE);
    r.overrides = {
        Override{ParamId::l4_thrust_n, true, hover, 0},
        Override{ParamId::l4_seg_count, false, 0.0F, 1},
        Override{ParamId::l4_seg1_t_us, false, 0.0F, 0},
        Override{ParamId::l4_seg1_roll, true, kSetpointFraction * param_value<ParamId::rate_max_roll>(), 0},
        Override{ParamId::l4_seg1_pitch, true, kSetpointFraction * param_value<ParamId::rate_max_pitch>(), 0},
        Override{ParamId::l4_seg1_yaw, true, kSetpointFraction * param_value<ParamId::rate_max_yaw>(), 0}};
    r.divisor = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());

    std::vector<marv_sil_param_override> ov;
    for (const Override& o : r.overrides) {
      marv_sil_param_override s{};
      s.id = static_cast<std::uint32_t>(o.id);
      s.type = o.is_f32 ? MARV_PARAM_F32 : MARV_PARAM_I32;
      s.f32 = o.f32;
      s.i32 = o.i32;
      ov.push_back(s);
    }
    marv_sil_config c{};
    c.struct_size = sizeof(marv_sil_config);
    c.imu_meas_size = sizeof(marv_imu_meas);
    c.override_size = sizeof(marv_sil_param_override);
    c.tick_period_num_us = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
    c.tick_period_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
    c.n_overrides = static_cast<std::uint32_t>(ov.size());
    c.overrides = ov.data();
    c.param_schema_hash = kParamSchemaHash;
    if (marv_sil_init(&c) != MARV_SIL_OK) {
      ADD_FAILURE() << "marv_sil_init";
      return r;
    }

    for (std::uint32_t n = 0; n < kTicks; ++n) {
      r.imu.push_back(sample_at(n));
    }
    r.t_us.assign(kTicks, 0);
    std::vector<std::uint16_t> d(static_cast<std::size_t>(kTicks) * kMotors, 0);
    marv_sil_out out{};
    out.struct_size = sizeof(marv_sil_out);
    out.capacity_ticks = kTicks;
    out.t_us = r.t_us.data();
    out.dshot = d.data();
    out.servo_us = nullptr;
    if (marv_sil_tick(0, kTicks, r.imu.data(), &out) != MARV_SIL_OK) {
      ADD_FAILURE() << "marv_sil_tick";
      return r;
    }
    r.dshot.resize(kTicks);
    for (std::size_t i = 0; i < kTicks; ++i) {
      for (std::size_t m = 0; m < kMotors; ++m) {
        r.dshot[i][m] = d[(i * kMotors) + m];
      }
    }
    return r;
  }();
  return run;
}

std::string float_text(float x) {
  std::array<char, 32> buf{};
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), x);
  return std::string(buf.data(), r.ptr);
}

std::string hex(float x) {
  std::array<char, 16> buf{};
  std::snprintf(buf.data(), buf.size(), "%08x", std::bit_cast<std::uint32_t>(x));
  return std::string(buf.data());
}

// The replay's input text (tools/sim/run_l4.py replay_input): every override, then every tick's stamp and sample.
std::string replay_input(const SilRun& r, const std::vector<marv_imu_meas>& imu) {
  std::ostringstream s;
  for (const Override& o : r.overrides) {
    s << "override " << param_name(o.id) << (o.is_f32 ? " f32 " + float_text(o.f32) : " i32 " + std::to_string(o.i32))
      << '\n';
  }
  for (std::size_t n = 0; n < imu.size(); ++n) {
    const marv_imu_meas& m = imu[n];
    s << "tick " << n << ' ' << r.t_us[n] << ' ' << hex(m.gyro_rad_s.x) << ' ' << hex(m.gyro_rad_s.y) << ' '
      << hex(m.gyro_rad_s.z) << ' ' << hex(m.accel_m_s2.x) << ' ' << hex(m.accel_m_s2.y) << ' ' << hex(m.accel_m_s2.z)
      << ' ' << hex(m.temp_k) << ' ' << m.flags << '\n';
  }
  return s.str();
}

struct Row {
  std::uint64_t tick = 0;
  Dshots dshot{};
  int fault = -1;
};

struct Dir {
  std::filesystem::path path;
  Dir() {
    std::string tmpl = (std::filesystem::temp_directory_path() / "l4_replay_parity_XXXXXX").string();
    EXPECT_NE(mkdtemp(tmpl.data()), nullptr);
    path = tmpl;
  }
  ~Dir() { std::filesystem::remove_all(path); }
};

// fork + exec of the replay binary on `input`; its rows, or an empty vector (with a test failure) on any error.
std::vector<Row> replay(const std::string& input) {
  const Dir dir;
  const std::string in_path = (dir.path / "input.txt").string();
  const std::string out_path = (dir.path / "output.txt").string();
  {
    std::ofstream f(in_path);
    f << input;
  }
  const pid_t pid = fork();
  if (pid == 0) {
    execl(MARV_L4_ACRO_REPLAY, MARV_L4_ACRO_REPLAY, in_path.c_str(), out_path.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  EXPECT_EQ(waitpid(pid, &status, 0), pid);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << "l4_acro_replay status " << status;
  std::vector<Row> rows;
  std::ifstream f(out_path);
  std::string line;
  if (!std::getline(f, line)) {
    ADD_FAILURE() << "no replay output";
    return rows;
  }
  while (std::getline(f, line)) {
    std::istringstream words(line);
    std::vector<std::string> w;
    for (std::string x; words >> x;) {
      w.push_back(x);
    }
    if (w.size() != kColumns) {
      ADD_FAILURE() << "replay row has " << w.size() << " columns: " << line;
      return {};
    }
    Row r;
    r.tick = std::stoull(w[kTickColumn]);
    for (std::size_t m = 0; m < kMotors; ++m) {
      r.dshot[m] = static_cast<std::uint16_t>(std::stoul(w[kDshotColumn + m]));
    }
    r.fault = std::stoi(w[kFaultColumn]);
    rows.push_back(r);
  }
  return rows;
}

// The number of rate executions whose replayed DShot differs from the composition's at that tick; -1 if the rows are
// not one per rate execution in order.
long mismatches(const SilRun& r, const std::vector<Row>& rows) {
  const std::size_t executions = (kTicks + r.divisor - 1) / r.divisor;
  if (r.divisor == 0 || rows.size() != executions) {
    return -1;
  }
  long count = 0;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    if (rows[k].tick != k * r.divisor) {
      return -1;
    }
    if (rows[k].dshot != r.dshot[rows[k].tick]) {
      ++count;
    }
  }
  return count;
}

TEST(L6ReplayParity, TheReplayReproducesTheCompositionsDshotAtEveryRateExecution) {
  const SilRun& r = composition_run();
  ASSERT_EQ(r.dshot.size(), kTicks);
  const std::vector<Row> rows = replay(replay_input(r, r.imu));
  EXPECT_EQ(mismatches(r, rows), 0);
  for (const Row& row : rows) {
    EXPECT_EQ(row.fault, 0) << "tick " << row.tick;
  }
  // The run is not trivial: the command moves (the rate loop answers the setpoint and the gyro).
  bool moved = false;
  for (const Dshots& d : r.dshot) {
    moved = moved || d != r.dshot.front();
  }
  EXPECT_TRUE(moved);
}

TEST(L6ReplayParity, ControlNonRateTickSamplesChangedBreaksTheMatch) {
  const SilRun& r = composition_run();
  ASSERT_EQ(r.dshot.size(), kTicks);
  ASSERT_GE(r.divisor, 2U);
  std::vector<marv_imu_meas> changed = r.imu;
  for (std::size_t n = 0; n < changed.size(); ++n) {
    if (n % r.divisor != 0) {
      changed[n] = r.imu[n - (n % r.divisor)];
    }
  }
  const std::vector<Row> rows = replay(replay_input(r, changed));
  EXPECT_GT(mismatches(r, rows), 0);
}

}  // namespace
