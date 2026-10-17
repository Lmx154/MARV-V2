// The FC3v2 IMU driver and tick loop (marv/hal_target/imu.hpp). Register and field values: boards/parts/lsm6dsv.hpp
// (ST DS13476 Rev 5). Pins and the mounting map: boards/madflight_fc3v2_revc.h.
//
// SPI1 in mode 3 (DS13476 §5.1.3), the clock at most fc(SPC) max (Table 6), CS driven as a GPIO. Each FIFO word is
// read in its own transfer from FIFO_DATA_OUT_TAG (78h), the procedure of §6.10.8 (the datasheet does not state what a
// burst past 7Eh reads). The LSM6DSV has no clock input, and its ODR-triggered mode excludes 7.68 kHz (§6.6): it runs
// on its internal oscillator, and GPIO 26 (the board's IMU_CLKIN net) is not driven.
//
// Watermark: one sample, 2 FIFO words (one gyroscope and one accelerometer word, the two sensors batched at the same
// rate); WTM counts words (§9.5 Table 33). INT1 is FIFO_WTM_IA, high while the FIFO holds at least that many words
// (§6.10.3), taken as a level interrupt: a handler that leaves a full sample behind runs again, so none is stranded.
// Temperature: OUT_TEMP (20h-21h) read when STATUS_REG TDA is set (§9.28), and carried by the samples after it.
//
// Stale IMU (decision 0040 Q2). The read deadline is D = FIFO depth / ODR (core EMB-6; imu_read_deadline_us). (a) A FIFO
// overrun seen at an interrupt entry increments the deadline-miss count (hal_imu_deadline_misses) before the batch is
// delivered. (b) A hardware alarm is re-armed at every delivered sample for D later; if it expires, or if an interrupt
// entry finds that D has passed since the last delivery (a stuck-high INT1 line starves a lower-priority alarm, and the
// entry check needs no alarm), hal_panic stops the motors with stop frames. The alarm's priority equals the IMU
// interrupt's, so it never preempts the tick: the panic path is the only other writer of the motor output.
#include <array>
#include <atomic>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/spi.h>
#include <hardware/timer.h>
#include <pico.h>

#include <parts/lsm6dsv.hpp>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/bench_stats.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/lsm6dsv_decode.hpp"
#include "marv/hal_target/motor_output.hpp"

namespace marv::hal_target {
namespace {

constexpr unsigned kCsPin = MADFLIGHT_FC3V2_IMU_CS_PIN;
constexpr unsigned kIntPin = MADFLIGHT_FC3V2_IMU_INT_PIN;

constexpr ImuMounting kMounting{
    {MADFLIGHT_FC3V2_IMU_FRD_X_CHIP_AXIS, MADFLIGHT_FC3V2_IMU_FRD_Y_CHIP_AXIS, MADFLIGHT_FC3V2_IMU_FRD_Z_CHIP_AXIS},
    {MADFLIGHT_FC3V2_IMU_FRD_X_SIGN, MADFLIGHT_FC3V2_IMU_FRD_Y_SIGN, MADFLIGHT_FC3V2_IMU_FRD_Z_SIGN}};

// FIFO words per sample: one gyroscope and one accelerometer word.
constexpr std::uint8_t kWatermarkWords = 2;
static_assert(kWatermarkWords == lsm6dsv::kFifoWordsPerSample, "the deadline's words per sample is the driver's batching");
// The largest DIFF_FIFO_[8:0] value (a 9-bit field): the most words one handler entry can be told to read.
constexpr std::size_t kMaxWords = (std::size_t{1} << (lsm6dsv::kDiffFifo8Shift + 1)) - 1;
// FIFO_STATUS1 (1Bh) through STATUS_REG (1Eh), one burst.
constexpr std::size_t kStatusBytes = lsm6dsv::kRegStatusReg - lsm6dsv::kRegFifoStatus1 + 1;
constexpr std::size_t kTempBytes = 2;
// A transfer: the command byte, then at most one FIFO word.
constexpr std::size_t kMaxTransfer = 1 + lsm6dsv::kFifoWordBytes;

struct RegWrite {
  std::uint8_t reg;
  std::uint8_t value;
};

// After the software reset, in order; each is read back. The FIFO stays in bypass (empty) until the gyroscope's
// turn-on time has passed.
constexpr std::array kConfig{
    RegWrite{lsm6dsv::kRegCtrl3, static_cast<std::uint8_t>(lsm6dsv::kCtrl3Bdu | lsm6dsv::kCtrl3IfInc)},
    RegWrite{lsm6dsv::kRegCtrl6, lsm6dsv::kCtrl6Fs2000Dps},
    RegWrite{lsm6dsv::kRegCtrl8, lsm6dsv::kCtrl8Fs16G},
    RegWrite{lsm6dsv::kRegFifoCtrl1, kWatermarkWords},
    RegWrite{lsm6dsv::kRegFifoCtrl3, lsm6dsv::kFifoCtrl3Bdr7680Hz},
    RegWrite{lsm6dsv::kRegFifoCtrl4, lsm6dsv::kFifoCtrl4Bypass},
    RegWrite{lsm6dsv::kRegInt1Ctrl, lsm6dsv::kInt1FifoTh},
    RegWrite{lsm6dsv::kRegCtrl1, lsm6dsv::kCtrl1Xl7680HzHighPerformance},
    RegWrite{lsm6dsv::kRegCtrl2, lsm6dsv::kCtrl2G7680HzHighPerformance},
};

spi_inst_t* imu_spi() noexcept { return SPI_INSTANCE(MADFLIGHT_FC3V2_IMU_SPI); }

// Owned by the interrupt once it is enabled (imu_start sets them up before).
ImuSink g_sink = nullptr;
RateCountsSource g_rate_counts = nullptr;
Lsm6dsvAssembler g_assembler{kMounting, lsm6dsv_scales()};
ImuStamper g_stamper{lsm6dsv::kOdrHz};
std::array<FifoWord, kMaxWords> g_words{};
ImuStats g_acc{};
TimeUs g_prev_t = 0;
bool g_have_prev = false;

// The seqlock: g_seq is odd while the interrupt writes g_published.
std::atomic<std::uint32_t> g_seq{0};
ImuStats g_published{};
std::atomic<bool> g_window_reset{false};

// The read deadline D in µs, the count of misses, the low 32 bits of hal_time_us() at the last delivery (or at arming) and the
// claimed hardware alarm. g_watch is set before the alarm is armed; the IMU and alarm handlers share a priority, so neither
// preempts the other.
std::uint32_t g_deadline_us = 0;
std::uint32_t g_last_delivery = 0;
unsigned g_alarm = 0;
bool g_watch = false;
std::atomic<std::uint32_t> g_deadline_misses{0};

[[noreturn]] void deadline_panic(std::uint32_t elapsed_us, const char* where) noexcept {
  static constexpr char kFormat[] =
      "IMU read deadline missed: %lu us without a delivered sample, limit D = %lu us = %lu samples of FIFO depth (INFERRED) "
      "x tick period (EMB-6), found by the %s";
  constexpr std::size_t kNumber = sizeof("4294967295");  // the widest of the three numbers
  static std::array<char, sizeof(kFormat) + kNumber + kNumber + kNumber + sizeof("interrupt entry")> msg{};
  static_cast<void>(std::snprintf(msg.data(), msg.size(), kFormat, static_cast<unsigned long>(elapsed_us),
                                  static_cast<unsigned long>(g_deadline_us), static_cast<unsigned long>(kFifoDepthSamples),
                                  where));
  hal_panic(msg.data());
}

// Re-arms the alarm for D after now and records now as the last delivery. A pending expiry from the earlier target is
// cleared; one that still reaches the handler finds the deadline not passed and returns.
void rearm_alarm() noexcept {
  const std::uint32_t now = timer_hw->timerawl;
  g_last_delivery = now;
  timer_hw->intr = 1U << g_alarm;
  timer_hw->alarm[g_alarm] = now + g_deadline_us;
}

void on_alarm() {
  timer_hw->intr = 1U << g_alarm;
  const std::uint32_t elapsed = timer_hw->timerawl - g_last_delivery;
  if (g_watch && elapsed >= g_deadline_us) {
    deadline_panic(elapsed, "alarm");
  }
}

void transfer(std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx) noexcept {
  gpio_put(kCsPin, false);
  static_cast<void>(spi_write_read_blocking(imu_spi(), tx.data(), rx.data(), tx.size()));
  gpio_put(kCsPin, true);
}

void read_regs(std::uint8_t reg, std::span<std::uint8_t> out) noexcept {
  std::array<std::uint8_t, kMaxTransfer> tx{};
  std::array<std::uint8_t, kMaxTransfer> rx{};
  tx[0] = static_cast<std::uint8_t>(reg | lsm6dsv::kSpiReadFlag);
  const std::size_t n = out.size() + 1;
  transfer(std::span{tx}.first(n), std::span{rx}.first(n));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = rx[i + 1];
  }
}

[[nodiscard]] std::uint8_t read_reg(std::uint8_t reg) noexcept {
  std::array<std::uint8_t, 1> v{};
  read_regs(reg, v);
  return v[0];
}

void write_reg(std::uint8_t reg, std::uint8_t value) noexcept {
  const std::array<std::uint8_t, 2> tx{reg, value};
  std::array<std::uint8_t, 2> rx{};
  transfer(tx, rx);
}

void publish() noexcept {
  g_acc.actuator_writes = actuator_writes();
  g_acc.actuator_drops = motor_frames_dropped();
  g_acc.rate = g_rate_counts();
  const std::uint32_t q = g_seq.load(std::memory_order_relaxed);
  g_seq.store(q + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  g_published = g_acc;
  g_seq.store(q + 2, std::memory_order_release);
}

void deliver(ImuSample& s, TimeUs t) noexcept {
  rearm_alarm();
  s.t_us = t;
  if (g_have_prev) {
    if (t <= g_prev_t) {
      ++g_acc.order_faults;
    } else {
      const TimeUs d = t - g_prev_t;
      if (g_acc.spacing_min_us == 0 || d < g_acc.spacing_min_us) {
        g_acc.spacing_min_us = d;
      }
      if (d > g_acc.spacing_max_us) {
        g_acc.spacing_max_us = d;
      }
    }
  }
  g_prev_t = t;
  g_have_prev = true;
  const std::uint32_t both = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  if ((s.flags & both) != both) {
    ++g_acc.partial;
  }
  ++g_acc.samples;
  g_acc.last = s;
  bench::stage(bench::Timed::Bench);
  bench::still_sample(s);
  bench::stage(bench::Timed::Decode);
  g_sink(s);
  bench::stage(bench::Timed::Decode);
  ++g_acc.ticks;
}

// INT1 (FIFO watermark), level high. The stamping rule is lsm6dsv_decode.hpp's.
void on_int1() {
  bench::irq_begin();
  if ((gpio_get_irq_event_mask(kIntPin) & GPIO_IRQ_LEVEL_HIGH) == 0) {
    bench::irq_end(0);
    return;
  }
  const TimeUs t_irq = hal_time_us();
  if (g_watch && static_cast<std::uint32_t>(t_irq) - g_last_delivery >= g_deadline_us) {
    deadline_panic(static_cast<std::uint32_t>(t_irq) - g_last_delivery, "interrupt entry");
  }
  ++g_acc.irqs;
  if (g_window_reset.exchange(false, std::memory_order_relaxed)) {
    g_acc.spacing_min_us = 0;
    g_acc.spacing_max_us = 0;
  }

  std::array<std::uint8_t, kStatusBytes> st{};
  read_regs(lsm6dsv::kRegFifoStatus1, st);
  const std::uint8_t status2 = st[lsm6dsv::kRegFifoStatus2 - lsm6dsv::kRegFifoStatus1];
  if ((status2 & lsm6dsv::kFifoStatus2OvrLatched) != 0) {
    ++g_acc.fifo_overflows;
    g_deadline_misses.fetch_add(1, std::memory_order_relaxed);
  }
  if ((st[lsm6dsv::kRegStatusReg - lsm6dsv::kRegFifoStatus1] & lsm6dsv::kStatusTda) != 0) {
    std::array<std::uint8_t, kTempBytes> t{};
    read_regs(lsm6dsv::kRegOutTempL, t);
    g_assembler.set_temperature(static_cast<std::int16_t>(static_cast<std::uint16_t>(t[0] | (t[1] << CHAR_BIT))));
  }
  const std::size_t n = static_cast<std::size_t>(st[0]) |
                        (static_cast<std::size_t>(status2 & lsm6dsv::kFifoStatus2DiffFifo8) << lsm6dsv::kDiffFifo8Shift);
  if (n == 0) {
    ++g_acc.empty_irqs;
    bench::stage(bench::Timed::Publish);
    publish();
    bench::irq_end(0);
    return;
  }
  for (std::size_t i = 0; i < n; ++i) {
    read_regs(lsm6dsv::kRegFifoDataOutTag, g_words[i]);
  }
  bench::stage(bench::Timed::Decode);
  g_acc.words += n;

  // The batch's sample count k first (on a copy of the assembler), then the samples, oldest first.
  std::size_t k = 0;
  {
    Lsm6dsvAssembler probe = g_assembler;
    ImuSample unused{};
    for (std::size_t i = 0; i < n; ++i) {
      k += probe.push(g_words[i], unused) == WordResult::Sample ? 1 : 0;
    }
  }
  std::size_t j = 0;
  for (std::size_t i = 0; i < n; ++i) {
    ImuSample s{};
    switch (g_assembler.push(g_words[i], s)) {
      case WordResult::Held:
        break;
      case WordResult::BadTag:
        ++g_acc.spi_errors;
        break;
      case WordResult::Sample:
        deliver(s, g_stamper.stamp(t_irq, k - 1 - j));
        ++j;
        break;
    }
  }
  g_acc.stamp_clamps = g_stamper.clamps();
  bench::stage(bench::Timed::Publish);
  publish();
  bench::irq_end(j);
}

}  // namespace

ImuBoot imu_start(ImuSink sink, RateCountsSource rate_counts, TickPeriod design_period) noexcept {
  if (sink == nullptr || rate_counts == nullptr) {
    hal_panic("imu_start: no sink or no rate-counts source");
  }
  if (!valid(kMounting)) {
    hal_panic("FC3v2 board header: the IMU mounting map is not a signed permutation of the chip axes");
  }
  g_sink = sink;
  g_rate_counts = rate_counts;

  ImuBoot boot;
  boot.spi_hz = spi_init(imu_spi(), lsm6dsv::kSpiClockMaxHz);
  spi_set_format(imu_spi(), CHAR_BIT, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(MADFLIGHT_FC3V2_IMU_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(MADFLIGHT_FC3V2_IMU_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(MADFLIGHT_FC3V2_IMU_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_init(kCsPin);
  gpio_put(kCsPin, true);
  gpio_set_dir(kCsPin, GPIO_OUT);
  gpio_init(kIntPin);
  gpio_set_dir(kIntPin, GPIO_IN);

  const std::uint8_t who = read_reg(lsm6dsv::kRegWhoAmI);
  if (who != lsm6dsv::kWhoAmI) {
    static constexpr char kFormat[] = "LSM6DSV: WHO_AM_I (0Fh) read 0x%02x, expected 0x%02x (SPI1, CS GPIO %u)";
    static std::array<char, sizeof(kFormat)> msg{};
    static_cast<void>(std::snprintf(msg.data(), msg.size(), kFormat, static_cast<unsigned>(who),
                                    static_cast<unsigned>(lsm6dsv::kWhoAmI), kCsPin));
    hal_panic(msg.data());
  }

  write_reg(lsm6dsv::kRegCtrl3, lsm6dsv::kCtrl3SwReset);
  const TimeUs reset_deadline = hal_time_us() + lsm6dsv::kGyroTurnOnUs;
  while ((read_reg(lsm6dsv::kRegCtrl3) & lsm6dsv::kCtrl3SwReset) != 0) {
    if (hal_time_us() > reset_deadline) {
      hal_panic("LSM6DSV: the software reset (CTRL3 SW_RESET) did not clear within the gyroscope turn-on time");
    }
  }
  // DS13476 states no time from SW_RESET clearing to the registers taking writes. On the bench (2026-10-16) a write
  // issued as the bit cleared was lost and one issued Ton later held, so the driver waits Ton (Table 4) here; each
  // write below is read back.
  busy_wait_us(lsm6dsv::kGyroTurnOnUs);
  for (const RegWrite& w : kConfig) {
    write_reg(w.reg, w.value);
    const std::uint8_t got = read_reg(w.reg);
    if (got != w.value) {
      static constexpr char kFormat[] = "LSM6DSV: register 0x%02x read back 0x%02x after writing 0x%02x";
      static std::array<char, sizeof(kFormat)> msg{};
      static_cast<void>(std::snprintf(msg.data(), msg.size(), kFormat, static_cast<unsigned>(w.reg),
                                      static_cast<unsigned>(got), static_cast<unsigned>(w.value)));
      hal_panic(msg.data());
    }
  }
  busy_wait_us(lsm6dsv::kGyroTurnOnUs);
  write_reg(lsm6dsv::kRegFifoCtrl4, lsm6dsv::kFifoCtrl4Continuous);
  boot.freq_fine = static_cast<std::int8_t>(read_reg(lsm6dsv::kRegInternalFreqFine));
  boot.odr_hz_by_trim = static_cast<double>(lsm6dsv::kOdrHz) * (1.0 + lsm6dsv::kFreqFineStep * boot.freq_fine);
  const TickPeriodUs trim_period = tick_period_from_trim(boot.freq_fine);
  if (!same_period(design_period.num, design_period.den, trim_period.num, trim_period.den)) {
    static constexpr char kFormat[] =
        "LSM6DSV: the rate design was checked at a tick period of %ld/%ld us, but this part's FREQ_FINE %d gives %lld/%lld us "
        "(DS13476 9.49)";
    constexpr std::size_t kNumber = sizeof("-9223372036854775808");  // the widest of the five numbers
    static std::array<char, sizeof(kFormat) + kNumber + kNumber + kNumber + kNumber + kNumber> msg{};
    static_cast<void>(std::snprintf(msg.data(), msg.size(), kFormat, static_cast<long>(design_period.num),
                                    static_cast<long>(design_period.den), static_cast<int>(boot.freq_fine),
                                    static_cast<long long>(trim_period.num), static_cast<long long>(trim_period.den)));
    hal_panic(msg.data());
  }

  // The tick source runs (FIFO continuous, above): claim the alarm, arm it for D and only then enable the interrupts.
  const TimeUs deadline = imu_read_deadline_us(trim_period.num, trim_period.den);
  if (deadline < 1 || deadline >= std::numeric_limits<std::uint32_t>::max()) {
    hal_panic("LSM6DSV: the IMU read deadline D = FIFO depth / ODR is not a positive 32-bit microsecond count");
  }
  g_deadline_us = static_cast<std::uint32_t>(deadline);
  g_alarm = static_cast<unsigned>(hardware_alarm_claim_unused(true));
  const unsigned alarm_irq = hardware_alarm_get_irq_num(g_alarm);
  irq_set_exclusive_handler(alarm_irq, &on_alarm);
  irq_set_priority(alarm_irq, PICO_HIGHEST_IRQ_PRIORITY);
  hw_set_bits(&timer_hw->inte, 1U << g_alarm);
  g_watch = true;
  rearm_alarm();
  irq_set_enabled(alarm_irq, true);

  gpio_add_raw_irq_handler(kIntPin, &on_int1);
  gpio_set_irq_enabled(kIntPin, GPIO_IRQ_LEVEL_HIGH, true);
  irq_set_priority(IO_IRQ_BANK0, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(IO_IRQ_BANK0, true);
  return boot;
}

ImuStats imu_stats() noexcept {
  ImuStats s{};
  for (;;) {
    const std::uint32_t q = g_seq.load(std::memory_order_acquire);
    if ((q & 1U) != 0) {
      continue;
    }
    s = g_published;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (g_seq.load(std::memory_order_relaxed) == q) {
      break;
    }
  }
  g_window_reset.store(true, std::memory_order_relaxed);
  return s;
}

}  // namespace marv::hal_target

namespace marv {

std::uint32_t hal_imu_deadline_misses() noexcept {
  return hal_target::g_deadline_misses.load(std::memory_order_relaxed);
}

}  // namespace marv
