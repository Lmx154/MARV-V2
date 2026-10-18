// The SD memory in SPI mode on SPI0 and the FatFs disk I/O functions over it (sd_spi.hpp). Commands, tokens, clocks and
// timeouts are the SD Physical Layer Simplified Specification 9.10's, in constants.hpp (kSd*). Polled and blocking; no
// interrupt is disabled or configured; every wait is bounded by hal_time_us. The data CRC16 is neither sent (CRC off, the
// SPI mode default, section 7.2.2) nor checked: the files on the card verify themselves (SHA-256), FatFs's own metadata
// reads do not.
#include "marv/hal_target/sd_spi.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/timer.h>

#include "marv/hal/hal.hpp"
#include "marv/prim/constants.hpp"

extern "C" {
#include "ff.h"
#include "diskio.h"
}

namespace marv::hal_target::sd {
namespace {

using namespace marv::prim;

constexpr unsigned kSck = MADFLIGHT_FC3V2_SD_SCK_PIN;
constexpr unsigned kMosi = MADFLIGHT_FC3V2_SD_MOSI_PIN;
constexpr unsigned kMiso = MADFLIGHT_FC3V2_SD_MISO_PIN;
constexpr unsigned kCs = MADFLIGHT_FC3V2_SD_CS_PIN;
constexpr unsigned kDetect = MADFLIGHT_FC3V2_SD_DETECT_PIN;
static_assert(MADFLIGHT_FC3V2_SD_SPI == 0, "the SD driver uses spi0");

constexpr std::uint64_t ms_to_us(std::uint32_t ms) noexcept {
  return static_cast<std::uint64_t>(ms) * kMicrosecondsPerMillisecond;
}
constexpr std::uint64_t kReadTimeoutUs = ms_to_us(kSdReadTimeoutMs);
// Section 4.6.2.2 asks for more than the 500 ms SDXC busy maximum: twice it.
constexpr std::uint64_t kWriteTimeoutUs = 2 * ms_to_us(kSdWriteBusyMaxMs);
// Section 4.2.3 asks for more than 1 s from the first ACMD41: twice it.
constexpr std::uint64_t kInitTimeoutUs = 2 * ms_to_us(kSdAcmd41MinTimeoutMs);
constexpr std::uint64_t kPowerUpUs = ms_to_us(kSdPowerUpMs);

// CRC7 of section 4.5 (x^7 + x^3 + 1), most significant bit first.
constexpr std::uint8_t crc7(const std::uint8_t* data, std::size_t n) noexcept {
  unsigned crc = 0;
  const unsigned mask = (1U << kSdCrc7Bits) - 1;
  for (std::size_t i = 0; i < n; ++i) {
    for (unsigned b = 0; b < kBitsPerByte; ++b) {
      const unsigned in = (static_cast<unsigned>(data[i]) >> (kBitsPerByte - 1 - b)) & 1U;
      const unsigned top = (crc >> (kSdCrc7Bits - 1)) & 1U;
      crc = (crc << 1) & mask;
      if (in != top) {
        crc ^= kSdCrc7Poly;
      }
    }
  }
  return static_cast<std::uint8_t>(crc);
}
constexpr std::uint8_t last_byte(const std::uint8_t* five) noexcept {
  return static_cast<std::uint8_t>((crc7(five, kSdCommandBytes - 1) << 1) | kSdCommandEnd);
}
// Section 7.2.2 gives the CMD0 frame: the CRC7 here reproduces its last byte.
constexpr std::array<std::uint8_t, kSdCommandBytes - 1> kCmd0Head{kSdCommandStart | kSdCmdGoIdle, 0, 0, 0, 0};
static_assert(last_byte(kCmd0Head.data()) == kSdCmd0Frame, "CRC7 differs from the CMD0 frame of section 7.2.2");

CardInfo g_info{};
bool g_sdhc = false;

std::uint8_t xfer(std::uint8_t tx) noexcept {
  std::uint8_t rx = 0;
  spi_write_read_blocking(spi0, &tx, &rx, 1);
  return rx;
}

void select() noexcept { gpio_put(kCs, false); }

// CS high, then one idle byte so the card releases the data line (section 7.2.4 on the data line after CS).
void deselect() noexcept {
  gpio_put(kCs, true);
  static_cast<void>(xfer(kSdIdleLine));
}

bool wait_ready(std::uint64_t timeout_us) noexcept {
  const TimeUs end = hal_time_us() + timeout_us;
  do {
    if (xfer(kSdIdleLine) == kSdIdleLine) {
      return true;
    }
  } while (hal_time_us() < end);
  return false;
}

// Sends one command with CS low and returns its R1, or kSdIdleLine if no response came within the response wait. CS is left
// low for the caller's data phase. An application command is preceded by CMD55.
std::uint8_t command(std::uint8_t index, std::uint32_t arg, bool app = false) noexcept {
  if (app) {
    const std::uint8_t r = command(kSdCmdAppCmd, 0);
    if (r > kSdR1Idle) {
      return r;
    }
    deselect();
  }
  select();
  if (index != kSdCmdStopTransmission && index != kSdCmdGoIdle && !wait_ready(kWriteTimeoutUs)) {
    return kSdIdleLine;
  }
  std::array<std::uint8_t, kSdCommandBytes> frame{};
  frame[0] = static_cast<std::uint8_t>(kSdCommandStart | index);
  for (std::size_t i = 0; i < kSdArgumentBytes; ++i) {
    frame[1 + i] = static_cast<std::uint8_t>(arg >> (kBitsPerByte * (kSdArgumentBytes - 1 - i)));
  }
  frame[kSdCommandBytes - 1] = last_byte(frame.data());
  spi_write_blocking(spi0, frame.data(), frame.size());
  if (index == kSdCmdStopTransmission) {
    static_cast<void>(xfer(kSdIdleLine));  // the stuff byte after CMD12 (ChaN, "How to Use MMC/SDC")
  }
  for (unsigned i = 0; i < kSdResponseWaitBytes; ++i) {
    const std::uint8_t r = xfer(kSdIdleLine);
    if ((r & kSdR1MsbMask) == 0) {
      return r;
    }
  }
  return kSdIdleLine;
}

// The trailing 4 bytes of an R3 or R7 response, big-endian.
std::uint32_t long_response() noexcept {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < kSdLongResponseTrailBytes; ++i) {
    v = (v << kBitsPerByte) | xfer(kSdIdleLine);
  }
  return v;
}

// A data block after the start token. CS stays low.
bool read_block(std::uint8_t* buf, std::size_t n) noexcept {
  const TimeUs end = hal_time_us() + kReadTimeoutUs;
  std::uint8_t token = kSdIdleLine;
  do {
    token = xfer(kSdIdleLine);
  } while (token == kSdIdleLine && hal_time_us() < end);
  if (token != kSdTokenStartBlock) {
    return false;  // timeout, or a data error token (section 7.3.3.3)
  }
  spi_read_blocking(spi0, kSdIdleLine, buf, n);
  for (std::size_t i = 0; i < kSdCrc16Bytes; ++i) {
    static_cast<void>(xfer(kSdIdleLine));
  }
  return true;
}

// A data block with its token, a dummy CRC and the data response. CS stays low.
bool write_block(std::uint8_t token, const std::uint8_t* buf) noexcept {
  static_cast<void>(xfer(token));
  spi_write_blocking(spi0, buf, kSdBlockBytes);
  for (std::size_t i = 0; i < kSdCrc16Bytes; ++i) {
    static_cast<void>(xfer(kSdIdleLine));
  }
  const std::uint8_t resp = xfer(kSdIdleLine);
  return (resp & kSdDataResponseMask) == kSdDataResponseAccepted;
}

std::uint64_t csd_field(const std::array<std::uint8_t, kSdCsdBytes>& csd, unsigned hi, unsigned lo) noexcept {
  std::uint64_t v = 0;
  for (unsigned n = hi + 1; n-- > lo;) {
    const unsigned bit = (static_cast<unsigned>(csd[kSdCsdBytes - 1 - n / kBitsPerByte]) >> (n % kBitsPerByte)) & 1U;
    v = (v << 1) | bit;
  }
  return v;
}

std::uint64_t sectors_from_csd(const std::array<std::uint8_t, kSdCsdBytes>& csd) noexcept {
  constexpr unsigned kSectorShift = std::bit_width(kSdBlockBytes) - 1;
  const std::uint64_t structure = csd_field(csd, kSdCsdStructureHi, kSdCsdStructureLo);
  if (structure == 1) {
    return (csd_field(csd, kSdCsdV2CSizeHi, kSdCsdV2CSizeLo) + 1) * (kSdCsdV2UnitBytes / kSdBlockBytes);
  }
  if (structure == 0) {
    const std::uint64_t blocks = (csd_field(csd, kSdCsdV1CSizeHi, kSdCsdV1CSizeLo) + 1)
                                 << (csd_field(csd, kSdCsdV1CSizeMultHi, kSdCsdV1CSizeMultLo) + kSdCsdV1CSizeMultOffset);
    return (blocks << csd_field(csd, kSdCsdV1ReadBlLenHi, kSdCsdV1ReadBlLenLo)) >> kSectorShift;
  }
  return 0;
}

}  // namespace

bool init() noexcept {
  g_info = CardInfo{};
  g_sdhc = false;
  gpio_init(kCs);
  gpio_put(kCs, true);
  gpio_set_dir(kCs, GPIO_OUT);
  spi_init(spi0, kSdInitClockHz);
  gpio_set_function(kSck, GPIO_FUNC_SPI);
  gpio_set_function(kMosi, GPIO_FUNC_SPI);
  gpio_set_function(kMiso, GPIO_FUNC_SPI);
  gpio_pull_up(kMiso);

  // Section 6.4.1.1: at least 74 clocks with CS high.
  for (unsigned i = 0; i < (kSdPowerUpClocks + kBitsPerByte - 1) / kBitsPerByte; ++i) {
    static_cast<void>(xfer(kSdIdleLine));
  }

  // CMD0 into SPI mode, retried for the card's power-up time (section 6.4.1.1).
  const TimeUs reset_end = hal_time_us() + kPowerUpUs;
  bool idle = false;
  do {
    idle = command(kSdCmdGoIdle, 0) == kSdR1Idle;
    deselect();
  } while (!idle && hal_time_us() < reset_end);
  if (!idle) {
    return false;
  }

  // CMD8 (section 7.2.1): no answer but an illegal command means a version 1.x card.
  constexpr std::uint32_t kCmd8Arg = (kSdCmd8Vhs27To36 << kSdCmd8VhsShift) | kSdCmd8CheckPattern;
  const std::uint8_t r8 = command(kSdCmdSendIfCond, kCmd8Arg);
  bool version2 = false;
  if (r8 == kSdR1Idle) {
    const std::uint32_t v = long_response();
    deselect();
    if (((v >> kSdCmd8VhsShift) & kSdCmd8VhsMask) != kSdCmd8Vhs27To36 || (v & kSdCmd8CheckMask) != kSdCmd8CheckPattern) {
      return false;
    }
    version2 = true;
  } else {
    deselect();
    if ((r8 & kSdR1IllegalCommand) == 0) {
      return false;
    }
  }

  // ACMD41 until the card leaves idle (section 4.2.3).
  const TimeUs init_end = hal_time_us() + kInitTimeoutUs;
  std::uint8_t r = kSdR1Idle;
  do {
    r = command(kSdAcmdSendOpCond, version2 ? (std::uint32_t{1} << kSdAcmd41HcsBit) : 0, true);
    deselect();
  } while (r == kSdR1Idle && hal_time_us() < init_end);
  if (r != 0) {
    return false;
  }

  CardType type = CardType::Sdsc;
  if (version2) {
    if (command(kSdCmdReadOcr, 0) != 0) {
      deselect();
      return false;
    }
    const std::uint32_t ocr = long_response();
    deselect();
    if (((ocr >> kSdOcrCcsBit) & 1U) != 0) {
      type = CardType::Sdhc;
    }
  }
  if (type == CardType::Sdsc) {
    const std::uint8_t r16 = command(kSdCmdSetBlockLen, kSdBlockBytes);
    deselect();
    if (r16 != 0) {
      return false;
    }
  }

  static_cast<void>(spi_set_baudrate(spi0, kSdDataClockHz));

  std::array<std::uint8_t, kSdCsdBytes> csd{};
  bool csd_ok = false;
  if (command(kSdCmdSendCsd, 0) == 0) {
    csd_ok = read_block(csd.data(), csd.size());
  }
  deselect();

  g_sdhc = type == CardType::Sdhc;
  g_info.type = type;
  g_info.sectors = csd_ok ? sectors_from_csd(csd) : 0;
  return true;
}

CardInfo info() noexcept { return g_info; }

void release() noexcept {
  g_info = CardInfo{};
  g_sdhc = false;
  gpio_put(kCs, true);
}

bool read(std::uint32_t sector, std::span<std::uint8_t> buf) noexcept {
  if (g_info.type == CardType::None || buf.empty() || buf.size() % kSdBlockBytes != 0) {
    return false;
  }
  const std::size_t count = buf.size() / kSdBlockBytes;
  const std::uint32_t addr = g_sdhc ? sector : sector * static_cast<std::uint32_t>(kSdBlockBytes);
  bool ok = false;
  if (count == 1) {
    ok = command(kSdCmdReadSingle, addr) == 0 && read_block(buf.data(), kSdBlockBytes);
  } else if (command(kSdCmdReadMultiple, addr) == 0) {
    ok = true;
    for (std::size_t i = 0; i < count && ok; ++i) {
      ok = read_block(buf.data() + i * kSdBlockBytes, kSdBlockBytes);
    }
    ok = (command(kSdCmdStopTransmission, 0) == 0) && ok;
  }
  deselect();
  return ok;
}

bool write(std::uint32_t sector, std::span<const std::uint8_t> buf) noexcept {
  if (g_info.type == CardType::None || buf.empty() || buf.size() % kSdBlockBytes != 0) {
    return false;
  }
  const std::size_t count = buf.size() / kSdBlockBytes;
  const std::uint32_t addr = g_sdhc ? sector : sector * static_cast<std::uint32_t>(kSdBlockBytes);
  bool ok = false;
  if (count == 1) {
    ok = command(kSdCmdWriteSingle, addr) == 0 && write_block(kSdTokenStartBlock, buf.data()) &&
         wait_ready(kWriteTimeoutUs);
  } else if (command(kSdCmdWriteMultiple, addr) == 0) {
    ok = true;
    for (std::size_t i = 0; i < count && ok; ++i) {
      ok = wait_ready(kWriteTimeoutUs) && write_block(kSdTokenStartBlockMultiWrite, buf.data() + i * kSdBlockBytes);
    }
    ok = wait_ready(kWriteTimeoutUs) && ok;
    static_cast<void>(xfer(kSdTokenStopTran));
    ok = wait_ready(kWriteTimeoutUs) && ok;
  }
  deselect();
  return ok;
}

bool sync() noexcept {
  if (g_info.type == CardType::None) {
    return false;
  }
  select();
  const bool ok = wait_ready(kWriteTimeoutUs);
  deselect();
  return ok;
}

DetectLevels detect_levels() noexcept {
  DetectLevels d{};
  gpio_init(kDetect);
  gpio_set_dir(kDetect, GPIO_IN);
  gpio_pull_up(kDetect);
  busy_wait_us_32(kMicrosecondsPerMillisecond);
  d.with_pull_up = gpio_get(kDetect);
  gpio_pull_down(kDetect);
  busy_wait_us_32(kMicrosecondsPerMillisecond);
  d.with_pull_down = gpio_get(kDetect);
  gpio_disable_pulls(kDetect);
  return d;
}

}  // namespace marv::hal_target::sd

// FatFs disk I/O over the driver (diskio.h): one drive, 512-byte sectors.
extern "C" {

DSTATUS disk_status(BYTE pdrv) {
  if (pdrv != 0 || marv::hal_target::sd::info().type == marv::hal_target::sd::CardType::None) {
    return STA_NOINIT;
  }
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
  if (pdrv != 0) {
    return STA_NOINIT;
  }
  return marv::hal_target::sd::init() ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || count == 0) {
    return RES_PARERR;
  }
  if (disk_status(pdrv) != 0) {
    return RES_NOTRDY;
  }
  return marv::hal_target::sd::read(sector, {buff, static_cast<std::size_t>(count) * marv::prim::kSdBlockBytes})
             ? RES_OK
             : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || count == 0) {
    return RES_PARERR;
  }
  if (disk_status(pdrv) != 0) {
    return RES_NOTRDY;
  }
  return marv::hal_target::sd::write(sector, {buff, static_cast<std::size_t>(count) * marv::prim::kSdBlockBytes})
             ? RES_OK
             : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  if (pdrv != 0) {
    return RES_PARERR;
  }
  if (disk_status(pdrv) != 0) {
    return RES_NOTRDY;
  }
  switch (cmd) {
    case CTRL_SYNC:
      return marv::hal_target::sd::sync() ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT:
      *static_cast<LBA_t*>(buff) = static_cast<LBA_t>(marv::hal_target::sd::info().sectors);
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

}  // extern "C"
