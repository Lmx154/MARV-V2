#pragma once

#include <cstddef>
#include <cstdint>

// The only file under fw/ exempt from gate G1 (no numeric literals other than 0, 1, 2 and 0.5).
// Every constant here carries its citation and a kind: math, physics or a published standard (SI, WGS 84, a protocol
// spec). Vehicle and part numbers (card, sensor profile, datasheet) never go here. Checked by
// tools/ci/check_constants.py.
// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
namespace marv::prim {

// Dimension of physical space. Citation: the definition of physical 3-D Euclidean space, which has three
// dimensions.
// Kind: math.
inline constexpr std::size_t kSpatialDim = 3;

// Smallest and largest DShot throttle value. Citation: Betaflight DShot notes
// (https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/Dshot.mdx): 0 is reserved for
// disarmed, 1-47 are special commands, 48-2047 is throttle, the field is 11 bits wide.
// Kind: standard.
inline constexpr std::uint16_t kDshotThrottleMin = 48;
inline constexpr std::uint16_t kDshotThrottleMax = 2047;

// DShot frame layout and the DShot600 timing. Citation: Betaflight DShot notes
// (https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/Dshot.mdx): a frame is 16 bits
// "SSSSSSSSSSSTCCCC", sent most significant bit first: 11 bits of throttle, 1 bit of telemetry request, 4 bits of CRC
// = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F over the 12 bits above it. DShot600: bitrate 600 kbit/s, bit period
// 1.67 us, T0H 0.625 us and T1H 1.25 us (the high time of a 1 is double that of a 0), which are 3/8 and 6/8 of the bit
// period; the frame lasts 16 bit periods (26.72 us).
// Kind: standard.
inline constexpr unsigned kDshotThrottleBits = 11;
inline constexpr unsigned kDshotTelemetryBits = 1;
inline constexpr unsigned kDshotCrcBits = 4;
inline constexpr unsigned kDshotFrameBits = 16;
inline constexpr std::uint32_t kDshot600BitRateHz = 600000;
inline constexpr unsigned kDshotT0hOfBitNum = 3;
inline constexpr unsigned kDshotT1hOfBitNum = 6;
inline constexpr unsigned kDshotBitFractionDen = 8;

// CRSF frame construction. Citation: TBS CRSF spec (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md), sections
// "Frame Construction" / "Frame Details": a frame is sync byte, frame length, type, payload, CRC; the sync byte is the
// serial sync byte 0xC8 or the broadcast device address 0x00 (or a device address); the frame length counts the bytes
// after the sync and length bytes (type, payload, CRC), valid range 2 to 62, anything else is discarded; a frame is at
// most 64 bytes including the sync and CRC bytes. Section "CRC": CRC8, polynomial x7 + x6 + x4 + x2 + x0 = 0xD5, over type
// and payload.
// Kind: standard.
inline constexpr std::uint8_t kCrsfSyncSerial = 0xC8;
inline constexpr std::uint8_t kCrsfAddrBroadcast = 0x00;
inline constexpr std::size_t kCrsfFrameBytesMax = 64;
inline constexpr std::uint8_t kCrsfLengthMin = 2;
inline constexpr std::uint8_t kCrsfLengthMax = 62;
inline constexpr std::uint8_t kCrsfCrc8Poly = 0xD5;

// CRSF frame types. Citation: TBS CRSF spec (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md), section
// "Broadcast Frame Types": "0x14 Link Statistics" and "0x16 RC Channels Packed Payload".
// Kind: standard.
inline constexpr std::uint8_t kCrsfTypeLinkStatistics = 0x14;
inline constexpr std::uint8_t kCrsfTypeRcChannelsPacked = 0x16;

// CRSF 0x16 RC Channels Packed Payload. Citation: TBS CRSF spec
// (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md), section "0x16 RC Channels Packed Payload": "16 channels
// packed into 22 bytes", the struct lists channel_01 .. channel_16 as 11-bit fields back to back (16 * 11 = 176 bits =
// 22 bytes). The spec does not state the bit order; least significant bit first (little-endian bit packing) is that of
// the struct as a C bit-field on a little-endian target and of the CRSF working group's example frame
// (https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_RC_CHANNELS_PACKED: "LSB-first, little endian").
// Kind: standard.
inline constexpr std::size_t kCrsfRcChannels = 16;
inline constexpr unsigned kCrsfRcChannelBits = 11;
inline constexpr std::size_t kCrsfRcPayloadBytes = 22;

// CRSF 0x14 Link Statistics payload layout. Citation: TBS CRSF spec
// (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md), section "0x14 Link Statistics": ten one-byte fields in this
// order, the SNR fields being int8_t: up_rssi_ant1, up_rssi_ant2, up_link_quality, up_snr, active_antenna, rf_profile,
// up_rf_power, down_rssi, down_link_quality, down_snr. The value is the byte offset into the payload.
// Kind: standard.
inline constexpr std::size_t kCrsfLsUpRssiAnt1 = 0;
inline constexpr std::size_t kCrsfLsUpRssiAnt2 = 1;
inline constexpr std::size_t kCrsfLsUpLinkQuality = 2;
inline constexpr std::size_t kCrsfLsUpSnr = 3;
inline constexpr std::size_t kCrsfLsActiveAntenna = 4;
inline constexpr std::size_t kCrsfLsRfProfile = 5;
inline constexpr std::size_t kCrsfLsUpRfPower = 6;
inline constexpr std::size_t kCrsfLsDownRssi = 7;
inline constexpr std::size_t kCrsfLsDownLinkQuality = 8;
inline constexpr std::size_t kCrsfLsDownSnr = 9;
inline constexpr std::size_t kCrsfLinkStatsPayloadBytes = 10;

// CRSF serial link to a flight controller. Citation: ExpressLRS master (dd96654), src/lib/OPTIONS/options.cpp:197, the
// receiver's UART baud rate defaults to 420000 ("rcvr-uart-baud" | 420000), and src/user_defines.txt:44-47 ("Changing this
// will prevent flashing via_BetaflightPassthrough, which expects a 420000 baud RX"); the receiver is configured for 8N1
// on its CRSF output (src/src/rx_main.cpp:1347, Serial.begin(serialBaud, serialConfig, ...)). The TBS CRSF spec
// (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md) differs: section "Frame Details" gives 416666 baud 8N1 for a
// device-to-flight-controller link (line 138) and 400 kbaud 8N1 for the handset link (line 134). UNVERIFIED: the two
// sources disagree (420000 against 416666, 0.8 %); the ELRS figure is used because the ELRS receiver transmits at it, and
// the 8N1 frame (8 data bits, 1 stop bit, no parity) is common to both. The bench shows which the receiver link needs.
// Kind: standard.
inline constexpr std::uint32_t kCrsfBaudElrs = 420000;
inline constexpr std::uint32_t kCrsfBaudTbs = 416666;
inline constexpr unsigned kCrsfUartDataBits = 8;
inline constexpr unsigned kCrsfUartStopBits = 1;

// CRSF channel ticks to pulse width. Citation: TBS CRSF spec (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md),
// section "0x16 RC Channels Packed Payload": TICKS_TO_US(x) = ((x - 992) * 5 / 8 + 1500), "Center (1500us) = 992". ELRS
// master src/include/crsf_protocol.h:20-24 gives the standard range: 172 = 988 us (-100 %), 992 = 1500 us, 1811 = 2012 us
// (+100 %); the spec's formula gives 987.5 us and 2011.875 us there. The normalisation used by the pilot-input contract
// (decision 0040, quad spec 4.0 M2): stick x = (us - 1500) / 512 with 512 us the half span of the 988 .. 2012 us range;
// throttle = (us - 988) / 1024 with 1024 us its span.
// Kind: standard.
inline constexpr std::int32_t kCrsfTickCentre = 992;
inline constexpr float kCrsfUsCentre = 1500.0F;
inline constexpr float kCrsfUsPerTickNum = 5.0F;
inline constexpr float kCrsfUsPerTickDen = 8.0F;
inline constexpr float kRcUsHalfSpan = 512.0F;
inline constexpr float kRcUsThrottleMin = 988.0F;
inline constexpr float kRcUsThrottleSpan = 1024.0F;

// Percent. Citation: the definition of the percent, one hundredth (CRSF link quality is reported in percent, TBS CRSF spec
// "0x14 Link Statistics").
// Kind: math.
inline constexpr float kPercentPerUnit = 100.0F;

// Pi. Citation: the mathematical constant, the ratio of a circle's circumference to its diameter.
// Kind: math.
inline constexpr double kPi = 3.14159265358979323846;

// Seconds per minute. Citation: SI definition (the minute is exactly 60 s, an accepted non-SI unit of time).
// Kind: standard.
inline constexpr double kSecondsPerMinute = 60.0;

// CRC-32 (the bench IMU log's frame check, fw/hal/target/include/marv/hal_target/imu_log_format.hpp). Citation: ITU-T
// V.42 (03/2002) 8.1.1.6.2 and ISO/IEC 3309 (the IEEE 802.3 frame check sequence): generator polynomial x32 + x26 + x23 +
// x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1, which is 0x04C11DB7 and 0xEDB88320 with its bits reversed;
// the register starts at all ones, the bytes are processed least significant bit first (the reflected form) and the
// result is complemented. The check value over the ASCII bytes "123456789" is 0xCBF43926 (the CRC-32/ISO-HDLC catalogue
// entry, Greg Cook's CRC RevEng catalogue).
// Kind: standard.
inline constexpr std::uint32_t kCrc32PolyReflected = 0xEDB88320;

// Microseconds per second. Citation: SI definition (the prefix micro is exactly 10^-6, so one second is exactly
// 10^6 microseconds).
// Kind: standard.
inline constexpr double kMicrosecondsPerSecond = 1000000.0;

// Microseconds per millisecond. Citation: SI definition (the prefixes micro and milli are exactly 10^-6 and 10^-3, so one
// millisecond is exactly 1000 microseconds).
// Kind: standard.
inline constexpr std::uint32_t kMicrosecondsPerMillisecond = 1000;

// The radix of the decimal numeral system. Citation: the positional notation of the decimal digits 0-9, ten symbols
// (ISO 80000-2:2019, 7.1, "decimal numeral system", base ten).
// Kind: math.
inline constexpr std::uint32_t kDecimalRadix = 10;

// Degrees per half turn. Citation: BIPM SI Brochure, 9th ed. (2019), Table 8: the degree, 1° = (π/180) rad.
// Kind: standard.
inline constexpr double kDegreesPerHalfTurn = 180.0;

// Standard acceleration of gravity g_n, m/s^2. Citation: 3rd CGPM (1901), declaration on the unit of mass and on the
// definition of weight, conventional value of g_n = 980.665 cm/s^2 (BIPM SI Brochure, 9th ed. (2019), Appendix 1).
// Kind: standard.
inline constexpr double kStandardGravity = 9.80665;

// Kelvin temperature at 0 °C. Citation: BIPM SI Brochure, 9th ed. (2019), section 2.3.1: t/°C = T/K - 273.15.
// Kind: standard.
inline constexpr double kKelvinAtZeroCelsius = 273.15;

// WGS 84 normal-gravity constants. Citation for every entry below: NIMA TR8350.2, "Department of Defense World
// Geodetic System 1984", 3rd ed., 4 July 1997, incl. Amendment 1 (3 January 2000) ("NIMA TR8350.2 3rd ed. Amdt 1
// (2000)").
// Kind: standard.
// Semi-major axis a, m. Table 3.1.
inline constexpr double kWgs84A = 6378137.0;
// Inverse flattening 1/f, defining parameter. Table 3.1.
inline constexpr double kWgs84InvF = 298.257223563;
// Flattening f. Derived: f = 1 / (1/f) from the defining parameter above.
inline constexpr double kWgs84F = 1.0 / kWgs84InvF;
// First eccentricity squared e^2. Table 3.3.
inline constexpr double kWgs84E2 = 6.69437999014e-3;
// Normal gravity at the equator gamma_e, m/s^2. Table 3.4.
inline constexpr double kWgs84GammaE = 9.7803253359;
// Somigliana's constant k = b gamma_p / (a gamma_e) - 1. Table 3.4.
inline constexpr double kWgs84K = 0.00193185265241;
// m = omega^2 a^2 b / GM. Table 3.4.
inline constexpr double kWgs84M = 0.00344978650684;
// Coefficient of the h^2 term of the height series, gamma_h = gamma [1 - (2/a)(1 + f + m - 2 f sin^2 phi) h +
// (3/a^2) h^2]. Eq. (4-3).
inline constexpr double kWgs84HeightQuadCoeff = 3.0;

}  // namespace marv::prim
// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
