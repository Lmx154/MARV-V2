#pragma once

#include <array>
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

// SHA-256 (the config file's payload and config hashes, fw/store/include/marv/store/sha256.hpp).
// Citation: NIST FIPS 180-4, "Secure Hash Standard (SHS)", August 2015. Section 2.2: the digest is
// 256 bits (32 bytes) and the
// algorithm works on 32-bit words. Section 4.1.2: Sigma0(x) = ROTR^2 ^ ROTR^13 ^ ROTR^22, Sigma1(x) = ROTR^6 ^ ROTR^11 ^
// ROTR^25, sigma0(x) = ROTR^7 ^ ROTR^18 ^ SHR^3, sigma1(x) = ROTR^17 ^ ROTR^19 ^ SHR^10. Section 4.2.2: the 64 round
// constants K. Section 5.1.1: padding appends one 1 bit (the byte 0x80 for a message of whole bytes), zeros, and the
// message length in bits as a 64-bit big-endian integer, to a multiple of the 512-bit (64-byte) block. Section 5.3.3: the
// initial hash value H(0). Section 6.2.2: the message schedule W_t = sigma1(W_{t-2}) + W_{t-7} + sigma0(W_{t-15}) +
// W_{t-16} for t = 16..63, W_0..W_15 being the block's sixteen big-endian words.
// Kind: standard.
inline constexpr std::size_t kSha256DigestBytes = 32;
inline constexpr std::size_t kSha256BlockBytes = 64;
inline constexpr std::size_t kSha256LengthBytes = 8;
inline constexpr std::uint8_t kSha256PadByte = 0x80;
inline constexpr std::array<unsigned, 3> kSha256BigSigma0Rotr{2, 13, 22};
inline constexpr std::array<unsigned, 3> kSha256BigSigma1Rotr{6, 11, 25};
inline constexpr std::array<unsigned, 2> kSha256SmallSigma0Rotr{7, 18};
inline constexpr unsigned kSha256SmallSigma0Shr = 3;
inline constexpr std::array<unsigned, 2> kSha256SmallSigma1Rotr{17, 19};
inline constexpr unsigned kSha256SmallSigma1Shr = 10;
inline constexpr std::size_t kSha256LagSigma1 = 2;
inline constexpr std::size_t kSha256LagPlain = 7;
inline constexpr std::size_t kSha256LagSigma0 = 15;
inline constexpr std::size_t kSha256LagOldest = 16;
inline constexpr std::array<std::uint32_t, 64> kSha256K{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
inline constexpr std::array<std::uint32_t, 8> kSha256H0{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                                        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

// The largest UDP payload over IPv4, bytes (hal_sim's ground-link socket, fw/hal/sim/include/marv/hal_sim/link_udp.hpp).
// Citation: RFC 768 (UDP): the 16-bit length field counts the 8-byte UDP header and the data; RFC 791 (IPv4): the 16-bit total
// length field counts the datagram with its header, at least 20 bytes. 65535 - 20 - 8 = 65507.
// Kind: standard.
inline constexpr std::size_t kUdpIpv4PayloadMaxBytes = 65507;

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

// Bits per byte. Citation: ISO/IEC 80000-13:2008, item 13-9: one byte is eight bits.
// Kind: standard.
inline constexpr unsigned kBitsPerByte = 8;

// SD memory in SPI mode (fw/hal/target/src/sd_spi.cpp), clocks and power-up. "SD PLSS 9.10" below is the SD Association's
// "SD Specifications Part 1 Physical Layer Simplified Specification Version 9.10", December 1, 2023.
// Citation: SD PLSS 9.10, section 4.2.1 (identification clock rate fOD, the 400 kHz default) and section 4.4 (the
// application note on polling ACMD41: continuous clock in the range 100 kHz to 400 kHz); section 2 (System Features), Bus
// Speed Mode (1): Default Speed mode, frequency up to 25 MHz, and table 5-6 (TRAN_SPEED 32h is 25 MHz, "the mandatory
// maximum operating frequency"); section 6.4.1.1 (power-up: ready for the first command within 1 ms; the host supplies at
// least 74 clocks with CS held high in SPI mode).
// Kind: standard.
inline constexpr std::uint32_t kSdInitClockHz = 400000;
inline constexpr std::uint32_t kSdDataClockHz = 25000000;
inline constexpr unsigned kSdPowerUpClocks = 74;
inline constexpr std::uint32_t kSdPowerUpMs = 1;

// SD memory in SPI mode, timeouts. Citation: SD PLSS 9.10, section 4.2.3 (application note: "The host shall set ACMD41
// timeout more than 1 second", counted from the first ACMD41); section 4.6.2.1 (read: "The host should use 100 ms timeout
// (minimum) for single and multiple read operations"); section 4.6.2.2 (write: busy up to 250 ms, up to 500 ms for SDXC and
// SDUC in the listed cases, and "strongly recommended for hosts to implement more than 500ms timeout value").
// Kind: standard.
inline constexpr std::uint32_t kSdAcmd41MinTimeoutMs = 1000;
inline constexpr std::uint32_t kSdReadTimeoutMs = 100;
inline constexpr std::uint32_t kSdWriteBusyMaxMs = 500;

// SD memory in SPI mode, command frame. Citation: SD PLSS 9.10, section 7.3.1.1, table 7-1 (a command is 6 bytes: start bit
// 0, transmission bit 1, a 6-bit index, a 32-bit argument, a 7-bit CRC and the end bit 1); section 4.5 (the CRC7 polynomial
// is x^7 + x^3 + 1); section 7.2.2 (the CMD0 frame is 0x40, 0x00, 0x00, 0x00, 0x00, 0x95); table 7-3 (command indices).
// Kind: standard.
inline constexpr std::size_t kSdCommandBytes = 6;
inline constexpr std::size_t kSdArgumentBytes = 4;
inline constexpr std::uint8_t kSdCommandStart = 0x40;
inline constexpr std::uint8_t kSdCommandEnd = 0x01;
inline constexpr unsigned kSdCrc7Bits = 7;
inline constexpr std::uint8_t kSdCrc7Poly = 0x09;
inline constexpr std::uint8_t kSdCmd0Frame = 0x95;
inline constexpr std::uint8_t kSdCmdGoIdle = 0;
inline constexpr std::uint8_t kSdCmdSendIfCond = 8;
inline constexpr std::uint8_t kSdCmdSendCsd = 9;
inline constexpr std::uint8_t kSdCmdStopTransmission = 12;
inline constexpr std::uint8_t kSdCmdSetBlockLen = 16;
inline constexpr std::uint8_t kSdCmdReadSingle = 17;
inline constexpr std::uint8_t kSdCmdReadMultiple = 18;
inline constexpr std::uint8_t kSdCmdWriteSingle = 24;
inline constexpr std::uint8_t kSdCmdWriteMultiple = 25;
inline constexpr std::uint8_t kSdCmdAppCmd = 55;
inline constexpr std::uint8_t kSdCmdReadOcr = 58;
inline constexpr std::uint8_t kSdAcmdSendOpCond = 41;

// SD memory in SPI mode, command arguments. Citation: SD PLSS 9.10, table 4-18 and table 7-3 (CMD8 argument: voltage
// supplied VHS in bits 11:8, 0001b is 2.7-3.6 V, and an 8-bit check pattern in bits 7:0 that is "any 8-bit pattern", here
// 0xAA, alternating bits, a free choice); table 7-4 (ACMD41 argument: HCS is bit 30); section 5.1, table 5-1 (OCR bit 30 is
// the capacity status CCS, 1 for SDHC and SDXC); section 7.3.2.6 and figure 7-12 (the R7 response is the R1 byte and 4 more
// bytes: command version and reserved bits, voltage accepted in bits 11:8, the echoed check pattern in bits 7:0); section
// 7.3.2.4 (the R3 response, to CMD58, is the R1 byte and the 4 OCR bytes).
// Kind: standard.
inline constexpr std::uint32_t kSdCmd8Vhs27To36 = 1;
inline constexpr unsigned kSdCmd8VhsShift = 8;
inline constexpr std::uint32_t kSdCmd8VhsMask = 0xF;
inline constexpr std::uint32_t kSdCmd8CheckPattern = 0xAA;
inline constexpr std::uint32_t kSdCmd8CheckMask = 0xFF;
inline constexpr unsigned kSdAcmd41HcsBit = 30;
inline constexpr unsigned kSdOcrCcsBit = 30;
inline constexpr std::size_t kSdLongResponseTrailBytes = 4;

// SD memory in SPI mode, R1 response and tokens. Citation: SD PLSS 9.10, section 7.3.2.1 and figure 7-9 (R1 is one byte,
// the MSB is always 0, bit 0 is "in idle state", bit 2 "illegal command", bits 6:1 are the error flags); section 7.3.3.1
// (data response token "x x x 0 Status 1", status 010b is "data accepted"); section 7.3.3.2 (start block token 11111110b
// for single block read and write and for multiple block read, 11111100b for each block of a multiple block write, the stop
// tran token 11111101b); section 7.3.2.2 (the busy signal is a zero value, a non-zero value is ready); section 7.3.3.3 (a
// data error token replaces a data block). The host holds the data line high (0xFF) while it only clocks (section 7.2).
// Kind: standard.
inline constexpr std::uint8_t kSdIdleLine = 0xFF;
inline constexpr std::uint8_t kSdR1MsbMask = 0x80;
inline constexpr std::uint8_t kSdR1Idle = 0x01;
inline constexpr std::uint8_t kSdR1IllegalCommand = 0x04;
inline constexpr std::uint8_t kSdR1ErrorMask = 0x7E;
inline constexpr std::uint8_t kSdTokenStartBlock = 0xFE;
inline constexpr std::uint8_t kSdTokenStartBlockMultiWrite = 0xFC;
inline constexpr std::uint8_t kSdTokenStopTran = 0xFD;
inline constexpr std::uint8_t kSdDataResponseMask = 0x1F;
inline constexpr std::uint8_t kSdDataResponseAccepted = 0x05;

// SD memory in SPI mode, blocks and the register read. Citation: SD PLSS 9.10, section 7.2.3 (SDHC and SDXC block length is
// fixed at 512 bytes); section 7.3.3.2 (a data block is followed by a 16-bit CRC, 2 bytes); section 7.2.6 (the CSD is read
// as a block of 16 bytes).
// Kind: standard.
inline constexpr std::size_t kSdBlockBytes = 512;
inline constexpr std::size_t kSdCrc16Bytes = 2;
inline constexpr std::size_t kSdCsdBytes = 16;

// SD memory in SPI mode, response wait. Citation: the command response time NCR, 0 to 8 bytes for an SD memory, in ChaN,
// "How to Use MMC/SDC", https://elm-chan.org/docs/mmc/mmc_e.html, section "SPI mode" ("The response is sent back within
// command response time (NCR), 0 to 8 bytes for SDC"). UNVERIFIED against the SD specification: the timing values (SD
// PLSS 9.10, section 7.5.4) are removed from the Simplified Specification.
// Kind: standard.
inline constexpr unsigned kSdResponseWaitBytes = 8;

// SD memory capacity from the CSD register. Citation: SD PLSS 9.10, section 5.3.1 and table 5-3 (CSD_STRUCTURE is CSD bits 127:126, 0 for
// version 1.0 and 1 for version 2.0); section 5.3.3 and table 5-16 (version 2.0: C_SIZE is bits 69:48, capacity =
// (C_SIZE + 1) * 512 KByte); section 5.3.2 and table 5-4 (version 1.0: C_SIZE bits 73:62, C_SIZE_MULT bits 49:47, READ_BL_LEN
// bits 83:80; capacity = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN bytes).
// Kind: standard.
inline constexpr unsigned kSdCsdStructureHi = 127;
inline constexpr unsigned kSdCsdStructureLo = 126;
inline constexpr unsigned kSdCsdV2CSizeHi = 69;
inline constexpr unsigned kSdCsdV2CSizeLo = 48;
inline constexpr std::uint64_t kSdCsdV2UnitBytes = 524288;
inline constexpr unsigned kSdCsdV1CSizeHi = 73;
inline constexpr unsigned kSdCsdV1CSizeLo = 62;
inline constexpr unsigned kSdCsdV1CSizeMultHi = 49;
inline constexpr unsigned kSdCsdV1CSizeMultLo = 47;
inline constexpr unsigned kSdCsdV1ReadBlLenHi = 83;
inline constexpr unsigned kSdCsdV1ReadBlLenLo = 80;
inline constexpr unsigned kSdCsdV1CSizeMultOffset = 2;


// USB device identity and descriptor numbers of the FC3v2 image (fw/hal/target/src/usb.cpp). "USB 2.0" below is the Universal
// Serial Bus Specification, Revision 2.0, April 27, 2000. Citation: the vendor ID 0x2E8A is Raspberry Pi's, assigned by the
// USB Implementers Forum (https://www.usb.org/sites/default/files/vendor_ids.pdf); the product ID 0x0009 is the one the Pico SDK
// 2.3.1 gives its CDC device on RP2350 (src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c, USBD_PID), kept so the host
// path /dev/serial/by-id/usb-Raspberry_Pi_Pico_<serial>-if00 does not change; bcdUSB 2.10 (the Binary Object Store the reset
// interface's Microsoft OS 2.0 descriptor needs, USB 2.0 ECN "Link Power Management and BOS", also in that file) and bcdDevice
// 1.00 are that file's values; USB 2.0 section 9.6.3 (bMaxPower is in units of 2 mA; 250 mA is that file's USBD_MAX_POWER_MA);
// section 9.6.7 and table 9-15 (string descriptor language 0x0409 is English, United States, from "Universal Serial Bus
// Language Identifiers (LANGIDs)", Version 1.0); section 5.8.3 (a full-speed bulk endpoint's maximum packet size is 64 bytes);
// CDC notification endpoint maximum packet size 8 bytes is that file's USBD_CDC_CMD_MAX_SIZE.
// Kind: standard.
inline constexpr std::uint16_t kUsbVendorIdRaspberryPi = 0x2E8A;
inline constexpr std::uint16_t kUsbProductIdPicoSdkCdc = 0x0009;
inline constexpr std::uint16_t kUsbBcdUsb210 = 0x0210;
inline constexpr std::uint16_t kUsbBcdDevice100 = 0x0100;
inline constexpr unsigned kUsbConfigMaxPowerMa = 250;
inline constexpr std::uint16_t kUsbLangIdEnglishUs = 0x0409;
inline constexpr unsigned kUsbFullSpeedBulkPacketBytes = 64;
inline constexpr unsigned kUsbCdcNotifyPacketBytes = 8;

// USB endpoint addresses of the FC3v2 image. Citation: USB 2.0 section 9.6.6 and table 9-13 (bEndpointAddress: bits 3..0 the
// endpoint number, bit 7 the direction, 1 = IN to the host). Endpoints 1 and 2 are the console CDC's, the numbers the Pico
// SDK's own CDC uses (stdio_usb_descriptors.c: notification IN 0x81, data OUT 0x02, data IN 0x82); 3 and 4 are the link CDC's,
// the next unused numbers.
// Kind: standard.
inline constexpr std::uint8_t kUsbEpConsoleNotifyIn = 0x81;
inline constexpr std::uint8_t kUsbEpConsoleDataOut = 0x02;
inline constexpr std::uint8_t kUsbEpConsoleDataIn = 0x82;
inline constexpr std::uint8_t kUsbEpLinkNotifyIn = 0x83;
inline constexpr std::uint8_t kUsbEpLinkDataOut = 0x04;
inline constexpr std::uint8_t kUsbEpLinkDataIn = 0x84;

}  // namespace marv::prim
// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
