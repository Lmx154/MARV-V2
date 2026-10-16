// STMicroelectronics LSM6DSV: the register map, field codes and figures the FC3v2 driver uses (quad spec §4.0 M1;
// fw/hal/target/src/lsm6dsv.cpp). Every number is cited to ST DS13476 Rev 5 (August 2023),
// https://www.st.com/resource/en/datasheet/lsm6dsv.pdf, the source of sensors/profiles/madflight_fc3v2_revc.yaml.
//
// A part description beside the board header, outside fw/ for the reason the board header is (decision 0027 item 3):
// gate G1 exempts only fw/prim/include/marv/prim/constants.hpp, and that file holds no datasheet numbers
// (tools/ci/check_constants.py). Included by the target driver and by the L09 host tests of its decode; never by a
// flight module above the HAL.
#pragma once

#include <cstdint>

namespace marv::lsm6dsv {

// --- SPI (§5.1.3) ---
// First byte of a transfer: bit 0 on the wire (the MSb) is RW, 1 = read; bits 1-7 are the register address.
inline constexpr std::uint8_t kSpiReadFlag = 0x80;
// fc(SPC) max, Table 6 (SPI slave timing values).
inline constexpr std::uint32_t kSpiClockMaxHz = 10000000;

// --- Register addresses (§8, Table 23) ---
inline constexpr std::uint8_t kRegFifoCtrl1 = 0x07;
inline constexpr std::uint8_t kRegFifoCtrl3 = 0x09;
inline constexpr std::uint8_t kRegFifoCtrl4 = 0x0A;
inline constexpr std::uint8_t kRegInt1Ctrl = 0x0D;
inline constexpr std::uint8_t kRegWhoAmI = 0x0F;
inline constexpr std::uint8_t kRegCtrl1 = 0x10;
inline constexpr std::uint8_t kRegCtrl2 = 0x11;
inline constexpr std::uint8_t kRegCtrl3 = 0x12;
inline constexpr std::uint8_t kRegCtrl6 = 0x15;
inline constexpr std::uint8_t kRegCtrl8 = 0x17;
inline constexpr std::uint8_t kRegFifoStatus1 = 0x1B;
inline constexpr std::uint8_t kRegFifoStatus2 = 0x1C;
inline constexpr std::uint8_t kRegStatusReg = 0x1E;
inline constexpr std::uint8_t kRegOutTempL = 0x20;     // OUT_TEMP_L, OUT_TEMP_H (21h)
inline constexpr std::uint8_t kRegInternalFreqFine = 0x4F;
inline constexpr std::uint8_t kRegFifoDataOutTag = 0x78;

// WHO_AM_I (0Fh) fixed value, §9.13 Table 48.
inline constexpr std::uint8_t kWhoAmI = 0x70;

// CTRL3 (12h), §9.16 Tables 55-56: BDU bit 6, IF_INC bit 2, SW_RESET bit 0 (self-clearing).
inline constexpr std::uint8_t kCtrl3Bdu = 0x40;
inline constexpr std::uint8_t kCtrl3IfInc = 0x04;
inline constexpr std::uint8_t kCtrl3SwReset = 0x01;

// CTRL1 (10h) / CTRL2 (11h), §9.14-9.15 Tables 49-54: OP_MODE_[2:0] = 000 high-performance mode (bits 6:4),
// ODR_[3:0] = 1100 7.68 kHz (high-performance mode) (bits 3:0), the same code for the accelerometer (Table 51) and
// the gyroscope (Table 54).
inline constexpr std::uint8_t kCtrl1Xl7680HzHighPerformance = 0x0C;
inline constexpr std::uint8_t kCtrl2G7680HzHighPerformance = 0x0C;
// The rate those codes select, Hz (Tables 51 and 54; nominal, the internal oscillator sets the actual rate).
inline constexpr std::uint32_t kOdrHz = 7680;

// CTRL6 (15h), §9.19 Tables 61-62: FS_G_[3:0] = 0100 ±2000 dps (bits 3:0); LPF1_G_BW_[2:0] = 000 (bits 6:4), unused
// while LPF1_G_EN in CTRL7 (16h) keeps its default 0.
inline constexpr std::uint8_t kCtrl6Fs2000Dps = 0x04;
// CTRL8 (17h), §9.21 Tables 66-67: FS_XL_[1:0] = 11 ±16 g (bits 1:0); HP_LPF2_XL_BW_[2:0] = 000, XL_DualC_EN = 0.
inline constexpr std::uint8_t kCtrl8Fs16G = 0x03;

// FIFO_CTRL3 (09h), §9.7 Tables 36-37: BDR_GY_[3:0] (bits 7:4) and BDR_XL_[3:0] (bits 3:0) = 1100, 7.68 kHz.
inline constexpr std::uint8_t kFifoCtrl3Bdr7680Hz = 0xCC;
// FIFO_CTRL4 (0Ah), §9.8 Tables 38-39: FIFO_MODE_[2:0] (bits 2:0) = 000 bypass (FIFO reset), 110 continuous; timestamp
// and temperature not batched (bits 7:4 = 0).
inline constexpr std::uint8_t kFifoCtrl4Bypass = 0x00;
inline constexpr std::uint8_t kFifoCtrl4Continuous = 0x06;
// INT1_CTRL (0Dh), §9.11 Tables 44-45: INT1_FIFO_TH bit 3.
inline constexpr std::uint8_t kInt1FifoTh = 0x08;

// FIFO_STATUS2 (1Ch), §9.26 Tables 77-78: FIFO_OVR_LATCHED bit 3 (cleared by the read), DIFF_FIFO_8 bit 0.
inline constexpr std::uint8_t kFifoStatus2OvrLatched = 0x08;
inline constexpr std::uint8_t kFifoStatus2DiffFifo8 = 0x01;
// DIFF_FIFO_[8:0] is 9 bits wide (FIFO_STATUS1 bits 7:0, FIFO_STATUS2 bit 0): the shift of DIFF_FIFO_8.
inline constexpr unsigned kDiffFifo8Shift = 8;
// STATUS_REG (1Eh), §9.28 Tables 81-82: TDA bit 2 (new temperature data available).
inline constexpr std::uint8_t kStatusTda = 0x04;

// FIFO word, §6.10.8: one tag byte (78h) and 6 data bytes (79h-7Eh), X, Y, Z each low byte first (Tables 211-216).
inline constexpr unsigned kFifoWordBytes = 7;
// FIFO_DATA_OUT_TAG (78h), §9.81 Tables 208-210: TAG_SENSOR_[4:0] bits 7:3, TAG_CNT_[1:0] bits 2:1.
inline constexpr unsigned kTagSensorShift = 3;
inline constexpr unsigned kTagCntShift = 1;
inline constexpr std::uint8_t kTagCntMask = 0x03;
inline constexpr std::uint8_t kTagGyroNc = 0x01;
inline constexpr std::uint8_t kTagAccelNc = 0x02;

// Sensitivities at the configured full scales, §4.1 Table 3: G_So 70 mdps/LSB at ±2000 dps; LA_So 0.488 mg/LSB at
// ±16 g. Written in dps/LSB and g/LSB.
inline constexpr double kGyroDpsPerLsb = 70.0e-3;
inline constexpr double kAccelGPerLsb = 0.488e-3;
// Temperature sensor, §4.3 Table 5: TSen 256 LSB/°C (typ.); 0 LSB at 25 °C (typ., footnote 3).
inline constexpr double kTempLsbPerDegC = 256.0;
inline constexpr double kTempDegCAtZeroLsb = 25.0;

// INTERNAL_FREQ_FINE (4Fh), §9.49 Tables 136-137: FREQ_FINE_[7:0], two's complement, the effective ODR's difference
// from typical in steps of 0.13 %: ODR_actual = 7680 Hz x (1 + 0.0013 x FREQ_FINE) / ODRcoeff, ODRcoeff 1 at 7.68 kHz
// (Table 138).
inline constexpr double kFreqFineStep = 0.0013;

// Ton, gyroscope turn-on time, 30 ms typ., §4.2 Table 4, in µs.
inline constexpr std::uint32_t kGyroTurnOnUs = 30000;

}  // namespace marv::lsm6dsv
