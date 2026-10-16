// madflight FC3v2 Rev-C: the M1 board (decision 0027; quad §4.0 M1).
//
// Pins: https://madflight.com/Board-FC3/ (the "Pinout FC3v2" table, read 2026-10-15). Every pin below is a row of that
// table. The page lists the I2C0 pads (SMD Pads) with SDA and SCL swapped against its own table (GPIO 33 SDA, GPIO 32
// SCL); the table rows are used here (GPIO 32 SDA, GPIO 33 SCL), and the pins stay UNVERIFIED until the bench.
// The page names the chip RP2350B; the chip on the board is an RP2354B (marking 2354B0A4, Luis 2026-10-15). Its 2 MB
// in-package flash is the only flash.
//
// Modelled on boards/include/boards/pico2.h of the Pico SDK 2.3.1 (the RP2350 flash and stepping settings).
//
// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

#ifndef _BOARDS_MADFLIGHT_FC3V2_REVC_H
#define _BOARDS_MADFLIGHT_FC3V2_REVC_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// For board detection
#define MADFLIGHT_FC3V2_REVC

// --- RP2350 VARIANT ---
// The B package (80 pins, 48 GPIO; RP2354B is the B package with in-package flash). The IMU's SCK and MOSI are GPIO 30
// and 31, which the A package (GPIO 0-29) does not have.
#define PICO_RP2350A 0

// --- UART ---
// No PICO_DEFAULT_UART*: SER0 (GPIO 0/1) is the receiver port, and the SDK's default console is USB CDC.

// --- IMU: SPI1 ---
#define MADFLIGHT_FC3V2_IMU_SPI 1
#define MADFLIGHT_FC3V2_IMU_MISO_PIN 28
#define MADFLIGHT_FC3V2_IMU_SCK_PIN 30
#define MADFLIGHT_FC3V2_IMU_MOSI_PIN 31
#define MADFLIGHT_FC3V2_IMU_CS_PIN 29
#define MADFLIGHT_FC3V2_IMU_INT_PIN 27
#define MADFLIGHT_FC3V2_IMU_CLKIN_PIN 26
// IMU_INT (GPIO 27) is the LSM6DSV's INT1: INFERRED from madflight (MIT, github.com/qqqlab/madflight @ 1d5efa7),
// whose LSM6DSV driver routes its data-ready interrupt to INT1_CTRL and reads it on pin_imu_int 27. The LSM6DSV has no
// clock input; GPIO 26 is not driven.

// --- IMU mounting: board FRD axis i = SIGN_i x LSM6DSV axis CHIP_AXIS_i (0 = x, 1 = y, 2 = z) ---
// sensors/profiles/madflight_fc3v2_revc.yaml, mounting_axis_signs (-1, +1, -1), no permutation; INFERRED from madflight.
#define MADFLIGHT_FC3V2_IMU_FRD_X_CHIP_AXIS 0
#define MADFLIGHT_FC3V2_IMU_FRD_X_SIGN (-1)
#define MADFLIGHT_FC3V2_IMU_FRD_Y_CHIP_AXIS 1
#define MADFLIGHT_FC3V2_IMU_FRD_Y_SIGN 1
#define MADFLIGHT_FC3V2_IMU_FRD_Z_CHIP_AXIS 2
#define MADFLIGHT_FC3V2_IMU_FRD_Z_SIGN (-1)

// --- Serial ports: SER0 (radio receiver), SER1 (GPS) ---
// SER0 is wired to the ExpressLRS receiver (Luis, 2026-10-16). GPIO 0 and 1 are UART0 TX and RX (the RP2350 GPIO function
// table; pico-sdk boards/pico2.h: PICO_DEFAULT_UART 0, TX pin 0, RX pin 1).
#define MADFLIGHT_FC3V2_SER0_UART 0
#define MADFLIGHT_FC3V2_SER0_TX_PIN 0
#define MADFLIGHT_FC3V2_SER0_RX_PIN 1
#define MADFLIGHT_FC3V2_SER1_TX_PIN 4
#define MADFLIGHT_FC3V2_SER1_RX_PIN 5

// --- Motor outputs OUT0-OUT3 (ESC for motors 1-4) ---
#define MADFLIGHT_FC3V2_OUT0_PIN 6
#define MADFLIGHT_FC3V2_OUT1_PIN 7
#define MADFLIGHT_FC3V2_OUT2_PIN 8
#define MADFLIGHT_FC3V2_OUT3_PIN 9

// --- I2C0: sensor bus (barometer, battery monitor, magnetometer) ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 32
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 33
#endif

// --- LEDs ---
// The page publishes no active level for either LED.
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 45
#endif
#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 46
#endif

// --- FLASH ---
// W25Q080 boot stage 2 and PICO_FLASH_SPI_CLKDIV 2 as every RP2350 board of the SDK sets them (pico2.h). The SDK has no
// RP2354 board and no RP2354-specific define; that this setting suits the RP2354B in-package flash is INFERRED until the
// first flashed image runs.
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// The SDK default, as pico2.h sets it: software support for the RP2350 A2 stepping stays on (the chip is marked A4).
pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
