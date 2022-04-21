/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * CREALITY (STM32F103) board pin assignments
 */

#ifndef __STM32F1__
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#endif

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "CREALITY supports up to 1 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "CREALITY V4"
#define DEFAULT_MACHINE_NAME "Creality V2.5S1"

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  // FLASH
  //#define FLASH_EEPROM_EMULATION

  // I2C
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0 used only for display settings
  #if ENABLED(IIC_BL24CXX_EEPROM)
    #define IIC_EEPROM_SDA                  PA11
    #define IIC_EEPROM_SCL                  PA12
    #define MARLIN_EEPROM_SIZE 0x800              // 2Kb (24C16)
  #else
    #define SDCARD_EEPROM_EMULATION               // SD EEPROM until all EEPROM is BL24CXX
    #define MARLIN_EEPROM_SIZE 0x800              // 2Kb
  #endif

  // SPI
  //#define SPI_EEPROM                            // EEPROM on SPI-0
  //#define SPI_CHAN_EEPROM1  ?
  //#define SPI_EEPROM1_CS    ?

  // 2K EEPROM
  //#define SPI_EEPROM2_CS    ?

  // 32Mb FLASH
  //#define SPI_FLASH_CS      ?
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PC4
#define Y_STOP_PIN                          PC5

//
// Servos
//
#if ENABLED(BLTOUCH)
  #define Z_MIN_PIN                           PC15   // BLTouch IN PIN
  #define SERVO0_PIN                          PC14   // BLTouch OUT PIN
#elif ENABLED(PROBE_ACTIVATION_SWITCH)
  #define Z_MIN_PIN                           PC15
  #define PROBE_TARE_PIN                      PC14
  #define PROBE_ACTIVATION_SWITCH_PIN         PB2
#else
  #define Z_MIN_PIN                           PC15
#endif

//
// Filament Runout Sensor
//
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_PIN                      PA15   // "Pulled-high"
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC3
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB7

#define Y_ENABLE_PIN                        PC3
#define Y_STEP_PIN                          PB6
#define Y_DIR_PIN                           PB5

#define Z_ENABLE_PIN                        PC3
#define Z_STEP_PIN                          PB4
#define Z_DIR_PIN                           PB3

#define E0_ENABLE_PIN                       PC3
#define E0_STEP_PIN                         PC2
#define E0_DIR_PIN                          PB9

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB1   // TH1
#define TEMP_BED_PIN                        PB0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB14  // HEATER1
#define HEATER_BED_PIN                      PB13  // HOT BED

#define FAN_PIN                             PB15  // FAN
#define FAN1_PIN                            PC13  // FAN
#define FAN_SOFT_PWM

//
// SD Card
//
#define SD_DETECT_PIN                       PC7
#define SDCARD_CONNECTION                ONBOARD
#define ON_BOARD_SPI_DEVICE                    1
#define ONBOARD_SD_CS_PIN                   PC12   // SDSS
#define SDIO_SUPPORT
#define NO_SD_HOST_DRIVE                           // This board's SD is only seen by the printer

#if ENABLED(CR10_STOCKDISPLAY) && NONE(RET6_12864_LCD, VET6_12864_LCD)
  #error "Define RET6_12864_LCD or VET6_12864_LCD to select pins for CR10_STOCKDISPLAY with the Creality V4 controller."
#endif

#if ENABLED(RET6_12864_LCD)

  /* RET6 12864 LCD */
  #define LCD_PINS_RS                       PA4
  #define LCD_PINS_ENABLE                   PA6
  #define LCD_PINS_D4                       PA5

  #define BTN_ENC                           PB2
  #define BTN_EN1                           PA2
  #define BTN_EN2                           PC1

  #define BEEPER_PIN                        PC6
#elif ENABLED(VET6_12864_LCD)

  /* VET6 12864 LCD */
  #define LCD_PINS_RS                       PA4
  #define LCD_PINS_ENABLE                   PA7
  #define LCD_PINS_D4                       PA5

  #define BTN_ENC                           PC5
  #define BTN_EN1                           PB10
  #define BTN_EN2                           PA6

#elif ENABLED(DWIN_CREALITY_LCD)

  /* RET6 DWIN ENCODER LCD */
  #define BTN_ENC                           PB14
  #define BTN_EN1                           PB15
  #define BTN_EN2                           PB12

  //#define LCD_LED_PIN                     PB2
  #define BEEPER_PIN                        PB13

#elif ENABLED(DWIN_VET6_CREALITY_LCD)

  /* VET6 DWIN ENCODER LCD */
  #define BTN_ENC                           PA6
  #define BTN_EN1                           PA7
  #define BTN_EN2                           PA4

  #define BEEPER_PIN                        PA5

#endif

#define LED_CONTROL_PIN     PA7

//
// Suicide Power
//
#define SHUTIDOWN_PIN       PA0
#define MOTOR_CIRCUIT_PIN   PA1

//
// Motor Protect
//
#define MOTOR_PROTECT_PIN   PC0

//
// WiFI Reset
//
#define RESET_WIFI_PIN      PB12
