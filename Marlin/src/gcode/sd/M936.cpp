/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
#include "../../inc/MarlinConfig.h"
#include "../../core/serial.h"
#include "../../../Configuration.h"
#include "../../libs/BL24CXX.h"
#include "../gcode.h"
#include "stdio.h"
#include "string.h"
#include "../parser.h"
#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/dwin/lcd_rts.h"
#endif

#define ENABLE_OTA
#if ENABLED(ENABLE_OTA)
typedef unsigned char u8;
u8* strchr_pointer;

#define OTA_FLAG_EEPROM  90

/**
 * M936: OTA update firmware.
 */
void GcodeSuite::M936()
{
  // TODO: feedrate support?
  static u8 ota_updata_flag = 0X00;
  // char temp[20];
  if (parser.seenval('V'))
  {
    const int16_t oatvalue = parser.value_celsius();
    switch (oatvalue)
    {
      case 2:
        ota_updata_flag = 0X01;
        BL24CXX::write(OTA_FLAG_EEPROM, &ota_updata_flag, sizeof(ota_updata_flag));
        delay(10);
        SERIAL_ECHOLN("M936 V2");
        // 需要将OTA升级标志位设置成1，表示下次上电需要OTA升级。
        delay(50);
        SERIAL_ECHOLN("\r\n Motherboard upgrade \r\n");
        #if ENABLED(RTS_AVAILABLE)
          rtscheck.RTS_SndData(ExchangePageBase + 41, ExchangepageAddr);
          change_page_font = 41;
          rtscheck.RTS_SndData(Error_205, ABNORMAL_PAGE_TEXT_VP);
        #endif
        delay(50);
        nvic_sys_reset();   // MCU复位进入BOOTLOADER
        break;

      case 3:
        ota_updata_flag = 0X02;
        BL24CXX::write(OTA_FLAG_EEPROM, &ota_updata_flag, sizeof(ota_updata_flag));
        delay(10);
        SERIAL_ECHOLN("M936 V3");
        // 需要将OTA升级标志位设置成1，表示下次上电需要OTA升级。
        delay(50);
        SERIAL_ECHOLN("\r\n DIWIN upgrade！！ \r\n");
        #if ENABLED(RTS_AVAILABLE)
          rtscheck.RTS_SndData(ExchangePageBase + 41, ExchangepageAddr);
          change_page_font = 41;
          rtscheck.RTS_SndData(Error_206, ABNORMAL_PAGE_TEXT_VP);
        #endif
        delay(50);
        nvic_sys_reset();   // MCU复位进入BOOTLOADER
        break;

      default:
        break;
    }
  }
}

#endif // DIRECT_STEPPING
