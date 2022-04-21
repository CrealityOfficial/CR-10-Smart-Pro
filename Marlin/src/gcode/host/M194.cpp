/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#if ENABLED(RTS_AVAILABLE)
  #include "../../../src/lcd/dwin/lcd_rts.h"
#endif
#include "../../MarlinCore.h"

extern uint8_t wifi_enable_flag;

/**
 * M194: RESET WIFI MODULE
 */
void GcodeSuite::M194()
{
  wifi_enable_flag = parser.seenval('S');
  #if ENABLED(HAS_MENU_RESET_WIFI)
    // record press state and output low level
    WIFI_STATE = PRESSED;
    OUT_WRITE(RESET_WIFI_PIN, LOW);

    SERIAL_ECHOPGM(STR_OK);
    SERIAL_ECHOPGM("wifi is reseting");
  #endif
}