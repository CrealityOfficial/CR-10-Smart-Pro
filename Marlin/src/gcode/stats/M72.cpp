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

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../lcd/ultralcd.h"
#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/dwin/lcd_rts.h"
#endif
#include "../../feature/runout.h"

/**
 * M72: Open a file
 *
 * The path is relative to the root directory
 */
void GcodeSuite::M72()
{
  // Simplify3D includes the size, so zero out all spaces (#7227)
  for (char *fn = parser.string_arg; *fn; ++fn)
  {
    if (*fn == ' ')
    {
      *fn = '\0';
    }
  }
  card.openFileRead(parser.string_arg);

  #if ENABLED(RTS_AVAILABLE)
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      if(runout.enabled == true)
      {
        runout.filament_ran_out = false;
      }
    #endif
    if(PoweroffContinue == false)
    {
      int fileselect = 0;
      for(int i = 0;i < 20;i ++)
      {
        // clean print file
        rtscheck.RTS_SndData(0, PRINT_FILE_TEXT_VP + i);
      }

      char *cloudfilename = parser.string_arg;
      int cloudnamelen = strlen(parser.string_arg);
      for(fileselect = 0;fileselect < CardRecbuf.Filesum;fileselect ++)
      {
        if(!strncmp(CardRecbuf.Cardshowfilename[fileselect], cloudfilename, cloudnamelen))
        {
          break;
        }
      }
      int j = 1;
      while((strncmp(&cloudfilename[j], ".gcode", 6) && strncmp(&cloudfilename[j], ".GCODE", 6)) && ((j ++) < cloudnamelen));

      if (j >= TEXTBYTELEN)
      {
        strncpy(&parser.string_arg[TEXTBYTELEN - 3], "..", 2);
        parser.string_arg[TEXTBYTELEN - 1] = '\0';
        j = TEXTBYTELEN - 1;
      }

      strncpy(CardRecbuf.Cardshowfilename[fileselect], parser.string_arg, j);

      rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[fileselect], PRINT_FILE_TEXT_VP);
    }
  #endif

  TERN_(LCD_SET_PROGRESS_MANUALLY, ui.set_progress(0));
}

#endif // SDSUPPORT
