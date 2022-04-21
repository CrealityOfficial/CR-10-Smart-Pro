#include "lcd_rts.h"
#include <wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <libmaple/usart.h>
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../module/configuration_store.h"
#include "../../core/serial.h"
#include "../../core/macros.h"

#include "../fontutils.h"
#include "../ultralcd.h"
#include "../../sd/cardreader.h"
#include "../../feature/powerloss.h"
#include "../../feature/runout.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../../module/probe.h"

#include "../../libs/duration_t.h"

#if ENABLED(BLTOUCH)
  #include "../../module/endstops.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #include "../../feature/bedlevel/bedlevel.h"
  #include "../../feature/bedlevel/abl/abl.h"
#endif

#if ENABLED(RTS_AVAILABLE)
#if ENABLED(HAS_MENU_RESET_WIFI)
  unsigned char WIFI_STATE = INITIAL;
#endif

char errorway = 0;
char errornum = 0;
char home_errornum = 0;
char error_sd_num = 0;

#if ENABLED(BABYSTEPPING)
  float zprobe_zoffset;
  float last_zoffset = 0.0;
#endif

int power_off_type_yes = 0;

const float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 60};
constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

float default_nozzle_ptemp = DEFAULT_Kp;
float default_nozzle_itemp = DEFAULT_Ki;
float default_nozzle_dtemp = DEFAULT_Kd;

float default_hotbed_ptemp = DEFAULT_bedKp;
float default_hotbed_itemp = DEFAULT_bedKi;
float default_hotbed_dtemp = DEFAULT_bedKd;

float cache_pla_nozzletemp = PREHEAT_1_TEMP_HOTEND;
float cache_pla_hotbedtemp = PREHEAT_1_TEMP_BED;

float cache_abs_nozzletemp = PREHEAT_2_TEMP_HOTEND;
float cache_abs_hotbedtemp = PREHEAT_2_TEMP_BED;

int startprogress = 0;
CRec CardRecbuf;
int16_t temphot = 0;
uint8_t afterprobe_fan0_speed = 0;
float pause_e = 0;
bool sdcard_pause_check = true;
bool pause_action_flag = false;
bool print_preheat_check = false;
bool probe_offset_flag = false;

float ChangeFilamentTemp = 200;
int heatway = 0;
millis_t next_rts_update_ms      = 0;
millis_t next_shutdown_update_ms = 0;
millis_t next_wifireset_update_ms = 0;
unsigned int count_ms = 0;
unsigned int wifiresetcount_ms = 0;
unsigned long count_lcd_down = 0;
bool flag_lcd_down = false;

int last_target_temperature[4] = {0};
int last_target_temperature_bed;
char waitway = 0;
int change_page_font = 1;
unsigned char Percentrecord = 0;
// represents to update file list
bool CardUpdate = false;

uint8_t fileCnt = 0;
uint8_t file_current_page = 1;
uint8_t file_total_page = 1;
uint8_t page_total_file = 0;

extern CardReader card;

RTSSHOW rtscheck;

uint8_t lang = 2;
// represents SD-card status, true means SD is available, false means opposite.
bool lcd_sd_status;

char Checkfilenum = 0;
char cmdbuf[20] = {0};
float FilamentLOAD = 10;
float FilamentUnLOAD = 10;

// 1 for 10mm, 2 for 1mm, 3 for 0.1mm
unsigned char AxisUnitMode;
float axis_unit = 10.0;
bool LEDStatus = false;

int Update_Time_Value = 0;

bool PoweroffContinue = false;
char commandbuf[30];

uint16_t remain_time = 0;

bool flag_over_shutdown = false;
bool flag_counter_printover_to_shutdown = false;
bool flag_counter_wifireset = false;

xyze_pos_t resume_position;

inline void RTS_line_to_current(AxisEnum axis)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
  }
}

static void RTS_line_to_filelist()
{
  for(int j = 0;j < 4;j ++)
  {
    // clean filename Icon
    for(int i = 0;i < TEXTBYTELEN;i ++)
    {
      rtscheck.RTS_SndData(0, CardRecbuf.addr[j] + i);
    }
  }
  memset(&CardRecbuf, 0, sizeof(CardRecbuf));

  int num = 0;
  for(uint16_t i = (file_current_page - 1) * 4; i < (file_current_page * 4);i ++)
  {
    card.selectFileByIndex(fileCnt - 1 - i);
    char *pointFilename = card.longFilename;
    int filenamelen = strlen(card.longFilename);
    int j = 1;
    while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen));

    if (j >= TEXTBYTELEN)
    {
      strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
      card.longFilename[TEXTBYTELEN - 1] = '\0';
      j = TEXTBYTELEN - 1;
    }

    strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, j);

    strcpy(CardRecbuf.Cardfilename[num], card.filename);
    CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 20);
    rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);
    CardRecbuf.Filesum = (++num);
  }
  page_total_file = CardRecbuf.Filesum;
  CardRecbuf.Filesum = ((file_total_page - 1) * 4) + page_total_file;
}

RTSSHOW::RTSSHOW(void)
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

void RTSSHOW::RTS_SDCardInit(void)
{
  if(RTS_SD_Detected())
  {
    card.mount();
  }
  if(CardReader::flag.mounted)
  {
    fileCnt = card.get_num_Files();
    card.getWorkDirName();
    if(card.filename[0] != '/')
    {
      card.cdup();
    }

    if(fileCnt > 4)
    {
      file_total_page = (fileCnt / 4) + 1;
    }
    else
    {
      file_total_page = 1;
    }
    RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
    file_current_page = 1;
    RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

    RTS_line_to_filelist();

    if(PoweroffContinue == true)
    {
      return;
    }
    else
    {
      for(int j = 0;j < 20;j ++)
      {
        // clean print file
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
      }
    }
    lcd_sd_status = IS_SD_INSERTED();
  }
  else
  {
    if(PoweroffContinue == true)
    {
      return;
    }
    else
    {
      // clean filename Icon
      for(int j = 0;j < MaxFileNumber;j ++)
      {
        // clean filename Icon
        for(int i = 0;i < TEXTBYTELEN;i ++)
        {
          RTS_SndData(0, CardRecbuf.addr[j] + i);
        }
      }
      memset(&CardRecbuf, 0, sizeof(CardRecbuf));
    }
  }
}

bool RTSSHOW::RTS_SD_Detected(void)
{
  static bool last;
  static bool state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if(tmp != last)
  {
    flag_stable = false;
  }
  else
  {
    if(!flag_stable)
    {
      flag_stable = true;
      stable_point_time = millis();
    }
  }

  if(flag_stable)
  {
    if((millis() - stable_point_time) > 30)
    {
      state = tmp;
    }
  }

  last = tmp;

  return state;
}

void RTSSHOW::RTS_SDCardUpate(void)
{
  const bool sd_status = RTS_SD_Detected();
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      // SD card power on
      card.mount();
      RTS_SDCardInit();
    }
    else
    {
      if(PoweroffContinue == true)
      {
        return;
      }
      else
      {
        card.release();
        for(int i = 0;i < CardRecbuf.Filesum;i ++)
        {
          for(int j = 0;j < 20;j ++)
          {
            RTS_SndData(0, CardRecbuf.addr[i] + j);
          }
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
        }

        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        memset(&CardRecbuf, 0, sizeof(CardRecbuf));
        RTS_SndData(1, PRINT_COUNT_PAGE_DATA_VP);
        file_current_page = 1;
        RTS_SndData(1, PRINT_CURRENT_PAGE_DATA_VP);
      }
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
  {
    RTS_line_to_filelist();
    for(uint16_t i = 0;i < 5;i ++)
    {
      delay(1);
      RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
    }
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_Init(void)
{
  AxisUnitMode = 1;
  lang = language_change_font;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bool zig = true;
    int8_t inStart, inStop, inInc, showcount;
    showcount = 0;
    settings.load();
    for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
    {
      // away from origin
      if (zig)
      {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else
      {
        // towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for (int x = inStart; x != inStop; x += inInc)
      {
        RTS_SndData(z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }
    queue.enqueue_now_P(PSTR("M420 S1"));
  #endif
  last_zoffset = zprobe_zoffset = probe.offset.z;
  RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

  for(int i = 0;i < 9;i ++)
  {
    RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
  }
  RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
  languagedisplayUpdate();
  delay(500);

  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature_bed = thermalManager.temp_bed.target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, HEAD_SET_TEMP_VP);
  RTS_SndData(0, BED_SET_TEMP_VP);
  RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
  delay(20);
  RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
  delay(20);
  RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
  RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
  RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);

  RTS_SndData(planner.settings.max_feedrate_mm_s[0], MAX_VELOCITY_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[1], MAX_VELOCITY_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[2], MAX_VELOCITY_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_feedrate_mm_s[3], MAX_VELOCITY_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[0], MAX_ACCEL_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[1], MAX_ACCEL_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[2], MAX_ACCEL_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.max_acceleration_mm_per_s2[3], MAX_ACCEL_EAXIS_DATA_VP);

  RTS_SndData(planner.max_jerk.x * 100, MAX_JERK_XAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.y * 100, MAX_JERK_YAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.z * 100, MAX_JERK_ZAXIS_DATA_VP);
  RTS_SndData(planner.max_jerk.e * 100, MAX_JERK_EAXIS_DATA_VP);

  RTS_SndData(planner.settings.axis_steps_per_mm[0] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[1] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[2] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[3] * 10, MAX_STEPSMM_EAXIS_DATA_VP);

  RTS_SndData(PID_PARAM(Kp, e) * 100, NOZZLE_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(PID_PARAM(Ki, e)) * 100, NOZZLE_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(PID_PARAM(Kd, e)) * 100, NOZZLE_TEMP_D_DATA_VP);
  RTS_SndData(thermalManager.temp_bed.pid.Kp * 100, HOTBED_TEMP_P_DATA_VP);
  RTS_SndData(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100, HOTBED_TEMP_I_DATA_VP);
  RTS_SndData(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 10, HOTBED_TEMP_D_DATA_VP);

  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
  RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
  LEDStatus = false;
  delay(5);
  if(flag_over_shutdown)
  {
    rtscheck.RTS_SndData(0, PRINTER_AUTO_SHUTDOWN_ICON_VP);
  }
  else
  {
    rtscheck.RTS_SndData(1, PRINTER_AUTO_SHUTDOWN_ICON_VP);
  }

  /*********transmit SD card filename to screen***************/
  delay(200);
  RTS_SDCardInit();

  /***************transmit Printer information to screen*****************/
  RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
  RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
  RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
  RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
  RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
  delay(5);
  if(1 == lang)
  {
    RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
  }
  else
  {
    RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
  }

  if(wifi_enable_flag)
  {
    RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
    RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
  }
  else
  {
    RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
    RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
  }

  if(recovery.enabled)
  {
    RTS_SndData(0, POWERCONTINUE_CONTROL_ICON_VP);
  }
  else
  {
    RTS_SndData(1, POWERCONTINUE_CONTROL_ICON_VP);
  }

  if(runout.enabled)
  {
    RTS_SndData(0, FILAMENT_CONTROL_ICON_VP);
  }
  else
  {
    RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
  }

  /**************************some info init*******************************/
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(1, PREHAEAT_NOZZLE_ICON_VP);
  RTS_SndData(1, PREHAEAT_HOTBED_ICON_VP);

  rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
  change_page_font = 0;

  for(startprogress = 0; startprogress <= 100; startprogress++)
  {
    rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
    delay(150);
  }

  delay(500);
  delay(500);

  Update_Time_Value = RTS_UPDATE_VALUE;
}

int RTSSHOW::RTS_RecData(void)
{
  static int recnum = 0;
  while((MYSERIAL1.available() > 0) && (recnum < SizeofDatabuf))
  {
    delay(1);
    databuf[recnum] = MYSERIAL1.read();
    if(databuf[0] == FHONE)
    {
      recnum++;
    }
    else if(databuf[0] == FHTWO)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      recnum += 2;
    }
    else if(databuf[0] == FHLENG)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      recnum += 3;
    }
    else if(databuf[0] == VarAddr_R)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      databuf[3] = VarAddr_R;
      recnum += 4;
    }
    else
    {
      recnum = 0;
    }
  }

  // receive nothing
  if(recnum < 1)
  {
    return -1;
  }
  else if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && (recnum > 2))
  {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    // response for writing byte
    if((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
    {
      memset(databuf, 0, sizeof(databuf));
      recnum = 0;
      return -1;
    }
    else if(recdat.command == 0x83)
    {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8) | databuf[5];
      recdat.bytelen = databuf[6];
      for(unsigned int i = 0; i < recdat.bytelen; i += 2)
      {
        recdat.data[i / 2] = databuf[7 + i];
        recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
      }
    }
    else if(recdat.command == 0x81)
    {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
      for(unsigned int i = 0; i < recdat.bytelen; i ++)
      {
        recdat.data[i] = databuf[6 + i];
        // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
      }
    }
  }
  else
  {
    memset(databuf, 0, sizeof(databuf));
    recnum = 0;
    // receive the wrong data
    return -1;
  }
  memset(databuf, 0, sizeof(databuf));
  recnum = 0;
  return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;

    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    usart_tx(MYSERIAL1.c_dev(), databuf, snddat.len + 3);
    MYSERIAL1.flush();
    // for(int i = 0;i < (snddat.len + 3); i ++)
    // {
    //   MYSERIAL1.write(databuf[i]);
    //   delayMicroseconds(1);
    // }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }

    for(uint8_t i = 0;i < (len + 6);i ++)
    {
      MYSERIAL1.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SDcard_Stop(void)
{
  if(PoweroffContinue == true)
  {
    planner.synchronize();
    // card.endFilePrint();
    card.flag.abort_sd_printing = true;
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif
    print_job_timer.reset();
    thermalManager.setTargetHotend(0, 0);
    RTS_SndData(0, HEAD_SET_TEMP_VP);
    thermalManager.setTargetBed(0);
    RTS_SndData(0, BED_SET_TEMP_VP);
    temphot = 0;
    thermalManager.zero_fan_speeds();
    wait_for_heatup = wait_for_user = false;
    PoweroffContinue = false;
    sd_printing_autopause = false;
    if(CardReader::flag.mounted)
    {
      #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
        card.removeJobRecoveryFile();
      #endif
    }
    flag_counter_printover_to_shutdown = false;
    CardRecbuf.recordcount = -1;
  }
  SERIAL_ECHOLN("M79 S4");   // 4:cloud print stop

  // shut down the stepper motor.
  // queue.enqueue_now_P(PSTR("M84"));
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  delay(2);
  for(int j = 0;j < 20;j ++)
  {
    // clean screen.
    RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    // clean filename
    RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
  }
  RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  change_page_font = 1;
}

void RTSSHOW::RTS_HandleData(void)
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i ++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      Checkkey = i;
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch(Checkkey)
  {
    case MainEnterKey:
      if(recdat.data[0] == 1)
      {
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        for (int j = 0; j < 20; j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        for(int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, FILE1_SELECT_ICON_VP + j);
        }
        RTS_SDCardUpate();

        RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
        file_current_page = 1;
        RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

        RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
        if(CardReader::flag.mounted)
        {
          RTS_line_to_filelist();
        }
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
        change_page_font = 25;
      }
      else if(recdat.data[0] == 5)
      {
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_VP);
        delay(2);
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        print_job_timer.reset();
        sd_printing_autopause = false;
        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        CardRecbuf.recordcount = -1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
        flag_counter_printover_to_shutdown = false;
      }
      else if(recdat.data[0] == 6)
      {
        #if ENABLED(BLTOUCH)
          waitway = 3;
          RTS_SndData(lang, BEDLEVELING_WAIT_TITLE_VP);
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;

          #if ENABLED(PROBING_HEATERS_OFF)
            if(thermalManager.temp_hotend[0].target > 0)
            {
              temphot = thermalManager.temp_hotend[0].target;
              thermalManager.setTargetHotend(0, 0);
              rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
            }
          #endif

          #if ENABLED(PROBING_FANS_OFF)
            afterprobe_fan0_speed = thermalManager.fan_speed[0];
            if(thermalManager.fan_speed[0] > 0)
            {
              thermalManager.set_fan_speed(0, 0);
              rtscheck.RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
            }
            if(!thermalManager.fan_speed[1])
            {
              thermalManager.set_fan_speed(1, 0);
            }
          #endif

          queue.enqueue_now_P(PSTR("G28\nG29"));
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
        #endif
      }
      else if(recdat.data[0] == 7)
      {
        if(errorway == 1)
        {

        }
        else if(errorway == 2)
        {
          // auto home fail
        }
        else if(errorway == 3)
        {
          // bed leveling fail
        }
        else if(errorway == 4) 
        {

        }
      }
      else if(recdat.data[0] == 8)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      break;

    case AdjustEnterKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.fan_speed[0] ? RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP) : RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);

        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          if(card.isPrinting())
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
        }
        else
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        settings.save();
      }
      else if(recdat.data[0] == 3)
      {
        if(thermalManager.fan_speed[0])
        {
          RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
          thermalManager.set_fan_speed(0, 0);
        }
        else
        {
          RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
          thermalManager.set_fan_speed(0, 255);
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(LEDStatus)
        {
          RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
          digitalWrite(LED_CONTROL_PIN, HIGH);
          LEDStatus = false;
        }
        else
        {
          RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
          digitalWrite(LED_CONTROL_PIN, LOW);
          LEDStatus = true;
        }
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        change_page_font = 15;
      }
      else if(recdat.data[0] == 6)
      {
        if(flag_over_shutdown)
        {
          RTS_SndData(1, PRINTER_AUTO_SHUTDOWN_ICON_VP);
          flag_over_shutdown = false;
        }
        else
        {
          RTS_SndData(0, PRINTER_AUTO_SHUTDOWN_ICON_VP);
          flag_over_shutdown = true;
        }
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
        settings.save();
      }
      else if(recdat.data[0] == 8)
      {
        if(runout.enabled)
        {
          RTS_SndData(1, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = false;
        }
        else
        {
          RTS_SndData(0, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = true;
        }
      }
      else if(recdat.data[0] == 9)
      {
        if(recovery.enabled)
        {
          RTS_SndData(1, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = false;
        }
        else
        {
          RTS_SndData(0, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = true;
        }
      }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      SERIAL_ECHOLNPAIR("M220 S", feedrate_percentage);
      break;

    case StopPrintKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 13, ExchangepageAddr);
        change_page_font = 13;
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          Update_Time_Value = 0;
          temphot = 0;
          RTS_SDcard_Stop();
        }
        else if(PoweroffContinue == false)
        {
          while(planner.movesplanned() && (queue.length >= (BUFSIZE - 2)) && (destination != current_position))
          {
            idle();
          }
          queue.clear();
          delay(20);

          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);

          queue.enqueue_now_P(PSTR("M79 S4"));   // 4:cloud print stop

          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(PoweroffContinue == true)
        {
          runout.filament_ran_out = false;
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if(runout.enabled == true)
            {
              pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
              lcd_pause_show_message(PAUSE_MESSAGE_RESUME);
            }
          #endif
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          Update_Time_Value = 0;
          temphot = 0;
          RTS_SDcard_Stop();
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
          SERIAL_ECHOLN("M79 S4");   // 4:cloud print stop
        }
        else if((CardRecbuf.recordcount > 0) && (PoweroffContinue == false))
        {
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }
        else if(PoweroffContinue == false)
        {
          runout.filament_ran_out = false;
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          while(planner.movesplanned() && (queue.length >= (BUFSIZE - 2)) && (destination != current_position))
          {
            idle();
          }
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          lcd_pause_show_message(PAUSE_MESSAGE_RESUME);
          queue.clear();
          delay(20);
          queue.enqueue_now_P(PSTR("M79 S4"));   // 4:cloud print stop

          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          Update_Time_Value = 0;
          temphot = 0;
          RTS_SDcard_Stop();
        }
      }
      break;

    case PausePrintKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        change_page_font = 11;
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          if(card.isPrinting() && (wait_for_heatup == false) && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 2)) && (thermalManager.temp_bed.celsius >= thermalManager.temp_bed.target))
          {
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            waitway = 1;
            pause_e = current_position[E_AXIS] - 3;
            if(!temphot)
            {
              temphot = thermalManager.temp_hotend[0].target;
            }

            card.pauseSDPrint();
            print_job_timer.pause();

            pause_action_flag = true;
            Update_Time_Value = 0;
            planner.synchronize();
            sdcard_pause_check = false;
          }
          else
          {
            break;
          }
        }
        else if(PoweroffContinue == false)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          queue.enqueue_now_P(PSTR("M79 S2"));   // 2:cloud print pause
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      break;

    case ResumePrintKey:
      if(recdat.data[0] == 1)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif

        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          #if ENABLED(HAS_RESUME_CONTINUE)
            if(wait_for_user)
            {
              wait_for_user = false;
            }
            else
          #endif
            {
              memset(commandbuf, 0, sizeof(commandbuf));
              sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
              queue.enqueue_one_now(commandbuf);

              card.startFileprint();
              print_job_timer.start();
              Update_Time_Value = 0;
              sdcard_pause_check = true;
            }
          SERIAL_ECHOLN("M79 S3");   // 3:cloud print resume
        }
        else
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          queue.enqueue_now_P(PSTR("M79 S3"));   // 3:cloud print resume
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(thermalManager.temp_hotend[0].target >= EXTRUDE_MINTEMP)
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        }
        else if(temphot >= EXTRUDE_MINTEMP)
        {
          thermalManager.setTargetHotend(temphot, 0);
        }
        else
        {
          thermalManager.setTargetHotend(200, 0);
        }
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if(1 == READ(FIL_RUNOUT_PIN))
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
          else
          {
            if(!planner.has_blocks_queued())
            {
              #if ENABLED(PROBING_HEATERS_OFF)
                if(temphot > 0)
                {
                  RTS_SndData(temphot, HEAD_SET_TEMP_VP);
                  thermalManager.setTargetHotend(temphot, 0);
                  if (temphot > (thermalManager.temp_hotend[0].celsius - 5))
                  {
                    thermalManager.wait_for_hotend(0);
                  }

                  while (ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
                  {
                    idle();
                  }
                }
                RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
                change_page_font = 8;
              #endif
            }
            else
            {
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              break;
            }
          }
        #endif
      }
      else if(recdat.data[0] == 3)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif
        if(PoweroffContinue == true)
        {
          runout.filament_ran_out = false;
          // Move XY to starting position, then Z
          do_blocking_move_to_xy(resume_position, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));

          // Move Z_AXIS to saved position
          do_blocking_move_to_z(resume_position.z, feedRate_t(NOZZLE_PARK_Z_FEEDRATE));
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          lcd_pause_show_message(PAUSE_MESSAGE_RESUME);
          queue.inject_P(PSTR("M108"));

          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          card.startFileprint();
          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
        }
        else if(PoweroffContinue == false)
        {
          if(CardRecbuf.recordcount > 0)
          {
            char cmd[30];
            char *c;
            sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
            for (c = &cmd[4]; *c; c++)
              *c = tolower(*c);

            PoweroffContinue = true;
            memset(cmdbuf, 0, sizeof(cmdbuf));
            strcpy(cmdbuf, cmd);
            queue.enqueue_one_now(cmd);
            delay(20);
            queue.enqueue_now_P(PSTR("M24"));
            // clean screen.
            for (int j = 0; j < 20; j ++)
            {
              RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
            }

            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

            delay(2);

            #if ENABLED(BABYSTEPPING)
              RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
            #endif
            feedrate_percentage = 100;
            RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            Update_Time_Value = 0;
            // 1:cloud print satrt
            SERIAL_ECHOLN("M79 S1");
          }
          else if(runout.filament_ran_out == true)
          {
            runout.filament_ran_out = false;
            // Move XY to starting position, then Z
            do_blocking_move_to_xy(resume_position, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));

            // Move Z_AXIS to saved position
            do_blocking_move_to_z(resume_position.z, feedRate_t(NOZZLE_PARK_Z_FEEDRATE));
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;

            // 3:cloud print resume
            SERIAL_ECHOLN("M79 S3");

            card.startFileprint();
            print_job_timer.start();
            Update_Time_Value = 0;
            sdcard_pause_check = true;
          }
          else
          {
            runout.filament_ran_out = false;
            // Move XY to starting position, then Z
            do_blocking_move_to_xy(resume_position, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));

            // Move Z_AXIS to saved position
            do_blocking_move_to_z(resume_position.z, feedRate_t(NOZZLE_PARK_Z_FEEDRATE));

            pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
            lcd_pause_show_message(PAUSE_MESSAGE_RESUME);

            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;

            print_job_timer.start();
            Update_Time_Value = 0;
            sdcard_pause_check = true;
          }
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(CardReader::flag.mounted)
        {
          SD_Card_status = true;
          card.startFileprint();
          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          sd_printing_autopause = false;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else
        {
          CardUpdate = true;
          RTS_SDCardUpate();
          // card.mount();
          RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
          change_page_font = 47;
        }
      }
      break;

    case ZoffsetEnterKey:
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
      {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
      }
      probe.offset.z = zprobe_zoffset;
      // settings.save();
      break;

    case TempControlKey:
      if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
        change_page_font = 22;
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        change_page_font = 23;
      }
      else if(recdat.data[0] == 5)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[0].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[0].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if(recdat.data[0] == 6)
      {
        thermalManager.temp_hotend[0].target = ui.material_preset[1].hotend_temp;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        thermalManager.temp_bed.target = ui.material_preset[1].bed_temp;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 8)
      {
        RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      break;

    case CoolDownKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      break;

    case HeaterTempEnterKey:
      temphot = recdat.data[0];
      thermalManager.temp_hotend[0].target = temphot;
      thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      break;

    case HotBedTempEnterKey:
      thermalManager.temp_bed.target = recdat.data[0];
      thermalManager.setTargetBed(thermalManager.temp_bed.target);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      break;

    case PrepareEnterKey:
      if(recdat.data[0] == 1)
      {
        RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
        change_page_font = 28;
      }
      else if(recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        delay(2);
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if(recdat.data[0] == 4)
      {
        OUT_WRITE(SHUTIDOWN_PIN, LOW);
        delay(2000);
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
        RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
        RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
        RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
        RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
        delay(5);
        if(1 == lang)
        {
          RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
        }
        else
        {
          RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
        }
        RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
        change_page_font = 24;
      }
      else if(recdat.data[0] == 6)
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
        change_page_font = 43;
      }
      else if(recdat.data[0] == 8)
      {
        ui.material_preset[0].hotend_temp = cache_pla_nozzletemp;
        ui.material_preset[0].bed_temp = cache_pla_hotbedtemp;

        ui.material_preset[1].hotend_temp = cache_abs_nozzletemp;
        ui.material_preset[1].bed_temp = cache_abs_hotbedtemp;
        settings.save();
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if(recdat.data[0] == 9)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if(recdat.data[0] == 0xA)
      {
        RTS_SndData(ExchangePageBase + 42, ExchangepageAddr);
        change_page_font = 42;
      }
      else if(recdat.data[0] == 0xB)
      {
         #if ENABLED(HAS_MENU_RESET_WIFI)
          WIFI_STATE = PRESSED;
          OUT_WRITE(RESET_WIFI_PIN, LOW);
        #endif
        flag_counter_wifireset = true;
        RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
        change_page_font = 45;
      }
      else if(recdat.data[0] == 0xC)
      {
        RTS_SndData(ExchangePageBase + 44, ExchangepageAddr);
        change_page_font = 44;
      }
      else if(recdat.data[0] == 0xD)
      {
        settings.reset();
        settings.save();
        language_change_font = lang = 2;
        languagedisplayUpdate();
        for(int i = 0;i < 9;i ++)
        {
          RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        }
        RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;

        RTS_SndData(PREHEAT_1_TEMP_HOTEND, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(PREHEAT_1_TEMP_BED, PREHEAT_PLA_SET_BED_TEMP_VP);
        RTS_SndData(PREHEAT_2_TEMP_HOTEND, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(PREHEAT_2_TEMP_BED, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(1000);
      }
      else if(recdat.data[0] == 0xE)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
          change_page_font = 33;
        }
      }
      else if(recdat.data[0] == 0xF)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          change_page_font = 21;
        }
        settings.save();
      }
      break;

    case BedLevelKey:
      if(recdat.data[0] == 1)
      {
        planner.synchronize();
        waitway = 6;
        if((!TEST(axis_known_position, X_AXIS)) || (!TEST(axis_known_position, Y_AXIS)))
        {
          queue.enqueue_now_P(PSTR("G28"));
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28 Z0"));
        }
        queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 2)
      {
        last_zoffset = zprobe_zoffset;
        if(WITHIN((zprobe_zoffset + 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.05);
            zprobe_zoffset = zprobe_zoffset - 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if(recdat.data[0] == 3)
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.05), -5.02, 5.02))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.05);
            zprobe_zoffset = zprobe_zoffset + 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          change_page_font = 25;
        }
      }
      else if(recdat.data[0] == 5)
      {
        // Assitant Level ,  Centre 1
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (TEST(axis_known_position, Z_AXIS)))
        {
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X150 Y150 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
      }
      else if (recdat.data[0] == 6)
      {
        // Assitant Level , Front Left 2
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (TEST(axis_known_position, Z_AXIS)))
        {
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X40 Y40 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
      }
      else if (recdat.data[0] == 7)
      {
        // Assitant Level , Front Right 3
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (TEST(axis_known_position, Z_AXIS)))
        {
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X260 Y40 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
      }
      else if (recdat.data[0] == 8)
      {
        // Assitant Level , Back Right 4
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (TEST(axis_known_position, Z_AXIS)))
        {
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X260 Y260 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
      }
      else if (recdat.data[0] == 9)
      {
        // Assitant Level , Back Left 5
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (TEST(axis_known_position, Z_AXIS)))
        {
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            queue.enqueue_now_P(PSTR("G1 X40 Y260 F3000"));
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            waitway = 0;
          }
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
      }
      else if(recdat.data[0] == 0x0A)
      {
        RTS_SndData(0, AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(0, AUTO_LEVELING_PERCENT_DATA_VP);
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;
        }
      }
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      break;

    case AutoHomeKey:
      if(recdat.data[0] == 1)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
        RTS_SndData(3, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
        change_page_font = 17;
        RTS_SndData(2, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);
        change_page_font = 18;
        RTS_SndData(1, MOVEAXIS_UNIT_ICON_VP);
      }
      else if(recdat.data[0] == 4)
      {
        waitway = 4;
        queue.enqueue_now_P(PSTR("G28 X Y"));
        Update_Time_Value = 0;
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      else if(recdat.data[0] == 5)
      {
        waitway = 4;
        if((TEST(axis_known_position, X_AXIS)) && (TEST(axis_known_position, Y_AXIS)) && (current_position[X_AXIS] >= 50) && (current_position[Y_AXIS] >= 50))
        {
          queue.enqueue_now_P(PSTR("G28 Z"));
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28"));
        }
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        Update_Time_Value = 0;
      }
      break;

    case XaxismoveKey:
      float x_min, x_max;
      waitway = 4;
      x_min = 0;
      x_max = X_MAX_POS;
      current_position[X_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[X_AXIS] < x_min)
      {
        current_position[X_AXIS] = x_min;
      }
      else if(current_position[X_AXIS] > x_max)
      {
        current_position[X_AXIS] = x_max;
      }
      RTS_line_to_current(X_AXIS);
      RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case YaxismoveKey:
      float y_min, y_max;
      waitway = 4;
      y_min = 0;
      y_max = Y_MAX_POS;
      current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[Y_AXIS] < y_min)
      {
        current_position[Y_AXIS] = y_min;
      }
      else if(current_position[Y_AXIS] > y_max)
      {
        current_position[Y_AXIS] = y_max;
      }
      RTS_line_to_current(Y_AXIS);
      RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case ZaxismoveKey:
      float z_min, z_max;
      waitway = 4;
      z_min = Z_MIN_POS;
      z_max = Z_MAX_POS;
      current_position[Z_AXIS] = ((float)recdat.data[0])/10;
      if (current_position[Z_AXIS] < z_min)
      {
        current_position[Z_AXIS] = z_min;
      }
      else if (current_position[Z_AXIS] > z_max)
      {
        current_position[Z_AXIS] = z_max;
      }
      RTS_line_to_current(Z_AXIS);
      RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
      delay(1);
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      waitway = 0;
      break;

    case HeaterLoadEnterKey:
      FilamentLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            break;
          }
        #endif
        current_position[E_AXIS] += FilamentLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
          break;
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
          break;
        }
        else
        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterUnLoadEnterKey:
      FilamentUnLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            break;
          }
        #endif
        current_position[E_AXIS] -= FilamentUnLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
          break;
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
          break;
        }
        else
        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterLoadStartKey:
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
              change_page_font = 46;
              break;
            }
          #endif

          if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
          {
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
            break;
          }
          else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
          {
            thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
            RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
            break;
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            planner.synchronize();
          }
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
      }
      break;

    case SelectLanguageKey:
      if(recdat.data[0] != 0)
      {
        lang = recdat.data[0];
      }
      language_change_font = lang;
      for(int i = 0;i < 9;i ++)
      {
        RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
      }
      RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
      languagedisplayUpdate();
      settings.save();
      break;

    case PowerContinuePrintKey:
      if(recdat.data[0] == 1)
      {
        if((recovery.info.recovery_flag == true) && (PoweroffContinue == true))
        {
          PoweroffContinue = true;
          power_off_type_yes = 1;
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          // recovery.resume();
          queue.enqueue_now_P(PSTR("M1000"));

          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          // 3:cloud print resume
          SERIAL_ECHOLN("M79 S3");
        }
        else if((recovery.info.recovery_flag == true) && (PoweroffContinue == false))
        {
          queue.enqueue_now_P(PSTR("M79 S3"));   // 3:cloud print resume
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(PoweroffContinue == true)
        {
          Update_Time_Value = RTS_UPDATE_VALUE;
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;

          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          Update_Time_Value = 0;
          RTS_SDcard_Stop();
        }
        else if(PoweroffContinue == false)
        {
          queue.enqueue_now_P(PSTR("M79 S4"));   // 4:cloud print stop
        }
      }
      break;

    case PLAHeadSetEnterKey:
      cache_pla_nozzletemp = recdat.data[0];
      RTS_SndData(cache_pla_nozzletemp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      break;

    case PLABedSetEnterKey:
      cache_pla_hotbedtemp = recdat.data[0];
      RTS_SndData(cache_pla_hotbedtemp, PREHEAT_PLA_SET_BED_TEMP_VP);
      break;

    case ABSHeadSetEnterKey:
      cache_abs_nozzletemp = recdat.data[0];
      RTS_SndData(cache_abs_nozzletemp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      break;

    case ABSBedSetEnterKey:
      cache_abs_hotbedtemp = recdat.data[0];
      RTS_SndData(cache_abs_hotbedtemp, PREHEAT_ABS_SET_BED_TEMP_VP);
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 1)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      if(recdat.data[0] == 2)
      {
        queue.enqueue_now_P(PSTR("M502"));
        language_change_font = lang = 2;
        languagedisplayUpdate();
        for(int i = 0;i < 9;i ++)
        {
          RTS_SndData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        }
        RTS_SndData(1, LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
        last_zoffset = zprobe_zoffset = probe.offset.z = 0;
        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();

        RTS_SndData(PREHEAT_1_TEMP_HOTEND, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        RTS_SndData(PREHEAT_1_TEMP_BED, PREHEAT_PLA_SET_BED_TEMP_VP);
        RTS_SndData(PREHEAT_2_TEMP_HOTEND, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        RTS_SndData(PREHEAT_2_TEMP_BED, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(1000);
      }
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if(recdat.data[0] == 4)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        change_page_font = 34;
      }
      else if(recdat.data[0] == 5)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
        change_page_font = 39;
      }
      else if(recdat.data[0] == 6)
      {
        if(wifi_enable_flag)
        {
          wifi_enable_flag = 0;
          queue.inject_P(PSTR("M115"));
          RTS_SndData(1, ADV_SETTING_WIFI_ICON_VP);
          RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
          settings.save();
        }
        else
        {
          wifi_enable_flag = 1;
          queue.inject_P(PSTR("M115"));
          RTS_SndData(0, ADV_SETTING_WIFI_ICON_VP);
          RTS_SndData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
          settings.save();
        }
      }
      else if(recdat.data[0] == 7)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
        change_page_font = 38;
      }
      else if(recdat.data[0] == 8)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
        change_page_font = 36;
      }
      else if(recdat.data[0] == 9)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      else if(recdat.data[0] == 0x0A)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
        change_page_font = 35;
      }
      else if(recdat.data[0] == 0x0B)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      }
      else if(recdat.data[0] == 0x0C)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      }
      break;

    case FanSpeedEnterKey:
      thermalManager.fan_speed[0] = recdat.data[0];
      RTS_SndData(thermalManager.fan_speed[0], FAN_SPEED_CONTROL_DATA_VP);
      break;

    case VelocityXaxisEnterKey:
      float velocity_xaxis;
      velocity_xaxis = planner.settings.max_feedrate_mm_s[0];
      velocity_xaxis = recdat.data[0];
      RTS_SndData(velocity_xaxis, MAX_VELOCITY_XAXIS_DATA_VP);
      planner.set_max_feedrate(X_AXIS, velocity_xaxis);
      break;

    case VelocityYaxisEnterKey:
      float velocity_yaxis;
      velocity_yaxis = planner.settings.max_feedrate_mm_s[1];
      velocity_yaxis = recdat.data[0];
      RTS_SndData(velocity_yaxis, MAX_VELOCITY_YAXIS_DATA_VP);
      planner.set_max_feedrate(Y_AXIS, velocity_yaxis);
      break;

    case VelocityZaxisEnterKey:
      float velocity_zaxis;
      velocity_zaxis = planner.settings.max_feedrate_mm_s[2];
      velocity_zaxis = recdat.data[0];
      RTS_SndData(velocity_zaxis, MAX_VELOCITY_ZAXIS_DATA_VP);
      planner.set_max_feedrate(Z_AXIS, velocity_zaxis);
      break;

    case VelocityEaxisEnterKey:
      float velocity_eaxis;
      velocity_eaxis = planner.settings.max_feedrate_mm_s[3];
      velocity_eaxis = recdat.data[0];
      RTS_SndData(velocity_eaxis, MAX_VELOCITY_EAXIS_DATA_VP);
      planner.set_max_feedrate(E_AXIS, velocity_eaxis);
      break;


    case AccelXaxisEnterKey:
      float accel_xaxis;
      accel_xaxis = planner.settings.max_acceleration_mm_per_s2[0];
      accel_xaxis = recdat.data[0];
      RTS_SndData(accel_xaxis, MAX_ACCEL_XAXIS_DATA_VP);
      planner.set_max_acceleration(X_AXIS, accel_xaxis);
      break;

    case AccelYaxisEnterKey:
      float accel_yaxis;
      accel_yaxis = planner.settings.max_acceleration_mm_per_s2[1];
      accel_yaxis = recdat.data[0];
      RTS_SndData(accel_yaxis, MAX_ACCEL_YAXIS_DATA_VP);
      planner.set_max_acceleration(Y_AXIS, accel_yaxis);
      break;

    case AccelZaxisEnterKey:
      float accel_zaxis;
      accel_zaxis = planner.settings.max_acceleration_mm_per_s2[2];
      accel_zaxis = recdat.data[0];
      RTS_SndData(accel_zaxis, MAX_ACCEL_ZAXIS_DATA_VP);
      planner.set_max_acceleration(Z_AXIS, accel_zaxis);
      break;

    case AccelEaxisEnterKey:
      float accel_eaxis;
      accel_eaxis = planner.settings.max_acceleration_mm_per_s2[3];
      accel_eaxis = recdat.data[0];
      RTS_SndData(accel_eaxis, MAX_ACCEL_EAXIS_DATA_VP);
      planner.set_max_acceleration(E_AXIS, accel_eaxis);
      break;

    case JerkXaxisEnterKey:
      float jerk_xaxis;
      jerk_xaxis = planner.max_jerk.x;
      jerk_xaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_xaxis * 100, MAX_JERK_XAXIS_DATA_VP);
      planner.set_max_jerk(X_AXIS, jerk_xaxis);
      break;

    case JerkYaxisEnterKey:
      float jerk_yaxis;
      jerk_yaxis = planner.max_jerk.y;
      jerk_yaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_yaxis * 100, MAX_JERK_YAXIS_DATA_VP);
      planner.set_max_jerk(Y_AXIS, jerk_yaxis);
      break;

    case JerkZaxisEnterKey:
      float jerk_zaxis;
      jerk_zaxis = planner.max_jerk.z;
      jerk_zaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_zaxis * 100, MAX_JERK_ZAXIS_DATA_VP);
      planner.set_max_jerk(Z_AXIS, jerk_zaxis);
      break;

    case JerkEaxisEnterKey:
      float jerk_eaxis;
      jerk_eaxis = planner.max_jerk.e;
      jerk_eaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_eaxis * 100, MAX_JERK_EAXIS_DATA_VP);
      planner.set_max_jerk(E_AXIS, jerk_eaxis);
      break;

    case StepsmmXaxisEnterKey:
      float stepsmm_xaxis;
      stepsmm_xaxis = planner.settings.axis_steps_per_mm[0];
      stepsmm_xaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_xaxis * 10, MAX_STEPSMM_XAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[X_AXIS] = stepsmm_xaxis;
      break;

    case StepsmmYaxisEnterKey:
      float stepsmm_yaxis;
      stepsmm_yaxis = planner.settings.axis_steps_per_mm[1];
      stepsmm_yaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_yaxis * 10, MAX_STEPSMM_YAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Y_AXIS] = stepsmm_yaxis;
      break;

    case StepsmmZaxisEnterKey:
      float stepsmm_zaxis;
      stepsmm_zaxis = planner.settings.axis_steps_per_mm[2];
      stepsmm_zaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_zaxis * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Z_AXIS] = stepsmm_zaxis;
      break;

    case StepsmmEaxisEnterKey:
      float stepsmm_eaxis;
      stepsmm_eaxis = planner.settings.axis_steps_per_mm[3];
      stepsmm_eaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_eaxis * 10, MAX_STEPSMM_EAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[E_AXIS] = stepsmm_eaxis;
      break;

    case NozzlePTempEnterKey:
      float nozzle_ptemp;
      nozzle_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
      PID_PARAM(Kp, e) = nozzle_ptemp;
      break;

    case NozzleITempEnterKey:
      float nozzle_itemp;
      nozzle_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      PID_PARAM(Ki, e) = scalePID_i(nozzle_itemp);
      break;

    case NozzleDTempEnterKey:
      float nozzle_dtemp;
      nozzle_dtemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      PID_PARAM(Kd, e) = scalePID_d(nozzle_dtemp);
      break;

    case HotbedPTempEnterKey:
      float hotbed_ptemp;
      hotbed_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.Kp = hotbed_ptemp;
      break;

    case HotbedITempEnterKey:
      float hotbed_itemp;
      hotbed_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.Ki = scalePID_i(hotbed_itemp);
      break;

    case HotbedDTempEnterKey:
      float hotbed_dtemp;
      hotbed_dtemp = (float)recdat.data[0] / 10;
      RTS_SndData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.Kd = scalePID_d(hotbed_dtemp);
      break;

    case SelectFileKey:
      if (RTS_SD_Detected())
      {
        if (recdat.data[0] > CardRecbuf.Filesum)
        {
          break;
        }

        CardRecbuf.recordcount = recdat.data[0] - 1;

        for(int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        delay(2);
        for(int j = 1;j <= CardRecbuf.Filesum;j ++)
        {
          RTS_SndData((unsigned long)0x073F, FilenameNature + j * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP - 1 + j);
        }
        RTS_SndData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);
        RTS_SndData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      }

      RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      change_page_font = 1;
      delay(20);
      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
      break;

    case StartFileKey:
      if((recdat.data[0] == 1) && RTS_SD_Detected())
      {
        if(CardRecbuf.recordcount < 0)
        {
          break;
        }

        if(CardReader::flag.mounted)
        {
          char cmd[30];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          for (c = &cmd[4]; *c; c++)
            *c = tolower(*c);

          memset(cmdbuf, 0, sizeof(cmdbuf));
          strcpy(cmdbuf, cmd);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              sdcard_pause_check = false;
              break;
            }
          #endif
          PoweroffContinue = true;
          queue.enqueue_one_now(cmd);
          delay(20);
          queue.enqueue_now_P(PSTR("M24"));
          // clean screen.
          for (int j = 0; j < 20; j ++)
          {
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }

          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

          delay(2);

          #if ENABLED(BABYSTEPPING)
            RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
          #endif
          feedrate_percentage = 100;
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          Update_Time_Value = 0;
          // 1:cloud print satrt
          SERIAL_ECHOLN("M79 S1");
        }
        else
        {
          break;
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if((file_total_page > file_current_page) && (file_current_page <= (MaxFileNumber / 4)))
          {
            file_current_page ++;
          }
          else
          {
            break;
          }
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if(CardReader::flag.mounted)
          {
            RTS_line_to_filelist();
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if(file_current_page > 1)
          {
            file_current_page --;
          }
          else
          {
            break;
          }
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if(CardReader::flag.mounted)
          {
            RTS_line_to_filelist();
          }
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = 1;
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = file_total_page;
          RTS_SndData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      }
      break;

    case ChangePageKey:
      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      {
        RTS_line_to_filelist();
        for(uint16_t i = 0;i < 5;i ++)
        {
          delay(1);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          RTS_SndData(0, FILE1_SELECT_ICON_VP + i);
        }
      }

      RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
      RTS_SndData(FIRMWARE_VERSION, FIREWARE_VERSION_ABOUT_TEXT_VP);
      RTS_SndData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
      RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
      RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);

      if(1 == lang)
      {
        RTS_SndData(CORP_WEBSITE_C, WEBSITE_ABOUT_TEXT_VP);
      }
      else
      {
        RTS_SndData(CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
      }

      if(thermalManager.fan_speed[0] == 0)
      {
        RTS_SndData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else
      {
        RTS_SndData(1, PRINTER_FANOPEN_TITLE_VP);
      }

      if(LEDStatus)
      {
        RTS_SndData(1, PRINTER_LEDOPEN_TITLE_VP);
      }
      else
      {
        RTS_SndData(0, PRINTER_LEDOPEN_TITLE_VP);
      }
      Percentrecord = card.percentDone() + 1;
      if (Percentrecord <= 100)
      {
        rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      }
      rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);

      RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      languagedisplayUpdate();

      RTS_SndData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;

    case ErrorKey:
      {
        if(recdat.data[0] == 1)
        {
          if(printingIsActive())
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else if(printingIsPaused())
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
          else
          {
            RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
          }

          if(errorway == 4)
          {
            // reboot
            nvic_sys_reset();
          }
        }
      }
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void EachMomentUpdate(void)
{
  millis_t ms = millis();

  // print finish shot down
  if(flag_counter_printover_to_shutdown)
  {
    millis_t ms_2 = millis();
    if(ms_2 > next_shutdown_update_ms)
    {
      next_shutdown_update_ms = ms_2 + 1000;
      count_ms++;
    }

    if(count_ms > TIME_PRINT_OVER_SHUTDOWN)
    {
      OUT_WRITE(SHUTIDOWN_PIN, 0);
      count_ms = 0;
    }
  }
  else
  {
    count_ms = 0;
  }

  if(flag_counter_wifireset)
  {
    millis_t ms_3 = millis();
    if(ms_3 > next_wifireset_update_ms)
    {
      next_wifireset_update_ms = ms_3 + 1000;
      wifiresetcount_ms++;
      rtscheck.RTS_SndData((TIME_WIFI_RESET_BACKPAGE - wifiresetcount_ms), WIFI_RESET_REMAIN_TIME_DATA_VP);
    }

    if(wifiresetcount_ms > TIME_WIFI_RESET_BACKPAGE)
    {
      flag_counter_wifireset = false;
      rtscheck.RTS_SndData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
      rtscheck.RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
      change_page_font = 33;
    }
  }
  else
  {
    wifiresetcount_ms = 0;
  }

  if(ms > next_rts_update_ms)
  {
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;

        fileCnt = card.get_num_Files();
        card.getWorkDirName();
        if(card.filename[0] != '/')
        {
          card.cdup();
        }

        for(uint16_t i = 0;(i < fileCnt) && (i < MaxFileNumber);i ++)
        {
          card.selectFileByIndex(fileCnt - 1 - i);
          char *pointFilename = card.longFilename;
          int filenamelen = strlen(card.longFilename);
          int j = 1;
          while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen));

          for (int j = 0; j < 20; j ++)
          {
            rtscheck.RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }

          if (j >= TEXTBYTELEN)
          {
            strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
            card.longFilename[TEXTBYTELEN - 1] = '\0';
            j = TEXTBYTELEN - 1;
          }

          strncpy(CardRecbuf.Cardshowfilename[i], card.longFilename, j);

          strcpy(CardRecbuf.Cardfilename[i], card.filename);
          CardRecbuf.addr[i] = PRINT_FILE_TEXT_VP + 20;
          rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
          {
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            break;
          }
        }
        rtscheck.RTS_SndData(ExchangePageBase + 27, ExchangepageAddr);
        change_page_font = 27;
        PoweroffContinue = true;
        SERIAL_ECHOLN("M79 S6");   // 6:cloud print power continue
      }
      return;
    }
    else if((power_off_type_yes == 0) && (recovery.info.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE;
        rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      return;
    }
    else
    {
      // need to optimize
      duration_t elapsed = print_job_timer.duration();
      static unsigned char last_cardpercentValue = 100;
      rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
      rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);

      if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
      {
        if((unsigned char) card.percentDone() > 0)
        {
          Percentrecord = card.percentDone();
          if(Percentrecord <= 100)
          {
            rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
          }
        }
        else
        {
          rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        }
        rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
        last_cardpercentValue = card.percentDone();
        rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
      }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G0 F3000 X0 Y0"));
        thermalManager.setTargetHotend(0, 0);
        rtscheck.RTS_SndData(0, HEAD_SET_TEMP_VP);
      }

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      #if ENABLED(SDSUPPORT)
        if((sd_printing_autopause == true) && (PoweroffContinue == true))
        {
          if(true == sdcard_pause_check)
          {
            rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
            sdcard_pause_check = false;
          }
        }

        if((false == sdcard_pause_check) && (false == card.isPrinting()) && !planner.has_blocks_queued())
        {
          if(CardReader::flag.mounted)
          {
            rtscheck.RTS_SndData(1, CHANGE_SDCARD_ICON_VP);
          }
          else
          {
            rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
          }
        }
      #endif

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        heatway = 0;
        rtscheck.RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        rtscheck.RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }

      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if(1 == READ(FIL_RUNOUT_PIN))
        {
          rtscheck.RTS_SndData(0, FILAMENT_LOAD_ICON_VP);
        }
        else
        {
          rtscheck.RTS_SndData(1, FILAMENT_LOAD_ICON_VP);
        }
      #endif
    }
    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void RTSSHOW::languagedisplayUpdate(void)
{
  RTS_SndData(lang, MAIN_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLUE_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLUE_TITLE_VP);
  RTS_SndData(lang, MAIN_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SELECT_FILE_BLACK_TITLE_VP);
  RTS_SndData(lang, PREPARE_PAGE_BLACK_TITLE_VP);
  RTS_SndData(lang, SETTING_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PRINT_ADJUST_MENT_TITLE_VP);
  RTS_SndData(lang, PRINT_SPEED_TITLE_VP);
  RTS_SndData(lang, HEAD_SET_TITLE_VP);
  RTS_SndData(lang, BED_SET_TITLE_VP);
  RTS_SndData(lang, LEVEL_ZOFFSET_TITLE_VP);
  RTS_SndData(lang, FAN_CONTROL_TITLE_VP);
  RTS_SndData(lang, LED_CONTROL_TITLE_VP);

  RTS_SndData(lang, MOVE_AXIS_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_GREY_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_GREY_TITLE_VP);
  RTS_SndData(lang, MOVE_AXIS_ENTER_BLACK_TITLE_VP);
  RTS_SndData(lang, CHANGE_FILAMENT_BLACK_TITLE_VP);
  RTS_SndData(lang, PREHAET_PAGE_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_BUTTON_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_BUTTON_TITLE_VP);
  RTS_SndData(lang, COOL_DOWN_BUTTON_TITLE_VP);

  RTS_SndData(lang, FILAMENT_LOAD_BUTTON_TITLE_VP);
  RTS_SndData(lang, FILAMENT_UNLOAD_BUTTON_TITLE_VP);

  RTS_SndData(lang, LANGUAGE_SELECT_ENTER_VP);
  RTS_SndData(lang, FACTORY_DEFAULT_ENTER_TITLE_VP);
  RTS_SndData(lang, LEVELING_PAGE_TITLE_VP);

  RTS_SndData(lang, PRINTER_DEVICE_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_GREY_TITLE_VP);
  RTS_SndData(lang, PRINTER_DEVICE_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_ADVINFO_BLACK_TITLE_VP);
  RTS_SndData(lang, PRINTER_INFO_ENTER_BLACK_TITLE_VP);

  RTS_SndData(lang, PREHEAT_PLA_SET_TITLE_VP);
  RTS_SndData(lang, PREHEAT_ABS_SET_TITLE_VP);

  RTS_SndData(lang, STORE_MEMORY_CONFIRM_TITLE_VP);
  RTS_SndData(lang, STORE_MEMORY_CANCEL_TITLE_VP);

  RTS_SndData(lang, FILAMENT_UNLOAD_IGNORE_TITLE_VP);
  RTS_SndData(lang, FILAMENT_USEUP_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CONFIRM_TITLE_VP);
  RTS_SndData(lang, BUTTON_CHECK_CANCEL_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_TITLE_VP);
  RTS_SndData(lang, FILAMENT_LOAD_RESUME_TITLE_VP);
  RTS_SndData(lang, PAUSE_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, STOP_PRINT_POP_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_POP_TITLE_VP);
  RTS_SndData(lang, AUTO_HOME_WAITING_POP_TITLE_VP);

  RTS_SndData(0, BEDLEVELING_WAIT_TITLE_VP);
  RTS_SndData(lang, RESTORE_FACTORY_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_TITLE_VP);
  RTS_SndData(lang, KILL_THERMAL_RUNAWAY_TITLE_VP);
  RTS_SndData(lang, KILL_HEATING_FAIL_TITLE_VP);
  RTS_SndData(lang, KILL_THERMISTOR_ERROR_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, RESET_WIFI_SETTING_BUTTON_VP);
  RTS_SndData(lang, PRINTER_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndData(lang, WIND_AUTO_SHUTDOWN_PAGE_VP);
  RTS_SndData(lang, AUTO_LEVELING_START_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_GREY_TITLE_VP);
  RTS_SndData(lang, AUX_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, AUTO_LEVELING_BLACK_TITLE_VP);
  RTS_SndData(lang, LANGUAGE_SELECT_PAGE_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_MOTION_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_PID_TITLE_VP);
  RTS_SndData(lang, ADV_SETTING_WIFI_TITLE_VP);

  RTS_SndData(lang, MOTION_SETTING_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_STEPSMM_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_ACCEL_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_JERK_TITLE_VP);
  RTS_SndData(lang, MOTION_SETTING_VELOCITY_TITLE_VP);

  RTS_SndData(lang, MAX_VELOCITY_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_VELOCITY_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_ACCEL_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_ACCEL_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_JERK_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_JERK_EAXIS_TITLE_VP);

  RTS_SndData(lang, MAX_STEPSMM_SETTING_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_XAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_YAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_ZAXIS_TITLE_VP);
  RTS_SndData(lang, MAX_STEPSMM_EAXIS_TITLE_VP);

  RTS_SndData(lang, TEMP_PID_SETTING_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_P_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_I_TITLE_VP);
  RTS_SndData(lang, NOZZLE_TEMP_D_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_P_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_I_TITLE_VP);
  RTS_SndData(lang, HOTBED_TEMP_D_TITLE_VP);

  RTS_SndData(lang, FILAMENT_CONTROL_TITLE_VP);
  RTS_SndData(lang, POWERCONTINUE_CONTROL_TITLE_VP);

  RTS_SndData(lang, MACHINE_TYPE_ABOUT_CHAR_VP);
  RTS_SndData(lang, FIREWARE_VERSION_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_DISPLAY_VERSION_TITLE_VP);
  RTS_SndData(lang, HARDWARE_VERSION_ABOUT_TITLE_VP);
  RTS_SndData(lang, WIFI_DN_CODE_CHAR_VP);
  RTS_SndData(lang, WEBSITE_ABOUT_CHAR_VP);
  RTS_SndData(lang, PRINTER_PRINTSIZE_TITLE_VP);
}

// looping at the loop function
void RTSUpdate(void)
{
  // Check the status of card
  rtscheck.RTS_SDCardUpate();

  EachMomentUpdate();

  // wait to receive massage and response
  if(rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
}

void RTS_PauseMoveAxisPage(void)
{
  if(waitway == 1)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    waitway = 0;
    SERIAL_ECHOLN("M79 S2");   // 2:cloud print pause
  }
  else if(waitway == 5)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
    change_page_font = 7;
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage(void)
{
  if(waitway == 3)
  {
    rtscheck.RTS_SndData(0, BEDLEVELING_WAIT_TITLE_VP);
    rtscheck.RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
    change_page_font = 26;
    waitway = 0;
  }
}

void RTS_MoveAxisHoming(void)
{
  if(waitway == 4)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 16 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 16;
    waitway = 0;
  }
  else if(waitway == 6)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
    change_page_font = 25;
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click Print finish
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    waitway = 0;
  }

  rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
  rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
  rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
}

void RTS_CommandPause(void)
{
  if(printingIsActive())
  {
    rtscheck.RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
    change_page_font = 10;
    // card.pauseSDPrint();
    // print_job_timer.pause();
    // pause_action_flag = true;
  }
}

void ErrorHanding(void)
{
  // No more operations
  if(errorway == 1)
  {
    errorway = errornum = 0;
  }
  else if(errorway == 2)
  {
    // Z axis home failed
    home_errornum ++;
    if(home_errornum <= 3)
    {
      errorway = 0;
      waitway  = 4;
      queue.enqueue_now_P(PSTR("G28"));
      rtscheck.RTS_SndData(0, MOTOR_FREE_ICON_VP);
      Update_Time_Value = 0;
    }
    else
    {
      // After three failed returns to home, it will report the failure interface
      home_errornum = 0;
      errorway = 0;
      rtscheck.RTS_SndData(ExchangePageBase + 41, ExchangepageAddr);
      change_page_font = 41;
      // Z axis home failed
      rtscheck.RTS_SndData(Error_202, ABNORMAL_PAGE_TEXT_VP);
      
      if(printingIsActive())
      {
        rtscheck.RTS_SndData(0, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData(0, PRINT_TIME_MIN_VP);
        Update_Time_Value = 0;

        rtscheck.RTS_SDcard_Stop();
      }
    }
  }
  else if(errorway == 3)
  {
    // No more operations
    reset_bed_level();
    errorway = 0;
    errornum = 0;
  }
  else if(errorway == 4)
  {

  }
}

#endif
