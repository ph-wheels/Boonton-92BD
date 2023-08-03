#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <Preferences.h>
#include <SerialCommand.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "WiFi.h" 
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_timer.h>
#include "esp_core_dump.h"
#include <esp_task_wdt.h>

/*
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * article on adc: 
 * https://github.com/espressif/esp-idf/blob/release/v4.4/examples/peripherals/adc/single_read/single_read/main/single_read.c#L68
 * 
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Version history:
 * 
 * Ver 1.2b1    initial version for esp32, issue            
 *        b2    issue resolved was usage op pin 12 -> rerouted to pin 34
 *              'CMD' jumper switched with 'C_E' to allow easier access 
 *        b3    removed C_E option, routed the adc value to chan 7 adc1 @ P35
 *              re-arranged main loop as to avoid any blocking operation
 *              by removing delay functions
 *        b4    added selectable averaged adc value for range compare
 *        b5    general cleanup investigate odd behaviour at -20dBm
 *        b6    swap io pin 2 & 15 as to correct function to jmp
 *        b7    swap io pin 16 & 17 as to have proper phasing with v2.1 brd
 *     1.3b1    changed sampling delays to be timer & interrupt driven
 *     1.4b1    replaced serial command function with 'SerialCommand' as
 *              to allow it to properly run on esp32 'Copyright (C) 2012 Stefan Rado'
 *              current version should run on both V2 and V3 pcb (future version)
 *        b2    added option to use auto or manual timing for SSR values and 
 *              select between either option
 *        b3    removed option to (attempt) controle of pulse duration
 *     1.5b1    changed setup & usage of delays as to avoid blocking
 *        b2    added WDT functions
 *        b3    added counts to mV
 *        b4    correction in adc read funcion usage
 *     1.6b1    improved ratio value for adc counts => mV
 */

Preferences PREF;
SerialCommand sCmd; // This seems to be place critical !!
static esp_adc_cal_characteristics_t adc1_chars;

char version[] = "1.6b1"; // added menu as to set parameters

#define WDT_TIMEOUT 1     // define a 1 seconds WDT (Watch Dog Timer)
#define speed 115200
#define adjust 45
// SSR ratio for pre and pulse time
#define pls 0.94
#define del 0.03
//#define NEW_PCB true    // enable definition to obtain GPIO pins for V2.7 PCB
//#define timing true

const int ANA_PIN = 35;   // analog input pin
const int DIG_AUT = 5;    // digital input for auto ranging
const int DIG_CMD = 2;    // enable menu / test cmd's
#if NEW_PCB
  const int DIG_M42 = 4;   // enable usage for 4200D unit
  const int DIG_P[] = {22, 32, 33, 25, 26, 27, 23, 21};   // output pin array for range 0 - 7 (V2.7 PCB)
#else
  const int DIG_M42 = 15;    // enable usage for 4200D unit
  const int DIG_P[] = {22, 32, 33, 25, 26, 27, 14, 21};   // output pin array for range 0 - 7 (V2.2 PCB)
#endif

char helper_arr[8][5] = {
      "1", "3", "10","30", "100", "300","1000", "3000"};

const int range_min = 0;  // lowest range value
const int range_max = 7;  // highest range value
int range_dir = 0;        // ranging up (1) of down (-1)
int range_val = 0;        // selected range value
bool auto_mode = false;   // mode uato or manual
bool scan_once = true;    // inhibit io updates when no range change needed
bool init_once = true;

int AVG_S[] = {0,1,2,4,8,16};
static int adc_raw[2][10];

uint64_t time_ms = 0;

hw_timer_t * timer_adc = NULL;
hw_timer_t * timer_cmd = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
  int16_t lo_lvl;
  int16_t hi_lvl;
  int16_t lo_dur;
  int16_t an_mde;
  int16_t an_del;
  int16_t an_scn;
  int16_t an_avg;
  float   an_rat;
} Sett_t;

const byte Led_Pin_0 = 19;
const byte Led_Pin_1 = 18;
const byte Int_Pin_0 = 17;
const byte Int_Pin_1 = 16;
const byte Tst_Pin_0 = 12;
volatile byte state_0 = HIGH;
volatile byte state_1 = HIGH;

volatile int adc_isr_cnt;
volatile int cmd_isr_cnt;
volatile int cmd_isr_max;
volatile int r_adc;
volatile int adc_tmp;
volatile byte out_rdy = LOW;
volatile int smp_max;
volatile long smp_sum;
volatile int16_t adc_avg;

volatile bool calibrated = false;
volatile unsigned long t_start = 0;
volatile unsigned long t_delta = 0;
volatile float time_freq = 0;
volatile byte cnt = 0;
uint16_t pre_time = 0;
int DIG_P_Idx = 0;

int16_t lo_lvl = 200;     // lo trigger level to select lower range
int16_t hi_lvl = 1800;    // hi trigger level to select higher range
int16_t lo_dur = 100;     // pre pulse time in uSec
int16_t an_mde = 1;       // automatic or customer mode timing
int16_t an_del = 100;     // delay after channel select and new adc sample
int16_t an_scn = 10;      // adc sample interval 
int16_t an_avg = 8;       // # of analog sample to produce average value
float   an_rat = 6.66;    //

volatile bool ph_0_busy = false;
volatile bool ph_1_busy = false;

Sett_t settingBase;

void makeSettingBase() {
  settingBase.lo_lvl = 200;
  settingBase.hi_lvl = 1800;
  settingBase.lo_dur = 100;
  settingBase.an_mde = 1;
  settingBase.an_del = 200;
  settingBase.an_scn = 10;
  settingBase.an_avg = 8;
  settingBase.an_rat = 6.66;
}

static bool adc_calibration_init(void);

bool saveSettingBase() {
  //tmrControl(false);
  PREF.putBytes("Setting", (uint8_t*)&settingBase, sizeof(settingBase));
  #if nvr_debug
    for(int i = 0; i < sizeof(Sett_t); i++)
      printf("%#x ",(unsigned int) ((char*)&settingBase)[i]);
    printf("\r\n");
  #endif
  //tmrControl(true);
  return true;
}

bool loadSettingBase() {
  
  size_t length = PREF.getBytesLength("Setting");
  
  if (length != sizeof(settingBase)) {
    makeSettingBase();
    saveSettingBase();
    printf("save Setting base\n");
  } else {
    PREF.getBytes("Setting", (uint8_t*)&settingBase, length);
  }
  #if nvr_debug
    for(int i = 0; i < sizeof(Setting_t); i++)
      printf("%#x ",(unsigned int) ((char*)&settingBase)[i]);
    printf("\r\n");
  #endif
  return true;  
}

void initSettingBase() {

  lo_lvl = settingBase.lo_lvl;
  hi_lvl = settingBase.hi_lvl;
  lo_dur = settingBase.lo_dur;
  an_mde = settingBase.an_mde;
  an_del = settingBase.an_del;
  an_scn = settingBase.an_scn;
  an_avg = settingBase.an_avg;
  an_rat = settingBase.an_rat;
}

/*
 * actual i/o control to de-select previous then select new range
 */
void set_range (int val, int dir) {
  // intend is to release current range first then set new
  if (dir == -1) {
    // scan from high to low
    for(int i = 8; i-- > 0; ) {
      if (val == i) {
        digitalWrite (DIG_P[i], LOW);
      } else {
        digitalWrite (DIG_P[i], HIGH);
      }
    }
  }
  if (dir == 1) { 
    // scan from low to high
    for (int i = 0; i < 8; i++) {
      if (val == i) {
        digitalWrite (DIG_P[i], LOW);
      } else {
        digitalWrite (DIG_P[i], HIGH);
      }
    }
  }
  // addition => model 4200 support, which activates both outputs
  if (digitalRead (DIG_M42) == 0) {
    if ((val == 6) || (val == 7)) {
      digitalWrite (DIG_P[6], LOW);
      digitalWrite (DIG_P[7], LOW);
    }
  }
}

/*
 * clear all outputs
 */
void clr_range () {
  for (int i = 0; i < 8; i++) {
    digitalWrite (DIG_P[i], HIGH);
  }
}

/*
 * menu routine to diplay or get values for lo / hi thresholds
 */
void set_lh() {
  /* used for adc low and high value */
  int aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 100) && (aNumber <= 200)) {
      printf("ADC low  count %d will applied\n", aNumber);
      settingBase.lo_lvl = aNumber;
    }
  }
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    // 3.21 = battery correction factor
    if ((aNumber >= 500) && (aNumber <= 2000)) {
      printf("ADC high count %d will applied\n\n", aNumber);
      settingBase.hi_lvl = aNumber;
    }
  }
}

void dur_lh() {
  /* used for adc low and high value */
  int aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 50) && (aNumber < 250)) {
      printf("Chopper delay time, %d uSec will applied\n", aNumber);
      settingBase.lo_dur = aNumber;
    }
  }
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 0) && (aNumber <= 1)) {
      settingBase.an_mde = aNumber;
    } else {
      printf ("valid selections are 0 or 1.\n");
    }
  } else {
    printf("One argument expected\n");
  }
}

/*
 * menu routine to diplay or get values for delay times
 */
void set_dt() {
  int aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 25) && (aNumber < 200)) {
      //printf("ADC low  count %d will applied\n", aNumber);
      settingBase.an_scn = aNumber;
    }
  }
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    // 3.21 = battery correction factor
    if ((aNumber >= 5) && (aNumber < 20)) {
      //printf("ADC high count %d will applied\n\n", aNumber);
      settingBase.an_del = aNumber;
    }
  }
}

/*
 * menu routine to diplay and set values n count avage samples
 */
void avg_lc() {
  int aNumber;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber % 2 == 0) && ((aNumber >= 2) && (aNumber < 16))) {
      //printf("ADC low  count %d will applied\n", aNumber);
      settingBase.an_avg = aNumber;
    } else {
      printf ("valid selections are 2, 4, 8 or 16 samples.\n");
    }
  } else {
    printf("One argument expected\n");
  }
}

/*
 * menu routine to diplay or get/set eeprom values
 */
void set_ap() {
  int aNumber1, aNumber2;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber1 = atoi(arg);
    if (aNumber1 >= 2) {
      PREF.clear();               /* reset to factory */
      printf ("Flash cleared and loaded with defaults\n");
      delay(500);
      ESP.restart();
    } else if (aNumber1 >= 1) {
      saveSettingBase();
      printf ("Params saved to flash\n");
    } else {
      //loadSettingBase();
      printf ("\nParams loaded from flash\n");
      printf ("    lo_lvl %d, hi_lvl %d mV\n", settingBase.lo_lvl, settingBase.hi_lvl);
      printf ("    Pre-delay %d, active mode %s.\n", settingBase.lo_dur, settingBase.an_mde ? "Auto" : "Manual");
      printf ("    Scan   %d, Delay  %d mS\n", settingBase.an_scn, settingBase.an_del);
      printf ("    Sample %d count\n", settingBase.an_avg);
      printf ("    Ratio for adc count to mV = %6.2f\n", settingBase.an_rat);
    }
  }
}

/*
 * menu routine to diplay adc values n times as to obtain calibration values
 */
void chk_ad() {
  int aNumber;
  char *arg;
  int adc_val;
  int tmp1 = 0;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 1) && (aNumber <= 100)) {
      smp_sum = 0;
      smp_max = aNumber;
      for (int j = 0; j < smp_max; j++ ) {
        delay (100);

        if (adc_calibration_init()) {
          adc_raw[0][0] = adc1_get_raw(ADC1_CHANNEL_7);
          adc_val = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
        } else {
          adc_val = adc1_get_raw(ADC1_CHANNEL_7);
        }

        printf ("adc value: %d counts\n", adc_val);
        smp_sum = smp_sum + adc_val;
      } 
      tmp1 = smp_sum/smp_max;
      printf ("adc average: %d counts, %6.2f mV\n", tmp1 , tmp1 * settingBase.an_rat);
    }
  }
}



void cal_ad() {
  int aNumber;
  char *arg;
  float ratio;
  int adc_val;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);
    if ((aNumber >= 4000) && (aNumber <= 6000)) {
      if (adc_calibration_init()) {
        adc_raw[0][0] = adc1_get_raw(ADC1_CHANNEL_7);
        adc_val = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
        ratio = (float) aNumber / (float)adc_val;
      } else {
        adc_val = adc1_get_raw(ADC1_CHANNEL_7);
        ratio = (float) aNumber / (float)adc_val;
      }
      settingBase.an_rat = ratio;
      printf ("adc value: %d, mid scale value %d, ratio (cnt => mV) %6.2f\n", adc_val, aNumber , ratio);
    }
  }
}

/*
 * menu routine to turn on selected range for n sec as to test it's operation
 */
void out_ch() {
  int aNumber1, aNumber2;
  char *arg;

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber1 = atoi(arg);
    if ((aNumber1 >= 0) && (aNumber1 <= 7)) {
      DIG_P_Idx = aNumber1;
      arg = sCmd.next();
      if (arg != NULL) {
        aNumber2 = atoi(arg);
        if ((aNumber2 >= 2) && (aNumber2 <= 10)) {
          printf ("With right input level a value should display\n");
          digitalWrite (DIG_P[aNumber1], LOW);
          cmd_isr_cnt = 0;
          cmd_isr_max = aNumber2 * 1000000 / an_scn * 0.8; // 0.8 = timing correction factor
          out_rdy = HIGH; // once enabled & time elapsed the main scan will show result
        } else {
          printf ("Duration for range not supported (2 - 10 sec)\n");
        }
      }
    } else {
      printf ("Measure range not supported (0 - 7)\n");
    }
  }
}


bool  loadSettingBase();
void  initSettingBase();

void IRAM_ATTR isr_0() {
  portENTER_CRITICAL_ISR(&timerMux);
  state_0 = LOW;
  digitalWrite(Led_Pin_1, HIGH);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR isr_1() {
  portENTER_CRITICAL_ISR(&timerMux);
  state_1 = LOW;
  digitalWrite(Led_Pin_0, HIGH);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  adc_isr_cnt++;
  cmd_isr_cnt++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

static bool adc_calibration_init(void) {
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    }

    return cali_enable;
}

/*
 * menu items display
 */
void unrecognized(const char *command) {
  /* provides overview of available commands */
  printf ("\nCommand's are:\n");
  printf ("\thlr x y where x = low, y = high adc count\n");
  printf ("\t    x = 200, y = 1800 as default\n");
  printf ("\tdur x y where x = pre-delay, y = auto (1) vs manual (0) pre-delay\n");
  printf ("\t    x = 100 uSec, y = 1 as default\n");
  printf ("\tdly x y where x = range, y = sample delays\n");
  printf ("\t    x = 100, y = 10 as default\n");
  printf ("\tavg x   where x = adc average sample count\n");
  printf ("\tadc x   adc test, where x = # number of samples\n");
  printf ("\tout x y where x = chan, y = # sec to test range 1- 3000mV.\n");
  printf ("\t        Note: No button should be depressed now!\n");
  printf ("\tcal x   where x = measured value @ rear terminals in mV driven by\n");
  printf ("\t        input signal on range 3 or 10 mV to obtain 5000mV output\n");
  printf ("\tpar x   Saves or retrieves Flash data or shows its data\n");
  printf ("\t        where x 0 = display actual, 1 = save, 2 = erase/set to default\n");
  printf("Valid (lowercase only) commands & parameters are:\n");
}

void dis_BT_Wifi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  esp_wifi_stop();
  esp_bt_controller_disable();
}

void setup() {
  /* init debug serial port */
  Serial.flush();
  Serial.begin(speed);

  //BootReason = esp_reset_reason();
  //  Serial.print("Reset/Boot Reason was: "); Serial.println( BootReason );
    
    esp_reset_reason_t reason = esp_reset_reason();
 
    switch (reason) {
        case ESP_RST_UNKNOWN:
          Serial.println("Reset reason can not be determined");
        break;

        case ESP_RST_POWERON:
          Serial.println("Reset due to power-on event");
        break;

        case ESP_RST_EXT:
          Serial.println("Reset by external pin (not applicable for ESP32)");
        break;

        case ESP_RST_SW:
          Serial.println("Software reset via esp_restart");
        break;

        case ESP_RST_PANIC:
          Serial.println("Software reset due to exception/panic");
        break;

        case ESP_RST_INT_WDT:
          Serial.println("Reset (software or hardware) due to interrupt watchdog");
        break;

        case ESP_RST_TASK_WDT:
          Serial.println("Reset due to task watchdog");
        break;

        case ESP_RST_WDT:
          Serial.println("Reset due to other watchdogs");
        break;                                

        case ESP_RST_DEEPSLEEP:
          Serial.println("Reset after exiting deep sleep mode");
        break;

        case ESP_RST_BROWNOUT:
          Serial.println("Brownout reset (software or hardware)");
        break;
        
        case ESP_RST_SDIO:
          Serial.println("Reset over SDIO");
        break;
        
        default:
        break;
    }

  /* io pins for ssr triggered by isr routines */
  pinMode(Led_Pin_0, OUTPUT);
  pinMode(Led_Pin_1, OUTPUT);

  #ifdef timing
    pinMode(Tst_Pin_0, OUTPUT);
  #endif

  // define input for chopper control lines
  pinMode(Int_Pin_0, INPUT_PULLUP);
  pinMode(Int_Pin_1, INPUT_PULLUP);
  // define input for auto range switch
  pinMode (DIG_AUT, INPUT_PULLUP);
  // define input for command mode
  pinMode (DIG_CMD, INPUT_PULLUP);
  // Boonton 42 device mode
  pinMode (DIG_M42, INPUT_PULLUP);

  // make sure WiFI & BT are off
  dis_BT_Wifi();

  attachInterrupt(digitalPinToInterrupt(Int_Pin_0), isr_0, FALLING);
  attachInterrupt(digitalPinToInterrupt(Int_Pin_1), isr_1, FALLING);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  pre_time = 100;
 
  // get stored value
  PREF.begin("RF92BD", false);

  loadSettingBase();
  initSettingBase();

  auto_mode = false;

  // set io to output mode and to high state
  for (int i = 0; i < 8; i++) {
    pinMode (DIG_P[i], OUTPUT);
    digitalWrite (DIG_P[i], HIGH);
  }

  // select starting range #4 = 100mV
  range_val = 4;

  if (digitalRead (DIG_CMD) == 0) {
    printf ("\nMCU Started. (v%s)\n", version);
    printf ("Service menu active, type 'x' for details\n");

    // command list

    sCmd.addCommand("hlr", set_lh);    // get/set lo_lvl & hi_lvl range params values
    sCmd.addCommand("dur", dur_lh);    // set manual SSR timing lo_dur & hi_dur value's
    sCmd.addCommand("dly", set_dt);    // get/set select & scan delay params
    sCmd.addCommand("avg", avg_lc);    // get/set average sample count
    sCmd.addCommand("par", set_ap);    // get/set flash data from params
    sCmd.addCommand("adc", chk_ad);    // get adc sample value(s) and average
    sCmd.addCommand("cal", cal_ad);    // get output voltage at rear terminals
    sCmd.addCommand("out", out_ch);    // set range channel active for xx sec
    sCmd.setDefaultHandler(unrecognized);

  } //else {
    // will be periodically interrupt for adc sampling
    timer_adc = timerBegin(0, 80, true);
    timerAttachInterrupt(timer_adc, &onTimer, true);
    timerAlarmWrite(timer_adc, an_scn, true); /* <= make this an par variable */
    timerAlarmEnable(timer_adc);
 // }
  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);

}

void loop() {

  static uint64_t lastread = time_ms;
  time_ms = esp_timer_get_time()/1000;
  static int32_t total = 0;
  static int idx = 0;
  static int16_t samples[16];
  int adc_val;
  //static int16_t adc_avg;

  /* upon startup check chopper freq to be around 94Hz ~ 10.5 mS */
  if (!calibrated) {
    if (state_0 == LOW)  {
      if (cnt == 0) {
        printf("\nCalibration for timing value's started\n");
        t_start = millis();
      }

      if (cnt > 99) {
        // calculated timing value's
        t_delta = millis() - t_start;
        time_freq = (float) t_delta/100;
        printf("Measured and derived values based on your chopper driver frequency\n");
        printf("t_delta %lu, period %.3f mS, pulse %.3f mS, pre-delay %.3f mS\n", t_delta, time_freq, time_freq/2 * pls, time_freq/2 * del);
        // convert mSec to uSec
        pre_time = (settingBase.an_mde) ? (uint16_t) (time_freq/2 * del * 1000) - adjust : settingBase.lo_lvl;
        if (settingBase.an_mde) {
          printf("Advised timing value's for pre-delay %d uS in use\n", pre_time + adjust);
        } else {
          printf("Manual timing value's for pre-delay %d uS in use\n", settingBase.lo_dur + adjust);
        }
        calibrated = true;
      }
      portENTER_CRITICAL(&timerMux);
      state_0 = HIGH;
      state_1 = HIGH;
      portEXIT_CRITICAL(&timerMux);
      cnt++;
    }
  }
  if (calibrated) {
    /* as we should have proper timing value's to startup chopper values */
    if (state_0 == LOW) {
      portENTER_CRITICAL(&timerMux);
      state_0 = HIGH;
      portEXIT_CRITICAL(&timerMux);
      if (state_1 == LOW) printf("Ph_Err_0\n");
      if (!ph_0_busy) {
        ph_1_busy = false;
        #ifdef timing
          digitalWrite(Tst_Pin_0, LOW);
          digitalWrite(Tst_Pin_0, HIGH);
        #endif
        digitalWrite(Led_Pin_0, HIGH);
        delayMicroseconds((settingBase.an_mde) ? pre_time : settingBase.lo_dur);
        digitalWrite(Led_Pin_0, LOW);
        digitalWrite(Led_Pin_1, HIGH);
        ph_0_busy = true;
      }
    } 

    if (state_1 == LOW) {
      portENTER_CRITICAL(&timerMux);
      state_1 = HIGH;
      portEXIT_CRITICAL(&timerMux);
      if (state_0 == LOW) printf("Ph_Err_1\n");
      if (!ph_1_busy) {
        ph_0_busy = false;
        #ifdef timing
          digitalWrite(Tst_Pin_0, LOW);
          digitalWrite(Tst_Pin_0, HIGH);
        #endif
        digitalWrite(Led_Pin_1, HIGH);
        delayMicroseconds((settingBase.an_mde) ? pre_time : settingBase.lo_dur);
        digitalWrite(Led_Pin_1, LOW);
        digitalWrite(Led_Pin_0, HIGH);
        ph_1_busy = true;
      }
    }

    if ((adc_isr_cnt > 0)){
      portENTER_CRITICAL(&timerMux);
      adc_isr_cnt--;
      portEXIT_CRITICAL(&timerMux);
      // get new adc sample
      r_adc = adc1_get_raw(ADC1_CHANNEL_7); /* get adc sample */
      // usage of the averaged adc signal over x samples
      total -= samples[idx] ; 
      total += r_adc; 
      samples[idx] = r_adc;  
      idx = (idx + 1) % an_avg;  
      adc_avg = total / an_avg; //>> (an_avg == 16) ? 4 : (an_avg == 8) ? 3 : (an_avg == 4) ? 2 : 1;
      //adc_avg = r_adc;
      if(range_dir != 0) {
        if (auto_mode) {
          set_range (range_val, range_dir);
          scan_once = true;
        }
        range_dir =  0;
      }
    }
  }

  esp_task_wdt_reset();            // Added to repeatedly reset the Watch Dog Timer

  /* input auto range switch, auto mode ? */
  if (digitalRead (DIG_AUT) == 0){
    //digitalWrite(DIG_P[range_val], LOW); /* start auto range mode @ level 4, 100mV */
    auto_mode = true;
  } else {
    // deselect auto range mode
    if (scan_once) { 
      /* cleanup io output, set to high */
      clr_range ();
      scan_once = false;
    }
    // auto mode => off
    auto_mode = false;
  }
  /* only allow updates every x mS */
  if(time_ms >= (lastread + an_del)) {
    lastread = time_ms;
    // avoid multiple step if previous step has not completed
    if ((auto_mode) && (range_dir == 0)) {
      // lower than threshold and above lowest range ?
      if ((adc_avg <= lo_lvl) && (range_val > range_min)) {
        // select lower range
        range_val--;
        range_dir =  -1;
      }
      // higher than threshold and below highest range ?
      if ((adc_avg >= hi_lvl) && (range_val < range_max)) {
        // select higher range
        range_val++;
        range_dir =  1;
      }
    }
  }

  /* Process pending serial commands */
  if (digitalRead (DIG_CMD) == 0) {
    sCmd.readSerial();
  }

  if ((out_rdy == HIGH) && (cmd_isr_cnt >= cmd_isr_max)) {

    if (adc_calibration_init()) {
      adc_raw[0][0] = adc1_get_raw(ADC1_CHANNEL_7);
      adc_val = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
    } else {
      adc_val = adc1_get_raw(ADC1_CHANNEL_7);
    }
    printf ("range %s mV, adc value: %d counts, %6.2f mV\n", helper_arr[DIG_P_Idx], adc_val , adc_val * settingBase.an_rat);
    digitalWrite (DIG_P[DIG_P_Idx], HIGH);
    out_rdy = LOW;
  }
}
