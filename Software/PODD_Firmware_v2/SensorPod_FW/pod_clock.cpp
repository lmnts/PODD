/*==============================================================================
  Various time/clock-related constants and functions.

  This file is part of the LMN PODD distribution:
    https://github.com/lmnts/PODD

  CONTRIBUTORS:
    Chris Savage (2019)

  COPYRIGHT/LICENSE:
  Copyright (c) 2019 LMN Architects

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Affero General Public License for more details.

  You should have received a copy of the GNU Affero General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

==============================================================================*/

#include "pod_clock.h"
#include "pod_eeprom.h"

#include <ctype.h>
#include <Time.h>
#include <Timezone.h>
#include <SparkFunDS3234RTC.h>
#include <EEPROM.h>
#ifdef CLOCK_TESTING
#include "pod_serial.h"
#endif



// Constants/global variables ==================================================

// SPI chip-select pin for RTC
#define RTC_PIN_CS 17
// Minimum valid unix time (~ 2001-09-09)
#define UTC_CUTOFF 1000000000ul

// Day of year for first day of each month (0-indexed)
const uint16_t MONTH_DOY[] = {0,31,59,90,120,151,181,212,243,273,304,334};
// ...for leap year
const uint16_t MONTH_DOLY[] = {0,31,60,91,121,152,182,213,244,274,305,335};

// DS3234 RTC register addresses
#define DS3234_TIME_ADDR 0x00
#define DS3234_TIME_LEN 7
#define DS3234_TEMP_ADDR 0x11
#define DS3234_TEMP_LEN 2

// SPI settings for RTC communication
SPISettings rtcSPISettings(4000000, MSBFIRST, SPI_MODE3);

// Clock configuration.
// Contains a version number used for EEPROM storage checking
// and two labels representing the timezones without and with
// daylight saving time, respectively.
#define CLOCK_CONFIG_VERSION 10000
struct ClockConfig {
  uint16_t version;
  char label1[6],label2[6];
};
ClockConfig clockConfig {CLOCK_CONFIG_VERSION,"",""};

// Timezone settings
//TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};
//TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420};
//TimeChangeRule usMST = {"MST", First, Sun, Nov, 2, -420};
//TimeChangeRule usMDT = {"MDT", Second, Sun, Mar, 2, -360};
//TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360};
//TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300};
//TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};
//TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};
//Timezone usPacific(usPST,usPDT);
//Timezone usMountain(usMST,usMDT);
//Timezone usCentral(usCST,usCDT);
//Timezone usEastern(usEST,usEDT);
Timezone timezone((TimeChangeRule){"PST",First,Sun,Nov,2,-480},
                  (TimeChangeRule){"PDT",Second,Sun,Mar,2,-420});



// General Time Functions ======================================================

//------------------------------------------------------------------------------
/* Initializes the RTC. */
void initRTC() {
  // Initialize communication with DS3234 RTC
  initDS3234();
  // Load local timezone from non-volatile memory if available
  // or set to default (Pacific).
  initTimezone();
}


//------------------------------------------------------------------------------
/* Test connection with the RTC. */
bool probeRTC() {
  return probeDS3234();
}


//------------------------------------------------------------------------------
/* Set the current time using unix time: number of seconds since 1970-01-01
   at 00:00:00 UTC.  Backed by RTC.  As a safety measure, time will not be
   set if t < 1000000000 (~ 2001-09-09). */
void setUTC(time_t t) {
  setDS3234Time(t);
}


//------------------------------------------------------------------------------
/* Get the current time as unix time: number of seconds since 1970-01-01
   at 00:00:00 UTC.  Backed by RTC.  Returns 0 if failed to extract time
   from RTC. */
time_t getUTC() {
  return getDS3234Time();
}



// Timezone Functions ==========================================================

//------------------------------------------------------------------------------
/* Loads timezone information from EEPROM if available, otherwise sets to
   Pacific timezone with DST. */
void initTimezone() {
  /*
  // NOTE: No built-in ability to see if timezone was already written
  // to EEPROM.  We instead check if first timezone rule label is 3-5
  // alphabetical characters.  Timezone class does not give access to
  // timezone rules, so we check memory locations directly: THIS CHECK
  // WILL BREAK IF TIMEZONE LIBRARY STORAGE FORMAT CHANGES!!!
  char label[6];
  uint8_t label_len;
  bool valid;
  for (int k=0; k < 6; k++) label[k] = EEPROM.read(EEPROM_CLOCK_ADDR + k);
  label_len = 0;
  valid = true;
  for (int k=0; k < 6; k++) {
    if (label[k] == 0) break;
    label_len++;
    if (!isalpha(label[k])) {
      valid = false;
      break;
    }
  }
  if (label_len > 5) valid = false;
  
  // Looks like timezone saved to EEPROM
  if (valid) {
    timezone.readRules(EEPROM_CLOCK_ADDR);
  } else {
    // Already initialized to Pacific time
    //TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};
    //TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420};
    //timezone.setRules(usPST, usPDT);
  }
  */

  // Load clock config from EEPROM
  for (size_t k = 0; k < sizeof(clockConfig); k++) {
      *((char*)&clockConfig + k) = EEPROM.read(EEPROM_CLOCK_ADDR + k);
  }

  // Check if loaded data is valid
  if (clockConfig.version == CLOCK_CONFIG_VERSION) {
    setTimezone(clockConfig.label1,clockConfig.label2);
  // Default to Pacific time (will be saved to EEPROM)
  } else {
    setTimezone("PST","PDT");
  }
}


//------------------------------------------------------------------------------
/* Helper function for below setTimezone function that returns a timezone rule
   for the given label. */
TimeChangeRule timezoneRuleByLabel(String label) {
  if (label.equals("EST")) {
    return (TimeChangeRule){"EST", First, Sun, Nov, 2, -300};
  } else if (label.equals("EDT")) {
    return (TimeChangeRule){"EDT", Second, Sun, Mar, 2, -240};
  } else if (label.equals("CST")) {
    return (TimeChangeRule){"CST", First, Sun, Nov, 2, -360};
  } else if (label.equals("CDT")) {
    return (TimeChangeRule){"CDT", Second, Sun, Mar, 2, -300};
  } else if (label.equals("MST")) {
    return (TimeChangeRule){"MST", First, Sun, Nov, 2, -420};
  } else if (label.equals("MDT")) {
    return (TimeChangeRule){"MDT", Second, Sun, Mar, 2, -360};
  } else if (label.equals("PST")) {
    return (TimeChangeRule){"PST", First, Sun, Nov, 2, -480};
  } else if (label.equals("PDT")) {
    return (TimeChangeRule){"PDT", Second, Sun, Mar, 2, -420};
  } else if (label.equals("UTC")) {
    return (TimeChangeRule){"UTC", Last, Sun, Mar, 1, 0};
  }
  // Unknown label: use UTC
  return (TimeChangeRule){"UTC", Last, Sun, Mar, 1, 0};
}


//------------------------------------------------------------------------------
/* Set the timezone by the regular & daylight saving labels (e.g. "PST"
   and "PDT").  If a second label is not provided, the first timezone is
   used yearwide and daylight savings is not applied.
   Currently implemented labels: EST, EDT, CST, CDT, MST, MDT, PST, PDT,
   UTC.  Unrecognized labels will be set to UTC.  */
void setTimezone(String tz, String dtz) {
  if (dtz.equals("")) {
    TimeChangeRule rule1 = timezoneRuleByLabel(tz);
    timezone.setRules(rule1,rule1);
  } else {
    TimeChangeRule rule1 = timezoneRuleByLabel(tz);
    TimeChangeRule rule2 = timezoneRuleByLabel(dtz);
    timezone.setRules(rule1,rule2);
  }

  // Update clock config structure and save to EEPROM
  clockConfig.version = CLOCK_CONFIG_VERSION;
  //clockConfig.label1 = tz;
  //clockConfig.label2 = (dtz.equals("")) ? tz : dtz;
  tz.toCharArray(clockConfig.label1,6);
  if (dtz.equals("")) {
    tz.toCharArray(clockConfig.label2,6);
  } else {
    dtz.toCharArray(clockConfig.label2,6);
  }
  for (size_t k = 0; k < sizeof(clockConfig); k++) {
    // Put only writes byte if different from current EEPROM value
    // (reduces EEPROM wear).  If configuration hasn't changed,
    // this will not actually write anything.
    EEPROM.put(EEPROM_CLOCK_ADDR + k, *((char*)&clockConfig + k));
  }
}


//------------------------------------------------------------------------------
/* Gets the local timezone label for the given UTC unix time; the
   current time is used if the argument is zero. */
String getTimezoneLabel(time_t t) {
  if (t == 0) t = getUTC();
  TimeChangeRule *rule;
  timezone.toLocal(t,&rule);
  return rule -> abbrev;
}


//------------------------------------------------------------------------------
/* Gets the standard and daylight saving timezone labels.  */
String getStandardTimezoneLabel() {return clockConfig.label1;}
String getDaylightSavingTimezoneLabel() {return clockConfig.label2;}
//String getTimezoneLabel1() {return clockConfig.label1;}
//String getTimezoneLabel2() {return clockConfig.label2;}



// Date/Time Format Functions ==================================================

//------------------------------------------------------------------------------
/* Converts the given local time (in seconds since 1970-01-01 00:00:00) to a 
   date string.  Unlike other routines, this time argument is relative to the
   local time, not UTC. */
String getDateString(time_t t) {
  tmElements_t tm;
  breakTime(t,tm);
  char sbuffer[16];
  sprintf(sbuffer,"%4d-%02d-%02d",1970+tm.Year,tm.Month,tm.Day);
  return sbuffer;
}


//------------------------------------------------------------------------------
/* Converts the given local time (in seconds since 1970-01-01 00:00:00) to a 
   time string.  Unlike other routines, this time argument is relative to the
   local time, not UTC. */
String getTimeString(time_t t) {
  tmElements_t tm;
  breakTime(t,tm);
  char sbuffer[16];
  sprintf(sbuffer,"%02d:%02d:%02d",tm.Hour,tm.Minute,tm.Second);
  return sbuffer;
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a date string for the local timezone.  Current time will be used if
   the argument is zero. */
String getLocalDateString(time_t t) {
  if (t == 0) t = getUTC();
  // If t = 0, did not retrieve time from RTC (do not convert)
  time_t tloc = (t > 0) ? timezone.toLocal(t) : 0;
  return getDateString(tloc);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a time string for the local timezone.  Current time will be used if
   the argument is zero. */
String getLocalTimeString(time_t t) {
  if (t == 0) t = getUTC();
  // If t = 0, did not retrieve time from RTC (do not convert)
  time_t tloc = (t > 0) ? timezone.toLocal(t) : 0;
  return getTimeString(tloc);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a date & time string for the local timezone.  Current time will be used
   if the argument is zero. */
String getLocalDateTimeString(time_t t) {
  if (t == 0) t = getUTC();
  TimeChangeRule *rule;
  time_t tloc = timezone.toLocal(t,&rule);
  // If t = 0, did not retrieve time from RTC (reset to 0).
  // Above still needed to retrieve timezone label.
  if (t == 0) tloc = 0;
  return getDateString(tloc) + " " + getTimeString(tloc) + " " + String(rule->abbrev);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a UTC date string.  Current time will be used if the argument is zero. */
String getUTCDateString(time_t t) {
  if (t == 0) t = getUTC();
  return getDateString(t);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a UTC time string.  Current time will be used if the argument is zero. */
String getUTCTimeString(time_t t) {
  if (t == 0) t = getUTC();
  return getTimeString(t);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a UTC date & time string.  Current time will be used if the argument is
   zero. */
String getUTCDateTimeString(time_t t) {
  if (t == 0) t = getUTC();
  return getDateString(t) + " " + getTimeString(t) + " UTC";
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a date string intended for database uploads.  Current time will
   be used if the argument is zero. */
String getDBDateString(time_t t) {
  return getUTCDateString(t);
  //return getLocalDateString(t);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a time string intended for database uploads.  Current time will
   be used if the argument is zero. */
String getDBTimeString(time_t t) {
  return getUTCTimeString(t);
  //return getLocalTimeString(t);
}


//------------------------------------------------------------------------------
/* Converts the given unix time (in seconds since 1970-01-01 00:00:00 UTC) 
   to a date & time string intended for database uploads.  Current time will
   be used if the argument is zero. */
String getDBDateTimeString(time_t t) {
  if (t == 0) t = getUTC();
  // MySQL: cannot directly submit ISO 8601 formatted datetime strings.
  // Just submit and store a MySQL-compatible UTC datetime string.
  return getDateString(t) + " " + getTimeString(t);
  
  // ISO 8601 formats:
  //   YYYY-MM-DDThh:mm:ss
  //   YYYY-MM-DDThh:mm:ss[{+|-}hh:mm]
  //   YYYY-MM-DDThh:mm:ssZ
  // In this form (note the 'T' between the date and time), SQL should
  // interpret this datetime correctly, without have to set the locale or
  // define year/month/day order.  The second version allows a timezone
  // offset to be specified and the third version indicates the time is
  // in UTC.
  
  // LOCAL TIME
  //TimeChangeRule *rule;
  //time_t tloc = timezone.toLocal(t,&rule);
  //return getDateString(tloc) + "T" + getTimeString(tloc);
  
  // LOCAL TIME WITH OFFSET
  //TimeChangeRule *rule;
  //time_t tloc = timezone.toLocal(t,&rule);
  //int offset = rule->offset;
  //char buff[16];
  //sprintf(buff,"%c%02d%02d", (offset>=0) ? '+' : '-',
  //        abs(offset) / 60, abs(offset) % 60);
  //return getDateString(tloc) + "T" + getTimeString(tloc) + buff;
  
  // UTC TIME
  //return getDateString(t) + "T" + getTimeString(t);
  
  // UTC TIME WITH TIMEZONE CODE
  //return getDateString(t) + "T" + getTimeString(t) + "Z";
}



// Testing Functions ===========================================================

//------------------------------------------------------------------------------
/* Clock date/time testing routine.
   Will output clock information for the given number cycles with the given
   interval [ms] between cycles. */
// Clock testing >>>>>>>>>>>>>>>>>>>>>>>
#ifdef CLOCK_TESTING
void testClock(unsigned long cycles, unsigned long interval) {
  Serial.println();
  Serial.println(F("Real-time clock testing"));
  Serial.println(F("-----------------------"));
  Serial.println();
  Serial.println(F("Initializing RTC...."));
  initRTC();
  delay(10);
  if (!probeRTC()) {
    Serial.println(F("Warning: Failed to connect to the RTC."));
  }
  Serial.println(F("Starting clock sampling...."));
  unsigned long t0 = millis();
  
  // Arduino implementation of printf drops %f support to reduce
  // memory usage.  We use dsostrf instead.
  char hbuffer1[128],hbuffer2[128];
  char sbuffer[128];

  // Table header
  //Serial.println();
  sprintf(hbuffer1,"  %8s  %8s  %10s  %23s  %23s",
          "time[ms]","timezone"," unix time","       UTC time       ","      local time      ");
  //Serial.println(hbuffer1);
  sprintf(hbuffer2,"  %8s  %8s  %10s  %23s  %23s",
          "--------","--------","----------","-----------------------","-----------------------");
  //Serial.println(hbuffer2);

  // Repeat measurements cycles number of times.
  // Logic here allows endless looping for cycles=-1 (maximum unsigned long).
  for (unsigned long k = 1; k <= cycles; k++) {
    //delay(sampleInterval);
    // Break out of the testing loop if the user sends anything
    // over the serial interface.
    if (getSerialChar(interval) != (char)(-1)) break;

    // Repeat header every so often
    // Ensure write on first loop (k=1)
    if (k % 50 == 1) {
      Serial.println();
      Serial.println(hbuffer1);
      Serial.println(hbuffer2);
    }

    unsigned long t = millis() - t0;
    unsigned long utc = getUTC();
    String tzstring = getStandardTimezoneLabel() + "/" + getDaylightSavingTimezoneLabel();
    String utcstring = getUTCDateTimeString(utc);
    String locstring = getLocalDateTimeString(utc);
    //String locstring = getDBDateTimeString(utc);
    sprintf(sbuffer,"%10ld  %8s  %10ld  %23s  %23s",
            t,tzstring.c_str(),utc,utcstring.c_str(),locstring.c_str());
    Serial.println(sbuffer);
  }
  
  Serial.println();
  Serial.println(F("Clock testing is complete."));
  Serial.println();
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// Helper Functions ============================================================

// Helper functions for other routines.  May not be declared in header file
// as they are not intended for external use.

//------------------------------------------------------------------------------
/* Get 0-indexed month and day of month for the given day of year and
   array if first days of month. */
void findMonthDay(const uint16_t month_doy[12], const uint16_t doy, uint8_t &M, uint8_t &D) {
  // Check if at or above highest array value
  if (doy >= month_doy[11]) {
    M = 11;
    D = doy - month_doy[M];
    return;
  }
  // Binary search
  uint8_t i1 = 0;
  uint8_t i2 = 11;
  while (i1 + 1 < i2) {
    uint8_t imid = (i1+i2) / 2;
    if (doy >= month_doy[imid]) {
      i1 = imid;
    } else {
      i2 = imid;
    }
  }
  M = i1;
  D = doy - month_doy[M];
  return;
}


//------------------------------------------------------------------------------
/* Convert between binary-coded decimal and binary.  Binary-coded decimal
   has 5-7 bits, where the lowest 4 bits are the decimal singles digit and
   the higher bits are the decimal tens digit. */
uint8_t toBCD(const uint8_t v) {return ((v / 10) << 4) + (v % 10);}
uint8_t fromBCD(const uint8_t v) {return 10 * (v >> 4) + (v & 0x0F);}



// DS3234 RTC Functions ========================================================

// Initialization, clock set/get, and low-level SPI read/write routines
// for the DS3234 RTC module.  Added here to avoid need for external library.
// Note the low-level routines do _not_ check if register address(es) are valid
// for this module!

// Good reference for Arduino SPI interaction with DS3234 RTC:
//   https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library
// Below SPI routines closely follow SPI routines from above library.


//------------------------------------------------------------------------------
/* Initializes the DS3234 RTC SPI bus. */
void initDS3234() {
  // Set signal select pin high (inactive)
  pinMode(RTC_PIN_CS, OUTPUT);
  digitalWrite(RTC_PIN_CS, HIGH);
  SPI.begin();
}


//------------------------------------------------------------------------------
/* Test connection with DS3234 RTC. */
bool probeDS3234() {
  // Check connection by successfully retrieving a non-zero byte.
  // The binary-coded decimal date byte should be between 1-31 (or
  // 1-49 prior to decoding).
  // Assuming here that SPI returns zero value if bus not working...
  uint8_t v;
  readDS3234Byte(0x04, v);
  //Serial.println(v);
  return (v >= 1) && (v <= 49);
}


//------------------------------------------------------------------------------
/* Get DS3234 RTC time as unix time (number of seconds since 1970-01-01 at
   00:00:00 UTC).  Returns 0 if could not retrieve time or returned time was
   invalid. */
time_t getDS3234Time() {
  // Get data from RTC in byte array.
  uint8_t v[DS3234_TIME_LEN];
  readDS3234Bytes(DS3234_TIME_ADDR, v, DS3234_TIME_LEN);
  /*
  for (int k = 0; k < DS3234_TIME_LEN; k++) {
    Serial.print(" ");
    Serial.print(v[k]);
  }
  Serial.println();
  */
  
  // Pull out individual components.
  // Convert some to 0-indexed.
  uint8_t s   = fromBCD(v[0]);      // seconds [0-59]
  uint8_t m   = fromBCD(v[1]);      // minutes [0-59]
  uint8_t h   = fromBCD(v[2]);      // hours [0-23]
  uint8_t dow = fromBCD(v[3]) - 1;  // day of week [1-7 -> 0-6]
  uint8_t D   = fromBCD(v[4]) - 1;  // day of month [1-31 -> 0-30]
  uint8_t M   = fromBCD(v[5]) - 1;  // month of year [1-12 -> 0-11]
  uint8_t Y   = fromBCD(v[6]);      // year [00-99]
  /*
  Serial.print(" "); Serial.print(s);
  Serial.print(" "); Serial.print(m);
  Serial.print(" "); Serial.print(h);
  Serial.print(" "); Serial.print(dow);
  Serial.print(" "); Serial.print(D);
  Serial.print(" "); Serial.print(M);
  Serial.print(" "); Serial.print(Y);
  Serial.println();
  */
  
  // Check validity of data.
  // Return 0 if invalid.
  if (D > 30) return 0;  // day of month [0-30]
  if (M > 11) return 0;  // month of year [0-11]

  // Suppress unused variable warning
  (void)dow;
  
  // Check if leap year, number of prior leap years (since 2000)
  bool isLeapYear = (Y % 4) == 0;
  uint8_t ld = (Y + 3) / 4;
  
  // Number of days since 2000-01-01
  uint16_t D2000 = 365 * Y + ld + (isLeapYear ? MONTH_DOLY[M] : MONTH_DOY[M]) + D;
  
  // Number of seconds since 2000-01-01
  time_t t = (((time_t)D2000 * 24 + h) * 60 + m) * 60 + s;

  // Return number of seconds since 1970-01-01
  const time_t UTC2000 = 946684800ul;
  return UTC2000 + t;
}


//------------------------------------------------------------------------------
/* Set DS3234 RTC time as unix time (number of seconds since 1970-01-01 at
   00:00:00 UTC).  Note this routine is intended only for times within the
   years 2000 - 2099 and will set the clock improperly outside this date
   range. */
void setDS3234Time(const time_t t) {
  // Do nothing if not a recent time (likely invalid)
  if (t < UTC_CUTOFF) return;
  // Working time
  time_t t0 = t;
  
  // Pull out seconds, minutes, hours
  uint8_t s = t0 % 60;
  t0 /= 60;
  uint8_t m = t0 % 60;
  t0 /= 60;
  uint8_t h = t0 % 24;
  t0 /= 24;
  
  // t0 is now number of days since 1970-01-01
  // Get day of week (0 is Sunday, 1970-01-01 was Thursday)
  uint8_t dow = (t0 + 4) % 7;
  // Change to days since 2000-01-01
  t0 -= 10957;  // 365*30 + 7 leap days
  
  // Check if leap year, get number of prior leap years
  bool isLeapYear = (t0 % 1461) < 366;  // First year of 4-year leap cycle
  // Get year (since 2000) and day of year
  uint8_t Y = 4 * (t0 / 1461);  // Number of completed 4-year leap cycles
  uint16_t doy = t0 % 1461;  // Days into 4-year leap cycle
  // Adjust for year in 4-year leap cycle
  if (doy >= 366) {
    Y += 1;
    doy -= 366;
    while (doy >= 365) {
      Y += 1;
      doy -= 365;
    }
  }
  
  // Month and day of month
  uint8_t M,D;
  if (isLeapYear) {
    findMonthDay(MONTH_DOLY,doy,M,D);
  } else {
    findMonthDay(MONTH_DOY,doy,M,D);
  }

  // Build byte array.
  // Must convert some of these from 0-indexed.
  uint8_t v[DS3234_TIME_LEN];
  v[0] = toBCD(s);      // seconds [0-59]
  v[1] = toBCD(m);      // minutes [0-59]
  v[2] = toBCD(h);      // hours [0-23]
  v[3] = toBCD(dow+1);  // day of week [1-7]
  v[4] = toBCD(D+1);    // day of month [1-31]
  v[5] = toBCD(M+1);    // month of year [1-12]
  v[6] = toBCD(Y);      // year [00-99]

  // Send data to RTC.
  writeDS3234Bytes(DS3234_TIME_ADDR, v, DS3234_TIME_LEN);
}


//------------------------------------------------------------------------------
/* Read a single byte from the given register of the DS3234 RTC. */
void readDS3234Byte(const uint8_t reg, uint8_t &v) {
  SPI.beginTransaction(rtcSPISettings);
  digitalWrite(RTC_PIN_CS, LOW);
  SPI.transfer(reg);
  v = SPI.transfer(0x00);
  digitalWrite(RTC_PIN_CS, HIGH);
  SPI.endTransaction();
}


//------------------------------------------------------------------------------
/* Read multiple bytes starting from the given register of the DS3234 RTC. */
void readDS3234Bytes(const uint8_t reg, uint8_t *v, const uint8_t len) {
  SPI.beginTransaction(rtcSPISettings);
  digitalWrite(RTC_PIN_CS, LOW);
  SPI.transfer(reg);
  for (size_t k = 0; k < len; k++) {
    v[k] = SPI.transfer(0x00);
  }
  digitalWrite(RTC_PIN_CS, HIGH);
  SPI.endTransaction();
}


//------------------------------------------------------------------------------
/* Write a single byte to the given register of the DS3234 RTC. */
void writeDS3234Byte(const uint8_t reg, const uint8_t v) {
  SPI.beginTransaction(rtcSPISettings);
  digitalWrite(RTC_PIN_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(v);
  digitalWrite(RTC_PIN_CS, HIGH);
  SPI.endTransaction();
}


//------------------------------------------------------------------------------
/* Write multiple bytes starting at the given register of the DS3234 RTC. */
void writeDS3234Bytes(const uint8_t reg, const uint8_t *v, const uint8_t len) {
  SPI.beginTransaction(rtcSPISettings);
  digitalWrite(RTC_PIN_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t k = 0; k < len; k++) {
    SPI.transfer(v[k]);
  }
  digitalWrite(RTC_PIN_CS, HIGH);
  SPI.endTransaction();
}



//==============================================================================
