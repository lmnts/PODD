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

#pragma once

// Standard libraries
// Contributed libraries
#include <Arduino.h>
#include <TimeLib.h>
// Local headers

// Define DEBUG, PODD_DEBUG, or PODD_CLOCK_DEBUG to enable
// debug statements below
#if defined(DEBUG) && !defined(PODD_DEBUG)
#define PODD_DEBUG
#endif
#if defined(PODD_DEBUG) && !defined(PODD_CLOCK_DEBUG)
#define PODD_CLOCK_DEBUG
#endif

// Define this to enable clock testing routines.
//#define CLOCK_TESTING


// Constants/global variables ==================================================


// Functions ===================================================================

// Initializes the RTC
void initRTC();
// Indicates is RTC is present and accessible
bool probeRTC();

// Set or get the time using unix time: number of seconds since 1970-01-01
// at 00:00:00 UTC.  Backed by RTC.
// As a safety measure, time will not be set if t < 1000000000.
void setUTC(time_t t);
time_t getUTC();

// Number of seconds since 1970-01-01 at 00:00:00 in configured timezone.
time_t getLocalTime();

// Same as above, but using a structure with date/time elements.
// Note the year field in this structure is numbers of years since 1970
// (10 -> 1980), whereas the DS3234 RTC uses the last two year digits
// (10 -> 2010); these routines handle that conversion.  Also, day-of-
// week is 1-7 in structure (1: Sunday), but 0-6 for DS3234 (0: Sunday).
//void setUTC(const tmElements_t &t);
//void getUTC(tmElements_t &t);

// Load the timezone from EEPROM if available, otherwise use a
// default (Pacific)
void initTimezone();

// Set the timezone by the regular & daylight saving labels (e.g. "PST"
// and "PDT").  If a second label is not provided, the first timezone is
// used yearwide and daylight savings is not applied.
// Currently implemented labels: EST, EDT, CST, CDT, MST, MDT, PST, PDT,
// UTC.
void setTimezone(String tz, String dtz="");

// Gets the local timezone label for the given UTC unix time, or the
// current time if the argument is zero.
String getTimezoneLabel(time_t t=0);

// Gets the timezone labels as provided to setTimezone routine.
String getStandardTimezoneLabel();
String getDaylightSavingTimezoneLabel();
//String getTimezoneLabel1();
//String getTimezoneLabel2();

// Date and/or time strings for UTC or local time.
// Argument is unix time to convert; current time will be used if
// argument is zero.
String getLocalTimeString(time_t t=0);
String getLocalDateString(time_t t=0);
String getLocalDateTimeString(time_t t=0);  // includes timezone label
String getUTCTimeString(time_t t=0);
String getUTCDateString(time_t t=0);
String getUTCDateTimeString(time_t t=0);  // includes timezone label

// Date/time string format intended for database time field.
String getDBDateString(time_t t=0);
String getDBTimeString(time_t t=0);
String getDBDateTimeString(time_t t=0);

// Clock testing routine.
// Number of cycles (-1 for infinite) and interval between cycles.
#ifdef CLOCK_TESTING
void testClock(unsigned long cycles = -1, unsigned long interval = 1000);
#endif

// DS3234 RTC module initialization, clock, and low-level communication.
void initDS3234();
bool probeDS3234();
time_t getDS3234Time();
void setDS3234Time(const time_t t);
void readDS3234Byte(const uint8_t reg, uint8_t &v);
void readDS3234Bytes(const uint8_t reg, uint8_t *v, const uint8_t len);
void writeDS3234Byte(const uint8_t reg, const uint8_t v);
void writeDS3234Bytes(const uint8_t reg, const uint8_t *v, const uint8_t len);


//==============================================================================
