/*==============================================================================
  Various serial interface constants and functions.

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
#include <limits.h>
// Contributed libraries
#include <Arduino.h>
// Local headers

// Define DEBUG, PODD_DEBUG, or PODD_SERIAL_DEBUG to enable
// debug statements below
#if defined(DEBUG) && !defined(PODD_DEBUG)
#define PODD_DEBUG
#endif
#if defined(PODD_DEBUG) && !defined(PODD_SERIAL_DEBUG)
#define PODD_SERIAL_DEBUG
#endif


// Constants/global variables ==================================================


// Functions ===================================================================

// Removes any data from the serial interface input.
void clearSerial();

// Get a single character or a string from serial input.
// Can optionally wait a maximum amount of time for input.
// In the string case, newline characters are stripped.
char getSerialChar(unsigned long timeout=0);
String getSerialString(unsigned long timeout=0);

// Prompt the user to provide information, with an optional default
// value (which will be shown) that will be returned if the user
// provides an empty response.  For boolean and numeric types,
// the user can optionally be reprompted if response if of invalid
// format.
char serialCharPrompt(String prompt, char default0=(char)(-1));
String serialStringPrompt(String prompt, String default0="");
bool serialYesNoPrompt(String prompt, bool reprompt, bool default0);
bool serialTrueFalsePrompt(String prompt, bool reprompt, bool default0);
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0);
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0, char tchar, char fchar);
int serialIntegerPrompt(String prompt, bool reprompt, int default0=INT_MIN);
long serialLongPrompt(String prompt, bool reprompt, long default0=LONG_MIN);
float serialFloatPrompt(String prompt, bool reprompt, float default0=NAN);


//==============================================================================
