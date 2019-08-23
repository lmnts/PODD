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

#include "pod_serial.h"


// Constants/global variables ==================================================


// Functions ===================================================================

//------------------------------------------------------------------------------
/* Removes any data from the serial interface input. */
void clearSerial() {
  while (Serial.available()) Serial.read();
}


//------------------------------------------------------------------------------
/* Returns the first character available on the serial interface,
   optionally waiting the given amount of time [ms] for data to
   become available.  Remaining serial data is cleared.  If timeout
   is reached before data becomes available, -1 is returned.
   A timeout of -1 can be given to wait indefinitely for data. */
char getSerialChar(unsigned long timeout) {
  unsigned long t0 = millis();
  do {
    if (Serial.available()) {
      char c = Serial.read();
      // Clear remaining serial data (including any end-of-line
      // characters).  We give a little time for multi-character
      // serial data to make it across the bus (not complete
      // until serial buffer remains empty for 1 ms).
      // Double while loops are not redundant...
      delay(1);
      while (Serial.available()) {
        while (Serial.available()) Serial.read();
        delay(1);
      }
      return c;
    }
    delay(1);
  } while (millis() - t0 <= timeout);
  return -1;
}


//------------------------------------------------------------------------------
/* Returns data available on the serial interface up to a newline
   character, optionally waiting the given amount of time [ms] for
   data to become available.  Remaining serial data is cleared.
   If timeout is reached before data becomes available, an empty
   string is returned.  A timeout of -1 can be given to wait
   indefinitely for data. */
String getSerialString(unsigned long timeout) {
  String s = "";
  bool complete = false;
  unsigned long t0 = millis();
  // Parse serial until end-of-line character found
  // or timeout reached.
  do {
    while (!complete && Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case '\r':
          complete = true;
          break;
        case '\n':
          complete = true;
          break;
        default:
          s = s + c;
          break;
      }
    }
    if (complete) break;
    delay(1);
  } while (millis() - t0 <= timeout);

  // Clear remaining serial data (including any end-of-line
  // characters).  We give a little time for multi-character
  // serial data to make it across the bus (not complete
  // until serial buffer remains empty for 1 ms).
  if (complete) {
    delay(1);
    // Double while loops are not redundant...
    while (Serial.available()) {
      while (Serial.available()) Serial.read();
      delay(1);
    }
  }
  
  return s;
}


//------------------------------------------------------------------------------
/* Retrieves and parses serial data to an integer value.
   Returns smallest integer value (INT_MIN from <limits.h>) if
   data is empty except for newline character.  Can optionally
   reprompt for user input if data is non-empty, but invalid. */
int getSerialInt(bool reprompt) {
  const int EMPTY_VALUE = INT_MIN;
  while (true) {
    String s = getSerialString(-1);
    s.trim();
    // If provided input is empty, return smallest integer value
    // (our empty indicator value)
    if (s.equals("")) return EMPTY_VALUE;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      // coerce float value to integer range
      if (f > INT_MAX) return INT_MAX;
      if (f <= INT_MIN+1) return INT_MIN + 1;
      return (int)f;
    }
    if (!reprompt) return EMPTY_VALUE;
    Serial.println(s);  // Serial Monitor does not echo inputs
    Serial.print(F("Invalid input.  Try again: "));
  }
  // Should not reach here...
  return EMPTY_VALUE;
}


//------------------------------------------------------------------------------
/* Retrieves and parses serial data to a long integer value.
   Returns smallest integer value (LONG_MIN from <limits.h>) if
   data is empty except for newline character.  Can optionally
   reprompt for user input if data is non-empty, but invalid. */
long getSerialLong(bool reprompt) {
  const long EMPTY_VALUE = LONG_MIN;
  while (true) {
    String s = getSerialString(-1);
    s.trim();
    // If provided input is empty, return smallest integer value
    // (our empty indicator value)
    if (s.equals("")) return EMPTY_VALUE;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    //double f = strtod(buff,&end);
    // end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return l;
      // coerce float value to integer range
      //if (f > LONG_MAX) return LONG_MAX;
      //if (f <= LONG_MIN+1) return LONG_MIN + 1;
      //return (long)f;
    }
    if (!reprompt) return EMPTY_VALUE;
    Serial.println(s);  // Serial Monitor does not echo inputs
    Serial.print(F("Invalid input.  Try again: "));
  }
  // Should not reach here...
  return EMPTY_VALUE;
}


//------------------------------------------------------------------------------
/* Retrieves and parses serial data to a float value.
   Returns NAN (from <math.h>) if
   data is empty except for newline character.  Can optionally
   reprompt for user input if data is non-empty, but invalid. */
float getSerialFloat(bool reprompt) {
  const float EMPTY_VALUE = NAN;
  while (true) {
    String s = getSerialString(-1);
    s.trim();
    // If provided input is empty, return smallest integer value
    // (our empty indicator value)
    if (s.equals("")) return EMPTY_VALUE;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return f;
    }
    if (!reprompt) return EMPTY_VALUE;
    Serial.println(s);  // Serial Monitor does not echo inputs
    Serial.print(F("Invalid input.  Try again: "));
  }
  // Should not reach here...
  return EMPTY_VALUE;
}


//------------------------------------------------------------------------------
/* Retrieves a single character from an interactive serial
   interface, with the given prompt provided to the user.
   A default value can be specified, used if an empty
   response is given. */
char serialCharPrompt(String prompt, char default0) {
  // Provide prompt to user
  Serial.print(prompt);
  if (default0 != (char)(-1)) {
    Serial.print(F(" ["));
    Serial.print(default0);
    Serial.print(F("]"));
  }
  Serial.print(F(": "));
  // Get user's response
  char c = getSerialChar(-1);
  //Serial.println();
  Serial.println(c);  // Serial Monitor does not echo inputs
  // Empty response: use default
  if ((c == '\r') || (c == '\n')) return default0;
  return c;
}


//------------------------------------------------------------------------------
/* Retrieves a string from an interactive serial interface,
   with the given prompt provided to the user.
   A default value can be specified, used if an empty
   response is given. */
String serialStringPrompt(String prompt, String default0) {
  // Provide prompt to user
  Serial.print(prompt);
  if (!default0.equals("")) {
    Serial.print(F(" [\""));
    Serial.print(default0);
    Serial.print(F("\"]"));
  }
  Serial.print(F(": "));
  // Get user's response
  String s = getSerialString(-1);
  //Serial.println();
  Serial.println(s);  // Serial Monitor does not echo inputs
  s.trim();
  // Empty response: use default
  if (s.equals("")) return default0;
  return s;
}


//------------------------------------------------------------------------------
/* Retrieves a boolean response from an interactive serial 
   interface, with the given prompt provided to the user.
   The user can optionally be reprompted if an invalid
   response is received.
   A default value must be specified, used if an empty
   response is given. */
bool serialYesNoPrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'y','n');
}
bool serialTrueFalsePrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'t','f');
}
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'t','f');
}

/* Main routine for above serial boolean prompt routines.
   Allows prompt's true/false characters to be specified,
   but this applies only to the prompt strings, not to the
   allowed input (hardcoded to look for 't'/'y' or 'f'/'n'
   as first character, case-insensitive).
   This routine is not intended to be called by anything
   other than the above routines (use those instead). */
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0, char tchar, char fchar) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    Serial.print(F(" ["));
    Serial.print(default0 ? (char)toupper(tchar) : (char)toupper(fchar));
    Serial.print(F("]"));
    Serial.print(F(": "));
    // Get user's response
    char c = getSerialChar(-1);
    //Serial.println();
    Serial.println(c);  // Serial Monitor does not echo inputs
    // Process response
    switch (tolower(c)) {
      // true/yes
      case 't':
      case 'y':
        return true;
        break;
      // false/no
      case 'f':
      case 'n':
        return false;
        break;
      // Empty response: use default
      case '\r':
      case '\n':
        return default0;
        break;
    }
    if (!reprompt) return default0;
    Serial.print(F("  Invalid input (use '"));
    Serial.print(tchar);
    Serial.print(F("' or '"));
    Serial.print(fchar);
    Serial.print(F("'."));
    Serial.println();
  }
  // Should not reach here...
  return default0;
}


//------------------------------------------------------------------------------
/* Retrieves a number from an interactive serial interface,
   with the given prompt provided to the user.  The user
   can optionally be reprompted if a non-numeric response
   is received.
   A default value can be specified, used if an empty
   response is given. */
int serialIntegerPrompt(String prompt, bool reprompt, int default0) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    if (default0 != INT_MIN) {
      Serial.print(F(" ["));
      Serial.print(default0);
      Serial.print(F("]"));
    }
    Serial.print(F(": "));
    // Get user's response
    String s = getSerialString(-1);
    //Serial.println();
    Serial.println(s);  // Serial Monitor does not echo inputs
    s.trim();
    // Empty response: use default
    if (s.equals("")) return default0;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // *end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      // coerce float value to integer range
      if (f >= INT_MAX) return INT_MAX;
      if (f <= INT_MIN) return INT_MIN;
      return (int)f;
    }
    if (!reprompt) return default0;
    Serial.println(F("  Invalid input (not numeric)."));
  }
  // Should not reach here...
  return default0;
}


//------------------------------------------------------------------------------
/* Retrieves a number from an interactive serial interface,
   with the given prompt provided to the user.  The user
   can optionally be reprompted if a non-numeric response
   is received.
   A default value can be specified, used if an empty
   response is given. */
long serialLongPrompt(String prompt, bool reprompt, long default0) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    if (default0 != LONG_MIN) {
      Serial.print(F(" ["));
      Serial.print(default0);
      Serial.print(F("]"));
    }
    Serial.print(F(": "));
    // Get user's response
    String s = getSerialString(-1);
    //Serial.println();
    Serial.println(s);  // Serial Monitor does not echo inputs
    s.trim();
    // Empty response: use default
    if (s.equals("")) return default0;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    //double f = strtod(buff,&end);
    // *end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return l;
      // coerce float value to integer range
      //if (f >= LONG_MAX) return LONG_MAX;
      //if (f <= LONG_MIN) return LONG_MIN;
      //return (long)f;
    }
    if (!reprompt) return default0;
    Serial.println(F("  Invalid input (not numeric)."));
  }
  // Should not reach here...
  return default0;
}


//------------------------------------------------------------------------------
/* Retrieves a number from an interactive serial interface,
   with the given prompt provided to the user.  The user
   can optionally be reprompted if a non-numeric response
   is received.
   A default value can be specified, used if an empty
   response is given. */
float serialFloatPrompt(String prompt, bool reprompt, float default0) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    if (!isnan(default0)) {
      Serial.print(F(" ["));
      Serial.print(default0);
      Serial.print(F("]"));
    }
    Serial.print(F(": "));
    // Get user's response
    String s = getSerialString(-1);
    //Serial.println();
    Serial.println(s);  // Serial Monitor does not echo inputs
    s.trim();
    // Empty response: use default
    if (s.equals("")) return default0;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // *end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return (float)f;
    }
    if (!reprompt) return default0;
    Serial.println(F("  Invalid input (not numeric)."));
  }
  // Should not reach here...
  return default0;
}


//==============================================================================
