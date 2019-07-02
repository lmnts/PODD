/*==============================================================================
  Various utility constants and functions.

  This file is part of the LMN PODD distribution:
    https://github.com/lmnts/PODD

  CONTRIBUTORS:
    Chris Savage (2018)

  COPYRIGHT/LICENSE:
  Copyright (c) 2018 LMN Architects

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

#include "pod_util.h"

// Grab some pin definitions
#include "pod_config.h"
#include "pod_logging.h"
#include "pod_network.h"
#include "pod_sensors.h"


// Constants/global variables ==================================================


// Functions ===================================================================

//------------------------------------------------------------------------------
// The amount of free memory, as the size of the space between the heap
// and the stack.  See:
//   https://playground.arduino.cc/Code/AvailableMemory
//   https://github.com/arduino/Arduino/issues/5289
//   http://www.controllerprojects.com/2011/05/23/determining-sram-usage-on-arduino/ [defunct]
// Note there may be additional unused space within the heap, but this is
// the maximum free RAM available to the stack.
// 
size_t freeRAM() {
  extern int __heap_start;
  extern int *__brkval;
  // This contains end of heap location and itself is located at edge of stack
  const size_t heaploc = (size_t)(__brkval == 0 ? &__heap_start : __brkval);
  if ((size_t)(&heaploc) > heaploc) {
    return (size_t)(&heaploc) - heaploc;
  } else {
    // This is bad....
    return 0;
  }
}


//------------------------------------------------------------------------------
// Writes to serial the status of the given pin, with optional
// label to include in output.  Note this gives digital states:
// if pin is configured for analog output (PWM or something else),
// the output here may not be meaningful.
void pinCheck(const int pin, const String s /*=""*/) {
  Serial.print("Pin ");
  if (pin >= 100) Serial.print(" ");
  if (pin >= 10)  Serial.print(" ");
  Serial.print(pin);
  Serial.print(": ");
  // See: https://arduino.stackexchange.com/a/21017
  String state;
  if (pin >= NUM_DIGITAL_PINS) {
    state = "INVALID";
  } else {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg = portModeRegister(port);
    volatile uint8_t *out = portOutputRegister(port);
    bool output = *reg & bit;
    bool high   = *out & bit;
    if (output) {
      state = high ? "OUTPUT (HIGH)" : "OUTPUT (LOW) ";
    } else {
      state = high ? "INPUT_PULLUP " : "INPUT        ";
    }
  }
  Serial.print(state);
  if (!s.equals("")) Serial.print("  [" + s + "]");
  Serial.println();
}


//------------------------------------------------------------------------------
// Routine to output status of various PODD pins.  Used for
// debugging.
void poddPinChecks() {
  // Here, check hard-coded pin via constant or explicit
  // value (if constant not accessible), followed by
  // intended pin using teensy preprocessor constant.
  pinCheck(/*WIZ812MJ_ES_PIN*/ 20,F("Ethernet chip select"));
  pinCheck(PIN_B0,                F("B0"));
  pinCheck(ETHERNET_EN,           F("Ethernet enable"));
  pinCheck(PIN_F5,                F("F5"));
  
  pinCheck(/*RTC_PIN_CS*/ 17,F("RTC chip select"));
  pinCheck(PIN_C7,           F("C7"));
  
  pinCheck(/*SD_CHIP_SELECT*/ 10,F("SD chip select"));
  pinCheck(PIN_C0,               F("C0"));
  
  pinCheck(/*PM_ENABLE*/ 42,F("PM enable"));
  pinCheck(PIN_F4,          F("F4"));
  pinCheck(16,              F("PM 2.5"));
  pinCheck(PIN_C6,          F("C6"));
  pinCheck(15,              F("PM 10"));
  pinCheck(PIN_C5,          F("C5"));
  
  // Note A0 is an analog pin but PIN_A0 is not: the latter
  // refers to pins on the A register, not "analog".
  pinCheck(/*MIC_PIN*/ A0,F("Microphone"));
  pinCheck(A0,            F("A0"));
  //pinCheck(PIN_A0,        F("PIN_A0"));
  
  pinCheck(A1,F("Globe temp"));
  pinCheck(A1,F("A1"));
  
  pinCheck(A2,F("Light"));
  pinCheck(A2,F("A2"));
}


//------------------------------------------------------------------------------
// Prints compilation info to serial output, with optional prefix
// for each output line and name of compilation file (defaults to
// main compilation input file).  Info includes compilation file,
// compilation date & time, compiler version, Arduino version.
void printCompilationInfo(const String prefix, const String file) {
  if (!file.equals("")) {
    Serial.print(prefix);
    Serial.print(F("Compilation file:    "));
    // Get rid of any leading directories in file name
    int loc = max(file.lastIndexOf('\\'),file.lastIndexOf('/')) + 1;
    Serial.println(file.substring(loc));
  }
  Serial.print(prefix);
  Serial.println(F("Compilation date:    "  __DATE__ " " __TIME__));
  #if defined(__VERSION__)
  Serial.print(prefix);
  Serial.println(F("Compiler version:    " __VERSION__));
  #endif
  #if defined(ARDUINO)
  Serial.print(prefix);
  Serial.print(  F("Arduino version:     "));
  Serial.println(ARDUINO,DEC);
  #endif
  #if defined(TEENSYDUINO)
  Serial.print(prefix);
  Serial.print(  F("Teensyduino version: "));
  Serial.println(TEENSYDUINO,DEC);
  #endif
}


//==============================================================================
