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
  pinCheck(/*WIZ812MJ_ES_PIN*/ 20,"Ethernet chip select");
  pinCheck(PIN_B0,"B0");
  pinCheck(ETHERNET_EN,"Ethernet enable");
  pinCheck(PIN_F5,"F5");
  
  pinCheck(DS13074_CS_PIN,"RTC chip select");
  pinCheck(PIN_C7,"C7");
  
  pinCheck(SD_CHIP_SELECT,"SD chip select");
  pinCheck(PIN_C0,"C0");
  
  pinCheck(/*PM_EN*/ 42,"PM enable");
  pinCheck(PIN_F4,"F4");
  pinCheck(16,"PM 2.5");
  pinCheck(PIN_C6,"C6");
  pinCheck(15,"PM 10");
  pinCheck(PIN_C5,"C5");
  
  // Note A0 is an analog pin but PIN_A0 is not: the latter
  // refers to pins on the A register, not "analog".
  pinCheck(MIC_PIN,"Microphone");
  pinCheck(A0,"A0");
  //pinCheck(PIN_A0,"PIN_A0");
  
  pinCheck(A1,"Globe temp");
  pinCheck(A1,"A1");
  
  pinCheck(A2,"Light");
  pinCheck(A2,"A2");
}


//==============================================================================

