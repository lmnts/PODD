/*==============================================================================
  Testing of the OPT3001 ambient light sensor.

  This file is part of the LMN PODD distribution:
    https://github.com/lmnts/PODD

  REQUIREMENTS: ClosedCube OPT3001 Arduino Library (available through
  Arduino library repository).

  HARDWARE: The OPT3001 ambient light sensor connected through I2C.
  If hardware address is not 0x45 (address pin connected to VDD), update
  the address below. Tested with ClosedCube OPT3001 breakout board on a
  Teensy++ 2.0.

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

#include <ClosedCube_OPT3001.h>

// OPT3001 I2C address:
//   0x44  ADDR pin to GND
//   0x45  ADDR pin to VDD
//   0x46  ADDR pin to SDA
//   0x47  ADDR pin to SDL
#define OPT3001_ADDR 0x45

ClosedCube_OPT3001 opt3001;


// INITIALIZATION ==============================================================

void setup() {
  Serial.begin(9600);
  
  // Give chance for serial monitor to be connected
  delay(5000);

  // Initialize sensor at given I2C address
  opt3001.begin(OPT3001_ADDR);

  // Initial configuration
  OPT3001_Config config;
  // Use 0000b to 1011b to explicitly set range,
  // or use 1100b for automatic scaling of range.
  config.RangeNumber = B1100;
  // Conversion time: 0 for 100ms, 1 for 800ms
  config.ConvertionTime = B1;  // [sic]
  // Use 00b to shutdown the sensor, 01b for a single-shot read (returns
  // to 00b after the read completes), or 11b for continuous sensor reading.
  config.ModeOfConversionOperation = B11;
  // The other configuration fields are irrelevant to the testing here
  // and not set.  See documentation for other possibilities (notably if
  // intending to use the interrupt pin).
  
  // Note automatic scaling goes up or down by 1-2 scales (x2 or x0.5) with
  // each measurement cycle until it reaches a reasonable range.  With the
  // longer integration time, it may take several seconds (up to ~ 10s) for
  // the readings to stabilize; during that time, the readings may be too
  // low and/or lack precision.
  
  // Upload configuration to sensor
  OPT3001_ErrorCode err = opt3001.writeConfig(config);
  if (err == NO_ERROR) {
    Serial.println("OPT3001 configured.");
  } else {
    Serial.print("OPT3001 configuration error: ");
    Serial.println(err);
  }
  delay(1000);
}


// LOOP ========================================================================

void loop() {
  char cbuf[32];
  
  sprintf(cbuf,"%10lu: ",millis());
  Serial.print(cbuf);
  
  OPT3001 reading = opt3001.readResult();
  if (reading.error == NO_ERROR) {
    // Resolution is as small as 0.01 lux, maximum is ~ 80,000 lux
    // Arduino implementation of printf drops %f support for performance?
    //sprintf(cbuf,"%10.2f lux",(double)reading.lux);  // cast to avoid warning
    //Serial.println(cbuf);
    dtostrf(reading.lux,10,2,cbuf);
    Serial.print(cbuf);
    Serial.println(" lux");
  } else {
    sprintf(cbuf,"Error reading sensor (%d)",reading.error);
    Serial.println(cbuf);
  }
  delay(1000);
}


//==============================================================================


