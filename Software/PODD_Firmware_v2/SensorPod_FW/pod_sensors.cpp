
/*
 * pod_sensors.cpp
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 * 
 * Handle setup and reading from various sensors.
 */

#include "pod_sensors.h"

#include <AsyncDelay.h>
#include "cozir.h"
#include <Wire.h>
#include <HIH61xx.h>

// SOUND
unsigned int knock;
#define sampletime_DB 5000

// CO2
SoftwareSerial nss(26, 25); //used pins 25 for rx, 26 for tx.
COZIR czr(nss);

// PM Sensor
AsyncDelay samplingPM;
#define sampletime_PM 8000UL
#define PM_PIN_2_5 16 // Pin number switched with PM_PIN_10 due to pinout being reversed in schematic
#define PM_PIN_10 15 // Pin number switched with PM_PIN_2_5 due to pinout being reversed in schematic
unsigned long durationL1;
unsigned long durationL2;
unsigned long lowpulseoccupancy1 = 0;
unsigned long lowpulseoccupancy2 = 0;
float ratio1 = 0;
float ratio2 = 0;
float concentration2_5 = 0;
float concentration10 = 0;
int endOfSampling = 0;

// CO
//#define numCoRead 4
//int samples[numCoRead];
#define CoSpecSensor A3

// Thermistor and light Pins
#define TempGpin A1
#define lightPin A2

// HIH
#define HIH_ADDR 0x27
HIH61xx<TwoWire> hih(Wire);


//--------------------------------------------------------------------------------------------- [Sensor Reads]

void sensorSetup() {
  // PM Sensor
  pinMode(PM_PIN_2_5, INPUT);
  pinMode(PM_PIN_10, INPUT);
  pinMode(PM_ENABLE, OUTPUT);
  digitalWrite(PM_ENABLE, HIGH);

  analogReference(EXTERNAL);

  hih.initialise(HIH_ADDR);
  czr.SetOperatingMode(CZR_POLLING);
}

bool verifySensors() {
  // gets data from all sensors
  // returns false if a sensors data is out of the valid range (indicating sensor failure)
  float tval_f;
  double tval_d;
  int tval_i;

  tval_f = getRHTemp();
  if (tval_f < MIN_RH_TEMP || tval_f > MAX_RX_TEMP) {
    Serial.println(F("Humidity T Failure: ") + String(tval_f));
    return false;
  }
 
  tval_f = getRHHum();
  if (tval_f < MIN_RH_HUM || tval_f > MAX_RX_HUM) {
    Serial.println(F("Humidity RH Failure: ") + String(tval_f));
    return false;
  }

  tval_f = getLight();
  if (tval_f < MIN_LIGHT || tval_f > MAX_LIGHT) {
    Serial.println(F("Light Failure: ") + String(tval_f));
    return false;
  }

  tval_d = getGlobeTemp();
  if (tval_d < MIN_GLOBE_TEMP || tval_d > MAX_GLOBE_TEMP) {
    Serial.println(F("Temp Failure: ") + String(tval_d));
    return false;
  }

  tval_d = getSound();
  if (tval_d < MIN_SOUND || tval_d > MAX_SOUND) {
    Serial.println(F("Sound Failure: ") + String(tval_d));
    return false;
  }

  tval_i = getCO2();
  if (tval_i < MIN_CO2 || tval_i > MAX_CO2) {
    Serial.println(F("CO2 Failure: ") + String(tval_i));
    return false;
  }

  tval_f = getCO();
  if (tval_f < MIN_CO || tval_f > MAX_CO) {
    Serial.println(F("CO Failure: ") + String(tval_f));
    return false;
  }

  updatePM();
  tval_f = getPM2_5();
  if (tval_f < MIN_PM2_5 || tval_f > MAX_PM2_5) {
    Serial.println(F("PM 2.5 Failure: ") + String(tval_f));
    return false;
  }
  tval_f = getPM10();
  if (tval_f < MIN_PM10 || tval_f > MAX_PM10) {
    Serial.println(F("PM 10 Failure: ") + String(tval_f));
    return false;
  }

  
  return true;
}

void calibrateCO2() {
  // see documentation for usage
  // requires external sources of known concentration CO2 gas
  // CO2 calibration
  /*Here is a step by step
    uncomment czr.SetOperatingMode(CZR_POLLING);
    Upload Sketch
    comment czr.SetOperatingMode(CZR_POLLING);
    and uncomment czr.CalibrateFreshAir();
    Upload Sketch
    comment czr.CalibrateFreshAir();
    Upload sketch*/
  //czr.SetOperatingMode(CZR_POLLING);
  //czr.SetOperatingMode(CZR_STREAMING);
  //czr.CalibrateFreshAir();
  //czr.SetDigiFilter(32);
}

/* TODO: Potential for optimization of HIH Temp+RH sensor here.
   A single hih.read() updates both values and a timer can be
   used to signal when to collect the reading after initiating
   the measurement, rather than just sitting here waiting. */

float getRHTemp() {
  hih.read();  // Blocks for ~45ms as sensor is read
  return hih.getAmbientTemp()/100.0;  // hih gives temp x 100
}

float getRHHum() {
  hih.read();  // Blocks for ~45ms as sensor is read
  return hih.getRelHumidity()/100.0;  // hih gives RH[%] x 100
}

float getLight() {
  float lightRaw = analogRead(lightPin);
  return (lightRaw / 1023) * 3300; // TODO: comment this function, what's with these magic numbers
}

double getGlobeTemp() {
  float tempV = analogRead(TempGpin);
  int R = 10000; //actual resistor value
  double Rt, logRt, T;
  // Floats from U.S Sensor Corp. Curve J sheet
  float A = 0.00147530413409933;
  float B = 0.000236552076866679;
  float C = 0.000000118857119853526;
  float D = -0.000000000074635312369958;
  Rt = R * (1024.0 / tempV - 1.0);
  logRt = log(Rt);
  T = ((1.0 / (A + (B * logRt) + (C * (logRt * logRt * logRt)) + (D * (logRt * logRt * logRt * logRt * logRt)))) - 273.15) * 1.8 + 32;
  return T;
}

double getSound() {
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  unsigned long start = millis(); // Start of sample window
  while (millis() - start <= sampletime_DB)
  {
    knock = analogRead(MIC_PIN);
    if (knock > signalMax)
    {
      signalMax = knock;  // save just the max levels
    }
      else if (knock < signalMin)
    {
      signalMin = knock;  // save just the min levels
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double SoundAmp = map(peakToPeak, 0, 1023, 300, 1000);
  return SoundAmp;
}

int getCO2() {
  int c = czr.CO2();
  return c;
}

float getCO() {
  return analogRead(CoSpecSensor); // TODO: Conversion!
}

void updatePM() {
  lowpulseoccupancy1 = 0;
  lowpulseoccupancy2 = 0;
  
  //PM sampling starts here
  samplingPM.start(sampletime_PM, AsyncDelay::MILLIS);
  Serial.println(F("PM sampling started...."));
  do
  {
    if (samplingPM.isExpired()) {
      endOfSampling = 1;
      Serial.println(F("expired"));
    }
    else
    {
      endOfSampling = 0;
      durationL1 = pulseIn(PM_PIN_2_5, LOW);
      lowpulseoccupancy1 = lowpulseoccupancy1 + durationL1;
      Serial.print(F("P1 measured: "));
      Serial.println(lowpulseoccupancy1);

      durationL2 = pulseIn(PM_PIN_10, LOW);
      lowpulseoccupancy2 = lowpulseoccupancy2 + durationL2;
      Serial.print(F("P2 measured: "));
      Serial.println(lowpulseoccupancy2);
    }
  } while (endOfSampling == 0);

  ratio1 = lowpulseoccupancy1 / (sampletime_PM);
  ratio1 = (ratio1 * 0.001);
  concentration2_5 = 1.1 * pow(ratio1, 3) - 3.8 * pow(ratio1, 2) + 520 * ratio1 + 0.62; // using spec sheet curve
  Serial.print(F("PM 2.5: "));
  Serial.println(concentration2_5, 3);
  
  ratio2 = lowpulseoccupancy2 / (sampletime_PM);
  ratio2 = (ratio2 * 0.001);
  concentration10 = 1.1 * pow(ratio2, 3) - 3.8 * pow(ratio2, 2) + 520 * ratio2 + 0.62; // using spec sheet curve
  Serial.print(F("PM < 10: "));
  Serial.println(concentration10, 3);
}

float getPM2_5() {
  return concentration2_5;
}

float getPM10() {
  return concentration10;
}

