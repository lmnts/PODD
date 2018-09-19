
/*
 * pod_sensors.cpp
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 * 
 * Handle setup and reading from various sensors.
 */

#include "pod_sensors.h"

//#include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#include <AsyncDelay.h>
#include <ClosedCube_OPT3001.h>
#include "cozir.h"
#include <Wire.h>
#include <HIH61xx.h>

// SOUND
unsigned int knock;
#define sampletime_DB 5000

// Light
#define OPT3001_ADDR 0x45
ClosedCube_OPT3001 opt3001;

// CO2
// Note Rx/Tx labeled for Teensy side of serial
// (reverse of Rx/Tx label on CO2 sensor)
#define CO2_PIN_RX PIN_B6
#define CO2_PIN_TX PIN_B5
// WARNING: Software serial implementations can interfere with
// other serial interfaces as processing routines prevent
// necessary interrupts from occurring in a timely manner.
// Alternative software implementations such as AltSoftSerial
// and NeoSWSerial should work better than the builtin
// SoftwareSerial, but they have their own issues: AltSoftSerial
// requires specific Rx/Tx pins and NeoSWSerial uses one of the
// hardware timers (hopefully nothing else is trying to use it...).
//SoftwareSerial nss(CO2_PIN_RX,CO2_PIN_TX);
NeoSWSerial nss(CO2_PIN_RX,CO2_PIN_TX);
// Note COZIR library modified to remove Serial.begin() call in
// constructor as we do _not_ want the serial interface running
// except when we actually want to communicate with the sensor.
COZIR czr(nss);

// Particulate Matter (PM) Sensor
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

// Particulate Matter (PM) Sensor: SM-PWM-01C
// Pins for ~ 2 um and ~ 10 um dust particle pulses.
// These may be switched from schematics due to ambiguity in sensor
// documentation (reversing header flips only these two pins).
#define PM_PIN_P1 PIN_C6
#define PM_PIN_P2 PIN_C5
// Time scale (ms) over which to generate a moving average of
// the sensor readings.  Set to 0 to use current values only.
#define PM_SAMPLE_WEIGHTING_TIME 0
// Various calculation quantities.
// Volatile necessary when used in both interrupts and main thread.
// UNITS: Sample times in milliseconds, pulse times in microseconds.
//        If data sheet suggestion of ~ 100 ms pulse lengths, then
//        milliseconds might be sufficient for all values.
volatile bool pmSampling = false;  // Indicate if currently running
unsigned long pmLastSampleTime = ((unsigned long)(-1) >> 1); // Last calculation time [ms]
volatile unsigned long pmSampleT0 = 0;  // Sampling start time [ms]
volatile unsigned long pmPulse1T0 = 0;  // Pulse 1 start time [us]
volatile unsigned long pmPulse2T0 = 0;  // Pulse 2 start time [us]
volatile unsigned long pmPulse1TSum = 0;  // Pulse 1 cumulative time (low) [us]
volatile unsigned long pmPulse2TSum = 0;  // Pulse 2 cumulative time (low) [us]
volatile unsigned int pmPulse1N = 0;  // Pulse 1 count
volatile unsigned int pmPulse2N = 0;  // Pulse 2 count
volatile uint8_t pmPulse1State = HIGH;  // Current pulse 1 state
volatile uint8_t pmPulse2State = HIGH;  // Current pulse 2 state
float pmDensity02 = 0.0;  // Density of ~ 2 um dust (ug/m^3)
float pmDensity10 = 0.0;  // Density of ~ 10 um dust (ug/m^3)

// Extra PM parameters for sensor testing
#define PM_TESTING
#ifdef PM_TESTING
volatile uint8_t pmPulseState = B11;  // Bit k: pulse k (1 is high)
volatile unsigned int pmPulseCount[4];
volatile unsigned long pmPulseT0[4];
volatile unsigned long pmPulseTSum[4];
#endif

// CO
//#define numCoRead 4
//int samples[numCoRead];
#define CoSpecSensor A3

// Thermistor and light Pins
#define TempGpin A1
#define lightPin A2

// HIH
//#define HIH_ADDR 0x27  // Hard-coded to this in HIH library
HIH61xx<TwoWire> hih(Wire);


//--------------------------------------------------------------------------------------------- [Sensor Reads]

void sensorSetup() {
  // OPT3001 ambient light sensor
  opt3001.begin(OPT3001_ADDR);
  OPT3001_Config opt3001Config;
  opt3001Config.RangeNumber = B1100;
  opt3001Config.ConvertionTime = B1;  // [sic]
  opt3001Config.ModeOfConversionOperation = B11;
  OPT3001_ErrorCode opt3001Err = opt3001.writeConfig(opt3001Config);
  if (opt3001Err == NO_ERROR) {
    Serial.println(F("OPT3001 configured."));
  } else {
    Serial.print(F("OPT3001 configuration error: "));
    Serial.println(opt3001Err);
  }
  
  // PM Sensor
  pinMode(PM_PIN_2_5, INPUT);
  pinMode(PM_PIN_10, INPUT);
  pinMode(PM_ENABLE, OUTPUT);
  digitalWrite(PM_ENABLE, HIGH);

  analogReference(EXTERNAL);

  hih.initialise();

  // COZIR sensor communicates at 9600 baud
  nss.begin(9600);
  nss.listen();
  czr.SetOperatingMode(CZR_POLLING);
  // Suspend serial interface as this software serial can disrupt
  // other serial interfaces (and we do not need it except when
  // communicating with CO2 sensor).
  nss.ignore();
  //Serial.println("DEBUGGING: Turning off software serial (CO2 interface).");
  //nss.end();
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
  // OPT3001: Range is 0.01 - 80,000 lux with resolution as
  // small as 0.01 lux.  Note with current configuration, it
  // may take several seconds for readings to stabilize if
  // the lighting condition changes drastically and rapidly.
  // That is, don't use this at a rave.
  OPT3001 reading = opt3001.readResult();
  // Return unphysical value if sensor read error
  if (reading.error != NO_ERROR) {
    Serial.print(F("Error reading OPT3001 sensor ("));
    Serial.print(reading.error);
    Serial.println(F(")."));
    return -1.0;
  }
  return reading.lux;
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
  // Only enable software serial during interaction
  nss.listen();
  int c = czr.CO2();
  nss.ignore();
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


/* Initializes the particulate matter sensor, but does not power it up. */
void initPM() {
  pinMode(PM_PIN_P1, INPUT);
  pinMode(PM_PIN_P2, INPUT);
  pinMode(PM_ENABLE, OUTPUT);
  digitalWrite(PM_ENABLE, LOW);
}


/* Power up the particulate matter sensor.
   This should be done at least 90 seconds before sampling is performed:
   the heater element that draws the bulk of the power generates an air
   current through the sensor that takes time to stabilize. */
void startPM() {
  digitalWrite(PM_ENABLE,HIGH);
}


/* Turn off power to the particulate matter sensor.
   If the sensor is currently sampling, any not-yet-processed data
   will be lost. */
void stopPM() {
  if (pmSampling) stopPMSampling();
  digitalWrite(PM_ENABLE,LOW);
}


/* Indicates if the particulate matter sensor is currently taking data. */
bool isPMSampling() {
  return pmSampling;
}


/* Resets parameters used in particulate matter calculations. */
void resetPMSampling() {
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  
  // Set pulse start times to now in case currently in pulse
  // or pulse begins after this routine but before ISR can run.
  unsigned long t0m = millis();
  unsigned long t0u = micros();
  pmSampleT0 = t0m;
  pmPulse1T0 = t0u;
  pmPulse2T0 = t0u;
  pmPulse1TSum = 0;
  pmPulse2TSum = 0;
  pmPulse1N = 0;
  pmPulse2N = 0;
  // If not sampling (ISR not active), ignore any current pulse
  // we are in as we may miss the end of pulse between now and
  // when sampling begins.
  if (pmSampling) {
    pmPulse1State = digitalRead(PM_PIN_P1);
    pmPulse2State = digitalRead(PM_PIN_P2);
  } else {
    pmPulse1State = HIGH;
    pmPulse2State = HIGH;
  }
  
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  pmPulseState = B11;
  for (uint8_t k = 0; k < 4; k++) {
    pmPulseCount[k] = 0;
    pmPulseT0[k] = t0u;
    pmPulseTSum[k] = 0;
  }
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Restore interrupt status
  SREG = oldSREG;
}


/* Process a particulate matter sensor state change (start/end 
   of pulse).  Intended to be run as an ISR on each of the two
   sensor pulse pins. */
void processPMPulseISR() {
  if (!pmSampling) return;
  unsigned long t0u = micros();
  uint8_t state;
  // Process pulse 1
  state = digitalRead(PM_PIN_P1);
  if (state != pmPulse1State) {
    pmPulse1State = state;
    // Start of pulse
    if (state == LOW) {
      pmPulse1T0 = t0u;
    // End of pulse
    } else {
      pmPulse1TSum += (t0u - pmPulse1T0);
      pmPulse1N++;
    }
  }
  // Process pulse 2
  state = digitalRead(PM_PIN_P2);
  if (state != pmPulse2State) {
    pmPulse2State = state;
    // Start of pulse
    if (state == LOW) {
      pmPulse2T0 = t0u;
    // End of pulse
    } else {
      pmPulse2TSum += (t0u - pmPulse2T0);
      pmPulse2N++;
    }
  }
  
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  uint8_t pulseState =   (digitalRead(PM_PIN_P1) == HIGH ? 1 : 0)
                       + (digitalRead(PM_PIN_P2) == HIGH ? 2 : 0);
  if (pulseState != pmPulseState) {
    pmPulseTSum[pmPulseState] += (t0u - pmPulseT0[pmPulseState]);
    pmPulseCount[pmPulseState]++;
    pmPulseT0[pulseState] = t0u;
    pmPulseState = pulseState;
  }
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}


/* Perform particulate matter sensor calculations based upon data 
   collected since last call to processPM() or startPM().  Longer
   collection periods increase accuracy & precision; 10+ seconds
   is strongly suggested, but 30+ seconds is preferrable. */
void processPM() {
  if (!pmSampling) {
    Serial.println("Warning: No new particulate matter data to process.");
    Serial.println("  Using old results.");
    return;
  }
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  
  unsigned long t0m = millis();
  unsigned long t0u = micros();
  
  // Account for any incomplete pulses
  if (pmPulse1State == LOW) {
      pmPulse1TSum += (t0u - pmPulse1T0);
      pmPulse1N++;  // Cannot add 0.5...
  }
  if (pmPulse2State == LOW) {
      pmPulse2TSum += (t0u - pmPulse2T0);
      pmPulse2N++;  // Cannot add 0.5...
  }
  
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  uint8_t pulseState =   (digitalRead(PM_PIN_P1) == HIGH ? 1 : 0)
                       + (digitalRead(PM_PIN_P2) == HIGH ? 2 : 0);
  pmPulseTSum[pmPulseState] += (t0u - pmPulseT0[pmPulseState]);
  pmPulseCount[pmPulseState]++;
  pmPulseT0[pulseState] = t0u;
  pmPulseState = pulseState;
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Copy relevant shared variables to local variables so we can
  // quickly re-enable interrupts.
  unsigned long _pmSampleT0 = pmSampleT0;
  unsigned long _pmPulse1TSum = pmPulse1TSum;
  unsigned long _pmPulse2TSum = pmPulse2TSum;
  unsigned int _pmPulse1N = pmPulse1N;
  unsigned int _pmPulse2N = pmPulse2N;
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  unsigned int _pmPulseCount[4];
  unsigned long _pmPulseTSum[4];
  for (uint8_t k = 0; k < 4; k++) {
    _pmPulseCount[k] = pmPulseCount[k];
    _pmPulseTSum[k]  = pmPulseTSum[k];
  }
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Restore interrupt status.
  // Re-enables interrupts if they were previously active.
  SREG = oldSREG;
  
  // Length of sampling interval [ms]
  unsigned long sampleInterval = t0m - _pmSampleT0;
  
  // Low pulse occupancy: fraction of time spent low.
  // Must correct for different units (us vs ms).
  float inverseSampleInterval = 1 / (1000.0 * sampleInterval);
  float lpo1 = _pmPulse1TSum * inverseSampleInterval;
  float lpo2 = _pmPulse2TSum * inverseSampleInterval;
  
  // This polynomial approximation to the SM-PWM-01C data sheet's
  // LPO vs. Dust Concentration curve is from Dan Tudose's library:
  //   https://github.com/dantudose/SM-PWM-01A
  // Here, lpo is fraction (not percentage).
  //float density = 0.62 + lpo*(5.2e4 + lpo*(3.8e4 + lpo*(1.1e6)))
  
  // This is a fit to the form lpo = 1 - exp(-a*density), a
  // functional form that would be appropriate for occupancy,
  // in certain limits.  Treating the minimum & maximum curves
  // on the data sheet plot as the 1-sigma band (which is likely
  // way overestimating the errors) and performing a weighted
  // least-squares fit gives the following:
  //   a   = 1.04e-4 +/- 1.0e-5
  //   1/a = 9555 +/- 943
  // The data sheet makes no distinction between the LPO's for
  // the two different pulse types; it is unclear if or why
  // the two should have an identical curve, but this is what
  // everyone else seems to do.... (CAVEAT EMPTOR)
  float density02 = -9555*log(1-min(0.99,lpo1));
  float density10 = -9555*log(1-min(0.99,lpo2));

  // Particle densities given as a moving average with an
  // exponential weighting in time.
  if (PM_SAMPLE_WEIGHTING_TIME > 0) {
    float f = exp(-(t0m-pmLastSampleTime)/(float(PM_SAMPLE_WEIGHTING_TIME)));
    pmDensity02 = f*pmDensity02 + (1-f)*density02;
    pmDensity10 = f*pmDensity10 + (1-f)*density10;
  } else {
    pmDensity02 = density02;
    pmDensity10 = density10;
  }
  
  // Reset calculation parameters for next measurement interval
  resetPMSampling();
  pmLastSampleTime = millis();
  
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  char sbuffer[64];
  Serial.println("Particulate Matter Sensor (SM-PWM-01C):");
  sprintf(sbuffer,"%8.2f",(double)pmDensity02);
  Serial.print("  PM2  [ug/m^3]: ");
  Serial.println(sbuffer);
  sprintf(sbuffer,"%8.6f",(double)pmDensity10);
  Serial.print("  PM10 [ug/m^3]: ");
  Serial.println(sbuffer);
  Serial.print("  Pulse 1 LPO, count, frequency[Hz]: ");
  sprintf(sbuffer,"%8.6f %6d %8.3f",
          (double)lpo1,_pmPulse1N,_pmPulse1N/(0.001*sampleInterval));
  Serial.println(sbuffer);
  Serial.print("  Pulse 2 LPO, count, frequency[Hz]: ");
  sprintf(sbuffer,"%8.6f %6d %8.3f",
          (double)lpo2,_pmPulse2N,_pmPulse2N/(0.001*sampleInterval));
  Serial.println(sbuffer);
  Serial.println("  Pulse 1 state, pulse 2 state, LPO, count, frequency[Hz]: ");
  for (uint8_t k = 3; k >= 0; k--) {
    float lpo  = _pmPulseTSum[k] * inverseSampleInterval;
    float freq = _pmPulseCount[k]/(0.001*sampleInterval);
    sprintf(sbuffer,"    %7s %7s %8.6f %6d %8.3f",
            k & 1 ? "inactive" : " active ",
            k & 2 ? "inactive" : " active ",
            (double)lpo,_pmPulseCount[k],(double)freq);
    Serial.println(sbuffer);
  }
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}


/* Start particulate matter sensor sampling.  Ideally, particle 
   meter should be powered for 90+ seconds prior to sampling. */
void startPMSampling() {
  // If already sampling, just reset sampling data
  if (pmSampling) {
    resetPMSampling();
    return;
  }
  // Check if sensor is not powered up
  if (digitalRead(PM_ENABLE) != HIGH) {
    startPM();
    Serial.println("Warning: Particulate matter sensor was not powered on prior to sampling.");
    Serial.println("  Particle meter readings may be inaccurate.");
    delay(10);
  }
  resetPMSampling();
  attachInterrupt(digitalPinToInterrupt(PM_PIN_P1),processPMPulseISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PM_PIN_P2),processPMPulseISR,CHANGE);
  pmSampling = true;
}


/* Stop particulate matter sensor sampling.
   If currently sampling and sampling data has been unprocessed for
   at least updateInterval milliseconds, it will be processed: an
   updateInterval of -1 (the default) means data will not be processed. */
void stopPMSampling(unsigned long updateInterval/*=-1*/) {
  if (!pmSampling) return;
  if ((millis() - pmLastSampleTime) > updateInterval) {
    processPM();
  }
  pmSampling = false;
  detachInterrupt(digitalPinToInterrupt(PM_PIN_P1));
  detachInterrupt(digitalPinToInterrupt(PM_PIN_P2));
}


/* Returns the most recently measured/calculated PM_2 value in ug/m^3.
   PM_2 is a measurement of particulate matter 2 um in diameter or
   smaller.  The sensor does not discriminate between particle sizes
   very well, so this is only an approximation of PM_2.  Could treat
   this as PM_2.5, given the limitations in particle size
   discrimination. */
/*
float getPM2() {
  return pmDensity02;
}
*/


/* Returns the most recently measured/calculated PM_10 value in ug/m^3.
   PM_10 is a measurement of particulate matter 10 um in diameter or
   smaller.  The sensor does not discriminate between particle sizes
   very well, so this is only an approximation of PM_10. */
/*
float getPM10() {
  return pmDensity10;
}
*/



