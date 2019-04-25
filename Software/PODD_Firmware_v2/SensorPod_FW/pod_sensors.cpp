
/*
   pod_sensors.cpp
   2017 - Nick Turner and Morgan Redfield
   Licensed under the AGPLv3. For full license see LICENSE.md
   Copyright (c) 2017 LMN Architects, LLC

   Handle setup and reading from various sensors.
*/

#include "pod_sensors.h"
#include "pod_config.h"

#include <limits.h>

#include <NeoSWSerial.h>
//#include <SoftwareSerial.h>
#include <Wire.h>
#include <AsyncDelay.h>
#include <ClosedCube_OPT3001.h>
//#include "cozir.h"
//#include <HIH61xx.h>
//#include <TimerOne.h>
#include <TimerThree.h>

#ifdef USE_SPS30_PM
#include <sps30.h>
#endif


// ADC / analog measurements
// Analog pin to continuously sample
#define FREE_RUNNING_PIN MIC_PIN
// ...converted to value used in data registers
#define FR_PIN (FREE_RUNNING_PIN - PIN_F0)
// ADC clock prescaling here specific to 8 MHz.
// See 'wiring_private.h' in teensy core files for other speeds.
#ifndef ADC_PRESCALER
#if defined(F_CPU) && (F_CPU != 8000000)
#error "CPU speed must be set to 8 MHz"
#endif
#define ADC_PRESCALER 0x06
#endif
// Various ADC-related data register values:
//   Set analog reference and pin to measure
#define FR_ADMUX (w_analog_reference | (FR_PIN & 0x1F))
//   Set trigger to free-running mode
//   Can also enabled higher speed (but higher power) conversion mode
#define FR_ADCSRB (0x00)
//#define FR_ADCSRB (0x00 | (1 << ADHSM))
//   Enable ADC and set clock prescaling,
//   but do not enable free-running mode
#define DEF_ADCSRA  ((1 << ADEN) | ADC_PRESCALER)
//   Enable ADC, set clock prescaling,
//   set auto-trigger and start ADC conversions
#define FR_ADCSRA ((1 << ADEN) | (1 << ADSC) |(1 << ADATE) | ADC_PRESCALER)
// Flag to indicate if ADC is currently in free-running mode
volatile bool adcFreeRunning = false;

// Light [OPT3001]
#define OPT3001_ADDR 0x45
ClosedCube_OPT3001 opt3001;

// Sound [SparkFun 12758]
//unsigned int knock;
//#define sampletime_DB 5000
// Interval between sound samples in microseconds.
// Sampling occurs through ISR, but rate should be limited to
// only what is necessary as this will impact other ISRs.
#define SOUND_SAMPLE_INTERVAL_US 10000
// Flag to indicate if sound is currently being sampled
volatile bool soundSampling = false;

// Temperature/humidity [HIH8120]
// Address already hard-coded to this in HIH library
#define HIH_ADDR 0x27
// Library works for 8xxx line as well
//HIH61xx<TwoWire> hih(Wire);

// Radiant temperature [PR222J2]
#define RADIANT_TEMP_PIN A1

// CO2 [CozIR-A]
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
//SoftwareSerial CO2_serial(CO2_PIN_RX,CO2_PIN_TX);
NeoSWSerial CO2_serial(CO2_PIN_RX, CO2_PIN_TX);
// Note COZIR library modified to remove Serial.begin() call in
// constructor as we do _not_ want the serial interface running
// except when we actually want to communicate with the sensor.
//COZIR czr(CO2_serial);

// Particulate Matter (PM) Sensor
#ifdef USE_OLD_PM
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
#endif

// Particulate Matter (PM) Sensor: SM-PWM-01C
#ifdef USE_SMPWM01C_PM
// Pins for ~ 2 um and ~ 10 um dust particle pulses.
// NOTE: The PCB schematics have the pin labels reversed;
// pins defined here refer to the sensor P1 & P2 pins, not
// the PCB labels.
#define PM_PIN_P1 PIN_C6
#define PM_PIN_P2 PIN_C5
// Time scale (ms) over which to generate a moving average of
// the sensor readings.  Set to 0 to use current values only.
#define PM_SAMPLE_WEIGHTING_TIME (30*60*1000ul)
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
float pmWeight = 0.0;  // Weighting used for moving average

// Extra PM parameters for sensor testing
#ifdef PM_TESTING
volatile uint8_t pmPulseState = B11;  // Bit k: pulse k (1 is high)
volatile unsigned int pmPulseCount[4];
volatile unsigned long pmPulseT0[4];
volatile unsigned long pmPulseTSum[4];
#endif
#endif

// Particulate Matter (PM) Sensor: Sensirion SPS30
#ifdef USE_SPS30_PM
// PM pins and associated JST connector wire colors
// (may be specific to this batch of connector wires):
//   1 (blue):   5V
//   2 (green):  SDA/RX
//   3 (yellow): SCL/TX
//   4 (black):  SEL
//   5 (red):    GND
// The sensor draws ~ 20 mA in idle mode and ~ 60 mA in
// measurement mode.  New measurements are available every
// 1 second, without any built-in warmup period.  However,
// sensor readings seems to take 80-100 seconds to settle
// down once measurement mode is started, so the sensor
// should be run for that long before taking data for
// accuracy purposes.
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// NOTE: The PODD board's voltage level translators are one-way
//   and cannot be used for either serial or I2C communication.
//   However, I2C relies on pull-up resistors to achieve the
//   high level and is LVTTL 3.3V compatible, so the SPS30
//   I2C lines be connected to the existing 3.3V I2C bus
//   (PODD PCB has 2.2 kOhm pull-up resistors already).
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Note Rx/Tx labeled for Teensy side of serial
// (reverse of Rx/Tx label on PM sensor).
// 5-pin connected oriented on PCB such that disconnected SEL line
// is on bottom, where orientations is such tat teensy-side of PCB
// is "top".
#define PM_PIN_RX PIN_C6
#define PM_PIN_TX PIN_C5
// SPS30 object.
// Uses default I2C (Wire) interface.  Note the 32-byte buffer
// used by this class for teensy boards is insufficient to hold
// all the data returned by the sensor: the mass densities can
// be retrieved, but the number densities will not (set to zer0).
SPS30 sps30;
bool pmPowered = false;
bool pmRunning = false;
sps_values pmData;
#endif

// CO
//#define numCoRead 4
//int samples[numCoRead];
#define CoSpecSensor A3

// Thermistor and light Pins
#define TempGpin A1
//#define lightPin A2


//--------------------------------------------------------------------------------------------- [Sensor Reads]

void sensorSetup() {
  // ADC initialization:
  // Using low-level ADC access rather than higher-level
  // Arduino analog routines for speed.  That means
  // readAnalog() must be used instead of analogRead()!
  //analogReference(EXTERNAL);
  initADC();
  
  // Light sensor
  initLightSensor();

  // Sound sensor
  initSoundSensor();
  
  // Temperature/humidity sensor
  initTemperatureSensor();

  // Globe/radiant temperature sensor
  initGlobeTemperatureSensor();
  
  // CO2 sensor
  initCO2Sensor();
  
  // PM Sensor
  // Power to sensor initially turned off: must call
  // appropriate routines to power up and start PM sensor.
  initPMSensor();
}

bool verifySensors() {
  // gets data from all sensors
  // returns false if a sensors data is out of the valid range (indicating sensor failure)
  float tval_f;
  double tval_d;
  int tval_i;

  //tval_f = getRHTemp();
  retrieveTemperatureData();
  tval_f = getTemperature();
  if (tval_f < MIN_RH_TEMP || tval_f > MAX_RX_TEMP) {
    Serial.println(F("Humidity T Failure: ") + String(tval_f));
    return false;
  }

  //tval_f = getRHHum();
  retrieveTemperatureData();
  tval_f = getRelHumidity();
  if (tval_f < MIN_RH_HUM || tval_f > MAX_RX_HUM) {
    Serial.println(F("Humidity RH Failure: ") + String(tval_f));
    return false;
  }

  tval_f = getLight();
  if (tval_f < MIN_LIGHT || tval_f > MAX_LIGHT) {
    Serial.println(F("Light Failure: ") + String(tval_f));
    return false;
  }
  
  /*
  tval_d = getGlobeTemp();
  if (tval_d < MIN_GLOBE_TEMP || tval_d > MAX_GLOBE_TEMP) {
    Serial.println(F("Temp Failure: ") + String(tval_d));
    return false;
  }
  */
  tval_f = getGlobeTemperature();
  if (tval_f < MIN_GLOBE_TEMP || tval_f > MAX_GLOBE_TEMP) {
    Serial.println(F("Temp Failure: ") + String(tval_f));
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


// ADC / Analog Measurements -------------------------------------------

/* To allow for rapid ADC measurements of the microphone without
   frequent CPU tie-ups (a standard ADC conversion takes ~ 0.1 ms,
   too long for an ISR) that might interfere with network/xbee
   communications, low-level ADC calls are used in place of the
   high-level Arduino routines.  This allows the ADC to be placed
   in free-running (continuously measuring) mode, though we must
   temporarily suspend that mode to take analog measurements on
   other (non-microphone) pins. */

// See pins_teensy.c for use of low-level ADC access on
// AT90USB1286 microcontroller.

/* Initializes ADC. */
void initADC() {
  // PODD has an external voltage reference (3.3V power line)
  analogReference(EXTERNAL);
  // Turn on ADC
  ADCSRA |= (1 << ADEN);
}


/* Places the ADC in free-running mode, taking continuous measurements
   of a specific analog pin (intended for microphone pin). */
void startADCFreeRunning() {
  if (adcFreeRunning) return;

  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts

  // Disable digital input on pin
  DIDR0 |= (1 << FR_PIN);  // achieved with pinMode()?

  // Set ADC-related data registers:
  // * set analog reference and pin to measure
  //ADMUX = w_analog_reference | (FR_PIN & 0x1F);
  ADMUX = FR_ADMUX;
  // * set trigger to free-running mode
  //   can also enabled higher speed (but higher power) conversion mode
  //ADCSRB = 0x00;  // | (1 << ADHSM)
  ADCSRB = FR_ADCSRB;
  // * enable ADC and set clock prescaling
  //ADCSRA = (1 << ADEN) | ADC_PRESCALER;
  // * set auto-trigger and start ADC conversions
  //ADCSRA |= (1 << ADSC) | (1 << ADATE);
  // * enable ADC, set auto-trigger, set clock prescaling,
  //   and start ADC conversions
  ADCSRA = FR_ADCSRA;
  
  adcFreeRunning = true;
  
  // Restore interrupt status.
  // Re-enables interrupts if they were previously active.
  SREG = oldSREG;

  // Give chance for ADC to settle
  //delay(1);
}


/* Indicates if the ADC is currently in free-running (continuously
   sampling) mode. */
bool isADCFreeRunning() {
  return adcFreeRunning;
}


/* Stops the ADC from continously taking measurements. */
void stopADCFreeRunning() {
  if (!adcFreeRunning) return;
  adcFreeRunning = false;
  
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  
  // Enable ADC and set clock prescaling,
  // but omit auto-trigger flag
  //ADCSRA = (1 << ADEN) | ADC_PRESCALER;
  ADCSRA = DEF_ADCSRA;

  // Restore interrupt status.
  // Re-enables interrupts if they were previously active.
  SREG = oldSREG;
}


/* Takes an analog measurement of the given pin.  If the ADC is in
   free-running mode, it will stop, read the new pin, and then resume
   free-running on the original pin. */
int readAnalog(uint8_t pin) {
  bool wasrunning = adcFreeRunning;
  if (adcFreeRunning) stopADCFreeRunning();
  // analogRead will wait for ADC conversion
  int v = analogRead(pin);
  if (wasrunning) startADCFreeRunning();
  return v;
}


/* Returns the most recent ADC measurement in free-running mode.
   If not in free-running mode or a new measurement is not
   available since the last read, returns -1.  Measurements are
   restricted to the fixed free-running pin (intended to be the
   microphone pin).  This routine should be safe to call from
   an ISR. */
int readAnalogFast() {
  if (!adcFreeRunning) return -1;
  // Check if data registers are correct. If not, reset data registers.
  // This may occur if someone calls analogRead() instead of
  // readAnalog().
  const uint8_t ADCSRA_MASK = (1 << ADEN) | (1 << ADATE) | 0x08;
  if ((ADMUX != FR_ADMUX) || ((ADCSRA & ADCSRA_MASK) != (FR_ADCSRA  & ADCSRA_MASK))) {
    adcFreeRunning = false;  // otherwise next line will do nothing
    startADCFreeRunning();
    return -1;
  }
  // We assume there is at most one ISR calling this routine,
  // in which case we do not need to disable interrupts.
  // Check if in middle of conversion (always true for free-running?).
  //if (ADCSRA & (1 << ADSC)) return -1;
  // Check if interrupt (conversion complete) flag set.
  // If not set, there is no _new_ analog data to read.
  if (!(ADCSRA & (1 << ADIF))) return -1;
  // Read of low field locks results until high field read.
  // Read of high field will clear data available flag?
  uint8_t low = ADCL;
  int v = (ADCH << 8) | low;
  // Clear interrupt flag
  ADCSRA |= (1 << ADIF);
  return v;
}


// Sensor Values -------------------------------------------------------

/* TODO: Potential for optimization of HIH Temp+RH sensor here.
   A single hih.read() updates both values and a timer can be
   used to signal when to collect the reading after initiating
   the measurement, rather than just sitting here waiting. */

/*
float getRHTemp() {
  hih.read();  // Blocks for ~45ms as sensor is read
  return hih.getAmbientTemp() / 100.0; // hih gives temp x 100
}
*/

/*
float getRHHum() {
  hih.read();  // Blocks for ~45ms as sensor is read
  return hih.getRelHumidity() / 100.0; // hih gives RH[%] x 100
}
*/

/*
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
*/

/*
float getGlobeTemp() {
  float tempV = analogRead(TempGpin);
  int R = 10000; //actual resistor value
  double Rt, logRt, T;
  // Floats from U.S Sensor Corp. Curve J sheet
  const float A = 0.00147530413409933;
  const float B = 0.000236552076866679;
  const float C = 0.000000118857119853526;
  const float D = -0.000000000074635312369958;
  Rt = R * (1024.0 / tempV - 1.0);
  logRt = log(Rt);
  T = ((1.0 / (A + (B * logRt) + (C * (logRt * logRt * logRt)) + (D * (logRt * logRt * logRt * logRt * logRt)))) - 273.15) * 1.8 + 32;
  return T;
}
*/

/*
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
*/

/*
int getCO2() {
  // Only enable software serial during interaction
  // SoftwareSerial must restart interface and listen,
  // NeoSWSerial need only listen.
  CO2_serial.begin(9600);
  CO2_serial.listen();
  int c = czr.CO2();
  // NeoSWSerial can toggle serial interface with just listen/ignore:
  CO2_serial.ignore();
  //Serial.println("DEBUGGING: Turning off software serial (CO2 interface).");
  //CO2_serial.end();
  // SoftwareSerial has no ignore and must be turned off...
  //CO2_serial.end();
  return c;
}
*/

float getCO() {
  //return analogRead(CoSpecSensor); // TODO: Conversion!
  return readAnalog(CoSpecSensor); // TODO: Conversion!
}


// Light Sensor [OPT3001] ----------------------------------------------

/* Initializes the OPT3001 ambient light sensor. */
void initLightSensor() {
  opt3001.begin(OPT3001_ADDR);
  OPT3001_Config opt3001Config;
  opt3001Config.RangeNumber = B1100;
  opt3001Config.ConvertionTime = B1;  // [sic]
  opt3001Config.ModeOfConversionOperation = B11;
  OPT3001_ErrorCode opt3001Err = opt3001.writeConfig(opt3001Config);
  (void)opt3001Err;  // suppress unused variable warning
  //if (opt3001Err == NO_ERROR) {
  //  Serial.println(F("OPT3001 configured."));
  //} else {
  //  Serial.print(F("OPT3001 configuration error: "));
  //  Serial.println(opt3001Err);
  //}
}


/* Gets the ambient light level in lux.  Returns NAN if there
   is a problem reading the sensor. */
float getLight() {
  // OPT3001: Range is 0.01 - 80,000 lux with resolution as
  // small as 0.01 lux.  Note with current configuration, it
  // may take several seconds for readings to stabilize if
  // the lighting condition changes drastically and rapidly.
  // That is, don't use this at a rave.
  OPT3001 reading = opt3001.readResult();
  // Return unphysical value if sensor read error
  if (reading.error != NO_ERROR) {
    //Serial.print(F("Error reading OPT3001 sensor ("));
    //Serial.print(reading.error);
    //Serial.println(F(")."));
    return NAN;
  }
  return reading.lux;
}


/* Tests communication with the ambient light sensor. */
bool probeLightSensor() {
  // Check by trying to get a measurement value
  OPT3001 reading = opt3001.readResult();
  return (reading.error == NO_ERROR);
}


// Sound Sensor --------------------------------------------------------

// Routines below are for analog readings of a simple microphone.
// The one used in the PODDs is the SparkFun 12758: an electret
// microphone with a x60 amplifier.

// Sound levels are based upon the standard deviation of many samples
// of microphone output.  Conversion to dB(Z) (no frequency weighting)
// is for this particular microphone and setup.


/* Sound data structure. */
struct SoundData {
  unsigned long tstart;
  uint32_t N;
  int32_t sum;
  uint64_t sum2;  // 32-bit can overflow with many samples
  //int32_t sum2;  // 32-bit can overflow with many samples
  int min0,max0;
  void reset() {tstart=millis(); N=0; sum=0; sum2=0; min0=INT_MAX; max0=INT_MIN;}
  void add(int v) {
    // quick, integer-type operations only: may be called from ISR
    N++; sum+=v; int32_t v0=v; sum2+=(v0*v0);  // v^2 can overflow int
    if(v<min0) min0=v; if(v>max0) max0=v;
  }
  float ave() const {return (N > 0) ? ((float)sum)/N : 0;}
  float rms2() const {return (N > 0) ? ((float)sum2)/N : 0;}
  float rms() const {return (N > 0) ? sqrt(rms2()) : 0;}
  float sd() const {
    if (N == 0) return 0;
    float a=ave(); float a2=a*a; float r2 = rms2();
    return (r2 > a2) ? sqrt(r2-a2) : 0;
  }
  // min/max are macros (cannot use as member name...)
  float smin() const {return min0;}
  float smax() const {return max0;}
};
// Problems with functions in volatile struct...
//volatile SoundData soundData;
SoundData soundData;


/* Initializes the sound sensor (microphone) and associated data
   structures. */
void initSoundSensor() {
  pinMode(MIC_PIN,INPUT);
  //soundSampling = false;
  stopSoundSampling();
  //resetSoundData();
  soundData.reset();
}


/* Gets the average-fluctuation-based sound level since the last call 
   to this routine (or since sampling started).  Returns -1 if not
   currently sampling or no samples have been taken since last call. */
float getSound() {
  if (!soundSampling) return -1;
  
  // Copy sound data into local variable to avoid ISR modifying
  // working copy.  Reset global structure to start a new
  // sampling period.  Temporarily disable interrupts to
  // prevent ISRs from modifying data structure while we make
  // the copy.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  SoundData sd = soundData;
  soundData.reset();
  SREG = oldSREG;

  // Return invalid value if no samples were taken
  if (sd.N == 0) return -1;

  // Standard deviation of ADC samples (no dB(Z) conversion).
  // Use of s.d. limits clipping-issues and provides for a better
  // time-average metric than max-min does.
  return sd.sd();

  // TODO: conversion to decibels (Z-weighted -> dBz)
}


/* Starts sampling sound in the background.
   Enables a timer-based ISR that continuously samples the microphone
   level and accumulates data until the sound level is read.
   Note this adds to the CPU workload and may interfere with other ISRs,
   though we strive for this ISR to be fast enough not to cause an issue. */
void startSoundSampling() {
  if (soundSampling) return;
  
  soundData.reset();
  
  // Put ADC in continuously-sampling mode
  startADCFreeRunning();
  
  // Begin timer and attach interrupt service routine (required order?)
  Timer3.initialize(SOUND_SAMPLE_INTERVAL_US);
  Timer3.attachInterrupt(sampleSoundISR);
  
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  soundSampling = true;
  SREG = oldSREG;
  
}


/* Stops sampling sound. */
void stopSoundSampling() {
  if (!soundSampling) return;

  // Stop timer, remove ISR
  Timer3.stop();
  Timer3.detachInterrupt();
  
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  soundSampling = false;
  SREG = oldSREG;
  
  // Halt ADC's continuously-sampling mode
  stopADCFreeRunning();
  
}


/* Indicates if sound sampling is currently occurring in the background. */
bool isSoundSampling() {
  return soundSampling;
}


/* Takes a sound sample.  Intended to be run as a timer-based ISR. */
void sampleSoundISR() {
  if (!soundSampling) return;
  // Take most recent measurement from continuously-sampling ADC.
  // Will return -1 if no new measurement available or if ADC not
  // in continuously-sampling mode.
  int v = readAnalogFast();
  if (v < 0) return;
  soundData.add(v);
}


/* Resets accumulated sound data for a new round of sound sampling.
   ISR-safe. */
void resetSoundData() {
  // Disable interrupts to prevent ISRs from changing values.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  soundData.reset();
  SREG = oldSREG;
}


/* Sound level testing routine.
   Will sample the sensor for the given number of sample periods.
   Sample period in milliseconds. */
void testSoundSensor(unsigned int cycles, unsigned long sampleInterval) {
  bool wasSampling = isSoundSampling();
  Serial.println();
  Serial.println(F("Sound level testing"));
  Serial.println(F("-------------------"));
  Serial.println();
  if (!wasSampling) {
    Serial.println(F("Initializing sound sensor...."));
    initADC();
    initSoundSensor();
    Serial.println(F("Starting sound sampling...."));
    startSoundSampling();
  } else {
    // Start a new set of samples
    Serial.println(F("Starting sound sampling...."));
    resetSoundData();
  }
  unsigned long t0 = millis();
  
  // Arduino implementation of printf drops %f support to reduce
  // memory usage.  We use dsostrf instead.
  char hbuffer1[128],hbuffer2[128];
  char sbuffer[128];
  char fbuffers[3][16];

  // Table header
  //Serial.println();
  sprintf(hbuffer1," %9s %8s %6s %8s %8s %8s %6s %6s %6s",
          " time[ms]","interval",
          "     N"," average","   rms  ","   sd   ","   min","   max","  diff");
  //Serial.println(hbuffer1);
  sprintf(hbuffer2," %9s %8s %6s %8s %8s %8s %6s %6s %6s",
          " --------","--------",
          " -----"," -------"," -------"," -------","   ---","   ---","  ----");
  //Serial.println(hbuffer2);

  for (unsigned int k = 0; k < cycles; k++) {
    //delay(sampleInterval);
    // Break out of the testing loop if the user sends anything
    // over the serial interface.
    if (getSerialChar(sampleInterval) != (char)(-1)) break;

    // Repeat header every so often
    if (k % 50 == 0) {
      Serial.println();
      Serial.println(hbuffer1);
      Serial.println(hbuffer2);
    }

    // Copy sound data into local variable to avoid ISR modifying
    // working copy.  Reset global structure to start a new
    // sampling period.  Temporarily disable interrupts to
    // prevent ISRs from modifying data structure while we make
    // the copy.
    uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
    cli();  // Disable interrupts
    SoundData sd = soundData;
    soundData.reset();
    SREG = oldSREG;

    unsigned long t = millis() - t0;
    unsigned long dt = millis() - sd.tstart;
    dtostrf(sd.ave(),8,3,fbuffers[0]);
    dtostrf(sd.rms(),8,3,fbuffers[1]);
    dtostrf(sd.sd(),8,3,fbuffers[2]);
    sprintf(sbuffer," %9ld %8ld %6ld %8s %8s %8s %6d %6d %6d",
            t,dt,sd.N,fbuffers[0],fbuffers[1],fbuffers[2],sd.min0,sd.max0,sd.max0-sd.min0);
    Serial.println(sbuffer);
  }
  
  Serial.println();
  if (!wasSampling) {
    Serial.println(F("Stopping sound sampling...."));
    stopSoundSampling();
  }
  Serial.println(F("Sound sensor testing is complete."));
  Serial.println();
}


// Temperature/Humidity Sensor [HIH8120] -------------------------------

/* Temperature/humidity data structure.
   Uses NAN when data is invalid. */
struct TempRHData {
  float _T;   // [C]
  float _RH;  // [%]
  void reset() {_T = NAN; _RH = NAN;}
  float Tc() {return _T;}
  float Tf() {return (isnan(_T)) ? NAN : 1.8*_T + 32;}
  float RH() {return _RH;}
};
TempRHData temperatureData;


/* Initializes the HIH8120 temperature/humidity sensor and associated
   data structures. */
void initTemperatureSensor() {
  //hih.initialise();
  temperatureData.reset();
}


/* Retrieves measurements from the temperature/humidity sensor.
   Returns true on success.  Actual data values can be accessed
   through below routines.  Takes ~40ms for sensor to perform
   conversion and return data. */
bool retrieveTemperatureData() {
  temperatureData.reset();
  // Sending an (empty) write command triggers a
  // sensor measurement
  Wire.beginTransmission(HIH_ADDR);
  if (Wire.endTransmission(HIH_ADDR) != 0) return false;

  // I2C data encoded in four bytes
  // See Honeywell's technical note on I2C communications with HumidIcon
  // sensors for a description.
  const size_t BUFF_LEN = 4;
  uint8_t buff[BUFF_LEN];
  
  // Typical measurement conversion time is ~ 37 ms.
  // Will repeatedly poll for data until we get new results
  // or we timeout (100 ms).
  unsigned long t0 = millis();
  delay(35);  // appears to be sufficient most of the time
  while (true) {
    size_t n = Wire.requestFrom(HIH_ADDR,BUFF_LEN);
    // Communication failed
    if (n != BUFF_LEN) return false;
    // Pull data from I2C buffer
    for (size_t k = 0; k < n; k++) buff[k] = Wire.read();
    // Check if returned data contains the new measurement
    // (two highest bits of first byte are zero)
    if ((buff[0] >> 6) == 0) {
      uint16_t rhraw = ((uint16_t)(buff[0] & 0x3F) << 8) | (uint16_t)buff[1];
      uint16_t traw = ((uint16_t)buff[2] << 6) | ((uint16_t)buff[3] >> 2);
      const float A = (1 / (float)16382);
      temperatureData._RH = 100 * A * rhraw;
      temperatureData._T  = 165 * A * traw - 40;
      return true;
    }
    // Timed-out
    if (millis() - t0 > 100) return false;
    delay(10);
  } 
  // Should not reach here...
  return false;
}


/* Returns the most recently retrieved temperature measurement in
   Farenheit (measurements can be retrieved using
   retrieveTemperatureData()).  Returns NAN if measurement
   failed/invalid. */
float getTemperature() {
  return temperatureData.Tf();
}


/* Returns the most recently retrieved relative humidity measurement 
   in percent (measurements can be retrieved using
   retrieveTemperatureData()).  Returns NAN if measurement
   failed/invalid. */
float getRelHumidity() {
  return temperatureData.RH();
}


/* Tests communication with the temperature/humidity sensor. */
bool probeTemperatureSensor() {
  // Test communication by requesting data from sensor.
  // Note writing to sensor triggers a conversion and would
  // tie up the sensor.
  const size_t BUFF_LEN = 2;
  //uint8_t buff[BUFF_LEN];
  size_t n = Wire.requestFrom(HIH_ADDR,BUFF_LEN);
  // Clear I2C buffer
  while (Wire.available()) Wire.read();
  // Communication successful if we received as many
  // bytes as requested
  return (n == BUFF_LEN);
}

/* Temperature/humidity testing routine.
   Will sample the sensor for the given number cycles.
   Sample period in milliseconds. */
void testTemperatureSensor(unsigned int cycles, unsigned long sampleInterval) {
  Serial.println();
  Serial.println(F("Temperature/humidity testing"));
  Serial.println(F("----------------------------"));
  Serial.println();
  Serial.println(F("Initializing temperature/humidity sensor...."));
  initTemperatureSensor();
  Serial.println(F("Starting temperature/humidity measurements...."));
  unsigned long t0 = millis();
  
  // Arduino implementation of printf drops %f support to reduce
  // memory usage.  We use dsostrf instead.
  char hbuffer1[128],hbuffer2[128];
  char sbuffer[128];
  char fbuffers[2][16];

  // Table header
  //Serial.println();
  sprintf(hbuffer1," %9s  %7s  %7s",
          " time[ms]","  T[F] ","  RH[%]");
  //Serial.println(hbuffer1);
  sprintf(hbuffer2," %9s  %7s  %7s",
          " --------"," ------"," ------");
  //Serial.println(hbuffer2);

  for (unsigned int k = 0; k < cycles; k++) {
    //delay(sampleInterval);
    // Break out of the testing loop if the user sends anything
    // over the serial interface.
    if (getSerialChar(sampleInterval) != (char)(-1)) break;

    // Repeat header every so often
    if (k % 50 == 0) {
      Serial.println();
      Serial.println(hbuffer1);
      Serial.println(hbuffer2);
    }

    unsigned long t = millis() - t0;
    if (retrieveTemperatureData()) {
      float T = getTemperature();
      dtostrf(T,7,2,fbuffers[0]);
      float RH = getRelHumidity();
      dtostrf(RH,7,2,fbuffers[1]);
    } else {
      strcpy(fbuffers[0],"   --- ");
      strcpy(fbuffers[1],"   --- ");
    }
    sprintf(sbuffer," %9ld  %7s  %7s",
            t,fbuffers[0],fbuffers[1]);
    Serial.println(sbuffer);
  }
  
  Serial.println();
  Serial.println(F("Temperature/humidity sensor testing is complete."));
  Serial.println();
}



// Radiant Temperature Sensor [PR222J2] --------------------------------

// In order to measure radiant temperature, a temperature sensor is
// placed inside a dark sphere.  The intent is for the air within the
// sphere to equalize with the incident radiation on the sphere, but
// in practice, there will be some heat transfer with the outside air,
// more so if there is an air current.  The "globe" temperature is
// thus going to be somewhere between the radiant temperature and
// ambient air temperature, but should at least give an indication if
// the two differ significantly.

/* Initializes the temperature sensor placed inside a dark sphere;
   intended for radiant temperature measurements. */
void initGlobeTemperatureSensor() {
  // nothing to do
}


/* Returns the current temperature within the sphere (globe) in 
   Farenheit.  Returns NAN if the sensor cannot be read or the
   data is invalid (e.g. missing sensor). */
float getGlobeTemperature() {
  // Voltage across R in GND-R-Rt-3.3V voltage divider.
  // Voltage in units of analog resolution (units will cancel).
  // Note use of readAnalog() instead of analogRead().
  float V = readAnalog(RADIANT_TEMP_PIN);
  // If V is very small, either the thermistor is not connected
  // or we are at the South Pole on a cold day
  if (V < 10) return NAN;
  // Fixed resistor value
  const int R = 10000; //actual resistor value
  // Invert voltage divider to get thermistor's resistance
  float Rt = R * (1024.0 / V - 1.0);

  // Find temperature corresponding to above resistance
  // Floats from U.S Sensor Corp. Curve J sheet
  const float A = 0.00147530413409933;
  const float B = 0.000236552076866679;
  const float C = 0.000000118857119853526;
  const float D = -0.000000000074635312369958;
  float logRt = log(Rt);
  float logRt2 = logRt * logRt;
  // Inverse of temperature in Kelvin
  float Tkinv = A + logRt * (B + logRt2 * (C + logRt2 * (D)));

  // Return in Kelvin
  //return 1/Tkinv;
  // Return in Celcius
  //return 1/Tkinv - 273.15;
  // Return in Farenheit
  return 1.8 * (1/Tkinv - 273.15) + 32;
}


/* Tests validity of the globe temperature sensor. */
bool probeGlobeTemperatureSensor() {
  // Make measurement of voltage across fixed resistor R in
  // GND-R-Rt-3.3V voltage divider.  If voltage is nearly
  // zero, the thermistor is likely not connected (effectively
  // infinite resistance) and any readings are invalid.
  // Note use of readAnalog() instead of analogRead().
  return (readAnalog(RADIANT_TEMP_PIN) >= 10);
}



// CO2 Sensor [CozIR-A] ------------------------------------------------

// Good reference for Arduino serial interface with CozIR CO2 sensor:
//   https://github.com/roder/cozir


/* Initializes the CO2 sensor. */
void initCO2Sensor() {
  // COZIR sensor communicates at 9600 baud
  CO2_serial.begin(9600);
  // Only enable serial interface while using it
  enableCO2Serial();
  // Set operating mode to polling
  cozirSendCommand('K',2);
  //czr.SetOperatingMode(CZR_POLLING);
  // Set digital filter to 32: measurements are moving average of
  // previous NN measurements, which are taken at 2 Hz.
  cozirSendCommand('A',32);
  disableCO2Serial();
}


/* Gets the current CO2 level, in ppm.  Returns -1 if something failed. */
int getCO2() {
  // Only enable serial interface while using it
  enableCO2Serial();
  //int v = czr.CO2();
  int v = cozirGetValue('Z');
  disableCO2Serial();
  return v;
}


/* Sets the current CO2 level, in ppm.  Use to calibrate sensor. */
void setCO2(int ppm) {
  // Ignore invalid values
  if (ppm <= 0) return;
  if (ppm > 10000) return;
  // Only enable serial interface while using it
  enableCO2Serial();
  //czr.CalibrateKnownGas((uint16_t)ppm);
  cozirSendCommand('X',ppm);
  disableCO2Serial();
}


/* Tests communication with the CO2 sensor. */
bool probeCO2() {
  // Only enable serial interface while using it
  enableCO2Serial();
  bool b = cozirSendCommand('a');  // arbitrary info command
  disableCO2Serial();
  return b;
}


/* Enable serial interface with CO2 sensor. */
void enableCO2Serial() {
  // SoftwareSerial must restart interface and listen,
  // NeoSWSerial need only listen.
  //CO2_serial.begin(9600);
  CO2_serial.listen();
}


/* Disable serial interface with CO2 sensor. */
void disableCO2Serial() {
  // NeoSWSerial can toggle serial interface with just listen/ignore:
  CO2_serial.ignore();
  // SoftwareSerial has no ignore and must be turned off...
  //CO2_serial.end();
}


/* Converts single character command and, optionally, an integer value
   to a valid CozIR CO2 sensor command string.  If integer is negative,
   it will be omitted. */
String cozirCommandString(char c, int v) {
  if (v < 0) {
    return String(c);
  }
  char buff[10];
  // Require non-negative integers of limited range
  uint16_t v0 = (uint16_t)(v > 65535 ? 65535 : v);
  sprintf(buff,"%c %u",c,v0);
  return String(buff);
}


// TODO: Have commands read start of response to check for success
//  (CozIR always sends response starting with sent command character).

/* Sends single character command and, optionally, an integer value to 
   the CozIR CO2 sensor over the serial interface.  If integer is
   negative, it will be omitted.  Returns true if communication was
   successful (sensor returned command character).  Note the serial
   buffer will be left with whatever the sensor sends after the
   command character. */
bool cozirSendCommand(char c, int v) {
  //cozirSendCommand(cozirCommandString(c,v));
  // Clear incoming serial buffer first
  while(CO2_serial.available()) CO2_serial.read();
  // Send command
  String s = cozirCommandString(c,v);
  CO2_serial.print(s);
  CO2_serial.print("\r\n");
  // Wait a limited time for response
  const int TIMEOUT_MS = 20;
  for (int k = 0; k < TIMEOUT_MS; k++) {
    if (CO2_serial.available()) {
      // First non-space character should be same as command
      // character sent.
      char c0 = CO2_serial.read();
      if (c0 == ' ') continue;
      return (c0 == c);
      // Note post-command-character data received from sensor
      // remains in the serial buffer.
    }
    delay(1);
  }
  // Timed out
  return false;
}


/* Returns the (first) integer value provided by the CozIR CO2 sensor after
   sending the given command.  Returns -1 if something failed. */
int cozirGetValue(char c, int v) {
  // Send command
  if (!cozirSendCommand(c,v)) return -1;
  
  // Retrieve response
  // Note cozirSendCommand already stripped off command character
  // from serial response.
  // This buffer is not large enough for commands that return
  // multiple data fields.
  const size_t BUFF_LEN = 12;
  size_t n = 0;
  char buff[BUFF_LEN];
  // Continue reading until no new input available for 2ms
  // (in case serial data still arriving).
  // Double while loops here are not redundant...
  delay(2);
  while (CO2_serial.available()) {
    while (CO2_serial.available()) {
      if (n < BUFF_LEN-1) {
        buff[n] = CO2_serial.read();
        n++;
      }
    }
    delay(2);
  }
  buff[n] = '\0';

  // Parse response
  // Should begin with a space...
  if (n < 2) return -1;
  // Standard Arduino routines do not include means to check
  // for invalid input, so we use following instead.
  //const char *buff = s.c_str();
  char *end;
  long l = strtol(buff,&end,10);
  if (end == buff) return -1;
  return (int)l;
}


// OLD
/*
void calibrateCO2() {
  // see documentation for usage
  // requires external sources of known concentration CO2 gas
  // CO2 calibration
  / * Here is a step by step
    uncomment czr.SetOperatingMode(CZR_POLLING);
    Upload Sketch
    comment czr.SetOperatingMode(CZR_POLLING);
    and uncomment czr.CalibrateFreshAir();
    Upload Sketch
    comment czr.CalibrateFreshAir();
    Upload sketch * /
  //czr.SetOperatingMode(CZR_POLLING);
  //czr.SetOperatingMode(CZR_STREAMING);
  //czr.CalibrateFreshAir();
  //czr.SetDigiFilter(32);
}
*/


// Particular Matter Sensor (OLD) --------------------------------------

#ifdef USE_OLD_PM

/* Initializes the particulate matter sensor, but does not power it up. */
void initPMSensor() {
  pinMode(PM_PIN_2_5, INPUT);
  pinMode(PM_PIN_10, INPUT);
  pinMode(PM_ENABLE, OUTPUT);
  digitalWrite(PM_ENABLE, HIGH);
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

#endif


// Particular Matter Sensor [SPS30] ------------------------------------

#ifdef USE_SPS30_PM

/* Initializes the particulate matter sensor, but does not power it up. */
void initPMSensor() {
  //pinMode(PM_PIN_P1,INPUT);
  //pinMode(PM_PIN_P2,INPUT);
  pinMode(PM_ENABLE,OUTPUT);
  // NOTE: PODD PCB only allows signals from PM to teensy and not
  //   in the reverse direction, so serial will not work through
  //   the board's PM lines.
  //sps30.SetSerialPin(PM_PIN_RX, PM_PIN_TX);
  digitalWrite(PM_ENABLE, LOW);
  resetPMData();
}


/* Power up the particulate matter sensor.
   Does not start sampling; should draw lower power in this idle
   state (documentation says < 8 mA, but PODD PCB + SPS30 system
   draws closer to 20 mA). */
void powerOnPMSensor() {
  if (pmPowered) return;
  digitalWrite(PM_ENABLE, HIGH);
  Serial.println(F("Powered on PM sensor."));
  delay(100);
  //sps30.begin(SOFTWARE_SERIAL);
  /*
  if (sps30.begin(SOFTWARE_SERIAL)) {
    Serial.println("Successfully started PM serial interface.");
  } else {
    Serial.println("Could not start PM serial interface.");
  }
  */
  //sps30.begin(I2C_COMMS);
  if (sps30.begin(I2C_COMMS)) {
    Serial.println(F("Successfully started PM sensor I2C interface."));
  } else {
    Serial.println(F("Could not start PM sensor I2C interface."));
  }
  pmPowered = true;
}


/* Power down the particulate matter sensor. */
void powerOffPMSensor() {
  if (!pmPowered) return;
  if (pmRunning) {
    stopPMSensor();
  }
  delay(100);
  digitalWrite(PM_ENABLE, LOW);
  Serial.println(F("Powered off PM sensor."));
  pmPowered = false;
}


/* Indicates if the particulate matter sensor is currently powered on. */
bool isPMSensorPowered() {
  return pmPowered;
}


/* Start taking measurements with the particulate matter sensor.
   Measurements are continually taken every second whether or not
   the sensor is polled.  The sensor takes ~ 5-8 seconds to warm up
   before the first valid data is available [cannot seem to find
   this number in documentation, but find ~ 5 seconds from
   experimentation].  Sensor draws ~ 60 mA while running.
   Note the measurements themselves do not stabilize until 80-120
   seconds after measurement mode is enabled.

   Argument indicates if routine should wait long enough for sensor
   to start up and begin taking measurements. */
void startPMSensor(bool wait) {
  if (pmRunning) return;
  if (!pmPowered) powerOnPMSensor();
  //if (!sps30.start()) return;
  if (sps30.start()) {
    Serial.println(F("Successfully started PM sensor."));
  } else {
    Serial.println(F("Could not start PM sensor."));
    return;
  }
  if (wait) delay(8000);
  pmRunning = true;
}


/* Stop the particulate matter sensor from taking measurements.
   Reduces power usage from ~ 60 mA to < 8 mA. */
void stopPMSensor() {
  if (!pmRunning) return;
  sps30.stop();
  Serial.println(F("Stopped PM sensor."));
}


/* Indicates if the particulate matter sensor is currently taking data. */
bool isPMSensorRunning() {
  return pmRunning;
}


/* Tests communication with the particulate matter sensor. */
bool probePMSensor() {
  if (!pmPowered) return false;
  if (!pmRunning) return false;
  return sps30.probe();
}


/* Cleans the particulate matter sensor by running the fan at high
   speed for 10 seconds.  Sensor must be powered and running.
   Returns boolean indicating if cleaning process occurred.

   Argument indicates if routine should wait long enough for sensor
   cleaning to complete. */
bool cleanPMSensor(bool wait) {
  if (!pmPowered) return false;
  if (!pmRunning) return false;
  if (!sps30.clean()) return false;
  Serial.println(F("Cleaning PM sensor.... (takes 12 seconds)"));
  if (wait) delay(12000);  // Need 10s, or 10s + spinup/down?
  return true;
}


/*  Sets the particulate matter sensor data to invalid values. */
void resetPMData() {
  pmData.MassPM1  = -1;
  pmData.MassPM2  = -1;
  pmData.MassPM4  = -1;
  pmData.MassPM10 = -1;
  pmData.NumPM0   = -1;
  pmData.NumPM1   = -1;
  pmData.NumPM2   = -1;
  pmData.NumPM4   = -1;
  pmData.NumPM10  = -1;
  pmData.PartSize = -1;
}


/* Retrieves measurements from the particulate matter sensor.
   Returns true on success.  Actual data values can be accessed
   through below routines.  Sensor must be powered on and
   running; new data is only available every 1 second
   (retrieval will fail if attempting to too soon after last
   retrieval). */
bool retrievePMData() {
  if (!pmPowered) return false;
  if (!pmRunning) return false;
  if (sps30.GetValues(&pmData) != ERR_OK) {
    resetPMData();
    return false;
  }
  return true;
}


/* Returns the most recently retrieved PM_2.5 measurement in ug/m^3
   (measurements can be retrieved using retrievePMData()).
   PM_2.5 is a measurement of particulate matter 2.5 um in diameter
   or smaller.  Sensor's particle size threshold is ~ 0.3 um.
   Returns -1 if measurement failed/invalid. */
float getPM2_5() {
  return pmData.MassPM2;
}


/* Returns the most recently retrieved PM_10 measurement in ug/m^3
   (measurements can be retrieved using retrievePMData()).
   PM_10 is a measurement of particulate matter 10 um in diameter
   or smaller.  Sensor's particle size threshold is ~ 0.3 um.
   Returns -1 if measurement failed/invalid. */
float getPM10() {
  return pmData.MassPM10;
}


/* Utility function to write dots to serial output over N consecutive
   pause intervals [ms]. */
// Sensor testing >>>>>>>>>>>>>>>>>>>>>>
#ifdef PM_TESTING
void printPMPauseProgress(unsigned int N, unsigned long pause) {
  for (unsigned int k = 0; k < N; k++) {
    delay(pause);
    Serial.print('.');
    Serial.flush();
  }
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/* Particulate matter sensor (Sensirion SPS30) testing routine.
   Will sample the sensor for the given number of cycles after an
   initial warmup period.  Sample and warmup periods in
   milliseconds. */
// Sensor testing >>>>>>>>>>>>>>>>>>>>>>
#ifdef PM_TESTING
void testPMSensor(unsigned int cycles, unsigned long sampleInterval,
                  unsigned long offTime, unsigned long idleTime) {
  Serial.println();
  Serial.println(F("Particulate Matter Sensor (Sensirion SPS30) testing"));
  Serial.println(F("---------------------------------------------------"));
  Serial.println();
  Serial.println(F("Initializing particulate matter sensor...."));
  initPMSensor();
  Serial.print(F("Waiting in unpowered state ("));
  Serial.print(offTime/1000);
  Serial.print(F("s)"));
  printPMPauseProgress(offTime/1000,1000);
  Serial.println();
  Serial.println(F("Powering up particulate matter sensor...."));
  powerOnPMSensor();
  Serial.print(F("Waiting in idle mode ("));
  Serial.print(idleTime/1000);
  Serial.print(F("s)"));
  printPMPauseProgress(idleTime/1000,1000);
  Serial.println();
  
  // Debugging
  /*
  char buf[32];
  int sn_err = sps30.GetSerialNumber(buf,32);
  Serial.print("Serial number error code: ");
  Serial.println(sn_err);
  */
  
  Serial.println(F("Starting measurements...."));
  startPMSensor(false);
  unsigned long t0 = millis();
  
  // Arduino implementation of printf drops %f support to reduce
  // memory usage.  We use dsostrf instead.
  char sbuffer[128];
  char fbuffers[10][8];

  // Table header
  Serial.println();
  sprintf(sbuffer," %8s  %27s  %34s  %8s",
          "time[ms]"," mass concentration [ug/m3]",
          "   number concentration [#/cm3]   "," average");
  Serial.println(sbuffer);
  sprintf(sbuffer," %8s  %6s %6s %6s %6s  %6s %6s %6s %6s %6s  %8s",
          "        "," PM1.0"," PM2.5"," PM4.0","PM10.0",
          "PM0.5"," PM1.0"," PM2.5"," PM4.0","PM10.0","size[um]");
  Serial.println(sbuffer);
  sprintf(sbuffer," %8s  %27s  %34s  %8s",
          "--------","---------------------------",
          "----------------------------------","--------");
  Serial.println(sbuffer);

  for (unsigned int k = 0; k < cycles; k++) {
    //delay(sampleInterval);
    // Break out of the testing loop if the user sends anything
    // over the serial interface.
    if (getSerialChar(sampleInterval) != (char)(-1)) break;
    
    unsigned long t = millis() - t0;
    if (!probePMSensor()) {
      sprintf(sbuffer," %8ld  %73s",
              t,"                       <failed to probe PM sensor>                       ");
      Serial.println(sbuffer);
      continue;
    }
    if (!retrievePMData()) {
      sprintf(sbuffer," %8ld  %73s",
              t,"                    <failed to retrieve sensor data>                     ");
      Serial.println(sbuffer);
      continue;
    }
    dtostrf(pmData.MassPM1,6,2,fbuffers[0]);
    dtostrf(pmData.MassPM2,6,2,fbuffers[1]);
    dtostrf(pmData.MassPM4,6,2,fbuffers[2]);
    dtostrf(pmData.MassPM10,6,2,fbuffers[3]);
    dtostrf(pmData.NumPM0,6,2,fbuffers[4]);
    dtostrf(pmData.NumPM1,6,2,fbuffers[5]);
    dtostrf(pmData.NumPM2,6,2,fbuffers[6]);
    dtostrf(pmData.NumPM4,6,2,fbuffers[7]);
    dtostrf(pmData.NumPM10,6,2,fbuffers[8]);
    dtostrf(pmData.PartSize,8,2,fbuffers[9]);
    sprintf(sbuffer," %8ld  %6s %6s %6s %6s  %6s %6s %6s %6s %6s  %8s",
            t,fbuffers[0],fbuffers[1],fbuffers[2],fbuffers[3],
            fbuffers[4],fbuffers[5],fbuffers[6],fbuffers[7],fbuffers[8],
            fbuffers[9]);
    Serial.println(sbuffer);
  }

  Serial.println();
  Serial.println(F("Stopping measurements...."));
  stopPM();
  Serial.println(F("Powering down particulate matter sensor...."));
  powerOffPM();
  Serial.println(F("Particulate matter sensor testing is complete."));
  Serial.println();
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#endif


// Particular Matter Sensor [SM-PWM-01C] -------------------------------

#ifdef USE_SMPWM01C_PM

/* Initializes the particulate matter sensor, but does not power it up. */
void initPMSensor() {
  pinMode(PM_PIN_P1,INPUT);
  pinMode(PM_PIN_P2,INPUT);
  pinMode(PM_ENABLE,OUTPUT);
  digitalWrite(PM_ENABLE,LOW);
}


/* Power up the particulate matter sensor.
   This should be done at least 90 seconds before sampling is performed:
   the heater element that draws the bulk of the power generates an air
   current through the sensor that takes time to stabilize. */
void startPMSensor() {
  digitalWrite(PM_ENABLE,HIGH);
  sleep(100);
}


/* Turn off power to the particulate matter sensor.
   If the sensor is currently sampling, any not-yet-processed data
   will be lost. */
void stopPMSensor() {
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
  // Use below to avoid "unused variable" compiler warnings
  // (doesn't actually do anything).
  (void)_pmPulse1N;
  (void)_pmPulse2N;
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
  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  // Higher resolution sample interval
  unsigned long pmPulseTSumTotal = 0;
  for (uint8_t k = 0; k < 4; k++) pmPulseTSumTotal += pmPulseTSum[k];
  inverseSampleInterval = 1.0 / pmPulseTSumTotal;
  #endif
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
  // Note: abs() here used only to avoid -0.0 values; the results
  // should be non-negative already.
  float density02 = abs(-9555*log(1-min(0.99,lpo1)));
  float density10 = abs(-9555*log(1-min(0.99,lpo2)));

  // Particle densities given as a moving average with an
  // exponential weighting in time.
  if ((PM_SAMPLE_WEIGHTING_TIME > 0)) {
    float f = exp(-float(t0m-pmLastSampleTime)/(float(PM_SAMPLE_WEIGHTING_TIME)));
    // Biased toward first measurement early on.
    //pmDensity02 = f*pmDensity02 + (1-f)*density02;
    //pmDensity10 = f*pmDensity10 + (1-f)*density10;
    // Use of weights here allows for more equitable use of
    // all early measurements to establish moving average.
    // More weight to measurements with longer sampling periods.
    // In the long run, gives same result as simpler above form.
    float w0   = pmWeight;
    float wtot = f*w0 + sampleInterval;  // Weight by sample time, fade old weights
    pmDensity02 = (f*w0*pmDensity02 + sampleInterval*density02) / wtot;
    pmDensity10 = (f*w0*pmDensity10 + sampleInterval*density10) / wtot;
    pmWeight = wtot;
  } else {
    pmDensity02 = density02;
    pmDensity10 = density10;
  }

  // Reset calculation parameters for next measurement interval
  resetPMSampling();
  pmLastSampleTime = millis();

  // Sensor testing >>>>>>>>>>>>>>>>>>>>
  #ifdef PM_TESTING
  // Arduino implementation of printf drops %f support to reduce
  // memory usage.  We use dsostrf instead.
  char sbuffer[128];
  char fbuffer1[16],fbuffer2[16];
  Serial.println(F("Particulate Matter Sensor (SM-PWM-01C):"));
  if (PM_SAMPLE_WEIGHTING_TIME > 0) {
    dtostrf(pmDensity02,8,3,fbuffer1);
    dtostrf(density02,  7,3,fbuffer2);
    sprintf(sbuffer,"  PM2  [ug/m^3]: %s  [%s]",fbuffer1,fbuffer2);
    Serial.println(sbuffer);
    dtostrf(pmDensity10,8,3,fbuffer1);
    dtostrf(density10,  7,3,fbuffer2);
    sprintf(sbuffer,"  PM10 [ug/m^3]: %s  [%s]",fbuffer1,fbuffer2);
    Serial.println(sbuffer);
  } else {
    dtostrf(pmDensity02,8,3,fbuffer1);
    sprintf(sbuffer,"  PM2  [ug/m^3]: %s",fbuffer1);
    Serial.println(sbuffer);
    dtostrf(pmDensity10,8,3,fbuffer1);
    sprintf(sbuffer,"  PM10 [ug/m^3]: %s",fbuffer1);
    Serial.println(sbuffer);
  }

  unsigned long pulseTAve;
  dtostrf(lpo1,8,6,fbuffer1);
  dtostrf(_pmPulse1N/(0.001*sampleInterval),8,3,fbuffer2);
  pulseTAve = _pmPulse1N > 0 ? _pmPulse1TSum / _pmPulse1N : 0;
  sprintf(sbuffer,"  Pulse 1 LPO, count, Tave[us], freq[Hz]:  %s %6d %8ld %s",
          fbuffer1,_pmPulse1N,pulseTAve,fbuffer2);
  Serial.println(sbuffer);
  dtostrf(lpo2,8,6,fbuffer1);
  dtostrf(_pmPulse2N/(0.001*sampleInterval),8,3,fbuffer2);
  pulseTAve = _pmPulse2N > 0 ? _pmPulse2TSum / _pmPulse2N : 0;
  sprintf(sbuffer,"  Pulse 2 LPO, count, Tave[us], freq[Hz]:  %s %6d %8ld %s",
          fbuffer1,_pmPulse2N,pulseTAve,fbuffer2);
  Serial.println(sbuffer);

  sprintf(sbuffer,"    %8s  %8s  %8s %6s  %8s  %8s  %8s",
          "Pulse 1","Pulse 2","  LPO   "," count","Ttot[us]","Tave[us]","freq[Hz]");
  Serial.println(sbuffer);
  for (int8_t k = 3; k >= 0; k--) {
    float lpo  = _pmPulseTSum[k] * inverseSampleInterval;
    float freq = _pmPulseCount[k]/(0.001*sampleInterval);
    dtostrf(lpo,8,6,fbuffer1);
    dtostrf(freq,8,3,fbuffer2);
    pulseTAve = _pmPulseCount[k] > 0 ? _pmPulseTSum[k] / _pmPulseCount[k] : 0;
    sprintf(sbuffer,"    %8s  %8s  %8s %6d  %8ld  %8ld  %8s",
            k & 1 ? "inactive" : " active ",
            k & 2 ? "inactive" : " active ",
            fbuffer1,_pmPulseCount[k],_pmPulseTSum[k],pulseTAve,fbuffer2);
    Serial.println(sbuffer);
  }

  unsigned long timeTotal = 0;
  for (int8_t k = 3; k >= 0; k--) {
    timeTotal += _pmPulseTSum[k];
  }
  sprintf(sbuffer,"  Total time [us]:  %8ld  (%ld)",timeTotal,sampleInterval*1000);
  Serial.println(sbuffer);
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

  #ifdef __AVR__
  // Cannot use pin-change interrupts on most Teensy++ 2.0 pins
  // and the ones that can be used for interrupts are already
  // in use.  As a workaround, use timer-based, interrupt-driven
  // function calls to explicitly check pin statuses at regular
  // intervals.
  // NOTE: This clashes with the coordinator XBee monitoring
  // routine -- the last one to initialize takes control of the
  // timer and the other will not run!
  Timer1.initialize(1000);
  Timer1.attachInterrupt(processPMPulseISR);
  #else
  attachInterrupt(digitalPinToInterrupt(PM_PIN_P1),processPMPulseISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PM_PIN_P2),processPMPulseISR,CHANGE);
  #endif

  pmSampling = true;
}


/* Stop particulate matter sensor sampling.
   If currently sampling and sampling data has been unprocessed for
   at least updateInterval milliseconds, it will be processed: an
   updateInterval of -1 (the default) means data will not be processed. */
void stopPMSampling(unsigned long updateInterval=-1) {
  if (!pmSampling) return;
  if ((millis() - pmLastSampleTime) > updateInterval) {
    processPM();
  }
  pmSampling = false;

  #ifdef __AVR__
  // Workaround for lack of pin-change interrupts for Teensy++
  // 2.0 pins.
  Timer1.stop();
  Timer1.detachInterrupt();
  #else
  detachInterrupt(digitalPinToInterrupt(PM_PIN_P1));
  detachInterrupt(digitalPinToInterrupt(PM_PIN_P2));
  #endif
}


/* Returns the most recently measured/calculated PM_2 value in ug/m^3.
   PM_2 is a measurement of particulate matter 2 um in diameter or
   smaller.  The sensor does not discriminate between particle sizes
   very well, so this is only an approximation of PM_2.  Could treat
   this as PM_2.5, given the limitations in particle size
   discrimination. */
float getPM2() {
  return pmDensity02;
}


/* Returns the most recently measured/calculated PM_10 value in ug/m^3.
   PM_10 is a measurement of particulate matter 10 um in diameter or
   smaller.  The sensor does not discriminate between particle sizes
   very well, so this is only an approximation of PM_10. */
float getPM10() {
  return pmDensity10;
}


/* Utility function to write dots to serial output over N consecutive
   pause intervals [ms]. */
// Sensor testing >>>>>>>>>>>>>>>>>>>>>>
#ifdef PM_TESTING
void printPMPauseProgress(unsigned int N, unsigned long pause) {
  for (unsigned int k = 0; k < N; k++) {
    delay(pause);
    Serial.print('.');
    Serial.flush();
  }
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/* Particulate matter sensor (SM-PWM-01C) testing routine.
   Will sample the sensor for the given number of cycles after an
   initial warmup period.  Sample and warmup periods in
   milliseconds. */
// Sensor testing >>>>>>>>>>>>>>>>>>>>>>
#ifdef PM_TESTING
void testPMSensor(unsigned int cycles, unsigned long sampleTime,
                  unsigned long warmupTime) {
  Serial.println();
  Serial.println(F("Particulate Matter Sensor (SM-PWM-01C) testing"));
  Serial.println(F("----------------------------------------------"));
  Serial.println();
  Serial.println(F("Initializing particulate matter sensor...."));
  initPM();
  Serial.println(F("Powering up particulate matter sensor...."));
  startPM();
  Serial.print(F("Waiting for sensor to warm up ("));
  Serial.print(warmupTime/1000);
  Serial.print(F("s)"));
  printPMPauseProgress(warmupTime/1000,1000);
  Serial.println();
  Serial.println(F("Starting measurements...."));
  startPMSampling();

  for (unsigned int k = 0; k < cycles; k++) {
    Serial.println();
    Serial.print(F("Sampling ("));
    Serial.print(sampleTime/1000);
    Serial.print(F("s)"));
    printPMPauseProgress(sampleTime/1000,1000);
    Serial.println();
    processPM();

    // If PM_TESTING is defined, processPM() already writes
    // measurement info to serial.
    #if !defined(PM_TESTING)
    // Arduino implementation of printf drops %f support to reduce
    // memory usage.  We use dsostrf instead.
    char sbuffer[16];
    Serial.println(F("Results:"));
    //sprintf(sbuffer,"%8.3f",(double)pmDensity02);
    dtostrf(pmDensity02,8,3,sbuffer);
    Serial.print(F("  PM2  [ug/m^3]: "));
    Serial.println(fbuffer);
    //sprintf(sbuffer,"%8.3f",(double)pmDensity10);
    dtostrf(pmDensity10,8,3,sbuffer);
    Serial.print(F("  PM10 [ug/m^3]: "));
    Serial.println(fbuffer);
    #endif
  }

  Serial.println();
  Serial.println(F("Stopping measurements...."));
  stopPMSampling();
  Serial.println(F("Powering down particulate matter sensor...."));
  stopPM();
  Serial.println(F("Particulate matter sensor testing is complete."));
  Serial.println();
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#endif
