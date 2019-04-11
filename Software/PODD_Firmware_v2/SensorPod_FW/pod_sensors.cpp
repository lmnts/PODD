
/*
   pod_sensors.cpp
   2017 - Nick Turner and Morgan Redfield
   Licensed under the AGPLv3. For full license see LICENSE.md
   Copyright (c) 2017 LMN Architects, LLC

   Handle setup and reading from various sensors.
*/

#include "pod_sensors.h"

#include <NeoSWSerial.h>
//#include <SoftwareSerial.h>
#include <AsyncDelay.h>
#include <ClosedCube_OPT3001.h>
#include "cozir.h"
#include <Wire.h>
#include <HIH61xx.h>
#include <TimerOne.h>

#ifdef USE_SPS30_PM
#include <sps30.h>
#endif


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
//SoftwareSerial CO2_serial(CO2_PIN_RX,CO2_PIN_TX);
NeoSWSerial CO2_serial(CO2_PIN_RX, CO2_PIN_TX);
// Note COZIR library modified to remove Serial.begin() call in
// constructor as we do _not_ want the serial interface running
// except when we actually want to communicate with the sensor.
COZIR czr(CO2_serial);

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
  // Initially turn off power to sensor.
  // Initialization or other PM routines should enable power
  // when necessary.
  pinMode(PM_ENABLE, OUTPUT);
  digitalWrite(PM_ENABLE, LOW);
  initPM();

  analogReference(EXTERNAL);

  hih.initialise();

  // COZIR sensor communicates at 9600 baud
  CO2_serial.begin(9600);
  CO2_serial.listen();
  czr.SetOperatingMode(CZR_POLLING);
  // Suspend serial interface as this software serial can disrupt
  // other serial interfaces (and we do not need it except when
  // communicating with CO2 sensor).
  // NeoSWSerial can toggle serial interface with just listen/ignore:
  CO2_serial.ignore();
  //Serial.println("DEBUGGING: Turning off software serial (CO2 interface).");
  //CO2_serial.end();
  // SoftwareSerial has no ignore and must be turned off...
  //CO2_serial.end();
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
  return hih.getAmbientTemp() / 100.0; // hih gives temp x 100
}

float getRHHum() {
  hih.read();  // Blocks for ~45ms as sensor is read
  return hih.getRelHumidity() / 100.0; // hih gives RH[%] x 100
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

float getCO() {
  return analogRead(CoSpecSensor); // TODO: Conversion!
}


// Particular Matter Sensor (OLD) --------------------------------------

#ifdef USE_OLD_PM

/* Initializes the particulate matter sensor, but does not power it up. */
void initPM() {
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
void initPM() {
  //pinMode(PM_PIN_P1,INPUT);
  //pinMode(PM_PIN_P2,INPUT);
  //pinMode(PM_ENABLE,OUTPUT);
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
void powerOnPM() {
  if (pmPowered) return;
  digitalWrite(PM_ENABLE, HIGH);
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
    Serial.println("Successfully started PM I2C interface.");
  } else {
    Serial.println("Could not start PM I2C interface.");
  }
  pmPowered = true;
}


/* Power down the particulate matter sensor. */
void powerOffPM() {
  if (!pmPowered) return;
  if (pmRunning) {
    stopPM();
  }
  delay(100);
  digitalWrite(PM_ENABLE, LOW);
  pmPowered = false;
}


/* Indicates if the particulate matter sensor is currently powered on. */
bool isPMPowered() {
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
void startPM(bool wait) {
  if (pmRunning) return;
  if (!pmPowered) powerOnPM();
  //if (!sps30.start()) return;
  if (sps30.start()) {
    Serial.println("Successfully started PM.");
  } else {
    Serial.println("Could not start PM.");
    return;
  }
  if (wait) delay(8000);
  pmRunning = true;
}


/* Stop the particulate matter sensor from taking measurements.
   Reduces power usage from ~ 60 mA to < 8 mA. */
void stopPM() {
  if (!pmRunning) return;
  sps30.stop();
}


/* Indicates if the particulate matter sensor is currently taking data. */
bool isPMRunning() {
  return pmRunning;
}


/* Tests communication with the particulate matter sensor. */
bool probePM() {
  if (!pmPowered) return false;
  if (!pmRunning) return false;
  return sps30.probe();
}


/* Cleans the particulate matter sensor by running the fan at high
   speed for 10 seconds.  Sensor must be powered and running.
   Returns boolean indicating if cleaning process occurred.

   Argument indicates if routine should wait long enough for sensor
   cleaning to complete. */
bool cleanPM(bool wait) {
  if (!pmPowered) return false;
  if (!pmRunning) return false;
  if (!sps30.clean()) return false;
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


/* Returns the most recently retrieved PM_2.5 measurement in ug/m^3.
   PM_2.5 is a measurement of particulate matter 2.5 um in diameter
   or smaller.  Sensor's particle size threshold is ~ 0.3 um.
   Returns -1 if measurement failed/invalid. */
float getPM2_5() {
  return pmData.MassPM2;
}


/* Returns the most recently retrieved PM_10 measurement in ug/m^3.
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
  initPM();
  Serial.print(F("Waiting in unpowered state ("));
  Serial.print(offTime/1000);
  Serial.print(F("s)"));
  printPMPauseProgress(offTime/1000,1000);
  Serial.println();
  Serial.println(F("Powering up particulate matter sensor...."));
  powerOnPM();
  Serial.print(F("Waiting in idle mode ("));
  Serial.print(idleTime/1000);
  Serial.print(F("s)"));
  printPMPauseProgress(idleTime/1000,1000);
  Serial.println();
  
  char buf[32];
  int sn_err = sps30.GetSerialNumber(buf,32);
  Serial.print("Serial number error code: ");
  Serial.println(sn_err);
  
  Serial.println(F("Starting measurements...."));
  startPM(false);
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
    delay(sampleInterval);
    unsigned long t = millis() - t0;
    if (!probePM()) {
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
void initPM() {
  pinMode(PM_PIN_P1,INPUT);
  pinMode(PM_PIN_P2,INPUT);
  pinMode(PM_ENABLE,OUTPUT);
  digitalWrite(PM_ENABLE,LOW);
}


/* Power up the particulate matter sensor.
   This should be done at least 90 seconds before sampling is performed:
   the heater element that draws the bulk of the power generates an air
   current through the sensor that takes time to stabilize. */
void startPM() {
  digitalWrite(PM_ENABLE,HIGH);
  sleep(100);
  sps30.begin(SOFTWARE_SERIAL);
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
