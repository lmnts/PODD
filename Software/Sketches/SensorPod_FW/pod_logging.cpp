
/*
 * pod_logging.cpp  
 * 2017 - Nick Turner and Morgan Redfield
 * 2018 - Chris Savage
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 * 
 * Manage the Real-Time clock and SD card.
 * The timers managed in this file also control
 * when various sensors are sampled.
 */

#include "pod_util.h"
#include "pod_clock.h"
#include "pod_logging.h"
#include "pod_config.h"
#include "pod_network.h"
#include "pod_sensors.h"

#include <SD.h>

// This library must be modified to increase the maximum number
// of available alarms by changing dtNBR_ALARMS  (default is 6,
// should allow for 10-12).
#include <TimeAlarms.h>

// Frequencies at which to poll NTP server for current time
// and at which to broadcast the current time to other nodes;
// both apply only to the coordinator node.  Intervals are in
// seconds.
#define NTP_POLL_INTERVAL 3600
#define CLOCK_BROADCAST_INTERVAL 60

// Frequency at which to broadcast the coordinator's address.
// Needed by drones to permit unicast addressing, which reduces
// network congestion (relative to broadcasting all packets).
// Once the address is received by a drone, it will be
// remembered permanently until a different address is received
// through a broadcast.  That means this rate can be fairly
// low, with the only drawback that drones may take some time
// to update their destination if the coordinator node changes.
#define ADDRESS_BROADCAST_INTERVAL 60

#define logint 01 // whenever seconds hit 01 (RTC)
//SET START MONTH, DAY, HOUR, AND MINUTE.
int startMonth = 0, startDay = 0, startHr = 0, startMinute = 0;

// RTC 3234 settings
#define PRINT_USA_DATE //U.S. format mm/dd/yy

// SD card
#define SD_CHIP_SELECT 10
File dataFile;
File setFile;
char timestamp[30];
#ifdef DEBUG
File logFile;
#endif


//----------------------------------------------------------------------

void setupPodSD() {
  // SD CARD
  Serial.print(F("Initializing SD card...."));
  pinMode(SD_CHIP_SELECT, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println(F(" Card failed, or not present. Readings will not be stored locally."));
  }
  else {
    Serial.println(F(" card initialized"));
  }
}

void setupSDLogging() {  
  // Data log file directory and name based on date/time.
  // Use UTC time.
  //time_t t = getUTC();
  // Use local time.
  time_t t = getLocalTime();
  tmElements_t tm;
  breakTime(t,tm);
  
  // Create data log directory (/data/YYYY/MM/)
  char dirname[16];
  sprintf(dirname,"/data/%04d/%02d/",1970+tm.Year,tm.Month);
  //sprintf(dirname,"/data/%02d%02d/",(1970+tm.Year)%100,tm.Month);
  if (!SD.exists(dirname)) {
    SD.mkdir(dirname);
  }
  
  // Create data log file (YYMMDDHH.CSV)
  char filename[32];
  sprintf(filename,"%s%02d%02d%02d%02d.CSV",dirname,
          ((1970+tm.Year) % 100),tm.Month,tm.Day,tm.Hour);
 // Note FILE_WRITE will create non-existent file, append to
  // existent file.
  SdFile::dateTimeCallback(sdDateTime);
  dataFile = SD.open(filename,FILE_WRITE);
  
  /*if (!dataFile) {
    Serial.print(F("Error opening "));
    Serial.println(filename);
    return;
  }*/

  Serial.print(F("Logging to: "));
  Serial.println(filename);
  String header = F("Timestamp, Date/Time, Light, RH, Air Temp (F), Globe Temp, Sound (dB), CO2 (PPM), PM 2.5, PM 10, CO_SpecSensor"); // FILE HEADER
  dataFile.println(header);
}

void logDataSD(String sensorData) {
  //if (! dataFile)
  //  return;
  #ifdef DEBUG
  writeDebugLog(F("Fxn: logDataSD"));
  #endif
  dataFile.println(sensorData);
  // ending the loop and clearing variables
  dataFile.flush();
}

void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID) {
  char setname[] = "PODSET.CSV";
  SdFile::dateTimeCallback(sdDateTime);
  // Save to SD
  if (! SD.exists(setname)) {
    Serial.println(F("No settings file detected. Creating...."));
    // only open a new file if it doesn't exist
    setFile = SD.open(setname, FILE_WRITE);
    delay(1000);
    String setheader = "Date, Time, Device ID, Project, Location, Coordinator?, Network Code, Setup Date, Teardown Date, Upload Rate, Light, RH, Globe Temp, Sound, CO2, Particle, CO"; // FILE HEADER
    setFile.println(setheader);
    Serial.println(F("Settings file created."));
    setFile.flush();
  } else if (SD.exists(setname)) {
    Serial.println(F("Settings file found. Updating... "));
    setFile = SD.open(setname, FILE_WRITE);
  }

  if (! setFile) {
    Serial.print(F("\nError opening "));
    Serial.print(setname);
    Serial.println("!");
  }

  String settingData = "";
  time_t utc = getUTC();
  String TS(utc);
  String DT = getDBDateTimeString(utc);
  String setheader = "Timestamp, Date/Time, Device ID, Project, Location, Coordinator?, Network Code, Setup Date, Teardown Date, Upload Rate, Light, RH, Globe Temp, Sound, CO2, Particle, CO"; // FILE HEADER

  //void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime) {
  settingData = (TS + ", " + DT + ", " + DID + ", " + Project + ", " + Location + ", " + Coordinator + ", " + NetID + ", " + Setup + ", " + Teardown + ", " + getRateUpload() + ", " + getRateLight() + ", " + getRateRH() + ", " + getRateGlobeTemp() + ", " + getRateSound() + ", " + getRateCO2() + ", " + getRatePM() + ", " + getRateCO());
  setFile.println(settingData);
  setFile.close();
  Serial.println(F("Settings file updated."));
}

void handleLoopLogging() {
  // do any tasks required by the config in loop
  if(getModeCoord()) {
    Alarm.delay(0);
    processXBee();
  }
  else {
    Alarm.delay(250); // Checks all alarm.timerRepeat events from setup()
    processXBee();
  }
}

void setupSensorTimers() {
  // set up timers for sensors.
  // Add a delay for each to avoid pileups if using the same
  // interval times (or multiples of each other).  Since TimeAlarms
  // resolution is in seconds, make sure the delay is at least one
  // second.
  //const int init_delay = 1500;
  // NOTE: Delay in XBee sensor reading upload routine prevents pileup.
  const int init_delay = 0;

  // Illuminance
  if(getRateLight() > 0) {
    if (probeLightSensor()) {
      Alarm.timerRepeat(getRateLight(),lightLog);
      delay(init_delay);
    } else {
      Serial.println(F("WARNING: Failed to communicate with light sensor."));
      Serial.println(F("         No readings will be performed."));
    }
  }

  // Sound: turn off background sampling if not needed
  if(getRateSound() > 0) {
    startSoundSampling();
    Alarm.timerRepeat(getRateSound(),soundLog);
    delay(init_delay);
  } else {
    stopSoundSampling();
  }

  // Humidity/temperature
  if(getRateRH() > 0) {
    if (probeTemperatureSensor()) {
      Alarm.timerRepeat(getRateRH(),humidityLog);
      delay(init_delay);
    } else {
      Serial.println(F("WARNING: Failed to communicate with temperature/humidity sensor."));
      Serial.println(F("         No readings will be performed."));
    }
  }

  // Radiant temperature
  if(getRateGlobeTemp() > 0) {
    Alarm.timerRepeat(getRateGlobeTemp(),tempLog);
    delay(init_delay);
  }

  // CO2 sensor
  if(getRateCO2()> 0) {
    // Communication with CO2 sensor sometimes intermittently fails:
    // try a few times to ensure sensor really unavailable before
    // disabling measurements.
    bool b = false;
    for (int k = 0; k < 3; k++) {
      delay(10);
      b = probeCO2Sensor();
      if (b) break;
    }
    if (b) {
      Alarm.timerRepeat(getRateCO2(),co2Log);
      delay(init_delay);
    } else {
      Serial.println(F("WARNING: Failed to communicate with CO2 sensor."));
      Serial.println(F("         No readings will be performed."));
    }
  }
  
  // CO sensor
  if(getRateCO() > 0) {
    Alarm.timerRepeat(getRateCO(),coLog);  
    delay(init_delay);
  }

  // Particulate matter sensor: turn off if not using
  // or if long time between measurements.
  // First check if sensor is available.
  bool pmAvailable = false;
  if (getRatePM() > 0) {
    powerOnPMSensor();
    delay(10);
    startPMSensor();
    delay(10);
    pmAvailable = probePMSensor();
    if (!pmAvailable) {
      Serial.println(F("WARNING: Failed to communicate with particulate matter sensor."));
      Serial.println(F("         No readings will be performed."));
    }
  }
  if (!pmAvailable) {
    stopPMSensor();
    powerOffPMSensor();
  } else if(getRatePM() > 120) {
    stopPMSensor();
    powerOffPMSensor();
    Alarm.timerRepeat(getRatePM(), particleWarmup);
    delay(init_delay);
  } else if(getRatePM() > 0){
    powerOnPMSensor();
    delay(10);
    startPMSensor();
    Alarm.timerRepeat(getRatePM(), particleLog);
    delay(init_delay);
  } else {
    stopPMSensor();
    powerOffPMSensor();
  }
}

/* Set up timers for network-related tasks, like updating the
   time from NTP, broadcasting the time across XBee network,
   and broadcasting the coordinator's address. */
void setupNetworkTimers() {
  if (getModeCoord()) {
    Alarm.timerRepeat(NTP_POLL_INTERVAL,updateClockFromNTP);
    Alarm.timerRepeat(CLOCK_BROADCAST_INTERVAL,broadcastClock);
    Alarm.timerRepeat(ADDRESS_BROADCAST_INTERVAL,broadcastCoordinatorAddress);
  }
}


//----------------------------------------------------------------------
// sensor logging functions

void humidityLog() {
  if (!retrieveTemperatureData()) {
    Serial.println(F("Failed to retrieve temperature/humidity data."));
    return;
  }
  float AirTemp = getTemperature();
  float RH = getRelHumidity();
  Serial.print(F("Humidity: "));
  Serial.print(RH);
  Serial.println(F("%"));
  Serial.print(F("Temperature: "));
  //Serial.print(AirTemp * 1.8 + 32);
  Serial.print(AirTemp);
  Serial.println(F(" °F"));
  String RHstr(RH);
  String AirTempstr(AirTemp);
  saveReading("", RHstr, AirTempstr, "", "", "", "", "", "");
}

void lightLog() {
  float light = getLight();
  if (isnan(light)) {
    Serial.println(F("Failed to retrieve light data."));
    return;
  }
  Serial.print(F("Light: "));
  Serial.print(light);
  Serial.println(F(" lux"));
  String lightstr(light);
  saveReading(lightstr, "", "", "", "", "", "", "", "");
}

void tempLog() {
  float T = getGlobeTemperature();
  if (isnan(T)) {
    Serial.println(F("Failed to retrieve globe temperature."));
    return;
  }
  Serial.print(F("TempG: "));
  Serial.print(T);
  Serial.println(F(" °F"));
  String GTempstr(T);
  saveReading("", "", "", GTempstr, "", "", "", "", "");
}

void soundLog() {
  float sound_amp = getSound();
  if (isnan(sound_amp)) {
    Serial.println(F("Failed to retrieve sound level."));
    return;
  }
  Serial.print(F("Sound: "));
  Serial.print(sound_amp);
  Serial.println(F(" [arb]"));
  String Soundstr(sound_amp);
  saveReading("", "", "", "", Soundstr, "", "", "", "");
}

void co2Log() {
  int co2 = getCO2();
  Serial.print(F("CO2: "));
  Serial.print(co2);
  Serial.println(F(" ppm"));
  String CO2str(co2);
  saveReading("", "", "", "", "", CO2str, "", "", "");
}

void coLog() {
  float CoSpecRaw = getCO();
  Serial.print(F("CO: "));
  Serial.print(CoSpecRaw);
  Serial.println(F(" [arb]"));
  String CoSpecRawstr(CoSpecRaw);
  saveReading("", "", "", "", "", "", "", "", CoSpecRawstr);
}


void particleWarmup() {
  powerOnPMSensor();
  delay(10);
  startPMSensor();
  // Sensor does not return data for ~ 5 seconds,
  // but takes 80-120 seconds for measurements to
  // settle down (initially very inaccurate).
  Alarm.timerOnce(120,particleLog);
  Serial.println(F("Warming up particulate matter sensor."));
}

void particleLog() {
  //updatePM();
  if (retrievePMData()) {
    double c2_5 = getPM2_5();
    double c10 = getPM10();
    
    Serial.print(F("PM_2.5: "));
    Serial.print(c2_5);
    Serial.println(F(" ug/m^3"));
    Serial.print(F("PM_10:  "));
    Serial.print(c10);
    Serial.println(F(" ug/m^3"));
    String PM2_5str(c2_5);
    String PM10str(c10);
    saveReading("", "", "", "", "", "", PM2_5str, PM10str, "");
  } else {
    Serial.println(F("Failed to retrieve particle meter data."));
  }

  if(getRatePM() > 120) {
    powerOffPMSensor();
  }
}

//------------------------------------------------------------------------------
// call back for file timestamps
void sdDateTime(uint16_t* date, uint16_t* time) {
  time_t t = getUTC();
  tmElements_t tm;
  breakTime(t,tm);
  *date = FAT_DATE(1970+tm.Year,tm.Month,tm.Day);
  *time = FAT_TIME(tm.Hour,tm.Minute,tm.Second);
}


//------------------------------------------------------------------------------

#ifdef DEBUG
void writeDebugLog(String message) {
  char logname[] = "DEBUG.CSV";
  SdFile::dateTimeCallback(sdDateTime);
  // Save to SD
  if (! SD.exists(logname)) {
    Serial.println(F("No debug log file detected. Creating...."));
    // only open a new file if it doesn't exist
    logFile = SD.open(logname, FILE_WRITE);
    delay(1000);
    String logheader = F("Timestamp, Date/Time, Message, Free RAM"); // FILE HEADER
    logFile.println(logheader);
    Serial.println(F("Settings file created."));
    logFile.flush();
  } else if (SD.exists(logname)) {
    Serial.println(F("Debug log file found. Updating... "));
    logFile = SD.open(logname, FILE_WRITE);
  }

  if (! logFile) {
    Serial.print(F("\nError opening "));
    Serial.print(logname);
    Serial.println("!");
  }
  
  time_t utc = getUTC();
  String logData = "";
  String TS(utc);
  String DT = getDBDateTimeString(utc);
  logData = (TS + ", " + DT + ", " + message + ", " + freeRAM());
  logFile.println(logData);
  logFile.close();
  Serial.println(F("Debug log file updated."));
}
#endif
