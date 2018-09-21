
/*
 * pod_logging.cpp  
 * 2017 - Nick Turner and Morgan Redfield
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 * 
 * Manage the Real-Time clock and SD card.
 * The timers managed in this file also control
 * when various sensors are sampled.
 */

#include "pod_util.h"
#include "pod_logging.h"
#include "pod_config.h"
#include "pod_network.h"
#include "pod_sensors.h"

#include <SparkFunDS3234RTC.h>
#include <SD.h>

// This library must be modified to increase the maximum number
// of available alarms by changing dtNBR_ALARMS  (default is 6,
// should allow for 10-12).
#include <TimeAlarms.h>


#define logint 01 // whenever seconds hit 01 (RTC)
//SET START MONTH, DAY, HOUR, AND MINUTE.
int startMonth = 0, startDay = 0, startHr = 0, startMinute = 0;

// RTC 3234 settings
#define PRINT_USA_DATE //U.S. format mm/dd/yy

// SD card
File dataFile;
File setFile;
char timestamp[30];
#ifdef DEBUG
File logFile;
#endif

// declaring strings
String amp;

void setupRTC() {
    // RTC 3234
  rtc.begin(DS13074_CS_PIN);
  // NOTE: time is set if/when NTP is established
  
  // Get time/date values, so we can set alarms
  rtc.update();
  // NOTE: Interrupt pin not attached and alarms not used,
  // so we disable this code [CS 2018-08-25]
  /*
  // Configure Alarm(s):
  rtc.enableAlarmInterrupt();
  rtc.setAlarm1(logint);
  // Set alarm2 to alert when minute increments by 1
  bool alarm = true;
  rtc.setAlarm2(0, 8, 1, alarm); // Alarm2 8:00 on Monday (1)
  */
}

void setupPodSD() {
  // SD CARD
  Serial.print(F("Initializing SD card..."));
  pinMode(SD_CHIP_SELECT, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println(F("Card failed, or not present. Readings will not be stored locally."));
  }
  else {
    Serial.println(F("card initialized"));
  }
}

void setupSDLogging() {  
  // create a new file for data logging
  char filename[] = "LOG000.CSV";
  for (uint16_t i = 0; i < 1000; i++) {
    filename[3] = i / 100 + '0';
    filename[4] = (i % 100)/10 + '0';
    filename[5] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      
      SdFile::dateTimeCallback(dateTime);
      dataFile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  /*if (! dataFile) {
    Serial.print("Error opening ");
    Serial.println(filename);
    return;
  }*/

  Serial.print(F("Logging to: "));
  Serial.println(filename);
  String header = F("Date, Time, Light, RH, Air Temp (F), Globe Temp, Sound (dB), CO2 (PPM), PM 2.5, PM 10, CO_SpecSensor"); // FILE HEADER
  dataFile.println(header);
}

void logDataSD(String sensorData) {
  //if (! dataFile)
  //  return;
  #ifdef DEBUG
  writeDebugLog(F("Fxn: logDataSD"));
  #endif
  dataFile.println(sensorData);
  //Serial.println("Date, Time, Light, RH, Air Temp (F), Globe Temp, Sound (dB), CO2, PM 2.5, PM 10, CO");
  //Serial.println(sensorData);
  // ending the loop and clearing variables
  dataFile.flush();
}

void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID) {
  char setname[] = "PODSET.CSV";
  SdFile::dateTimeCallback(dateTime);
  // Save to SD
  if (! SD.exists(setname)) {
    Serial.println(F("No settings file detected. Creating..."));
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
  String Dstamp = formatDate();
  String Tstamp = formatTime();
  String setheader = "Date, Time, Device ID, Project, Location, Coordinator?, Network Code, Setup Date, Teardown Date, Upload Rate, Light, RH, Globe Temp, Sound, CO2, Particle, CO"; // FILE HEADER

  //void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime) {
  settingData = (Dstamp + ", " + Tstamp + ", " + DID + ", " + Project + ", " + Location + ", " + Coordinator + ", " + NetID + ", " + Setup + ", " + Teardown + ", " + getRateUpload() + ", " + getRateLight() + ", " + getRateRH() + ", " + getRateGlobeTemp() + ", " + getRateSound() + ", " + getRateCO2() + ", " + getRatePM() + ", " + getRateCO());
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
    Alarm.delay(1000); // Checks all alarm.timerRepeat events from setup()
  }
}

void setupSensorTimers() {
  // set up timers for sensors
  if(getRateRH() > 0)
    Alarm.timerRepeat(getRateRH(),humidityLog);
  if(getRateSound() > 0)
    Alarm.timerRepeat(getRateSound(),soundLog);
  if(getRateLight() > 0)
    Alarm.timerRepeat(getRateLight(),lightLog);
  if(getRateGlobeTemp() > 0)
    Alarm.timerRepeat(getRateGlobeTemp(),tempLog);
  if(getRateCO2()> 0)
    Alarm.timerRepeat(getRateCO2(),co2Log);
  if(!getModeCoord()) {
    if(getRatePM() > 120) {
      Alarm.timerRepeat(getRatePM(), particleWarmup);
    } else if(getRatePM() > 0){
      Alarm.timerRepeat(getRatePM(), particleLog);
    }
  } else 
    Alarm.timerRepeat(getRatePM(), particleLog);
  if(getRateCO() > 0)
    Alarm.timerRepeat(getRateCO(),coLog);  
}

String formatTime(){
  String h, colon, mt, s;
  rtc.update();
  h = String(rtc.hour(), DEC);
  colon = String(':');
  mt = String(rtc.minute(), DEC);
  s = String(rtc.second(), DEC);
  
  return (h + colon + mt + colon + s);
}

String formatDate(){
  String y, slash, m, d;
  rtc.update();
  d = String (rtc.date(), DEC);
  y = "20" + String (rtc.year(), DEC);
  slash = String ('-');
  m = String(rtc.month(), DEC);
  
  return (y + slash + m + slash + d);
}

String getStringDatetime() {
  #ifdef DEBUG
  writeDebugLog(F("Fxn: getStringDatetime()"));
  #endif
  return formatDate() + " " + formatTime();
  //return String(year()) + "-" + month() + "-" + day() + " " + hour() + ":" + minute() + ":" + second();
}


void printTime()
{
  Serial.print(String(rtc.hour()) + ": "); // Print hour
  if (rtc.minute() < 10)
    Serial.print('0'); // Print leading '0' for minute
  Serial.print(String(rtc.minute()) + ": "); // Print minute
  if (rtc.second() < 10)
    Serial.print('0'); // Print leading '0' for second
  Serial.print(String(rtc.second())); // Print second

  if (rtc.is12Hour()) // If we're in 12-hour mode
  {
    // Use rtc.pm() to read the AM/PM state of the hour
    if (rtc.pm()) Serial.print(" PM"); // Returns true if PM
    else Serial.print(" AM");
  }
  Serial.print(" | ");
  Serial.print(rtc.dayStr()); // Print day string
  Serial.print(" - ");
#ifdef PRINT_USA_DATE
  Serial.print(String(rtc.month()) + " / " +   // Print month
               String(rtc.date()) + " / ");  // Print date
#else
  Serial.print(String(rtc.date()) + " / " +    // (or) print date
               String(rtc.month()) + " / "); // Print month
#endif
  Serial.println(String(rtc.year()));        // Print year

  //Serial.println("end of code -----------------------------------------------");
}

void setRTCTime(unsigned long epoch) {
  Serial.println(epoch);
  setTime(epoch);
  String temp = String(year());
  int yr = temp.substring(2,4).toInt();
  rtc.setTime(second(), minute(), hour(), weekday(), day(), month(), yr);
  Serial.println(formatDate());
  Serial.println(formatTime());
}

// sensor logging functions

void humidityLog() {
  float AirTemp = getRHTemp();
  float RH = getRHHum();
  Serial.print("Humidity: ");
  Serial.print(RH);
  Serial.println(" % ");
  Serial.print("Temperature: ");
  Serial.print(AirTemp * 1.8 + 32);
  Serial.println(" F");
  String RHstr = "";
  String AirTempstr = "";
  RHstr = String (RH);
  AirTempstr = String (AirTemp * 1.8 + 32);
  saveReading("", RHstr, AirTempstr, "", "", "", "", "", "");
}

void lightLog() {
  float light = getLight();
  Serial.print("light : ");
  Serial.print(light);
  Serial.println(" Lux");
  String lightstr = "";
  lightstr = String(light);
  saveReading(lightstr, "", "", "", "", "", "", "", "");
}

void tempLog() {
  double T = getGlobeTemp();
  Serial.print("TempG : ");
  Serial.print(T);
  Serial.println(" deg F");
  String GTempstr = "";
  GTempstr = String (T);
  saveReading("", "", "", GTempstr, "", "", "", "", "");
}

void soundLog() {
  Serial.print("Sound = ");
  double sound_amp = getSound();
  Serial.print(sound_amp / 10);
  Serial.println(" dB");
  String Soundstr = "";
  Soundstr = String (sound_amp / 10);
  saveReading("", "", "", "", Soundstr, "", "", "", "");
}

void co2Log() {
  int co2 = getCO2();
  Serial.print("CO2 : "); Serial.println(co2);
  String CO2str = "";
  CO2str = String (co2);
  saveReading("", "", "", "", "", CO2str, "", "", "");
}

void coLog() {
  float CoSpecRaw = getCO();
  Serial.print("CO_Spec: ");
  Serial.println(CoSpecRaw);
  String CoSpecRawstr = "";
  CoSpecRawstr = String (CoSpecRaw);
  saveReading("", "", "", "", "", "", "", "", CoSpecRawstr);
}


void particleWarmup() {
  digitalWrite(PM_ENABLE, HIGH);
  Alarm.timerOnce(120,particleLog);
  Serial.println("Warming up Particle Meter.");
}

void particleLog() {
  updatePM();
  double c2_5 = getPM2_5_OLD();
  double c10 = getPM10_OLD();

  String PM2_5str = "";
  String PM10str = "";
  PM2_5str = String (c2_5);
  PM10str = String (c10);
  saveReading("", "", "", "", "", "", PM2_5str, PM10str, "");

  if(getRatePM() > 120 and ! getModeCoord())
    digitalWrite(PM_ENABLE, LOW);
}

//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
 sprintf(timestamp, "%02d:%02d:%02d %2d/%2d/%2d \n", hour(),minute(),second(),month(),day(),year()-2000);
 //Serial.println(F("yy"));
 //Serial.println(timestamp);
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(year(), month(), day());

 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(hour(), minute(), second());
}
//------------------------------------------------------------------------------

#ifdef DEBUG

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void writeDebugLog(String message) {
  char logname[] = "DEBUG.CSV";
  SdFile::dateTimeCallback(dateTime);
  // Save to SD
  if (! SD.exists(logname)) {
    Serial.println(F("No debug log file detected. Creating..."));
    // only open a new file if it doesn't exist
    logFile = SD.open(logname, FILE_WRITE);
    delay(1000);
    String logheader = F("Date, Time, Message, Free RAM"); // FILE HEADER
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

  String logData = "";
  String Dstamp = formatDate();
  String Tstamp = formatTime();

  logData = (Dstamp + ", " + Tstamp + ", " + message + ", " + freeRam());
  logFile.println(logData);
  logFile.close();
  Serial.println(F("Debug log file updated."));
}
#endif


