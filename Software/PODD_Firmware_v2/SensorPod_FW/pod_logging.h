
/*
 * pod_logging.h  
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_LOGGING_H
#define POD_LOGGING_H

#include "Arduino.h"

#define DS13074_CS_PIN 17 // DeadOn RTC Chip-select (CS) pin
#define SD_CHIP_SELECT 10 // SD card chip select
#define SEC_2017 1482192000
//#define DEBUG 1 // Debug flag

#ifdef DEBUG
void writeDebugLog(String message);
#endif

void setupRTC();
void setupPodSD();
void setupSDLogging();
void logDataSD(String sensorData);
void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID);
void dateTime(uint16_t* date, uint16_t* time);
void setupSensorTimers();
void handleLoopLogging();
String formatTime();
String formatDate();
String formatDateTime();
String getStringDatetime();
void printTime();
void setRTCTime(unsigned long epoch);

// log readings
void humidityLog();
void lightLog();
void tempLog();
void soundLog();
void co2Log();
void coLog();
void particleWarmup();
void particleLog();

#endif
