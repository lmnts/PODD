
/*
 * pod_logging.h  
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_LOGGING_H
#define POD_LOGGING_H

#include "Arduino.h"

#ifdef DEBUG
void writeDebugLog(String message);
#endif

void setupRTC();
void setupPodSD();
void setupSDLogging();
void logDataSD(String sensorData);
void writeSDConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID);
void sdDateTime(uint16_t* date, uint16_t* time);
void setupSensorTimers();
void setupClockTimers();
void handleLoopLogging();

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
