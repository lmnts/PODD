
/*
 * pod_sensors.h
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_SENSORS_H
#define POD_SENSORS_H

#include "Arduino.h"

#define MIC_PIN A0
#define PM_ENABLE 42

#define MIN_RH_TEMP -40 // Operating range, degrees C
#define MAX_RX_TEMP 125 // Operating range, degress C
#define MIN_RH_HUM 0
#define MAX_RX_HUM 100
#define MIN_LIGHT 10 // Experimental
#define MAX_LIGHT 3300 // Experimental
#define MIN_GLOBE_TEMP -67 // Operating range, degrees F
#define MAX_GLOBE_TEMP 176 // Operating range, degrees F
#define MIN_SOUND 200
#define MAX_SOUND 1500
#define MIN_CO2 0 // Operating range, ppm
#define MAX_CO2 2000 // Operating range, ppm 
#define MIN_CO 0 // Operating range, ppm
#define MAX_CO 1000 // Operatiing range, ppm
#define MIN_PM2_5 -1
#define MAX_PM2_5 10000
#define MIN_PM10 -1
#define MAX_PM10 10000

//--------------------------------------------------------------------------------------------- [Sensor Reads]

void sensorSetup();
bool verifySensors();

// get readings
float getRHTemp();
float getRHHum();
float getLight();
double getGlobeTemp();
double getSound();
int getCO2();
float getCO();
void updatePM();
float getPM2_5();
float getPM10();

#endif
