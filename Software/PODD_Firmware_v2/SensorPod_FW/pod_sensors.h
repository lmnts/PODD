
/*
 * pod_sensors.h
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_SENSORS_H
#define POD_SENSORS_H

#include "Arduino.h"


// Define this to enable individual sensor testing output and routines.
//#define SENSOR_TESTING


//--------------------------------------------------------------------------------------------- [Sensor Reads]

// Multi-sensor routines
void sensorSetup();
void initSensors();
void printSensorCheck();
void testSensors(unsigned long cycles = -1, unsigned long sampleInterval = 1000);

// ADC routines
void initADC();
void startADCFreeRunning();
void stopADCFreeRunning();
bool isADCFreeRunning();
int readAnalog(uint8_t pin);
int readAnalogFast();

// get readings
//float getRHTemp();
//float getRHHum();
//float getLight();
//double getGlobeTemp();
//double getSound();
//int getCO2();
//float getCO();

// Ambient light sensor
void initLightSensor();
bool probeLightSensor();
float getLight();

// Sound sensor
void initSoundSensor();
float getSound();
void startSoundSampling();
void stopSoundSampling();
bool isSoundSampling();
void sampleSoundISR();
void resetSoundData();
#ifdef SENSOR_TESTING
void testSoundSensor(unsigned long cycles = -1, unsigned long sampleInterval = 1000);
#endif

// Temperature/humidity sensor
void initTemperatureSensor();
bool probeTemperatureSensor();
bool retrieveTemperatureData();
float getTemperature();
float getRelHumidity();
#ifdef SENSOR_TESTING
void testTemperatureSensor(unsigned long cycles = -1, unsigned long sampleInterval = 1000);
#endif

// Globe/radiant temperature sensor
void initGlobeTemperatureSensor();
bool probeGlobeTemperatureSensor();
float getGlobeTemperature();

// CO2 sensor
void initCO2Sensor();
bool probeCO2Sensor();
int getCO2();
void setCO2(int ppm);
void enableCO2Serial();
void disableCO2Serial();
String cozirCommandString(char c, int v);
bool cozirSendCommand(char c, int v=-1);
int cozirGetValue(char c, int v=-1);

// CO sensor
void initCOSensor();
bool probeCOSensor();
float getCO();

// Sensirion SPS30 Particulate Matter Sensor routines
void initPMSensor();
void powerOnPMSensor();
void powerOffPMSensor();
bool isPMSensorPowered();
void startPMSensor(bool wait=false);
void stopPMSensor();
bool isPMSensorRunning();
bool probePMSensor();
bool cleanPMSensor(bool wait=false);
void resetPMData();
bool retrievePMData();
inline void updatePM(){retrievePMData();}  // for compatibility
float getPM2_5();
float getPM10();
// Below only used for testing
#ifdef SENSOR_TESTING
void printPMPauseProgress(unsigned int N, unsigned long pause = 1000);
void testPMSensor(unsigned long cycles = -1, unsigned long sampleInterval = 3000,
                  unsigned long offTime = 10000, unsigned long idleTime = 10000);
#endif

#endif
