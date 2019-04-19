
/*
 * pod_config.h  
 * 2017 - Nick Turner and Morgan Redfield
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_CONFIG_H
#define POD_CONFIG_H

#include "Arduino.h"

#include <limits.h>

//--------------------------------------------------------------------------------------------- [Intro and Setup]

#define setupTimeout 60000 // 60000ms = 1 min
#define CONFIG_VERSION "demo" // contains extra "\0" character on end.
#define CONFIG_START 32

#define lightT_default 60
#define humidityT_default 60
#define tempT_default 60
#define soundT_default 120
#define co2T_default 60
#define pmT_default 600 //Takes two minutes to power on. Should be set to 120+.
#define coT_default 60
#define DeviceID "proto003"
#define project_default "Demonstration"
#define server_default "ec2-35-160-73-213.us-west-2.compute.amazonaws.com"
#define setup_default "1900-01-01"
#define teardown_default "1970-01-01"
#define upload_default 3600
#define update_default "2017-04-23 12:57:00"
#define network_default "ABCD"

void podIntro();


int podConfig();
bool podConfigChanged();
void clearPodConfigChanged();
void loadPodConfig();
void savePodConfig();

void updateTimer(String sensor);

char * getServer();
char * getDevID();
char * getNetID();
char * getProject();
char * getRoom();

bool getModeCoord();

int getRateUpload();
int getRateLight();
int getRateRH();
int getRateGlobeTemp();
int getRateSound();
int getRateCO2();
int getRatePM();
int getRateCO();

void clearSerial();
char getSerialChar(unsigned long timeout=0);
String getSerialString(unsigned long timeout=0);

char serialCharPrompt(String prompt, char default0=(char)(-1));
String serialStringPrompt(String prompt, String default0="");
bool serialYesNoPrompt(String prompt, bool reprompt, bool default0);
bool serialTrueFalsePrompt(String prompt, bool reprompt, bool default0);
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0);
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0, char tchar, char fchar);
int serialIntegerPrompt(String prompt, bool reprompt, int default0=INT_MIN);
float serialFloatPrompt(String prompt, bool reprompt, float default0=NAN);

void configureProjectSettings();
void configureNodeSettings();
void configureSensorTimingSettings();

void interactivePrompt(unsigned long timeout=30000);
void mainMenu();
void showMenuProjectSettings();
void showMenuNodeSettings();
void showMenuSensorTimingEntry(String s, int v);
void showMenuSensorTimingSettings();
void showMenuClockSettings();

void clockMenu();
void configMenu();
void sensorMenu();

#endif
