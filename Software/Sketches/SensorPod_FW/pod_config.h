
/*
 * pod_config.h  
 * 2017 - Nick Turner and Morgan Redfield
 * 2018 - Chris Savage
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_CONFIG_H
#define POD_CONFIG_H

#include "Arduino.h"


//--------------------------------------------------------------------------------------------- [Intro and Setup]

#define setupTimeout 60000 // 60000ms = 1 min
#define CONFIG_VERSION "demo" // contains extra "\0" character on end.
//#define CONFIG_START 32

#define lightT_default 60
#define humidityT_default 60
#define tempT_default 60
#define soundT_default 60
#define co2T_default 60
#define pmT_default 600 //Takes two minutes to warm up and settle down
#define coT_default 60
#define DeviceID "DEFAULT_DEVICEID"
#define project_default "Demonstration"
#define server_default "ec2-54-212-239-216.us-west-2.compute.amazonaws.com"
#define setup_default "1900-01-01"
#define teardown_default "1970-01-01"
#define upload_default 3600
#define update_default "1970-01-01 00:00:00"
#define network_default "ABCD"

struct PodConfigStruct {
  char pod_version[5], server[61], devid[17], project [17], room[17], setupD[11], teardownD[11], lastUpdate[20], networkID[5];
  char coord; // 
  int uploadT,lightT,humidityT,tempT,soundT,co2T,pmT,coT;
};


void loadPodConfig();
void savePodConfig();
PodConfigStruct& getPodConfig();
bool podConfigChanged();
void setPodConfigChanged();
void clearPodConfigChanged();

void savePodRates();
bool podRatesChanged();
void setPodRatesChanged();
void clearPodRatesChanged();

bool getDebugMode();
void setDebugMode(bool debug);

void replaceSettingString(String s, char arr[], size_t len, String label);
void updateSensorTime(String label, int *v);

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

#endif
