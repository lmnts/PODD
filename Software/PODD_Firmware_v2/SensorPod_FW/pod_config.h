
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

void savePodConfig();

int podConfig();

void updateTimer(String sensor);

void loadPodConfig();

char * getServer();
char * getDevID();
char * getNetID();

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
