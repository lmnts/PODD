
/*
 * pod_config.cpp  
 * 2017 - Nick Turner and Morgan Redfield
 * 2018 - Chris Savage
 * 
 * Handle the Configuration of a SensorPod.
 * Configurations are stored to EEPROM on the Teensy.
 * 
 * Configurations can be changed via USB-Serial interface
 * when the device boots. If no configuration option is
 * chosen after ~2minutes, the configuration utility will
 * time out, and normal operation will resume.
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */
 
#include "pod_config.h"
#include "pod_eeprom.h"
//#include "pod_util.h"
#include "pod_clock.h"
#include "pod_serial.h"
#include "pod_sensors.h"
#include "pod_network.h"
#include "pod_logging.h"

#include "EEPROM.h"


PodConfigStruct storage = {
  CONFIG_VERSION, // Maybe add field for software name before config version for more robust read/write
  server_default,
  DeviceID,
  project_default,
  "Kitchen",
  setup_default,
  teardown_default,
  update_default,
  network_default,
  'N',
  upload_default,
  lightT_default, humidityT_default, tempT_default, soundT_default, co2T_default, pmT_default, coT_default
};
bool configChanged = false;

bool ratesChanged = false;

bool debugMode = false;


//--------------------------------------------------------------------------------------------- [Intro and Setup]

void savePodConfig() { // Save configuration to SD, EEPROM, and DB.  
  //String Datetime = getStringDatetime();
  String Datetime = getDBDateTimeString();
  Datetime.toCharArray(storage.lastUpdate, 20);
  
  // Save to EEPROM
  for (unsigned int t=0; t<sizeof(storage); t++) {
    //EEPROM.write(EEPROM_CONFIG_ADDR + t, *((char*)&storage + t));
    // Put only writes byte if different from current EEPROM value
    // (reduces EEPROM wear).
    EEPROM.put(EEPROM_CONFIG_ADDR + t, *((char*)&storage + t));
  }
  
  if (storage.coord == 'Y') {
    writeSDConfig(storage.devid, storage.room, "Coordinator", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
    updateConfig(storage.devid, storage.room, "Coordinator", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
  } else if (storage.coord == 'N') {
    writeSDConfig(storage.devid, storage.room, "Drone", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
    updateConfig(storage.devid, storage.room, "Drone", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
  } else {
    Serial.println(F("Coordinator setting invalid. ID, Project, Room, Upload Rate, Coordinator Status, Setup Date, and Teardown Date not sent to DB."));
  }

  // Reset configuration change flag
  clearPodConfigChanged();
}

void loadPodConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(EEPROM_CONFIG_ADDR + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(EEPROM_CONFIG_ADDR + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(EEPROM_CONFIG_ADDR + 2) == CONFIG_VERSION[2] &&
      EEPROM.read(EEPROM_CONFIG_ADDR + 3) == CONFIG_VERSION[3])
    for (unsigned int t=0; t<sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(EEPROM_CONFIG_ADDR + t);
}

PodConfigStruct& getPodConfig() {
  return storage;
}

bool podConfigChanged() {
  return configChanged;
}

void setPodConfigChanged() {
  configChanged = true;
}

void clearPodConfigChanged() {
  configChanged = false;
}

void savePodRates() { // Save rates to DB.  
  //String Datetime = getStringDatetime();
  String Datetime = getDBDateTimeString();
  Datetime.toCharArray(storage.lastUpdate, 20);
  
  // Save to DB - Rates
  // If these are sent too fast over XBee network by non-coordinators,
  // there may be buffer overflows and/or packet loss
  //const unsigned long delta = (storage.coord == 'Y') ? 250 : 1000;
  // XBee rate upload routine already adds delay to avoid pileup.
  const unsigned long delta = (storage.coord == 'Y') ? 250 : 250;
  updateRate(storage.devid, "Light", storage.lightT, Datetime);
  delay(delta);
  updateRate(storage.devid, "Humidity", storage.humidityT, Datetime);
  delay(delta);
  updateRate(storage.devid, "GlobeTemp", storage.tempT, Datetime);
  delay(delta);
  updateRate(storage.devid, "Sound", storage.soundT, Datetime);
  delay(delta);
  updateRate(storage.devid, "CO2", storage.co2T, Datetime);
  delay(delta);
  updateRate(storage.devid, "Particle", storage.pmT, Datetime);
  delay(delta);
  updateRate(storage.devid, "CO", storage.coT, Datetime);
  delay(delta);

  // Reset rates change flag
  clearPodRatesChanged();
}

bool podRatesChanged() {
  return ratesChanged;
}

void setPodRatesChanged() {
  ratesChanged = true;
}

void clearPodRatesChanged() {
  ratesChanged = false;
}

bool getDebugMode() {
  return debugMode;
}

void setDebugMode(bool debug) {
  debugMode = debug;
}

char * getServer() {
  return storage.server;
}

char * getDevID() {
  return storage.devid;
}

char * getProject() {
  return storage.project;
}

char * getRoom() {
  return storage.room;
}

bool getModeCoord() {
  return (storage.coord == 'Y');
}

int getRateUpload() {
  return storage.uploadT;
}
int getRateLight() {
  return storage.lightT;
}
int getRateRH() {
  return storage.humidityT;
}
int getRateGlobeTemp() {
  return storage.tempT;
}
int getRateSound() {
  return storage.soundT;
}
int getRateCO2() {
  return storage.co2T;
}
int getRatePM() {
  return storage.pmT;
}
int getRateCO() {
  return storage.coT;
}

char * getNetID() {
  return storage.networkID;
}

/* Embeds the given string in the given character array (intended to
   be one of the char[] fields of the settings structure), flags a
   configuration change, and notifies the user the setting with the
   given label has been changed.  len here is the length of arr. */
void replaceSettingString(String s, char arr[], size_t len, String label) {
  setPodConfigChanged();
  memset(arr,'\0',len);
  s.toCharArray(arr,len);
  if (!label.equals("")) {
    Serial.print(F("  "));
    Serial.print(label);
    Serial.print(F(" changed to \""));
    Serial.print(arr);
    Serial.println(F("\"."));
  }
}

/* Prompts the user to change the given sensor measurement interval,
   flags a configuration change, and notifies the user the timing
   has been changed.  The v argument should be a pointer to the
   appropriate timing field of a PodConfigStruct. */
void updateSensorTime(String label, int *v) {
  int i = serialIntegerPrompt(String("") + label,true,*v);
  if (i < 0) i = 0;
  if (i != *v) {
    setPodRatesChanged();
    *v = i;
    if (*v > 0) {
      Serial.print(F("  Interval changed to "));
      Serial.print(*v);
      Serial.println(F(" seconds."));
    } else {
      Serial.println(F("  Sensor now disabled."));
    }
  }
}


//----------------------------------------------------------------------
