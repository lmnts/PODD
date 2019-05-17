
/*
 * pod_config.cpp  
 * 2017 - Nick Turner and Morgan Redfield
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


//struct PodConfigStruct {
//  char pod_version[5], server[61], devid[17], project [17], room[17], setupD[11], teardownD[11], lastUpdate[20], networkID[5];
//  char coord; // 
//  int uploadT,lightT,humidityT,tempT,soundT,co2T,pmT,coT;
//};

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

String rx_str = "";


//--------------------------------------------------------------------------------------------- [Intro and Setup]

void podIntro() { // Introduction control loop
  unsigned long timeout = millis() + setupTimeout;
  int choice;
  int finished = 0;
  int custom = 0;

  Serial.println(F("\nDo you wish to manually configure Sensor Pod? [Y]/[N]"));
  while (millis() < timeout) { // 60000 = 1 min
    if (Serial.available()) {
      choice = (Serial.read());
      clearSerial();
      if (choice == 121 || choice == 89) { // Y or y entered
        while(!custom){
          podConfig();
          Serial.println(F("\nIs current configuration acceptable? [Y/N]"));
          while(!finished){
            if(Serial.available()) {
              finished=Serial.read();
              clearSerial();
              if (finished == 121 || finished == 89) {
                custom = 1;
                Serial.println(F("Y"));
              } else if(finished == 110 || finished == 78){
                custom = 0;
                Serial.println(F("N"));
              } else {
                Serial.println(F("Invalid choice. Please select [Y]es or [N]o"));
                finished = 0;
              }
            }
          }
          finished = 0;
        }
        xbeeConfig();
        savePodConfig();
        timeout = setupTimeout;
      }
      else if (choice == 110 || choice == 78){ // N or n entered
        timeout -= setupTimeout;
      }
      else {
        Serial.println(F("Invalid choice. Please select [Y]es or [N]o"));
      }
    }
  }
  Serial.println(F("\nRestart Sensor Pod to readjust settings."));
  Serial.println(F("\nSensing begins with the following settings:"));
  
  Serial.print(F("Version: "));
  Serial.println(storage.pod_version);
  Serial.print(F("Device ID: "));
  Serial.println(storage.devid);
  Serial.print(F("Project: "));
  Serial.println(storage.project);
  Serial.print(F("Location: "));
  Serial.println(storage.room);
  Serial.print(F("Coordinator?: "));
  Serial.println(storage.coord);
  // XBee network configuration handled externally (for now)
  //Serial.print(F("Network Code: "));
  //Serial.println(storage.networkID);
  Serial.print(F("Setup Date: "));
  Serial.println(storage.setupD);
  Serial.print(F("Teardown Date: "));
  Serial.println(storage.teardownD);
  
  Serial.print(F("Light measurement interval: "));
  Serial.println(storage.lightT);
  Serial.print(F("Humidity measurement interval: "));
  Serial.println(storage.humidityT);
  Serial.print(F("Temperature measurement interval: "));
  Serial.println(storage.tempT);
  Serial.print(F("Sound measurement interval: "));
  Serial.println(storage.soundT);
  Serial.print(F("CO2 measurement interval: "));
  Serial.println(storage.co2T);
  Serial.print(F("Particle measurement interval: "));
  Serial.println(storage.pmT);
  Serial.print(F("CO measurement interval: "));
  Serial.println(storage.coT);
}

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

/* Helper function for podConfig routine below:
   Embeds the given string in the given character array (intended to
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

/* Helper function for podConfig routine below:
   Prompts the user to change the given sensor measurement interval,
   flags a configuration change, and notifies the user the timing
   has been changed.  The v argument should be a pointer to the
   appropriate timing field of a PodConfigStruct. */
void updateSensorTime(String label, int *v) {
  int i = serialIntegerPrompt(String("") + label,true,*v);
  if (i < 0) i = 0;
  if (i != *v) {
    setPodConfigChanged();
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


// NOTE: Most of the code in this routine has been broken into
// subroutines for easier use by other routines.  This routine
// will likely be removed in the future.
int podConfig() {
  //int serial;
  //int complete = 0;
  //unsigned int roomLen, serverLen, idLen, projLen, dateLen, netLen;
  //int timer;
  bool b;
  //int i;
  //float f;
  String s;
  
  Serial.println(F("Proceeding with Pod configuration."));

  //******************** Pod Settings ************************// 
  Serial.println();
  Serial.println(F("Current settings are shown in square brackets."));
  Serial.println(F("Press enter to keep the current setting."));
  Serial.println();

  s = serialStringPrompt(F("Upload server"),storage.server);
  if (!s.equals(storage.server)) {
    replaceSettingString(s,storage.server,sizeof(storage.server),F("Server"));
    /*
    setPodConfigChanged();
    memset(storage.server,'\0',sizeof(storage.server));
    s.toCharArray(storage.server,sizeof(storage.server));
    Serial.print(F("  Server changed to \""));
    Serial.print(storage.server);
    Serial.println("\".");
    */
  }
  
  s = serialStringPrompt(F("Project name (max 16 characters)"),storage.project);
  if (!s.equals(storage.project)) {
    replaceSettingString(s,storage.project,sizeof(storage.project),F("Project name"));
    /*
    setPodConfigChanged();
    memset(storage.project,'\0',sizeof(storage.project));
    s.toCharArray(storage.project,sizeof(storage.project));
    Serial.print(F("  Project name changed to \""));
    Serial.print(storage.project);
    Serial.println("\".");
    */
  }
  
  s = serialStringPrompt(F("Project setup date (YYY-MM-DD)"),storage.setupD);
  if (!s.equals(storage.setupD)) {
    replaceSettingString(s,storage.setupD,sizeof(storage.setupD),F("Project setup date"));
    /*
    setPodConfigChanged();
    memset(storage.setupD,'\0',sizeof(storage.setupD));
    s.toCharArray(storage.setupD,sizeof(storage.setupD));
    Serial.print(F("  Project setup date changed to \""));
    Serial.print(storage.setupD);
    Serial.println("\".");
    */
  }
  
  s = serialStringPrompt(F("Project teardown date (YYY-MM-DD)"),storage.teardownD);
  if (!s.equals(storage.teardownD)) {
    replaceSettingString(s,storage.teardownD,sizeof(storage.teardownD),F("Project teardown date"));
    /*
    setPodConfigChanged();
    memset(storage.teardownD,'\0',sizeof(storage.teardownD));
    s.toCharArray(storage.teardownD,sizeof(storage.teardownD));
    Serial.print(F("  Project teardown date changed to \""));
    Serial.print(storage.teardownD);
    Serial.println("\".");
    */
  }
  
  s = serialStringPrompt(F("Device ID (max 16 characters)"),storage.devid);
  if (!s.equals(storage.devid)) {
    replaceSettingString(s,storage.devid,sizeof(storage.devid),F("Device ID"));
    /*
    setPodConfigChanged();
    memset(storage.devid,'\0',sizeof(storage.devid));
    s.toCharArray(storage.devid,sizeof(storage.devid));
    Serial.print(F("  Device ID changed to \""));
    Serial.print(storage.devid);
    Serial.println("\".");
    */
  }
  
  s = serialStringPrompt(F("Room/location (max 16 characters)"),storage.room);
  if (!s.equals(storage.room)) {
    replaceSettingString(s,storage.room,sizeof(storage.room),F("Room/location"));
    /*
    setPodConfigChanged();
    memset(storage.room,'\0',sizeof(storage.room));
    s.toCharArray(storage.room,sizeof(storage.room));
    Serial.print(F("  Room/location changed to \""));
    Serial.print(storage.room);
    Serial.println("\".");
    */
  }

  // Use XBee Grove dev board + XCTU exclusively to configure
  // XBee network, aside from coordinator mode: current firmware
  // not set up to provide access to many of the useful settings
  // and it is too easy to misconfigure the XBee. 
  /*
  s = serialStringPrompt(F("Network ID (hexadecimal from 0000 to 07FFF)"),storage.networkID);
  if (!s.equals(storage.networkID)) {
    replaceSettingString(s,storage.networkID,sizeof(storage.networkID),F("Network ID"));
    / *
    setPodConfigChanged();
    memset(storage.networkID,'\0',sizeof(storage.networkID));
    s.toCharArray(storage.networkID,sizeof(storage.networkID));
    Serial.print(F("  Network ID changed to \""));
    Serial.print(storage.networkID);
    Serial.println("\".");
    * /
  }
  */
  
  bool iscoord = getModeCoord();
  b = serialYesNoPrompt(F("Network coordinator (y/n)"),true,iscoord);
  if (b != iscoord) {
    setPodConfigChanged();
    if (b) {
      storage.coord = 'Y';
      Serial.println(F("  This node now set to network coordinator."));
    } else {
      storage.coord = 'N';
      Serial.println(F("  This node is no longer the network coordinator."));
    }
  }
  
  /*
  Serial.print(F("\nThis Pod will upload to the server \""));
  Serial.print(storage.server);
  Serial.println('"');
  Serial.println(F("Do you want to change the server? [Y]/[N]"));

  while (!complete) {
    if (Serial.available()) {
      serial = (Serial.read());
      clearSerial();
      if (serial == 121 || serial == 89) { // Y or y entered
        Serial.print(F("Enter new server address (Default is \""));
        Serial.print(server_default);
        Serial.println(F("\")"));
        while (!complete) {
          if (Serial.available()) {
            serverLen = Serial.available();
            for (unsigned int x=0;x<serverLen;x++)
              storage.server[x] = char(Serial.read());
            for (unsigned int y = serverLen;y<sizeof(storage.server);y++)
              storage.server[y] = *"";
            Serial.print(F("Server is now "));
            Serial.println(storage.server);
            complete = 1;
          }
        }
      }
      else if (serial == 110 || serial == 78){ // N or n entered
        Serial.println(F("Server unchanged."));
        complete = 1;
      }
      else {
        Serial.println(F("Invalid choice. Please select [Y]es or [N]o"));
      }
    }
  }
  complete = 0;

  */

  /*
  Serial.print(F("\nThis Pod's Device ID is currently \""));
  Serial.print(storage.devid);
  Serial.println('"');
  Serial.println(F("What would you like the new Device ID to be? (Max 16 characters)"));
  while(!complete) {
    if(Serial.available()){
      idLen = Serial.available();
      for (unsigned int v=0;v<idLen;v++)
        storage.devid[v] = char(Serial.read());
      for (unsigned int w = idLen;w<sizeof(storage.devid);w++)
        storage.devid[w] = *"";
      Serial.print(F("Pod is now "));
      Serial.println(storage.devid);
      complete = 1;
    }
  }

  complete = 0;
  */

  /*
  Serial.print(F("\nThis Pod's Project is currently \""));
  Serial.print(storage.project);
  Serial.println('"');
  Serial.println(F("What would you like the new Project to be? (Max 16 characters)"));
  while(!complete) {
    if(Serial.available()){
      projLen = Serial.available();
      for (unsigned int v=0;v<projLen;v++)
        storage.project[v] = char(Serial.read());
      for (unsigned int w = projLen;w<sizeof(storage.project);w++)
        storage.project[w] = *"";
      Serial.print(F("Project is now "));
      Serial.println(storage.project);
      complete = 1;
    }
  }

  complete = 0;
  */

  /*
  Serial.print(F("\nThis Pod was most recently set to "));
  if(storage.coord == 'Y')
    Serial.println(F("be a coordinator."));
  else if(storage.coord == 'N')
    Serial.println(F("be a drone."));
  else
    Serial.println(F("an invalid coordinator selection."));
  Serial.println(F("Is this Pod going to be a coordinator? [Y]/[N]"));

  while (!complete) {
    if (Serial.available()) {
      serial = (Serial.read());
      if (serial == 121 || serial == 89) { // Y or y entered
        storage.coord = 'Y';
        Serial.println(storage.coord);
        Serial.println(F("This Pod IS a coordinator."));
        complete  = 1;
      }
      else if (serial == 110 || serial == 78){ // N or n entered
        storage.coord = 'N';
        Serial.println(storage.coord);
        Serial.println(F("This Pod is NOT a coordinator."));
        complete = 1;
      }
      else {
        Serial.println(F("Invalid choice. Please select [Y]es or [N]o"));
      }
    }
  }

  complete = 0;
  */

  /*
  Serial.print(F("\nThis Pod was most recently assigned to \""));
  Serial.print(storage.room);
  Serial.println('"');
  Serial.println(F("What room is this Pod in now? (Max 16 characters)"));
  while(!complete) {
    if(Serial.available()){
      roomLen = Serial.available();
      for (unsigned int t=0;t<roomLen;t++)
        storage.room[t] = char(Serial.read());
      for (unsigned int u = roomLen;u<sizeof(storage.room);u++)
        storage.room[u] = *"";
      Serial.print(F("Pod is in "));
      Serial.println(storage.room);
      complete = 1;
    }
  }

  complete = 0;
  */

  /*
  Serial.print(F("\nThis Pod's project setup date is currently \""));
  Serial.print(storage.setupD);
  Serial.println('"');
  Serial.println(F("What would you like the new project setup date to be? (YYYY-MM-DD)"));
  while(!complete) {
    if(Serial.available()){
      dateLen = Serial.available();
      for (unsigned int v=0;v<dateLen;v++)
        storage.setupD[v] = char(Serial.read());
      for (unsigned int w = dateLen;w<sizeof(storage.setupD);w++)
        storage.setupD[w] = *"";
      Serial.print(F("Setup date is now "));
      Serial.println(storage.setupD);
      complete = 1;
    }
  }

  complete = 0;
  */
  
  /*
  Serial.print(F("\nThis Pod's project teardown date is currently \""));
  Serial.print(storage.teardownD);
  Serial.println('"');
  Serial.println(F("What would you like the new project teardown date to be? (YYYY-MM-DD)"));
  while(!complete) {
    if(Serial.available()){
      dateLen = Serial.available();
      for (unsigned int v=0;v<dateLen;v++)
        storage.teardownD[v] = char(Serial.read());
      for (unsigned int w = dateLen;w<sizeof(storage.teardownD);w++)
        storage.teardownD[w] = *"";
      Serial.print(F("Teardown date is now "));
      Serial.println(storage.teardownD);
      complete = 1;
    }
  }

  complete = 0;
  */

  /*
  Serial.print(F("\nThis Pod's network code is currently \""));
  Serial.print(storage.networkID);
  Serial.println('"');
  Serial.println(F("What would you like the new network code to be? (must be between 0000 and 7999)"));
  while(!complete) {
    //getUserInput();

    if(Serial.available()){
      netLen = Serial.available();
      for (unsigned int v=0;v<netLen;v++)
        storage.networkID[v] = char(Serial.read());
      for (unsigned int w = netLen;w<sizeof(storage.networkID);w++)
        storage.networkID[w] = *"";
      Serial.print(F("Network code is now "));
      Serial.println(storage.networkID);
      complete = 1;
    }
  }

  complete = 0;
  */

  Serial.println();
  b = serialYesNoPrompt(F("Configure sensor measurement intervals (y/n)?"),true,false);
  if (!b) return 1;

  Serial.println();
  Serial.println(F("Enter the number of seconds between measurements for each sensor,"));
  Serial.println(F("or '0' to disable the sensor.  Current settings are shown in"));
  Serial.println(F("square brackets (press enter to keep the current setting)."));
  Serial.println();
  updateSensorTime(F("Temperature/humidity"),&(storage.humidityT));
  updateSensorTime(F("Radiant temperature"),&(storage.tempT));
  updateSensorTime(F("Light"),&(storage.lightT));
  updateSensorTime(F("Sound level"),&(storage.soundT));
  updateSensorTime(F("Particulate matter"),&(storage.pmT));
  updateSensorTime(F("Carbon dioxide"),&(storage.co2T));
  updateSensorTime(F("Carbon monoxide"),&(storage.coT));
  //updateSensorTime(F("Data upload interval"),&(storage.uploadT));
  Serial.println();

  return 1;

  /*
  Serial.println(F("\nDo you wish to configure timer settings? [Y]/[N]"));
  while (!complete) {
    if (Serial.available()) {
      serial = (Serial.read());
      if (serial == 121 || serial == 89) { // Y or y entered
        complete  = 1;
      }
      else if (serial == 110 || serial == 78){ // N or n entered
        return 0;
      }
      else {
        Serial.println(F("Invalid choice. Please select [Y]es or [N]o"));
      }
    }
  }
  */

  //******************** Timer Settings *********************//
  /*
  Serial.println(F("\nAll intervals are in seconds. Enter '0' to disable sensor."));

  complete = 0;

  Serial.print(F("\nCurrent database upload rate is "));
  Serial.println(storage.uploadT);
  Serial.println(F("What would you like the new upload rate to be?"));
  while(!complete) {
    if(Serial.available()){
      timer = Serial.parseInt();
      if (timer > 0) {
        storage.uploadT = timer;
        complete = 1;
      } else {
        Serial.println(F("Not a valid interval."));
      }
    }
  }

  Serial.print(F("Database upload rate is now "));
  Serial.println(storage.uploadT);

  updateTimer("light");
  updateTimer("humidity");
  updateTimer("temperature");
  updateTimer("sound");
  updateTimer("CO2");
  updateTimer("particle");
  updateTimer("CO");

  return 1;
  */
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

void updateTimer(String sensor){
  int data = 0;
  int timer = 0;

  if (sensor == "light") {
    timer = storage.lightT;
  } else if (sensor == "humidity") {
    timer = storage.humidityT;
  } else if (sensor == "temperature") {
    timer = storage.tempT;
  } else if (sensor == "sound") {
    timer = storage.soundT;
  } else if (sensor == "CO2") {
    timer = storage.co2T;
  } else if (sensor == "particle") {
    timer = storage.pmT;
  } else if (sensor == "CO") {
    timer = storage.coT;
  } else {
    Serial.println(F("Not a valid sensor"));
  }

  Serial.println("\nCurrent " + sensor + " sensor measurement interval is " + timer + " seconds.");
  Serial.println(F("What is the desired measurement interval for the ") + sensor + " sensor?");

  while(!data) {
    if(Serial.available()){
      timer = Serial.parseInt();
      if (sensor == "light") {
        storage.lightT = timer;
      } else if (sensor == "humidity") {
        storage.humidityT = timer;
      } else if (sensor == "temperature") {
        storage.tempT = timer;
      } else if (sensor == "sound") {
        storage.soundT = timer;
      } else if (sensor == "CO2") {
        storage.co2T = timer;
      } else if (sensor == "particle") {
        storage.pmT = timer;
      } else if (sensor == "CO") {
        storage.coT = timer;
      } else {
        Serial.println(F("Not a valid sensor"));
      }
      data = 1;
    }
  }

  if (sensor == "light") {
    timer = storage.lightT;
  } else if (sensor == "humidity") {
    timer = storage.humidityT;
  } else if (sensor == "temperature") {
    timer = storage.tempT;
  } else if (sensor == "sound") {
    timer = storage.soundT;
  } else if (sensor == "CO2") {
    timer = storage.co2T;
  } else if (sensor == "particle") {
    timer = storage.pmT;
  } else if (sensor == "CO") {
    timer = storage.coT;
  }
  Serial.println("The " + sensor + " sensor measurement interval is now " + timer + " seconds.");
}

PodConfigStruct& getPodConfig() {
  return storage;
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


//----------------------------------------------------------------------
