
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
 
#include "pod_util.h"
#include "pod_config.h"
#include "pod_sensors.h"
#include "pod_network.h"
#include "pod_logging.h"

#include "EEPROM.h"

#include <stdlib.h>
#include <limits.h>

struct StoreStruct {
  char pod_version[5], server[61], devid[17], project [17], room[17], setupD[11], teardownD[11], lastUpdate[20], networkID[5];
  char coord; // 
  int uploadT,lightT,humidityT,tempT,soundT,co2T,pmT,coT;
} storage = {
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

// Alternative to F() macro wrapping so we can reuse these strings
//const char INDENT2[] PROGMEM = "        ";
//const char CONTINUE_PROMPT[] PROGMEM = "(hit any key to continue)";
// PROGMEM not working correctly...
const char INDENT2[] = "        ";
const char CONTINUE_PROMPT[] = "(hit any key to continue)";


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
  String Datetime = getStringDatetime();
  Datetime.toCharArray(storage.lastUpdate, 20);
  
  // Save to EEPROM
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
  
  // Save to DB - Rates
  // If these are sent too fast over XBee network by non-coordinators,
  // there may be buffer overflows and/or packet loss
  const unsigned long delta = (storage.coord == 'Y') ? 250 : 1000;
  updateRate(storage.devid, "Light", storage.lightT, Datetime);
  delay(delta);
  updateRate(storage.devid, "Humidity", storage.humidityT, Datetime);
  delay(delta);
  updateRate(storage.devid, "GlobalTemp", storage.tempT, Datetime);
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
  configChanged = true;
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

void updateSensorTime(String label, int *v) {
  int i = serialIntegerPrompt(String("") + label,true,*v);
  if (i < 0) i = 0;
  if (i != *v) {
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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
    configChanged = true;
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

void loadPodConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2] &&
      EEPROM.read(CONFIG_START + 3) == CONFIG_VERSION[3])
    for (unsigned int t=0; t<sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
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


// Serial interaction --------------------------------------------------

/* Removes any data from the serial interface. */
void clearSerial() {
  while (Serial.available()) Serial.read();
}


/* Returns the first character available on the serial interface,
   optionally waiting the given amount of time [ms] for data to
   become available.  Remaining serial data is cleared.  If timeout
   is reached before data becomes available, -1 is returned.
   A timeout of -1 can be given to wait indefinitely for data. */
char getSerialChar(unsigned long timeout) {
  unsigned long t0 = millis();
  do {
    if (Serial.available()) {
      char c = Serial.read();
      // Clear remaining serial data (including any end-of-line
      // characters).  We give a little time for multi-character
      // serial data to make it across the bus (not complete
      // until serial buffer remains empty for 1 ms).
      // Double while loops are not redundant...
      delay(1);
      while (Serial.available()) {
        while (Serial.available()) Serial.read();
        delay(1);
      }
      return c;
    }
    delay(1);
  } while (millis() - t0 <= timeout);
  return -1;
}


/* Returns data available on the serial interface up to a newline
   character, optionally waiting the given amount of time [ms] for
   data to become available.  Remaining serial data is cleared.
   If timeout is reached before data becomes available, an empty
   string is returned.  A timeout of -1 can be given to wait
   indefinitely for data. */
String getSerialString(unsigned long timeout) {
  String s = "";
  bool complete = false;
  unsigned long t0 = millis();
  // Parse serial until end-of-line character found
  // or timeout reached.
  do {
    while (!complete && Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case '\r':
          complete = true;
          break;
        case '\n':
          complete = true;
          break;
        default:
          s = s + c;
          break;
      }
    }
    if (complete) break;
    delay(1);
  } while (millis() - t0 <= timeout);

  // Clear remaining serial data (including any end-of-line
  // characters).  We give a little time for multi-character
  // serial data to make it across the bus (not complete
  // until serial buffer remains empty for 1 ms).
  if (complete) {
    delay(1);
    // Double while loops are not redundant...
    while (Serial.available()) {
      while (Serial.available()) Serial.read();
      delay(1);
    }
  }
  
  return s;
}


/* Retrieves and parses serial data to an integer value.
   Returns smallest integer value (INT_MIN from <limits.h>) if
   data is empty except for newline character.  Can optionally
   reprompt for user input if data is non-empty, but invalid. */
int getSerialInt(bool reprompt) {
  const int EMPTY_VALUE = INT_MIN;
  while (true) {
    String s = getSerialString(-1);
    s.trim();
    // If provided input is empty, return smallest integer value
    // (our empty indicator value)
    if (s.equals("")) return EMPTY_VALUE;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      // coerce float value to integer range
      if (f > INT_MAX) return INT_MAX;
      if (f <= INT_MIN+1) return INT_MIN + 1;
      return (int)f;
    }
    if (!reprompt) return EMPTY_VALUE;
    Serial.println(s);  // Serial Monitor does not echo inputs
    Serial.print(F("Invalid input.  Try again: "));
  }
  // Should not reach here...
  return EMPTY_VALUE;
}


/* Retrieves and parses serial data to a float value.
   Returns NAN (from <math.h>) if
   data is empty except for newline character.  Can optionally
   reprompt for user input if data is non-empty, but invalid. */
float getSerialFloat(bool reprompt) {
  const float EMPTY_VALUE = NAN;
  while (true) {
    String s = getSerialString(-1);
    s.trim();
    // If provided input is empty, return smallest integer value
    // (our empty indicator value)
    if (s.equals("")) return EMPTY_VALUE;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return f;
    }
    if (!reprompt) return EMPTY_VALUE;
    Serial.println(s);  // Serial Monitor does not echo inputs
    Serial.print(F("Invalid input.  Try again: "));
  }
  // Should not reach here...
  return EMPTY_VALUE;
}


/* Retrieves a single character from an interactive serial
   interface, with the given prompt provided to the user.
   A default value can be specified, used if an empty
   response is given. */
char serialCharPrompt(String prompt, char default0) {
  // Provide prompt to user
  Serial.print(prompt);
  if (default0 != (char)(-1)) {
    Serial.print(F(" ["));
    Serial.print(default0);
    Serial.print(F("]"));
  }
  Serial.print(F(": "));
  // Get user's response
  char c = getSerialChar(-1);
  //Serial.println();
  Serial.println(c);  // Serial Monitor does not echo inputs
  // Empty response: use default
  if ((c == '\r') || (c == '\n')) return default0;
  return c;
}


/* Retrieves a string from an interactive serial interface,
   with the given prompt provided to the user.
   A default value can be specified, used if an empty
   response is given. */
String serialStringPrompt(String prompt, String default0) {
  // Provide prompt to user
  Serial.print(prompt);
  if (!default0.equals("")) {
    Serial.print(F(" [\""));
    Serial.print(default0);
    Serial.print(F("\"]"));
  }
  Serial.print(F(": "));
  // Get user's response
  String s = getSerialString(-1);
  //Serial.println();
  Serial.println(s);  // Serial Monitor does not echo inputs
  s.trim();
  // Empty response: use default
  if (s.equals("")) return default0;
  return s;
}


/* Retrieves a boolean response from an interactive serial 
   interface, with the given prompt provided to the user.
   The user can optionally be reprompted if an invalid
   response is received.
   A default value must be specified, used if an empty
   response is given. */
bool serialYesNoPrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'y','n');
}
bool serialTrueFalsePrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'t','f');
}
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0) {
  return serialBooleanPrompt(prompt,reprompt,default0,'t','f');
}

/* Main routine for above serial boolean prompt routines.
   Allows prompt's true/false characters to be specified,
   but this applies only to the prompt strings, not to the
   allowed input (hardcoded to look for 't'/'y' or 'f'/'n'
   as first character, case-insensitive).
   This routine is not intended to be called by anything
   other than the above routines (use those instead). */
bool serialBooleanPrompt(String prompt, bool reprompt, bool default0, char tchar, char fchar) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    Serial.print(F(" ["));
    Serial.print(default0 ? (char)toupper(tchar) : (char)toupper(fchar));
    Serial.print(F("]"));
  Serial.print(F(": "));
    // Get user's response
    char c = getSerialChar(-1);
    //Serial.println();
    Serial.println(c);  // Serial Monitor does not echo inputs
    // Process response
    switch (tolower(c)) {
      // true/yes
      case 't':
      case 'y':
        return true;
        break;
      // false/no
      case 'f':
      case 'n':
        return false;
        break;
      // Empty response: use default
      case '\r':
      case '\n':
        return default0;
        break;
    }
    if (!reprompt) return default0;
    Serial.print(F("  Invalid input (use '"));
    Serial.print(tchar);
    Serial.print(F("' or '"));
    Serial.print(fchar);
    Serial.print(F("'."));
    Serial.println();
  }
  // Should not reach here...
  return default0;
}


/* Retrieves a number from an interactive serial interface,
   with the given prompt provided to the user.  The user
   can optionally be reprompted if a non-numeric response
   is received.
   A default value can be specified, used if an empty
   response is given. */
int serialIntegerPrompt(String prompt, bool reprompt, int default0) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    if (default0 != INT_MIN) {
      Serial.print(F(" ["));
      Serial.print(default0);
      Serial.print(F("]"));
    }
  Serial.print(F(": "));
    // Get user's response
    String s = getSerialString(-1);
    //Serial.println();
    Serial.println(s);  // Serial Monitor does not echo inputs
    s.trim();
    // Empty response: use default
    if (s.equals("")) return default0;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //long l = strtol(buff,&end,10);
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // *end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      // coerce float value to integer range
      if (f >= INT_MAX) return INT_MAX;
      if (f <= INT_MIN) return INT_MIN;
      return (int)f;
    }
    if (!reprompt) return default0;
    Serial.println(F("  Invalid input (not numeric)."));
  }
  // Should not reach here...
  return default0;
}


/* Retrieves a number from an interactive serial interface,
   with the given prompt provided to the user.  The user
   can optionally be reprompted if a non-numeric response
   is received.
   A default value can be specified, used if an empty
   response is given. */
float serialFloatPrompt(String prompt, bool reprompt, float default0) {
  while (true) {
    // Provide prompt to user
    Serial.print(prompt);
    if (!isnan(default0)) {
      Serial.print(F(" ["));
      Serial.print(default0);
      Serial.print(F("]"));
    }
  Serial.print(F(": "));
    // Get user's response
    String s = getSerialString(-1);
    //Serial.println();
    Serial.println(s);  // Serial Monitor does not echo inputs
    s.trim();
    // Empty response: use default
    if (s.equals("")) return default0;
    // Parse string to number.
    // Standard Arduino routines do not include means to check
    // for invalid input, so we use following instead.
    const char *buff = s.c_str();
    char *end;
    //float f = strtof(buff,&end);  // Not available for Arduino/teensy?
    double f = strtod(buff,&end);
    // *end points to next character in string after successfully
    // parsed characters, or beginning of string if nothing
    // successfully parsed.
    if (end != buff) {
      return (float)f;
    }
    if (!reprompt) return default0;
    Serial.println(F("  Invalid input (not numeric)."));
  }
  // Should not reach here...
  return default0;
}


// Interactive menus ---------------------------------------------------

/* For up to the given amount of time [ms], will provide (over the 
   serial interface) regular prompts to press a key, in which case,
   a menu-based interactive mode will be started.  Otherwise, the
   PODD will start running in automatic mode. */
void interactivePrompt(unsigned long timeout) {
  Serial.println();

  // Clear serial input
  clearSerial();
  
  // Check for interaction
  bool interactive = false;
  for (int k = (timeout/1000); k > 0; k--) {
    Serial.print(F("PODD will start in ")); 
    Serial.print(k);
    Serial.print(F(" seconds.  Press any key to enter interactive mode."));
    Serial.println();
    if (getSerialChar(1000) != (char)(-1)) {
      interactive = true;
      break;
    }
  }
  
  if (interactive) mainMenu();
  
  Serial.println(F("Now beginning automated mode."));
}


/* Main menu for interactive mode.  Will continue to be shown until
   an appropriate selection is made. */
void mainMenu() {
  
  // Will continue offering main menu until appropriate response
  // received.
  while (true) {
    // Clear terminal by printing a bunch of blank lines
    for (int k = 0; k < 100; k++) Serial.println();
    // Print menu options
    Serial.println();
    //Serial.println(F("======== PODD MENU ====================================================="));
    //Serial.println(F("<<<<<<<< PODD MENU >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
    Serial.println(F("========================================================================"));
    Serial.println(F("|        PODD MENU                                                     |"));
    Serial.println(F("========================================================================"));
    Serial.println();
    // Project-related settings
    Serial.println(F("  (1) Project settings"));
    showMenuProjectSettings();
    /*
    Serial.print(INDENT2);
    Serial.print("Project: " + String(getProject()));
    Serial.println();
    Serial.print(INDENT2);
    Serial.print("Server:  " + String(getServer()));
    Serial.println();
    */
    // Node-related settings
    Serial.println(F("  (2) Node settings"));
    showMenuNodeSettings();
    /*
    Serial.print(INDENT2);
    Serial.print("Device ID:   " + String(getDevID()));
    Serial.println();
    Serial.print(INDENT2);
    Serial.print("Location:    " + String(getRoom()));
    Serial.println();
    Serial.print(INDENT2);
    Serial.print("Coordinator: " + String(getModeCoord() ? "Yes" : "No"));
    Serial.println();
    */
    // Sensor-related settings
    //Serial.println(F("  (-) Sensor settings"));
    Serial.println(F("  (3) Sensor timing"));
    showMenuSensorTimingSettings();
    Serial.println(F("  (4) Sensor testing"));
    // RTC time/settings
    Serial.println(F("  (C) Clock settings"));
    showMenuClockSettings();
    /*
    Serial.println(F("  (T) RTC"));
    Serial.print(INDENT2);
    Serial.print(F("Current time: "));
    Serial.print(formatDateTime());
    Serial.println();
    //Serial.print(  F("  (T) RTC time ["));
    //Serial.print(formatDateTime());
    //Serial.print(F("]"));
    //Serial.println();
    */
    // Show compilation info
    Serial.println(F("  (I) Compilation info"));
    //Serial.println(F("  "));
    Serial.println();
    // Enter old configuration menu
    Serial.println(F("  (O) Old configuration menu"));
    Serial.println();
    // Leave menu and enter automatic mode
    Serial.println(F("  (Q) Quit interactive mode"));
    Serial.println();
    //Serial.println(F("                        ------------------------                        "));
    Serial.println(F("        ----------------                                                "));
    //Serial.println(F("========================================================================"));
    //Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));

    // Get selection
    clearSerial();
    Serial.println();
    Serial.print("Enter the item to view/configure: ");
    char c = getSerialChar(-1);
    Serial.println(c);  // Serial Monitor does not echo inputs
    Serial.println();

    // Process selection
    // We commit any configuration changes when leaving
    // the menu or going to an option that we might not
    // return from (e.g. sensor testing).
    bool showContinuePrompt = true;
    switch (c) {
      case '1':
        configureProjectSettings();
        break;
      case '2':
        configureNodeSettings();
        break;
      case '3':
        configureSensorTimingSettings();
        break;
      case '4':
        Serial.println(F("<option not yet implemented>"));
        break;
      case '5':
        Serial.println(F("<option not yet implemented>"));
        break;
      case 'C':
      case 'c':
        Serial.println(F("<option not yet implemented>"));
        break;
      case 'I':
      case 'i':
        printCompilationInfo("  ","");
        Serial.println();
        break;
      case 'O':
      case 'o':
        if (podConfigChanged()) savePodConfig();
        podIntro();
        //showContinuePrompt = false;
        break;
      case 'Q':
      case 'q':
        if (podConfigChanged()) savePodConfig();
        showContinuePrompt = false;
        return;
        break;
      default:
        Serial.println(F("Invalid option.  Select an option from the list."));
        break;
    }

    if (showContinuePrompt) {
      //Serial.println();
      Serial.println(CONTINUE_PROMPT);
      getSerialChar(-1);
    }
    
    Serial.println();
  }
}


/* Prints to serial various project settings.
   Intended to be used just below menu's project settings entry. */
void showMenuProjectSettings() {
  Serial.print(INDENT2);
  Serial.print("Project: " + String(getProject()));
  Serial.println();
  Serial.print(INDENT2);
  Serial.print("Server:  " + String(getServer()));
  Serial.println();
}


/* Prints to serial various node settings.
   Intended to be used just below menu's node settings entry. */
void showMenuNodeSettings() {
  Serial.print(INDENT2);
  Serial.print("Device ID:   " + String(getDevID()));
  Serial.println();
  Serial.print(INDENT2);
  Serial.print("Location:    " + String(getRoom()));
  Serial.println();
  Serial.print(INDENT2);
  Serial.print("Coordinator: " + String(getModeCoord() ? "Yes" : "No"));
  Serial.println();
}


/* Prints to serial a single sensor timing setting.
   Intended to be used just below menu's sensor timing settings entry. */
void showMenuSensorTimingEntry(String s, int v) {
  //return String(INDENT2) + s + " " + ((v > 0) ? (String(v) + " s") : "(disabled)");
  Serial.print(INDENT2);
  Serial.print(s + F(": "));
  for (size_t k = 20; k > s.length(); k--) Serial.print(' ');
  if (v > 0) {
    char buff[8];
    sprintf(buff,"%5d s",v);
    Serial.print(buff);
  } else {
    Serial.print(F("(disabled)"));
  }
  Serial.println();
}


/* Prints to serial various sensor timing settings.
   Intended to be used just below menu's sensor timing settings entry. */
void showMenuSensorTimingSettings() {
  showMenuSensorTimingEntry(F("Temperature/humidity"),(storage.humidityT));
  showMenuSensorTimingEntry(F("Radiant temperature"),(storage.tempT));
  showMenuSensorTimingEntry(F("Light"),(storage.lightT));
  showMenuSensorTimingEntry(F("Sound level"),(storage.soundT));
  showMenuSensorTimingEntry(F("Particulate matter"),(storage.pmT));
  showMenuSensorTimingEntry(F("Carbon dioxide"),(storage.co2T));
  showMenuSensorTimingEntry(F("Carbon monoxide"),(storage.coT));
  //showMenuSensorTimingEntry(F("Data upload interval"),(storage.uploadT));
}


/* Prints to serial various clock settings.
   Intended to be used just below menu's clock settings entry. */
void showMenuClockSettings() {
  Serial.print(INDENT2);
  Serial.print(F("Current time: "));
  Serial.print(formatDateTime());
  Serial.println();
}


// Interactive configuration -------------------------------------------

/* Prompt the user to update project settings over the serial interface. */
void configureProjectSettings() {
  String s;
  
  Serial.println();
  Serial.println(F("Current settings are shown in square brackets."));
  Serial.println(F("Press enter to keep the current setting."));
  Serial.println();

  s = serialStringPrompt(F("Upload server"),storage.server);
  if (!s.equals(storage.server)) {
    replaceSettingString(s,storage.server,sizeof(storage.server),F("Server"));
  }
  
  s = serialStringPrompt(F("Project name (max 16 characters)"),storage.project);
  if (!s.equals(storage.project)) {
    replaceSettingString(s,storage.project,sizeof(storage.project),F("Project name"));
  }
  
  s = serialStringPrompt(F("Project setup date (YYY-MM-DD)"),storage.setupD);
  if (!s.equals(storage.setupD)) {
    replaceSettingString(s,storage.setupD,sizeof(storage.setupD),F("Project setup date"));
  }
  
  s = serialStringPrompt(F("Project teardown date (YYY-MM-DD)"),storage.teardownD);
  if (!s.equals(storage.teardownD)) {
    replaceSettingString(s,storage.teardownD,sizeof(storage.teardownD),F("Project teardown date"));
  }
  
  // Use XBee Grove dev board + XCTU exclusively to configure
  // XBee network, aside from coordinator mode: current firmware
  // not set up to provide access to many of the useful settings
  // and it is too easy to misconfigure the XBee. 
  /*
  s = serialStringPrompt(F("Project network ID (hexadecimal from 0000 to 07FFF)"),storage.networkID);
  if (!s.equals(storage.networkID)) {
    replaceSettingString(s,storage.networkID,sizeof(storage.networkID),F("Network ID"));
  }
  */
  
  Serial.println();
}


/* Prompt the user to update node settings over the serial interface. */
void configureNodeSettings() {
  bool b;
  String s;
  
  Serial.println();
  Serial.println(F("Current settings are shown in square brackets."));
  Serial.println(F("Press enter to keep the current setting."));
  Serial.println();

  s = serialStringPrompt(F("Device ID (max 16 characters)"),storage.devid);
  if (!s.equals(storage.devid)) {
    replaceSettingString(s,storage.devid,sizeof(storage.devid),F("Device ID"));
  }
  
  s = serialStringPrompt(F("Room/location (max 16 characters)"),storage.room);
  if (!s.equals(storage.room)) {
    replaceSettingString(s,storage.room,sizeof(storage.room),F("Room/location"));
  }

  bool iscoord = getModeCoord();
  b = serialYesNoPrompt(F("Network coordinator (y/n)"),true,iscoord);
  if (b != iscoord) {
    configChanged = true;
    if (b) {
      storage.coord = 'Y';
      Serial.println(F("  This node now set to network coordinator."));
    } else {
      storage.coord = 'N';
      Serial.println(F("  This node is no longer the network coordinator."));
    }
  }
  
  Serial.println();
}


/* Prompt the user to update sensor measurement timing settings over the
   serial interface. */
void configureSensorTimingSettings() {
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
}



//----------------------------------------------------------------------
