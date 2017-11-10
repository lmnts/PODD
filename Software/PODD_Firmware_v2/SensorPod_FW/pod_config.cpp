
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
#include "pod_sensors.h"
#include "pod_network.h"
#include "pod_logging.h"

#include "EEPROM.h"

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
      if (choice == 121 || choice == 89) { // Y or y entered
        while(!custom){
          podConfig();
          Serial.println(F("\nIs current configuration acceptable? [Y/N]"));
          while(!finished){
            if(Serial.available()) {
              finished=Serial.read();
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
  Serial.print(F("Network Code: "));
  Serial.println(storage.networkID);
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
  updateRate(storage.devid, "Light", storage.lightT, Datetime);
  delay(100);
  updateRate(storage.devid, "Humidity", storage.humidityT, Datetime);
  delay(100);
  updateRate(storage.devid, "GlobalTemp", storage.tempT, Datetime);
  delay(100);
  updateRate(storage.devid, "Sound", storage.soundT, Datetime);
  delay(100);
  updateRate(storage.devid, "CO2", storage.co2T, Datetime);
  delay(100);
  updateRate(storage.devid, "Particle", storage.pmT, Datetime);
  delay(100);
  updateRate(storage.devid, "CO", storage.coT, Datetime);

  if (storage.coord == 'Y') {
    writeSDConfig(storage.devid, storage.room, "Coordinator", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
    updateConfig(storage.devid, storage.room, "Coordinator", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
  } else if (storage.coord == 'N') {
    writeSDConfig(storage.devid, storage.room, "Drone", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
    updateConfig(storage.devid, storage.room, "Drone", storage.project, storage.uploadT, storage.setupD, storage.teardownD, Datetime, storage.networkID);
  } else {
    Serial.println(F("Coordinator setting invalid. ID, Project, Room, Upload Rate, Coordinator Status, Setup Date, and Teardown Date not sent to DB."));
  }
}

int podConfig() {
  int serial;
  int complete = 0;
  unsigned int roomLen, serverLen, idLen, projLen, dateLen, netLen;
  int timer;
  
  Serial.println(F("Proceeding with Pod configuration."));

  //******************** Pod Settings ************************// 
  Serial.print(F("\nThis Pod will upload to the server \""));
  Serial.print(storage.server);
  Serial.println('"');
  Serial.println(F("Do you want to change the server? [Y]/[N]"));

  while (!complete) {
    if (Serial.available()) {
      serial = (Serial.read());
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

  //******************** Timer Settings *********************//
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

