/*
 * SensorPod_FW  
 * 2017 - Nick Turner and Morgan Redfield
 * 2018 - Chris Savage
 * 
 * This sketch is intended for use with the LMN Post-Occupancy
 * SensorPods. It will sample each of several sensors at a
 * configurable rate. Those samples will be stored locally to
 * an SD card, and also uploaded to a remote server.
 * 
 * Uploading to the cloud is accomplished through a mesh network
 * controlled by on-board XBees. One (and only one) of the
 * SensorPods should be a coordinator connected to Ethernet. The
 * other SensorPods will transmit data wirelessly to the
 * coordinator.
 * 
 * Please note that the SensorPod hardware is intended for use with
 * a Teensy++ 2.0 running at 3.3V. At that voltage, 16MHz is too
 * fast for the processor. Set the CPU speed to 8MHz before
 * compiling and running the sketch.
 * 
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

// Ensure compilation set for 8 MHz CPU speed.
// Can be set on Arduino IDE under Tools -> CPU Speed.
#if defined(F_CPU) && (F_CPU != 8000000)
#error "CPU speed must be set to 8 MHz"
#endif

#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Project Files
#include "pod_util.h"
#include "pod_serial.h"
#include "pod_clock.h"
#include "pod_config.h"
#include "pod_menu.h"
#include "pod_sensors.h"
#include "pod_network.h"
#include "pod_logging.h"

//--------------------------------------------------------------------------------------------- [Default Configs and Variables]

#define LED_PIN LED_BUILTIN

//--------------------------------------------------------------------------------------------- [setup]

void setup() {
  // Places string in flash memory rather than dynamic memory
  FType LINE = F("------------------------------------------------------------------------");
  
  // Time delay gives chance to connect a terminal after reset
  delay(5000);
  Serial.begin(9600);
  
  // Compilation info
  Serial.println(F("PODD firmware starting...."));
  printCompilationInfo("  ",__FILE__);
  Serial.println();
  delay(1000);
  
  #ifdef CLOCK_TESTING
  testClock(-1,1000);
  #endif
  
  #ifdef SENSOR_TESTING
  Wire.begin();
  // Sound sensor testing
  testSoundSensor(-1,1000);
  // Temperature/humidity sensor testing
  testTemperatureSensor(-1,1000);
  // Particulate matter sensor testing
  testPMSensor(-1,5000,10000,10000);
  #endif
  
  Serial.println(LINE);
  Serial.println(F("Starting setup...."));
  delay(2000);
  
  Serial.println(F("Setting up XBee...."));
  initXBee();
  Serial.print(F("  Serial number: "));
  Serial.println(getXBeeSerialNumberString());
  
  Serial.println(F("Setting up I2C...."));
  Wire.begin();
  
  Serial.println(F("Setting up RTC...."));
  initRTC();
  Serial.print(F("  Current date/time: "));
  Serial.println(getLocalDateTimeString());
  
  // Ensure LED is off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println(F("Setting up sensors...."));
  initSensors();
  printSensorCheck();
  Serial.println(F("Testing sensors (10 seconds)...."));
  testSensors(10,1000);
  //testSensors(-1,1000);
  
  //serialCharPrompt(F("DEBUG: Waiting for keypress"));
  
  // SD uses > 100 mA when initializing/writing, but < 1 mA when idle
  Serial.println(F("Setting up SD...."));
  setupPodSD();
  
  // Ethernet uses ~ 150 mA when powered.
  // Should initialize XBee first (uses XBee SN for MAC address).
  Serial.println(F("Setting up ethernet...."));
  ethernetSetup();
  
  Serial.println();
  Serial.println(LINE);
  Serial.println();
  
  Serial.println(F("Loading PODD configuration...."));
  loadPodConfig();
  Serial.print(F("  Device:  "));
  Serial.println(getDevID());
  Serial.print(F("  Project: "));
  Serial.println(getProject());

  // Prompt for interactive menu, proceed to menu if user responds.
  // Times out in ~30 seconds if no response.
  interactivePrompt();
  
  Serial.println(F("Configuring XBee...."));
  configureXBee(getModeCoord());
  Serial.print(F("  Coordinator:   "));
  Serial.println(getModeCoord() ? "yes" : "no");
  Serial.print(F("  Serial number: "));
  Serial.println(getXBeeSerialNumberString());
  Serial.print(F("  Destination:   "));
  Serial.println(getXBeeDestinationString());

  // Try again to connect coordinator to network, if not
  // currently connected.
  if (getModeCoord()) {
    if (!ethernetConnected()) {
      Serial.print(F("Re-attempting to connect to the network.... ("));
      ethernetBegin(3);
    }
    //ethernetMaintain();
    if (!ethernetConnected()) {
      Serial.println(F("Internet connection could not be established.  Readings will not be"));
      Serial.println(F("pushed to remote database until connection can be established."));
    }
  }
  
  // Save and upload to database the current PODD configuration.
  // Even if the configuration has not changed, these logs will
  // include a useful timestamp indicating when the PODD started.
  savePodConfig();
  // Can optionally only save/upload config if it changed since
  // last time it was logged.
  //if (podConfigChanged()) savePodConfig();

  // Upload PODD sensor reading rates, only if they have changed
  // (this is slow due to numerous network packets being sent,
  // so we do not do this every time).
  if (podRatesChanged()) savePodRates();
  
  // Begin background process to pull data from the XBee for later
  // processing.  Used by coordinator to buffer packets arriving from
  // other nodes until they can be sent to the database over the internet.
  // Used by drones to buffer clock syncing packets (the delay in
  // processing means the clock may be off by a few seconds relative to
  // the coordinator).
  // NOTE: On the coordinator, various sensor-processing routines
  // (notably those for sound and CO2) might occasionally cause an
  // XBee bus character to be missed, corrupting a packet that will get
  // passed onto the database.  Though those sensor routines have been
  // redesigned to greatly reduce that possibility, it is unclear at
  // this time whether the corruption rate is so low as to be ignorable.
  // If safety is desired, avoid reading some/all of the sensors on the
  // coordinator node.
  Serial.println(F("Starting XBee monitoring process...."));
  startXBee();
  
  Serial.println(F("Starting SD logging...."));
  setupSDLogging();
  
  Serial.println(F("Starting sensor timers...."));
  setupSensorTimers();

  if (getModeCoord()) {
    Serial.println(F("Starting network timers...."));
    setupNetworkTimers();
  }
  
  // power optimizations
  // Sensor power handling is in setupSensorTimers()
  if(!getModeCoord()){
    //Disable Ethernet for Drones
    Serial.println(F("Powering down ethernet...."));
    digitalWrite(ETHERNET_EN, LOW);
    
    if (getDebugMode()) {
      Serial.println(F("DEBUG: Drone serial output will not be disabled."));
    } else {
      Serial.println(F("Powering down USB...."));
      Serial.println(F("Serial output will now end."));
      Serial.println();
      Serial.println(LINE);
      Serial.println();
      Serial.flush();
      Serial.end();
      USBCON |= (1<<FRZCLK); // Disable USB to save power.
    }
  }
  
  // turn off LED to save power
  // by the time we get here, the user has either configured the SensorPod
  // or it's been around a minute and a half and the setup has timed out
  digitalWrite(LED_PIN, LOW);
  
  Serial.println();
  Serial.println(F("Initialization and setup complete.  The PODD will now begin taking data."));
  Serial.println();
  Serial.println(LINE);
  Serial.println();
  
  #ifdef DEBUG
  //writeDebugLog(F("Fxn: setup()"));
  #endif
  
  sei(); //Enable interrupts
}


//--------------------------------------------------------------------------------------------- [loop]
void loop() {
  // Check ethernet connection.  Reinitialize if necessary.
  if(getModeCoord()) {
    ethernetMaintain();
  }

  // Sample sensors, log data to SD, upload to server, etc.
  handleLoopLogging();
}
