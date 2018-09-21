
/*
 * SensorPod_FW  
 * 2017 - Nick Turner and Morgan Redfield
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

// Project Files
#include "pod_util.h"
#include "pod_config.h"
#include "pod_sensors.h"
#include "pod_network.h"
#include "pod_logging.h"

//--------------------------------------------------------------------------------------------- [Default Configs and Variables]

#define LED_PIN LED_BUILTIN

//--------------------------------------------------------------------------------------------- [setup]

void setup() {
  // Alternative to F() macro wrapping so we can reuse this string
  const char LINE[] PROGMEM = "------------------------------------------------------------------------";
  
  // Time delay gives chance to connect a terminal after reset
  delay(5000);
  Serial.begin(9600);
  
  // Compilation info
  Serial.println(F("PODD firmware starting...."));
  // Get rid of any leading directories in file name
  //Serial.println(F("  Compilation file: "  __FILE__));
  if (strrchr(__FILE__,'\\') != NULL) {
    Serial.print(F("  Compilation file: "));
    Serial.println(strrchr(__FILE__,'\\')+1);
  } else if (strrchr(__FILE__,'/') != NULL) {
    Serial.print(F("  Compilation file: "));
    Serial.println(strrchr(__FILE__,'/')+1);
  } else {
    Serial.println(F("  Compilation file: "  __FILE__));
  }
  Serial.println(F("  Compilation date: "  __DATE__ " " __TIME__));
  Serial.println(F("  Compiler version: " __VERSION__));
  Serial.print(  F("  Arduino version:  "));
  Serial.println(ARDUINO,DEC);
  Serial.println();
  delay(1000);

  #ifdef PM_TESTING
  testPMSensor(-1,30000,30000);
  #endif

  Serial.println(LINE);
  Serial.println(F("Starting setup...."));
  delay(2000);

  Serial.println(F("Setting up XBee...."));
  initXBee();
  
  Serial.println(F("Setting up I2C...."));
  Wire.begin();
  
  Serial.println(F("Setting up RTC...."));
  setupRTC();

  // Ensure LED is off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println(F("Setting up sensors...."));
  sensorSetup();
  if (verifySensors()) {
	  digitalWrite(LED_PIN, HIGH);
  }

  Serial.println(F("Setting up SD...."));
  setupPodSD();

  Serial.println(F("Setting up ethernet...."));
  ethernetSetup();
  
  Serial.println();
  Serial.println(LINE);
  Serial.println();
  
  loadPodConfig();
  podIntro();

  Serial.println(F("Starting SD logging...."));
  setupSDLogging();
  
  Serial.println(F("Starting sensor timers...."));
  setupSensorTimers();

  // power optimizations
  if (getRatePM() <= 0) {
    Serial.println(F("Powering down particulate matter sensor...."));
    digitalWrite(PM_ENABLE, LOW);
  }
  if(! getModeCoord()){
    if(getRatePM() > 120) {
      Serial.println(F("Powering down particulate matter sensor until next reading...."));
      digitalWrite(PM_ENABLE, LOW);
    }

    //Disable Ethernet for Drones
    Serial.println(F("Powering down ethernet...."));
    digitalWrite(ETHERNET_EN, LOW);

    #define SERIAL_DEBUG
    #if defined(SERIAL_DEBUG)
      Serial.println(F("DEBUG: Drone serial output will not be disabled."));
    #else
      Serial.println(F("Powering down USB...."));
      Serial.println(F("Serial output will now end."));
      Serial.println();
      Serial.println(LINE);
      Serial.println();
      Serial.flush();
      Serial.end();
      USBCON |= (1<<FRZCLK); // Disable USB to save power.
    #endif
  }
  
  //Serial.println(F("DEBUG: Ethernet disabled."));
  //digitalWrite(ETHERNET_EN, LOW);
  
  // turn off LED to save power
  // by the time we get here, the user has either configured the SensorPod
  // or it's been around a minute and a half and the setup has timed out
  digitalWrite(LED_PIN, LOW);

  // Begin background process to pull data from the XBee for later
  // processing.  Currently only necessary for the coordinator.
  if(getModeCoord()){
    Serial.println(F("Starting coordinator XBee monitoring process...."));
    startXBee();
  }
  
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
  // digitalWrite(CP, HIGH);
  // checks to see if start date and time have passed or not:
  if(ethernetOnline() && getModeCoord())
    ethernetMaintain();

  handleLoopLogging();
}
