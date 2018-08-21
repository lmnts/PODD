
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

#define LED_PIN 6

//--------------------------------------------------------------------------------------------- [setup]

void setup() {
  delay(5000);
  Serial.begin(9600);
  Serial.println(F("starting setup"));
  delay(2000);

  xbeeSetup();
  Wire.begin(); // for humidity sensor

  setupRTC();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println(F("setting up sensors"));

  sensorSetup();
  if (verifySensors()) {
	  digitalWrite(LED_PIN, HIGH);
  }

  Serial.println(F("setting up SD and ethernet"));

  setupPodSD();

  ethernetSetup();

  loadPodConfig();
  podIntro();

  Serial.println(F("XBee configured"));

  setupSDLogging();

  Serial.println(F("SD logging set up"));

  setupSensorTimers();

  // power optimizations
  if(! getModeCoord()){
    if(getRatePM() > 120) {
      digitalWrite(PM_ENABLE, LOW);
    }

    //Disable Ethernet for Drones
    digitalWrite(ETHERNET_EN, LOW);

    Serial.println(F("powering down USB"));
    Serial.end();
    USBCON |= (1<<FRZCLK); // Disable USB to save power.
  }
  EIFR |= (1<<INTF2); // Clear INT2 flag
  EIMSK |= (1<<INT2); // Enable INT2 (same pin as USART Rx)
  
  // turn off LED to save power
  // by the time we get here, the user has either configured the SensorPod
  // or it's been around a minute and a half and the setup has timed out
  digitalWrite(LED_PIN, LOW);
  
  #ifdef DEBUG
  writeDebugLog(F("Fxn: setup()"));
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
