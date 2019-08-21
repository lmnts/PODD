/*==============================================================================
  Constants and functions related to the serial interface menu.

  This file is part of the LMN PODD distribution:
    https://github.com/lmnts/PODD

  CONTRIBUTORS:
    Chris Savage (2019)

  COPYRIGHT/LICENSE:
  Copyright (c) 2019 LMN Architects

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Affero General Public License for more details.

  You should have received a copy of the GNU Affero General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

==============================================================================*/

#pragma once

// Standard libraries
// Contributed libraries
#include <Arduino.h>
// Local headers

// Define DEBUG, PODD_DEBUG, or PODD_MENU_DEBUG to enable
// debug statements below
#if defined(DEBUG) && !defined(PODD_DEBUG)
#define PODD_DEBUG
#endif
#if defined(PODD_DEBUG) && !defined(PODD_MENU_DEBUG)
#define PODD_MENU_DEBUG
#endif


// Constants/global variables ==================================================


// Functions ===================================================================

// Provides countdown to entering automatic running mode over serial
// interface; enters interactive menu if interrupted by user response.
void interactivePrompt(unsigned long timeout=30000);

// Main PODD menu
void mainMenu();
// Routines to show information within the main menu
void showMenuProjectSettings();
void showMenuNodeSettings();
void showMenuSensorTimingEntry(String s, int v);
void showMenuSensorTimingSettings();
void showMenuNetworkSettings();
void showMenuXBeeSettings();
void showMenuClockSettings();

// Interactive prompts to configure various settings
void configureProjectSettings();
void configureNodeSettings();
void configureSensorTimingSettings();
void configureNetworkSettings();
void configureXBeeSettings();
void configureClockSettings();
void configureDebugSettings();

// Sensor menu
void sensorMenu();
// Routines to respond to sensor menu selections
void sensorMenuCalibrateCO2Sensor();
void sensorMenuTogglePMSensor();
void sensorMenuCleanPMSensor();


//==============================================================================
