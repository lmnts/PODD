/*==============================================================================
  Various eeprom-related constants and functions.

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
//#include <Arduino.h>
//#include <EEPROM.h>
// Local headers

// Define DEBUG, PODD_DEBUG, or PODD_EEPROM_DEBUG to enable
// debug statements below
#if defined(DEBUG) && !defined(PODD_DEBUG)
#define PODD_DEBUG
#endif
#if defined(PODD_DEBUG) && !defined(PODD_EEPROM_DEBUG)
#define PODD_EEPROM_DEBUG
#endif


// Constants/global variables ==================================================

// Define EEPROM memory locations in one location here to avoid accidental
// clashing.  Teensy++ 2.0 has 4096 bytes of EEPROM [0x0000 - 0x0FFF].
// Constants below are for starting address of memory blocks.

// Location of configuration data [0x0020 - 0x01FF].
// Currently uses ~ 200 bytes, but leaving space for future
// expansion.
#define EEPROM_CONFIG_ADDR 0x0020
// Location of clock data [0x0400 - 0x0479].
// Currently contains timezone information.
#define EEPROM_CLOCK_ADDR 0x0400
// Location of network configuration data [0x0500 - 0x0579].
// Currently contains timezone information.
#define EEPROM_NETWORK_ADDR 0x0500


// Functions ===================================================================

// No functions


//==============================================================================
