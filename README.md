# Post Occupancy Data Device

![PODD logo](images/logo%20draft%201%20bg.png)

*A tool to understand how buildings perform in terms of occupant comfort.*

*&nbsp;&nbsp;&nbsp;&nbsp;https://lmnarchitects.com/tech-studio/*

The Post Occupancy Data Device (PODD) is a small, networked, multi-sensor device that:

  * addresses the need for more granular data on buildings once they have been occupied
  * offers a lower cost, less precious alternative to standard building enviornmental sensors
  * gives building designers greater control over the tools available to them to understand their buildings
 
PODD is under active development and should be thought of as early alpha quality for both the software and hardware.

- [Features](#features)
- [Hardware](#hardware)
  - [Operational Hardware](#operational-hardware)
  - [Sensors](#sensors)
- [Building](#building)
  - [Hardware Setup](#hardware-setup)
  - [Software Setup](#software-setup)
- [Data Access](#data-access)
- [Known Issues](#known-issues)
- [License](#license)
- [Reporting Issues](#reporting-issues)
- [Contact](#contact)


## Features

The PODD is our solution to the gap in the available post-occupancy evaluation tools. The units are designed to be networked together in large numbers, storing and uploading data from seven sensors that have been identified as most relevant for occupant comfort in buildings.

Each unit has the following features:

  * Air quality sensors: particulate matter (PM2.5 and PM10), CO<sub>2</sub>, CO
  * Thermal comfort sensors: combined temperature and humidity sensor, radiant temperature
  * Illuminance sensor
  * Sound level sensor
  * On board data storage: micro SD card
  * Real Time Clock (RTC)
  * Wireless communication using 802.15.4 standard (900 MHz)
  * Onboard Ethernet 
  * Power via battery, USB, or PoE
  * Current cost: ~$400 (note: the CO and CO<sub>2</sub> sensors together are 1/4 of the cost and the production cost for the board is currently in the small-batch range)


## Hardware

Details can be found in [Hardware](Hardware).

The PODD is designed to be a self-powered unit that can communicate with other units as well as transmit information over ethernet. The portable battery currently used is the primary source of power. All other power sources provide power by charging the battery. Depending on sample rates, the units can run on battery power for up to 8 days.


### Operational Hardware

The current version of PODD uses the following components to run:

  * Teensy++ 2.0
  * MicroSD Reader 
  * DS3234 Real Time Clock
  * Ethernet Module (for server or other remote communication)
  * PoE module
  * XBee 900 Mesh Kit
  * Rechargeable USB battery
  * Various supporting components (resistors, capacitors, op-amps, etc.)


### Sensors

The current version of PODD uses the following sensors:

  * Sensirior SPS30: particulate matter
  * COZIR-A: CO<sub>2</sub>
  * SPEC Sensor 3SP_CO_1000: CO
  * Honeywell HIH8120-021: dry bulb temperature and humidity
  * PR222J2: radiant temperature
  * OPT3001: illuminance
  * CMR-2747PB-A: sound level


## Building


### Hardware Setup

The PODD can be built entirely on a breadboard for initial prototyping and testing. All parts can be obtained in a DIP package for breadboarding or soldering to a perf-board. For use with the PCB design included, you will need to obtain the supporting components in their SMD packages. The gerber files in the Hardware folder can be used for making the PCB or having it manufactured.

A full list of parts can be found in [this spreadsheet](Hardware/PODD_v1.2_PartsList.xlsx).

To assemble the PODD, see [this assembly document](Hardware/PODD_v1.2_Assembly.pdf).  This document also describes issues with various sensors and other components, and how to operate the sensors.


### Software Setup

You will need the following:

  * [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  * [Teensy add-ons](https://www.pjrc.com/teensy/td_download.html)
  * The sensor and component-specific libraries included in [Libraries](Software/Libraries)

After installing the Arduino IDE and the Teensy add-ons, follow the instructions to [install Arduino Libraries](https://www.arduino.cc/en/Guide/Libraries).

Once the above software is installed, the PODD firmware in [SensorPod_FW](Software/Sketches/SensorPod_FW) can be uploaded to the PODD.  The firmware provides an interactive menu which can be accessed through a serial interface such as provided by the Arduino IDE.  The interactive menu can only be accessed during the PODD startup sequence -- once the PODD enters data-taking mode, it will no longer respond to serial input.


## Data Access
_work in progress_

The PODD collects data from all the sensors at the specified sample rates and logs those to the on-board SD card in the form of a CSV file. All settings are also stored on the SD card as a separate CSV file. If any unit is designated as a controller and connected to Ethernet, the data from all the units on the same network will be uploaded to a server specified by the user.


## Known Issues
_work in progress_

1. Most portable USB power banks automatically shut off to save power whenever the current draw is low: as the PODD draws a much smaller amount of power than the cell phones these power banks are designed to charge, these batteries will often not provide continuous power to a PODD and cannot be used.  The TalentCell batteries we use do not have this feature, but also cannot be recharged via USB/PoE as intended in the current PCB design.
1. R11, which is part of the battery charging circuit, should be 390 Ohms.  The PCB design called for a larger (too large) resistor.
1. The CO sensor output is uncalibrated and we are as yet unsure how to quantify the results.  The C and R pins many be reversed on this sensor's PCB headers.
1. The sound sensor currently provides time-averaged sound levels in an arbitrary linear unit, rather than the logarithmic decibel (dB) scale.  Conversion to dB(Z) is being investigated, but the frequency-weighted dB(A) or dB(C) more commonly used for human hearing may require processing capabilities beyond that provided by the current hardware.


## License

**PODD hardware is released under [Creative Commons Share-alike Non-Commercial 4.0 International](https://creativecommons.org/licenses/by-nc-sa/4.0/).**

**PODD code, firmware, and software is released under the [GNU Affero General Public License](https://www.gnu.org/licenses/).**

See the full text files for details on both licenses.

Our open source goals include 1) sharing new ways to understand buildings with the AEC industry and 2) engaging with a larger group of practitioners and researchers to develop new tools for the AEC industry.


## Reporting Issues

You can open a ticket on [GitHub](https://github.com/lmnts/PODD/issues).


## Contact

Contact pmilusheva@lmnarchitects.com or csavage@lmnarchitects.com with any questions or comments. 


