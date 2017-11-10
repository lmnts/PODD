# Post Occupancy Data Device

![PODD logo](/images/logo%20draft%201%20bg.png)

*A tool to understand how buildings perform in terms of occupant comfort. https://lmnarchitects.com/tech-studio/*

The Post Occupancy Data Device (PODD) is a small, networked, multi-sensor device that:

  * addresses the need for more granular data on buildings once they have been occupied
  * offers a lower cost, less precious alternative to standard building enviornmental sensors
  * gives building designers greater control over the tools available to them to understand their buildings
 

PODD is under active development and should be thought of as early alpha quality for both the software and hardware.

- [Features](#features)
- [Hardware](#hardware)
  - [Operational Hardware](#operational-hardware)
  - [Sensors](#sensors)
- [Software](#software)
- [Building PODD](#building-podd)
  - [Hardware Setup](#hardware-setup)
  - [Software Setup](#software-setup)
- [Data Access](#data-access)
- [Known Issues](#known-issues)
- [License](#license)
- [Reporting Issues](#reporting-issues)
- [Contact](#contact)

## Features

The PODD is our solution to the gap in the available post occupancy evaluation tools. The units are designed to be networked together in large numbers, storing and uploading data from seven sensors that have been identified as most relevant for occupant comfort in buildings.

Each unit has the following features:

  * Air quality: Particle meter (2.5 and 10), CO2, CO
  * Temperature and humidity: Combined Temperature and Humidity sensor, Radiant temperature
  * Illuminance 
  * Sound level 
  * On board data storage: Micro SD card
  * Real Time Clock
  * Wireless communication using 802.15.4 standard (2.4 GHz)
  * Onboard Ethernet 
  * Power via battery, USB, or PoE
  * Current cost: ~$400 (note: the CO and CO2 sensors together are 1/4 of the cost and the production cost for the board is currently in the small-batch range)

## Hardware

Details can be found in [Hardware](/Hardware)

### Operational Hardware

The current version of PODD uses the following components to run:

  * Teensy++2.0
  * MicroSD Reader 
  * DS3234 Real Time Clock
  * Ethernet Module (for server or other remote communication)
  * PoE module
  * XBee 900 Mesh Kit
  * Rechargeable USB battery (5V, 10Ah)
  * various supporting components (resitors, capacitors, Op-Amp)


### Sensors

The current version of PODD uses the following sensors:

  * Telaire/Amphenol SM-PWM-01C - Particle meter
  * COZIR - CO2
  * 3SP_CO_1000 - CO
  * HIH8120-021 - Dry Bulb Temperature and Humidity
  * PR222J2 - Radiant temperature
  * SLD-68-026 - Illuminance
  * CMR-2747PB-A - Sound level

## Software

You will need the following:

  * [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  * [Teensy add-ons](https://www.pjrc.com/teensy/td_download.html)
  * The sensor and component specific libraries included in [Libraries](/Software/Libraries)

After installing the Arduino IDE and the Teensy add-ons, follow the instructions to [install Arduino Libraries](https://www.arduino.cc/en/Guide/Libraries)

## Building PODD

The PODD is designed to be a self-powered unit that can communicate with other units as well as transmit information over ethernet. The 10Ah battery currently used is the primary source of power. All other power sources provide power by charging the battery. Depending on sample rates, the units can run on battery power for up to 5 days. 

### Hardware Setup
work in progress

The PODD can be built entirely on a breadboard for initial prorotyping and testing. All parts can be obtained in a DIP package for breadboarding or soldering to a perf-board. For use with the PCB design included, you will need to obtain the supporting components in their SMD packages. The gerber files in the Hardware folder can be used for making the PCB or having it manufactired.


### Software Setup
work in progress

## Data Access
work in progress

The PODD collects data from all the sensors at the specified sample rates and logs those to the on-board SD card in the form of a CSV file. All settings are also stored on the SD card as a separate CSV file. If any unit is designated as a controller and connected to Ethernet, the data from all the units on the same network will be uploaded to a server specified by the user. 

## Known Issues
work in progress

1. Sample rate change (higher frequency) causes inability to upload to server
2. Date and Time do not update correctly when units are not plugged into ethernet during setup
3. The resistor in the battery charging section of the circuit is too big


## License

**PODD hardware is released under [Creative Commons Share-alike Non-Commercial 4.0 International](https://creativecommons.org/licenses/by-nc-sa/4.0/).**

**PODD code, firmware, and software is released under the [GNU Affero General Public License](https://www.gnu.org/licenses/).**

See the full text files for details on both licenses.

Our open source goals include 1) sharing new ways to understand buildings with the AEC industry and 2) engaging with a larger group of practitioners and researchers to develop new tools for the AEC industry.

## Reporting Issues

You can open a ticket on [GitHub](https://github.com/lmnts/PODD/issues)

## Contact

Contact pmilusheva@lmnarchitects.com with any questions or comments. 


