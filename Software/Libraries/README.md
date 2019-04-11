## Libraries

Libraries found here are mainly pulled from various [GitHub](https://github.com) repositories.  To compile the PODD firmware, the Arduino build process must be able to find these libraries.  There are several options for doing this.  The easiest option is to install the library via the Arduino library manager (`Sketch → Include Library → Manage Libraries...`); not all of these libraries are available this way.  Alternatively, you can copy the library folders here into the `libraries` subdirectory of your Arduino sketchbook folder (see [here](https://www.arduino.cc/en/Guide/Libraries) for help) or simply copy all `.h` and `.c` files in the subdirectories here into the folder of the sketch to be compiled.


### Sources
Those marked with (†) are not in the Arduino library repository and must be installed manually.
- **AsyncDelay:**
  <https://github.com/stevemarple/AsyncDelay>
- **ClosedCube OPT3001:**
  <https://github.com/closedcube/ClosedCube_OPT3001_Arduino>
- **Cozir CO2 Sensor (†):**
  <https://github.com/roder/cozir>

  The version included here has been modified from the original to improve read timing, address compiler warnings, allow for generic serial interfaces, and remove a Serial.begin() statement (the PODD firmware will control the serial interfaces).
- **HIH61xx:**
  <https://github.com/stevemarple/HIH61xx>
  
  This library is also valid for the 8XXX series of Honeywell HumidIcon humidity/temperature sensors.
- **NeoSWSerial:**
  <https://github.com/SlashDevin/NeoSWSerial>
- **SparkFun DS3234 RTC (†):**
  <https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library>
- **Sensirion SPS30 (†):**
  <https://github.com/paulvha/sps30>

  Paul van Haastrecht's SPS30 library is easier to use than the official Sensirion Arduino libraries.  The version here has been modified to exclude the serial interface code (not used by PODDs).  A bug has also been fixed in where the stop routine was sending the start command instead.
- **Time library:**
  <https://github.com/PaulStoffregen/Time>
- **TimeAlarms (†):**
  <https://github.com/PaulStoffregen/TimeAlarms>

  The version included here has been modified from the original to increase the maximum number of timers/alarms.
- **TimerOne:**
  <https://github.com/PaulStoffregen/TimerOne>

