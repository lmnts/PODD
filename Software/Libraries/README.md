## Libraries

Libraries found here are mainly pulled from various [GitHub](https://github.com) repositories.  To compile the PODD firmware, the Arduino build process must be able to find these libraries.  There are several options for doing this.  The easiest option is to install the library via the Arduino library manager (`Sketch → Include Library → Manage Libraries...`); not all of these libraries are available this way.  Alternatively, you can copy the library folders here into the `libraries` subdirectory of your Arduino sketchbook folder (see [here](https://www.arduino.cc/en/Guide/Libraries) for help) or simply copy all `.h` and `.c` files in the subdirectories here into the folder of the sketch to be compiled.


### Sources
Those marked with (†) are not in the Arduino library repository and must be installed manually.
- **AsyncDelay:**
  <https://github.com/stevemarple/AsyncDelay>
- **Cozir CO2 Sensor (†):**
  <https://github.com/roder/cozir>

  The version included here has been modified from the original to improve read timing and to address compiler warnings.
- **HIH61xx:**
  <https://github.com/stevemarple/HIH61xx>
  
  This library is also valid for the 8XXX series of Honeywell HumidIcon humidity/temperature sensors.
- **SparkFun DS3234 RTC (†):**
  <https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library>
- **Time library:**
  <https://github.com/PaulStoffregen/Time>
- **TimeAlarms (†):**
  <https://github.com/PaulStoffregen/TimeAlarms>
  
  Though this library is available in the Arduino library repository, the PODD firmware requires modifications in the GitHub repo that have not made it into the Arduino repository yet (version 1.5.1 or later is required).

