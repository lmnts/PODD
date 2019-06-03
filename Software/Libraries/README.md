## Libraries

Libraries found here are mainly pulled from various [GitHub](https://github.com) repositories.  To compile the PODD firmware, the Arduino build process must be able to find these libraries.  There are several options for doing this.  The easiest option is to install the library via the Arduino library manager (`Sketch → Include Library → Manage Libraries...`); not all of these libraries are available this way.  Alternatively, you can copy the library folders here into the `libraries` subdirectory of your Arduino sketchbook folder (see [here](https://www.arduino.cc/en/Guide/Libraries) for help) or simply copy all `.h` and `.c` files in the subdirectories here into the folder of the sketch to be compiled.


### Sources
Those marked with (†) are not in the Arduino library repository and must be installed manually.
- **ClosedCube OPT3001:**
  <https://github.com/closedcube/ClosedCube_OPT3001_Arduino>
- **NeoSWSerial:**
  <https://github.com/SlashDevin/NeoSWSerial>
- **sps30 (†):**
  <https://github.com/paulvha/sps30>

  The version included here has been modified from the original to decrease timeouts when the SPS30 sensor is not present.  This modification is not necessary to operate a PODD, though unnecessary delays will occur when testing sensors or otherwise attempting to read a non-existent SPS30 sensor.
- **Time library:**
  <https://github.com/PaulStoffregen/Time>
- **TimeAlarms (†):**
  <https://github.com/PaulStoffregen/TimeAlarms>

  The version included here has been modified from the original to increase the maximum number of timers/alarms.
- **TimerOne:**
  <https://github.com/PaulStoffregen/TimerOne>
- **TimerThree:**
  <https://github.com/PaulStoffregen/TimerThree>
- **Timezone:**
  <https://github.com/JChristensen/Timezone>

