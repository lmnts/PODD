HIH61xx
=======

HIH61xx is a library for accessing the humidity and temperature data
from the I2C range of Honeywell HIH61xx humidity sensors (HIH6120-021,
HIH6121-021, HIH6130-021 and HIH6131-021). Do not confuse these
sensors with the SPI versions (HIH6130-000 and HIH6131-000).

The sensor can be accessed using either hardware I2C (with the `Wire`
library) or software I2C (using the `SoftWire` library); examples
demonstrate both uses. For low-power operation an optional power pin
can be used to control power to the device. A state machine ensures
the relevant timing constraints are observed.


Requirements
------------

The following libraries are required:

* AsyncDelay: see https://github.com/stevemarple/AsyncDelay
* SoftWire: see https://github.com/stevemarple/SoftWire

Examples
--------

The `HIH61xx_Wire_demo` and `HIH61xx_SoftWire_demo` sketches demonstrates the state machine operation.

License
-------

The HIH61xx library is released under the GNU Lesser General Public
License, version 2.1. See [LICENSE.txt](LICENSE.txt) for details.
