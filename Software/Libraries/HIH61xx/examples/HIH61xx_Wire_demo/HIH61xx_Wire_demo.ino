// This example demonstrates how to use the HIH61xx class with the Wire library. The HIH61xx state machine
// enables others tasks to run whilst the HIH61xx is powering up etc.

#include <Wire.h>
#include <HIH61xx.h>
#include <AsyncDelay.h>

// The "hih" object must be created with a reference to the "Wire" object which represents the I2C bus it is using.
// Note that the class for the Wire object is called "TwoWire", and must be included in the templated class name.
HIH61xx<TwoWire> hih(Wire);

AsyncDelay samplingInterval;

void setup(void)
{
#if F_CPU >= 12000000UL
    Serial.begin(115200);
#else
	Serial.begin(9600);
#endif


	Wire.begin();

	hih.initialise();
	samplingInterval.start(3000, AsyncDelay::MILLIS);
}


bool printed = true;
void loop(void)
{
	if (samplingInterval.isExpired() && !hih.isSampling()) {
		hih.start();
		printed = false;
		samplingInterval.repeat();
		Serial.println("Sampling started (using Wire library)");
	}

	hih.process();

	if (hih.isFinished() && !printed) {
		printed = true;
		// Print saved values
		Serial.print("RH: ");
		Serial.print(hih.getRelHumidity() / 100.0);
		Serial.println(" %");
		Serial.print("Ambient: ");
		Serial.print(hih.getAmbientTemp() / 100.0);
		Serial.println(" deg C");
		Serial.print("Status: ");
		Serial.println(hih.getStatus());
	}

}
