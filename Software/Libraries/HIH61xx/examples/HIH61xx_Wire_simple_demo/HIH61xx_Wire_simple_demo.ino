// This example demonstrates how to use the HIH61xx class with the Wire library. A blocking read is made to the
// HIH61xx device. See HIH61xx_Wire_demo for a more sophisticated example which allows other tasks to run
// whilst the HIH61xx takes its measurements.


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


void loop(void)
{
    // Instruct the HIH61xx to take a measurement. This blocks until the measurement is ready.
    hih.read();

    // Fetch and print the results
    Serial.print("Relative humidity: ");
    Serial.print(hih.getRelHumidity() / 100.0);
    Serial.println(" %");
    Serial.print("Ambient temperature: ");
    Serial.print(hih.getAmbientTemp() / 100.0);
    Serial.println(" deg C");
    Serial.print("Status: ");
    Serial.println(hih.getStatus());

    // Wait a second
    delay(1000);
}