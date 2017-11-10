
// Use this sketch to calibrate the
// CO2 sensor on a SensorPod
//
// 1. Upload this sketch to the SensorPod
// 2. Wait until the SensorPod has warmed up
//    and air has diffused fully into the CO2
//    sensor.
// 3. Measure the CO2 with a calibrated sensor
// 4. Send the command G####, where #### is the 
//    CO2 concentration shown by the reference
#include "cozir.h"


// CO2
SoftwareSerial nss(26, 25); //used pins 25 from rx, 26 from tx.
COZIR czr(nss);

void setup() {
  // put your setup code here, to run once:
  czr.SetOperatingMode(CZR_POLLING);


  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.read() == 'G') {
    // TODO: read a number
    int ref = Serial.parseInt();
    czr.CalibrateKnownGas(ref);
  }
  Serial.println(czr.CO2());
  delay(2000);
}
