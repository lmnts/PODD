#include "SMPWM01A.h"
#define SAMPLE_MS 1000ul

SMPWM01A dust;

void setup () { 
  
  Serial.begin(9600);
  dust.begin();
 
}


void loop() {
 

  //Print the small particle concentration, 1-3um in size
  //Measurement (should be) in ug/m^3
  Serial.print("PM2.5: ");
  Serial.println(dust.getPM2());
  
  //Print the large particle concentration, ~10um in size
  //Measurement (should be) in ug/m^3
  Serial.print("PM10: ");
  Serial.println(dust.getPM10());
  
  delay(SAMPLE_MS);
}
