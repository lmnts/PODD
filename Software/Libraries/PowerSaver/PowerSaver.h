#ifndef PowerSaver_h
#define PowerSaver_h

#include "Arduino.h"

class PowerSaver
{
  public:
    PowerSaver();
    void sleepInterruptSetup();
    void turnOffSPI();
    void turnOnSPI();
    void turnOffADC();
		void turnOnADC();
    void turnOffBOD();
		void turnOffWDT();
		void turnOnWDTInterrupt();
		void goodNight();
		
  private:
		byte d1;
		byte p1;
		byte adc;
		byte i2c;
};

#endif
