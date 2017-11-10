
//
// Use interrupts to time low pulse occupancy because this class of devices notoriously 
// returns zeros at low concentrations. Using pulseIn(pin, LOW) can cause blocking 
// problems in the loop() function. 
//
// Here we're using pin change interrups to conserve dedicated hardware interrupt pins.
//
// http://www.howmuchsnow.com/arduino/airquality/grovedust/
//
// Also consider eliminating the constant term in the concentration calculation above if 
// using a smoothing filter. 
//
// http://plchowto.com/plc-filter/
//

#include "SMPWM01A.h"


int SMPWM01A::statePM10; //state variables for the two inputs
int SMPWM01A::statePM02;
		
unsigned long SMPWM01A::startPM10;
unsigned long SMPWM01A::startPM02; 
		
volatile unsigned long SMPWM01A::lpoPM10; // low pulse occupancy variables in microseconds
volatile unsigned long SMPWM01A::lpoPM02; 

float SMPWM01A::concPM10; //concentration values 
float SMPWM01A::concPM02;


SMPWM01A::SMPWM01A() {}


void SMPWM01A::begin() {
  
  concPM10 = 0;
  concPM02 = 0;
  
  statePM10 = HIGH;
  statePM02 = HIGH;
  
  // set up Timer 1
  TCCR1A = 0;          							// normal operation
  TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12);   	// CTC, scale to clock / 1024
  OCR1A  = 62500;       						// compare A register value (62500 * clock speed / 1024)
  TIMSK1 = 1<<OCIE1A;             				// interrupt on Compare A Match

  // Enable pin change interrupt
  // Note: These values are specific to Arduino Uno, for other devices refer to:  
  // http://gammon.com.au/interrupts
  PCMSK2 |= (1<<PCINT0);  // want pin  2
  PCMSK2 |= (1<<PCINT1);  // want pin  3
  PCIFR  |= (1<<PCIF2);   	// clear any outstanding interrupts
  PCICR  |= (1<<PCIE2);   	// enable pin change interrupts for PD4 & PD5

  pinMode(P1, INPUT);      
  pinMode(P2, INPUT);

  // PPD42NS only, enabling pullup resistors is not required for this device
  // digitalWrite(9, HIGH);
  // digitalWrite(10, HIGH);
  }




ISR(PCINT2_vect) {
 SMPWM01A::PCINT2_ISR();
  
} 


void SMPWM01A::PCINT2_ISR() {
	unsigned long us = micros();

  // determine which pins changed state and take appropriate action
  if (statePM10 != digitalRead(P2)) 
    if ((statePM10 = digitalRead(P2)) == LOW) 
      startPM10 = us;
    else 
      lpoPM10 += (us - startPM10);
  if (statePM02 != digitalRead(P1)) 
    if ((statePM02 = digitalRead(P1)) == LOW) 
      startPM02 = us;
    else 
      lpoPM02 += (us - startPM02);
}

//Timer Interrupt for periodic dust concentration calculation. Called roughly once every 4 seconds
ISR(TIMER1_COMPA_vect) {
  SMPWM01A::TIMER1_COMPA_ISR();
  
}


//Handler for the TIMER1_COMPA interrupt. 
void SMPWM01A::TIMER1_COMPA_ISR() {
    
  float ratio, conc; //temporary variables for concentration calculations
  
  //percentage of time dust has been detected in one sampling period
  ratio = 0.1 * float(lpoPM10) / float(SAMPLE_MS);
  // polynomial approximation of the curve in the datasheet. Yields the concentration in micrograms per cubic meter
  conc = 1.1 * ratio * ratio * ratio  - 3.8 * ratio * ratio + 520 * ratio; // + 0.62;
  // some filtering
  concPM10 = (conc  + concPM10 * (FILTER_WEIGHT - 1.0)) / FILTER_WEIGHT;
  //reseting the count
  lpoPM10 = 0;
  
  ratio = 0.1 * float(lpoPM02) / float(SAMPLE_MS);
  conc = 1.1 * ratio * ratio * ratio  - 3.8 * ratio * ratio + 520 * ratio; // + 0.62;
  concPM02 = (conc  + concPM02 * (FILTER_WEIGHT - 1.0)) / FILTER_WEIGHT;
  lpoPM02 = 0;
}

//Returns the PM2 dust concentration
float SMPWM01A::getPM2() {
	return concPM02;
}

//Returns the PM10 dust concentration
float SMPWM01A::getPM10() {
	return concPM10;
}

