// OSBSS Power Saver library for Atmel ATmega328P.
// Last updated - February 18, 2015

#include "Arduino.h"
#include "PowerSaver.h"

  //****************************************************************

PowerSaver::PowerSaver()
{
		// empty constructor - not required
}

  //****************************************************************
  
void PowerSaver::sleepInterruptSetup()
{
  SMCR = (1<<SE) | (1<<SM1);  // setup sleep in powerdown mode
  cli();	// disable global interrupts
  //DDRB |= ~(1<<DDB0); // set pin D8 as INPUT
  PORTB |= (1<<PORTB0); //Activate pullup on pin D8
  PCICR |= (1<<PCIE0); // enable interrupts on PCINT[7:0]
  PCMSK0 |= (1<<PCINT0); // pin change mask register for pin D8
  sei();	// enable global interrupts
}

  //****************************************************************
	
void PowerSaver::turnOffSPI()
{
  PORTB |= ((1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1));  // set SPI pins (13, 12, 11, 10, 9) to HIGH
  DDRB &= ~((1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1));  // set SPI pins (13, 12, 11, 10, 9) to INPUT
  SPCR = 0;  // reset SPI control register
}
	
  //****************************************************************
	  	
void PowerSaver::turnOnSPI()
{
  DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2);  // set pin 13, 11, 10 as OUTPUT
  PORTB &= ~((1<<DDB5) | (1<<DDB3));	// set pin 13, 11 as LOW
}
		  
  //****************************************************************

void PowerSaver::turnOffADC()
{
  d1 = DDRC;	// save direction of analog pins
  p1 = PORTC; // save pinMode of analog pins
  adc = ADCSRA;  // save ADCSRA byte
  i2c = TWCR;		// save TWCR byte
  delay(1);
  //DDRC = 0;		// Setting all analog pins to INPUT 
  //PORTC = 0;	// Setting all analog pins to LOW (disable internal pull-ups)
  ADCSRA = 0;	// disable ADC
  TWCR = 0; // disable I2C
  //ADCSRA = ~(1<<ADEN); // This is the ADC enable bit. Writing it to 0 will turn off ADC
}
		  
  //****************************************************************
	
void PowerSaver::turnOnADC()
{
  //ADCSRA = (1<<ADEN); // This is the ADC enable bit. Writing it to 1 will turn on ADC
  DDRC = d1;	// restore direction of analog pins
  PORTC = p1;	// restore pinMode of analog pins
  ADCSRA = adc;		//restore ADCSRA byte
  TWCR = i2c;  // restore TWCR byte
}
	
	//****************************************************************

void PowerSaver::turnOffBOD()
{
  // turn off brown-out enable in software (temporary)
  MCUCR |= (1<<BODS) | (1<<BODSE);  // turn on brown-out enable select
  MCUCR |= (1<<BODS);        // this must be done within 4 clock cycles of above
  MCUCR &= ~(1<<BODSE); 	// the processor must sleep within 3 clock cycles after this or BOD disable is cancelled
}

  //****************************************************************
	
	void PowerSaver::turnOnWDTInterrupt()
{
  cli();		// Disable global interrupts
  asm("wdr");		// Reset Watchdog Timer
  MCUSR = 0; // clear reset flags
  WDTCSR |= (1<<WDCE) | (1<<WDE);		// Write logical one to WDCE and WDE
  //WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP0);	// Turn on WDT interrupt - set prescale at 0.5s
  //WDTCSR = (1<<WDIE) | (1<<WDP2);	// Turn on WDT interrupt - set prescale at 0.25s
  WDTCSR = (1<<WDIE) | (1<<WDP1) | (1<<WDP0);	// Turn on WDT interrupt - set prescale at 0.125s
  //WDTCSR = (1<<WDIE) | ~(1<<WDP3) | ~(1<<WDP2)| ~(1<<WDP1) | ~(1<<WDP0);	// Turn on WDT interrupt - set prescale at 0.016s
  sei();	// Enable global interrupts
}

   //****************************************************************
	
void PowerSaver::turnOffWDT()
{
  cli();		// Disable global interrupts
  asm("wdr");		// Reset Watchdog Timer
  MCUSR &= ~(1<<WDRF);	// Clear WDRF in MCUSR
  WDTCSR |= (1<<WDCE) | (1<<WDE);		// Write logical one to WDCE and WDE
  WDTCSR = 0x00;	// Turn off WDT
  sei();	// Enable global interrupts
}

   //****************************************************************
	 
void PowerSaver::goodNight()
{
  asm("sleep");  // this will put processor in power-down mode
	
}
	//****************************************************************
  