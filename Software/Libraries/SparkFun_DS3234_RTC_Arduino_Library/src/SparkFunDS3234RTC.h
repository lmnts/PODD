/******************************************************************************
SparkFunDS3234RTC.h
Jim Lindblom @ SparkFun Electronics
original creation date: October 2, 2016
https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library

Prototypes the DS3234 class. Defines sketch-usable global variables.

Resources:
Wire.h - Arduino I2C Library

Development environment specifics:
Arduino 1.6.8
SparkFun RedBoard
SparkFun Real Time Clock Module (v14)

Updated 16 October 2016 by Vassilis Serasidis <avrsite@yahoo.gr>
- Added readFromSRAM' and 'writeToSRAM' functions

******************************************************************************/
#include <Arduino.h>
#include <SPI.h>

#ifndef SPARKFUNDS3234RTC_H
#define SPARKFUNDS3234RTC_H

#define TWELVE_HOUR_MODE (1<<6) // 12/24-hour Mode bit in Hour register
#define TWELVE_HOUR_PM (1<<5)   // am/pm bit in hour register

#define AM false
#define PM true

#define SQW_CONTROL_MASK 0xE3 // SQW bit(s) mask in control register
#define SQW_ENABLE_BIT (1<<2) // SQW output enable bit in control register

#define ALARM_MODE_BIT (1<<7)   // Alarm mode bit (A1M1, A1M2, etc) in alarm registers
#define ALARM_DAY_BIT (1<<6)    // Alarm day/date control bit in alarm day registers
#define ALARM_1_FLAG_BIT (1<<0) // Alarm 1 flag in control/status register
#define ALARM_2_FLAG_BIT (1<<1) // Alarm 2 flag in control/status register
#define ALARM_INTCN_BIT (1<<2)  // Interrupt-enable bit in control register

#define TIME_ARRAY_LENGTH 7 // Total number of writable time values in device
enum time_order {
	TIME_SECONDS, // 0
	TIME_MINUTES, // 1
	TIME_HOURS,   // 2
	TIME_DAY,     // 3
	TIME_DATE,    // 4
	TIME_MONTH,   // 5
	TIME_YEAR,    // 6
};

// sqw_rate -- enum for possible SQW pin output settings
enum sqw_rate {
	SQW_SQUARE_1,  // 0
	SQW_SQUARE_1K, // 1
	SQW_SQUARE_4K, // 2
	SQW_SQUARE_8K  // 3
};

// DS3234_registers -- Definition of DS3234 registers
enum DS3234_registers {
	DS3234_REGISTER_SECONDS, // 0x00
	DS3234_REGISTER_MINUTES, // 0x01
	DS3234_REGISTER_HOURS,   // 0x02
	DS3234_REGISTER_DAY,     // 0x03
	DS3234_REGISTER_DATE,    // 0x04
	DS3234_REGISTER_MONTH,   // 0x05
	DS3234_REGISTER_YEAR,    // 0x06
	DS3234_REGISTER_A1SEC,   // 0x07
	DS3234_REGISTER_A1MIN,   // 0x08
	DS3234_REGISTER_A1HR,    // 0x09
	DS3234_REGISTER_A1DA,    // 0x0A
	DS3234_REGISTER_A2MIN,   // 0x0B
	DS3234_REGISTER_A2HR,    // 0x0C
	DS3234_REGISTER_A2DA,    // 0x0D
	DS3234_REGISTER_CONTROL, // 0x0E
	DS3234_REGISTER_STATUS,  // 0x0F
	DS3234_REGISTER_XTAL,    // 0x10
	DS3234_REGISTER_TEMPM,   // 0x11
	DS3234_REGISTER_TEMPL,   // 0x12
	DS3234_REGISTER_TEMPEN,  // 0x13
  DS3234_REGISTER_RESERV1, // 0x14
  DS3234_REGISTER_RESERV2, // 0x15
  DS3234_REGISTER_RESERV3, // 0x16
  DS3234_REGISTER_RESERV4, // 0x17
  DS3234_REGISTER_SRAMA,   // 0x18
  DS3234_REGISTER_SRAMD    // 0x19
};

// Base register for complete time/date readings
#define DS3234_REGISTER_BASE DS3234_REGISTER_SECONDS

// dayIntToStr -- convert day integer to the string
static const char *dayIntToStr[7] {
	"Sunday",
	"Monday",
	"Tuesday",
	"Wednesday",
	"Thursday",
	"Friday",
	"Saturday"	
};

// dayIntToChar -- convert day integer to character
static const char dayIntToChar[7] = {'U', 'M', 'T', 'W', 'R', 'F', 'S' };

class DS3234
{
public:

	////////////////////
	// Initialization //
	////////////////////
	// Constructor -- Initialize class variables to 0
	DS3234();
	// Begin -- Initialize SPI interface, and sets up the chip-select pin
	void begin(uint8_t csPin);
	
	///////////////////////
	// Setting the Clock //
	///////////////////////
	// setTime -- Set time and date/day registers of DS3234
	void setTime(uint8_t sec, uint8_t min, uint8_t hour,
	             uint8_t day, uint8_t date, uint8_t month, uint8_t year);
	void setTime(uint8_t sec, uint8_t min, uint8_t hour12, bool pm, 
	             uint8_t day, uint8_t date, uint8_t month, uint8_t year);
	// setTime -- Set time and date/day registers of DS3234 (using data array)
	void setTime(uint8_t * time, uint8_t len);
	// autoTime -- Set time with compiler time/date
	void autoTime();
	
	// To set specific values of the clock, use the set____ functions:
	void setSecond(uint8_t s);
	void setMinute(uint8_t m);
	void setHour(uint8_t h);
	void setDay(uint8_t d);
	void setDate(uint8_t d);
	void setMonth(uint8_t mo);
	void setYear(uint8_t y);
	
	///////////////////////
	// Reading the Clock //
	///////////////////////
	// update -- Read all time/date registers and update the _time array
	void  update(void);
	// update should be performed before any of the following. It will update
	//   all values at one time.	
	inline uint8_t second(void) { return BCDtoDEC(_time[TIME_SECONDS]); };
	inline uint8_t minute(void) { return BCDtoDEC(_time[TIME_MINUTES]); };
	inline uint8_t hour(void) { return BCDtoDEC(_time[TIME_HOURS]); };
	inline uint8_t day(void) { return BCDtoDEC(_time[TIME_DAY]); };
	inline const char dayChar(void) { return dayIntToChar[BCDtoDEC(_time[TIME_DAY]) - 1]; };
	inline const char * dayStr(void) { return dayIntToStr[BCDtoDEC(_time[TIME_DAY]) - 1]; };
	inline uint8_t date(void) { return BCDtoDEC(_time[TIME_DATE]); };
	inline uint8_t month(void) { return BCDtoDEC(_time[TIME_MONTH]);	};
	inline uint8_t year(void) { return BCDtoDEC(_time[TIME_YEAR]); };
	
	// To read a single value at a time, use the get___ functions:
	uint8_t getSecond(void);
	uint8_t getMinute(void);
	uint8_t getHour(void);
	uint8_t getDay(void);
	uint8_t getDate(void);
	uint8_t getMonth(void);
	uint8_t getYear(void);
	
	// is12Hour -- check if the DS3234 is in 12-hour mode | returns true if 12-hour mode
	bool is12Hour(void);
	// pm -- Check if 12-hour state is AM or PM | returns true if PM
	bool pm(void);
	
	// DS3234 has a die-temperature reading. Value is produced in multiples of 0.25 deg C
	float temperature(void);
	
	/////////////////////////////
	// Alarm Setting Functions //
	/////////////////////////////
	// Alarm 1 can be set to trigger on seconds, minutes, hours, and/or date/day.
	// Any of those can be masked out -- ignored for an alarm match. By setting
	// If a value is set to 255, the library will mask out that data.
	// By default the "date" value is the day-of-month (1-31)
	// The "day" boolean changes the "date" value to a day (between 1-7). 
	void setAlarm1(uint8_t second = 255, uint8_t minute = 255, uint8_t hour = 255, uint8_t date = 255, bool day = false);
	// Second alarm 1 set function if the clock is in 12-hour mode:
	void setAlarm1(uint8_t second, uint8_t minute, uint8_t hour12, bool pm, uint8_t date = 255, bool day = false);
	// Alarm 2 functions similarly to alarm 1, but does not have a seconds value:
	void setAlarm2(uint8_t minute = 255, uint8_t hour = 255, uint8_t date = 255, bool day = false);
	// Second alarm 2 set function if the clock is in 12-hour mode:
	void setAlarm2(uint8_t minute, uint8_t hour12, bool pm, uint8_t date = 255, bool day = false);
	
	// The SQW pin can be used as an interrupt. It is active-low, and can be
	// set to trigger on alarm 1 and/or alarm 2:
	void enableAlarmInterrupt(bool alarm1 = true, bool alarm2 = true);
	
	// alarm1 and alarm2 check their respective flag in the control/status
	// register. They return true if the flag is set.
	// The "clear" boolean, commands the alarm function to clear the flag
	// (assuming it was set).
	bool alarm1(bool clear = true);
	bool alarm2(bool clear = true);	
	
	///////////////////////////////
	// SQW Pin Control Functions //
	///////////////////////////////
	void writeSQW(sqw_rate value); // Write SQW pin high, low, or to a set rate
	
	/////////////////////////////
	// Misc. Control Functions //
	/////////////////////////////
	void enable(void); // Enable the oscillator
	void disable(void); // Disable the oscillator (no counting!)
	
	void set12Hour(bool enable12 = true); // Enable/disable 12-hour mode
	void set24Hour(bool enable24 = true); // Enable/disable 24-hour mode
	
  void writeToSRAM(uint8_t address, uint8_t data);
  uint8_t readFromSRAM(uint8_t address);
  
private:
	uint8_t _csPin;
	uint8_t _time[TIME_ARRAY_LENGTH];
	bool _pm;
	
	uint8_t BCDtoDEC(uint8_t val);
	uint8_t DECtoBCD(uint8_t val);
	
	void spiWriteBytes(DS3234_registers reg, uint8_t * values, uint8_t len);
	void spiWriteByte(DS3234_registers reg, uint8_t value);
	uint8_t spiReadByte(DS3234_registers reg);
	void spiReadBytes(DS3234_registers reg, uint8_t * dest, uint8_t len);
};

extern DS3234 rtc;

#endif // SPARKFUNDS3234RTC_H
