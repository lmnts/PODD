/******************************************************************************
SparkFunDS3234RTC.cpp
Jim Lindblom @ SparkFun Electronics
original creation date: October 2, 2016
https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library

Implementation of DS3234 real time clock functions

Resources:
SPI.h - Arduino I2C Library

Development environment specifics:
Arduino 1.6.8
SparkFun RedBoard
SparkFun Real Time Clock Module (v14)

Updated 16 October 2016 by Vassilis Serasidis <avrsite@yahoo.gr>
- Added readFromSRAM' and 'writeToSRAM' functions

******************************************************************************/

#include "SparkFunDS3234RTC.h"

// Parse the __DATE__ predefined macro to generate date defaults:
// __Date__ Format: MMM DD YYYY (First D may be a space if <10)
// <MONTH>
#define BUILD_MONTH_JAN ((__DATE__[0] == 'J') && (__DATE__[1] == 'a')) ? 1 : 0
#define BUILD_MONTH_FEB (__DATE__[0] == 'F') ? 2 : 0
#define BUILD_MONTH_MAR ((__DATE__[0] == 'M') && (__DATE__[1] == 'a') && (__DATE__[2] == 'r')) ? 3 : 0
#define BUILD_MONTH_APR ((__DATE__[0] == 'A') && (__DATE__[1] == 'p')) ? 4 : 0
#define BUILD_MONTH_MAY ((__DATE__[0] == 'M') && (__DATE__[1] == 'a') && (__DATE__[2] == 'y')) ? 5 : 0
#define BUILD_MONTH_JUN ((__DATE__[0] == 'J') && (__DATE__[1] == 'u') && (__DATE__[2] == 'n')) ? 6 : 0
#define BUILD_MONTH_JUL ((__DATE__[0] == 'J') && (__DATE__[1] == 'u') && (__DATE__[2] == 'l')) ? 7 : 0
#define BUILD_MONTH_AUG ((__DATE__[0] == 'A') && (__DATE__[1] == 'u')) ? 8 : 0
#define BUILD_MONTH_SEP (__DATE__[0] == 'S') ? 9 : 0
#define BUILD_MONTH_OCT (__DATE__[0] == 'O') ? 10 : 0
#define BUILD_MONTH_NOV (__DATE__[0] == 'N') ? 11 : 0
#define BUILD_MONTH_DEC (__DATE__[0] == 'D') ? 12 : 0
#define BUILD_MONTH BUILD_MONTH_JAN | BUILD_MONTH_FEB | BUILD_MONTH_MAR | \
                    BUILD_MONTH_APR | BUILD_MONTH_MAY | BUILD_MONTH_JUN | \
                    BUILD_MONTH_JUL | BUILD_MONTH_AUG | BUILD_MONTH_SEP | \
                    BUILD_MONTH_OCT | BUILD_MONTH_NOV | BUILD_MONTH_DEC
// <DATE>
#define BUILD_DATE_0 ((__DATE__[4] == ' ') ? 0 : (__DATE__[4] - 0x30))
#define BUILD_DATE_1 (__DATE__[5] - 0x30)
#define BUILD_DATE ((BUILD_DATE_0 * 10) + BUILD_DATE_1)
// <YEAR>
#define BUILD_YEAR (((__DATE__[7] - 0x30) * 1000) + ((__DATE__[8] - 0x30) * 100) + \
                    ((__DATE__[9] - 0x30) * 10)  + ((__DATE__[10] - 0x30) * 1))

// Parse the __TIME__ predefined macro to generate time defaults:
// __TIME__ Format: HH:MM:SS (First number of each is padded by 0 if <10)
// <HOUR>
#define BUILD_HOUR_0 ((__TIME__[0] == ' ') ? 0 : (__TIME__[0] - 0x30))
#define BUILD_HOUR_1 (__TIME__[1] - 0x30)
#define BUILD_HOUR ((BUILD_HOUR_0 * 10) + BUILD_HOUR_1)
// <MINUTE>
#define BUILD_MINUTE_0 ((__TIME__[3] == ' ') ? 0 : (__TIME__[3] - 0x30))
#define BUILD_MINUTE_1 (__TIME__[4] - 0x30)
#define BUILD_MINUTE ((BUILD_MINUTE_0 * 10) + BUILD_MINUTE_1)
// <SECOND>
#define BUILD_SECOND_0 ((__TIME__[6] == ' ') ? 0 : (__TIME__[6] - 0x30))
#define BUILD_SECOND_1 (__TIME__[7] - 0x30)
#define BUILD_SECOND ((BUILD_SECOND_0 * 10) + BUILD_SECOND_1)

// DS3234 SPI Settings:
#define DS3234_MAX_SCLK 4000000 // 4MHz max SPI clock rate
SPISettings DS3234SPISettings(DS3234_MAX_SCLK, MSBFIRST, SPI_MODE3);

// Constructor -- Initialize class variables to 0
DS3234::DS3234()
{
	for (int i=0; i<TIME_ARRAY_LENGTH; i++)
	{
		_time[i] = 0;
	}
	_pm = false;
}

// Begin -- Initialize SPI interface
void DS3234::begin(uint8_t csPin)
{
	_csPin = csPin;
	
#ifdef ARDUINO_ARCH_AVR
	pinMode(SS, OUTPUT); // In master mode, Arduino Uno's (ATmega328 in general) pin 10 must be output
#endif
	pinMode(_csPin, OUTPUT);
	digitalWrite(_csPin, HIGH); // Default to SS in-active
	SPI.begin();
}

// setTime -- Set time and date/day registers of DS3234
void DS3234::setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	_time[TIME_SECONDS] = DECtoBCD(sec);
	_time[TIME_MINUTES] = DECtoBCD(min);
	_time[TIME_HOURS] = DECtoBCD(hour);
	_time[TIME_DAY] = DECtoBCD(day);
	_time[TIME_DATE] = DECtoBCD(date);
	_time[TIME_MONTH] = DECtoBCD(month);
	_time[TIME_YEAR] = DECtoBCD(year);
	
	setTime(_time, TIME_ARRAY_LENGTH);
}

void DS3234::setTime(uint8_t sec, uint8_t min, uint8_t hour12, bool pm, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	_time[TIME_SECONDS] = DECtoBCD(sec);
	_time[TIME_MINUTES] = DECtoBCD(min);
	_time[TIME_HOURS] = DECtoBCD(hour12);
	_time[TIME_HOURS] |= TWELVE_HOUR_MODE;
	if (pm)
		_time[TIME_HOURS] |= TWELVE_HOUR_PM;
	_time[TIME_DAY] = DECtoBCD(day);
	_time[TIME_DATE] = DECtoBCD(date);
	_time[TIME_MONTH] = DECtoBCD(month);
	_time[TIME_YEAR] = DECtoBCD(year);
	
	setTime(_time, TIME_ARRAY_LENGTH);
}

// setTime -- Set time and date/day registers of DS3234 (using data array)
void DS3234::setTime(uint8_t * time, uint8_t len)
{
	if (len != TIME_ARRAY_LENGTH)
		return;
	
	spiWriteBytes(DS3234_REGISTER_BASE, time, TIME_ARRAY_LENGTH);
}

// autoTime -- Fill DS3234 time registers with compiler time/date
void DS3234::autoTime()
{
	_time[TIME_SECONDS] = DECtoBCD(BUILD_SECOND);
	_time[TIME_MINUTES] = DECtoBCD(BUILD_MINUTE);
	_time[TIME_HOURS] = BUILD_HOUR;
	// Convert hour to 12-hour if the DS3234 is in 12-hour mode:
	if (is12Hour())
	{
		uint8_t pmBit = 0;
		if (_time[TIME_HOURS] <= 11)
		{
			if (_time[TIME_HOURS] == 0)
				_time[TIME_HOURS] = 12;
		}
		else
		{
			pmBit = TWELVE_HOUR_PM;
			if (_time[TIME_HOURS] >= 13)
				_time[TIME_HOURS] -= 12;
		}
		DECtoBCD(_time[TIME_HOURS]);
		_time[TIME_HOURS] |= pmBit;
		_time[TIME_HOURS] |= TWELVE_HOUR_MODE;
	}
	else
	{
		_time[TIME_HOURS] = DECtoBCD(_time[TIME_HOURS]);
	}
	
	_time[TIME_MONTH] = DECtoBCD(BUILD_MONTH);
	_time[TIME_DATE] = DECtoBCD(BUILD_DATE);
        //! Not Y2K (or Y2.1K)-proof :
	_time[TIME_YEAR] = DECtoBCD(BUILD_YEAR - 2000); 
	
	// Calculate weekday (from here: http://stackoverflow.com/a/21235587)
	// Result: 0 = Sunday, 6 = Saturday
	int d = BUILD_DATE;
	int m = BUILD_MONTH;
	int y = BUILD_YEAR;
	int weekday = (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
	weekday += 1; // Library defines Sunday=1, Saturday=7
	_time[TIME_DAY] = DECtoBCD(weekday);
	
	
	setTime(_time, TIME_ARRAY_LENGTH);
}

// update -- Read all time/date registers and update the _time array
void DS3234::update(void)
{
	uint8_t rtcReads[TIME_ARRAY_LENGTH];
	
	spiReadBytes(DS3234_REGISTER_BASE, rtcReads, TIME_ARRAY_LENGTH);
	
	for (int i=0; i<TIME_ARRAY_LENGTH; i++)
	{
		_time[i] = rtcReads[i];
	}
	
	if (_time[TIME_HOURS] & TWELVE_HOUR_MODE)
	{
		if (_time[TIME_HOURS] & TWELVE_HOUR_PM)
			_pm = true;
		else
			_pm = false;
		
		_time[TIME_HOURS] &= 0x1F; // Mask out 24-hour bit, am/pm from hours
	}
	else
	{
		_time[TIME_HOURS] &= 0x3F; // Mask out 24-hour bit from hours
	}
}

// getSecond -- read/return seconds register of DS3234
uint8_t DS3234::getSecond(void)
{
	_time[TIME_SECONDS] = spiReadByte(DS3234_REGISTER_SECONDS);
	
	return BCDtoDEC(_time[TIME_SECONDS]);
}

// getMinute -- read/return minutes register of DS3234
uint8_t DS3234::getMinute(void)
{
	_time[TIME_MINUTES] = spiReadByte(DS3234_REGISTER_MINUTES);

	return BCDtoDEC(_time[TIME_MINUTES]);	
}

// getHour -- read/return hour register of DS3234
uint8_t DS3234::getHour(void)
{
	uint8_t hourRegister = spiReadByte(DS3234_REGISTER_HOURS);
	
	if (hourRegister & TWELVE_HOUR_MODE)
		hourRegister &= 0x1F; // Mask out am/pm, 24-hour bit
	_time[TIME_HOURS] = hourRegister;

	return BCDtoDEC(_time[TIME_HOURS]);
}

// getDay -- read/return day register of DS3234
uint8_t DS3234::getDay(void)
{
	_time[TIME_DAY] = spiReadByte(DS3234_REGISTER_DAY);

	return BCDtoDEC(_time[TIME_DAY]);		
}

// getDate -- read/return date register of DS3234
uint8_t DS3234::getDate(void)
{
	_time[TIME_DATE] = spiReadByte(DS3234_REGISTER_DATE);

	return BCDtoDEC(_time[TIME_DATE]);		
}

// getMonth -- read/return month register of DS3234
uint8_t DS3234::getMonth(void)
{
	_time[TIME_MONTH] = spiReadByte(DS3234_REGISTER_MONTH);
	_time[TIME_MONTH] &= 0x7F; // Mask out century bit

	return BCDtoDEC(_time[TIME_MONTH]);	
}

// getYear -- read/return year register of DS3234
uint8_t DS3234::getYear(void)
{
	_time[TIME_YEAR] = spiReadByte(DS3234_REGISTER_YEAR);

	return BCDtoDEC(_time[TIME_YEAR]);		
}

// setSecond -- set the second register of the DS3234
void DS3234::setSecond(uint8_t s)
{
	if (s <= 59)
	{
		uint8_t _s = DECtoBCD(s);
		spiWriteByte(DS3234_REGISTER_SECONDS, _s);
	}
}

// setMinute -- set the minute register of the DS3234
void DS3234::setMinute(uint8_t m)
{
	if (m <= 59)
	{
		uint8_t _m = DECtoBCD(m);
		spiWriteByte(DS3234_REGISTER_MINUTES, _m);		
	}
}

// setHour -- set the hour register of the DS3234
void DS3234::setHour(uint8_t h)
{
	//! Check if 24-hour mode, am/pm
	if (h <= 23)
	{
		uint8_t _h = DECtoBCD(h);
		spiWriteByte(DS3234_REGISTER_HOURS, _h);
	}
}

// setDay -- set the day register of the DS3234
void DS3234::setDay(uint8_t d)
{
	if ((d >= 1) && (d <= 7))
	{
		uint8_t _d = DECtoBCD(d); //! Unecessary?
		return spiWriteByte(DS3234_REGISTER_DAY, _d);
	}
}

// setDate -- set the date register of the DS3234
void DS3234::setDate(uint8_t d)
{
	if (d <= 31)
	{
		uint8_t _d = DECtoBCD(d);
		spiWriteByte(DS3234_REGISTER_DATE, _d);
	}
}

// setMonth -- set the month register of the DS3234
void DS3234::setMonth(uint8_t mo)
{
	if ((mo >= 1) && (mo <= 12))
	{
		uint8_t _mo = DECtoBCD(mo);
		spiWriteByte(DS3234_REGISTER_MONTH, _mo);
	}
}

// setYear -- set the year register of the DS3234
void DS3234::setYear(uint8_t y)
{
	if (y <= 99)
	{
		uint8_t _y = DECtoBCD(y);
		spiWriteByte(DS3234_REGISTER_YEAR, _y);
	}
}

// set12Hour -- set (or not) to 12-hour mode) | enable12 defaults to  true
void DS3234::set12Hour(bool enable12)
{
	if (enable12)
		set24Hour(false);
	else
		set24Hour(true);
}

// set24Hour -- set (or not) to 24-hour mode) | enable24 defaults to  true
void DS3234::set24Hour(bool enable24)
{
	uint8_t hourRegister = spiReadByte(DS3234_REGISTER_HOURS);
	
	bool hour12 = hourRegister & TWELVE_HOUR_MODE;
	if ((hour12 && !enable24) || (!hour12 && enable24))
		return;
	
	uint8_t oldHour;
	uint8_t newHour;
	
	if (enable24)
	{
		oldHour = hourRegister & 0x1F; // Mask out am/pm and 12-hour mode
		oldHour = BCDtoDEC(oldHour); // Convert to decimal
		newHour = oldHour;
		
		bool hourPM = hourRegister & TWELVE_HOUR_PM;
		if ((hourPM) && (oldHour >= 1)) newHour += 12;
		else if (!(hourPM) && (oldHour == 12)) newHour = 0;
		newHour = DECtoBCD(newHour);
	}
	else
	{
		oldHour = hourRegister & 0x3F; // Mask out am/pm and 12-hour mode
		oldHour = BCDtoDEC(oldHour); // Convert to decimal
		newHour = oldHour;
		
		if (oldHour == 0) 
			newHour = 12;
		else if (oldHour >= 13)
			newHour -= 12;
		
		newHour = DECtoBCD(newHour);
		newHour |= TWELVE_HOUR_MODE; // Set bit 6 to set 12-hour mode
		if (oldHour >= 12)
			newHour |= TWELVE_HOUR_PM; // Set PM bit if necessary
	}
	
	return spiWriteByte(DS3234_REGISTER_HOURS, newHour);
}

// is12Hour -- check if the DS3234 is in 12-hour mode
bool DS3234::is12Hour(void)
{
	uint8_t hourRegister = spiReadByte(DS3234_REGISTER_HOURS);
	
	return hourRegister & TWELVE_HOUR_MODE;
}

// pm -- Check if 12-hour state is AM or PM
bool DS3234::pm(void)
{
	uint8_t hourRegister = spiReadByte(DS3234_REGISTER_HOURS);
	
	return hourRegister & TWELVE_HOUR_PM;	
}

// enable -- enable the DS3234's oscillator.
void DS3234::enable(void)
{
	uint8_t controlRegister = spiReadByte(DS3234_REGISTER_CONTROL);
	
	controlRegister &= ~(1<<7);
	
	spiWriteByte(DS3234_REGISTER_CONTROL, controlRegister);
}

// disable -- disable the DS3234's oscillator
//  (Only effects chip when powered by battery.)
void DS3234::disable(void)
{
	uint8_t controlRegister = spiReadByte(DS3234_REGISTER_CONTROL);
	
	controlRegister |= (1<<7);
	
	spiWriteByte(DS3234_REGISTER_CONTROL, controlRegister);
}

// setAlarm1 -- Alarm 1 can be set to trigger on seconds, minutes, hours, and/or date/day.
// Any of those can be masked out -- ignored for an alarm match. By setting
// If a value is set to 255, the library will mask out that data.
// By default the "date" value is the day-of-month (1-31)
// The "day" boolean changes the "date" value to a day (between 1-7).
void DS3234::setAlarm1(uint8_t second, uint8_t minute, uint8_t hour, uint8_t date, bool day)
{
	uint8_t alarmRegister[4];
	uint8_t timeValue[4] = {second, minute, hour, date};
	uint8_t timeMin[4] = {0, 0, 0, 1};
	uint8_t timeMax[4] = {59, 59, 23, 31};
	if (day)
		timeMax[3] = 7;
	
	spiReadBytes(DS3234_REGISTER_A1SEC, alarmRegister, 4); // Read current alarm values
	
	// Run through all four alarm values and set their register values:
	for (int i=0; i<4; i++)
	{
		if (timeValue[i] == 255) // If 255, disable the check on that value
			alarmRegister[i] |= ALARM_MODE_BIT;
		else if ((timeValue[i] >= timeMin[i]) && (timeValue[i] <= timeMax[i]))
			alarmRegister[i] = DECtoBCD(timeValue[i]);
	}
	if (day) 
		alarmRegister[3] |= ALARM_DAY_BIT;
	
	spiWriteBytes(DS3234_REGISTER_A1SEC, alarmRegister, 4); // Write the values
}

// setAlarm1 (12-hour mode)
void DS3234::setAlarm1(uint8_t second, uint8_t minute, uint8_t hour12, bool pm, uint8_t date, bool day)
{
	uint8_t alarmRegister[4];
	uint8_t timeValue[4] = {second, minute, hour12, date};
	uint8_t timeMin[4] = {0, 0, 0, 1};
	uint8_t timeMax[4] = {59, 59, 23, 31};
	if (day)
		timeMax[3] = 7;
	
	spiReadBytes(DS3234_REGISTER_A1SEC, alarmRegister, 4); // Read current alarm values
	
	// Run through all four alarm values and set their register values:
	for (int i=0; i<4; i++)
	{
		if (timeValue[i] == 255) // If 255, disable the check on that value
			alarmRegister[i] |= ALARM_MODE_BIT;
		else if ((timeValue[i] >= timeMin[i]) && (timeValue[i] <= timeMax[i]))
			alarmRegister[i] = DECtoBCD(timeValue[i]);
	}
	if (day) 
		alarmRegister[3] |= ALARM_DAY_BIT;
	alarmRegister[2] |= TWELVE_HOUR_MODE;
	if (pm) alarmRegister[2] |= TWELVE_HOUR_PM;
	
	spiWriteBytes(DS3234_REGISTER_A1SEC, alarmRegister, 4); // Write the values
}

// setAlarm2 -- Alarm 2 can be set to trigger on minutes, hours, and/or date/day.
// Any of those can be masked out -- ignored for an alarm match. By setting
// If a value is set to 255, the library will mask out that data.
// By default the "date" value is the day-of-month (1-31)
// The "day" boolean changes the "date" value to a day (between 1-7).
void DS3234::setAlarm2(uint8_t minute, uint8_t hour, uint8_t date, bool day)
{
	uint8_t alarmRegister[3];
	uint8_t timeValue[3] = {minute, hour, date};
	uint8_t timeMin[3] = {0, 0, 1};
	uint8_t timeMax[3] = {59, 23, 31};
	if (day)
		timeMax[2] = 7;
	
	spiReadBytes(DS3234_REGISTER_A2MIN, alarmRegister, 3); // Read all alarm 2 registers
	
	for (int i=0; i<3; i++)
	{
		if (timeValue[i] == 255) // If a value is 255, disable that alarm check
			alarmRegister[i] |= ALARM_MODE_BIT;
		else if ((timeValue[i] >= timeMin[i]) && (timeValue[i] <= timeMax[i]))
			alarmRegister[i] = DECtoBCD(timeValue[i]);
	}
	if (day) 
		alarmRegister[2] |= ALARM_DAY_BIT;
	
	spiWriteBytes(DS3234_REGISTER_A2MIN, alarmRegister, 3);
}

// setAlarm2 (12-hour mode)
void DS3234::setAlarm2(uint8_t minute, uint8_t hour12, bool pm, uint8_t date, bool day)
{	
	uint8_t alarmRegister[3];
	uint8_t timeValue[3] = {minute, hour12, date};
	uint8_t timeMin[3] = {0, 0, 1};
	uint8_t timeMax[3] = {59, 23, 31};
	if (day)
		timeMax[2] = 7;
	
	spiReadBytes(DS3234_REGISTER_A2MIN, alarmRegister, 3); // Read all alarm 2 registers
	
	for (int i=0; i<3; i++)
	{
		if (timeValue[i] == 255) // If a value is 255, disable that alarm check
			alarmRegister[i] |= ALARM_MODE_BIT;
		else if ((timeValue[i] >= timeMin[i]) && (timeValue[i] <= timeMax[i]))
			alarmRegister[i] = DECtoBCD(timeValue[i]);
	}
	if (day) 
		alarmRegister[1] |= ALARM_DAY_BIT;
	alarmRegister[1] |= TWELVE_HOUR_MODE;
	if (pm) alarmRegister[1] |= TWELVE_HOUR_PM;
	
	spiWriteBytes(DS3234_REGISTER_A2MIN, alarmRegister, 3);
}

// alarm1 -- Check the alarm 1 flag in the status register
bool DS3234::alarm1(bool clear)
{
	uint8_t statusRegister = spiReadByte(DS3234_REGISTER_STATUS);
	
	if (statusRegister & ALARM_1_FLAG_BIT)
	{
		if (clear)
		{
			statusRegister &= ~(ALARM_1_FLAG_BIT);
			spiWriteByte(DS3234_REGISTER_STATUS, statusRegister);
		}
		return true;
	}
	
	return false;
}

// alarm2 -- Check the alarm 2 flag in the status register
bool DS3234::alarm2(bool clear)
{
	uint8_t statusRegister = spiReadByte(DS3234_REGISTER_STATUS);
	
	if (statusRegister & ALARM_2_FLAG_BIT)
	{
		if (clear)
		{
			statusRegister &= ~(ALARM_2_FLAG_BIT);
			spiWriteByte(DS3234_REGISTER_STATUS, statusRegister);
		}
		return true;	
	}
	
	return false;
}

// enableAlarmInterrupt -- Enable the SQW interrupt output on one, or both, alarms
void DS3234::enableAlarmInterrupt(bool alarm1, bool alarm2)
{
	uint8_t controlRegister = spiReadByte(DS3234_REGISTER_CONTROL);
	controlRegister |= ALARM_INTCN_BIT;
	if (alarm1) 
		controlRegister |= (1<<0);
	if (alarm2)
		controlRegister |= (1<<1);
	spiWriteByte(DS3234_REGISTER_CONTROL, controlRegister);
}

// writeSQW -- Set the SQW pin high, low, or to one of the square wave frequencies
void DS3234::writeSQW(sqw_rate value)
{
	uint8_t controlRegister = spiReadByte(DS3234_REGISTER_CONTROL);
	
	controlRegister &= SQW_CONTROL_MASK; // Mask out RS1, RS2 bits (bits 3 and 4)
	controlRegister |= (value << 3); // Add rate bits, shift left 3
	controlRegister &= ~(SQW_ENABLE_BIT); // Clear INTCN bit to enable SQW output
	spiWriteByte(DS3234_REGISTER_CONTROL, controlRegister);
}

// temperature -- Read the DS3234's die-temperature. 
//  Value is produced in multiples of 0.25 deg C
float DS3234::temperature(void)
{
	float retVal = 0;
	int8_t integer;
	uint8_t tempRegister[2];
	spiReadBytes(DS3234_REGISTER_TEMPM, tempRegister, 2);
	
	integer = tempRegister[0];
	tempRegister[1] = tempRegister[1] >> 6;
	retVal = integer + ((float) tempRegister[1] * 0.25);
	
	return retVal;
}

// BCDtoDEC -- convert binary-coded decimal (BCD) to decimal
uint8_t DS3234::BCDtoDEC(uint8_t val)
{
	return ( ( val / 0x10) * 10 ) + ( val % 0x10 );
}

// BCDtoDEC -- convert decimal to binary-coded decimal (BCD)
uint8_t DS3234::DECtoBCD(uint8_t val)
{
	return ( ( val / 10 ) * 0x10 ) + ( val % 10 );
}

// spiWriteBytes -- write a set number of bytes to an SPI device, incrementing from a register
void DS3234::spiWriteBytes(DS3234_registers reg, uint8_t * values, uint8_t len)
{
	uint8_t writeReg = reg | 0x80;
	
	SPI.beginTransaction(DS3234SPISettings);
	digitalWrite(_csPin, LOW);
	SPI.transfer(writeReg);
	for (int i=0; i<len; i++)
	{
		SPI.transfer(values[i]);
	}
	digitalWrite(_csPin, HIGH);
	SPI.endTransaction();
}

// spiWriteBytes -- write a byte value to an spi device's register
void DS3234::spiWriteByte(DS3234_registers reg, uint8_t value)
{
	uint8_t writeReg = reg | 0x80;
	
	SPI.beginTransaction(DS3234SPISettings);
	digitalWrite(_csPin, LOW);
	SPI.transfer(writeReg);
	SPI.transfer(value);
	digitalWrite(_csPin, HIGH);
	SPI.endTransaction();
}

// spiWriteBytes -- read a byte from an spi device's register
uint8_t DS3234::spiReadByte(DS3234_registers reg)
{
	uint8_t retVal = 0;
	SPI.beginTransaction(DS3234SPISettings);
	digitalWrite(_csPin, LOW);
	SPI.transfer(reg);
	retVal = SPI.transfer(0x00);
	digitalWrite(_csPin, HIGH);
	SPI.endTransaction();
	
	return retVal;
}

// spiWriteBytes -- read a set number of bytes from an spi device, incrementing from a register
void DS3234::spiReadBytes(DS3234_registers reg, uint8_t * dest, uint8_t len)
{
	SPI.beginTransaction(DS3234SPISettings);
	digitalWrite(_csPin, LOW);
	SPI.transfer(reg);
	for (int i=0; i<len; i++)
	{
		dest[i] = SPI.transfer(0x00);
	}
	digitalWrite(_csPin, HIGH);
	SPI.endTransaction();
}

void DS3234::writeToSRAM(uint8_t address, uint8_t data){
  spiWriteByte(DS3234_REGISTER_SRAMA, address);
  spiWriteByte(DS3234_REGISTER_SRAMD, data);
}

uint8_t DS3234::readFromSRAM(uint8_t address){
  spiWriteByte(DS3234_REGISTER_SRAMA, address);
  return spiReadByte(DS3234_REGISTER_SRAMD);
}


DS3234 rtc; // Use rtc in sketches
