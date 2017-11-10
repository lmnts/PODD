/*
This library is intended for use with the DS3234 real time clock and was created to utilize the Alarm functions on the RTC. 
This library uses the dedicated SPI hardware on the Atmel Atmege328p (arduino uno, pro mini) and will not work on other micros.
Read comments in header file for details on how to use the library
Created by Akram Ali and Zachary Zanzinger at the Illinoiis Institute of Technology for the Opens Source Building
Science Sensors Project. 
*/

#ifndef DS3234lib3_h
#define DS3234lib3_h
#include <EEPROM.h>
#include "Arduino.h"


class DS3234
{
  public:
    DS3234();        //Constructor to initialize DS3234 and choose slave select pin.  Use arduino digital pin 10 for SS if available.
                                    //If SSpin other than arduino digital pin 10 is used, pin 10 must still be written to output
    void fetchAndSetTime();   //Fetches time from computer using OSBSS processing sketch
    String timeStamp();       //
    void checkInterval(int hour, int minute, long interval);
		void checkDST();
		void startDST();
		void stopDST();
		void disableAlarms();
		void alarmFlagClear();
    void spiInit();
    void minuteAlarmBegin();
		void secondAlarmBegin();
		void secondAlarmSet(int s);
		void minuteAlarmSet(int m);
		void hourAlarmSet(int m, int h);
		int GetSeconds();
		void setNewAlarm(long _interval);
		void setNextAlarm();
    boolean alarm2set(int date, int hour, int minute);
    byte ConvertIntToPackedBCD(int integer);
		void getLaunchParameters(long &_interval, int &_dayStart, int &_hourStart, int &_minStart);
		void parseTimeStamp();
		int year;
		int month;
		int day;
		int hour;
		int minute;
		int second;
		
  private:
		int timestamp[18];
		long interval;
		long t;
		int hr;
		int min;
		int sec;
		byte alarmType;
};

#endif
