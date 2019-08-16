/*
 * pod_network.h  
 * 2017 - Nick Turner and Morgan Redfield
 * 2018 - Chris Savage
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_NETWORK_H
#define POD_NETWORK_H

#include "Arduino.h"

//--------------------------------------------------------------------------------------------- [XBee Management]

// XBee command mode routines:
// used by higher-level XBee routines below
bool startXBeeCommandMode();
void stopXBeeCommandMode(const bool write=true);
bool submitXBeeCommand(const String cmd);
String getXBeeCommandResponse(const String cmd);
uint32_t getXBeeNumericResponse(const String cmd);
uint64_t getXBeeSerialNumber();
uint64_t getXBeeDestination();
void setXBeeDestination(const uint64_t dest);

String getXBeeSerialNumberString();
String getXBeeDestinationString();

// XBee routines
void initXBee();
void configureXBee(const bool coord);
void startXBee();
void readXBeeISR();
void readXBee();
void sendXBee(const String packet);
void broadcastXBee(const String packet);
bool holdXBeeBuffer();
void releaseXBeeBuffer();
void resetXBeeBuffer();
void cleanXBeeBuffer(const bool cleanStart=true, const bool cleanEnd=true);
String getXBeeBufferPacket();
void processXBee();
bool submitXBeeCommand(const String cmd);

void xbeeRate(String incoming);
void xbeeSettings(String incoming, String incoming2);
void xbeeReading(String incoming);

void broadcastCoordinatorAddress();
void processDestinationPacket(const String packet);


//--------------------------------------------------------------------------------------------- [Upload Support]

#define ETHERNET_EN 43

void initMACAddress();
String getMACAddressString();

void ethernetSetup();
bool ethernetBegin(int attempts=1);
bool ethernetHasIPAddress();
bool ethernetConnected();
void ethernetMaintain();
//String formatTime();
//String formatDate();
void saveReading(String lstr, String rstr, String atstr, String gtstr, String sstr, String c2str, String p1str, String p2str, String cstr);
void postReading(String DID, String ST, String R, String TS, String DT);
void updateRate(String DID, String ST, String R, String DT);
void updateConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID);
byte postPage(const char* domainBuffer, int thisPort, const char* page, const char* thisData);

//void getTimeFromWeb();
//void sendNTPpacket(const char* address);
void updateClockFromNTP();
void broadcastClock();
void processClockPacket(const String packet);

#endif
