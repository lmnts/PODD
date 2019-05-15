/*
 * pod_network.h  
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 */

#ifndef POD_NETWORK_H
#define POD_NETWORK_H

#include "Arduino.h"

//--------------------------------------------------------------------------------------------- [XBee Management]

// XBee packet buffering
#define XBEE_BUFFER_SIZE 256

// Use ASCII "start of text" and "end of text" control characters
// to mark the start and end of packets.  The use of both allows
// incomplete packets to be identified during buffer overruns.
#define PACKET_START_TOKEN '\x02'
#define PACKET_END_TOKEN '\x03'

void initXBee();
void setXBeeCoordinatorMode(const bool coord);
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

void xbeeGetMac(byte * macL, uint8_t max_mac_len);
void xbeeConfig();
void xbeeRate(String incoming);
void xbeeSettings(String incoming, String incoming2);
void xbeeReading(String incoming);
bool xbeeCommandMode();
void xbeeRequestSetting(String setting);
void xbeeUpdateSetting(String setting, String val);
bool xbeeWriteSettings();
void xbeeCloseCommand();
void xbeeGetNetwork(byte * netID);

//--------------------------------------------------------------------------------------------- [Upload Support]

#define ETHERNET_EN 43

void ethernetSetup();
bool ethernetBegin();
bool ethernetOnline();
void ethernetMaintain();
//String formatTime();
//String formatDate();
void saveReading(String lstr, String rstr, String atstr, String gtstr, String sstr, String c2str, String p1str, String p2str, String cstr);
void postReading(String DID, String ST, String R, String DT);
void updateRate(String DID, String ST, String R, String DT);
void updateConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID);
byte postPage(const char* domainBuffer, int thisPort, const char* page, const char* thisData);

//void getTimeFromWeb();
//void sendNTPpacket(const char* address);
void updateClockFromNTP();
void broadcastClock();
void processClockPacket(const String packet);

#endif
