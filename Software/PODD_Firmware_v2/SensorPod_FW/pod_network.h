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

#define BUFFXBEE_SIZE 128

void xbeeSetup();
void xbeeGetMac(byte * macL, uint8_t max_mac_len);
void uploadXBee();
void xbeeConfig();
void xbeeRate(String incoming);
void xbeeSettings(String incoming, String incoming2);
void xbeeReading(String incoming);
void XBeeSend(String message);
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
String formatTime();
String formatDate();
void saveReading(String lstr, String rstr, String atstr, String gtstr, String sstr, String c2str, String p1str, String p2str, String cstr);
void postReading(String DID, String ST, String R, String DT);
void updateRate(String DID, String ST, String R, String DT);
void updateConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID);
byte postPage(const char* domainBuffer, int thisPort, const char* page, const char* thisData);
void getTimeFromWeb();
void sendNTPpacket(const char* address);

#endif
