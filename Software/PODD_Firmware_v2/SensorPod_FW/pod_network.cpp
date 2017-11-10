
/*
 * pod_network.cpp  
 * 2017 - Nick Turner and Morgan Redfield
 * Licensed under the AGPLv3. For full license see LICENSE.md 
 * Copyright (c) 2017 LMN Architects, LLC
 * 
 * Handle networking via Ethernet and XBee.
 */

#include "pod_network.h"
#include "pod_config.h"
#include "pod_logging.h"

#include <SPI.h>
#include "Ethernet.h"
#include "EthernetUdp.h"

#define xbee Serial1

// XBee packet buffering
char BuffXBee[BUFFXBEE_SIZE];
unsigned int BuffHead = 0;
unsigned int BuffTail = 0;
String set1;
String set2;
#define NETID_LEN 4
byte network[] = {0xA,0xB,0xC,0xD};

// Ethernet connection settings
#define MAC_LEN 6
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC address can be anything, as long as it is unique on network
//char timeServer[] =  "time.nist.gov"; // government NTP server
#define timeServer "time.nist.gov"
#define localPort 8888
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
EthernetUDP Udp;
#define serverPort 80
char pageName[] = "/LMNSensePod.php"; // Name of submission page. Log into EC2 Server and navigate to /var/www/html to view
int totalCount = 0;
char params[200];// insure params is big enough to hold your variables
#define delayMillis 10000UL // Delay between establishing connection and making POST request
unsigned long thisMillis = 0;
unsigned long lastMillis = 0;
#define WIZ812MJ_ES_PIN 20 // WIZnet SPI chip-select pin
#define WIZ812MJ_RESET_PIN 9 // WIZnet reset pin
bool online = false;

//--------------------------------------------------------------------------------------------- [XBee Management]

ISR(INT2_vect){
  //cli();

  if (xbee.available()){
    BuffXBee[BuffHead] = xbee.read();
    BuffHead++;
    if(BuffHead > BUFFXBEE_SIZE)
      BuffHead = 0;
  }

  //sei();
}

void xbeeSetup() {
  xbee.begin(9600);

  BuffHead = 0;
  BuffTail = 0;
}

void uploadXBee(){
  String incoming;

  //cli();
  if (xbee.available()){
    BuffXBee[BuffHead] = xbee.read();
    if(BuffXBee[BuffHead] != -1) {
      BuffHead++;
      if(BuffHead > BUFFXBEE_SIZE)
        BuffHead = 0;
    }
  }
  //sei();
  

  if (BuffTail == BuffHead)
    return; // we have nothing in the buffer

  // first get the type of packet
  char pkt_type = BuffXBee[BuffTail];

  // then get the packet itself
  unsigned int str_idx = BuffTail;
  while (BuffXBee[str_idx] != ';' && str_idx != BuffHead) {
    incoming.concat(BuffXBee[str_idx]);
    str_idx++;
    if (str_idx > BUFFXBEE_SIZE) {
      str_idx = 0;
    }
  }

  if (BuffXBee[str_idx] == ';' && str_idx != BuffHead) {
    // we got a full packet
    BuffTail = str_idx + 1;
  } else if (incoming.length() > 64) {
    // we got something corrupt, so drop it
    BuffTail = str_idx + 1;
    while (BuffTail != BuffHead && BuffXBee[BuffTail] != ';') {
      BuffTail++;
      if (BuffTail > BUFFXBEE_SIZE) {
        BuffTail = 0;
      }
    }
    return;
  } else {
    // didn't get a full packet, but we may yet
    return;
  }

  Serial.println(incoming);
  //return;
  switch(pkt_type) {
    case 'V':
      xbeeReading(incoming);
      break;
    case 'R':
      xbeeRate(incoming);
      break;
    case 'S':
      set1 = (incoming);
      if (set1.length() > 0 && set2.length() > 0) {
        xbeeSettings(set1, set2);
        set1 = "";
        set2 = "";
      }
      break;
    case 'T':
      set2 = (incoming);
      if (set1.length() > 0 && set2.length() > 0) {
        xbeeSettings(set1, set2);
        set1 = "";
        set2 = "";
      }
      break;
    default:
      // invalid packet, do nothing
      break;
  }
}


void xbeeConfig() {
  String meshMode;
  if (getModeCoord()) {
    meshMode = "1"; //coordinator
  } else {
    meshMode = "0"; //drone
  }

  xbeeUpdateSetting("CE","0"); // Set device to drone before entering new network.
  xbeeUpdateSetting("ID", getNetID());
  
  delay(1100);
  xbee.print("+++");
  delay(1100);
  Serial.print((char)xbee.read());
  xbee.print("ATCE " + meshMode + "\r");
  delay(100);
  Serial.print((char)xbee.read());
  xbee.print("ATWR\r");
  delay(100);
  Serial.print((char)xbee.read());
  xbee.print("ATCN\r");
  delay(100);
  while(xbee.available())
    xbee.read();

}

void xbeeGetMac(byte * macL, uint8_t max_mac_len){
  if (max_mac_len < 6) {
    Serial.println(F("mac array is too short"));
    return;
  }

  //byte macL[6];
  xbeeCommandMode();
  //Serial.println();
  xbee.print("ATSL\r");
  xbee.readBytes(macL,6);
  //xbee.print("ATCN\r");
  delay(100);
  while(xbee.available())
    //Serial.print((char)xbee.read());
    xbee.read();
  //Serial.println();
  //for(int x = 0; x<sizeof(macL);x++)
    //Serial.println(macL[x]);
  //return macL;
}

void xbeeGetNetwork(byte * netID, uint8_t max_net_len){
  //Serial.println(sizeof(netID));
  if (max_net_len < 4){
    Serial.println(F("Network ID array is too short"));
    return;
  }
  
  xbeeRequestSetting("ID");
  xbee.readBytes(netID,5);
  while(xbee.available()){
    Serial.println(F("Leftover Buffer: "));
    Serial.write((char)xbee.read());
  }
  xbeeCloseCommand();
  //Serial.write(netID, sizeof(netID));
}

void xbeeRate(String incoming){
  String did, sensor, val, datetime;
  int one,two,three, four;

  one = incoming.indexOf(',') +1;
  two = incoming.indexOf(',',one) +1;
  three = incoming.indexOf(',',two) +1;
  four = incoming.indexOf(',',three) +1;
  did = incoming.substring(one,two-1);
  sensor = incoming.substring(two, three-1);
  val = incoming.substring(three, four-1);
  datetime = incoming.substring(four);
  /*Serial.println(one);
  Serial.println(two);
  Serial.println(three);
  Serial.println(did);
  Serial.println(sensor);
  Serial.println(val);*/
  updateRate(did,sensor,val,datetime);
}

void xbeeSettings(String incoming, String incoming2){
  String did,location,project; 
  String coordinator,rate,build,teardown,datetime,netid;
  int one,two,three,four,five,six,seven,eight, nine;

  one = incoming.indexOf(',') +1;
  two = incoming.indexOf(',',one) +1;
  three = incoming.indexOf(',',two) +1;
  did = incoming.substring(one,two-1);
  location = incoming.substring(two, three-1);
  project = incoming.substring(three);
  
  four = incoming2.indexOf(',') +1;
  five = incoming2.indexOf(',',four) +1;
  six = incoming2.indexOf(',',five) +1;
  seven = incoming2.indexOf(',',six) +1;
  eight = incoming2.indexOf(',',seven) +1;
  nine = incoming2.indexOf(',',eight) +1;
  coordinator = incoming2.substring(four, five-1);
  rate = incoming2.substring(five,six-1);
  build = incoming2.substring(six,seven-1);
  teardown = incoming2.substring(seven,eight-1);
  datetime = incoming2.substring(eight, nine-1);
  netid = incoming2.substring(nine);
  
  updateConfig(did,location,coordinator,project,rate,build,teardown,datetime,netid);
}

void xbeeReading(String incoming){
  String did, sensor, val, datetime;
  int one,two,three, four;

  one = incoming.indexOf(',') +1;
  two = incoming.indexOf(',',one) +1;
  three = incoming.indexOf(',',two) +1;
  four = incoming.indexOf(',',three) +1;
  did = incoming.substring(one,two-1);
  sensor = incoming.substring(two, three-1);
  val = incoming.substring(three, four-1);
  datetime = incoming.substring(four);
  Serial.println(one);
  Serial.println(two);
  Serial.println(three);
  Serial.println(did);
  Serial.println(sensor);
  Serial.println(val);
  Serial.println(datetime);
  postReading(did,sensor,val,datetime);
}


void XBeeSend(String message)
{
  #ifdef DEBUG
  writeDebugLog(F("Fxn: XBeeSend"));
  #endif
  char tx[message.length()+1];
  message.toCharArray(tx,message.length()+1);

  Serial.println(tx);
  
  xbee.write(tx);
}

void xbeeRequestSetting(String setting){
  xbeeCommandMode();
  xbee.print("AT" + setting + "\r");
  delay(200);
  
}

void xbeeUpdateSetting(String setting, String val) {
  xbeeCommandMode();
  xbee.print("AT" + setting + " " + val + "\r");
  delay(100);
  while(xbee.available()){
    xbee.read();
  }
  xbeeWriteSettings();
  xbeeCloseCommand();
}

bool xbeeCommandMode() {
  byte ack[2];
  delay(1100);
  xbee.print("+++");
  delay(1100);
  xbee.readBytes(ack,2);
  if(ack[0] != 'O' && ack[1] != 'K'){
    return false;
  }
  while(xbee.available())
    xbee.read();
  return true;
}

bool xbeeWriteSettings(){
  byte ack[2];
  xbee.print("ATWR\r");
  delay(100);
  xbee.readBytes(ack,2);
  if(ack[0] != 'O' && ack[1] != 'K'){
    return false;
  }
  return true;
}

void xbeeCloseCommand(){
  xbee.print("ATCN\r");
  delay(100);
  while(xbee.available())
    xbee.read();
}

//--------------------------------------------------------------------------------------------- [Upload Support]

void ethernetSetup() {
  // Ethernet connection
  
  pinMode(ETHERNET_EN, OUTPUT);
  digitalWrite(ETHERNET_EN, HIGH);
  delay(10);

  pinMode(WIZ812MJ_RESET_PIN,OUTPUT);
  digitalWrite(WIZ812MJ_RESET_PIN,LOW);
  delay(2);
  digitalWrite(WIZ812MJ_RESET_PIN,HIGH);

  pinMode(WIZ812MJ_ES_PIN, OUTPUT); 
  Ethernet.init(WIZ812MJ_ES_PIN);


  Serial.println(F("Starting ethernet..."));
  unsigned long eth_timeout = 10000;
  xbeeGetMac(mac, MAC_LEN);
  Serial.println(F("got mac..."));
  //Serial.write(mac,6);
  if(!Ethernet.begin(mac, eth_timeout)){
    Serial.println(F("failed. Readings will not be pushed to remote database."));
  }
  else {
    online = true;
    Serial.println(Ethernet.localIP());
    Udp.begin(localPort);
    getTimeFromWeb();
  }
}

bool ethernetOnline() {
  return online;
}

void ethernetMaintain() {
    Ethernet.maintain(); // Must be performed regularly to maintain connection
}

void saveReading(String lstr, String rstr, String atstr, String gtstr, String sstr, String c2str, String p1str, String p2str, String cstr) {
  String Dstamp = formatDate();
  String Tstamp = formatTime();
  String DTstamp = Dstamp + " " + Tstamp;
  String sensorData = (Dstamp + ", " + Tstamp + ", " + lstr + ", " + rstr + ", " + atstr + ", " + gtstr + ", " + sstr + ", " + c2str + ", " + p1str + ", " + p2str + ", " + cstr);
  logDataSD(sensorData);

  if (lstr != "")
    postReading(getDevID(), "Light", lstr, DTstamp);
  if (rstr != "")
    postReading(getDevID(), "Humidity", rstr, DTstamp);
  if (atstr != "")
    postReading(getDevID(), "AirTemp", atstr, DTstamp);
  if (gtstr != "")
    postReading(getDevID(), "GlobalTemp", gtstr, DTstamp);
  if (sstr != "")
    postReading(getDevID(), "Sound", sstr, DTstamp);
  if (c2str != "")
    postReading(getDevID(), "CO2", c2str, DTstamp);
  if (p1str != "")
    postReading(getDevID(), "PM_2.5", p1str, DTstamp);
  if (p2str != "")
    postReading(getDevID(), "PM_10", p2str, DTstamp);
  if (cstr != "")
    postReading(getDevID(), "CO", cstr, DTstamp);
}

void postReading(String DID, String ST, String R, String DT)
{
  #ifdef DEBUG
  writeDebugLog(ST);
  #endif
  if(getModeCoord()){
    // String in temp string is data to be submitted to MySQL
    char p[200];
    String amp = "&";
    //String Datetime = getStringDatetime();
    String temp = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "Reading=" + R + amp + "ReadTime=" + DT;
    temp.toCharArray(p,200);
    if(!postPage(getServer(),serverPort,pageName,p)){
      Serial.println(F("Failed to upload sensor reading to remote. \n"));
      #ifdef DEBUG
      writeDebugLog(F("Failed to upload sensor reading to remote. \n"));
      #endif
    }
    else Serial.println(F("Pass \n"));
  } else {
    String message = "V,"+DID+","+ST+","+R+","+DT+";"; 
    XBeeSend(message);
    delay(1000);
  }
}

void updateRate(String DID, String ST, String R, String DT)
{
  if(getModeCoord()){
    char p[200];
    String amp = "&";
    //String Datetime = getStringDatetime();
    String temp = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "SampleRate=" + R + amp + "RateChange=" + DT;
    temp.toCharArray(p,200);
    if(!postPage(getServer(),serverPort,pageName,p))Serial.print(F("Failed to update sensor rate on remote. "));
    else Serial.print(F("Pass "));
  } else {
    String message = "R,"+DID+","+ST+","+R+","+DT+";";
    Serial.println("XBee String: " + message);
    XBeeSend(message);
    delay(2500);
  }
}

void updateConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID)
{
  Datetime = getStringDatetime();
  if(getModeCoord()) {
    char p[200];
    String amp = "&";
    String temp = ("DeviceID=" + DID + amp + "Project=" + Project + amp + "Coordinator=" + Coordinator+ amp + "UploadRate=" + Rate + amp + "Location=" + Location + amp + "SetupDate=" + Setup + amp + "TeardownDate=" + Teardown + amp + "ConfigChange=" + Datetime + amp + "NetID=" + NetID);
    temp.toCharArray(p,200);
    if(!postPage(getServer(),serverPort,pageName,p))Serial.print(F("Failed to update device configuration on remote. "));
    else Serial.print(F("Pass "));
  } else {
    String message = "S,"+DID+","+Location+","+Project+";"; // Out of order from function call to balance XBee packet size
    String message2 = "T,"+Coordinator+","+Rate+","+Setup+","+Teardown+","+Datetime+","+NetID+";"; // out of order from function call to balance XBee packet size
    Serial.println("XBee String: " + message + message2+" " + message.length() + " " + message2.length());
    XBeeSend(message);
    delay(2000);
    XBeeSend(message2);
  }
}

// postPage is function that performs POST request and prints results.
byte postPage(char* domainBuffer, int thisPort, char* page, char* thisData)
{
  #ifdef DEBUG
  writeDebugLog(F("Fxn: postPage()"));
  #endif
  //int inChar;
  char outBuf[200];
  EthernetClient client;

  //Serial.print(F("connecting..."));

  if(client.connect(domainBuffer,thisPort) == 1)
  {
    //Serial.println(F("connected"));
    sprintf(outBuf,"POST %s HTTP/1.1",page);
    client.println(outBuf);
    sprintf(outBuf,"Host: %s",domainBuffer);
    client.println(outBuf);
    client.println(F("Connection: close\r\nContent-Type: application/x-www-form-urlencoded"));
    sprintf(outBuf,"Content-Length: %u\r\n",strlen(thisData));
    client.println(outBuf);

    client.print(thisData);
  }
  else
  {
    //Serial.println(F("failed"));
    return 0;
  }

  client.stop();

  return 1;
}

void getTimeFromWeb(){
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket() == NTP_PACKET_SIZE) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print(F("Seconds since Jan 1 1900 = "));
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print(F("Unix time = "));
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    const unsigned long sevenHours = 25200UL; //Time zone difference.
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears - sevenHours;
    // print Unix time:
    if (epoch > SEC_2017) {
        setRTCTime(epoch);
    }

  } else Serial.println(F("Unable to get server time"));
}

// send an NTP request to the time server at the given address
void sendNTPpacket(char* address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

