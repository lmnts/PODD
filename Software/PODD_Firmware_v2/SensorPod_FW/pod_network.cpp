
/*
   pod_network.cpp
   2017 - Nick Turner and Morgan Redfield
   Licensed under the AGPLv3. For full license see LICENSE.md
   Copyright (c) 2017 LMN Architects, LLC

   Handle networking via Ethernet and XBee.
*/

#include "pod_network.h"
#include "pod_config.h"
#include "pod_logging.h"
#include "pod_util.h"
#include "pod_clock.h"

#include <SPI.h>
#include <TimerOne.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define xbee Serial1

// XBee packet buffering
// If there is sufficient dynamic memory remaining for firmware
// operation, might want to increase XBee buffer size to reduce
// packet loss due to buffer overruns.
#define XBEE_BUFFER_SIZE 256
volatile char xbeeBuffer[XBEE_BUFFER_SIZE];
volatile size_t xbeeBufferHead = 0;
volatile size_t xbeeBufferElements = 0;
volatile size_t xbeeBufferOverrun = 0;
volatile bool xbeeBufferHold = false;

// Use ASCII "start of text" and "end of text" control characters
// to mark the start and end of packets.  The use of both allows
// incomplete packets to be identified during buffer overruns.
#define PACKET_START_TOKEN '\x02'
#define PACKET_END_TOKEN '\x03'

// In addition to the use of the above tokens, a two-digit
// hexadecimal packet length (modulo 256) is prepended to a packet
// when sent over the XBee network; incoming packets are dropped
// if the length does not match.  That may occur if the serial
// interface missed a character or the serial buffer overflowed,
// which may happen if any firmware routines prevent background
// ISRs from running.

// How frequently data is pulled from hardware serial buffer (microseconds)
// through the use of a timer-driven interrupt service routine (ISR).
// Arduino buffer is size 64 (for Teensy++ 2.0 as of Arduino 1.8.5);
// at 9600 baud, should pull data off buffer at least every
// 64*10/9600 seconds = 67ms (8-N-1: 10 serial bits sent per byte of data).
//#define XBEE_READ_INTERVAL 67000
// ...However, trying to read a (nearly) full buffer can take a while,
// which can delay other ISRs from performing their duties (like low-level
// serial interface I/O ISRs).  More frequent reads mean shorter time
// spent in each ISR call.
#define XBEE_READ_INTERVAL 10000
// Other timing considerations: Serial1 hardware register on Teensy++ 2.0
// contains a single byte of received data? [UNCLEAR]  Serial1's own ISR
// must be able to run as rapidly as that register fills (~1ms); our XBee
// read ISR run time must be shorter than that so it does not interfere
// with the Serial1 ISR timing.

String set1;
String set2;
//#define NETID_LEN 4
//byte network[] = {0xA, 0xB, 0xC, 0xD};

// Ethernet connection settings
#define MAC_ADDRESS_LEN 6
byte ethMACAddress[MAC_ADDRESS_LEN];
#define MAC_LEN 6
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC address can be anything, as long as it is unique on network
//char timeServer[] =  "time.nist.gov"; // government NTP server
//#define timeServer "time.nist.gov"
#define localPort 8888
//#define NTP_PACKET_SIZE 48
//byte packetBuffer[NTP_PACKET_SIZE];
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

// Maximum amount of time to allow ethernet to obtain connection.
#define ETHERNET_START_TIMEOUT 5000

// Number of packets uploaded to server (successfully or unsuccessfully)
unsigned long packetsUploaded = 0;

// Maximum amount of time [ms] to wait for a server response when
// posting data.  If PODD closes connection too early, the server
// may not process the POST request.  However, if this is too long
// and the server is in fact inaccessible, PODD sensor reads can be
// delayed (those reads should still be available through the SD
// data logs).
#define HTTP_POST_TIMEOUT 250

// NTP settings
#define NTP_SERVER "time.nist.gov"
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48




//--------------------------------------------------------------------------------------------- [XBee Management]

// Minimum amount of time between network reconnect attempts [ms].
// Prevents frequent restarts if the network is unavailable.
#define NETWORK_RECONNECT_INTERVAL (5*60000UL)
// Number of consecutive unsuccessful network interactions before
// attempting a network reconnect.  Occasional fails may be due
// to network instability or congestion instead of an unconnected
// network.
#define NETWORK_FAIL_EVENTS 2
// Minimum length of time for network interactions to be unsuccessful
// before attempting a network reconnect [ms].  Short blips may occur
// even when network is "connected".
#define NETWORK_FAIL_TIME (10000UL)

/* Structure that keeps track of the network status via recent activity
   to determine if/when the network needs to be reinitialized/restarted. */
struct NetworkStatus {
  // Whether last network interaction succeeded
  bool _connected = false;
  // Number of consecutive successful or unsuccessful network
  // interactions (depending on connected status)
  int consecutive = 0;
  // Most recent time attempt was made to initialize/start network
  unsigned long tstart = 0;
  // Earliest time of current string of successful network interactions
  // (0 if last network connection unsuccessful)
  unsigned long tsuccess = 0;
  // Earliest time of current string of unsuccessful network interactions
  // (0 if last network connection successful)
  unsigned long tfail = 0;

  // Indicates if last network interaction succeeded
  bool connected() {return _connected;}
  // Indicates if network should be (re)initialized/started
  bool needsRestart() {
    if (_connected) return false;
    if (tstart == 0) return true;
    unsigned long t0 = millis();
    if ((tstart > 0) && (t0 - tstart < NETWORK_RECONNECT_INTERVAL)) return false;
    if ((tfail > 0) && (t0 - tfail < NETWORK_FAIL_TIME)) return false;
    if (consecutive < NETWORK_FAIL_EVENTS) return false;
    return true; 
  }
  // Resets struct fields (intended for internal use)
  void reset(bool success) {
    _connected = success;
    consecutive = 0;
    tsuccess = success ? millis() : 0;
    tfail = success ? 0 : millis();
  }
  // Call when a network (re)start attempt is made, with the
  // success of that event.
  void restarted(bool success) {
    tstart = millis();
    reset(success);
  }
  // Call when a network interaction succeeds
  void succeeded() {
    if (!_connected) reset(true);
    consecutive++;
  }
  // Call when a network interaction fails
  void failed() {
    if (_connected) reset(false);
    consecutive++;
  }
};

NetworkStatus ethStatus;


//--------------------------------------------------------------------------------------------- [XBee Management]

/* Initialize the XBee serial interface and buffering mechanism.
   Must call startXBee() to start listening to XBee. */
void initXBee() {
  // Start serial interface between XBee and microcontroller.
  // Note there are multiple levels of buffers: the XBee
  // contains send/receive buffers and the Arduino core libraries
  // implement 64-byte microcontroller buffers for sending/
  // receiving data to/from the XBee.  On top of that, we have
  // our own buffer for received data to overcome limitations in
  // the Arduino receive buffer....
  xbee.begin(9600);

  // Initialize ring buffer for data that came across the XBee
  // network.
  xbeeBufferHold = true;
  resetXBeeBuffer();
}


/* Sends the given command to the XBee, indicating if the command
   was successfully sent.  XBee must already be in coordinator mode.
   Do not include trailing '\r'. */
bool submitXBeeCommand(const String cmd) {
  // Clear incoming buffer
  while (xbee.available()) xbee.read();
  // Submit command
  xbee.print(cmd + "\r");
  // Wait for response, which should be "OK\r" or "ERROR\r"
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (xbee.available() >= 3) break;
    delay(10);
  }
  if (!xbee.available()) return false;
  // In case last few characters have not yet arrived...
  delay(5);
  char buff[7];
  size_t len = 0;
  while (xbee.available()) {
    buff[len++] = xbee.read();
    if (len > 5) break;
  }
  buff[len] = '\0';
  // Clear remaining incoming buffer
  while (xbee.available()) xbee.read();
  // Check response
  String response(buff);
  if (response.equals(F("OK\r"))) return true;
  if (response.equals(F("ERROR\r"))) return false;
  // Missing/invalid/unknown response
  return false;
}


/* Sends the given command to the XBee and returns the response.
   Returns empty string if communication was unsuccessful.
   XBee must already be in coordinator mode.
   Do not include trailing '\r'. */
String getXBeeCommandResponse(const String cmd) {
  // Clear incoming buffer
  while (xbee.available()) xbee.read();
  // Submit command
  xbee.print(cmd + "\r");
  const int BUF_LEN = 16;
  size_t pos = 0;
  char buf[BUF_LEN];
  // Wait for response until '\r'
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (xbee.available()) {
      char c = xbee.read();
      if (c == '\r') break;
      if (c == '\n') break;
      buf[pos++] = c;
      if (pos >= BUF_LEN) break;
    }
    delay(5);
  }
  // Check for overflow
  if (pos >= BUF_LEN) pos = BUF_LEN - 1;
  buf[pos] = '\0';
  // Clear remaining incoming buffer
  while (xbee.available()) xbee.read();
  //Serial.print(F("XBee response: "));
  //Serial.println(buf);
  return String(buf);
}


/* Sends the given command to the XBee and returns the numeric response.
   Returns 0 if communication was unsuccessful or response was not
   hexadecimal.  XBee must already be in coordinator mode.
   Do not include trailing '\r'. */
uint32_t getXBeeNumericResponse(const String cmd) {
  String response = getXBeeCommandResponse(cmd);
  uint32_t v = 0;
  for (size_t k = 0; k < response.length(); k++) {
    char c = response.charAt(k);
    if ((c >= '0') && (c <= '9')) {
      v = (v << 4) + (uint8_t)(c - '0');
    } else if ((c >= 'A') && (c <= 'F')) {
      v = (v << 4) + (uint8_t)(c - 'A' + 10);
    } else if ((c >= 'a') && (c <= 'f')) {
      v = (v << 4) + (uint8_t)(c - 'a' + 10);
    } else {
      return 0;
    }
  }
  return v;
}


/* Gets the XBee serial number.  64-bit number is divided into
   high and low 32-bit parts.  Returns 0 is could not obtain
   the serial number. */
void getXBeeSerialNumber(uint32_t &SH, uint32_t &SL) {
  // Put XBee in command mode
  delay(1100);
  xbee.print(F("+++"));
  delay(1100);
  while (xbee.available()) xbee.read();

  // Get serial numbers
  SH = getXBeeNumericResponse("ATSH");  // For XBee S3B, this is 0x0013A2XX
  SL = getXBeeNumericResponse("ATSL");

  // Exit command mode
  submitXBeeCommand(F("ATCN"));
}


/* Sets the XBee as coordinator (true) or drone (false).
   The destination address is set to the broadcast or
   coordinator address, respectively. */
void setXBeeCoordinatorMode(const bool coord) {
  // XBee responds with "OK\r" or "ERROR\n" after each sent
  // command.
  
  // Put XBee in command mode
  delay(1100);
  xbee.print(F("+++"));
  delay(1100);
  while (xbee.available()) xbee.read();

  // Settings for coordinator
  if (coord) {
    // Set as coordinator
    submitXBeeCommand(F("ATCE 1"));
    // Set destination to broadcast address (0x000000000000FFFF).
    // Note command string omits '0x'.
    submitXBeeCommand(F("DH 00000000"));
    submitXBeeCommand(F("DL 0000FFFF"));

  // Settings for drone
  } else {
    // Set as non-coordinator
    submitXBeeCommand(F("ATCE 0"));
    // Set destination to coordinator address (0x0000000000000000).
    // Note command string omits '0x'.
    xbee.print(F("DH 00000000"));
    xbee.print(F("DL 00000000"));
  }
  
  // Save new configuration and exit command mode (applies changes)
  submitXBeeCommand(F("ATWR"));
  submitXBeeCommand(F("ATCN"));
}


/* Begin XBee processing.  Starts a timer-based ISR to grab data
   from the Arduino buffer into a ring buffer for more leisurely
   processing. */
void startXBee() {
  // Clear buffers and start fresh.
  // First disable interrupts to prevent ISRs from changing buffers.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  xbee.clear();
  resetXBeeBuffer();
  xbeeBufferHold = false;
  SREG = oldSREG;  // Restore interrupt status

  // Use timer-based, interrupt-driven function calls to
  // ensure data is getting pulled from the Arduino buffer
  // before it can fill.
  Timer1.initialize(XBEE_READ_INTERVAL);
  Timer1.attachInterrupt(readXBeeISR);
}


/* Reads data from the XBee serial interface into a circular buffer.
   A wrapper to the readXBee() function that checks a flag to avoid
   modifying the buffer if the buffer is being accessed elsewhere,
   making this routine a thread-safe ISR when the main thread is
   accessing the buffer (assuming the main thread sets the flag). */
void readXBeeISR() {
  // Avoid modifying the buffer if currently in use by main thread
  // routines (which should set this flag).  Allows this routine
  // to be safely interrupt-driven.
  if (xbeeBufferHold) return;
  //Serial.print(F("readXBeeISR: "));
  //Serial.println(millis());
  readXBee();
}


/* Reads data from the XBee serial interface into a circular buffer.
   Note Arduino uses interrupts to grab hardware serial data as it
   arrives, placing it into a 64 character buffer (for Teensy++ 2.0,
   as of Arduino 1.8.5).  This routine pulls data from that buffer
   into our own, larger buffer, which reduced the chance of overflow
   and allows for better overflow handling.  This routine should be
   called often to ensure the Arduino buffer does not overflow and
   data is lost. */
void readXBee() {
  // Define XBEE_DEBUG for verbose XBee debugging output.
  // This can considerably slow this ISR-called routine
  // and use of Serial output in an ISR can introduce
  // unintended behaviour.
#if defined(XBEE_DEBUG)
  if (!xbee.available()) return;
  Serial.print(F("readXBee ["));
  Serial.print(millis());
  Serial.print("]: ");
#endif
  // Will extract all currently available data
  while (xbee.available()) {
    // If buffer overruns, ignore incoming data.
    if (xbeeBufferElements >= XBEE_BUFFER_SIZE) {
      // Standard serial: continue through loop to clear buffer
      //xbee.read();
      //xbeeBufferOverrun++;
      // Teensy serial: can clear buffer all at once
      xbeeBufferOverrun += xbee.available();
      xbee.clear();
#if defined(XBEE_DEBUG)
      Serial.println(F("(overflow)"));
#endif
      return;
    } else {
      xbeeBuffer[xbeeBufferHead] = xbee.read();
      //Serial.print(xbeeBuffer[xbeeBufferHead]);
      xbeeBufferHead = (xbeeBufferHead + 1) % XBEE_BUFFER_SIZE;
      xbeeBufferElements++;
    }
  }
#if defined(XBEE_DEBUG)
  Serial.println();
#endif
}


/* Send the given packet over the XBee network to the coordinator.
   Adds start & stop tokens and 2-digit packet hex length prefix to
   help coordinator with packet parsing. */
void sendXBee(const String packet)
{
  // Serial output should be flushed here as activity may
  // interfere with XBee communication.
  Serial.print(F("XBee send: "));
  Serial.println(packet);
  Serial.flush();
  
  // Length of packet, in 2-digit hex (mod 256)
  char lbuf[3];
  sprintf(lbuf,"%02X",(uint8_t)(packet.length() % 256));
  lbuf[2] = '\0';
  
  // Would probably work...
  //xbee.write(PACKET_START_TOKEN);
  //xbee.write(lbuf);
  //xbee.write(packet);
  //xbee.write(PACKET_END_TOKEN);

  // Send payload to serial all at once in the hopes that the
  // XBee will place it in a single network packet.
  // By default, data in XBee input buffer is sent when packet
  // is full or 3 UART character transmission times have passed
  // without activity.
  // Note we omit null-termination character.
  //size_t bufLength = packet.length() + 2;
  size_t bufLength = packet.length() + 4;
  char buf[bufLength];
  //strcpy(&buf[1], packet.c_str());
  strcpy(&buf[1], lbuf);
  strcpy(&buf[3], packet.c_str());
  buf[0] = PACKET_START_TOKEN;
  buf[bufLength - 1] = PACKET_END_TOKEN;
  xbee.write(buf);

  // Hardware serial interface is operated through ISRs.
  // Give dedicated time here for those ISRs to run as any
  // routines that delay the ISRs risk causing I/O errors and
  // garbage appearing in the data packets.  This is a
  // precaution and may not be necessary if the rest of the
  // code is well-behaved....
  // Time: 1000*10/9600 ms per character @ 9600 baud, but add
  // add extra time as there may be gaps between characters.
  //delay(1000 * 15 * bufLength / 9600);
  // Even better: flush() waits for Arduino serial output buffer
  // to finish sending to the XBee.  Then add delay for XBee to
  // upload data over network.
  xbee.flush();
  delay(100);
}


/* Prevent the XBee buffer from being modified by ISR.
   Returns the prior hold state. */
bool holdXBeeBuffer() {
  // Disable interrupts to prevent ISR from running during routine.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();
  bool prevState = xbeeBufferHold;
  xbeeBufferHold = true;
  SREG = oldSREG;  // Restore interrupt status
  return prevState;
}


/* Allows the XBee buffer to be modified by ISR. */
void releaseXBeeBuffer() {
  // Disable interrupts to prevent ISR from running during routine.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();
  xbeeBufferHold = false;
  SREG = oldSREG;  // Restore interrupt status
}


/* Resets the XBee buffer to its empty state. */
void resetXBeeBuffer() {
  // Disable interrupts to prevent ISR from changing the buffer.
  // Store previous interrupt state so we can restore it afterwards.
  uint8_t oldSREG = SREG;  // Save interrupt status (among other things)
  cli();  // Disable interrupts
  xbeeBufferHead = 0;
  xbeeBufferElements = 0;
  xbeeBufferOverrun = 0;
  SREG = oldSREG;  // Restore interrupt status
}


/* Removes buffer data before the first packet and or after the
   last complete packet. The latter should be performed on a
   buffer overrun as data beyond the last complete packet belongs
   to a packet where data was thrown out. */
void cleanXBeeBuffer(const bool cleanStart, const bool cleanEnd) {
  // Prevent ISR from modifying buffer during this routine.
  // Will return to previous hold state.
  bool wasHeld = holdXBeeBuffer();

  // Empty buffer
  if (xbeeBufferElements == 0) {
    //xbeeBufferHead = 0;
    if (!wasHeld) releaseXBeeBuffer();
    return;
  }
  // Remove everything prior to first start token
  if (cleanStart) {
    while ((xbeeBufferElements > 0) && (xbeeBuffer[(xbeeBufferHead - xbeeBufferElements) % XBEE_BUFFER_SIZE] != PACKET_START_TOKEN)) {
      xbeeBufferElements--;
    }
  }
  // Remove everything after last end token
  if (cleanEnd) {
    while ((xbeeBufferElements > 0) && (xbeeBuffer[(xbeeBufferHead - 1) % XBEE_BUFFER_SIZE] != PACKET_END_TOKEN)) {
      xbeeBufferElements--;
      xbeeBufferHead = (xbeeBufferHead - 1) % XBEE_BUFFER_SIZE;
    }
  }
  if (!wasHeld) releaseXBeeBuffer();
}


/* Returns the next available XBee data packet from the XBee buffer,
   or an empty string if no packet is available. */
String getXBeeBufferPacket() {
  const String EMPTY_STRING = "";

  // Ensure buffer is not modified while in this routine.
  // At return, we will return held state back to original state.
  bool wasHeld = holdXBeeBuffer();

  //noInterrupts();
  //const bool hold = xbeeBufferHold;
  //if (!hold) xBeeBufferHold = true;
  //interrupts();

  // Empty buffer
  if (xbeeBufferElements == 0) {
    if (!wasHeld) releaseXBeeBuffer();
    return EMPTY_STRING;
  }

  // Loop over buffer until we find a valid packet
  // or we reach the end.
  while (1) {
    
    // Remove everything prior to first start token
    while ((xbeeBufferElements > 0) && (xbeeBuffer[(xbeeBufferHead - xbeeBufferElements) % XBEE_BUFFER_SIZE] != PACKET_START_TOKEN)) {
      xbeeBufferElements--;
    }
    
    // (Nearly) empty buffer: no valid packets
    if (xbeeBufferElements < 2) break;
    
    // Look for end token
    size_t endLoc = (xbeeBufferHead - xbeeBufferElements + 1) % XBEE_BUFFER_SIZE;
    while ((endLoc != xbeeBufferHead) && (xbeeBuffer[endLoc] != PACKET_END_TOKEN)) {
      endLoc = (endLoc + 1) % XBEE_BUFFER_SIZE;
    }
    if (endLoc == xbeeBufferHead) break;
    
    // Search backwards from end for start token, in case there are multiple
    size_t startLoc = (endLoc - 1) % XBEE_BUFFER_SIZE;
    while (xbeeBuffer[startLoc] != PACKET_START_TOKEN) {
      startLoc = (startLoc - 1) % XBEE_BUFFER_SIZE;
    }
    if (startLoc != (xbeeBufferHead - xbeeBufferElements) % XBEE_BUFFER_SIZE) {
      xbeeBufferElements = (xbeeBufferHead - startLoc) % XBEE_BUFFER_SIZE;
      Serial.println(F("Warning: Invalid XBee data dropped (possible buffer overrun)."));
    }

    // At this point, startLoc should point to start token, endLoc points
    // to end token, and everything in between should be a two-character
    // hexadecimal packet length (mod 256) followed by the packet.
    
    // Ignore empty packets
    if ((endLoc - startLoc) % XBEE_BUFFER_SIZE <= 2) {
      xbeeBufferElements = (xbeeBufferHead - endLoc + 1) % XBEE_BUFFER_SIZE;
      Serial.println(F("Warning: Dropped empty XBee packet."));
      continue;
    }
    
    // Ignore packets missing length prefix
    if ((endLoc - startLoc) % XBEE_BUFFER_SIZE <= 4) {
      xbeeBufferElements = (xbeeBufferHead - endLoc + 1) % XBEE_BUFFER_SIZE;
      Serial.println(F("Warning: Dropped invalid XBee packet."));
      continue;
    }
    
    // Check packet length.  First two characters should be
    // remaining packet length in hexadecimal (mod 256).
    const size_t packetLen = ((endLoc - startLoc) % XBEE_BUFFER_SIZE) - 3;
    char lbuf[3];
    sprintf(lbuf,"%02X",(uint8_t)(packetLen % 256));
    lbuf[2] = '\0';
    if ((xbeeBuffer[(startLoc + 1) % XBEE_BUFFER_SIZE] != lbuf[0])
        || (xbeeBuffer[(startLoc + 2) % XBEE_BUFFER_SIZE] != lbuf[1])) {
      xbeeBufferElements = (xbeeBufferHead - endLoc + 1) % XBEE_BUFFER_SIZE;
      Serial.println(F("Warning: Dropped invalid XBee packet (length mismatch)."));
      continue;
    }
    
    // We have found a valid packet: extract packet characters, excluding
    // start & end tokens and two-character packet length prefix.
    const size_t packetBufSize = ((endLoc - startLoc) % XBEE_BUFFER_SIZE) - 2;
    char packetBuf[packetBufSize];
    for (size_t pos = 0; pos < packetBufSize - 1; pos++) {
      packetBuf[pos] = xbeeBuffer[(startLoc + 3 + pos) % XBEE_BUFFER_SIZE];
    }
    packetBuf[packetBufSize - 1] = '\0';
    xbeeBufferElements = (xbeeBufferHead - endLoc + 1) % XBEE_BUFFER_SIZE;
    if (!wasHeld) releaseXBeeBuffer();
    return packetBuf;
  }
  
  // If we reach here, we did not find a valid packet
  if (!wasHeld) releaseXBeeBuffer();
  return EMPTY_STRING;
}


/* Retrieves the next available XBee packet and processes it: packet
   data is extracted and then passed on the the remote database.
   If a full packet is not currently available, this function returns
   immediately. */
void processXBee() {
  // Prevent buffer from being altered by ISR
  holdXBeeBuffer();

  // Not necessary if an ISR is used to pull data from the
  // Arduino serial buffer, but it doesn't hurt.
  readXBee();

  // If there was a buffer overrun, remove any incomplete packet
  // at the end.
  if (xbeeBufferOverrun > 0) {
    Serial.print("Warning: XBee buffer overran by ");
    Serial.print(xbeeBufferOverrun);
    Serial.println(" bytes.  Some data lost.");
    Serial.flush();
    cleanXBeeBuffer(true, true);
    xbeeBufferOverrun = 0;
  }

  // Timing note: The web upload routines can take a long time
  // to complete.  We re-enable the XBee buffer read ISR to
  // prevent buffer overrun during this time. The
  // getXBeeBufferPacket() is ISR-safe.
  releaseXBeeBuffer();

  // Cycle over packets until we find a valid one.
  String packet;
  bool uploaded = false;
  while ((packet = getXBeeBufferPacket()).length() > 0) {
    Serial.print(F("XBee packet: "));
    Serial.println(packet);
    Serial.flush();
    switch (packet.charAt(0)) {
      case 'V':
        if (!getModeCoord()) break;
        xbeeReading(packet);
        uploaded = true;
        break;
      case 'R':
        if (!getModeCoord()) break;
        xbeeRate(packet);
        uploaded = true;
        break;
      case 'S':
        if (!getModeCoord()) break;
        set1 = packet;
        if (set1.length() > 0 && set2.length() > 0) {
          xbeeSettings(set1, set2);
          uploaded = true;
          set1 = "";
          set2 = "";
        }
        break;
      case 'T':
        if (!getModeCoord()) break;
        set2 = packet;
        if (set1.length() > 0 && set2.length() > 0) {
          xbeeSettings(set1, set2);
          uploaded = true;
          set1 = "";
          set2 = "";
        }
        break;
      case 'C':
        if (!getModeCoord()) processClockPacket(packet);
        break;
      // Invalid packet: do nothing
      default:
        break;
    }
    // If a packet was uploaded, do not parse another one
    // in this function call to avoid spending an extended
    // time in this routine.
    //if (uploaded) break;
    // Use below to avoid compiler warning if above commented...
    (void)uploaded;
  }
}


void xbeeConfig() {
  String meshMode;
  if (getModeCoord()) {
    meshMode = "1"; //coordinator
  } else {
    meshMode = "0"; //drone
  }

  // Use XBee Grove dev board + XCTU exclusively to configure
  // XBee network, aside from coordinator mode: current firmware
  // not set up to provide access to many of the useful settings
  // and it is too easy to misconfigure the XBee.
  //xbeeUpdateSetting("CE","0"); // Set device to drone before entering new network.
  //xbeeUpdateSetting("ID", getNetID());

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
  while (xbee.available())
    xbee.read();

}

void xbeeGetMac(byte * macL, uint8_t max_mac_len) {
  if (max_mac_len < 6) {
    Serial.println(F("mac array is too short"));
    return;
  }

  //byte macL[6];
  xbeeCommandMode();
  //Serial.println();
  xbee.print("ATSL\r");
  xbee.readBytes(macL, 6);
  //xbee.print("ATCN\r");
  delay(100);
  while (xbee.available())
    //Serial.print((char)xbee.read());
    xbee.read();
  //Serial.println();
  //for(int x = 0; x<sizeof(macL);x++)
  //Serial.println(macL[x]);
  //return macL;
}

void xbeeGetNetwork(byte * netID, uint8_t max_net_len) {
  //Serial.println(sizeof(netID));
  if (max_net_len < 4) {
    Serial.println(F("Network ID array is too short"));
    return;
  }

  xbeeRequestSetting("ID");
  xbee.readBytes(netID, 5);
  while (xbee.available()) {
    Serial.println(F("Leftover Buffer: "));
    Serial.write((char)xbee.read());
  }
  xbeeCloseCommand();
  //Serial.write(netID, sizeof(netID));
}

void xbeeRate(String incoming) {
  String did, sensor, val, datetime;
  int one, two, three, four;

  one = incoming.indexOf(',') + 1;
  two = incoming.indexOf(',', one) + 1;
  three = incoming.indexOf(',', two) + 1;
  four = incoming.indexOf(',', three) + 1;
  did = incoming.substring(one, two - 1);
  sensor = incoming.substring(two, three - 1);
  val = incoming.substring(three, four - 1);
  datetime = incoming.substring(four);
  /*Serial.println(one);
    Serial.println(two);
    Serial.println(three);
    Serial.println(did);
    Serial.println(sensor);
    Serial.println(val);*/
  updateRate(did, sensor, val, datetime);
}

void xbeeSettings(String incoming, String incoming2) {
  String did, location, project;
  String coordinator, rate, build, teardown, datetime, netid;
  int one, two, three, four, five, six, seven, eight, nine;

  one = incoming.indexOf(',') + 1;
  two = incoming.indexOf(',', one) + 1;
  three = incoming.indexOf(',', two) + 1;
  did = incoming.substring(one, two - 1);
  location = incoming.substring(two, three - 1);
  project = incoming.substring(three);

  four = incoming2.indexOf(',') + 1;
  five = incoming2.indexOf(',', four) + 1;
  six = incoming2.indexOf(',', five) + 1;
  seven = incoming2.indexOf(',', six) + 1;
  eight = incoming2.indexOf(',', seven) + 1;
  nine = incoming2.indexOf(',', eight) + 1;
  coordinator = incoming2.substring(four, five - 1);
  rate = incoming2.substring(five, six - 1);
  build = incoming2.substring(six, seven - 1);
  teardown = incoming2.substring(seven, eight - 1);
  datetime = incoming2.substring(eight, nine - 1);
  netid = incoming2.substring(nine);

  updateConfig(did, location, coordinator, project, rate, build, teardown, datetime, netid);
}

void xbeeReading(String incoming) {
  String did, sensor, val, timestamp, datetime;
  int one, two, three, four, five;

  one = incoming.indexOf(',') + 1;
  two = incoming.indexOf(',', one) + 1;
  three = incoming.indexOf(',', two) + 1;
  four = incoming.indexOf(',', three) + 1;
  five = incoming.indexOf(',', four) + 1;
  did = incoming.substring(one, two - 1);
  sensor = incoming.substring(two, three - 1);
  val = incoming.substring(three, four - 1);
  timestamp = incoming.substring(four, five - 1);
  datetime = incoming.substring(five);
  //Serial.println(one);
  //Serial.println(two);
  //Serial.println(three);
  //Serial.println(did);
  //Serial.println(sensor);
  //Serial.println(val);
  //Serial.println(datetime);
  postReading(did, sensor, val, timestamp, datetime);
}


void xbeeRequestSetting(String setting) {
  xbeeCommandMode();
  xbee.print("AT" + setting + "\r");
  delay(200);

}

void xbeeUpdateSetting(String setting, String val) {
  xbeeCommandMode();
  xbee.print("AT" + setting + " " + val + "\r");
  delay(100);
  while (xbee.available()) {
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
  xbee.readBytes(ack, 2);
  if (ack[0] != 'O' && ack[1] != 'K') {
    return false;
  }
  while (xbee.available())
    xbee.read();
  return true;
}

bool xbeeWriteSettings() {
  byte ack[2];
  xbee.print("ATWR\r");
  delay(100);
  xbee.readBytes(ack, 2);
  if (ack[0] != 'O' && ack[1] != 'K') {
    return false;
  }
  return true;
}

void xbeeCloseCommand() {
  xbee.print("ATCN\r");
  delay(100);
  while (xbee.available())
    xbee.read();
}

//--------------------------------------------------------------------------------------------- [Upload Support]

/* Sets the ethernet MAC address using the XBee serial number. */
void initMACAddress() {
  // Use XBee's serial number for ethernet MAC address
  // (as it does not have its own)
  uint32_t SH,SL;
  getXBeeSerialNumber(SH,SL);
  // If failed to get XBee serial number, provide a default.
  // For XBee S3B, SH would be 0x0013A2XX; we use the same here
  // to allow for network whitelisting by MAC range.
  // if (SH == 0) SH = 0xF0E1D2C3;
  if (SH == 0) SH = 0x0013A200;
  if (SL == 0) SL = 0xB4A59687;
  ethMACAddress[0] = (uint8_t)((SH >> 24) & 0xFF);
  ethMACAddress[1] = (uint8_t)((SH >> 16) & 0xFF);
  ethMACAddress[2] = (uint8_t)((SH >>  8) & 0xFF);
  //ethMACAddress[] = (uint8_t)((SH >>  0) & 0xFF);
  //ethMACAddress[] = (uint8_t)((SL >> 24) & 0xFF);
  ethMACAddress[3] = (uint8_t)((SL >> 16) & 0xFF);
  ethMACAddress[4] = (uint8_t)((SL >>  8) & 0xFF);
  ethMACAddress[5] = (uint8_t)((SL >>  0) & 0xFF);
}


String getMACAddressString() {
  String s = "";
  char buf[3];
  for (size_t k = 0; k < MAC_ADDRESS_LEN; k++) {
    if (k != 0) s = s + ":";
    sprintf(buf,"%02X",ethMACAddress[k]);
    s = s + buf;
  }
  return s;
}


void ethernetSetup() {
  // Ethernet connection

  pinMode(ETHERNET_EN, OUTPUT);
  digitalWrite(ETHERNET_EN, HIGH);
  delay(10);

  pinMode(WIZ812MJ_RESET_PIN, OUTPUT);
  digitalWrite(WIZ812MJ_RESET_PIN, LOW);
  delay(2);
  digitalWrite(WIZ812MJ_RESET_PIN, HIGH);

  pinMode(WIZ812MJ_ES_PIN, OUTPUT);
  Ethernet.init(WIZ812MJ_ES_PIN);

  initMACAddress();
  Serial.print(F("Ethernet MAC Address: "));
  Serial.println(getMACAddressString());

  ethernetBegin();

  if (ethStatus.connected()) {
    updateClockFromNTP();
  }
}

bool ethernetBegin() {
  //Serial.println(F("Starting ethernet..."));

  // Reset ethernet chip
  digitalWrite(WIZ812MJ_RESET_PIN, LOW);
  delay(1);
  digitalWrite(WIZ812MJ_RESET_PIN, HIGH);
  delay(1);

  // Power cycle
  //digitalWrite(ETHERNET_EN,LOW);
  //delay(10);
  //digitalWrite(ETHERNET_EN,HIGH);
  //delay(10);

  //Ethernet.init(WIZ812MJ_ES_PIN);

  // Link status (not supported by W5100)
  //Serial.print(F("Ethernet link status: "));
  //Serial.println(Ethernet.linkStatus());

  //unsigned long eth_timeout = 10000;
  //xbeeGetMac(mac, MAC_LEN);
  //Serial.println(F("got mac..."));
  //Serial.write(mac,6);
  //if (!Ethernet.begin(mac, eth_timeout)) {
  if (!Ethernet.begin(ethMACAddress,ETHERNET_START_TIMEOUT)) {
    //online = false;
    ethStatus.restarted(false);
    Serial.println(F("Ethernet initialization failed.  Readings will not be pushed to remote"));
    Serial.println(F("database until connection can be established."));
    return false;
  } else {
    //online = true;
    ethStatus.restarted(true);
    Serial.print(F("Ethernet initialized. IP: "));
    Serial.println(Ethernet.localIP());
    Udp.begin(localPort);
    // Update time only in initialization routine instead of
    // for every network reconnect.  There is a separate process
    // that will make regular NTP updates.
    //getTimeFromWeb();
    //updateClockFromNTP();
    return true;
  }
  
}


//bool ethernetOnline() {
//  return online;
//}


/* Indicates if last network interaction was successful. */
bool ethernetConnected() {
  return ethStatus.connected();
}


void ethernetMaintain() {
  // Restart ethernet if not have had recent successful connection
  if (ethStatus.needsRestart()) {
    Serial.println(F("Extended period without successful internet connection.  Restarting ethernet..."));
    ethernetBegin();
    return;
  }

  // Routine will perform occasional network connection
  // maintenance, like renewing DHCP lease when necessary.
  // Should be called regularly.
  int stat = Ethernet.maintain();
  switch (stat) {
    case 0:
      // No action performed
      break;
    case 1:
      Serial.println(F("Ethernet: DHCP renewal failed."));
      break;
    case 2:
      Serial.println(F("Ethernet: DHCP renewal succeeded."));
      break;
    case 3:
      Serial.println(F("Ethernet: DHCP rebind failed."));
      break;
    case 4:
      Serial.println(F("Ethernet: DHCP rebind succeeded."));
      break;
    default:
      Serial.print(F("Ethernet: Unknown DHCP maintain error ("));
      Serial.print(stat);
      Serial.println(F(")."));
      break;
  }
}

void saveReading(String lstr, String rstr, String atstr, String gtstr, String sstr, String c2str, String p1str, String p2str, String cstr) {
  time_t utc = getUTC();
  String TS(utc);
  //String D = formatDate();
  //String D = getDBDateString(utc);
  //String T = formatTime();
  //String T = getDBTimeString(utc);
  //String DT = D + " " + T;
  String DT = getDBDateTimeString(utc);
  // Use local time in log file, but also include unix timestamp
  //String sensorData = (D + ", " + T + ", " + lstr + ", " + rstr + ", " + atstr + ", " + gtstr + ", " + sstr + ", " + c2str + ", " + p1str + ", " + p2str + ", " + cstr);
  String sensorData = (TS + ", " + DT + ", " + lstr + ", " + rstr + ", " + atstr + ", " + gtstr + ", " + sstr + ", " + c2str + ", " + p1str + ", " + p2str + ", " + cstr);
  logDataSD(sensorData);

  if (lstr != "")
    postReading(getDevID(), "Light", lstr, TS, DT);
  if (rstr != "")
    postReading(getDevID(), "Humidity", rstr, TS, DT);
  if (atstr != "")
    postReading(getDevID(), "AirTemp", atstr, TS, DT);
  if (gtstr != "")
    postReading(getDevID(), "GlobeTemp", gtstr, TS, DT);
  if (sstr != "")
    postReading(getDevID(), "Sound", sstr, TS, DT);
  if (c2str != "")
    postReading(getDevID(), "CO2", c2str, TS, DT);
  if (p1str != "")
    postReading(getDevID(), "PM_2.5", p1str, TS, DT);
  if (p2str != "")
    postReading(getDevID(), "PM_10", p2str, TS, DT);
  if (cstr != "")
    postReading(getDevID(), "CO", cstr, TS, DT);
}

void postReading(String DID, String ST, String R, String TS, String DT)
{
#ifdef DEBUG
  writeDebugLog(ST);
#endif
  if (getModeCoord()) {
    // String in temp string is data to be submitted to MySQL
    char p[200];
    String amp = "&";
    //String Datetime = getStringDatetime();
    String temp = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "Reading=" + R + amp + "TimeStamp=" + TS + amp + "ReadTime=" + DT;
    temp.toCharArray(p, 200);
    if (!postPage(getServer(), serverPort, pageName, p)) {
      //Serial.println(F("Failed to upload sensor reading to remote. \n"));
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Failed to upload sensor reading to remote."));
#ifdef DEBUG
      writeDebugLog(F("Failed to upload sensor reading to remote. \n"));
#endif
    } else {
      //Serial.println(F("Uploaded sensor reading."));
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Uploaded sensor reading (") + ST + F(" @ ") + DID + F(")."));
    }
  } else {
    String message = "V," + DID + "," + ST + "," + R + "," + TS + "," + DT;
    sendXBee(message);
    delay(1000);
  }
}

void updateRate(String DID, String ST, String R, String DT)
{
  if (getModeCoord()) {
    char p[200];
    String amp = "&";
    //String Datetime = getStringDatetime();
    String temp = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "SampleRate=" + R + amp + "RateChange=" + DT;
    temp.toCharArray(p, 200);
    if (!postPage(getServer(), serverPort, pageName, p)) {
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Failed to update sensor rate on remote."));
    } else {
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Uploaded sensor rate."));
    }
  } else {
    String message = "R," + DID + "," + ST + "," + R + "," + DT;
    Serial.println("XBee String: " + message);
    sendXBee(message);
    delay(2000);
  }
}

void updateConfig(String DID, String Location, String Coordinator, String Project, String Rate, String Setup, String Teardown, String Datetime, String NetID)
{
  //Datetime = getStringDatetime();
  Datetime = getDBDateTimeString();
  if (getModeCoord()) {
    char p[200];
    String amp = "&";
    String temp = ("DeviceID=" + DID + amp + "Project=" + Project + amp + "Coordinator=" + Coordinator + amp + "UploadRate=" + Rate + amp + "Location=" + Location + amp + "SetupDate=" + Setup + amp + "TeardownDate=" + Teardown + amp + "ConfigChange=" + Datetime + amp + "NetID=" + NetID);
    temp.toCharArray(p, 200);
    if (!postPage(getServer(), serverPort, pageName, p)) {
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Failed to update device configuration on remote."));
    } else {
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Uploaded device configuration."));
    }
  } else {
    String message = "S," + DID + "," + Location + "," + Project; // Out of order from function call to balance XBee packet size
    String message2 = "T," + Coordinator + "," + Rate + "," + Setup + "," + Teardown + "," + Datetime + "," + NetID; // out of order from function call to balance XBee packet size
    Serial.println("XBee String: " + message + message2 + " " + message.length() + " " + message2.length());
    sendXBee(message);
    delay(100);
    sendXBee(message2);
    delay(2000);
  }
}

// postPage is function that performs POST request and prints results.
byte postPage(const char* domainBuffer, int thisPort, const char* page, const char* thisData)
{
  // Keep track of POST attempts (successful or not)
  packetsUploaded++;
#ifdef DEBUG
  writeDebugLog(F("Fxn: postPage()"));
#endif
  //int inChar;
  char outBuf[200];
  EthernetClient client;

  //Serial.print(F("connecting..."));

  int stat;
  if ((stat = client.connect(domainBuffer, thisPort)) == 1)
  {
    // Debugging
    //Serial.print(F("HTTP server IP: "));
    //Serial.print(client.remoteIP());
    //Serial.print(':');
    //Serial.println(client.remotePort());
    //Serial.print(F("Connected: "));
    //Serial.println(client.connected());
    //Serial.print(F("Available for write: "));
    //Serial.println(client.availableForWrite());
    //Serial.print(F(""));
    
    //Serial.println(F("connected"));
    sprintf(outBuf, "POST %s HTTP/1.1", page);
    client.println(outBuf);
    sprintf(outBuf, "Host: %s", domainBuffer);
    client.println(outBuf);
    client.println(F("Connection: close\r\nContent-Type: application/x-www-form-urlencoded"));
    sprintf(outBuf, "Content-Length: %u\r\n", strlen(thisData));
    client.println(outBuf);

    client.print(thisData);
    client.flush();

    // Wait for server to respond before closing connection.
    // Otherwise, server may have failed to receive the full POST.
    unsigned long t0 = millis();
    while (millis() - t0 < HTTP_POST_TIMEOUT) {
      if (client.available()) break;
      delay(1);
    }
    //Serial.print(F("Available: "));
    //Serial.println(client.available());
    if (!client.available()) {
      Serial.println(F("Warning: Server did not respond before timeout.  Data upload may have failed."));
      // May not want to flag this: probably a server issue, not
      // an ethernet connection issue.
      //ethStatus.failed();
    } else {
      //Serial.print(F("Server response time: "));
      //Serial.println(millis() - t0);
      // Successfully connected to server:
      // clear bad ethernet connection flags
      ethStatus.succeeded();
    }
    client.stop();
    
  } else {
    // Flag bad ethernet connection
    ethStatus.failed();
    
    // Indicate error.  Note most network errors are '0' (uninformative).
    //Serial.println(F("failed"));
    switch (stat) {
      case 1:
        // Success
        break;
      case 0:
        Serial.println(F("Remote server upload failed"));
        break;
      // Below are only for DNS lookup errors?
      case -1:
        Serial.println(F("Remote server upload failed: timed out"));
        break;
      case -2:
        Serial.println(F("Remote server upload failed: invalid server"));
        break;
      case -3:
        Serial.println(F("Remote server upload failed: truncated"));
        break;
      case -4:
        Serial.println(F("Remote server upload failed: invalid response"));
        break;
      default:
        Serial.print(F("Remote server upload failed: unknown error ("));
        Serial.print(stat);
        Serial.println(F(")."));
        break;
    }
    return 0;
  }

  //client.stop();

  return 1;
}

/*
void getTimeFromWeb() {
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
    //const unsigned long sevenHours = 25200UL; //Time zone difference.
    // subtract seventy years:
    //unsigned long epoch = secsSince1900 - seventyYears - sevenHours;
    time_t utc = secsSince1900 - seventyYears;
    Serial.println(utc);
    // print Unix time:
    //if (epoch > SEC_2017) {
    //  setRTCTime(epoch);
    //  setUTC(utc);
    //}
    if (utc > SEC_2017) {
      setUTC(utc);
    }

  } else Serial.println(F("Unable to get server time"));
}
*/

/*
// send an NTP request to the time server at the given address
void sendNTPpacket(const char* address) {
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
*/


/* Attempt to update the RTC with the current time from an NTP server. */
void updateClockFromNTP() {
  Serial.println(F("Retrieving NTP data...."));
  
  // Build NTP request packet
  byte packet[NTP_PACKET_SIZE];
  memset(packet,0,NTP_PACKET_SIZE);
  packet[0]  = 0b11100011;  // LI, Version, Mode
  packet[1]  = 0;           // Stratum, or type of clock
  packet[2]  = 6;           // Polling Interval
  packet[3]  = 0xEC;        // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packet[12] = 49;
  packet[13] = 0x4E;
  packet[14] = 49;
  packet[15] = 52;

  // Send request packet to NTP server.  Do nothing if cannot connect.
  if (!Udp.beginPacket(NTP_SERVER,NTP_PORT)) {
    Serial.println(F("Warning: Failed to connect to NTP server."));
    // Flag bad ethernet connection
    ethStatus.failed();
    return;
  }
  Udp.write(packet,NTP_PACKET_SIZE);
  if (!Udp.endPacket()) {
    Serial.println(F("Warning: Failed to connect to NTP server."));
    // Flag bad ethernet connection
    ethStatus.failed();
    return;
  }
  
  // Wait for reply.
  unsigned long t0 = millis();
  do {
    delay(50);
  } while ((millis() - t0 < 1000) && (Udp.parsePacket() == 0));
  if (Udp.available() != NTP_PACKET_SIZE) {
    Serial.println(F("Warning: Failed to retrieve NTP data."));
    return;
  }
  
  // Successfully connected to NTP server:
  // clear bad ethernet connection flags
  ethStatus.succeeded();
  
  // Get returned packet contents and extract timestamp
  // from bytes 40-43.
  Udp.read(packet,NTP_PACKET_SIZE);
  unsigned long tntp = (((unsigned long)packet[40]) << 24)
                     | (((unsigned long)packet[41]) << 16)
                     | (((unsigned long)packet[42]) <<  8)
                     | (((unsigned long)packet[43]) <<  0);

  // NTP time is seconds since 1900-01-01 00:00:00 UTC.
  // Unix time is seconds since 1970-01-01 00:00:00 UTC.
  const time_t NTP1970 = 2208988800ul;
  time_t utc = tntp - NTP1970;
  Serial.print(F("  Retrieved NTP timestamp:  "));
  Serial.println(tntp);
  //Serial.print(F("  Unix timestamp: "));
  //Serial.println(utc);
  
  // Update RTC only if time is recent (otherwise, ntp
  // data must be invalid).
  //const time_t UTC2000 = 946684800ul;
  const time_t UTC2019 = 1546300800ul;
  if (utc < UTC2019) {
    Serial.println(F("Warning: Invalid NTP data (ignoring)."));
    return;
  }
  setUTC(utc);

  time_t utc0 = getUTC();
  Serial.println(F("RTC updated.  New time:"));
  Serial.print(F("  Universal time: "));
  Serial.println(getUTCDateTimeString(utc0));
  Serial.print(F("  Local time:     "));
  Serial.println(getLocalDateTimeString(utc0));
  Serial.print(F("  Unix timestamp: "));
  Serial.println(utc0);
}


/* Broadcast the current UTC timestamp over the XBee network.
   Intended to be called regularly from coordinator to sync
   all node clocks. */
void broadcastClock() {
  // Only broadcast from coordinator (only node with
  // destination address set to broadcast address).
  if (!getModeCoord()) return;
  
  Serial.println(F("Broadcasting clock timestamp to all nodes...."));
  time_t utc = getUTC();
  
  // Broadcast time only if recent (otherwise, clock time must
  // be invalid).
  //const time_t UTC2000 = 946684800ul;
  const time_t UTC2019 = 1546300800ul;
  if (utc < UTC2019) {
    Serial.println(F("Warning: Invalid RTC time (ignoring)."));
    return;
  }
  
  // Zero-padded timestamp.
  char buff[16];
  sprintf(buff,"C%010ld",utc);
  sendXBee(buff);
}


/* Parses an XBee clock broadcast packet and updates clock if
   packet is valid. */
void processClockPacket(const String packet) {
  if ((packet.length() != 11) || (packet.charAt(0) != 'C')) {
    Serial.println(F("Warning: Received invalid clock broadcast (ignoring)."));
    return;
  }
  
  time_t utc = 0;
  for (int k = 1; k < 11; k++) {
    char c = packet.charAt(k);
    if ((c < '0') || (c > '9')) {
      Serial.println(F("Warning: Received invalid clock broadcast (ignoring)."));
      return;
    }
    utc = 10*utc + (c - '0');
  }
  
  // Update RTC only if time is recent (otherwise, broadcast
  // data must be invalid).
  //const time_t UTC2000 = 946684800ul;
  const time_t UTC2019 = 1546300800ul;
  if (utc < UTC2019) {
    Serial.println(F("Warning: Received invalid clock broadcast (ignoring)."));
    return;
  }
  setUTC(utc);

  time_t utc0 = getUTC();
  Serial.print(F("RTC updated.  New time:"));
  Serial.print(F("  Universal time: "));
  Serial.println(getUTCDateTimeString(utc0));
  Serial.print(F("  Local time:     "));
  Serial.println(getLocalDateTimeString(utc0));
  Serial.print(F("  Unix timestamp: "));
  Serial.println(utc0);
}
