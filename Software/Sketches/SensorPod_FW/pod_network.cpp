
/*
   pod_network.cpp
   2017 - Nick Turner and Morgan Redfield
   2018 - Chris Savage
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
#define XBEE_BUFFER_SIZE 512
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

// MOVED XBEE VALUES TO STRUCTURE BELOW.
// The XBee's serial number.  To be extracted from XBee.
//uint64_t xbeeSerialNumber = 0;
// If not a coordinator, the destination is the serial number of the
// coordinator.  Initially extracted from XBee, but will be updated from
// network broadcast by coordinator.  Cached here instead of rereading
// from XBee due to ~ 2 second command mode period for each read.
//uint64_t xbeeDestination = 0;

// Structure to store various XBee configuration settings.
// Useful to retrieve/set in bulk as each switch into command mode
// takes 2+ seconds.
struct XBeeConfigStruct {
  // NI: node identifier.
  String identifier = "";
  // CE: messaging mode.
  // Should be 0 for router/drone, 1 for coordinator.
  uint8_t coordinator = 0;
  // SH+SL: XBee serial number
  uint64_t serialNumber = 0;
  // DH+DL: messaging destination.
  // If this PODD is not a coordinator, the destination is the
  // serial number of the coordinator.  Initially extracted from
  // XBee, but will be updated from network broadcast by coordinator.
  uint64_t destination = 0;
  // HP: preamble ID (0-7).
  // XBee's with different preambles are essentially on different
  // networks; this can be used to create distinct PODD network
  // groups.  Best to avoid 0 as that is the out-of-box default
  // (most likely to have interference with any non-PODD XBees
  // in the area).
  uint8_t preamble = -1;
  // ID: network ID (0x0000 - 0x7FFF)
  uint32_t network = -1;
} xbeeConfig;

// Network group number (1-7).
// Used to create different PODD networks: each active group
// should have one (and only one) coordinator unit.  PODDs
// with different group numbers do not communicate with each
// other.
// Corresponds to XBee preamble ID.
uint8_t xbeeGroup = 0;

String set1;
String set2;

// Ethernet connection settings
#define WIZ812MJ_ES_PIN 20 // WIZnet SPI chip-select pin
#define WIZ812MJ_RESET_PIN 9 // WIZnet reset pin
#define MAC_ADDRESS_LEN 6
byte ethMACAddress[MAC_ADDRESS_LEN];
const char SERVER_PAGE_NAME[] = "/LMNSensePod.php"; // Name of submission page. Log into EC2 Server and navigate to /var/www/html to view
#define SERVER_PORT 80
#define LOCAL_PORT 8888

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
// Note network reinitialization prevents device from performing
// other tasks until it succeeds or times out.
//#define NETWORK_RECONNECT_INTERVAL (5*60000UL)
#define NETWORK_RECONNECT_INTERVAL (60000UL)
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
    // If we do not have an IP address, network needs reinitialization
    // (do not wait for multiple network interaction failures).
    if (!ethernetHasIPAddress()) return true;
    // If network interactions have consistently failed, try
    // reinitializing network.  This may be a server issue rather
    // than a network issue, so trying not to be too aggressive with
    // restarts here.
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


//------------------------------------------------------------------------------

/* Places the XBee in command mode.  Takes ~ 2 seconds.  XBee will remain 
   in command mode for ~ 1 second or until explicitly ended. */
bool startXBeeCommandMode() {
  // Clear incoming buffer
  while (xbee.available()) xbee.read();
  // Put XBee in command mode
  delay(1100);
  xbee.print(F("+++"));
  delay(1100);
  // Get response: should receive "OK\r" if successful
  const int BUF_LEN = 4;
  char buf[BUF_LEN];
  size_t pos = 0;
  while (xbee.available()) {
    buf[pos++] = xbee.read();
    if (buf[pos-1] == '\r') break;
    if (pos >= BUF_LEN - 1) break;
  }
  buf[pos] = '\0';
  // Clear any remaining buffer
  while (xbee.available()) xbee.read();
  String s(buf);
  // Check response
  //Serial.print(F("XBee response: "));
  //Serial.println(s);
  return s.equals(F("OK\r"));
}


/* Ends the XBee command mode.  Argument indicates if any modified
   settings should be saved; otherwise, changes will only be applied
   until the XBee is reset. */
void stopXBeeCommandMode(const bool write) {
  // Save new configuration
  if (write) submitXBeeCommand(F("ATWR"));
  // Exit command mode (applies changes)
  submitXBeeCommand(F("ATCN"));
}


/* Sends the given command to the XBee, indicating if the command
   was successfully sent (but not necessarily successfully processed...).
   XBee must already be in command mode.  Do not include trailing '\r'. */
bool submitXBeeCommand(const String cmd) {
  // XBee responds with "OK\r" or "ERROR\n" after each sent command,
  // except when data is returned.
  // Clear incoming buffer
  while (xbee.available()) xbee.read();
  // Submit command
  xbee.print(cmd + "\r");
  // Wait for response, which should end in "\r"
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    while (xbee.available()) {
      if (xbee.read() == '\r') return true;
    }
    delay(10);
  }
  while (xbee.available()) {
    if (xbee.read() == '\r') return true;
  }
  return false;
}


/* Sends the given command to the XBee and returns the response.
   Returns empty string if communication was unsuccessful.
   XBee must already be in command mode.  Do not include trailing '\r'. */
String getXBeeCommandResponse(const String cmd) {
  // XBee responds with "OK\r" or "ERROR\n" after each sent command,
  // except when data is returned.
  // Clear incoming buffer
  while (xbee.available()) xbee.read();
  // Submit command
  xbee.print(cmd + "\r");
  const int BUF_LEN = 16;
  size_t pos = 0;
  char buf[BUF_LEN];
  // Wait for response until '\r'
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
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
   hexadecimal.  XBee must already be in command mode.
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


/* Gets the XBee serial number as 64-bit number.  Returns 0 if 
   could not obtain the serial number.  XBee must already be in
   command mode. */
uint64_t getXBeeSerialNumber() {
  // Get serial number as upper and lower 32-bits
  uint32_t SH = getXBeeNumericResponse(F("ATSH"));  // For XBee S3B, this is 0x0013A2XX
  uint32_t SL = getXBeeNumericResponse(F("ATSL"));
  // Construct 64-bit number
  return ((uint64_t)SH << 32) | (uint64_t)SL;
}


/* Gets the XBee destination as 64-bit number.  Returns 0 if 
   could not obtain the destination.  XBee must already be in
   command mode. */
uint64_t getXBeeDestination() {
  // Get destination as upper and lower 32-bits
  uint32_t DH = getXBeeNumericResponse(F("ATDH"));
  uint32_t DL = getXBeeNumericResponse(F("ATDL"));
  // Construct 64-bit number
  return ((uint64_t)DH << 32) | (uint64_t)DL;
}


/* Sets the XBee destination as 64-bit number.  XBee must already 
   be in command mode. */
void setXBeeDestination(const uint64_t dest) {
  // Set destination as upper and lower 32-bits
  uint32_t DH = (dest >> 32) & 0xFFFFFFFF;
  uint32_t DL = (dest >>  0) & 0xFFFFFFFF;
  submitXBeeCommand(F("ATDH ") + String(DH,HEX));
  submitXBeeCommand(F("ATDL ") + String(DL,HEX));
}


/* Create a zero-padded hex string for the given 64-bit integer. */
String uint64ToHexString(uint64_t v) {
  uint32_t vh = (v >> 32) & 0xFFFFFFFF;
  uint32_t vl = (v >>  0) & 0xFFFFFFFF;
  char buff[20];
  sprintf(buff,"0x%08lX%08lX",vh,vl);
  return String(buff);
  
  //String s = "0x";
  //char buf[3];
  //const int VBYTES = 8;
  //for (size_t k = 0; k < VBYTES; k++) {
  //  sprintf(buf,"%02X",(uint8_t)((v >> (8*(VBYTES-k-1))) & 0xFF));
  //  s = s + buf;
  //}
  //return s;
}

String getXBeeSerialNumberString() {return uint64ToHexString(xbeeConfig.serialNumber);}
String getXBeeDestinationString() {return uint64ToHexString(xbeeConfig.destination);}

uint8_t getXBeeGroup() {
  return xbeeGroup;
}

void setXBeeGroup(uint8_t group) {
  if ((group >= 1) && (group <= 7)) xbeeGroup = group;
}



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

  // Get various XBee configuration settings.
  // Not all these are currently being used in the firmware, but
  // these settings might be useful at some point.
  startXBeeCommandMode();
  //xbeeSerialNumber = getXBeeSerialNumber();
  //xbeeDestination = getXBeeDestination();
  xbeeConfig.identifier = getXBeeCommandResponse(F("ATNI"));
  xbeeConfig.coordinator = (uint8_t)getXBeeNumericResponse(F("ATCE"));
  xbeeConfig.serialNumber = getXBeeSerialNumber();
  xbeeConfig.destination = getXBeeDestination();
  xbeeConfig.preamble = (uint8_t)getXBeeNumericResponse(F("ATHP"));
  xbeeConfig.network = getXBeeNumericResponse(F("ATID"));
  stopXBeeCommandMode(false);

  // PODD group number is equivalent to preamble ID
  xbeeGroup = (xbeeConfig.preamble > 0) ? xbeeConfig.preamble : 0;
  
  //Serial.println(F("  XBee serial number: ") + getXBeeSerialNumberString());
  //Serial.println(F("  XBee destination:   ") + getXBeeDestinationString());
}


/* Sets the XBee as coordinator (true) or drone (false),
   sets the destination address, and performs any other
   XBee configuration.  Argument indicates if this PODD
   is the coordinator. */
void configureXBee(const bool coord) {
  // Takes ~ 2 seconds to enter command mode.
  startXBeeCommandMode();

  // Update XBee identifier to device ID (if necessary)
  if (!xbeeConfig.identifier.equals(getDevID())) {
    xbeeConfig.identifier = getDevID();
    submitXBeeCommand(F("ATNI ") + xbeeConfig.identifier);
  }

  // Update XBee preamble ID to PODD XBee group number (if necessary)
  if ((xbeeConfig.preamble != xbeeGroup) && (xbeeGroup >= 1) && (xbeeGroup <= 7)) {
    xbeeConfig.preamble = xbeeGroup;
    submitXBeeCommand(F("ATHP ") + String(xbeeConfig.preamble,HEX));
  }
  
  // Settings for coordinator
  if (coord) {
    // Set as coordinator
    submitXBeeCommand(F("ATCE 1"));
    // Set destination to broadcast address (0x000000000000FFFF).
    // Note command string omits '0x'.
    //submitXBeeCommand(F("ATDH 00000000"));
    //submitXBeeCommand(F("ATDL 0000FFFF"));
    xbeeConfig.destination = 0xFFFF;
    setXBeeDestination(xbeeConfig.destination);
    
  // Settings for drone
  } else {
    // Set as non-coordinator
    submitXBeeCommand(F("ATCE 0"));
    // Set destination to coordinator address (0x0000000000000000).
    // Note command string omits '0x'.
    //submitXBeeCommand(F("ATDH 00000000"));
    //submitXBeeCommand(F("ATDL 00000000"));
    //xbeeDestination = 0x0000;
    //setXBeeDestination(xbeeDestination);
    // NOTE: Unlike conventional XBee network topologies, Digimesh
    // does not have a coordinator node and the "coordinator" will
    // not receive packets sent to the generic coordinator address.
    // Instead, the destination should be set to the serial number
    // of the designated "coordinator" PODD.  If the XBee destination
    // was set to an explicit XBee serial number, use that for now,
    // otherwise broadcast data until we get the coordinator address
    // via a broadcast.
    // XBee 900HP serial numbers are of form 0x0013A2XXXXXXXXXX.
    if (((xbeeConfig.destination >> 40) & 0xFFFFFF) == 0x0013A2) {
      // do nothing (keep current destination)
    } else {
      xbeeConfig.destination = 0xFFFF;
      setXBeeDestination(xbeeConfig.destination);
    }
  }
  
  // Read settings
  //Serial.println(F("ATCE: ") + getXBeeCommandResponse(F("ATCE")));
  //Serial.println(F("ATDH: ") + getXBeeCommandResponse(F("ATDH")));
  //Serial.println(F("ATDL: ") + getXBeeCommandResponse(F("ATDL")));
  
  //Serial.println(F("  XBee serial number: ") + getXBeeSerialNumberString());
  //Serial.println(F("  XBee destination:   ") + getXBeeDestinationString());
  
  // Save new configuration and exit command mode (applies changes)
  stopXBeeCommandMode(true);
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
  size_t nuploaded = 0;
  while ((packet = getXBeeBufferPacket()).length() > 0) {
    Serial.print(F("XBee packet: "));
    Serial.println(packet);
    Serial.flush();
    switch (packet.charAt(0)) {
      case 'V':
        if (!getModeCoord()) break;
        xbeeReading(packet);
        nuploaded++;
        break;
      case 'R':
        if (!getModeCoord()) break;
        xbeeRate(packet);
        nuploaded++;
        break;
      case 'S':
        if (!getModeCoord()) break;
        set1 = packet;
        if (set1.length() > 0 && set2.length() > 0) {
          xbeeSettings(set1, set2);
          nuploaded++;
          set1 = "";
          set2 = "";
        }
        break;
      case 'T':
        if (!getModeCoord()) break;
        set2 = packet;
        if (set1.length() > 0 && set2.length() > 0) {
          xbeeSettings(set1, set2);
          nuploaded++;
          set1 = "";
          set2 = "";
        }
        break;
      case 'C':
        if (!getModeCoord()) processClockPacket(packet);
        break;
      case 'D':
        if (!getModeCoord()) processDestinationPacket(packet);
        break;
      // Invalid packet: do nothing
      default:
        break;
    }
    // If a packet was uploaded, do not parse another one in this
    // function call to avoid spending an extended time in this
    // routine.  Note if XBee packets come in faster than they can
    // be uploaded to the server, we can get stuck in this loop
    // indefinitely without forcing a break.  If delays are caused
    // by a bad network connection, the network maintenance/restart
    // routines would then never get called...
    if (nuploaded >= 1) break;
    // Use below to avoid compiler warning if above commented...
    //(void)nuploaded;
  }
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


/* Broadcast the coordinator's address over the XBee network,
   to be used as the destination address by other XBees.
   Intended to be called regularly from coordinator. */
void broadcastCoordinatorAddress() {
  // Only broadcast from coordinator.
  if (!getModeCoord()) return;
  
  Serial.println(F("Broadcasting coordinator address to all nodes...."));
  uint32_t SH = (xbeeConfig.serialNumber >> 32) & 0xFFFFFFFF;
  uint32_t SL = (xbeeConfig.serialNumber >>  0) & 0xFFFFFFFF;
  
  // Zero-padded hex string.
  char buff[18];
  sprintf(buff,"D%08lX%08lX",SH,SL);
  sendXBee(buff);
}


/* Parses an XBee destination broadcast packet and updates destination 
   if packet is valid and provides a new address. */
void processDestinationPacket(const String packet) {
  if ((packet.length() != 17) || (packet.charAt(0) != 'D')) {
    Serial.println(F("Warning: Received invalid coordinator address broadcast (ignoring)."));
    return;
  }

  uint64_t v = 0;
  for (int k = 0; k < 16; k++) {
    char c = packet.charAt(k+1);
    if ((c >= '0') && (c <= '9')) {
      v = (v << 4) + (uint8_t)(c - '0');
    } else if ((c >= 'A') && (c <= 'F')) {
      v = (v << 4) + (uint8_t)(c - 'A' + 10);
    } else if ((c >= 'a') && (c <= 'f')) {
      v = (v << 4) + (uint8_t)(c - 'a' + 10);
    } else {
      Serial.println(F("Warning: Received invalid coordinator address broadcast (ignoring)."));
      return;
    }
  }
  
  // Update destination address only if it has changed
  if (v != xbeeConfig.destination) {
    xbeeConfig.destination = v;
    startXBeeCommandMode();
    setXBeeDestination(xbeeConfig.destination);
    stopXBeeCommandMode(true);
    Serial.print(F("Coordinator (destination) address updated: "));
    uint32_t DH = (xbeeConfig.destination >> 32) & 0xFFFFFFFF;
    uint32_t DL = (xbeeConfig.destination >>  0) & 0xFFFFFFFF;
    char buff[20];
    sprintf(buff,"0x%08lX%08lX",DH,DL);
    Serial.println(buff);
  }
}


//--------------------------------------------------------------------------------------------- [Upload Support]

/* Sets the ethernet MAC address using the XBee serial number. */
void initMACAddress() {
  // Use XBee's serial number for ethernet MAC address
  // (as it does not have its own). Serial number should
  // be cached already.
  uint32_t SH = (xbeeConfig.serialNumber >> 32) & 0xFFFFFFFF;
  uint32_t SL = (xbeeConfig.serialNumber >>  0) & 0xFFFFFFFF;
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
  // It is useful to increase the HTTP post timeout for debugging,
  // but long timeouts should not be used in production as the
  // coordinator can easily get overwhelmed if there are issues
  // connecting to the server.
  #if HTTP_POST_TIMEOUT >= 1000
  Serial.println(F("******** DEBUG: Long HTTP post timeout in use ********"));
  #endif
  
  // Provide power to the ethernet board
  pinMode(ETHERNET_EN, OUTPUT);
  digitalWrite(ETHERNET_EN, HIGH);
  delay(10);

  // Ethernet chip reset pin
  // WIZnet W5100 documentation says this needs to be pulled low for
  // as short as 2us to reinitialize all internal registers to their
  // default states.  However, it takes 10ms to reset to internal
  // PLOCK (whatever that is...).
  pinMode(WIZ812MJ_RESET_PIN, OUTPUT);
  digitalWrite(WIZ812MJ_RESET_PIN, LOW);
  delay(1);
  digitalWrite(WIZ812MJ_RESET_PIN, HIGH);
  delay(10);
  
  // SPI bus chip select pin
  pinMode(WIZ812MJ_ES_PIN, OUTPUT);
  Ethernet.init(WIZ812MJ_ES_PIN);
  
  // WIZnet 5100 has no hardware MAC address: we must provide one.
  // We use the XBee serial number to construct a (hopefully unique)
  // MAC address.
  initMACAddress();
  Serial.print(F("Ethernet MAC Address: "));
  Serial.println(getMACAddressString());

  // Connect to the network.
  // For whatever reason, this seems to fail 20-30% of the time,
  // at least when testing on LMN network.
  ethernetBegin(1);
  
  // Sometimes a second attempt will solve DHCP issues.
  if (!ethStatus.connected()) {
    Serial.println(F("Re-attempting to connect to the network...."));
    delay(1000);
    int stat = Ethernet.maintain();
    switch (stat) {
      case 0:
        // No action performed
        // If ethernet was not successfully connected before,
        // this should not occur?
        break;
      case 1:
        Serial.println(F("Ethernet: DHCP renewal failed."));
        break;
      case 2:
        Serial.println(F("Ethernet: DHCP renewal succeeded."));
        ethStatus.restarted(true);
        break;
      case 3:
        Serial.println(F("Ethernet: DHCP rebind failed."));
        break;
      case 4:
        Serial.println(F("Ethernet: DHCP rebind succeeded."));
        ethStatus.restarted(true);
        break;
      default:
        Serial.print(F("Ethernet: Unknown DHCP error ("));
        Serial.print(stat);
        Serial.println(F(")."));
        break;
    }
  }

  if (ethStatus.connected()) {
    updateClockFromNTP();
  }
}

bool ethernetBegin(int attempts) {
  //Serial.println(F("Starting ethernet...."));

  if (attempts < 1) attempts = 1;
  
  // Power cycle ethernet
  //digitalWrite(ETHERNET_EN,LOW);
  //delay(10);
  //digitalWrite(ETHERNET_EN,HIGH);
  //delay(10);
  
  // Try multiple times to start ethernet
  for (int k = 1; k <= attempts; k++) {
    // Reset ethernet chip
    // WIZnet W5100 documentation says this needs to be pulled low for
    // as short as 2us to reinitialize all internal registers to their
    // default states.  However, it takes 10ms to reset to internal
    // PLOCK (whatever that is...).
    digitalWrite(WIZ812MJ_RESET_PIN, LOW);
    delay(1);
    digitalWrite(WIZ812MJ_RESET_PIN, HIGH);
    delay(10);
    
    //Ethernet.init(WIZ812MJ_ES_PIN);
    
    // Link status (not supported by W5100)
    //Serial.print(F("Ethernet link status: "));
    //Serial.println(Ethernet.linkStatus());
    
    // For whatever reason, this seems to fail 20-30% of the time,
    // at least when testing on LMN network.  Firmware should be
    // capable of calling this routine again if begin() fails, perhaps
    // after some interval of time to avoid getting stuck in a
    // reinitialization loop (if the network is actually inaccessible).
    if (!Ethernet.begin(ethMACAddress,ETHERNET_START_TIMEOUT)) {
      ethStatus.restarted(false);
      if (k < attempts) {
        Serial.println(F("Ethernet initialization failed.  Retrying..."));
      } else {
        Serial.println(F("Ethernet initialization failed."));
      }
    } else {
      ethStatus.restarted(true);
      Serial.print(F("Ethernet initialized. IP: "));
      Serial.println(Ethernet.localIP());
      return true;
    }
  }
  return false;
}


/* Indicates if the ethernet currently has a valid IP address.
   A false indicates the ethernet is not connected to the network.
   However, true does not necessarily mean the ethernet is
   connected to the network (only that it was connected at some
   point previously). */
bool ethernetHasIPAddress() {
  IPAddress ip = Ethernet.localIP();
  if (ip == IPAddress(0ul) || ip == IPAddress(0xFFFFFFFFul)) return false;
  return true;
}


/* Indicates if last network interaction was successful. */
bool ethernetConnected() {
  return ethStatus.connected();
}


void ethernetMaintain() {
  // Restart ethernet if have not had recent successful connection
  if (ethStatus.needsRestart()) {
    Serial.println(F("Extended period without successful internet connection.  Restarting ethernet...."));
    ethernetBegin(1);
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
  String DT = getDBDateTimeString(utc);
  // Use local time in log file, but also include unix timestamp
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
    String amp = "&";
    String content = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "Reading=" + R + amp + "TimeStamp=" + TS + amp + "ReadTime=" + DT;
    //char p[200];
    //content.toCharArray(p, 200);
    //if (!postPage(getServer(), SERVER_PORT, SERVER_PAGE_NAME, p)) {
    if (!postPage(getServer(), SERVER_PORT, SERVER_PAGE_NAME, content.c_str())) {
      Serial.print("[" + String(packetsUploaded) + "] ");
      Serial.println(F("Failed to upload sensor reading to remote."));
      #ifdef DEBUG
      writeDebugLog(F("Failed to upload sensor reading to remote. \n"));
      #endif
    } else {
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
    String amp = "&";
    String content = "DeviceID=" + DID + amp + "SensorType=" + ST + amp + "SampleRate=" + R + amp + "RateChange=" + DT;
    if (!postPage(getServer(), SERVER_PORT, SERVER_PAGE_NAME, content.c_str())) {
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
  Datetime = getDBDateTimeString();
  if (getModeCoord()) {
    String amp = "&";
    String content = ("DeviceID=" + DID + amp + "Project=" + Project + amp + "Coordinator=" + Coordinator + amp + "UploadRate=" + Rate + amp + "Location=" + Location + amp + "SetupDate=" + Setup + amp + "TeardownDate=" + Teardown + amp + "ConfigChange=" + Datetime + amp + "NetID=" + NetID);
    if (!postPage(getServer(), SERVER_PORT, SERVER_PAGE_NAME, content.c_str())) {
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

  // If we do not have IP address, we will be unable to upload data
  // to server, so we do not even attempt a network connection.
  // The ethernetMaintain() routine should eventually try to reconnect.
  if (!ethernetHasIPAddress()) {
    // Flag bad ethernet connection
    ethStatus.failed();
    Serial.println(F("Remote server upload failed: no internet connection"));
    return 0;
  }
  
  //int inChar;
  char outBuf[200];
  EthernetClient client;

  //Serial.print(F("connecting...."));

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
    //Serial.print(F("Network socket: "));
    //Serial.println(client.getSocketNumber());
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
      if (getDebugMode()) {
        Serial.print(F("Server response time: "));
        Serial.println(millis() - t0);
      }
      // Successfully connected to server:
      // clear bad ethernet connection flags
      ethStatus.succeeded();
    }
    client.stop();
    
  } else {
    // Flag bad ethernet connection
    ethStatus.failed();
    
    // Ensure connection is closed.
    // It appears that the connect routine may occasionally fails
    // due to the network socket being (prematurely) closed.
    // However, in some cases, that socket becomes(?) open and/or
    // it's status is not cleared, leaving it in a state where
    // it is no longer usable.  With only four sockets on the
    // WIZnet W5100, those sockets can quickly become exhausted,
    // in which case the network can no longer be used until the
    // ethernet is reset.  The delay and stop() here seems to
    // catch these socket leak cases (unclear if the delay is
    // necessary; the assumption is that the external ethernet
    // board may still be processing a socket opening event).
    delay(5);
    if (client.getSocketNumber() != MAX_SOCK_NUM) {
      Serial.print(F("Warning: Ethernet socket was not released ("));
      Serial.print(client.getSocketNumber());
      Serial.println(F(")."));
    }
    // Does nothing if the socket has been closed?
    client.stop();
    
    // Indicate error.  Note most network errors are '0' (uninformative).
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


//--------------------------------------------------------------------------------------------- [Upload Support]

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
  
  // Open port to receive UDP response packet.
  EthernetUDP Udp;
  if (!Udp.begin(LOCAL_PORT)) {
    // Flag bad ethernet connection
    ethStatus.failed();
    Serial.println(F("Warning: Failed to open port to receive NTP response."));
    Udp.stop();
    return;
  }
  
  // If we do not have IP address, we will be unable to connect to
  // NTP server.  The ethernetMaintain() routine should eventually
  // try to reconnect.
  if (!ethernetHasIPAddress()) {
    // Flag bad ethernet connection
    ethStatus.failed();
    Serial.println(F("Warning: Failed to connect to NTP server (no internet connection)."));
    Udp.stop();
    return;
  }
  
  // Send request packet to NTP server.  Do nothing if cannot connect.
  if (!Udp.beginPacket(NTP_SERVER,NTP_PORT)) {
    // Flag bad ethernet connection
    ethStatus.failed();
    Serial.println(F("Warning: Failed to connect to NTP server."));
    Udp.stop();
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
    Udp.stop();
    return;
  }
  
  // Successfully connected to NTP server:
  // clear bad ethernet connection flags
  ethStatus.succeeded();
  
  // Get returned packet contents and release incoming port.
  Udp.read(packet,NTP_PACKET_SIZE);
  Udp.stop();
  
  // Extract timestamp from packet bytes 40-43.
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
  Serial.println(F("RTC updated.  New time:"));
  Serial.print(F("  Universal time: "));
  Serial.println(getUTCDateTimeString(utc0));
  Serial.print(F("  Local time:     "));
  Serial.println(getLocalDateTimeString(utc0));
  Serial.print(F("  Unix timestamp: "));
  Serial.println(utc0);
}
