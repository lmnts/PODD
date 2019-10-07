/*==============================================================================
  Constants and functions related to the serial interface menu.

  This file is part of the LMN PODD distribution:
    https://github.com/lmnts/PODD

  CONTRIBUTORS:
    Chris Savage (2019)

  COPYRIGHT/LICENSE:
  Copyright (c) 2019 LMN Architects

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Affero General Public License for more details.

  You should have received a copy of the GNU Affero General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

==============================================================================*/

#include "pod_menu.h"
#include "pod_util.h"
#include "pod_serial.h"
#include "pod_clock.h"
#include "pod_config.h"
#include "pod_network.h"
#include "pod_logging.h"
#include "pod_sensors.h"

#include <Ethernet.h>


// Constants/global variables ==================================================

// Various string constants.
// PROGMEM used to store in flash instead of dynamic memory, but must
// cast to (FType) when using.  Note F() cannot be used here, outside
// of a function.
static const char MENU_HEADERLINE[] PROGMEM = "========================================================================";
static const char MENU_INDENT2[] PROGMEM = "        ";
static const char CONTINUE_PROMPT[] PROGMEM = "(hit any key to continue)";

// No casting required, but uses dynamic memory.
//static const char MENU_INDENT2[] = "        ";
//static const char CONTINUE_PROMPT[] = "(hit any key to continue)";



// Functions ===================================================================

//------------------------------------------------------------------------------
/* For up to the given amount of time [ms], will provide (over the 
   serial interface) regular prompts to press a key, in which case,
   a menu-based interactive mode will be started.  Otherwise, the
   PODD will start running in automatic mode. */
void interactivePrompt(unsigned long timeout) {
  Serial.println();

  // Clear serial input
  clearSerial();
  
  // Check for interaction
  bool interactive = false;
  for (int k = (timeout/1000); k > 0; k--) {
    Serial.print(F("PODD will start in ")); 
    Serial.print(k);
    Serial.print(F(" seconds.  Press any key to enter interactive mode."));
    Serial.println();
    if (getSerialChar(1000) != (char)(-1)) {
      interactive = true;
      break;
    }
  }
  
  if (interactive) mainMenu();
  
  Serial.println(F("Now beginning automated mode."));
}


//------------------------------------------------------------------------------
/* Main menu for interactive mode.  Will continue to be shown until
   an appropriate selection is made. */
void mainMenu() {
  // Will continue offering menu until appropriate response received.
  while (true) {
    // Clear terminal by printing a bunch of blank lines
    for (int k = 0; k < 100; k++) Serial.println();
    // Print menu options
    Serial.println();
    //Serial.println(F("======== PODD MENU ====================================================="));
    //Serial.println(F("<<<<<<<< PODD MENU >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
    Serial.println(F("========================================================================"));
    Serial.println(F("|        PODD MENU                                                     |"));
    Serial.println(F("========================================================================"));
    //Serial.println((FType)MENU_LINE);
    Serial.println();
    // Project-related settings
    Serial.println(F("  (1) Project settings"));
    showMenuProjectSettings();
    // Node-related settings
    Serial.println(F("  (2) Node settings"));
    showMenuNodeSettings();
    // Sensor-related settings
    //Serial.println(F("  (-) Sensor settings"));
    Serial.println(F("  (3) Sensor timing"));
    showMenuSensorTimingSettings();
    Serial.println(F("  (4) Sensor configuration/testing"));
    // Ethernet settings
    Serial.println(F("  (N) Network settings"));
    showMenuNetworkSettings();
    // XBee settings
    Serial.println(F("  (X) XBee settings"));
    showMenuXBeeSettings();
    // RTC time/settings
    Serial.println(F("  (C) Clock settings"));
    showMenuClockSettings();
    // Show compilation info
    Serial.println(F("  (I) Compilation info"));
    // Enable or disable debug mode
    if (getDebugMode()) {
      Serial.println(F("  (D) Disable debug mode"));
    } else {
      Serial.println(F("  (D) Enable debug mode"));
    }
    //Serial.println(F("  "));
    Serial.println();
    // Leave menu and enter automatic mode
    Serial.println(F("  (Q) Quit interactive mode"));
    Serial.println();
    //Serial.println(F("                        ------------------------                        "));
    Serial.println(F("        ----------------                                                "));
    //Serial.println(F("========================================================================"));
    //Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));

    // Get selection
    clearSerial();
    Serial.println();
    Serial.print("Enter the item to view/configure: ");
    char c = getSerialChar(-1);
    Serial.println(c);  // Serial Monitor does not echo inputs
    Serial.println();

    // Process selection
    // We commit any configuration changes when leaving
    // the menu or going to an option that we might not
    // return from (e.g. sensor testing).
    bool showContinuePrompt = true;
    switch (c) {
      case '1':
        configureProjectSettings();
        break;
      case '2':
        configureNodeSettings();
        break;
      case '3':
        configureSensorTimingSettings();
        break;
      case '4':
        sensorMenu();
        showContinuePrompt = false;
        break;
      //case '5':
      //  Serial.println(F("<option not yet implemented>"));
      //  break;
      case 'N':
      case 'n':
        configureNetworkSettings();
        break;
      case 'X':
      case 'x':
        configureXBeeSettings();
        break;
      case 'C':
      case 'c':
        configureClockSettings();
        break;
      case 'I':
      case 'i':
        Serial.println(F("  Firmware version:    " CONFIG_VERSION));
        printCompilationInfo("  ","");
        Serial.println();
        break;
      case 'D':
      case 'd':
        configureDebugSettings();
        break;
      case 'Q':
      case 'q':
        // Moved to later in sketch's startup to allow for
        // for more opportunities to connect to the network first.
        //if (podConfigChanged()) savePodConfig();
        //if (podRateChanged()) savePodRates();
        showContinuePrompt = false;
        return;
        break;
      default:
        Serial.println(F("Invalid option.  Select an option from the list."));
        break;
    }

    if (showContinuePrompt) {
      //Serial.println();
      Serial.println((FType)CONTINUE_PROMPT);
      getSerialChar(-1);
    }
    
    Serial.println();
  }
}


//----------------------------------------------
/* Prints to serial various project settings.
   Intended to be used just below menu's project settings entry. */
void showMenuProjectSettings() {
  Serial.print((FType)MENU_INDENT2);
  Serial.print("Project: " + String(getProject()));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print("Server:  " + String(getServer()));
  Serial.println();
}


//----------------------------------------------
/* Prints to serial various node settings.
   Intended to be used just below menu's node settings entry. */
void showMenuNodeSettings() {
  Serial.print((FType)MENU_INDENT2);
  Serial.print("Device ID:   " + String(getDevID()));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print("Location:    " + String(getRoom()));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print("Coordinator: " + String(getModeCoord() ? "Yes" : "No"));
  Serial.println();
}


//----------------------------------------------
/* Prints to serial a single sensor timing setting.
   Intended to be used just below menu's sensor timing settings entry. */
void showMenuSensorTimingEntry(String s, int v) {
  //return String((FType)MENU_INDENT2) + s + " " + ((v > 0) ? (String(v) + " s") : "(disabled)");
  Serial.print((FType)MENU_INDENT2);
  Serial.print(s + F(": "));
  for (size_t k = 20; k > s.length(); k--) Serial.print(' ');
  if (v > 0) {
    char buff[8];
    sprintf(buff,"%5d s",v);
    Serial.print(buff);
  } else {
    Serial.print(F("(disabled)"));
  }
  Serial.println();
}


//----------------------------------------------
/* Prints to serial various sensor timing settings.
   Intended to be used just below menu's sensor timing settings entry. */
void showMenuSensorTimingSettings() {
  showMenuSensorTimingEntry(F("Temperature/humidity"),(getPodConfig().humidityT));
  showMenuSensorTimingEntry(F("Radiant temperature"),(getPodConfig().tempT));
  showMenuSensorTimingEntry(F("Light"),(getPodConfig().lightT));
  showMenuSensorTimingEntry(F("Sound level"),(getPodConfig().soundT));
  showMenuSensorTimingEntry(F("Particulate matter"),(getPodConfig().pmT));
  showMenuSensorTimingEntry(F("Carbon dioxide"),(getPodConfig().co2T));
  showMenuSensorTimingEntry(F("Carbon monoxide"),(getPodConfig().coT));
  //showMenuSensorTimingEntry(F("Data upload interval"),(getPodConfig().uploadT));
}


//----------------------------------------------
/* Prints to serial various network settings.
   Intended to be used just below menu's network settings entry. */
void showMenuNetworkSettings() {
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Connected:   "));
  Serial.print(ethernetConnected() ? F("yes") : F("no"));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("MAC address: "));
  Serial.print(getMACAddressString());
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("IP address:  "));
  Serial.print(Ethernet.localIP());
  if ((getNetworkFlags() & 0x01) != 0) {
    Serial.print(F("  [static]"));
  } else {
    Serial.print(F("  [DHCP]"));
  }
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Gateway:     "));
  Serial.print(Ethernet.gatewayIP());
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Subnet:      "));
  Serial.print(Ethernet.subnetMask());
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("DNS server:  "));
  Serial.print(Ethernet.dnsServerIP());
  Serial.println();
  //Serial.print((FType)MENU_INDENT2);
  //Serial.print(F("XBee group:     "));
  //Serial.print(getXBeeGroup());
  //Serial.println();
}


//----------------------------------------------
/* Prints to serial various XBee settings.
   Intended to be used just below menu's XBee settings entry. */
void showMenuXBeeSettings() {
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Serial number:  "));
  Serial.print(getXBeeSerialNumberString());
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Destination:    "));
  Serial.print(getXBeeDestinationString());
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Network group:  "));
  Serial.print(getXBeeGroup());
  Serial.println();
}


//----------------------------------------------
/* Prints to serial various clock settings.
   Intended to be used just below menu's clock settings entry. */
void showMenuClockSettings() {
  // Old clock routines
  //Serial.print((FType)MENU_INDENT2);
  //Serial.print(F("Current time: "));
  //Serial.print(formatDateTime());
  //Serial.println();
  time_t utc = getUTC();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Universal time: "));
  Serial.print(getUTCDateTimeString(utc));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Local time:     "));
  Serial.print(getLocalDateTimeString(utc));
  Serial.println();
  Serial.print((FType)MENU_INDENT2);
  Serial.print(F("Unix timestamp:   "));
  Serial.print(utc);
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update project settings over the serial interface. */
void configureProjectSettings() {
  String s;
  
  Serial.println();
  Serial.println(F("Current settings are shown in square brackets."));
  Serial.println(F("Press enter to keep the current setting."));
  Serial.println();

  s = serialStringPrompt(F("Upload server"),getPodConfig().server);
  if (!s.equals(getPodConfig().server)) {
    replaceSettingString(s,getPodConfig().server,sizeof(getPodConfig().server),F("Server"));
  }
  
  s = serialStringPrompt(F("Project name (max 16 characters)"),getPodConfig().project);
  if (!s.equals(getPodConfig().project)) {
    replaceSettingString(s,getPodConfig().project,sizeof(getPodConfig().project),F("Project name"));
  }
  
  s = serialStringPrompt(F("Project setup date (YYYY-MM-DD)"),getPodConfig().setupD);
  if (!s.equals(getPodConfig().setupD)) {
    replaceSettingString(s,getPodConfig().setupD,sizeof(getPodConfig().setupD),F("Project setup date"));
  }
  
  s = serialStringPrompt(F("Project teardown date (YYYY-MM-DD)"),getPodConfig().teardownD);
  if (!s.equals(getPodConfig().teardownD)) {
    replaceSettingString(s,getPodConfig().teardownD,sizeof(getPodConfig().teardownD),F("Project teardown date"));
  }
  
  // Use XBee Grove dev board + XCTU exclusively to configure
  // XBee network, aside from coordinator mode: current firmware
  // not set up to provide access to many of the useful settings
  // and it is too easy to misconfigure the XBee. 
  /*
  s = serialStringPrompt(F("Project network ID (hexadecimal from 0000 to 07FFF)"),getPodConfig().networkID);
  if (!s.equals(getPodConfig().networkID)) {
    replaceSettingString(s,getPodConfig().networkID,sizeof(getPodConfig().networkID),F("Network ID"));
  }
  */
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update node settings over the serial interface. */
void configureNodeSettings() {
  bool b;
  String s;
  
  Serial.println();
  Serial.println(F("Current settings are shown in square brackets."));
  Serial.println(F("Press enter to keep the current setting."));
  Serial.println();

  s = serialStringPrompt(F("Device ID (max 16 characters)"),getPodConfig().devid);
  if (!s.equals(getPodConfig().devid)) {
    replaceSettingString(s,getPodConfig().devid,sizeof(getPodConfig().devid),F("Device ID"));
  }
  
  s = serialStringPrompt(F("Room/location (max 16 characters)"),getPodConfig().room);
  if (!s.equals(getPodConfig().room)) {
    replaceSettingString(s,getPodConfig().room,sizeof(getPodConfig().room),F("Room/location"));
  }

  bool iscoord = getModeCoord();
  b = serialYesNoPrompt(F("Network coordinator (y/n)"),true,iscoord);
  if (b != iscoord) {
    setPodConfigChanged();
    if (b) {
      getPodConfig().coord = 'Y';
      Serial.println(F("  This node now set to network coordinator."));
    } else {
      getPodConfig().coord = 'N';
      Serial.println(F("  This node is no longer the network coordinator."));
    }
  }
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update sensor measurement timing settings over the
   serial interface. */
void configureSensorTimingSettings() {
  Serial.println();
  Serial.println(F("Enter the number of seconds between measurements for each sensor,"));
  Serial.println(F("or '0' to disable the sensor.  Current settings are shown in"));
  Serial.println(F("square brackets (press enter to keep the current setting)."));
  Serial.println();
  
  updateSensorTime(F("Temperature/humidity"),&(getPodConfig().humidityT));
  updateSensorTime(F("Radiant temperature"),&(getPodConfig().tempT));
  updateSensorTime(F("Light"),&(getPodConfig().lightT));
  updateSensorTime(F("Sound level"),&(getPodConfig().soundT));
  updateSensorTime(F("Particulate matter"),&(getPodConfig().pmT));
  updateSensorTime(F("Carbon dioxide"),&(getPodConfig().co2T));
  updateSensorTime(F("Carbon monoxide"),&(getPodConfig().coT));
  //updateSensorTime(F("Data upload interval"),&(getPodConfig().uploadT));
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update network settings over the serial interface. */
void configureNetworkSettings() {
  bool b;
  bool usingStaticIP = (getNetworkFlags() & 0x01) != 0;

  Serial.println(F("Current ethernet status:"));
  Serial.print(F("  Connected:   "));
  Serial.print(ethernetConnected() ? F("yes") : F("no"));
  Serial.println();
  Serial.print(F("  MAC address: "));
  Serial.print(getMACAddressString());
  Serial.println();
  Serial.print(F("  IP address:  "));
  Serial.print(Ethernet.localIP());
  if (usingStaticIP) {
    Serial.print(F("  [static]"));
  } else {
    Serial.print(F("  [DHCP]"));
  }
  Serial.println();
  Serial.print(F("  DNS server:  "));
  Serial.print(Ethernet.dnsServerIP());
  Serial.println();
  Serial.println();
  
  Serial.println(F("The IP address can be set to a fixed (static) value or assigned"));
  Serial.println(F("automatically using DHCP.  This PODD is currently set to use"));
  if (usingStaticIP) {
    Serial.println(F("a static IP address."));
    Serial.println();
    b = serialYesNoPrompt(F("Switch to DHCP (y/n)?"),true,false);
    if (b) {
      usingStaticIP = false;
      setNetworkFlags(getNetworkFlags() & 0xFE);
    }
  } else {
    Serial.println(F("DHCP."));
    Serial.println();
    b = serialYesNoPrompt(F("Switch to a static IP address (y/n)?"),true,false);
    if (b) {
      usingStaticIP = true;
      setNetworkFlags((getNetworkFlags() & 0xFE) | 0x01);
    }
  }
  Serial.println();

  if (usingStaticIP) {
    IPAddress ip;
    String ipstr;
    String s;

    // Static IP address
    ip = IPAddress(getNetworkStaticIP());
    //ipstr = ip.toString();
    ipstr = String() + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3];
    s = serialStringPrompt(F("Static IP address"),ipstr);
    if (!s.equals(ipstr)) {
      IPAddress newip;
      if (newip.fromString(s)) {
        setNetworkStaticIP((uint32_t)newip);
        Serial.print(F("Static IP address changed to "));
        Serial.print(IPAddress(getNetworkStaticIP()));
        Serial.println();
      } else {
        Serial.println(F("Invalid IP address.  Static IP address will not be changed."));
      }
    }
    
    // Gateway IP address
    ip = IPAddress(getNetworkGatewayIP());
    //ipstr = ip.toString();
    ipstr = String() + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3];
    s = serialStringPrompt(F("Gateway IP address"),ipstr);
    if (!s.equals(ipstr)) {
      IPAddress newip;
      if (newip.fromString(s)) {
        setNetworkGatewayIP((uint32_t)newip);
        Serial.print(F("Gateway IP address changed to "));
        Serial.print(IPAddress(getNetworkGatewayIP()));
        Serial.println();
      } else {
        Serial.println(F("Invalid IP address.  Gateway IP address will not be changed."));
      }
    }
    
    // Subnet mask
    ip = IPAddress(getNetworkSubnetMask());
    //ipstr = ip.toString();
    ipstr = String() + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3];
    s = serialStringPrompt(F("Subnet mask"),ipstr);
    if (!s.equals(ipstr)) {
      IPAddress newip;
      if (newip.fromString(s)) {
        setNetworkSubnetMask((uint32_t)newip);
        Serial.print(F("Subnet mask changed to "));
        Serial.print(IPAddress(getNetworkSubnetMask()));
        Serial.println();
      } else {
        Serial.println(F("Invalid subnet mask.  Subnet mask will not be changed."));
      }
    }
    
    // DNS server IP address
    ip = IPAddress(getNetworkDNSServerIP());
    //ipstr = ip.toString();
    ipstr = String() + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3];
    s = serialStringPrompt(F("DNS server IP address"),ipstr);
    if (!s.equals(ipstr)) {
      IPAddress newip;
      if (newip.fromString(s)) {
        setNetworkDNSServerIP((uint32_t)newip);
        Serial.print(F("DNS server IP address changed to "));
        Serial.print(IPAddress(getNetworkDNSServerIP()));
        Serial.println();
      } else {
        Serial.println(F("Invalid IP address.  DNS server IP address will not be changed."));
      }
    }
    
    Serial.println();
  }
  
  // Save network configuration to EEPROM (no actual writes if settings
  // did not change).
  saveNetworkConfig();
  
  b = serialYesNoPrompt(F("(Re)initialize ethernet connection (y/n)?"),true,false);
  if (b) {
    Serial.println(F("Initializing ethernet...."));
    ethernetBegin(1);
  }
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update XBee settings over the serial interface. */
void configureXBeeSettings() {
  int i;

  Serial.println(F("XBee settings:"));
  Serial.print(F("  Serial number:   "));
  Serial.print(getXBeeSerialNumberString());
  Serial.println();
  Serial.print(F("  Destination:     "));
  Serial.print(getXBeeDestinationString());
  Serial.println();
  Serial.println();
  Serial.println(F("The destination address is that of the most recent coordinator"));
  Serial.println(F("this PODD has been associated with (or the XBee broadcast"));
  Serial.println(F("address if this PODD was a coordinator itself).  If the"));
  Serial.println(F("coordinator changes, the new address will automatically be"));
  Serial.println(F("acquired after this PODD enters running mode.  It may take"));
  Serial.println(F("1-2 minutes to acquire the new address."));
  Serial.println(F(""));
  Serial.println(F("PODDs may be configured in multiple independent groups, each"));
  Serial.println(F("with its own XBee network.  Each group must have its own"));
  Serial.println(F("coordinator unit to upload sensor data within that group."));
  Serial.println(F(""));
  
  i = serialIntegerPrompt(F("New PODD group (1-7) or enter to keep current"),true,getXBeeGroup());
  if (i != getXBeeGroup()) {
    if ((i >= 1) && (i <= 7)) {
      Serial.println(F("Updating PODD group to ") + String(i) + F("."));
      setXBeeGroup(i);
    } else {
      Serial.println(F("Invalid group: must be within 1-7."));
    }
  }
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update clock settings over the serial interface. */
void configureClockSettings() {
  bool b;
  
  // Time zone settings
  String stz = getStandardTimezoneLabel();
  String dtz = getDaylightSavingTimezoneLabel();
  Serial.println();
  Serial.println(F("Current timezone:"));
  Serial.print(F("    standard time:        "));
  Serial.println(stz);
  Serial.print(F("    daylight saving time: "));
  Serial.println(dtz);
  b = serialYesNoPrompt(F("Change timezone (y/n)?"),true,false);
  if (b) {
    Serial.println();
    Serial.println(F("Enter the three-character label for the standard and daylight saving"));
    Serial.println(F("time zone.  If daylight saving is not observed, use the same label"));
    Serial.println(F("for both periods.  Timezones are currently limited to 'EST', 'EDT',"));
    Serial.println(F("'CST', 'CDT', 'MST', 'MDT', 'PST', 'PDT', and 'UTC'.  Unknown labels"));
    Serial.println(F("will be set to UTC."));
    Serial.println();
    String stz0 = serialStringPrompt("Standard time",stz);
    String dtz0 = serialStringPrompt("Daylight saving time",dtz);
    if (!stz0.equals(stz) || !dtz0.equals(dtz)) {
      setTimezone(stz0,dtz0);
      // Get new timezone labels; may differ from stz0/dtz0 if those
      // contained unknown timezone labels.
      stz = getStandardTimezoneLabel();
      dtz = getDaylightSavingTimezoneLabel();
      Serial.println();
      Serial.println(F("New timezone:"));
      Serial.print(F("    standard time:        "));
      Serial.println(stz);
      Serial.print(F("    daylight saving time: "));
      Serial.println(dtz);
    }
  }

  // Clock settings
  Serial.println();
  time_t utc = getUTC();
  Serial.println(F("Current time:"));
  Serial.print(F("    universal time: "));
  Serial.println(getUTCDateTimeString(utc));
  Serial.print(F("    local time:     "));
  Serial.println(getLocalDateTimeString(utc));
  Serial.print(F("    unix timestamp:   "));
  Serial.println(utc);
  b = serialYesNoPrompt(F("Change current time (y/n)?"),true,false);
  if (b) {
    Serial.println();
    Serial.println(F("Enter the current time as a unix timestamp (seconds since 00:00:00"));
    Serial.println(F("on 1970-01-01 UTC).  The current unix time can be found on various"));
    Serial.println(F("websites by searching for \"unix time\".  Note the unix timestamp is"));
    Serial.println(F("defined to be relative to UTC, not the local timezone.  The timestamp"));
    Serial.println(F("will be applied at the point in time at which it is submitted."));
    Serial.println();
    time_t utc0 = serialLongPrompt(F("New unix timestamp"),true,utc);
    // Require reasonably recent timestamp
    if (utc0 < 1000000000ul) {
      Serial.println(F("Invalid timestamp: must be greater than 1000000000 (2001-09-09)."));
      Serial.println(F(""));
    // Do nothing if timestamp did not change
    } else if (utc0 != utc) {
      setUTC(utc0);
      Serial.println();
      utc = getUTC();
      Serial.println(F("Current time:"));
      Serial.print(F("    universal time: "));
      Serial.println(getUTCDateTimeString(utc));
      Serial.print(F("    local time:     "));
      Serial.println(getLocalDateTimeString(utc));
      Serial.print(F("    unix timestamp:   "));
      Serial.println(utc);
    }
  }
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Prompt the user to update debugging settings over the serial interface. */
void configureDebugSettings() {
  //bool b;

  Serial.println(F("Debugging mode enables additional serial output.  For drone"));
  Serial.println(F("devices, this means keeping the USB system active, which is"));
  Serial.println(F("normally disabled to save power.  Debug mode is disabled"));
  Serial.println(F("when the device starts and must be manually enabled each"));
  Serial.println(F("time it is desired."));
  Serial.println(F(""));
  /*
  b = false;
  if (getDebugMode()) {
    Serial.println(F("Debug mode is currently enabled."));
    b = serialYesNoPrompt(F("Disable debug mode (y/n)?"),true,false);
  } else {
    Serial.println(F("Debug mode is currently disabled."));
    b = serialYesNoPrompt(F("Enable debug mode (y/n)?"),true,false);
  }
  if (b) {
    setDebugMode(!getDebugMode());
  }
  */
  
  setDebugMode(!getDebugMode());
  if (getDebugMode()) {
    Serial.println(F("Debug mode is now enabled."));
  } else {
    Serial.println(F("Debug mode is now disabled."));
  }
  
  Serial.println();
}


//------------------------------------------------------------------------------
/* Sensor setup/config/testing menu.  Will continue to be shown until
   an appropriate selection is made. */
void sensorMenu() {
  // Will continue offering menu until appropriate response received.
  while (true) {
    // Clear terminal by printing a bunch of blank lines
    for (int k = 0; k < 100; k++) Serial.println();
    // Print menu options
    Serial.println();
    //Serial.println(F("======== SENSOR MENU ==================================================="));
    //Serial.println(F("<<<<<<<< SENSOR MENU >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
    Serial.println(F("========================================================================"));
    Serial.println(F("|        SENSOR MENU                                                   |"));
    Serial.println(F("========================================================================"));
    Serial.println();
    // Continuously show sensor data
    Serial.println(F("  (1) Run sensor test"));
    // Show whether sensors are connected/available
    Serial.println(F("  (2) Show sensor availability"));
    // Start/stop sound sampling
    if (isSoundSampling()) {
      Serial.println(F("  (3) Stop sound sampling"));
    } else {
      Serial.println(F("  (3) Start sound sampling"));
    }
    // Calibrate CO2 sensor
    Serial.println(F("  (4) Calibrate CO2 sensor"));
    // Power on/off the particulate matter sensor
    if (isPMSensorPowered()) {
      Serial.println(F("  (5) Power off particulate matter sensor"));
    } else {
      Serial.println(F("  (5) Power on particulate matter sensor"));
    }
    // Clean particulate matter sensor
    Serial.println(F("  (6) Clean particulate matter sensor"));
    Serial.println();
    // Leave menu and return to main menu
    Serial.println(F("  (Q) Quit sensor menu"));
    Serial.println();
    //Serial.println(F("                        ------------------------                        "));
    Serial.println(F("        ----------------                                                "));
    //Serial.println(F("========================================================================"));
    //Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));

    // Get selection
    clearSerial();
    Serial.println();
    Serial.print("Enter the item to view/configure: ");
    char c = getSerialChar(-1);
    Serial.println(c);  // Serial Monitor does not echo inputs
    Serial.println();

    // Process selection
    bool showContinuePrompt = true;
    switch (c) {
      case '1':
        testSensors(-1,1000);
        break;
      case '2':
        printSensorCheck();
        Serial.println();
        Serial.println(F("The presence of some sensors cannot be immediately determined, but may"));
        Serial.println(F("be inferred from the measurements shown in the sensor test routine."));
        Serial.println();
        break;
      case '3':
        if (isSoundSampling()) {
          Serial.println(F("Stopping sound sampling...."));
          stopSoundSampling();
        } else {
          Serial.println(F("Starting sound sampling...."));
          startSoundSampling();
        }
        break;
      case '4':
        sensorMenuCalibrateCO2Sensor();
        break;
      case '5':
        sensorMenuTogglePMSensor();
        break;
      case '6':
        sensorMenuCleanPMSensor();
        break;
      case 'Q':
      case 'q':
        showContinuePrompt = false;
        return;
        break;
      default:
        Serial.println(F("Invalid option.  Select an option from the list."));
        break;
    }

    if (showContinuePrompt) {
      //Serial.println();
      Serial.println((FType)CONTINUE_PROMPT);
      getSerialChar(-1);
    }
    
    Serial.println();
  }
}


//----------------------------------------------
// OLD ROUTINE
/* Calibrates CO2 sensor by allowing user to specify current CO2 level. */
void sensorMenuCalibrateCO2Sensor_OLD() {
  Serial.println(F("The CO2 sensor response can drift with time and should be recalibrated every"));
  Serial.println(F("3-6 months to maintain accuracy.  The sensor can be recalibrated if the current"));
  Serial.println(F("ambient CO2 level is known, either from another (calibrated) CO2 meter or because"));
  Serial.println(F("the PODD is outdoors or in a well-ventilated area: outdoor air has a CO2"));
  Serial.println(F("concentration of 400-450 ppm (you might find a weather/CO2 station online"));
  Serial.println(F("that provides local outdoor CO2 levels)."));
  Serial.println(F(""));
  bool b = serialYesNoPrompt(F("Proceed with calibration (y/n)?"),true,false);
  if (!b) return;
  // Get current sensor measurement
  int ppm0 = getCO2();
  if (ppm0 < 0) {
    Serial.println();
    Serial.println(F("Failed to retrieve current CO2 data: there is a problem interacting"));
    Serial.println(F("with the CO2 sensor.  Calibration cannot be performed."));
    return;
  }
  // Get "true" CO2 level
  int ppm = serialIntegerPrompt(F("Current CO2 level in ppm"),true,ppm0);
  if ((ppm < 400) || (ppm > 2000)) {
    Serial.println(F("This program only allows calibration values within 400 - 2000 ppm."));
    return;
  }
  // Send "true" level to sensor
  if (ppm != ppm0) {
    setCO2(ppm);
    Serial.print(F("CO2 sensor set to "));
    Serial.print(ppm);
    Serial.println(F(" ppm."));
    delay(600);  // 2 Hz reading rate, up to 100ms delay for readings
    int ppm1 = getCO2();
    Serial.print(F("CO2 sensor now measures "));
    Serial.print(ppm1);
    Serial.println(F(" ppm."));
  } else {
    Serial.println(F("Skipping calibration (no change)."));
  }
  Serial.println();
  return;
}


//----------------------------------------------
/* Calibrates CO2 sensor by allowing user to specify actual CO2 level
   for a given CO2 reading. */
void sensorMenuCalibrateCO2Sensor() {
  Serial.println(F("The CO2 sensor response can drift with time and should be recalibrated every"));
  Serial.println(F("3-6 months to maintain accuracy.  The sensor can be recalibrated if the current"));
  Serial.println(F("ambient CO2 level is known, either from another (calibrated) CO2 meter or because"));
  Serial.println(F("the PODD is outdoors or in a well-ventilated area: outdoor air has a CO2"));
  Serial.println(F("concentration of 400-450 ppm (you might find a weather/CO2 station online"));
  Serial.println(F("that provides local outdoor CO2 levels).  The sensor can also be recalibrated"));
  Serial.println(F("if the actual CO2 level is known for any particular sensor reading in the past."));
  Serial.println(F(""));
  bool b = serialYesNoPrompt(F("Proceed with calibration (y/n)?"),true,false);
  if (!b) return;
  // Get current sensor measurement
  int ppm0 = getCO2();
  if (ppm0 < 0) {
    Serial.println();
    Serial.println(F("Failed to retrieve current CO2 data: there is a problem interacting"));
    Serial.println(F("with the CO2 sensor.  Calibration cannot be performed."));
    return;
  }
  Serial.println();
  Serial.println(F("Provide both a sensor reading and the corresponding actual CO2 level."));
  Serial.print(F("The current CO2 level is "));
  Serial.print(ppm0);
  Serial.println(F(" ppm."));
  Serial.println(F(""));
  // Get "true" CO2 level
  int ppm_reading = serialIntegerPrompt(F("Sensor reading in ppm"),true,ppm0);
  int ppm_actual = serialIntegerPrompt(F("Actual CO2 level for above reading in ppm"),true,ppm_reading);
  if ((ppm_actual < 400) || (ppm_actual > 2000)) {
    Serial.println(F("This program only allows calibration values within 400 - 2000 ppm."));
    return;
  }
  // Send "true" level to sensor
  if (ppm_reading != ppm_actual) {
    setCO2(ppm_reading,ppm_actual);
    Serial.print(F("CO2 sensor recalibrated so that "));
    Serial.print(ppm_reading);
    Serial.print(F(" now reads as "));
    Serial.print(ppm_actual);
    Serial.println(F(" ppm."));
    delay(600);  // 2 Hz reading rate, up to 100ms delay for readings
    int ppm1 = getCO2();
    Serial.print(F("CO2 sensor now measures "));
    Serial.print(ppm1);
    Serial.println(F(" ppm."));
  } else {
    Serial.println(F("Skipping calibration (no change)."));
  }
  Serial.println();
  return;
}


//----------------------------------------------
/* Toggles the powered state of the particulate matter sensor.
   Will also start/stop the sensor. */
void sensorMenuTogglePMSensor() {
  if (isPMSensorPowered()) {
    Serial.println(F("Powering down particulate matter sensor...."));
    if (isPMSensorRunning()) {
      stopPMSensor();
      delay(100);
    }
    powerOffPMSensor();
    //delay(1000);
  } else {
    Serial.println(F("Powering up particulate matter sensor...."));
    powerOnPMSensor();
    delay(1000);
    startPMSensor();
    //bool b = serialYesNoPrompt(F("Start sensor running as well (y/n)?"),true,true);
    //if (b) {
    //  delay(1000);
    //  startPMSensor();
    //}
  }
}


//----------------------------------------------
/* Cleans particulate matter sensor by running its fan at high speed.
   Hardware-internal routine that takes ~ 10 seconds. */
void sensorMenuCleanPMSensor() {
  bool b = serialYesNoPrompt(F("Clean particulate matter sensor by running its fan at high speed\nfor 10-12 seconds (y/n)?"),true,true);
  if (!b) return;
  
  // Must turn on and start sensor if not already running
  bool wasPMPowered = isPMSensorPowered();
  bool wasPMRunning = isPMSensorRunning();
  if (!wasPMPowered) {
    //Serial.println(F("Initializing particulate matter sensor...."));
    //initPMSensor();
    //delay(100);
    Serial.println(F("Powering up particulate matter sensor...."));
    powerOnPMSensor();
    delay(1000);
  }
  if (!wasPMRunning && isPMSensorPowered()) {
    Serial.println(F("Starting particulate matter sensor...."));
    startPMSensor(false);
    delay (1000);
  }
  
  // Send clean command and wait for it to complete.
  // Called routine already writes info line to serial when
  // clean command successfully sent to sensor.
  if (!cleanPMSensor(true)) {
    Serial.println(F("Failed to initiate/complete sensor cleaning process (unknown reason)."));
  }
  
  // Turn off and/or stop sensor if we started it
  if (!wasPMRunning && isPMSensorRunning()) {
    Serial.println(F("Stopping particulate matter sensor...."));
    stopPMSensor();
  }
  if (!wasPMPowered && isPMSensorPowered()) {
    Serial.println(F("Powering down particulate matter sensor...."));
    powerOffPMSensor();
  }
}


//==============================================================================
