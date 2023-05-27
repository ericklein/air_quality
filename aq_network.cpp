#include "Arduino.h"
#include "aq_network.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Shared helper function we call here too...
extern void debugMessage(String messageText, int messageLevel);

// Includes and defines specific to WiFi network connectivity
#ifdef WIFI
  #if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
    #include <WiFiNINA.h>
  #elif defined(ARDUINO_SAMD_MKR1000)
    #include <WiFi101.h>
  #elif defined(ARDUINO_ESP8266_ESP12)
    #include <ESP8266WiFi.h>
  #else
    #include <WiFi.h>
#endif

  WiFiClient client;
  //WiFiClientSecure client; // for SSL
#endif

// Includes and defines specific to Ethernet (wired) network connectivity
#ifdef RJ45
  // Set MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  #include <SPI.h>
  #include <Ethernet.h>
  EthernetClient client;
#endif

// Network services independent of physical connection
#if defined(WIFI) || defined(RJ45)
  // NTP setup
  #include "time.h"

  // Generalized access to HTTP services atop WiFi or Ethernet connections
  #include <HTTPClient.h>
#endif

// MQTT interface depends on the underlying network client object, which is defined and
// managed here (so needs to be defined here).
#ifdef MQTT
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);
#endif

//****************************************************************************************************
// AQ_Network Class and Member Functions
//

// Initialize network and connect.  If connection succeeds initialize NTP connection so
// device can report accurate local time.  Returns boolean indicating whether network is
// connected and available.  Depends on configuration #defines in config.h to determine
// what network hardware is attached, and key network settings there as well (e.g. SSID).
bool AQ_Network::networkBegin() {
  bool result = false;

  #ifdef WIFI
    // set hostname has to come before WiFi.begin
    WiFi.hostname(CLIENT_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++) {
      // Attempts WiFi connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
      if (WiFi.status() == WL_CONNECTED)
      {
        debugMessage("WiFi IP address is: " + WiFi.localIP().toString(),1);
        debugMessage("RSSI is: " + String(getWiFiRSSI()) + " dBm",2);
        result = true;
        break;
      }
      debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed",1);
      // use of delay() OK as this is initialization code
      delay(CONNECT_ATTEMPT_INTERVAL * 1000); // convered into milliseconds
    }
  #endif

  #ifdef RJ45
    // Configure Ethernet CS pin, not needed if using default D10
    //Ethernet.init(10);  // Most Arduino shields
    //Ethernet.init(5);   // MKR ETH shield
    //Ethernet.init(0);   // Teensy 2.0
    //Ethernet.init(20);  // Teensy++ 2.0
    //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
    //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

    // Initialize Ethernet and UDP
    if (Ethernet.begin(mac) == 0) {
      // identified errors
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        debugMessage("Ethernet hardware not found",1);
      } else if (Ethernet.linkStatus() == LinkOFF) {
        debugMessage("Ethernet cable not connected",1);
      } else {
        // generic error
        debugMessage("Failed to configure Ethernet",1);
      }
    } else {
      debugMessage(String("Ethernet IP address is: ") + Ethernet.localIP().toString(),1);
      result = true;
    }
  #endif
  return (result);
}

void AQ_Network::setTime(long timeZoneOffset, long daylightOffset) 
{
  // Only use if there is a network and a screen to display data...
  #if (defined(WIFI) || defined(RJ45)) && (defined(SCREEN))
    // Get time from NTP
    configTime(timeZoneOffset, daylightOffset, ntpServer);
    debugMessage("Time zone adjusted local time : " + dateTimeString(),1);
  #endif
}

String AQ_Network::httpGETRequest(const char* serverName) {
  String payload = "{}";

#if defined(WIFI) || defined(RJ45)
  HTTPClient http;

  // servername is domain name w/URL path or IP address w/URL path
  http.begin(client, serverName);

  // Send HTTP GET request
  int httpResponseCode = http.GET();

  if (httpResponseCode == HTTP_CODE_OK) {
    // HTTP reponse OK code
    payload = http.getString();
  } else {
    debugMessage("HTTP GET error code: " + httpResponseCode,1);
    payload = "HTTP GET error";
  }
  // free resources
  http.end();
#endif
  return payload;
}

void AQ_Network::networkStop() {
#if defined(WIFI) || defined(RJ45)
  client.stop();
#endif
}

int AQ_Network::httpPOSTRequest(String serverurl, String contenttype, String payload) {
  int httpCode = -1;
#if defined(WIFI) || defined(RJ45)
  HTTPClient http;

  http.begin(client, serverurl);
  http.addHeader("Content-Type", contenttype);

  httpCode = http.POST(payload);

  // httpCode will be negative on error, but HTTP status might indicate failure
  if (httpCode > 0) {
    // HTTP POST complete, print result code
    debugMessage("HTTP POST [" + serverurl + "], result code: " + String(httpCode),2);

    // If POST succeeded, output response as debug messages
    if (httpCode == HTTP_CODE_OK) {
      const String& payload = http.getString();
      debugMessage("received payload:\n<<",2);
      debugMessage(payload,2);
      debugMessage(">>",2);
    }
  } else {
    debugMessage("HTTP POST [" + serverurl + "] failed, error: " + http.errorToString(httpCode).c_str(),1);
  }

  http.end();
  debugMessage("closing connection for dweeting",1);
#endif

  return (httpCode);
}

// Utility functions that may be of use

// Return local IP address as a String
String AQ_Network::getLocalIPString() {
#if defined(WIFI) || defined(RJ45)
  return (client.localIP().toString());
#else
  return ("No network");
#endif
}

// Return RSSI for WiFi network, simulate out-of-range value for non-WiFi
int AQ_Network::getWiFiRSSI() {
#ifdef WIFI
  return (WiFi.RSSI());
#else
  return (-255);  //Arbitrary out-of-range value
#endif
}

// Returns true if WIFI defined in config.h, otherwise false
bool AQ_Network::isWireless() {
#ifdef WIFI
  return (true);
#else
  return (false);
#endif
}

// Returns true if RJ45 (Ethernet) defined in config.h, otherwise false
bool AQ_Network::isWired() {
#ifdef RJ45
  return (true);
#else
  return (false);
#endif
}

// Returns connection status (via Client class), or false if no network defined in config.h
bool AQ_Network::isConnected() {
#if defined(WIFI) || defined(RJ45)
  return (client.connected());
#else
  return (false);
#endif
}

// Converts system time into human readable strings. Use NTP service
String AQ_Network::dateTimeString() {
  String dateTime;

#if defined(WIFI) || defined(RJ45)
  struct tm timeInfo;
  if (getLocalTime(&timeInfo)) {
    int day = timeInfo.tm_wday;
    // int month = timeInfo.tm_mon;
    // int year = timeInfo.tm_year + 1900;
    int hour = timeInfo.tm_hour;
    int minutes = timeInfo.tm_min;
    // int seconds = timeinfo.tm_sec;

    // short human readable format
    dateTime = weekDays[day];
    dateTime += " at ";
    if (hour < 10) dateTime += "0";
    dateTime += hour;
    dateTime += ":";
    if (minutes < 10) dateTime += "0";
    dateTime += minutes;

    // long human readable
    // dateTime = weekDays[day];
    // dateTime += ", ";

    // if (month<10) dateTime += "0";
    // dateTime += month;
    // dateTime += "-";
    // if (day<10) dateTime += "0";
    // dateTime += day;
    // dateTime += " at ";
    // if (hour<10) dateTime += "0";
    // dateTime += hour;
    // dateTime += ":";
    // if (minutes<10) dateTime += "0";
    // dateTime += minutes;

    // zulu format
    // dateTime = year + "-";
    // if (month()<10) dateTime += "0";
    // dateTime += month;
    // dateTime += "-";
    // if (day()<10) dateTime += "0";
    // dateTime += day;
    // dateTime += "T";
    // if (hour<10) dateTime += "0";
    // dateTime += hour;
    // dateTime += ":";
    // if (minutes<10) dateTime += "0";
    // dateTime += minutes;
    // dateTime += ":";
    // if (seconds<10) dateTime += "0";
    // dateTime += seconds;
    // switch (gmtOffset_sec)
    // {
    //   case 0:
    //     dateTime += "Z";
    //     break;
    //   case -28800:
    //     dateTime += "PDT";
    //     break;
    // }
  } else {
    dateTime = "Can't reach time service";
  }
#else
  // If no network defined
  dateTime = "No network to set time";
#endif

  return dateTime;
}