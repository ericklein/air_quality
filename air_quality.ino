/*
  Project Name:   air_quality
  Developer:      Eric Klein Jr. (temp2@ericklein.com)
  Description:    Regularly sample and log temperature, humidity

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
//#define DEBUG         // Output to the serial port
//#define SDLOG         // output sensor data to SD Card
//#define SCREEN        // output sensor data to screen
#define MQTTLOG        // Output to MQTT broker defined in secrets.h
#define RJ45            // use Ethernet
//#define WIFI          // use WiFi (credentials in secrets.h)
//#if defined(SDLOG) || defined(DEBUG)
#define NTP         // query network time server for logging
//#endif

// Gloval variables
unsigned long syncTime = 0;        // holds millis() [milliseconds] for timing functions 

// AHT20 (temperature and humidity)
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;

#ifdef DEBUG
  // millisecond delay between sensor reads
  #define LOG_INTERVAL 60000  // standard test interval
  //#define LOG_INTERVAL 600000   // long term test interval
#else
  #define LOG_INTERVAL 600000
#endif

#ifdef SDLOG
  #include <SPI.h>
  #include <SD.h>
  #define SDPIN 4
  File logfile;
#endif

// MQTT credentials and network device setup
#include "secrets.h"

#ifdef WIFI
  #if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
    #include <WiFiNINA.h>
  #elif defined(ARDUINO_SAMD_MKR1000)
    #include <WiFi101.h>
  #elif defined(ARDUINO_ESP8266_ESP12)
    #include <ESP8266WiFi.h>
  #endif

  WiFiClient client;
  //WiFiClientSecure client; // for SSL

  #ifdef NTP
    #include <WiFiUdp.h>
    WiFiUDP Udp;
    unsigned int localPort = 8888;       // local port to listen for UDP packets
  #endif
#endif

#ifdef RJ45
  // Set MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  #include <SPI.h>
  #include <Ethernet.h>
  EthernetClient client;

  #ifdef NTP
    #include <EthernetUdp.h>
    EthernetUDP Udp;
    unsigned int localPort = 8888;       // local port to listen for UDP packets
  #endif
#endif

#ifdef MQTTLOG
  unsigned long previousMQTTPingTime = 0;
  #include "Adafruit_MQTT.h"
  #include "Adafruit_MQTT_Client.h"
  Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC1);
  Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC2);
  Adafruit_MQTT_Publish roomPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC4);
#endif

#ifdef NTP
  // Time (and time related networking)
  #include <TimeLib.h>
  // NTP Servers
  //IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
  // IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
  // IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
  IPAddress timeServer(132,163,97,6); // time.nist.gov

  // Time Zone support
  const int timeZone = 0;     //UTC
  //const int timeZone = -5;  // Eastern Standard Time (USA)
  //const int timeZone = -4;  // Eastern Daylight Time (USA)
  //const int timeZone = -8;  // Pacific Standard Time (USA)
  //const int timeZone = -7;    // Pacific Daylight Time (USA)
  const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
#endif

#ifdef SCREEN
  #include <SPI.h> 
  #include <Wire.h>
  //#include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  //#include <Adafruit_SH110X.h>
  Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
  //Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
  //#include <Adafruit_LiquidCrystal.h>
  //#define lcdRows   2
  //#define lcdColumns 16
  //Adafruit_LiquidCrystal display(0); //i2c connection, default address #0 (A0-A2 not jumpered)
#endif

void setup() 
{
  // used for fatal error messaging
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial)
    {
      // wait for serial port
    }
  #endif
  debugMessage("Indoor Air Quality Started");

  // sensor check
  if (! aht.begin())
  {
    #ifdef DEBUG
      Serial.println("FATAL ERROR: AHT sensor not detected");
    #endif
    #ifdef SCREEN
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0);
      display.print("ERR 01");
      display.setCursor(0,8*2);
      display.print("Sensor")
      display.display(); 
    #endif
    while (1)
      {
        // endless loop communicating fatal error 02
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
  }
  #ifdef DEBUG
    Serial.println("AHT10 or AHT20 sensor ready");
  #endif

  #ifdef SCREEN
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
    //display.begin(0x3C, true); // Address 0x3C default
    //display.begin(lcdColumns, lcdRows);

    // Set display parameters 
    //display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    //display.setTextColor(SH110X_WHITE);
    //display.setRotation(1); // SH110X only?

    screenMessage(true,0,,"Waiting for");
    screenMessage(false,1,,"first read")
  #endif

  #ifdef SDLOG
    // initialize SD card
    if (!SD.begin(SDPIN)) 
    {
      debugMessage("Card failed, or not present");
      screenMessage(true,0,,"SD card");
      screenMessage(false,1,,"failure");
      stopApp;
    }
  
    // create a new file on SD card
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) 
    {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) 
      {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
  
    if (! logfile) 
    {
      debugMessage("Couldn't create log file on SD card");
      screenMessage(true,0,,"log file");
      screenMessage(false,1,,"failure");
      stopApp;
    }
  
    #ifdef DEBUG
      String message = "Logging to: " + filename;
      debugMessage(message);
    #endif

    // Log output headers
      logfile.println("time,room,humidity,temp");    
  #endif

  #ifdef WIFI
    uint8_t tries = 1;
    #define MAXTRIES 11

    // Connect to WiFi access point
    String message = "Connecting to WiFI AP: " + WIFI_SSID;
    debugMessage(message);
  
    // set hostname has to come before WiFi.begin
    WiFi.hostname(MQTT_CLIENT_ID);
    // WiFi.setHostname(MQTT_CLIENT_ID); //for WiFiNINA
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) 
    {
      // Error handler - WiFi does not initially connect
      message = "WiFi AP connect attempt " + tries + "of " + MAXTRIES + " in " + (tries*10) +" seconds";
      debugMessage(message);
      // use of delay OK as this is initialization code
      delay(tries*10000);
      tries++;

      // FATAL ERROR 03 - WiFi doesn't connect
      if (tries == MAXTRIES)
      {
        message = "Can not connect to WiFi after " + MAXTRIES + " attempts";
        debugMessage(message);
        screenMessage(true,0,,"WiFi AP");
        screenMessage(false,1,,"failure");
        #ifndef SDLOG
          stopApp;
        #endif
      }
    }
    message = "WiFi IP address is: " + (WiFi.localIP());
    debugMessage(message);
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
    if (Ethernet.begin(mac) == 0)
    {      
      // identified errors
      if (Ethernet.hardwareStatus() == EthernetNoHardware)
        debugMessage("Ethernet hardware not found");
      else if (Ethernet.linkStatus() == LinkOFF) 
        debugMessage("Ethernet cable is not connected");
      else
        // generic error
        debugMessage("Failed to configure Ethernet using DHCP");
      screenMessage(true,0,,"Ethernet");
      screenMessage(false,1,,"failure");
      #ifndef SDLOG
        stopApp;
      #endif
    }
    String message = "Ethernet IP address is: " + (Ethernet.localIP);
    debugMessage(message);
  #endif
    
  #ifdef NTP
    // Get time from NTP
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);

    // wait until the time is set by the sync provider
    while(timeStatus()== timeNotSet);

    #ifdef DEBUG
      Serial.print("The NTP time is ");
      Serial.println(zuluDateTimeString());
    #endif
  #endif

  #ifdef DEBUG
    Serial.println("setup() complete");
  #endif
}

void loop() 
{
  #ifdef DEBUG
    // display heartbeat every 20 seconds between sensor reads; depending on client might display >1 message per interval
    if (((millis() - syncTime) % 20000) == 0)
    {
      Serial.print(((millis()-syncTime)/1000));
      Serial.print(" seconds elapsed before next read at ");
      Serial.print(LOG_INTERVAL/1000);
      Serial.println(" seconds");
    }
  #endif

  #ifdef MQTTLOG
  {
    MQTT_connect();
  }
  #endif

  // update display and determine if it is time to read/process sensors
  if ((millis() - syncTime) < LOG_INTERVAL) return;
  syncTime = millis();

  // populate temp and humidity objects with fresh data
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  #ifdef SCREEN
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.print("Tmp:");
    display.println((temp.temperature*1.8+32));  // will get truncated to 2 decimal places
    //display.setCursor(0,8*2);           // associated with text size
    display.print("Hum:");
    display.println(humidity.relative_humidity);
    display.display(); 
  #endif

  #ifdef MQTTLOG
    #ifdef DEBUG
      Serial.print("temperature via MQTT publish to '");
      Serial.print(MQTT_PUB_TOPIC1);
    #endif
    if (!tempPub.publish((temp.temperature*1.8+32)))
    {
      #ifdef DEBUG
        Serial.println("' failed");
      #endif
      #ifdef SCREEN
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0,0);
          display.print("ERR 101");
          display.setCursor(0,8*2);
          display.print("MQTT");
          display.display(); 
      #endif
    }
    else 
    {
      #ifdef DEBUG
        Serial.println("' successful");
      #endif
    }
    #ifdef DEBUG
      Serial.print("humidity via MQTT publish to '");
      Serial.print(MQTT_PUB_TOPIC2);
    #endif
    if (!humidityPub.publish(humidity.relative_humidity))
    {
      #ifdef DEBUG
        Serial.println("' failed");
      #endif
    }
    else 
    {
      #ifdef DEBUG
        Serial.println("' successful");
      #endif
    }
    #ifdef DEBUG
      Serial.print("room name via MQTT publish to '");
      Serial.print(MQTT_PUB_TOPIC4);
    #endif
    if (!roomPub.publish(room))
    {
      #ifdef DEBUG
        Serial.println("' failed");
      #endif
    }
    else 
    {
      #ifdef DEBUG
        Serial.println("' successful");
      #endif
    }
  #endif

  #if defined(SDLOG) || defined(DEBUG)
    String logString = zuluDateTimeString();
    logString += ",";    
    logString += room;
    logString += ",";
    logString += humidity.relative_humidity;
    logString += ",";
    logString += temp.temperature*1.8+32;
  #endif

  #ifdef SDLOG
    logfile.println(logString);
    logfile.flush();
    #ifdef DEBUG
      Serial.println("Log data written to SD card");
    #endif
  #endif

  #ifdef DEBUG
    Serial.println(logString);
  #endif
}

#ifdef NTP
  time_t getNtpTime()
  {
    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    #ifdef DEBUG
      Serial.println("Transmitting NTP Request");
    #endif
    sendNTPpacket(timeServer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500)
    {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE)
      {
        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      }
    }
    #ifdef DEBUG
      Serial.println("No NTP response");
    #endif
    return 0; // return 0 if unable to get the time
  }

  // send an NTP request to the time server at the given address
  void sendNTPpacket(IPAddress &address)
  {
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    // send a packet requesting a timestamp:                 
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
#endif

String zuluDateTimeString()
{
  String logString;
  #ifdef NTP
    logString = year();
    logString += "-";
    if (month()<10) logString += "0";
    logString += month();
    logString += "-";
    if (day()<10) logString += "0";
    logString += day();
    logString += "T";
    if (hour()<10) logString += "0";
    logString += hour();
    logString += ":";
    if (minute()<10) logString += "0";
    logString += minute();
    logString += ":";
    if (second()<10) logString += "0";
    logString += second();
    logString += "Z";
  #else
    logString ="Time not set";
  #endif
  return logString;
}

#ifdef MQTTLOG
  // Connects and reconnects to MQTT broker, call from loop() to maintain connection
  void MQTT_connect()
  {
    uint8_t mqttErr;
    uint8_t tries = 1;
    #define MAXTRIES 3

    // exit if already connected
    if (mqtt.connected()) 
    {
      return;
    }
    #ifdef DEBUG
      Serial.print("connecting to MQTT broker: ");
      Serial.println(MQTT_BROKER);
    #endif

    while ((mqtt.connect() != 0)&&(tries<=MAXTRIES))
    {
      // Error handler - can not connect to MQTT broker
      #ifdef DEBUG
        Serial.println(mqtt.connectErrorString(mqttErr));
        Serial.print("MQTT broker connect attempt ");
        Serial.print(tries);
        Serial.print(" of ");
        Serial.print(MAXTRIES);
        Serial.print(" in ");
        Serial.print(tries*10);
        Serial.println(" seconds");
      #endif
      mqtt.disconnect();
      delay(tries*10000);
      tries++;

      if (tries == MAXTRIES) 
      {
        digitalWrite(LED_BUILTIN, HIGH);
        #ifdef SCREEN
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0,0);
          display.print("ERR 10");
          display.setCursor(0,8*2);
          display.print("MQTT");
          display.display(); 
        #endif
      }
    }
    if (tries<=MAXTRIES)
    {
      #ifdef DEBUG
        Serial.println("connected to MQTT broker");
      #endif
    }
  }
#endif

void debugMessage(String messageText)
{
  #ifdef DEBUG
    Serial.println(messageText);
  #endif
}

// void screenMessage(int row, int column, int textSize, String messageText)
void screenMessage(bool clear, int row, int textSize, String messageText)
// first row is zero
// textSize is for pixel displays only
{
  #ifdef SCREEN
    // clear the screen if needed
    if (clear)
      // Clear the display
      display.clearDisplay();
      //display.clear(); // ???
    // set the row to display text in
    display.setCursor(1, row);
    // set the textSize if needed
    if (textSize)
      display.setTextSize(textSize);
    // add text to display
    display.print(messageText);
    // actually display the message
    display.display();
  #endif
}

void stopApp()
{
  while(1)
  {
    // endless loop communicating fatal error
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}