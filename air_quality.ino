/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
#define DEBUG         // Output to the serial port
//#define SDLOG         // output sensor data to SD Card
#define SCREEN        // output sensor data to screen
#define MQTTLOG        // Output to MQTT broker defined in secrets.h
//#define RJ45            // use Ethernet
#define WIFI          // use WiFi (credentials in secrets.h)
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
  #else
    #include <WiFi.h>
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
  #define ATTEMPT_LIMIT 3
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
  #include <Adafruit_GFX.h>

  #include <Adafruit_ST7789.h>
  Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET);

  //#include <Adafruit_SH110X.h>
  //Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

  //#include <Adafruit_SSD1306.h>
  //Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
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
    Serial.println("Indoor Air Quality started");
  #endif

  // sensor check
  if (! aht.begin())
  {
    debugMessage("FATAL ERROR: AHT sensor not detected");
    screenMessage("ERROR 1: Sensor");
    stopApp();
  }
  debugMessage("AHTX0 sensor ready");

  #ifdef SCREEN
    // Color definitions
    #define BLACK    0x0000
    #define BLUE     0x001F
    #define RED      0xF800
    #define GREEN    0x07E0
    #define CYAN     0x07FF
    #define MAGENTA  0xF81F
    #define YELLOW   0xFFE0 
    #define WHITE    0xFFFF

    // Initialize ST7789 screen
    display.init(240, 240);
    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on
    display.fillScreen(BLACK);

    // // Initialize SH110X screen
    // display.setRotation(1); // SH110X only?

    // // Initialize SSD1306 screen

    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.println("Waiting for first sensor data");
  #endif

  #ifdef SDLOG
    // initialize SD card
    if (!SD.begin(SDPIN)) 
    {
      debugMessage("SD card failed or not present");
      screenMessage("SD card failed or not present");
      stopApp();
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
      screenMessage("Couldn't create log file on SD card");
      stopApp();
    }
  
    debugMessage("Logging to: " + filename);

    // Log output headers
    logfile.println("time,room,humidity,temp");    
  #endif

  #ifdef WIFI
    uint8_t tries = 1;
    #define MAX_TRIES 5

    // set hostname has to come before WiFi.begin
    WiFi.hostname(MQTT_CLIENT_ID);
    // WiFi.setHostname(MQTT_CLIENT_ID); //for WiFiNINA
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) 
    {
      // Error handler - WiFi does not initially connect
      debugMessage(String("Connection attempt ") + tries + " of " + MAX_TRIES + " to " + WIFI_SSID + " in " + (tries*10) + " seconds");
      // use of delay OK as this is initialization code
      delay(tries*10000);

      // FATAL ERROR 03 - WiFi doesn't connect
      if (tries == MAX_TRIES)
      {
        debugMessage(String("Can not connect to WFii after ") + MAX_TRIES + " attempts");
        screenMessage(String("Can not connect to WiFi after ") + MAX_TRIES + " attempts");
        #ifndef SDLOG
          stopApp();
        #endif
      }
      tries++;
    }
    debugMessage("WiFi IP address is: " + ip2CharArray(WiFi.localIP()));
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
      {
        // generic error
        debugMessage("Failed to configure Ethernet using DHCP");
        screenMessage("Failed to configure Ethernet using DHCP");
        stopApp();
      }
    }
    // #ifdef DEBUG
    //   Serial.print("Ethernet address is:");
    //   Serial.println(Ethernet.localIP);
    // #endif 
    //String ipText = DisplayAddress(Ethernet.localIP);
    //debugMessage(String("Ethernet IP address is: ") + ipText);
    debugMessage(String("Ethernet IP address is: ") + ip2CharArray(Ethernet.localIP()));

  #endif
    
  #ifdef NTP
    // Get time from NTP
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);

    // wait until the time is set by the sync provider
    while(timeStatus()== timeNotSet);
    debugMessage("NTP time is " + zuluDateTimeString());
  #endif
}

void loop() 
{
  #ifdef DEBUG
    // display heartbeat every 20 seconds between sensor reads; depending on client might display >1 message per interval
    if (((millis() - syncTime) % 20000) == 0)
    {
      debugMessage(String(((millis()-syncTime)/1000)) + " seconds elapsed before next read at " + (LOG_INTERVAL/1000) + " seconds");
    }
  #endif

  // update display and determine if it is time to read/process sensors
  if ((millis() - syncTime) < LOG_INTERVAL) return;

  syncTime = millis();
  #ifdef MQTTLOG
  {
    MQTT_connect();
  }
  #endif

  // populate temp and humidity objects with fresh data
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  #ifdef SCREEN
    display.fillScreen(BLACK);
    display.setCursor(0,0);
    display.setTextColor(YELLOW);
    display.print("Temperature: ");
    display.setTextColor(RED);
    display.println((temp.temperature*9/5+32));  // will get truncated to 2 decimal places
    display.setTextColor(YELLOW);
    display.print("Humidity: ");
    display.setTextColor(RED);
    display.println(humidity.relative_humidity);
  #endif

  #ifdef MQTTLOG
    if (!tempPub.publish((temp.temperature*9/5+32)))
    {
      debugMessage(String(MQTT_PUB_TOPIC1) + " MQTT publish failed at " + zuluDateTimeString());
    }
    else 
    {
      debugMessage(String("Published ") + (temp.temperature*9/5+32) + " to MQTT endpoint " + MQTT_PUB_TOPIC1 + " at " + zuluDateTimeString());
    }
    if (!humidityPub.publish(humidity.relative_humidity))
    {
      debugMessage(String(MQTT_PUB_TOPIC2) + " MQTT publish failed");
    }
    else 
    {
      debugMessage(String("Published ") + (humidity.relative_humidity) + " to MQTT endpoint " + MQTT_PUB_TOPIC2);
    }
    if (!roomPub.publish(MQTT_CLIENT_ID))
    {
      debugMessage(String(MQTT_PUB_TOPIC4) + " MQTT publish failed");
    }
    else 
    {
      debugMessage(String("Published ") + MQTT_CLIENT_ID + " to MQTT endpoint " + MQTT_PUB_TOPIC4);
    }
  #endif

  String logString = zuluDateTimeString() + "," + MQTT_CLIENT_ID + "," + humidity.relative_humidity + "," + String(temp.temperature*9/5+32);

  #ifdef SDLOG
    logfile.println(logString);
    logfile.flush();
    debugMessage("Log data written to SD card");
  #endif

  #ifndef MQTTLOG   // reduces log clutter
    debugMessage(logString);
  #endif
}

#ifdef NTP
  time_t getNtpTime()
  {
    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    //debugMessage("Transmitting NTP Request");
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
    debugMessage("No NTP response");
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
    int8_t mqttErr;
    int8_t tries = 1;

    // exit if already connected
    if (mqtt.connected()) 
    {
      return;
    }

    // Useful if device is always on and log gaps between reads
    // if (WiFi.status() != WL_CONNECTED)
    // {
    //   return;
    // }

    while ((mqttErr = mqtt.connect() != 0) && (tries<=ATTEMPT_LIMIT))
    {
      // generic MQTT error
      // debugMessage(mqtt.connectErrorString(mqttErr));

      // Adafruit IO connect errors
      switch (mqttErr) 
      {
        case 1: debugMessage("Wrong protocol"); break;
        case 2: debugMessage("ID rejected"); break;
        case 3: debugMessage("Server unavailable"); break;
        case 4: debugMessage("Incorrect user or password"); break;
        case 5: debugMessage("Not authorized"); break;
        case 6: debugMessage("Failed to subscribe"); break;
        default: debugMessage("GENERIC - Connection failed"); break;
      }
      debugMessage(String(MQTT_BROKER) + " connect attempt " + tries + " of " + ATTEMPT_LIMIT + " happens in " + (tries*10) + " seconds");
      mqtt.disconnect();
      delay(tries*10000);
      tries++;

      if (tries == ATTEMPT_LIMIT) 
      {
        debugMessage(String("Connection failed to MQTT broker ") + MQTT_BROKER);
        screenMessage("ERROR 10: MQTT Broker connection failed");
      }
    }
    if (tries<ATTEMPT_LIMIT)
    {
      debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
    }
  }
#endif

void debugMessage(String messageText)
{
  #ifdef DEBUG
    Serial.println(messageText);
  #endif
}

void screenMessage(String messageText)
{
  #ifdef SCREEN
    display.fillScreen(BLACK);
    display.println(messageText);
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

String ip2CharArray(IPAddress ip) 
{
  static char a[16];
  sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return a;
}