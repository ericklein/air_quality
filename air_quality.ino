/*
  Project Name:   air_quality
  Developer:      Eric Klein Jr. (temp2@ericklein.com)
  Description:    Measures and logs temp, humidity, and CO2 levels

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
//#define CO2           // Output CO2 data
//#define DEBUG         // Output to the serial port
//#define SDLOG         // output sensor data to SD Card
//#define SCREEN          // output sensor data to screen
#define MQTTLOG        // Output to MQTT broker defined in secrets.h
#define RJ45            // use Ethernet
//#define WIFI          // use WiFi (credentials in secrets.h)
//#if defined(SDLOG) || defined(DEBUG)
#define NTP         // query network time server for logging
//#endif

// Gloval variables
unsigned long syncTime = 0;        // holds millis() [milliseconds] for timing functions 

// DHT (digital humidity and temperature) sensor
#include "DHT.h"
//#define DHTPIN  2      // Digital pin connected to DHT sensor (Huzzah for master_bedroom)
#define DHTPIN  11      // Digital pin connected to DHT sensor (M0 for lab)
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

#ifdef CO2
  // SGP30 (eCO2) sensor
  #include <Wire.h>
  #include "Adafruit_SGP30.h"
  Adafruit_SGP30 sgp;
#endif

#ifdef DEBUG
  // millisecond delay between sensor reads
  //#define LOG_INTERVAL 60000  // standard test interval
  #define LOG_INTERVAL 600000   // long term test interval
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
  Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC3);
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
    Serial.println("Indoor Air Quality started");
  #endif

  dht.begin();

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

    // msg displayed until error or first reading is displayed
    display.setCursor(0,0);
    display.println("Waiting for");
    //display.setCursor(0,8*2);  // associated with text size
    display.println("first read");
    display.display();
  #endif

  #ifdef CO2
    if (!sgp.begin())
    {
      #ifdef DEBUG
        Serial.println("SGP30 sensor not found, no CO2 readings will be sent");
      #endif
      #ifdef SCREEN
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0,0);
        display.print("ERR 05");
        display.setCursor(0,8*2);
        display.print("SGP30");
        display.display(); 
      #endif
    //while (1);
    }
    // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
    //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!
  #endif

  #ifdef DEBUG
    Serial.println("DHT22 sensor ready");
    #ifdef CO2
        Serial.println("SGP30 sensor ready");  
    #endif
  #endif

  #ifdef SDLOG
    // initialize SD card
    if (!SD.begin(SDPIN)) 
    {
      #ifdef DEBUG
        Serial.println("Card failed, or not present");
      #endif
      while (1);
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
      #ifdef DEBUG
        Serial.println("Couldn't create log file");
      #endif
      while (1);
    }
  
    #ifdef DEBUG
      Serial.print("Logging to: ");
      Serial.println(filename);
    #endif

    // Log output headers
      logfile.println("time,humidity,temp,heat_index,eCO2");    
  #endif

  #ifdef WIFI
    uint8_t tries = 1;
    #define MAXTRIES 11

    // Connect to WiFi access point
    #ifdef DEBUG
      Serial.print("Connecting to WiFI AP ");
      Serial.println(WIFI_SSID);
    #endif

    // set hostname has to come before WiFi.begin
    WiFi.hostname(MQTT_CLIENT_ID);
    // WiFi.setHostname(MQTT_CLIENT_ID); //for WiFiNINA
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) 
    {
      // Error handler - WiFi does not initially connect
      #ifdef DEBUG
        Serial.print("WiFi AP connect attempt ");
        Serial.print(tries);
        Serial.print("of");
        Serial.print(MAXTRIES);
        Serial.print(" in ");
        Serial.print(tries*10);
        Serial.println(" seconds");
      #endif
      // use of delay OK as this is initialization code
      delay(tries*10000);
      tries++;

      // FATAL ERROR 03 - WiFi doesn't connect
      if (tries == MAXTRIES)
      {
        #ifdef DEBUG
          Serial.print("FATAL ERROR 03; can not connect to WiFi after ");
          Serial.print(MAXTRIES);
          Serial.println(" attempts");
        #endif
        #ifdef SCREEN
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0,0);
          display.print("ERR 03");
          display.setCursor(0,8*2);
          display.print("WiFi")
          display.display(); 
        #endif
        #ifndef SDLOG
          while (1)
          {
            // endless loop communicating fatal error 03
            digitalWrite(LED_BUILTIN, HIGH);
            delay(3000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(3000);
          }
        #endif
      }
    }
    #ifdef DEBUG
      Serial.println();  // finishes the status dots print
      Serial.print("WiFi IP address is: ");
      Serial.println(WiFi.localIP());
    #endif
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
      // FATAL ERROR 02 - Ethernet doesn't connect
      
      // generic error
      #ifdef DEBUG
        Serial.println("FATAL ERROR 02; Failed to configure Ethernet using DHCP");
      #endif

      // identified errors
      if (Ethernet.hardwareStatus() == EthernetNoHardware)
      {
        #ifdef DEBUG
          Serial.println("FATAL ERROR 02; Ethernet hardware not found");
        #endif
      } 
      else if (Ethernet.linkStatus() == LinkOFF) 
      {
        #ifdef DEBUG
          Serial.println("FATAL ERROR 02; Ethernet cable is not connected");
        #endif
      }
      #ifdef SCREEN
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0,0);
        display.print("ERR 02");
        display.setCursor(0,8*2);
        display.print("Ethernet")
        display.display(); 
      #endif
      #ifndef SDLOG
        while (1)
        {
          // endless loop communicating fatal error 02
          digitalWrite(LED_BUILTIN, HIGH);
          delay(2000);
          digitalWrite(LED_BUILTIN, LOW);
          delay(2000);
        }
      #endif
    }
    #ifdef DEBUG
      Serial.print("Ethernet IP address is: ");
      Serial.println(Ethernet.localIP());
    #endif
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

  // note: reading temperature or humidity takes ~ 250ms
  float humidity; 
  if (isnan(dht.readHumidity()))
  {
    #ifdef DEBUG
      Serial.println("Failed to read humidity from DHT sensor");
    #endif
    humidity = -1;
  }
  else
  {
    humidity = dht.readHumidity();
  }

  // Read temperature as Fahrenheit via true parameter
  float temperature_fahr;
  if (isnan(dht.readTemperature(true)))
  {
    #ifdef DEBUG
      Serial.println("Failed to read temperature from DHT sensor");
    #endif
    temperature_fahr = -1;
  }
  else
  {
    temperature_fahr = dht.readTemperature(true);
    // float t = dht.readTemperature(); // temperature in celsius
  }

  #ifdef CO2
    // set the absolute humidity to enable the humditiy compensation for the SGP30 air quality signals
    sgp.setHumidity(getAbsoluteHumidity(temperature_fahr, humidity));

    //float tvocReading;
    float ecO2Reading;
 
    if (! sgp.IAQmeasure()) 
    {
      #ifdef DEBUG
        Serial.println("SGP30 Measurement failed");
      #endif
      //tvocReading = -1;
      ecO2Reading = -1;  
    }
    else
    {
      // tvocReading = sgp.TVOC;
      ecO2Reading = sgp.eCO2; 
    } 
  #endif

  #ifdef SCREEN
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.print("Tmp:");
    display.println(temperature_fahr);  // will get truncated to 2 decimal places
    //display.setCursor(0,8*2);           // associated with text size
    display.print("Hum:");
    display.println(humidity);
    display.display(); 
  #endif

  #ifdef MQTTLOG
    #ifdef DEBUG
      Serial.print("temperature via MQTT publish to '");
      Serial.print(MQTT_PUB_TOPIC1);
    #endif
    if (!tempPub.publish(temperature_fahr))
    {
      #ifdef DEBUG
        Serial.println("' failed");
      #endif
      #ifdef SCREEN
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0,0);
          display.print("ERR 04");
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
    if (!humidityPub.publish(humidity))
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
    #ifdef CO2
      #ifdef DEBUG
        Serial.print("CO2 via MQTT publish to '");
        Serial.print(MQTT_PUB_TOPIC3);
      #endif
      if (!co2Pub.publish(ecO2Reading))
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
    logString += humidity;
    logString += ",";
    logString += temperature_fahr;
    #ifdef CO2
      logString += ",";
      logString += ecO2Reading;
    #endif
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

uint32_t getAbsoluteHumidity(float temperature, float humidity) 
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
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
          display.print("ERR 01");
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