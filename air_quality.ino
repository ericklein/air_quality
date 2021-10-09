/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
#define DEBUG         // Output to the serial port
#define SCREEN        // output sensor data to screen
//#define MQTTLOG        // Output to MQTT broker defined in secrets.h
//#define RJ45            // use Ethernet
//#define WIFI          // use WiFi (credentials in secrets.h)
//#define NTP         // query network time server for logging
#define BATTERY

// Gloval variables
unsigned long syncTime = 0;        // holds millis() [milliseconds] for timing functions 

// AHTX0 (temperature and humidity)
// #include <Adafruit_AHTX0.h>
// Adafruit_AHTX0 th;

// Si7021 (temperature and humidity)
#include "Adafruit_Si7021.h"
Adafruit_Si7021 th = Adafruit_Si7021();

// Battery voltage sensor
#include "Adafruit_LC709203F.h"
Adafruit_LC709203F lc;

#ifdef DEBUG
  // microsecond delay for deep sleep
  //#define LOG_INTERVAL 240000000  // debug interval; 240 seconds
  #define LOG_INTERVAL 60000000  // debug interval; 60 seconds 
#else
  #define LOG_INTERVAL 1800000000 // production interval; 1800 seconds
#endif

// MQTT credentials and network IDs
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
  Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC1,MQTT_QOS_1);
  Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC2, MQTT_QOS_1);
  Adafruit_MQTT_Publish roomPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC4, MQTT_QOS_1);
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

  // magtag support
  #include "Adafruit_ThinkInk.h"
  #include <Fonts/FreeSans9pt7b.h>
  #define EPD_DC      7 // can be any pin, but required!
  #define EPD_RESET   6  // can set to -1 and share with chip Reset (can't deep sleep)
  #define EPD_CS      8  // can be any pin, but required!
  #define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
  #define EPD_BUSY    5  // can set to -1 to not use a pin (will wait a fixed delay)
  ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
  // colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK

  // LED, OLED color definitions
  // #define BLACK    0x0000
  // #define BLUE     0x001F
  // #define RED      0xF800
  // #define GREEN    0x07E0
  // #define CYAN     0x07FF
  // #define MAGENTA  0xF81F
  // #define YELLOW   0xFFE0 
  // #define WHITE    0xFFFF

  // #include <Adafruit_ST7789.h>
  // Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET);

  //#include <Adafruit_SH110X.h>
  //Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

  //#include <Adafruit_SSD1306.h>
  //Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
#endif

void setup() 
{
  // used for fatal error messaging
  pinMode(LED_BUILTIN, OUTPUT);

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial)
    {
      // wait for serial port
    }
    Serial.println("Indoor Air Quality started");
  #endif

  // temp/humidity sensor check
  if (!th.begin())
  {
    debugMessage("FATAL ERROR: temp/humidity sensor not detected");
    screenUpdate(0,0,"temp/humidity sensor not detected");
    stopApp();
  }
  debugMessage("temp/humidity sensor ready");

  #ifdef BATTERY
    if (!lc.begin())
    {
      debugMessage("Battery and voltage monitor not detected");
      stopApp();
    }
    debugMessage("Battery voltage monitor ready");
    // required for accurate battert temp reading
    lc.setThermistorB(3950);
    lc.setPackSize(LC709203F_APA_2000MAH);
    // do not know why I need this
    lc.setAlarmVoltage(3.8);
    // LC709203F_APA_100MAH = 0x08,
    // LC709203F_APA_200MAH = 0x0B,
    // LC709203F_APA_500MAH = 0x10,
    // LC709203F_APA_1000MAH = 0x19,
    // LC709203F_APA_2000MAH = 0x2D,
    // LC709203F_APA_3000MAH = 0x36,
  #endif

  #ifdef SCREEN
    // Initialize ST7789 screen
    // display.init(240, 240);
    // pinMode(TFT_BACKLIGHT, OUTPUT);
    // digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on
    // display.fillScreen(BLACK);

    // // Initialize SH110X screen
    // display.setRotation(1); // SH110X only?

    // // Initialize SSD1306 screen

    // Initalize e-ink screen
    display.begin(THINKINK_GRAYSCALE4);
    // display.begin(THINKINK_MONO);
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
        screenUpdate(0,0,String("Can not connect to WiFi after ") + MAX_TRIES + " attempts");
        //stopApp();
        deepSleep();
      }
      tries++;
    }
    debugMessage("WiFi IP address is: " + ip2CharArray(WiFi.localIP()));
    debugMessage("RSSI is: " + String(WiFi.RSSI()) + " dBm");
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
      {
        debugMessage("Ethernet hardware not found");
        screenUpdate(0,0,"Ethernet hardware not found");
      }
      else if (Ethernet.linkStatus() == LinkOFF) 
      {
        debugMessage("Ethernet cable not connected");
        screenUpdate(0,0,"Ethernet cable not connected");
      }
      else
      {
        // generic error
        debugMessage("Failed to configure Ethernet");
        screenUpdate(0,0,"Failed to configure Ethernet");
      }
      stopApp();
    }
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

  // One time run of main logic before sleep
  // populate temp and humidity objects with fresh data
  // AHTX0
  // sensors_event_t humidity, temp;
  // th.getEvent(&humidity, &temp);

  // intermediate variable helps moving between sensor APIs easier in code
  // AHTX0
  // float temperature = (temp.temperature*1.8)+32;
  // float humidity = humidity.relative_humidity;

  // SiH7021
  float temperature = (th.readTemperature()*1.8)+32;
  float humidity = th.readHumidity();
  
  #ifdef MQTTLOG
    MQTT_connect();
    if ((!tempPub.publish(temperature)) || (!humidityPub.publish(humidity)) || (!roomPub.publish(MQTT_CLIENT_ID)))
    {
      debugMessage("Part/all of MQTT publish failed at " + zuluDateTimeString());
      screenUpdate(temperature, humidity, "MQTT publish failed at " + zuluDateTimeString());
    }
    else 
   {
      debugMessage("Published to MQTT: " + zuluDateTimeString() + " , " + MQTT_CLIENT_ID + " , " + temperature + " , " + humidity);
      screenUpdate(temperature, humidity,(String(MQTT_CLIENT_ID) + " publish at " + zuluDateTimeString()));
    }
    mqtt.disconnect();
  #else
  {
    debugMessage(zuluDateTimeString() + "->" + temperature + "F , " + humidity + "%");
    screenUpdate(temperature, humidity,"");
  }
  #endif

  // // done, put device into proper state
  // #ifdef DEBUG
  //   // keep the device alive for a window before sleeping for the same interval
  //   debugMessage("You have " + (LOG_INTERVAL/100000) + "seconds to reflash device"); 
  //   delay(LOG_INTERVAL);
  // else
  // // deep sleep
  //  #ifdef WIFI
  //     client.stop();
  //   #endif
  //   deepSleep();
  // #endif 

  // deep sleep
  #ifdef WIFI
    client.stop();
  #endif
  deepSleep();
}

void loop() 
{
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

void screenUpdate(float temperature, float humidity, String messageText)
{
  #ifdef SCREEN
    display.clearBuffer();
    screenBorders();
    screenBatteryStatus();
    
    // Information display
    // Labels
    display.setTextColor(EPD_BLACK);
    display.setCursor(((display.width()/4)-10),((display.height()*1/8)-11));
    display.print("Here");
    display.setCursor(((display.width()*3/4-12)),((display.height()*1/8)-11));
    display.print("Outside");

    // Temperature data
    display.setFont(&FreeSans9pt7b);
    display.setTextSize(1);
    display.setCursor(5,((display.height()*3/8)-10));
    if (temperature>0) display.print(String("Temp ") + temperature + "F");
    display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
    // Stub for outside temperature
    display.print("Temp XXX.xxF");

    // Humidity data
    display.setCursor(5,((display.height()*5/8)-10));
    if (temperature>0) display.print(String("Humidity ") + humidity + "%");
    display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
    // Stub for outside humidity
    display.print("Humidity YY.yy%");

    // CO2 data
    display.setCursor(5,((display.height()*7/8)-10));
    // Stub for outside C02
    display.print("CO2 ZZZppm");    
    display.setCursor(((display.width()/2)+5),((display.height()*7/8)-10));
    // Stub for outside AQM
    display.print("AQM ZZZppm");    

    // Mesages
    display.setFont();  // resets to system default monospace font
    display.setCursor(5,(display.height()-8));
    display.print(messageText);

    display.display();
  #endif
}

void screenBorders()
{
  #ifdef SCREEN
    // ThinkInk 2.9" epd is 296x128 pixels
    // isolating this function for partial screen redraws
    // label border
    display.drawLine(0,(display.height()/8),display.width(),(display.height()/8),EPD_GRAY);
    // temperature area
    display.drawLine(0,(display.height()*3/8),display.width(),(display.height()*3/8),EPD_GRAY);
    // humidity area
    display.drawLine(0,(display.height()*5/8),display.width(),(display.height()*5/8), EPD_GRAY);
    // CO2 area
    display.drawLine(0,(display.height()*7/8),display.width(),(display.height()*7/8), EPD_GRAY);
    // splitting sensor vs. outside values
    display.drawLine((display.width()/2),0,(display.width()/2),(display.height()*7/8),EPD_GRAY);
  #endif
}

void screenBatteryStatus()
{
  #if defined(SCREEN) && defined(BATTERY) 
    // render battery percentage to screen
    // 28 pixels wide, 7 pixels/25% battery life? 
    display.drawRect((display.width()-33),((display.height()*7/8)+3),(display.width()-5),(display.height()-3),EPD_BLACK);
    //calculate fill
    display.fillRect((display.width()-32),((display.height()*7/8)+4),((display.width()-32)+(int(lc.cellPercent()*28))),(display.height()-2),EPD_GRAY);
  #endif
  debugMessage("Battery: " + String(lc.cellVoltage()) + " v");
  debugMessage("Battery: " + String(int(lc.cellPercent())) + " %");
  debugMessage("Battery: " + String(lc.getCellTemperature()) + " F");
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

#if defined(WIFI) || defined(RJ45)
  String ip2CharArray(IPAddress ip) 
    {
      static char a[16];
      sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      return a;
    }
#endif

void deepSleep()
{
  debugMessage(String("Going to sleep for ") + (LOG_INTERVAL/100000) + " seconds");
  #ifdef SCREEN
    display.powerDown();
    digitalWrite(EPD_RESET, LOW); // hardware power down mode
  #endif
  esp_sleep_enable_timer_wakeup(LOG_INTERVAL);
  esp_deep_sleep_start();
}