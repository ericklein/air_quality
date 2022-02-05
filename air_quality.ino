/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// hardware and internet service configuration parameters
#include "config.h"

// Gloval variables
#ifndef ONE_TIME
  unsigned long syncTime = 0;   // stores millis() for timing functions 
#endif

// internal and outside environmental conditions
typedef struct
{
  uint16_t internalTemp;
  uint16_t internalHumidity;
  uint16_t internalCO2;
  uint16_t extTemperature;
  uint16_t extHumidity;
  uint16_t extAQM;
} envData;

#ifdef WEATHER 
  #include <HTTPClient.h> 
  #include "ArduinoJson.h"
#endif

#ifdef CO2_SENSOR
  // temp, humidity, and CO2
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x envSensor;
#else
  // temp, humidity
  //AHTX0
  //#include <Adafruit_AHTX0.h>
  //Adafruit_AHTX0 envSensor;

  // Si7021
  // #include "Adafruit_Si7021.h"
  // Adafruit_Si7021 envSensor = Adafruit_Si7021();

  // BME680
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  #define SEALEVELPRESSURE_HPA (1013.25)

  Adafruit_BME280 envSensor;
#endif

// Battery voltage sensor
#ifdef BATTERY
  #include "Adafruit_LC709203F.h"
  Adafruit_LC709203F lc;
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
  Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);
  Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC1,MQTT_QOS_1);
  Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC2, MQTT_QOS_1);
  Adafruit_MQTT_Publish errMsgPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC5, MQTT_QOS_1);
  #ifdef CO2_SENSOR
    Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC3, MQTT_QOS_1);
  #endif
  //Adafruit_MQTT_Subscribe weatherSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_WEATHER_TOPIC);
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
    debugMessage("Air Quality started");
  #endif

  // Handle environment sensor
  #ifdef CO2_SENSOR
    uint16_t error;

    Wire.begin();
    envSensor.begin(Wire);

    // stop potentially previously started measurement
    error = envSensor.stopPeriodicMeasurement();
    if (error) 
    {
      debugMessage("Error trying to execute stopPeriodicMeasurement(): ");
    }
    // Start Measurement
    error = envSensor.startPeriodicMeasurement();
    if (error)
    {
      debugMessage("Error trying to execute startPeriodicMeasurement(): ");
    }
    delay(5000);
    debugMessage("temp/humidity/CO2 sensor ready");
  #else
    // temp/humidity sensor check
    if (!envSensor.begin())
    {
      debugMessage("FATAL ERROR: temp/humidity sensor not detected");
      screenUpdate(0,0,0,0,0,0,"temp/humidity sensor not detected");
      stopApp();
    }
    debugMessage("temp/humidity sensor ready");
  #endif

  #ifdef BATTERY
    if (!lc.begin())
    {
      debugMessage("Battery and voltage monitor not detected");
      screenUpdate(0,0,0,0,0,0,"battery sensor not detected");
      stopApp();
    }
    debugMessage("Battery voltage monitor ready");
    lc.setPackSize(BATTERYSIZE);
    // optional thermistor added on board for accurate battery temp reading
    //lc.setThermistorB(3950);

    //lc.setAlarmVoltage(3.8);
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
    WiFi.hostname(CLIENT_ID);
    // WiFi.setHostname(CLIENT_ID); //for WiFiNINA
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
        screenUpdate(0,0,0,0,0,0, String("Can not connect to WiFi after ") + MAX_TRIES + " attempts");
        #ifdef ONE_TIME
          deepSleep();
        #else
          stopApp();
        #endif
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
        screenUpdate(0,0,0,0,0,0,"Ethernet hardware not found");
      }
      else if (Ethernet.linkStatus() == LinkOFF) 
      {
        debugMessage("Ethernet cable not connected");
        screenUpdate(0,0,0,0,0,0,"Ethernet cable not connected");
      }
      else
      {
        // generic error
        debugMessage("Failed to configure Ethernet");
        screenUpdate(0,0,0,0,0,0, "Failed to configure Ethernet");
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

  // Handle one time sensor read and information display/storage
  #ifdef ONE_TIME
    actionSequence();
    // deep sleep device, which restarts code when repowered
    deepSleep();
  #endif
}

void loop() 
{
  #ifndef ONE_TIME
    #ifdef DEBUG
      // display heartbeat every 20 seconds between sensor reads; depending on client might display >1 message per interval
      if (((millis() - syncTime) % 20000) == 0)
      {
        debugMessage(String(((millis()-syncTime)/1000)) + " seconds elapsed before next read at " + (LOG_INTERVAL*60) + " seconds");
      }
    #endif

    // update display and determine if it is time to read/process sensors
    if ((millis() - syncTime) < LOG_INTERVAL*LOG_INTERVAL_MS_MODIFIER) return;

    syncTime = millis();
    actionSequence();
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

#ifdef ONE_TIME
  void deepSleep()
  {
    // I don't think these needed as I never enable them
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // pinMode(SPEAKER_SHUTDOWN, OUTPUT);
    // digitalWrite(SPEAKER_SHUTDOWN, LOW); // off
    // digitalWrite(NEOPIXEL_POWER, HIGH); // off

    debugMessage(String("Going to sleep for ") + LOG_INTERVAL + " minutes");
    #ifdef SCREEN
      display.powerDown();
      digitalWrite(EPD_RESET, LOW); // hardware power down mode
    #endif
    digitalWrite(LED_BUILTIN, LOW);
    #ifdef WIFI
      client.stop();
      // esp_wifi_stop();
    #endif
    #ifdef CO2_SENSOR
      envSensor.stopPeriodicMeasurement();
    #endif
    esp_sleep_enable_timer_wakeup(LOG_INTERVAL*LOG_INTERVAL_US_MODIFIER);
    esp_deep_sleep_start();
  }
#endif

envData readEnvironment()
{
  envData sensorData;
  
  //sensor (room) measurement

  #ifdef CO2_SENSOR
    uint8_t error;
    float sensorTemp;
    float sensorHumidity;

    error = envSensor.readMeasurement(sensorData.internalCO2, sensorTemp, sensorHumidity);
    if (error) 
    {
      debugMessage("Error trying to read temp/humidity/CO2 sensor");
      sensorData.internalCO2 = 10000;
      sensorData.internalTemp = 10000;
      sensorData.internalHumidity = 10000;
    }
    // convert C to F for temp, round to int
    sensorData.internalTemp = (int) sensorTemp*1.8+32+0.5;
    sensorData.internalHumidity = (int) sensorHumidity + 0.5;
  #else
    // AHTX0
    // sensors_event_t sensorHumidity, sensorTemp;
    // envSensor.getEvent(&sensorHumidity, &sensorTemp);
    // // no error handling?!
    // sensorData.internalTemp = (int) sensorTemp.temperature+.5;
    // sensorData.internalHumidity = (int) sensorHumidity.relative_humidity+.5;
    // sensorData.internalCO2 = 0;

    // bme280, SiH7021
    sensorData.internalTemp = (int) (envSensor.readTemperature()*1.8)+32;
    sensorData.internalHumidity = (int) envSensor.readHumidity();
    sensorData.internalCO2 = 0;

    // SiH7021
    // no error handling?
    // sensorData.internalTemp = (int) (envSensor.readTemperature()*1.8)+32;
    // sensorData.internalHumidity = (int) envSensor.readHumidity();
    // sensorData.internalCO2 = 0;
  #endif

  // external conditions
  #ifdef WEATHER
    String jsonBuffer;

    // Get local temp and humidity
    String serverPath = String(OWM_SERVER) + OWM_WEATHER_PATH + OWM_LAT_LONG + "&units=imperial" + "&APPID=" + OWM_KEY;
      
    jsonBuffer = httpGETRequest(serverPath.c_str());
    debugMessage(jsonBuffer);
    
    StaticJsonDocument<1024> doc;

    DeserializationError httpError = deserializeJson(doc, jsonBuffer);

    if (httpError)
    {
      debugMessage("Unable to parse weather JSON object");
      Serial.println(httpError.c_str());
      sensorData.extTemperature = 0;
      sensorData.extHumidity = 0;
      return sensorData;
    }

    // double coord_lon = doc["coord"]["lon"]; // -122.2221
    // float coord_lat = doc["coord"]["lat"]; // 47.5706

    //JsonObject weather_0 = doc["weather"][0];
    // int weather_0_id = weather_0["id"]; // 804
    // const char* weather_0_main = weather_0["main"]; // "Clouds"
    // const char* weather_0_description = weather_0["description"]; // "overcast clouds"
    // const char* weather_0_icon = weather_0["icon"]; // "04n"

    // const char* base = doc["base"]; // "stations"

    JsonObject main = doc["main"];
    sensorData.extTemperature = main["temp"];
    // float main_feels_like = main["feels_like"]; // 21.31
    // float main_temp_min = main["temp_min"]; // 18.64
    // float main_temp_max = main["temp_max"]; // 23.79
    // int main_pressure = main["pressure"]; // 1010
    sensorData.extHumidity = main["humidity"]; // 81

    // int visibility = doc["visibility"]; // 10000

    // JsonObject wind = doc["wind"];
    // float wind_speed = wind["speed"]; // 1.99
    // int wind_deg = wind["deg"]; // 150
    // float wind_gust = wind["gust"]; // 5.99

    // int clouds_all = doc["clouds"]["all"]; // 90

    // long dt = doc["dt"]; // 1640657831

    // JsonObject sys = doc["sys"];
    // int sys_type = sys["type"]; // 2
    // long sys_id = sys["id"]; // 2030051
    // const char* sys_country = sys["country"]; // "US"
    // long sys_sunrise = sys["sunrise"]; // 1640620588
    // long sys_sunset = sys["sunset"]; // 1640651017

    // int timezone = doc["timezone"]; // -28800
    // long id = doc["id"]; // 5803139
    // const char* name = doc["name"]; // "Mercer Island"
    // int cod = doc["cod"]; // 200

    debugMessage(String("Ext temp is ")+sensorData.extTemperature+" F");
    debugMessage(String("Ext humidity is ")+sensorData.extHumidity+" %");

    // Get local AQM data
    serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;
      
    jsonBuffer = httpGETRequest(serverPath.c_str());
    debugMessage(jsonBuffer);
    
    StaticJsonDocument<384> doc1;

    httpError = deserializeJson(doc1, jsonBuffer);

    if (httpError)
    {
      debugMessage("Unable to parse air quality JSON object");
      Serial.println(httpError.c_str());
      sensorData.extAQM = 0;
      return sensorData;
    }

    // double coord_lon = doc1["coord"]["lon"]; // -122.2221
    // float coord_lat = doc1["coord"]["lat"]; // 47.5707

    JsonObject list_0 = doc1["list"][0];

    sensorData.extAQM = list_0["main"]["aqi"]; // 2

    // JsonObject list_0_components = list_0["components"];
    // float list_0_components_co = list_0_components["co"]; // 453.95
    // float list_0_components_no = list_0_components["no"]; // 0.47
    // float list_0_components_no2 = list_0_components["no2"]; // 52.09
    // float list_0_components_o3 = list_0_components["o3"]; // 17.17
    // float list_0_components_so2 = list_0_components["so2"]; // 7.51
    // float list_0_components_pm2_5 = list_0_components["pm2_5"]; // 8.04
    // float list_0_components_pm10 = list_0_components["pm10"]; // 9.96
    // float list_0_components_nh3 = list_0_components["nh3"]; // 0.86

    // long list_0_dt = list_0["dt"]; // 1641434400

    debugMessage(String("Ext AQM is ")+sensorData.extAQM);
  #endif

  return sensorData;
}

void mqttBatteryAlert(int percent)
{
  #ifdef MQTTLOG
    // msg to backend if battery is running low
    if (percent<20)
    {
      String errMessage = String(CLIENT_ID) + " battery at " + percent + " percent at " + zuluDateTimeString();  
      MQTT_connect();


    int charArrayLength = errMessage.length()+1;
    char textInChars[charArrayLength];
    errMessage.toCharArray(textInChars, charArrayLength);

      if (!errMsgPub.publish(textInChars))
      //if (!errMsgPub.publish(errMessage))
      {
        debugMessage("MQTT low battery publish failed at " + zuluDateTimeString());
      }
      else 
      {
        debugMessage("MQTT publish: " + errMessage);
      }
      mqtt.disconnect();
    }
  #endif
}

int mqttSensorUpdate(float temperature, float humidity, uint16_t co2)
{
  #ifdef MQTTLOG
    MQTT_connect();
    #ifdef CO2_SENSOR
      if ((!tempPub.publish(temperature)) || (!humidityPub.publish(humidity)) || (!co2Pub.publish(co2)))
    #else
      if ((!tempPub.publish(temperature)) || (!humidityPub.publish(humidity)))
    #endif
    {
      debugMessage("Part/all of MQTT publish failed at " + zuluDateTimeString());
      mqtt.disconnect();
      return 1;
    }
    else 
    {
      #ifdef CO2_SENSOR
        debugMessage("Published to MQTT: " + zuluDateTimeString() + " , " + CLIENT_ID + " , " + temperature + " , " + humidity + " , " + co2);
      #else
        debugMessage("Published to MQTT: " + zuluDateTimeString() + " , " + CLIENT_ID + " , " + temperature + " , " + humidity);
      #endif
      mqtt.disconnect();
      return 0;
    }
  #endif
  return 1;
}

void screenUpdate(int temperature, int humidity, uint16_t co2, uint16_t extTemperature, uint16_t extHumidity, uint16_t extAQM, String messageText)
{
  #ifdef SCREEN
    display.clearBuffer();
    screenBorders();
    screenBatteryStatus();

    display.setTextColor(EPD_BLACK);
    // Labels
    display.setCursor(((display.width()/4)-10),((display.height()*1/8)-11));
    display.print("Here");
    display.setCursor(((display.width()*3/4-12)),((display.height()*1/8)-11));
    display.print("Outside");

    display.setTextSize(1);
    display.setFont(&FreeSans9pt7b);
    // Indoor information
    // Temperature
    display.setCursor(5,((display.height()*3/8)-10));
    if (temperature>0) display.print(String("Temp ") + temperature + "F");
    // Humidity
    display.setCursor(5,((display.height()*5/8)-10));
    if (temperature>0) display.print(String("Humidity ") + humidity + "%");
    #ifdef CO2_SENSOR
      display.setCursor(5,((display.height()*7/8)-10));
      display.print(String("C02 ") + co2 + " ppm");
    #endif

    // Outdoor information
    #ifdef WEATHER
    // Temperature
      display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
      display.print(String("Temp ") + extTemperature + "F");
      // Humidity
      display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
      display.print(String("Humidity ") + extHumidity +"%");
      // AQM
      display.setCursor(((display.width()/2)+5),((display.height()*7/8)-10));
      display.print("AQI ");
      switch (extAQM)
      {
        case 1:
          display.print("Good");
          break;
        case 2:
          display.print("Fair");
          break;
        case 3:
          display.print("Moderate");
          break;
        case 4:
          display.print("Poor");
          break;
        case 5:
          display.print("Very Poor");
          break;
        default:
          display.print("???");
          break;        
      }
    #endif

    // Mesages
    display.setFont();  // resets to system default monospace font
    display.setCursor(5,(display.height()-10));
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
    // C02 area
    display.drawLine(0,(display.height()*7/8),display.width(),(display.height()*7/8), EPD_GRAY);
    // splitting sensor vs. outside values
    display.drawLine((display.width()/2),0,(display.width()/2),(display.height()*7/8),EPD_GRAY);
  #endif
}

void screenBatteryStatus()
{
  #if defined(SCREEN) && defined(BATTERY) 
    // render battery percentage to screen

    int barHeight = 10;
    int barWidth = 28;
    // stored so we don't call the function twice in the routine
    float percent = lc.cellPercent();
    debugMessage("Battery is at " + String(percent) + " percent capacity");
    debugMessage("Battery voltage: " + String(lc.cellVoltage()) + " v");
    //debugMessage("Battery temperature: " + String(lc.getCellTemperature()) + " F");

    //calculate fill
    display.fillRect((display.width()-33),((display.height()*7/8)+4),(int((percent/100)*barWidth)),barHeight,EPD_GRAY);
    // border
    display.drawRect((display.width()-33),((display.height()*7/8)+4),barWidth,barHeight,EPD_BLACK);
    mqttBatteryAlert(int(percent));
  #endif

}

void actionSequence()
{
  debugMessage("actionsequence started");
  envData sensorData = readEnvironment();
  #ifdef MQTTLOG
    if (mqttSensorUpdate(sensorData.internalTemp, sensorData.internalHumidity, sensorData.internalCO2))
      screenUpdate(sensorData.internalTemp, sensorData.internalHumidity, sensorData.internalCO2, sensorData.extTemperature, sensorData.extHumidity, sensorData.extAQM, (String(CLIENT_ID) + " pub fail:" + zuluDateTimeString()));
    else
      screenUpdate(sensorData.internalTemp, sensorData.internalHumidity, sensorData.internalCO2, sensorData.extTemperature, sensorData.extHumidity, sensorData.extAQM, (String(CLIENT_ID) + " pub:" + zuluDateTimeString()));
  #else
    screenUpdate(sensorData.internalTemp, sensorData.internalHumidity, sensorData.internalCO2, sensorData.extTemperature, sensorData.extHumidity, sensorData.extAQM,("Last update at " + zuluDateTimeString()));
    debugMessage(zuluDateTimeString() + "->" + sensorData.internalTemp + "F , " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
  #endif
}

#ifdef WEATHER
  String httpGETRequest(const char* serverName) 
  {
    HTTPClient http;
      
    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName);
    
    // Send HTTP POST request
    int httpResponseCode = http.GET();
    
    String payload = "{}"; 
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      payload = http.getString();
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();

    return payload;
  }
#endif