/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// environment characteristics
typedef struct
{
  uint16_t internalTemp;
  uint16_t internalHumidity;
  uint16_t internalCO2;
  uint16_t extTemperature;
  uint16_t extHumidity;
  uint16_t extAQI;
} envData;

// global for air characteristics
envData sensorData;

bool screenAvailable;
bool batteryAvailable;
bool internetAvailable;

// used to retreive weather information
#include <HTTPClient.h> 
#include "ArduinoJson.h"

// initialize environment sensors
// SCD40; temp, humidity, CO2
#include <SensirionI2CScd4x.h>
SensirionI2CScd4x envSensor;

// AHTX0; temp, humidity
//#include <Adafruit_AHTX0.h>
//Adafruit_AHTX0 envSensor;

// Si7021
// #include "Adafruit_Si7021.h"
// Adafruit_Si7021 envSensor = Adafruit_Si7021();

// BME680
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>
// #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BME280 envSensor;

// Battery voltage sensor
#include "Adafruit_LC709203F.h"
Adafruit_LC709203F lc;

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

  // NTP support
  #include <WiFiUdp.h>
  WiFiUDP Udp;
#endif

#ifdef RJ45
  // Set MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  #include <SPI.h>
  #include <Ethernet.h>
  EthernetClient client;

  // NTP support
  #include <EthernetUdp.h>
  EthernetUDP Udp;
#endif

// MQTT setup
// unsigned long previousMQTTPingTime = 0; // needed?
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC1,MQTT_QOS_1);
Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC2, MQTT_QOS_1);
Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC3, MQTT_QOS_1);
Adafruit_MQTT_Publish errMsgPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC4, MQTT_QOS_1);

// NTP setup
#include <TimeLib.h>
unsigned int localPort = 8888;       // local port to listen for UDP packets

// NTP Servers
//IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
IPAddress timeServer(132,163,97,6); // time.nist.gov

// time zone support
// const int timeZone = 0;     // UTC
//const int timeZone = -5;  // USA EST
//const int timeZone = -4;  // USA EDT
const int timeZone = -8;  // USA PST
//const int timeZone = -7;  // USA PDT
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// screen support
// Adafruit MagTag
#include <Adafruit_GFX.h>
#include "Adafruit_ThinkInk.h"
#include <Fonts/FreeSans9pt7b.h>
#define EPD_DC      7   // can be any pin, but required!
#define EPD_RESET   6   // can set to -1 and share with chip Reset (can't deep sleep)
#define EPD_CS      8   // can be any pin, but required!
#define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
#define EPD_BUSY    5   // can set to -1 to not use a pin (will wait a fixed delay)
ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
// colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK

void setup()
// One time run of code, then deep sleep
{
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial)
    {
      // wait for serial port
    }
    debugMessage("Air Quality started");
  #endif

  if (initScreen())
  {
    debugMessage("screen initialized");
    screenAvailable = true;
  }
  else
  {
    debugMessage("screen not detected");
    screenAvailable = false;
  }

  if (initSensor())
  {
    debugMessage("environment sensor initialized");
  }
  else
  {
    debugMessage("environment sensor failed to initialize, going to sleep");
    if (screenAvailable)
      alertScreen("environment sensor failure");
    deepSleep();
  }

  // add the logic here to determine if sample vs. log
  // sample
  // this is a logging session
  readSensor();

  if (lc.begin())
  {
    debugMessage("Battery voltage monitor ready");
    lc.setPackSize(BATTERYSIZE);
    batteryAvailable = true;
  }
  else
  {
    debugMessage("Battery and voltage monitor not detected");
    batteryAvailable = false;
  }

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

      if (tries == MAX_TRIES)
      {
        debugMessage(String("Can not connect to WFii after ") + MAX_TRIES + " attempts");
        internetAvailable = false;
      }
      tries++;
    }
    if (internetAvailable != false)
    {
      debugMessage("WiFi IP address is: " + ip2CharArray(WiFi.localIP()));
      debugMessage("RSSI is: " + String(WiFi.RSSI()) + " dBm");
      internetAvailable = true;
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
    if (Ethernet.begin(mac) == 0)
    {      
      // identified errors
      if (Ethernet.hardwareStatus() == EthernetNoHardware)
      {
        debugMessage("Ethernet hardware not found");
      }
      else if (Ethernet.linkStatus() == LinkOFF) 
      {
        debugMessage("Ethernet cable not connected");
      }
      else
      {
        // generic error
        debugMessage("Failed to configure Ethernet");
      }
      internetAvailable = false;
    }
    else
    {
      debugMessage(String("Ethernet IP address is: ") + ip2CharArray(Ethernet.localIP()));
      internetAvailable = true;
    }
  #endif
    
  if (internetAvailable)
  {
    // Get time from NTP
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);

    // wait until the time is set by the sync provider
    while(timeStatus()== timeNotSet);
    debugMessage("NTP time is " + zuluDateTimeString());
  }

  getWeather();

  if (internetAvailable)
  // update mqtt broker and screen
  {
    if (mqttSensorUpdate())
    {
      infoScreen(String(CLIENT_ID) + " pub fail:" + zuluDateTimeString());
    }
    else
    {
      infoScreen(String(CLIENT_ID) + " pub:" + zuluDateTimeString());
    }
    // if battery low, update mqtt broker
    mqttBatteryAlert();
  }
  else
  {
    // update screen only
    infoScreen("Last update at " + zuluDateTimeString());
  }
  deepSleep();
}

void loop()
{}

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

String zuluDateTimeString()
// Converts system time into human readable string
{
  String logString;
  if (timeStatus() != timeNotSet)
  {
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
    //logString += "Z";
    logString += "PST";
  }
  else
  {
    logString ="Time not set";
  }
  return logString;
}

void debugMessage(String messageText)
// wraps Serial.println as #define conditional
{
  #ifdef DEBUG
    Serial.println(messageText);
  #endif
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
// Powers down hardware in preparation for board deep sleep
{
  debugMessage(String("Going to sleep for ") + LOG_INTERVAL + " minutes");
  if (screenAvailable)
  {
    display.powerDown();
    digitalWrite(EPD_RESET, LOW); // hardware power down mode
  }
  #ifdef WIFI
    client.stop();
  #endif
  // SCD40 only
  envSensor.stopPeriodicMeasurement();
  esp_sleep_enable_timer_wakeup(LOG_INTERVAL*LOG_INTERVAL_US_MODIFIER);
  esp_deep_sleep_start();
}

void getWeather()
// retrieves weather from Open Weather Map APIs and stores data to environment global
{
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
      //Serial.println(httpError.c_str());
      debugMessage(String(httpError.c_str()));
      sensorData.extTemperature = 0;
      sensorData.extHumidity = 0;
    }

    //JsonObject weather_0 = doc["weather"][0];
    // int weather_0_id = weather_0["id"]; // 804
    // const char* weather_0_main = weather_0["main"]; // "Clouds"
    // const char* weather_0_description = weather_0["description"]; // "overcast clouds"
    // const char* weather_0_icon = weather_0["icon"]; // "04n"

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

    // long sys_sunrise = sys["sunrise"]; // 1640620588
    // long sys_sunset = sys["sunset"]; // 1640651017

    // int timezone = doc["timezone"]; // -28800
    // long id = doc["id"]; // 5803139
    // const char* name = doc["name"]; // "Mercer Island"
    // int cod = doc["cod"]; // 200

    // Get local AQI
    serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;
      
    jsonBuffer = httpGETRequest(serverPath.c_str());
    debugMessage(jsonBuffer);
    
    StaticJsonDocument<384> doc1;

    httpError = deserializeJson(doc1, jsonBuffer);

    if (httpError)
    {
      debugMessage("Unable to parse air quality JSON object");
      //Serial.println(httpError.c_str());
      debugMessage(String(httpError.c_str()));
      sensorData.extAQI = 10000;
    }

    // double coord_lon = doc1["coord"]["lon"]; // -122.2221
    // float coord_lat = doc1["coord"]["lat"]; // 47.5707

    JsonObject list_0 = doc1["list"][0];

    sensorData.extAQI = list_0["main"]["aqi"]; // 2

    // JsonObject list_0_components = list_0["components"];
    // float list_0_components_co = list_0_components["co"]; // 453.95
    // float list_0_components_no = list_0_components["no"]; // 0.47
    // float list_0_components_no2 = list_0_components["no2"]; // 52.09
    // float list_0_components_o3 = list_0_components["o3"]; // 17.17
    // float list_0_components_so2 = list_0_components["so2"]; // 7.51
    // float list_0_components_pm2_5 = list_0_components["pm2_5"]; // 8.04
    // float list_0_components_pm10 = list_0_components["pm10"]; // 9.96
    // float list_0_components_nh3 = list_0_components["nh3"]; // 0.86
    
    debugMessage(zuluDateTimeString() + "OWM->" + sensorData.extTemperature + "F , " + sensorData.extHumidity + "%, " + sensorData.extAQI);
}

void mqttConnect()
// Connects and reconnects to MQTT broker, call from loop() to maintain connection
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

  while ((mqttErr = mqtt.connect() != 0) && (tries<=MQTT_ATTEMPT_LIMIT))
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
    debugMessage(String(MQTT_BROKER) + " connect attempt " + tries + " of " + MQTT_ATTEMPT_LIMIT + " happens in " + (tries*10) + " seconds");
    mqtt.disconnect();
    delay(tries*10000);
    tries++;

    if (tries == MQTT_ATTEMPT_LIMIT) 
    {
      debugMessage(String("Connection failed to MQTT broker ") + MQTT_BROKER);
    }
  }
  if (tries<MQTT_ATTEMPT_LIMIT)
  {
    debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
  }
}

void mqttBatteryAlert()
// Publishes battery percent to MQTT broker when <= pre-defined level is met
{
  // stored so we don't call the function twice in the routine
  float percent = lc.cellPercent();

  if ((batteryAvailable) && (percent<20))
  {
    String errMessage = String(CLIENT_ID) + " battery at " + percent + " percent at " + zuluDateTimeString();  
    mqttConnect();

    int charArrayLength = errMessage.length()+1;
    char textInChars[charArrayLength];
    errMessage.toCharArray(textInChars, charArrayLength);

    if (!errMsgPub.publish(textInChars))
    {
      debugMessage("MQTT low battery publish failed at " + zuluDateTimeString());
    }
    else 
    {
      debugMessage("MQTT publish: " + errMessage);
    }
    mqtt.disconnect();
}

int mqttSensorUpdate()
// Publishes sensor data to MQTT broker
{
  mqttConnect();
  if ((tempPub.publish(sensorData.internalTemp)) || (humidityPub.publish(sensorData.internalHumidity)) || (co2Pub.publish(sensorData.internalCO2)))
  {
    debugMessage("Published to MQTT: " + zuluDateTimeString() + " , " + CLIENT_ID + " , " + sensorData.internalTemp + " , " + sensorData.internalHumidity + " , " + sensorData.internalCO2);
    mqtt.disconnect();
    return 1;
  }
  else
  {
    debugMessage("Part/all of MQTT publish failed at " + zuluDateTimeString());
    mqtt.disconnect();
    return 0;
  }
}

int initScreen()
{
    // initalize epd screen
    display.begin(THINKINK_GRAYSCALE4);
    return 1;
}

void alertScreen(String messageText)
// Display critical error message on screen
{
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()/2));
  display.print(messageText);

  //update display
  display.display();
}

void infoScreen(String messageText)
// Display environmental information on screen
{
  envData sensorData;

  display.clearBuffer();

  // borders
  // ThinkInk 2.9" epd is 296x128 pixels
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

  // battery status
  screenBatteryStatus();

  display.setTextColor(EPD_BLACK);

  // indoor and outdoor labels
  display.setCursor(((display.width()/4)-10),((display.height()*1/8)-11));
  display.print("Here");
  display.setCursor(((display.width()*3/4-12)),((display.height()*1/8)-11));
  display.print("Outside");

  display.setTextSize(1);
  display.setFont(&FreeSans9pt7b);

  // indoor info
  if (!sensorData.internalTemp==10000)
  {
    display.setCursor(5,((display.height()*3/8)-10));
    display.print(String("Temp ") + sensorData.internalTemp + "F");
  }
  if (!sensorData.internalHumidity==10000)
  {
    display.setCursor(5,((display.height()*5/8)-10));
    display.print(String("Humidity ") + sensorData.internalHumidity + "%");
  }
  if (!sensorData.internalCO2==10000)
  {
    display.setCursor(5,((display.height()*7/8)-10));
    display.print(String("C02 ") + sensorData.internalCO2 + " ppm");    
  }

  // outdoor info
  if (!sensorData.extTemperature==10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
    display.print(String("Temp ") + sensorData.extTemperature + "F");
  }
  if (!sensorData.extHumidity==10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
    display.print(String("Humidity ") + extHumidity +"%");
  }
  // air quality index (AQI)
  if (!sensorData.extAQI==10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*7/8)-10));
    display.print("AQI ");
    switch (extAQI)
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
  }

  // message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()-10));
  display.print(messageText);

  //update display
  display.display();
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
{
  if batteryAvailable
  {
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
  }
}

int initSensor()
{
  //SCD40
  uint16_t error;

  Wire.begin();
  envSensor.begin(Wire);
  if (envSensor.startPeriodicMeasurement())
  // error = envSensor.startPeriodicMeasurement();
  // if (error)
  {
    // delay(5000); // allow the sensor to warm up
    return 1;
  }
  else
  {
    //delay(5000);
    return 0;
  }
  // ATHX0, SiH7021, BME280
  // if (envSensor.begin())
  //   {
  //     return 1;
  //   }
  //   else
  //   {
  //     return 0;
  //   }
}

void readSensor()
// reads environment sensor and stores data to environment global
{ 
  // SCD40
  //uint8_t error;
  float sensorTemp;
  float sensorHumidity;

  // error = envSensor.readMeasurement(sensorData.internalCO2, sensorTemp, sensorHumidity);
  // if (error)
  if (envSensor.readMeasurement(sensorData.internalCO2, sensorTemp, sensorHumidity))
  {
    debugMessage("Error reading SCD40 sensor");
    sensorData.internalCO2 = 10000;
    sensorData.internalTemp = 10000;
    sensorData.internalHumidity = 10000;
  }
  // convert C to F for temp, round to int
  sensorData.internalTemp = (int) sensorTemp*1.8+32+0.5;
  sensorData.internalHumidity = (int) sensorHumidity + 0.5;

  // AHTX0
  // sensors_event_t sensorHumidity, sensorTemp;
  // envSensor.getEvent(&sensorHumidity, &sensorTemp);
  // // no error handling?!
  // sensorData.internalTemp = (int) sensorTemp.temperature+.5;
  // sensorData.internalHumidity = (int) sensorHumidity.relative_humidity+.5;
  // sensorData.internalCO2 = 10000;

  // bme280, SiH7021
  // sensorData.internalTemp = (int) (envSensor.readTemperature()*1.8)+32;
  // sensorData.internalHumidity = (int) envSensor.readHumidity();
  // sensorData.internalCO2 = 10000;

  // SiH7021
  // no error handling?
  // sensorData.internalTemp = (int) (envSensor.readTemperature()*1.8)+32;
  // sensorData.internalHumidity = (int) envSensor.readHumidity();
  // sensorData.internalCO2 = 10000;

  debugMessage(zuluDateTimeString() + "sensor->" + sensorData.internalTemp + "F , " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
}

String httpGETRequest(const char* serverName) 
{
  HTTPClient http;
    
  // servername is domain name w/URL path or IP address w/URL path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0)
  {
    // Serial.print("HTTP Response code: ");
    // Serial.println(httpResponseCode);
    debugMessage("HTTP Response code: " + httpResponseCode);
    payload = http.getString();
  }
  else
  {
    // Serial.print("Error code: ");
    // Serial.println(httpResponseCode);
    debugMessage("Error code: " + httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}