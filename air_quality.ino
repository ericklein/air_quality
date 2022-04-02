/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

int sampleCounter;
float intermediateTemp;
float intermediateHumidity;
uint16_t intermediateCO2;

// environment characteristics
typedef struct
{
  float internalTemp;
  float internalHumidity;
  uint16_t internalCO2;
  int extTemperature;
  int extHumidity;
  int extAQI;
} envData;

// global for air characteristics
envData sensorData;

bool screenAvailable = false;
bool batteryAvailable = false;
bool internetAvailable = false;

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

//#include <TimeLib.h> // time related functions

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
  WiFiUDP ntpUDP;
#endif

#ifdef RJ45
  // Set MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  #include <SPI.h>
  #include <Ethernet.h>
  EthernetClient client;

  // NTP support
  #include <EthernetUdp.h>
  EthernetUDP ntpUDP;
#endif

#if defined(WIFI) || defined(RJ45)
  // NTP setup
  #include <NTPClient.h>
  NTPClient timeClient(ntpUDP);

  // used to retreive OWM information
  #include <HTTPClient.h> 
  #include "ArduinoJson.h"

  #ifdef MQTTLOG
    // MQTT setup
    #include "Adafruit_MQTT.h"
    #include "Adafruit_MQTT_Client.h"
    Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

    Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC1,MQTT_QOS_1);
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC2, MQTT_QOS_1);
    Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC3, MQTT_QOS_1);
    Adafruit_MQTT_Publish batteryLevelPub = Adafruit_MQTT_Publish(&mqtt, MQTT_PUB_TOPIC4, MQTT_QOS_1);
  #endif

  #ifdef INFLUX
    extern void post_influx(uint16_t co2, float tempF, float humidity);
  #endif

#endif

void setup()
// One time run of code, then deep sleep
{
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial)
    {
      // wait for serial port
    }
    // Confirm key site configuration parameters
    debugMessage("Air Quality started");
    debugMessage("---------------------------------");
    debugMessage(String(SAMPLE_INTERVAL) + " minute sample interval");
    debugMessage(String(SAMPLE_SIZE) + " samples before logging");
    #ifdef SAMPLE_INTERVAL_ESP_MODIFIER
      debugMessage("ESP microsecond modified is active");
    #endif
    debugMessage("Site lat/long: " + String(OWM_LAT_LONG));
    debugMessage("Client ID: " + String(CLIENT_ID));
    #ifdef DWEET
      debugMessage("Dweet device: " + String(DWEET_DEVICE));
    #endif
  #endif

  if (initScreen())
  {
    debugMessage("Screen ready");
    screenAvailable = true;
  }
  else
  {
    debugMessage("Screen not detected");
  }

  if (!initSensor())
  {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    if (screenAvailable)
      alertScreen("Environment sensor not detected");
    deepSleep();
  }

  readSensor();
  sampleCounter = readNVStorage();
  nvStorage.putInt("counter",++sampleCounter);
  debugMessage(String("Sample count TO nv storage is ") + sampleCounter);

  if (sampleCounter < SAMPLE_SIZE)
  // update intermediate sensor values and go back to sleep
  {
    nvStorage.putFloat("temp",(sensorData.internalTemp+intermediateTemp));
    nvStorage.putFloat("humidity",(sensorData.internalHumidity+intermediateHumidity));
    if (sensorData.internalCO2 == 10000)
    {
    debugMessage(String("Intermediate values TO nv storage: Temp:")+(sensorData.internalTemp+intermediateTemp)+" Humidity:"+(sensorData.internalHumidity+intermediateHumidity));
    }
    else
    {
      nvStorage.putUInt("co2",(sensorData.internalCO2+intermediateCO2));
      debugMessage(String("Intermediate values TO nv storage: Temp:") + (sensorData.internalTemp+intermediateTemp) + " Humidity:" + (sensorData.internalHumidity+intermediateHumidity) + ", CO2:" + (sensorData.internalCO2+intermediateCO2));
    }
    deepSleep();
  }
  else
  {
    // average intermediate values
    intermediateTemp = ((sensorData.internalTemp+intermediateTemp)/SAMPLE_SIZE);
    intermediateHumidity = ((sensorData.internalHumidity+intermediateHumidity)/SAMPLE_SIZE);
    if (sensorData.internalCO2 != 10000)
    {
      intermediateCO2 = ((sensorData.internalCO2+intermediateCO2)/SAMPLE_SIZE);
    }
    // publish new averages to persistent storage
    nvStorage.putFloat("temp",intermediateTemp);
    nvStorage.putFloat("humidity",intermediateHumidity);
    if (sensorData.internalCO2 == 10000)
    {
    debugMessage(String("Averaged values TO nv storage: Temp:")+(sensorData.internalTemp+intermediateTemp)+", Humidity:"+intermediateHumidity);
    }
    else
    {
      nvStorage.putUInt("co2",intermediateCO2);
      debugMessage(String("Averaged values TO nv storage: Temp:")+intermediateTemp+", Humidity:"+intermediateHumidity+", CO2:"+intermediateCO2);
    }
    //reset and store sample count
    sampleCounter = 1;
    nvStorage.putInt("counter",sampleCounter);
    debugMessage(String("Sample counter reset to ")+sampleCounter);
  }

  if (lc.begin())
  // Check battery monitoring status
  {
    debugMessage("Battery monitor ready");
    lc.setPackAPA(BATTERY_APA);
    batteryAvailable = true;
  }
  else
  {
    debugMessage("Battery monitor not detected");
  }

  #ifdef WIFI
    uint8_t tries;
    #define MAX_TRIES 5
  
    // set hostname has to come before WiFi.begin
    WiFi.hostname(CLIENT_ID);
    // WiFi.setHostname(CLIENT_ID); //for WiFiNINA
    
    // Connect to WiFi.  Prepared to wait a reasonable interval for the connection to
    // succeed, but not forever.  Will check status and, if not connected, delay an
    // increasing amount of time up to a maximum of MAX_TRIES delay intervals. 
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    for(tries=1;tries<=MAX_TRIES;tries++) 
    {
      debugMessage(String("Connection attempt ") + tries + " of " + MAX_TRIES + " to " + WIFI_SSID + " in " + (tries*10) + " seconds");
      if(WiFi.status() == WL_CONNECTED) 
      {
        // Successful connection!
        internetAvailable = true;
        break;
      }
      // use of delay OK as this is initialization code
      delay(tries*10000);   // Waiting longer each time we check for status
    }
    if(internetAvailable)
    {
      debugMessage("WiFi IP address is: " + ip2CharArray(WiFi.localIP()));
      debugMessage("RSSI is: " + String(WiFi.RSSI()) + " dBm");  
    }
    else
    {
      // Couldn't connect, alas
      debugMessage(String("Can not connect to WFii after ") + MAX_TRIES + " attempts");   
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
    }
    else
    {
      debugMessage(String("Ethernet IP address is: ") + ip2CharArray(Ethernet.localIP()));
      internetAvailable = true;
    }
  #endif

  // Implement a variety of internet services, if networking hardware is present and the
  // network is connected.  Services supported include:
  //
  //  NTP to get date and time information
  //  Open Weather Map (OWM) to get local weather and AQI info
  //  MQTT to publish data to an MQTT broker on specified topics
  //  DWEET to publish data to the DWEET service

  // Get local weather and air quality info from Open Weather Map
  getWeather();

  #if defined(WIFI) || defined(RJ45)
  // if there is a network interface (so networking code will compile)
    if (internetAvailable)
    // and internet is verified
    {
      // Get time from NTP
      timeClient.begin();
      // Set offset time in seconds to adjust for your timezone
      timeClient.setTimeOffset(timeZone*60*60);
      debugMessage("NTP time: " + dateTimeString());

      #ifdef MQTTLOG
        if ((mqttSensorUpdate()) && (mqttBatteryUpdate()))
        {
          infoScreen("Updated [+M] " + dateTimeString());     
        }
        else
        {
          infoScreen("Updated " + dateTimeString());
        }
      #endif

      // update screen only
      infoScreen("Updated " + dateTimeString());

      #ifdef DWEET
        post_dweet(intermediateCO2, intermediateTemp, intermediateHumidity);
      #endif
    }
    else
    {
      // update screen only
      infoScreen("Updated " + dateTimeString());
    }
    deepSleep();
  #else
    // update screen only
    infoScreen("Updated " + dateTimeString());
    deepSleep();
  #endif
}

void loop()
{}

#if defined(WIFI) || defined(RJ45)
  String ip2CharArray(IPAddress ip) 
  {
    static char a[16];
    sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    return a;
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
      debugMessage("HTTP Response code: " + httpResponseCode);
      payload = http.getString();
    }
    else
    {
      debugMessage("Error code: " + httpResponseCode);
    }
    // free resources
    http.end();
  
    return payload;
  }
#endif

#if ((defined(WIFI) || defined(RJ45)) && defined(MQTTLOG))
  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {
    int8_t mqttErr;
    int8_t tries = 1;
  
    // exit if already connected
    if (mqtt.connected())
    {
      return;
    }
  
    while ((mqttErr = mqtt.connect() != 0) && (tries<=MQTT_ATTEMPT_LIMIT))
    {
      // generic MQTT error
      // debugMessage(mqtt.connectErrorString(mqttErr));
  
      // Adafruit IO connect errors
      switch (mqttErr)
      {
        case 1: debugMessage("Adafruit MQTT: Wrong protocol"); break;
        case 2: debugMessage("Adafruit MQTT: ID rejected"); break;
        case 3: debugMessage("Adafruit MQTT: Server unavailable"); break;
        case 4: debugMessage("Adafruit MQTT: Incorrect user or password"); break;
        case 5: debugMessage("Adafruit MQTT: Not authorized"); break;
        case 6: debugMessage("Adafruit MQTT: Failed to subscribe"); break;
        default: debugMessage("Adafruit MQTT: GENERIC - Connection failed"); break;
      }
      debugMessage(String(MQTT_BROKER) + " connect attempt " + tries + " of " + MQTT_ATTEMPT_LIMIT + " happens in " + (tries*10) + " seconds");
      mqtt.disconnect();
      delay(tries*10000);
      tries++;
  
      if (tries == MQTT_ATTEMPT_LIMIT)
      {
        debugMessage(String("Connection failed to MQTT broker: ") + MQTT_BROKER);
      }
    }
    if (tries < MQTT_ATTEMPT_LIMIT)
    {
      debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
    }
  }

  int mqttBatteryUpdate()
  {
    if (batteryAvailable)
    {
      // stored so we don't call the function twice in the routine
      float percent = lc.cellPercent();
      if (batteryLevelPub.publish(percent))
      {
        debugMessage(String("MQTT battery percent publish with value:") + percent);
        return 1;
      }
      else
      {
        debugMessage(String("MQTT battery percent publish failed at:") + dateTimeString());
        return 0;
      }
      mqtt.disconnect();
    }
  }
  
  int mqttSensorUpdate()
  // Publishes sensor data to MQTT broker
  {
    if ((sensorData.internalCO2==10000)&&(sensorData.internalTemp==10000))
    // no sensor data to publish
    {
      debugMessage("No sensor data to publish to MQTT broker");
    }
    else
    {
      mqttConnect();
      if ((tempPub.publish(intermediateTemp)) && (humidityPub.publish(intermediateHumidity)))
      {
        if (sensorData.internalCO2!=10000)
        {
          if(co2Pub.publish(intermediateCO2))
          {
            debugMessage("MQTT publish at " + dateTimeString() + "->" + CLIENT_ID + "," + intermediateTemp + "," + intermediateHumidity + "," + intermediateCO2);
            return 1;
          }
          else
          {
            debugMessage("MQTT publish at  " + dateTimeString() + " , " + CLIENT_ID + " , " + intermediateTemp + " , " + intermediateHumidity);
            debugMessage("MQTT CO2 publish failed at " + dateTimeString());
            return 0;
          }
        }
      }
      else
      {
        debugMessage("MQTT temp and humidity publish failed at " + dateTimeString());
        return 0;   
      } 
    mqtt.disconnect();
    } 
  }
#endif

String dateTimeString()
// Converts system time into human readable strings
{
  String dateTime;
  String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

  #if defined(WIFI) || defined(RJ45)
    if (timeClient.update())
    {
      // NTPClient doesn't include date information, get it from time structure
      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime ((time_t *)&epochTime);
      int day = ptm->tm_mday;
      int month = ptm->tm_mon+1;
      int year = ptm->tm_year+1900;

      dateTime = weekDays[timeClient.getDay()];
      dateTime += ", ";

      if (month<10) dateTime += "0";
      dateTime += month;
      dateTime += "/";
      if (day<10) dateTime += "0";
      dateTime += day;
      dateTime += " at ";
      if (timeClient.getHours()<10) dateTime += "0";
      dateTime += timeClient.getHours();
      dateTime += ":";
      if (timeClient.getMinutes()<10) dateTime += "0";
      dateTime += timeClient.getMinutes();

      // zulu format
      // dateTime = year + "-";
      // if (month()<10) dateTime += "0";
      // dateTime += month;
      // dateTime += "-";
      // if (day()<10) dateTime += "0";
      // dateTime += day;
      // dateTime += "T";
      // if (timeClient.getHours()<10) dateTime += "0";
      // dateTime += timeClient.getHours();
      // dateTime += ":";
      // if (timeClient.getMinutes()<10) dateTime += "0";
      // dateTime += timeClient.getMinutes();
      // dateTime += ":";
      // if (timeClient.getSeconds()<10) dateTime += "0";
      // dateTime += timeClient.getSeconds();
      // switch (timeZone)
      // {
      //   case 0:
      //     dateTime += "Z";
      //     break;
      //   case -7:
      //     dateTime += "PDT";
      //     break;
      //   case -8:
      //     dateTime += "PST";
      //     break;     
      // }
    }
    else
    {
      dateTime = "Time not set";
    }
  #else
    dateTime = "Time not set";
  #endif
  return dateTime;
}

void debugMessage(String messageText)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  Serial.println(messageText);
  Serial.flush();  // Make sure the message gets output (before any sleeping...)
#endif
}

void deepSleep()
// Powers down hardware in preparation for board deep sleep
{
  debugMessage(String("Going to sleep for ") + SAMPLE_INTERVAL + " minute(s)");
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

  nvStorage.end();
  esp_sleep_enable_timer_wakeup(SAMPLE_INTERVAL*SAMPLE_INTERVAL_ESP_MODIFIER);
  esp_deep_sleep_start();
}

void getWeather()
// retrieves weather from Open Weather Map APIs and stores data to environment global
{
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (internetAvailable)
    // and internet is verified
    {
      String jsonBuffer;
  
      // Get local temp and humidity
      String serverPath = String(OWM_SERVER) + OWM_WEATHER_PATH + OWM_LAT_LONG + "&units=imperial" + "&APPID=" + OWM_KEY;
  
      jsonBuffer = httpGETRequest(serverPath.c_str());
      //debugMessage(jsonBuffer);
  
      StaticJsonDocument<1024> doc;
  
      DeserializationError httpError = deserializeJson(doc, jsonBuffer);
  
      if (httpError)
      {
        debugMessage("Unable to parse weather JSON object");
        debugMessage(String(httpError.c_str()));
        sensorData.extTemperature = 10000;
        sensorData.extHumidity = 10000;
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
      //debugMessage(jsonBuffer);
  
      StaticJsonDocument<384> doc1;
  
      httpError = deserializeJson(doc1, jsonBuffer);
  
      if (httpError)
      {
        debugMessage("Unable to parse air quality JSON object");
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
    }
  #else
    sensorData.extTemperature = 10000;
    sensorData.extHumidity = 10000;
    sensorData.extAQI = 10000;
  #endif
  debugMessage(String("Open Weather Map returned: ") + sensorData.extTemperature + "F, " + sensorData.extHumidity + "%, " + sensorData.extAQI + " AQI");
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
  String aqiLabels[5]={"Good", "Fair", "Moderate", "Poor", "Very Poor"};
  
  display.clearBuffer();

  // borders
  // ThinkInk 2.9" epd is 296x128 pixels
  // label border
  display.drawLine(0,(display.height()/8),display.width(),(display.height()/8),EPD_GRAY);
  // temperature area
  display.drawLine(0,(display.height()*3/8),display.width(),(display.height()*3/8),EPD_GRAY);
  // humidity area
  display.drawLine(0,(display.height()*5/8),display.width(),(display.height()*5/8),EPD_GRAY);
  // C02 area
  display.drawLine(0,(display.height()*7/8),display.width(),(display.height()*7/8),EPD_GRAY);
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
  int temperatureDelta = ((int)(sensorData.internalTemp +0.5)) - ((int) (intermediateTemp + 0.5));
  int humidityDelta = ((int)(sensorData.internalHumidity +0.5)) - ((int) (intermediateHumidity + 0.5));

  display.setCursor(5,((display.height()*3/8)-10));
  display.print(String("Temp ") + (int)(sensorData.internalTemp+0.5) + "F (" + temperatureDelta + ")");

  display.setCursor(5,((display.height()*5/8)-10));
  display.print(String("Humid ") + (int)(sensorData.internalHumidity+0.5) + "% (" + humidityDelta + ")");

  if (sensorData.internalCO2!=10000)
  {
    display.setCursor(5,((display.height()*7/8)-10));
    display.print(String("C02 ") + sensorData.internalCO2 + " (" + (sensorData.internalCO2 - intermediateCO2) + ")");
  }

  // outdoor info
  if (sensorData.extTemperature!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
    display.print(String("Temp ") + sensorData.extTemperature + "F");
  }
  if (sensorData.extHumidity!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
    display.print(String("Humidity ") + sensorData.extHumidity + "%");
  }
  // air quality index (AQI)
  if (sensorData.extAQI!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*7/8)-10));
    display.print("AQI ");
    display.print(aqiLabels[(sensorData.extAQI-1)]);
  }

  // message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()-10));
  display.print(messageText);

  //update display
  display.display();
  debugMessage("Screen updated");
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
{
  if (batteryAvailable)
  {
    // render battery percentage to screen

    int barHeight = 10;
    int barWidth = 28;
    // stored so we don't call the function twice in the routine
    float percent = lc.cellPercent();
    debugMessage("Battery is at " + String(percent) + " percent capacity");
    debugMessage("Battery voltage: " + String(lc.cellVoltage()) + " v");

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
  error = envSensor.startPeriodicMeasurement();
  if (error)
  {
    delay(5000); // allow the sensor to warm up
    return 0;
  }
  else
  {
    delay(5000);
    return 1;
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
  uint8_t error = envSensor.readMeasurement(sensorData.internalCO2, sensorData.internalTemp, sensorData.internalHumidity);
  if (error)
  {
    debugMessage("Error reading SCD40 sensor");
    deepSleep();
  }
  // convert C to F for temp
  sensorData.internalTemp = (sensorData.internalTemp*1.8)+32;
  debugMessage(String("SCD40 environment sensor values: ") + sensorData.internalTemp + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");

  // AHTX0
  // no error handling?!
  // sensors_event_t sensorHumidity, sensorTemp;
  // envSensor.getEvent(&sensorHumidity, &sensorTemp);
  // sensorData.internalTemp = sensorTemp.temperature;
  // sensorData.internalHumidity = sensorHumidity.relative_humidity;
  // sensorData.internalCO2 = 10000;
  // debugMessage(String("AHTX0 environment sensor values: ") + sensorData.internalTemp + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");


  // bme280, SiH7021
  // no error handling?!
  // sensorData.internalTemp = (envSensor.readTemperature()*1.8)+32;
  // sensorData.internalHumidity = envSensor.readHumidity();
  // sensorData.internalCO2 = 10000;
  // debugMessage(String("BME280/SiH7021 environment sensor values: ") + sensorData.internalTemp + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
}

int readNVStorage()
{
  int counter;

  nvStorage.begin("air-quality", false);
  // get previously stored values. If they don't exist, create them as zero
  counter = nvStorage.getInt("counter",1);
  // read value or insert current sensor reading if this is the first read from nv storage
  intermediateTemp = nvStorage.getFloat("temp",sensorData.internalTemp);
  intermediateHumidity = nvStorage.getFloat("humidity",sensorData.internalHumidity);
  if (sensorData.internalCO2 == 10000)
  // there is no CO2 sensor value to use
  {
    debugMessage(String("Intermediate values FROM nv storage: Temp:")+intermediateTemp+", Humidity:"+intermediateHumidity);
  }
  else
  // include CO2 data
  {
    intermediateCO2 = nvStorage.getUInt("co2",sensorData.internalCO2);
    debugMessage(String("Intermediate values FROM nv storage: Temp:")+intermediateTemp+", Humidity:"+intermediateHumidity+", CO2:"+intermediateCO2);
  }
  debugMessage(String("Sample count FROM nv storage is ") + counter);
  return counter;
}

#ifdef DWEET
// Post a dweet to report the various sensor reading.  This routine blocks while
// talking to the network, so may take a while to execute.
void post_dweet(uint16_t co2, float tempF, float humidity)
{

  if(WiFi.status() != WL_CONNECTED) {
    debugMessage("Lost network connection to '" + String(WIFI_SSID) + "'!");
    // show_disconnected();  / Future feature to display state of network connectivity (LED?)
    return;
  }

  debugMessage("connecting to " + String(DWEET_HOST));

  // Use WiFiClient class to create TCP connections
  WiFiClient dweet_client;
  const int httpPort = 80;
  if (!dweet_client.connect(DWEET_HOST, httpPort)) {
    debugMessage("connection failed: " + String(DWEET_HOST));
    return;
  }

  // Use HTTP post and send a data payload as JSON
  String device_info = "{\"rssi\":\""   + String(WiFi.RSSI())        + "\"," +
                       "\"ipaddr\":\"" + WiFi.localIP().toString()  + "\",";
  String battery_info;
  if(batteryAvailable) {
    battery_info = "\"battery_percent\":\"" + String(lc.cellPercent()) + "\"," +
                   "\"battery_voltage\":\"" + String(lc.cellVoltage()) + "\",";
  }
  else {
    battery_info = "";
  }
  String sensor_info = "\"co2\":\""         + String(co2)             + "\"," +
                       "\"temperature\":\"" + String(tempF, 2)         + "\"," +
                       "\"humidity\":\""    + String(humidity, 2)      + "\"}";

  String postdata = device_info + battery_info + sensor_info;
  dweet_client.println("POST /dweet/for/" + String(DWEET_DEVICE) + " HTTP/1.1");
  dweet_client.println("Host: dweet.io");
  dweet_client.println("Cache-Control: no-cache");
  dweet_client.println("Content-Type: application/json");
  dweet_client.print("Content-Length: ");
  dweet_client.println(postdata.length());
  dweet_client.println();
  dweet_client.println(postdata);
  debugMessage(postdata);
  delay(1500);

  // Fetch any reply from server and print as debug messages
  while(dweet_client.available()) {
    String line = dweet_client.readStringUntil('\r');
    debugMessage(line);
  }
  dweet_client.stop();
  debugMessage("closing connection for dweeting");
}
#endif