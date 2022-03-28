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

#include "ArduinoJson.h"  // Needed by getWeather()

// Networking subsystem functions (see aq_network.cpp)
extern bool networkBegin();
extern String dateTimeString();
extern void networkStop();
extern String httpGETRequest(const char* serverName);
extern int httpPOSTRequest(String serverurl, String contenttype, String payload);

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
  extern void post_influx(uint16_t co2, float tempF, float humidity, float battpct, float battv);
#endif

#ifdef DWEET
  extern void post_dweet(uint16_t co2, float tempF, float humidity, float battpct, float battv);
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
    debugMessage("Log interval: " + String(LOG_INTERVAL));
    debugMessage("Site lat/long: " + String(OWM_LAT_LONG));
    debugMessage("Client ID: " + String(CLIENT_ID));
    #ifdef DWEET
      debugMessage("Dweet device: " + String(DWEET_DEVICE));
    #endif
    debugMessage("---------------------------------");
    debugMessage("Air Quality started");

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

  if (initSensor())
  {
    debugMessage("Environment sensor ready");
    readSensor();
  }
  else
  {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    if (screenAvailable)
      alertScreen("Environment sensor not detected");
    deepSleep();
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

  // Setup whatever network connection is specified in config.h
  internetAvailable = networkBegin();  

  // Implement a variety of internet services, if networking hardware is present and the
  // network is connected.  Services supported include:
  //
  //  NTP to get date and time information (via Network subsystem)
  //  Open Weather Map (OWM) to get local weather and AQI info
  //  MQTT to publish data to an MQTT broker on specified topics
  //  DWEET to publish data to the DWEET service

  // if there is an activenetwork connection
  if (internetAvailable)
  {
    // Get local weather and AQI info
    getWeather();

    #ifdef MQTTLOG
      if ((mqttSensorUpdate()) && (mqttBatteryUpdate()))
      {
        // Update screen and indicate MQTT update happened
        infoScreen("Updated [+M]:" + dateTimeString());  
      }
    #else
      // Update screen if not posting via MQTT
      infoScreen("Updated: " + dateTimeString());
    #endif

    #ifdef DWEET
      if(sensorData.internalCO2 != 10000) {
        float battpct = lc.cellPercent();
        float battv   = lc.cellVoltage();
        post_dweet(sensorData.internalCO2, sensorData.internalTemp, sensorData.internalHumidity,
          battpct, battv);
      }
    #endif

    #ifdef INFLUX
      if(sensorData.internalCO2 != 10000) {
        float battpct = lc.cellPercent();
        float battv   = lc.cellVoltage();
        post_influx(sensorData.internalCO2, sensorData.internalTemp, sensorData.internalHumidity,
          battpct, battv);
      }
    #endif
  }
  else
  {
    // update screen only (if no internet)
    infoScreen("Updated: " + dateTimeString());
  }
  deepSleep();
}

void loop()
{}

#ifdef MQTTLOG
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
    if ((sensorData.internalCO2==10000)&&(sensorData.internalTemp=10000))
    // no sensor data to publish
    {
      debugMessage("No sensor data to publish to MQTT broker");
    }
    else
    {
      mqttConnect();
      if ((tempPub.publish(sensorData.internalTemp)) && (humidityPub.publish(sensorData.internalHumidity)))
      {
        if (sensorData.internalCO2!=10000)
        {
          if(co2Pub.publish(sensorData.internalCO2))
          {
            debugMessage("MQTT publish at " + dateTimeString() + "->" + CLIENT_ID + "," + sensorData.internalTemp + "," + sensorData.internalHumidity + "," + sensorData.internalCO2);
            return 1;
          }
          else
          {
            debugMessage("MQTT publish at  " + dateTimeString() + " , " + CLIENT_ID + " , " + sensorData.internalTemp + " , " + sensorData.internalHumidity);
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
  debugMessage(String("Going to sleep for ") + LOG_INTERVAL + " minutes");
  if (screenAvailable)
  {
    display.powerDown();
    digitalWrite(EPD_RESET, LOW); // hardware power down mode
  }
  networkStop();  // End any active network connection (if supported)
  // SCD40 only
  envSensor.stopPeriodicMeasurement();
  esp_sleep_enable_timer_wakeup(LOG_INTERVAL*LOG_INTERVAL_US_MODIFIER);
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
      debugMessage(jsonBuffer);
  
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
      debugMessage(jsonBuffer);
  
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
  debugMessage(String("OWM->") + sensorData.extTemperature + "F," + sensorData.extHumidity + "%, " + sensorData.extAQI + " AQI");
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
  if (sensorData.internalTemp!=10000)
  {
    display.setCursor(5,((display.height()*3/8)-10));
    display.print(String("Temp ") + sensorData.internalTemp + "F");
  }
  if (sensorData.internalHumidity!=10000)
  {
    display.setCursor(5,((display.height()*5/8)-10));
    display.print(String("Humidity ") + sensorData.internalHumidity + "%");
  }
  if (sensorData.internalCO2!=10000)
  {
    display.setCursor(5,((display.height()*7/8)-10));
    display.print(String("C02 ") + sensorData.internalCO2 + " ppm");
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
  uint8_t error;
  float sensorTemp;
  float sensorHumidity;

  error = envSensor.readMeasurement(sensorData.internalCO2, sensorTemp, sensorHumidity);
  if (error)
  {
    debugMessage("Error reading SCD40 sensor");
    sensorData.internalCO2 = 10000;
    sensorData.internalTemp = 10000;
    sensorData.internalHumidity = 10000;
  }
  else
  {
    // convert C to F for temp, round to int, and store
    sensorData.internalTemp = (int) ((sensorTemp*1.8)+32+0.5);
    sensorData.internalHumidity = (int) (sensorHumidity + 0.5);
  }

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

  debugMessage(String("Sensor->") + sensorData.internalTemp + "F," + sensorData.internalHumidity + "%," + sensorData.internalCO2 + " ppm");
}
