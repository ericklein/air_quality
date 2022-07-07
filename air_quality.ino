/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Generalized network handling
#include "aq_network.h"
AQ_Network aq_network;

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// Global variables
// accumulating sensor readings
float averageTempF;
float averageHumidity;
uint16_t averageCO2;

// environment sensor data
typedef struct
{
  float internalTempF;
  float internalHumidity;
  uint16_t internalCO2;
} envData;

// global for air characteristics
envData sensorData;

// Open Weather Map Current data
typedef struct
{
  // "lon": 8.54
  float lon;
  // "lat": 47.37
  float lat;
  // "id": 521
  uint16_t weatherId;
  // "main": "Rain"
  String main;
  // "description": "shower rain"
  String description;
  // "icon": "09d"
  String icon;
  String iconMeteoCon;
  // "temp": 290.56
  float temp;
  // "pressure": 1013
  uint16_t pressure;
  // "humidity": 87
  uint16_t humidity;
  // "temp_min": 289.15
  float tempMin;
  // "temp_max": 292.15
  float tempMax;
  // visibility: 10000
  uint16_t visibility;
  // "wind": {"speed": 1.5}
  float windSpeed;
  // "wind": {deg: 226.505}
  float windDeg;
  // "clouds": {"all": 90}
  uint8_t clouds;
  // "dt": 1527015000
  time_t observationTime;
  // "country": "CH"
  String country;
  // "sunrise": 1526960448
  time_t sunrise;
  // "sunset": 1527015901
  time_t sunset;
  // "name": "Zurich"
  String cityName;
  time_t timezone;
} OpenWeatherMapCurrentData;

// global for OWM current data
OpenWeatherMapCurrentData owmCurrentData;

typedef struct
{
  // "lon": 8.54
  float lon;
  // "lat": 47.37
  float lat;
  // "aqi": 2
  uint16_t aqi;
  // "co": 453.95
  float co;
  // "no": 0.47
  float no;
  // "no2": 52.09
  float no2;
  // "o3": 17.17
  float o3;
  // "so2": 7.51
  float so2;
  // "pm2.5": 8.04
  float pm2_5;
  // "pm10": 9.96
  float pm10;
  // "nh3": 0.86
  float nh3;
} OpenWeatherMapAirQuality;

// global for OWM current data
OpenWeatherMapAirQuality owmAirQuality;

bool batteryAvailable = false;
bool internetAvailable = false;

// initialize environment sensors
// SCD40; temp, humidity, CO2
#include <SensirionI2CScd4x.h>
SensirionI2CScd4x envSensor;

// unified Adafruit sensor setup
// AHTX0; temp, humidity
//#include <Adafruit_AHTX0.h>
//Adafruit_AHTX0 envSensor;

// BME280; temp, humidity
// #include <Adafruit_BME280.h>
// Adafruit_BME280 envSensor; // i2c interface
// Adafruit_Sensor *envSensor_temp = envSensor.getTemperatureSensor();
// Adafruit_Sensor *envSensor_humidity = envSensor.getHumiditySensor();

// Battery voltage sensor
#include <Adafruit_LC709203F.h>
Adafruit_LC709203F lc;

// screen support
#ifdef SCREEN
  #include <Adafruit_ThinkInk.h>

  #include "Fonts/meteocons48pt7b.h"
  #include "Fonts/meteocons24pt7b.h"
  #include "Fonts/meteocons20pt7b.h"
  #include "Fonts/meteocons16pt7b.h"

  #include <Fonts/FreeSans9pt7b.h>
  #include <Fonts/FreeSans12pt7b.h>
  #include <Fonts/FreeSans18pt7b.h>

  // MagTag board definition
  // #define EPD_RESET   6   // can set to -1 and share with chip Reset (can't deep sleep)
  // #define EPD_DC      7   // can be any pin, but required!
  // #define EPD_CS      8   // can be any pin, but required!
  #define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
  #define EPD_BUSY    5   // can set to -1 to not use a pin (will wait a fixed delay)

  // ESP32S2 w/BME280 board definition
  // #define EPD_RESET   -1   // can set to -1 and share with chip Reset (can't deep sleep)
  // #define EPD_DC      10   // can be any pin, but required!
  // #define EPD_CS      9   // can be any pin, but required!
  // #define SRAM_CS     6  // can set to -1 to not use a pin (uses a lot of RAM!)
  // #define EPD_BUSY    -1   // can set to -1 to not use a pin (will wait a fixed delay)

  ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
  // colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK
#endif

#include "ArduinoJson.h"  // Needed by getWeather()

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_p, float battery_v, int rssi);
#endif

#ifdef DWEET
  extern void post_dweet(uint16_t co2, float tempF, float humidity, float battpct, float battv, int rssi);
#endif

#ifdef MQTTLOG
  extern void mqttConnect();
  extern int mqttDeviceInfoUpdate(float cellPercent, float cellVoltage, int rssi);
  extern int mqttSensorUpdate(uint16_t co2, float tempF, float humidity);
#endif

void setup()
// One time run of code, then deep sleep
{
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // turn on the I2C power by setting pin to opposite of 'rest state'
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
  #endif

  #ifdef SCREEN
    // there is no way to query screen for status
    //display.begin(THINKINK_GRAYSCALE4);
    display.begin(THINKINK_MONO);
    debugMessage("Display ready");
  #endif

  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);

    // Confirm key site configuration parameters
    debugMessage("Air Quality started");
    // debugMessage("---------------------------------");
    // debugMessage(String(SAMPLE_INTERVAL) + " minute sample interval");
    // debugMessage(String(SAMPLE_SIZE) + " samples before logging");
    // debugMessage("Site lat/long: " + String(OWM_LAT_LONG));
    // debugMessage("Site altitude: " + String(SITE_ALTITUDE));
    // debugMessage("Client ID: " + String(CLIENT_ID));
    #ifdef DWEET
      debugMessage("Dweet device: " + String(DWEET_DEVICE));
    #endif
  #endif

  // Initialize environmental sensor.  Returns non-zero if initialization fails
  if (initSensor()) 
  {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    alertScreen("Env sensor not detected");
    deepSleep();
  }

  // Environmental sensor available, so fetch values
  int sampleCounter;
  if(readSensor())
  {
    debugMessage("Environment sensor failed to read, going to sleep");
    alertScreen("Env sensor no data");
    deepSleep();
  }
  sampleCounter = readNVStorage();
  sampleCounter++;

  if (sampleCounter < SAMPLE_SIZE)
  // add to the accumulating, intermediate sensor values and go back to sleep
  {
    nvStorage.putInt("counter", sampleCounter);
    debugMessage(String("Sample count TO nv storage is ") + sampleCounter);
    nvStorage.putFloat("temp", (sensorData.internalTempF + averageTempF));
    nvStorage.putFloat("humidity", (sensorData.internalHumidity + averageHumidity));
    if (sensorData.internalCO2 != 10000) {
      nvStorage.putUInt("co2", (sensorData.internalCO2 + averageCO2));
    }
    debugMessage(String("Intermediate values TO nv storage: Temp:") + (sensorData.internalTempF + averageTempF) + " Humidity:" + (sensorData.internalHumidity + averageHumidity) + ", CO2:" + (sensorData.internalCO2 + averageCO2));
    deepSleep();
  } 
  else
  {
    // average intermediate values
    averageTempF = ((sensorData.internalTempF + averageTempF) / SAMPLE_SIZE);
    averageHumidity = ((sensorData.internalHumidity + averageHumidity) / SAMPLE_SIZE);
    if (sensorData.internalCO2 != 10000) 
    {
      averageCO2 = ((sensorData.internalCO2 + averageCO2) / SAMPLE_SIZE);
    }
    debugMessage(String("Averaged values Temp:") + averageTempF + "F, Humidity:" + averageHumidity + ", CO2:" + averageCO2);
    //reset and store sample set variables
    nvStorage.putInt("counter", 0);
    nvStorage.putFloat("temp",0);
    nvStorage.putFloat("humidity",0);
    nvStorage.putUInt("co2",0);
    debugMessage("Intermediate values in nv storage reset to zero");
  }

  initBattery();

  // Setup whatever network connection is specified in config.h
  internetAvailable = aq_network.networkBegin();

  // Implement a variety of internet services, if networking hardware is present and the
  // network is connected.  Services supported include:
  //
  //  NTP to get date and time information (via Network subsystem)
  //  Open Weather Map (OWM) to get local weather and AQI info
  //  MQTT to publish data to an MQTT broker on specified topics
  //  DWEET to publish data to the DWEET service

  // Get local weather and air quality info from Open Weather Map
  if (!getOWMWeather())
  {
    owmCurrentData.temp = 10000;
    owmCurrentData.humidity = 10000;
  }

  if (!getOWMAQI())
  {
    owmAirQuality.aqi = 10000;
  }

  String upd_flags = "";  // To indicate whether services succeeded
  if (internetAvailable)
  {
    float battpct, battv;
    int rssi;

    rssi = aq_network.getWiFiRSSI();

    if (batteryAvailable)
    {
      battpct = lc.cellPercent(); // buffered to prevent issues associated with repeated calls within short time
      battv = lc.cellVoltage();
    }
    else
    {
      // Error values
      battpct = 10000;
      battv = 10000;
    }

    #ifdef MQTTLOG
      int sensor_pub = 0;
      int device_pub = 0;
      sensor_pub = mqttSensorUpdate(averageCO2, averageTempF, averageHumidity);
      device_pub = mqttDeviceInfoUpdate(battpct, battv, rssi);
      if (sensor_pub && device_pub) {
        upd_flags += "M";
      }
    #endif

    #ifdef DWEET
      post_dweet(averageCO2, averageTempF, averageHumidity, battpct, battv, rssi);
      upd_flags += "D";
    #endif

    #ifdef INFLUX
      // Returns true if successful
      if (post_influx(averageCO2, averageTempF, averageHumidity, battpct, battv, rssi)) {
        upd_flags += "I";
      }
    #endif
  }

  // Update the screen if available
  if (upd_flags == "") {
    // None of the services succeeded (gasp!)
    infoScreen(aq_network.dateTimeString());
  } else {
    infoScreen("[+" + upd_flags + "] " + aq_network.dateTimeString());
  }
  deepSleep();
}

void loop() {}

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
#ifdef SCREEN
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
#endif
  aq_network.networkStop();
  // SCD40 only
  envSensor.stopPeriodicMeasurement();
  envSensor.powerDown();

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // Rev B board is LOW to enable
  // Rev C board is HIGH to enable
  digitalWrite(PIN_I2C_POWER, HIGH);
#endif

  nvStorage.end();
  esp_sleep_enable_timer_wakeup(SAMPLE_INTERVAL * SAMPLE_INTERVAL_ESP_MODIFIER);
  esp_deep_sleep_start();
}

bool getOWMWeather()
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

      jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
      if (jsonBuffer=="HTTP GET error"){
        return false;
      }
      debugMessage(jsonBuffer);

      DynamicJsonDocument doc(2000);
      //StaticJsonDocument<2000> doc;

      DeserializationError error = deserializeJson(doc, jsonBuffer);

      if (error)
      {
        debugMessage("deserializeJson failed: ");
        debugMessage(String(error.c_str()));
        return false;
      }
      
      int code = (int) doc["cod"];
      if(code != 200)
      {
        debugMessage(String("OpenWeatherMap error: ") + (const char *)doc["message"]);
        return false;
      }

      owmCurrentData.lat = (float) doc["coord"]["lat"];
      owmCurrentData.lon = (float) doc["coord"]["lon"];
      
      owmCurrentData.main = (const char*) doc["weather"][0]["main"];  
      owmCurrentData.description = (const char*) doc["weather"][0]["description"];
      owmCurrentData.icon = (const char*) doc["weather"][0]["icon"];
      
      owmCurrentData.cityName = (const char*) doc["name"];
      owmCurrentData.visibility = (uint16_t) doc["visibility"];
      owmCurrentData.timezone = (time_t) doc["timezone"];
      
      owmCurrentData.country = (const char*) doc["sys"]["country"];
      owmCurrentData.observationTime = (time_t) doc["dt"];
      owmCurrentData.sunrise = (time_t) doc["sys"]["sunrise"];
      owmCurrentData.sunset = (time_t) doc["sys"]["sunset"];
      
      owmCurrentData.temp = (float) doc["main"]["temp"];
      owmCurrentData.pressure = (uint16_t) doc["main"]["pressure"];
      owmCurrentData.humidity = (uint8_t) doc["main"]["humidity"];
      owmCurrentData.tempMin = (float) doc["main"]["temp_min"];
      owmCurrentData.tempMax = (float) doc["main"]["temp_max"];

      owmCurrentData.windSpeed = (float) doc["wind"]["speed"];
      owmCurrentData.windDeg = (float) doc["wind"]["deg"];
      debugMessage(String("Open Weather Map returned: ") + owmCurrentData.temp + "F, " + owmCurrentData.humidity + "%");
      return true;
    }
  #endif
  return false;
}

bool getOWMAQI()
{
  // retrieves weather from Open Weather Map APIs and stores data to environment global
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (internetAvailable)
    // and internet is verified
    {
      String jsonBuffer;

      // Get local AQI
      String serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;

      jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
      if (jsonBuffer=="HTTP GET error"){
        return false;
      }
      debugMessage(jsonBuffer);

      DynamicJsonDocument doc(384);
      //StaticJsonDocument<384> doc;

      DeserializationError error = deserializeJson(doc, jsonBuffer);

      if (error)
      {
        debugMessage("deserializeJson failed: ");
        debugMessage(String(error.c_str()));
        return false;
      }
    
      int code = (int) doc["cod"];
      if(code != 200)
      {
        debugMessage(String("OpenWeatherMap error: ") + (const char *)doc["message"]);
        return false;
      }

      // owmAirQuality.lon = (float) doc1["coord"]["lon"];
      // owmAirQuality.lat = (float) doc1["coord"]["lat"];

      JsonObject list_0 = doc["list"][0];

      owmAirQuality.aqi = list_0["main"]["aqi"];

      // JsonObject list_0_components = list_0["components"];
      // owmAirQuality.co = (float) list_0_components["co"]
      // owmAirQuality.no = (float) list_0_components["no"]
      // owmAirQuality.no2 = (float) list_0_components["no2"]
      // owmAirQuality.o3 = (float) list_0_components["o3"]
      // owmAirQuality.so2 = (float) list_0_components["so2"]
      // owmAirQuality.pm2_5 = (float) list_0_components["pm2_5"]
      // owmAirQuality.pm10 = (float) list_0_components["pm10"]
      // owmAirQuality.nh3 = (float) list_0_components["nh3"]
      debugMessage(String("Open Weather Map returned: ") + owmAirQuality.aqi + " AQI");
      return true;
    }
  #endif
  return false;
}

void alertScreen(String messageText)
// Display critical error message on screen
{
#ifdef SCREEN
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(20, (display.height() / 2 + 6));
  display.print(messageText);

  //update display
  display.display();
#endif
}

void infoScreen(String messageText)
// Display environmental information on screen
{
#ifdef SCREEN
  String aqiLabels[5] = { "Good", "Fair", "Moderate", "Poor", "Very Poor" };
  String co2Labels[3]={"Good","Poor","Bad"};
  int co2range;
  
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // borders
  // ThinkInk 2.9" epd is 296x128 pixels
  // label border
  display.drawLine(0,(display.height()/8),display.width(),(display.height()/8),EPD_GRAY);
  // horizontal temp/humidity border
  // display.drawLine(0,(display.height()*3/8),display.width(),(display.height()*3/8),EPD_GRAY);
  // horizontal humidity/AQ border
  // display.drawLine(0,(display.height()*5/8),display.width(),(display.height()*5/8),EPD_GRAY);
  // horizontal AQ/message text border
  display.drawLine(0,(display.height()*7/8),display.width(),(display.height()*7/8),EPD_GRAY);
  // splitting sensor vs. outside values
  display.drawLine((display.width()/2),0,(display.width()/2),(display.height()*7/8),EPD_GRAY);
  // battery status
  screenBatteryStatus();

  // indoor and outdoor labels
  display.setFont();
  display.setCursor(((display.width()/4)-10),((display.height()*1/8)-11));
  display.print("Here");
  display.setCursor(((display.width()*3/4-12)),((display.height()*1/8)-11));
  display.print("Outside");

  display.setFont(&FreeSans9pt7b);

  // indoor info
  int temperatureDelta = ((int)(sensorData.internalTempF +0.5)) - ((int) (averageTempF + 0.5));
  int humidityDelta = ((int)(sensorData.internalHumidity +0.5)) - ((int) (averageHumidity + 0.5));

  display.setCursor(5,((display.height()*3/8)-10));
  display.print(String("TMP ") + (int)(sensorData.internalTempF+0.5) + "F");
  if (temperatureDelta!=0)
  {
    if(temperatureDelta>0)
    {
      // upward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(90,((display.height()*3/8)-10),110,((display.height()*3/8)-10),100,(((display.height()*3/8)-10)-9), EPD_BLACK);
    }
    else
    {
      // (left pt, right pt, bottom pt)
      display.fillTriangle(90,(((display.height()*3/8)-10)-9),110,(((display.height()*3/8)-10)-9),100,((display.height()*3/8)-10), EPD_BLACK);
    }
    display.setCursor(112,((display.height()*3/8)-10));
    display.print(abs(temperatureDelta));
  }

  display.setCursor(5,((display.height()*5/8)-10));
  display.print(String("HMD ") + (int)(sensorData.internalHumidity+0.5) + "%");
  if (humidityDelta!=0)
  {
    if(humidityDelta>0)
    {
      // upward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(90,((display.height()*5/8)-10),110,((display.height()*5/8)-10),100,(((display.height()*5/8)-10)-9), EPD_BLACK);
    }
    else
    {
      // (left pt, right pt, bottom pt)
      display.fillTriangle(90,(((display.height()*5/8)-10)-9),110,(((display.height()*5/8)-10)-9),100,((display.height()*5/8)-10), EPD_BLACK);
    }
    display.setCursor(112,((display.height()*5/8)-10));
    display.print(abs(humidityDelta));
  }

  if (sensorData.internalCO2!=10000)
  {
    co2range = 2;
    if (sensorData.internalCO2<1001)
    {co2range = 0;}
    else if ((sensorData.internalCO2>1000)&&(sensorData.internalCO2<2001))
    {co2range = 1;}
    display.setCursor(5,((display.height()*7/8)-10));
    display.print(String("C02 ") + co2Labels[co2range]);
    if ((sensorData.internalCO2-averageCO2)!=0)
    {
      if(sensorData.internalCO2-averageCO2>0)
      {
          // upward triangle (left pt, right pt, bottom pt)
          display.fillTriangle(90,((display.height()*7/8)-10),110,((display.height()*7/8)-10),100,(((display.height()*7/8)-10)-9), EPD_BLACK);
      }
        else
      {
          // (left pt, right pt, bottom pt)
          display.fillTriangle(90,(((display.height()*7/8)-10)-9),110,(((display.height()*7/8)-10)-9),100,((display.height()*7/8)-10), EPD_BLACK);
      }
      display.setCursor(112,((display.height()*7/8)-10));
      display.print(abs(sensorData.internalCO2 - averageCO2));
    }
  }

  // outdoor info
  if (owmCurrentData.temp!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
    display.print(String("TMP ") + owmCurrentData.temp + "F");
  }
  if (owmCurrentData.humidity!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
    display.print(String("HMD ") + owmCurrentData.humidity + "%");
  }
  // air quality index (AQI)
  if (owmAirQuality.aqi!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*7/8)-10));
    display.print("AQI ");
    display.print(aqiLabels[(owmAirQuality.aqi-1)]);
  }

  // message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()-10));
  display.print(messageText);

  //update display
  display.display();
  debugMessage("Screen updated");
#endif
}

void initBattery() {
  if (lc.begin())
  // Check battery monitoring status
  {
    debugMessage("Battery monitor ready");
    lc.setPackAPA(BATTERY_APA);
    batteryAvailable = true;
  } else {
    debugMessage("Battery monitor not detected");
  }
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
// used in XXXScreen() routines
{
#ifdef SCREEN
  if (batteryAvailable) {
    // render battery percentage to screen

    int barHeight = 10;
    int barWidth = 28;
    // stored so we don't call the function twice in the routine
    float percent = lc.cellPercent();
    debugMessage("Battery is at " + String(percent) + " percent capacity");
    debugMessage("Battery voltage: " + String(lc.cellVoltage()) + " v");

    //calculate fill
    display.fillRect((display.width() - 33), ((display.height() * 7 / 8) + 4), (int((percent / 100) * barWidth)), barHeight, EPD_GRAY);
    // border
    display.drawRect((display.width() - 33), ((display.height() * 7 / 8) + 4), barWidth, barHeight, EPD_BLACK);
  }
#endif
}

int initSensor() 
{
  //SCD40
  uint16_t error;
  char errorMessage[256];

  Wire.begin();
  envSensor.begin(Wire);
  envSensor.wakeUp();
  envSensor.setSensorAltitude(SITE_ALTITUDE); // optimizes CO2 reading

  error = envSensor.startPeriodicMeasurement();
  if (error) 
  {
    // Failed to initialize SCD40
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + "executing SCD40 startPeriodicMeasurement()");
    return error;
  } 
  else 
  {
    delay(5000);  // Give SCD40 time to warm up
    return 0;     // error = 0 in this case
  }

  // ATHX0, BME280
  // if (envSensor.begin())
  // {
  //   // ID of 0x56-0x58 or 0x60 is a BME 280, 0x61 is BME680, 0x77 is BME280 on ESP32S2 Feather
  //   debugMessage(String("Environment sensor ready, ID is: ")+envSensor.sensorID());
  //   return 0;
  // }
  // else
  // {
  //   return 1;
  // }
}

uint16_t readSensor()
// reads environment sensor and stores data to environment global
{
  // AHTX0, BME280
  // sensors_event_t temp_event, humidity_event;
  // envSensor_temp->getEvent(&temp_event);
  // envSensor_humidity->getEvent(&humidity_event);
 
  // sensorData.internalTempF = (temp_event.temperature * 1.8) +32;
  // sensorData.internalHumidity = humidity_event.relative_humidity;
  // sensorData.internalCO2 = 10000;

  // SCD40
  uint16_t error;
  char errorMessage[256];

  error = envSensor.readMeasurement(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity);
  if (error) 
  {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + "executing SCD40 readMeasurement()");
    return error;
  }
  //convert C to F for temp
  sensorData.internalTempF = (sensorData.internalTempF * 1.8) + 32;

  debugMessage(String("environment sensor values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
  return 0;
}

int readNVStorage() {
  int storedCounter;
  float storedTempF;
  float storedHumidity;

  nvStorage.begin("air-quality", false);
  // get previously stored values. If they don't exist, create them as zero
  storedCounter = nvStorage.getInt("counter", 1);
  // read value or insert current sensor reading if this is the first read from nv storage
  storedTempF = nvStorage.getFloat("temp", sensorData.internalTempF);
  // BME280 often issues nan when not configured properly
  if (isnan(storedTempF)) {
    // bad value, replace with current temp
    averageTempF = (sensorData.internalTempF * storedCounter);
    debugMessage("Unexpected tempF value in nv storage replaced with multiple of current temperature");
  } else {
    // good value, pass it along
    averageTempF = storedTempF;
  }
  storedHumidity = nvStorage.getFloat("humidity", sensorData.internalHumidity);
  if (isnan(storedHumidity)) {
    // bad value, replace with current temp
    averageHumidity = (sensorData.internalHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity");
  } else {
    // good value, pass it along
    averageHumidity = storedHumidity;
  }
  averageCO2 = nvStorage.getUInt("co2", sensorData.internalCO2);
  debugMessage(String("Intermediate values FROM nv storage: Temp:") + averageTempF + ", Humidity:" + averageHumidity + ", CO2:" + averageCO2);
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter);
  return storedCounter;
}

String getMeteoconIcon(String icon)
{
  // clear sky
  // 01d
  if (icon == "01d")  {
    return "B";
  }
  // 01n
  if (icon == "01n")  {
    return "C";
  }
  // few clouds
  // 02d
  if (icon == "02d")  {
    return "H";
  }
  // 02n
  if (icon == "02n")  {
    return "4";
  }
  // scattered clouds
  // 03d
  if (icon == "03d")  {
    return "N";
  }
  // 03n
  if (icon == "03n")  {
    return "5";
  }
  // broken clouds
  // 04d
  if (icon == "04d")  {
    return "Y";
  }
  // 04n
  if (icon == "04n")  {
    return "%";
  }
  // shower rain
  // 09d
  if (icon == "09d")  {
    return "R";
  }
  // 09n
  if (icon == "09n")  {
    return "8";
  }
  // rain
  // 10d
  if (icon == "10d")  {
    return "Q";
  }
  // 10n
  if (icon == "10n")  {
    return "7";
  }
  // thunderstorm
  // 11d
  if (icon == "11d")  {
    return "P";
  }
  // 11n
  if (icon == "11n")  {
    return "6";
  }
  // snow
  // 13d
  if (icon == "13d")  {
    return "W";
  }
  // 13n
  if (icon == "13n")  {
    return "#";
  }
  // mist
  // 50d
  if (icon == "50d")  {
    return "M";
  }
  // 50n
  if (icon == "50n")  {
    return "M";
  }
  // Nothing matched: N/A
  return ")";
}