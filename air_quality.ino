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

// environment characteristics
typedef struct
{
  float internalTempF;
  float internalHumidity;
  uint16_t internalCO2;
  int extTemperature;
  int extHumidity;
  int extAQI;
} envData;

// global for air characteristics
envData sensorData;

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
// #include <Adafruit_Si7021.h>
// Adafruit_Si7021 envSensor = Adafruit_Si7021();

// BME280
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>
// altitude calculation support
// #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BME280 envSensor;

// Battery voltage sensor
#include <Adafruit_LC709203F.h>
Adafruit_LC709203F lc;

// screen support
#ifdef SCREEN
// Adafruit MagTag
#include <Adafruit_ThinkInk.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
// These are set in the MagTag board definition. Uncomment and change for other epd.
// #define EPD_RESET   6   // can set to -1 and share with chip Reset (can't deep sleep)
// #define EPD_DC      7   // can be any pin, but required!
// #define EPD_CS      8   // can be any pin, but required!
#define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
#define EPD_BUSY    5   // can set to -1 to not use a pin (will wait a fixed delay)
ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
// colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK
#endif

#include "ArduinoJson.h"  // Needed by getWeather()

#ifdef INFLUX
extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_p, float battery_v);
#endif

#ifdef DWEET
extern void post_dweet(uint16_t co2, float tempF, float humidity, float battpct, float battv);
#endif

#ifdef MQTTLOG
extern void mqttConnect();
extern int mqttBatteryUpdate(float cellPercent);
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

  // Initialize environmental sensor.  Returns non-zero if initialization fails
  if (initSensor()) {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    alertScreen("Env sensor not detected");
    deepSleep();
  }

  // Environmental sensor available, so fetch values
  int sampleCounter;
  readSensor();
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
  getWeather();

  String upd_flags = "";  // To indicate whether services succeeded
  if (internetAvailable)
  // and internet is verified
  {
#ifdef MQTTLOG
    int sensor_pub = 0;
    int batt_pub = 0;
    sensor_pub = mqttSensorUpdate(averageCO2, averageTempF, averageHumidity);
    batt_pub = mqttBatteryUpdate(lc.cellPercent());
    if (sensor_pub && batt_pub) {
      upd_flags += "M";
    }
#endif

#ifdef DWEET
    float battpct = lc.cellPercent();
    float battv = lc.cellVoltage();
    post_dweet(averageCO2, averageTempF, averageHumidity, battpct, battv);
    upd_flags += "D";
#endif

#ifdef INFLUX
    float battpct = lc.cellPercent();
    float battv = lc.cellVoltage();
    // Returns true if successful
    if (post_influx(averageCO2, averageTempF, averageHumidity, battpct, battv)) {
      upd_flags += "I";
    }
#endif
  }

  // Update the screen if available
  if (upd_flags == "") {
    // None of the services succeeded (gasp!)
    infoScreen("Updated " + aq_network.dateTimeString());
  } else {
    infoScreen("Updated [+" + upd_flags + "] " + aq_network.dateTimeString());
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

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // Rev B board is LOW to enable
  // Rev C board is HIGH to enable
  digitalWrite(PIN_I2C_POWER, HIGH);
#endif

  nvStorage.end();
  esp_sleep_enable_timer_wakeup(SAMPLE_INTERVAL * SAMPLE_INTERVAL_ESP_MODIFIER);
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

    jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
    //debugMessage(jsonBuffer);

    StaticJsonDocument<1024> doc;

    DeserializationError httpError = deserializeJson(doc, jsonBuffer);

    if (httpError) {
      debugMessage("Unable to parse OWM weather JSON object");
      debugMessage(String(httpError.c_str()));
      sensorData.extTemperature = 10000;
      sensorData.extHumidity = 10000;
    } else {
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
      sensorData.extHumidity = main["humidity"];  // 81

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
    }

    // Get local AQI
    serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;

    jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
    //debugMessage(jsonBuffer);

    StaticJsonDocument<384> doc1;

    httpError = deserializeJson(doc1, jsonBuffer);

    if (httpError) {
      debugMessage("Unable to parse OWM air quality JSON object");
      debugMessage(String(httpError.c_str()));
      sensorData.extAQI = 10000;
    } else {
      // double coord_lon = doc1["coord"]["lon"]; // -122.2221
      // float coord_lat = doc1["coord"]["lat"]; // 47.5707

      JsonObject list_0 = doc1["list"][0];

      sensorData.extAQI = list_0["main"]["aqi"];  // 2

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
  } else {
    sensorData.extTemperature = 10000;
    sensorData.extHumidity = 10000;
    sensorData.extAQI = 10000;
  }
#else
  sensorData.extTemperature = 10000;
  sensorData.extHumidity = 10000;
  sensorData.extAQI = 10000;
#endif
  debugMessage(String("Open Weather Map returned: ") + sensorData.extTemperature + "F, " + sensorData.extHumidity + "%, " + sensorData.extAQI + " AQI");
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
  if (sensorData.extTemperature!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*3/8)-10));
    display.print(String("TMP ") + sensorData.extTemperature + "F");
  }
  if (sensorData.extHumidity!=10000)
  {
    display.setCursor(((display.width()/2)+5),((display.height()*5/8)-10));
    display.print(String("HMD ") + sensorData.extHumidity + "%");
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

int initSensor() {
  //SCD40
  uint16_t error;
  char errorMessage[256];

  Wire.begin();
  envSensor.begin(Wire);

  error = envSensor.startPeriodicMeasurement();
  if (error) {
    // Failed to initialize SCD40
    debugMessage("Error executing SCD40 startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    debugMessage(errorMessage);
    return error;
  } else {
    delay(5000);  // Give SCD40 time to warm up
    return 0;     // error = 0 in this case
  }

  // ATHX0, SiH7021, BME280
  // if (envSensor.begin())
  // {
  //   return 0;
  // }
  // else
  // {
  //   // ID of 0x56-0x58 or 0x60 is a BME 280, 0x61 is BME680, 0x77 is BME280 on ESP32S2 Feather
  //   debugMessage(String("Environment sensorID is: 0x")+envSensor.sensorID());
  //   return 1;
  // }
}

void readSensor()
// reads environment sensor and stores data to environment global
{
  // SCD40
  uint8_t error = envSensor.readMeasurement(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity);
  if (error) {
    debugMessage("Error reading SCD40 sensor");
    deepSleep();
  }
  // convert C to F for temp
  sensorData.internalTempF = (sensorData.internalTempF * 1.8) + 32;
  debugMessage(String("SCD40 environment sensor values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");

  // AHTX0
  // sensors_event_t sensorHumidity, sensorTemp;
  // envSensor.getEvent(&sensorHumidity, &sensorTemp);
  // sensorData.internalTempF = sensorTemp.temperature;
  // sensorData.internalHumidity = sensorHumidity.relative_humidity;
  // sensorData.internalCO2 = 10000;
  // debugMessage(String("AHTX0 environment sensor values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");


  // bme280, SiH7021
  // sensorData.internalTempF = (envSensor.readTemperature()*1.8)+32;
  // sensorData.internalHumidity = envSensor.readHumidity();
  // sensorData.internalCO2 = 10000;
  // debugMessage(String("BME280/SiH7021 environment sensor values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
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