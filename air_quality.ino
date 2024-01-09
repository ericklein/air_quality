/*
  Project:      air_quality
  Description:  Regularly sample and log temperature, humidity, and if sensor available, co2 levels
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// accumulating sensor readings
float averageTempF;
float averageHumidity;
uint16_t averageCO2;

// environment sensor data
typedef struct {
  float ambientHumidity;  // RH [%]
  float ambientTemperatureF;
  uint16_t ambientCO2;
} envData;
envData sensorData;  // global variable for environment sensor data

// stores CO2 sample values for sparkline
uint16_t co2Samples[sensorSampleSize];

// hardware status data
typedef struct {
  float batteryPercent;
  float batteryVoltage;
  uint8_t rssi;
} hdweData;
hdweData hardwareData;  // global variable for hardware characteristics

// OpenWeatherMap Current data
typedef struct {
  // float lon;              // "lon": 8.54
  // float lat;              // "lat": 47.37
  // uint16_t weatherId;     // "id": 521
  // String main;            // "main": "Rain"
  // String description;     // "description": "shower rain"
  String icon;               // "icon": "09d"
  float tempF;                // "temp": 90.56, in F (API request for imperial units)
  // uint16_t pressure;      // "pressure": 1013, in hPa
  uint16_t humidity;         // "humidity": 87, in RH%
  // float tempMin;          // "temp_min": 89.15
  // float tempMax;          // "temp_max": 92.15
  // uint16_t visibility;    // visibility: 10000, in meters
  // float windSpeed;        // "wind": {"speed": 1.5}, in meters/s
  // float windDeg;          // "wind": {deg: 226.505}
  // uint8_t clouds;         // "clouds": {"all": 90}, in %
  // time_t observationTime; // "dt": 1527015000, in UTC
  // String country;         // "country": "CH"
  // time_t sunrise;         // "sunrise": 1526960448, in UTC
  // time_t sunset;          // "sunset": 1527015901, in UTC
  String cityName;           // "name": "Zurich"
  // time_t timezone;        // shift in seconds from UTC
} OpenWeatherMapCurrentData;
OpenWeatherMapCurrentData owmCurrentData;  // global variable for OWM current data

// OpenWeatherMap Air Quality data
typedef struct {
  // float lon;    // "lon": 8.54
  // float lat;    // "lat": 47.37
  uint16_t aqi;  // "aqi": 2  [European standards body value]
  // float co;     // "co": 453.95, in μg/m3
  // float no;     // "no": 0.47, in μg/m3
  // float no2;    // "no2": 52.09, in μg/m3
  // float o3;     // "o3": 17.17, in μg/m3
  // float so2;    // "so2": 7.51, in μg/m3
  float pm25;  // "pm2.5": 8.04, in μg/m3
  // float pm10;   // "pm10": 9.96, in μg/m3
  // float nh3;    // "nh3": 0.86, in μg/m3
} OpenWeatherMapAirQuality;
OpenWeatherMapAirQuality owmAirQuality;  // global variable for OWM current data

#ifndef SIMULATE_SENSOR
  // initialize environment sensor
  #ifdef SCD40
    // SCD40; temp, humidity, CO2
    #include <SensirionI2CScd4x.h>
    SensirionI2CScd4x envSensor;
  #elif defined(BME280)
    #include <Adafruit_BME280.h>
    Adafruit_BME280 envSensor;  // i2c interface
  #elif defined(AHTXX)
    #include <Adafruit_AHTX0.h>
    Adafruit_AHTX0 envSensor;
  #endif
  #if defined(BME280) || defined(AHTXX)
    Adafruit_Sensor *envSensorTemp = envSensor.getTemperatureSensor();
    Adafruit_Sensor *envSensorHumidity = envSensor.getHumiditySensor();
  #endif

  // Battery voltage sensor
  #include <Adafruit_LC709203F.h>
  Adafruit_LC709203F lc;
#endif

// screen support
#ifdef SCREEN
  #include <GxEPD2_BW.h>
  GxEPD2_BW<GxEPD2_290_T5D, GxEPD2_290_T5D::HEIGHT> display(GxEPD2_290_T5D(EPD_CS, EPD_DC, EPD_RESET, EPD_BUSY)); // GDEW029T5D 128x296, UC8151D

  #include "Fonts/meteocons20pt7b.h"
  #include "Fonts/meteocons16pt7b.h"
  #include "Fonts/meteocons12pt7b.h"

  #include <Fonts/FreeSans9pt7b.h>
  #include <Fonts/FreeSans12pt7b.h>
  #include <Fonts/FreeSans18pt7b.h>

  // Special glyphs for the UI
  #include "Fonts/glyphs.h"

  // screen layout assists
  const uint16_t xMargins = 5;
  const uint16_t xOutdoorMargin = ((display.width() / 2) + xMargins);
  const uint16_t yMargins = 2;
  const uint16_t yCO2 = 20;
  const uint16_t ySparkline = 40;
  const uint16_t yTemperature = 100;
  const uint16_t yStatus = (display.height() * 7 / 8);
  const uint16_t sparklineHeight = 40;
  const uint16_t batteryBarWidth = 28;
  const uint16_t batteryBarHeight = 10;
  const uint16_t wifiBarWidth = 3;
  const uint16_t wifiBarHeightIncrement = 2;
  const uint16_t wifiBarSpacing = 5;
#endif

// activate only if using network data endpoints
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
  #if defined(ESP8266)
    #include <ESP8266WiFi.h>
  #elif defined(ESP32)
    #include <WiFi.h>
  #elif
    #include <WiFiNINA.h> // PyPortal
  #endif
#endif

// activate WiFi and associated services if using network data endpoints
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT) || defined(DWEET)
  WiFiClient client;
  // NTP setup using Esperiff library
  #include <time.h>
  #ifdef SCREEN
    // Libraries needed to access Open Weather Map
    #include <HTTPClient.h>
    #include "ArduinoJson.h"  // Needed by OWM retrieval routines
  #endif
#endif

#ifdef INFLUX
extern boolean post_influx(uint16_t co2, float temperatureF, float humidity, float batteryVoltage, uint8_t rssi);
#endif

#ifdef DWEET
extern void post_dweet(uint16_t co2, float temperatureF, float humidity, float batteryVoltage, uint8_t rssi);
#endif

#ifdef MQTT
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  // Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID);
  
  extern bool mqttDeviceWiFiUpdate(uint8_t rssi);
  extern bool mqttDeviceBatteryUpdate(float batteryVoltage);
  extern bool mqttSensorTemperatureFUpdate(float temperatureF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  extern bool mqttSensorCO2Update(uint16_t co2);
  #ifdef HASSIO_MQTT
    extern void hassio_mqtt_publish(uint16_t co2, float temperatureF, float humidity, float batteryVoltage);
  #endif
#endif

void setup()
// One time run of code, then deep sleep
{
// handle Serial first so debugMessage() works
#ifdef DEBUG
  Serial.begin(115200);
  // wait for serial port connection
  while (!Serial)
    ;
#endif

debugMessage("Air Quality Device ID: " + String(DEVICE_ID), 1);
debugMessage(String("Sample interval is ") + sensorSampleInterval + " seconds", 2);
debugMessage(String("Number of samples before reporting is ") + sensorSampleSize, 2);
debugMessage(String("Internet service reconnect delay is ") + networkConnectAttemptInterval + " seconds", 2);
#ifdef DWEET
  debugMessage("Dweet device: " + String(DWEET_DEVICE), 2);
#endif

hardwareData.batteryVoltage = 0;  // 0 = no battery attached
hardwareData.rssi = 0;            // 0 = no WiFi

powerI2CEnable();

#ifdef SCREEN
  // there is no way to query screen for status
  display.init(115200);
  debugMessage("screen initialized as mono", 1);
  display.setTextWrap(false);
#endif

  // Initialize environmental sensor
  if (!sensorInit()) {
    screenAlert("Env sensor not detected");
    // This error often occurs after a firmware flash and then resetting the board
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(hardwareRebootInterval);
  }

  // Environmental sensor available, so fetch values
  uint8_t sampleCounter;
  if (!sensorRead()) {
    // hard coded for SCD40 as there is no way to read error condition on other sensors
    debugMessage("SCD40 returned no/bad data", 1);
    screenAlert("SCD40 read issue");
    powerDisable(hardwareRebootInterval);
  }
  sampleCounter = nvStorageRead();
  sampleCounter++;

  if (sampleCounter < sensorSampleSize)
  // add to the accumulating, intermediate sensor values and sleep device
  {
    nvStorageWrite(sampleCounter, sensorData.ambientTemperatureF + averageTempF, sensorData.ambientHumidity + averageHumidity, sensorData.ambientCO2);
    powerDisable(sensorSampleInterval);
  }

  // sampleCounter == sensorSampleSize, so average values for reporting

  // add in most recent sample then average
  averageTempF = ((sensorData.ambientTemperatureF + averageTempF) / (sensorSampleSize + 1));
  averageHumidity = ((sensorData.ambientHumidity + averageHumidity) / (sensorSampleSize + 1));

  // aggregate stored, rolling CO2 values
  for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
    averageCO2 = averageCO2 + co2Samples[loop];
  }

  // add in most recent sample then average
  averageCO2 = uint16_t((sensorData.ambientCO2 + averageCO2) / (sensorSampleSize + 1));

  debugMessage(String("Averaged values Temp:") + averageTempF + "F, Humidity:" + averageHumidity + ", CO2:" + averageCO2, 2);

  batteryRead(batteryReadsPerSample);

  // Setup network connection specified in config.h
  if (networkConnect())
    hardwareData.rssi = abs(WiFi.RSSI());

  // Get local weather and air quality info from Open Weather Map
  if (!OWMCurrentWeatherDataRead()) {
    owmCurrentData.tempF = 10000;
    owmCurrentData.humidity = 10000;
  }
  networkGetTime(networkTimeZone);

  if (!OWMAirPollutionRead()) {
    owmAirQuality.aqi = 10000;
  }

  String upd_flags = "";  // To indicate whether services succeeded
  if (hardwareData.rssi != 0) {
#ifdef MQTT
    if ((mqttSensorTemperatureFUpdate(averageTempF)) && (mqttSensorHumidityUpdate(averageHumidity)) && (mqttSensorCO2Update(averageCO2)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage))) {
      upd_flags += "M";
#ifdef HASSIO_MQTT
      debugMessage("Establishing MQTT for Home Assistant", 1);
      // Either configure sensors in Home Assistant's configuration.yaml file
      // directly or attempt to do it via MQTT auto-discovery
      // hassio_mqtt_setup();  // Config for MQTT auto-discovery
      hassio_mqtt_publish(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity, hardwareData.batteryVoltage);
#endif
    }
#endif

#ifdef DWEET
    post_dweet(averageCO2, averageTempF, averageHumidity, hardwareData.batteryVoltage, hardwareData.rssi);
    upd_flags += "D";
#endif

#ifdef INFLUX
    // Returns true if successful
    if (post_influx(averageCO2, averageTempF, averageHumidity, hardwareData.batteryVoltage, hardwareData.rssi)) {
      upd_flags += "I";
    }
#endif

    if (upd_flags == "") {
      // External data services not updated but we have network time
      screenInfo(dateTimeString("short"));
    } else {
      // External data services not updated and we have network time
      screenInfo("[+" + upd_flags + "] " + dateTimeString("short"));
    }
  } else {
// no internet connection, update screen with sensor data only
#ifdef DEBUG
    screenInfo("Test msg from DEBUG");
#else
    screenInfo("");
#endif
  }
  //reset nvStorage values for next recording period
  nvStorageWrite(-1, 0, 0, 0);
  debugMessage("nvStorage values reset", 2);

  powerDisable(sensorSampleInterval);
}

void loop() {}

void debugMessage(String messageText, uint8_t messageLevel)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  if (messageLevel <= DEBUG) {
    Serial.println(messageText);
    Serial.flush();  // Make sure the message gets output (before any sleeping...)
  }
#endif
}

void OWMCurrentWeatherDataSimulate()
// Simulates Open Weather Map (OWM) Current Weather data
{
  // Improvement - variable length names
  owmCurrentData.cityName = "Pleasantville";
  // Temperature
  owmCurrentData.tempF = ((random(sensorTempMin,sensorTempMax) / 100.0) * 1.8) + 32;
  // Humidity
  owmCurrentData.humidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
  // IMPROVEMENT - variable icons
  owmCurrentData.icon = "09d";
}

void OWMAirPollutionSimulate()
// Simulates Open Weather Map (OWM) Air Pollution data
{
  owmAirQuality.aqi = random(OWMAQIMin, OWMAQIMax);  // overrides error code value
  owmAirQuality.pm25 = random(OWMPM25Min, OWMPM25Max) / 100.0;
}

bool OWMCurrentWeatherDataRead()
// Gets Open Weather Map Current Weather data
{
  // compile only if screen available
  #ifdef SCREEN
    #ifdef HARDWARE_SIMULATE
      OWMAirPollutionSimulate();
      return true;
    #else
      if (hardwareData.rssi != 0) // internet connectivity?
      {
        String jsonBuffer;

        // Get local weather conditions
        String serverPath = String(OWM_SERVER) + OWM_WEATHER_PATH + OWM_LAT_LONG + "&units=imperial" + "&APPID=" + OWM_KEY;

        jsonBuffer = networkHTTPGETRequest(serverPath.c_str());
        debugMessage("Raw JSON from OWM Current Weather feed", 2);
        debugMessage(jsonBuffer, 2);
        if (jsonBuffer == "HTTP GET error") {
          return false;
        }

        DynamicJsonDocument doc(2048);

        DeserializationError error = deserializeJson(doc, jsonBuffer);

        if (error) {
          debugMessage(String("deserializeJson failed with error message: ") + error.c_str(), 1);
          return false;
        }

        uint8_t code = (uint8_t)doc["cod"];
        if (code != 200) {
          debugMessage(String("OWM error: ") + (const char *)doc["message"], 1);
          return false;
        }

        // owmCurrentData.lat = (float) doc["coord"]["lat"];
        // owmCurrentData.lon = (float) doc["coord"]["lon"];

        // owmCurrentData.main = (const char*) doc["weather"][0]["main"];
        // owmCurrentData.description = (const char*) doc["weather"][0]["description"];
        owmCurrentData.icon = (const char *)doc["weather"][0]["icon"];

        owmCurrentData.cityName = (const char *)doc["name"];
        // owmCurrentData.visibility = (uint16_t) doc["visibility"];
        // owmCurrentData.timezone = (time_t) doc["timezone"];

        // owmCurrentData.country = (const char*) doc["sys"]["country"];
        // owmCurrentData.observationTime = (time_t) doc["dt"];
        // owmCurrentData.sunrise = (time_t) doc["sys"]["sunrise"];
        // owmCurrentData.sunset = (time_t) doc["sys"]["sunset"];

        owmCurrentData.tempF = (float)doc["main"]["temp"];
        // owmCurrentData.pressure = (uint16_t) doc["main"]["pressure"];
        owmCurrentData.humidity = (uint8_t)doc["main"]["humidity"];
        // owmCurrentData.tempMin = (float) doc["main"]["temp_min"];
        // owmCurrentData.tempMax = (float) doc["main"]["temp_max"];

        // owmCurrentData.windSpeed = (float) doc["wind"]["speed"];
        // owmCurrentData.windDeg = (float) doc["wind"]["deg"];
        debugMessage(String("OWM Current Weather set: ") + owmCurrentData.tempF + "F, " + owmCurrentData.humidity + "%", 1);
        return true;
      }
    #endif
  #endif
  return false;
}

bool OWMAirPollutionRead()
// stores local air pollution info from Open Weather Map in environment global
{
  // compile only if screen available
  #ifdef SCREEN
    #ifdef HARDWARE_SIMULATE
      OWMAirPollutionSimulate();
      return true;
    #else
      if (hardwareData.rssi != 0)
      // ...and internet is verified
      {
        String jsonBuffer;

        // Get local AQI
        String serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;

        jsonBuffer = networkHTTPGETRequest(serverPath.c_str());
        debugMessage("Raw JSON from OWM AQI feed", 2);
        debugMessage(jsonBuffer, 2);
        if (jsonBuffer == "HTTP GET error") {
          return false;
        }

        DynamicJsonDocument doc(384);

        DeserializationError error = deserializeJson(doc, jsonBuffer);
        if (error) {
          debugMessage(String("deserializeJson failed with error message: ") + error.c_str(), 1);
          return false;
        }

        // owmAirQuality.lon = (float) doc["coord"]["lon"];
        // owmAirQuality.lat = (float) doc["coord"]["lat"];
        JsonObject list_0 = doc["list"][0];
        owmAirQuality.aqi = list_0["main"]["aqi"];
        JsonObject list_0_components = list_0["components"];
        // owmAirQuality.co = (float) list_0_components["co"];
        // owmAirQuality.no = (float) list_0_components["no"];
        // owmAirQuality.no2 = (float) list_0_components["no2"];
        // owmAirQuality.o3 = (float) list_0_components["o3"];
        // owmAirQuality.so2 = (float) list_0_components["so2"];
        owmAirQuality.pm25 = (float)list_0_components["pm2_5"];
        // owmAirQuality.pm10 = (float) list_0_components["pm10"];
        // owmAirQuality.nh3 = (float) list_0_components["nh3"];
        return true;
      }
    #endif
  #endif
  return false;
}

void screenAlert(String messageText)
// Display error message centered on screen
{
  #ifdef SCREEN
    debugMessage("screenAlert refresh start", 1);

    int16_t x1, y1;
    uint16_t width, height;

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSans12pt7b);
    display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &width, &height);
    if (width >= display.width()) {
      debugMessage("ERROR: screenAlert message text too long", 1);
    }

    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
      display.setCursor(display.width() / 2 - width / 2, display.height() / 2 + height / 2);
      display.print(messageText);
    }
    while (display.nextPage());

    debugMessage("screenAlert end", 1);
  #endif
}

void screenInfo(String messageText)
// Display environmental information on screen
{
  #ifdef SCREEN
    debugMessage("Starting screenInfo refresh", 1);

    display.setTextColor(GxEPD_BLACK);
    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
      // borders
      display.drawFastHLine(0, yStatus, display.width(), GxEPD_BLACK);
      // splitting sensor vs. outside values
      display.drawFastVLine((display.width() / 2), 0, yStatus, GxEPD_BLACK);

      // screen helper routines
      // draws battery in the lower right corner, -3 in first parameter accounts for battery nub
      screenHelperBatteryStatus((display.width() - xMargins - batteryBarWidth - 3), (display.height() - yMargins - batteryBarHeight), batteryBarWidth, batteryBarHeight);

      // -70 moves it to the left of the battery display
      screenHelperWiFiStatus((display.width() - xMargins - 70), (display.height() - yMargins), wifiBarWidth, wifiBarHeightIncrement, wifiBarSpacing);

      // draws any status message in the lower left corner. -8 in the first parameter accounts for fixed font height
      screenHelperStatusMessage(xMargins, (display.height() - yMargins - 8), messageText);

      // display sparkline
      screenHelperSparkLine(xMargins, ySparkline, ((display.width() / 2) - (2 * xMargins)), sparklineHeight);

      // Indoor
      // CO2 level
      // calculate CO2 value range in 400ppm bands
      uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
      co2range = constrain(co2range, 0, 4);  // filter CO2 levels above 2400

      // main line
      display.setFont(&FreeSans12pt7b);
      display.setCursor(xMargins, yCO2);
      display.print("CO");
      display.setCursor(xMargins + 50, yCO2);
      display.print(": " + String(co2Labels[co2range]));
      display.setFont(&FreeSans9pt7b);
      display.setCursor(xMargins + 35, (yCO2 + 10));
      display.print("2");
      // value line
      display.setFont();
      display.setCursor((xMargins + 88), (yCO2 + 7));
      display.print("(" + String(sensorData.ambientCO2) + ")");

      // Indoor temp
      display.setFont(&FreeSans12pt7b);
      display.setCursor(xMargins, yTemperature);
      display.print(String((uint8_t)(sensorData.ambientTemperatureF + .5)));
      display.setFont(&meteocons12pt7b);
      display.print("+");

      // Indoor humidity
      display.setFont(&FreeSans12pt7b);
      display.setCursor(xMargins + 60, yTemperature);
      display.print(String((uint8_t)(sensorData.ambientHumidity + 0.5)));
      // original icon ratio was 5:7?
      display.drawBitmap(xMargins + 90, yTemperature - 21, epd_bitmap_humidity_icon_sm4, 20, 28, GxEPD_BLACK);

      // Outside
      // location label
      display.setFont();
      display.setCursor((display.width() * 5 / 8), yMargins);
      display.print(owmCurrentData.cityName);

      // Outside temp
      if (owmCurrentData.tempF != 10000) {
        display.setFont(&FreeSans12pt7b);
        display.setCursor(xOutdoorMargin, yTemperature);
        display.print(String((uint8_t)(owmCurrentData.tempF + 0.5)));
        display.setFont(&meteocons12pt7b);
        display.print("+");
      }

      // Outside humidity
      if (owmCurrentData.humidity != 10000) {
        display.setFont(&FreeSans12pt7b);
        display.setCursor(xOutdoorMargin + 60, yTemperature);
        display.print(String((uint8_t)(owmCurrentData.humidity + 0.5)));
        display.drawBitmap(xOutdoorMargin + 90, yTemperature - 21, epd_bitmap_humidity_icon_sm4, 20, 28, GxEPD_BLACK);
      }

      // weather icon
      String weatherIcon = OWMtoMeteoconIcon(owmCurrentData.icon);
      // if getMeteoIcon doesn't have a matching symbol, skip display
      if (weatherIcon != ")") {
        // display icon
        display.setFont(&meteocons20pt7b);
        display.setCursor((display.width() * 17 / 20), (display.height() / 2) + 10);
        display.print(weatherIcon);
      }

      // Outside air quality index (AQI) + PM25 value
      if (owmAirQuality.aqi != 10000) {
        // main line
        display.setFont(&FreeSans9pt7b);
        display.setCursor(xOutdoorMargin, ySparkline - 5);
        // European standards-body AQI value
        //display.print(aqiEuropeanLabels[(owmAirQuality.aqi-1)]);

        // US standards-body AQI value
        float aqiUS = pm25toAQI(owmAirQuality.pm25);
        debugMessage(String("US AQI value is ") + aqiUS, 2);

        display.print(aqiUSALabels[aqiUSLabelValue(owmAirQuality.pm25)]);
        display.print(" AQI");
        // value line
        display.setFont();
        display.setCursor((xOutdoorMargin + 20), (ySparkline + 3));
        display.print("(" + String(owmAirQuality.pm25) + ")");
      }
    }
    while (display.nextPage());

    debugMessage("screenInfo refresh complete", 1);
  #endif
}

void screenHelperStatusMessage(uint16_t initialX, uint16_t initialY, String messageText)
// helper function for screenXXX() routines that draws a status message
// uses system default font, so text drawn x+,y+ from initialX,Y
{
// IMPROVEMENT : Screen dimension boundary checks for function parameters
#ifdef SCREEN
  display.setFont();  // resets to system default monospace font (6x8 pixels)
  display.setCursor(initialX, initialY);
  display.print(messageText);
#endif
}

void screenHelperWiFiStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeightIncrement, uint8_t barSpacing)
// helper function for screenXXX() routines that draws WiFi signal strength
{
#ifdef SCREEN
  if (hardwareData.rssi != 0) {
    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    uint8_t barCount = constrain((6 - ((hardwareData.rssi / 10) - 3)), 0, 5);
    if (barCount > 0) {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (uint8_t loop = 1; loop <= barCount; loop++) {
        display.fillRect((initialX + (loop * barSpacing)), (initialY - (loop * barHeightIncrement)), barWidth, loop * barHeightIncrement, GxEPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount + " bars", 2);
    } else {
      // you could do a visual representation of no WiFi strength here
      debugMessage("RSSI too low, no display", 1);
    }
  }
#endif
}

void screenHelperBatteryStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
// IMPROVEMENT : Screen dimension boundary checks for passed parameters
// IMPROVEMENT : Check for offscreen drawing based on passed parameters
#ifdef SCREEN
  if (hardwareData.batteryVoltage > 0) {
    // battery nub; width = 3pix, height = 60% of barHeight
    display.fillRect((initialX + barWidth), (initialY + (uint8_t(barHeight / 5))), 3, (uint8_t(barHeight * 3 / 5)), GxEPD_BLACK);
    // battery border
    display.drawRect(initialX, initialY, barWidth, barHeight, GxEPD_BLACK);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), uint8_t(0.5 + (hardwareData.batteryPercent * ((barWidth - 4) / 100.0))), (barHeight - 4), GxEPD_BLACK);
    debugMessage(String("battery percent visualized=") + hardwareData.batteryPercent + "%, " + uint8_t(0.5 + (hardwareData.batteryPercent * ((barWidth - 4) / 100.0))) + " pixels of " + (barWidth - 4) + " max", 1);
  } else
    debugMessage("No battery voltage for screenHelperBatteryStatus() to render", 1);
#endif
}

void screenHelperSparkLine(uint16_t initialX, uint16_t initialY, uint16_t xWidth, uint16_t yHeight) {
#ifdef SCREEN
  // TEST ONLY: load test CO2 values
  // testSparkLineValues(sensorSampleSize);

  uint16_t co2Min = co2Samples[0];
  uint16_t co2Max = co2Samples[0];
  // # of pixels between each samples x and y coordinates
  uint8_t xPixelStep, yPixelStep;

  uint16_t sparkLineX[sensorSampleSize], sparkLineY[sensorSampleSize];

  // horizontal distance (pixels) between each displayed co2 value
  xPixelStep = (xWidth / (sensorSampleSize - 1));

  // determine min/max of CO2 samples
  // could use recursive function but sensorSampleSize should always be relatively small
  for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
    if (co2Samples[loop] > co2Max) co2Max = co2Samples[loop];
    if (co2Samples[loop] < co2Min) co2Min = co2Samples[loop];
  }
  debugMessage(String("Max CO2 in stored sample range is ") + co2Max + ", min is " + co2Min, 2);

  // vertical distance (pixels) between each displayed co2 value
  yPixelStep = round(((co2Max - co2Min) / yHeight) + .5);

  debugMessage(String("xPixelStep is ") + xPixelStep + ", yPixelStep is " + yPixelStep, 2);

  // TEST ONLY : sparkline border box
  // display.drawRect(initialX,initialY, xWidth,yHeight, GxEPD_BLACK);

  // determine sparkline x,y values
  for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
    sparkLineX[loop] = (initialX + (loop * xPixelStep));
    sparkLineY[loop] = ((initialY + yHeight) - (uint8_t)((co2Samples[loop] - co2Min) / yPixelStep));
    // draw/extend sparkline after first value is generated
    if (loop != 0)
      display.drawLine(sparkLineX[loop - 1], sparkLineY[loop - 1], sparkLineX[loop], sparkLineY[loop], GxEPD_BLACK);
  }
  for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
    debugMessage(String("X,Y coordinates for CO2 sample ") + loop + " is " + sparkLineX[loop] + "," + sparkLineY[loop], 2);
  }
  debugMessage("sparkline drawn to screen", 1);
#endif
}

void batterySimulate()
// Simulate battery data
{
  // IMPROVEMENT: Simulate battery below SCD40 required level
  #ifdef HARDWARE_SIMULATE
    hardwareData.batteryVoltage = random(batterySimVoltageMin, batterySimVoltageMax) / 100.00;
    hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
  #endif
}

void batteryRead(uint8_t reads)
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef HARDWARE_SIMULATE
    batterySimulate();
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  #else
    // check to see if i2C monitor is available
    if (lc.begin())
    // Check battery monitoring status
    {
      debugMessage(String("Version: 0x") + lc.getICversion(), 2);
      lc.setPackAPA(BATTERY_APA);
      //lc.setThermistorB(3950);

      hardwareData.batteryPercent = lc.cellPercent();
      hardwareData.batteryVoltage = lc.cellVoltage();
      //hardwareData.batteryTemperatureF = 32 + (1.8* lc.getCellTemperature());
    }
    else 
    {
      // use supported boards to read voltage
      #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
        // modified from the Adafruit power management guide for Adafruit ESP32V2
        float accumulatedVoltage = 0.0;
        for (uint8_t loop = 0; loop < reads; loop++)
        {
          accumulatedVoltage += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
        }
        hardwareData.batteryVoltage = accumulatedVoltage/reads; // we now have the average reading
        // convert into volts  
        hardwareData.batteryVoltage *= 2;    // we divided by 2, so multiply back
        hardwareData.batteryVoltage /= 1000; // convert to volts!
        hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
      #endif
    }
    if (hardwareData.batteryVoltage != 0) {
      debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%", 1);
    }
  #endif
}

uint8_t batteryGetChargeLevel(float volts) {
  uint8_t idx = 50;
  uint8_t prev = 0;
  uint8_t half = 0;
  if (volts >= 4.2) {
    return 100;
  }
  if (volts <= 3.2) {
    return 0;
  }
  while (true) {
    half = abs(idx - prev) / 2;
    prev = idx;
    if (volts >= batteryVoltageTable[idx]) {
      idx = idx + half;
    } else {
      idx = idx - half;
    }
    if (prev == idx) {
      break;
    }
  }
  debugMessage(String("Battery percentage as int is ") + idx + "%", 1);
  return idx;
}

bool sensorInit()
// initializes environment sensor if available. Supports SCD40, ATHX0, BME280 sensors
{

  #ifdef HARDWARE_SIMULATE
    return true;
  #endif

  #ifdef SCD40
    char errorMessage[256];

    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      // these boards have two I2C ports so we have to initialize the appropriate port
      Wire1.begin();
      envSensor.begin(Wire1);
    #else
      // only one I2C port
      Wire.begin();
      envSensor.begin(Wire);
    #endif

    // IMPROVEMENT success on wakeup means we were in sleep mode
    envSensor.wakeUp();
    envSensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading

    // IMPROVEMENT measureSingleShot means we dont need this?
    uint16_t error = envSensor.startPeriodicMeasurement();
    if (error) {
      // Failed to initialize SCD40
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()", 1);
      return false;
    } else {
      debugMessage("SCD40 ready", 1);
      return true;
    }
  #else
    // ATHX0, BME280
    if (envSensor.begin()) {
      debugMessage("Environment sensor ready", 1);
      return true;
    } else {
      debugMessage("Environment sensor failed to initialize", 1);
      return false;
    }
  #endif
}

void sensorSimulate()
// Simulate environment sensor data
// Improvement - implement stable, rapid rise and fall 
{
  #ifdef HARDWARE_SIMULATE
    // Temperature
    // keep this value in C, as it is converted to F in sensorRead
    sensorData.ambientTemperatureF = random(sensorTempMin,sensorTempMax) / 100.0;
    // Humidity
    sensorData.ambientHumidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    #ifdef SCD40
      // CO2
      sensorData.ambientCO2 = random(sensorCO2Min, sensorCO2Max);
    #else
      // AHTX0, BME280
      sensorData.ambientCO2 = 10000;
    #endif
  #endif
}

bool sensorRead()
// stores environment sensor to environment global
{
  #ifdef HARDWARE_SIMULATE
    sensorSimulate();
  #else
    #ifdef SCD40
      char errorMessage[256];

      for (uint8_t loop = 1; loop <= sensorReadsPerSample; loop++) {
        // SCD40 datasheet suggests 5 second delay between SCD40 reads
        // assume sensorSampleInterval will create needed delay for loop == 1 
        if (loop > 1) delay(5000);
        // read and store data if successful
        // IMPROVEMENT measureSingleShot
        uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity);
        // handle SCD40 errors
        if (error) {
          errorToString(error, errorMessage, 256);
          debugMessage(String(errorMessage) + " during SCD4X read", 1);
          return false;
        }
        if (sensorData.ambientCO2 < 440 || sensorData.ambientCO2 > 6000) {
          debugMessage(String("SCD40 CO2 reading out of range at ") + sensorData.ambientCO2, 1);
          return false;
        }
        //convert C to F for temp
        sensorData.ambientTemperatureF = (sensorData.ambientTemperatureF * 1.8) + 32;
        debugMessage(String("SCD40 read ") + loop + " of " + sensorReadsPerSample + ": " + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm", 2);
      }
    #else
      // AHTX0, BME280
      // FIX : Can we get error conditions from this API?
      sensors_event_t humidityEvent, tempEvent;
      envSensorTemp->getEvent(&tempEvent);
      envSensorHumidity->getEvent(&humidityEvent);
      sensorData.ambientTemperatureF = (tempEvent.temperature * 1.8) + 32;
      sensorData.ambientHumidity = humidityEvent.relative_humidity;
      sensorData.ambientCO2 = 10000;
    #endif
  #endif
  debugMessage(String("Environment sensor measurement ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm", 1);
  return true;
}

uint8_t nvStorageRead()
// reads data from non-volatile storage and stores in appropriate global variables
// FIX: CO2 values only need to be read for sparkline generation when counter == sensorSampleSize, not every sample
{
  nvStorage.begin("air-quality", false);
  uint8_t storedCounter = nvStorage.getInt("counter", -1);  // counter tracks a 0 based array
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter, 2);

  // read value or insert current sensor reading if this is the first read from nv storage
  averageTempF = nvStorage.getFloat("temp", 0);
  // BME280 often issues nan when not configured properly
  if (isnan(averageTempF)) {
    // bad value, replace with current temp
    averageTempF = (sensorData.ambientTemperatureF * storedCounter);
    debugMessage("Unexpected temperatureF value in nv storage replaced with multiple of current temperature", 2);
  }

  averageHumidity = nvStorage.getFloat("humidity", 0);
  if (isnan(averageHumidity)) {
    // bad value, replace with current temp
    averageHumidity = (sensorData.ambientHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity", 2);
  }

  debugMessage(String("Intermediate values FROM nv storage: Temp:") + averageTempF + ", Humidity:" + averageHumidity, 2);

  // only deal with CO2 if you are getting data from sensor
  if (sensorData.ambientCO2 != 10000) {
    // Read CO2 array. If they don't exist, create them as 400 (CO2 floor)
    String nvStoreBaseName;
    for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
      nvStoreBaseName = "co2Sample" + String(loop);
      co2Samples[loop] = nvStorage.getLong(nvStoreBaseName.c_str(), 400);
      debugMessage(String(nvStoreBaseName) + " retrieved from nv storage is " + co2Samples[loop], 2);
    }
  }
  return storedCounter;
}

void nvStorageWrite(uint8_t storedCounter, float temperatureF, float humidity, uint16_t co2)
// temperatureF and humidity stored as running totals, CO2 stored in array for sparkline
{
  nvStorage.putInt("counter", storedCounter);
  debugMessage(String("Sample count TO nv storage is ") + storedCounter, 2);
  nvStorage.putFloat("temp", temperatureF);
  nvStorage.putFloat("humidity", humidity);
  debugMessage(String("Intermediate values TO nv storage: Temp: ") + temperatureF + ", Humidity: " + humidity, 2);
  // only deal with CO2 if there is sensor data
  if ((sensorData.ambientCO2 != 10000) && (co2 != 0)) {
    String nvStoreBaseName = "co2Sample" + String(storedCounter);
    nvStorage.putLong(nvStoreBaseName.c_str(), co2);
    debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + co2, 2);
  }
  if (co2 == 0)  // reset all the values
  {
    for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
      String nvStoreBaseName = "co2Sample" + String(loop);
      nvStorage.putLong(nvStoreBaseName.c_str(), co2);
      debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + co2, 2);
    }
  }
}

void powerI2CEnable()
// enables I2C across multiple Adafruit ESP32 variants
{
  debugMessage("powerEnable started", 1);

// enable I2C on devices with two ports
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  // ESP32 is kinda odd in that secondary ports must be manually assigned their pins with setPins()!
  Wire1.setPins(SDA1, SCL1);
  debugMessage("power on: ESP32 variant with two I2C ports", 2);
#endif

// Adafruit ESP32 I2C power management
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  // Rev B board is LOW to enable
  // Rev C board is HIGH to enable
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  debugMessage("power on: Feather ESP32S2 I2C", 1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
  // Turn on the I2C power
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  debugMessage("power on: Feather ESP32V2 I2C", 1);
#endif
}

void powerDisable(uint16_t deepSleepTime)
// Powers down hardware activated via powerEnable() then deep sleep MCU
{
  debugMessage("powerDisable started", 1);

// power down epd
#ifdef SCREEN
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  debugMessage("power off: epd", 1);
#endif

networkDisconnect();

// power down SCD40 by stopping potentially started measurement then power down SCD40
#ifdef SCD40
  uint16_t error = envSensor.stopPeriodicMeasurement();
  if (error) {
    char errorMessage[256];
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()", 1);
  }
  envSensor.powerDown();
  debugMessage("power off: SCD40", 1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
  // Turn off the I2C power
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
  debugMessage("power off: ESP32V2 I2C", 1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // Rev B board is LOW to enable
  // Rev C board is HIGH to enable
  digitalWrite(PIN_I2C_POWER, LOW);
  debugMessage("power off: ESP32S2 I2C", 1);
#endif

  esp_sleep_enable_timer_wakeup(deepSleepTime * 1000000);  // ESP microsecond modifier
  debugMessage(String("powerDisable complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds", 1);
  esp_deep_sleep_start();
}

String OWMtoMeteoconIcon(String icon)
// Maps OWM icon data to the appropropriate Meteocon font character
// https://www.alessioatzeni.com/meteocons/#:~:text=Meteocons%20is%20a%20set%20of,free%20and%20always%20will%20be.
{
  if (icon == "01d")  // 01d = sunny = Meteocon "B"
    return "B";
  if (icon == "01n")  // 01n = clear night = Meteocon "C"
    return "C";
  if (icon == "02d")  // 02d = partially sunny = Meteocon "H"
    return "H";
  if (icon == "02n")  // 02n = partially clear night = Meteocon "4"
    return "4";
  if (icon == "03d")  // 03d = clouds = Meteocon "N"
    return "N";
  if (icon == "03n")  // 03n = clouds night = Meteocon "5"
    return "5";
  if (icon == "04d")  // 04d = broken clouds = Meteocon "Y"
    return "Y";
  if (icon == "04n")  // 04n = broken night clouds = Meteocon "%"
    return "%";
  if (icon == "09d")  // 09d = rain = Meteocon "R"
    return "R";
  if (icon == "09n")  // 09n = night rain = Meteocon "8"
    return "8";
  if (icon == "10d")  // 10d = light rain = Meteocon "Q"
    return "Q";
  if (icon == "10n")  // 10n = night light rain = Meteocon "7"
    return "7";
  if (icon == "11d")  // 11d = thunderstorm = Meteocon "P"
    return "P";
  if (icon == "11n")  // 11n = night thunderstorm = Meteocon "6"
    return "6";
  if (icon == "13d")  // 13d = snow = Meteocon "W"
    return "W";
  if (icon == "13n")  // 13n = night snow = Meteocon "#"
    return "#";
  if ((icon == "50d") || (icon == "50n"))  // 50d = mist = Meteocon "M"
    return "M";
  // Nothing matched
  debugMessage("OWM icon not matched to Meteocon, why?", 1);
  return ")";
}

float pm25toAQI(float pm25)
// Converts pm25 reading to AQI using the AQI Equation
// (https://forum.airnowtech.org/t/the-aqi-equation/169)
{
  if (pm25 <= 12.0) return (fmap(pm25, 0.0, 12.0, 0.0, 50.0));
  else if (pm25 <= 35.4) return (fmap(pm25, 12.1, 35.4, 51.0, 100.0));
  else if (pm25 <= 55.4) return (fmap(pm25, 35.5, 55.4, 101.0, 150.0));
  else if (pm25 <= 150.4) return (fmap(pm25, 55.5, 150.4, 151.0, 200.0));
  else if (pm25 <= 250.4) return (fmap(pm25, 150.5, 250.4, 201.0, 300.0));
  else if (pm25 <= 500.4) return (fmap(pm25, 250.5, 500.4, 301.0, 500.0));
  else return (505.0);  // AQI above 500 not recognized
}

float fmap(float x, float xmin, float xmax, float ymin, float ymax) {
  return (ymin + ((x - xmin) * (ymax - ymin) / (xmax - xmin)));
}

uint8_t aqiUSLabelValue(float pm25)
// converts pm25 value to a 0-5 value associated with US AQI labels
{
  if (pm25 <= 12.0) return (0);
  else if (pm25 <= 35.4) return (1);
  else if (pm25 <= 55.4) return (2);
  else if (pm25 <= 150.4) return (3);
  else if (pm25 <= 250.4) return (4);
  else if (pm25 <= 500.4) return (5);
  else return (6);  // AQI above 500 not recognized
}

void testSparkLineValues(uint8_t sampleSetSize)
// generates test data to exercise the screenSparkLine function
{
  // generate test data
  for (uint8_t loop = 0; loop < sampleSetSize; loop++) {
    // standard range for indoor CO2 values
    co2Samples[loop] = random(600, 2400);
  }
}

void networkSimulate()
// Simulates successful WiFi connection data
{
  // IMPROVEMENT : simulate IP address?
  hardwareData.rssi = random(networkRSSIMin, networkRSSIMax);
}

bool networkConnect() 
// Connect to WiFi network specified in secrets.h
{
  #ifdef SIMULATE_SENSOR
    networkSimulate();
    return true;
  #endif

  // Run only if using network data endpoints
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)

    // reconnect to WiFi only if needed
    if (WiFi.status() == WL_CONNECTED) 
    {
      debugMessage("Already connected to WiFi",2);
      return true;
    }
    // set hostname has to come before WiFi.begin
    WiFi.hostname(DEVICE_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (uint8_t loop = 1; loop <= networkConnectAttemptLimit; loop++)
    // Attempts WiFi connection, and if unsuccessful, re-attempts after networkConnectAttemptInterval second delay for networkConnectAttemptLimit times
    {
      if (WiFi.status() == WL_CONNECTED) {
        hardwareData.rssi = abs(WiFi.RSSI());
        debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString(), 1);
        debugMessage(String("WiFi RSSI is: ") + hardwareData.rssi + " dBm", 1);
        return true;
      }
      debugMessage(String("Connection attempt ") + loop + " of " + networkConnectAttemptLimit + " to " + WIFI_SSID + " failed", 1);
      // use of delay() OK as this is initialization code
      delay(networkConnectAttemptInterval * 1000);  // converted into milliseconds
    }
  #endif
  return false;
}

void networkDisconnect()
// Disconnect from WiFi network
{
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT) || defined(DWEET)
  {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    debugMessage("power off: WiFi",1);
  }
  #endif
}

bool networkGetTime(String timezone)
// Set local time from NTP server specified in config.h
{
  // https://randomnerdtutorials.com/esp32-ntp-timezones-daylight-saving/

  struct tm timeinfo;

  // connect to NTP server with 0 TZ offset
  configTime(0, 0, networkNTPAddress.c_str());
  if(!getLocalTime(&timeinfo))
  {
    debugMessage("Failed to obtain time from NTP Server",1);
    return false;
  }
  // set local timezone
  setTimeZone(timezone);
  return true;
}

String networkHTTPGETRequest(const char* serverName) {
  String payload = "{}";

  HTTPClient http;

  // servername is domain name w/URL path or IP address w/URL path
  http.begin(client, serverName);

  // Send HTTP GET request
  uint16_t httpResponseCode = http.GET();

  if (httpResponseCode == HTTP_CODE_OK) {
    // HTTP reponse OK code
    payload = http.getString();
  } else {
    debugMessage("HTTP GET error code: " + httpResponseCode,1);
    payload = "HTTP GET error";
  }
  // free resources
  http.end();
  return payload;
}

void setTimeZone(String timezone)
// Set local time based on timezone set in config.h
{
  debugMessage(String("setting Timezone to ") + timezone.c_str(),2);
  setenv("TZ",networkTimeZone.c_str(),1);
  tzset();
  debugMessage(String("Local time: ") + dateTimeString("short"),1);
}

String dateTimeString(String formatType)
// Converts time into human readable string
{
  // https://cplusplus.com/reference/ctime/tm/

  String dateTime;
  struct tm timeInfo;

  if (getLocalTime(&timeInfo)) 
  {
    if (formatType == "short")
    {
      // short human readable format
      dateTime = weekDays[timeInfo.tm_wday];
      dateTime += " at ";
      if (timeInfo.tm_hour < 10) dateTime += "0";
      dateTime += timeInfo.tm_hour;
      dateTime += ":";
      if (timeInfo.tm_min < 10) dateTime += "0";
      dateTime += timeInfo.tm_min;
    }
    else if (formatType == "long")
    {
      // long human readable
      dateTime = weekDays[timeInfo.tm_wday];
      dateTime += ", ";
      if (timeInfo.tm_mon<10) dateTime += "0";
      dateTime += timeInfo.tm_mon;
      dateTime += "-";
      if (timeInfo.tm_wday<10) dateTime += "0";
      dateTime += timeInfo.tm_wday;
      dateTime += " at ";
      if (timeInfo.tm_hour<10) dateTime += "0";
      dateTime += timeInfo.tm_hour;
      dateTime += ":";
      if (timeInfo.tm_min<10) dateTime += "0";
      dateTime += timeInfo.tm_min;
    }
  }
  else dateTime = "Can't reach time service";
  return dateTime;
}