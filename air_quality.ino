/*
  Project:      air_quality
  Description:  Regularly sample, display, and log temperature, humidity, and co2 levels
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// accumulating sensor readings
// IMPROVEMENT: nvStorageRead() should return these locally
float accumulatingTempF;
float accumulatingHumidity;

// environment sensor data
typedef struct {
  float ambientTemperatureF;    // range -10C to 60C
  float ambientHumidity;        // RH [%], range 0 to 100
  uint16_t  ambientCO2;         // ppm, range 400 to 2000
} envData;
envData sensorData;

// stores CO2 sample values for sparkline
uint16_t co2Samples[sensorSampleSize];

// hardware status data
typedef struct {
  float batteryPercent;
  float batteryVoltage;
  // float batteryTemperatureF;
  uint8_t rssi;
} hdweData;
hdweData hardwareData; 

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

#ifndef HARDWARE_SIMULATE
  // instanstiate SCD4X hardware object
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x co2Sensor;

  // battery voltage sensor
  #include <Adafruit_LC709203F.h>
  Adafruit_LC709203F lc;

  // WiFi support
  #if defined(ESP8266)
    #include <ESP8266WiFi.h>
  #elif defined(ESP32)
    #include <WiFi.h>
  #endif
  #include <HTTPClient.h>
  WiFiClient client;   // used by OWM and MQTT
#endif

// e-paper display support
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

// Libraries needed to access Open Weather Map
#include "ArduinoJson.h"  // Needed by OWM retrieval routines

// internet services if using network data endpoints
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT) || defined(DWEET)
  // NTP setup using Esperiff library
  #include <time.h>
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
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);
  
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
  while (!Serial);
  debugMessage("starting Air Quality, Device ID: " + String(DEVICE_ID), 1);
  debugMessage(String("SCD4x sample interval is ") + sensorSampleInterval + " seconds", 1);
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
    debugMessage(String("Number of samples before reporting is ") + sensorSampleSize, 1);
  #endif
  debugMessage(String("Internet service reconnect delay is ") + networkConnectAttemptInterval + " seconds", 2);
  #ifdef DWEET
    debugMessage("Dweet device: " + String(DWEET_DEVICE), 2);
  #endif
#endif

// generate random numbers for every boot cycle
randomSeed(analogRead(0));

powerI2CEnable();

// initialize e-paper first to display hardware error messages
display.init(115200);
debugMessage("epd initialized", 1);
display.setTextWrap(false);
display.setRotation(displayRotation);

// Initialize SCD4X
if (!sensorCO2Init()) {
  screenAlert("No SCD4X");
  // This error often occurs right after a firmware flash and reset.
  // Hardware deep sleep typically resolves it, so quickly cycle the hardware
  powerDisable(hardwareRebootInterval);
}

  // Environmental sensor available, so fetch values
  if (!sensorCO2Read()) {
    // hard coded for SCD40 as there is no way to read error condition on other sensors
    debugMessage("SCD40 returned no/bad data", 1);
    screenAlert("SCD40 read issue");
    powerDisable(hardwareRebootInterval);
  }
  uint8_t sampleCounter = nvStorageRead();
  sampleCounter++;
  debugMessage(String("Sample count: ") + (sampleCounter) + " of " + sensorSampleSize,1);

  accumulatingTempF += sensorData.ambientTemperatureF;
  accumulatingHumidity += sensorData.ambientHumidity;

  if (sampleCounter < sensorSampleSize)
  // add to the accumulating, intermediate sensor values and sleep device
  {
    nvStorageWrite(sampleCounter, accumulatingTempF, accumulatingHumidity, sensorData.ambientCO2);
    powerDisable(sensorSampleInterval);
  }

  // this code only executes if sampleCounter == sensorSampleSize
  // average values, send to network endpoint if possible, display values, reset values, then sleep device

  float averageTempF = accumulatingTempF / sampleCounter;
  float averageHumidity = accumulatingHumidity / sampleCounter;

  // aggregate stored, rolling CO2 values
  uint16_t accumulatingCO2 = 0;
  for (uint8_t loop = 0; loop < sensorSampleSize; loop++) {
    accumulatingCO2 += co2Samples[loop];
  }
  // add in most CO2 recent sample then average
  uint16_t averageCO2 = (accumulatingCO2 + sensorData.ambientCO2) / sampleCounter;
  debugMessage(String("Averaged values Temp:") + averageTempF + "F, Humidity:" + averageHumidity + ", CO2:" + averageCO2, 1);

  if (!batteryRead(batteryReadsPerSample))
    hardwareData.batteryVoltage = 0;

  // Setup network connection specified in config.h
  networkConnect();

  networkGetTime(networkTimeZone);

  // Get local weather and air quality info from Open Weather Map
  if (!OWMCurrentWeatherRead()) {
    owmCurrentData.tempF = 10000;
  }

  if (!OWMAirPollutionRead()) {
    owmAirQuality.aqi = 10000;
  }

  String upd_flags = "";  // indicates whether network endpoint publish(es) succeeded
  if (hardwareData.rssi != 0) {
    #ifdef MQTT
        if ((mqttSensorTemperatureFUpdate(averageTempF)) && (mqttSensorHumidityUpdate(averageHumidity)) && (mqttSensorCO2Update(averageCO2)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage))) {
          upd_flags += "M";
          debugMessage("MQTT publish successful",1);
        }
        else
          debugMessage("One or more MQTT publishes unsuccessful",1);
        #ifdef HASSIO_MQTT
              debugMessage("Establishing MQTT for Home Assistant", 1);
              // Either configure sensors in Home Assistant's configuration.yaml file
              // directly or attempt to do it via MQTT auto-discovery
              // hassio_mqtt_setup();  // Config for MQTT auto-discovery
              hassio_mqtt_publish(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity, hardwareData.batteryVoltage);
        #endif
    #endif

    #ifdef DWEET
        post_dweet(averageCO2, averageTempF, averageHumidity, hardwareData.batteryVoltage, hardwareData.rssi);
        upd_flags += "D";
    #endif

    #ifdef INFLUX
        // Returns true if successful
        if (post_influx(averageCO2, averageTempF, averageHumidity, hardwareData.batteryVoltage, hardwareData.rssi)) {
          upd_flags += "I";
          debugMessage("Influx publish successful",1);
        }
        else
          debugMessage("Influx publish unsuccessful",1);
    #endif

    if (upd_flags == "") {
      // network endpoints not updated but we have network time
      screenInfo(dateTimeString("short"));
    } 
    else {
      // network endpoints updated and we have network time
      screenInfo("[+" + upd_flags + "] " + dateTimeString("short"));
    }
  } 
  else {
    // no internet connection, update screen with sensor data only
    #ifdef DEBUG
        screenInfo("DEBUG + no Internet");
    #else
        screenInfo("");
    #endif
  }
  //reset nvStorage values for next recording period
  nvStorageWrite(0, 0, 0, 0);
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

bool OWMCurrentWeatherRead()
// Gets Open Weather Map Current Weather data
{
  // compile only if screen available
  #ifdef HARDWARE_SIMULATE
    OWMCurrentWeatherSimulate();
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
      debugMessage(String("OWM Current Weather: ") + owmCurrentData.tempF + "F, " + owmCurrentData.humidity + "% RH", 1);
      return true;
    }
  #endif
  return false;
}

bool OWMAirPollutionRead()
// stores local air pollution info from Open Weather Map in environment global
{
  #ifdef HARDWARE_SIMULATE
    OWMAirPollutionSimulate();
    return true;
  #else
    // check for internet connectivity
    if (hardwareData.rssi != 0)
    {
      String jsonBuffer;

      // Get local AQI
      String serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;

      jsonBuffer = networkHTTPGETRequest(serverPath.c_str());
      debugMessage("Raw JSON from OWM Air Pollution feed", 2);
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
      debugMessage(String("OWM Air Pollution: ") + owmAirQuality.aqi + " AQI, " + owmAirQuality.pm25 + "ppm PM2.5", 1);
      return true;
    }
    return false;
  #endif
}

void screenAlert(String messageText)
// Display error message centered on screen
{
  debugMessage(String("screenAlert '") + messageText + "' start",1);

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

  debugMessage("screenAlert() end", 1);
}

void screenInfo(String messageText)
// Display environmental information on screen
{
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

    // main line
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xMargins, yCO2);
    display.print("CO");
    display.setCursor(xMargins + 50, yCO2);
    display.print(": " + String(warningLabels[co2Range(sensorData.ambientCO2)]));
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
    // do we have OWM Current data to display?
    if (owmCurrentData.tempF != 10000) {
      // location label
      display.setFont();
      display.setCursor((display.width() * 5 / 8), yMargins);
      display.print(owmCurrentData.cityName);

      // Outside temp
        display.setFont(&FreeSans12pt7b);
        display.setCursor(xOutdoorMargin, yTemperature);
        display.print(String((uint8_t)(owmCurrentData.tempF + 0.5)));
        display.setFont(&meteocons12pt7b);
        display.print("+");

      // Outside humidity
      display.setFont(&FreeSans12pt7b);
      display.setCursor(xOutdoorMargin + 60, yTemperature);
      display.print(String((uint8_t)(owmCurrentData.humidity + 0.5)));
      display.drawBitmap(xOutdoorMargin + 90, yTemperature - 21, epd_bitmap_humidity_icon_sm4, 20, 28, GxEPD_BLACK);

      // weather icon
      String weatherIcon = OWMtoMeteoconIcon(owmCurrentData.icon);
      // if getMeteoIcon doesn't have a matching symbol, skip display
      if (weatherIcon != ")") {
        // display icon
        display.setFont(&meteocons20pt7b);
        display.setCursor((display.width() * 17 / 20), (display.height() / 2) + 10);
        display.print(weatherIcon);
      }
    }

    // Outside air quality index (AQI) + PM25 value
    if (owmAirQuality.aqi != 10000) {
      // main line
      display.setFont(&FreeSans9pt7b);
      display.setCursor(xOutdoorMargin, ySparkline - 5);
      display.print(OWMAQILabels[(owmAirQuality.aqi-1)]);
      display.print(" AQI");
      // value line
      display.setFont();
      display.setCursor((xOutdoorMargin + 20), (ySparkline + 3));
      display.print(String(uint16_t(owmAirQuality.pm25+.5)) + " pm25");
    }
  }
  while (display.nextPage());

  debugMessage("screenInfo refresh complete", 1);
}

void screenHelperStatusMessage(uint16_t initialX, uint16_t initialY, String messageText)
// helper function for screenXXX() routines that draws a status message
// uses system default font, so text drawn x+,y+ from initialX,Y
{
  // IMPROVEMENT : Screen dimension boundary checks for function parameters
  display.setFont();  // resets to system default monospace font (6x8 pixels)
  display.setCursor(initialX, initialY);
  display.print(messageText);
  debugMessage(String("screenHelperStatusMessage() displayed: ") + messageText + " at X:" + initialX + ",Y:" + initialY, 2);
}

void screenHelperWiFiStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeightIncrement, uint8_t barSpacing)
// helper function for screenXXX() routines that draws WiFi signal strength
{
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
      debugMessage(String("screenHelperWiFiStatus() complete: WiFi signal strength displayed:") + barCount + " bars", 2);
    } 
    else {
      // you could do a visual representation of no WiFi strength here
      debugMessage("screenHelperWiFiStatus() complete: RSSI too low to display", 1);
    }
  }
}

void screenHelperBatteryStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for passed parameters
  // IMPROVEMENT : Check for offscreen drawing based on passed parameters
  if (hardwareData.batteryVoltage > 0) {
    // battery nub; width = 3pix, height = 60% of barHeight
    display.fillRect((initialX + barWidth), (initialY + (uint8_t(barHeight / 5))), 3, (uint8_t(barHeight * 3 / 5)), GxEPD_BLACK);
    // battery border
    display.drawRect(initialX, initialY, barWidth, barHeight, GxEPD_BLACK);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), uint8_t(0.5 + (hardwareData.batteryPercent * ((barWidth - 4) / 100.0))), (barHeight - 4), GxEPD_BLACK);
    debugMessage(String("screenHelperBatteryStatus() complete: battery percent displayed:") + hardwareData.batteryPercent + "%, " + uint8_t(0.5 + (hardwareData.batteryPercent * ((barWidth - 4) / 100.0))) + " pixels of " + (barWidth - 4) + " max", 1);
  } else
    debugMessage("screenHelperBatteryStatus() complete: No battery voltage to render", 1);
}

void screenHelperSparkLine(uint16_t initialX, uint16_t initialY, uint16_t xWidth, uint16_t yHeight) {
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
  debugMessage("screenHelperSparkLine() complete", 1);
}

// Hardware simulation routines
#ifdef HARDWARE_SIMULATE
  void OWMCurrentWeatherSimulate()
  // Simulates Open Weather Map (OWM) Current Weather data
  {
    // Improvement - variable length names
    owmCurrentData.cityName = "Pleasantville";
    // Temperature
    owmCurrentData.tempF = (random(sensorTempMinF,sensorTempMaxF) / 100.0);
    // Humidity
    owmCurrentData.humidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    // IMPROVEMENT - variable icons
    owmCurrentData.icon = "09d";
    debugMessage(String("SIMULATED OWM Current Weather: ") + owmCurrentData.tempF + "F, " + owmCurrentData.humidity + "%", 1);
  }

  void OWMAirPollutionSimulate()
  // Simulates Open Weather Map (OWM) Air Pollution data
  {
    owmAirQuality.aqi = random(OWMAQIMin, OWMAQIMax);
    owmAirQuality.pm25 = random(OWMPM25Min, OWMPM25Max) / 100.0;
    debugMessage(String("SIMULATED OWM Air Pollution PM2.5: ") + owmAirQuality.pm25 + ", AQI: " + owmAirQuality.aqi,1);
  }

  void networkSimulate()
  // Simulates successful WiFi connection data
  {
    // IMPROVEMENT : simulate IP address?
    hardwareData.rssi = random(networkRSSIMin, networkRSSIMax);
    debugMessage(String("SIMULATED WiFi RSSI: ") + hardwareData.rssi,1);
  }

  void batterySimulate()
  // Simulate battery data
  // IMPROVEMENT: Simulate battery below SCD40 required level
  {
    hardwareData.batteryVoltage = random(batterySimVoltageMin, batterySimVoltageMax) / 100.00;
    hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%", 1);  
  }

  void sensorCO2Simulate()
  // Simulate ranged data from the SCD40
  // Improvement - implement stable, rapid rise and fall 
  {
    sensorData.ambientTemperatureF = (random(sensorTempMinF,sensorTempMaxF) / 100.0);
    sensorData.ambientHumidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    sensorData.ambientCO2 = random(sensorCO2Min, sensorCO2Max);
    debugMessage(String("SIMULATED SCD40: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
  }
#endif

bool batteryRead(uint8_t reads)
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef HARDWARE_SIMULATE
    batterySimulate();
    return true;
  #else
    // is LC709203F on i2c available?
    if (lc.begin())
    {
      lc.setPackAPA(BATTERY_APA);
      //lc.setThermistorB(3950);

      hardwareData.batteryPercent = lc.cellPercent();
      hardwareData.batteryVoltage = lc.cellVoltage();
      //hardwareData.batteryTemperatureF = 32 + (1.8* lc.getCellTemperature());
    }
    else 
    {
      // read gpio pin on supported boards for battery level
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
      return true;
    }
    else
      return false;
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

bool sensorCO2Init()
// initializes SCD4X to read
{
  #ifdef HARDWARE_SIMULATE
    return true;
 #else
    char errorMessage[256];
    uint16_t error;

    Wire.begin();
    co2Sensor.begin(Wire);

    // Question : needed for MagTag version, but not ESP32V2?!
    co2Sensor.wakeUp();

    // stop potentially previously started measurement.
    error = co2Sensor.stopPeriodicMeasurement();
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String("Error: SCD4X stopPeriodicMeasurement() returned: ") + errorMessage,1);
      return false;
    }

    // Check onboard configuration settings while not in active measurement mode
    float offset;
    error = co2Sensor.getTemperatureOffset(offset);
    if (error == 0){
        error = co2Sensor.setTemperatureOffset(sensorTempCOffset);
        if (error == 0)
          debugMessage(String("Initial SCD4X temperature offset ") + offset + " ,set to " + sensorTempCOffset,2);
    }

    uint16_t sensor_altitude;
    error = co2Sensor.getSensorAltitude(sensor_altitude);
    if (error == 0){
      error = co2Sensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading
      if (error == 0)
        debugMessage(String("Initial SCD4X altitude ") + sensor_altitude + " meters, set to " + SITE_ALTITUDE,2);
    }

    // Start Measurement.  For high power mode, with a fixed update interval of 5 seconds
    // (the typical usage mode), use startPeriodicMeasurement().  For low power mode, with
    // a longer fixed sample interval of 30 seconds, use startLowPowerPeriodicMeasurement()
    // error = co2Sensor.startPeriodicMeasurement();
    error = co2Sensor.startLowPowerPeriodicMeasurement();
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD4X startLowPowerPeriodicMeasurement()",1);
      return false;
    }
    else
    {
      debugMessage("SCD4X starting low power periodic measurements",1);
      return true;
    }
  #endif
}

bool sensorCO2Read()
// sets global environment values from SCD4X sensor
{
  #ifdef HARDWARE_SIMULATE
    sensorCO2Simulate();
    return true;
  #else
    char errorMessage[256];
    bool status;
    uint16_t co2 = 0;
    float temperatureC = 0.0f;
    float humidity = 0.0f;
    uint16_t error;

    debugMessage("SCD4X read initiated",1);

    // Loop attempting to read Measurement
    status = false;
    while(!status) {
      delay(100);

      // Is data ready to be read?
      bool isDataReady = false;
      error = co2Sensor.getDataReadyFlag(isDataReady);
      if (error) {
          errorToString(error, errorMessage, 256);
          debugMessage(String("Error trying to execute getDataReadyFlag(): ") + errorMessage,1);
          continue; // Back to the top of the loop
      }
      if (!isDataReady) {
          continue; // Back to the top of the loop
      }
      debugMessage("SCD4X data available",2);

      error = co2Sensor.readMeasurement(co2, temperatureC, humidity);
      if (error) {
          errorToString(error, errorMessage, 256);
          debugMessage(String("SCD4X executing readMeasurement(): ") + errorMessage,1);
          // Implicitly continues back to the top of the loop
      }
      else if (co2 < sensorCO2Min || co2 > sensorCO2Max)
      {
        debugMessage(String("SCD4X CO2 reading: ") + sensorData.ambientCO2 + " is out of expected range",1);
        //(sensorData.ambientCO2 < sensorCO2Min) ? sensorData.ambientCO2 = sensorCO2Min : sensorData.ambientCO2 = sensorCO2Max;
        // Implicitly continues back to the top of the loop
      }
      else
      {
        // Successfully read valid data
        sensorData.ambientTemperatureF = (temperatureC*1.8)+32.0;
        sensorData.ambientHumidity = humidity;
        sensorData.ambientCO2 = co2;
        debugMessage(String("SCD4X: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
        // Update global sensor readings
        status = true;  // We have data, can break out of loop
      }
    }
  #endif
  return(true);
}

uint8_t nvStorageRead()
// reads data from non-volatile storage and stores in appropriate global variables
// FIX: CO2 values only need to be read for sparkline generation when counter == sensorSampleSize, not every sample
{
  nvStorage.begin("air-quality", false);
  uint8_t storedCounter = nvStorage.getInt("counter", 0);  // counter tracks a 0 based array
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter, 2);

  // read value or insert current sensor reading if this is the first read from nv storage
  accumulatingTempF = nvStorage.getFloat("temp", 0);
  // BME280 often issues nan when not configured properly
  if (isnan(accumulatingTempF)) {
    // bad value, replace with current temp
    accumulatingTempF = (sensorData.ambientTemperatureF * storedCounter);
    debugMessage("Unexpected temperatureF value in nv storage replaced with multiple of current temperature", 2);
  }

  accumulatingHumidity = nvStorage.getFloat("humidity", 0);
  if (isnan(accumulatingHumidity)) {
    // bad value, replace with current temp
    accumulatingHumidity = (sensorData.ambientHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity", 2);
  }

  debugMessage(String("Intermediate values FROM nv storage: Temp:") + accumulatingTempF + ", Humidity:" + accumulatingHumidity, 2);

  // Read CO2 array. If they don't exist, create them as sensorCO2Min
  String nvStoreBaseName;
  for (uint8_t loop = 1; loop < sensorSampleSize; loop++) {
    nvStoreBaseName = "co2Sample" + String(loop);
    co2Samples[loop] = nvStorage.getLong(nvStoreBaseName.c_str(), sensorCO2Min);
    debugMessage(String(nvStoreBaseName) + " retrieved from nv storage is " + co2Samples[loop], 2);
  }
  return storedCounter;
}

void nvStorageWrite(uint8_t counter, float accumulatedTempF, float accumulatedHumidity, uint16_t co2)
// temperatureF and humidity stored as running totals, CO2 stored in array for sparkline
{
  nvStorage.putInt("counter", counter);
  debugMessage(String("Sample count TO nv storage is ") + counter, 2);
  nvStorage.putFloat("temp", accumulatedTempF);
  nvStorage.putFloat("humidity", accumulatedHumidity);
  debugMessage(String("running totals TO nv storage: TempF: ") + accumulatedTempF + ", Humidity: " + accumulatedHumidity, 2);
  if (co2 != 0){
    // write current sample
    String nvStoreBaseName = "co2Sample" + String(counter);
    nvStorage.putLong(nvStoreBaseName.c_str(), co2);
    debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + co2, 2);
  }
  else
  {
    // reset all c02 values
    for (uint8_t loop = 1; loop < sensorSampleSize; loop++) {
      String nvStoreBaseName = "co2Sample" + String(loop);
      nvStorage.putLong(nvStoreBaseName.c_str(), co2);
      debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + co2, 2);
    }
  debugMessage("co2 values in nv storage zeroed",1);
  }
}

void powerI2CEnable()
// enables I2C across multiple Adafruit ESP32 variants
{
  debugMessage("powerI2CEnable() start", 2);

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn on the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    debugMessage("power on: Feather ESP32V2 I2C", 2);
  #endif

  debugMessage("powerI2CEnable() complete", 1);
}

void powerDisable(uint16_t deepSleepTime)
// turns off component hardware then puts ESP32 into deep sleep mode for specified seconds
{
  debugMessage("powerDisable() start", 1);

  // power down epd
  display.powerOff();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  debugMessage("power off: epd", 2);

  networkDisconnect();

  // power down SCD4X by stopping potentially started measurement then power down SCD4X
  #ifndef HARDWARE_SIMULATE
    uint16_t error = co2Sensor.stopPeriodicMeasurement();
    if (error) {
      char errorMessage[256];
      errorToString(error, errorMessage, 256);
      debugMessage(String("Error: SCD4X stopPeriodicMeasurement() returned: ") + errorMessage,1);
    }
    co2Sensor.powerDown();
    debugMessage("power off: SCD4X",2);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
    debugMessage("power off: ESP32V2 I2C", 2);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime * 1000000);  // ESP microsecond modifier
  debugMessage(String("powerDisable() complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds", 1);
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

uint8_t co2Range(uint16_t co2)
// converts co2 value to index value for labeling and color
{
  uint8_t co2Range;
  if (co2 <= co2Fair) co2Range = 0;
  else if (co2 <= co2Poor) co2Range = 1;
  else if (co2 <= co2Bad) co2Range = 2;
  else co2Range =3;
  debugMessage(String("CO2 input of ") + co2 + " yields co2Range of " + co2Range,2);
  return co2Range;
}

uint8_t aqiRange(float pm25)
// converts pm25 value to index value for labeling and color
{
  uint8_t aqi;
  if (pm25 <= pmFair) aqi = 0;
  else if (pm25 <= pmPoor) aqi = 1;
  else if (pm25 <= pm2Bad) aqi = 2;
  else aqi = 3;
  debugMessage(String("PM2.5 input of ") + pm25 + " yields " + aqi + " aqi",2);
  return aqi;
}

void testSparkLineValues(uint8_t sampleSetSize)
// generates test data to exercise the screenSparkLine function
{
  // generate test data
  for (uint8_t loop = 0; loop < sampleSetSize; loop++) {
    // standard range for indoor CO2 values
    co2Samples[loop] = random(sensorCO2Min, sensorCO2Max);
  }
}

bool networkConnect() 
// Connect to WiFi network specified in secrets.h
{
  #ifdef HARDWARE_SIMULATE
    networkSimulate();
    return true;
  #else
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
      debugMessage(String("WiFi status message ") + networkWiFiMessage(WiFi.status()),2);
      // use of delay() OK as this is initialization code
      delay(networkConnectAttemptInterval * 1000);  // converted into milliseconds
    }
    return false;
  #endif
}

void networkDisconnect()
// Disconnect from WiFi network
{
  hardwareData.rssi = 0;
  #ifdef HARDWARE_SIMULATE
    debugMessage("power off: SIMULATED WiFi",2);
    return;
  #else
    // IMPROVEMENT: What if disconnect call fails?
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    debugMessage("power off: WiFi",2);
  #endif
}

bool networkGetTime(String timezone)
// Set local time from NTP server specified in config.h
{
  // !!! ESP32 hardware dependent, using Esperif library
  // https://randomnerdtutorials.com/esp32-ntp-timezones-daylight-saving/
  #ifdef HARDWARE_SIMULATE
    // IMPROVEMENT: Add random time
    return false;
  #else
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
  #endif
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
#ifdef HARDWARE_SIMULATE
  dateTime = "SIMULATE TIME";
  return dateTime;
#else
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
#endif
}

String networkHTTPGETRequest(const char* serverName) 
{
  String payload = "{}";
  #ifdef HARDWARE_SIMULATE
    return payload;
  #else
    // !!! ESP32 hardware dependent, using Esperif library

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
  #endif
}

#ifndef HARDWARE_SIMULATE
  const char* networkWiFiMessage(wl_status_t status)
  // Converts WiFi.status() to string
  {
    switch (status) {
      case WL_NO_SHIELD: return "WL_NO_SHIELD";
      case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
      case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
      case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
      case WL_CONNECTED: return "WL_CONNECTED";
      case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
      case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
      case WL_DISCONNECTED: return "WL_DISCONNECTED";
    }
  }
#endif