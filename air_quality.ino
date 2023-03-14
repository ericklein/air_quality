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

// accumulating sensor readings
float averageTempF;
float averageHumidity;
uint16_t averageCO2;

// environment sensor data
typedef struct
{
  float ambientTempF;
  float ambientHumidity;
  uint16_t ambientCO2;
} envData;
envData sensorData; // global variable for environment sensor data

// stores CO2 sample values for sparkline
uint16_t co2Samples[SAMPLE_SIZE];

// hardware status data
typedef struct
{
  float batteryPercent;
  float batteryVoltage;
  int rssi;
} hdweData;
hdweData hardwareData;  // global variable for hardware characteristics          

// OpenWeatherMap Current data
typedef struct
{
  float lon;              // "lon": 8.54
  float lat;              // "lat": 47.37
  uint16_t weatherId;     // "id": 521
  String main;            // "main": "Rain"
  String description;     // "description": "shower rain"
  String icon;            // "icon": "09d"
  float temp;             // "temp": 90.56
  uint16_t pressure;      // "pressure": 1013, in hPa
  uint16_t humidity;      // "humidity": 87, as %
  float tempMin;          // "temp_min": 89.15
  float tempMax;          // "temp_max": 92.15
  uint16_t visibility;    // visibility: 10000, in meters
  float windSpeed;        // "wind": {"speed": 1.5}, in meters/s
  float windDeg;          // "wind": {deg: 226.505}
  uint8_t clouds;         // "clouds": {"all": 90}, in %
  time_t observationTime; // "dt": 1527015000, in UTC
  String country;         // "country": "CH"
  time_t sunrise;         // "sunrise": 1526960448, in UTC
  time_t sunset;          // "sunset": 1527015901, in UTC
  String cityName;        // "name": "Zurich"
  time_t timezone;        // shift in seconds from UTC
} OpenWeatherMapCurrentData;
OpenWeatherMapCurrentData owmCurrentData; // global variable for OWM current data

// OpenWeatherMap Air Quality data
typedef struct
{
  float lon;    // "lon": 8.54
  float lat;    // "lat": 47.37
  int aqi;      // "aqi": 2  [European standards body value]
  float co;     // "co": 453.95, in μg/m3
  float no;     // "no": 0.47, in μg/m3
  float no2;    // "no2": 52.09, in μg/m3
  float o3;     // "o3": 17.17, in μg/m3
  float so2;    // "so2": 7.51, in μg/m3
  float pm25;   // "pm2.5": 8.04, in μg/m3
  float pm10;   // "pm10": 9.96, in μg/m3
  float nh3;    // "nh3": 0.86, in μg/m3
} OpenWeatherMapAirQuality;
OpenWeatherMapAirQuality owmAirQuality; // global variable for OWM current data

// initialize environment sensors

#ifdef SCD40
  // SCD40; temp, humidity, CO2
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x envSensor;
#else
  // unified Adafruit sensor setup
  // AHTX0; temp, humidity
  //#include <Adafruit_AHTX0.h>
  //Adafruit_AHTX0 envSensor;

  // BME280; temp, humidity
  #include <Adafruit_BME280.h>
  Adafruit_BME280 envSensor; // i2c interface
  Adafruit_Sensor *envSensor_temp = envSensor.getTemperatureSensor();
  Adafruit_Sensor *envSensor_humidity = envSensor.getHumiditySensor();
#endif

// Battery voltage sensor
#include <Adafruit_LC709203F.h>
Adafruit_LC709203F lc;

// screen support
#ifdef SCREEN
  #include <Adafruit_ThinkInk.h>

  #include "Fonts/meteocons20pt7b.h"
  #include "Fonts/meteocons16pt7b.h"
  #include "Fonts/meteocons12pt7b.h"

  #include <Fonts/FreeSans9pt7b.h>
  #include <Fonts/FreeSans12pt7b.h>
  #include <Fonts/FreeSans18pt7b.h>

  // Special glyphs for the UI
  #include "Fonts/glyphs.h"

  // 2.96" greyscale display with 196x128 pixels
  // colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK
  ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

  // screen layout assists
  const int xMargins = 5;
  const int xOutdoorMargin = ((display.width()/2) + xMargins);
  const int yMargins = 2;
  // yCO2 not used
  const int yCO2 = 20;
  const int ySparkline = 40;
  const int yTemp = 100;
  // BUG, 7/8 = 112, WiFi status is 15 (5*3) pixels high
  const int yStatus = (display.height()*7/8);
  const int sparklineHeight = 40;
  const int batteryBarWidth = 28;
  const int batteryBarHeight = 10;
#endif

#include "ArduinoJson.h"  // Needed by OWM retrieval routines

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float batteryVoltage, int rssi);
#endif

#ifdef DWEET
  extern void post_dweet(uint16_t co2, float tempF, float humidity, float batteryVoltage, int rssi);
#endif

#ifdef MQTT
  extern bool mqttDeviceWiFiUpdate(int rssi);
  extern bool mqttDeviceBatteryUpdate(float batteryVoltage);
  extern bool mqttSensorTempFUpdate(float tempF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  extern bool mqttSensorCO2Update(uint16_t co2);
#endif

void setup()
// One time run of code, then deep sleep
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
  #endif
  // Confirm key site configuration parameters
  debugMessage("Air Quality started");
  debugMessage(String("Sample interval is ") + SAMPLE_INTERVAL + " seconds");
  debugMessage(String("Number of samples before reporting is ") + SAMPLE_SIZE);
  debugMessage(String("Internet service reconnect delay is ") + CONNECT_ATTEMPT_INTERVAL + " seconds");
  #ifdef DWEET
    debugMessage("Dweet device: " + String(DWEET_DEVICE));
  #endif

  hardwareData.batteryVoltage = 0;  // 0 = no battery attached
  hardwareData.rssi = 0;            // 0 = no WiFi 

  powerEnable();

  #ifdef SCREEN
    // there is no way to query screen for status
    display.begin(THINKINK_MONO); // changed from THINKINK_GRAYSCALE4 to eliminate black screen border
    debugMessage("Display ready");
  #endif

  // Initialize environmental sensor
  if (!sensorInit()) 
  {
    debugMessage("Environment sensor failed to initialize");
    screenAlert("Env sensor not detected");
    // This error often occurs after a firmware flash and then resetting the board
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // Environmental sensor available, so fetch values
  int sampleCounter;
  if(!sensorRead())
  {
    debugMessage("SCD40 returned no/bad data");
    screenAlert("SCD40 no/bad data");
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }
  sampleCounter = nvStorageRead();
  sampleCounter++;

  if (sampleCounter < SAMPLE_SIZE)
  // add to the accumulating, intermediate sensor values and sleep device
  {
    nvStorageWrite(sampleCounter, sensorData.ambientTempF+averageTempF, sensorData.ambientHumidity+averageHumidity, sensorData.ambientCO2);
    powerDisable(SAMPLE_INTERVAL);
  } 
  
  // sampleCounter == SAMPLE_SIZE, so average values for reporting
  averageTempF = ((sensorData.ambientTempF + averageTempF) / SAMPLE_SIZE);
  averageHumidity = ((sensorData.ambientHumidity + averageHumidity) / SAMPLE_SIZE);
  for(int i=0;i<SAMPLE_SIZE;i++)
  {
    averageCO2 = averageCO2 + co2Samples[i];
  }
  averageCO2 = uint16_t(averageCO2/SAMPLE_SIZE);
  debugMessage(String("Averaged values Temp:") + averageTempF + "F, Humidity:" + averageHumidity + ", CO2:" + averageCO2);

  batteryRead();

  // Setup network connection specified in config.h
  if (aq_network.networkBegin())
    hardwareData.rssi = abs(aq_network.getWiFiRSSI());    

  // Get local weather and air quality info from Open Weather Map
  if (!OWMCurrentWeatherDataRead())
  {
    owmCurrentData.temp = 10000;
    owmCurrentData.humidity = 10000;
    // SECONDARY: Set UTC time offset based on config.h time zone
    aq_network.setTime(gmtOffset_sec, daylightOffset_sec);
  }
  // PRIMARY: Set UTC time offset based on OWM local time zone
  aq_network.setTime(owmCurrentData.timezone, 0);

  if (!OWMAirPollutionRead())
  {
    owmAirQuality.aqi = 10000;
  }

  String upd_flags = "";  // To indicate whether services succeeded
  if (hardwareData.rssi!=0)
  {
    #ifdef MQTT
      if ((mqttSensorTempFUpdate(averageTempF)) && (mqttSensorHumidityUpdate(averageHumidity)) && (mqttSensorCO2Update(averageCO2)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage)))
      {
        upd_flags += "M";
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
  
    if (upd_flags == "") 
    {
      // External data services not updated but we have network time
      screenInfo(aq_network.dateTimeString());
    } 
    else 
    {
      // External data services not updated and we have network time
      screenInfo("[+" + upd_flags + "] " + aq_network.dateTimeString());
    }
  }
  else
  {
    // no internet connection, update screen with sensor data only
    #ifdef DEBUG
      screenInfo("Test msg from DEBUG");
    #else
      screenInfo("");
    #endif
  }
  //reset nvStorage values for next recording period
  nvStorageWrite(-1,0,0,0);
  debugMessage("nvStorage values reset");

  powerDisable(SAMPLE_INTERVAL);
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

bool OWMCurrentWeatherDataRead()
// stores local current weather info from Open Weather Map in environment global
{
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (hardwareData.rssi!=0)
    // and internet is verified
    {
      String jsonBuffer;

      // Get local weather conditions
      String serverPath = String(OWM_SERVER) + OWM_WEATHER_PATH + OWM_LAT_LONG + "&units=imperial" + "&APPID=" + OWM_KEY;

      jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
      debugMessage("Raw JSON from OWM Current Weather feed");
      debugMessage(jsonBuffer);      
      if (jsonBuffer=="HTTP GET error")
      {
        return false;
      }

      DynamicJsonDocument doc(2048);

      DeserializationError error = deserializeJson(doc, jsonBuffer);

      if (error)
      {
        debugMessage(String("deserializeJson failed with error message: ") + error.c_str());
        return false;
      }
      
      int code = (int) doc["cod"];
      if(code != 200)
      {
        debugMessage(String("OWM error: ") + (const char *)doc["message"]);
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
      debugMessage(String("OWM Current Weather set: ") + owmCurrentData.temp + "F, " + owmCurrentData.humidity + "%");
      return true;
    }
  #endif
  return false;
}

bool OWMAirPollutionRead()
// stores local air pollution info from Open Weather Map in environment global
{
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (hardwareData.rssi!=0)
    // and internet is verified
    {
      String jsonBuffer;

      // Get local AQI
      String serverPath = String(OWM_SERVER) + OWM_AQM_PATH + OWM_LAT_LONG + "&APPID=" + OWM_KEY;

      jsonBuffer = aq_network.httpGETRequest(serverPath.c_str());
      debugMessage("Raw JSON from OWM AQI feed");
      debugMessage(jsonBuffer);      
      if (jsonBuffer=="HTTP GET error")
      {
        return false;
      }

      DynamicJsonDocument doc(384);

      DeserializationError error = deserializeJson(doc, jsonBuffer);
      if (error)
      {
        debugMessage(String("deserializeJson failed with error message: ") + error.c_str());
        return false;
      }

      owmAirQuality.lon = (float) doc["coord"]["lon"];
      owmAirQuality.lat = (float) doc["coord"]["lat"];
      JsonObject list_0 = doc["list"][0];
      owmAirQuality.aqi = list_0["main"]["aqi"];
      JsonObject list_0_components = list_0["components"];
      owmAirQuality.co = (float) list_0_components["co"];
      owmAirQuality.no = (float) list_0_components["no"];
      owmAirQuality.no2 = (float) list_0_components["no2"];
      owmAirQuality.o3 = (float) list_0_components["o3"];
      owmAirQuality.so2 = (float) list_0_components["so2"];
      owmAirQuality.pm25 = (float) list_0_components["pm2_5"];
      debugMessage(String("OWM current PM2.5 is ") + owmAirQuality.pm25 + " in μg/m3");      
      owmAirQuality.pm10 = (float) list_0_components["pm10"];
      owmAirQuality.nh3 = (float) list_0_components["nh3"];
      return true;
    }
  #endif
  return false;
}

void screenAlert(String messageText)
// Display critical error message on screen
{
#ifdef SCREEN
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xMargins,(display.height()/2));
  display.print(messageText);

  //update display
  display.display();
#endif
}

void screenInfo(String messageText)
// Display environmental information on screen
{
#ifdef SCREEN  

  // TEST ONLY: load values normally supplied by OWM
  // testOWMValues();
  
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // borders
  // label
  display.drawLine(0,yStatus,display.width(),yStatus,EPD_GRAY);
  // splitting sensor vs. outside values
  display.drawLine((display.width()/2),0,(display.width()/2),yStatus,EPD_GRAY);
  
  // screen helper routines
  // draws battery in the lower right corner. -3 in first parameter accounts for battery nub
  screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight),batteryBarWidth,batteryBarHeight);
  
  // -70 moves it to the left of the battery display
  screenHelperWiFiStatus((display.width() - xMargins - 70), (display.height() - yMargins),3,3,5);
  
  // draws any status message in the lower left corner. -8 in the first parameter accounts for fixed font height
  screenHelperStatusMessage(xMargins,(display.height()-yMargins-8), messageText);

  // display sparkline
  screenHelperSparkLines(xMargins,ySparkline,((display.width()/2) - (2 * xMargins)),sparklineHeight);

  // Indoor
  // CO2 level
  // calculate CO2 value range in 400ppm bands
  int co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400

  // main line
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xMargins, yCO2);
  display.print("CO");
  display.setCursor(xMargins+50,yCO2);
  display.print(": " + String(co2Labels[co2range]));
  display.setFont(&FreeSans9pt7b);
  display.setCursor(xMargins+35,(yCO2+10));
  display.print("2");
  // value line
  display.setFont();
  display.setCursor((xMargins+88),(yCO2+7));
  display.print("(" + String(sensorData.ambientCO2) + ")");

  // Indoor temp
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xMargins,yTemp);
  display.print(String((int)(sensorData.ambientTempF + .5)));
  display.setFont(&meteocons12pt7b);
  display.print("+");

  // Indoor humidity
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xMargins+60, yTemp);
  display.print(String((int)(sensorData.ambientHumidity + 0.5)));
  // original icon ratio was 5:7?
  display.drawBitmap(xMargins+90,yTemp-21,epd_bitmap_humidity_icon_sm4,20,28,EPD_BLACK);

  // Outside
  // location label
  display.setFont();
  display.setCursor((display.width()*5/8),yMargins);
  display.print(owmCurrentData.cityName);

  // Outside temp
  if (owmCurrentData.temp!=10000)
  {
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xOutdoorMargin,yTemp);
    display.print(String((int)(owmCurrentData.temp+0.5)));
    display.setFont(&meteocons12pt7b);
    display.print("+");
  }

  // Outside humidity
  if (owmCurrentData.humidity!=10000)
  {
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xOutdoorMargin + 60, yTemp);
    display.print(String((int)(owmCurrentData.humidity+0.5)));
    display.drawBitmap(xOutdoorMargin+90,yTemp-21,epd_bitmap_humidity_icon_sm4,20,28,EPD_BLACK);
  }

  // weather icon
  String weatherIcon = OWMtoMeteoconIcon(owmCurrentData.icon);
  // if getMeteoIcon doesn't have a matching symbol, skip display
  if (weatherIcon!=")")
  {
    // display icon
    display.setFont(&meteocons20pt7b);
    display.setCursor((display.width()*4/5),(display.height()/2));
    display.print(weatherIcon);
  }

  // Outside air quality index (AQI)
  if (owmAirQuality.aqi!=10000)
  {
    display.setFont(&FreeSans9pt7b);
    display.setCursor(xOutdoorMargin,ySparkline - 5);
    // European standards-body AQI value
    //display.print(aqiEuropeanLabels[(owmAirQuality.aqi-1)]);

    // US standards-body AQI value
    float aqiUS = pm25toAQI(owmAirQuality.pm25);
    debugMessage(String("US AQI value is ") + aqiUS);

    display.print(aqiUSALabels[aqiUSLabelValue(owmAirQuality.pm25)]);
    display.print(" AQI");
  }

  //update display
  display.display();
  debugMessage("Screen updated");
#endif
}

void screenHelperStatusMessage(int initialX, int initialY, String messageText)
// helper function for screenXXX() routines that draws a status message
// uses system default font, so text drawn x+,y+ from initialX,Y
{
  // IMPROVEMENT : Screen dimension boundary checks for function parameters
  display.setFont();  // resets to system default monospace font (6x8 pixels)
  display.setCursor(initialX, initialY);
  display.print(messageText);
}

void screenHelperWiFiStatus(int initialX, int initialY, int barWidth, int barHeightMultiplier, int barSpacingMultipler)
// helper function for screenXXX() routines that draws WiFi signal strength
{
  if (hardwareData.rssi!=0) 
  {
    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    int barCount = constrain((6-((hardwareData.rssi/10)-3)),0,5);
    if (barCount>0)
    {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++)
      {
        display.fillRect((initialX + (b * barSpacingMultipler)), (initialY - (b * barHeightMultiplier)), barWidth, b * barHeightMultiplier, EPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount +" bars");
    }
    else
    {
      // you could do a visual representation of no WiFi strength here
      debugMessage("RSSI too low, no display");
    }
  }
}

void screenHelperBatteryStatus(int initialX, int initialY, int barWidth, int barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for function parameters
  #ifdef SCREEN
    if (hardwareData.batteryVoltage>0) 
    {
      // battery nub; width = 3pix, height = 60% of barHeight
      display.fillRect((initialX+barWidth),(initialY+(int(barHeight/5))),3,(int(barHeight*3/5)),EPD_BLACK);
      // battery border
      display.drawRect(initialX,initialY,barWidth,barHeight,EPD_BLACK);
      //battery percentage as rectangle fill, 1 pixel inset from the battery border
      display.fillRect((initialX + 2),(initialY + 2),(int((hardwareData.batteryPercent/100)*barWidth) - 4),(barHeight - 4),EPD_GRAY);
      debugMessage(String("battery status drawn to screen as ") + hardwareData.batteryPercent + "%" );
    }
  #endif
}

void screenHelperSparkLines(int initialX, int initialY, int xWidth, int yHeight)
{
  // TEST ONLY: load test CO2 values
  // testSparkLineValues(SAMPLE_SIZE);

  uint16_t co2Min = co2Samples[0];
  uint16_t co2Max = co2Samples[0];
  // # of pixels between each samples x and y coordinates
  int xPixelStep, yPixelStep;

  int sparkLineX[SAMPLE_SIZE], sparkLineY[SAMPLE_SIZE];

  // horizontal distance (pixels) between each displayed co2 value
  xPixelStep = (xWidth / (SAMPLE_SIZE - 1));

  // determine min/max of CO2 samples
  // could use recursive function but SAMPLE_SIZE should always be relatively small
  for(int i=0;i<SAMPLE_SIZE;i++)
  {
    if(co2Samples[i] > co2Max) co2Max = co2Samples[i];
    if(co2Samples[i] < co2Min) co2Min = co2Samples[i];
  }
  debugMessage(String("Max CO2 in stored sample range is ") + co2Max +", min is " + co2Min);

  // vertical distance (pixels) between each displayed co2 value
  yPixelStep = round(((co2Max - co2Min) / yHeight)+.5);

  debugMessage(String("xPixelStep is ") + xPixelStep + ", yPixelStep is " + yPixelStep);

  // TEST ONLY : sparkline border box
  // display.drawRect(initialX,initialY, xWidth,yHeight, EPD_BLACK);

  // determine sparkline x,y values
  for(int i=0;i<SAMPLE_SIZE;i++)
  {
    sparkLineX[i] = (initialX + (i * xPixelStep));
    sparkLineY[i] = ((initialY + yHeight) - (int)((co2Samples[i]-co2Min) / yPixelStep));
    // draw/extend sparkline after first value is generated
    if (i != 0)
      display.drawLine(sparkLineX[i-1],sparkLineY[i-1],sparkLineX[i],sparkLineY[i],EPD_BLACK);  
  }
  for (int i=0;i<SAMPLE_SIZE;i++)
  {
    debugMessage(String("X,Y coordinates for CO2 sample ") + i + " is " + sparkLineX[i] + "," + sparkLineY[i]);
  }
    debugMessage("sparkline drawn to screen");
}

void batteryRead()
// stores battery voltage if available in hardware characteristics global 
{
  // check to see if i2C monitor is available
  if (lc.begin())
  // Check battery monitoring status
  {
    debugMessage("Reading battery voltage from LC709203F");
    lc.setPackAPA(BATTERY_APA);
    hardwareData.batteryPercent = lc.cellPercent();
    hardwareData.batteryVoltage = lc.cellVoltage();
  } 
  else
  {
  // use supported boards to read voltage
    #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
      // see project supporting material for other ways to quantify battery %
      pinMode(VBATPIN,INPUT);

      // assumes default ESP32 analogReadResolution (4095)
      // the 1.05 is a fudge factor original author used to align reading with multimeter
      hardwareData.batteryVoltage = ((float)analogRead(VBATPIN) / 4095) * 3.3 * 2 * 1.05;
      hardwareData.batteryPercent = (uint8_t)(((hardwareData.batteryVoltage - BATTV_MIN) / (BATTV_MAX - BATTV_MIN)) * 100);
    #endif
  }
  if (hardwareData.batteryVoltage!=0) 
  {
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%");
  }
}

bool sensorInit()
// initializes environment sensor if available. Supports SCD40, ATHX0, BME280 sensors
{
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

    envSensor.wakeUp();
    envSensor.setSensorAltitude(SITE_ALTITUDE); // optimizes CO2 reading

    uint16_t error = envSensor.startPeriodicMeasurement();
    if (error) 
    {
      // Failed to initialize SCD40
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()");
      return false;
    }
    else 
    {
      debugMessage("SCD40 initialized");
      return true;
    }
  #else
    // ATHX0, BME280
    if (envSensor.begin())
    {
      // ID of 0x56-0x58 or 0x60 is a BME 280, 0x61 is BME680, 0x77 is BME280 on ESP32S2 Feather
      debugMessage(String("Environment sensor ready, ID is: ")+envSensor.sensorID());
      return true;
    }
    else
    {
      return false;
    }
  #endif
}

bool sensorRead()
// stores environment sensor to environment global
{

  #ifdef SCD40
    char errorMessage[256];

    for (int loop=1; loop<=READS_PER_SAMPLE; loop++)
    {
      // minimum time between SCD40 reads
      delay(5000);
      // read and store data if successful
      uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity);
      // handle SCD40 errors
      if (error) {
        errorToString(error, errorMessage, 256);
        debugMessage(String(errorMessage) + " during SCD4X read");
        return false;
      }
      if (sensorData.ambientCO2<400 || sensorData.ambientCO2>6000)
      {
        debugMessage(String("SCD40 CO2 reading out of range at ") + sensorData.ambientCO2);
        return false;
      }
      //convert C to F for temp
      sensorData.ambientTempF = (sensorData.ambientTempF * 1.8) + 32;
      debugMessage(String("SCD40 read ") + loop + " of " + READS_PER_SAMPLE + ": " + sensorData.ambientTempF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm");
    }
    return true;
  #else
    // AHTX0, BME280
    sensors_event_t temp_event, humidity_event;
    // FIX : Can we get error conditions from this API?
    envSensor_temp->getEvent(&temp_event);
    envSensor_humidity->getEvent(&humidity_event);
   
    sensorData.ambientTempF = (temp_event.temperature * 1.8) +32;
    sensorData.ambientHumidity = humidity_event.relative_humidity;
    sensorData.ambientCO2 = 10000;
    return true;
  #endif
}

int nvStorageRead() 
// reads data from non-volatile storage and stores in appropriate global variables
// FIX: CO2 values only need to be read for sparkline generation when counter == SAMPLE_SIZE, not every sample
{
  nvStorage.begin("air-quality", false);
  int storedCounter = nvStorage.getInt("counter", -1); // counter tracks a 0 based array
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter);

  // read value or insert current sensor reading if this is the first read from nv storage
  averageTempF = nvStorage.getFloat("temp", 0);
  // BME280 often issues nan when not configured properly
  if (isnan(averageTempF))
  {
    // bad value, replace with current temp
    averageTempF = (sensorData.ambientTempF * storedCounter);
    debugMessage("Unexpected tempF value in nv storage replaced with multiple of current temperature");
  }

  averageHumidity = nvStorage.getFloat("humidity", 0);
  if (isnan(averageHumidity)) 
  {
    // bad value, replace with current temp
    averageHumidity = (sensorData.ambientHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity");
  }

  debugMessage(String("Intermediate values FROM nv storage: Temp:") + averageTempF + "F, Humidity:" + averageHumidity + "%");

  // Read CO2 array. If they don't exist, create them as 400 (CO2 floor)
  String nvStoreBaseName;
  for (int i=0; i<SAMPLE_SIZE; i++)
  {
    nvStoreBaseName = "co2Sample" + String(i);
    co2Samples[i] = nvStorage.getLong(nvStoreBaseName.c_str(),400);
    debugMessage(String(nvStoreBaseName) + " retrieved from nv storage is " + co2Samples[i]);
  }  
  return storedCounter;
}

void nvStorageWrite(int storedCounter, float tempF, float humidity, uint16_t co2)
// tempF and humidity stored as running totals, CO2 stored in array for sparkline
{
  nvStorage.putInt("counter", storedCounter);
  debugMessage(String("Sample count TO nv storage is ") + storedCounter);
  nvStorage.putFloat("temp", tempF);
  nvStorage.putFloat("humidity", humidity);
  debugMessage(String("Intermediate values TO nv storage: Temp: ") + tempF + "F, Humidity: " + humidity + "%");
  if ((sensorData.ambientCO2 != 10000) && (co2 != 0))
  {
    String nvStoreBaseName = "co2Sample" + String(storedCounter);
    nvStorage.putLong(nvStoreBaseName.c_str(),co2);
    debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + co2);
  }
  if (co2 == 0) // reset all the values
  {
    for (int i=0;i<SAMPLE_SIZE;i++)
    {
      String nvStoreBaseName = "co2Sample" + String(i);
      nvStorage.putLong(nvStoreBaseName.c_str(),co2);
      debugMessage(String(nvStoreBaseName) + " stoin nv storage as " + co2);
    }
  }
}

void powerEnable()
// enable appropriate hardware
{
  // Handle two ESP32 I2C ports
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary ports must be manually
    // assigned their pins with setPins()!
    Wire1.setPins(SDA1, SCL1);
    debugMessage("enabled ESP32 hardware with two I2C ports");
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

    // if you need to turn the neopixel on
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, HIGH);
    debugMessage("enabled Adafruit Feather ESP32S2 I2C power");
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn on the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);

    // Turn on neopixel
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, HIGH);
    debugMessage("enabled Adafruit Feather ESP32 V2 I2C power");
  #endif
}

void powerDisable(int deepSleepTime)
// Powers down hardware activated via powerEnable() then deep sleep MCU
{
  char errorMessage[256];

  debugMessage("Starting power down activities");
  // power down epd
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  debugMessage("powered down epd");

  aq_network.networkStop();

  // power down SCD40
  // stops potentially started measurement then powers down SCD40
  uint16_t error = envSensor.stopPeriodicMeasurement();
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()");
  }
  envSensor.powerDown();
  debugMessage("SCD40 powered down");

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32 V2 I2C power");
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32S2 I2C power");
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("Going to sleep for ") + deepSleepTime + " seconds");
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
  if ((icon == "50d") || (icon == "50n")) // 50d = mist = Meteocon "M"
    return "M";
  // Nothing matched
  debugMessage("OWM icon not matched to Meteocon, why?");
  return ")";
}

float pm25toAQI(float pm25)
// Converts pm25 reading to AQI using the AQI Equation
// (https://forum.airnowtech.org/t/the-aqi-equation/169)
{  
  if(pm25 <= 12.0)       return(fmap(pm25,  0.0, 12.0,  0.0, 50.0));
  else if(pm25 <= 35.4)  return(fmap(pm25, 12.1, 35.4, 51.0,100.0));
  else if(pm25 <= 55.4)  return(fmap(pm25, 35.5, 55.4,101.0,150.0));
  else if(pm25 <= 150.4) return(fmap(pm25, 55.5,150.4,151.0,200.0));
  else if(pm25 <= 250.4) return(fmap(pm25,150.5,250.4,201.0,300.0));
  else if(pm25 <= 500.4) return(fmap(pm25,250.5,500.4,301.0,500.0));
  else return(505.0);  // AQI above 500 not recognized
}

float fmap(float x, float xmin, float xmax, float ymin, float ymax)
{
    return( ymin + ((x - xmin)*(ymax-ymin)/(xmax - xmin)));
}

int aqiUSLabelValue(float pm25)
// converts pm25 value to a 0-5 value associated with US AQI labels
{
  if(pm25 <= 12.0)       return(0);
  else if(pm25 <= 35.4)  return(1);
  else if(pm25 <= 55.4)  return(2);
  else if(pm25 <= 150.4) return(3);
  else if(pm25 <= 250.4) return(4);
  else if(pm25 <= 500.4) return(5);
  else return(6);  // AQI above 500 not recognized 
}

void testOWMValues()
// Test data to drive screenInfo() when not connected to Internet
{
  hardwareData.rssi = 47;
  owmCurrentData.cityName = "Mercer Island";
  owmCurrentData.temp = 101.01;
  owmCurrentData.humidity = 56.43;
  owmCurrentData.icon = "09d";
  // aqi values
  owmAirQuality.aqi = 3; // overrides error code value
  owmAirQuality.pm25 = 248.04;
}

void testSparkLineValues(int sampleSetSize)
// generates test data to exercise the screenSparkLine function
{
    // generate test data
  for(int i=0;i<sampleSetSize;i++)
  {
    // standard range for indoor CO2 values
    co2Samples[i]=random(600,2400);
  }
}