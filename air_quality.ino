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

bool batteryVoltageAvailable = false;
bool internetAvailable = false;

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
  #include <Fonts/FreeSans24pt7b.h>

  ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
  // colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK
#endif

#include "ArduinoJson.h"  // Needed by OWM retrieval routines

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_v, int rssi);
#endif

#ifdef DWEET
  extern void post_dweet(uint16_t co2, float tempF, float humidity, float battpct, float battv, int rssi);
#endif

#ifdef MQTT
  //extern void mqttConnect();
  extern int mqttDeviceWiFiUpdate(int rssi);
  extern int mqttDeviceBatteryUpdate(float cellVoltage);
  extern int mqttSensorUpdate(uint16_t co2, float tempF, float humidity);
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

  enableInternalPower();

  #ifdef SCREEN
    // there is no way to query screen for status
    display.begin(THINKINK_MONO); // changed from THINKINK_GRAYSCALE4 to eliminate black screen border
    debugMessage("Display ready");
  #endif

  // Initialize environmental sensor.  Returns non-zero if initialization fails
  if (!initSensor()) 
  {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    screenAlert("Env sensor not detected");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    disableInternalPower(HARDWARE_ERROR_INTERVAL);
  }

  // Environmental sensor available, so fetch values
  int sampleCounter;
  if(!readSensor())
  {
    debugMessage("SCD40 returned no/bad data, going to sleep");
    screenAlert("SCD40 no/bad data");
    disableInternalPower(HARDWARE_ERROR_INTERVAL);
  }
  sampleCounter = readNVStorage();
  sampleCounter++;

  if (sampleCounter < SAMPLE_SIZE)
  // add to the accumulating, intermediate sensor values and go back to sleep
  {
    nvStorage.putInt("counter", sampleCounter);
    debugMessage(String("Sample count TO nv storage is ") + sampleCounter);
    nvStorage.putFloat("temp", (sensorData.ambientTempF + averageTempF));
    nvStorage.putFloat("humidity", (sensorData.ambientHumidity + averageHumidity));
    if (sensorData.ambientCO2 != 10000) {
      nvStorage.putUInt("co2", (sensorData.ambientCO2 + averageCO2));
    }
    debugMessage(String("Intermediate values TO nv storage: Temp:") + (sensorData.ambientTempF + averageTempF) + ", Humidity:" + (sensorData.ambientHumidity + averageHumidity) + ", CO2:" + (sensorData.ambientCO2 + averageCO2));
    disableInternalPower(SAMPLE_INTERVAL);
  } 
  else
  {
    // average intermediate values
    averageTempF = ((sensorData.ambientTempF + averageTempF) / SAMPLE_SIZE);
    averageHumidity = ((sensorData.ambientHumidity + averageHumidity) / SAMPLE_SIZE);
    if (sensorData.ambientCO2 != 10000) 
    {
      averageCO2 = ((sensorData.ambientCO2 + averageCO2) / SAMPLE_SIZE);
    }
    debugMessage(String("Averaged values Temp:") + averageTempF + "F, Humidity:" + averageHumidity + ", CO2:" + averageCO2);
    //reset and store sample set variables
    nvStorage.putInt("counter", 0);
    nvStorage.putFloat("temp",0);
    nvStorage.putFloat("humidity",0);
    nvStorage.putUInt("co2",0);
    debugMessage("Intermediate values in nv storage reset to zero");
  }

  batteryReadVoltage();

  // Setup network connection specified in config.h
  internetAvailable = aq_network.networkBegin();

  // Get local weather and air quality info from Open Weather Map
  if (!getOWMCurrentWeatherData())
  {
    owmCurrentData.temp = 10000;
    owmCurrentData.humidity = 10000;
    // SECONDARY: Set UTC time offset based on config.h time zone
    aq_network.setTime(gmtOffset_sec, daylightOffset_sec);
  }
  // PRIMARY: Set UTC time offset based on OWM local time zone
  aq_network.setTime(owmCurrentData.timezone, 0);

  if (!getOWMAirPollution())
  {
    owmAirQuality.aqi = 10000;
  }

  String upd_flags = "";  // To indicate whether services succeeded
  if (internetAvailable)
  {
    hardwareData.rssi = abs(aq_network.getWiFiRSSI());

    #ifdef MQTT
      if ((mqttSensorUpdate(sensorData.ambientCO2, sensorData.ambientTempF,sensorData.ambientHumidity)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage)))
      {
        upd_flags += "M";
      }
    #endif

    #ifdef DWEET
      post_dweet(averageCO2, averageTempF, averageHumidity, hardwareData.batteryPercent, hardwareData.batteryVoltage, hardwareData.rssi);
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
    screenInfo("");
  }
  disableInternalPower(SAMPLE_INTERVAL);
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

bool getOWMCurrentWeatherData()
// stores local current weather info from Open Weather Map in environment global
{
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (internetAvailable)
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

bool getOWMAirPollution()
// stores local air pollution info from Open Weather Map in environment global
{
  #if defined(WIFI) || defined(RJ45)
    // if there is a network interface (so it will compile)
    if (internetAvailable)
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
  display.setCursor(40,(display.height()/2+6));
  display.print(messageText);

  //update display
  display.display();
#endif
}

void screenInfo(String messageText)
// Display environmental information on screen
{
#ifdef SCREEN  
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // screen size cheats

  int x_indoor_left_margin = (display.width()/20);
  int x_mid_point = (display.width()/2);
  int x_outdoor_left_margin = (display.width()*11/20);

  // borders
  // ThinkInk 2.9" epd is 296x128 pixels
  // label border
  display.drawLine(0,(display.height()*7/8),display.width(),(display.height()*7/8),EPD_GRAY);
  // splitting sensor vs. outside values
  display.drawLine(x_mid_point,0,x_mid_point,(display.height()*7/8),EPD_GRAY);
  
  // battery status
  screenBatteryStatus();

  // wifi status
  screenWiFiStatus();

  // Indoor
  int temperatureDelta = ((int)(sensorData.ambientTempF +0.5)) - ((int) (averageTempF + 0.5));
  int humidityDelta = ((int)(sensorData.ambientHumidity +0.5)) - ((int) (averageHumidity + 0.5));

  // Indoor temp
  display.setFont(&FreeSans24pt7b);
  display.setCursor(x_indoor_left_margin,(display.height()/3));
  display.print(String((int)(sensorData.ambientTempF+0.5)));
  // move the cursor to raise the F indicator
  //display.setCursor(x,y);
  display.setFont(&meteocons16pt7b);
  display.print("+");

  // Indoor temp delta
  if (temperatureDelta!=0)
  {
    display.setFont();
    if(temperatureDelta>0)
    {
      // upward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(110,((display.height()*3/8)-10),130,((display.height()*3/8)-10),120,(((display.height()*3/8)-10)-9), EPD_BLACK);
    }
    else
    {
      // downward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(110,(((display.height()*3/8)-10)-9),130,(((display.height()*3/8)-10)-9),120,((display.height()*3/8)-10), EPD_BLACK);
    }
    display.setCursor(130,((display.height()*3/8)-10));
    display.print(abs(temperatureDelta));
  }

  // Indoor humidity
  display.setFont(&FreeSans12pt7b);
  display.setCursor(x_indoor_left_margin,((display.height()*9/16)));
  display.print(String((int)(sensorData.ambientHumidity+0.5)) + "%");
  // Indoor humidity delta
  if (humidityDelta!=0)
  {
    display.setFont();
    if(humidityDelta>0)
    {
      // upward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(110,((display.height()*5/8)-10),130,((display.height()*5/8)-10),120,(((display.height()*5/8)-10)-9), EPD_BLACK);
    }
    else
    {
      // (left pt, right pt, bottom pt)
      display.fillTriangle(110,(((display.height()*5/8)-10)-9),130,(((display.height()*5/8)-10)-9),120,((display.height()*5/8)-10), EPD_BLACK);
    }
    display.setCursor(130,((display.height()*5/8)-10));
    display.print(abs(humidityDelta));
  }

  // Indoor CO2 level
  if (sensorData.ambientCO2!=10000)
  {
    // calculate CO2 value range in 400ppm bands
    int co2range = ((sensorData.ambientCO2 - 400) / 400);
    co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
    display.setFont(&FreeSans12pt7b);
    display.setCursor(x_indoor_left_margin,(display.height()*13/16));
    display.setFont(&FreeSans9pt7b); 
    display.print(String(co2Labels[co2range])+ " CO2");
    if ((sensorData.ambientCO2-averageCO2)!=0)
    {
      display.setFont();
      if(sensorData.ambientCO2-averageCO2>0)
      {
      // upward triangle (left pt, right pt, bottom pt)
      display.fillTriangle(110,((display.height()*7/8)-10),130,((display.height()*7/8)-10),120,(((display.height()*7/8)-10)-9), EPD_BLACK);
      }
      else
      {
        // (left pt, right pt, bottom pt)
        display.fillTriangle(110,(((display.height()*7/8)-10)-9),130,(((display.height()*7/8)-10)-9),120,((display.height()*7/8)-10), EPD_BLACK);
      }
      display.setCursor(130,((display.height()*7/8)-10));
      display.print(abs(sensorData.ambientCO2 - averageCO2));
    }
  }

  // Outside
  // location label
  display.setFont();
  display.setCursor((display.width()*5/8),((display.height()*1/8)-11));
  display.print(owmCurrentData.cityName);

  // Outside temp
  display.setFont(&FreeSans12pt7b);
  if (owmCurrentData.temp!=10000)
  {
    display.setCursor(x_outdoor_left_margin,(display.height()/4));
    display.print(String((int)(owmCurrentData.temp+0.5)));
    display.setFont(&meteocons12pt7b);
    display.print("+");
  }

  // weather icon
  String weatherIcon = getMeteoconIcon(owmCurrentData.icon);
  // if getMeteoIcon doesn't have a matching symbol, skip display
  if (weatherIcon!=")")
  {
    // display icon
    display.setFont(&meteocons20pt7b);
    display.setCursor((display.width()*4/5),(display.height()/2));
    display.print(weatherIcon);
  }

  // Outside humidity
  display.setFont(&FreeSans12pt7b);
  if (owmCurrentData.humidity!=10000)
  {
    display.setCursor(x_outdoor_left_margin,(display.height()*9/16));
    display.print(String(owmCurrentData.humidity) + "%");
  }

  // Outside air quality index (AQI)
  display.setFont(&FreeSans9pt7b);
  if (owmAirQuality.aqi!=10000)
  {
    display.setCursor((x_outdoor_left_margin),(display.height()*13/16));
    // European standards-body value
    //display.print(aqiLabels[(owmAirQuality.aqi-1)]);
    // US standards-body value
    display.print(aqiLabels[int((pm25toAQI(owmAirQuality.pm25)-1))]);
    display.print(" AQI");
  }

  // status message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()-9));
  display.print(messageText);

  //update display
  display.display();
  debugMessage("Screen updated");
#endif
}

void screenWiFiStatus()
// helper function for XXXScreen() routines that draws WiFi signal strength
{
  if (internetAvailable) 
  {
    const int barWidth = 3;
    const int barHeightMultiplier = 3;
    const int barSpacingMultipler = 5;
    const int barStartingXModifier = 70;
    int barCount;

    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    barCount = (6-((hardwareData.rssi/10)-3));
    if (barCount>5) barCount = 5;
    if (barCount>0)
    {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++)
      {
        // display.fillRect(((display.width() - 70) + (b * 5)), ((display.height()) - (b * 5)), barWidth, b * 5, EPD_BLACK);
        display.fillRect(((display.width() - barStartingXModifier) + (b * barSpacingMultipler)), ((display.height()) - (b * barHeightMultiplier)), barWidth, b * barHeightMultiplier, EPD_BLACK);
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

void screenBatteryStatus()
// helper function for XXXScreen() routines that draws remaining battery %
{
#ifdef SCREEN
  if (batteryVoltageAvailable) 
  {
    int barHeight = 10;
    int barWidth = 28;

    // battery nub (3pix wide, 6pix high)
    display.drawRect((display.width()-5-3),((display.height()*7/8)+7),3,6,EPD_BLACK);
    //battery percentage as rectangle fill
    display.fillRect((display.width()-barWidth-5-3),((display.height()*7/8)+5),(int((hardwareData.batteryPercent/100)*barWidth)),barHeight,EPD_GRAY);
    // battery border
    display.drawRect((display.width()-barWidth-5-3),((display.height()*7/8)+5),barWidth,barHeight,EPD_BLACK);
  }
#endif
}

void batteryReadVoltage()
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
    batteryVoltageAvailable = true;
  } 
  else
  {
  // use supported boards to read voltage
    #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
      // see project supporting material for other ways to quantify battery %
      pinMode(VBATPIN,INPUT);
      #define BATTV_MAX           4.2     // maximum voltage of battery
      #define BATTV_MIN           3.2     // what we regard as an empty battery

      // assumes default ESP32 analogReadResolution (4095)
      // the 1.05 is a fudge factor original author used to align reading with multimeter
      hardwareData.batteryVoltage = ((float)analogRead(VBATPIN) / 4095) * 3.3 * 2 * 1.05;
      hardwareData.batteryPercent = (uint8_t)(((hardwareData.batteryVoltage - BATTV_MIN) / (BATTV_MAX - BATTV_MIN)) * 100);

      batteryVoltageAvailable = true;
    #endif
  }
  if (batteryVoltageAvailable) 
  {
    debugMessage("Battery voltage: " + String(hardwareData.batteryVoltage) + " v");
    debugMessage("Battery percentage: " + String(hardwareData.batteryPercent) + " %");
  }
}

int initSensor()
// initializes environment sensor if available. Supports SCD40, ATHX0, BME280 sensors
{

  #ifdef SCD40
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
      debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()");
      return 0;
    }
    else 
    {
      debugMessage("SCD40 initialized, waiting 5 sec for first measurement");
      delay(5000);  // Give SCD40 time to warm up
      return 1; // success
    }
  #else
    // ATHX0, BME280
    if (envSensor.begin())
    {
      // ID of 0x56-0x58 or 0x60 is a BME 280, 0x61 is BME680, 0x77 is BME280 on ESP32S2 Feather
      debugMessage(String("Environment sensor ready, ID is: ")+envSensor.sensorID());
      return 1;
    }
    else
    {
      return 0;
    }
  #endif
}

uint16_t readSensor()
// stores environment sensor to environment global
{

  #ifdef SCD40
    uint16_t error;
    char errorMessage[256];

    for (int loop=1; loop<=READS_PER_SAMPLE; loop++)
    {
      // minimum time between SCD40 reads
      delay(5000);
      // read and store data if successful
      error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity);
      // handle SCD40 errors
      if (error) {
        errorToString(error, errorMessage, 256);
        debugMessage(String(errorMessage) + " during SCD4X read");
        return 0;
      }
      if (sensorData.ambientCO2<440 || sensorData.ambientCO2>6000)
      {
        debugMessage("SCD40 CO2 reading out of range");
        return 0;
      }
      //convert C to F for temp
      sensorData.ambientTempF = (sensorData.ambientTempF * 1.8) + 32;
      debugMessage(String("SCD40 read ") + loop + " of 5: " + sensorData.ambientTempF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm");
    }
    return 1;
  #else
    // AHTX0, BME280
    sensors_event_t temp_event, humidity_event;
    envSensor_temp->getEvent(&temp_event);
    envSensor_humidity->getEvent(&humidity_event);
   
    sensorData.ambientTempF = (temp_event.temperature * 1.8) +32;
    sensorData.ambientHumidity = humidity_event.relative_humidity;
    sensorData.ambientCO2 = 10000;
    return 1;
  #endif
}

int readNVStorage() 
// reads data from non-volatile storage and stores in appropriate global variables
{
  int storedCounter;
  float storedTempF;
  float storedHumidity;

  nvStorage.begin("air-quality", false);
  // get previously stored values. If they don't exist, create them as zero
  storedCounter = nvStorage.getInt("counter", 1);
  // read value or insert current sensor reading if this is the first read from nv storage
  storedTempF = nvStorage.getFloat("temp", sensorData.ambientTempF);
  // BME280 often issues nan when not configured properly
  if (isnan(storedTempF)) {
    // bad value, replace with current temp
    averageTempF = (sensorData.ambientTempF * storedCounter);
    debugMessage("Unexpected tempF value in nv storage replaced with multiple of current temperature");
  } else {
    // good value, pass it along
    averageTempF = storedTempF;
  }
  storedHumidity = nvStorage.getFloat("humidity", sensorData.ambientHumidity);
  if (isnan(storedHumidity)) {
    // bad value, replace with current temp
    averageHumidity = (sensorData.ambientHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity");
  } else {
    // good value, pass it along
    averageHumidity = storedHumidity;
  }
  averageCO2 = nvStorage.getUInt("co2", sensorData.ambientCO2);
  debugMessage(String("Intermediate values FROM nv storage: Temp:") + averageTempF + ", Humidity:" + averageHumidity + ", CO2:" + averageCO2);
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter);
  return storedCounter;
}

void enableInternalPower()
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

void disableInternalPower(int deepSleepTime)
// Powers down hardware activated via enableInternalPower() then deep sleep MCU
{
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  aq_network.networkStop();

  uint16_t error;
  char errorMessage[256];

  // stop potentially previously started measurement
  error = envSensor.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    debugMessage(errorMessage);
  }
  envSensor.powerDown();

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

  debugMessage(String("Going to sleep for ") + deepSleepTime + " seconds");
  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  esp_deep_sleep_start();
}

String getMeteoconIcon(String icon)
// Maps OWM icon data to the appopropriate Meteocon font character 
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