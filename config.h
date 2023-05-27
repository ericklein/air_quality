/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// Configuration Step 1: Set debug message output
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 2

// Configuration Step 2: Set network transport, if desired
//#define RJ45  	// use Ethernet
#define WIFI    	// use WiFi

// Configuration Step 3: Set network data endpoints
// these require a network transport from Step 2
#define MQTT 		// log sensor data to MQTT broker
//#define DWEET // Post sensor readings to dweet.io
#define INFLUX 	// Log data to remote InfluxDB server

// Configuration Step 4: Select environment sensor and configure read intervals
// #define SCD40		// use SCD40 to read temperature, humidity, and CO2
#define BME280	// use BME280 to read temperature and humidity
// #define AHTXX		// use AHT series device to read temperature and humidity

// environment sensor sample timing
#ifdef DEBUG
	// number of times sensor is read, last read is the sample value
	#define READS_PER_SAMPLE	1
	// time between samples in seconds
	#define SAMPLE_INTERVAL		60
	// number of samples to average. this is also the # of uint_16 CO2 samples saved to nvStorage, so limit this
  #define SAMPLE_SIZE				2
#else
	#ifdef SCD40 
		// SCD40 needs >= 5 samples to get to a reliable reading from a cold start
		#define READS_PER_SAMPLE	5
	#else
		#define READS_PER_SAMPLE	1
	#endif
	#define SAMPLE_INTERVAL 	300
  #define SAMPLE_SIZE 			6
#endif

// Configuration Step 5: Set screen parameters, if desired
// #define	SCREEN		// use screen as output

// Pin config for e-paper display
#ifdef SCREEN
  #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // ESP32-S2 with Adafruit 2.9" E-Ink Featherwing (PID 4777)
    #define EPD_CS      9
    #define EPD_DC      10     
    #define SRAM_CS     6  // can set to -1 to not use a pin (uses a lot of RAM!)
    // on Featherwing EPD_RESET and EPD_BUSY must be set to -1 as these lines are not connected
    #define EPD_RESET   -1
    #define EPD_BUSY    -1
    // #define EPD_RESET   8 // can set to -1 and share with microcontroller Reset!
    // #define EPD_BUSY    7 // can set to -1 to not use a pin (will wait a fixed delay)
  #else
    // Adafruit MagTag, some values come from board definition package
    #define EPD_CS      8   
    #define EPD_DC      7   
    #define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
    // #define EPD_RESET   6   // can set to -1 and share with chip Reset (can't deep sleep)
    #define EPD_BUSY    5   // can set to -1 to not use a pin (will wait a fixed delay)
  #endif
#endif

// Configuration Step 6: Set battery size, if applicable
// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
// #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// used for reading battery voltage from analog PIN on applicable devices
const float batteryMaxVoltage	= 4.2; 	// maximum battery voltage
const float batteryMinVoltage	= 3.2; 	// what we regard as an empty battery

// Configuration Step 7: Set parameters for NTP time configuration
// this will only be used if network transport is defined in Step 2
#define ntpServer "pool.ntp.org"
// const long  gmtOffset_sec = 0; // UTC
// const long  gmtOffset_sec = 3600; // Ireland
const long  gmtOffset_sec = -28800; // PST
// const int   daylightOffset_sec = 0;
const int   daylightOffset_sec = 3600; // US DT

// Configuration Step 8: Set network data endpoint parameters, if applicable

// set client ID; used by mqtt and wifi
// structure is AQ_room-name; e.g. AQ_kitchen
#define CLIENT_ID "AQ_cellar"

#ifdef MQTT
	// structure is site/structure/room/device/data
	#define MQTT_PUB_TEMPF			"7828/cellar/aq/temperature"
	#define MQTT_PUB_HUMIDITY	"7828/cellar/aq/humidity"
	#define MQTT_PUB_CO2				"7828/cellar/aq/co2"
	#define MQTT_PUB_BATTVOLT	"7828/cellar/aq/battery-voltage"
	#define MQTT_PUB_RSSI			"7828/cellar/aq/rssi"
#endif

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
  
	// Standard set of tag values used for each sensor data point stored to InfluxDB.  Reuses
  // CLIENT_ID as defined anove here in config.h as well as device location (e.g., room in 
  // the house) and site (indoors vs. outdoors, typically).
	// #define DEVICE_LOCATION "AQ-demo"
	// #define DEVICE_LOCATION "kitchen"
	#define DEVICE_LOCATION "cellar"
	// #define DEVICE_LOCATION "lab-office"
	// #define DEVICE_LOCATION "master bedroom"

	#define DEVICE_SITE "indoor"
	#define DEVICE_TYPE "air quality"
#endif

#ifdef DWEET
	// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
	// unique name you want associated with this reporting device, allowing
	// data to be easily retrieved through the web or Dweet's REST API.
	#define DWEET_HOST "dweet.io"   // Typically dweet.io
	#define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif

// Configuration variables that are less likely to require changes

// Sleep time if hardware error occurs in seconds
#define HARDWARE_ERROR_INTERVAL 10

#define CONNECT_ATTEMPT_LIMIT	3 // max connection attempts to internet services
#define CONNECT_ATTEMPT_INTERVAL 5 // seconds between internet service connect attempts

const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};

// if using OWM aqi value, these are the European standards-body conversions from numeric valeu
// const String aqiEuropeanLabels[5] = { "Good", "Fair", "Moderate", "Poor", "Very Poor" };

// US standards-body conversions from numeric value
const String aqiUSALabels[6] = {"Good", "Moderate", "Unhealthy (SG)", "Unhealthy", "Very Unhealthy", "Hazardous"};

// used in aq_network.cpp
const String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Open Weather Map parameters
#define OWM_SERVER			"http://api.openweathermap.org/data/2.5/"
#define OWM_WEATHER_PATH	"weather?"
#define OWM_AQM_PATH		"air_pollution?"
