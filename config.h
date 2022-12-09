/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// Step 1: Set conditional compile flags
// #define DEBUG 	// Output to serial port
//#define RJ45  	// use Ethernet
#define WIFI    	// use WiFi
#define MQTT 	// log sensor data to MQTT broker
//#define DWEET     // Post sensor readings to dweet.io
#define INFLUX  	// Log data to remote InfluxDB server
#define	SCREEN		// use screen as output
#define SCD40			// use SCD40 to read temp, humidity, and CO2

// Step 2: Set battery size if applicable
// based on a settings curve in the LC709203F datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
#define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
// #define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// Pin config for e-paper display

// Adafruit MagTag, some values come from board definition package
#define EPD_CS      8   
#define EPD_DC      7   
#define SRAM_CS     -1  // can set to -1 to not use a pin (uses a lot of RAM!)
// #define EPD_RESET   6   // can set to -1 and share with chip Reset (can't deep sleep)
#define EPD_BUSY    5   // can set to -1 to not use a pin (will wait a fixed delay)

// ESP32-S2 with Adafruit 2.9" E-Ink Featherwing (PID 4777)
#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
	#define EPD_CS      9
	#define EPD_DC      10     
	#define SRAM_CS     6  // can set to -1 to not use a pin (uses a lot of RAM!)
	// on Featherwing EPD_RESET and EPD_BUSY must be set to -1 as these lines are not connected
	#define EPD_RESET   -1
	#define EPD_BUSY    -1
	// #define EPD_RESET   8 // can set to -1 and share with microcontroller Reset!
	// #define EPD_BUSY    7 // can set to -1 to not use a pin (will wait a fixed delay)
#endif

// Interval between enviroment sensor samples in minutes
#ifdef DEBUG
	#define SAMPLE_INTERVAL 1
#else
	#define SAMPLE_INTERVAL 10
#endif

// Sleep time if hardware error occurs in seconds
#define HARDWARE_ERROR_INTERVAL 1

// number of samples captured before logging
#ifdef DEBUG
  #define SAMPLE_SIZE 2
#else
  #define SAMPLE_SIZE 6
#endif

#define CONNECT_ATTEMPT_LIMIT	3 // max connection attempts to internet services
#define CONNECT_ATTEMPT_INTERVAL 10 // seconds between internet service connect attempts

const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// used and defined by OWM
const String aqiLabels[5] = { "Good", "Fair", "Moderate", "Poor", "Very Poor" };
// used in aq_network.cpp
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// Open Weather Map parameters
#define OWM_SERVER			"http://api.openweathermap.org/data/2.5/"
#define OWM_WEATHER_PATH	"weather?"
#define OWM_AQM_PATH		"air_pollution?"

// NTP time configuration

//https://cplusplus.com/reference/ctime/tm/

#define ntpServer "pool.ntp.org"
// gmtOffset_sec is a backup used if OWM data not available
// const long  gmtOffset_sec = 0; // UTC
// const long  gmtOffset_sec = 3600; // Ireland
const long  gmtOffset_sec = -28800; // PST
const int   daylightOffset_sec = 0;
// const int   daylightOffset_sec = 3600; // US DT

// set client ID; used by mqtt and wifi
// #define CLIENT_ID "AQ-demo"
//#define CLIENT_ID "AQ-test"
#define CLIENT_ID "AQ-lab-office"
// #define CLIENT_ID "AQ-kitchen"
//#define CLIENT_ID "AQ-cellar"
//#define CLIENT_ID "AQ-master-bedroom"

#ifdef MQTT
	// Adafruit I/O
	// structure: username/feeds/groupname.feedname or username/feeds/feedname
	// e.g. #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"
	
	// structure: site/room/device/data	
	#define MQTT_PUB_TOPIC1		"7828/lab-office/aq/temperature"
	#define MQTT_PUB_TOPIC2		"7828/lab-office/aq/humidity"
	#define MQTT_PUB_TOPIC3		"7828/lab-office/aq/co2"
	#define MQTT_PUB_TOPIC5		"7828/lab-office/aq/battery-voltage"
	#define MQTT_PUB_TOPIC6		"7828/lab-office/aq/rssi"
#endif

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
  
	// Standard set of tag values used for each sensor data point stored to InfluxDB.  Reuses
  // CLIENT_ID as defined anove here in config.h as well as device location (e.g., room in 
  // the house) and site (indoors vs. outdoors, typically).
	// #define DEVICE_LOCATION "AQ-demo"
  //#define DEVICE_LOCATION "test"
	// #define DEVICE_LOCATION "kitchen"
	// #define DEVICE_LOCATION "cellar"
	#define DEVICE_LOCATION "lab-office"
	//#define DEVICE_LOCATION "master bedroom"

	#define DEVICE_SITE "indoor"
	#define DEVICE_TYPE "air quality"
#endif

// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
// unique name you want associated with this reporting device, allowing
// data to be easily retrieved through the web or Dweet's REST API.
#ifdef DWEET
	#define DWEET_HOST "dweet.io"   // Typically dweet.io
	#define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif

// The following parameters are defined in secrets.h.
// 	WiFi credentials (if WiFi enabled)
// 	#define WIFI_SSID
// 	#define WIFI_PASS       

// 	Open Weather Map
// 	#define OWM_KEY
//	#define OWM_LAT_LONG  // example "lat=47.9001&lon=-122.4001"

//	#define SITE_ELEVATION // in meters

// If MQTT enabled
// 	#define MQTT_PORT
// 	#define MQTT_USER
// 	#define MQTT_BROKER
// 	#define MQTT_PASS

// If InfluxDB data storage enabled
// For an InfluxDB v1.X server:
// #define INFLUX_V1
// #define INFLUXDB_URL 
// #define INFLUXDB_DB_NAME
// #define INFLUXDB_USER
// #define INFLUXDB_PASSWORD
//
// For an InfluxDB v2.X server:
// #define INFLUX_V2
// #define INFLUXDB_URL 
// #define INFLUXDB_TOKEN
// #define INFLUXDB_ORG
// #define INFLUXDB_BUCKET