// conditional compile flags
//#define DEBUG 	// Output to serial port
//#define RJ45  	// use Ethernet
#define WIFI    	// use WiFi
#define MQTTLOG 	// log sensor data to MQTT broker
//#define DWEET     // Post sensor readings to dweet.io
#define INFLUX  	// Log data to remote InfluxDB server
#define	SCREEN		// use screen as output

// sample timing in minutes
#ifdef DEBUG
	#define SAMPLE_INTERVAL 1
#else
	#define SAMPLE_INTERVAL 10
#endif
// number of samples captured before logging
#ifdef DEBUG
  #define SAMPLE_SIZE 1
#else
  #define SAMPLE_SIZE 6
#endif

#define WIFI_ATTEMPT_LIMIT	5 // max connection attempts to WiFi AP

// millisecond modifier to minutes for sampling interval (ARM)
// #define SAMPLE_INTERVAL_ARM_MODIFIER 60000
// microsecond modifier to minutes for sampling interval (ESP)
#define SAMPLE_INTERVAL_ESP_MODIFIER 60000000

// Open Weather Map parameters
#define OWM_SERVER			"http://api.openweathermap.org/data/2.5/"
#define OWM_WEATHER_PATH	"weather?"
#define OWM_AQM_PATH		"air_pollution?"

// select time zone, used by NTPClient
//const int timeZone = 0;  	// UTC
//const int timeZone = -5;  // USA EST
//const int timeZone = -4;  // USA EDT
//const int timeZone = -8;  // USA PST
const int timeZone = -7;  // USA PDT

// Battery parameters
// based on a settings curve in the LC709203F datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
// #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// set client ID; used by mqtt and wifi
//#define CLIENT_ID "AQ-test-room"
//#define CLIENT_ID "AQ-lab-office"
//#define CLIENT_ID "AQ-kitchen"
#define CLIENT_ID "AQ-cellar"
//#define CLIENT_ID "AQ-master-bedroom"

#ifdef MQTTLOG
	// set MQTT parameters
	#define MQTT_ATTEMPT_LIMIT 	3 	// max connection attempts to MQTT broker

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/pocket-office.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/pocket-office.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/pocket-office.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/pocket-office.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/pocket-office.rssi"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/master-bedroom.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/master-bedroom.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/master-bedroom.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/master-bedroom.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/master-bedroom.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/master-bedroom.rssi"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/lab-office.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/lab-office.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/lab-office.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/lab-office.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/lab-office.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/lab-office.rssi"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/kitchen.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/kitchen.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/kitchen.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/kitchen.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/kitchen.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/kitchen.rssi"

	#define MQTT_PUB_TOPIC1		"sircoolio/feeds/cellar.temperature"
	#define MQTT_PUB_TOPIC2		"sircoolio/feeds/cellar.humidity"
	#define MQTT_PUB_TOPIC3		"sircoolio/feeds/cellar.co2"
	#define MQTT_PUB_TOPIC4		"sircoolio/feeds/cellar.battery-level"
	#define MQTT_PUB_TOPIC5		"sircoolio/feeds/cellar.battery-voltage"
	#define MQTT_PUB_TOPIC6		"sircoolio/feeds/cellar.rssi"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/test-room.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/test-room.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/test-room.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/test-room.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/test-room.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/test-room.rssi"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/test-headless.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/test-headless.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/test-headless.co2"
	// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/test-headless.battery-level"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/test-headless.battery-voltage"
	// #define MQTT_PUB_TOPIC6		"sircoolio/feeds/test-headless.rssi"

#endif

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
  
	// Standard set of tag values used for each sensor data point stored to InfluxDB.  Reuses
  // CLIENT_ID as defined anove here in config.h as well as device location (e.g., room in 
  // the house) and site (indoors vs. outdoors, typically).
	//#define DEVICE_LOCATION "test room"
	//#define DEVICE_LOCATION "kitchen"
	#define DEVICE_LOCATION "cellar"
	//#define DEVICE_LOCATION "lab-office"
	//#define DEVICE_LOCATION "master bedroom"

	#define DEVICE_SITE "indoor"
	#define DEVICE_TYPE "air quality"

	#define INFLUX_ATTEMPT_LIMIT 	5 	// max connection attempts to Influxdb
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