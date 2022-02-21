// conditional compile flags
//#define DEBUG     // Output to serial port
//#define RJ45    // use Ethernet
#define WIFI    // use WiFi
#define MQTTLOG // log sensor data to MQTT broker
#define DWEET     // Post sensor readings to dweet.io
#define INFLUX  // Log data to remote InfluxDB server

// set logging interval in minutes
#ifdef DEBUG
	#define LOG_INTERVAL 1
#else
	#define LOG_INTERVAL 30
#endif

// millisecond modifier to minutes for logging interval (ARM)
// #define LOG_INTERVAL_MS_MODIFIER 60000
// microsecond modifier to minutes for logging interval (ESP)
#define LOG_INTERVAL_US_MODIFIER 60000000

// set device ID; used by mqtt and screen
#define CLIENT_ID "test_room"

// open weather map parameters
#define OWM_SERVER 			  "http://api.openweathermap.org/data/2.5/"
#define OWM_WEATHER_PATH	"weather?"
#define OWM_AQM_PATH		  "air_pollution?"
#define OWM_LAT_LONG		  "lat=47.5707&lon=-122.2221"

// select time zone
//const int timeZone = 0;  	// UTC
//const int timeZone = -5;  // USA EST
//const int timeZone = -4;  // USA EDT
const int timeZone = -8;  // USA PST
//const int timeZone = -7;  // USA PDT

// set MQTT parameters
// #define MQTT_KEEP_ALIVE 	300 // needed?
#define MQTT_ATTEMPT_LIMIT 	3 	// number of connection attempts for MQTT broker

// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"
// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/pocket-office.humidity"
// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/pocket-office.co2"
// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/pocket-office.errmessage"

// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/master-bedroom.temperature"
// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/master-bedroom.humidity"
// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/master-bedroom.co2"
// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/master-bedroom.errmessage"

// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/lab-office.temperature"
// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/lab-office.humidity"
// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/lab-office.co2"
// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/lab-office.errmessage"

// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/kitchen.temperature"
// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/kitchen.humidity"
// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/kitchen.co2"
// #define MQTT_PUB_TOPIC4		"sircoolio/feeds/kitchen.errmessage"

#define MQTT_PUB_TOPIC1		"sircoolio/feeds/test-room.temperature"
#define MQTT_PUB_TOPIC2		"sircoolio/feeds/test-room.humidity"
#define MQTT_PUB_TOPIC3		"sircoolio/feeds/test-room.co2"
#define MQTT_PUB_TOPIC4		"sircoolio/feeds/test-room.errmessage"

// Current version of Adafruit LC709203F library sets battery size (APA value) 
// using a fixed, limited set of defined values enumerated in LC709203F.h.  
// Select the one closest to the battery size being used
// #define BATTERYSIZE LC709203F_APA_100MAH // 0x08
// #define BATTERYSIZE LC709203F_APA_200MAH // 0x0B
// #define BATTERYSIZE LC709203F_APA_500MAH	// 0x10
#define BATTERYSIZE LC709203F_APA_1000MAH // 0x19
// #define BATTERYSIZE LC709203F_APA_2000MAH // 0x2D
// #define BATTERYSIZE LC709203F_APA_3000MAH // 0x36

// A pending update to the Adafruit LC709203F library adds a new function that
// allows setitng battery APA value directly, based on a settings curve in the
// LC709203F datasheet.  Once that version becomes available we'll switch, which
// will require a differernt #define scheme.
// #define BATTERY_APA 0x1D // 1200 mAH per LC709203F datasheet

// Set the threshold for generating a low-battery alert via MQTT
#define BATTERY_ALERT_PCT 20  // Low battery alert at or below 20% charge

// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
// unique name you want associated with this reporting device, allowing
// data to be easily retrieved through the web or Dweet's REST API.
#ifdef DWEET
  #define DWEET_HOST "dweet.io"   // Typically dweet.io
  #define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif

#ifdef INFLUX
  // InfluxDB server url using name or IP address (not localhost)
  #define INFLUXDB_URL "http://jakku.local:8086"
  // InfluxDB v1 database name 
  #define INFLUXDB_DB_NAME "home"
  // InfluxDB v1 user name
  #define INFLUXDB_USER "grafana"
  // InfluxDB v1 user password
  #define INFLUXDB_PASSWORD "anafarg1729"
  
  // Tags values for InfluxDB data points
  #define DEVICE_NAME "airquality"
  #define DEVICE_LOCATION "dining room"
  #define DEVICE_SITE "indoor"
#endif


// the following parameters are defined in secrets.h
// #ifdef WIFI
// 	// WiFi credentials
// 	#define WIFI_SSID
// 	#define WIFI_PASS       
// #endif

// 	// open weather map key
// 	#define OWM_KEY

// 	#define MQTT_PORT
// 	#define MQTT_USER
// 	#define MQTT_BROKER
// 	#define MQTT_PASS
