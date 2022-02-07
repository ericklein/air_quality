// conditional compile flags

//#define DEBUG     // Output to serial port
#define SCREEN      // output to screen
//#define MQTTLOG     // Output to MQTT broker
//#define RJ45      // use Ethernet
#define WIFI        // use WiFi
#define NTP         // query network time server
//#define BATTERY		// use battery monitoring hardware
#define CO2_SENSOR	// CO2, temp, humidity sensor instead of temp, humidity sensor
#define ONE_TIME	// run every x minutes and sleep
#define WEATHER  	// query weather sensor for outside conditions
#define DWEET     // Post sensor readings to dweet.io

// logging interval in minutes
#ifdef DEBUG
	#define LOG_INTERVAL 1
#else
	#define LOG_INTERVAL 30
#endif

// millisecond modifier to minutes for logging interval (ARM)
#define LOG_INTERVAL_MS_MODIFIER 60000
// microsecond modifier to minutes for logging interval (ESP)
#define LOG_INTERVAL_US_MODIFIER 60000000

// device ID used for MQTTLOG and SCREEN
#define CLIENT_ID "test_room"

#ifdef WEATHER
	// open weather map configuration
	#define OWM_SERVER "http://api.openweathermap.org/data/2.5/"
	#define OWM_WEATHER_PATH "weather?"
	#define OWM_AQM_PATH "air_pollution?"
	#define OWM_LAT_LONG "lat=47.5707&lon=-122.2221"
#endif

#ifdef MQTTLOG
	#define MQTT_KEEP_ALIVE 	300
	
	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/pocket-office.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/pocket-office.co2"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/pocket-office.errmessage"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/master-bedroom.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/master-bedroom.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/master-bedroom.co2"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/master-bedroom.errmessage"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/lab-office.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/lab-office.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/lab-office.co2"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/lab-office.errmessage"

	// #define MQTT_PUB_TOPIC1		"sircoolio/feeds/kitchen.temperature"
	// #define MQTT_PUB_TOPIC2		"sircoolio/feeds/kitchen.humidity"
	// #define MQTT_PUB_TOPIC3		"sircoolio/feeds/kitchen.co2"
	// #define MQTT_PUB_TOPIC5		"sircoolio/feeds/kitchen.errmessage"

	#define MQTT_PUB_TOPIC1		"sircoolio/feeds/test-room.temperature"
	#define MQTT_PUB_TOPIC2		"sircoolio/feeds/test-room.humidity"
	#define MQTT_PUB_TOPIC3		"sircoolio/feeds/test-room.co2"
	#define MQTT_PUB_TOPIC5		"sircoolio/feeds/test-room.errmessage"
#endif

#ifdef BATTERY
    // #define BATTERYSIZE LC709203F_APA_100MAH // 0x08
    // #define BATTERYSIZE LC709203F_APA_200MAH // 0x0B
    // #define BATTERYSIZE LC709203F_APA_500MAH	// 0x10
    #define BATTERYSIZE LC709203F_APA_1000MAH // 0x19
	  // #define BATTERYSIZE LC709203F_APA_2000MAH // 0x2D
    // #define BATTERYSIZE LC709203F_APA_3000MAH // 0x36
#endif

// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
// unique name you want associated with this reporting device, allowing
// data to be easily retrieved through the web or Dweet's REST API.
#ifdef DWEET
  #define DWEET_HOST "dweet.io"   // Typically dweet.io
  #define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif

// the following parameters are defined in secrets.h
// #ifdef WIFI
// 	// WiFi credentials
// 	#define WIFI_SSID
// 	#define WIFI_PASS       
// #endif

// #ifdef WEATHER
// 	// open weather map key
// 	#define OWM_KEY
// #endif

// #ifdef MQTTLOG
// 	#define MQTT_PORT
// 	#define MQTT_USER

// 	#define MQTT_BROKER
// 	#define MQTT_PASS
// #endif
