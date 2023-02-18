// The following parameters are defined in secrets.h

// 	WiFi credentials (if WiFi enabled)
// 	#define WIFI_SSID	"key_value"
// 	#define WIFI_PASS   "key_value"

// 	Open Weather Map
//	#define OWM_KEY 		"keyvalue"
// 	#define OWM_LAT_LONG	"lat=34.1448&lon=-118.1509" // Pasadena, CA
//	#define SITE_ALTITUDE	263 // calibrates SCD40, Pasadena, CA, in meters above sea level

// #ifdef MQTT
// 	#define MQTT_PORT  		port_number	// use 8883 for SSL
// 	#define MQTT_USER		"key_value"
// 	#define MQTT_BROKER		"ip_address"
// 	#define MQTT_PASS 		"key_value"
// #endif

// #ifdef INFLUX
	// For an InfluxDB v1.X server:
	// #define INFLUX_V1
	// #define INFLUXDB_URL 
	// #define INFLUXDB_DB_NAME
	// #define INFLUXDB_USER
	// #define INFLUXDB_PASSWORD

	// For an InfluxDB v2.X server:
	// #define INFLUX_V2 
	// #define INFLUXDB_URL "http://ip_address:port_number"
	// #define INFLUXDB_TOKEN "key_value"
	// #define INFLUXDB_ORG "key_value"
	// #define INFLUXDB_BUCKET "key_value"
// #endif