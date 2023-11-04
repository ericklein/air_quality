/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

// Configuration Step 1: Set debug message output
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 1

// Configuration Step 2: Set network transport, if desired
//#define RJ45  	// use Ethernet
#define WIFI    	// use WiFi

// Configuration Step 3: Set network data endpoints
// these require a network transport from Step 2
//  #define MQTT 		// log sensor data to MQTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
// #define DWEET // Post sensor readings to dweet.io
#define INFLUX 	// Log data to remote InfluxDB server

// Configuration Step 4: Select environment sensor and configure read intervals
#define SCD40		// use SCD40 to read temperature, humidity, and CO2
// #define BME280	// use BME280 to read temperature and humidity
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
#define	SCREEN		// use screen as output

// Pin config for e-paper display
#ifdef SCREEN
	#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
		// new primary development hardware
		#define EPD_CS      12
    #define EPD_DC      27     
    #define SRAM_CS     14  // can set to -1 to not use a pin, which uses a lot of RAM
    #define EPD_RESET   15  // can set to -1 and share with MCU Reset, can't deep sleep
    #define EPD_BUSY    32  // can set to -1 to not use a pin and wait a fixed delay
  #else
	// Adafruit MagTag, some values come from board definition package
		// primary production build target, being replaced with ESP32V2 and epd friend plus screen
	#define EPD_CS      8   
	#define EPD_DC      7   
	#define SRAM_CS     -1  // can set to -1 to not use a pin, which uses a lot of RAM
	// #define EPD_RESET   6   // can set to -1 and share with MCU Reset, can't deep sleep
	#define EPD_BUSY    5   // can set to -1 to not use a pin and wait a fixed delay
  #endif
#endif

// Configuration Step 6: Set battery size, if applicable
// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
//  #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// battery pin for Adafruit ESP32V2 used for reading battery voltage
// used for reading battery voltage from analog PIN on applicable devices
#define VBATPIN A13
const int   batteryReads = 5;

// Configuration Step 7: Set parameters for NTP time configuration
// this will only be used if network transport is defined in Step 2
#define ntpServer "pool.ntp.org"
#define timeZoneString "PST8PDT,M3.2.0,M11.1.0" // America/Los_Angeles

#ifdef INFLUX  
	// Specify Measurement to use with InfluxDB for sensor and device info
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
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

// battery charge level lookup table
const float voltageTable[101] = {
  3.200,  3.250,  3.300,  3.350,  3.400,  3.450,
  3.500,  3.550,  3.600,  3.650,  3.700,  3.703,
  3.706,  3.710,  3.713,  3.716,  3.719,  3.723,
  3.726,  3.729,  3.732,  3.735,  3.739,  3.742,
  3.745,  3.748,  3.752,  3.755,  3.758,  3.761,
  3.765,  3.768,  3.771,  3.774,  3.777,  3.781,
  3.784,  3.787,  3.790,  3.794,  3.797,  3.800,
  3.805,  3.811,  3.816,  3.821,  3.826,  3.832,
  3.837,  3.842,  3.847,  3.853,  3.858,  3.863,
  3.868,  3.874,  3.879,  3.884,  3.889,  3.895,
  3.900,  3.906,  3.911,  3.917,  3.922,  3.928,
  3.933,  3.939,  3.944,  3.950,  3.956,  3.961,
  3.967,  3.972,  3.978,  3.983,  3.989,  3.994,
  4.000,  4.008,  4.015,  4.023,  4.031,  4.038,
  4.046,  4.054,  4.062,  4.069,  4.077,  4.085,
  4.092,  4.100,  4.111,  4.122,  4.133,  4.144,
  4.156,  4.167,  4.178,  4.189,  4.200 };