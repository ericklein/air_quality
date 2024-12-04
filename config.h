/*
  Project Name:   air_quality
  Description:    public (non-secret) configuration data
*/	

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug message output
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 2

// Configuration Step 3: simulate WiFi and sensor hardware,
// returning random but plausible values
// comment out to turn off
// #define HARDWARE_SIMULATE

// Configuration Step 4: Set network data endpoints
 // #define MQTT 		// log sensor data to MQTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
// #define DWEET // Post sensor readings to dweet.io
// #define INFLUX 	// Log data to remote InfluxDB server

// Configuration Step 5: Set battery size, if applicable
// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
 // #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// Configuration variables that change rarely

// Time
// NTP time parameters
const String networkNTPAddress = "pool.ntp.org";
const String networkTimeZone = "PST8PDT,M3.2.0,M11.1.0"; // America/Los_Angeles
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// network endpoints
#ifdef INFLUX  
	// Specify Measurement to use with InfluxDB for sensor and device info
	const String influxEnvMeasurement = "weather";  // Used for environmental sensor data
	const String influxDevMeasurement =  "device";   // Used for logging AQI device data (e.g. battery)
#endif

#ifdef DWEET
	// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
	// unique name you want associated with this reporting device, allowing
	// data to be easily retrieved through the web or Dweet's REST API.
	#define DWEET_HOST "dweet.io"   // Typically dweet.io
	#define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif

// Open Weather Map (OWM)
#define OWM_SERVER			"http://api.openweathermap.org/data/2.5/"
#define OWM_WEATHER_PATH	"weather?"
#define OWM_AQM_PATH		"air_pollution?"
const String OWMAQILabels[5] = {"Good", "Fair", "Moderate", "Poor", "Very Poor"};

// Sample and reporting intervals
#ifdef DEBUG
	// time between samples in seconds
	const uint16_t sensorSampleInterval = 60;
	// number of samples to average. this is also the # of uint_16 CO2 samples saved to nvStorage, so limit this
  const uint8_t sensorSampleSize = 4;
#else
	const uint16_t sensorSampleInterval = 300;
  const uint8_t sensorSampleSize = 6;
#endif

// warnings
const String warningLabels[4]={"Good", "Fair", "Poor", "Bad"};
// Subjective color scheme using 16 bit ('565') RGB colors
// const uint16_t warningColor[4] = {
//   0x07E0, // Green = "Good"
//   0xFFE0, // Yellow = "Fair"
//   0xFD20, // Orange = "Poor"
//   0xF800  // Red = "Bad"
//   };

// Hardware

// Simulation boundary values
#ifdef HARDWARE_SIMULATE
  const uint16_t sensorTempMinF =       1400; // divided by 100.0 to give floats
  const uint16_t sensorTempMaxF =       14000;
  const uint16_t sensorHumidityMin =    0; // RH%, divided by 100.0 to give float
  const uint16_t sensorHumidityMax =    10000;

  const uint8_t OWMAQIMin = 1;  // https://openweathermap.org/api/air-pollution
  const uint8_t OWMAQIMax = 5;

  const uint16_t OWMPM25Min = 0;  // will be divided by 100.0 to give float
  const uint32_t OWMPM25Max = 10000; // will be divided by 100.0 to give float

  const uint16_t batterySimVoltageMin = 370; // will be divided by 100.0 to give floats
  const uint16_t batterySimVoltageMax = 410;

  const uint8_t networkRSSIMin = 30;
  const uint8_t networkRSSIMax = 90;
#endif

// CO2 from SCD4x
// CO2 value thresholds for labeling
const uint16_t co2Fair = 800;
const uint16_t co2Poor = 1200;
const uint16_t co2Bad = 2000;

const uint16_t sensorCO2Min =      400;   // in ppm
const uint16_t sensorCO2Max =      2000;  // in ppm
const uint8_t sensorTempCOffset = 0;     // in Celcius

// particulates (pm1, pm2.5, pm4, pm10) from SEN5x
// CO2 value thresholds for labeling
const uint16_t pmFair = 25;
const uint16_t pmPoor = 50;
const uint16_t pm2Bad = 150;

// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;

//Battery 
// analog pin used to reading battery voltage
#define BATTERY_VOLTAGE_PIN A13
// number of analog pin reads sampled to average battery voltage
const uint8_t   batteryReadsPerSample = 5;
// battery charge level lookup table
const float batteryVoltageTable[101] = {
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

// display
// Pin config for e-paper display
#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
	// new primary development hardware
	#define EPD_CS      33 
	#define EPD_DC      27     
	#define SRAM_CS     26  // can set to -1 to not use a pin, which uses a lot of RAM
	#define EPD_RESET   25  // can set to -1 and share with MCU Reset, can't deep sleep
	#define EPD_BUSY    32  // can set to -1 to not use a pin and wait a fixed delay
  const uint8_t displayRotation = 1; // rotation 1 means x0,y0 is away from flex cable, right aligned with flex cable label
#else
	// Adafruit MagTag, some values come from board definition package
	// primary production build target, being replaced with ESP32V2 and epd friend plus screen
	#define EPD_CS      8   
	#define EPD_DC      7   
	#define SRAM_CS     -1  // can set to -1 to not use a pin, which uses a lot of RAM
	// #define EPD_RESET   6   // can set to -1 and share with MCU Reset, can't deep sleep
	#define EPD_BUSY    5   // can set to -1 to not use a pin and wait a fixed delay
  const uint8_t displayRotation = 3; // MagTag screen
#endif


// buttons
// MagTag BUTTON_A = 15, BUTTON_B = 14, BUTTON_C = 12, BUTTON_D = 11 (all RTC pins)
#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
	#define BUTTON_A 34; // A2, input only
	#define BUTTON_B 39; // A3, input only
	#define BUTTON_C 36; // A4, input only
#endif

// ext1 wakeup (multiple buttons via bitmask)
//#define BUTTON_PIN_BITMASK 0xD800 // All buttons; 2^15+2^14+2^12+2^11 in hex
// #define BUTTON_PIN_BITMASK 0xC000 // Buttons A,B; 2^15+2^14 in hex
// #define BUTTON_PIN_BITMASK 0x9800 // Buttons A,C,D; 2^15+2^12+2^11 in hex
// #define BUTTON_PIN_BITMASK 0x8800 // Buttons A,D; 2^15+2^11 in hex
// #define BUTTON_PIN_BITMASK 0x1800 // Buttons C,D; 2^12+2^11 in hex
// #define BUTTON_PIN_BITMASK 0x8000 // Button A; 2^15 in hex

// Network
// max connection attempts to network services
const uint8_t networkConnectAttemptLimit = 3;
// seconds between network service connect attempts
const uint8_t networkConnectAttemptInterval = 10;