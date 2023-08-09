### Purpose
Air Quality (aka AQ) samples temperature, humidity, and if connected to the appropriate sensor, CO2 (carbon dioxide) levels. It can log this data to a number of network endpoints. It can also display the local outdoor weather to compliment local sensor readings.

### Features
![Screenshot](readme/ui_mar23.jpg)
- #1 = Indoor air quality information
	- If available, displays the US standard CO2 label and value
	- Displays the temperature and humidity
- #2 = Outside air quality information
	- Uses OpenWeatherMap data, if available, to display
		- latest air quality information
		- current weather condition as icon
		- temperature and humidity levels
- #3 = Data publishing information
	- displays the last time the screen information was updated.
	- [+I] the indoor temperature, humidity, and CO2 information was successfully published to the defined InfluxDB server
	- [+M] the indoor temperature, humidity, and CO2 information was successfully published to the defined MQTT broker
- #4 = WiFi signal strength based on RSSI
- #5 = Battery level

### What is AQI Anyway?
It's worth noting that while most people have heard of an "Air Quality Index" from their local weather service or news, the calculation of AQI from sensor readings is less well known.  Sensors measure and report particulate concentrations in various size ranges, the
most widely used of which is "PM2.5", meaning airborne particulates of 2.5 microns in diameter or smaller, measured in micrograms per cubic meter.  
- The more particulates measured the worse the air quality and, eventually, the greater the danger to humans and animals.  Howeveer, the perceived quality of the air and the risk from exposure vary in a non-obvious way based on the actual PM2.5 values observed.
- That variability is what gave rise to the idea of an Air Quality Index in the
first place, though as is often the case in associating health factors and risk with environmental data different governing bodies and standards organizations have put forward different ways of calculating risk from sensor data.  You can read much more about this in the Wikipedia page for [Air Quality](https://en.wikipedia.org/wiki/Air_quality_index).
In the US, the Environmental Protection Agency (EPA) developed its own AQI measure, dividing the normal range of measured particulates and pollutants into six categories indicating increased levels of health concerns.  An overall AQI value is calculated from a piecewise linear function, with scaling and transition points defined by the EPA.  More details on that math are shared in the
Wikipedia page cited above, as well as this [post](https://forum.airnowtech.org/t/the-aqi-equation/169) on the AirNow tech forum.
- Most information is kept in the "supporting material" folder, which is not synched to GitHub. Maybe I should do that? ping me if you want more info on CO2 measurements, etc.

### Target configuration
- Important access settings like WiFi SSID and password, ThingSpeak keys, and InfluxDB credentials are contained in a `secrets.h` file that is not included in this repo.  Instead you'll find the file `secrets_template.h`, which should be copied to `secrets.h` and then edited to supply the right access credentials and configuration values to match your deployment environment.
- See config.h for parameter configuration

### Bill of Materials (BOM)
- MCU
	- ESP32
- Ethernet
	- uncomment #define RJ45, comment #define WIFI in config.h
	- Supported hardware
		- [Particle Ethernet Featherwing](https://www.adafruit.com/product/4003)
		- [Silicognition PoE Featherwing](https://www.crowdsupply.com/silicognition/poe-featherwing)
	- Technical References
		- https://docs.particle.io/datasheets/accessories/gen3-accessories/
		- https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
		- https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
		- https://www.arduino.cc/en/reference/ethernet
		- https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
- WiFi
	- comment #define RJ45, uncomment #define WIFI in config.h
	- Supported hardware
		- ESP32 based boards
- Environment Sensors
	- for SCD40 support, uncomment #define SCD40
	- for BME280 support, comment #define SCD40
	- AHT2x support
		- comment #define SCD40
		- look for AHTX0 in globals section of air_quality.ino
			- uncomment AHTX0 section
			- comment BME280 section 
	- Supported hardware
		- [AHT20 temp/humidity sensor](https://www.adafruit.com/product/4566) or similar products
		- [SCD40 temp/humidity/CO2 sensor](https://www.adafruit.com/product/5187)
		- [BME280 temp/humidity sensor](https://www.adafruit.com/product/2652)
	- Technical references 
		- https://learn.adafruit.com/adafruit-aht20
		- https://cdn-learn.adafruit.com/assets/assets/000/104/015/original/Sensirion_CO2_Sensors_SCD4x_Datasheet.pdf?1629489682
		- https://github.com/Sensirion/arduino-i2c-scd4x
		- https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library
		- https://emariete.com/en/sensor-co2-sensirion-scd40-scd41-2/
- Battery (monitor)
	- define battery size in Step 2 of config.h
	- batteryRead() looks for LC709203F, then tries to use supported board's voltage monitor GPIO pin
		- if neither is found, battery voltage is not displayed or reported
	- Supported hardware
		- 	[LC709203F battery voltage monitor](https://www.adafruit.com/product/4712)
		- [Adafruit batteries](https://www.adafruit.com/product/2011)
- Screen
	- uncomment #define SCREEN
	- code assumes MagTag
		- if using Featherwing, comment MagTag specific code inside config.h
	- Supported hardware
		- 296x128 e-paper display
			- [Adafruit MagTag (EPD)](https://www.adafruit.com/product/4800)
			- [Adafruit 2.9" E-Ink Featherwing](https://www.adafruit.com/product/4777)
	- Technical References
		- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf

### Supported Internet Services for data logging
- The routines that post data to back end services are generalized as much as practical, though do need to be customized to match the data fieles of interest both within the scope of the project and based on what users want to report and monitor.  Configuration values in config.h help with basic customization, e.g. name of the device, tags to use for Influx data, though in some cases code may need to be modified in the associated post routine.

- MQTT Broker
	- uncomment #define MQTT
	- set appropriate parameters in config.h and secrets.h
	- Technical References
		- https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/
- Influx
- DWEET

### Issues and Feature Requests
- See GitHub Issues for project

### .plan (big ticket items)
- WiFI Manager support
- OTA Firmware update
- GPIO (button) ESP32 wakeup support to have multiple screens of information