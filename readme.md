### Purpose
Air Quality (aka AQ) samples and logs device temperature, humidity, and if available, co2 levels

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

### Configuring targets
- Step 1: set conditional compile flags in config.h
Set parameters in secrets.h (see config.h for list of required parameters)
- Set parameters in config.h
- Switch between AHT2x/BME280 and SCD40 in multiple locations within air_quality.ino

### working BOM [hardware configuration]
- MCU
	- ESP32
- Ethernet
	- uncomment #define RJ45, comment #define WIFI in config.h
	- Supported hardware
		- Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
		- Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
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
		- AHT20 temp/humidity sensor: https://www.adafruit.com/product/4566, https://www.adafruit.com/product/5183
		- SCD40 True CO2, Temperature and Humidity Sensor: https://www.adafruit.com/product/5187
		- BME280 temp/humidity sensor: https://www.adafruit.com/product/2652
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
	- 	LC709203F battery voltage monitor: https://www.adafruit.com/product/4712
		- Adafruit batteries: https://www.adafruit.com/product/2011
- Screen
	- uncomment #define SCREEN
	- code assumes MagTag
		- if using Featherwing, comment MagTag specific code inside config.h
	- Supported hardware
		- 296x128 e-paper display
			- Adafruit MagTag (EPD): https://www.adafruit.com/product/4800
			- Adafruit 2.9" E-Ink Featherwing: https://www.adafruit.com/product/4777
		- LCD and OLED screens
			- see code history before 02/2022
			- any adafruit gfx compatible display will work, but AQ doesn't currently support screens requiring continuous power
	- Technical References
		- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf

### Supported Internet Services for data logging
- MQTT Broker
	- uncomment #define MQTT
	- set appropriate parameters in config.h and secrets.h
	- Technical References
		- https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/
- Influx
- DWEET

### External Software Dependencies
- add library for appropriate sensor from known, working BOM
	- Sensirion I2C SCD4x library or Adafruit Unified Sensor + appropriate hardware (AHT2x or BME280) library
	- Adafruit LC709203F library (#define BATTERY)
	- Adafruit EPD library (MagTag)
	- Arduino_json (parsing OWM data)
- include all dependencies to these libraries
- Time
	-
	- Technical References
		- 

### .plan (big ticket items)
- WiFI Manager support
- OTA Firmware update
- GPIO (button) ESP32 wakeup support to have multiple screens of information

### Information Sources
- Most information is kept in the "supporting material" folder, which is not synched to GitHub. Maybe I should do that? ping me if you want more info on CO2 measurements, etc.

### Issues and Feature Requests
- See GitHub Issues for project