### Purpose
Air Quality samples and logs temperature, humidity, and if sensor available, co2 levels

### Configuring targets
- Set parameters in secrets.h (see config.h for list of required parameters)
- Set parameters in config.h
- Switch between AHT2x/BME280 and SCD40 in multiple locations within air_quality.ino

### External Software Dependencies
- add library for appropriate sensor from known, working BOM
	- Sensirion I2C SCD4x library or Adafruit Unified Sensor + appropriate hardware (AHT2x or BME280) library
	- Adafruit LC709203F library (#define BATTERY)
	- Adafruit EPD library (MagTag)
	- Arduino_json (parsing OWM data)
- include all dependencies to these libraries

### known, working BOM
- MCU
	- ESP32, ESP8266
	- ARM(m0,m4)
		- deepSleep() not implemented
- Ethernet
	- Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
	- Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
- WiFi
	- esp32s2 boards
- environment sensor
	- AHT20 temp/humidity sensor: https://www.adafruit.com/product/4566, https://www.adafruit.com/product/5183
	- SCD40 True CO2, Temperature and Humidity Sensor: https://www.adafruit.com/product/5187
	- BME280 temp/humidity sensor: https://www.adafruit.com/product/2652
- battery monitor
	- LC709203F battery voltage monitor: https://www.adafruit.com/product/4712
- screen
	- Adafruit MagTag (EPD): https://www.adafruit.com/product/4800
	- LCD and OLED screens -> see code history before 02/2022
- battery
	- Adafruit battery: https://www.adafruit.com/product/2011

### Information Sources
- Most information is kept in the "supporting material" folder, which is not synched to GitHub. Maybe I should do that? ping author if you want more info on CO2 measurements, etc.

- SD card
	- https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
- NTP
	- https://github.com/PaulStoffregen/Time/tree/master/examples/TimeNTP
- Sensors 
	- https://learn.adafruit.com/adafruit-aht20
	- https://cdn-learn.adafruit.com/assets/assets/000/104/015/original/Sensirion_CO2_Sensors_SCD4x_Datasheet.pdf?1629489682
	- https://github.com/Sensirion/arduino-i2c-scd4x
	- https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library
	- https://emariete.com/en/sensor-co2-sensirion-scd40-scd41-2/
- Ethernet
	- https://docs.particle.io/datasheets/accessories/gen3-accessories/
	- https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
	- https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
	- https://www.arduino.cc/en/reference/ethernet
	- https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
- Display
	- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf
- MQTT services
	- https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/

### Issues
- See GitHub Issues for project

### Feature Requests
- See GitHub Issues for project

### Questions