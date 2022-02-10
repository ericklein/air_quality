### Purpose
Air Quality samples and logs temperature, humidity, and if sensor available, co2 levels

### Configuring targets
- Set parameters in secrets.h (see config.h for list of required parameters)
- Set parameters in config.h
- Switch screen types in SCREEN routines in air_quality.ino
- Switch sensor types if needed in/near CO2_SENSOR in air_quality.ino

### Software Dependencies
- add library for appropriate sensor from known, working BOM
	- Sensirion I2C SCD4x library (#define CO2_SENSOR)
	- Adafruit LC709203F library (#define BATTERY)
	- Adafruit EPD library (MagTag)
- include all dependencies to these libraries

### known, working BOM
- MCU
	- ESP32S2. ESP8266
	- ARM(m0,m4)
		- deepSleep() not implemented
- Ethernet
	- Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
	- Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
- WiFi
	- esp32s2 boards
- environment sensor
	- AHT20 temp/humidity sensor: https://www.adafruit.com/product/4566, https://www.adafruit.com/product/5183
	- SiH7021 temp/humidity sensor: https://www.adafruit.com/product/3251
	- SCD40 True CO2, Temperature and Humidity Sensor: https://www.adafruit.com/product/5187
	- DHT11,21 -> see code history
- battery monitor
	- LC709203F battery voltage monitor: https://www.adafruit.com/product/4712
- screen
	- Adafruit MagTag (EPD): https://www.adafruit.com/product/4800
	- LCD and OLED screens -> see code history before 02/2022
- battery
	- Adafruit 2000mA battery: https://www.adafruit.com/product/2011

### Information Sources
- SD card
	- https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
- NTP
	- https://github.com/PaulStoffregen/Time/tree/master/examples/TimeNTP
- Sensors 
	- https://learn.adafruit.com/adafruit-aht20
- Ethernet
	- https://docs.particle.io/datasheets/accessories/gen3-accessories/
	- https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
	- https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
	- https://www.arduino.cc/en/reference/ethernet
	- https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
- Display
	- https://learn.adafruit.com/adafruit-oled-featherwing/overview
	- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf
	- https://engineeringnotes.blogspot.com/2013/07/using-ssd1306-textgraphics-display.html
	- https://github.com/adafruit/Adafruit_SSD1306/issues/106 (turning OLED on and off)
- MQTT services
	- https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/

### Issues
- See GitHub Issues for project

### Feature Requests
- See GitHub Issues for project

### Questions
- [Q]090721: I've failed at compressing zuluDateTimeString() twice, what is the issue relative to string buildout?
- [Q]100621: how did LadyAda calculate battery capacity, as hex values are not on a linear formula though battery capacity is?
- [Q]012622: does SparkFun or Adafruit have a sensor measurement API that is consistent across temp/humidity sensors? [readEnvironment]

### Revisions
- 020922 - First refactored version