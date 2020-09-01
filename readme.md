# project_name
air_quality

### Purpose
Measure and log temperature, humidity, and CO2 levels every 30 minutes

### Contributors

### Software Dependencies
- DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
- Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

### BOM
1x: Arduino Ethernet board: https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
1X: DHT22 temp/humidity sensor: https://www.adafruit.com/product/385
1X: SGP30 gas sensor: https://www.adafruit.com/product/3709

### Pinouts
- Arduino Ethernet board reserves Pin 4 for SD and 10,11,12,13 for Ethernet
- DHT sensor
	- Connect pin 1 (on the left) of DHT22 to +5V or 3.3v depending on board
	- Connect pin 2 of the sensor to whatever your DHTPIN is
	- Connect pin 4 (on the right) of the sensor to GROUND
	- Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
- SGP30 sensor
	- VIN to 3.3v, 5V
	- GND to ground
	- SDA to SDA
	- SCL to SCL

### Information Sources
- https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
- https://www.geekstips.com/arduino-time-sync-ntp-server-esp8266-udp/
- https://learn.adafruit.com/adafruit-sgp30-gas-tvoc-eco2-mox-sensor/arduino-code

### Issues
- 083120: Need to add baseline readings for the SGP30 (EPROM, FLASH)
- 083120: Arduino Ethernet doesn't have enough memory for code + libraries, might even be an SRAM issue b4 that

### Questions
- MMDDYY:

### Learnings
- MMDDYY:

### Feature Requests
- 083120: Upload data to internet db

### Revisions
- 083120: First version based on merged sample code for sensors, SD. Ethernet code NOT working.