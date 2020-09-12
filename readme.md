# project_name
air_quality

### Purpose
Measure and log temperature, humidity, and CO2 levels every 30 minutes

### Contributors

### Software Dependencies
- DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
- Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

### BOM
- 1x: Arduino Feather M4 Express: https://www.adafruit.com/product/3857
- 1x: Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
- 1X: DHT22 temp/humidity sensor: https://www.adafruit.com/product/385
- 1X: SGP30 gas sensor: https://www.adafruit.com/product/3709

### Pinouts
- Particle Ethernet Featherwing connects to the following pins
	- SPI MISO
	- SPI MOSI
	- SPI SCK
	- D3 RESET
	- D4 INTERRUPT
	- D5 CHIP SELECT

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
- SD card
	- https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
- NTP
	- https://github.com/PaulStoffregen/Time/tree/master/examples/TimeNTP
- Sensors 
	- https://learn.adafruit.com/adafruit-sgp30-gas-tvoc-eco2-mox-sensor/arduino-code
- Ethernet
	- https://docs.particle.io/datasheets/accessories/gen3-accessories/
	- https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
	- https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
	- https://www.arduino.cc/en/reference/ethernet
	- https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe

### Issues
- 083120: Need to add baseline readings for the SGP30 (EPROM, FLASH)
- 091120: Set SYNC_INTERVAL to minimum for AIO and use in main CLOUDLOG
- 091120: Crashes again after one loop
	- might be a DHT issue
	- main delay(SYNC_INTERVAL) must be factored out, it could be impacting networking
- 091120: Data not writing to AIO
	- check sample code

### Questions
- 090820: We are generating humidity, heat index, and absolute humidity?

### Learnings
- 090620: Just push your own MAC address if the device doesn't physically display its address, but avoid duplicates across projects when using common code to set it.
- 090620: Arduino Ethernet code can't tranverse a DNS fallback list, so if the primary fails (e.g. Pihole crash) it will stop connecting to outside addresses via DNS lookup

### Feature Requests
- 083120: Upload data to cloud db
- 090120: Add screen display support
- 090620: LED blink encoded error messages for non-DEBUG and while(1) errors
- 090820: Optimize code
- 090820: After adding cloud db support, try backport to Arduino Ethernet board (enough memory?)
- 090820: Switch to Adafruit M0 Datalogger and re-enable SDLOG
- 091120: Use timedisplay routines for log string

### Revisions
- 083120: First version based on merged sample code for sensors, SD. Ethernet code NOT working.
- 090520: Outside code work highlights Ethernet code is likely working, was blocked by failed primary DNS server
- 090820
	- Switched to Particle Ethernet Featherwing and Feather M4 Express
	- Switched to timelib getNtpTime example code
	- [FR] 090120: Conditional compile for network, SD saves, and display
	- [FR] 090120: Switch to ARM SoC for +memory (post conditionals?)
-091120
	- [I] 090820: Code is only running for one loop -> setSyncInterval(15) locked code on subsequent loops, also not needed
	- [I] - 090820: time is not correct, sample code is -> byproduct of setSyncInterval(15) issue
	- [FR] 090820: Display NTP time when received in DEBUG
