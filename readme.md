# Air Quality

### Purpose
Regularly sample and log temperature, humidity, and CO2 levels

### Contributors

### Software Dependencies not in Arduino Library Manager 

### BOM
- 1x: Arduino Feather M0/M4 Express
- 1x: Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
- 1X: DHT22 temp/humidity sensor: https://www.adafruit.com/product/385
- 1X: SGP30 gas sensor: https://www.adafruit.com/product/3709
- 1X: Featherwing OLED (128x32): https://www.adafruit.com/product/2900

### Pinouts
- Particle Ethernet Featherwing
	- SPI MISO
	- SPI MOSI
	- SPI SCK
	- D3 RESET
	- D4 INTERRUPT
	- D10 CHIP SELECT
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
- Featherwing OLED
	- SDA to SDA
	- SCL to SCL

### Information Sources
- SD card
	- https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
- NTP
	- https://github.com/PaulStoffregen/Time/tree/master/examples/TimeNTP
- Sensors 
	- https://learn.adafruit.com/adafruit-sgp30-gas-tvoc-eco2-mox-sensor/arduino-code
	- https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/overview
	- https://www.jaredwolff.com/finding-the-best-tvoc-sensor-ccs811-vs-bme680-vs-sgp30/
- Ethernet
	- https://docs.particle.io/datasheets/accessories/gen3-accessories/
	- https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
	- https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
	- https://www.arduino.cc/en/reference/ethernet
	- https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
- Display
	- https://learn.adafruit.com/adafruit-oled-featherwing/usage
	- https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf
	- https://engineeringnotes.blogspot.com/2013/07/using-ssd1306-textgraphics-display.html

### Learnings
- 090620: Just push your own MAC address if the device doesn't physically display its address, but avoid duplicates across projects when using common code to set it.
- 090620: Arduino Ethernet code can't tranverse a DNS fallback list, so if the primary fails (e.g. Pihole crash) it will stop connecting to outside addresses via DNS lookup

### Issues
- 083120: Need to add baseline readings for the SGP30 (EPROM, FLASH)
- 092020: If time isn't set by NTP via RJ45, DEBUG and SDLOG will have errors

### Feature Requests
- 090620: P3, LED blink encoded error messages for non-DEBUG and while(1) errors
- 091320: P1, Insert easily visible, detectable (-1) data points into data feed when sensors error during read
- 091420: P2, Try and move AIO keys to another group

### Questions
- 090820: We are generating humidity, heat index, and absolute humidity?
- 091420: eCO2 level never changes? (see baseline issue?)

### Revisions
- 083120: First version based on merged sample code for sensors, SD. Ethernet code NOT working.
- 090520: Outside code work highlights Ethernet code is likely working, was blocked by failed primary DNS server
- 090820
	- Switched to Particle Ethernet Featherwing and Feather M4 Express
	- Switched to timelib getNtpTime example code
	- [FR] 090120: Conditional compile for network, SD saves, and display
	- [FR] 090120: Switch to ARM SoC for +memory (post conditionals?)
- 091120
	- [I] 090820: Code is only running for one loop -> setSyncInterval(15) locked code on subsequent loops, also not needed
	- [I] - 090820: time is not correct, sample code is -> byproduct of setSyncInterval(15) issue
	- [FR] 090820: Display NTP time when received in DEBUG
- 091420
	- [I] 091120: Set SYNC_INTERVAL to minimum for AIO and use in main CLOUDLOG -> done
	- [I] 091120: Crashes again after one loop, might be a DHT issue -> DHT moved to pin 11, stopped collision with Ethernet on pin 10 (CS)
	- [I] 091120: main delay(SYNC_INTERVAL) must be factored out, it could be impacting networking -> done
	- [I] 091120: Data not writing to AIO -> side effect of DHT/Ethernet pin conflict
	- [FR] 083120: Upload data to cloud db -> code now working
	- [FR] 090820: After adding cloud db support, try backport to Arduino Ethernet board (enough memory?) -> does not fit, not worth the effort
	- [FR] 090120: Add screen display support
- 092020
	- [FR] 091420: P2, Switch to M0 Proto board
	- [FR] 091120: P1, Use timedisplay routines for log string
	- Added DEBUG and production sample rate definitions
	- [FR] 090820: P1, Optimize code
	- [I] 092020: SDLOG doesn't actually write values (code was dropped in previous revision) -> this has been true since initial Github checkin; fixed