# Air Quality

### Purpose
Regularly sample and log temperature, humidity, and CO2 levels

### Contributors

### Software Dependencies not in Arduino Library Manager 

### known, working BOM parts
- 1x: Arduino Feather M0 Basic Proto: https://www.adafruit.com/product/2772
- or
- 1x: Feather Huzzah 8266 (WiFi): https://www.adafruit.com/product/2821
- 1x: [optional] Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
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

### Error codes 
- ERR 01: Can not connect to MQTT broker after 10 tries of try# x 10 seconds intervals. Device must be restarted to proceed unless SDLOG enabled.
- ERR 02: Can not connect to SGP30 CO2 sensor at startup. No CO2 readings will be logged.
- ERR 03: Can not connect to WiFi after 10 tries of try# x 10 seconds intervals. Device must be restarted to proceed unless SDLOG enabled.
- ERR 04: MQTT publish failed.

### to change hardware build target
- change DHT pin
- change #defines

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
- 111420: There is no way to set Ethernet hostname in official library. One could edit dhcp.cpp and dhcp.h to change the six character host name, but...

### Issues
- [P2]083120: Need to add baseline readings for the SGP30 (EPROM, FLASH), values are likely incorrect
- [P3]092020: If time isn't set by NTP, DEBUG and SDLOG will have errors
- [P2]102420: Move local MQTT server to DNS named entry instead of IP address so DNS can resolve it if IP address changes
- [P3]111020: How do we better handle Daylight vs. Standard time?
- [P3]111120: Screen is on all the time, which could cause OLED burn in, and in dark environments, is very bright
- [P2]111120: Disable board lights on Feather Huzzah
- [P2]111120: Test what happens if SDLOG enabled but MQTT Connect fails
- [P2]111120: Review SDLOG while (1)
- [P2]111120: Review Ethernet while (1)
- [P2]111420: Review NTP wait until data
- [P2]111120: Test that when SGP30 is not detected, -1 entries will be logged properly
- [P1]111420: MQTT publish (to Adafruit IO?) requires NTP to be defined?! No idea why.

### Feature Requests
- [P3]100720: MQTT QoS 1
- [P3]110920: publish to secondary MQTT broker
- [P3]111020: publish to multiple MQTT brokers
- [P2]111120: ThinkSpeak investigation
- [P2]111120: BME680 integration https://www.adafruit.com/product/3660

### Questions
- 090820: We are generating humidity, heat index, and absolute humidity?
- 091420: eCO2 level never changes? (see baseline issue?)
- 100120: Can I just subscribe to the higher level topic in connectToBroker() to get all the subs
- 100220: Every five minutes we are generating a "Transmitting NTP request"?

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
	- [FR] 091420: P2; Switch to M0 Proto board
	- [FR] 091120: P1; Use timedisplay routines for log string
	- Added DEBUG and production sample rate definitions
	- [FR] 090820: P1; Optimize code
	- [I] 092020: SDLOG doesn't actually write values (code was dropped in previous revision) -> this has been true since initial Github checkin; fixed
- 100120
	- partial refactoring for WIFI
	- added initial MQTT support
	- updated credential items in secrets.h
	- moved network feed locations out of secrets.h
- 111020
	- added conditional for CO2 sensor read
	- [FR] 100120: P2; re-factor CLOUDLOG?, RJ45, and [i]092020 to add WiFi support -> WiFi code changes inherited from cub2bed_mqtt code base, though not addressing [i]092020 yet
	- [FR] 091420: P3; Try and move Adafruit IO feeds to another group -> implemented for master_bedroom as test
	- [FR] 110920: P2; Add LiPo battery so if power goes out we have redundant power supply for some period -> added to both deployed rooms
	- [I] 100220: P1; MQTT code previously has only updating every 10 minutes and then would stop. Code changes implemented to MQTT code, check for reliability -> resolved with tested MQTT code path from cub2bed_mqtt code base
	- [FR] 091320: P1; Insert detectable (-1) data points into data feed when sensors error during read or create out of parameter values (see code in loop()) -> Added support for read failures, not out of bounds conditions
	- [I] 100220: P3; leading zero problem on day in timestring() -> Added a leading zero for day()<10
-111120
	- [FR] 111020: P2; combine MQTT publish code blocks for AdafruitIO MQTT and generic MQTT, append username to the secrets.h MQTT_PUB_TOPIC[x] -> completed
	- [I] 102420: P1; Need an error indicator and better handling for non-DEBUG, while(1) errors, MQTT connection errors -> MQTT connection and SGP30 sensor init while(1) have better handling
	- [FR] 111020; P3: is #if defined(SDLOG) || defined(SCREEN) needed, because screen has its own output format? -> bug, fixed as #if defined(SDLOG) || defined(DEBUG)
-111420
	- [FR] 111420: P1; Add WiFi and Ethernet hostnames -> Added for WiFi but Ethernet not supported in library.
	- [FR] 100120: P1; refactor NTP time code into monolithic block, only used by SDLOG and DEBUG -> conditional #define NTPTIME
	- [I] 111020: P1; Display something on screen while waiting for first sensor read. If the device can't get to the MQTT broker on its initial boot, as an example, the screen will never be initialized. -> display text added
	- [I] 111120: P1; Review WiFi wait until connect that leaves device hung with no visible indicators -> WiFi connect now has 10 attempts then error handling and messaging