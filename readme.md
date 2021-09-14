 Air Quality

### Purpose
Regularly sample and log temperature, humidity

### Contributors

### Software Dependencies not in Arduino Library Manager 

### known, working BOM parts
- MCU
	- tested on ESP32S2, ESP8266, ARM(m0,m4)
	- networking
	- Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
	- Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
- sensors
	- AHT20 temp/humidity sensor: https://www.adafruit.com/product/4566, https://www.adafruit.com/product/5183
	- DHT11,21 also supported, see code history
	- SGP30 VO2 partially supported in separate code branch
- screens
	- LCD screen supported, see code history
	- Featherwing OLED (SSD1306, 128x32): https://www.adafruit.com/product/2900
	- Featherwing OLED (SH110X, 128x64): https://www.adafruit.com/product/4650
	- Adafruit Funhouse (ST7789, 240x240): https://www.adafruit.com/product/4985

### Pinouts
- Particle Ethernet Featherwing
	- SPI MISO
	- SPI MOSI
	- SPI SCK
	- D3 RESET
	- D4 INTERRUPT
	- D10 CHIP SELECT
- Featherwing OLED, AHT sensor
	- SDA to SDA
	- SCL to SCL

### Error codes
- FATAL
	- Always throws a DEBUG message and blinks built-in LED at 1 second intervals
	- ERR 01: Can not connect to temp/humidity sensor
	- ERR 02: Can not connect to Ethernet. Device must be restarted to proceed unless SDLOG enabled.
	- ERR 03: Can not connect to WiFi. Device must be restarted to proceed unless SDLOG enabled.
- CAUTION
	- ERR 10: Can not connect to MQTT broker

### to change hardware build target
- change #defines in air_quality.ino
- change appropriates values in secrets.h

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


### Learnings
- 090620: Just push your own MAC address if the device doesn't physically display its address, but avoid duplicates across projects when using common code to set it.
- 090620: Arduino Ethernet code can't tranverse a DNS fallback list, so if the primary fails (e.g. Pihole crash) it will stop connecting to outside addresses via DNS lookup
- 111420: There is no way to set Ethernet hostname in official library. One could edit dhcp.cpp and dhcp.h to change the six character host name, but...

### Issues
- [I][P3]112820: time; NTP is dependent to WiFi or Ethernet due to IPAddress
- [I][P2]102420: mqtt; Move local MQTT server to DNS named entry instead of IP address so DNS can resolve it if IP address changes
- [I][P1]111120: screen; Screen is on all the time, which could cause OLED burn in, and in dark environments, is very bright
- [I][P2]111420: time; Review NTP while until data
	- See Q about MQTT local server time stamping inbound data?
	- add code to let NTP fail through rather than while waiting
- [I][P1]091321: time; "No NTP response" does not reattempt until success
	- in zuluDateTimeString switch #ifdef NTP for check timeStatus() for timeNotSet
- [I][P2]111420: mqtt; MQTT publish (to Adafruit IO?) requires NTP to be defined?! No idea why.
- [I][P1]112820: enclosure; Temperature data is off by a few degrees F when inserted into case?
- [I][P2]112820: screen; pin 2 conflict with XXXX? on SH110x, not sure about SSD1306
- [I][P2]090921: sensor; values coming from standalone AHT20 and Funhouse AHT20 are very different. Calibration issue? See [FR] on this as well.
	- 091321 SiH7021 is close to the standalone AHT20 values
- [I][P1]091021: wifi; If WiFi comes down for an extended period, functionality does not recover
- [I][P2]091321: log; don't think MagTag BSP has LED_BUILTIN defined. No blinking LED on FATAL errors
- [I][P1]091321: screen; is the deepsleep function for EPD causing the screen to grey out? look at powerUp(), which I'm not using, and powerDown()
- [I][P3]091321: wifi; validate host name is being set via network admin tool

### Feature Requests
- [FR][P3]100720: mqtt; MQTT QoS 1
- [FR][P3]111020: mqtt; publish to multiple MQTT brokers
- [FR][P1]112020: screen; Heartbeat indicator on-screen
- [FR][P2]112920: screen; Rotating time display
- [FR][P2]112920: time; Get time from MQTT broker
- [FR][P3]120220: screen; Added error checking on string length to screenMessage()
- [FR][P2]012421: log; Async blinking of built-in LED for non-FATAL errors (e.g. MQTT publish)
- [FR][P3]120620: log; Air Quality messages to MQTT broker
	- add error field to adafruitIO->airquality
	- send error messages to MQTT for wait states
		- timestamp->machine->error message
- [FR][P2]090121: power; Low battery messaging (to MQTT)
- [FR][P2]090821: wifi; instead of while(1) if unable to connect to WiFi, it would be better to reset
- [FR][P2]090921: sensor; check calibration before reading and calibrate if needed
- [FR][P3]090921: wifi; more diagnostic information at connect in log
- [FR][P2]090921: mqtt; log MQTT server errors https://io.adafruit.com/blog/example/2016/07/06/mqtt-error-reporting/
- [FR][P3]091321: screen; convert pixel coordinates in screenUIBorders to offsets of display.width, display.height
- [FR][P3]091321: mqtt; inject lat/long into data, other extended fields for adafruit io?
- [FR][P3]091321: screen; cycle to local weather forcast

### Questions
- [Q]100120: mqtt; Can I just subscribe to the higher level topic in connectToBroker() to get all the subs
- [Q]120220: screen; Why do I need wire and spi for OLED displays?
- [Q]082921: time; when do I need to timestamp data bound for MQTT; adafruit.io time stamps for me, does a local server?
- [Q]090721: I've failed at compressing zuluDateTimeString() twice, what is the issue relative to string buildout?
- [Q]091321: should I push data to MQTT as JSON?

### Revisions
- 083120: First version based on merged sample code for sensors, SD. Ethernet code NOT working.
- 090520: Outside code work highlights Ethernet code is likely working, was blocked by failed primary DNS server
- 090820
	- Switched to Particle Ethernet Featherwing and Feather M4 Express
	- Switched to timelib getNtpTime example code
	- [FR]090120: Conditional compile for network, SD saves, and display
	- [FR]090120: Switch to ARM SoC for +memory (post conditionals?)
- 091120
	- [I]090820: Code is only running for one loop -> setSyncInterval(15) locked code on subsequent loops, also not needed
	- [I]090820: time is not correct, sample code is -> byproduct of setSyncInterval(15) issue
	- [FR]090820: Display NTP time when received in DEBUG
- 091420
	- [I]091120: Set SYNC_INTERVAL to minimum for AIO and use in main CLOUDLOG -> done
	- [I]091120: Crashes again after one loop, might be a DHT issue -> DHT moved to pin 11, stopped collision with Ethernet on pin 10 (CS)
	- [I]091120: main delay(SYNC_INTERVAL) must be factored out, it could be impacting networking -> done
	- [I]091120: Data not writing to AIO -> side effect of DHT/Ethernet pin conflict
	- [FR]083120: Upload data to cloud db -> code now working
	- [FR]090820: After adding cloud db support, try backport to Arduino Ethernet board (enough memory?) -> does not fit, not worth the effort
	- [FR]090120: Add screen display support
- 092020
	- [FR][P2]091420: Switch to M0 Proto board
	- [FR][P1]091120: Use timedisplay routines for log string
	- Added DEBUG and production sample rate definitions
	- [FR][P1]090820: Optimize code
	- [I]092020: SDLOG doesn't actually write values (code was dropped in previous revision) -> this has been true since initial Github checkin; fixed
- 100120
	- partial refactoring for WIFI
	- added initial MQTT support
	- updated credential items in secrets.h
	- moved network feed locations out of secrets.h
- 111020
	- added conditional for CO2 sensor read
	- [FR][P2]100120: re-factor CLOUDLOG?, RJ45, and [i]092020 to add WiFi support -> WiFi code changes inherited from cub2bed_mqtt code base, though not addressing [i]092020 yet
	- [FR][P3]091420: Try and move Adafruit IO feeds to another group -> implemented for master_bedroom as test
	- [FR][P2]110920: Add LiPo battery so if power goes out we have redundant power supply for some period -> added to both deployed rooms
	- [I][P1]100220: MQTT code previously has only updating every 10 minutes and then would stop. Code changes implemented to MQTT code, check for reliability -> resolved with tested MQTT code path from cub2bed_mqtt code base
	- [FR][P1]091320: Insert detectable (-1) data points into data feed when sensors error during read or create out of parameter values (see code in loop()) -> Added support for read failures, not out of bounds conditions
	- [I][P3]100220: leading zero problem on day in timestring() -> Added a leading zero for day()<10
-111120
	- [FR][P2]111020: combine MQTT publish code blocks for AdafruitIO MQTT and generic MQTT, append username to the secrets.h MQTT_PUB_TOPIC[x] -> completed
	- [I][P1]102420: Need an error indicator and better handling for non-DEBUG, while(1) errors, MQTT connection errors -> MQTT connection and SGP30 sensor init while(1) have better handling
	- [FR][P3]111020: is #if defined(SDLOG) || defined(SCREEN) needed, because screen has its own output format? -> bug, fixed as #if defined(SDLOG) || defined(DEBUG)
-111420
	- [FR][P1]111420: Add WiFi and Ethernet hostnames -> Added for WiFi but Ethernet not supported in library.
	- [FR][P1]100120: refactor NTP time code into monolithic block, only used by SDLOG and DEBUG -> conditional #define NTPTIME
	- [I][P1] 111020: Display something on screen while waiting for first sensor read. If the device can't get to the MQTT broker on its initial boot, as an example, the screen will never be initialized. -> display text added
	- [I][P1] 111120: Review WiFi wait until connect that leaves device hung with no visible indicators -> WiFi connect now has 10 attempts then error handling and messaging
-112820
	- Added rudimentary support for SH110x OLED screen
	- Extracted TimeString from NTP conditional compile
	- Modified TimeString to return a value if NTP not defined
	- Tested SCREEN + DEBUG code path
	- Added untested support for TARGET_ANNE_OFFICE
- 120120
	- Tested RJ45 + MQTT code path
- 123120
	- Standardized while (1) error handling for WiFi, RJ45, and MQTT_Connect
	- Partial support for proximity based activation of the screen. Should have been on a branch...
- 012421
	- [FR][P2]123120: Add error blink codes to onboard LED for while (1) -> Adding BUILT_IN_LED blinking matching the fatal error code, aligning with new code in air_quality
	- [I][P2]111120: Disable board lights on Feather Huzzah -> Built-in LED is supressed at startup then used only for fatal error messages
	- [I][P3]112820: if DHT pin not assigned properly, code crashes -> Just tested where DHT was pinned for Huzzah and compiled for M0. Code properly reported -1 for sensor read.
- 082521
	- changing room into a parameter of uploaded data
	- [I][P2]112920: Anonymize TARGET_XXX defines
	- proximity code removed as it is now in a branch. This branch has all proximity code removed, master had code in main() still
- 083021
	- removed while(1) for mqttconnect()
	- #DEBUG log formatting fixes
	- Time fixes
		- [I][P3]082521: quasi clock generates two messages every 10 seconds instead of one -> dependent on code and processor, # of messages dependent on times through loop within the 1ms
		- [Q]100220: time; Every five minutes we are generating a "Transmitting NTP request"? -> Expected behavior, any time a timelib function is used, timelib is checking and potentially synching to time provider. In this case, a time string was being generated associated with each mqtt log entry, causing the sync
		- [I][P3]111020: time; How do we better handle Daylight vs. Standard time? -> Switched to UTC time for logging, switched timeString to properly formatted zuluDateTimeString
		- [I][P3]092020: time; If time isn't set by NTP, DEBUG and SDLOG have errors -> zuluDateTimeString inserts "time not set" instead of UTC time if NTP not defined
	- [I][P2]112820: sdlog; data logged to SDLOG is not uniquely identified -> room and UTC time attached to readings
	- [Q]090820: We are generating humidity, heat index, and absolute humidity? -> dropping heat index support, absolute humidity required to calibrate eCO2 reading
- 090121
	- moving CO2 measurement to private branch, as it doesn't work
- 090321
	- switching to AHT20 (temp/humidity) for i2c connectivity and future proofing
- 090721
	- #define SCREEN work (untested)
		- integrate Adafruit Funhouse screen support
		- added screenMessage wrapper for Adafruit GFX xxx.print related to #define SCREEN
		- removed LCD code from #define SCREEN, not expecting to use again
		- general cleanup
	- implemented debugMessage wrapper for previous #define DEBUG serial messages
	- general bug fixes
- 090921
	- bug fixes in #define WiFi, MQTT code
	- log improvements in #define WiFi, MQTT code
	- moved #define SDLOG code to branch as this code is not being used
- 091021
	- e-ink support for the Adafruit Magtag
	- support for Adafruit SiH7021: https://www.adafruit.com/product/3251, temporary until I get more AHTx0 parts
- 091321
	- [FR][P2]090121: power; Deep sleep between sensor reads to lower battery consumption -> Sleep support for ESP32; moving from loop() to singular runs of setup()
	- Moving temp/humidity display to separate function
	- UI elements function
	- screenMessage updated to call UI element function