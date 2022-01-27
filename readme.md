### Purpose
Air Quality regularly samples and logs temperature, humidity, and if sensor available, co2 levels

### Configuring targets
- Set parameters in secrets.h
- Set appropriate conditional compile flags in air_quality.ino
- Switch screen types in SCREEN routines in air_quality.ino
- Switch sensor types if needed in/near CO2_SENSOR in air_quality.ino
- Select battery pack size in BATTERY setup() in air_quality.ino

### known, working BOM parts
- MCU
	- tested on ESP32S2, ESP8266, ARM(m0,m4)
- Ethernet
	- Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
	- Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
- WiFi
	- esp32s2 boards
- sensors
	- AHT20 temp/humidity sensor: https://www.adafruit.com/product/4566, https://www.adafruit.com/product/5183
	- SiH7021 temp/humidity sensor: https://www.adafruit.com/product/3251
	- DHT11,21 also supported, see code history
	- LC709203F battery voltage monitor: https://www.adafruit.com/product/4712
	- SCD40 True CO2, Temperature and Humidity Sensor: https://www.adafruit.com/product/5187
- screen
	- LCD screen supported, see code history
	- Featherwing OLED (SSD1306, 128x32): https://www.adafruit.com/product/2900
	- Featherwing OLED (SH110X, 128x64): https://www.adafruit.com/product/4650
	- Adafruit Funhouse (ST7789, 240x240): https://www.adafruit.com/product/4985
	- Adafruit MagTag (EPD): https://www.adafruit.com/product/4800
- battery
	= Adafruit 2000mA battery: https://www.adafruit.com/product/2011

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
- See GitHub Issues for project

### Feature Requests
- See GitHub Issues for project

### Questions
- [Q]090721: I've failed at compressing zuluDateTimeString() twice, what is the issue relative to string buildout?
- [Q]100621: how did LadyAda calculate battery capacity, as hex values are not on a linear formula though battery capacity is?
- [Q]012622: does SparkFun or Adafruit have a sensor measurement API that is consistent across temp/humidity sensors? [readEnvironment]

### Revisions
- 012622
	- splitting secrets.h
		- private information in secrets.h
		- config information moved to config.h
	- moved battery size parameter to config.h
	- moved log_interval and conversions to config.h
- 010622
	- initial version of AQM (air quality) support via OpenWeatherMap
- 010222
	- Modifications to the deepSleep() routine, trying to fix the rapid battery depletion issue that has emerged
- 122721
	- initial version of outside weather support via OpenWeatherMap
- 120821
	- [FR][P2]051021: screen; current time -> closed to conserve battery as primary screen type is eInk on a ONETIME loop
	- Moving FR and Issues to Github from readme.md
- 120721
	- [FR][P1]090121: mqtt; Low battery messaging to MQTT -> added
	- Changed MQTT publishing to per room feeds
	- [I][P2]091521: power; why is stemmaQT board getting power (LED light is on) in deepsleep()? -> closed because I'm cutting the trace bridge for these LEDs
- 111121
	- [FR][P1]093021: sensor; indoor C02 levels -> added support for SCD40 which measures temp, humidity, and CO2 level
	- refactor code to support #define ONE_TIME, which allows the code to run one time then sleep (e.g. ESP hardware), or loop continuously (e.g. RJ45)
- 110521
	- [FR][P1]090121: power; Low battery messaging to screen ->added
- 100421
	- [I][P3]091321: wifi; validate host name is being set via network admin tool -> validated
	- [I][P2]100421: screen; UI indication of clientname -> added
	- [I][P1]093021: screen; error string is wrapping into time display text -> resolved for MQTT error messages
	- [I][P1]093021: screen; error string is not being checked to fit on screen -> manually reviewing
	- Simplified error reporting for MQTT publish issues, as I can look at the data to see the actual problem or error in the server processing code
	- Reviewing UI changes to add outside temp and humidity plus CO2 display
	- [FR][P3]090921: wifi; more diagnostic information at connect in log -> now reporting RSSI value
	- [I][P2]091321: log; don't think MagTag BSP has LED_BUILTIN defined. No blinking LED on FATAL errors -> it works, red light on back
- 091621
	- [I][P1]091521: screen; can't call screenMessage after screenValues because the latter will overwrite the screen -> compressed all screen routines into a single screenUpdate function
	- [FR][P1]112920: screen; time display -> injected as messageText when displaying temperature and humidity
	- [Q]100120: mqtt; Can I just publish to the higher level topic in connectToBroker() to get all the subs -> no
	- [I][P2]091021: wifi; If WiFi comes down for an extended period, functionality does not recover in non-deepsleep code branches -> switched from stopApp() to deepSleep() for WiFi failures at initialization
	- [FR][P2]090821: wifi; instead of while(1) if unable to connect to WiFi, it would be better to reset -> switched from stopApp() to deepSleep() for WiFi failures at initialization
	- [I][P1]091321: mqtt; semi-consistently seeing issue where, over WiFI, temp is logging to mqtt properly but room and sometimes humidity is not -> solved by implementing MQTT QoS 1
	- [FR][P3]100720: mqtt; implement MQTT QoS 1 -> done
- 091521
	- validated code paths for only DEBUG and only DEBUG and SCREEN
	- [FR][P3]091321: screen; convert pixel coordinates in screen draws to offsets of display.width, display.height -> done
	- [I][P1]091321: mqtt; should I be using a mqtt.disconnect() during the sleep process? -> added
	- [I][P1]091321: screen; is the deepsleep function for EPD causing the screen to grey out? -> significant debugging, fixed by changing EPD_BUSY from -1 (default in all Adafruit code) to 5 (from Magtag pinout)
- 091321
	- [FR][P2]090121: power; Deep sleep between sensor reads to lower battery consumption -> Sleep support for ESP32; moving from loop() to singular runs of setup()
	- Moving temp/humidity display to separate function
	- UI elements function
	- screenMessage updated to call UI element function
- 091021
	- e-ink support for the Adafruit Magtag
	- support for Adafruit SiH7021: https://www.adafruit.com/product/3251, temporary until I get more AHTx0 parts
- 090921
	- bug fixes in #define WiFi, MQTT code
	- log improvements in #define WiFi, MQTT code
	- moved #define SDLOG code to branch as this code is not being used
- 090721
	- #define SCREEN work (untested)
		- integrate Adafruit Funhouse screen support
		- added screenMessage wrapper for Adafruit GFX xxx.print related to #define SCREEN
		- removed LCD code from #define SCREEN, not expecting to use again
		- general cleanup
	- implemented debugMessage wrapper for previous #define DEBUG serial messages
	- general bug fixes
- 090321
	- switching to AHT20 (temp/humidity) for i2c connectivity and future proofing
- 090121
	- moving CO2 measurement to private branch, as it doesn't work
- 083021
	- removed while(1) for mqttconnect()
	- #DEBUG log formatting fixes
	- Time fixes
		- [I][P3]082521: quasi clock generates two messages every 10 seconds instead of one -> dependent on code and processor, # of messages dependent on times through loop within the 1ms
		- [Q]100220: time; Every five minutes we are generating a "Transmitting NTP request"? -> Expected behavior, any time a timelib function is used, timelib is checking and potentially synching to time provider. In this case, a time string was being generated associated with each mqtt log entry, causing the sync. Also, MQTT reconnect is set for 5 minutes, and MQTT reconnect (see bug) is driving NTP request
		- [I][P3]111020: time; How do we better handle Daylight vs. Standard time? -> Switched to UTC time for logging, switched timeString to properly formatted zuluDateTimeString
		- [I][P3]092020: time; If time isn't set by NTP, DEBUG and SDLOG have errors -> zuluDateTimeString inserts "time not set" instead of UTC time if NTP not defined
	- [I][P2]112820: sdlog; data logged to SDLOG is not uniquely identified -> room and UTC time attached to readings
	- [Q]090820: We are generating humidity, heat index, and absolute humidity? -> dropping heat index support, absolute humidity required to calibrate eCO2 reading
- 082521
	- changing room into a parameter of uploaded data
	- [I][P2]112920: Anonymize TARGET_XXX defines
	- proximity code removed as it is now in a branch. This branch has all proximity code removed, master had code in main() still
- 012421
	- [FR][P2]123120: Add error blink codes to onboard LED for while (1) -> Adding BUILT_IN_LED blinking matching the fatal error code, aligning with new code in air_quality
	- [I][P2]111120: Disable board lights on Feather Huzzah -> Built-in LED is supressed at startup then used only for fatal error messages
	- [I][P3]112820: if DHT pin not assigned properly, code crashes -> Just tested where DHT was pinned for Huzzah and compiled for M0. Code properly reported -1 for sensor read.
- 123120
	- Standardized while (1) error handling for WiFi, RJ45, and MQTT_Connect
	- Partial support for proximity based activation of the screen. Should have been on a branch...
- 120120
	- Tested RJ45 + MQTT code path
- 112820
	- Added rudimentary support for SH110x OLED screen
	- Extracted TimeString from NTP conditional compile
	- Modified TimeString to return a value if NTP not defined
	- Tested SCREEN + DEBUG code path
	- Added untested support for TARGET_ANNE_OFFICE
- 111420
	- [FR][P1]111420: Add WiFi and Ethernet hostnames -> Added for WiFi but Ethernet not supported in library.
	- [FR][P1]100120: refactor NTP time code into monolithic block, only used by SDLOG and DEBUG -> conditional #define NTPTIME
	- [I][P1] 111020: Display something on screen while waiting for first sensor read. If the device can't get to the MQTT broker on its initial boot, as an example, the screen will never be initialized. -> display text added
	- [I][P1] 111120: Review WiFi wait until connect that leaves device hung with no visible indicators -> WiFi connect now has 10 attempts then error handling and messaging
- 111020
	- added conditional for CO2 sensor read
	- [FR][P2]100120: re-factor CLOUDLOG?, RJ45, and [i]092020 to add WiFi support -> WiFi code changes inherited from cub2bed_mqtt code base, though not addressing [i]092020 yet
	- [FR][P3]091420: Try and move Adafruit IO feeds to another group -> implemented for master_bedroom as test
	- [FR][P2]110920: Add LiPo battery so if power goes out we have redundant power supply for some period -> added to both deployed rooms
	- [I][P1]100220: MQTT code previously has only updating every 10 minutes and then would stop. Code changes implemented to MQTT code, check for reliability -> resolved with tested MQTT code path from cub2bed_mqtt code base
	- [FR][P1]091320: Insert detectable (-1) data points into data feed when sensors error during read or create out of parameter values (see code in loop()) -> Added support for read failures, not out of bounds conditions
	- [I][P3]100220: leading zero problem on day in timestring() -> Added a leading zero for day()<10
- 100120
	- partial refactoring for WIFI
	- added initial MQTT support
	- updated credential items in secrets.h
	- moved network feed locations out of secrets.h
- 092020
	- [FR][P2]091420: Switch to M0 Proto board
	- [FR][P1]091120: Use timedisplay routines for log string
	- Added DEBUG and production sample rate definitions
	- [FR][P1]090820: Optimize code
	- [I]092020: SDLOG doesn't actually write values (code was dropped in previous revision) -> this has been true since initial Github checkin; fixed
- 091420
	- [I]091120: Set SYNC_INTERVAL to minimum for AIO and use in main CLOUDLOG -> done
	- [I]091120: Crashes again after one loop, might be a DHT issue -> DHT moved to pin 11, stopped collision with Ethernet on pin 10 (CS)
	- [I]091120: main delay(SYNC_INTERVAL) must be factored out, it could be impacting networking -> done
	- [I]091120: Data not writing to AIO -> side effect of DHT/Ethernet pin conflict
	- [FR]083120: Upload data to cloud db -> code now working
	- [FR]090820: After adding cloud db support, try backport to Arduino Ethernet board (enough memory?) -> does not fit, not worth the effort
	- [FR]090120: Add screen display support
- 091120
	- [I]090820: Code is only running for one loop -> setSyncInterval(15) locked code on subsequent loops, also not needed
	- [I]090820: time is not correct, sample code is -> byproduct of setSyncInterval(15) issue
	- [FR]090820: Display NTP time when received in DEBUG	
- 090820
	- Switched to Particle Ethernet Featherwing and Feather M4 Express
	- Switched to timelib getNtpTime example code
	- [FR]090120: Conditional compile for network, SD saves, and display
	- [FR]090120: Switch to ARM SoC for +memory (post conditionals?)
- 090520
	- Outside code work highlights Ethernet code is likely working, was blocked by failed primary DNS server
- 083120
	- First version based on merged sample code for sensors, SD. Ethernet code NOT working.