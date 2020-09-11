/*
  Project Name:   air_quality
  Developer:      Eric Klein Jr. (temp2@ericklein.com)
  Description:    Measures and logs temp, humidity, and CO2 levels

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
#define DEBUG       // Output to the serial port
// #define SDLOG       // output sensor data to SD Card
//#define SCREEN      // output sensor data to screen
//#define CLOUDLOG    // output sensor data to cloud service
#define RJ45        // use Ethernet to send data to cloud service

// DHT (digital humidity and temperature) sensor
#include "DHT.h"
#define DHTPIN 10          // Digital pin connected to DHT sensor
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// SGP30 (eCO2) sensor
#include <Wire.h>
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;

#define LOG_INTERVAL  60000   // milliseconds between sensor reads
#define SYNC_INTERVAL 600000  // milliseconds before flush(); needed for CLOUDLOG?
uint32_t syncTime = 0;        // time of last write to SD or cloud

#ifdef SDLOG
  #include <SPI.h>
  #include <SD.h>
  #define SDPIN 4
  File logfile;
#endif

#ifdef RJ45
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EthernetUdp.h>
  // Enter a MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  EthernetUDP Udp;
  unsigned int localPort = 8888;       // local port to listen for UDP packets
#endif

// Time (and time related networking)
#include <TimeLib.h>
// NTP Servers
//IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
IPAddress timeServer(132,163,97,6); // time.nist.gov

// Time Zone support
//const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
const int timeZone = -7;  // Pacific Daylight Time (USA)
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

void setup() 
{
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) 
      {
        delay(1);
      }
    Serial.println("Indoor Air Quality started");
  #endif

  dht.begin();

  if (! sgp.begin())
  {
    #ifdef DEBUG
      Serial.println("SGP30 sensor not found");
    #endif
    while (1);
  }

  #ifdef DEBUG
    Serial.println("DHT22 and SGP30 sensors ready");
  #endif

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!

  #ifdef SDLOG
    // initialize SD card
    if (!SD.begin(SDPIN)) 
    {
      #ifdef DEBUG
        Serial.println("Card failed, or not present");
      #endif
      while (1);
    }
  
    // create a new file on SD card
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) 
    {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) 
      {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
  
    if (! logfile) 
    {
      #ifdef DEBUG
        Serial.println("Couldn't create log file");
      #endif
      while (1);
    }
  
    #ifdef DEBUG
      Serial.print("Logging to: ");
      Serial.println(filename);
    #endif

    // Log output headers
      logfile.println("time,humidity,temp,heat_index,eCO2");    
  #endif

  #ifdef RJ45
    // Configure Ethernet CS pin, not needed if using default D10
    //Ethernet.init(10);  // Most Arduino shields
    //Ethernet.init(5);   // MKR ETH shield
    //Ethernet.init(0);   // Teensy 2.0
    //Ethernet.init(20);  // Teensy++ 2.0
    //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
    //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

    // Initialize Ethernet and UDP
    if (Ethernet.begin(mac) == 0)
    {
      #ifdef DEBUG
        Serial.println("Failed to configure Ethernet using DHCP");
      #endif
      // Check for Ethernet hardware present
      if (Ethernet.hardwareStatus() == EthernetNoHardware)
      {
        #ifdef DEBUG
          Serial.println("Ethernet hardware not found");
        #endif
      } 
      else if (Ethernet.linkStatus() == LinkOFF) 
      {
        #ifdef DEBUG
          Serial.println("Ethernet cable is not connected.");
        #endif
      }
      while (1);
    }
    // Get time from NTP
    #ifdef DEBUG
      Serial.print("IP number assigned by DHCP is ");
      Serial.println(Ethernet.localIP());
    #endif
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);

    // wait until the time is set by the sync provider
    while(timeStatus()== timeNotSet);

    #ifdef DEBUG
      Serial.print("The NTP time is ");
      digitalClockDisplay();
    #endif
  #endif

  // serial output data headers
  #ifdef DEBUG
    Serial.println("time,humidity,temp,heat_index,eC02");
  #endif
}

void loop() 
{
  // #ifdef DEBUG
  //   Serial.println("Entering main loop");
  // #endif
  // delay between sensor readings
  delay(LOG_INTERVAL);

  // Reading temperature or humidity takes about 250 milliseconds!
  float humidity = dht.readHumidity();

  // Read temperature as Celsius (the default)
  //float t = dht.readTemperature();

  // Read temperature as Fahrenheit (isFahrenheit = true)
  float temperature_fahr = dht.readTemperature(true);

  // set the absolute humidity to enable the humditiy compensation for the SGP30 air quality signals
  sgp.setHumidity(getAbsoluteHumidity(temperature_fahr, humidity));

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature_fahr))
  //if (isnan(humidity) || isnan(t) || isnan(temperature_fahr))
  {
    #ifdef DEBUG
      Serial.println("Failed to read from DHT sensor!");
    #endif
    return;
  }

  if (! sgp.IAQmeasure()) 
  {
    Serial.println("SGP30 Measurement failed");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float heat_index_fahr = dht.computeHeatIndex(temperature_fahr, humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  //float hic = dht.computeHeatIndex(t, h, false);

  String logString = "";
  logString = year();
  logString += "/";
  logString += month();
  logString += "/";
  logString += day();
  logString += " ";
  logString += hour();
  logString += ":";
  logString += minute();
  logString += ":";
  logString += second();
  logString += ",";
  //logString += "Humidity: ";
  logString += humidity;
  logString += ",";
  //logString += "%  Temperature: ";
    //Serial.print(t);
    //Serial.print(F("°C "));
  logString += temperature_fahr;
  logString += ",";
  //logString += "°F  Heat index: ";
    //Serial.print(hic);
    //Serial.print(F("°C "));
  logString += heat_index_fahr;
  //logString += "°F";
    logString += ",";
  logString += sgp.eCO2;

  #ifdef DEBUG
    Serial.println(logString);
  #endif

  #ifdef SDLOG
    // writes to SD requires 2048 bytes of I/O to SD card which uses a bunch of power/time
    if ((millis() - syncTime) < SYNC_INTERVAL) return;
    syncTime = millis();
    logfile.flush();
    #ifdef DEBUG
      Serial.println("Log data written to SD card");
    #endif
  #endif
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  #ifdef DEBUG
    Serial.println("Transmitting NTP Request");
  #endif
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500)
  {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  #ifdef DEBUG
    Serial.println("No NTP response");
  #endif
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) 
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.println(year()); 
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}