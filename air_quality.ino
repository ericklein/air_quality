/*
  Project Name:   air_quality
  Developer:      Eric Klein Jr. (temp2@ericklein.com)
  Description:    Measures and logs temp, humidity, and CO2 levels

  See README.md for target information, revision history, feature requests, etc.
*/

// Conditional compile flags
#define DEBUG       // Output to the serial port

// DHT sensor setup
#include "DHT.h"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// SGP30 sensor (eCO2)
#include <Wire.h>
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// SD card
#include <SPI.h>
#include <SD.h>
#define SDPIN 4
File logfile;

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  60000 // mills between entries (reduce to take more/faster data)

//h ow many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 180000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

// // Ethernet
// #include <Ethernet.h>
// #include <EthernetUdp.h>
// // Enter a MAC address for your controller below.
// // Newer Ethernet shields have a MAC address printed on a sticker on the shield
// byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x30, 0x97};
// unsigned int localPort = 8888;       // local port to listen for UDP packets
// const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server
// const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
// byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// // A UDP instance to let us send and receive packets over UDP
// EthernetUDP Udp;

#include "TimeLib.h"

void setup() 
{
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) 
      {
        delay(1);
      }
      Serial.println("Air Quality started");
  #endif

  dht.begin();

    if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!

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
    Serial.println("Couldn't create log file");
    while (1);
  }
  
  #ifdef DEBUG
    Serial.print("Logging to: ");
    Serial.println(filename);
  #endif

  // Log output headers
    logfile.println("time,humidity,temp,heat_index,eCO2");    
  #ifdef DEBUG
    Serial.println("time,humidity,temp,heat_index,eC02");
  #endif

  // // Ethernet
  // // You can use Ethernet.init(pin) to configure the CS pin
  // Ethernet.init(10);  // Most Arduino shields
  // //Ethernet.init(5);   // MKR ETH shield
  // //Ethernet.init(0);   // Teensy 2.0
  // //Ethernet.init(20);  // Teensy++ 2.0
  // //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  // //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet
  // // start Ethernet and UDP
  // if (Ethernet.begin(mac) == 0) {
  //   Serial.println("Failed to configure Ethernet using DHCP");
  //   // Check for Ethernet hardware present
  //   if (Ethernet.hardwareStatus() == EthernetNoHardware) {
  //     Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  //   } else if (Ethernet.linkStatus() == LinkOFF) {
  //     Serial.println("Ethernet cable is not connected.");
  //   }
  //   // no point in carrying on, so do nothing forevermore:
  //   while (true) {
  //     delay(1);
  //   }
  // }
  // Udp.begin(localPort);
  // setTimeFromNTPServer();
}

void loop() 
{
  // delay for the amount of time we want between readings
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
  // Add timestamp (from NTP server) here
  logString += "MM/DD/YY HH:MM,";
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

  // write to SD. Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();
  #ifdef DEBUG
    Serial.println("file saved");
  #endif

  //Ethernet.maintain();
}

// // send an NTP request to the time server at the given address
// void sendNTPpacket(const char * address) {
//   // set all bytes in the buffer to 0
//   memset(packetBuffer, 0, NTP_PACKET_SIZE);
//   // Initialize values needed to form NTP request
//   // (see URL above for details on the packets)
//   packetBuffer[0] = 0b11100011;   // LI, Version, Mode
//   packetBuffer[1] = 0;     // Stratum, or type of clock
//   packetBuffer[2] = 6;     // Polling Interval
//   packetBuffer[3] = 0xEC;  // Peer Clock Precision
//   // 8 bytes of zero for Root Delay & Root Dispersion
//   packetBuffer[12]  = 49;
//   packetBuffer[13]  = 0x4E;
//   packetBuffer[14]  = 49;
//   packetBuffer[15]  = 52;

//   // all NTP fields have been given values, now
//   // you can send a packet requesting a timestamp:
//   Udp.beginPacket(address, 123); // NTP requests are to port 123
//   Udp.write(packetBuffer, NTP_PACKET_SIZE);
//   Udp.endPacket();
// }

// void setTimeFromNTPServer() 
// {
//   sendNTPpacket(timeServer); // send an NTP packet to a time server

//   // wait to see if a reply is available
//   delay(1000);
//   if (Udp.parsePacket()) 
//   {
//     // We've received a packet, read the data from it
//     Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

//     // the timestamp starts at byte 40 of the received packet and is four bytes,
//     // or two words, long. First, extract the two words:

//     unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
//     unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
//     // combine the four bytes (two words) into a long integer
//     // this is NTP time (seconds since Jan 1 1900):
//     unsigned long secsSince1900 = highWord << 16 | lowWord;
//     Serial.print("Seconds since Jan 1 1900 = ");
//     Serial.println(secsSince1900);

//     // now convert NTP time into everyday time:
//     Serial.print("Unix time = ");
//     // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
//     const unsigned long seventyYears = 2208988800UL;
//     // subtract seventy years:
//     unsigned long epoch = secsSince1900 - seventyYears;
//     // print Unix time:
//     Serial.println(epoch);
//     setTime(epoch);
//   }

  //   // print the hour, minute and second:
  //   Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
  //   Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
  //   Serial.print(':');
  //   if (((epoch % 3600) / 60) < 10) {
  //     // In the first 10 minutes of each hour, we'll want a leading '0'
  //     Serial.print('0');
  //   }
  //   Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
  //   Serial.print(':');
  //   if ((epoch % 60) < 10) {
  //     // In the first 10 seconds of each minute, we'll want a leading '0'
  //     Serial.print('0');
  //   }
  //   Serial.println(epoch % 60); // print the second
  // }
  // // wait ten seconds before asking for the time again
  // Ethernet.maintain();