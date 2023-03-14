#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

#include "aq_network.h"
extern AQ_Network aq_network;


#ifdef DWEET
#include <HTTPClient.h> 

// Shared helper function we call here too...
extern void debugMessage(String messageText);
// extern int httpPOSTRequest(String serverurl, String contenttype, String payload);

// Post a dweet to report the various sensor reading.  This routine blocks while
// talking to the network, so may take a while to execute.
void post_dweet(uint16_t co2, float tempF, float humidity, float batteryVoltage, int rssi)
{

  /*
  if(WiFi.status() != WL_CONNECTED) {
    debugMessage("Lost network connection to '" + String(WIFI_SSID) + "'!");
    // show_disconnected();  / Future feature to display state of network connectivity (LED?)
    return;
  }
  debugMessage("connecting to " + String(DWEET_HOST));
  */

  String dweeturl = "http://" + String(DWEET_HOST) + "/dweet/for/" + String(DWEET_DEVICE);

  /*
  // Use HTTP class to publish dweet
  HTTPClient dweetio;
  dweetio.begin(client,dweeturl);  
  dweetio.addHeader("Content-Type", "application/json");
  */

  // Use HTTP post and send a data payload as JSON
  String device_info = "{\"rssi\":\""   + String(rssi)        + "\"," +
                       "\"ipaddr\":\"" + WiFi.localIP().toString()  + "\",";
  String battery_info;
  if(batteryVoltage > 0) {
    battery_info = "\"battery_voltage\":\"" + String(batteryVoltage)   + "\",";
  }
  else {
    battery_info = "";
  }

  String sensor_info;
  if (co2 !=10000)
  {
    sensor_info = "\"co2\":\""         + String(co2)             + "\"," +
                       "\"temperature\":\"" + String(tempF, 2)         + "\"," +
                       "\"humidity\":\""    + String(humidity, 2)      + "\"}";
  }
  else
  {
    sensor_info = "\"temperature\":\"" + String(tempF, 2)         + "\"," +
                       "\"humidity\":\""    + String(humidity, 2)      + "\"}";
  }

  String postdata = device_info + battery_info + sensor_info;
  int httpCode = aq_network.httpPOSTRequest(dweeturl,"application/json",postdata);
  
  // httpCode will be negative on error, but HTTP status might indicate failure
  if (httpCode > 0) {
    // HTTP POST complete, print result code
    debugMessage("HTTP POST [dweet.io], result code: " + String(httpCode) );
  } else {
    debugMessage("HTTP POST [dweet.io] failed, result code: " + String(httpCode));
  }
}
#endif
