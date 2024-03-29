/*
  Project:      air_quality
  Description:  write sensor data to DWEET
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

#ifdef DWEET
  // Shared helper function
  extern void debugMessage(String messageText, int messageLevel);

  // Post a dweet to report the various sensor reading
  void post_dweet(uint16_t co2, float temperatureF, float humidity, float batteryVoltage, int rssi)
  {

    /*
    if(WiFi.status() != WL_CONNECTED) {
      debugMessage("Lost network connection to '" + String(WIFI_SSID) + "'!",1);
      // show_disconnected();  / Future feature to display state of network connectivity (LED?)
      return;
    }
    debugMessage("connecting to " + String(DWEET_HOST),1);
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
                         "\"temperature\":\"" + String(temperatureF, 2)         + "\"," +
                         "\"humidity\":\""    + String(humidity, 2)      + "\"}";
    }
    else
    {
      sensor_info = "\"temperature\":\"" + String(temperatureF, 2)         + "\"," +
                         "\"humidity\":\""    + String(humidity, 2)      + "\"}";
    }

    String postdata = device_info + battery_info + sensor_info;
    int httpCode = aq_network.httpPOSTRequest(dweeturl,"application/json",postdata);
    
    // httpCode will be negative on error, but HTTP status might indicate failure
    if (httpCode > 0) {
      // HTTP POST complete, print result code
      debugMessage("HTTP POST [dweet.io], result code: " + String(httpCode),1);
    } else {
      debugMessage("HTTP POST [dweet.io] failed, result code: " + String(httpCode),1);
    }
  }
#endif
