#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

#include "aq_network.h"
extern AQ_Network aq_network;

// Shared helper function we call here too...
extern void debugMessage(String messageText);

// Status variables shared across various functions
extern bool batteryVoltageAvailable;
extern bool internetAvailable;

#ifdef MQTT
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  extern Adafruit_MQTT_Client aq_mqtt;

  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {  
    // exit if already connected
    if (aq_mqtt.connected())
    {
      debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER);
      return;
    }

    int8_t mqttErr;

    for(int tries =1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    {
      if ((mqttErr = aq_mqtt.connect()) == 0)
      {
        debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
        return;
      }
      // Adafruit IO connect errors
      // switch (mqttErr)
      // {
      //   case 1: debugMessage("Adafruit MQTT: Wrong protocol"); break;
      //   case 2: debugMessage("Adafruit MQTT: ID rejected"); break;
      //   case 3: debugMessage("Adafruit MQTT: Server unavailable"); break;
      //   case 4: debugMessage("Adafruit MQTT: Incorrect user or password"); break;
      //   case 5: debugMessage("Adafruit MQTT: Not authorized"); break;
      //   case 6: debugMessage("Adafruit MQTT: Failed to subscribe"); break;
      //   default: debugMessage("Adafruit MQTT: GENERIC - Connection failed"); break;
      // }
      aq_mqtt.disconnect();
      debugMessage(String("MQTT connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " failed with error msg: " + aq_mqtt.connectErrorString(mqttErr));
      delay(CONNECT_ATTEMPT_INTERVAL*1000);
    }
  }

  bool mqttDeviceBatteryUpdate(float cellVoltage)
  {
    bool result = false;
    if (batteryVoltageAvailable)
    {
      // Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_BATTVOLT, MQTT_QOS_1); // if problematic, remove QOS parameter
      Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_BATTVOLT);
      mqttConnect();

      // publish battery voltage
      if (batteryVoltagePub.publish(cellVoltage))
      {
        debugMessage("MQTT publish: Battery Voltage succeeded");
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: Battery Percent failed");
      }
      //aq_mqtt.disconnect(); 
    }
    return(result);
  }

  bool mqttDeviceWiFiUpdate(int rssi)
  {
    bool result = false;
    if (internetAvailable)
    {
      // Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_RSSI, MQTT_QOS_1); // if problematic, remove QOS parameter
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_RSSI);
      
      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded");
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed");
      }
      //aq_mqtt.disconnect();
    }
    return(result);
  }
  
  bool mqttSensorUpdate(uint16_t co2, float tempF, float humidity)
  // Publishes sensor data to MQTT broker
  {
    bool result = false;
    if (internetAvailable)
    {
      // Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TEMPF, MQTT_QOS_1); // if problematic, remove QOS parameter
      // Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_HUMIDITY, MQTT_QOS_1);
      // Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_CO2, MQTT_QOS_1);
      Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TEMPF);
      Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_HUMIDITY);
      Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_CO2);   
      
      mqttConnect();

      // Attempt to publish sensor data
      if(tempPub.publish(tempF))
      {
        debugMessage("MQTT publish: Temperature succeeded");
      }
      else {
        debugMessage("MQTT publish: Temperature failed");
        result = false;
      }
      
      if(humidityPub.publish(humidity))
      {
        debugMessage("MQTT publish: Humidity succeeded");
      }
      else {
        debugMessage("MQTT publish: Humidity failed");
        result = false;
      }
      
      if (co2 != 10000)
      {
        if(co2Pub.publish(co2))
        {
          debugMessage("MQTT publish: CO2 succeeded");
        }
        else
        {
          debugMessage("MQTT publish: CO2 failed");
          result = false;
        }
      }
      //aq_mqtt.disconnect();
    }
    return(result);
  }
#endif