#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"
// network functionality
#include "aq_network.h"

// required external functions and data structures
extern void debugMessage(String messageText, int messageLevel);
extern AQ_Network aq_network;

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
      debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER,2);
      return;
    }

    int8_t mqttErr;

    for(int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    {
      if ((mqttErr = aq_mqtt.connect()) == 0)
      {
        debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER,2);
        return;
      }
      aq_mqtt.disconnect();
      debugMessage(String("MQTT connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " failed with error msg: " + aq_mqtt.connectErrorString(mqttErr),1);
      delay(CONNECT_ATTEMPT_INTERVAL*1000);
    }
  }

  bool mqttDeviceBatteryUpdate(float batteryVoltage)
  {
    bool result = false;
    if (batteryVoltage>0)
    {
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_BATTVOLT);
      mqttConnect();

      // publish battery voltage
      if (batteryVoltagePub.publish(batteryVoltage))
      {
        debugMessage("MQTT publish: Battery Voltage succeeded",1);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: Battery Voltage failed",1);
      }
    }
    return(result);
  }

  bool mqttDeviceWiFiUpdate(int rssi)
  {
    bool result = false;
    if (rssi!=0)
    {
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_RSSI);
      
      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded",1);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed",1);
      }
    }
    return(result);
  }
  
  bool mqttSensorTempFUpdate(float tempF)
  // Publishes temperature data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TEMPF);
    
    mqttConnect();

    // Attempt to publish sensor data
    if(tempPub.publish(tempF))
    {
      debugMessage("MQTT publish: Temperature succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Temperature failed",1);
    }
    return(result);
  }

  bool mqttSensorHumidityUpdate(float humidity)
  // Publishes humidity data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_HUMIDITY);
    
    mqttConnect();
    
    // Attempt to publish sensor data
    if(humidityPub.publish(humidity))
    {
      debugMessage("MQTT publish: Humidity succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Humidity failed",1);
    }
    return(result);
  }

  bool mqttSensorCO2Update(uint16_t co2)
  // Publishes CO2 data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_CO2);   
    
    mqttConnect();

    // Attempt to publish sensor data
    if (co2 != 10000)
    {
      if(co2Pub.publish(co2))
      {
        debugMessage("MQTT publish: CO2 succeeded",1);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: CO2 failed",1);
      }
    }
    return(result);
  }
#endif