/*
  Project Name:   air_quality
  Description:    Regularly sample and log temperature, humidity, and if available, co2 levels

  See README.md for target information and revision history
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Only compile if InfluxDB enabled
#ifdef INFLUX
  // Shared helper function we call here too...
  extern void debugMessage(String messageText);

  // Status variables shared across various functions
  extern bool batteryVoltageAvailable;
  extern bool internetAvailable;

  #include <InfluxDbClient.h>

  // InfluxDB setup.  See config.h and secrets.h for site-specific settings.  Both InfluxDB v1.X
  // and v2.X are supported here depending on configuration settings in secrets.h.  Code here
  // reflects a number of presumptions about the data schema and InfluxDB configuration:
  //

  #ifdef INFLUX_V1
  // InfluxDB client instance for InfluxDB 1
  InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_DB_NAME);
  #endif

  #ifdef INFLUX_V2
  // InfluxDB client instance for InfluxDB 2
  InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
  #endif

  // Post data to Influx DB using the connection established during setup
  // Operates over the network, so may take a while to execute.
  boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_v, int rssi)
  {
    bool result = false;

    if (internetAvailable){

      // InfluxDB Data point, binds to InfluxDB 'measurement' to use for data. See config.h for value used
      Point dbenvdata(INFLUX_ENV_MEASUREMENT);
      Point dbdevdata(INFLUX_DEV_MEASUREMENT);
      
      #ifdef INFLUX_V1
        // Set InfluxDB v1.X authentication params using values defined in secrets.h.  Not needed as such
        // for InfluxDB v2.X (which uses a token-based scheme via the constructor).
        dbclient.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);
      #endif
      
      // Add constant Influx data point tags - only do once, will be added to all individual data points
      // Modify if required to reflect your InfluxDB data model (and set values in config.h)
      // First for environmental data
      dbenvdata.addTag("device", DEVICE_TYPE);
      dbenvdata.addTag("location", DEVICE_LOCATION);
      dbenvdata.addTag("site", DEVICE_SITE);
      // And again for device data
      dbdevdata.addTag("device", DEVICE_TYPE);
      dbdevdata.addTag("location", DEVICE_LOCATION);
      dbdevdata.addTag("site", DEVICE_SITE);

      // Attempts influxDB connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
      for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++) {
        if (dbclient.validateConnection()) {
          debugMessage(String("Connected to InfluxDB: ") + dbclient.getServerUrl());
          result = true;
          break;
        }
        debugMessage(String("influxDB connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " failed with error msg: " + dbclient.getLastErrorMessage());
        delay(CONNECT_ATTEMPT_INTERVAL*1000);
      }
      if (result){
        // Connected, so store sensor values into timeseries data point
        dbenvdata.clearFields();
        // Report sensor readings
        dbenvdata.addField("tempF", tempF);
        dbenvdata.addField("humidity", humidity);
        if (co2 !=10000)
          dbenvdata.addField("co2", co2);
        // Write point via connection to InfluxDB host
        if (!dbclient.writePoint(dbenvdata)) {
          debugMessage("InfluxDB write failed: " + dbclient.getLastErrorMessage());
          result = false;
        }
        else
        {
          debugMessage(String("InfluxDB write success: ") + dbclient.pointToLineProtocol(dbenvdata));
        }

        // Now store device information 
        dbdevdata.clearFields();
        // Report device readings
        if (batteryVoltageAvailable)
          dbdevdata.addField("battery_volts", battery_v);
        if (internetAvailable)
          dbdevdata.addField("rssi", rssi);
        if ((batteryVoltageAvailable) || (internetAvailable))
        {
          if (!dbclient.writePoint(dbdevdata))
          {
            debugMessage("InfluxDB write failed: " + dbclient.getLastErrorMessage());
            result = false;
          }
          else
          {
            debugMessage(String("InfluxDB write success: ") + dbclient.pointToLineProtocol(dbdevdata));
          }
        }
        dbclient.flushBuffer();  // Clear pending writes (before going to sleep)
      }
    }
    return(result);
  }
#endif