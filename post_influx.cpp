#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Only compile if InfluxDB enabled
#ifdef INFLUX

// Shared helper function we call here too...
extern void debugMessage(String messageText);

#include <InfluxDbClient.h>
// InfluxDB setup.  See config.h and secrets.h for site-specific settings

// InfluxDB client instance for InfluxDB 1
InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_DB_NAME);

// InfluxDB Data point, binds to InfluxDB 'measurement' to use for data
Point dbenvdata("weather");

// Post data to Influx DB using the connection established during setup
// Operates over the network, so may take a while to execute.
void post_influx(uint16_t co2, float tempF, float humidity)
{
  Serial.println("Saving data to Influx");
  // Set InfluxDB 1 authentication params using values defined in secrets.h
  dbclient.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);
  
  // Add constant Influx data point tags - only do once, will be added to all individual data points
  // Modify if required to reflect your InfluxDB data model (and set values in config.h)
  dbenvdata.addTag("device", DEVICE_NAME);
  dbenvdata.addTag("location", DEVICE_LOCATION);
  dbenvdata.addTag("site", DEVICE_SITE);

  // If confirmed connection to InfluxDB server, store our data values (with retries)
  boolean dbsuccess = false;
  uint8_t dbtries;
  for (dbtries = 1; dbtries <= 5; dbtries++) {
    if (dbclient.validateConnection()) {
      debugMessage("Connected to InfluxDB: " + dbclient.getServerUrl());
      dbsuccess = true;
      break;
    }
    delay(dbtries * 10000); // Waiting longer each time we check for status
  }
  if(dbsuccess == false) {
    debugMessage("InfluxDB connection failed: " + dbclient.getLastErrorMessage());
    return;
  }
  else {
    // Connected, so store measured values into timeseries data point
    dbenvdata.clearFields();
    // Report sensor readings
    dbenvdata.addField("temperature", tempF);
    dbenvdata.addField("humidity", humidity);
    dbenvdata.addField("co2", co2);
    debugMessage("Writing: " + dbclient.pointToLineProtocol(dbenvdata));
    // Write point via connection to InfluxDB host
    if (!dbclient.writePoint(dbenvdata)) {
      debugMessage("InfluxDB write failed: " + dbclient.getLastErrorMessage());
    }
    dbclient.flushBuffer();  // Clear pending writes (before going to sleep)
  }
}
#endif