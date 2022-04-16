#ifndef AQNetwork_h
#define AQNetwork_h

#include "Arduino.h"

class AQ_Network {
  public:
    bool networkBegin();
    void networkStop();
    String dateTimeString();
    String httpGETRequest(const char* serverName);
    int httpPOSTRequest(String serverurl, String contenttype, String payload);
    String getLocalIPString();
    long getWiFiRSSI();
    bool isWireless();
    bool isWired();
    bool isConnected();
  private:
};

#endif
