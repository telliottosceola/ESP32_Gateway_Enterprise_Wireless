#ifndef NCDWIRELESS_H
#define NCDWIRELESS_H
#include <Arduino.h>
#include <ArduinoJson.h>

class NCDWireless{
public:
  bool parseData(uint8_t* data, int len, JsonObject& json, bool newDevice, bool addNodeID = true, bool addBatteryLevel = true);
  bool newDevice(uint8_t* data, int len, JsonObject& json, bool attributesOnly = false);

private:
  int rssiPin = 21;
  String lastHeardString = "[]";
};

#endif
