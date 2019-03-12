#ifndef NCDWIRELESS_H
#define NCDWIRELESS_H
#include <Arduino.h>
#include <ArduinoJson.h>

class NCDWireless{
public:
  bool parseData(uint8_t* data, int len, JsonObject& json, bool newDevice);
  bool newDevice(uint8_t* data, int len, JsonObject& json);

private:
  int rssiPin = 21;
};

#endif
