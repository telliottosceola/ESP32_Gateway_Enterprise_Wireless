#include <NCDWireless.h>

union sfp24bit {
  byte b[4];
  long result;
} tval24;

union sfp16bit {
  byte b[2];
  int16_t result;
} tval16;

int signedInt(uint8_t* data, int start, int bits){
  if(bits == 24){
    tval24.b[3] = data[start]; // low byte
    tval24.b[2] = data[start+1]; // middle byte
    tval24.b[1] = data[start+2]; // high byte
    tval24.b[0] = (data[start+2] & 0x80 ? 0xFF : 0);
    return tval24.result;
  }
  if(bits == 16){
    tval16.b[1] = data[start];
    tval16.b[0] = data[start+1];
    return tval16.result;
  }
}

bool NCDWireless::parseData(uint8_t* data, int len, JsonObject& json, bool newDevice){
  if(data[0] != 127){
    return false;
  }
  if(len < 9){
    return false;
  }

  JsonObject& dataObject = json["data"];

  int nodeID = data[1];
  int firmware = data[2];
  float battery = ((data[3]<<8)+data[4])*0.00322;
  int counter = data[5];
  int sensorType = (data[6]<<8)+data[7];
  dataObject["transmission_count"] = counter;
  dataObject["battery_level"] = battery;
  dataObject["type"] = sensorType;
  dataObject["node_id"] = nodeID;

  unsigned long pulseWidth = pulseIn(rssiPin, HIGH, 250);
  if(pulseWidth == 0){
    if(digitalRead(rssiPin) == HIGH){
      dataObject["rssi"] = 100;
    }else{
      // Serial.println("Theoretically this should never print");
      dataObject["rssi"] = 0;
    }
  }else{
    float highPercentage = (pulseWidth/208.00)*100.00;
    dataObject["rssi"] = (int)highPercentage;
  }

  bool rDevice = false;

  if(sensorType == 40){
    return false;
  }

  switch(sensorType){
    case(1):{
      //Temperature humidity
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Temperature humidity";
        json["SKU"] = "";
      }
      dataObject["humidity"] = (float)(((data[9]<<8)+data[10])/100.00);
      int16_t unconverted = (data[11]<<8)+data[12];
      dataObject["temperature"] = (float)(unconverted/100.00);
      rDevice = true;
      break;
    }
    case(2):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "2 channel Push Notification";
        json["SKU"] = "";
      }
      //2 channel Push Notification
      dataObject["input_1"] = data[9];
      dataObject["input_2"] = data[10];
      rDevice = true;
      break;
    }
    case(3):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "2 channel ADC";
        json["SKU"] = "";
      }
      //2 channel ADC
      dataObject["input_1"] = ((data[9]<<8)+data[10]);
      dataObject["input_2"] = ((data[11]<<8)+data[12]);
      rDevice = true;
      break;
    }
    case(4):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Thermocouple";
        json["SKU"] = "";
      }
      //Thermocouple
      int32_t unconverted = ((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12]);
      dataObject["temperature"] = (float)(unconverted/100.00);
      rDevice = true;
      break;
    }
    case(5):{
      if(len < 38){
        return false;
      }
      if(newDevice){
        json["Type"] = "Gyro/Magneto/Temperature";
        json["SKU"] = "";
      }
      //Gyro/Magneto/Temperature
      dataObject["accel_x"] = (float)(signedInt(data, 9, 24)/100.00);
      dataObject["accel_y"] = (float)(signedInt(data, 12, 24)/100.00);
      dataObject["accel_z"] = (float)(signedInt(data, 15, 24)/100.00);
      dataObject["magneto_x"] = (float)(signedInt(data, 18, 24)/100.00);
      dataObject["magneto_y"] = (float)(signedInt(data, 21, 24)/100.00);
      dataObject["magneto_z"] = (float)(signedInt(data, 24, 24)/100.00);
      dataObject["gyro_x"] = (float)(signedInt(data, 27, 24)/100.00);
      dataObject["gyro_y"] = (float)(signedInt(data, 30, 24)/100.00);
      dataObject["gyro_z"] = (float)(signedInt(data, 33, 24)/100.00);
      dataObject["temperature"] = (int16_t)(data[36]<<8)+data[37];
      rDevice = true;
      break;
    }
    case(6):{
      if(len < 17){
        return false;
      }
      if(newDevice){
        json["Type"] = "Temperature/Barometeric Pressure";
        json["SKU"] = "";
      }
      //Temperature/Barometeric Pressure
      dataObject["temperature"] = (int16_t)(data[9]<<8)+data[10];
      dataObject["absolute_pressure"] = (float)((uint16_t)(data[11]<<8)+data[12])/1000.00;
      dataObject["relative_pressure"] = (float)(signedInt(data, 13, 16)/1000.00);
      dataObject["altitude_change"] = (float)(signedInt(data, 15, 16)/100.00);
      rDevice = true;
      break;
    }
    case(7):{
      if(len < 29){
        return false;
      }
      if(newDevice){
        json["Type"] = "Impact Detection";
        json["SKU"] = "";
      }
      //Impact Detection
      dataObject["acc_x1"] = signedInt(data, 9, 16);
      dataObject["acc_x2"] = signedInt(data, 11, 16);
      dataObject["acc_x"] = signedInt(data, 13, 16);
      dataObject["acc_y1"] = signedInt(data, 15, 16);
      dataObject["acc_y2"] = signedInt(data, 17, 16);
      dataObject["acc_y"] = signedInt(data, 19, 16);
      dataObject["acc_z1"] = signedInt(data, 21, 16);
      dataObject["acc_z2"] = signedInt(data, 23, 16);
      dataObject["acc_z"] = signedInt(data, 25, 16);
      dataObject["temp_change"] = signedInt(data, 27, 16);
      rDevice = true;
      break;
    }
    case(8):{
      if(len < 38){
        return false;
      }
      if(newDevice){
        json["Type"] = "Vibration";
        json["SKU"] = "";
      }
      //Vibration
      dataObject["rms_x"] = (float)(signedInt(data, 9, 24)/100.00);
      dataObject["rms_y"] = (float)(signedInt(data, 12, 24)/100.00);
      dataObject["rms_z"] = (float)(signedInt(data, 15, 24)/100.00);
      dataObject["max_x"] = (float)(signedInt(data, 18, 24)/100.00);
      dataObject["max_y"] = (float)(signedInt(data, 21, 24)/100.00);
      dataObject["max_z"] = (float)(signedInt(data, 24, 24)/100.00);
      dataObject["min_x"] = (float)(signedInt(data, 27, 24)/100.00);
      dataObject["min_y"] = (float)(signedInt(data, 30, 24)/100.00);
      dataObject["min_z"] = (float)(signedInt(data, 33, 24)/100.00);
      dataObject["temperature"] = (int16_t)(data[36]<<8)+data[37];
      rDevice = true;
      break;
    }
    case(9):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Proximity";
        json["SKU"] = "";
      }
      //Proximity
      dataObject["proximity"] = ((data[9]<<8)+data[10]);
      dataObject["lux"] = (float)(((data[11]<<8)+data[12])*0.25);
      rDevice = true;
      break;
    }
    case(10):{
      if(len < 12){
        return false;
      }
      if(newDevice){
        json["Type"] = "Light";
        json["SKU"] = "";
      }
      //Light
      dataObject["lux"] = (data[9]<<16)+(data[10]<<8)+data[11];
      rDevice = true;
      break;
    }
    case(13):{
      if(len < 12){
        return false;
      }
      if(newDevice){
        json["Type"] = "Current Monitor";
        json["SKU"] = "";
      }
      //Current Monitor
      dataObject["Current"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11])/1000.00);
      rDevice = true;
      break;
    }
    case(24):{
      if(len < 17){
        return false;
      }
      if(newDevice){
        json["Type"] = "Activity Detection";
        json["SKU"] = "";
      }
      //Activity Detection
      dataObject["acc_x"] = signedInt(data, 9, 16);
      dataObject["acc_y"] = signedInt(data, 11, 16);
      dataObject["acc_z"] = signedInt(data, 13, 16);
      dataObject["temp_change"] = signedInt(data, 15, 16);
      rDevice = true;
      break;
    }
    case(25):{
      if(len < 17){
        return false;
      }
      if(newDevice){
        json["Type"] = "Asset Monitor";
        json["SKU"] = "";
      }
      //Asset Monitor
      dataObject["acc_x"] = signedInt(data, 9, 16);
      dataObject["acc_y"] = signedInt(data, 11, 16);
      dataObject["acc_z"] = signedInt(data, 13, 16);
      dataObject["temp_change"] = signedInt(data, 15, 16);
      rDevice = true;
      break;
    }
    case(27):{
        if(len < 25){
          return false;
        }
        if(newDevice){
          json["Type"] = "Environmental";
          json["SKU"] = "";
        }
        dataObject["temperature"] = signedInt(data, 9, 16) / 100;
        dataObject["pressure"] = ((data[11]<<24)+(data[12]<<16)+(data[13]<<8)+data[14]) / 100;
        dataObject["humidity"] = ((data[15]<<24)+(data[16]<<16)+(data[17]<<8)+data[18]) / 1000;
        dataObject["gas_resistance"] = ((data[19]<<24)+(data[20]<<16)+(data[21]<<8)+data[22]);
        dataObject["iaq"] = (data[23]<<8)+data[24];
        rDevice = true;
        break;
    }
    case(35):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "One Channel Counter";
        json["SKU"] = "";
      }
      //One Channel Counter
      dataObject["count"] = (data[9]<<8)+data[10];
      rDevice = true;
      break;
    }
    case(36):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Two Channel Counter";
        json["SKU"] = "";
      }
      //Two Channel Counter
      dataObject["count_1"] = (data[9]<<8)+data[10];
      dataObject["count_2"] = (data[11]<<8)+data[12];
      rDevice = true;
      break;
    }
    case(37):{
      if(len < 14){
        return false;
      }
      if(newDevice){
        json["Type"] = "7 Channel Push Notification";
        json["SKU"] = "";
      }
      //7 Channel Push Notification
      dataObject["input_1"] = (data[9]&1)?1:0;
      dataObject["input_2"] = (data[9]&2)?1:0;
      dataObject["input_3"] = (data[9]&4)?1:0;
      dataObject["input_4"] = (data[9]&8)?1:0;
      dataObject["input_5"] = (data[9]&16)?1:0;
      dataObject["input_6"] = (data[9]&32)?1:0;
      dataObject["input_7"] = (data[9]&64)?1:0;
      dataObject["adc_1"] = (data[10]<<8)+data[11];
      dataObject["adc_2"] = (data[12]<<8)+data[13];
      rDevice = true;
      break;
    }
    case(10006):{
      if(len < 17){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-Channel 4-20 mA Input";
        json["SKU"] = "";
      }
      //4-Channel 4-20 mA Input
      dataObject["input_1"] = (float)(((data[9]<<8)+data[10])/100.00);
      dataObject["input_2"] = (float)(((data[11]<<8)+data[12])/100.00);
      dataObject["input_3"] = (float)(((data[13]<<8)+data[14])/100.00);
      dataObject["input_4"] = (float)(((data[15]<<8)+data[16])/100.00);
      rDevice = true;
      break;
    }
    case(10007):{
      if(len < 17){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-Channel Current Monitor";
        json["SKU"] = "";
      }
      //4-Channel Current Monitor
      dataObject["channel_1"] = (float)(((data[9]<<8)+data[10])/1000.00);
      dataObject["channel_2"] = (float)(((data[11]<<8)+data[12])/1000.00);
      dataObject["channel_3"] = (float)(((data[13]<<8)+data[14])/1000.00);
      dataObject["channel_4"] = (float)(((data[15]<<8)+data[16])/1000.00);
      rDevice = true;
      break;
    }
    case(10012):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "2-Relay + 2-Input";
        json["SKU"] = "";
      }
      //2-Relay + 2-Input
      dataObject["relay_1"] = data[9];
      dataObject["relay_2"] = data[10];
      dataObject["input_1"] = data[11]?"On":"Off";
      dataObject["input_2"] = data[12]?"On":"Off";
      rDevice = true;
      break;
    }
  }
  if(!rDevice){
    return false;
    char rawDataChar[((len-10)*3)+1];
    char *format = "%02X ";
    char newData[4];
    if(len < 128){

    }else{
      #ifdef DEBUG
      Serial.printf("This is an unknown device sending too much data, length is:%i\n", len);
      #endif
      return false;
    }
    for(int i = 9; i < len; i++){
      sprintf(newData, format, data[i]);
      strcat(rawDataChar, newData);
    }
    dataObject["raw_data"] = rawDataChar;
    json["Type"] = "Unknown Device";
    json["SKU"] = "";
    return true;
  }else{
    return true;
  }
}

bool NCDWireless::newDevice(uint8_t* data, int len, JsonObject& json){

  if(data[0] != 127){
    return false;
  }

  DynamicJsonBuffer jsonBufferOne;
  JsonObject& dataObject = jsonBufferOne.createObject();
  dataObject.createNestedObject("data");
  if(!parseData(data, len, dataObject, true)){
    return false;
  }

  int nodeID = data[1];
  int firmware = data[2];
  int sensorType = (data[6]<<8)+data[7];

  JsonArray& tags = json["tags"].as<JsonArray>();

  JsonObject& nodeIDTag = tags.createNestedObject();
  nodeIDTag["key"] = "node_id";
  nodeIDTag["value"] = String(nodeID);

  JsonObject& firmwareTag = tags.createNestedObject();
  firmwareTag["key"] = "firmware_version";
  firmwareTag["value"] = String(firmware);

  JsonObject& deviceTypeTag = tags.createNestedObject();
  deviceTypeTag["key"] = "device_type_id";
  deviceTypeTag["value"] = String(sensorType);

  json["description"] = dataObject["Type"];

  JsonArray& attributes = json["attributes"].as<JsonArray>();

  JsonObject& attributesObject = dataObject["data"];
  for(auto kvp : attributesObject){
    JsonObject& attributeKVPObject = attributes.createNestedObject();
    attributeKVPObject["name"] = kvp.key;
    attributeKVPObject["dataType"] = "number";
  }
  #ifdef DEBUG
  Serial.println("newDevice ran");
  #endif
  return true;
}
