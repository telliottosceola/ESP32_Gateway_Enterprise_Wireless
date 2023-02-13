#include <NCDWireless.h>

union sfp32bit {
  byte b[4];
  int result;
} tval32;

union sfp24bit {
  byte b[4];
  int result;
} tval24;

union sfp16bit {
  byte b[2];
  int16_t result;
} tval16;

int32_t sign_extend_24_32(int32_t x) {
  const int bits = 24;
  uint32_t m = 1u << (bits - 1);
  return (x ^ m) - m;
}

int signedInt(uint8_t* data, int start, int bits){
  if(bits == 32){
    tval32.b[3] = data[start]; // low byte
    tval32.b[2] = data[start+1]; // middle byte
    tval32.b[1] = data[start+2]; // high byte
    tval32.b[0] = data[start+3];
    return tval32.result;
  }
  if(bits == 24){
    int prior = ((data[start])<<16)+(data[start+1]<<8)+(data[start+2]);
    int result = sign_extend_24_32(prior);
    return result;
  }
  if(bits == 16){
    tval16.b[1] = data[start];
    tval16.b[0] = data[start+1];
    return tval16.result;
  }
}

bool NCDWireless::parseData(uint8_t* data, int len, JsonObject& json, bool newDevice, bool addNodeID, bool addBatteryLevel){
  if(data[0] != 127){
    Serial.println("Not an NCD Device");
    return false;
  }
  if(len < 9){
    Serial.println("Packet too short");
    return false;
  }

  JsonObject& dataObject = json["data"];

  int nodeID = data[1];
  int firmware = data[2];
  float battery = ((data[3]<<8)+data[4])*0.00322;
  int counter = data[5];
  int sensorType = (data[6]<<8)+data[7];
  int reserveByte = data[8];
  dataObject["firmware_version"] = firmware;
  dataObject["transmission_count"] = counter;
  dataObject["reserve_byte"] = reserveByte;
  if(addBatteryLevel){
    dataObject["battery_level"] = battery;
  }
  dataObject["type"] = sensorType;
  if(addNodeID){
    dataObject["node_id"] = nodeID;
  }

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

  // if(sensorType == 40){
  //   return false;
  // }

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
    case(12):{
      if(len<21){
        return false;
      }
      if(newDevice){
        json["Type"] = "3 Channel Thermocouple";
        json["SKU"] = "";
      }
      int32_t unconvertedOne = ((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12]);
      dataObject["Channel_1"] = (float)(unconvertedOne/100.00);

      int32_t unconvertedTwo = ((data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16]);
      dataObject["Channel_2"] = (float)(unconvertedTwo/100.00);

      int32_t unconvertedThree = ((data[17]<<24)+(data[18]<<16)+(data[19]<<8)+data[20]);
      dataObject["Channel_3"] = (float)(unconvertedThree/100.00);

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
    case(14):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-20mA Current Receiver";
        json["SKU"] = "";
      }
      int rawADC = (data[9]<<8)+data[10];
      Serial.printf("Raw ADC: %i\n", rawADC);
      float mA = (float)(rawADC*(20.00/998));
      dataObject["raw_adc"] = rawADC;
      dataObject["mA"] = mA;
      Serial.printf("mA: %0.2f\n", mA);
      rDevice = true;
      break;
    }
    case(15):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "10-Bit 1-Channel ADC";
        json["SKU"] = "";
      }
      uint16_t raw = ((data[9]<<8)+data[10]);
      dataObject["Raw ADC"] = raw;
      dataObject["Voltage"] = (float)(raw*0.00322265625);
      rDevice = true;
      break;
    }
    case(16):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Soil Moisture Sensor";
        json["SKU"] = "PR55-2B";
      }
      uint16_t raw = ((data[9]<<8)+data[10]);
      if(raw >= 870){
        dataObject["Moisture"] = (float)100.00;
      }else{
        dataObject["Moisture"] = (float)(raw/870.00);
      }
      rDevice = true;
      break;
    }
    case(17):{
      if(len < 12){
        return false;
      }
      if(newDevice){
        json["Type"] = "AC Voltage Monitor";
        json["SKU"] = "PR52-13";
      }
      dataObject["voltage"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11])/1000.00);
      rDevice = true;
      break;
    }
    case(18):{
      if(len < 14){
        return false;
      }
      if(newDevice){
        json["Type"] = "Pulse/Frequency Meter";
        json["SKU"] = "";
      }
      dataObject["frequency"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11])/1000.00);
      dataObject["duty_cycle"] = (float)(((data[12]<<8)+data[13])/100.00);
      rDevice = true;
      break;
    }
    case(19):{
      if(len < 15){
        return false;
      }
      if(newDevice){
        json["Type"] = "2 channel 24-bit Current Monitor";
        json["SKU"] = "";
      }
      dataObject["channel_1_milliamps"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11]));
      dataObject["channel_2_milliamps"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15]));
      dataObject["channel_1_amps"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11])/1000);
      dataObject["channel_2_amps"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15])/1000);
      rDevice = true;
      break;
    }
    case(20):{
      if(len < 15){
        return false;
      }
      if(newDevice){
        json["Type"] = "Precision Pressure & Temperature";
        json["SKU"] = "";
      }
      dataObject["pressure_pascal"] = (float)(signedInt(data, 9, 32)/1000.00);
      dataObject["temperature_c"] = (float)(signedInt(data, 13, 16)/100.00);
      rDevice = true;
      break;
    }
    case(21):{
      if(len<13){
        return false;
      }
      if(newDevice){
        json["Type"] = "AMS Pressure & Temperature";
        json["SKU"] = "";
      }
      dataObject["pressure_psi"] = (float)(signedInt(data, 9, 16)/100.000);
      dataObject["temperature_c"] = (float)(signedInt(data, 11, 16)/100.00);
      rDevice = true;
      break;
    }
    case(22):{
      if(len<10){
        return false;
      }
      if(newDevice){
        json["Type"] = "Voltage Detection Input";
        json["SKU"] = "";
      }
      dataObject["Input"] = data[9];
      rDevice = true;
      break;
    }
    case(23):{
      if(len<17){
        return false;
      }
      if(newDevice){
        json["Type"] = "2 Channel Thermocouple";
        json["SKU"] = "";
      }
      int32_t unconvertedOne = ((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12]);
      dataObject["Channel_1"] = (float)(unconvertedOne/100.00);

      int32_t unconvertedTwo = ((data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16]);
      dataObject["Channel_2"] = (float)(unconvertedTwo/100.00);

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
    case(26):{
      // Serial.println("Pressure sensor");
      if(len < 15){
        return false;
      }
      if(newDevice){
        json["Type"] = "Pressure & Temperature Sensor(PSI)";
        json["SKU"] = "";
      }
      dataObject["Pressure_PSI"] = (float)(signedInt(data, 9, 32)/100.00);
      dataObject["Temperature_Celsius"] = (float)(signedInt(data, 13, 16)/100.00);
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
        dataObject["temperature"] = (float)signedInt(data, 9, 16) / 100.00;
        dataObject["pressure"] = (float)((data[11]<<24)+(data[12]<<16)+(data[13]<<8)+data[14]) / 100.00;
        dataObject["humidity"] = (float)((data[15]<<24)+(data[16]<<16)+(data[17]<<8)+data[18]) / 1000.00;
        dataObject["gas_resistance"] = (float)((data[19]<<24)+(data[20]<<16)+(data[21]<<8)+data[22]);
        dataObject["iaq"] = (float)(data[23]<<8)+data[24];
        rDevice = true;
        break;
    }
    case(28):{
      if(len < 18){
        return false;
      }
      if(newDevice){
        json["Type"] = "3 channel 24-bit Current Monitor";
        json["SKU"] = "";
      }
      dataObject["channel_1_milliamps"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11]));
      dataObject["channel_2_milliamps"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15]));
      dataObject["channel_3_milliamps"] = (float)(((data[17]<<16)+(data[18]<<8)+data[19]));
      dataObject["channel_1_amps"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11])/1000);
      dataObject["channel_2_amps"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15])/1000);
      dataObject["channel_3_amps"] = (float)(((data[17]<<16)+(data[18]<<8)+data[19])/1000);
      rDevice = true;
      break;
    }
    case(29):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "Linear Displacement Sensor";
        json["SKU"] = "";
      }
      uint16_t raw = ((data[9]<<8)+data[10]);
      dataObject["position"] = (float)(raw/1023.00)*100.00;
      rDevice = true;
      break;
    }
    case(30):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "Structural Monitoring Sensor";
        json["SKU"] = "";
      }
      uint16_t raw = ((data[9]<<8)+data[10]);
      dataObject["position"] = (float)(raw/1023.00)*100.00;
      rDevice = true;
      break;
    }
    case(31):{
      if(len < 21){
        Serial.printf("Packet length for TVOC sensor was: %i\n",len);
        return false;
      }
      if(newDevice){
        json["Type"] = "TVOC CO2eq Temperature Humidity";
        json["SKU"] = "";
      }
      uint16_t rawHumidity = ((data[9]<<8)+data[10]);
      dataObject["humidity"] = (float)rawHumidity/100.00;

      dataObject["temperature"] = (float)(signedInt(data, 11, 16) / 100.00);
      dataObject["co2eq"] = signedInt(data, 13, 16);
      dataObject["tvoc"] = signedInt(data, 15, 16);
      dataObject["raw_h2"] = signedInt(data, 17, 16);
      dataObject["raw_ethanol"] = signedInt(data, 19, 16);
      rDevice = true;
      break;
    }
    case(32):{
      Serial.println("Particulate matter sensor transmission, sensor type 32");
      Serial.printf("Length of data:%i\n",len);
      Serial.print("Received data: ");
      for(int i = 9; i < len; i++){
        Serial.printf("%02X ",data[i]);
      }
      Serial.println();
      if(len < 53){
        return false;
      }
      dataObject["mass_concentration_pm_1_0"] = float(((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12])/100.00);
      dataObject["mass_concentration_pm_2_5"] = float(((data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16])/100.00);
      dataObject["mass_concentration_pm_4_0"] = float(((data[17]<<24)+(data[18]<<16)+(data[19]<<8)+data[20])/100.00);
      dataObject["mass_concentration_pm_10_0"] = float(((data[21]<<24)+(data[22]<<16)+(data[23]<<8)+data[24])/100.00);
      dataObject["number_concentration_pm_0_5"] = float(((data[25]<<24)+(data[26]<<16)+(data[27]<<8)+data[28])/100.00);
      dataObject["number_concentration_pm_1_0"] = float(((data[29]<<24)+(data[30]<<16)+(data[31]<<8)+data[32])/100.00);
      dataObject["number_concentration_pm_2_5"] = float(((data[33]<<24)+(data[34]<<16)+(data[35]<<8)+data[36])/100.00);
      dataObject["number_concentration_pm_4_0"] = float(((data[37]<<24)+(data[38]<<16)+(data[39]<<8)+data[40])/100.00);
      dataObject["number_concentration_pm_10_0"] = float(((data[41]<<24)+(data[42]<<16)+(data[43]<<8)+data[44])/100.00);
      dataObject["typical_particle_size"] = float(((data[45]<<24)+(data[46]<<16)+(data[47]<<8)+data[48])/100.0);
      dataObject["humidity"] = float(((data[49]*256)+data[50])/100.00);
      dataObject["temperature"] = (float)(signedInt(data, 51, 16) / 100.00);
      rDevice = true;
      break;
    }
    case(33):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "Current Detection Sensor";
        json["SKU"] = "";
      }
      //2 channel Push Notification
      dataObject["current_detected"] = data[9];
      rDevice = true;
      break;
    }
    case(34):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "Tank Level Sensor";
        json["SKU"] = "";
      }
      dataObject["level"] = (data[9]<<8)+data[10];
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
      dataObject["count"] = (data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12];
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
    case(39):{
      Serial.println("Transmission from RTD");
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "RTD Temperature Sensor";
        json["SKU"] = "PR55-27";
      }
      dataObject["temperature_celsius"] = (float)(signedInt(data, 9, 32)/100.00);
      rDevice = true;
      break;
    }
    case(40):{
      if(len < 38){
        return false;
      }
      if(newDevice){
        json["Type"] = "Vibration V2 MEMS";
        json["SKU"] = "";
      }
      if(reserveByte == 3){
        dataObject["sensor_id"] = 2;
      }else{
        dataObject["sensor_id"] = 1;
      }
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
    case(41):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "RPM Proximity Sensor";
        json["SKU"] = "";
      }
      dataObject["Base"] = (int16_t)((data[9]<<8)+data[10]);
      dataObject["RPM"] = (int16_t)((data[11]<<8)+data[12]);
      rDevice = true;
      break;
    }
    case(42):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "0-24 Voltage Monitor";
        json["SKU"] = "";
      }
      dataObject["Voltage"] = (float)(((int16_t)((data[9]<<8)+data[10]))* 0.03824);
      dataObject["ADC"] = (int16_t)((data[9]<<8)+data[10]);
      rDevice = true;
      break;
    }
    case(43):{
      if(len < 19){
        Serial.println("Sensor packet too short");
        return false;
      }
      if(newDevice){
        json["Type"] = "Dual Temperature Humidity Current Detect Sensor";
        json["SKU"] = "";
      }
      dataObject["current_detection"] = (int)data[9];
      dataObject["humidity_one"] = (float)((((data[11]) * 256) + data[12]) /100.0);
      dataObject["temperature_one"] = (float)(signedInt(data, 13, 16)/100.00);
      dataObject["humidity_two"] = (float)((((data[15]) * 256) + data[16]) /100.0);
      dataObject["temperature_two"] = (float)(signedInt(data, 17, 16)/100.00);
      rDevice = true;
      break;
    }
    case(44):{
      if(len < 17){
        Serial.println("Sensor packet too short");
        return false;
      }
      if(newDevice){
        json["Type"] = "CO2 Temperature Humidity Sensor";
        json["SKU"] = "PR52-33Q";
      }
      dataObject["co2"] = (float)((((data[9])<<24) + (data[10]<<16)+(data[11]<<8) + data[12])  /100.0);
      dataObject["humidity"] = (float)(((data[13])<<8)| data[14]) /100.0;
      dataObject["temperature"] = (float)(((data[15])<<8)| data[16]) /100.0;
      rDevice = true;
      break;
    }
    case(45):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-20mA Current Receiver Industrial";
        json["SKU"] = "";
      }
      int rawADC = (data[9]<<8)+data[10];
      Serial.printf("Raw ADC: %i\n", rawADC);
      float mA = (float)(rawADC * 0.0006934);
      dataObject["mA"] = mA;
      dataObject["raw_adc"] = rawADC;
      Serial.printf("mA: %0.2f\n", mA);
      rDevice = true;
      break;
    }
    case(46):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "PIR Motion Detector";
        json["SKU"] = "";
      }
      //Motion Detector
      dataObject["motion"] = data[9];
      rDevice = true;
      break;
    }
    case(47):{
      if(len < 11){
        return false;
      }
      if(newDevice){
        json["Type"] = "Wireless Tilt Sensor";
        json["SKU"] = "";
      }
      //2 channel Push Notification
      dataObject["roll"] = (float)(signedInt(data, 9, 16))/100.00;
      dataObject["pitch"] = (float)(signedInt(data, 11, 16))/100.00;
      rDevice = true;
      break;
    }
    case(48):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-20mA Current Loop Receiver Industrial";
        json["SKU"] = "";
      }
      int rawADC = (data[9]<<8)+data[10];
      Serial.printf("Raw ADC: %i\n", rawADC);
      float mA = (float)(rawADC * 0.00063168);
      dataObject["mA"] = mA;
      dataObject["raw_adc"] = rawADC;
      Serial.printf("mA: %0.2f\n", mA);
      rDevice = true;
      break;
    }
    case(50):{
      if(len < 45){
        return false;
      }
      if(newDevice){
        json["Type"] = "Predictive Maintenance Sensor";
        json["SKU"] = "";
      }
      dataObject["rms_x"] = (float)(signedInt(data, 9, 24))/10.00;
      dataObject["rms_y"] = (float)(signedInt(data, 12, 24))/10.00;
      dataObject["rms_z"] = (float)(signedInt(data, 15, 24))/10.00;
      dataObject["max_x"] = (float)(signedInt(data, 18, 24))/10.00;
      dataObject["max_y"] = (float)(signedInt(data, 21, 24))/10.00;
      dataObject["max_z"] = (float)(signedInt(data, 24, 24))/10.00;
      dataObject["min_x"] = (float)(signedInt(data, 27, 24))/10.00;
      dataObject["min_y"] = (float)(signedInt(data, 30, 24))/10.00;
      dataObject["min_z"] = (float)(signedInt(data, 33, 24))/10.00;
      dataObject["Vibration_Celsius"] = (float)(signedInt(data, 36, 16));
      dataObject["Thermocouple_Celsius"] = (float)(signedInt(data,38,32))/100.00;
      dataObject["Current"] = (float)(signedInt(data, 42, 24))/1000.00;
      rDevice = true;
      break;
    }
    case(52):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "2 Channel 4-20mA Current Receiver Industrial";
        json["SKU"] = "";
      }
      int rawADC = (data[9]<<8)+data[10];
      Serial.printf("Raw ADC: %i\n", rawADC);
      float mA = (float)(rawADC * 0.0006863);
      dataObject["mA_1"] = mA;
      dataObject["raw_adc_1"] = rawADC;
      Serial.printf("mA: %0.2f\n", mA);

      int rawADC_2 = (data[11]<<8)+data[12];
      Serial.printf("Raw ADC: %i\n", rawADC_2);
      float mA_2 = (float)(rawADC_2 * 0.0006863);
      dataObject["mA_2"] = mA_2;
      dataObject["raw_adc_2"] = rawADC_2;
      Serial.printf("mA_2: %0.2f\n", mA_2);
      rDevice = true;
      break;
    }
    case(53):{
      if(len < 57){
        return false;
      }
      dataObject["mass_concentration_pm_1_0"] = float(((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12])/100.00);
      dataObject["mass_concentration_pm_2_5"] = float(((data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16])/100.00);
      dataObject["mass_concentration_pm_4_0"] = float(((data[17]<<24)+(data[18]<<16)+(data[19]<<8)+data[20])/100.00);
      dataObject["mass_concentration_pm_10_0"] = float(((data[21]<<24)+(data[22]<<16)+(data[23]<<8)+data[24])/100.00);
      dataObject["number_concentration_pm_0_5"] = float(((data[25]<<24)+(data[26]<<16)+(data[27]<<8)+data[28])/100.00);
      dataObject["number_concentration_pm_1_0"] = float(((data[29]<<24)+(data[30]<<16)+(data[31]<<8)+data[32])/100.00);
      dataObject["number_concentration_pm_2_5"] = float(((data[33]<<24)+(data[34]<<16)+(data[35]<<8)+data[36])/100.00);
      dataObject["number_concentration_pm_4_0"] = float(((data[37]<<24)+(data[38]<<16)+(data[39]<<8)+data[40])/100.00);
      dataObject["number_concentration_pm_10_0"] = float(((data[41]<<24)+(data[42]<<16)+(data[43]<<8)+data[44])/100.00);
      dataObject["typical_particle_size"] = float(((data[45]<<24)+(data[46]<<16)+(data[47]<<8)+data[48])/100.0);
      dataObject["humidity"] = float(((data[49]*256)+data[50])/100.00);
      dataObject["temperature"] = (float)(signedInt(data, 51, 16) / 100.00);
      dataObject["co2"] = (float)((((data[53])<<24) + (data[54]<<16)+(data[55]<<8) + data[56])  /100.0);
      rDevice = true;
      break;
    }
    case(57):{
      if(len<23){
        return false;
      }
      if(newDevice){
        json["Type"] = "Soil Sensor";
        json["SKU"] = "";
      }
      float soil_moisture =(((data[9] * 256) + data[10]));
      dataObject["soil_moisture"] = soil_moisture/10.0;
      float soil_temp =    float (((data[11] * 256) + data[12]));
      dataObject["soil_temp"] = soil_temp/10.0;
      dataObject["soil_EC"] = float(((data[13] * 256) + data[14]));
      dataObject["soil_N"] = float(((data[15] * 256) + data[16]));
      dataObject["soil_P"] = float(((data[17] * 256) + data[18]));
      dataObject["soil_K"] = float(((data[19] * 256) + data[20]));
      float soil_pH = float(((data[21] * 256) + data[22]));
      dataObject["soil_pH"] = soil_pH/100.0;
      rDevice = true;
      break;
    }
    case(58):{
      if(len<56){
        return false;
      }
      if(newDevice){
        json["Type"] = "Mulit Level Soil Moisture Temperature EC pH NPK Sensor";
        json["SKU"] = "";
      }
      float soil_moisture1 =(((data[9] * 256) + data[10]));
      dataObject["soil_moisture1"] = soil_moisture1/10.0;
      float soil_temp1 =    float (((data[11] * 256) + data[12]));
      dataObject["soil_temp1"] = soil_temp1/10.0;
      dataObject["soil_EC1"] = float(((data[13] * 256) + data[14]));
      dataObject["soil_N1"] = float(((data[15] * 256) + data[16]));
      dataObject["soil_P1"] = float(((data[17] * 256) + data[18]));
      dataObject["soil_K1"] = float(((data[19] * 256) + data[20]));
      dataObject["soil_sal1"] = float(((data[21] * 256) + data[22]));
      //soil_sal1 = soil_sal1/100.0;
      float soil_pH1 = float(((data[23] * 256) + data[24]));
      dataObject["soil_pH1"] = soil_pH1/100.0;

      float soil_moisture2 =(((data[25] * 256) + data[26]));
      dataObject["soil_moisture2"] = soil_moisture2/10.0;
      float soil_temp2 =    float (((data[27] * 256) + data[28]));
      dataObject["soil_temp2"] = soil_temp2/10.0;
      dataObject["soil_EC2"] = float(((data[29] * 256) + data[30]));
      dataObject["soil_N2"] = float(((data[31] * 256) + data[32]));
      dataObject["soil_P2"] = float(((data[33] * 256) + data[34]));
      dataObject["soil_K2"] = float(((data[35] * 256) + data[36]));
      dataObject["soil_sal2"] = float(((data[37] * 256) + data[38]));
      // soil_sal2 = soil_sal2/100.0;
      float soil_pH2 = float(((data[39] * 256) + data[40]));
      dataObject["soil_pH2"] = soil_pH2/100.0;

      float soil_moisture3 =(((data[41] * 256) + data[42]));
      dataObject["soil_moisture3"] = soil_moisture3/10.0;
      float soil_temp3 =    float (((data[43] * 256) + data[44]));
      dataObject["soil_temp3"] = soil_temp3/10.0;
      dataObject["soil_EC3"] = float(((data[45] * 256) + data[46]));
      dataObject["soil_N3"] = float(((data[47] * 256) + data[48]));
      dataObject["soil_P3"] = float(((data[49] * 256) + data[50]));
      dataObject["soil_K3"] = float(((data[51] * 256) + data[52]));
      dataObject["soil_sal3"] = float(((data[53] * 256) + data[54]));
      //soil_sal3 = soil_sal3/100.0;
      float soil_pH3 = float(((data[55] * 256) + data[56]));
      dataObject["soil_pH3"] = soil_pH3/100.0;
      rDevice = true;
      break;
    }

    case(61):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "pH Sensor";
        json["SKU"] = "";
      }
      dataObject["pH"] = float(((data[9]<<8)+data[10])/100.00);
      int16_t unconverted = (data[11]<<8)+data[12];
      dataObject["temperature"] = (float)(unconverted/100.00);
      rDevice = true;
      break;
    }
    case(60):{
      float pressure = float((((data[9])<<24) + (data[10] <<16)+ (data[11] <<8)+ data[12]) /1000.0);
      dataObject["pressure"]  = pressure;
      float pressure_inch = float(abs(pressure * 0.0040146));
      dataObject["pressure_inch"] = pressure_inch;
      float velocity = 4005 * sqrt(pressure_inch);
      dataObject["velocity"] = float(velocity * 0.00508);
      float cTemp = float((((data[13])<<8)| data[14]) /100.0);
      dataObject["cTemp"] = cTemp;
      dataObject["fTemp"] = float(cTemp * 1.8 + 32);
      dataObject["flow"] = float((((data[15])<<8)| data[16]) /1000.0);
      rDevice = true;
      break;
    }
    case(62):{
      if(len < 13){
        return false;
      }
      if(newDevice){
        json["Type"] = "ORP Sensor";
        json["SKU"] = "";
      }
      dataObject["orp"] = (float)(((data[9]) * 256) + data[10]);
      int16_t unconverted = (data[11]<<8)+data[12];
      dataObject["temperature"] = (float)(unconverted/100.00);
      rDevice = true;
      break;
    }

    case(63):{
      Serial.println("New OPT/pH sensor transmission");
      if(len < 17){
        Serial.printf("Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "ORP, pH, and Temperature Sensor";
        json["SKU"] = "";
      }
      float ORP = (((data[9]) * 256) + data[10]);
      int16_t cTempint_ORP = (((uint16_t)(data[11])<<8)| data[12]);
      float pH = ((((data[13]) * 256) + data[14]) /100.0);
      int16_t cTempint_pH = (((uint16_t)(data[15])<<8)| data[16]);

      float cTemp_ORP = (float)cTempint_ORP /100.0;
      float fTemp_ORP = cTemp_ORP * 1.8 + 32;
      float cTemp_pH = (float)cTempint_pH /100.0;
      float fTemp_pH = cTemp_pH * 1.8 + 32;

      dataObject["solution_orp"] = ORP;
      dataObject["temperature_orp"] = cTemp_ORP;
      dataObject["solution_ph"] = pH;
      dataObject["temperature_ph"] = cTemp_pH;

      rDevice = true;
      Serial.println("packet complete");
      break;
    }

    case(64):{
      if(len < 23){
        Serial.printf("Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "EC Salinity TDS and Temperature Sensor";
        json["SKU"] = "";
      }
      int32_t EC_int = (data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12];
      int32_t TDS_int = (data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16];
      int32_t Salinity_int = (data[17]<<24)+(data[18]<<16)+(data[19]<<8)+data[20];
      int16_t Temp_int = (data[21]<<8)+data[22];

      float EC = (float)EC_int/100.00;
      float TDS = (float)TDS_int/100.00;
      float Salinity = (float)Salinity_int/100.00;
      float Temp = (float)Temp_int/100.00;

      dataObject["EC"] = EC;
      dataObject["TDS"] = TDS;
      dataObject["Salinity"] = Salinity;
      dataObject["Temperature"] = Temp;

      rDevice = true;
      Serial.println("packet complete");
      break;
    }

    case(65):{
      if(len < 19){
        Serial.printf("Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "Dissolved Oxygen and Temperature Sensor";
        json["SKU"] = "";
      }
      int32_t DO_int = (data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12];
      int32_t DO_Sat_int = (data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16];
      int16_t Temp_int = (data[17]<<8)+data[18];

      float DO = (float)DO_int/100.00;
      float DO_Sat = (float)DO_Sat_int/100.00;
      float Temp = (float)Temp_int/100.00;

      dataObject["DO"] = DO;
      dataObject["DO_Saturation"] = DO_Sat;
      dataObject["Temperature"] = Temp;

      rDevice = true;
      Serial.println("packet complete");
      break;
    }

    case(66):{
      if(len < 33){
        Serial.printf("Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "EC Dissolved Oxygen and Temperature Sensor";
        json["SKU"] = "";
      }
      int32_t EC_int = (data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12];
      int32_t TDS_int = (data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16];
      int32_t Salinity_int = (data[17]<<24)+(data[18]<<16)+(data[19]<<8)+data[20];
      int16_t Temp_int = (data[21]<<8)+data[22];

      int32_t DO_int = (data[23]<<24)+(data[24]<<16)+(data[25]<<8)+data[26];
      int32_t DO_Sat_int = (data[27]<<24)+(data[28]<<16)+(data[29]<<8)+data[30];
      int16_t DO_Temp_int = (data[31]<<8)+data[32];

      float EC = (float)EC_int/100.00;
      float TDS = (float)TDS_int/100.00;
      float Salinity = (float)Salinity_int/100.00;
      float Temp = (float)Temp_int/100.00;
      float DO = (float)DO_int/100.00;
      float DO_Sat = (float)DO_Sat_int/100.00;
      float DO_Temp = (float)DO_Temp_int/100.00;

      dataObject["EC"] = EC;
      dataObject["TDS"] = TDS;
      dataObject["Salinity"] = Salinity;
      dataObject["Temperature"] = Temp;
      dataObject["DO"] = DO;
      dataObject["DO_Saturation"] = DO_Sat;
      dataObject["DO_Temperature"] = DO_Temp;

      rDevice = true;
      Serial.println("packet complete");
      break;
    }

    case(67):{
      Serial.println("PAR sensor");
      if(len < 13){
        Serial.printf("Packet length was %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "PAR Sensor";
        json["SKU"] = "";
      }
      //One Channel Counter
      dataObject["PAR"] = (float)((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12])/100.00;
      rDevice = true;
      break;
    }

    case(69):{
      if(len < 25){
        Serial.printf("Sensor type 69, Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "Soil Moisture Temperature and EC Sensor";
        json["SKU"] = "";
      }

      dataObject["moisture"] = (float)((data[9]<<24)+(data[10]<<16)+(data[11]<<8)+data[12])/100.00;
      dataObject["temperature_c"] = (float)((data[13]<<24)+(data[14]<<16)+(data[15]<<8)+data[16])/100.00;
      dataObject["ec"] = (float)((data[13]<<17)+(data[18]<<16)+(data[19]<<8)+data[20])/100.00;
      dataObject["salinity"] = (float)((data[21]<<17)+(data[22]<<16)+(data[23]<<8)+data[24])/100.00;

      rDevice = true;
      Serial.println("packet complete");
      break;
    }
    case(76):{
      Serial.println("Transmission from type 76 sensor");
      if(len < 11){
        Serial.printf("Type 76 Length too short, it was: %i\n", len);
        return false;
      }
      if(newDevice){
        json["Type"] = "Wireless CO Sensor";
        json["SKU"] = "";
      }
      if(firmware > 5){
        dataObject["CO_ppm"] = (float)(data[9]<<8+data[10])/100.00;
        // dataObject["CO_ppm"] = data[9]<<8+data[10];
      }else{
        if(len < 17){
          return false;
        }
        dataObject["adc"] = data[9]<<8+data[10];
        dataObject["mA"] = (float)(data[11]<<8+data[12])/100.00;
        dataObject["CO_ppm"] = (float)(signedInt(data, 13, 32)/100.00);
      }

      rDevice = true;
      Serial.println("packet complete");
      break;
    }
    case(80):{
      if(len < 55){
        return false;
      }
      int odr;
      dataObject["mode"] = data[9];
      switch(data[10]){
        case(6):{
          odr = 50;
          break;
        }
        case(7):{
          odr = 100;
          break;
        }
        case(8):{
          odr = 200;
          break;
        }
        case(9):{
          odr = 400;
          break;
        }
        case(10):{
          odr = 800;
          break;
        }
        case(11):{
          odr = 1600;
          break;
        }
        case(12):{
          odr = 3200;
          break;
        }
        case(13):{
          odr = 6400;
          break;
        }
        case(14):{
          odr = 128000;
          break;
        }
      }
      dataObject["odr"] = odr;
      dataObject["temperature"] = (float)(signedInt(data, 11, 16)/100.00);

      dataObject["x_rms_ACC_mg"] = (float)(signedInt(data, 13, 16)/1000.00);
      dataObject["x_max_ACC_mg"] = (float)(signedInt(data, 15, 16)/1000.00);
      dataObject["x_velocity_mm_sec"] = (float)(signedInt(data, 17, 16)/100.00);
      dataObject["x_displacement_mm"] = (float)(signedInt(data, 19, 16)/100.00);
      dataObject["x_peak_one_Hz"] = (int)(data[21]<<8+data[22]);
      dataObject["x_peak_two_Hz"] = (int)(data[23]<<8+data[24]);
      dataObject["x_peak_three_Hz"] = (int)(data[25]<<8+data[26]);

      dataObject["y_rms_ACC_mg"] = (float)(signedInt(data, 27, 16)/1000.00);
      dataObject["y_max_ACC_mg"] = (float)(signedInt(data, 29, 16)/1000.00);
      dataObject["y_velocity_mm_sec"] = (float)(signedInt(data, 31, 16)/100.00);
      dataObject["y_displacement_mm"] = (float)(signedInt(data, 33, 16)/100.00);
      dataObject["y_peak_one_Hz"] = (int)(data[35]<<8+data[36]);
      dataObject["y_peak_two_Hz"] = (int)(data[37]<<8+data[38]);
      dataObject["y_peak_three_Hz"] = (int)(data[39]<<8+data[40]);

      dataObject["z_rms_ACC_mg"] = (float)(signedInt(data, 41, 16)/1000.00);
      dataObject["z_max_ACC_mg"] = (float)(signedInt(data, 43, 16)/1000.00);
      dataObject["z_velocity_mm_sec"] = (float)(signedInt(data, 45, 16)/100.00);
      dataObject["z_displacement_mm"] = (float)(signedInt(data, 47, 16)/100.00);
      dataObject["z_peak_one_Hz"] = (int)(data[49]<<8+data[50]);
      dataObject["z_peak_two_Hz"] = (int)(data[51]<<8+data[52]);
      dataObject["z_peak_three_Hz"] = (int)(data[53]<<8+data[54]);
      rDevice = true;
      break;
    }

    case(81):{
      if(len < 100){
        Serial.println("transmission from 2 channel vibration sensor too short.");
        return false;
      }
      Serial.println("Message from 2 channel vibration sensor received");
      int odr;
      switch(data[10]){
        case(6):{
          odr = 50;
          break;
        }
        case(7):{
          odr = 100;
          break;
        }
        case(8):{
          odr = 200;
          break;
        }
        case(9):{
          odr = 400;
          break;
        }
        case(10):{
          odr = 800;
          break;
        }
        case(11):{
          odr = 1600;
          break;
        }
        case(12):{
          odr = 3200;
          break;
        }
        case(13):{
          odr = 6400;
          break;
        }
        case(14):{
          odr = 128000;
          break;
        }
      }
      dataObject["odr_1"] = odr;
      dataObject["temperature_1"] = (float)(signedInt(data, 11, 16)/100.00);

      dataObject["x_rms_ACC_mg_1"] = (float)(signedInt(data, 13, 16)/1000.00);
      dataObject["x_max_ACC_mg_1"] = (float)(signedInt(data, 15, 16)/1000.00);
      dataObject["x_velocity_mm_sec_1"] = (float)(signedInt(data, 17, 16)/100.00);
      dataObject["x_displacement_mm_1"] = (float)(signedInt(data, 19, 16)/100.00);
      dataObject["x_peak_one_Hz_1"] = (int)(data[21]<<8+data[22]);
      dataObject["x_peak_two_Hz_1"] = (int)(data[23]<<8+data[24]);
      dataObject["x_peak_three_Hz_1"] = (int)(data[25]<<8+data[26]);

      dataObject["y_rms_ACC_mg_1"] = (float)(signedInt(data, 27, 16)/1000.00);
      dataObject["y_max_ACC_mg_1"] = (float)(signedInt(data, 29, 16)/1000.00);
      dataObject["y_velocity_mm_sec_1"] = (float)(signedInt(data, 31, 16)/100.00);
      dataObject["y_displacement_mm_1"] = (float)(signedInt(data, 33, 16)/100.00);
      dataObject["y_peak_one_Hz_1"] = (int)(data[35]<<8+data[36]);
      dataObject["y_peak_two_Hz_1"] = (int)(data[37]<<8+data[38]);
      dataObject["y_peak_three_Hz_1"] = (int)(data[39]<<8+data[40]);

      dataObject["z_rms_ACC_mg_1"] = (float)(signedInt(data, 41, 16)/1000.00);
      dataObject["z_max_ACC_mg_1"] = (float)(signedInt(data, 43, 16)/1000.00);
      dataObject["z_velocity_mm_sec_1"] = (float)(signedInt(data, 45, 16)/100.00);
      dataObject["z_displacement_mm_1"] = (float)(signedInt(data, 47, 16)/100.00);
      dataObject["z_peak_one_Hz_1"] = (int)(data[49]<<8+data[50]);
      dataObject["z_peak_two_Hz_1"] = (int)(data[51]<<8+data[52]);
      dataObject["z_peak_three_Hz_1"] = (int)(data[53]<<8+data[54]);

      switch(data[55]){
        case(6):{
          odr = 50;
          break;
        }
        case(7):{
          odr = 100;
          break;
        }
        case(8):{
          odr = 200;
          break;
        }
        case(9):{
          odr = 400;
          break;
        }
        case(10):{
          odr = 800;
          break;
        }
        case(11):{
          odr = 1600;
          break;
        }
        case(12):{
          odr = 3200;
          break;
        }
        case(13):{
          odr = 6400;
          break;
        }
        case(14):{
          odr = 128000;
          break;
        }
      }
      dataObject["odr_2"] = odr;
      dataObject["temperature_2"] = (float)(signedInt(data, 56, 16)/100.00);

      dataObject["x_rms_ACC_mg_2"] = (float)(signedInt(data, 58, 16)/1000.00);
      dataObject["x_max_ACC_mg_2"] = (float)(signedInt(data, 60, 16)/1000.00);
      dataObject["x_velocity_mm_sec_2"] = (float)(signedInt(data, 62, 16)/100.00);
      dataObject["x_displacement_mm_2"] = (float)(signedInt(data, 64, 16)/100.00);
      dataObject["x_peak_one_Hz_2"] = (int)(data[66]<<8+data[67]);
      dataObject["x_peak_two_Hz_2"] = (int)(data[68]<<8+data[69]);
      dataObject["x_peak_three_Hz_2"] = (int)(data[70]<<8+data[71]);

      dataObject["y_rms_ACC_mg_2"] = (float)(signedInt(data, 72, 16)/1000.00);
      dataObject["y_max_ACC_mg_2"] = (float)(signedInt(data, 74, 16)/1000.00);
      dataObject["y_velocity_mm_sec_2"] = (float)(signedInt(data, 76, 16)/100.00);
      dataObject["y_displacement_mm_2"] = (float)(signedInt(data, 78, 16)/100.00);
      dataObject["y_peak_one_Hz_2"] = (int)(data[80]<<8+data[81]);
      dataObject["y_peak_two_Hz_2"] = (int)(data[82]<<8+data[83]);
      dataObject["y_peak_three_Hz_2"] = (int)(data[84]<<8+data[85]);

      dataObject["z_rms_ACC_mg_2"] = (float)(signedInt(data, 86, 16)/1000.00);
      dataObject["z_max_ACC_mg_2"] = (float)(signedInt(data, 88, 16)/1000.00);
      dataObject["z_velocity_mm_sec_2"] = (float)(signedInt(data, 90, 16)/100.00);
      dataObject["z_displacement_mm_2"] = (float)(signedInt(data, 92, 16)/100.00);
      dataObject["z_peak_one_Hz_2"] = (int)(data[94]<<8+data[95]);
      dataObject["z_peak_two_Hz_2"] = (int)(data[96]<<8+data[97]);
      dataObject["z_peak_three_Hz_2"] = (int)(data[98]<<8+data[99]);

      rDevice = true;
      break;
    }

    case(82):{
      if(len < 61){
        return false;
      }
      int odr;
      dataObject["mode"] = data[9];
      switch(data[10]){
        case(6):{
          odr = 50;
          break;
        }
        case(7):{
          odr = 100;
          break;
        }
        case(8):{
          odr = 200;
          break;
        }
        case(9):{
          odr = 400;
          break;
        }
        case(10):{
          odr = 800;
          break;
        }
        case(11):{
          odr = 1600;
          break;
        }
        case(12):{
          odr = 3200;
          break;
        }
        case(13):{
          odr = 6400;
          break;
        }
        case(14):{
          odr = 128000;
          break;
        }
      }
      dataObject["odr"] = odr;
      dataObject["temperature"] = (float)(signedInt(data, 11, 16)/100.00);

      dataObject["thermocouple_temperature"] = float(signedInt(data,13,24)/100.00);

      dataObject["current"] = float((data[16]<<16+data[17]<<8+data[18])/1000.00);

      dataObject["x_rms_ACC_mg"] = (float)(signedInt(data, 19, 16)/1000.00);
      dataObject["x_max_ACC_mg"] = (float)(signedInt(data, 21, 16)/1000.00);
      dataObject["x_velocity_mm_sec"] = (float)(signedInt(data, 23, 16)/100.00);
      dataObject["x_displacement_mm"] = (float)(signedInt(data, 25, 16)/100.00);
      dataObject["x_peak_one_Hz"] = (int)(data[27]<<8+data[28]);
      dataObject["x_peak_two_Hz"] = (int)(data[29]<<8+data[30]);
      dataObject["x_peak_three_Hz"] = (int)(data[31]<<8+data[32]);

      dataObject["y_rms_ACC_mg"] = (float)(signedInt(data, 33, 16)/1000.00);
      dataObject["y_max_ACC_mg"] = (float)(signedInt(data, 35, 16)/1000.00);
      dataObject["y_velocity_mm_sec"] = (float)(signedInt(data, 37, 16)/100.00);
      dataObject["y_displacement_mm"] = (float)(signedInt(data, 39, 16)/100.00);
      dataObject["y_peak_one_Hz"] = (int)(data[41]<<8+data[42]);
      dataObject["y_peak_two_Hz"] = (int)(data[43]<<8+data[44]);
      dataObject["y_peak_three_Hz"] = (int)(data[45]<<8+data[46]);

      dataObject["z_rms_ACC_mg"] = (float)(signedInt(data, 47, 16)/1000.00);
      dataObject["z_max_ACC_mg"] = (float)(signedInt(data, 49, 16)/1000.00);
      dataObject["z_velocity_mm_sec"] = (float)(signedInt(data, 51, 16)/100.00);
      dataObject["z_displacement_mm"] = (float)(signedInt(data, 53, 16)/100.00);
      dataObject["z_peak_one_Hz"] = (int)(data[55]<<8+data[56]);
      dataObject["z_peak_two_Hz"] = (int)(data[57]<<8+data[58]);
      dataObject["z_peak_three_Hz"] = (int)(data[59]<<8+data[60]);
      rDevice = true;
      break;
    }

    case(84):{
      if(len < 55){
        return false;
      }
      int odr;
      dataObject["mode"] = data[9];
      switch(data[10]){
        case(6):{
          odr = 50;
          break;
        }
        case(7):{
          odr = 100;
          break;
        }
        case(8):{
          odr = 200;
          break;
        }
        case(9):{
          odr = 400;
          break;
        }
        case(10):{
          odr = 800;
          break;
        }
        case(11):{
          odr = 1600;
          break;
        }
        case(12):{
          odr = 3200;
          break;
        }
        case(13):{
          odr = 6400;
          break;
        }
        case(14):{
          odr = 128000;
          break;
        }
      }
      dataObject["odr"] = odr;
      dataObject["temperature"] = (float)(signedInt(data, 11, 16)/100.00);

      dataObject["x_rms_ACC_mg"] = (float)(signedInt(data, 13, 16)/1000.00);
      dataObject["x_max_ACC_mg"] = (float)(signedInt(data, 15, 16)/1000.00);
      dataObject["x_velocity_mm_sec"] = (float)(signedInt(data, 17, 16)/100.00);
      dataObject["x_displacement_mm"] = (float)(signedInt(data, 19, 16)/100.00);
      dataObject["x_peak_one_Hz"] = (int)(data[21]<<8+data[22]);
      dataObject["x_peak_two_Hz"] = (int)(data[23]<<8+data[24]);
      dataObject["x_peak_three_Hz"] = (int)(data[25]<<8+data[26]);

      dataObject["y_rms_ACC_mg"] = (float)(signedInt(data, 27, 16)/1000.00);
      dataObject["y_max_ACC_mg"] = (float)(signedInt(data, 29, 16)/1000.00);
      dataObject["y_velocity_mm_sec"] = (float)(signedInt(data, 31, 16)/100.00);
      dataObject["y_displacement_mm"] = (float)(signedInt(data, 33, 16)/100.00);
      dataObject["y_peak_one_Hz"] = (int)(data[35]<<8+data[36]);
      dataObject["y_peak_two_Hz"] = (int)(data[37]<<8+data[38]);
      dataObject["y_peak_three_Hz"] = (int)(data[39]<<8+data[40]);

      dataObject["z_rms_ACC_mg"] = (float)(signedInt(data, 41, 16)/1000.00);
      dataObject["z_max_ACC_mg"] = (float)(signedInt(data, 43, 16)/1000.00);
      dataObject["z_velocity_mm_sec"] = (float)(signedInt(data, 45, 16)/100.00);
      dataObject["z_displacement_mm"] = (float)(signedInt(data, 47, 16)/100.00);
      dataObject["z_peak_one_Hz"] = (int)(data[49]<<8+data[50]);
      dataObject["z_peak_two_Hz"] = (int)(data[51]<<8+data[52]);
      dataObject["z_peak_three_Hz"] = (int)(data[53]<<8+data[54]);
      rDevice = true;
      break;
    }



    case(200):{
      if(len < 15){
        return false;
      }
      if(newDevice){
        json["Type"] = "4-20 mA Transmitter/Receiver";
        json["SKU"] = "";
      }
      dataObject["milliamp"] = (float)(((data[9]<<8)+data[10])/100.00);
      dataObject["raw_adc"] = (data[11]<<8)+data[12];
      dataObject["raw_dac"] = (data[13]<<8)+data[14];
      rDevice = true;
      break;
    }

    // case(502):{
    //   if(newDevice){
    //     json["Type"] = "Vibration/Gyro/Magneto/Temperature";
    //     json["SKU"] = "";
    //   }
    //   //Vibration
    //   dataObject["rms_x"] = (float)(signedInt(data, 9, 24)/100.00);
    //   dataObject["rms_y"] = (float)(signedInt(data, 12, 24)/100.00);
    //   dataObject["rms_z"] = (float)(signedInt(data, 15, 24)/100.00);
    //   dataObject["max_x"] = (float)(signedInt(data, 18, 24)/100.00);
    //   dataObject["max_y"] = (float)(signedInt(data, 21, 24)/100.00);
    //   dataObject["max_z"] = (float)(signedInt(data, 24, 24)/100.00);
    //   dataObject["min_x"] = (float)(signedInt(data, 27, 24)/100.00);
    //   dataObject["min_y"] = (float)(signedInt(data, 30, 24)/100.00);
    //   dataObject["min_z"] = (float)(signedInt(data, 33, 24)/100.00);
    //   dataObject["temperature"] = (int16_t)(data[36]<<8)+data[37];
    //   //Gyro/Magneto/Temperature
    //   dataObject["accel_x"] = (float)(signedInt(data, 38, 24)/100.00);
    //   dataObject["accel_y"] = (float)(signedInt(data, 41, 24)/100.00);
    //   dataObject["accel_z"] = (float)(signedInt(data, 44, 24)/100.00);
    //   dataObject["magneto_x"] = (float)(signedInt(data, 47, 24)/100.00);
    //   dataObject["magneto_y"] = (float)(signedInt(data, 50, 24)/100.00);
    //   dataObject["magneto_z"] = (float)(signedInt(data, 53, 24)/100.00);
    //   dataObject["gyro_x"] = (float)(signedInt(data, 56, 24)/100.00);
    //   dataObject["gyro_y"] = (float)(signedInt(data, 59, 24)/100.00);
    //   dataObject["gyro_z"] = (float)(signedInt(data, 62, 24)/100.00);
    //   dataObject["temperature"] = (int16_t)(data[65]<<8)+data[66];
    //   rDevice = true;
    //   break;
    // }
    case(502):{
      if(data[8] == 0xAA){
        JsonArray& dataArray = dataObject.createNestedArray("raw_data");
        Serial.print("Copied: ");
        for(int i = 9; i < len; i++){
          dataArray.add(data[i]);
        }
        rDevice = true;
        break;
      }

      Serial.println("503 Sensor");
      float cTemp = (float)((((data[9])<<8)| data[10]) /100.0);
      dataObject["temperature_c"]= cTemp;
      dataObject["temperature_f"] = (float)(cTemp * 1.8 + 32);
      dataObject["pressure"] = (float)((((data[11])<<24)+((data[12])<<16)+((data[13])<<8)+(data[14]))/100.00);
      dataObject["humidity"] = (float)((((data[15])<<24)+((data[16])<<16)+((data[17])<<8)+(data[18]))/1000.00);
      dataObject["gas_resistance"] = (float)((((data[19])<<24)+((data[20])<<16)+((data[21])<<8)+(data[22])));
      dataObject["iaq"] = (float)((((data[23])<<8)+(data[24])));
      dataObject["light"] = (float)((((data[25])<<8)+(data[26])));
      dataObject["sound"] = (float)((data[27]));
      rDevice = true;
      break;
    }
    case(510):{
      Serial.println("This is a 510 sensor");
      if(len < 11){
        Serial.printf("Packet too short, length is:%i\n",len);
        return false;
      }
      if(newDevice){
        json["Type"] = "4-20mA Current Receiver";
        json["SKU"] = "";
      }
      int rawADC = (data[9]<<8)+data[10];
      Serial.printf("Raw ADC: %i\n", rawADC);
      float mA = (float)(rawADC/100.00);
      char data[6];
      sprintf(data,"%0.2f",mA);
      dataObject["mA"] = data;
      Serial.printf("mA: %0.2f\n", mA);
      rDevice = true;
      break;
    }

    case(505):{
      if(len < 18){
        return false;
      }
      if(newDevice){
        json["Type"] = "Custom 1 Channel Current Monitor";
        json["SKU"] = "";
      }
      dataObject["rms_current"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11]));
      dataObject["max_current"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15]));
      dataObject["minimum_current"] = (float)(((data[17]<<16)+(data[18]<<8)+data[19]));
      // DynamicJsonBuffer jBuffer;
      // JsonObject& lastHeardObject = jBuffer.parseObject(lastHeardString);
      // String nodeKey = String(nodeID);
      // if(lastHeardObject.containsKey(nodeKey)){
      //   unsigned long last = lastHeardObject[nodeKey].as<unsigned long>();
      //   dataObject["timestamp"] = millis() - last;
      // }else{
      //   dataObject["timestamp"] = 0;
      // }
      // lastHeardObject[nodeKey] = millis();
      // lastHeardString = "";
      // lastHeardObject.printTo(lastHeardString);
      rDevice = true;
      break;
    }
    case(506):{
      if(len < 44){
        return false;
      }
      if(newDevice){
        json["Type"] = "Custom 3 Channel Current Monitor";
        json["SKU"] = "";
      }
      dataObject["rms_current_1"] = (float)(((data[9]<<16)+(data[10]<<8)+data[11]));
      dataObject["max_current_1"] = (float)(((data[13]<<16)+(data[14]<<8)+data[15]));
      dataObject["minimum_current_1"] = (float)(((data[17]<<16)+(data[18]<<8)+data[19]));

      dataObject["rms_current_2"] = (float)(((data[21]<<16)+(data[22]<<8)+data[23]));
      dataObject["max_current_2"] = (float)(((data[25]<<16)+(data[26]<<8)+data[27]));
      dataObject["minimum_current_2"] = (float)(((data[29]<<16)+(data[30]<<8)+data[31]));

      dataObject["rms_current_3"] = (float)(((data[33]<<16)+(data[34]<<8)+data[35]));
      dataObject["max_current_3"] = (float)(((data[37]<<16)+(data[38]<<8)+data[39]));
      dataObject["minimum_current_3"] = (float)(((data[41]<<16)+(data[42]<<8)+data[43]));

      // DynamicJsonBuffer jBuffer;
      // JsonObject& lastHeardObject = jBuffer.parseObject(lastHeardString);
      // String nodeKey = String(nodeID);
      // if(lastHeardObject.containsKey(nodeKey)){
      //   unsigned long last = lastHeardObject[nodeKey].as<unsigned long>();
      //   dataObject["timestamp"] = millis() - last;
      // }else{
      //   dataObject["timestamp"] = 0;
      // }
      // lastHeardObject[nodeKey] = millis();
      // lastHeardString = "";
      // lastHeardObject.printTo(lastHeardString);
      rDevice = true;
      break;
    }

    case(515):{

    }

    case(600):{
      //Gateway
      if(len < 9){
        return false;
      }
      if(newDevice){
        json["Type"] = "Gateway";
        json["SKU"] = "";
      }
      rDevice = true;
      break;
    }

    case(601):{
      //Repeater
      if(len < 9){
        return false;
      }
      if(newDevice){
        json["Type"] = "Repeater";
        json["SKU"] = "";
      }
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
    Serial.printf("This is an unknown device, type is %i\n", sensorType);
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

bool NCDWireless::newDevice(uint8_t* data, int len, JsonObject& json, bool attributesOnly){

  if(data[0] != 127){
    return false;
  }

  DynamicJsonBuffer jsonBufferOne;
  JsonObject& dataObject = jsonBufferOne.createObject();
  dataObject.createNestedObject("data");
  if(!parseData(data, len, dataObject, true)){
    return false;
  }
  Serial.print("dataObject: ");
  dataObject.printTo(Serial);
  Serial.println();

  Serial.print("JSON Object: ");
  json.printTo(Serial);
  Serial.println();

  if(attributesOnly){
    JsonArray& attributes = json["attributes"].as<JsonArray>();

    JsonObject& attributesObject = dataObject["data"];

    for(auto kvp : attributesObject){
      JsonObject& attributeKVPObject = attributes.createNestedObject();
      attributeKVPObject["name"] = kvp.key;
      attributeKVPObject["dataType"] = "number";
    }
    return true;
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

  JsonArray& attributes = json["attributes"].as<JsonArray>();

  JsonObject& attributesObject = dataObject["data"];
  json["description"] = String(attributesObject["type"].as<int>());
  for(auto kvp : attributesObject){
    if(sensorType == 32){
      if(memcmp(kvp.key, "device_type_id", sizeof(kvp.key) == 0)){
        JsonObject& attributeKVPObject = attributes.createNestedObject();
        attributeKVPObject["name"] = kvp.key;
        attributeKVPObject["dataType"] = "number";
        break;
      }
    }else{
      JsonObject& attributeKVPObject = attributes.createNestedObject();
      attributeKVPObject["name"] = kvp.key;
      attributeKVPObject["dataType"] = "number";
    }
  }
  if(sensorType == 32){
    Serial.println("New device packet built for type 32 sensor");
  }

  #ifdef DEBUG
  Serial.println("newDevice ran");
  #endif
  return true;
}
