#include <Wire.h>
#include <LIS3MDL.h>
#include <EEPROM.h>

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

char report[80];
bool done=false;

void setup()
{
  Serial.begin(19200);
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
}

void WriteToEEPROM()
{
  done = true;
  int eeAddress=0;
  EEPROM.put(eeAddress, running_min);
  eeAddress += sizeof(running_min);
  EEPROM.put(eeAddress, running_max);
  Serial.println("Values written to EEPROM!");
}


void loop()
{
  if(!done)
  {
    mag.read();
  
    running_min.x = min(running_min.x, mag.m.x);
    running_min.y = min(running_min.y, mag.m.y);
    running_min.z = min(running_min.z, mag.m.z);
  
    running_max.x = max(running_max.x, mag.m.x);
    running_max.y = max(running_max.y, mag.m.y);
    running_max.z = max(running_max.z, mag.m.z);
  
    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}   max: {%+6d, %+6d, %+6d}",
      running_min.x, running_min.y, running_min.z,
      running_max.x, running_max.y, running_max.z);
    Serial.println(report);
  }
  
  if(Serial.available())
  {
    if(Serial.read()=='W')
    {
      WriteToEEPROM();
    }
  }
  delay(100);
}
