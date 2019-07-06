#include <Wire.h>
#include "BaroSensor.h"

#define BARO_ADDR 0x0C

void setup()
{
  Serial.begin(115200);
  const byte baro_address = BARO_ADDR;
  BaroSensor.begin(baro_address);
}

void loop()
{
  if(!BaroSensor.isOK()) {
    Serial.println("Sensor not Found/OK. Error: "); 
//    Serial.println(BaroSensor.getError());
    BaroSensor.begin(BARO_ADDR); // Try to reinitialise the sensor if we can
  }
  else {
//    Serial.println(BaroSensor.getTemperature());
    Serial.println(BaroSensor.getPressure(OSR_8192, BARO_ADDR));
  }
}
