#include <BaroSensor.h>



void setup() {
  // put your setup code here, to run once:
  BaroSensor.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp;
  float pressure;
  if(!BaroSensor.getTempAndPressure(&temp, &pressure)){
  Serial.print("Error: ");
  Serial.println(BaroSensor.getError());
  }
  else {
  Serial.print("Temp: ");
  Serial.println(temp);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  }

}
