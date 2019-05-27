// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

#define NUM_FINGERS 2


void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial for output
  Serial.println("slave arduino");
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  const int expected_packet_size = NUM_FINGERS*(sizeof(uint16_t)+sizeof(int32_t));
  if(Wire.available()!=expected_packet_size){
    Serial.print("Unexpected packet size? ");
    Serial.println(Wire.available());
    while(Wire.available()){
      char c = Wire.read();
      Serial.print(c,HEX);
      Serial.print(' ');
    }
    Serial.println();
    return;
  }
  for(int i=0;i<NUM_FINGERS;i++){
    if(Wire.available()<2){
      Serial.println("Early exit!");
      return;
    }
    uint16_t proximity_value = Wire.read();
    proximity_value |= Wire.read()<<8;
    Serial.print(proximity_value);
    Serial.print(" ");
  }
  for(int i=0;i<NUM_FINGERS;i++){
    if(Wire.available()<4){
      Serial.println("Early exit!");
      return;
    }
    typedef union dumb_union{
      uint32_t u;
      int32_t s;
    }dumb;

    dumb tmp;
    tmp.u = Wire.read();
    tmp.u |= ((uint32_t)Wire.read())<<8;
    tmp.u |= ((uint32_t)Wire.read())<<16;
    tmp.u |= ((uint32_t)Wire.read())<<24;
    int32_t pressure_value = tmp.s;
    Serial.print(pressure_value);
    Serial.print(" ");
  }
  Serial.println();
}
