#include <Wire.h>

byte user_input;
byte close_finger[4];
byte open_finger[4];
byte apply_break[4];
byte command[4];

int outCount = 4;

// the pboard addrs actually are 1,2,3,4,5 but we perfrom bitshift accrd. to manual and hence 2,4,6,8,10
byte addList[5] = {0x02, 0x04, 0x06, 0x08, 0x10}; 


void scan_i2c(void) {
  byte ct, res, devCt;
  devCt = res = 0;
  byte addList[72];
  for (ct = 1; ct < 0x48; ct++) {
    Wire.beginTransmission(ct);
    Wire.write(0x1);
    res = Wire.endTransmission();
    if (!res) {
      addList[devCt] = ct;
      ++devCt;
    }
  }
  Serial.print(devCt);
  for (ct = 0; ct < devCt; ct++) {
    Serial.print(addList[ct]);
  }
}

void send_cmmnd(byte command[4]) {
  Wire.beginTransmission(command[0] >> 1);
  Wire.write(&command[1], outCount - 1);
  Wire.endTransmission();
}

void set_pid_gain(byte add){
  byte kp[3] = {add, 0x81, 0x3F000000};
  send_cmmnd(kp);
  byte ki[3] = {add, 0x81, 0x3F000000};
  send_cmmnd(kp);
  byte kd[3] = {add, 0x81, 0x3F000000};
  send_cmmnd(kp);
  
  }

void setup(void) {
  // Serial port
  Serial.begin(115200);

  //i2c
  Wire.begin();
  
}


void loop() {

  if (Serial.available() > 0)
  {
    user_input = Serial.read();

    if (user_input == 0x71) { // send chr 'q' to close
      byte close_finger[4] = {addList[1], 0x0C, 0x80, 0x20};
      send_cmmnd(close_finger);
    }

    if (user_input == 0x77) { // send chr 'w' to break
      byte apply_break[4] = {addList[1], 0x0C, 0x03, 0x00};
      send_cmmnd(apply_break);
    }

    if (user_input == 0x65) { // send chr 'e' to open
      byte open_finger[4] = {addList[1], 0x0C, 0xC0, 0x20};
      send_cmmnd(open_finger);
    }

  }

}
