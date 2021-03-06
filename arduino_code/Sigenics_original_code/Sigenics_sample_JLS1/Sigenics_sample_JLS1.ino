// 100ms timeout for one forward command
// command buffer 500 bytes



#include <Wire.h>

//FSM
#define fNONE 0 //defines constant value to variable name
#define fSTART 1
#define fESCAPE 2
#define fEND 3

#define ESCAPE 0x7D
#define HDLC 0x7E
#define SCAN_I2C 0x2C  //break hdlc to tell the arduino to scan the i2c bus
//and report back any ACK'd addresses
//return format goes:

//|| # of addresses || device 1 address || device 2 address || ... || device x address ||


byte inBuf[16];
byte i2cBuf[16];
byte outCount=0;
byte state=fNONE;
bool newCommand=false, toggle=false;
byte inByte;

void setup(void){
  // Serial port
  Serial.begin(115200); 

  //i2c
  Wire.begin(); //initiates Wire library.  defines master/slave I2C protocol
  newCommand=false;
}
void toggleLED(void){    //void command declares function: toggleLED
  toggle = !toggle;
  if (toggle){
    digitalWrite(13,HIGH);
    delay(1000);
  }
  else{
    digitalWrite(13,LOW);
    delay(1000);
  }
}

void scan_i2c(void){  //void command declares function: scan_i2c
  byte ct, res, devCt;
  devCt=res=0;
  byte addList[72];
  for (ct=1;ct<0x48;ct++){
    Wire.beginTransmission(ct);
    Wire.write(0x1);
    res = Wire.endTransmission();
    if(!res){
      addList[devCt]=ct;
      ++devCt; 
    }
  }
  Serial.write(devCt);
  for(ct=0;ct<devCt;ct++){
    Serial.write(addList[ct]);
  }
}

bool lookForData()
{
  if (Serial.available() > 0)
  {
    while(Serial.available()){
      inByte = Serial.read();

      switch(state){
      case fNONE:
        if(inByte==HDLC){
          state=fSTART;
          outCount=0;
        }
        else if(inByte==0x2C){  //break HDLC in this way to scan I2C bus
          //and report addresses of present devices
          //comment out the line 'scan_i2c()' to disable
          //and return to pure HDLC
          outCount=0;
          scan_i2c();
        }
        break;

      case fSTART:
        if(inByte==HDLC){
          state=fNONE;
          newCommand=true;
        }
        else if(inByte==ESCAPE){
          state=fESCAPE;
        }
        else{
          i2cBuf[outCount]=inByte;
          ++outCount;
          toggleLED();
        }
        break;

      case fESCAPE:
        if(inByte==0x5E){
          i2cBuf[outCount]=0x7E;
          ++outCount;   
        }
        else if(inByte==0x5D){
          i2cBuf[outCount]=0x7D;
          ++outCount;
        }
        state=fSTART;
        break;

      default:

        break;

      }
    }
    //uncomment the line below to echo back to debug
    //Serial.write(incomingByte); //echo back      
    return true;  //got data
  }
  else
  {
    return false; //no data
  }
}



void obey(){
  byte inByte;
  if(i2cBuf[0]&0x1){
    //it's a read command
    Wire.requestFrom(i2cBuf[0]>>1,outCount-1);
    while(Wire.available()){
      inByte=Wire.read();
      Serial.write(inByte);
    }
  }
  else{
    //it's a write command
    Wire.beginTransmission(i2cBuf[0]>>1); //'>>' bitshift command
    Wire.write(&i2cBuf[1], outCount-1);
    Wire.endTransmission();
  }


}
void loop(){  // loop function HERE
  lookForData(); //calls 'lookForData' function here
  toggleLED();
  if(newCommand==true){
    obey();  //calls 'obey' function
    newCommand=false;
  }

}

