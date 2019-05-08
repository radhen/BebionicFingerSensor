/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////


#include <Wire.h>
#include <bluefruit.h>
BLEUart bleuart; // uart over ble
//#include "rp_testing.h"


//***** Simultaneous use of two i2c ports ********//
//***** https://github.com/espressif/arduino-esp32/issues/977 ********//
#define SDA_IN 8 // (22) these are pin numbers on the MDBT50Q chip on the Sparkfun Pro nrf52840 Mini
#define SCL_IN 11 // (52) -------------- " -------------------- " ------------------- " -------------
#define SDA_OUT 6 // -------------- " -------------------- " ------------------- " -------------
#define SCL_OUT 9 // -------------- " -------------------- " ------------------- " -------------
// TwoWire defination from the Wire_nrf52.cpp file
TwoWire I2C_in(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_IN, SCL_IN);
TwoWire I2C_out(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_OUT, SCL_OUT);



/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x2B  // MS5637_02BA03 I2C address is 0x76(118)
#define CMD_RESET 0x1E
#define VCNL4040_ADDR 0x3D //7-bit unshifted I2C address of VCNL4040
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ID  0x0C
#define I2C_FASTMODE 1

#define NUM_FINGERS 1 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

#define MUX0_ADDR 112

/***** USER PARAMETERS *****/
int i2c_ids_[2] = {112, 113}; //muxAddresses
//int sensor_ports[NUM_FINGERS] = {0, 2, 4, 6}; // Mux board ports for each Barometer sensor {0,2,4,6}

typedef struct {
  byte irPort;
  byte barPort;
} Digit;

//Each finger is a pair of ports (as read off of the mux board. Should all be between 0 and 15).
//first number is the ir port, second number is the pressure port (ie, barPort).
//Digit fingers[NUM_FINGERS] = {{6, 6},  //index finger
//  {4, 4},  //middle finger
//  {2, 2},  //ring finger
//  {0, 0},  //pinky finger
//  {8, 8}
//}; //thumb

int muxStatus;

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NUM_FINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
int32_t data[3];

volatile int32_t pressure_value_[NUM_FINGERS];
volatile uint16_t proximity_value_[NUM_FINGERS];

//int32_t max_pressure[NUM_FINGERS] = {7800000.0, 6115000.0, 5950000.0, 6653000.0, 7000000.0};
//uint16_t max_proximity[NUM_FINGERS] = {40000.0, 35000.0, 30000.0, 17000.0, 25000.0};

int timer1_counter;


///////////////////////////////////////////////////////////
//////////////// SIGENICS INIT CODE BELOW ///////////////////////
//////////////////////////////////////////////////////////

// 100ms timeout for one forward command
// command buffer 500 bytes

//FSM
#define NUM_PBOARDS 6

byte user_input;
byte close_finger[4];
byte open_finger[4];
byte apply_break[4];
byte command[4];
int outCount = 4;

byte pBoardAddresses[NUM_PBOARDS] = {1, 2, 3, 4, 5, 6};

// the pboard addrs actually are 1,2,3,4,5 but we perfrom bitshift accrd. to manual and hence 2,4,6,8,10
byte addrs[NUM_PBOARDS] = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C};
//byte addrs[NUM_PBOARDS] = {0x0A};

#define fNONE 0
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
//byte outCount = 0;
byte state = fNONE;
bool newCommand = false, toggle = false, debugFlag = true;
byte inByte;

volatile uint16_t encoder_value_;

typedef struct data_packet_struct {
  int irVals[NUM_FINGERS];
  float pressVals[NUM_FINGERS];
  int encoders[NUM_PBOARDS];
} DataPacket;
DataPacket packet;


///////////////////////////////////////////////////////////
///////////// MISC FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

void scan_i2c_in(){
Serial.println("Scanning I2C Addresses Channel 1");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  I2C_in.beginTransmission(i);
  uint8_t ec=I2C_in.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}

void scan_i2c_out(){
Serial.println("Scanning I2C Addresses Channel 2");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  I2C_out.beginTransmission(i);
  uint8_t ec=I2C_out.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}

///////////////////////////////////////////////////////////
///////////// PCF SENSOR FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

// Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(commandCode);
  int err = I2C_in.endTransmission(false); //Send a restart command. Do not release bus.
//  Serial.println(err);
  I2C_in.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = I2C_in.read();
  data |= I2C_in.read() << 8;

  return (data);
}


//void selectSensor(int muxID, int i) {
//  I2C_in.beginTransmission(muxID);
//  I2C_in.write(1 << i);
//  int errcode = I2C_in.endTransmission();
////  Serial.println(errcode);
//}

/*
   Each individual mux allows selection between 8 different channels, the 8 bits of a single byte being used to set the on/off status of a given channel.
   We are going to keep track of the status of the two muxes the same way, using the 16 bits of a two-byte int.

*/
void selectSensor(int port) {
  int muxStatusGoal = 1 << port;
  //  Serial.print("Port: "); Serial.print(port);
  if ( (muxStatus & 0xFF) != (muxStatusGoal & 0xFF) ) { //We only need update mux0 if the desired state for mux0 is not its current state.
    I2C_in.beginTransmission(MUX0_ADDR); //Talk to mux0.
    I2C_in.write( (byte)(muxStatusGoal & 0xFF)); //Update mux0 to desired status.
    int errcode = I2C_in.endTransmission(); //Release i2c line.
    //    Serial.print("112"); Serial.print('\t'); Serial.println(errcode);
  }
  if ( (muxStatus & 0xFF00) != (muxStatusGoal & 0xFF00) ) { //We only need update mux1 if the desired state for mux1 is not its current state.
    I2C_in.beginTransmission(MUX0_ADDR + 1); //Talk to mux1.
    I2C_in.write( (byte)((muxStatusGoal & 0xFF00) >> 8)); //Update mux1 to desired status.
    int errcode = I2C_in.endTransmission(); //Release i2c line.
    //    Serial.print("113"); Serial.print('\t'); Serial.println(errcode);
  }
  muxStatus = muxStatusGoal;
}


void writeByte(byte addr, byte val) {
  I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(addr);
  I2C_in.write(val);
  I2C_in.endTransmission(); //Release bus
}


void initPressure(int id) {
  byte dataLo, dataHi;

  //  selectSensor(fingers[id].barPort);
  I2C_in.beginTransmission(BARO_ADDRESS);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  for (int i = 0; i < 6; i++) { //loop over Coefficient elements
    I2C_in.beginTransmission(BARO_ADDRESS);
    I2C_in.write(0xA2 + (i << 1));
    I2C_in.endTransmission();
    //      Serial.print(err); Serial.print('\t');

    I2C_in.requestFrom(BARO_ADDRESS, 2); // Request 2 bytes of data

    if (I2C_in.available() == 2) {
      //        Serial.println("got data");
      dataHi = I2C_in.read();
      dataLo = I2C_in.read();
    }
    //      Coff[i][id] = ((dataHi << 8) | dataLo);
    Coff[i][id] = ((dataHi * 256) + dataLo);
//    Serial.print(Coff[i][id]); Serial.print('\t');
  }
//  Serial.print('\n');
  delay(300);
  I2C_in.beginTransmission(BARO_ADDRESS);
  I2C_in.write(byte(0));
  I2C_in.endTransmission();

}

int32_t getPressureReading(int id) {
  //  selectSensor(muxAddr, sensor);
  //  selectSensor(fingers[id].barPort);

  I2C_in.beginTransmission(BARO_ADDRESS);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  I2C_in.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  I2C_in.write(0x1E); // Send reset command
  I2C_in.endTransmission(); // Stop I2C Transmission

  I2C_in.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  I2C_in.write(0x40); // Refresh pressure with the OSR = 256
  I2C_in.endTransmission(); // Stop I2C Transmission

  delayMicroseconds(800);

  //  selectSensor(fingers[id].barPort);

  I2C_in.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  I2C_in.write(byte(0x00));  // Select data register
  I2C_in.endTransmission(); // Stop I2C Transmission

  I2C_in.requestFrom(BARO_ADDRESS, 3); // Request 3 bytes of data

  if (I2C_in.available() == 3)
  {
    data[0] = I2C_in.read();
    data[1] = I2C_in.read();
    data[2] = I2C_in.read();
  }

  return ((data[0] * 65536.0) + (data[1] * 256.0) + data[2]);
}


void readPressureValues() {
  int count = 0;

  for (int i = 0; i < NUM_FINGERS; i++) {
    I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();
    pressure_value_[count] = getPressureReading(i);
    Serial.print(pressure_value_[count]); Serial.print('\t');

    //I2C_out
    I2C_out.beginTransmission(8);
    I2C_out.write(pressure_value_[count]); I2C_out.write('\t');
    I2C_out.endTransmission();
    
    if(bleuart.available()){
      bleuart.write(pressure_value_[count]); bleuart.write('\t');
    }
    
    count += 1;
  }

}


void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(commandCode);
  I2C_in.write(lowVal); //Low byte of command
  I2C_in.write(highVal); //High byte of command
  I2C_in.endTransmission(); //Release bus
}


void initVCNL4040() {
  //Clear PS_SD to turn on proximity sensing
  //byte conf1 = 0b00000000; //Clear PS_SD bit to begin reading
  byte conf1 = 0b00001110; //Integrate 8T, Clear PS_SD bit to begin reading
  byte conf2 = 0b00001000; //Set PS to 16-bit
  //byte conf2 = 0b00000000; //Clear PS to 12-bit
  writeToCommandRegister(PS_CONF1, conf1, conf2); //Command register, low byte, high byte

  //Set the options for PS_CONF3 and PS_MS bytes
  byte conf3 = 0x00;
  //byte ms = 0b00000010; //Set IR LED current to 100mA
  //byte ms = 0b00000110; //Set IR LED current to 180mA
  byte ms = 0b00000111; //Set IR LED current to 200mA
  writeToCommandRegister(PS_CONF3, conf3, ms);
}


void initIRSensor(int id) {

  //  selectSensor(fingers[id].irPort);
  I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  int deviceID = readFromCommandRegister(ID);
//  Serial.println(deviceID);
  if (deviceID != 0x186)
  {
    Serial.println("Device not found. Check wiring.");
    Serial.print("Expected: 0x186. Heard: 0x");
    Serial.println(deviceID, HEX);
    while (1); //Freeze!
  }
//  Serial.println("VCNL4040 detected!");
  initVCNL4040(); //Configure sensor

  //    delay(50);
  I2C_in.beginTransmission(VCNL4040_ADDR);
  I2C_in.write(byte(0));
  I2C_in.endTransmission();
}


void readIRValues() {
  int count = 0;
  for (int i = 0; i < NUM_FINGERS; i++) {
    //    selectSensor(fingers[i].irPort);
    proximity_value_[count] = readFromCommandRegister(PS_DATA_L);
    //Serial out
    Serial.print(proximity_value_[count]); Serial.print('\t');

    //I2C out
    I2C_out.beginTransmission(8);
    I2C_out.write(proximity_value_[count]); I2C_out.write('\t');
    I2C_out.endTransmission();
    
    //bluetooth out
     if(bleuart.available()){
      bleuart.write(proximity_value_[count]); bleuart.write('\t');
    }
    
    count += 1;
  }
}


//void readNNpredictions() {
//  for (int i = 0; i < NUM_FINGERS; i++) {
//    float *raw_data;
//    float nn_output;
//    // volatile float predictions[1];
//    raw_data = (float*)malloc(2 * sizeof(float));
//    raw_data[0] = proximity_value_[i] / float(max_proximity[i]);
//    raw_data[1] = pressure_value_[i] / float(max_pressure[i]);
//    nn_output = nnpred(raw_data);
//    Serial.print(nn_output); Serial.print('\t');
//    free(raw_data);
//  }
//}


///////////////////////////////////////////////////////////
////////////// SIGENICS FUNCTIONS BELOW ///////////////////
//////////////////////////////////////////////////////////

void toggleLED(void) {
  toggle = !toggle;
  if (toggle) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
}


void scan_i2c(void) {
  byte ct, res, devCt;
  devCt = res = 0;
  byte addList[72];
  for (ct = 1; ct < 0x48; ct++) {
    I2C_in.beginTransmission(ct);
    I2C_in.write(0x1);
    res = I2C_in.endTransmission();
    if (!res) {
      addList[devCt] = ct;
      ++devCt;
    }
  }
  Serial.println(devCt);
  for (ct = 0; ct < devCt; ct++) {
    Serial.println(addList[ct]);
  }
  //  Serial.print('\t');
}


bool lookForData()
{
  if (Serial.available() > 0)
  {
    while (Serial.available()) {
      inByte = Serial.read();

      switch (state) {
        case fNONE:
          if (inByte == HDLC) {
            state = fSTART;
            outCount = 0;
          }
          else if (inByte == 0x2C) { //break HDLC in this way to scan I2C bus
            //and report addresses of present devices
            //comment out the line 'scan_i2c()' to disable
            //and return to pure HDLC
            outCount = 0;
            scan_i2c();
          }
          break;

        case fSTART:
          if (inByte == HDLC) {
            state = fNONE;
            newCommand = true;
          }
          else if (inByte == ESCAPE) {
            state = fESCAPE;
          }
          else {
            i2cBuf[outCount] = inByte;
            ++outCount;
            toggleLED();
          }
          break;

        case fESCAPE:
          if (inByte == 0x5E) {
            i2cBuf[outCount] = 0x7E;
            ++outCount;
          }
          else if (inByte == 0x5D) {
            i2cBuf[outCount] = 0x7D;
            ++outCount;
          }
          state = fSTART;
          break;
        default:
          break;
      }
    }
    //uncomment the line below to echo back to debug
    //    Serial.write(inByte); //echo back
    return true;  //got data
  }
  else
  {
    return false; //no data
  }
}


void obey() {
  byte inByte;
  if (i2cBuf[0] & 0x1) {
    //it's a read command
    I2C_in.requestFrom(i2cBuf[0] >> 1, outCount - 1);
    while (I2C_in.available()) {
      inByte = I2C_in.read();
      Serial.println(inByte);
    }
  }
  else {
    //it's a write command
    //      Serial.println(i2cBuf[1]);
    I2C_in.beginTransmission(i2cBuf[0] >> 1);
    I2C_in.write(&i2cBuf[1], outCount - 1);
    I2C_in.endTransmission();
    //    delay(100);
  }
}


unsigned int readEncoderValue(int pennyAddress) {
  I2C_in.requestFrom(pennyAddress, 3);
  int msByte = I2C_in.read();
  int lsByte = I2C_in.read();
  int pAddr = I2C_in.read();
  unsigned int value = lsByte | (msByte << 8);  //switched msByte and lsByte
  return value;
}

void readMotorEncodersValues() {
  int count = 0;
  for (int i = 0; i < NUM_PBOARDS; i++) { // loop over penny boards
    encoder_value_ = readEncoderValue(pBoardAddresses[i]); //read encoder return 2bytes
    Serial.print(encoder_value_); Serial.print('\t');
    //    packet.encoders[count] = encoder_value_;
    count += 1;
  }
}


void send_cmmnd(byte command[4]) {
  I2C_in.beginTransmission(command[0] >> 1);
  I2C_in.write(&command[1], outCount - 1);
  I2C_in.endTransmission();
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
//  I2C_in.begin();
  I2C_out.begin();

  //  I2C_in.setClock(100000);
  pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope
  newCommand = false;
  delay(1000);

  // Initialize Bluetooth:
//  Bluefruit.begin();
//  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
//  Bluefruit.setTxPower(4);
//  Bluefruit.setName("RoboticMaterials_Centerboard");
//  bleuart.begin();
//
//  // Start advertising device and bleuart services
//  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//  Bluefruit.Advertising.addTxPower();
//  Bluefruit.Advertising.addService(bleuart);
//  Bluefruit.ScanResponse.addName();
//
//  Bluefruit.Advertising.restartOnDisconnect(true);
//  // Set advertising interval (in unit of 0.625ms):
//  Bluefruit.Advertising.setInterval(32, 244);
//  // number of seconds in fast mode:
//  Bluefruit.Advertising.setFastTimeout(30);
//  Bluefruit.Advertising.start(0);  

//  initialize attached devices
//  for (int i = 0; i < NUM_FINGERS; i++)
//  {
//    initIRSensor(i);
//    initPressure(i);
//  }


  while (!Serial) 
    {
    }
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  I2C_out.begin();
  for (byte i = 8; i < 120; i++)
  {
    I2C_out.beginTransmission (i);
    if (I2C_out.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  muxStatus = 0;

}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

  //  digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope

//    lookForData();
//    if (newCommand == true) {
//      obey();
//      newCommand = false;
//    }


//  readIRValues(); //-> array of IR values (2 bytes per sensor)
//  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  //  readNNpredictions();
//    readMotorEncodersValues();

//  Serial.print('\n');



}
