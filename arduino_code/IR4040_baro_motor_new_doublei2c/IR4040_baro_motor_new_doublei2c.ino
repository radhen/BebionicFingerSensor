/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////


#include <Wire.h>
#include <bluefruit.h>
BLEUart bleuart; // uart over ble
//#include "rp_testing.h"


//***** Simultaneous use of two i2c ports ********//
//***** https://github.com/espressif/arduino-esp32/issues/977 ********//
#define SDA_1 8 // these are pin numbers on the MDBT50Q chip on the Sparkfun Pro nrf52840 Mini
#define SCL_1 11 // -------------- " -------------------- " ------------------- " -------------
#define SDA_2 6 // -------------- " -------------------- " ------------------- " -------------
#define SCL_2 9 // -------------- " -------------------- " ------------------- " -------------

#define OUTPUT_I2C_ADDRESS 0x08

#define I2C_OUT_ENABLED false
#define BLUETOOTH_ENABLED false

// TwoWire definition from the Wire_nrf52.cpp file
//TwoWire I2C_in(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SDA_1, SCL_1);
TwoWire I2C_in(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_1, SCL_1);
#if(I2C_OUT_ENABLED)
TwoWire I2C_out(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SDA_2, SCL_2);
//TwoWire I2C_out(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_2, SCL_2);
#endif

#define LED_PIN 7


/***** GLOBAL CONSTANTS *****/
#define CMD_RESET 0x1E
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ID  0x0C
#define I2C_FASTMODE 1

#define NUM_FINGERS 2 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

typedef struct {
  byte baroAddr;
  byte irAddr;
} Digit;

//Each finger is a pair of ports (as read off of the mux board. Should all be between 0 and 15).
//first number is the ir port, second number is the pressure port (ir, barPort).
Digit fingers[NUM_FINGERS] = {{0x63,0x75}, {0x0E,0x18}};
int muxStatus;

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NUM_FINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
int32_t data[3];

bool light_on;
unsigned long last_light_switch;
#define BLINKY_LIGHT_PERIOD_MS 500

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
  Serial.println ("I2C scanner. Scanning ...");
  byte device_count = 0;
  I2C_in.begin();
  for (byte i = 0; i < 128; i++){
    I2C_in.beginTransmission (i);
    if (I2C_in.endTransmission () == 0) {
      Serial.print ("Found address: 0x");
      if(i<16){
        Serial.print("0");
      }
      Serial.print (i, HEX);
      Serial.println ("");
      device_count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (device_count, DEC);
  Serial.println (" device(s).");
}

#if(I2C_OUT_ENABLED)
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
#endif

///////////////////////////////////////////////////////////
///////////// PCF SENSOR FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

void initPressure(int fingerID) {
  byte dataLo, dataHi;

  const byte baro_address = fingers[fingerID].baroAddr;
  
  I2C_in.beginTransmission(baro_address);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  for (int i = 0; i < 6; i++) { //loop over Coefficient elements
    I2C_in.beginTransmission(baro_address);
    I2C_in.write(0xA2 + (i << 1));
    I2C_in.endTransmission();
    //      Serial.print(err); Serial.print('\t');

    I2C_in.requestFrom(baro_address, 2); // Request 2 bytes of data

    if (I2C_in.available() == 2) {
      //        Serial.println("got data");
      dataHi = I2C_in.read();
      dataLo = I2C_in.read();
    }
    //      Coff[i][id] = ((dataHi << 8) | dataLo);
    Coff[i][fingerID] = ((dataHi * 256) + dataLo);
//    Serial.print(Coff[i][id]); Serial.print('\t');
  }
//  Serial.print('\n');
  delay(300);
  I2C_in.beginTransmission(baro_address);
  I2C_in.write(byte(0));
  I2C_in.endTransmission();

}

int32_t getPressureReading(byte address) {
  I2C_in.beginTransmission(address);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  I2C_in.beginTransmission(address); // Start I2C Transmission
  I2C_in.write(0x1E); // Send reset command
  I2C_in.endTransmission(); // Stop I2C Transmission

  I2C_in.beginTransmission(address); // Start I2C Transmission
  I2C_in.write(0x40); // Refresh pressure with the OSR = 256
  I2C_in.endTransmission(); // Stop I2C Transmission

  delayMicroseconds(800);

  I2C_in.beginTransmission(address); // Start I2C Transmission
  I2C_in.write(byte(0x00));  // Select data register
  I2C_in.endTransmission(); // Stop I2C Transmission

  I2C_in.requestFrom(address, 3); // Request 3 bytes of data

  if (I2C_in.available() == 3)
  {
    data[0] = I2C_in.read();
    data[1] = I2C_in.read();
    data[2] = I2C_in.read();
  }

  return ((data[0] * 65536.0) + (data[1] * 256.0) + data[2]);
}


void readPressureValues() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    const byte baro_address = fingers[i].baroAddr;
    pressure_value_[i] = getPressureReading(baro_address);
    Serial.print(pressure_value_[i]); Serial.print('\t');

//    #if(I2C_OUT_ENABLED)
//      I2C_out.beginTransmission(OUTPUT_I2C_ADDRESS);
//      I2C_out.write(pressure_value_[i]); I2C_out.write('\t');
//      I2C_out.endTransmission();
//    #endif
    

    #if(BLUETOOTH_ENABLED)
      if(bleuart.available()){
        bleuart.write(pressure_value_[i]); bleuart.write('\t');
      }
    #endif
  }

}

// Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte address, byte commandCode)
{
  I2C_in.beginTransmission(address);
  I2C_in.write(commandCode);
  int err = I2C_in.endTransmission(false); //Send a restart command. Do not release bus.
//  Serial.println(err);
  I2C_in.requestFrom(address, 2); //Command codes have two bytes stored in them

  unsigned int data = I2C_in.read();
  data |= I2C_in.read() << 8;

  return (data);
}

void writeToCommandRegister(byte address, byte commandCode, byte lowVal, byte highVal)
{
  I2C_in.beginTransmission(address);
  I2C_in.write(commandCode);
  I2C_in.write(lowVal); //Low byte of command
  I2C_in.write(highVal); //High byte of command
  I2C_in.endTransmission(); //Release bus
}


void initVCNL4040(byte address) {
  //Clear PS_SD to turn on proximity sensing
  //byte conf1 = 0b00000000; //Clear PS_SD bit to begin reading
  byte conf1 = 0b00001110; //Integrate 8T, Clear PS_SD bit to begin reading
  byte conf2 = 0b00001000; //Set PS to 16-bit
  //byte conf2 = 0b00000000; //Clear PS to 12-bit
  writeToCommandRegister(address, PS_CONF1, conf1, conf2); //Command register, low byte, high byte

  //Set the options for PS_CONF3 and PS_MS bytes
  byte conf3 = 0x00;
  //byte ms = 0b00000010; //Set IR LED current to 100mA
  //byte ms = 0b00000110; //Set IR LED current to 180mA
  byte ms = 0b00000111; //Set IR LED current to 200mA
  writeToCommandRegister(address, PS_CONF3, conf3, ms);
}


void initIRSensor(byte address) {

  //  selectSensor(fingers[id].irPort);
  I2C_in.beginTransmission(address);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  int deviceID = readFromCommandRegister(address, ID);
//  Serial.println(deviceID);
const int response_code = 0x186;
  if (deviceID != response_code)
  {
    Serial.println("Device not found. Check wiring.");
    Serial.print("Expected: 0x186. Heard: 0x");
    Serial.println(deviceID, HEX);
    while (1); //Freeze!
  }
//  Serial.println("VCNL4040 detected!");
  initVCNL4040(address); //Configure sensor

  //    delay(50);
  I2C_in.beginTransmission(address);
  I2C_in.write(byte(0));
  I2C_in.endTransmission();
}


void readIRValues() {
  int count = 0;
  for (int i = 0; i < NUM_FINGERS; i++) {
    const byte vcnl_address = fingers[i].irAddr;
    proximity_value_[i] = readFromCommandRegister(vcnl_address, PS_DATA_L);
    //Serial out
    Serial.print(proximity_value_[i]); Serial.print('\t');

    //I2C out
//    #if(I2C_OUT_ENABLED)
//      I2C_out.beginTransmission(OUTPUT_I2C_ADDRESS);
//      int num_sent = I2C_out.write(proximity_value_[i]); 
//      if(num_sent!=2){
//        Serial.print("I2C_out write failed! ");
//        Serial.println(num_sent);
//        while(true);
//      }      
//      I2C_out.write('\t');
//      I2C_out.endTransmission();
//    #endif
    
    //bluetooth out
    #if(BLUETOOTH_ENABLED)
      if(bleuart.available()){
        bleuart.write(proximity_value_[i]); bleuart.write('\t');
      }
    #endif

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


void scan_i2c_penny(void) {
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
            scan_i2c_penny();
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

#if(BLUETOOTH_ENABLED)
void bluetooth_init(){
  Initialize Bluetooth:
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("RoboticMaterials_Centerboard");
  bleuart.begin();

  // Start advertising device and bleuart services
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  // Set advertising interval (in unit of 0.625ms):
  Bluefruit.Advertising.setInterval(32, 244);
  // number of seconds in fast mode:
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);  
}
#endif

void toggle_light(){
      last_light_switch = millis();
    digitalWrite(LED_PIN, light_on ? LOW : HIGH);
    light_on = !light_on;
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(LED_PIN, OUTPUT);
  toggle_light();

  Serial.begin(115200);
  I2C_in.begin();
  #if(I2C_OUT_ENABLED)
    I2C_out.begin();
  #endif

  delay(1000);
  //while(!Serial);//Wait for usb serial to wake up.
  toggle_light();

  #if(BLUETOOTH_ENABLED)
    bluetooth_init();
  #endif

  delay(1000);
  Serial.println("Starting up...");

/*************************/
/*** SCAN I2c CHANNELS ***/
/*************************/
//  scan_i2c_in();
//  #if(I2C_OUT_ENABLED)
//  scan_i2c_out();
//  #endif

   //  I2C_in.setClock(100000);
  //pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope
  newCommand = false;

//  initialize attached devices
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    initIRSensor(fingers[i].irAddr);
    initPressure(i);
  }

  muxStatus = 0;

}

//void transmitData(byte address){
//      I2C_out.beginTransmission(address);
//  for(int i=0;i<NUM_FINGERS;i++){
//
//    I2C_out.write(proximity_value_[i]&0xFF);
//    I2C_out.write((proximity_value_[i]>>8)&0xFF);
//  }
//  for(int i=0;i<NUM_FINGERS;i++){
//    uint32_t tmp = pressure_value_[i];
//    I2C_out.write((tmp)&0xFF);
//    I2C_out.write((tmp>>8)&0xFF);
//    I2C_out.write((tmp>>16)&0xFF);
//    I2C_out.write((tmp>>24)&0xFF);
//  }
//  I2C_out.endTransmission();
//}

//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {
  if((millis()-last_light_switch)>BLINKY_LIGHT_PERIOD_MS){
    toggle_light();
  }
  
  //  digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope

//    lookForData();
//    if (newCommand == true) {
//      obey();
//      newCommand = false;
//    }

  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  
  //  readNNpredictions();
//    readMotorEncodersValues();

  Serial.print('\n');
//  transmitData(OUTPUT_I2C_ADDRESS);

}
