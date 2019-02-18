/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////

#include <Wire.h>

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040
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


/***** USER PARAMETERS *****/
int i2c_ids_[] = {113}; //muxAddresses
int sensor_ports[NUM_FINGERS] = {2,4}; // Mux board ports for each Barometer sensor {0,2,4,6}

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NUM_FINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
int32_t data[3];

volatile int32_t pressure_value_;
volatile uint16_t proximity_value_;



///////////////////////////////////////////////////////////
//////////////// SIGENICS INIT CODE BELOW ///////////////////////
//////////////////////////////////////////////////////////

// 100ms timeout for one forward command
// command buffer 500 bytes

//FSM
#define NUM_P_BOARDS 2
int pBoardAddresses[NUM_P_BOARDS] = {3, 4};

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
byte outCount = 0;
byte state = fNONE;
bool newCommand = false, toggle = false, debugFlag = true;
byte inByte;



///////////////////////////////////////////////////////////
///////////// PCF SENSOR FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

// Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.

  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  unsigned int data = Wire.read();
  data |= Wire.read() << 8;

  return (data);
}


void selectSensor(int muxID, int i) {
  Wire.beginTransmission(muxID);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void writeByte(byte addr, byte val) {
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission(); //Release bus
}


void initPressure(int muxAddr) {
  Wire.beginTransmission(muxAddr);
  Wire.write(0);
  int errcode = Wire.endTransmission();
  Serial.println(errcode);
  for (int i = 0; i < NUM_FINGERS; i++) {
    selectSensor(muxAddr, sensor_ports[i]);
    for (int j = 0; j < 6; j++) { //loop over Coefficient elements
      // Start I2C Transmission
      Wire.beginTransmission(BARO_ADDRESS);
      // Select data register
      Wire.write(0xA2 + (2 * j));
      // Stop I2C Transmission
      Wire.endTransmission();

      // Request 2 bytes of data
      Wire.requestFrom(BARO_ADDRESS, 2);

      // Read 2 bytes of data
      // Coff msb, Coff lsb
      if (Wire.available() == 2)
      {
        data[0] = Wire.read();
        data[1] = Wire.read();
      }
      Coff[j][i] = ((data[0] * 256) + data[1]);
      //            Serial.println(Coff[j][i]);
      delay(300);
    }
  }
  Wire.beginTransmission(muxAddr);
  Wire.write(0);
  Wire.endTransmission();
}


int32_t getPressureReading(int muxAddr, int sensor) {
  selectSensor(muxAddr, sensor);
  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Send reset command
  Wire.write(0x1E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Refresh pressure with the OSR = 256
  Wire.write(0x40);
  // Stop I2C Transmission
  Wire.endTransmission();
  delayMicroseconds(800);

  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Select data register
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 3 bytes of data
  Wire.requestFrom(BARO_ADDRESS, 3);

  // Read 3 bytes of data
  // ptemp_msb1, ptemp_msb, ptemp_lsb
  if (Wire.available() == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  return ((data[0] * 65536.0) + (data[1] * 256.0) + data[2]);
}


void readPressureValues() {
  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NUM_FINGERS; j++) {
      pressure_value_ = getPressureReading(i2c_ids_[i], sensor_ports[j]);
      Serial.print(pressure_value_); Serial.print('\t');
    }
  }
}


void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.write(lowVal); //Low byte of command
  Wire.write(highVal); //High byte of command
  Wire.endTransmission(); //Release bus
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
  Wire.beginTransmission(id);
  Wire.write(0);
  //  Serial.println("WIRE IN");
  int errcode = Wire.endTransmission();
  //  Serial.println(errcode);

  // initialize each IR sensor
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    // specify IR sensor
    selectSensor(id, sensor_ports[i]);
    int deviceID = readFromCommandRegister(ID);
    if (deviceID != 0x186)
    {
      Serial.println("Device not found. Check wiring.");
      Serial.print("Expected: 0x186. Heard: 0x");
      Serial.println(deviceID, HEX);
      while (1); //Freeze!
    }
    //      Serial.println("VCNL4040 detected!");
    initVCNL4040(); //Configure sensor
  }
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();
}


void readIRValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NUM_FINGERS; j++) {
      selectSensor(i2c_ids_[i], sensor_ports[j]);
      proximity_value_ = readFromCommandRegister(PS_DATA_L);
      Serial.print(proximity_value_); Serial.print('\t');

      //------- Touch detection -----/
      // Use highpass filter instead: https://playground.arduino.cc/Code/Filters
    }
  }
}



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
  Serial.print('\t');
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
    Wire.requestFrom(i2cBuf[0] >> 1, outCount - 1);
    while (Wire.available()) {
      inByte = Wire.read();
      Serial.println(inByte);
    }
  }
  else {
    //it's a write command
    //      Serial.println(i2cBuf[1]);
    Wire.beginTransmission(i2cBuf[0] >> 1);
    Wire.write(&i2cBuf[1], outCount - 1);
    Wire.endTransmission();
  }
}


unsigned int readEncoderValue(int pennyAddress) {
  Wire.requestFrom(pennyAddress, 3);
  int msByte = Wire.read();
  int lsByte = Wire.read();
  int pAddr = Wire.read();
  unsigned int value = lsByte | (msByte << 8);  //switched msByte and lsByte
  Serial.print(value); Serial.print('\t');
  return value;
}

void readMotorEncodersValues() {
  unsigned int values[NUM_P_BOARDS];
  for (int i = 0; i < NUM_P_BOARDS; i++) { // loop over penny boards
    values[i] = readEncoderValue(pBoardAddresses[i]); //read encoder return 2bytes
  }
  Serial.println();
}


//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  Wire.begin();
  pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope
  newCommand = false;
  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);

  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    initIRSensor(i2c_ids_[i]);
    initPressure(i2c_ids_[i]);
  }
}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

    digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope

    readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
    readIRValues(); //-> array of IR values (2 bytes per sensor)

    readMotorEncodersValues();

    lookForData();
    if (newCommand == true) {
      obey();
      newCommand = false;
    }
}