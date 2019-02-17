// CORRELL FINGERTIP CODE HERE //
////////////////////////////////
#include <Wire.h>
#include <HardwareSerial.h>

/***** GLOBAL CONSTANTS *****/

//Device Addresses
#define MUX0_ADDR 112
#define VCNL4010_ADDRESS 0x13
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)

//Control Registers for VCNL4010
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_MOD 0x8F  // proximity modulator timing

//Tweakable Constants
#define PRESS_MEAS_DELAY_MS 10 //duration of each pressure measurement is twice this.
#define NFINGERS 5 // number of fingers connected

typedef struct {
  byte irPort;
  byte barPort;
} Digit;

/***** USER PARAMETERS *****/
int ir_current_ = 2; // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 1; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz

//Each finger is a pair of ports (as read off of the mux board. Should all be between 0 and 15).
//first number is the ir port, second number is the pressure port (ie, barPort).
Digit fingers[NFINGERS] = {{6, 7},  //index finger
                           {4, 5},  //middle finger
                           {2, 3},  //ring finger
                           {0, 1},  //pinky finger
                           {8, 9}}; //thumb

/***** GLOBAL VARIABLES *****/
int muxStatus;
unsigned long Coff[6][NFINGERS];

bool touched;
float touch_baseline;
float force;

unsigned int proximity_value[NFINGERS]; // current proximity reading
unsigned int average_value[NFINGERS];   // low-pass filtered proximity reading
signed int  fa1[NFINGERS];              // FA-I value;
signed int fa1derivative[NFINGERS];     // Derivative of the FA-I value;
signed int fa1deriv_last[NFINGERS];     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 45;            // Sensitivity of touch/release detection, values closer to zero increase sensitivity
int touch_analysis = 0;

/// Force Data Output Parameters ///
///////////////////////////////////

const int analogOutPins_bar[NFINGERS] = {3,5,0,0,6}; // Analog output pins for barometers
int outputValue = 0;        // value output to the PWM (analog out)



///////////////////////////
///////////////////////////


// SIGENICS CODE HERE  //
////////////////////////

#define NUM_PENNY_BOARDS 5
int pennyBoardAddresses[NUM_PENNY_BOARDS] = {1, 2, 3, 4, 5};

byte pennyBoardFingerMapping[6] = {3, 2, 1, 0, 4, 5};

const int analogOutPins_enc[NUM_PENNY_BOARDS] = {0,0,10,9,11}; //MIDDLE, INDEX, THUMB


//FSM
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
typedef struct data_packet_struct {
  byte startFlag[3];
  int irVals[5];
  float pressVals[5];
  int encoders[6];
} DataPacket;
DataPacket packet;

byte inBuf[16];
byte i2cBuf[16];
byte outCount = 0;
byte state = fNONE;
bool newCommand = false, toggle = false;
byte inByte;

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// VOID SETUP BELOW ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
  // Serial port
  Serial.begin(57600);

  //i2c
  Wire.begin();
  Wire.setClock(100000);
  //Wire.setClock(400000);
  newCommand = false;

  // CORRELL FINGERTIP CODE HERE //
  ////////////////////////////////

  delay(1000);

  for (int i = 0; i < NFINGERS; i++) {
    initIRsensor(i); // initialize each IR sensor
    initPressure(i); //initialize each Barometer sensor
  }
  
  touched = false;
  delay(100);

  for (int i = 0; i < NFINGERS; i++) {
    proximity_value[i] = readProximity(); // build arrays for touch detection
    average_value[i] = proximity_value[i];
    fa1[i] = 0;
  }

  for(int i=0;i<3;i++){
    packet.startFlag[i]=0x7E;
  }
  for(int i=0;i<5;i++){
    packet.irVals[i] = 0;
    packet.pressVals[i] = 0;
  }
  for(int i=0;i<6;i++){
    packet.encoders[i] = 0;
  }
  muxStatus = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// VOID SETUP ABOVE///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// SIGENICS FUNCTIONS BELOW ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

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
  Serial.write(devCt);
  for (ct = 0; ct < devCt; ct++) {
    Serial.write(addList[ct]);
  }
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
    //Serial.write(incomingByte); //echo back
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
    //      Wire.requestFrom(i2cBuf[0]>>1,outCount-1); //original Sigenics read function
    //      while(Wire.available()){
    //       inByte=Wire.read();
    //       //Serial.write(inByte);
    //      }
  }
  else {
    //it's a write command
    Wire.beginTransmission(i2cBuf[0] >> 1);
    Wire.write(&i2cBuf[1], outCount - 1);
    Wire.endTransmission();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// SIGENICS FUNCTIONS ABOVE ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// IR FUNCTIONS BELOW ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void initIRsensor(int id)
{
  selectSensor(fingers[id].irPort);
  writeByte(AMBIENT_PARAMETER, 0x7F);
  writeByte(IR_CURRENT, ir_current_);
  writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay

  delay(50);
}

unsigned int readProximity() {

  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure
  unsigned long startTime = millis();
  while (!(readByte(0x80) & 0x20)) { // Wait for the proximity data ready bit to be set
    if ((millis() - startTime) > 20) {
      return 0;
    }
  }

  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

void readIRValues() {

  for (int i = 0; i < NFINGERS; i++) {
    selectSensor(fingers[i].irPort);
    unsigned int prox_value = readProximity();
    //Need to flip endianness -- LabView is expecting the other one.
    packet.irVals[i] = ((prox_value&0xFF)<<8) | ((prox_value>>8)&0xFF);
    //Serial.write(buf,2);


    //      //------- Touch detection -----//
    //    proximity_value[i] = prox_value;
    //    fa1deriv_last[i] = fa1derivative[i];
    //    fa1derivative[i] = (signed int) average_value[i] - proximity_value[i] - fa1[i];
    //    fa1[i] = (signed int) average_value[i] - proximity_value[i];
    //    //Serial.print(proximity_value[i]); Serial.print('\t'); //Serial.print(","); Serial.print(fa2derivative);
    //
    //    if (touch_analysis) {
    //      //        Serial.print(",");
    //      if ((fa1deriv_last[i] < -sensitivity && fa1derivative[i] > sensitivity) || (fa1deriv_last[i] > 50 && fa1derivative[i] < -50)) { // zero crossing detected
    //        // Serial.print(proximity_value); Serial.print(","); Serial.print(fa2); Serial.print(","); Serial.println(fa2derivative);
    //        if (fa1[i] < -sensitivity) // minimum
    //        {
    //          //Serial.print("don't touch me!");
    //          touched = true;
    //        }
    //        else if (fa1[i] > sensitivity) // maximum
    //        {
    ////          Serial.print("R");
    //        }
    //      }
    //    }

  }
}

void writeByte(byte address, byte data){
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

byte readByte(byte address) {
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);

  Wire.endTransmission();
  Wire.requestFrom(VCNL4010_ADDRESS, 1);
  unsigned long timeBefore = millis();
  while (!Wire.available()) {
    if ((millis() - timeBefore) > 20) {
      return 0;
    }
  }
  byte data = Wire.read();
  return data;
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// IR FUNCTIONS ABOVE///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// BAROMETER FUNCTIONS BELOW///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


void initPressure(int id) {  // initialize Barometers while looping over fingers
  byte dataLo, dataHi;
  
  selectSensor(fingers[id].barPort);
  
  for (int i = 0; i < 6; i++) { //loop over Barometer coefficient elements
    Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
    Wire.write(0xA2 + (2 * i));           // Select data register
    Wire.endTransmission();               // Stop I2C Transmission

    Wire.requestFrom(BARO_ADDRESS, 2);    // Request 2 bytes of data
    
    if (Wire.available() == 2){
      dataHi = Wire.read();
      dataLo = Wire.read();
    }

    // Convert the data
    Coff[i][id] = ((dataHi << 8) | dataLo);
  }
  delay(300);
}

void readPressureValues() {
  unsigned long rawPressures[NFINGERS];
  unsigned long tempTemperature;
  
  unsigned long requestStart = millis();
  for (int i = 0; i < NFINGERS; i++) {
    requestPressureReading(i);
  }
  while (millis() - requestStart < PRESS_MEAS_DELAY_MS) {
    delay(1);
  }
  
  requestStart = millis();
  for (int i = 0; i < NFINGERS; i++) {
    rawPressures[i] = fetchReadPressure(i);
    requestTempReading(i);
  }
  while (millis() - requestStart < PRESS_MEAS_DELAY_MS) {
    delay(1);
  }

  int output_range_min[NFINGERS] = {2310,2600,0,0,2400}; // watch serial port to change
  int output_range_max[NFINGERS] = {2335,4000,0,0,3800}; // watch serial port to change
  
  
  for (int i = 0; i < NFINGERS; i++) {
    tempTemperature = fetchReadTemp(i);
    float result = processPressure(i, rawPressures[i], tempTemperature);
    //useReadPressure(result);
    packet.pressVals[i] = result;

    // Write force data to analog output //
//    Serial.println(result);
    int outputValue = map(result, output_range_min[i], output_range_max[i], 0, 255); //Map to analog output range
    analogWrite(analogOutPins_bar[i],outputValue); // Write to analog output port
//    Serial.print("mbar = "); Serial.println(result);
//    Serial.print("output = "); Serial.println(outputValue);

    
  }
}

void requestPressureReading(int id) {
  selectSensor(fingers[id].barPort);
  
  Wire.beginTransmission(BARO_ADDRESS);  // Start I2C Transmission
  Wire.write(0x1E);  // Send reset command
  Wire.endTransmission();  // Stop I2C Transmission

  Wire.beginTransmission(BARO_ADDRESS);  // Start I2C Transmission
  Wire.write(0x40);  // Refresh pressure with the OSR = 256
  Wire.endTransmission();  // Stop I2C Transmission
}

void requestTempReading(int id) {
  selectSensor(fingers[id].barPort);

  Wire.beginTransmission(BARO_ADDRESS);  // Start I2C Transmission
  Wire.write(0x50);  // Refresh temperature with the OSR = 256
  Wire.endTransmission();  // Stop I2C Transmission
}

unsigned long fetchReadPressure(int id) {
  byte dataHi, dataMid, dataLo;
  
  selectSensor(fingers[id].barPort);
  
  Wire.beginTransmission(BARO_ADDRESS);  // Start I2C Transmission
  Wire.write(0x00);  // Select data register
  Wire.endTransmission();  // Stop I2C Transmission

  Wire.requestFrom(BARO_ADDRESS, 3);  // Request 3 bytes of data

  if (Wire.available() == 3){
    dataHi = Wire.read();
    dataMid = Wire.read();
    dataLo = Wire.read();
  }

  // Convert the data
  return ((unsigned long)dataHi) << 16 | ((unsigned long)dataMid) << 8 | ((unsigned long)dataLo);
}

unsigned long fetchReadTemp(int id) {
  byte dataHi, dataMid, dataLo;
  
  selectSensor(fingers[id].barPort);
  
  Wire.beginTransmission(BARO_ADDRESS);  // Start I2C Transmission
  Wire.write(0x00);// Select data register
  Wire.endTransmission();// Stop I2C Transmission

  Wire.requestFrom(BARO_ADDRESS, 3);// Request 3 bytes of data

  if (Wire.available() == 3){
    dataHi = Wire.read();
    dataMid = Wire.read();
    dataLo = Wire.read();
  }

  // Convert the data
  return ((unsigned long)dataHi) << 16 | ((unsigned long)dataMid) << 8 | ((unsigned long)dataLo);
}

float processPressure(int id, unsigned long pressure, unsigned long temperature) {
  unsigned long D1 = pressure;
  unsigned long D2 = temperature;

  unsigned long dT = D2 - (Coff[4][id] * 256);
  unsigned long TEMP = 2000 + dT * (Coff[5][id] / 8388608);
  unsigned long long OFF = Coff[1][id] * 131072 + (Coff[3][id] * dT) / 64;
  unsigned long long SENS = Coff[0][id] * 65536 + (Coff[2][id] * dT) / 128;
  unsigned long P = (((D1 * SENS) / 2097152) - OFF) / 32768.0;
  float mbar = P / 100.0;

  //    // CORRECT CALCULATION HERE USING RADHEN'S NEW CODE //
  //  /////////////////////////////////////////////////////
  //  signed long dT = D2 - (Coff[4][id] << 8);
  //  signed long TEMP = 20000 + (((unsigned long long)dT) * Coff[5][id]) >> 23;
  //  signed long long OFF = Coff[1][id] << 17;// + (Coff[3]*((unsigned long long)dT))>>6;
  //  signed long long SENS = Coff[0][id] << 16;// + (Coff[2]*((unsigned long long)dT))>>7;
  //  float P = ((D1 * (SENS >> 21) - OFF)) >> 15;
  //  float mbar = P / 100.0;
  return (mbar);
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// BAROMETER FUNCTIONS ABOVE///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GENERAL FUNCTIONS BELOW///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/*
 * Each individual mux allows selection between 8 different channels, the 8 bits of a single byte being used to set the on/off status of a given channel.
 * We are going to keep track of the status of the two muxes the same way, using the 16 bits of a two-byte int.
 *
 */
void selectSensor(int port) {
  int muxStatusGoal = 1<<port;
  if( (muxStatus&0xFF) != (muxStatusGoal&0xFF) ){ //We only need update mux0 if the desired state for mux0 is not its current state.
    Wire.beginTransmission(MUX0_ADDR); //Talk to mux0.
    Wire.write( (byte)(muxStatusGoal&0xFF)); //Update mux0 to desired status.
    Wire.endTransmission(); //Release i2c line.
  }
  if( (muxStatus&0xFF00) != (muxStatusGoal&0xFF00) ){ //We only need update mux1 if the desired state for mux1 is not its current state.
    Wire.beginTransmission(MUX0_ADDR+1); //Talk to mux1.
    Wire.write( (byte)((muxStatusGoal&0xFF00)>>8)); //Update mux1 to desired status.
    Wire.endTransmission(); //Release i2c line.
  }
  muxStatus = muxStatusGoal;
}
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GENERAL FUNCTIONS ABOVE///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ENCODER FUNCTIONS BELOW///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void readAllEncoders() {

int output_range_min[NFINGERS] = {55000,55000,55000,55000,55000}; // watch serial port to change
int output_range_max[NFINGERS] = {65535,65535,65535,65535,65535}; // watch serial port to change

  
  unsigned int values[NUM_PENNY_BOARDS];
  for (int i = 0; i < NUM_PENNY_BOARDS; i++) { // loop over penny boards
    values[i] = readEncoderValue(pennyBoardAddresses[i]); //read encoder return 2bytes
    int pennyBoardAddr = pennyBoardAddresses[i];
    int associatedFingerID = pennyBoardFingerMapping[pennyBoardAddr-1];
    packet.encoders[associatedFingerID] = values[i];

   // Write encoder data to analog output //
//    Serial.println(values[i]);
    int outputValue = map(values[i], 55000, 65535, 255, 0); //Map to analog output range
    analogWrite(analogOutPins_enc[i],outputValue); // Write to analog output port
//    Serial.print("enc = "); Serial.println(values[i]);
//    Serial.print("output = "); Serial.println(outputValue);


    
//    Serial.print(i); Serial.print(":_");Serial.println(values[i]);
//    byte buf[2];
//    byte a = values[i];
//    byte b = values[i] >> 8; // bitshift the encoder values variable into 2 bytes
//    buf [1] = a; buf[0] = b; // fill buffer with bytes
//    Serial.write(buf, 2); //write 2 byte encoder data to serial port
  }
}

unsigned int readEncoderValue(int pennyAddress) {
  Wire.requestFrom(pennyAddress, 3);
  int msByte = Wire.read();
  int lsByte = Wire.read();
  int pAddr = Wire.read();
  unsigned int value = lsByte | (msByte << 8);  //switched msByte and lsByte
  return value;
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ENCODER FUNCTIONS ABOVE///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// VOID LOOP BELOW //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  long time_start = millis();

  ///////////////////////
  // SIGENICS CODE HERE //
  ///////////////////////
  lookForData();
  if (newCommand == true) {
    obey();
    newCommand = false;
  }

  ///////////////////////
  // CORRELL CODE HERE //
  ///////////////////////
  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  readAllEncoders(); //-> array of Encoder Values (2 bytes per penny board)
  byte* packetBytes = (byte*)&packet;
  Serial.write(packetBytes, sizeof(DataPacket));
  
  delay(20);

}



///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// VOID LOOP ABOVE //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
