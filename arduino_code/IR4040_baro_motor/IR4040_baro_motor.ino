/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////

#include <Wire.h>
#include "rp_testing.h"
//#include <Filters.h> // Library from arduino https://playground.arduino.cc/Code/Filters/ simple high/low pass filter
//#include <filters.h> // Library from a guy on git for butterworth high pass filter https://github.com/MartinBloedorn/libFilter


/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define CMD_RESET 0x1E
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

#define NUM_FINGERS 5 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

#define MUX0_ADDR 112


/***** USER PARAMETERS *****/
int i2c_ids_[2] = {112, 113}; //muxAddresses
int sensor_ports[NUM_FINGERS] = {0, 2, 4, 6}; // Mux board ports for each Barometer sensor {0,2,4,6}

typedef struct {
  byte irPort;
  byte barPort;
} Digit;

//Each finger is a pair of ports (as read off of the mux board. Should all be between 0 and 15).
//first number is the ir port, second number is the pressure port (ie, barPort).
Digit fingers[NUM_FINGERS] = {{0, 0},  //pinky finger
  {2, 2},  //ring finger
  {4, 4},  //middle finger
  {6, 6},  //index finger
  {8, 8}
};   //thumb


int muxStatus;

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NUM_FINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
int32_t data[3];

volatile int32_t pressure_value_[NUM_FINGERS];
volatile uint16_t proximity_value_[NUM_FINGERS];


////////////// Exponential Avg. variables for CONTACT detection/////////////////
// https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
volatile float highpass_proximity_value_[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};
volatile float EMA_a_ir[NUM_FINGERS] = {0.5, 0.5, 0.5, 0.5, 0.5};
volatile float EMA_S_ir[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};
float contact_threshold[NUM_FINGERS] = {0.22, 0.20, 0.23, 0.23, 0.3};
int contact_pwm[NUM_FINGERS] = {25, 21, 20, 20, 20};
bool contact_flag = false;
bool touch_flag[NUM_FINGERS] = {false, false, false, false, false};
int finger_num = 0;
///////////////////////////////////////////////////////////


///////////////// PID VARIABLES //////////////////////
volatile float prox_err[NUM_FINGERS];
volatile float prox_nrm[NUM_FINGERS];
volatile float pwm[NUM_FINGERS];
volatile float prev_prox_err[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};
volatile float diff_prox_err[NUM_FINGERS];
volatile float sum_prox_err[NUM_FINGERS];
volatile float press_err[NUM_FINGERS];
volatile float press_nrm[NUM_FINGERS];
volatile float prev_press_err[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};
volatile float diff_press_err[NUM_FINGERS];
volatile float sum_press_err[NUM_FINGERS];

float prox_target[NUM_FINGERS] = {0.5, 0.5, 0.5, 0.5, 0.5};
float kp_prox[NUM_FINGERS] = {60.0, 52.0, 40.0, 60.0, 120.0};
float kd_prox[NUM_FINGERS] = {2.0, 0.0, 3.0, 5.0, 0.0};
float ki_prox[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};

float press_target[NUM_FINGERS] = {0.05, 0.05, 0.05, 0.05, 0.05};
float kp_press[NUM_FINGERS] = {80.0, 80.0, 80.0, 600.0, 80.0};
float kd_press[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};
float ki_press[NUM_FINGERS] = {0.0, 0.0, 0.0, 0.0, 0.0};

bool break_flag = true;
bool min_flag_ir = true;
bool min_flag_baro =  true;
int drop_count_ir = 10;
int drop_count_baro = 10;
bool pid_flag = false;
//////////////////////////////////////////////////////


int32_t max_pressure[NUM_FINGERS] = {6653000.0, 5950000.0, 6115000.0, 7800000.0, 7000000.0};
volatile float min_pressure[NUM_FINGERS];
volatile uint16_t max_distance[NUM_FINGERS] = {10000.0, 20000.0, 20000.0, 20000.0, 11000.0};
volatile float min_distance[NUM_FINGERS];
uint16_t max_proximity[NUM_FINGERS] = {17000.0, 30000.0, 35000.0, 40000.0, 25000.0}; // for NN

int timer1_counter;


/////////// High Pass filter variable ////////////
//FilterOnePole highpassFilter(HIGHPASS, 200);
//RunningStatistics inputStats;

// Creating high-pass filter; maximum order is 2
const float cutoff_freq   = 37.0;   //Cutoff frequency in Hz
const float sampling_time = 0.006; //Sampling time in seconds.
//Filter fhp(cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);


///////////////////////////////////////////////////////////
//////////////// SIGENICS INIT CODE BELOW ///////////////////////
//////////////////////////////////////////////////////////

// 100ms timeout for one forward command
// command buffer 500 bytes

//FSM
#define NUM_PBOARDS 5

byte user_input;
byte close_finger[4];
byte open_finger[4];
byte apply_break[4];
byte command[4];
int outCount = 4;

byte pBoardAddresses[NUM_PBOARDS] = {1, 2, 3, 4, 5};

// the pboard addrs actually are 1,2,3,4,5 but we perfrom bitshift accrd. to manual and hence 2,4,6,8,10
byte addrs[NUM_PBOARDS] = {0x02, 0x04, 0x06, 0x08, 0x0A};
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


//void selectSensor(int muxID, int i) {
//  Wire.beginTransmission(muxID);
//  Wire.write(1 << i);
//  int errcode = Wire.endTransmission();
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
    Wire.beginTransmission(MUX0_ADDR); //Talk to mux0.
    Wire.write( (byte)(muxStatusGoal & 0xFF)); //Update mux0 to desired status.
    int errcode = Wire.endTransmission(); //Release i2c line.
    //    Serial.print("112"); Serial.print('\t'); Serial.println(errcode);
  }
  if ( (muxStatus & 0xFF00) != (muxStatusGoal & 0xFF00) ) { //We only need update mux1 if the desired state for mux1 is not its current state.
    Wire.beginTransmission(MUX0_ADDR + 1); //Talk to mux1.
    Wire.write( (byte)((muxStatusGoal & 0xFF00) >> 8)); //Update mux1 to desired status.
    int errcode = Wire.endTransmission(); //Release i2c line.
    //    Serial.print("113"); Serial.print('\t'); Serial.println(errcode);
  }
  muxStatus = muxStatusGoal;
}


void writeByte(byte addr, byte val) {
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission(); //Release bus
}


void initPressure(int id) {
  byte dataLo, dataHi;

  selectSensor(fingers[id].barPort);

  for (int i = 0; i < 6; i++) { //loop over Coefficient elements
    Wire.beginTransmission(BARO_ADDRESS);
    Wire.write(0xA2 + (i << 1));
    Wire.endTransmission();
    //      Serial.print(err); Serial.print('\t');

    Wire.requestFrom(BARO_ADDRESS, 2); // Request 2 bytes of data

    if (Wire.available() == 2) {
      //        Serial.println("got data");
      dataHi = Wire.read();
      dataLo = Wire.read();
    }
    //      Coff[i][id] = ((dataHi << 8) | dataLo);
    Coff[i][id] = ((dataHi * 256) + dataLo);
    //      Serial.print(Coff[i][id]); Serial.print('\t');
  }
  //    Serial.print('\n');
  delay(300);

}

int32_t getPressureReading(int id) {
  //  selectSensor(muxAddr, sensor);
  selectSensor(fingers[id].barPort);

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x1E); // Send reset command
  Wire.endTransmission(); // Stop I2C Transmission

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x40); // Refresh pressure with the OSR = 256
  Wire.endTransmission(); // Stop I2C Transmission

  delayMicroseconds(800);

  //  selectSensor(fingers[id].barPort);

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x00);  // Select data register
  Wire.endTransmission(); // Stop I2C Transmission

  Wire.requestFrom(BARO_ADDRESS, 3); // Request 3 bytes of data

  if (Wire.available() == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  return ((data[0] * 65536.0) + (data[1] * 256.0) + data[2]);
}


void readPressureValues() {

  if (drop_count_baro > 0) {
    // Drop first five values from all the sensors
    drop_count_baro -= 1;
    for (int i = 0; i < NUM_FINGERS; i++) {
      pressure_value_[i] = getPressureReading(i);
    }
  }
  for (int i = 0; i < NUM_FINGERS; i++) {
    pressure_value_[i] = getPressureReading(i);
    //        Serial.print(pressure_value_[count]); Serial.print('\t');

    //*********** NORMALIZE BARO SENSOR VALUES ************//
    // keep track of the running min values
    if (min_flag_baro == true) {
      min_pressure[i] = pressure_value_[i];
      //       Serial.print(min_pressure[i]); Serial.print('\t');
    }
    if (pressure_value_[i] < min_pressure[i]) {
      min_pressure[i] = pressure_value_[i];
    }
    press_nrm[i] = float(pressure_value_[i] - min_pressure[i]) / float(max_pressure[i] - min_pressure[i]);

//    Serial.print(press_nrm[i], 6); Serial.print('\t');


    //*********** PID POSITION CONTROL ************//
    press_err[i] = press_target[i] - press_nrm[i];
    diff_press_err[i] = prox_err[i] - prev_press_err[i];
    prev_press_err[i] = press_err[i];
    sum_press_err[i] += press_err[i];
    pwm[i] = press_err[i] * kp_press[i] + diff_press_err[i] * kd_press[i] + sum_press_err[i] * ki_press[i];
    //    Serial.print(pwm[i]); Serial.print('\t');


    //********* Single PWM calculation for open and close **********//
    //    if (pwm[3] > 0.0) {
    //      byte close_finger[4] = {addrs[3], 0x0C, 0x80, int(pwm[3])};
    //      send_cmmnd(close_finger);
    //    }
    //
    //    if (prox_err[3] < 0.0) {
    //      byte open_finger[4] = {addrs[3], 0x0C, 0xC0, abs(int(pwm[3]))};
    //      send_cmmnd(open_finger);
    //    }


  }

  min_flag_baro = false;

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

  selectSensor(fingers[id].irPort);
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

  //    delay(50);
}


void readIRValues() {

  if (drop_count_ir > 0 ) {
    // Drop first five values from all the sensors
    drop_count_ir -= 1;
    //    Serial.println("dropping values");
    for (int i = 0; i < NUM_FINGERS; i++) {
      selectSensor(fingers[i].irPort);
      proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
    }
  }

  else {
    float sum_pid_err = 0.0;
    for (int i = 0; i < NUM_FINGERS; i++) {
//      digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope
      selectSensor(fingers[i].irPort);
      proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
      //      Serial.print(proximity_value_[i]); Serial.print('\t');


      //********* high pass filter from guy from github **********//
      //          float sign_filt = fhp.filterIn(proximity_value_[i]);
      //          Serial.print(sign_filt); Serial.print('\t');


      //*********** NORMALIZE IR SENSOR VALUES ************//
      // keep track of the running min values
      if (min_flag_ir == true) {
        min_distance[i] = proximity_value_[i];
      }
      if (proximity_value_[i] < min_distance[i]) {
        min_distance[i] = proximity_value_[i];
      }
      prox_nrm[i] = float(proximity_value_[i] - min_distance[i]) / float(max_distance[i] - min_distance[i]);

//      Serial.print(prox_nrm[i]); Serial.print('\t');


      //******* high pass filter with arduino library ******//
      //      float highpass_ir = highpassFilter.input(prox_nrm[i]);
      //      Serial.print(highpass_ir); Serial.print('\t');

      EMA_S_ir[i] = (EMA_a_ir[i] * prox_nrm[i]) + ((1.0 - EMA_a_ir[i]) * EMA_S_ir[i]);
      highpass_proximity_value_[i] = prox_nrm[i] - EMA_S_ir[i];
      //      Serial.print(highpass_proximity_valu/e_[i], 6); Serial.print('\t');

//      if (contact_flag == true) {
//        //******** Exponential average for Contact detection. Losspass filter and then subtract the orig. singal ********//
//        //        EMA_S_ir[i] = (EMA_a_ir[i] * prox_nrm[i]) + ((1.0 - EMA_a_ir[i]) * EMA_S_ir[i]);
//        //        highpass_proximity_value_[i] = prox_nrm[i] - EMA_S_ir[i];
//        //        Serial.print(highpass_proximity_value_[i], 6); Serial.print('\t');
//
//        if (highpass_proximity_value_[finger_num] < contact_threshold[finger_num]) {
//          byte close_finger[4] = {addrs[finger_num], 0x0C, 0x80, contact_pwm[finger_num]};
//          send_cmmnd(close_finger);
//        }
//        else {
//          //          touch_flag[finger_num] = true;
//          byte break_finger[4] = {addrs[finger_num], 0x0C, 0x03, 0};
//          send_cmmnd(break_finger);
//          finger_num += 1;
//        }
//        if (finger_num == 4) {
//          contact_flag = false;
//        }
//      }


//      if (pid_flag == true) {
//        //*********** PID POSITION CONTROL ************//
//        prox_err[i] = prox_target[i] - prox_nrm[i];
//        diff_prox_err[i] = prox_err[i] - prev_prox_err[i];
//        prev_prox_err[i] = prox_err[i];
//        sum_prox_err[i] += prox_err[i];
//        pwm[i] = prox_err[i] * kp_prox[i] + diff_prox_err[i] * kd_prox[i] + sum_prox_err[i] * ki_prox[i];
//        //      Serial.print(prox_err[i]); Serial.print('\t');
//
//        //********* Single PWM calculation for open and close **********//
//        if (pwm[i] > 0.0) {
//          byte close_finger[4] = {addrs[i], 0x0C, 0x80, int(pwm[i])};
//          send_cmmnd(close_finger);
//        }
//        if (pwm[i] < 0.0) {
//          byte open_finger[4] = {addrs[i], 0x0C, 0xC0, abs(int(pwm[i]))};
//          send_cmmnd(open_finger);
//        }
//        sum_pid_err += prox_err[i];
//      }

    }

    min_flag_ir = false;

  }
}


void readNNpredictions() {
  float nn_output;
  unsigned long start, elapsed;
  for (int i = 0; i < NUM_FINGERS; i++) {
    float *raw_data;
    raw_data = (float*)malloc(2 * sizeof(float));
    raw_data[0] = float(prox_nrm[i]);
    raw_data[1] = float(press_nrm[i]);

    start = micros();
    nn_output = nnpred(raw_data);
    elapsed = micros() - start;

    //    Serial.print("nn_out/put: ");
//    Serial.print(nn_output); Serial.print('\t');
    //    Serial.print(", elapsed [ms]: ");
        Serial.println(elapsed/1000.0f);
//    digitalWrite(13, !digitalRead(13));
  }
  //  Serial.println("here");
  
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
    //    delay(100);
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
  Wire.beginTransmission(command[0] >> 1);
  Wire.write(&command[1], outCount - 1);
  Wire.endTransmission();
}

//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(100000);
  //  Wire.setClock( 400000L);
  // change the clock rate (behind Wires' back)
  // TWBR = ((16000000 / 400000L) - 16) / 2;
  pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope
  newCommand = false;
  delay(1000);

  //initialize attached devices
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    initIRSensor(i);
    initPressure(i);
  }

  //  for (int i = 0; i < NUM_FINGERS; i++) {
  //    packet.irVals[i] = 0;
  //    packet.pressVals[i] = 0;
  //  }
  //  for (int i = 0; i < NUM_PBOARDS; i++) {
  //    packet.encoders[i] = 0;
  //  }

  for (int i = 0; i < NUM_PBOARDS; i++) {
    byte zero_encoder[4] = {addrs[i], 0x08, 0xFF, 0xFF};
    send_cmmnd(zero_encoder);
  }

  muxStatus = 0;

  delay(1000);

}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

//  digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope

  //        lookForData();
  //        if (newCommand == true) {
  //          obey();
  //          newCommand = false;
  //        }


  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  readNNpredictions();
  //    readMotorEncodersValues();


  //  Serial.println(proximity_value_[3]);


  //  for (int i = 0; i < NUM_FINGERS; i++) {
  //    Serial.print(prox_nrm[i], 6); Serial.print('\t');
  //    Serial.print(press_nrm[i], 6); Serial.print('\t');
  //  }


  //    if (Serial.available() > 0){
  //
  //      user_input = Serial.read();
  //
  //      if (user_input == 0x2C){ // send chr ',' to read Pboard addrs
  //        scan_i2c();}
  //
  //      if (user_input == 0x71) { // send chr 'q' to close
  //        for (int i = 0; i < NUM_PBOARDS; i++) {
  //          byte close_finger[4] = {addrs[i], 0x0C, 0x80, 0x10};
  //          send_cmmnd(close_finger);}}
  //
  //      if (user_input == 0x77) { // send chr 'w' to break
  //        for (int i = 0; i < NUM_PBOARDS; i++) {
  //        byte apply_break[4] = {addrs[i], 0x0C, 0x03, 0x00};
  //        send_cmmnd(apply_break);}}
  //
  //      if (user_input == 0x65) { // send chr 'e' to open
  //        for (int i = 0; i < NUM_PBOARDS; i++) {
  //        byte open_finger[4] = {addrs[i], 0x0C, 0xC0, 0x10};
  //        send_cmmnd(open_finger);}}
  //
  //        if (user_input == 0x70){ // send chr 'p' to start pid position controller
  //          pid_flag = true;
  //          contact_flag = false;}
  //
  //      if (user_input == 0x63){ // send chr 'c' to start contact detection
  //          pid_flag = false;
  //          finger_num = 0;
  //          contact_flag = true;}
  //
  //      if (user_input == 0x6E){ // send chr 'n' to do nothing
  //          pid_flag = false;
  //          contact_flag = false;}}


  Serial.print('\n');

}
