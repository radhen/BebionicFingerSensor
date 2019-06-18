/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////

#include <Wire.h>
//#include "rp_testing.h"
#include <Smoothed.h> // available @ https://github.com/MattFryer/Smoothed

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x65  // MS5637_02BA03 I2C address is 0x76(118)
#define VCNL4040_ADDR 0x73 //7-bit unshifted I2C address of VCNL4040
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

bool min_flag_ir = true;
bool min_flag_baro =  true;
int drop_count_ir = 10;
int drop_count_baro = 10;
volatile float press_nrm[NUM_FINGERS];
volatile float prox_nrm[NUM_FINGERS];
volatile float min_pressure[NUM_FINGERS];
volatile float min_distance[NUM_FINGERS];

////////////// Exponential Avg. variables for CONTACT detection/////////////////
// https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
volatile float highpass_proximity_value_[NUM_FINGERS] = {0.0};
volatile float EMA_a_ir[NUM_FINGERS] = {0.3};
volatile float EMA_S_ir[NUM_FINGERS] = {0.0};
//////////////////////////////////////////////////////////////////////////////////

// moving avg.
Smoothed <float> smooth_ir;
Smoothed <float> smooth_baro; 


///////////////////////////////////////////////////////////
///////////// PCF SENSOR FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

// Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  int err = Wire.endTransmission(false); //Send a restart command. Do not release bus.
//  Serial.println(err);
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

  //  selectSensor(fingers[id].barPort);
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(byte(0));
  int errcode = Wire.endTransmission();

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
//    Serial.print(Coff[i][id]); Serial.print('\t');
  }
//  Serial.print('\n');
  delay(300);
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(byte(0));
  Wire.endTransmission();

}

int32_t getPressureReading(int id) {
  //  selectSensor(muxAddr, sensor);
  //  selectSensor(fingers[id].barPort);

  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(byte(0));
  int errcode = Wire.endTransmission();

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x1E); // Send reset command
  Wire.endTransmission(); // Stop I2C Transmission

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x40); // Refresh pressure with the OSR = 256
  Wire.endTransmission(); // Stop I2C Transmission

  delayMicroseconds(800);

  //  selectSensor(fingers[id].barPort);

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(byte(0x00));  // Select data register
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
  int count = 0;

  if (drop_count_baro > 0) {
    // Drop first five values from all the sensors
    drop_count_baro -= 1;
    for (int i = 0; i < NUM_FINGERS; i++) {
      pressure_value_[i] = getPressureReading(i);
    }
  }

  for (int i = 0; i < NUM_FINGERS; i++) {

///// ORIGINAL CODE. JUST READ RAW VALUES ////////
//    Wire.beginTransmission(VCNL4040_ADDR);
//    Wire.write(byte(0));
//    int errcode = Wire.endTransmission();
//    pressure_value_[count] = getPressureReading(i);
//    Serial.print(pressure_value_[count]); Serial.print('\t');
//    count += 1;


    pressure_value_[i] = getPressureReading(i);
    Serial.print(pressure_value_[count]); Serial.print('\t');
            
    //*********** NORMALIZE BARO SENSOR VALUES ************//
    // keep track of the running min values
    if (min_flag_baro == true) {
      min_pressure[i] = pressure_value_[i];
//      Serial.print(min_pressure[i]); Serial.print('\t');
    }
    if (pressure_value_[i] < min_pressure[i]) {
      if (pressure_value_[i] == 0){
        // dicarding the anomaly
        // do nothing
        }
      else{
        min_pressure[i] = pressure_value_[i];
        }
    }
    
    press_nrm[i] = pressure_value_[i] - min_pressure[i];
//    Serial.print(press_nrm[i], 6); Serial.print('\t');

    //******** Moving avg. ********//
      smooth_baro.add(pressure_value_[i]);
      // Get the smoothed values
      float smoothed_baro = smooth_baro.get();
//      Serial.print(smoothed_baro/50000.0, 6); Serial.print('\t'); //Found the max value 50000.0 by manually pressing the sensor
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

  //  selectSensor(fingers[id].irPort);
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(byte(0));
  int errcode = Wire.endTransmission();

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
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(byte(0));
  Wire.endTransmission();
}


void readIRValues() {

/// ORIGINAL CODE. JUST READ RAW VALUES  
//  int count = 0;
//  for (int i = 0; i < NUM_FINGERS; i++) {
//    //    selectSensor(fingers[i].irPort);
//    proximity_value_[count] = readFromCommandRegister(PS_DATA_L);
//    Serial.print(proximity_value_[count]); Serial.print('\t');
//    count += 1;
//  }
  
   if (drop_count_ir > 0 ) {
    // Drop first five values from all the sensors
    drop_count_ir -= 1;
    //    Serial.println("dropping values");
    for (int i = 0; i < NUM_FINGERS; i++) {
      proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
    }
  }

  for (int i = 0; i < NUM_FINGERS; i++) {
      proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
      Serial.print(proximity_value_[i]); Serial.print('\t');

      //*********** NORMALIZE IR SENSOR VALUES ************//
      // keep track of the running min values
      if (min_flag_ir == true) {
        min_distance[i] = proximity_value_[i];
      }
      if (proximity_value_[i] < min_distance[i]) {
        min_distance[i] = proximity_value_[i];
      }
      
      prox_nrm[i] = float(proximity_value_[i] - min_distance[i]);
//      Serial.print(prox_nrm[i]); Serial.print('\t');

      //******** Moving avg. ********//
      smooth_ir.add(prox_nrm[i]);
      // Get the smoothed values
      float smoothed_ir = smooth_ir.get();
//      Serial.print(smoothed_ir/4000.0, 6); Serial.print('\t'); //Found the max value 4000.0 by manually pressing the sensor
    }
    
    min_flag_ir = false;
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


//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  Wire.begin();
  //  Wire.setClock(100000);
  pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope
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

//  while (!Serial) 
//    {
//    }
//
//  Serial.println ();
//  Serial.println ("I2C scanner. Scanning ...");
//  byte count = 0;
//  
//  Wire.begin();
//  for (byte i = 8; i < 120; i++)
//  {
//    Wire.beginTransmission (i);
//    if (Wire.endTransmission () == 0)
//      {
//      Serial.print ("Found address: ");
//      Serial.print (i, DEC);
//      Serial.print (" (0x");
//      Serial.print (i, HEX);
//      Serial.println (")");
//      count++;
//      delay (1);  // maybe unneeded?
//      } // end of good response
//  } // end of for loop
//  Serial.println ("Done.");
//  Serial.print ("Found ");
//  Serial.print (count, DEC);
//  Serial.println (" device(s).");

  // moving avg. initalization  
  smooth_ir.begin(SMOOTHED_AVERAGE, 100); 
  smooth_baro.begin(SMOOTHED_AVERAGE, 50); 

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


  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  
  //  readNNpredictions();
  //  readMotorEncodersValues();


 Serial.print('\n');

}
