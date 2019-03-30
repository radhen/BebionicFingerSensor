#include <Wire.h>
#include <Filters.h>
#include "rp_testing.h"

/***** USER PARAMETERS *****/
int i2c_ids_[] = {113};//, MUX_ADDR|1};

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define COMMAND_0 0x80  // starts measurements, relays data ready info


/****** VCNL4040 ******/
#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ID  0x0C

#define NFINGERS 1 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.
#define DELTA_pressure_value__THRESH 1.0
#define I2C_FASTMODE 1

/***** GLOBAL VARIABLES *****/
int sensor_ports[NFINGERS] = {0}; // Mux board ports for each Barometer sensor {0,2,4,6}
float prev_pressure_value_[NFINGERS];

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NFINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
int32_t data[3];

int timer1_counter;

volatile int32_t pressure_value_[NFINGERS];
volatile uint16_t proximity_value_[NFINGERS];

//unsigned long long int prox_value_arr[1][NFINGERS]; // current proximity reading
//unsigned long long int prss_value_arr[1][NFINGERS]; // current pressure reading

unsigned int proximity_value[NFINGERS]; // current proximity reading
unsigned int average_value[NFINGERS];   // low-pass filtered proximity reading


/**** VAR. for running avg. ****/
double arr[NFINGERS];
const int numReadings = 5;                               // num. of readings to avg. over for running avg. purpose
unsigned long long int readings[NFINGERS][numReadings];  // the readings from the analog input
int readIndex[NFINGERS] = {0};                           // the index of the current reading
unsigned long long int total[NFINGERS] = {0};            // the running total
unsigned long long int average[NFINGERS] = {0};          // the average

float min_baro[NFINGERS];
float max_baro[NFINGERS];

/**** VAR. for expo avg. ****/
// https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
// global variables
float EMA_a = 0.7;
float EMA_S_baro;
float prev_baro;
float EMA_S_ir;
float prev_ir;
float prox_highpass = 0;

// create a one pole (RC) lowpass filter
FilterOnePole highpassFilter(HIGHPASS, 5.0);
FilterOnePole lowpassFilter(LOWPASS, 0.05);
RunningStatistics inputStats;



//Reads a two byte value from a command register
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
  //  Serial.println(errcode);
  for (int i = 0; i < NFINGERS; i++) {
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


void getTempReading() {
  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Refresh temperature with the OSR = 256
  Wire.write(0x50);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Select data register
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 3 bytes of data
  Wire.requestFrom(BARO_ADDRESS, 3);

  // Read 3 bytes of data
  // temp_msb1, temp_msb, temp_lsb
  if (Wire.available() == 3)
  {
    data[0] = Wire.read();

    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  //  data_result = ((data[0]*65536.0) + (data[1]*256.0) + data[2]);
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
  int count = 0;
  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {
      pressure_value_[count] = getPressureReading(i2c_ids_[i], sensor_ports[j]);
      Serial.print(pressure_value_[count]); Serial.print("\t");
      count += 1;
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
  for (int i = 0; i < NFINGERS; i++)
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


//unsigned int readProximity(int id, int sensor) {
//    selectSensor(id, sensor);
//    unsigned int proximity_value_ = readFromCommandRegister(PS_DATA_L);
//    return (proximity_value_);
//}


void readIRValues() {
  int count = 0;
  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {
      selectSensor(i2c_ids_[i], sensor_ports[j]);
      proximity_value_[count] = readFromCommandRegister(PS_DATA_L);
      Serial.print(proximity_value_[count]); Serial.print("\t");
      count += 1;
      //    unsigned int prox_value = readProximity(i2c_ids_[i],sensor_ports[j]);
      //    Serial.print(prox_value);
      //    Serial.print('\t');

      //------- Touch detection -----/
      // Use highpass filter instead: https://playground.arduino.cc/Code/Filters
    }
  }
}


void readNNpredictions() {
  for (int i = 0; i < NFINGERS; i++) {
    float *raw_data;
    float nn_output;
    // volatile float predictions[1];
    raw_data = (float*)malloc(2 * sizeof(float));
    raw_data[0] = pressure_value_[i];
    raw_data[1] = proximity_value_[i];
    nn_output = nnpred(raw_data);
    Serial.print(nn_output); Serial.print('\t');
    free(raw_data);
  }
}


void setup() {
  Serial.begin(57600);
  Wire.begin();
  TWBR = 10;
  pinMode(13, OUTPUT);
  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);

  //  unsigned long long int prox_value_arr[num_devices_][NFINGERS] = {0};
  //  unsigned long long int prss_value_arr[num_devices_][NFINGERS] = {0};

  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Wire.beginTransmission(i2c_ids_[i]);
    Wire.write(0);
    int errcode = Wire.endTransmission();
    //    Serial.println(errcode);
    initIRSensor(i2c_ids_[i]);
    initPressure(i2c_ids_[i]);
  }


  // setup code for moving avg over baro values
  //  for (int j = 0; j < NFINGERS; j++) {
  //    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
  //      readings[j][thisReading] = 0;
  //    }
  //  }

  // zeroing sensor values
  //  float baro[NFINGERS][300];
  //  int count = 1;
  //  while (count < 300){
  //      for (int j = 0; j < NFINGERS; j++) {
  //          baro[j][count] = readPressure(i2c_ids_[0], sensor_ports[j], j);
  ////          Serial.println(baro[0][count]);
  //          count += 1;
  //      }
  //  }

  //  for (int j = 0; j < NFINGERS; j++) {
  //    float min_value = baro[j][10];
  //    float max_value = baro[j][10];
  //    for (int i =11; i<120; i++){
  //      if (baro[j][i] < min_value) {
  //          min_value = baro[j][i];
  //      }
  //      if (baro[j][i] > max_value) {
  //          max_value = baro[j][i];
  //      }
  //    }
  //    min_baro[j] = min_value;
  //    max_baro[j] = max_value;
  //  }

  //  Serial.print("min baro value is");
  //  Serial.println(min_baro[0]);
  //  Serial.print("max baro value is");
  //  Serial.println(max_baro[0]);

  //  prev_baro = readPressure(i2c_ids_[0], sensor_ports[0], 0);
  //  prev_ir = readProximity(i2c_ids_[0],sensor_ports[0]);


}

void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

void loop() {
  digitalWrite(13, !digitalRead(13));
  //    unsigned long curtime = micros();

  // Print min- and max- values to set Y-axis in serial plotter
  //  Serial.print(0);  // To freeze the lower limit
  //  Serial.print(" ");
  //  Serial.print(65536);  // To freeze the upper limit
  //  Serial.print(" ");

  //  unsigned long long result = 0;
  //  float result = 0;

  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  readIRValues(); //-> array of IR values (2 bytes per sensor)

  //  result = pressure_value_;
  //  result = result << 32;
  //  result = result| proximity_value_;
  //  sendToPC(&result);

//  readNNpredictions();

  //  // RUNNING AVG FOR BARO
  //  // https://www.arduino.cc/en/Tutorial/Smoothing
  //  // subtract the last reading:
  //  for (int j = 0; j < NFINGERS; j++) {
  //    total[j] = total[j] - readings[j][readIndex[j]];
  //    readings[j][readIndex[j]] = readPressure(i2c_ids_[0], sensor_ports[j], j);
  //
  ////    readings[j][readIndex[j]] = abs(readings[j][readIndex[j]] - min_baro[j]);
  //
  ////    if (readings[j][readIndex[j]] >= min_baro[j]){
  ////      readings[j][readIndex[j]] = readings[j][readIndex[j]] - min_baro[j];
  ////      }
  ////    else{
  ////      readings[j][readIndex[j]] = min_baro[j] - readings[j][readIndex[j]];
  ////      min_baro[j] = readings[j][readIndex[j]];
  ////      }
  //
  //    total[j] = total[j] + readings[j][readIndex[j]];
  //    readIndex[j] = readIndex[j] + 1;
  //    // if we're at the end of the array...
  //    if (readIndex[j] >= numReadings) {
  //      readIndex[j] = 0; // ...wrap around to the beginning:
  //    }
  //    average[j] = total[j] / numReadings; // calculate the average:
  //    arr[j]= average[j]/ 7.0; // normalize the pressure value
  //  }


  // EXPONENTIAL AVG. FOR BARO
  //    float bar = readPressure(i2c_ids_[0], sensor_ports[0], 0);
  //    bar = abs(bar - min_baro[0]);
  //    EMA_S_baro = (EMA_a*bar) + ((1-EMA_a)*prev_baro);
  //    prev_baro = EMA_S_baro;
  //    EMA_S = constra/in(EMA_S, 0, 7000);
  //    Serial.println(EMA_S_baro);


  // EXPONENTIAL AVG. FOR IR
  //    float ir = readProximity(i2c_ids_[0],sensor_ports[0]);
  //    EMA_S_ir = (EMA_a*ir) + ((1-EMA_a)*prev_ir);
  //    prev_ir = EMA_S_ir;
  //    Serial.println(EMA_S_ir);


  /**** High pass filter IR ****/
  //    float lowpass_ir = lowpassFilter.input(readProximity(i2c_ids_[0],sensor_ports[0]));
  //    float highpass_ir = highpassFilter.input(lowpass_ir);
  //    Serial.println(lowpass_ir);

  //    float baro_constraint = constrain(readPressure(i2c_ids_[0], sensor_ports[0], 0), min_baro[0]*1.05, max_baro[0]);
  //    float baro_rescale = map(baro_constraint, min_baro[0]*1.05, max_baro[0], 0, 255);
  //    Serial.println(baro_rescale);

  //
  //     unsigned long starttime = micros();
  //    Serial.print(starttime - curtime);
  //    Serial.println();


  /****** Normalization between 0 and 1. Min max from taken from training data *********/
  //      float p = (readPressure(i2c_ids_[0], sensor_ports[0], 0) - 1800.0) / float(6200 - 1800.0);
  //      float ir = (readProximity(i2c_ids_[0],sensor_ports[0]) - 30000.0) / float(38000 - 30000.0);
  //      Serial.print(p);
  //      Serial.print(' ');
  //      Serial.println(ir);

  Serial.print("\n");

}
