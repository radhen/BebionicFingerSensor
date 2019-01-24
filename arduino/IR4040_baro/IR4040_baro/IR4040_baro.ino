#include <Wire.h>
#include <Filters.h>
#include <BaroSensor.h>


/***** USER PARAMETERS *****/
int i2c_ids_[] = {112};//, MUX_ADDR|1};


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
#define DELTA_MBAR_THRESH 1.0
#define I2C_FASTMODE 1

/***** GLOBAL VARIABLES *****/
int sensor_ports[NFINGERS] = {0}; // Mux board ports for each Barometer sensor {0,2,4,6}
float prev_mbar[NFINGERS];

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
uint16_t Coff[6][NFINGERS];
int32_t Ti = 0, offi = 0, sensi = 0;
unsigned int data[3];


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
FilterOnePole highpassFilter(HIGHPASS, 3.0);  
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
    Serial.println(errcode);
    for (int i = 0; i < NFINGERS; i++){
        selectSensor(muxAddr, sensor_ports[i]);
        for (int j = 0; j < 6; j++){ //loop over Coefficient elements
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

//typedef struct finger_struct{
//  int muxAddr;
//  int irPort;
//  int sensorPort;
//}FingerStruct;
//
//FingerStruct fingers[NUM_FINGERS] = {{112, 4, 4}};


int32_t getTempReading() {
  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Refresh temperature with the OSR = 8192
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
    data[0] = Wire.read();file:///home/radhen/Documents/bebionic_fingersensor/arduino/pressure.ino

    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  // Convert the data
  return ((data[0]*65536.0) + (data[1]*256.0) + data[2]);
}


int32_t getPressureReading() {
  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Send reset command
  Wire.write(0x1E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Refresh pressure with the OSR = 8192
  Wire.write(0x40);
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
  // ptemp_msb1, ptemp_msb, ptemp_lsb
  if (Wire.available() == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  // Convert the data
  return ((data[0]*65536.0) + (data[1]*256.0) + data[2]);
}
  
float readPressure(int muxAddr, int sensor, int j) {
  selectSensor(muxAddr, sensor);
  int32_t ptemp = getPressureReading();
//  int32_t temp = getTempReading();
  
//  Serial.print("D1: ");
//  Serial.println(D1);
//  Serial.print("D2: ");
//  Serial.println(D2);

  int64_t dT = ptemp - (Coff[4][j] * (1L<<8));
  int32_t temp = 2000 + (dT * Coff[5][j]) / (1L<<23);
  
  int64_t off = Coff[1][j] * (1LL<<17) + (Coff[3][j]*dT) / (1LL<<6);
  int64_t sens = Coff[0][j] * (1LL<<16) + (Coff[2][j]*dT) / (1LL<<7);

//  Serial.print("dT: ");
//  Serial.println(dT);
//  Serial.print("D2: ");
//  Serial.println(D2);
//  Serial.print("OFF: ");
//  Serial.println((unsigned long) OFF);
//  Serial.print("SENS: ");
//  Serial.println((unsigned long) SENS);


  // 2nd order temperature and pressure compensation
  /* Second order temperature compensation for pressure */
    if(temp < 2000) {
      /* Low temperature */
      int32_t tx = temp-2000;
      tx *= tx;
      int32_t off2 = 61 * tx / (1<<4);
      int32_t sens2 = 29 * tx / (1<<4);
      if(temp < -1500) {
        /* Very low temperature */
        tx = temp+1500;
        tx *= tx;
        off2 += 17 * tx;
        sens2 += 9 * tx;
      }
      off -= off2;
      sens -= sens2;
}

  // Adjust temp, off, sens based on 2nd order compensation
  temp -= Ti;
  off -= offi;
  sens -= sensi;

//  Serial.print("D1: ");
//  Serial.println((D1));
//  unsigned long a = D1;
//  unsigned long b = sens;

//  unsigned long long c = (ptemp * sens);
//  unsigned long d = (c / 2097152);
//  ptemp = (d - off);
//  ptemp /= 32768.0;

//  ptemp = (((ptemp * sens) / 2097152) - off);
  int32_t p = ((int64_t)ptemp * sens/(1LL<<21) - off) / (1LL << 15);
  float mbar = (float)p/100;

  return (mbar);
}


void readPressureValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {
     float mbar = readPressure(i2c_ids_[i], sensor_ports[j], j);
     Serial.print(' ');
     Serial.print(mbar);
     Serial.println(' ');
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


unsigned int readProximity(int id, int sensor) {
    selectSensor(id, sensor);
    unsigned int proximity_value_ = readFromCommandRegister(PS_DATA_L); 
    return (proximity_value_);
}


void readIRValues() {

  for (int i = 0; i < num_devices_; i++) {
     for (int j = 0; j < NFINGERS; j++) {
    unsigned int prox_value = readProximity(i2c_ids_[i],sensor_ports[j]);
    Serial.print(prox_value);
    Serial.print('\t');
    
    //------- Touch detection -----/
    // Use highpass filter instead: https://playground.arduino.cc/Code/Filters
     }
  }
}


void setup() {
  Serial.begin(9600);
  Wire.begin();

  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
  
  unsigned long long int prox_value_arr[num_devices_][NFINGERS] = {0};
  unsigned long long int prss_value_arr[num_devices_][NFINGERS] = {0};
  
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

  prev_baro = readPressure(i2c_ids_[0], sensor_ports[0], 0);
  prev_ir = readProximity(i2c_ids_[0],sensor_ports[0]);

  inputStats.setWindowSecs(1);
  
  
}


void loop() {
  unsigned long curtime = micros();


  // Print min- and max- values to set Y-axis in serial plotter
//  Serial.print(0);  // To freeze the lower limit
//  Serial.print(" ");
//  Serial.print(65536);  // To freeze the upper limit
//  Serial.print(" ");


  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
//  readIRValues(); //-> array of IR values (2 bytes per sensor)


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

  
//    unsigned long starttime = micros();
//    Serial.print(starttime - curtime);
//    Serial.println();
    

    /****** Standarizing, zero mean one std. *********/
    // https://www.element61.be/en/resource/standardization-case-real-time-predictions
//    float p = readPressure(i2c_ids_[0], sensor_ports[0], 0);
//    inputStats.input(p);
//    float st_p = (p - inputStats.mean())/inputStats.sigma();
//    float lowpass_p = lowpassFilter.input(st_p);
//    Serial.println(lowpass_p);

//    float ir = readProximity(i2c_ids_[0],sensor_ports[0]);
//    float st_ir = 0;
//    inputStats.input(ir);
//    float sd = inputStats.sigma();
//    if (sd == 0.0){
//      st_ir = 0.0;}
//    else{
//      st_ir = (ir - inputStats.mean())/sd;}
//    float lowpass_ir = lowpassFilter.input(st_ir);
//    Serial.println(st_ir);


      /****** Normalization between 0 and 1. Min max from taken from training data *********/
//      float p = (readPressure(i2c_ids_[0], sensor_ports[0], 0) - 1800.0) / float(6200 - 1800.0);
//      float ir = (readProximity(i2c_ids_[0],sensor_ports[0]) - 30000.0) / float(38000 - 30000.0);
//      Serial.print(p);
//      Serial.print(' ');
//      Serial.println(ir);

//        Serial.println();

}
