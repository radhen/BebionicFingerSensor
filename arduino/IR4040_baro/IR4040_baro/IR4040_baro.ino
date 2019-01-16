#include <Wire.h>

#define J0 0 //0 or 1
#define J1 0 //0 or 1
#define MUX_ADDRESS (0b01110000 | J0<<1 | J1<<2)

/***** USER PARAMETERS *****/
int i2c_ids_[] = {112};//, MUX_ADDR|1};
int ir_current_ = 20; // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 1; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz

/***** GLOBAL CONSTANTS *****/

#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define COMMAND_0 0x80  // starts measurements, relays data ready info

//// VCNL4040
#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040
//Command Registers have an upper byte and lower byte.
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ID  0x0C

//#define NUM_SENSORS 2 // Total number of sensors(ir + baro) connected
#define NFINGERS 1 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.
#define DELTA_MBAR_THRESH 1.0
#define I2C_FASTMODE 1
// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
int sensor_ports[NFINGERS] = {0}; // Mux board ports for each Barometer sensor {0,2,4,6}
float prev_mbar[NFINGERS];

int num_devices_;
unsigned int ambient_value_;
byte serialByte;
unsigned long Coff[6][NFINGERS];
unsigned long Ti = 0, offi = 0, sensi = 0;
unsigned int data[3];

unsigned long prev_time;

bool touched;
float touch_baseline;
float force;

//unsigned long long int prox_value_arr[1][NFINGERS]; // current proximity reading
//unsigned long long int prss_value_arr[1][NFINGERS]; // current pressure reading

unsigned int proximity_value[NFINGERS]; // current proximity reading
unsigned int average_value[NFINGERS];   // low-pass filtered proximity reading
signed int  fa1[NFINGERS];              // FA-I value;
signed int fa1derivative[NFINGERS];     // Derivative of the FA-I value;
signed int fa1deriv_last[NFINGERS];     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 20;            // Sensitivity of touch/release detection, values closer to zero increase sensitivity
int touch_analysis = 0;

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

#include <Filters.h>
 
// create a one pole (RC) lowpass filter
FilterOnePole highpassFilter(HIGHPASS, 3.0);  
FilterOnePole lowpassFilter(LOWPASS, 2.0); 
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


float convert_mbar_to_force(float above_baseline) {
  return (above_baseline / 0.19);
}


bool touch_ended(float mbar, float delta_mbar) {
  return (delta_mbar < DELTA_MBAR_THRESH);
}


bool touch_started(float mbar, float delta_mbar) {
  return (delta_mbar > DELTA_MBAR_THRESH);
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


unsigned long getTempReading() {
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

  // Convert the data
  return ((data[0]*65536.0) + (data[1]*256.0) + data[2]);
}


unsigned long getPressureReading() {
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
  unsigned long D1 = getPressureReading();
  unsigned long D2 = getTempReading();
  
//  Serial.print("D1: ");
//  Serial.println(D1);
//  Serial.print("D2: ");
//  Serial.println(D2);

//  unsigned long dT = D2 - (Coff[4][j] * 256);
//  D2 = 2000 + (dT * (Coff[5][j] / 8388608));
//  unsigned long long OFF = Coff[1][j] * 131072 + (Coff[3][j]*dT) / 64;
//  unsigned long long SENS = Coff[0][j] * 65536 + (Coff[2][j]*dT) / 128;

  unsigned long dT = D2 - (Coff[4][j] << 8);
  unsigned long TEMP = 2000 + (dT << (Coff[5][j] >> 23));
  unsigned long long OFF = Coff[1][j] << 17;// + ((Coff[3][j]*dT) >> 6);
  unsigned long long SENS = Coff[0][j] << 16;// + ((Coff[2][j]*dT) >> 7);
  
//  Serial.print("dT: ");
//  Serial.println(dT);
//  Serial.print("TEMP: ");
//  Serial.println(TEMP);
//  Serial.print("OFF: ");
//  Serial.println((unsigned long) OFF);
//  Serial.print("SENS: ");
//  Serial.println((unsigned long) SENS);


//  // 2nd order temperature and pressure compensation
//  if(D2 < 2000)
//  {
//    Ti = (dT * dT) / (pow(2,31));
//    offi = 5 * ((pow((D2 - 2000), 2))) / 2;
//    sensi =  offi / 2;
//    if(D2 < - 1500)
//    {
//       offi = offi + 7 * ((pow((D2 + 1500), 2)));
//       sensi = sensi + 11 * ((pow((D2 + 1500), 2)));
//    }
//  }
//  else if(D2 >= 2000)
//  {
//     Ti = 0;
//     offi = 0;
//     sensi = 0;
//  }
//
//  // Adjust temp, off, sens based on 2nd order compensation
//  D2 -= Ti;
//  OFF -= offi;
//  SENS -= sensi;

//  D1 = (((D1 * SENS) / 2097152) - OFF);
//  D1 /= 32768;

  D1 = (((D1 * SENS) >> 21) - OFF) >> 15;
  float mbar = D1/100.0;
  return (mbar);
}


void readPressureValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {

     float mbar = readPressure(i2c_ids_[i], sensor_ports[j], j);
     Serial.print(' ');
     Serial.print(mbar);
     Serial.print(' ');

    int mbar_int = (int) mbar; // converts float pressure value to integer pressure value

        if (touched) {
          //if (touch_ended(mbar, delta_mbar)) {
          touched = false;
          //}
          force = convert_mbar_to_force(mbar - touch_baseline);
          Serial.print(force);
          Serial.println();
        }
         else {
              touch_baseline = mbar;
        }
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
    proximity_value[j] = prox_value;
    fa1deriv_last[j] = fa1derivative[j];
    fa1derivative[j] = (signed int) average_value[j] - proximity_value[j] - fa1[j];
    fa1[i] = (signed int) average_value[j] - proximity_value[j];
    
//    Serial.print(fa1[i]);
//    Serial.print('\t');
//    Serial.print(fa1deriv_last[j]);
//    Serial.print('\t');
//    Serial.print(fa1derivative[j]);
    
    if (touch_analysis) {
      //        Serial.print(",");
      if ((fa1deriv_last[j] < -sensitivity && fa1derivative[j] > sensitivity) || (fa1deriv_last[j] > sensitivity && fa1derivative[j] < -sensitivity)) { // zero crossing detected
        if (fa1[j] < -sensitivity) // minimum
        {
          Serial.print("1");
          touched = true;
        }
        else if (fa1[j] > sensitivity) // maximum
        {
//          Serial.print("0");
        }
      }
      Serial.print("0");
    }

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

  prev_time = millis();

  // setup code for touch_analysis
  touched = false;
  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {
    proximity_value[i] = readProximity(i2c_ids_[i],sensor_ports[j]);
    average_value[i] = proximity_value[i];
    fa1[i] = 0;
    }
  }

  // setup code for moving avg over baro values
  for (int j = 0; j < NFINGERS; j++) {
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[j][thisReading] = 0;
    }
  }

  // zeroing sensor values
  float baro[NFINGERS][300];
  int count = 1; 
  while (count < 300){ 
      for (int j = 0; j < NFINGERS; j++) {
          baro[j][count] = readPressure(i2c_ids_[0], sensor_ports[j], j);
//          Serial.println(baro[0][count]);
          count += 1;       
      }
  }
    
  for (int j = 0; j < NFINGERS; j++) { 
    float min_value = baro[j][10];
    float max_value = baro[j][10]; 
    for (int i =11; i<120; i++){ 
      if (baro[j][i] < min_value) {
          min_value = baro[j][i];
      }
      if (baro[j][i] > max_value) {
          max_value = baro[j][i];
      }
    }
    min_baro[j] = min_value;
    max_baro[j] = max_value;
  }

//  Serial.print("min baro value is");
//  Serial.println(min_baro[0]);
//  Serial.print("max baro value is");
//  Serial.println(max_baro[0]);

  prev_baro = readPressure(i2c_ids_[0], sensor_ports[0], 0);
  prev_ir = readProximity(i2c_ids_[0],sensor_ports[0]);

  inputStats.setWindowSecs(0.5);
  
  
}


void loop() {
  
  unsigned long curtime = micros();

  // Print min- and max- values to set Y-axis in serial plotter
//  Serial.print(0);  // To freeze the lower limit
//  Serial.print(" ");
//  Serial.print(65536);  // To freeze the upper limit
//  Serial.print(" ");
  
  readIRValues(); //-> array of IR values (2 bytes per sensor)
//  Serial.println();
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
//  Serial.print(' ');
//  Serial.print(min_baro[0]);


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

//    inputStats.input(readPressure(i2c_ids_[0], sensor_ports[0], 0));
//    Serial.print(' ');
//    Serial.println(inputStats.mean());

//      float baro_constraint = constrain(readPressure(i2c_ids_[0], sensor_ports[0], 0), min_baro[0]*1.05, max_baro[0]);
//      float baro_rescale = map(baro_constraint, min_baro[0]*1.05, max_baro[0], 0, 255);
//      Serial.println(baro_rescale);

  
//  Serial.pri/nt(arr[0]);
//  Serial.print(' ');
//  readIRValue/s();
//  Serial.print('\t');
//  unsigned long starttime = micros();
//  Serial.print(starttime - curtime);
    Serial.println();

}
