#include <Wire.h>

#define J0 0 //0 or 1
#define J1 0 //0 or 1
#define MUX_ADDRESS (0b01110000 | J0<<1 | J1<<2)

/***** USER PARAMETERS *****/
int i2c_ids_[] = {112};//, MUX_ADDR|1}; or {113}
int ir_current_ = 4; // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 1; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz

unsigned long time;

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

#define NUM_SENSORS 4 // Total number of sensors(ir + baro) connected
#define NFINGERS 1 // number of fingers connected

#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

#define DELTA_MBAR_THRESH 1.0

#define I2C_FASTMODE 1

#define LOOP_TIME 20  // loop duration in ms

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

unsigned long starttime;

bool touched;
float touch_baseline;
float force;

unsigned int proximity_value[NFINGERS]; // current proximity reading
unsigned int average_value[NFINGERS];   // low-pass filtered proximity reading
signed int  fa1[NFINGERS];              // FA-I value;
signed int fa1derivative[NFINGERS];     // Derivative of the FA-I value;
signed int fa1deriv_last[NFINGERS];     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 15;            // Sensitivity of touch/release detection, values closer to zero increase sensitivity
int touch_analysis = 1;

void setup()
{
  Serial.begin(57600);
  Wire.begin();

  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);

  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Wire.beginTransmission(i2c_ids_[i]);
    Wire.write(0);
    int errcode = Wire.endTransmission();

    initIRSensor(i2c_ids_[i]);
    initPressure(i2c_ids_[i]);
  }

  prev_time = millis();
  touched = false;
  // Serial.println("Starting main loop...");
//  delay(100);

//  for (int i = 0; i < NFINGERS; i++) {
//    proximity_value[i] = readProximity();
//    average_value[i] = proximity_value[i];
//    fa1[i] = 0;
//  }

  starttime = micros();

}

//Configure the various parts of the sensor
void initVCNL4040()
{
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

unsigned int readProximity() {
  int data = readFromCommandRegister(PS_DATA_L); //Get proximity values
  return data;
}


unsigned int readIRValues_prox(int id, int sensor) {
    selectSensor(id, sensor);
    unsigned int proximity_value_ = readProximity();
    return (proximity_value_);
   
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
            Coff[j][i] = ((data[0] << 8) | data[1]);
//            Serial.println(Coff[j][i]);
//            delay(300);
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


float readPressure(int muxAddr, int sensor, int j) {
  selectSensor(muxAddr, sensor);
  unsigned long D1 = getPressureReading();
  unsigned long D2 = getTempReading();
//  Serial.println(D2);
//
//  signed long dT = D2 - (Coff[4][j] * 256);
//  signed long TEMP = 20000 + ((unsigned long long)dT)*(Coff[5][j]/8388608);
//  signed long long OFF = Coff[1][j]*131072 + (Coff[3][j]*dT)/64;
//  signed long long SENS = Coff[0][sensor]*65536 + (Coff[2][j]*dT)/128;
//  float P = (((D1*SENS)/2097152) - OFF)/32768;
//
  signed long dT = D2 - (Coff[4][j] << 8);
  signed long TEMP = 20000 + (((unsigned long long)dT) * Coff[5][j]) >> 23;
  signed long long OFF = Coff[1][j] << 17; // + (Coff[3][j]*dT)>>6;
  signed long long SENS = Coff[0][j] << 16; // + (Coff[2][j]*dT)>>7;
  float P = ((D1 * (SENS >> 21) - OFF)) >> 15;

  float mbar = P/100.0;
//  Serial.print(mbar);
//  Serial.print(" ");
  return (mbar);
}

void readPressureValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j=0; j<NFINGERS; j++) {

     float mbar = readPressure(i2c_ids_[i], sensor_ports[j], j);
     int mbar_int = (int) mbar; // converts float pressure value to integer pressure value
//    Serial.println("Pres: "); Serial.println(mbar_int);
    Serial.print(mbar);
    Serial.print('\t');
    Serial.print(mbar, HEX);
    Serial.println();
    
    byte * b = (byte *) &mbar; //convert float to 4 byte array (https://forum.arduino.cc/index.php?topic=112597.0)

//    Serial.write(b,4); // write pressure float variable to serial port
//    Serial.println();
//      Serial.write(buf2,2); // write pressure float variable to serial port

    
        if (touched) {
          //if (touch_ended(mbar, delta_mbar)) {
          touched = false;
          //}
          force = convert_mbar_to_force(mbar - touch_baseline);
//          Serial.print(force);
//          Serial.println();
        }
         else {
              touch_baseline = mbar;
        }
    }
  }
}


unsigned long getTempReading() {
  // Start I2C Transmission
  Wire.beginTransmission(BARO_ADDRESS);
  // Refresh temperature with the OSR = 256
  Wire.write(0x50);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(PRESS_MEAS_DELAY_MS);

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
  return ((unsigned long)data[0]) << 16 | ((unsigned long)data[1]) << 8 | ((unsigned long)data[2]);
}

unsigned long  getPressureReading() {
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
  delay(PRESS_MEAS_DELAY_MS);

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
  return ((unsigned long)data[0]) << 16 | ((unsigned long)data[1]) << 8 | ((unsigned long)data[2]);
}

void selectSensor(int muxID, int i) {
  Wire.beginTransmission(muxID);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void writeByte(byte addr, byte val)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission(); //Release bus
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



void readIRValues() {

  for (int i = 0; i < num_devices_; i++) {
     for (int j=0; j<NFINGERS; j++) {
    unsigned int prox_value = readIRValues_prox(i2c_ids_[i],sensor_ports[j]);
    Serial.print(prox_value);
    Serial.print('\t');
    Serial.print(prox_value, HEX);
    Serial.print('\t');

  // Convert prox_value into an array of bytes for serial communication//
    byte buf[2];
    byte a = prox_value;
    byte b = prox_value >> 8; // bitshift the prox_value variable into 2 bytes
    buf [1] = a; buf[0] = b; // fill buffer with bytes
    //    Serial.print("Finger:  ");Serial.println(sensor_IR_ports[j]);Serial.println(prox_value);
    //    delay(100);
    
//    Serial.write(buf,2);
//    Serial.print('\t');
    
    
      //------- Touch detection -----//
//    int touch_packet = 0;
    proximity_value[j] = prox_value;
    fa1deriv_last[j] = fa1derivative[j];
    fa1derivative[j] = (signed int) average_value[j] - proximity_value[j] - fa1[j];
    fa1[i] = (signed int) average_value[j] - proximity_value[j];
//    Serial.print(proximity_value[i]); Serial.print('\t'); //Serial.print(","); Serial.print(fa2derivative);

    if (touch_analysis) {
//              Serial.print("here");
      if ((fa1deriv_last[j] < -sensitivity && fa1derivative[j] > sensitivity) || (fa1deriv_last[j] > 50 && fa1derivative[j] < -50)) { // zero crossing detected
//         Serial.print(proximity_value); Serial.print(","); Serial.print(fa2); Serial.print(","); Serial.println(fa2derivative);
//        Serial.print("here");
        int touch_packet = 255;
//        Serial.print("touch = ");Serial.println(touch_packet);
        Serial.write(touch_packet);
        if (fa1[j] < -sensitivity) // minimum
        {
//          Serial.print("don't touch me!");
          touched = true;

//          Serial.println(touch_packet);
//          Serial.write(touch_packet);
        }
        else if (fa1[j] > sensitivity) // maximum
        {
//          Serial.print("R");
        }
      }
        else { 
        int touch_packet = 0;
        Serial.write(touch_packet);
        }
             
//             Serial.print("touch = ");Serial.println(touch_packet);
      }

     // Do this last
      average_value[j] = EA * prox_value + (1 - EA) * average_value[j];
      while (millis() < time + LOOP_TIME); // enforce constant loop time

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

// LabView Additions Here //
////////////////////////////

void sendFlag() {
  byte flag[3];
  flag[0] = 0x7E;flag[1] = 0x7E;flag[2] = 0x7E;
  Serial.write(flag,3); // write flag array to serial port to indicate start of data
}


void loop() {
  unsigned long curtime;

  curtime = micros();

//  sendFlag(); // -> send flag of "7E7E7E" to laptop   
  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
  delay(LOOP_TIME);

}
