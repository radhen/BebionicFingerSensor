#include <Wire.h>

#define J0 0 //0 or 1
#define J1 0 //0 or 1
#define MUX_ADDRESS (0b01110000 | J0<<1 | J1<<2)

/***** USER PARAMETERS *****/
int i2c_ids_[] = {113};//, MUX_ADDR|1};
int ir_current_ = 4; // range = [0, 20]. current = value * 10 mA
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

#define NUM_SENSORS 2 // Total number of sensors(ir + baro) connected
#define NFINGERS 1 // number of fingers connected

#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

#define DELTA_MBAR_THRESH 1.0

#define I2C_FASTMODE 1

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter



/***** GLOBAL VARIABLES *****/
int sensor_Bar_ports[NFINGERS] = {0};//Mux board ports for each IR sensor {0,2,4,6}
int sensor_IR_ports[NFINGERS] = {0};//{4};// Mux board ports for each Barometer sensor {0,2,4,6}
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
signed int sensitivity = 45;            // Sensitivity of touch/release detection, values closer to zero increase sensitivity
int touch_analysis = 0;

void setup()
{
  Serial.begin(57600);
  Wire.begin();

  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
//  Serial.print("Attached i2c devices: ");
//  Serial.println(num_devices_);

  
  
  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Wire.beginTransmission(i2c_ids_[i]);
    Wire.write(0);
    int errcode = Wire.endTransmission();

    initIRSensor(i2c_ids_[i]);

    for (int j = 0; j < NFINGERS; j++) {
      initPressure(i2c_ids_[i], sensor_Bar_ports[j], j); //new pressure function
      prev_mbar[j] = readPressure(i2c_ids_[i], sensor_Bar_ports[j]);
    }
  }

  prev_time = millis();
  touched = false;
  // Serial.println("Starting main loop...");
  delay(100);

  for (int i = 0; i < NFINGERS; i++) {
    proximity_value[i] = readProximity();
    average_value[i] = proximity_value[i];
    fa1[i] = 0;
  }

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


void initPressure(int muxAddr, int sensor, int index) {
//  Serial.println("initPressure");
  selectSensor(muxAddr, sensor);
//  Serial.print("index:");Serial.println(index);
  for (int i = 0; i < 6; i++){ //loop over Coefficient elements
//Serial.println("for loop");
//Serial.println(i);
    
    // Start I2C Transmission
    Wire.beginTransmission(BARO_ADDRESS);
    // Select data register
    Wire.write(0xA2 + (2 * i));
//    Serial.println("here");
//    Serial.println((0xA2 + (2*i)));
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

//    // Convert the data
//    Serial.print("sensor:_");Serial.println(sensor);
//    Serial.print("index:_");Serial.println(index);
//    Serial.print("data:_");Serial.println(((data[0] << 8) | data[1]));
    Coff[i][index] = ((data[0] << 8) | data[1]);
//          Serial.print(Coff[i][index]);
//          Serial.print("\t");
  }
//      Serial.println();
  delay(300);
}


float readPressure(int muxAddr, int sensor) {
  selectSensor(muxAddr, sensor);
  unsigned long D1 = getPressureReading();
  unsigned long D2 = getTempReading();

//  signed long dT = D2 - (Coff[4][sensor] * 256);
//  signed long TEMP = 20000 + ((unsigned long long)dT)*(Coff[5][sensor]/8388608);
//  signed long long OFF = Coff[1][sensor]*131072 + (Coff[3][sensor]*dT)/64;
//  signed long long SENS = Coff[0][sensor]*65536 + (Coff[2][sensor]*dT)/128;
//  float P = (((D1*SENS)/2097152) - OFF)/32768;

  signed long dT = D2 - (Coff[4][sensor] << 8);
  signed long TEMP = 20000 + (((unsigned long long)dT) * Coff[5][sensor]) >> 23;
  signed long long OFF = Coff[1][sensor] << 17;// + (Coff[3]*dT)>>6;
  signed long long SENS = Coff[0][sensor] << 16;// + ((Coff[2]*dT)>>7;
  float P = ((D1 * (SENS >> 21) - OFF)) >> 15;

  float mbar = P/100.0;
  Serial.print((mbar));
  Serial.println();
  return (mbar);
}

void readPressureValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j=0; j<NFINGERS; j++) {
      
     float mbar = readPressure(i2c_ids_[i], sensor_Bar_ports[j]);

    int mbar_int = (int) mbar; // converts float pressure value to integer pressure value

        if (touched) {
          //if (touch_ended(mbar, delta_mbar)) {
          touched = false;
          //}
          force = convert_mbar_to_force(mbar - touch_baseline);
          Serial.print(force);
          Serial.print('\t');
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
  for (int i = 0; i < NFINGERS; i += 2)
  {
    // specify IR sensor
    selectSensor(id, sensor_IR_ports[i]);
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
    unsigned int prox_value = readIRValues_prox(i2c_ids_[i],sensor_IR_ports[j]);
    Serial.print(prox_value);
    Serial.print(" ");

      //------- Touch detection -----//
    proximity_value[j] = prox_value;
    fa1deriv_last[j] = fa1derivative[j];
    fa1derivative[j] = (signed int) average_value[j] - proximity_value[j] - fa1[j];
    fa1[i] = (signed int) average_value[j] - proximity_value[j];
    if (touch_analysis) {
      //        Serial.print(",");
      if ((fa1deriv_last[j] < -sensitivity && fa1derivative[j] > sensitivity) || (fa1deriv_last[j] > 50 && fa1derivative[j] < -50)) { // zero crossing detected
        if (fa1[j] < -sensitivity) // minimum
        {
          touched = true;
        }
        else if (fa1[j] > sensitivity) // maximum
        {
          Serial.print("R");
        }
      }
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

void loop() {
  unsigned long curtime;

  curtime = micros();
  
  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)

  Serial.print(curtime - starttime);
  Serial.print('\t');

}



