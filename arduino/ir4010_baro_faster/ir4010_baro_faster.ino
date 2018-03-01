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

#define VCNL4010_ADDRESS 0x13
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_MOD 0x8F  // proximity modulator timing


#define NUM_SENSORS 8 // Total number of sensors(ir + baro) connected
#define NFINGERS 1 // number of fingers connected

#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

#define DELTA_MBAR_THRESH 1.0

#define I2C_FASTMODE 1

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter



/***** GLOBAL VARIABLES *****/
int sensor_Bar_ports[NFINGERS] = {5};//Mux board ports for each IR sensor {112 ports, 113 ports}
int sensor_IR_ports[NFINGERS] = {4};//{4};// Mux board ports for each Barometer sensor {112 ports, 113 ports}
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


unsigned int readIRValues_prox(int id, int sensor) {
  char buf[8];
  //  Serial.print(id);
  // read all 8 sensors on the strip
//  for (int i = 6; i < NUM_SENSORS; i += 2) // i should strt from 0
//  {
    selectSensor(id, sensor);

    //ambient_value_ = readAmbient();
    unsigned int proximity_value_ = readProximity();

    //sprintf(buf, "%6u", proximity_value_);
    return (proximity_value_);
    //return (buf);

//  }
//  Serial.print('\t');
  //  Wire.beginTransmission(id);
  //  Wire.write(0);
  //  Wire.endTransmission();
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

void writeByte(byte address, byte data)
{
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
    //Wire.beginTransmission(id);
    //Wire.write(1 << i);
    //Wire.endTransmission();
    //Feel free to play with any of these values, but check the datasheet first!
    writeByte(AMBIENT_PARAMETER, 0x7F);
    writeByte(IR_CURRENT, ir_current_);
    writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay

    delay(50);

    byte temp = readByte(PRODUCT_ID);
    byte proximityregister = readByte(IR_CURRENT);
    //Serial.println(proximityregister);
    if (temp != 0x21) { // Product ID Should be 0x21 for the 4010 sensor
            Serial.print("IR sensor failed to initialize: id = ");
            Serial.print(i);
            Serial.print(". ");
            Serial.println(temp, HEX);
    }
    else
    {
//            Serial.print("IR sensor online: id = ");
//            Serial.println(i);
    }
  }
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();

}


unsigned int readAmbient() {

  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  unsigned long startTime = millis();
  while (!(readByte(0x80) & 0x40)) { // Wait for the proximity data ready bit to be set
    if ((millis() - startTime) > 20) {
      return 0;
    }
  }

  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
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
    //Serial.print(proximity_value[i]); Serial.print('\t'); //Serial.print(","); Serial.print(fa2derivative);

    if (touch_analysis) {
      //        Serial.print(",");
      if ((fa1deriv_last[j] < -sensitivity && fa1derivative[j] > sensitivity) || (fa1deriv_last[j] > 50 && fa1derivative[j] < -50)) { // zero crossing detected
        // Serial.print(proximity_value); Serial.print(","); Serial.print(fa2); Serial.print(","); Serial.println(fa2derivative);
        if (fa1[j] < -sensitivity) // minimum
        {
          //Serial.print("don't touch me!");
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


void loop() {
  unsigned long curtime;

  curtime = micros();
  
  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)


//  Serial.print(curtime - starttime);
//  Serial.print("\t");
}

