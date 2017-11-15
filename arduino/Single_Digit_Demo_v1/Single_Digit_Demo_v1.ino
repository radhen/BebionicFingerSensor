
// CORRELL FINGERTIP CODE HERE //
////////////////////////////////
#include <Wire.h>
#include <HardwareSerial.h>

#define J0 0 //0 or 1
#define J1 0 //0 or 1
#define MUX_ADDRESS (0b01110000 | J0<<1 | J1<<2)
  
/***** USER PARAMETERS *****/
int i2c_ids_[] = {113};//,113};//, MUX_ADDR|1};
int ir_current_ = 2; // range = [0, 20]. current = value * 10 mA
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
int sensor_Bar_ports[NFINGERS] = {5};//{5};//Mux board ports for each IR sensor {112 ports, 113 ports}
int sensor_IR_ports[NFINGERS] = {4};//{4};// Mux board ports for each Barometer sensor {112 ports, 113 ports}
float prev_mbar[NFINGERS];


int num_devices_;
unsigned int ambient_value_;
byte serialByte;
unsigned long Coff[6][NFINGERS];
unsigned long Ti = 0, offi = 0, sensi = 0;
unsigned int data[2];
unsigned long prev_time;

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



///////////////////////////
///////////////////////////


// SIGENICS CODE HERE  //
////////////////////////

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


byte inBuf[16];
byte i2cBuf[16];
byte outCount=0;
byte state=fNONE;
bool newCommand=false, toggle=false;
byte inByte;

#define NUM_PENNY_BOARDS 5
int pennyBoardAddresses[NUM_PENNY_BOARDS] = {1,2,3,4,5};//,3};//,3,4,5,6};

////////////////////////////////
///////////////////////////////

void setup(void){
  // Serial port
  Serial.begin(57600);

  //i2c
  Wire.begin();
  newCommand=false;

  // CORRELL FINGERTIP CODE HERE //
  ////////////////////////////////

  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
//   Serial.print("Attached i2c devices: ");
//   Serial.println(num_devices_);


  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Wire.beginTransmission(i2c_ids_[i]);
    Wire.write(0);
    int errcode = Wire.endTransmission();
    initIRSensor(i2c_ids_[i]);

    //    initPressSensors(i2c_ids_[i]); //old pressure function
    for (int j = 0; j < NFINGERS; j++) {
    initPressure(i2c_ids_[i], sensor_Bar_ports[j], j); //new pressure function
    prev_mbar[j] = readPressure(i2c_ids_[i], sensor_Bar_ports[j]);
    }
  }

  prev_time = millis();
  touched = false;
//   Serial.println("Starting main loop...");
  delay(100);

  for (int i = 0; i < NFINGERS; i++) {
    proximity_value[i] = readProximity();
    average_value[i] = proximity_value[i];
    fa1[i] = 0;
  }
}

////////////////////////////
// SIGENICS FUNCTIONS HERE //
////////////////////////////

void toggleLED(void){
    toggle = !toggle;
      if (toggle){
        digitalWrite(13,HIGH);
      }
      else{
        digitalWrite(13,LOW);
      }
}

void scan_i2c(void){
  byte ct, res, devCt;
  devCt=res=0;
  byte addList[72];
  for (ct=1;ct<0x48;ct++){
    Wire.beginTransmission(ct);
    Wire.write(0x1);
    res = Wire.endTransmission();
    if(!res){
      addList[devCt]=ct;
      ++devCt;  
    }
  }
  Serial.write(devCt);
  for(ct=0;ct<devCt;ct++){
    Serial.write(addList[ct]);
  } 
}

bool lookForData()
{
  if (Serial.available() > 0)
  {
    while(Serial.available()){
      inByte = Serial.read();
      
      switch(state){
        case fNONE:
          if(inByte==HDLC){
            state=fSTART; 
            outCount=0;
          }
          else if(inByte==0x2C){  //break HDLC in this way to scan I2C bus
                                  //and report addresses of present devices
                                  //comment out the line 'scan_i2c()' to disable
                                  //and return to pure HDLC
            outCount=0;
            scan_i2c(); 
          }
          break;
    
        case fSTART:
          if(inByte==HDLC){
            state=fNONE;
            newCommand=true;
          }
          else if(inByte==ESCAPE){
            state=fESCAPE;
          }
          else{
            i2cBuf[outCount]=inByte;
            ++outCount;
             toggleLED();
          }
          break;
    
        case fESCAPE:
          if(inByte==0x5E){
           i2cBuf[outCount]=0x7E;
           ++outCount;    
          }
          else if(inByte==0x5D){
           i2cBuf[outCount]=0x7D;
           ++outCount;
          } 
          state=fSTART;
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



void obey(){
  byte inByte;
  if(i2cBuf[0]&0x1){
//    Serial.println();
//    Serial.println("!!!");
//    Serial.println();
      //it's a read command
//      Wire.requestFrom(i2cBuf[0]>>1,outCount-1);
//      while(Wire.available()){
//       inByte=Wire.read();
//       //Serial.write(inByte); 
//      }
    }
    else{
      //it's a write command
      Wire.beginTransmission(i2cBuf[0]>>1);
      Wire.write(&i2cBuf[1], outCount-1);
      Wire.endTransmission(); 
    }
}

////////////////////////////
// SIGENICS FUNCTIONS ABOVE //
////////////////////////////


////////////////////////
// JOHN FUNCTIONS HERE//
/////////////////////////

void readAllEncoders(){
  for(int i=0;i<NUM_PENNY_BOARDS;i++){
    short values[NUM_PENNY_BOARDS];
    values[i] = readEncoderValue(pennyBoardAddresses[i]);
    //Serial.println(values[i]);
    byte buf[2];
    byte a = values[i];
    byte b = values[i] >> 8; // bitshift the encoder values variable into 2 bytes
    buf [1] = a; buf[0] = b; // fill buffer with bytes
    Serial.write(buf,2);
//    Serial.println(buf[1]);
//    Serial.println(buf[2]);
  }
}

short readEncoderValue(int pennyAddress){
  Wire.requestFrom(pennyAddress, 3);
  int msByte = Wire.read();
  int lsByte = Wire.read();
  int pAddr = Wire.read();
  short value = lsByte | (msByte<<8);
//  if(pAddr!=pennyAddress){
//    Serial.println(pAddr);
//    Serial.print("Something went wrong?");
//  }
  return value;
}

typedef struct data_pack_struct{
  short irValues[5];
  float pressureValues[5];
  short encoderValues[6];
}DataPack;

// CORRELL FUNCTIONS HERE //
///////////////////////////

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


float readPressure(int muxAddr, int sensor) { //does all pressure reading stuff for one sensor
  selectSensor(muxAddr, sensor);
  unsigned long D1 = getPressureReading(); //asks baro sensor for pressure value, waits 10ms, then returns the value.
//  Serial.println(D1);
  unsigned long D2 = getTempReading(); //asks baro sensor for temp value, waits 10ms, then returns the data
//  Serial.println(D2);

  //  unsigned int PRESS_SENS = Coff[0];
  //  unsigned int PRESS_OFFS = Coff[1];
  //  unsigned int TCPS = Coff[2];
  //  unsigned int TCPO = Coff[3];
  //  unsigned int TEMP_REF = Coff[4];
  //  unsigned int TCT = Coff[5];
  //  Serial.print(D1);
  //  Serial.print("\t");
  signed long dT = D2 - (Coff[4][sensor] << 8);
  signed long TEMP = 20000 + (((unsigned long long)dT) * Coff[5][sensor]) >> 23;
  signed long long OFF = Coff[1][sensor] << 17;// + (Coff[3]*((unsigned long long)dT))>>6;
  signed long long SENS = Coff[0][sensor] << 16;// + (Coff[2]*((unsigned long long)dT))>>7;
  float P = ((D1 * (SENS >> 21) - OFF)) >> 15;
  float mbar = P / 100.0;
//    Serial.print((mbar));
//    Serial.println();
  return (mbar);
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
  Wire.beginTransmission(muxID); //choose which MUX board to talk to
  Wire.write(1 << i); //identify which MUX port to listen from
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

void initIRSensor(int id)
{
  Wire.beginTransmission(id);
  Wire.write(0);
//  Serial.println("WIRE IN");
  int errcode = Wire.endTransmission();
//  Serial.println(errcode);

  // initialize each IR sensor
  for (int i = 0; i < NFINGERS; i++)
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
//            Serial.print("IR sensor failed to initialize: id = ");
//            Serial.print(i);
//            Serial.print(". ");
//            Serial.println(temp, HEX);
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

//unsigned int readIRValues(int id, int sensor) {
//  char buf[8];
//  //  Serial.print(id);
//  // read all 8 sensors on the strip
////  for (int i = 6; i < NUM_SENSORS; i += 2) // i should strt from 0
////  {
//    selectSensor(id, sensor);
//
//    //ambient_value_ = readAmbient();
//    unsigned int proximity_value_ = readProximity();
//
//    //sprintf(buf, "%6u", proximity_value_);
//    return (proximity_value_);
//    //return (buf);
//
////  }
////  Serial.print('\t');
//  //  Wire.beginTransmission(id);
//  //  Wire.write(0);
//  //  Wire.endTransmission();
//}


void readIRValues() {

  for (int i = 0; i < num_devices_; i++) {
     for (int j=0; j<NFINGERS; j++) {
    unsigned int prox_value = readIRValues_prox(i2c_ids_[i],sensor_IR_ports[j]);
    // Convert prox_value into an array of bytes for serial communication//
    byte buf[2];
    byte a = prox_value;
    byte b = prox_value >> 8; // bitshift the prox_value variable into 2 bytes
    buf [1] = a; buf[0] = b; // fill buffer with bytes
//    Serial.print("Finger:  ");Serial.println(sensor_IR_ports[j]);Serial.println(prox_value);
//    delay(100);
    
    Serial.write(buf,2);

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

void requestPressureData(int muxAddr, int sensorID){
  selectSensor(muxAddr, sensorID);
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
}

unsigned long pickupPressureData(int muxAddr, int sensorID){
  selectSensor(muxAddr, sensorID);
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


void readPressureValues() {

  for (int i = 0; i < num_devices_; i++) {
    for (int j=0; j<NFINGERS; j++) {
      
     float mbar = readPressure(i2c_ids_[i], sensor_Bar_ports[j]);

//    Serial.print("Fingertip Number:  ");
//    Serial.println(sensor_Bar_ports[j]);
//    Serial.println(mbar); // print pressure reading to serial port
//    delay(100);

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

    // Convert prox_value into an array of bytes for serial communication//
    byte buf2[2];
    byte a2 = mbar_int;
    byte b2 = mbar_int >> 8; // bitshift the prox_value variable into 2 bytes
    buf2[1] = a2; buf2[0] = b2; // fill buffer with bytes

    byte * b = (byte *) &mbar; //convert float to 4 byte array (https://forum.arduino.cc/index.php?topic=112597.0)

    Serial.write(b,4); // write pressure float variable to serial port
  }
  }
}

void sendFlag() {
  byte flag[3];
  flag[0] = 0x7E;flag[1] = 0x7E;flag[2] = 0x7E;
  Serial.write(flag,3); // write flag array to serial port to indicate start of data
}

// VOID LOOP HERE //
////////////////////

void loop(){
  lookForData();
  if(newCommand==true){
    obey();
    newCommand=false;
  }
//Serial.println("here");
  //delay(20);
  
///////////////////////
// CORRELL CODE HERE //
///////////////////////
  sendFlag(); // -> send flag of "7E7E7E" to laptop   
  readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
//  readAllEncoders(); //-> array of Encoder Values (2 bytes per penny board)
//  Serial.write(encoderValues);
//  //bundle up all three arrays and Serial.print()  
  delay(20);
}

