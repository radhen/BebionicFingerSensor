/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////

#include <Wire.h>
//#include "rp_testing.h"
#include <Smoothed.h> // available @ https://github.com/MattFryer/Smoothed
#include <Filters.h> // available @ https://github.com/JonHub/Filters
#include <MedianFilter.h> //available @ https://github.com/daPhoosa/MedianFilter
#include <QueueArray.h> // available @ https://playground.arduino.cc/Code/QueueArray/

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x0C  // MS5637_02BA03 I2C address is on the fingertip sensor pcb
#define VCNL4040_ADDR 0x1A // VCNL_4040 IR sensor I2C address is on the fingertip sensor pcb
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


/***** USER PARAMETERS *****/
int num_devices_;
unsigned int ambient_value_;
byte serialByte;
//uint16_t Coff[6][NUM_FINGERS];
//uint16_t Ti = 0, offi = 0, sensi = 0;
//int32_t data[3];
unsigned long Coff[6][NUM_FINGERS], Ti = 0, offi = 0, sensi = 0;
unsigned int data[3];

volatile int32_t pressure_value_[NUM_FINGERS];
volatile uint16_t proximity_value_[NUM_FINGERS];

//int32_t max_pressure[NUM_FINGERS] = {7800000.0, 6115000.0, 5950000.0, 6653000.0, 7000000.0};
//uint16_t max_proximity[NUM_FINGERS] = {40000.0, 35000.0, 30000.0, 17000.0, 25000.0};

int timer1_counter;

bool min_flag_ir = true;
bool min_flag_baro =  true;
int drop_count_ir = 50;
int drop_count_baro = 25;
volatile float smoothed_baro[NUM_FINGERS];
volatile float smoothed_baro_2[NUM_FINGERS];
volatile float press_nrm[NUM_FINGERS];
volatile float prox_nrm[NUM_FINGERS];
volatile float min_pressure[NUM_FINGERS];
volatile float min_distance[NUM_FINGERS];

volatile float prev_smoothed_baro[NUM_FINGERS];
volatile float next_smoothed_baro[NUM_FINGERS];
volatile float second_der;


////////////// Exponential Avg. variables for CONTACT detection/////////////////
// https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
volatile float highpass_proximity_value_[NUM_FINGERS] = {0.0};
volatile float EMA_a_ir[NUM_FINGERS] = {0.1};
volatile float EMA_S_ir[NUM_FINGERS] = {0.0};
//////////////////////////////////////////////////////////////////////////////////

/////////// moving avg. for smoothing ////////////////
Smoothed <float> smooth_ir;
Smoothed <float> smooth_baro;

RunningStatistics inputStats; // create statistics to look at the raw test signal
FilterTwoPole filterOneLowpass( INTEGRATOR, 0.5 );  // create a one pole (RC) highpass filter
//RunningStatistics filterOneLowpassStats; // create running statistics to smooth these values

MedianFilter median_filter(10, 0);


float EMA_a_low = 0.1;     //initialization of EMA alpha (cutoff-frequency)
float EMA_a_high = 1;

int EMA_S_low = 0;          //initialization of EMA S
int EMA_S_high = 0;

int highpass = 0;
int bandpass = 0;
int bandstop = 0;

//Queue<char> queue = Queue<char>(5); // Max 5 chars!
// create a queue of characters.
QueueArray <float> queue;

volatile float highpass_pressure_value[NUM_FINGERS] = {0.0};
volatile float EMA_a_baro[NUM_FINGERS] = {0.03};
volatile float EMA_S_baro[NUM_FINGERS] = {0.0};

bool first_slop_flag = true;
float first_min_value;



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


void writeByte(byte addr, byte val) {
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission(); //Release bus
}


void initPressure(int id) {
  //  byte dataLo, dataHi;

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
      //      dataHi = Wire.read();
      //      dataLo = Wire.read();
      data[0] = Wire.read();
      data[1] = Wire.read();
    }
    //      Coff[i][id] = ((dataHi << 8) | dataLo);
    Coff[i][id] = ((data[0] * 256) + data[1]);
    //    Serial.print(Coff[i][id]); Serial.print('\t');
  }
  //  Serial.print('\n');
  delay(300);
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(byte(0));
  Wire.endTransmission();
}


unsigned long getPressureReading(int id) {
  //  selectSensor(muxAddr, sensor);
  //  selectSensor(fingers[id].barPort);

//  Wire.beginTransmission(BARO_ADDRESS);
//  Wire.write(byte(0));
//  int errcode = Wire.endTransmission();

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x1E); // Send reset command
  Wire.endTransmission(); // Stop I2C Transmission

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(0x40); // Refresh pressure with the OSR = 256
  Wire.endTransmission(); // Stop I2C Transmission

//  delayMicroseconds(800); // 800 is optimal
  delay(500); // recommended by the example code

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


unsigned long getTemperatureReading(int id) {

  Wire.beginTransmission(BARO_ADDRESS);   // Start I2C Transmission
  Wire.write(0x50); // Refresh temperature with the OSR = 256
  Wire.endTransmission(); // Stop I2C Transmission

//  delayMicroseconds(800); // use this for speedy sampling
  delay(500); // 500ms delay recommended by the example code

  Wire.beginTransmission(BARO_ADDRESS); // Start I2C Transmission
  Wire.write(byte(0x00)); // Select data register
  Wire.endTransmission(); // Stop I2C Transmission

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
  return ((data[0] * 65536.0) + (data[1] * 256.0) + data[2]);
}

void press_temp_compensation() {
  /* This is recommended by the baro chip datasheet.
    Example code can be found here https://github.com/freetronics/BaroSensor/blob/master/BaroSensor.cpp
    and here https://github.com/ControlEverythingCommunity/MS5637-02BA03/blob/master/Arduino/MS5637_02BA03.ino
    For some reason the math below works for some sensors and for some gives the final pressure value as ZERO.
    This compensated pressure still has drift and hence I am back to using the raw sensor values.
  */

  for (int i = 0; i < NUM_FINGERS; i++) {
    unsigned long ptemp = getPressureReading(i);
//    Serial.print(ptemp); Serial.print('\t');
    unsigned long temp = getTemperatureReading(i);
    //    Serial.print(ptemp); Serial.print('\t'); Serial.println(temp);

    // Pressure and Temperature Calculations
    // 1st order temperature and pressure compensation
    // Difference between actual and reference temperature
    unsigned long long dT = temp - ((Coff[4][i] * 256.0));
    temp = 2000.0 + (dT * (Coff[5][i] / pow(2, 23)));

    // Offset and Sensitivity calculation
    unsigned long long off = Coff[1][i] * 131072.0 + (Coff[3][i] * dT) / 64.0;
    unsigned long long sens = Coff[0][i] * 65536.0 + (Coff[2][i] * dT) / 128.0;

    // 2nd order temperature and pressure compensation
    if (temp < 2000)
    {
      Ti = (dT * dT) / (pow(2, 31));
      offi = 5 * ((pow((temp - 2000), 2))) / 2;
      sensi =  offi / 2;
      if (temp < - 1500)
      {
        offi = offi + 7 * ((pow((temp + 1500), 2)));
        sensi = sensi + 11 * ((pow((temp + 1500), 2)));
      }
    }
    else if (temp >= 2000)
    {
      Ti = 0;
      offi = 0;
      sensi = 0;
    }

    // Adjust temp, off, sens based on 2nd order compensation
    temp -= Ti;
    off -= offi;
    sens -= sensi;
//    Serial.print(float(off)); Serial.print('\t');

    // Convert the final data
    //    Serial.print(ptemp); Serial.print('\t');
    ptemp =  (((ptemp * sens) / 2097152) - off) ;
//    Serial.print(ptemp);
    ptemp /= 32768.0;
    float pressure = ptemp / 100.0;
    float ctemp = temp / 100.0;
    float fTemp = ctemp * 1.8 + 32.0;

    Serial.println(pressure, 6);
  }
}

void readPressureValues() {

  if (drop_count_baro > 0) {
    // Drop first five values from all the sensors
    drop_count_baro -= 1;
    for (int i = 0; i < NUM_FINGERS; i++) {
      pressure_value_[i] = getPressureReading(i);
      EMA_S_low = pressure_value_[i];        //set EMA S for t=1
      EMA_S_high = pressure_value_[i];
      prev_smoothed_baro[i] = pressure_value_[i];
      next_smoothed_baro[i] = pressure_value_[i];
      queue.enqueue(pressure_value_[i]);
    }
  }

  for (int i = 0; i < NUM_FINGERS; i++) {

    pressure_value_[i] = getPressureReading(i); // get just the 24-bit raw pressure values


    //********* Median filter to remove the noise ************//
    median_filter.in(int(pressure_value_[i]));
    pressure_value_[i] = float(median_filter.out());
    Serial.print(pressure_value_[i]/5950000.0, 6); Serial.print('\t');


    //******** Moving avg. to smooth the signal ********//
    smooth_baro.add(pressure_value_[i]);
    smoothed_baro[i] = smooth_baro.get(); // Get the smoothed values
//    Serial.print(smoothed_baro[i]); Serial.print('\t');
    // Serial.print(smoothed_baro/50000.0, 6); Serial.print('\t'); //Found the max value 50000.0 by manually pressing the sensor

    //*********** calculating second derivative *************//
    second_der = float(smoothed_baro[i] + next_smoothed_baro[i] - 2.0 * prev_smoothed_baro[i]) / float(0.01);
    //    Serial.println(second_der);
    next_smoothed_baro[i] = prev_smoothed_baro[i];
    prev_smoothed_baro[i] = smoothed_baro[i];

    //******** Running statistics. Calc mean var stddev ********//
    inputStats.input(smoothed_baro[i]);
//        Serial.print(inputStats.mean());
//        Serial.println(inputStats.variance());
    //    Serial.println();

    //************ low pass filter ****************//
    filterOneLowpass.input( smoothed_baro[i] );
//    Serial.print(filterOneLowpass.output()); Serial.println('\t');

    //******** Exponential average for more smoothing ********//
    EMA_S_baro[i] = (EMA_a_baro[i] * smoothed_baro[i]) + ((1.0 - EMA_a_baro[i]) * EMA_S_baro[i]);
    //      highpass_pressure_value[i] = pressure_value_[i] - EMA_S_baro[i];
    //      Serial.print(EMA_S_baro[i]); Serial.println('\t');

    queue.enqueue(EMA_S_baro[i]);
    float y0 = queue.dequeue();
    //    Serial.println(queue.count());  \\check the size of the queue. Should be equal to drop_baro_count variable

    //****** calculate the dy/dx on the smoothed signal ******//
    float slope = atan2((EMA_S_baro[i] - y0),900); // in radians. 100 in the denominator is delta x which is set experimentally

    //****** integrate the signal ******//
    float y1 = 75.0 * ((EMA_S_baro[i] + y0) / 2.0); // seems like another smoothing filter to me
//    Serial.println(y1);

//        Serial.print(1.57);  // To freeze the lower limit
//        Serial.print(" ");
//        Serial.print(0.349066);  // To freeze the lower limit
//        Serial.print(" ");
//        Serial.print(-1.57);  // To freeze the upper limit
//        Serial.print(" ");
//        Serial.print(-0.349066);  // To freeze the lower limit
//        Serial.print(" ");
//        Serial.println(slope);
    
//        if(slope >= 0.349066){
//    //      Serial.print(smoothed_baro[i]); Serial.print('\t');
//          if (first_slop_flag == true){
//            first_min_value = smoothed_baro[i];
//    //        Serial.println(first_min_value);
//            first_slop_flag = false;
//            }
//    //      press_nrm[i] = map(smoothed_baro[i], first_min_value, first_min_value+17000.0, 0.0, 1.0);
//          press_nrm[i] = (abs(smoothed_baro[i] - first_min_value))/(17000.0);
//          Serial.println(press_nrm[i]);
//          }
//        else{
//          Serial.println(0.0);
//          first_slop_flag = true;
//          }


    //**************** band stop filter ***************//
    //    EMA_S_low = (EMA_a_low * smoothed_baro[i]) + ((1 - EMA_a_low) * EMA_S_low);    //run the EMA
    //    EMA_S_high = (EMA_a_high * smoothed_baro[i]) + ((1 - EMA_a_high) * EMA_S_high);
    //    bandpass = EMA_S_high - EMA_S_low;        //find the band-pass as before
    //    bandstop = smoothed_baro[i] - bandpass;        //find the band-stop signal
    //    Serial.print(bandstop); Serial.print('\t');


    //*********** NORMALIZE BARO SENSOR VALUES ??? NOTHING WORKS ************//
    // keep track of the running min values
    //    if (min_flag_baro == true) {
    //      min_pressure[i] = smoothed_baro[i];
    //      //      Serial.print(min_pressure[i]); Serial.print('\t');
    //    }
    //    if (smoothed_baro[i] < min_pressure[i]) {
    //      if (smoothed_baro[i] == 0) {
    //        // dicarding the anomaly
    //        // do nothing
    //      }
    //      else {
    //        min_pressure[i] = smoothed_baro[i];
    //      }
    //    }

    //    press_nrm[i] = smoothed_baro[i] - min_pressure[i];
    //Serial.print(press_nrm[i], 6); Serial.print('\t');

  }

  min_flag_baro = false;

    press_temp_compensation();

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

    //******** Exponential average for Contact detection. Losspass filter and then subtract the orig. singal ********//
    EMA_S_ir[i] = (EMA_a_ir[i] * prox_nrm[i]) + ((1.0 - EMA_a_ir[i]) * EMA_S_ir[i]);
    highpass_proximity_value_[i] = prox_nrm[i] - EMA_S_ir[i];
//    Serial.print(highpass_proximity_value_[i], 6); Serial.print('\t');

    //******** Moving avg. ********//
    smooth_ir.add(proximity_value_[i]);
    // Get the smoothed values
    float smoothed_ir = smooth_ir.get();
//    Serial.print(smoothed_ir); Serial.print('\t');
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
  smooth_ir.begin(SMOOTHED_AVERAGE, 50);
  smooth_baro.begin(SMOOTHED_AVERAGE, 75);

  inputStats.setWindowSecs( 0.05 );
  //  filterOneLowpass.setWindowSecs( 0.1 );


}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

  //  digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope

      unsigned long start, elapsed;

  //    lookForData();
  //    if (newCommand == true) {
  //      obey();
  //      newCommand = false;
  //    }

//    start = micros();

     readIRValues(); //-> array of IR values (2 bytes per sensor)
  readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)

//    elapsed = micros() - start; // find elastime time to read sensor data
//    Serial.println(1000000.0 / float(elapsed)); // print sampling frequency

  //  readNNpredictions();


    Serial.print('\n');

}
