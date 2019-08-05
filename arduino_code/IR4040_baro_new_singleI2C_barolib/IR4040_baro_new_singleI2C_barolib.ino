/////////////////////////////////////////////////////////
///////////// PCF SENSOR INIT CODE BELOW ///////////////
////////////////////////////////////////////////////////

#include <Wire.h>
//#include "rp_testing.h"
#include <Smoothed.h> // available @ https://github.com/MattFryer/Smoothed
#include <Filters.h> // available @ https://github.com/JonHub/Filters
#include <MedianFilter.h> //available @ https://github.com/daPhoosa/MedianFilter
#include "BaroSensor.h" // available @ https://github.com/freetronics/BaroSensor
#include "curveFitting.h" // available @ https://github.com/Rotario/arduinoCurveFitting
#include <CircularBuffer.h> // available @ https://github.com/rlogiacco/CircularBuffer

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x63  // MS5637_02BA03 I2C address is on the fingertip sensor pcb
#define VCNL4040_ADDR 0x75 // VCNL_4040 IR sensor I2C address is on the fingertip sensor pcb
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

// SAMPLING FREQ. STUFF.
#define SAMPLING_INTERVAL 10000 // for eg. 10000 microseconds per sample for 100 Hz, change this for different sampling rates.
unsigned long lastMicros = 0;

unsigned long tic, toc;

/***** USER PARAMETERS *****/
int num_devices_;
unsigned int ambient_value_;
byte serialByte;
//uint16_t Coff[6][NUM_FINGERS];
//uint16_t Ti = 0, offi = 0, sensi = 0;
//int32_t data[3];
unsigned long Coff[6][NUM_FINGERS], Ti = 0, offi = 0, sensi = 0;
unsigned int data[3];

float pressure_value_[NUM_FINGERS];
volatile uint16_t proximity_value_[NUM_FINGERS];

//int32_t max_pressure[NUM_FINGERS] = {7800000.0, 6115000.0, 5950000.0, 6653000.0, 7000000.0};
//uint16_t max_proximity[NUM_FINGERS] = {40000.0, 35000.0, 30000.0, 17000.0, 25000.0};

int timer1_counter;

bool min_flag_ir = true;
bool min_flag_baro =  true;

const int drop_count_ir = 500;
float ir_avg[drop_count_ir]; 
float IR_AVG = 0.0;
float sum = 0.0;

int drop_count_baro = 500;

volatile float smoothed_baro[NUM_FINGERS];
volatile float smoothed_baro_2[NUM_FINGERS];
volatile float press_nrm[NUM_FINGERS];
volatile float prox_nrm[NUM_FINGERS];
volatile float min_pressure[NUM_FINGERS];
volatile float min_distance[NUM_FINGERS];

volatile float prev_smoothed_baro[NUM_FINGERS] = {0.0};
volatile float next_smoothed_baro[NUM_FINGERS];
volatile float second_der;


////////////// Exponential Avg. variables for CONTACT detection/////////////////
// https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
volatile float highpass_proximity_value_[NUM_FINGERS] = {0.0};
volatile float EMA_a_ir[NUM_FINGERS] = {0.075};
volatile float EMA_S_ir[NUM_FINGERS] = {0.0};
//////////////////////////////////////////////////////////////////////////////////

/////////// moving avg. for smoothing ////////////////
Smoothed <float> smooth_ir;
Smoothed <float> smooth_baro;

RunningStatistics inputStats; // create statistics to look at the raw test signal
FilterOnePole filterOneLowpass( LOWPASS, 2.0 );  // create a one pole (RC) highpass filter
//RunningStatistics filterOneLowpassStats; // create running statistics to smooth these values
FilterOnePole filterOneHighpass( HIGHPASS, 4.0 );  // create a one pole (RC) highpass filter

MedianFilter median_filter(20, 0);

float integrate = 0.0;

// source bandstop filter: https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/
float EMA_a_low = 0.001;     //initialization of EMA alpha (cutoff-frequency) 0Hz
float EMA_a_high = 0.1; // 2.2Hz
int EMA_S_low = 0;          //initialization of EMA S
int EMA_S_high = 0;
int highpass = 0;
int bandpass = 0;
int bandstop = 0;


volatile float highpass_pressure_value[NUM_FINGERS] = {0.0};
volatile float EMA_a_baro[NUM_FINGERS] = {0.05};
volatile float EMA_S_baro[NUM_FINGERS] = {0.0};

bool first_slop_flag = true;
float first_min_value;

float current_mean = 0.0;
bool idle_flag = true;
bool contact_flag = false;


// Derivate and Integrate variables
const int DERI_LEN = 10;
const int INTE_LEN = 10;
const int DOUBLE_DERI_LEN = 10;
const int DOUBLE_INTE_LEN = 10;
float deri;
float inte;
float double_deri;
float double_inte;
CircularBuffer<float, DERI_LEN> deri_buffer;
CircularBuffer<float, INTE_LEN> inte_buffer;
CircularBuffer<float, DOUBLE_DERI_LEN> double_deri_buffer;
CircularBuffer<float, DOUBLE_INTE_LEN> double_inte_buffer;


// CURVE FITTING VARIABLES
const int POLY_LEN = 30;
const int ORDER = 2;
double x[POLY_LEN];
double t[POLY_LEN];
double coeffs[ORDER + 1];

const float THRESHOLD = 50.0 ;
int running_mean;

int counter = 0;

volatile float smoothed_ir;
int flag_counter = 0;


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


void initPressure(int id) {
  BaroSensor.begin(BARO_ADDRESS);
}


void readPressureValues() {

  if (drop_count_baro > 0) {
    // Drop first five values from all the sensors to fill up variables. Should be in the setup
    drop_count_baro -= 1;
    for (int i = 0; i < NUM_FINGERS; i++) {
      pressure_value_[i] = BaroSensor.getPressure(OSR_256, BARO_ADDRESS);
      filterOneLowpass.input( pressure_value_[i] );
      smoothed_baro[i] = filterOneLowpass.output();
      
      prev_smoothed_baro[i] = pressure_value_[i];
      next_smoothed_baro[i] = pressure_value_[i];
//            smooth_baro.add(pressure_value_[i]);
      //      smoothed_baro[i] = smooth_baro.get();
      inputStats.input(smoothed_baro[i]);
      current_mean = inputStats.mean();
      inte_buffer.push(0.0);
      EMA_S_low = (EMA_a_low * pressure_value_[i]) + ((1 - EMA_a_low) * EMA_S_low);    //run the EMA
    EMA_S_high = (EMA_a_high * pressure_value_[i]) + ((1 - EMA_a_high) * EMA_S_high);
    bandpass = EMA_S_high - EMA_S_low;        //find the band-pass as before
    bandstop = pressure_value_[i] - bandpass;
      deri_buffer.push(pressure_value_[i]);
    }
  }

  for (int i = 0; i < NUM_FINGERS; i++) {

    pressure_value_[i] = BaroSensor.getPressure(OSR_256, BARO_ADDRESS); // get just the 24-bit raw pressure values
//    Serial.print(int(pressure_value_[i])); Serial.print('\t');

     //**************** band stop filter ***************//
    EMA_S_low = (EMA_a_low * pressure_value_[i]) + ((1 - EMA_a_low) * EMA_S_low);    //run the EMA
    EMA_S_high = (EMA_a_high * pressure_value_[i]) + ((1 - EMA_a_high) * EMA_S_high);
    bandpass = EMA_S_high - EMA_S_low;        //find the band-pass as before
    bandstop = pressure_value_[i] - bandpass;        //find the band-stop signal
//    Serial.print(bandpass); Serial.print('\t');


      filterOneHighpass.input(pressure_value_[i]);
//      smoothed_baro[i] = filterOneHighpass.output();
//      Serial.print(filterOneHighpass.output()); Serial.print('\t');


//    inputStats.input(pressure_value_[i]);
////    Serial.print(inputStats.mean()); Serial.print('\t');
////    Serial.println(inputStats.variance());
//    if(inputStats.variance() < 60000000.0){
//      median_filter.in(0);
//        }
//    else{
//        median_filter.in(int(pressure_value_[i]));
//      }


    //********* Median filter to remove the noise ************//
//        median_filter.in(int(pressure_value_[i]));
//        pressure_value_[i] = float(median_filter.out());
//        Serial.print(median_filter.out()); Serial.print('\t');


    //******** Moving avg. to smooth the signal ********//
//    smooth_baro.add(pressure_value_[i]);
//    smoothed_baro[i] = smooth_baro.get(); // Get the smoothed values
//    Serial.print(smoothed_baro[i]); Serial.print('\t');


    //    Serial.print(BaroSensor.getTemperature(CELSIUS, OSR_256, BARO_ADDRESS)); Serial.print('\t');


    //    //*********** calculating second derivative *************//
    //    second_der = float(smoothed_baro[i] + next_smoothed_baro[i] - 2.0 * prev_smoothed_baro[i]) / float(0.01);
    //    //    Serial.println(second_der);
    //    next_smoothed_baro[i] = prev_smoothed_baro[i];
    //    prev_smoothed_baro[i] = smoothed_baro[i];


    //******** Running statistics. Calc mean var stddev ********//
    //      inputStats.input(smoothed_baro[i]);
    //    Serial.print(inputStats.mean()); Serial.print('\t');
    //      Serial.println(inputStats.variance());

    //    if(smoothed_baro[i] < 1.01*(current_mean+8.0)){
    //      idle_flag = true;
    //      contact_flag = false;
    //      Serial.println(0.0);
    //      current_mean = inputStats.mean();
    //      }
    //    else{
    //      idle_flag = false;
    //      contact_flag = true;
    //      }
    //    if(contact_flag == true){
    //      Serial.println(smoothed_baro[i] - current_mean);
    //      }


  

       //************ low pass filter ****************//
//      filterOneLowpass.input( pressure_value_[i] );
//      smoothed_baro[i] = filterOneLowpass.output();
//      Serial.print(smoothed_baro[i]); Serial.print('\t');

//      filterOneLowpass.input( pressure_value_[i] );
//      smoothed_baro[i] = filterOneLowpass.output();
//      Serial.print(smoothed_baro[i]);/ Serial.print('\t');



    //******** Exponential average for more smoothing and the sub to get highpass response ********//
//        EMA_S_baro[i] = (EMA_a_baro[i] * smoothed_baro[i]) + ((1.0 - EMA_a_baro[i]) * EMA_S_baro[i]);
//        highpass_pressure_value[i] = smoothed_baro[i] - EMA_S_baro[i];
//        Serial.print(EMA_S_baro[i]); Serial.println('\t');



    //************* CURVE FITTING *************//
//        for(int i=0; i<=POLY_LEN; i++){
//        x[i] = deri_buffer[i];}
//        for (int i = 0; i < sizeof(x)/sizeof(double); i++){
//        t[i] = i;}
//        int ret = fitCurve(ORDER, sizeof(x)/sizeof(double), t, x, sizeof(coeffs)/sizeof(double), coeffs);
//        if (ret == 0){ //Returned value is 0 if no error
//        uint8_t c = 'a';
////        Serial.println("Coefficients are");
////        for (int i = 0; i < sizeof(coeffs)/sizeof(double); i++){
////          Serial.printf("%c=%f\t ",c++, coeffs[i]);
////            }
//        }
////      Serial.print(abs(smoothed_baro[i] - coeffs[ORDER+1])); Serial.print('\t');

//Serial.print(smoothed_ir); Serial.print('\t');

    //********** normalizing based on contact detection event ***********//
        if(smoothed_ir > IR_AVG + 3500.0){
          if (first_slop_flag == true){
              first_min_value = pressure_value_[i];
  //                Serial.println(first_min_value);
              first_slop_flag = false;
            }
          
          press_nrm[i] = pressure_value_[i] - first_min_value; 
          press_nrm[i] = constrain(press_nrm[i],0,1000000000000.0); // neglect -ve values
          press_nrm[i] = (press_nrm[i]) / (16678896.0 - first_min_value);
//          Serial.print(press_nrm[i]); Serial.print('\t');   

          if(press_nrm[i] > 1.03 * first_min_value & flag_counter == 0){
            flag_counter = 1;
            }
          if(press_nrm[i] < 1.03 * first_min_value & flag_counter == 1){
            press_nrm[i] = 0.0;
            flag_counter = 0;
            }
        }
        else{
          press_nrm[i] = 0.0;
          first_slop_flag = true;
        }

        Serial.print(press_nrm[i]); Serial.print('\t');

//        Serial.print(0);  // To freeze the upper limit
//        Serial.print(" ");
//        Serial.print(1);  // To freeze the lower limit
//        Serial.print(" ");
        
          
        
          


    //****** dy/dx on the smoothed signal ******//
    deri_buffer.push(pressure_value_[i]);
//    float last_baro = deri_buffer.shift(); 
    float slope = atan2((deri_buffer.last() - deri_buffer.first()),0.01*DERI_LEN); // in radians. 100 in the denominator is delta x which is set experimentally
    deri = (deri_buffer.last() - deri_buffer.first()) / 10.0; // ((1/float(SAMPLING_INTERVAL))*DERI_LEN*1000000.0)
//    Serial.print(deri); Serial.print('\t');
//    Serial.print(deri_buffer.first()); Serial.print('\t');
//    Serial.print(deri_buffer.last()); Serial.print('\t');

    double_deri_buffer.push(deri);
    double_deri = (double_deri_buffer.last() - double_deri_buffer.first()) / 10.0; // ((1/float(SAMPLING_INTERVAL))*DERI_LEN*1000000.0)
//    Serial.print(double_deri); Serial.print('\t');


    if(deri > -THRESHOLD & deri < THRESHOLD){
          deri = 0.0;
          }
//        else{
//          if(deri > THRESHOLD){
//            deri = deri - THRESHOLD;  
//            }
//          if(deri < -THRESHOLD){
//            deri = deri + THRESHOLD;
//            }
//        }
//        Serial.print(deri); Serial.print('\t');

  

//    median_filter.in(int(deri));
//    Serial.print(median_filter.out()); Serial.print('\t');


    //    //****** integrate the signal ******//
    inte_buffer.push(filterOneHighpass.output());
    inte += (((1/float(SAMPLING_INTERVAL))*DERI_LEN*1000000.0) * (inte_buffer.first() + inte_buffer.last()) * 0.5); // delta_x * delta_y * 0.5
//    Serial.print(int(inte)); Serial.print('\t');


//    inputStats.input(pressure_value_[i]);
////    Serial.print(inputStats.mean()); Serial.print('\t');
////    Serial.println(inputStats.variance());
//    if(inputStats.variance() < 60000000.0){
//      median_filter.in(0);
//        }
//    else{
//      inte_buffer.push(deri);
//      inte += (((1/float(SAMPLING_INTERVAL))*DERI_LEN*1000000.0) * (inte_buffer.first() + inte_buffer.last()) * 0.5);
//      median_filter.in(int(inte));
//      }
//      Serial.print(median_filter.out()); Serial.print('\t');
    
//    if(inte > -3000000.0 & inte < 3000000.0){
//          inte = 0.0;
//          }

//    Serial.print(inte); Serial.println('\t');
//    Serial.print(inte_buffer.first()); Serial.print('\t');
//    Serial.print(inte_buffer.last()); Serial.print('\t');

//    double_inte_buffer.push(inte);
//    double_inte += (((1/float(SAMPLING_INTERVAL))*DERI_LEN*1000000.0) * (double_inte_buffer.first() + double_inte_buffer.last()) * 0.5); // delta_x * delta_y * 0.5
//    Serial.print(double_inte); Serial.println('\t');  




//    Serial.print(double_inte - inputStats.mean()); Serial.println('\t');  
  
//            Serial.print(0.0);  // To freeze the lower limit
//            Serial.print(" ");
//            Serial.print(2000000000.0);  // To freeze the lower limit
//            Serial.print(" ");

    //        Serial.println(integrate);
    //
    ////        if(slope >= 0.349066){
    ////    //      Serial.print(smoothed_baro[i]); Serial.print('\t');
    ////          if (first_slop_flag == true){
    ////            first_min_value = smoothed_baro[i];
    ////    //        Serial.println(first_min_value);
    ////            first_slop_flag = false;
    ////            }
    ////    //      press_nrm[i] = map(smoothed_baro[i], first_min_value, first_min_value+17000.0, 0.0, 1.0);
    ////          press_nrm[i] = (abs(smoothed_baro[i] - first_min_value))/(17000.0);
    ////          Serial.println(press_nrm[i]);
    ////          }
    ////        else{
    ////          Serial.println(0.0);
    ////          first_slop_flag = true;
    ////          }


    //************** DC BLOCKER. Similar to HIGH PASS FILTER: https://ccrma.stanford.edu/~jos/fp/DC_Blocker.html ****************//
    //************** Available here https://ccrma.stanford.edu/~jos/fp/DC_Blocker.html and here https://www.dsprelated.com/freebooks/filters/DC_Blocker.html****************//
//    EMA_S_baro[i] = integrate - prev_smoothed_baro[i] + 0.998 * EMA_S_baro[i];
    //    Serial.print(EMA_S_baro[i]); Serial.println('\t');
//    prev_smoothed_baro[i] = integrate;

   


    //*********** NORMALIZE BARO SENSOR VALUES ??? NOTHING WORKS ************//
    // keep track of the running min values
    //      if (min_flag_baro == true) {
    //        min_pressure[i] = smoothed_baro[i];
    //        //      Serial.print(min_pressure[i]); Serial.print('\t');
    //      }
    //      if (smoothed_baro[i] < min_pressure[i]) {
    //          min_pressure[i] = smoothed_baro[i];
    //      }
    //    press_nrm[i] = smoothed_baro[i] - min_pressure[i];
    //    Serial.print(press_nrm[i], 6); Serial.print('\t');

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

  

  for (int i = 0; i < NUM_FINGERS; i++) {
    proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
//    Serial.print(proximity_value_[i]); Serial.print('\t');


    //*********** NORMALIZE IR SENSOR VALUES ************//
    // keep track of the running min values
//    if (min_flag_ir == true) {
//      min_distance[i] = proximity_value_[i];
//    }
//    if (proximity_value_[i] < min_distance[i]) {
//      min_distance[i] = proximity_value_[i];
//    }

//    prox_nrm[i] = float(proximity_value_[i] - min_distance[i]);
    //      Serial.print(prox_nrm[i]); Serial.print('\t');
    

    //******** Exponential average for Contact detection. Losspass filter and then subtract the orig. singal ********//
    EMA_S_ir[i] = (EMA_a_ir[i] * proximity_value_[i]) + ((1.0 - EMA_a_ir[i]) * EMA_S_ir[i]);
    highpass_proximity_value_[i] = proximity_value_[i] - EMA_S_ir[i];
//    Serial.print(highpass_proximity_value_[i], 6); Serial.print('\t');
    

    //******** Moving avg. ********//
    smooth_ir.add(proximity_value_[i]);
    // Get the smoothed values
    smoothed_ir = smooth_ir.get();
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


//    while (!Serial)
//      {
//      }
//  
//    Serial.println ();
//    Serial.println ("I2C scanner. Scanning ...");
//    byte count = 0;
//  
//    Wire.begin();
//    for (byte i = 8; i < 120; i++)
//    {
//      Wire.beginTransmission (i);
//      if (Wire.endTransmission () == 0)
//        {
//        Serial.print ("Found address: ");
//        Serial.print (i, DEC);
//        Serial.print (" (0x");
//        Serial.print (i, HEX);
//        Serial.println (")");
//        count++;
//        delay (1);  // maybe unneeded?
//        } // end of good response
//    } // end of for loop
//    Serial.println ("Done.");
//    Serial.print ("Found ");
//    Serial.print (count, DEC);
//    Serial.println (" device(s).");

  // moving avg. initalization
  smooth_ir.begin(SMOOTHED_AVERAGE, 10);
  smooth_baro.begin(SMOOTHED_AVERAGE, 10);

  inputStats.setWindowSecs( 0.3 );
  //  filterOneLowpass.setWindowSecs( 0.1 );

  float initial_pressure_value = BaroSensor.getPressure(OSR_256, BARO_ADDRESS);
  EMA_S_low = initial_pressure_value;        //set EMA S for t=1
  EMA_S_high = initial_pressure_value;


// Get the avg of first few samples of IR signal
  for(int j = 0; j < drop_count_ir; j++) {
    for (int i = 0; i < NUM_FINGERS; i++) {
      proximity_value_[i] = readFromCommandRegister(PS_DATA_L);
      ir_avg[j] = proximity_value_[i];
    }
  }

  for(int i = 0; i < drop_count_ir; i++) {
      sum = sum + ir_avg[i];
   }
   IR_AVG = (float)sum / drop_count_ir;
//   Serial.print(IR_AVG);

}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

  //  digitalWrite(13, !digitalRead(13)); // to measure samp. frq. using oscilloscope


  if (micros() - lastMicros > SAMPLING_INTERVAL) {
    lastMicros = micros(); // do this first or your interval is too long!

    readIRValues(); //-> array of IR values (2 bytes per sensor)
    readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)

    Serial.print('\n');
  }


//***** CALCULATE SAMPLING FREQ. BASED ON NORDIC CLOCK ****//   
// WITH THIS BARO ALONE IS 146Hz and IR IS AROUND 1kHz. COMBINED AS EXPECTED IS 146Hz
// SETTING THE SAMPLING RATE  TO 100Hz (SAMPLING_INTERVAL = 10000 microseconds)


//    unsigned long start, elapsed;  
//    start = micros();
//
//    readIRValues(); //-> array of IR values (2 bytes per sensor)
//    readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)
//
//    elapsed = micros() - start; // find elastime time to read sensor data
//    long freq = 1000000L/elapsed;
//    Serial.println(freq); // print sampling frequency

}
