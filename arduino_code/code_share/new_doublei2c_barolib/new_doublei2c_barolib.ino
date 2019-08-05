#include <Wire.h>
#include <bluefruit.h>
BLEUart bleuart; // uart over ble
#include "BaroSensor.h" // available @ https://github.com/freetronics/BaroSensor


//***** Simultaneous use of two i2c ports ********//
//***** https://github.com/espressif/arduino-esp32/issues/977 ********//
#define SDA_1 8 // these are pin numbers on the MDBT50Q chip on the Sparkfun Pro nrf52840 Mini
#define SCL_1 11 // -------------- " -------------------- " ------------------- " -------------
#define SDA_2 6 // -------------- " -------------------- " ------------------- " -------------
#define SCL_2 9 // -------------- " -------------------- " ------------------- " -------------

#define OUTPUT_I2C_ADDRESS 0x08

#define I2C_OUT_ENABLED false
#define BLUETOOTH_ENABLED false

// TwoWire definition from the Wire_nrf52.cpp file
//TwoWire I2C_in(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SDA_1, SCL_1);
TwoWire I2C_in(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_1, SCL_1);
#if(I2C_OUT_ENABLED)
TwoWire I2C_out(NRF_TWIM0, NRF_TWIS0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SDA_2, SCL_2);
//TwoWire I2C_out(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA_2, SCL_2);
#endif

#define LED_PIN 7


/***** GLOBAL CONSTANTS *****/
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

#define NUM_FINGERS 5 // number of fingers connected
#define PRESS_MEAS_DELAY_MS 20 //duration of each pressure measurement is twice this.

typedef struct {
  byte baroAddr;
  byte irAddr;
} Digit;

//Each finger is a pair of ports (as read off of the mux board. Should all be between 0 and 15).
//first number is the barometer add., second number is the ir add. (baroAddr, irAddr).
// refer translatedAddress.xlsx file to know address of ir and baro for a sensor with specific address
// OR use scan_i2c_in() function to scan the connected i2c devices
Digit fingers[NUM_FINGERS] = {{0x0E,0x18},{0x0C,0x1A}, {0x63,0x75}, {0x65,0x73}, {0x2F, 0x39}}; //  

bool light_on;
unsigned long last_light_switch;
#define BLINKY_LIGHT_PERIOD_MS 500

volatile int32_t pressure_value_[NUM_FINGERS];
volatile uint16_t proximity_value_[NUM_FINGERS];

//int32_t max_pressure[NUM_FINGERS] = {7800000.0, 6115000.0, 5950000.0, 6653000.0, 7000000.0};
//uint16_t max_proximity[NUM_FINGERS] = {40000.0, 35000.0, 30000.0, 17000.0, 25000.0};

int timer1_counter;

// SAMPLING FREQ. STUFF.
#define SAMPLING_INTERVAL 10000 // for eg. 10000 microseconds per sample for 100 Hz, change this for different sampling rates.
unsigned long lastMicros = 0;

bool toggle = false;

///////////////////////////////////////////////////////////
///////////// MISC FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////


void scan_i2c_in(){
  Serial.println ("I2C scanner. Scanning ...");
  byte device_count = 0;
  I2C_in.begin();
  for (byte i = 0; i < 128; i++){
    I2C_in.beginTransmission (i);
    if (I2C_in.endTransmission () == 0) {
      Serial.print ("Found address: 0x");
      if(i<16){
        Serial.print("0");
      }
      Serial.print (i, HEX);
      Serial.println ("");
      device_count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (device_count, DEC);
  Serial.println (" device(s).");
}


#if(I2C_OUT_ENABLED)
void scan_i2c_out(){
Serial.println("Scanning I2C Addresses Channel 2");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  I2C_out.beginTransmission(i);
  uint8_t ec=I2C_out.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}
#endif


void toggleLED(void) {
  toggle = !toggle;
  if (toggle) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
}


#if(BLUETOOTH_ENABLED)
void bluetooth_init(){
  Initialize Bluetooth:
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Centerboard");
  bleuart.begin();

  // Start advertising device and bleuart services
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  // Set advertising interval (in unit of 0.625ms):
  Bluefruit.Advertising.setInterval(32, 244);
  // number of seconds in fast mode:
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);  
}
#endif


void toggle_light(){
      last_light_switch = millis();
    digitalWrite(LED_PIN, light_on ? LOW : HIGH);
    light_on = !light_on;
}


//void transmitData(byte address){
//      I2C_out.beginTransmission(address);
//  for(int i=0;i<NUM_FINGERS;i++){
//
//    I2C_out.write(proximity_value_[i]&0xFF);
//    I2C_out.write((proximity_value_[i]>>8)&0xFF);
//  }
//  for(int i=0;i<NUM_FINGERS;i++){
//    uint32_t tmp = pressure_value_[i];
//    I2C_out.write((tmp)&0xFF);
//    I2C_out.write((tmp>>8)&0xFF);
//    I2C_out.write((tmp>>16)&0xFF);
//    I2C_out.write((tmp>>24)&0xFF);
//  }
//  I2C_out.endTransmission();
//}


///////////////////////////////////////////////////////////
///////////// PCF SENSOR FUNCTIONS BELOW ///////////////
//////////////////////////////////////////////////////////

void initPressure(byte address, int i) {
  BaroSensor.begin(address, i);
}

void readPressureValues() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    const byte baro_address = fingers[i].baroAddr;
    pressure_value_[i] = BaroSensor.getPressure(OSR_256, baro_address, i);
    Serial.print(pressure_value_[i]); Serial.print('\t');

//    #if(I2C_OUT_ENABLED)
//      I2C_out.beginTransmission(OUTPUT_I2C_ADDRESS);
//      I2C_out.write(pressure_value_[i]); I2C_out.write('\t');
//      I2C_out.endTransmission();
//    #endif
    

    #if(BLUETOOTH_ENABLED)
      if(bleuart.available()){
        bleuart.write(pressure_value_[i]); bleuart.write('\t');
      }
    #endif
  }

}

// Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte address, byte commandCode)
{
  I2C_in.beginTransmission(address);
  I2C_in.write(commandCode);
  int err = I2C_in.endTransmission(false); //Send a restart command. Do not release bus.
//  Serial.println(err);
  I2C_in.requestFrom(address, 2); //Command codes have two bytes stored in them

  unsigned int data = I2C_in.read();
  data |= I2C_in.read() << 8;

  return (data);
}

void writeToCommandRegister(byte address, byte commandCode, byte lowVal, byte highVal)
{
  I2C_in.beginTransmission(address);
  I2C_in.write(commandCode);
  I2C_in.write(lowVal); //Low byte of command
  I2C_in.write(highVal); //High byte of command
  I2C_in.endTransmission(); //Release bus
}


void initVCNL4040(byte address) {
  //Clear PS_SD to turn on proximity sensing
  //byte conf1 = 0b00000000; //Clear PS_SD bit to begin reading
  byte conf1 = 0b00001110; //Integrate 8T, Clear PS_SD bit to begin reading
  byte conf2 = 0b00001000; //Set PS to 16-bit
  //byte conf2 = 0b00000000; //Clear PS to 12-bit
  writeToCommandRegister(address, PS_CONF1, conf1, conf2); //Command register, low byte, high byte

  //Set the options for PS_CONF3 and PS_MS bytes
  byte conf3 = 0x00;
  //byte ms = 0b00000010; //Set IR LED current to 100mA
  //byte ms = 0b00000110; //Set IR LED current to 180mA
  byte ms = 0b00000111; //Set IR LED current to 200mA
  writeToCommandRegister(address, PS_CONF3, conf3, ms);
}


void initIRSensor(byte address) {

  //  selectSensor(fingers[id].irPort);
  I2C_in.beginTransmission(address);
  I2C_in.write(byte(0));
  int errcode = I2C_in.endTransmission();

  int deviceID = readFromCommandRegister(address, ID);
//  Serial.println(deviceID);
const int response_code = 0x186;
  if (deviceID != response_code)
  {
    Serial.println("Device not found. Check wiring.");
    Serial.print("Expected: 0x186. Heard: 0x");
    Serial.println(deviceID, HEX);
//    while (1); //Freeze!
  }
//  Serial.println("VCNL4040 detected!");
  initVCNL4040(address); //Configure sensor

  //    delay(50);
  I2C_in.beginTransmission(address);
  I2C_in.write(byte(0));
  I2C_in.endTransmission();
}


void readIRValues() {
  int count = 0;
  for (int i = 0; i < NUM_FINGERS; i++) {
    const byte ir_address = fingers[i].irAddr;
    proximity_value_[i] = readFromCommandRegister(ir_address, PS_DATA_L);
    //Serial out
    Serial.print(proximity_value_[i]); Serial.print('\t');

    //I2C out
//    #if(I2C_OUT_ENABLED)
//      I2C_out.beginTransmission(OUTPUT_I2C_ADDRESS);
//      int num_sent = I2C_out.write(proximity_value_[i]); 
//      if(num_sent!=2){
//        Serial.print("I2C_out write failed! ");
//        Serial.println(num_sent);
//        while(true);
//      }      
//      I2C_out.write('\t');
//      I2C_out.endTransmission();
//    #endif
    
    //bluetooth out
    #if(BLUETOOTH_ENABLED)
      if(bleuart.available()){
        bleuart.write(proximity_value_[i]); bleuart.write('\t');
      }
    #endif

  }
}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID SETUP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(LED_PIN, OUTPUT);
  toggle_light();

  Serial.begin(115200);
  I2C_in.begin();
  
  #if(I2C_OUT_ENABLED)
    I2C_out.begin();
  #endif

  delay(1000);
  //while(!Serial);//Wait for usb serial to wake up.
  toggle_light();

  #if(BLUETOOTH_ENABLED)
    bluetooth_init();
  #endif

//  delay(1000);
//  Serial.println("Starting up...");


/*************************/
/*** SCAN I2c CHANNELS ***/
/*************************/
//  scan_i2c_in();
//  #if(I2C_OUT_ENABLED)
//  scan_i2c_out();
//  #endif


   //  I2C_in.setClock(100000);
  //pinMode(13, OUTPUT); // to measure samp. frq. using oscilloscope

//  initialize attached devices
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    initIRSensor(fingers[i].irAddr);
    initPressure(fingers[i].baroAddr, i);
  }

}



//////////////////////////////////////////////////////////////////////
/////////////////////////// VOID LOOP BELOW ///////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {
//  if((millis()-last_light_switch)>BLINKY_LIGHT_PERIOD_MS){
//    toggle_light();
//  }

  
//  if (micros() - lastMicros > SAMPLING_INTERVAL) {
//    lastMicros = micros(); // do this first or your interval is too long!

    readIRValues(); //-> array of IR values (2 bytes per sensor)
    readPressureValues(); //-> array of Pressure Values (4 bytes per sensor)

    Serial.print('\n');
//  }


//  transmitData(OUTPUT_I2C_ADDRESS);

}
