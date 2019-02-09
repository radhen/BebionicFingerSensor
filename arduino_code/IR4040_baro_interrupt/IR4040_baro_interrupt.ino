#include <Wire.h>
#include <Filters.h>
#include <BaroSensor.h>


/***** USER PARAMETERS *****/
#define NFINGERS 1 // number of fingers connected
int i2c_ids_[] = {112};//, MUX_ADDR|1};
int sensor_ports[NFINGERS] = {0}; // Mux board ports for each Barometer sensor {0,2,4,6}
int32_t data[3];
uint16_t Coff[6][NFINGERS];
int timer1_counter;
volatile int32_t mbar;
int num_devices_;

/***** GLOBAL CONSTANTS *****/
#define BARO_ADDRESS 0x76  // MS5637_02BA03 I2C address is 0x76(118)
#define COMMAND_0 0x80  // starts measurements, relays data ready info


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


void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 10;
  delay(1000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
  
  //initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Wire.beginTransmission(i2c_ids_[i]);
    Wire.write(0);
    int errcode = Wire.endTransmission();
//    Serial.println(errcode);
    initPressure(i2c_ids_[i]); 
  }

  // Initialize Timer
 pinMode(13, OUTPUT);

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer1_counter = 45536;   // preload timer 65536-16MHz/8/2000Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/2Hz
  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}


ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  digitalWrite(13,!digitalRead(13));
  readPressureValues();
//checking_interrupt();
}

void selectSensor(int muxID, int i) {
  Wire.beginTransmission(muxID);
  Wire.write(1 << i);
  Wire.endTransmission();
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
  delayMicroseconds(10);

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

   return ((data[0]*65536.0) + (data[1]*256.0) + data[2]);
}
  

void readPressureValues() {
  for (int i = 0; i < num_devices_; i++) {
    for (int j = 0; j < NFINGERS; j++) {
     mbar = getPressureReading(i2c_ids_[i], sensor_ports[j]);
  }
}
}

void loop() {
//  cli();
//  float p  = mbar;
//  sei();
  Serial.println("loop works!");
}
