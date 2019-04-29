#include "neural_network_functions.h"
#include <stdio.h>

#define WINDOW_SIZE 5
#define NUM_ADC 2

int analogPin1 = A0;
int analogPin2 = A1;
int idx = 0;
float ** window;
unsigned long elapsed, start;

volatile float output[3];

double atan_137s(double x) {
  const double c1 = 48.70107004404898384;
  const double c2 = 49.5326263772254345;
  const double c3 =  9.40604244231624;
  const double c4 = 48.70107004404996166;
  const double c5 = 65.7663163908956299;
  const double c6 = 21.587934067020232;
  double x2;

  x2 = x * x;
  return (x * (c1 + x2 * (c2 + x2 * c3)) / (c4 + x2 * (c5 + x2 * (c6 + x2))));
}

void feed_forward(float ** window) {

  struct Conv1D L1;
  set_conv1D(&L1, WINDOW_SIZE * NUM_ADC, 1, 5, 3);
  fwd_conv1D(&L1, 5, 1, 3, &W_0[0][0][0], b_0, window);

  struct Flatten2D1D FL;
  setflatten2D1D(&FL, L1.output_shape, L1.filters);
  flatten2D1D(&FL, L1.h);

  struct Dense D2;
  set_dense(&D2, FL.output_size, 3, 'l');
  fwd_dense(&D2, D2.input_size, D2.output_size, &W_1[0][0], b_1, FL.h);

  output[0] = D2.h[0];
  output[1] = D2.h[1];
  output[2] = D2.h[2];
}


void setup() {
  Serial.begin(115200);
  window = (float**) malloc (WINDOW_SIZE * NUM_ADC * sizeof(float*));
  for (int i = 0; i < WINDOW_SIZE * NUM_ADC ; i++) window[i] = (float*)malloc (1 * sizeof(float));
}

void loop() {
  int val1 = analogRead(analogPin1);
  int val2 = analogRead(analogPin2);

  if (idx < WINDOW_SIZE * NUM_ADC) {
    window[idx++][0] = 1.00;//(float)val1 / (float)255.0;
    window[idx++][0] = 1.00;//(float)val2 / (float)255.0;
  } else {
    idx = 0;
    start = micros();
    feed_forward(window);
    elapsed= micros()- start;
    
    Serial.print("elapsed: ");
    Serial.println(elapsed/1000.00f, 7);

    // Printing results (make sure you do not do any printing inside feed_forward() as it affects the speed)
//      Serial.print(output[0],7);
//      Serial.print(", ");
//      Serial.print(output[1],7);
//      Serial.print(", ");
//      Serial.print(output[2],7);
//      Serial.println();
    window = (float**) malloc (WINDOW_SIZE * NUM_ADC * sizeof(float*));
    for (int i = 0; i < WINDOW_SIZE * NUM_ADC ; i++) window[i] = (float*)malloc (1 * sizeof(float));
  }
}
