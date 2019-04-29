#include "neural_network_functions.h"

//float nnpred(float window[2]){
float nnpred(float * window) {
  // This will feed forward the neural newtork.
  // First, we need to declare the structures
  // for each layer.
  
  struct Dense D1;
  set_dense(&D1, WINDOW_SIZE*NUM_ADC, 2, 'r');
  fwd_dense(&D1, D1.input_size, D1.output_size, &W_0[0][0], b_0, window);

  struct Dense D2;
  set_dense(&D2, D1.output_size, 6, 'r');
  fwd_dense(&D2, D2.input_size, D2.output_size, &W_1[0][0], b_1, D1.h);

  struct Dense D3;
  set_dense(&D3, D2.output_size, 12, 'r');
  fwd_dense(&D3, D3.input_size, D3.output_size, &W_2[0][0], b_2, D2.h);

  struct Dense D4;
  set_dense(&D4, D3.output_size, 4, 'r');
  fwd_dense(&D4, D4.input_size, D4.output_size, &W_3[0][0], b_3, D3.h);

  struct Dense D5;
  set_dense(&D5, D4.output_size, 1, 'l');
  fwd_dense(&D5, D5.input_size, D5.output_size, &W_4[0][0], b_4, D4.h);

//
//  printf("%d\n", D5.output_size);
 float predictions;
//  
//  printf("Prediction:");
    predictions = D5.h[0];

  free(D5.h);

  return predictions;
  //return 0;
}
/*
int main(void) {

  float * window;
  float predictions[1];
  window = (float*)malloc(2 * sizeof(float));
  for (int i = 0; i < 2; i++) window[i] = 1.0;

  // printf(" %f ", nnpred(window));

  // Hey Radhen, I realized in C you can get away with this:
  nnpred(window, predictions);
  for (int i=0; i<3; i++) printf("%f ", predictions[i]);

  return 0;
}*/
