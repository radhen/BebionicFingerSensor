#ifndef __NEURAL_NETWORK_H__
#define __NEURAL_NETWORK_H__

#define WINDOW_SIZE     1
#define NUM_ADC         2

volatile int num_layers= 0;

struct Dense {
    int input_size;
    int output_size;
    char activation;
    float *h;
};

#endif