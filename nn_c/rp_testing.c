#include <stdio.h>
#include "../neural_network.h"
#include "../neural_network.c"



int main(){

// This will feed forward the neural newtork.
// First, we need to declare the structures
// for each layer.


float ** window= (float**)malloc(1*sizeof(float*));
for (int i=0; i<1; i++){
    window[i]= (float*)malloc(2*sizeof(float));
}

for (int i=0; i<1; i++){
    for (int j=0; j<2; j++){
        window[i][j]= 1.0;
    }
}

//struct Conv1D L1;
//set_conv1D(&L1, WINDOW_SIZE, NUM_ADC, 4, 8);
//fwd_conv1D(&L1, 4, 2, 8, W_0, b_0, window);
//
//struct Conv1D L2;
//set_conv1D(&L2, 47, 8, 4, 8);
//fwd_conv1D(&L2, 4, 8, 8, W_1, b_1, L1.h);
//
//struct Flatten2D1D FL;
//flatten2D1D(&FL, 44, 8, L2.h);

struct Dense D1;
set_dense(&D1, WINDOW_SIZE, NUM_ADC, 'r');
fwd_dense(&D1, D1.input_size, D1.output_size, W_0, b_0, FL.h);

struct Dense D2;
set_dense(&D2, D1.output_size, 6, 'r');
fwd_dense(&D2, D2.input_size, D2.output_size, W_1, b_1, D1.h);

struct Dense D3;
set_dense(&D3, D2.output_size, 12, 'r');
fwd_dense(&D3, D3.input_size, D3.output_size, W_2, b_2, D2.h);

struct Dense D4;
set_dense(&D4, D3.output_size, 4, 'r');
fwd_dense(&D4, D4.input_size, D4.output_size, W_3, b_3, D3.h);

struct Dense D5;
set_dense(&D5, D4.output_size, 1, 'r');
fwd_dense(&D5, D5.input_size, D5.output_size, W_4, b_4, D4.h);

printf("Prediction:");
for (int i=0; i<D5.output_size; i++){
    printf("%.6f  ", D5.h[i]);
}
printf("\n");

printf("num_layers= %d\n", num_layers);

// should be [[ 0.01106095  0.02823588 -1.213638   ]] from python implementation

return 0;
}