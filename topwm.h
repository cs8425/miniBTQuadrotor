#ifndef __toPWM_H
#define __toPWM_H

#include "pwm_api.h"

#define MIN 0
#define MAX 2559

#define CENTER 1279


inline uint16_t LIM(int x);
//void calpwm(void);
void calpwm(uint16_t *input, uint16_t *output);


void calpwm(uint16_t *input, uint16_t *output){
    
    output[0] = LIM(CENTER-input[0] + input[1]-CENTER + input[2] + CENTER-input[3]);
    output[1] = LIM(input[0]-CENTER + input[1]-CENTER + input[2] + input[3]-CENTER);
    output[2] = LIM(CENTER-input[0] + CENTER-input[1] + input[2] + input[3]-CENTER);
    output[3] = LIM(input[0]-CENTER + CENTER-input[1] + input[2] + CENTER-input[3]);
    
    /*output[0] = input[2];
    output[1] = input[2];
    output[2] = input[2];
    output[3] = input[2];*/
}


inline uint16_t LIM(int x){
    if(x < MIN){
        x = MIN;
    }else{
        if(x > MAX){
            x = MAX;
        }
    }
    return x;
}

#endif
