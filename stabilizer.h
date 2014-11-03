#ifndef __stabilizer_H
#define __stabilizer_H

#include "PID.h"


// http://en.wikipedia.org/wiki/Category:Control_theory

//PIDClass PitchPID(0.25, 0, 0.01);   // Kp,Ki,Kd 0.16 0.04 0.01
PIDClass PitchPID(1,0,1,0);
//PIDClass RollPID(0, 0, 0);   // Kp,Ki,Kd
PIDClass RollPID(1,0,1,0);

//void calPID(uint16_t *input, float *degree, uint16_t *output);
void calPID(uint16_t *input, float *degree, float *gyr, uint16_t *output);

// input 0 ~ 1023
// degree -180.0 ~ 180.0
// output 0~2560
void calPID(uint16_t *input, float *degree, float *gyr, uint16_t *output){
    float out_roll;
    float out_pitch;
    
    float pitch = (input[0] - 511) * 32 / 512.0;
    float roll = (input[1] - 511) * 32 / 512.0;
    
    out_pitch = PitchPID.update(pitch, degree[1]+1.0, gyr[1]);
    out_roll = RollPID.update(roll, degree[0], gyr[0]);
    
    output[0] = out_pitch * 1280 / 180 + 1279;
    output[1] = out_roll * 1280 / 180 + 1279;
    output[2] = (input[2] << 1) + (input[2] >> 1); // 0~1023 >>x2.5>> 0~2559
    output[3] = (input[3] << 1) + (input[3] >> 1); // 0~1023 >>x2.5>> 0~2559
}

#endif