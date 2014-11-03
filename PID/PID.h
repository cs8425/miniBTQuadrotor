/* Copyright (c) <2013> MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * Nested PID for Quadcopter Control
 *
 
 */
 
#ifndef PID_H
#define PID_H
 
#include "mbed.h"
 
class PIDClass
{
 
   public:
   float Angle_Kp,Angle_Ki,Rate_Kp,Rate_Kd;
   
   PIDClass(float AngleKp,float AngleKi, float RateKp, float RateKd);
   
   float update(float Setpoint, float CurrentPosition, float Rate);
   
   private:
   
        //float Angle_Kp,Angle_Ki,Rate_Kp,Rate_Kd;
        float AngleError,RateError;
        float Angle_I,Rate_D;
        float Output;
        float PreviousError;
        uint32_t last_time;
 
};
#endif
/*
#ifndef PID_H
#define PID_H

#include "mbed.h"

class PIDClass
{

   public:
   float Kp,Ki,Kd;
   
   PIDClass(float Angle_Kp,float Angle_Ki, float Angle_Kd);
   
   int update(float Setpoint, float CurrentPosition);
   
   private:

        //float Kp,Ki,Kd;
        
        float error;
        float integral;
        float derivative;
        
        float output;
        
        float previous_error;
        
        uint32_t last_time;

};
#endif
*/