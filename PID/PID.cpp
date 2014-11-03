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
 
#include "PID.h"
 
PIDClass::PIDClass(float AngleKp,float AngleKi, float RateKp, float RateKd){
 
        Angle_Kp=AngleKp;
        Angle_Ki=AngleKi;
        Rate_Kp=RateKp;
        Rate_Kd=RateKd;
        
        last_time = LPC_TMR32B1->TC;
}
 
float PIDClass::update(float Setpoint, float CurrentPosition, float Rate){
      
      uint32_t now = LPC_TMR32B1->TC;
      uint32_t delta = now - last_time;
      //uint32_t dt = now - last_time;
      float dt = delta / 4000000.0;
      
      AngleError = Setpoint - CurrentPosition;
      
      Angle_I +=AngleError*dt;
      
      if(Angle_I>180.0f) Angle_I=180.0f;
      else if(Angle_I<-180.0f) Angle_I= -180.0f;
      
      RateError = (AngleError*Angle_Kp)- Rate;
      
      Rate_D = (RateError-PreviousError)/dt;
      
      Output = (RateError*Rate_Kp) + (Angle_I*Angle_Ki) + (Rate_D*Rate_Kd);
      
      PreviousError = RateError;
      
      last_time = LPC_TMR32B1->TC;
      
      return Output;
}

/*
#include "PID.h"

PIDClass::PIDClass(float Angle_Kp,float Angle_Ki, float Angle_Kd){

    Kp = Angle_Kp;
    Ki = Angle_Ki;
    Kd = Angle_Kd;
    
    previous_error = 0;
    integral = 0;
    
    last_time = LPC_TMR32B1->TC;
}

int PIDClass::update(float Setpoint, float CurrentPosition){
      
    uint32_t now = LPC_TMR32B1->TC;
    //uint32_t delta = now - last_time;
    uint32_t dt = now - last_time;
    //float dt = delta / 4000000.0;

    error = Setpoint - CurrentPosition;

    integral += error*dt;

    if(integral>180.0f) integral = 180.0f;
    else if(integral<-180.0f) integral = -180.0f;

    derivative = (error - previous_error)/dt;

    output = Kp*error + Ki*integral + Kd*derivative;

    previous_error = error;
    
    last_time = LPC_TMR32B1->TC;
    
    return output;
}
*/