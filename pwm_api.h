#ifndef __PWM_API_H
#define __PWM_API_H

#include "pinmap.h"

#define PERIOD 2560

#define TCR_CNT_EN       0x00000001
#define TCR_RESET        0x00000002

void pwm_init(void);
void pwm_pulsewidth(uint16_t *pwm);

void pwm_init(void) {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);    // Enable TIMER16_0 clock
    LPC_IOCON->PIO0_8 = 0x02;        // PIO0_8 IS MAT0 output
    LPC_IOCON->PIO0_9 = 0x02;        // PIO0_9 IS MAT1 output
    LPC_TMR16B0->MR3 = PERIOD;          // MR3 = Period
    LPC_TMR16B0->MR0 = PERIOD - 1;            // MR0 = 50% duty cycle
    LPC_TMR16B0->MR1 = PERIOD - 1;            // MR1 = 50% duty cycle
    LPC_TMR16B0->MCR = 0x400;        // MR3 resets timer
    LPC_TMR16B0->PWMC = 0x0B;        // Enable PWM0 and PWM3
    LPC_TMR16B0->TCR = TCR_CNT_EN;            // Enable Timer0
    
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);    // Enable TIMER16_1 clock
    LPC_IOCON->PIO1_10 = 0x02;        // PIO1_10 IS MAT1 output
    LPC_TMR16B1->MR3 = PERIOD;          // MR3 = Period
    LPC_TMR16B1->MR1 = PERIOD - 1;            // MR1 = 50% duty cycle
    LPC_TMR16B1->MCR = 0x400;        // MR3 resets timer
    LPC_TMR16B1->PWMC = 0x0A;        // Enable PWM1 and PWM3
    LPC_TMR16B1->TCR = TCR_CNT_EN;            // Enable Timer1
    
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);    // Enable TIMER32_0 clock
    LPC_IOCON->R_PIO0_11 = 0x03;        // PIO1_10 IS MAT1 output
    LPC_TMR32B0->MR2 = PERIOD;          // MR2 = Period
    LPC_TMR32B0->MR3 = PERIOD - 1;            // MR3 = 50% duty cycle
    LPC_TMR32B0->MCR = 0x80;        // MR2 resets timer
    LPC_TMR32B0->PWMC = 0x0C;        // Enable PWM2 and PWM3
    LPC_TMR32B0->TCR = TCR_CNT_EN;            // Enable Timer2
}

//PwmOut2 M_BL(P0_8);  // P0_8  M0  TMR16B0 MR0
//PwmOut M_FL(P0_9);   // P0_9  M1  TMR16B0 MR1
//PwmOut M_BR(P1_10);  // P1_10 M2  TMR16B1 MR1
//PwmOut2 M_FR(P0_11); // P0_11 M3  TMR32B0 MR3
void pwm_pulsewidth(uint16_t *pwm){
    //uint32_t t_off;
    //timer->TCR = TCR_RESET;
    
    /*LPC_TMR16B0->MR0 = LPC_TMR16B0->MR3 - (pwm[0] >> 2);
    LPC_TMR16B0->MR1 = LPC_TMR16B0->MR3 - (pwm[1] >> 2);
    LPC_TMR16B1->MR1 = LPC_TMR16B1->MR3 - (pwm[2] >> 2);
    LPC_TMR32B0->MR3 = LPC_TMR32B0->MR2 - (pwm[3] >> 2);*/
    
    /*LPC_TMR16B0->MR0 = LPC_TMR16B0->MR3 - pwm[0];
    LPC_TMR16B0->MR1 = LPC_TMR16B0->MR3 - pwm[1];
    LPC_TMR16B1->MR1 = LPC_TMR16B1->MR3 - pwm[2];
    LPC_TMR32B0->MR3 = LPC_TMR32B0->MR2 - pwm[3];*/
    
    LPC_TMR16B0->MR0 = PERIOD - pwm[0];
    LPC_TMR16B0->MR1 = PERIOD - pwm[1];
    LPC_TMR16B1->MR1 = PERIOD - pwm[2];
    LPC_TMR32B0->MR3 = PERIOD - pwm[3];
    
    //LPC_TMR16B0->TCR = TCR_CNT_EN;
    //LPC_TMR16B1->TCR = TCR_CNT_EN;
    //LPC_TMR32B0->TCR = TCR_CNT_EN;

    //timer->TCR = TCR_CNT_EN;
}

#endif

