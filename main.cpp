#include "mbed.h"
#include "power_profiles.h"
#include "pwm_api.h"
#include "topwm.h"
#include "systicks.h"
#include "FastIO.h"
#include "MPU6050.h"

#include "ring_buffer.h"
#include "tools.h"

#include "sensfusion.h"
#include "stabilizer.h"

Serial pc(P1_7, P1_6); // tx, rx
//RawSerial pc(P1_7, P1_6); // tx, rx
//http://tf3dm.com/3d-model/uh60-helicopter-47194.html

MPU6050 mpu(P0_5, P0_4); // I2C_SDA, I2C_SCL

FastOut<P2_0, 0> led0;  // led0
FastOut<P2_6, 0> led1;  // led1
FastOut<P3_3, 0> led2;  // led2

FastOut<P3_0, 0> led3;  // led3
FastOut<P3_1, 0> led4;  // led4
FastOut<P2_3, 0> led5;  // led5
FastOut<P1_4, 0> led6;  // led6

FastOut<P2_9, 0> led7;  // led7
FastOut<P2_10, 0> led8; // led8


void status_upate(void);
void led_thread(void);
void BT_thread(void);
void sensfusion6_thread(void);
uint32_t readInt(void);

ring_buffer RX_buf;

void BT_RX_ISR(void);

uint16_t input[4] = {511, 511, 0, 511};
uint16_t pwm[4] = {0, 0, 0, 0};
uint16_t pidout[4] = {0, 0, 0, 0};

//int16_t acc_raw[3] = {0, 0, 0};
//int16_t gyr_raw[3] = {0, 0, 0};
float acc_raw[3] = {0, 0, 0};
float gyr_raw[3] = {0, 0, 0};

//int32_t degree[3] = {0, 0, 0};
float degree[3] = {0, 0, 0};

// bit 2 => BT '\n' ready
// bit 1 => ready to get MPU6050 data & run sensfusion
// bit 0 => ready to output debug data
volatile uint8_t status = 0x00;

#define TH_CNT 2
#define MS 4000
volatile uint32_t lastrun[TH_CNT] = {4*MS, 0};
uint32_t period[TH_CNT] = {50 * MS, 5*MS};


int main() {
    pc.baud(115200);
    
    /*  Turn off the      BOD, ADC & WDT */
    LPC_SYSCON->PDRUNCFG |= (1<<3)|(1<<4)|(1<<6);

    pwm_init();

    
    if(config_power_mode(PARAM_CPU_EXEC)){
        //pc.printf("48MHz maximum CPU performance Config Fail\n");
        pc.puts("48MHz maximum CPU performance Config Fail\n");
    }else{
        //pc.printf("48MHz maximum CPU performance Config OK~\n");
        pc.puts("48MHz maximum CPU performance Config OK~\n");
    }

    if(mpu.testConnection()){
        //pc.printf("MPU6050 OK~\n");
        pc.puts("MPU6050 OK~\n");
        wait_ms(100);
        mpu.setBW(MPU6050_BW_256); // acc low pass 260 Hz
        mpu.setFullScaleAccelRange(MPU6050_ACCELERO_RANGE_8G);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_RANGE_1000);
        mpu.initialize();
    }else{
        //pc.printf("MPU6050 failed!\n");
        pc.puts("MPU6050 failed!\n");
        NVIC_SystemReset();
    }


    pc.attach(&BT_RX_ISR, pc.RxIrq);
    NVIC_EnableIRQ(UART_IRQn);
    
    ticker_init();
    //status |= 0x02;
    
    //pc.printf("initialize OK!\n");
    pc.puts("initialize OK!\n");

    while(1) {
        status_upate();
        switch(status){
            case 0x00:
            
            break;

            case 0x01:
                led1 = 1;
                status &= ~(0x01);
                //pc.printf("%f %f %f\n", degree[0], degree[1], degree[2]);
                
                //pc.printf("%f\t%f\t%f\t%f\n", q0, q1, q2, q3);
                encodeQ(q0, q1, q2, q3);
                pc.printf("%s\n", buf);
                //pc.puts(buf);
                //pc.printf("%d\t%d\n", LPC_TMR32B1->TC, sys_tick);
                
                led1 = 0;
            break;
            
            case 0x03:
            case 0x02:
                led2 = 1;
                status &= ~(0x02);
                sensfusion6_thread();
                calPID(input, degree, gyr_raw, pidout);
                calpwm(pidout, pwm);
                pwm_pulsewidth(pwm);
                led2 = 0;
            break;
            
            default:
            case 0x04:
                status &= ~(0x04);
                BT_thread();
                calPID(input, degree, gyr_raw, pidout);
                calpwm(pidout, pwm);
                pwm_pulsewidth(pwm);
            break;

        }
    }
}

void status_upate(void) {
    uint32_t now = LPC_TMR32B1->TC;
    //uint32_t delta = now - lasttime;
    uint8_t i;
    uint32_t delta;
    for(i = 0; i<TH_CNT; i++){
        delta = now - lastrun[i];
        if(delta > period[i]){
            status |= 1 << i;
            lastrun[i] = now;
            //pc.printf("%d\t%d\t%d\t%d\n", LPC_TMR32B1->TC, now, i, delta);
        }
    }
}

void BT_RX_ISR(void) {
    //led2 = 1;
    unsigned char c;

    c = LPC_UART->RBR;
    //pc.putc(c+1);
    if(c != '\n'){
        RX_buf.write(c);
    }else{
        status |= 0x04;
        //status |= 0x02;
    }
    //led2 = 0;
}

void led_thread(void) {
    uint8_t i = led0;
    i = 1 - i;
    led0 = i;

    /*input[0] = i;
    pwm_pulsewidth(input);
    i = (i + 512) % 10240;*/
    
}
void BT_thread(void) {
    int c;
    char i;
    char s;
    //led2 = 1;

    s = 0;
    i = 0;
    c = RX_buf.read();
    while(c != -1){
        switch(c){
            case 'P':
                if(i != 0){
                    RX_buf.reset();
                }
                twoKp = readInt() / 10000.0;
            break;
            case 'I':
                if(i != 0){
                    RX_buf.reset();
                }
                twoKi = readInt() / 10000.0;
            break;
            
            case 'A':
                if(i != 0){
                    RX_buf.reset();
                }
                //PitchPID.Kp = readInt() / 10000.0;
                PitchPID.Angle_Kp = readInt() / 10000.0;
            break;
            case 'B':
                if(i != 0){
                    RX_buf.reset();
                }
                //PitchPID.Ki = readInt() / 10000.0;
                PitchPID.Angle_Ki = readInt() / 10000.0;
            break;
            case 'C':
                if(i != 0){
                    RX_buf.reset();
                }
                //PitchPID.Kd = readInt() / 10000.0;
                PitchPID.Rate_Kp = readInt() / 10000.0;
            break;
            
            case 'D':
                if(i != 0){
                    RX_buf.reset();
                }
                //RollPID.Kp = readInt() / 10000.0;
                PitchPID.Rate_Kd = readInt() / 10000.0;
            break;
            case 'E':
                if(i != 0){
                    RX_buf.reset();
                }
                //RollPID.Ki = readInt() / 10000.0;
                RollPID.Angle_Kp = readInt() / 10000.0;
            break;
            case 'F':
                if(i != 0){
                    RX_buf.reset();
                }
                //RollPID.Kd = readInt() / 10000.0;
                RollPID.Angle_Ki = readInt() / 10000.0;
            break;
            case 'G':
                if(i != 0){
                    RX_buf.reset();
                }
                RollPID.Rate_Kp = readInt() / 10000.0;
            break;
            case 'H':
                if(i != 0){
                    RX_buf.reset();
                }
                RollPID.Rate_Kd = readInt() / 10000.0;
            break;
            
            case ',':
                s = 0;
                i++;
                if(i == 4) break;
            break;
            
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if(!s){
                    input[i] = c - '0';
                    s++;
                }else{
                    input[i] = input[i] * 10 + c - '0';
                }
            break;
                
            default:
                RX_buf.reset();
            break;
        }
        //pc.putc(c);
        c = RX_buf.read();
    }
    //pc.printf("%d %d %d %d \n", input[0], input[1], input[2], input[3]);
    //pwm_pulsewidth(input);
    //calpwm(input, pwm);
    //led2 = 0;
    
    //status |= 0x02;
}
void sensfusion6_thread(void) {
    //led1 = 1;
    //mpu.getAcceleroRaw(acc_raw);
    //mpu.getGyroRaw(gyr_raw);
    mpu.getAccelero(acc_raw);
    mpu.getGyro(gyr_raw);
    //pc.printf("%d\t%d\t%d\t%d\t%d\t%d\n", acc_raw[0], acc_raw[1], acc_raw[2], gyr_raw[0], gyr_raw[1], gyr_raw[2]);
    //status |= 0x02;
    //led1 = 0;
    
    calsensfusion(acc_raw, gyr_raw, degree);
    //led1 = 0;
}

uint32_t readInt(void) {
    uint32_t val = 0;
    int c = RX_buf.read();
    while(c != -1){
        val = val * 10 + c - '0';
        c = RX_buf.read();
    }
    return val;
}

