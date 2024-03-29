/**
 * Includes
 */
#include "MPU6050.h"

MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl) {
    connection.frequency(400000);
    
    //this->setSleepMode(false);
    
    //Initializations:
    currentGyroRange = 0;
    currentAcceleroRange = 0;
}
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050::initialize()
{
    setFullScaleGyroRange(MPU6050_GYRO_RANGE_250);
    setFullScaleAccelRange(MPU6050_ACCELERO_RANGE_2G);
    this->setSleepMode(false); // thanks to Jack Elston for pointing this one out!
    this->Calibrate();
}


//--------------------------------------------------
//-------------------General------------------------
//--------------------------------------------------

void MPU6050::write(char address, char data) {
    char temp[2];
    temp[0]=address;
    temp[1]=data;
    
    connection.write(MPU6050_ADDRESS * 2,temp,2);
}

char MPU6050::read(char address) {
    char retval;
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, &retval, 1);
    return retval;
}

void MPU6050::read(char address, char *data, int length) {
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, data, length);
}

void MPU6050::setSleepMode(bool state) {
    char temp;
    temp = this->read(MPU6050_PWR_MGMT_1_REG);
    if (state == true)
        temp |= 1<<MPU6050_SLP_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_SLP_BIT);
    this->write(MPU6050_PWR_MGMT_1_REG, temp);
}

bool MPU6050::testConnection( void ) {
    char temp;
    temp = this->read(MPU6050_WHO_AM_I_REG);
    return (temp == (MPU6050_ADDRESS & 0xFE));
}

void MPU6050::setBW(char BW) {
    char temp;
    BW=BW & 0x07;
    temp = this->read(MPU6050_CONFIG_REG);
    temp &= 0xF8;
    temp = temp + BW;
    this->write(MPU6050_CONFIG_REG, temp);
}

void MPU6050::setI2CBypass(bool state) {
    char temp;
    temp = this->read(MPU6050_INT_PIN_CFG);
    if (state == true)
        temp |= 1<<MPU6050_BYPASS_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_BYPASS_BIT);
    this->write(MPU6050_INT_PIN_CFG, temp);
}

void MPU6050::Calibrate(void) {
    int32_t GYRO_X_SMPL_SUM = 0;
    int32_t GYRO_Y_SMPL_SUM = 0;
    int32_t GYRO_Z_SMPL_SUM = 0;
    int x = 0;
    for (x = 0; x < CALIBRATE_SAMPLES; x++) {
        GYRO_X_SMPL_SUM += this->getGyroRawX();
        GYRO_Y_SMPL_SUM += this->getGyroRawY();
        GYRO_Z_SMPL_SUM += this->getGyroRawZ();
    }
    
    GYRO_X_OFFSET = GYRO_X_SMPL_SUM / CALIBRATE_SAMPLES;
    GYRO_Y_OFFSET = GYRO_Y_SMPL_SUM / CALIBRATE_SAMPLES;
    GYRO_Z_OFFSET = GYRO_Z_SMPL_SUM / CALIBRATE_SAMPLES;
}

//--------------------------------------------------
//----------------Accelerometer---------------------
//--------------------------------------------------

void MPU6050::setFullScaleAccelRange( char range ) {
    char temp;
    range = range & 0x03;
    currentAcceleroRange = range;
    
    temp = this->read(MPU6050_ACCELERO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + (range<<3);
    this->write(MPU6050_ACCELERO_CONFIG_REG, temp);
}

int MPU6050::getAcceleroRawX( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}
    
int MPU6050::getAcceleroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getAcceleroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

void MPU6050::getAcceleroRaw( int16_t *data ) {
    char temp[6];
    this->read(MPU6050_ACCEL_XOUT_H_REG, temp, 6);
    data[0] = (int16_t)((temp[0]<<8) + temp[1]);
    data[1] = (int16_t)((temp[2]<<8) + temp[3]);
    data[2] = (int16_t)((temp[4]<<8) + temp[5]);
}

void MPU6050::getAccelero( float *data ) {
    int16_t temp[3];
    this->getAcceleroRaw(temp);
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_2G) {
        data[0]=(float)temp[0] / 16384.0;
        data[1]=(float)temp[1] / 16384.0;
        data[2]=(float)temp[2] / 16384.0;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_4G){
        data[0]=(float)temp[0] / 8192.0;
        data[1]=(float)temp[1] / 8192.0;
        data[2]=(float)temp[2] / 8192.0;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_8G){
        data[0]=(float)temp[0] / 4096.0;
        data[1]=(float)temp[1] / 4096.0;
        data[2]=(float)temp[2] / 4096.0;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_16G){
        data[0]=(float)temp[0] / 2048.0;
        data[1]=(float)temp[1] / 2048.0;
        data[2]=(float)temp[2] / 2048.0;
        }
    
    #ifdef DOUBLE_ACCELERO
        data[0]*=2;
        data[1]*=2;   
        data[2]*=2;
    #endif   
}

//--------------------------------------------------
//------------------Gyroscope-----------------------
//--------------------------------------------------
void MPU6050::setFullScaleGyroRange( char range ) {
    char temp;
    currentGyroRange = range;
    range = range & 0x03;
    temp = this->read(MPU6050_GYRO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + range<<3;
    this->write(MPU6050_GYRO_CONFIG_REG, temp);
}

int MPU6050::getGyroRawX( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}
    
int MPU6050::getGyroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getGyroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

void MPU6050::getGyroRaw( int16_t *data ) {
    char temp[6];
    this->read(MPU6050_GYRO_XOUT_H_REG, temp, 6);
    data[0] = (int16_t)((temp[0]<<8) + temp[1]) - GYRO_X_OFFSET;
    data[1] = (int16_t)((temp[2]<<8) + temp[3]) - GYRO_Y_OFFSET;
    data[2] = (int16_t)((temp[4]<<8) + temp[5]) - GYRO_Z_OFFSET;
}

void MPU6050::getGyro( float *data ) {
    int16_t temp[3];
    this->getGyroRaw(temp);
    if (currentGyroRange == MPU6050_GYRO_RANGE_250) {
        data[0]=(float)temp[0] / 7505.7;
        data[1]=(float)temp[1] / 7505.7;
        data[2]=(float)temp[2] / 7505.7;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_500){
        data[0]=(float)temp[0] / 3752.9;
        data[1]=(float)temp[1] / 3752.9;
        data[2]=(float)temp[2] / 3752.9;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_1000){
        data[0]=(float)temp[0] / 1879.3;
        data[1]=(float)temp[1] / 1879.3;
        data[2]=(float)temp[2] / 1879.3;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_2000){
        data[0]=(float)temp[0] / 939.7;
        data[1]=(float)temp[1] / 939.7;
        data[2]=(float)temp[2] / 939.7;
        }
}
//--------------------------------------------------
//-------------------Temperature--------------------
//--------------------------------------------------
int MPU6050::getTempRaw( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_TEMP_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

float MPU6050::getTemp( void ) {
    float retval;
    retval=(float)this->getTempRaw();
    retval=(retval+521.0)/340.0+35.0;
    return retval;
}

