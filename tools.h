#ifndef __tools_H
#define __tools_H

char buf[40] = {0};

typedef union {
  uint32_t Int;
  float Float;
  char Byte[4];
} chType;

typedef union {
  uint16_t UInt;
  int16_t Int;
  char Byte[2];
} chType16;



void encodeQ(float q0, float q1, float q2, float q3);
void encodeRPY(float roll, float pitch, float yaw);

void encodeQ(float q0, float q1, float q2, float q3){
    chType data;
    data.Float = q0;
    buf[0] = (data.Byte[0] >> 4) + '0';
    buf[1] = (data.Byte[0] & 0x0F) + '0';
    buf[2] = (data.Byte[1] >> 4) + '0';
    buf[3] = (data.Byte[1] & 0x0F) + '0';
    buf[4] = (data.Byte[2] >> 4) + '0';
    buf[5] = (data.Byte[2] & 0x0F) + '0';
    buf[6] = (data.Byte[3] >> 4) + '0';
    buf[7] = (data.Byte[3] & 0x0F) + '0';
    
    data.Float = q1;
    buf[8] = (data.Byte[0] >> 4) + '0';
    buf[9] = (data.Byte[0] & 0x0F) + '0';
    buf[10] = (data.Byte[1] >> 4) + '0';
    buf[11] = (data.Byte[1] & 0x0F) + '0';
    buf[12] = (data.Byte[2] >> 4) + '0';
    buf[13] = (data.Byte[2] & 0x0F) + '0';
    buf[14] = (data.Byte[3] >> 4) + '0';
    buf[15] = (data.Byte[3] & 0x0F) + '0';
    
    data.Float = q2;
    buf[16] = (data.Byte[0] >> 4) + '0';
    buf[17] = (data.Byte[0] & 0x0F) + '0';
    buf[18] = (data.Byte[1] >> 4) + '0';
    buf[19] = (data.Byte[1] & 0x0F) + '0';
    buf[20] = (data.Byte[2] >> 4) + '0';
    buf[21] = (data.Byte[2] & 0x0F) + '0';
    buf[22] = (data.Byte[3] >> 4) + '0';
    buf[23] = (data.Byte[3] & 0x0F) + '0';
    
    data.Float = q3;
    buf[24] = (data.Byte[0] >> 4) + '0';
    buf[25] = (data.Byte[0] & 0x0F) + '0';
    buf[26] = (data.Byte[1] >> 4) + '0';
    buf[27] = (data.Byte[1] & 0x0F) + '0';
    buf[28] = (data.Byte[2] >> 4) + '0';
    buf[29] = (data.Byte[2] & 0x0F) + '0';
    buf[30] = (data.Byte[3] >> 4) + '0';
    buf[31] = (data.Byte[3] & 0x0F) + '0';
    
    buf[32] = 0;
}

void encodeRPY(float roll, float pitch, float yaw){
    chType data;
    data.Float = roll;
    buf[0] = (data.Byte[0] >> 4) + '0';
    buf[1] = (data.Byte[0] & 0x0F) + '0';
    buf[2] = (data.Byte[1] >> 4) + '0';
    buf[3] = (data.Byte[1] & 0x0F) + '0';
    buf[4] = (data.Byte[2] >> 4) + '0';
    buf[5] = (data.Byte[2] & 0x0F) + '0';
    buf[6] = (data.Byte[3] >> 4) + '0';
    buf[7] = (data.Byte[3] & 0x0F) + '0';
    
    data.Float = pitch;
    buf[8] = (data.Byte[0] >> 4) + '0';
    buf[9] = (data.Byte[0] & 0x0F) + '0';
    buf[10] = (data.Byte[1] >> 4) + '0';
    buf[11] = (data.Byte[1] & 0x0F) + '0';
    buf[12] = (data.Byte[2] >> 4) + '0';
    buf[13] = (data.Byte[2] & 0x0F) + '0';
    buf[14] = (data.Byte[3] >> 4) + '0';
    buf[15] = (data.Byte[3] & 0x0F) + '0';
    
    data.Float = yaw;
    buf[16] = (data.Byte[0] >> 4) + '0';
    buf[17] = (data.Byte[0] & 0x0F) + '0';
    buf[18] = (data.Byte[1] >> 4) + '0';
    buf[19] = (data.Byte[1] & 0x0F) + '0';
    buf[20] = (data.Byte[2] >> 4) + '0';
    buf[21] = (data.Byte[2] & 0x0F) + '0';
    buf[22] = (data.Byte[3] >> 4) + '0';
    buf[23] = (data.Byte[3] & 0x0F) + '0';
    
    buf[24] = 0;
}

#endif
