#include "mpu_6050.h"

// Simple delay for bit-bang speed control
static void I2C_Delay(void) {
    for (volatile int i = 0; i < 500; i++);
}

// ---------- Basic I2C ----------
void I2C_Start(void) {
    SDA_HIGH(); SCL_HIGH(); I2C_Delay();
    SDA_LOW();  I2C_Delay();
    SCL_LOW();  I2C_Delay();
}

void I2C_Stop(void) {
    SDA_LOW(); SCL_HIGH(); I2C_Delay();
    SDA_HIGH(); I2C_Delay();
}

uint8_t I2C_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SDA_HIGH(); else SDA_LOW();
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        SCL_LOW();  I2C_Delay();
    }
    // ACK
    SDA_HIGH(); // release SDA
    SCL_HIGH(); I2C_Delay();
    uint8_t ack = SDA_READ();
    SCL_LOW(); I2C_Delay();
    return ack; // 0 if ACK received
}

uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t data = 0;
    SDA_HIGH(); // release SDA
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        SCL_HIGH(); I2C_Delay();
        if (SDA_READ()) data |= 1;
        SCL_LOW(); I2C_Delay();
    }
    // Send ACK/NACK
    if (ack) SDA_LOW(); else SDA_HIGH();
    SCL_HIGH(); I2C_Delay();
    SCL_LOW();  I2C_Delay();
    SDA_HIGH(); // release
    return data;
}

// ---------- MPU6050 ----------
void MPU6050_Init(void) {
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0); // write
    I2C_WriteByte(0x6B);              // PWR_MGMT_1
    I2C_WriteByte(0x00);              // wake up
    I2C_Stop();
}

uint8_t MPU6050_ReadByte(uint8_t reg) {
    uint8_t data;
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0); // write
    I2C_WriteByte(reg);
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1); // read
    data = I2C_ReadByte(0); // NACK
    I2C_Stop();
    return data;
}

void MPU6050_ReadData(float* accel_ms2, float* gyro_dps) {
    uint8_t buffer[14];
    int16_t accel[3], gyro[3];

    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 0);
    I2C_WriteByte(0x3B); // ACCEL_XOUT_H
    I2C_Start();
    I2C_WriteByte(MPU_ADDR << 1 | 1);

    for (int i = 0; i < 13; i++) buffer[i] = I2C_ReadByte(1); // ACK
    buffer[13] = I2C_ReadByte(0); // NACK
    I2C_Stop();

    // Combine high/low bytes
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    gyro[0]  = (buffer[8] << 8) | buffer[9];
    gyro[1]  = (buffer[10] << 8) | buffer[11];
    gyro[2]  = (buffer[12] << 8) | buffer[13];

    for(int i=0;i<3;i++){
        accel_ms2[i] = ((float)accel[i] / 16384.0f) ; // accel in m/s²
        gyro_dps[i]  = (float)gyro[i] / 131.0f;                  // gyro in °/s
    }
}
