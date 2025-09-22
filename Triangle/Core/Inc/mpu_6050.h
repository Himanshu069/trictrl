#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"   // Change to your MCU's HAL include

#ifdef __cplusplus
extern "C" {
#endif

// I2C pins (bit-banged)
#define SDA_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

// MPU6050 I2C address
#define MPU_ADDR 0x68

// I2C functions
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ack);

// MPU6050 functions
void MPU6050_Init(void);
uint8_t MPU6050_ReadByte(uint8_t reg);
void MPU6050_ReadData(float* accel, float* gyro);

#ifdef __cplusplus
}
#endif

#endif
