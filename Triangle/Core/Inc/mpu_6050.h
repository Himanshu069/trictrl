#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

// Default I2C Address for MPU6050 (AD0 Pin Grounded)
#define MPU_ADDR 0x68  

// --- CUSTOM PCB BIT-BANG CONFIGURATION (PB0 = SCL, PB1 = SDA) ---
// Uses STM32 Atomic Bit Set/Reset Registers (BSRR/BRR) for maximum speed
#define SCL_HIGH()  GPIOB->BSRR = GPIO_PIN_0   // Set PB0 High
#define SCL_LOW()   GPIOB->BRR  = GPIO_PIN_0   // Reset PB0 Low
#define SDA_HIGH()  GPIOB->BSRR = GPIO_PIN_1   // Set PB1 High
#define SDA_LOW()   GPIOB->BRR  = GPIO_PIN_1   // Reset PB1 Low
#define SDA_READ()  ((GPIOB->IDR & GPIO_PIN_1) ? 1 : 0) // Read input data from PB1

// Driver Function Prototypes
void MPU6050_Init(void);
uint8_t MPU6050_ReadByte(uint8_t reg);
void MPU6050_ReadData(float* accel_ms2, float* gyro_dps);

#endif /* MPU6050_H */