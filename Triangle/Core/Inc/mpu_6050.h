#ifndef MPU_6050_H
#define MPU_6050_H

#include "main.h"

/* ============================================================
 * MPU6050 I2C ADDRESS
 * AD0 pin grounded → 0x68
 * AD0 pin to VCC   → 0x69
 * ============================================================ */
#define MPU_ADDR 0x68

/* ============================================================
 * SOFTWARE BIT-BANG I2C PIN MAPPING
 * Custom PCB: PB0 = SCL, PB1 = SDA
 * Uses STM32 atomic BSRR/BRR registers for maximum toggle speed.
 * ============================================================ */
#define SCL_HIGH()  (GPIOB->BSRR = GPIO_PIN_0)            /* PB0 → High */
#define SCL_LOW()   (GPIOB->BRR  = GPIO_PIN_0)            /* PB0 → Low  */
#define SDA_HIGH()  (GPIOB->BSRR = GPIO_PIN_1)            /* PB1 → High */
#define SDA_LOW()   (GPIOB->BRR  = GPIO_PIN_1)            /* PB1 → Low  */
#define SDA_READ()  ((GPIOB->IDR & GPIO_PIN_1) ? 1 : 0)   /* Read PB1   */

/* ============================================================
 * DRIVER FUNCTION PROTOTYPES
 * ============================================================ */
void    MPU6050_Init(void);
uint8_t MPU6050_ReadByte(uint8_t reg);
void    MPU6050_ReadData(float *accel_ms2, float *gyro_dps);

#endif /* MPU_6050_H */