#ifndef MPU6050_H
#define MPU6050_H

// I2C address (0x68 by default, shifted left by 1 for HAL functions)
#define MPU6050_ADDR       (0x68 << 1)

// Register addresses
#define ACCEL_XOUT_H       0x3B
#define ACCEL_YOUT_H       0x3D
#define ACCEL_ZOUT_H       0x3F
#define GYRO_XOUT_H        0x43
#define GYRO_YOUT_H        0x45
#define GYRO_ZOUT_H        0x47
#define PWR_MGMT_1         0x6B

#endif
