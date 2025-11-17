#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx.h"

#define MPU6050_ADDR 0xD0

// Регистры MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_DATA_START   0x3B


#endif
