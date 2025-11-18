#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f4xx.h"

/* 7-битный адрес (AD0 = GND) */
#define MPU6050_ADDR 0x68

/* Регистры */
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_WHO_AM_I 0x75

/* Значения */
#define MPU6050_DEVICE_RESET 0x80
#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_SMPLRT_7DIV 0x07      // 1 kHz / (7+1) = 125 Hz
#define MPU6050_GYRO_CONFIG_2000 0x18 // ±2000 dps
#define MPU6050_ACCEL_CONFIG_8G 0x10  // ±8g
#define MPU6050_INT_DATA_RDY 0x01

void MPU6050_Init(void);
uint8_t MPU6050_ReadWhoAmI(void);
void MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
float MPU6050_AccelLSB_to_g(int16_t raw);
float MPU6050_GyroLSB_to_dps(int16_t raw);
float MPU6050_TempLSB_to_C(int16_t raw);
#endif
