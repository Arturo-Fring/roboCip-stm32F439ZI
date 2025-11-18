#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx.h"
#include <stdint.h>

// 7-битный адрес устройства
#define MPU6050_ADDR            0x68U        // AD0 заземлён или висит в воздухе → 0x68

// Регистры
#define MPU6050_REG_WHO_AM_I        0x75U
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_INT_ENABLE      0x38U
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU

// Значения
#define MPU6050_DEVICE_RESET        0x80U
#define MPU6050_CLOCK_PLL_XGYRO     0x01U    // самый точный и рекомендуемый источник
#define MPU6050_SMPLRT_DIV_1KHZ     0x07U    // 8 кГц / (1+7) = 1 кГц
#define MPU6050_GYRO_FS_2000        0x18U    // ±2000 °/s
#define MPU6050_ACCEL_FS_8G         0x10U    // ±8 g
#define MPU6050_DATA_READY_INT      0x01U

void MPU6050_Init(void);
void I2C_WriteReg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t data);
void MPU6050_ReadData(int16_t* accel, int16_t* gyro);

#endif
