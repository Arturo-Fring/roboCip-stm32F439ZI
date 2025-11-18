#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx.h"
#include "stdint.h"

#define MPU6050_ADDR            0x68U << 1 // Адрес самого гироскопа MPU6050

// Адреса регистров взяты из референс мануала на MPU6050

#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_INT_ENABLE  0x38U
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU   // начиная отсюда 14 байт акселерометр + гироскоп

// Значения битов взяты из референс мануала на MPU6050. Все в 16-ной системе

#define MPU6050_CLOCK_SOURSE_HSI_8MHz 0x00
#define MPU6050_DEVICE_RESET 0x80
#define MPU6050_CLOCK_SOURSE_PLL_Xref 0x01
#define MPU6050_SMPLRT_7DIV 0x07
#define MPU6050_GYRO_CONFIG_2000 0x18
#define MPU6050_ACCEL_CONFIG_8g 0x10
#define MPU6050_FSYNC_INT_EN 0x01





void MPU6050_Init(void); // Базовая настройка гироскопа
void I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
void MPU6050_ReadData(int16_t* accel, int16_t* gyro);



#endif
