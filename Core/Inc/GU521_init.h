#ifndef GU521_INIT_H
#define GU521_INIT_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Инициализация I2C1 и GPIO под модуль GY-521 (MPU6050).
 * Используем:
 *   PB9 → I2C1_SDA (AF4)
 *   PB8 → I2C1_SCL (AF4)
 * Это Arduino A4/A5 на Nucleo-F429ZI.
 */

void GY521_I2C1_Init(void);

#endif // GU521_INIT_H
