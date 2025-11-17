#ifndef CLOCK_H
#define CLOCK_H
/*
 * Инициализация системы тактирования:
 *  - HSI -> PLL -> SYSCLK = 168 МГц
 *  - AHB  = 168 МГц
 *  - APB1 = 42  МГц
 *  - APB2 = 84  МГц
 *  - TIM1 = 168 МГц (на APB2, делитель ≠ 1 -> x2)
 */

#include <stdint.h>
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"
#include "stm32f4xx.h"

void Clock_Init(void);
void MCO_init(void);

#endif